#!/usr/bin/env python3
"""Sparse monocular SLAM: validated visual poses, native BA, optional desktop view."""

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np

from dmap import Map
from frame import Frame, TrackingError, estimate_pose, match_features, recover_relative
from geometry import project, spatial_support, triangulate, triangulate_valid
from point import Point
from recovery import RecoveryKeyframes
from feature_mask import prepare_feature_mask
from observation_quality import write_quality_report


def camera_parameters(width, height, focal=525.0, max_width=1024, calibration=None):
    """Return processed (width,height), K and distortion for pinhole BGR input.

    Fallback focal is in source pixels. K uses proportional pixel-coordinate
    scaling; actual integer output dimensions determine sx and sy independently.
    """
    if width <= 0 or height <= 0 or max_width <= 0 or not np.isfinite(focal) or focal <= 0:
        raise ValueError('Image dimensions, width limit, and focal length must be positive')
    distortion = np.zeros(5)
    if calibration is None:
        k = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1.0]])
    else:
        if calibration.get('model') != 'pinhole':
            raise ValueError('Only pinhole calibration is supported; rectify other lens models externally')
        if calibration.get('width') != width or calibration.get('height') != height:
            raise ValueError('Calibration dimensions do not match the decoded source image')
        k = np.asarray(calibration.get('K'), dtype=float)
        distortion = np.asarray(calibration.get('distortion', [0] * 5), dtype=float)
        if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1]) > 1e-12 or abs(k[1, 0]) > 1e-12):
            raise ValueError('Calibration K must be a finite zero-skew pinhole matrix with positive focal lengths')
        if distortion.ndim != 1 or len(distortion) not in (4, 5, 8, 12, 14) or not np.isfinite(distortion).all():
            raise ValueError('Invalid pinhole distortion coefficients')
    processed_width = min(width, max_width)
    processed_height = max(1, int(height * processed_width / width))
    # Scale the entire intrinsics row, including principal point, after resizing.
    k = np.diag([processed_width / width, processed_height / height, 1.0]) @ k
    return processed_width, processed_height, k, distortion


@dataclass
class FrameResult:
    frame_id: int
    timestamp: float
    status: str
    reason: str = ''
    features: int = 0
    matches: int = 0
    inliers: int = 0
    added_points: int = 0
    landmarks: int = 0
    processing_seconds: float = 0.0
    ba: dict | None = None
    diagnostics: dict | None = None
    recovered_from: int | None = None
    landmark_quality: dict | None = None
    observation_quality: dict | None = None


class SLAM:
    def __init__(self, k, features=2000, mask_bottom=0.0, condition_pnp=True, spatial_mapping=True, robust_pnp=False,
                 recovery=True, feature_mask=None, landmark_maturity=False, observation_history=True):
        self.map = Map(landmark_maturity=landmark_maturity, observation_history=observation_history)
        self.k = k
        self.detector = cv2.ORB_create(nfeatures=features)
        self.reference = None
        self.mask_bottom = mask_bottom
        if not np.isfinite(mask_bottom) or not 0 <= mask_bottom < 1:
            raise ValueError('mask_bottom must be in [0, 1)')
        if feature_mask is not None:
            if (feature_mask.dtype != np.uint8 or feature_mask.ndim != 2
                    or not np.isin(feature_mask, [0, 255]).all()):
                raise ValueError('feature_mask must be a binary uint8 processed-image mask')
            feature_mask = feature_mask.copy()
            feature_mask.setflags(write=False)
        self.feature_mask = feature_mask
        self._mask_shape = None
        self._extraction_mask = None
        self.condition_pnp = condition_pnp
        self.robust_pnp = robust_pnp
        self.spatial_mapping = spatial_mapping
        self.recovery = recovery
        self.keyframes = RecoveryKeyframes()

    def pose_landmark(self, point, recovery=False):
        """New points cannot estimate poses, except the initial two-view bootstrap."""
        return (point is not None and not point.deleted and
                (not self.map.landmark_maturity or point.state == 'active' or
                 (not recovery and len(self.map.frames) == 2 and point.bootstrap)))

    def extraction_mask(self, shape):
        """Combine immutable processed exclusions with the legacy bottom strip once."""
        if self._mask_shape is not None:
            if tuple(shape) != self._mask_shape:
                raise ValueError('Image dimensions changed after preparing the feature mask')
            return self._extraction_mask
        mask = self.feature_mask
        if mask is not None and mask.shape != tuple(shape):
            raise ValueError('Processed feature mask dimensions do not match the image')
        if self.mask_bottom:
            mask = np.full(shape, 255, np.uint8) if mask is None else mask.copy()
            mask[int(shape[0] * (1 - self.mask_bottom)):] = 0
        if mask is not None:
            mask.setflags(write=False)
        self._mask_shape, self._extraction_mask = tuple(shape), mask
        return mask

    def projection_matches(self, frame, pose, points, indices, diagnostics=None, trace=None, recovery_reference=None,
                           candidate_validation=False, quality_attempts=None):
        """Extend proposals using positive-depth map projections, one match per point."""
        used_points, used_indices = set(points), set(indices)
        if recovery_reference is None:
            candidates = [p for p in self.map.points if p not in used_points and p.frames
                          and frame.id - p.frames[-1].id <= 30
                          and ((p.state == 'candidate') if candidate_validation else self.pose_landmark(p))]
            descriptors = None
        else:
            # Only a geometrically checked recovery seed may search old points.
            # Restrict this pass to that view's live landmarks, using descriptors
            # from the same old view rather than an unrelated last observation.
            descriptors = {p: recovery_reference.des[i] for i, p in enumerate(recovery_reference.pts)
                           if self.pose_landmark(p, recovery=True) and p not in used_points}
            candidates = list(descriptors)
        if diagnostics is not None:
            diagnostics.update(candidates=len(candidates), positive_depth=0, in_image=0,
                               with_nearby_features=0, with_available_features=0, added_matches=0)
        if not candidates or not len(frame.des):
            return points, indices
        pixels, _, visible = project(frame.k, pose, [p.point for p in candidates])
        if diagnostics is not None:
            diagnostics['positive_depth'] = int(visible.sum())
        visible &= (pixels[:, 0] >= 0) & (pixels[:, 0] < frame.w) & (pixels[:, 1] >= 0) & (pixels[:, 1] < frame.h)
        if diagnostics is not None:
            diagnostics['in_image'] = int(visible.sum())
        if trace is not None:
            trace.update(visible_pixels=pixels[visible].tolist(), added_feature_indices=[])
        for point, pixel, valid in zip(candidates, pixels, visible):
            if not valid:
                continue
            if quality_attempts is not None:
                quality_attempts.setdefault(point, None)
            nearby = frame.kd.query_ball_point(pixel, 5.0)
            reference = point.frames[-1].des[point.idx[-1]] if descriptors is None else descriptors[point]
            ranked = sorted((cv2.norm(reference, frame.des[i], cv2.NORM_HAMMING), i)
                            for i in nearby if i not in used_indices)
            if diagnostics is not None:
                diagnostics['with_nearby_features'] += bool(nearby)
                diagnostics['with_available_features'] += bool(ranked)
            if ranked and ranked[0][0] < 32 and (len(ranked) == 1 or ranked[0][0] < 0.8 * ranked[1][0]):
                _, index = ranked[0]
                points.append(point)
                indices.append(index)
                if quality_attempts is not None:
                    quality_attempts[point] = int(index)
                used_indices.add(index)
                if diagnostics is not None:
                    diagnostics['added_matches'] += 1
                if trace is not None:
                    trace['added_feature_indices'].append(int(index))
        return points, indices

    def recover_pose(self, frame, diagnostics=None, trace=None, quality_attempts=None):
        """Try at most three old views; return a proposal without mutating the map.

        Descriptor retrieval is only a shortlist. Pose fitting uses live world
        XYZ, pixel measurements and the usual depth/residual/coverage/rank gates.
        At least 30 original descriptor correspondences, and half the original
        matched population, must survive the final fit. Projected additions
        cannot manufacture that independent descriptor-support requirement.
        """
        evidence = diagnostics if diagnostics is not None else {}
        evidence.update(status='failed', archive_size=len(self.keyframes.frames),
                        scanned_keyframes=0, candidates=[], attempts=[], chosen_frame_id=None)
        if len(frame.des) < 30:
            evidence['status'] = 'insufficient_features'
            return None
        ranked = []
        for reference in self.keyframes.candidates(frame, self.reference):
            evidence['scanned_keyframes'] += 1
            live = np.array([i for i, point in enumerate(reference.pts)
                             if self.pose_landmark(point, recovery=True)], dtype=int)
            if len(live) < 30:
                evidence['candidates'].append({'frame_id': reference.id, 'mapped_matches': 0})
                continue
            # Match only surviving landmark descriptors, then explicitly remap
            # train rows to the original feature slots; culling can leave holes.
            i, matched = match_features(frame, SimpleNamespace(des=reference.des[live]))
            j = live[matched]
            points, indices, used = [], [], set()
            for a, b in zip(i, j):
                point = reference.pts[b]
                if point is not None and not point.deleted and point not in used:
                    used.add(point)
                    points.append(point)
                    indices.append(int(a))
            evidence['candidates'].append({'frame_id': reference.id, 'mapped_matches': len(points)})
            if len(points) >= 30:
                ranked.append((reference, points, indices))
        ranked.sort(key=lambda item: (-len(item[1]), -item[0].id))
        for reference, original_points, original_indices in ranked[:3]:
            candidate_attempts = dict(zip(original_points, original_indices)) if quality_attempts is not None else None
            attempt = {'frame_id': reference.id, 'age_frames': frame.id - reference.id,
                       'age_seconds': frame.timestamp - reference.timestamp,
                       'mapped_matches': len(original_points), 'status': 'rejected', 'stages': {}}
            evidence['attempts'].append(attempt)
            samples = {} if trace is not None else None
            def observe(name):
                return attempt['stages'].setdefault(name, {})
            def snapshot(name):
                return samples.setdefault(name, {}) if samples is not None else None
            try:
                pose, rows = estimate_pose(frame, original_points, original_indices, require_coverage=False,
                    diagnostics=observe('provisional_pnp'), trace=snapshot('provisional_pnp'),
                    condition=self.condition_pnp, robust=False)
                points = [original_points[n] for n in rows]
                indices = [original_indices[n] for n in rows]
                points, indices = self.projection_matches(frame, pose, points, indices,
                    observe('projection_search'), snapshot('projection_search'), recovery_reference=reference,
                    quality_attempts=candidate_attempts)
                pose, rows = estimate_pose(frame, points, indices, diagnostics=observe('final_pnp'),
                    trace=snapshot('final_pnp'), condition=self.condition_pnp, robust=self.robust_pnp)
                originals = set(zip(original_points, original_indices))
                retained = sum((points[n], indices[n]) in originals for n in rows)
                attempt['original_inliers'] = retained
                if retained < 30 or retained < .5 * len(original_points):
                    raise TrackingError('insufficient original keyframe support for recovery')
            except (TrackingError, cv2.error) as exc:
                attempt['reason'] = str(exc)
                if samples is not None:
                    trace.setdefault('attempts', []).append(samples)
                continue
            attempt.update(status='accepted', final_inliers=len(rows))
            evidence.update(status='recovered', chosen_frame_id=reference.id)
            if quality_attempts is not None:
                # Only the winning recovery attempt has an accepted pose for
                # interpreting misses; discarded candidates cannot penalize points.
                quality_attempts.clear()
                quality_attempts.update(candidate_attempts)
            if trace is not None:
                trace['selected'] = samples
            stages = dict(attempt['stages'], reference_frame_id=reference.id)
            return pose, [points[n] for n in rows], [indices[n] for n in rows], stages, samples
        return None

    def validate_candidates(self, current, previous, i, j, diagnostics=None, quality_attempts=None):
        """Observe candidates only after the camera pose has been committed.

        Propagated descriptors and bounded-recency projections propose matches;
        the fixed accepted T_cw must support each with positive depth and <=3px
        error. These observations never enter this frame's pose fit/inlier count.
        """
        points, indices, used = [], [], set()
        for a, b in zip(i, j):
            point = previous.pts[b]
            if (point is not None and not point.deleted and point.state == 'candidate'
                    and point not in used and current.pts[a] is None):
                points.append(point)
                indices.append(int(a))
                used.add(point)
                if quality_attempts is not None:
                    quality_attempts[point] = int(a)
        # Reserve every committed observation before looking for extra matches.
        committed = [(p, n) for n, p in enumerate(current.pts) if p is not None]
        combined_points = [p for p, _ in committed] + points
        combined_indices = [n for _, n in committed] + indices
        combined_points, combined_indices = self.projection_matches(
            current, current.pose, combined_points, combined_indices, candidate_validation=True,
            quality_attempts=quality_attempts)
        points, indices = combined_points[len(committed):], combined_indices[len(committed):]
        accepted = 0
        if points:
            pixels, _, visible = project(current.k, current.pose, [p.point for p in points])
            valid = visible & (np.linalg.norm(pixels - current._kps[indices], axis=1) <= 3.)
            for point, index, good in zip(points, indices, valid):
                if good:
                    point.add_observation(current, index)
                    accepted += 1
        if diagnostics is not None:
            diagnostics.update(proposals=len(points), accepted=accepted, rejected=len(points) - accepted,
                               pose_source='committed pose; candidates excluded from estimation')

    def assess_searches(self, frame, attempts):
        """Commit passive, deduplicated search evidence only for accepted cameras.

        Reproject against final T_cw: a provisional search is not proof that a
        landmark was visible. Out-of-image, masked or invalid final projections
        are unassessed, not failed reobservations. Actual occlusion is unknown.
        """
        if not attempts:
            return
        points = list(attempts)
        pixels, depths, visible = project(frame.k, frame.pose, [p.point for p in points])
        actual = {p: i for i, p in enumerate(frame.pts) if p is not None}
        mask = self.extraction_mask((frame.h, frame.w))
        for point, pixel, depth, valid in zip(points, pixels, depths, visible):
            index = actual.get(point, attempts[point])
            residual = float(np.linalg.norm(pixel - frame._kps[index])) if valid and index is not None else None
            inside = valid and 0 <= pixel[0] < frame.w and 0 <= pixel[1] < frame.h
            if inside and mask is not None:
                u, v = np.floor(pixel + .5).astype(int)
                inside = u < frame.w and v < frame.h and mask[v, u] != 0
            if point in actual:
                outcome = 'accepted'
            elif not inside:
                outcome = 'unassessed'
            else:
                outcome = 'unmatched' if index is None else 'rejected'
            point.record_quality(frame.id, frame.id, 'search', outcome, residual, depth, index)

    def add_points(self, current, previous, i, j, image, bootstrap=False):
        free = np.array([current.pts[a] is None and previous.pts[b] is None for a, b in zip(i, j)], dtype=bool)
        i, j = i[free], j[free]
        xyz, good = triangulate_valid(previous.pose, current.pose, previous.kps[j], current.kps[i], self.k)
        count = 0
        for location, valid, a, b in zip(xyz, good, i, j):
            if valid:
                u, v = np.rint(current._kps[a]).astype(int)
                if not (0 <= u < current.w and 0 <= v < current.h):
                    continue
                point = Point(self.map, location, image[v, u, ::-1], born_frame_id=current.id, bootstrap=bootstrap)
                point.add_observation(previous, b)
                point.add_observation(current, a)
                count += 1
        return count

    def replenish_spatial_points(self, current, previous, image, diagnostics=None):
        """Fill sparse 4x4 image cells using one older accepted triangulation view.

        The ordinary mapping pass uses at least 0.15 seconds of separation.
        Here 0.45 seconds offers more parallax for distant features; time alone
        never authorizes a landmark. Retain the same two-view depth, 1-degree
        parallax and 3-pixel reprojection gates, with at most four points per cell.
        """
        older = [frame for frame in self.map.frames[:-1]
                 if current.timestamp - frame.timestamp >= .45 and frame is not previous]
        if diagnostics is not None:
            diagnostics.update(reference_frame_id=None, candidate_matches=0,
                               geometry_pass=0, added_points=0, status='no_older_reference')
        if not older:
            return 0
        cells = np.floor(current._kps / [current.w / 4, current.h / 4]).astype(int)
        cell_ids = cells[:, 1] * 4 + cells[:, 0]
        occupied = np.array([point is not None for point in current.pts], dtype=bool)
        counts = np.bincount(cell_ids[occupied], minlength=16)
        if not np.any((counts[cell_ids] < 4) & ~occupied):
            if diagnostics is not None:
                diagnostics['status'] = 'no_under_supported_features'
            return 0
        reference = older[-1]
        ti, tj = match_features(current, reference)
        free = np.array([current.pts[a] is None and reference.pts[b] is None
                         and counts[cell_ids[a]] < 4 for a, b in zip(ti, tj)], dtype=bool)
        ti, tj = ti[free], tj[free]
        _, good = triangulate_valid(reference.pose, current.pose, reference.kps[tj], current.kps[ti], self.k)
        # Descriptor matches arrive in evidence order. Reserve only geometrically
        # valid proposals until each sparse cell reaches its small support target.
        chosen = []
        for row, valid in enumerate(good):
            cell = cell_ids[ti[row]]
            if valid and counts[cell] < 4:
                chosen.append(row)
                counts[cell] += 1
        added = self.add_points(current, reference, ti[chosen], tj[chosen], image) if chosen else 0
        if diagnostics is not None:
            diagnostics.update(reference_frame_id=reference.id, candidate_matches=len(ti),
                               geometry_pass=int(good.sum()), added_points=added, status='evaluated')
        return added

    def process(self, image, frame_id, timestamp, diagnostics=False, capture_trace=False):
        """Produce a frame result; failed visual estimates never mutate the map."""
        start = time.perf_counter()
        # Scalar diagnostics can be retained per frame. Pixel/XYZ snapshots are
        # transient and requested only in the diagnostic window, before BA edits.
        self.last_trace = {} if capture_trace else None
        evidence = {'reference_frame_id': self.reference.id if self.reference else None,
                    'reference_age_frames': frame_id - self.reference.id if self.reference else None,
                    'reference_age_seconds': timestamp - self.reference.timestamp if self.reference else None,
                    'stages': {}, 'timing_seconds': {}} if diagnostics else None
        mask = self.extraction_mask(image.shape[:2])
        frame = Frame(self.map, image, self.k, frame_id, timestamp, self.detector, mask)
        result = FrameResult(frame_id, timestamp, 'initializing', features=len(frame.des), diagnostics=evidence)
        if evidence is not None:
            evidence['timing_seconds']['extraction'] = time.perf_counter() - start
            evidence['stages']['extraction'] = {'spatial_support': spatial_support(frame._kps, frame.w, frame.h)}
            evidence['stages']['extraction']['excluded_fraction'] = float(np.mean(mask == 0)) if mask is not None else 0.0
        if self.last_trace is not None:
            self.last_trace['feature_pixels'] = frame._kps.tolist()
        stage = 'reference'

        def observe(name):
            """Allocate optional evidence without altering a producer's decisions."""
            return evidence['stages'].setdefault(name, {}) if evidence is not None else None

        def snapshot(name):
            return self.last_trace.setdefault(name, {}) if self.last_trace is not None else None

        try:
            if self.reference is None:
                if len(frame.des) < 30:
                    raise TrackingError('insufficient features for a reference frame')
                self.reference = frame
                result.reason = 'waiting for a second view with adequate parallax'
            else:
                previous = self.reference
                stage = 'descriptor_matching'
                stage_start = time.perf_counter()
                i, j = match_features(frame, previous, observe(stage))
                if evidence is not None:
                    evidence['timing_seconds'][stage] = time.perf_counter() - stage_start
                if self.last_trace is not None:
                    self.last_trace['matches'] = {'current_pixels': frame._kps[i].tolist(),
                                                  'previous_pixels': previous._kps[j].tolist()}
                result.matches = len(i)
                if not self.map.frames:
                    stage = 'initialization'
                    pose, inliers = recover_relative(previous.kps[j], frame.kps[i], min(self.k[0, 0], self.k[1, 1]))
                    i, j = i[inliers], j[inliers]
                    frame.pose = pose @ previous.pose
                    xyz, valid = triangulate_valid(previous.pose, frame.pose, previous.kps[j], frame.kps[i], self.k)
                    # A two-view map needs redundancy for descriptor dropout in
                    # the next frame, not just the solver's minimum sample count.
                    if np.count_nonzero(valid) < 60:
                        raise TrackingError('insufficient triangulation/parallax for an initial map')
                    spans = np.ptp(frame._kps[i[valid]], axis=0)
                    if spans[0] < 0.1 * frame.w or spans[1] < 0.1 * frame.h:
                        raise TrackingError('initial landmarks have insufficient image coverage')
                    # Initialize one consistent arbitrary baseline, then use PnP
                    # against this map rather than accumulating unit translations.
                    self.map.add_frame(previous)
                    self.map.add_frame(frame)
                    result.added_points = self.add_points(frame, previous, i[valid], j[valid], image, bootstrap=True)
                    result.inliers = int(np.count_nonzero(valid))
                    result.status = 'initialized'
                    self.reference = frame
                else:
                    points, indices = [], []
                    for a, b in zip(i, j):
                        if self.pose_landmark(previous.pts[b]):
                            points.append(previous.pts[b])
                            indices.append(int(a))
                    quality_attempts = dict(zip(points, indices)) if self.map.observation_history else None
                    if evidence is not None:
                        evidence['stages']['landmark_selection'] = {
                            'policy': ('bootstrap' if len(self.map.frames) == 2 else 'active_only')
                                      if self.map.landmark_maturity else 'legacy',
                            'pose_inputs': len(points),
                            'candidate_inputs': sum(p.state == 'candidate' for p in points)
                                               if self.map.landmark_maturity else None}
                    try:
                        # A provisional pose may guide map search; it is never
                        # committed until the expanded matches pass full coverage.
                        stage = 'provisional_pnp'
                        stage_start = time.perf_counter()
                        # Keep the search seed on the established solver path;
                        # robust updates use the complete post-search population.
                        pose, selected = estimate_pose(frame, points, indices, require_coverage=False,
                                                       diagnostics=observe(stage), trace=snapshot(stage),
                                                       condition=self.condition_pnp, robust=False)
                        if evidence is not None:
                            evidence['timing_seconds'][stage] = time.perf_counter() - stage_start
                        points = [points[n] for n in selected]
                        indices = [indices[n] for n in selected]
                        stage = 'projection_search'
                        stage_start = time.perf_counter()
                        points, indices = self.projection_matches(frame, pose, points, indices, observe(stage), snapshot(stage),
                                                                  quality_attempts=quality_attempts)
                        if evidence is not None:
                            evidence['timing_seconds'][stage] = time.perf_counter() - stage_start
                        stage = 'final_pnp'
                        stage_start = time.perf_counter()
                        frame.pose, selected = estimate_pose(frame, points, indices,
                                                             diagnostics=observe(stage), trace=snapshot(stage),
                                                             condition=self.condition_pnp, robust=self.robust_pnp)
                        if evidence is not None:
                            evidence['timing_seconds'][stage] = time.perf_counter() - stage_start
                    except TrackingError as exc:
                        if not self.recovery:
                            raise
                        failed_stage = stage
                        if evidence is not None:
                            evidence['timing_seconds'][stage] = time.perf_counter() - stage_start
                            evidence['normal_tracking_failure'] = {'stage': stage, 'reason': str(exc),
                                'stages': dict(evidence['stages'])}
                        stage = 'recovery'
                        stage_start = time.perf_counter()
                        recovered = self.recover_pose(frame, observe(stage), snapshot(stage), quality_attempts=quality_attempts)
                        if evidence is not None:
                            evidence['timing_seconds'][stage] = time.perf_counter() - stage_start
                        if recovered is None:
                            stage = failed_stage
                            raise
                        frame.pose, points, indices, recovered_stages, recovered_trace = recovered
                        selected = np.arange(len(points))
                        result.recovered_from = recovered_stages.pop('reference_frame_id')
                        if evidence is not None:
                            evidence['stages'].update(recovered_stages)
                        if self.last_trace is not None:
                            self.last_trace['normal_tracking'] = {
                                key: value for key, value in self.last_trace.items() if key != 'recovery'}
                            self.last_trace.update(recovered_trace)
                    # Commit only after the final map-based pose passes validation;
                    # newly associated points already contributed to this estimate.
                    self.map.add_frame(frame)
                    for n in selected:
                        points[n].add_observation(frame, indices[n])
                    result.inliers = len(selected)
                    if self.map.landmark_maturity:
                        stage_start = time.perf_counter()
                        self.validate_candidates(frame, previous, i, j, observe('candidate_validation'), quality_attempts)
                        if evidence is not None:
                            evidence['timing_seconds']['candidate_validation'] = time.perf_counter() - stage_start
                    if quality_attempts is not None:
                        self.assess_searches(frame, quality_attempts)
                    # Adjacent dashcam frames often have too little parallax to
                    # replenish landmarks. Reuse an older accepted view with a
                    # larger time baseline, then apply the same geometric gates.
                    if result.recovered_from is None:
                        older = [f for f in self.map.frames[:-1] if timestamp - f.timestamp >= 0.15]
                        triangulation_frame = older[-1] if older else self.map.frames[0]
                        stage_start = time.perf_counter()
                        ti, tj = match_features(frame, triangulation_frame)
                        result.added_points = self.add_points(frame, triangulation_frame, ti, tj, image)
                        if self.spatial_mapping:
                            result.added_points += self.replenish_spatial_points(
                                frame, triangulation_frame, image, observe('spatial_replenishment'))
                        if evidence is not None:
                            evidence['stages']['triangulation'] = {'reference_frame_id': triangulation_frame.id,
                                'matches': len(ti), 'added_points': result.added_points}
                            evidence['timing_seconds']['triangulation'] = time.perf_counter() - stage_start
                    self.reference = frame
                    result.status = 'tracking'
                if self.map.frames and len(self.map.frames) % 5 == 0:
                    stage_start = time.perf_counter()
                    result.ba = self.map.optimize().to_dict()
                    self.map.cull(frame.id)
                    if evidence is not None:
                        evidence['timing_seconds']['optimization_and_culling'] = time.perf_counter() - stage_start
            if self.recovery and result.status in ('initialized', 'tracking'):
                if not self.keyframes.frames:
                    self.keyframes.add(self.map.frames[0])
                self.keyframes.add(frame)
        except TrackingError as exc:
            result.status = 'lost' if self.map.frames else 'initializing'
            result.reason = str(exc)
            if evidence is not None:
                evidence['failure_stage'] = stage
                if stage in ('provisional_pnp', 'final_pnp'):
                    evidence['timing_seconds'].setdefault(stage, time.perf_counter() - stage_start)
            # Refresh an unusable initializer, but never reset an established map's
            # scale/origin after loss. Subsequent frames retry the last valid view.
            if not self.map.frames and self.reference is not None and frame.id - self.reference.id > 60:
                self.reference = frame if len(frame.des) >= 30 else None
        result.landmarks = len(self.map.points)
        if self.map.landmark_maturity:
            result.landmark_quality = self.map.maturity_summary()
        if self.map.observation_history:
            if result.status == 'lost':
                self.map.quality_events['tracking/lost_unassessed'] += 1
            result.observation_quality = self.map.observation_summary()
        result.processing_seconds = time.perf_counter() - start
        return frame, result


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('video', type=Path)
    cli.add_argument('--headless', action='store_true', help='Run without importing a GUI backend')
    cli.add_argument('--hold', action='store_true', help='Keep the final desktop map open')
    cli.add_argument('--max-frames', type=int, help='Maximum decoded frames to process')
    cli.add_argument('--start-frame', type=int, default=os.getenv('SEEK', '0'))
    cli.add_argument('--width', type=int, default=1024, help='Maximum processed image width')
    cli.add_argument('--focal', type=float, default=os.getenv('F', '525'), help='Approximate focal length in source pixels')
    cli.add_argument('--calibration', type=Path, help='Pinhole JSON calibration at source resolution')
    cli.add_argument('--features', type=int, default=2000)
    cli.add_argument('--spatial-mapping', action=argparse.BooleanOptionalAction, default=True,
                     help='Replenish sparse image cells using a longer triangulation baseline')
    cli.add_argument('--condition-pnp', action=argparse.BooleanOptionalAction, default=True,
                     help='Centre/scale pose fitting with validated consensus refits (default: enabled)')
    cli.add_argument('--robust-pnp', action=argparse.BooleanOptionalAction, default=False,
                     help='Opt in to block-Huber refinement; Jacobian checks are always active')
    cli.add_argument('--recovery', action=argparse.BooleanOptionalAction, default=True,
                     help='Recover failed tracking against a bounded archive of older views')
    cli.add_argument('--landmark-maturity', action=argparse.BooleanOptionalAction, default=False,
                     help='Opt in to candidate validation and mature-only tracking, recovery and BA')
    cli.add_argument('--observation-history', action=argparse.BooleanOptionalAction, default=True,
                     help='Record bounded passive landmark quality histories (default: enabled)')
    cli.add_argument('--quality-report', type=Path, help='Detailed landmark history JSON, separate from the trajectory report')
    cli.add_argument('--mask-bottom', type=float, default=0.0, help='Exclude this image-height fraction (0 <= fraction < 1)')
    cli.add_argument('--feature-mask', type=Path, help='Source-size grayscale PNG: 0 excludes, 255 allows features')
    cli.add_argument('--seed', type=int, default=0)
    cli.add_argument('--threads', type=int, default=1, help='OpenCV worker threads')
    cli.add_argument('--report', type=Path, help='Save JSON counts, outcomes and accepted world-to-camera poses')
    cli.add_argument('--diagnostics-dir', type=Path, help='New directory for tracking evidence and sampled PNG overlays')
    cli.add_argument('--diagnostics-start', type=int, default=850, help='First source frame to capture (inclusive)')
    cli.add_argument('--diagnostics-end', type=int, default=1000, help='Last source frame to capture (inclusive)')
    cli.add_argument('--diagnostics-every', type=int, default=10, help='Capture stride; also capture status transitions')
    return cli


def run(args):
    """Run sequential decoding and always release the capture and desktop viewer."""
    if not args.video.is_file():
        raise ValueError(f'Video does not exist: {args.video}')
    if args.start_frame < 0 or args.width <= 0 or args.features < 30 or args.threads < 1:
        raise ValueError('Invalid start frame, width, feature count, or thread count')
    if args.max_frames is not None and args.max_frames <= 0:
        raise ValueError('--max-frames must be positive')
    if not np.isfinite(args.mask_bottom) or not 0 <= args.mask_bottom < 1:
        raise ValueError('--mask-bottom must be in [0, 1)')
    if not -2147483648 <= args.seed <= 2147483647:
        raise ValueError('--seed must fit a signed 32-bit integer')
    if args.hold and args.headless:
        raise ValueError('--hold requires the desktop viewer')
    if args.diagnostics_start < 0 or args.diagnostics_end < args.diagnostics_start or args.diagnostics_every < 1:
        raise ValueError('Invalid diagnostic frame range or capture stride')
    calibration = json.loads(args.calibration.read_text()) if args.calibration else None
    if calibration is not None and not isinstance(calibration, dict):
        raise ValueError('Calibration must be a JSON object')
    if args.quality_report and not args.observation_history:
        raise ValueError('--quality-report requires observation history')
    inputs = {p.resolve() for p in (args.video, args.calibration, args.feature_mask) if p is not None}
    outputs = set()
    for output in (args.report, args.quality_report):
        if output is None:
            continue
        paths = {output.resolve(), output.with_name(output.name + '.tmp').resolve()}
        if paths & inputs:
            raise ValueError('Report must not overwrite video, calibration or feature mask')
        if paths & outputs:
            raise ValueError('Trajectory and quality reports must use distinct paths, including temporary files')
        outputs.update(paths)
        output.parent.mkdir(parents=True, exist_ok=True)
    cv2.setRNGSeed(args.seed)
    cv2.setNumThreads(args.threads)
    diagnostic_writer = None
    if args.diagnostics_dir:
        from tracking_diagnostics import DiagnosticWriter
        diagnostic_writer = DiagnosticWriter(args.diagnostics_dir, args.diagnostics_start,
                                             args.diagnostics_end, args.diagnostics_every)
    if os.getenv('REVERSE') is not None:
        print('Notice: REVERSE is ignored; translation sign is selected by cheirality.', file=sys.stderr)
    capture = cv2.VideoCapture(str(args.video))
    viewer = None
    results = []
    tracker = None
    outcome, failure = 'completed', None
    start = time.perf_counter()
    metadata = {}
    mask_metadata = None
    expected_frames = 0
    try:
        if not capture.isOpened():
            raise ValueError(f'Cannot open video: {args.video}')
        count = capture.get(cv2.CAP_PROP_FRAME_COUNT)
        expected_frames = int(count) if np.isfinite(count) and count > 0 else 0
        # Decode to the requested start rather than assuming compressed-video
        # random seeking preserves an exact source-frame identity.
        for _ in range(args.start_frame):
            if not capture.grab():
                raise ValueError('--start-frame is beyond the decodable video')
        while args.max_frames is None or len(results) < args.max_frames:
            ok, image = capture.read()
            if not ok:
                if expected_frames and args.start_frame + len(results) < expected_frames:
                    raise ValueError('Decoding stopped before the reported end of the video')
                break
            frame_id = args.start_frame + len(results)
            timestamp = capture.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if not np.isfinite(timestamp) or (results and timestamp <= results[-1].timestamp):
                raise ValueError('Decoder returned invalid/non-increasing timestamps')
            if tracker is None:
                source_h, source_w = image.shape[:2]
                w, h, k, distortion = camera_parameters(source_w, source_h, args.focal, args.width, calibration)
                metadata = {'source_size': [source_w, source_h], 'processed_size': [w, h],
                            'K': k.tolist(), 'distortion': distortion.tolist(),
                            'calibration_status': 'provided' if calibration else 'approximate',
                            'timestamp_source': 'OpenCV CAP_PROP_POS_MSEC', 'scale': 'arbitrary'}
                if calibration is None:
                    print('Approximate intrinsics: supply --calibration for camera-specific geometry; scale is arbitrary.', file=sys.stderr)
                maps = cv2.initUndistortRectifyMap(k, distortion, None, k, (w, h), cv2.CV_32FC1) if np.any(distortion) else None
                feature_mask, mask_metadata = prepare_feature_mask(args.feature_mask, (source_w, source_h), (w, h), maps)
                tracker = SLAM(k, args.features, args.mask_bottom, args.condition_pnp, args.spatial_mapping,
                               args.robust_pnp, args.recovery, feature_mask=feature_mask,
                               landmark_maturity=args.landmark_maturity, observation_history=args.observation_history)
                effective_mask = tracker.extraction_mask((h, w))
                mask_metadata.update(bottom_fraction=args.mask_bottom,
                    excluded_fraction=float(np.mean(effective_mask == 0)) if effective_mask is not None else 0.0,
                    effective_sha256=hashlib.sha256(effective_mask.tobytes()).hexdigest() if effective_mask is not None else None)
                if not args.headless:
                    from display import Viewer
                    viewer = Viewer()
            if image.shape[:2] != (source_h, source_w):
                raise ValueError('Video dimensions changed during decoding')
            image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
            if maps is not None:
                image = cv2.remap(image, *maps, cv2.INTER_LINEAR)
            frame, result = tracker.process(image, frame_id, timestamp,
                diagnostics=diagnostic_writer is not None,
                capture_trace=diagnostic_writer is not None and diagnostic_writer.captures(frame_id))
            results.append(result)
            if diagnostic_writer:
                diagnostic_start = time.perf_counter()
                diagnostic_writer.write(image, result, tracker.last_trace, mask=effective_mask)
                result.diagnostics['timing_seconds']['artifact_write'] = time.perf_counter() - diagnostic_start
            if len(results) == 1 or len(results) % 30 == 0 or result.recovered_from is not None:
                source = '' if result.recovered_from is None else f' recovered_from={result.recovered_from}'
                print(f'frame={frame_id} state={result.status} inliers={result.inliers} points={result.landmarks}{source}', flush=True)
            if viewer and not viewer.update(image, frame, tracker.map, result.status, mask=effective_mask):
                outcome = 'closed'
                break
        if not results:
            raise ValueError('No frames decoded in the requested range')
        if viewer and viewer.open:
            viewer.update(image, frame, tracker.map, result.status, force=True, mask=effective_mask)
            if args.hold:
                viewer.hold()
    except KeyboardInterrupt:
        outcome, failure = 'interrupted', 'keyboard interrupt'
    except Exception as exc:
        outcome, failure = 'failed', f'{type(exc).__name__}: {exc}'
    finally:
        capture.release()
        if viewer:
            viewer.close()
        if diagnostic_writer:
            diagnostic_writer.finish()
    elapsed = time.perf_counter() - start
    poses = [] if tracker is None else [{'frame_id': f.id, 'timestamp': f.timestamp, 'T_cw': f.pose.tolist()} for f in tracker.map.frames]
    summary = {'schema_version': 1, 'outcome': outcome, 'error': failure,
               'video': str(args.video.resolve()), 'camera': metadata, 'feature_mask': mask_metadata,
               'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                               'numpy': np.__version__, 'opencv': cv2.__version__,
                               'g2opy': importlib.metadata.version('g2opy')},
               'configuration': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
               'decoded_frames': len(results), 'accepted_poses': len(poses),
               'pose_coverage': len(poses) / len(results) if results else 0.0,
               'recovered_frames': sum(r.recovered_from is not None for r in results),
               'states': dict(Counter(r.status for r in results)), 'elapsed_seconds': elapsed,
               'frames': [asdict(r) for r in results], 'poses': poses,
               'landmarks': len(tracker.map.points) if tracker else 0,
               'landmark_quality': tracker.map.maturity_summary() if tracker and tracker.map.landmark_maturity else None,
               'observation_quality': tracker.map.observation_summary() if tracker else None}
    if args.report or args.quality_report:
        with args.video.open('rb') as handle:
            summary['video_sha256'] = hashlib.file_digest(handle, 'sha256').hexdigest()
        if args.calibration:
            summary['calibration_sha256'] = hashlib.sha256(args.calibration.read_bytes()).hexdigest()
    if args.quality_report:
        last_id = results[-1].frame_id if results else args.start_frame
        quality = {'schema_version': 1, 'kind': 'landmark_observation_quality',
                   'video_sha256': summary['video_sha256'], 'camera': metadata, 'feature_mask': mask_metadata,
                   'configuration': summary['configuration'], 'environment': summary['environment'],
                   'outcome': outcome, 'error': failure, 'last_processed_frame_id': last_id,
                   'summary': summary['observation_quality'],
                   'residual_units': 'processed rectified pixels', 'depth_units': 'arbitrary monocular map units'}
        write_quality_report(args.quality_report, quality, tracker.map.points if tracker else [],
                             tracker.map.retired_quality if tracker else [], last_id)
    if args.report:
        temporary = args.report.with_name(args.report.name + '.tmp')
        temporary.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        temporary.replace(args.report)
    print(f'Finished: {len(results)} frames, {len(poses)} accepted poses, {summary["landmarks"]} landmarks, {elapsed:.2f}s')
    if failure:
        print(failure, file=sys.stderr)
    return 130 if outcome == 'interrupted' else (1 if outcome == 'failed' else (0 if len(poses) >= 2 else 2))


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    try:
        return run(args)
    except (ValueError, OSError, cv2.error) as exc:
        cli.error(str(exc))


if __name__ == '__main__':
    sys.exit(main())
