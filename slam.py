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

import cv2
import numpy as np

from dmap import Map
from frame import Frame, TrackingError, estimate_pose, match_features, recover_relative
from geometry import project, triangulate, triangulate_valid
from point import Point


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


class SLAM:
    def __init__(self, k, features=2000, mask_bottom=0.0):
        self.map = Map()
        self.k = k
        self.detector = cv2.ORB_create(nfeatures=features)
        self.reference = None
        self.mask_bottom = mask_bottom

    def projection_matches(self, frame, pose, points, indices):
        """Extend proposals using positive-depth map projections, one match per point."""
        used_points, used_indices = set(points), set(indices)
        candidates = [p for p in self.map.points if p not in used_points and p.frames
                      and frame.id - p.frames[-1].id <= 30]
        if not candidates or not len(frame.des):
            return points, indices
        pixels, _, visible = project(frame.k, pose, [p.point for p in candidates])
        visible &= (pixels[:, 0] >= 0) & (pixels[:, 0] < frame.w) & (pixels[:, 1] >= 0) & (pixels[:, 1] < frame.h)
        for point, pixel, valid in zip(candidates, pixels, visible):
            if not valid:
                continue
            nearby = frame.kd.query_ball_point(pixel, 5.0)
            reference = point.frames[-1].des[point.idx[-1]]
            ranked = sorted((cv2.norm(reference, frame.des[i], cv2.NORM_HAMMING), i)
                            for i in nearby if i not in used_indices)
            if ranked and ranked[0][0] < 32 and (len(ranked) == 1 or ranked[0][0] < 0.8 * ranked[1][0]):
                _, index = ranked[0]
                points.append(point)
                indices.append(index)
                used_indices.add(index)
        return points, indices

    def add_points(self, current, previous, i, j, image):
        free = np.array([current.pts[a] is None and previous.pts[b] is None for a, b in zip(i, j)], dtype=bool)
        i, j = i[free], j[free]
        xyz, good = triangulate_valid(previous.pose, current.pose, previous.kps[j], current.kps[i], self.k)
        count = 0
        for location, valid, a, b in zip(xyz, good, i, j):
            if valid:
                u, v = np.rint(current._kps[a]).astype(int)
                if not (0 <= u < current.w and 0 <= v < current.h):
                    continue
                point = Point(self.map, location, image[v, u, ::-1])
                point.add_observation(previous, b)
                point.add_observation(current, a)
                count += 1
        return count

    def process(self, image, frame_id, timestamp):
        """Produce a frame result; failed visual estimates never mutate the map."""
        start = time.perf_counter()
        mask = None
        if self.mask_bottom:
            mask = np.full(image.shape[:2], 255, dtype=np.uint8)
            mask[int(image.shape[0] * (1 - self.mask_bottom)):] = 0
        frame = Frame(self.map, image, self.k, frame_id, timestamp, self.detector, mask)
        result = FrameResult(frame_id, timestamp, 'initializing', features=len(frame.des))
        try:
            if self.reference is None:
                if len(frame.des) < 30:
                    raise TrackingError('insufficient features for a reference frame')
                self.reference = frame
                result.reason = 'waiting for a second view with adequate parallax'
            else:
                previous = self.reference
                i, j = match_features(frame, previous)
                result.matches = len(i)
                if not self.map.frames:
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
                    result.added_points = self.add_points(frame, previous, i[valid], j[valid], image)
                    result.inliers = int(np.count_nonzero(valid))
                    result.status = 'initialized'
                    self.reference = frame
                else:
                    points, indices = [], []
                    for a, b in zip(i, j):
                        if previous.pts[b] is not None:
                            points.append(previous.pts[b])
                            indices.append(int(a))
                    # A provisional pose may guide map search; it is never
                    # committed until the expanded matches pass full coverage.
                    pose, selected = estimate_pose(frame, points, indices, require_coverage=False)
                    points = [points[n] for n in selected]
                    indices = [indices[n] for n in selected]
                    points, indices = self.projection_matches(frame, pose, points, indices)
                    frame.pose, selected = estimate_pose(frame, points, indices)
                    # Commit only after the final map-based pose passes validation;
                    # newly associated points already contributed to this estimate.
                    self.map.add_frame(frame)
                    for n in selected:
                        points[n].add_observation(frame, indices[n])
                    result.inliers = len(selected)
                    # Adjacent dashcam frames often have too little parallax to
                    # replenish landmarks. Reuse an older accepted view with a
                    # larger time baseline, then apply the same geometric gates.
                    older = [f for f in self.map.frames[:-1] if timestamp - f.timestamp >= 0.15]
                    triangulation_frame = older[-1] if older else self.map.frames[0]
                    ti, tj = match_features(frame, triangulation_frame)
                    result.added_points = self.add_points(frame, triangulation_frame, ti, tj, image)
                    self.reference = frame
                    result.status = 'tracking'
                if self.map.frames and len(self.map.frames) % 5 == 0:
                    result.ba = self.map.optimize().to_dict()
                    self.map.cull(frame.id)
        except TrackingError as exc:
            result.status = 'lost' if self.map.frames else 'initializing'
            result.reason = str(exc)
            # Refresh an unusable initializer, but never reset an established map's
            # scale/origin after loss. Subsequent frames retry the last valid view.
            if not self.map.frames and self.reference is not None and frame.id - self.reference.id > 60:
                self.reference = frame if len(frame.des) >= 30 else None
        result.landmarks = len(self.map.points)
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
    cli.add_argument('--mask-bottom', type=float, default=0.0, help='Exclude this image-height fraction (0 <= fraction < 1)')
    cli.add_argument('--seed', type=int, default=0)
    cli.add_argument('--threads', type=int, default=1, help='OpenCV worker threads')
    cli.add_argument('--report', type=Path, help='Save JSON counts, outcomes and accepted world-to-camera poses')
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
    calibration = json.loads(args.calibration.read_text()) if args.calibration else None
    if calibration is not None and not isinstance(calibration, dict):
        raise ValueError('Calibration must be a JSON object')
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        if args.report.resolve() in {args.video.resolve(), args.calibration.resolve() if args.calibration else None}:
            raise ValueError('Report must not overwrite video or calibration')
    cv2.setRNGSeed(args.seed)
    cv2.setNumThreads(args.threads)
    if os.getenv('REVERSE') is not None:
        print('Notice: REVERSE is ignored; translation sign is selected by cheirality.', file=sys.stderr)
    capture = cv2.VideoCapture(str(args.video))
    viewer = None
    results = []
    tracker = None
    outcome, failure = 'completed', None
    start = time.perf_counter()
    metadata = {}
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
                tracker = SLAM(k, args.features, args.mask_bottom)
                maps = cv2.initUndistortRectifyMap(k, distortion, None, k, (w, h), cv2.CV_32FC1) if np.any(distortion) else None
                if not args.headless:
                    from display import Viewer
                    viewer = Viewer()
            if image.shape[:2] != (source_h, source_w):
                raise ValueError('Video dimensions changed during decoding')
            image = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
            if maps is not None:
                image = cv2.remap(image, *maps, cv2.INTER_LINEAR)
            frame, result = tracker.process(image, frame_id, timestamp)
            results.append(result)
            if len(results) == 1 or len(results) % 30 == 0:
                print(f'frame={frame_id} state={result.status} inliers={result.inliers} points={result.landmarks}', flush=True)
            if viewer and not viewer.update(image, frame, tracker.map, result.status):
                outcome = 'closed'
                break
        if not results:
            raise ValueError('No frames decoded in the requested range')
        if viewer and viewer.open:
            viewer.update(image, frame, tracker.map, result.status, force=True)
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
    elapsed = time.perf_counter() - start
    poses = [] if tracker is None else [{'frame_id': f.id, 'timestamp': f.timestamp, 'T_cw': f.pose.tolist()} for f in tracker.map.frames]
    summary = {'schema_version': 1, 'outcome': outcome, 'error': failure,
               'video': str(args.video.resolve()), 'camera': metadata,
               'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                               'numpy': np.__version__, 'opencv': cv2.__version__,
                               'g2opy': importlib.metadata.version('g2opy')},
               'configuration': {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
               'decoded_frames': len(results), 'accepted_poses': len(poses),
               'pose_coverage': len(poses) / len(results) if results else 0.0,
               'states': dict(Counter(r.status for r in results)), 'elapsed_seconds': elapsed,
               'frames': [asdict(r) for r in results], 'poses': poses,
               'landmarks': len(tracker.map.points) if tracker else 0}
    if args.report:
        with args.video.open('rb') as handle:
            summary['video_sha256'] = hashlib.file_digest(handle, 'sha256').hexdigest()
        if args.calibration:
            summary['calibration_sha256'] = hashlib.sha256(args.calibration.read_bytes()).hexdigest()
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
