"""World-space landmarks with reciprocal, one-per-frame observations."""

import numpy as np
from geometry import project
from observation_quality import ObservationQuality, measurement


class Point:
    def __init__(self, img_map, location, color, born_frame_id=None, bootstrap=False):
        self.point = np.asarray(location, dtype=np.float64).copy()
        if self.point.shape != (3,) or not np.isfinite(self.point).all():
            raise ValueError('Landmark must be a finite world-space XYZ point')
        self.frames = []
        self.idx = []
        self.color = np.asarray(color, dtype=np.uint8).copy()  # RGB, not OpenCV BGR.
        self.map = img_map
        self.id = img_map.max_point
        img_map.max_point += 1
        img_map.points.append(self)
        self.deleted = False
        self.state = 'candidate'
        self.born_frame_id = born_frame_id
        self.bootstrap = bootstrap
        self.quality = {'reason': 'needs_three_views', 'parallax_degrees': None, 'max_residual_px': None}
        self.retirement_reason = None
        self.map.landmark_counts['candidate'] += 1
        self.observation_quality = ObservationQuality() if self.map.observation_history else None

    def record_quality(self, assessed, observed, phase, outcome, residual=None, depth=None, index=None):
        if self.observation_quality is not None:
            if self.observation_quality.record(assessed, observed, phase, outcome, residual, depth, index):
                self.map.quality_events[f'{phase}/{outcome}'] += 1

    def quality_snapshot(self, current_id):
        """Serializable evidence; maturity classification is separate from history."""
        return {'point_id': self.id, 'state': self.state if self.map.landmark_maturity or self.deleted else 'unassessed',
                'born_frame_id': self.born_frame_id,
                'age_source_frames': max(0, current_id - self.born_frame_id) if self.born_frame_id is not None else None,
                'live_observations': len(self.frames), 'retirement_reason': self.retirement_reason,
                'history': self.observation_quality.summary() if self.observation_quality is not None else None}

    def _set_state(self, state):
        if state == self.state:
            return
        self.map.landmark_counts[self.state] -= 1
        self.map.landmark_counts[state] += 1
        if state == 'active':
            self.map.maturity_events['promotions'] += 1
        elif self.state == 'active' and state == 'candidate':
            self.map.maturity_events['demotions'] += 1
        self.state = state

    def refresh_maturity(self):
        """Assess live first/latest-two views; thresholds are pixels and degrees.

        A point needs three distinct accepted cameras, including a view after
        its creation. World-space ray angles are scale-invariant: invert T_cw
        to obtain camera centres, then compare normalized point-minus-centre
        vectors. This is a conservative support policy, not a covariance.
        """
        if self.deleted:
            return
        self.quality = {'reason': 'needs_three_views', 'parallax_degrees': None, 'max_residual_px': None}
        if len(self.frames) < 3:
            self._set_state('candidate')
            return
        pairs = sorted(zip(self.frames, self.idx), key=lambda pair: pair[0].id)
        views = [pairs[0], pairs[-2], pairs[-1]]
        if len({f.id for f, _ in views}) < 3 or views[-1][0].id <= self.born_frame_id:
            self._set_state('candidate')
            return
        rays, errors = [], []
        for frame, index in views:
            pixel, _, visible = project(frame.k, frame.pose, [self.point])
            error = float(np.linalg.norm(pixel[0] - frame._kps[index]))
            if not visible[0] or not np.isfinite(error) or error > 3.0:
                self.quality['reason'] = 'invalid_depth_or_residual'
                self._set_state('candidate')
                return
            centre = -frame.pose[:3, :3].T @ frame.pose[:3, 3]
            ray = self.point - centre
            extent = np.max(np.abs(ray))
            if not np.isfinite(extent) or extent == 0:
                self.quality['reason'] = 'invalid_viewing_ray'
                self._set_state('candidate')
                return
            ray = ray / extent
            rays.append(ray / np.linalg.norm(ray))
            errors.append(error)
        cosine = np.clip(np.asarray(rays) @ np.asarray(rays).T, -1., 1.)
        angle = float(np.degrees(np.arccos(np.min(cosine))))
        self.quality.update(parallax_degrees=angle, max_residual_px=max(errors),
                            checked_frame_ids=[f.id for f, _ in views],
                            reason='supported' if angle >= 1.0 else 'insufficient_parallax')
        self._set_state('active' if angle >= 1.0 else 'candidate')

    def orb(self):
        return [frame.des[index] for frame, index in zip(self.frames, self.idx)]

    def add_observation(self, frame, index):
        if self.deleted:
            raise ValueError('Cannot observe a deleted landmark')
        if not 0 <= index < len(frame.pts):
            raise IndexError('Observation feature index is out of bounds')
        if frame in self.frames:
            if self.idx[self.frames.index(frame)] == index and frame.pts[index] is self:
                return
            raise ValueError('Landmark already has an observation in this frame')
        if frame.pts[index] is not None:
            raise ValueError('Feature already belongs to another landmark')
        # Validate both sides before committing either link; BA reads this exact
        # stored index rather than searching an ambiguous feature-slot list.
        self.frames.append(frame)
        self.idx.append(int(index))
        frame.pts[index] = self
        if self.born_frame_id is None:
            self.born_frame_id = frame.id
        if self.observation_quality is not None:
            residual, depth = measurement(self.point, frame, index)
            assessed = max(frame.id, self.map.frames[-1].id if self.map.frames else frame.id)
            self.record_quality(assessed, frame.id, 'observation', 'added', residual, depth, index)
        if self.map.landmark_maturity:
            self.refresh_maturity()

    def remove_observation(self, frame, reason='removed', assessed_frame_id=None):
        if frame not in self.frames:
            return
        position = self.frames.index(frame)
        index = self.idx[position]
        if frame.pts[index] is not self:
            raise ValueError('Broken reciprocal observation')
        self.idx.pop(position)
        self.frames.pop(position)
        frame.pts[index] = None
        if not self.deleted:
            assessed = assessed_frame_id if assessed_frame_id is not None else (self.map.frames[-1].id if self.map.frames else frame.id)
            self.record_quality(assessed, frame.id, 'observation', reason, index=index)
        if self.map.landmark_maturity:
            self.refresh_maturity()

    def delete_point(self, reason='retired', assessed_frame_id=None):
        if self.deleted:
            return
        self.deleted = True  # Unlinking a retired point is not a quality demotion.
        for frame in list(self.frames):
            self.remove_observation(frame)
        if self in self.map.points:
            self.map.points.remove(self)
        self.retirement_reason = reason
        self._set_state('outlier' if reason == 'outlier' else 'retired')
        if self.observation_quality is not None:
            assessed = assessed_frame_id if assessed_frame_id is not None else (self.map.frames[-1].id if self.map.frames else (self.born_frame_id or 0))
            self.record_quality(assessed, assessed, 'retirement', reason)
            self.map.retired_quality.append(self.quality_snapshot(assessed))

    def homogenous(self):
        return np.r_[self.point, 1.0]
