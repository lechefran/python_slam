"""Deliberate mapping-keyframe selection; no tracking or retirement side effects."""

from dataclasses import asdict, dataclass

import numpy as np

from geometry import project, valid_pose


@dataclass(frozen=True)
class KeyframePolicy:
    min_interval_seconds: float = .15
    max_interval_seconds: float = 1.
    min_support: int = 30
    min_shared: int = 20
    min_parallax_degrees: float = 1.
    min_rotation_degrees: float = 10.
    min_overlap: float = .7
    min_support_ratio: float = .7
    max_residual_pixels: float = 3.

    def __post_init__(self):
        values = asdict(self)
        if any(not np.isfinite(value) or value <= 0 for value in values.values()):
            raise ValueError('Keyframe thresholds must be finite and positive')
        if (self.max_interval_seconds < self.min_interval_seconds
                or not 0 < self.min_overlap <= 1 or not 0 < self.min_support_ratio <= 1
                or self.min_shared < 2 or self.min_support < self.min_shared
                or type(self.min_shared) is not int or type(self.min_support) is not int
                or self.min_parallax_degrees > 180 or self.min_rotation_degrees > 180):
            raise ValueError('Invalid keyframe policy ordering or range')


class MappingKeyframes:
    """Select live accepted frames; keep insertion evidence separate from BA state.

    Selection does not remove mapping frames or change optimizer membership.
    The existing bounded recovery archive has its own independent policy.
    """

    def __init__(self, mapping, policy=None):
        self.mapping = mapping
        self.policy = policy or KeyframePolicy()
        self.frames = []
        self.insertions = []
        self._reference_support = frozenset()

    def _verified(self, frame, allowed):
        """Recheck existing estimator support in positive depth and rectified pixels."""
        pairs = [(index, point) for index, point in enumerate(frame.pts)
                 if point is not None and not point.deleted and point.id in allowed]
        if not pairs:
            return {}
        pixels, _, visible = project(frame.k, frame.pose, [p.point for _, p in pairs])
        errors = np.linalg.norm(pixels - frame._kps[[i for i, _ in pairs]], axis=1)
        return {point.id: index for (index, point), good, error in zip(pairs, visible, errors)
                if good and np.isfinite(error) and error <= self.policy.max_residual_pixels}

    def _check_frame(self, frame):
        if frame not in self.mapping.frames or not valid_pose(frame.pose) or not np.isfinite(frame.timestamp):
            raise ValueError('Keyframes must be finite, accepted mapping frames')
        if self.frames and (frame.id <= self.frames[-1].id or frame.timestamp <= self.frames[-1].timestamp):
            raise ValueError('Keyframes must advance source ID and time')

    def _insert(self, frame, decision, support):
        self.mapping.trajectory.mark_keyframe(frame.id)
        self.frames.append(frame)
        self._reference_support = frozenset(support)
        self.insertions.append(decision)
        return decision

    def initialize(self, frames):
        """Preserve the already validated two-view initialization anchors."""
        if self.frames:
            raise ValueError('Mapping keyframes are already initialized')
        if len(frames) != 2:
            raise ValueError('Expected two initialization anchors')
        # Check both anchors before publishing either; bootstrap is a deliberate
        # exception to the interval/retained-support gates, not a second fit.
        for frame in frames:
            self._check_frame(frame)
        if frames[0].id >= frames[1].id or frames[0].timestamp >= frames[1].timestamp:
            raise ValueError('Initialization anchors must advance source ID and time')
        for frame in frames:
            support = self._verified(frame, {p.id for p in frame.pts if p is not None})
            decision = {'frame_id': frame.id, 'selected': True, 'reasons': ['initialization'],
                        'reference_frame_id': None if not self.frames else self.frames[-1].id,
                        'support': len(support)}
            self._insert(frame, decision, support)
        return self.insertions[-1].copy()

    def consider(self, frame, tracking_points, recovered=False):
        """Assess committed T_cw using pre-triangulation pose-estimator landmarks.

        Newborn points cannot manufacture support for their own insertion. BA
        may have corrected poses or culled observations; recheck current evidence.
        """
        if not self.frames:
            raise ValueError('Initialize mapping keyframes first')
        self._check_frame(frame)
        reference = self.frames[-1]
        support = self._verified(frame, {p.id for p in tracking_points})
        reference_support = self._verified(reference, self._reference_support)
        shared = sorted(support.keys() & reference_support.keys())
        elapsed = float(frame.timestamp - reference.timestamp)
        overlap = len(shared) / len(self._reference_support) if self._reference_support else None
        ratio = len(support) / len(self._reference_support) if self._reference_support else None
        parallax = None
        if len(shared) >= self.policy.min_shared:
            # Measured normalized rays rotate from camera to world using R_cw.T.
            # Comparing them in that common frame removes pure camera rotation;
            # no translation magnitude or assumed metric scale enters the angle.
            rays = []
            for view, observations in ((reference, reference_support), (frame, support)):
                normalized = view.kps[[observations[identifier] for identifier in shared]]
                world = np.column_stack((normalized, np.ones(len(shared)))) @ view.pose[:3, :3]
                rays.append(world / np.linalg.norm(world, axis=1)[:, None])
            angles = np.degrees(np.arctan2(np.linalg.norm(np.cross(*rays), axis=1),
                                          np.sum(rays[0] * rays[1], axis=1)))
            if np.isfinite(angles).all():
                parallax = float(np.median(angles))
        relative_rotation = frame.pose[:3, :3] @ reference.pose[:3, :3].T
        rotation = float(np.degrees(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1))))
        decision = {'frame_id': frame.id, 'reference_frame_id': reference.id, 'selected': False,
                    'reasons': [], 'elapsed_seconds': elapsed, 'support': len(support),
                    'reference_support_at_insertion': len(self._reference_support),
                    'shared_support': len(shared), 'overlap': overlap, 'support_ratio': ratio,
                    'median_parallax_degrees': parallax, 'rotation_degrees': rotation}
        if len(support) < self.policy.min_support:
            decision['reasons'] = ['insufficient_verified_support']
            return decision
        if elapsed < self.policy.min_interval_seconds and not recovered:
            decision['reasons'] = ['minimum_interval']
            return decision
        reasons = []
        if recovered:
            reasons.append('recovery')
        if parallax is not None and parallax >= self.policy.min_parallax_degrees:
            reasons.append('parallax')
        if rotation >= self.policy.min_rotation_degrees:
            reasons.append('rotation')
        if overlap is not None and overlap < self.policy.min_overlap:
            reasons.append('overlap_drop')
        if ratio is not None and ratio < self.policy.min_support_ratio:
            reasons.append('support_drop')
        if elapsed >= self.policy.max_interval_seconds:
            reasons.append('maximum_interval')
        decision['selected'] = bool(reasons)
        decision['reasons'] = reasons or ['redundant_view']
        return self._insert(frame, decision, support) if reasons else decision

    def summary(self):
        return {'schema_version': 1, 'policy': asdict(self.policy),
                'count': len(self.frames), 'insertions': self.insertions.copy(),
                'affects_estimation': False}
