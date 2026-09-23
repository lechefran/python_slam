"""Lightweight trajectory history, independent of feature/observation storage."""

from dataclasses import dataclass, replace

import numpy as np

from geometry import valid_pose


@dataclass(frozen=True, slots=True)
class TrajectoryRecord:
    """One source frame; T_cw is accepted world-to-camera geometry or None.

    Status/reason describe the original processing outcome. Initialization can
    accept an earlier reference later without rewriting that historical outcome.
    Pose tuples own their values and cannot alias mutable mapping-frame arrays.
    """
    frame_id: int
    timestamp: float
    status: str = 'pending'
    reason: str = ''
    T_cw: tuple | None = None


class Trajectory:
    def __init__(self):
        self._records = {}

    @property
    def records(self):
        return tuple(self._records.values())

    @property
    def accepted(self):
        return tuple(record for record in self._records.values() if record.T_cw is not None)

    def begin(self, frame_id, timestamp):
        if frame_id in self._records:
            raise ValueError('Duplicate trajectory frame ID')
        if not np.isfinite(timestamp):
            raise ValueError('Invalid trajectory timestamp')
        if self._records and frame_id <= next(reversed(self._records)):
            raise ValueError('Trajectory frame IDs must increase')
        self._records[frame_id] = TrajectoryRecord(frame_id, float(timestamp))

    def finish(self, frame_id, status, reason=''):
        self._records[frame_id] = replace(self._records[frame_id], status=status, reason=reason)

    def accept(self, frame_id, timestamp, pose):
        if not valid_pose(pose):
            raise ValueError('Invalid trajectory pose')
        if frame_id not in self._records:
            self.begin(frame_id, timestamp)
            self.finish(frame_id, "accepted")
        if self._records[frame_id].timestamp != timestamp:
            raise ValueError('Trajectory timestamp mismatch')
        self.update_poses({frame_id: pose})

    def update_poses(self, poses):
        """Publish validated 4x4 T_cw corrections together, without frame references."""
        updates = {}
        for frame_id, pose in poses.items():
            if not valid_pose(pose):
                raise ValueError('Invalid trajectory correction')
            updates[frame_id] = replace(self._records[frame_id],
                                       T_cw=tuple(tuple(float(x) for x in row) for row in pose))
        self._records.update(updates)

    def pose_rows(self):
        """Keep the existing JSON pose contract, independently of mapping frames."""
        return [{'frame_id': r.frame_id, 'timestamp': r.timestamp,
                 'T_cw': [list(row) for row in r.T_cw]} for r in self.accepted]
