"""Lightweight trajectory history, independent of feature/observation storage."""

from dataclasses import asdict, dataclass, replace

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
    T_cw_initial: tuple | None = None
    submap_id: int | None = None
    scale_status: str | None = None
    is_keyframe: bool = False
    retired: bool = False
    reference_keyframe_id: int | None = None
    T_cr: tuple | None = None


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

    @staticmethod
    def _pose(pose):
        pose = np.asarray(pose, dtype=float)
        if not valid_pose(pose):
            raise ValueError('Invalid trajectory pose or correction')
        return tuple(tuple(float(x) for x in row) for row in pose)

    def _accepted_record(self, frame_id):
        record = self._records.get(frame_id)
        if record is None or record.T_cw is None:
            raise ValueError('Trajectory reference requires an accepted pose')
        return record

    def accept(self, frame_id, timestamp, pose, submap_id=0):
        owned = self._pose(pose)
        if type(submap_id) is not int or submap_id < 0:
            raise ValueError('Submap ID must be a nonnegative integer')
        if frame_id not in self._records:
            self.begin(frame_id, timestamp)
            self.finish(frame_id, "accepted")
        record = self._records[frame_id]
        if record.timestamp != timestamp or record.T_cw is not None:
            raise ValueError('Trajectory timestamp mismatch or pose already accepted')
        self._records[frame_id] = replace(record, T_cw=owned, T_cw_initial=owned,
                                          submap_id=submap_id, scale_status='arbitrary')

    def mark_keyframe(self, frame_id):
        record = self._accepted_record(frame_id)
        if record.retired:
            raise ValueError('Cannot promote a retired frame')
        self._records[frame_id] = replace(record, is_keyframe=True,
                                          reference_keyframe_id=None, T_cr=None)

    def _linked(self, record, reference):
        if not reference.is_keyframe or reference.retired or reference.T_cw is None:
            raise ValueError('Reference must be an accepted keyframe')
        if record.frame_id == reference.frame_id or record.is_keyframe:
            raise ValueError('Only non-keyframes can have a distinct reference')
        if (record.submap_id, record.scale_status) != (reference.submap_id, reference.scale_status):
            raise ValueError('Cannot link different submap or scale contexts')
        # T_cr maps reference-camera coordinates to current-camera coordinates:
        # T_cw = T_cr @ T_rw, hence T_cr = T_cw @ inverse(T_rw).
        relative = np.asarray(record.T_cw) @ np.linalg.inv(np.asarray(reference.T_cw))
        return replace(record, reference_keyframe_id=reference.frame_id, T_cr=self._pose(relative))

    def set_reference(self, frame_id, reference_id):
        record = self._accepted_record(frame_id)
        reference = self._accepted_record(reference_id)
        self._records[frame_id] = self._linked(record, reference)

    def reanchor_dependents(self, old_reference_id, new_reference_id):
        """Move dependencies before future keyframe retirement, preserving T_cw."""
        old = self._accepted_record(old_reference_id)
        new = self._accepted_record(new_reference_id)
        if not old.is_keyframe or not new.is_keyframe or old.frame_id == new.frame_id:
            raise ValueError('Reanchoring requires two distinct keyframes')
        if (old.submap_id, old.scale_status) != (new.submap_id, new.scale_status):
            raise ValueError('Cannot reanchor across submap or scale contexts')
        updates = {r.frame_id: self._linked(r, new) for r in self._records.values()
                   if r.reference_keyframe_id == old_reference_id}
        self._records.update(updates)

    def retirement_updates(self, frame_id, replacement_id):
        """Prepare pose-preserving reanchoring and demotion, without publication."""
        record = self._accepted_record(frame_id)
        reference = self._accepted_record(replacement_id)
        if reference.retired:
            raise ValueError('Cannot reanchor to a retired frame')
        updates = {r.frame_id: self._linked(r, reference) for r in self._records.values()
                   if r.reference_keyframe_id == frame_id}
        updates[frame_id] = self._linked(replace(record, is_keyframe=False, retired=True), reference)
        return updates

    def update_poses(self, poses, independent_ids=()):
        """Atomically correct T_cw and propagate to dependent historical records.

        Explicit estimates win over propagation. Retained mapping frames listed
        in independent_ids keep their authoritative poses if absent from poses;
        refresh their relative transforms when their reference changes. Archived
        records instead retain T_cr and follow the corrected reference camera.
        """
        independent = set(independent_ids)
        updates = {}
        for frame_id, pose in poses.items():
            record = self._accepted_record(frame_id)
            updates[frame_id] = replace(record, T_cw=self._pose(pose))
        # All references are keyframes (roots), so this is one atomic pass, not a
        # recursive chain. Compute everything before publishing any correction.
        for record in self._records.values():
            reference_id = record.reference_keyframe_id
            if reference_id is None or (record.frame_id not in updates and reference_id not in updates):
                continue
            current = updates.get(record.frame_id, record)
            reference = updates.get(reference_id, self._records[reference_id])
            if record.frame_id in poses or record.frame_id in independent:
                updates[record.frame_id] = self._linked(current, reference)
            else:
                pose = np.asarray(record.T_cr) @ np.asarray(reference.T_cw)
                updates[record.frame_id] = replace(record, T_cw=self._pose(pose))
        self._records.update(updates)

    def to_dict(self):
        """Correction provenance is additive; the existing poses list stays stable."""
        return {'schema_version': 1, 'pose_convention': 'world_to_camera',
                'relative_convention': 'T_cw = T_cr @ T_reference_world',
                'initial_pose_semantics': 'first accepted pose before later BA corrections',
                'records': [asdict(record) for record in self._records.values()]}

    def pose_rows(self):
        """Keep the existing JSON pose contract, independently of mapping frames."""
        return [{'frame_id': r.frame_id, 'timestamp': r.timestamp,
                 'T_cw': [list(row) for row in r.T_cw]} for r in self.accepted]
