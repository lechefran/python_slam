"""Feature storage expires independently of complete timestamped pose history."""
from collections import deque
from copy import copy
import gc
import weakref

import numpy as np
import pytest

from recovery import RecoveryKeyframes
from slam import SLAM


def test_long_sequence_bounds_full_frames_and_preserves_corrections(scene):
    mapping, _, _ = scene
    mapping.keyframes.initialize(mapping.frames[:2])
    mapping.trajectory.set_reference(2, 1)
    recent = deque(mapping.frames, maxlen=8)
    recovery = RecoveryKeyframes(capacity=3, interval_seconds=.5)
    template = mapping.frames[-1]
    expired = weakref.ref(template)
    # Repeated views isolate storage lifetime from new geometry or selection.
    for identifier in range(3, 160):
        frame = copy(template)
        frame.id, frame.timestamp = identifier, identifier / 30
        frame.pts = [None] * len(template.pts)
        mapping.add_frame(frame)
        mapping.trajectory.finish(identifier, 'tracking', '')
        mapping.trajectory.set_reference(identifier, 1)
        for index, point in enumerate(mapping.points):
            point.add_observation(frame, index)
        recent.append(frame)
        recovery.add(frame)
        mapping.evict_tracking_frames([*recent, *recovery.frames])
        assert len(mapping.frames) <= 2 + recent.maxlen + recovery.capacity
        mapping.check_integrity()
    del template
    gc.collect()
    assert expired() is None  # No hidden landmark or trajectory reference.
    assert len(mapping.trajectory.records) == mapping.accepted_frame_count == 160
    old = mapping.trajectory._records[2]
    assert old.retired and old.timestamp == 2 / 30
    corrected = mapping.frames[1].pose.copy()
    corrected[1, 3] += .1
    mapping.trajectory.update_poses({1: corrected}, independent_ids=[f.id for f in mapping.frames])
    new = mapping.trajectory._records[2]
    np.testing.assert_allclose(new.T_cw, np.asarray(old.T_cr) @ corrected)
    assert new.T_cw_initial == old.T_cw_initial
    assert (new.status, new.reason, new.timestamp) == (old.status, old.reason, old.timestamp)


def test_expiry_unlinks_weak_landmarks_and_keeps_lost_records(scene):
    mapping, _, _ = scene
    mapping.keyframes.initialize(mapping.frames[:2])
    frame = mapping.frames[-1]
    mapping.trajectory.set_reference(frame.id, 1)
    # One surviving observation is insufficient for a live 3D landmark.
    point = mapping.points[0]
    point.remove_observation(mapping.frames[0])
    before = mapping.trajectory.pose_rows()
    mapping.trajectory.begin(3, .1)
    mapping.trajectory.finish(3, 'lost', 'insufficient support')
    mapping.evict_tracking_frames([])
    assert point.deleted and point not in mapping.points
    assert not point.frames and all(p is None for p in frame.pts)
    assert mapping.trajectory.pose_rows() == before
    lost = mapping.trajectory.records[-1]
    assert lost.T_cw is None and lost.status == 'lost' and lost.timestamp == .1
    mapping.check_integrity()
    # Native graph still has both fixed initialization cameras.
    assert mapping.optimize().status != 'rejected'


@pytest.mark.parametrize('capacity', [0, 1, -1, True, 2.5])
def test_invalid_cache_capacity(capacity):
    with pytest.raises(ValueError, match='tracking_cache_size'):
        SLAM(np.eye(3), tracking_cache_size=capacity)


@pytest.mark.parametrize('recovery', [False, True])
def test_lost_input_preserves_tracking_cache_and_records_status(scene, recovery):
    mapping, _, k = scene
    mapping.keyframes.initialize(mapping.frames[:2])
    mapping.trajectory.set_reference(2, 1)
    tracker = SLAM(k, tracking_cache_size=2, recovery=recovery)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    tracker.tracking_frames.extend(mapping.frames[-2:])
    cached = list(tracker.tracking_frames)
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 3, .1)
    assert result.status == 'lost'
    assert list(tracker.tracking_frames) == cached
    assert tracker.reference is cached[-1]
    record = mapping.trajectory.records[-1]
    assert record.timestamp == .1 and record.status == 'lost' and record.T_cw is None
    assert tracker.frame_storage_summary()['recovery_capacity'] == (64 if recovery else 0)
    mapping.check_integrity()


def test_invalid_expiry_reference_leaves_observations_untouched(scene):
    mapping, _, _ = scene
    mapping.keyframes.initialize(mapping.frames[:2])
    before = mapping.trajectory.records
    frames = list(mapping.frames)
    # A malformed imported non-keyframe has no reference: fail before removal.
    with pytest.raises(ValueError, match='accepted pose'):
        mapping.evict_tracking_frames([])
    assert mapping.trajectory.records == before and mapping.frames == frames
    assert all(len(point.frames) == 3 for point in mapping.points)
    mapping.check_integrity()
