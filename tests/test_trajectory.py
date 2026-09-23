import numpy as np
import pytest

from slam import SLAM
from trajectory import Trajectory


def test_delayed_initialization_and_owned_pose():
    history = Trajectory()
    history.begin(0, 0.)
    history.finish(0, 'initializing', 'waiting')
    history.begin(1, .1)
    history.finish(1, 'initializing', 'low parallax')
    history.begin(2, .2)
    pose = np.eye(4)
    history.accept(0, 0., pose)
    history.accept(2, .2, pose)
    history.finish(2, 'initialized')
    pose[0, 3] = 99
    assert [r.frame_id for r in history.accepted] == [0, 2]
    assert history.records[0].status == 'initializing'
    assert history.records[1].T_cw is None
    assert history.accepted[0].T_cw[0][3] == 0
    rows = history.pose_rows()
    rows[0]['T_cw'][0][3] = 44
    assert history.accepted[0].T_cw[0][3] == 0
    with pytest.raises(ValueError, match='Duplicate'):
        history.begin(2, .2)


def test_native_corrections_and_frame_storage_independence(scene):
    mapping, _, _ = scene
    mapping.frames[2].pose[0, 3] += .1
    result = mapping.optimize(iterations=15)
    assert result.status == 'accepted'
    for frame, record in zip(mapping.frames, mapping.trajectory.accepted):
        np.testing.assert_array_equal(frame.pose, record.T_cw)
    before = mapping.trajectory.pose_rows()
    # Retiring feature storage later must not erase the already published path.
    mapping.frames.clear()
    assert mapping.trajectory.pose_rows() == before


def test_invalid_correction_is_atomic():
    history = Trajectory()
    history.accept(0, 0., np.eye(4))
    history.accept(1, .1, np.eye(4))
    before = history.records
    corrected = np.eye(4)
    corrected[0, 3] = 2
    with pytest.raises(ValueError):
        history.update_poses({0: corrected, 1: np.full((4, 4), np.nan)})
    assert history.records == before


def test_lost_frame_has_outcome_without_identity_pose(scene):
    mapping, _, k = scene
    tracker = SLAM(k)
    tracker.map = mapping
    tracker.reference = mapping.frames[-1]
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 3, .1)
    record = mapping.trajectory.records[-1]
    assert record.status == result.status == 'lost'
    assert result.keyframe is None and not mapping.keyframes.frames
    assert record.reason == result.reason
    assert record.T_cw is None
    assert len(mapping.trajectory.accepted) == 3
