import numpy as np
import pytest

from dmap import Map
from frame import Frame, extract, match_features, estimate_pose, TrackingError
from slam import SLAM


def test_empty_frontend_and_no_implicit_registration():
    image = np.zeros((180, 320, 3), np.uint8)
    k = np.array([[300., 0, 160], [0, 300, 90], [0, 0, 1]])
    mapping = Map()
    frame = Frame(mapping, image, k)
    assert (frame.w, frame.h) == (320, 180)
    assert frame._kps.shape == (0, 2) and frame.des.shape == (0, 32)
    assert mapping.frames == []
    i, j = match_features(frame, frame)
    assert i.shape == j.shape == (0,)
    tracker = SLAM(k)
    _, result = tracker.process(image, 0, 0)
    assert result.status == 'initializing' and not tracker.map.frames


def test_pnp_recovers_map_scale_and_rejects_outliers(scene):
    mapping, xyz, _ = scene
    current = mapping.frames[2]
    truth = current.pose.copy()
    current._kps[:8] += 80
    pose, inliers = estimate_pose(current, mapping.points, np.arange(len(xyz)))
    assert len(inliers) == len(xyz) - 8
    np.testing.assert_allclose(pose, truth, atol=1e-6)
    with pytest.raises(TrackingError):
        estimate_pose(current, mapping.points[:4], np.arange(4))


def test_blank_frame_after_initialization_does_not_change_map(scene):
    mapping, _, k = scene
    tracker = SLAM(k)
    tracker.map = mapping
    tracker.reference = mapping.frames[-1]
    old_frames = list(mapping.frames)
    old_points = list(mapping.points)
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 3, .1)
    assert result.status == 'lost'
    assert mapping.frames == old_frames and mapping.points == old_points
    mapping.check_integrity()
