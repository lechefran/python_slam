"""Long-baseline replenishment must add geometry, not merely fill a picture grid."""

import numpy as np
from scipy.spatial import cKDTree

from geometry import triangulate_valid
from frame import Frame
from slam import SLAM


def distant_features(mapping, world=None):
    """Append independent far-point observations that need the older baseline."""
    if world is None:
        world = np.array([(x, y, 30.) for y in (-8., 8.) for x in (-14., -7., 7., 14.)])
    descriptors = np.random.default_rng(123).integers(0, 256, (len(world), 32), dtype=np.uint8)
    first = len(mapping.frames[0]._kps)
    for camera, timestamp in zip(mapping.frames, [0., .4, .6]):
        camera.timestamp = timestamp
        local = world + camera.pose[:3, 3]
        pixels = np.column_stack((500 * local[:, 0] / local[:, 2] + 320,
                                  510 * local[:, 1] / local[:, 2] + 240))
        camera._kps = np.vstack((camera._kps, pixels))
        camera.kps = np.vstack((camera.kps, (pixels - [320, 240]) / [500, 510]))
        camera.des = np.vstack((camera.des, descriptors))
        camera.pts.extend([None] * len(world))
        camera.kd = cKDTree(camera._kps)
    return world, np.arange(first, first + len(world))


def test_longer_baseline_adds_supported_points_without_moving_existing_state(scene):
    mapping, _, k = scene
    world, rows = distant_features(mapping)
    older, previous, current = mapping.frames
    tracker = SLAM(k, spatial_mapping=True)
    tracker.map = mapping
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    _, short_valid = triangulate_valid(previous.pose, current.pose, previous.kps[rows], current.kps[rows], k)
    assert not short_valid.any()  # 0.4 world units gives less than one degree here.
    points_before = [point.point.copy() for point in mapping.points]
    poses_before = [frame.pose.copy() for frame in mapping.frames]
    evidence = {}
    added = tracker.replenish_spatial_points(current, previous, image, evidence)
    assert added > 0 and added == evidence['added_points']
    assert evidence['reference_frame_id'] == older.id
    for row, expected in zip(rows, world):
        point = current.pts[row]
        if point is not None:
            np.testing.assert_allclose(point.point, expected, atol=1e-8)
            assert older.pts[row] is point and previous.pts[row] is None
    for point, expected in zip(mapping.points, points_before):
        np.testing.assert_array_equal(point.point, expected)
    for frame, expected in zip(mapping.frames, poses_before):
        np.testing.assert_array_equal(frame.pose, expected)
    mapping.check_integrity()


def test_spatial_replenishment_cannot_create_points_without_baseline(scene):
    mapping, _, k = scene
    _, rows = distant_features(mapping)
    current, previous = mapping.frames[-1], mapping.frames[-2]
    older = mapping.frames[0]
    # Make the candidate view coincide with the current camera, with consistent
    # rays. Time separation and a sparse cell cannot replace physical parallax.
    older.pose = current.pose.copy()
    older.kps[rows] = current.kps[rows]
    older._kps[rows] = current._kps[rows]
    tracker = SLAM(k, spatial_mapping=True)
    tracker.map = mapping
    count = len(mapping.points)
    evidence = {}
    assert tracker.replenish_spatial_points(current, previous, np.zeros((480, 640, 3), np.uint8), evidence) == 0
    assert evidence['geometry_pass'] == 0 and len(mapping.points) == count
    assert all(current.pts[row] is None for row in rows)


def test_replenishment_caps_new_support_in_a_busy_candidate_cell(scene):
    mapping, _, k = scene
    world = np.array([(14 + .01 * i, 8., 30.) for i in range(12)])
    _, rows = distant_features(mapping, world)
    tracker = SLAM(k, spatial_mapping=True)
    tracker.map = mapping
    current, previous = mapping.frames[-1], mapping.frames[-2]
    added = tracker.replenish_spatial_points(current, previous, np.zeros((480, 640, 3), np.uint8))
    assert added == 4
    assert sum(current.pts[row] is not None for row in rows) == 4
    mapping.check_integrity()


def test_replenishment_handles_an_empty_feature_array(scene):
    mapping, _, k = scene
    image = np.zeros((480, 640, 3), np.uint8)
    current = Frame(mapping, image, k, timestamp=.6)
    mapping.add_frame(current)
    tracker = SLAM(k, spatial_mapping=True)
    tracker.map = mapping
    evidence = {}
    assert tracker.replenish_spatial_points(current, mapping.frames[-2], image, evidence) == 0
    assert evidence['status'] == 'no_under_supported_features'
    mapping.check_integrity()
