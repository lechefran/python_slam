import numpy as np
import pytest

from dmap import camera_vertex
from point import Point


def test_native_camera_projection(scene):
    mapping, xyz, k = scene
    frame = mapping.frames[1]
    vertex = camera_vertex(frame, 0, True)
    # SBACam's native projection must match the independently generated pixels.
    projection = vertex.estimate().w2i @ np.r_[xyz[0], 1.0]
    np.testing.assert_allclose(projection[:2] / projection[2], frame._kps[0], atol=1e-10)


@pytest.mark.parametrize("scene", [[0, 1, 65536]], indirect=True)
def test_native_bundle_adjustment_and_large_ids(scene):
    mapping, xyz, _ = scene
    anchors = [f.pose.copy() for f in mapping.frames[:2]]
    mapping.frames[2].pose[0, 3] += .1
    rng = np.random.default_rng(12)
    for point in mapping.points:
        point.point += rng.normal(0, .02, 3)
    result = mapping.optimize(iterations=15)
    assert result.status == 'accepted', result
    assert result.after_chi2 < result.before_chi2 * .001
    for frame, expected in zip(mapping.frames[:2], anchors):
        np.testing.assert_array_equal(frame.pose, expected)
    np.testing.assert_allclose(mapping.frames[2].pose[:3, 3], [-.8, 0, 0], atol=1e-5)
    mapping.check_integrity()


def test_observation_conflicts_and_idempotent_deletion(scene):
    mapping, _, _ = scene
    point, other = mapping.points[:2]
    frame = mapping.frames[0]
    point.add_observation(frame, 0)
    assert len(point.frames) == 3
    with pytest.raises(ValueError):
        point.add_observation(frame, 1)
    new = Point(mapping, [0, 0, 5], [0, 0, 255])
    with pytest.raises(ValueError):
        new.add_observation(frame, 0)
    assert not new.frames and frame.pts[0] is point
    point.delete_point()
    point.delete_point()
    assert point not in mapping.points and all(f.pts[0] is None for f in mapping.frames)
    mapping.check_integrity()


def test_stale_and_invalid_landmark_culling(scene):
    mapping, _, _ = scene
    stale = mapping.points[0]
    stale.remove_observation(mapping.frames[-1])
    invalid = mapping.points[1]
    invalid.point[2] = -5
    mapping.cull(50)
    assert stale.deleted and invalid.deleted
    assert not mapping.points[0].deleted
    mapping.check_integrity()


def test_empty_optimizer_and_failed_native_result_leave_state(scene, monkeypatch):
    import dmap
    mapping, _, _ = scene
    before = [f.pose.copy() for f in mapping.frames]
    class FailingOptimizer:
        def __init__(self):
            raise RuntimeError('injected native failure')
    monkeypatch.setattr(dmap.g2o, 'SparseOptimizer', FailingOptimizer)
    result = mapping.optimize()
    assert result.status == 'failed'
    for frame, expected in zip(mapping.frames, before):
        np.testing.assert_array_equal(frame.pose, expected)
    assert dmap.Map().optimize().status == 'skipped'
