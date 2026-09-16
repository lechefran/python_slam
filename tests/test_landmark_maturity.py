"""Lifecycle evidence uses independent synthetic cameras and actual native fits."""

from copy import copy

import numpy as np
import pytest
from scipy.spatial import cKDTree

import slam
from point import Point
from slam import SLAM


def enable(mapping):
    mapping.landmark_maturity = True
    for point in mapping.points:
        point.refresh_maturity()


def test_three_views_with_later_support_promote_and_duplicate_does_not(scene):
    mapping, _, _ = scene
    point = mapping.points[0]
    last = mapping.frames[-1]
    point.remove_observation(last)
    enable(mapping)
    assert point.state == 'candidate'
    point.add_observation(mapping.frames[1], 0)
    assert point.state == 'candidate' and len(point.frames) == 2
    point.add_observation(last, 0)
    assert point.state == 'active'
    assert point.quality['parallax_degrees'] >= 1
    assert point.quality['max_residual_px'] < 1e-8
    mapping.check_integrity()


@pytest.mark.parametrize('failure', ['same_centre', 'residual', 'behind', 'no_later_view'])
def test_three_observations_alone_do_not_establish_maturity(scene, failure):
    mapping, _, _ = scene
    point = mapping.points[0]
    if failure == 'same_centre':
        # Identical camera centres with perfectly consistent measurements have
        # zero triangulation baseline, regardless of the number of observations.
        for camera in mapping.frames[1:]:
            camera.pose = mapping.frames[0].pose.copy()
            camera._kps[0] = mapping.frames[0]._kps[0]
    elif failure == 'residual':
        mapping.frames[-1]._kps[0] += 10
    elif failure == 'behind':
        point.point[2] *= -1
    else:
        point.born_frame_id = mapping.frames[-1].id
    enable(mapping)
    assert point.state == 'candidate'


def test_removing_support_demotes_and_deletion_is_atomic_and_idempotent(scene):
    mapping, _, _ = scene
    enable(mapping)
    point = mapping.points[0]
    assert mapping.landmark_counts['active'] == 60
    point.remove_observation(mapping.frames[-1])
    assert point.state == 'candidate' and mapping.maturity_events['demotions'] == 1
    point.delete_point('outlier')
    point.delete_point()
    assert point.state == 'outlier' and point.retirement_reason == 'outlier'
    assert mapping.landmark_counts['outlier'] == 1
    assert mapping.maturity_events['demotions'] == 1
    assert all(f.pts[0] is None for f in mapping.frames)
    mapping.check_integrity()


def test_maturity_is_invariant_to_world_origin_and_monocular_scale(scene):
    mapping, _, _ = scene
    enable(mapping)
    before = [p.quality['parallax_degrees'] for p in mapping.points]
    shift, scale = np.array([1e8, -2e8, 3e8]), 1e4
    for frame in mapping.frames:
        frame.pose[:3, 3] = scale * frame.pose[:3, 3] - frame.pose[:3, :3] @ shift
    for point, angle in zip(mapping.points, before):
        point.point = scale * point.point + shift
        point.refresh_maturity()
        assert point.state == 'active'
        assert point.quality['parallax_degrees'] == pytest.approx(angle, abs=1e-8)


def test_candidates_expire_by_creation_age_but_supported_points_survive(scene):
    mapping, _, _ = scene
    point = mapping.points[0]
    point.remove_observation(mapping.frames[-1])
    enable(mapping)
    mapping.cull(40, stale_after=1000)
    assert point.deleted and point.state == 'retired'
    assert len(mapping.points) == 59 and all(p.state == 'active' for p in mapping.points)
    mapping.check_integrity()


def test_bootstrap_is_limited_to_initial_pair_and_never_recovery(scene):
    mapping, _, k = scene
    point = mapping.points[0]
    point.remove_observation(mapping.frames[-1])
    enable(mapping)
    tracker = SLAM(k, landmark_maturity=True)
    tracker.map = mapping
    point.bootstrap = True
    assert not tracker.pose_landmark(point)
    third = mapping.frames.pop()
    assert tracker.pose_landmark(point)
    assert not tracker.pose_landmark(point, recovery=True)
    mapping.frames.append(third)
    assert not tracker.pose_landmark(point)


def test_candidate_validation_uses_fixed_pose_without_influencing_it(scene):
    mapping, _, k = scene
    current = mapping.frames[-1]
    points = list(mapping.points)
    for point in points:
        point.remove_observation(current)
    enable(mapping)
    current.des = mapping.frames[1].des.copy()
    current._kps[:5] += 50  # These candidate associations must not be committed.
    current.kd = cKDTree(current._kps)
    tracker = SLAM(k, landmark_maturity=True)
    tracker.map = mapping
    before = current.pose.copy()
    evidence = {}
    tracker.validate_candidates(current, mapping.frames[1], np.arange(60), np.arange(60), evidence)
    np.testing.assert_array_equal(current.pose, before)
    assert evidence['accepted'] == 55 and evidence['rejected'] == 5
    assert all(p.state == 'active' for p in points[5:])
    assert all(current.pts[i] is None for i in range(5))
    mapping.check_integrity()


def test_candidate_only_recovery_and_ba_are_rejected(scene):
    mapping, _, k = scene
    for point in mapping.points:
        point.remove_observation(mapping.frames[-1])
    enable(mapping)
    tracker = SLAM(k, landmark_maturity=True)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    tracker.keyframes.add(mapping.frames[0])
    current = copy(mapping.frames[0])
    current.id, current.timestamp = 150, 5.
    assert tracker.recover_pose(current) is None
    assert mapping.optimize().reason == 'no supported local landmarks'


def test_native_recovery_uses_only_active_points_and_does_not_mutate_state(scene):
    mapping, _, k = scene
    enable(mapping)
    tracker = SLAM(k, landmark_maturity=True)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    tracker.keyframes.add(mapping.frames[0])
    current = copy(mapping.frames[-1])
    current.id, current.timestamp = 150, 5.
    current.des = mapping.frames[0].des.copy()
    current.pts = [None] * 60
    for point in mapping.points[:5]:
        point.remove_observation(mapping.frames[-1])
    before = mapping.maturity_summary()
    proposal = tracker.recover_pose(current)
    assert proposal is not None
    np.testing.assert_allclose(proposal[0], current.pose, atol=1e-6)
    assert len(proposal[1]) == 55 and all(p.state == 'active' for p in proposal[1])
    assert mapping.maturity_summary() == before
    assert all(p is None for p in current.pts)
    mapping.check_integrity()


def test_process_estimates_from_active_support_then_matures_remaining_candidates(scene, monkeypatch):
    mapping, xyz, k = scene
    enable(mapping)
    current = copy(mapping.frames[-1])
    current.id, current.timestamp = 3, .1
    current.pose = np.eye(4)
    current.pose[0, 3] = -1.2
    local = xyz + current.pose[:3, 3]
    current._kps = np.column_stack((500 * local[:, 0] / local[:, 2] + 320,
                                   510 * local[:, 1] / local[:, 2] + 240))
    current.kd = cKDTree(current._kps)
    current.pts = [None] * 60
    for point in mapping.points[:10]:
        point.remove_observation(mapping.frames[0])
    tracker = SLAM(k, landmark_maturity=True)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    monkeypatch.setattr(slam, 'Frame', lambda *a, **kw: current)
    calls = []
    original = slam.estimate_pose
    def checked(frame, points, indices, **kwargs):
        assert all(p.state == 'active' for p in points)
        calls.append(len(points))
        return original(frame, points, indices, **kwargs)
    monkeypatch.setattr(slam, 'estimate_pose', checked)
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 3, .1, diagnostics=True)
    assert result.status == 'tracking' and result.inliers == 50
    assert calls == [50, 50]
    assert result.diagnostics['stages']['candidate_validation']['accepted'] == 10
    assert all(p.state == 'active' for p in mapping.points)
    assert len([p for p in current.pts if p is not None]) == 60
    mapping.check_integrity()


def test_native_mature_ba_revalidates_support_after_culling(scene):
    mapping, _, _ = scene
    enable(mapping)
    mapping.frames[-1].pose[0, 3] += .05
    result = mapping.optimize()
    assert result.status == 'accepted'
    mapping.cull(3)
    assert all(p.state == 'active' for p in mapping.points)
    mapping.check_integrity()


def test_failed_pose_does_not_mature_candidates(scene, monkeypatch):
    mapping, _, k = scene
    for point in mapping.points:
        point.remove_observation(mapping.frames[-1])
    enable(mapping)
    current = copy(mapping.frames[1])
    current.id, current.timestamp = 5, 5 / 30
    current.pts = [None] * 60
    tracker = SLAM(k, landmark_maturity=True)
    tracker.map, tracker.reference = mapping, mapping.frames[1]
    monkeypatch.setattr(slam, 'Frame', lambda *a, **kw: current)
    before = mapping.maturity_summary()
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 5, 5 / 30)
    assert result.status == 'lost'
    assert mapping.maturity_summary() == before
    assert all(p is None for p in current.pts)
    mapping.check_integrity()
