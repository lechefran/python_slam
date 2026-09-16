"""Recovery uses real descriptor matching/PnP and independently projected cameras."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from scipy.spatial import cKDTree

import slam
from geometry import normalize
from recovery import RecoveryKeyframes
from slam import SLAM
from tracking_diagnostics import DiagnosticWriter


class Camera:
    def __init__(self, **values):
        self.__dict__.update(values)


def returning_camera(scene, frame_id=150):
    """Revisit the first camera after the normal reference has different texture.

    Landmarks were last observed at frame 2, over 30 source frames ago. Current
    pixels are derived directly from XYZ with known T_cw, independently of the
    production projection function. Feature detection alone is bypassed in
    process tests, so descriptor matching, native PnP and association run normally.
    """
    mapping, xyz, k = scene
    old = mapping.frames[0]
    rotation = cv2.Rodrigues(np.array([.025, -.035, .01]))[0]
    translation = np.array([.06, -.025, .015])
    camera = xyz @ rotation.T + translation
    pixels = np.column_stack((k[0, 0] * camera[:, 0] / camera[:, 2] + k[0, 2],
                               k[1, 1] * camera[:, 1] / camera[:, 2] + k[1, 2]))
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation, translation
    current = Camera(id=frame_id, timestamp=frame_id / 30, k=k, w=640, h=480,
        _kps=pixels, kps=normalize(np.linalg.inv(k), pixels), des=old.des.copy(),
        pts=[None] * len(xyz), pose=np.eye(4), kd=cKDTree(pixels))
    tracker = SLAM(k)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    tracker.keyframes.add(old)
    return tracker, current, pose


def test_archive_is_bounded_and_retains_temporal_endpoints():
    bank = RecoveryKeyframes(capacity=8)
    frames = [SimpleNamespace(id=i, timestamp=i / 30) for i in range(3000)]
    for camera in frames:
        bank.add(camera)
    assert len(bank.frames) == 8
    assert bank.frames[0] is frames[0]
    assert frames[-1].timestamp - bank.frames[-1].timestamp < .5
    assert len({f.id for f in bank.frames}) == 8
    assert [f.timestamp for f in bank.frames] == sorted(f.timestamp for f in bank.frames)
    assert all(f is not bank.frames[-1] for f in bank.candidates(frames[-1], bank.frames[-1]))


def test_native_recovery_uses_old_landmarks_without_mutating_map(scene):
    tracker, current, truth = returning_camera(scene)
    old_frames, old_points = list(tracker.map.frames), list(tracker.map.points)
    poses = [f.pose.copy() for f in old_frames]
    evidence, trace = {}, {}
    recovered = tracker.recover_pose(current, evidence, trace)
    assert recovered is not None and evidence['chosen_frame_id'] == 0
    pose, points, indices, _, _ = recovered
    np.testing.assert_allclose(pose, truth, atol=1e-6)
    assert len(points) == len(indices) == 60 and len(set(indices)) == 60
    assert evidence['attempts'][0]['original_inliers'] == 60
    assert tracker.map.frames == old_frames and tracker.map.points == old_points
    assert all(p is None for p in current.pts)
    for camera, before in zip(old_frames, poses):
        np.testing.assert_array_equal(camera.pose, before)
    tracker.map.check_integrity()


@pytest.mark.parametrize('diagnostics', [False, True])
def test_process_recovers_after_normal_failure_and_commits_once(scene, monkeypatch, diagnostics):
    tracker, current, truth = returning_camera(scene)
    monkeypatch.setattr(slam, 'Frame', lambda *a, **kw: current)
    previous = tracker.reference
    image = np.zeros((480, 640, 3), np.uint8)
    camera, result = tracker.process(image, current.id, current.timestamp,
                                     diagnostics=diagnostics, capture_trace=diagnostics)
    assert result.status == 'tracking' and result.recovered_from == 0
    assert result.added_points == 0 and result.inliers == 60
    assert len(tracker.map.frames) == 4 and tracker.map.frames.count(camera) == 1
    assert tracker.reference is current and tracker.reference is not previous
    np.testing.assert_allclose(camera.pose, truth, atol=1e-6)
    assert all(point.frames[-1] is current for point in tracker.map.points)
    tracker.map.check_integrity()
    if diagnostics:
        assert result.diagnostics['normal_tracking_failure']['stage'] == 'provisional_pnp'
        assert result.diagnostics['stages']['recovery']['status'] == 'recovered'
        assert result.diagnostics['stages']['final_pnp']['gate'] == 'accepted'
        assert tracker.last_trace['final_pnp']['T_cw'] == current.pose.tolist()


def test_projection_age_limit_is_bypassed_only_for_the_chosen_recovery_view(scene):
    tracker, current, truth = returning_camera(scene)
    points, indices = tracker.projection_matches(current, truth, [], [])
    assert not points and not indices
    points, indices = tracker.projection_matches(current, truth, [], [],
                                                recovery_reference=tracker.keyframes.frames[0])
    assert len(points) == len(indices) == 60
    assert len(set(points)) == len(set(indices)) == 60
    assert all(point is None for point in current.pts)


def test_recovered_pose_is_captured_between_samples_without_status_transition(scene, monkeypatch, tmp_path):
    tracker, current, _ = returning_camera(scene, 151)
    monkeypatch.setattr(slam, 'Frame', lambda *a, **kw: current)
    image = np.zeros((480, 640, 3), np.uint8)
    _, result = tracker.process(image, current.id, current.timestamp,
                                diagnostics=True, capture_trace=True)
    writer = DiagnosticWriter(tmp_path / 'frames', 140, 160, every=10)
    writer.previous_status = 'tracking'
    writer.write(image, result, tracker.last_trace)
    writer.finish()
    assert result.recovered_from == 0
    assert [sample['frame_id'] for sample in writer.samples] == [151]
    rendered = cv2.imread(str(tmp_path / 'frames/frame-000151.png'))
    assert rendered.shape == (1168, 1280, 3)
    assert (tmp_path / 'frames/frame-000151.json').is_file()


def test_culled_feature_holes_preserve_descriptor_to_landmark_alignment(scene):
    tracker, current, truth = returning_camera(scene)
    old = tracker.keyframes.frames[0]
    for index in range(0, 60, 3):
        old.pts[index].remove_observation(old)
    expected = {i for i, point in enumerate(old.pts) if point is not None}
    recovered = tracker.recover_pose(current)
    assert recovered is not None
    pose, points, indices, _, _ = recovered
    np.testing.assert_allclose(pose, truth, atol=1e-6)
    assert set(indices) == expected
    assert all(point is old.pts[index] for point, index in zip(points, indices))
    tracker.map.check_integrity()


def test_geometrically_valid_small_consensus_cannot_authorize_recovery(scene):
    tracker, current, _ = returning_camera(scene)
    current._kps[:35] = np.random.default_rng(611).uniform([20, 20], [620, 460], (35, 2))
    current.kd = cKDTree(current._kps)
    evidence = {}
    assert tracker.recover_pose(current, evidence) is None
    attempt = evidence['attempts'][0]
    assert attempt['stages']['final_pnp']['gate'] == 'accepted'
    assert attempt['original_inliers'] < 30
    assert attempt['reason'] == 'insufficient original keyframe support for recovery'
    assert all(point is None for point in current.pts)
    tracker.map.check_integrity()


@pytest.mark.parametrize('failure', ['wrong_geometry', 'deleted', 'weak_support', 'disabled'])
def test_failed_recovery_does_not_change_map_or_reference(scene, monkeypatch, failure):
    tracker, current, _ = returning_camera(scene)
    if failure == 'wrong_geometry':
        current._kps = np.random.default_rng(89).uniform([20, 20], [620, 460], (60, 2))
        current.kd = cKDTree(current._kps)
        current.kps = normalize(np.linalg.inv(current.k), current._kps)
    elif failure == 'deleted':
        for point in list(tracker.map.points):
            point.delete_point()
    elif failure == 'weak_support':
        current.des[29:] = np.random.default_rng(71).integers(0, 256, (31, 32), dtype=np.uint8)
    else:
        tracker.recovery = False
    monkeypatch.setattr(slam, 'Frame', lambda *a, **kw: current)
    frames, points, reference = list(tracker.map.frames), list(tracker.map.points), tracker.reference
    pose_before = [f.pose.copy() for f in frames]
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), current.id, current.timestamp, diagnostics=True)
    assert result.status == 'lost' and result.recovered_from is None
    assert tracker.map.frames == frames and tracker.map.points == points and tracker.reference is reference
    for camera, pose in zip(frames, pose_before):
        np.testing.assert_array_equal(camera.pose, pose)
    assert all(point is None for point in current.pts)
    tracker.map.check_integrity()


def test_healthy_tracking_does_not_call_recovery(scene, monkeypatch):
    tracker, current, _ = returning_camera(scene, 3)
    current.des = tracker.reference.des.copy()
    monkeypatch.setattr(slam, 'Frame', lambda *a, **kw: current)
    monkeypatch.setattr(tracker, 'recover_pose', lambda *a, **kw: pytest.fail('Healthy tracking invoked recovery'))
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), current.id, current.timestamp)
    assert result.status == 'tracking' and result.recovered_from is None


def test_at_most_three_pose_candidates_and_native_failures_are_isolated(scene, monkeypatch):
    tracker, current, _ = returning_camera(scene)
    old = tracker.keyframes.frames[0]
    tracker.keyframes.frames = [Camera(id=i, timestamp=i / 30, pts=old.pts, des=old.des)
                               for i in range(10)]
    calls = []
    def failed_solver(*args, **kwargs):
        calls.append(1)
        raise cv2.error('injected native failure')
    monkeypatch.setattr(slam, 'estimate_pose', failed_solver)
    evidence = {}
    assert tracker.recover_pose(current, evidence) is None
    assert len(evidence['attempts']) == len(calls) == 3
    assert evidence['scanned_keyframes'] == 10
    assert all(a['status'] == 'rejected' for a in evidence['attempts'])


def test_duplicate_landmarks_cannot_inflate_recovery_support(scene):
    tracker, current, _ = returning_camera(scene)
    old = tracker.keyframes.frames[0]
    # Defensive check of retrieval's unique-point accounting. This malformed
    # test view is never inserted into the map or used to create observations.
    tracker.keyframes.frames = [Camera(id=0, timestamp=0., des=old.des, pts=[old.pts[0]] * 60)]
    evidence = {}
    assert tracker.recover_pose(current, evidence) is None
    assert evidence['candidates'][0]['mapped_matches'] == 1
    assert evidence['attempts'] == []
