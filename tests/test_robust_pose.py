"""Independent projection, derivative and degeneracy oracles for robust PnP."""

from types import SimpleNamespace
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

import frame
from geometry import condition_points
from robust_pose import huber_blocks, inspect_pose, linearize, robust_refine


def problem():
    """120 known world points, nonidentity T_cw, pixel noise and 24 biased rows."""
    rng = np.random.default_rng(420)
    xyz = rng.uniform([-2, -1.5, 4], [2, 1.5, 10], (120, 3))
    k = np.array([[510., 0, 320], [0, 495., 240], [0, 0, 1.]])
    rotation = cv2.Rodrigues(np.array([.14, -.19, .09]))[0]
    translation = np.array([-.4, .2, .1])
    camera = xyz @ rotation.T + translation
    truth = project_oracle(k, camera)
    pixels = truth + rng.normal(0, .08, truth.shape)
    pixels[:24] += [2.7, .3]
    return xyz, k, rotation, translation, truth, pixels


def project_oracle(k, camera):
    homogeneous = camera @ k.T
    return homogeneous[:, :2] / homogeneous[:, 2:]


def test_block_huber_rotational_invariance_and_zero_residual():
    errors = np.array([[0., 0.], [1., 0.], [3., 4.]])
    weights, cost = huber_blocks(errors)
    np.testing.assert_allclose(weights, [1, 1, .3])
    assert cost == 13.75
    theta = .43
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    rotated, rotated_cost = huber_blocks(errors @ rotation.T)
    np.testing.assert_allclose(rotated, weights)
    np.testing.assert_allclose(rotated_cost, cost)


def test_pixel_jacobian_matches_independent_finite_differences():
    xyz, k, rotation, translation, _, _ = problem()
    k[0, 1] = 17.  # Check skew as well as unequal focal lengths.
    camera = xyz @ rotation.T + translation
    _, jacobian, pivot, radius = linearize(k, camera)
    epsilon = 1e-6
    for axis in range(6):
        step = np.eye(6)[axis] * epsilon
        plus = ((camera - pivot) @ cv2.Rodrigues(step[:3])[0].T + pivot + radius * step[3:])
        minus = ((camera - pivot) @ cv2.Rodrigues(-step[:3])[0].T + pivot - radius * step[3:])
        numeric = (project_oracle(k, plus) - project_oracle(k, minus)) / (2 * epsilon)
        np.testing.assert_allclose(jacobian[:, :, axis], numeric, atol=1e-6, rtol=1e-7)


def test_robust_fit_reduces_known_pose_bias_and_has_monotonic_fixed_support_cost():
    xyz, k, _, _, truth, pixels = problem()
    _, rvec, tvec = cv2.solvePnP(xyz, pixels, k, None, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(xyz, pixels, k, None, rvec, tvec)
    initial_r = cv2.Rodrigues(rvec)[0]
    rotation, translation, evidence = robust_refine(k, xyz, pixels, initial_r, tvec)
    error_before = np.sqrt(np.mean((project_oracle(k, xyz @ initial_r.T + tvec.ravel()) - truth) ** 2))
    error_after = np.sqrt(np.mean((project_oracle(k, xyz @ rotation.T + translation) - truth) ** 2))
    assert error_after < .8 * error_before
    assert evidence['initial']['downweighted_count'] == 24
    assert 1 <= evidence['accepted_steps'] <= evidence['attempted_steps'] <= 3
    for step in evidence['steps']:
        assert step['after_cost_px2'] < step['before_cost_px2']


def test_public_refinement_accepts_robust_improvement_and_preserves_support():
    xyz, k, rotation, translation, truth, pixels = problem()
    camera = SimpleNamespace(k=k, _kps=pixels, w=640, h=480)
    arguments = (camera, xyz, pixels, cv2.Rodrigues(rotation)[0], translation, np.arange(len(xyz)))
    baseline, _, _ = frame.refine_pose(*arguments, robust=False)
    actual, evidence, _ = frame.refine_pose(*arguments)
    assert evidence['selected'] == 'robust'
    np.testing.assert_array_equal(actual['rows'], baseline['rows'])
    def true_rmse(candidate):
        pose = candidate['pose']
        projected = project_oracle(k, xyz @ pose[:3, :3].T + pose[:3, 3])
        return np.sqrt(np.mean((projected - truth) ** 2))
    assert true_rmse(actual) < .8 * true_rmse(baseline)


def test_lower_robust_cost_cannot_discard_checked_correspondences():
    xyz, k, _, _, _, pixels = problem()
    pixels[:24, 0] += .8
    camera = SimpleNamespace(k=k, _kps=pixels, w=640, h=480)
    # This least-squares seed fits every row within three pixels. Robust fitting
    # can lower cost by sacrificing some boundary rows; that proposal must not
    # silently remove the consensus used by the next mapping stage.
    _, rvec, tvec = cv2.solvePnP(xyz, pixels, k, None, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(xyz, pixels, k, None, rvec, tvec)
    candidate, evidence, proposals = frame.refine_pose(camera, xyz, pixels, rvec, tvec, np.arange(len(xyz)))
    report = evidence['robust']
    assert report['proposed_full_cost_px2'] < report['before_full_cost_px2']
    assert report['selection_reason'] == 'seed_support_changed'
    assert len(proposals['robust']['rows']) < len(candidate['rows']) == len(xyz)
    assert evidence['selected'] != 'robust'


@pytest.mark.parametrize('scale,shift', [(1e-4, [1e6, -2e6, 3e6]), (1e5, [1e9, 2e9, -3e9])])
def test_condition_spectrum_and_robust_updates_do_not_depend_on_world_units(scale, shift):
    xyz, k, rotation, translation, _, pixels = problem()
    reference = inspect_pose(k, xyz @ rotation.T + translation, pixels)[0]
    local = condition_points(scale * xyz + shift)
    world_t = scale * translation - rotation @ shift
    local_t = local.local_translation(rotation, world_t).ravel()
    actual = inspect_pose(k, local.points @ rotation.T + local_t, pixels)[0]
    np.testing.assert_allclose(actual['weighted']['singular_values'],
                               reference['weighted']['singular_values'], rtol=2e-5)
    r1, t1, _ = robust_refine(k, xyz, pixels, rotation, translation)
    r2, t2, _ = robust_refine(k, local.points, pixels, rotation, local_t)
    np.testing.assert_allclose(r2, r1, atol=2e-6)
    reconstructed = (local.world_translation(r2, t2).ravel() + r2 @ shift) / scale
    np.testing.assert_allclose(reconstructed, t1, atol=1e-4)


@pytest.mark.parametrize('noise,status,rank', [(0., 'rank_deficient', 5), (1e-7, 'ill_conditioned', 6)])
def test_unobservable_or_weak_line_support_cannot_produce_a_step(noise, status, rank):
    x = np.linspace(-2, 2, 40)
    camera = np.column_stack((x, x, 6 + .1 * x))
    camera += np.random.default_rng(45).normal(0, noise, camera.shape)
    k = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1.]])
    pixels = project_oracle(k, camera) + [2., 0.]
    rotation, translation, evidence = robust_refine(k, camera, pixels, np.eye(3), np.zeros(3))
    assert evidence['initial']['weighted']['rank'] == rank
    assert evidence['stop_reason'] == status
    assert evidence['attempted_steps'] == 0
    np.testing.assert_array_equal(rotation, np.eye(3))
    np.testing.assert_array_equal(translation, np.zeros(3))


def test_planar_points_are_not_blanket_rejected():
    x, y = np.meshgrid(np.linspace(-2, 2, 6), np.linspace(-1, 1, 5))
    camera = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 6.)))
    k = np.diag([500., 500., 1.])
    metrics, *_ = inspect_pose(k, camera, project_oracle(k, camera))
    assert metrics['weighted']['rank'] == 6
    assert metrics['weighted']['status'] == 'well_conditioned'


@pytest.mark.parametrize('failure', ['behind', 'nan', 'coincident'])
def test_invalid_geometry_returns_no_proposal(failure):
    xyz, k, rotation, translation, _, pixels = problem()
    if failure == 'behind':
        translation[2] = -100
    elif failure == 'nan':
        xyz[0, 1] = np.nan
    else:
        xyz[:] = xyz[0]
    new_r, new_t, report = robust_refine(k, xyz, pixels, rotation, translation)
    assert new_r is None and new_t is None
    assert report['stop_reason'] == 'numerical_failure'


@pytest.mark.parametrize('robust', [False, True])
def test_rank_deficient_fit_is_rejected_before_pose_commit(monkeypatch, robust):
    x = np.linspace(-2, 2, 40)
    xyz = np.column_stack((x, x, 6 + .1 * x))
    k = np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1.]])
    camera = SimpleNamespace(k=k, _kps=project_oracle(k, xyz), w=640, h=480, pose=np.eye(4))
    local = condition_points(xyz)
    monkeypatch.setattr(cv2, 'solvePnPRansac', lambda *a, **kw: (
        True, np.zeros((3, 1)), local.local_translation(np.eye(3), np.zeros(3)), np.arange(len(x))[:, None]))
    for method in ('solvePnPRefineLM', 'solvePnPRefineVVS'):
        monkeypatch.setattr(cv2, method, lambda xyz, pixels, k, d, r, t: (r, t))
    points = [SimpleNamespace(point=p.copy(), id=i) for i, p in enumerate(xyz)]
    evidence = {}
    with pytest.raises(frame.TrackingError, match='rank-deficient'):
        frame.estimate_pose(camera, points, np.arange(len(x)), diagnostics=evidence, robust=robust)
    assert evidence['gate'] == 'pose_conditioning'
    assert evidence['refinement']['selected'] is None
    np.testing.assert_array_equal(camera.pose, np.eye(4))
    np.testing.assert_array_equal([p.point for p in points], xyz)


def test_failed_robust_linear_solve_keeps_independently_checked_seed(monkeypatch):
    xyz, k, _, _, _, pixels = problem()
    camera = SimpleNamespace(k=k, _kps=pixels, w=640, h=480)
    points = [SimpleNamespace(point=p, id=i) for i, p in enumerate(xyz)]
    cv2.setRNGSeed(0)
    baseline, rows = frame.estimate_pose(camera, points, np.arange(len(xyz)), robust=False)
    def failed_svd(*args, **kwargs):
        raise np.linalg.LinAlgError('injected failed linear solve')
    monkeypatch.setattr(np.linalg, 'lstsq', failed_svd)
    cv2.setRNGSeed(0)
    evidence = {}
    pose, selected = frame.estimate_pose(camera, points, np.arange(len(xyz)), diagnostics=evidence)
    np.testing.assert_array_equal(pose, baseline)
    np.testing.assert_array_equal(selected, rows)
    assert evidence['refinement']['robust']['stop_reason'] == 'numerical_failure'
    assert not evidence['refinement']['robust']['selected']


def test_invalid_robust_proposal_cannot_replace_validated_seed(monkeypatch):
    xyz, k, _, _, _, pixels = problem()
    camera = SimpleNamespace(k=k, _kps=pixels, w=640, h=480)
    points = [SimpleNamespace(point=p, id=i) for i, p in enumerate(xyz)]
    cv2.setRNGSeed(0)
    baseline, rows = frame.estimate_pose(camera, points, np.arange(len(xyz)), robust=False)
    def bad_proposal(k, xyz, pixels, rotation, translation, enabled):
        _, _, evidence = robust_refine(k, xyz, pixels, rotation, translation, False)
        evidence['accepted_steps'] = 1
        return rotation, np.array([0., 0., -100.]), evidence
    monkeypatch.setattr(frame, 'robust_refine', bad_proposal)
    cv2.setRNGSeed(0)
    evidence = {}
    pose, selected = frame.estimate_pose(camera, points, np.arange(len(xyz)), diagnostics=evidence)
    np.testing.assert_array_equal(pose, baseline)
    np.testing.assert_array_equal(selected, rows)
    assert evidence['refinement']['candidates']['robust']['gate'] == 'refined_pose'
    assert evidence['refinement']['robust']['selection_reason'] == 'proposal_rejected'


def test_failed_initial_condition_check_rejects_pose(monkeypatch):
    xyz, k, _, _, _, pixels = problem()
    camera = SimpleNamespace(k=k, _kps=pixels, w=640, h=480)
    points = [SimpleNamespace(point=p, id=i) for i, p in enumerate(xyz)]
    def failed_svd(*args, **kwargs):
        raise np.linalg.LinAlgError('injected failed condition check')
    monkeypatch.setattr(np.linalg, 'svd', failed_svd)
    with pytest.raises(frame.TrackingError, match='rank-deficient'):
        frame.estimate_pose(camera, points, np.arange(len(xyz)))


def test_coverage_recovery_refits_actual_support_without_widening_gates():
    data = json.loads((Path(__file__).parent / 'fixtures/frame1652_pnp.json').read_text())
    xyz, pixels, pose = (np.array(data[key]) for key in ('xyz', 'pixels', 'seed_T_cw'))
    camera = SimpleNamespace(k=np.array(data['K']), w=1024, h=576)
    seed = frame._pose_candidate(camera, xyz, pixels, cv2.Rodrigues(pose[:3, :3])[0],
        pose[:3, 3], 3., True)
    assert seed['metrics']['gate'] == 'image_coverage'
    candidate, evidence, _ = frame._recover_coverage(camera, xyz, pixels, condition_points(xyz),
        seed, {'selected': None, 'inspected': 'seed'}, {'seed': seed}, 3.)
    assert evidence['coverage_recovery']['accepted']
    assert 1 <= evidence['coverage_recovery']['attempted_rounds'] <= 2
    assert candidate['metrics']['clipped_cost_px2'] < seed['metrics']['clipped_cost_px2']
    local = xyz @ candidate['pose'][:3, :3].T + candidate['pose'][:3, 3]
    errors = np.linalg.norm(project_oracle(camera.k, local) - pixels, axis=1)
    rows = candidate['rows']
    assert np.all(local[rows, 2] > 0) and np.all(errors[rows] <= 3)
    assert len(rows) >= 12 and np.all(np.ptp(pixels[rows], axis=0) / [1024, 576] >= .1)


def test_coverage_recovery_cannot_commit_an_intrinsically_narrow_band():
    xyz, k, rotation, translation, _, _ = problem()
    # World points at varied X/Z remain 3D but project into a deliberately thin
    # horizontal band. A solver must not manufacture image coverage for them.
    xyz[:, 1] *= .001
    rotation, translation = np.eye(3), np.zeros(3)
    pixels = project_oracle(k, xyz)
    camera = SimpleNamespace(k=k, w=640, h=480)
    seed = frame._pose_candidate(camera, xyz, pixels, np.zeros(3), translation, 3., True)
    candidate, evidence, _ = frame._recover_coverage(camera, xyz, pixels, condition_points(xyz),
        seed, {'selected': None, 'inspected': 'seed'}, {'seed': seed}, 3.)
    assert candidate['metrics']['gate'] == 'image_coverage'
    assert evidence['selected'] is None and not evidence['coverage_recovery']['accepted']
    assert evidence['coverage_recovery']['attempted_rounds'] <= 2
