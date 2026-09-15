"""Centred consensus recovery: native failures, fixed evidence and unchanged gates."""

import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from frame import TrackingError, estimate_pose


def saved_problem():
    data = json.loads((Path(__file__).parent / 'fixtures/frame874_pnp.json').read_text())
    xyz, pixels = np.array(data['xyz']), np.array(data['pixels'])
    camera = SimpleNamespace(k=np.array(data['K']), _kps=pixels,
                             w=data['processed_size'][0], h=data['processed_size'][1])
    points = [SimpleNamespace(id=i, point=point) for i, point in enumerate(xyz)]
    return camera, points, xyz


def test_native_consensus_recovery_rescues_failed_centred_seed():
    camera, points, xyz = saved_problem()
    evidence, trace = {}, {}
    cv2.setRNGSeed(0)
    pose, selected = estimate_pose(camera, points, np.arange(len(points)),
                                   diagnostics=evidence, trace=trace, condition=True)
    assert evidence['ransac_inliers'] >= 100
    assert evidence['refinement']['seed_recovery']['status'] == 'accepted'
    assert any(name.startswith('sqpnp_') and metrics['status'] == 'accepted'
               for name, metrics in evidence['refinement']['candidates'].items())
    assert len(selected) >= 110 and min(evidence['span_fraction']) >= .1
    assert evidence['clipped_cost_px2'] < 370
    # Use the exported world transform, not the solver's temporary local frame,
    # to independently verify physical visibility and every selected residual.
    camera_xyz = xyz @ pose[:3, :3].T + pose[:3, 3]
    homogeneous = camera_xyz @ camera.k.T
    error = np.linalg.norm(homogeneous[:, :2] / homogeneous[:, 2:] - camera._kps, axis=1)
    assert np.all(camera_xyz[selected, 2] > 1e-9)
    assert np.all(error[selected] <= 3.)
    np.testing.assert_array_equal(trace['refined_rows'], selected)
    np.testing.assert_array_equal([point.point for point in points], xyz)


def test_consensus_refits_actual_support_with_monotonic_full_input_cost():
    camera, points, _ = saved_problem()
    evidence = {}
    cv2.setRNGSeed(0)
    estimate_pose(camera, points, np.arange(len(points)), diagnostics=evidence, condition=True)
    consensus = evidence['refinement']['consensus']
    assert 1 <= consensus['accepted_rounds'] <= consensus['attempted_rounds'] <= 2
    assert consensus['rounds'][0]['input_count'] != evidence['ransac_inliers']
    for proposal in consensus['rounds']:
        if proposal['accepted']:
            assert proposal['proposed_cost_px2'] < proposal['before_cost_px2']


def test_consensus_failure_keeps_valid_pose_and_excludes_outlier_rows(scene, monkeypatch):
    mapping, xyz, _ = scene
    camera = mapping.frames[-1]
    camera._kps[:8] += 50
    rotation = cv2.Rodrigues(camera.pose[:3, :3])[0]
    from geometry import condition_points
    conditioning = condition_points(xyz)
    translation = conditioning.local_translation(camera.pose[:3, :3], camera.pose[:3, 3])
    fits = []

    def initial(*args, **kwargs):
        # Deliberately misleading mask: only the projected pose can identify the
        # eight correspondence outliers. The seed itself is the known camera.
        return True, rotation.copy(), translation.copy(), np.arange(len(xyz))[:, None]

    def refiner(points, pixels, k, distortion, rvec, tvec):
        fits.append(pixels.copy())
        if len(fits) == 1:
            return rvec, tvec
        rvec[:] = np.nan
        tvec[:] = np.nan
        raise cv2.error('injected consensus refinement failure')

    monkeypatch.setattr(cv2, 'solvePnPRansac', initial)
    monkeypatch.setattr(cv2, 'solvePnPRefineLM', refiner)
    monkeypatch.setattr(cv2, 'solvePnPRefineVVS', refiner)
    evidence = {}
    pose, selected = estimate_pose(camera, mapping.points, np.arange(len(xyz)),
                                   diagnostics=evidence, condition=True)
    np.testing.assert_array_equal(selected, np.arange(8, len(xyz)))
    np.testing.assert_allclose(pose, camera.pose, atol=1e-10)
    assert 1 <= evidence['refinement']['consensus']['attempted_rounds'] <= 2
    for pixels in fits[1:]:
        np.testing.assert_array_equal(pixels, camera._kps[8:])


@pytest.mark.parametrize('failure', ['native_error', 'no_solution', 'nonfinite'])
def test_failed_seed_recovery_does_not_authorize_invalid_pose(monkeypatch, failure):
    camera, points, _ = saved_problem()
    calls = []

    def failed_solver(xyz, pixels, k, distortion, flags):
        calls.append(len(xyz))
        if failure == 'native_error':
            raise cv2.error('injected SQPnP failure')
        if failure == 'no_solution':
            return False, None, None
        return True, np.full((3, 1), np.nan), np.full((3, 1), np.nan)

    monkeypatch.setattr(cv2, 'solvePnP', failed_solver)
    evidence = {}
    cv2.setRNGSeed(0)
    with pytest.raises(TrackingError, match='invalid refined'):
        estimate_pose(camera, points, np.arange(len(points)), diagnostics=evidence, condition=True)
    assert calls == [evidence['ransac_inliers']]
    assert evidence['refinement']['selected'] is None
    assert evidence['refinement']['seed_recovery']['status'] in ('failed', 'rejected')


def test_valid_centred_fit_does_not_invoke_seed_recovery(scene, monkeypatch):
    mapping, xyz, _ = scene

    def unexpected_solver(*args, **kwargs):
        pytest.fail('A validated primary fit must not spend a recovery solve')

    monkeypatch.setattr(cv2, 'solvePnP', unexpected_solver)
    evidence = {}
    pose, _ = estimate_pose(mapping.frames[-1], mapping.points, np.arange(len(xyz)),
                             diagnostics=evidence, condition=True)
    assert not evidence['refinement']['seed_recovery']['attempted']
    np.testing.assert_allclose(pose, mapping.frames[-1].pose, atol=1e-6)
