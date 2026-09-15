"""Native PnP regression and transactional handling of failed refinements."""

import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from frame import estimate_pose, refine_pose, TrackingError


def test_frame941_refinement_retains_distributed_support():
    """Reproduce the real numerical failure using only small XYZ/pixel arrays.

    These measurements have no independent pose truth. The regression establishes
    a valid, lower-residual fit under unchanged gates, not real-road accuracy.
    """
    data = json.loads((Path(__file__).parent / 'fixtures/frame941_pnp.json').read_text())
    pixels, xyz = np.array(data['pixels']), np.array(data['xyz'])
    width, height = data['processed_size']
    camera = SimpleNamespace(k=np.array(data['K']), _kps=pixels, w=width, h=height)
    points = [SimpleNamespace(point=point, id=i) for i, point in enumerate(xyz)]
    evidence, trace = {}, {}
    cv2.setRNGSeed(0)
    pose, selected = estimate_pose(camera, points, np.arange(len(points)), diagnostics=evidence, trace=trace)
    assert len(selected) >= 90
    assert min(evidence['span_fraction']) >= .1
    assert evidence['clipped_cost_px2'] < 240
    assert evidence['refinement']['selected'] is not None
    # Calculate the projection from the returned transform independently of the
    # production validation helper and account for every input in the score.
    local = xyz @ pose[:3, :3].T + pose[:3, 3]
    projected = local @ camera.k.T
    errors = np.linalg.norm(projected[:, :2] / projected[:, 2:] - pixels, axis=1)
    assert np.all(local[selected, 2] > 0) and np.all(errors[selected] <= 3)
    np.testing.assert_allclose(evidence['clipped_cost_px2'], np.minimum(errors, 3).dot(np.minimum(errors, 3)))
    assert trace['refinement'][evidence['refinement']['selected']]['rows'] == selected.tolist()


@pytest.mark.parametrize('failure', ['nonfinite', 'native_error', 'behind_camera'])
def test_failed_lm_uses_original_seed_for_native_vvs(scene, monkeypatch, failure):
    mapping, xyz, _ = scene
    camera = mapping.frames[-1]
    rotation = cv2.Rodrigues(camera.pose[:3, :3])[0]
    translation = camera.pose[:3, 3:4].copy() + np.array([[.005], [-.004], [.001]])
    original_r, original_t = rotation.copy(), translation.copy()
    native_vvs = cv2.solvePnPRefineVVS
    received_seed = {}

    def bad_lm(xyz, pixels, k, distortion, rvec, tvec):
        # Native refiner arguments are in/out arrays. Mutating a rejected
        # candidate must not poison the independent fallback's initial pose.
        received_seed.update(rvec=rvec.copy(), tvec=tvec.copy(), xyz=xyz.copy())
        rvec[:] = np.nan if failure == 'nonfinite' else 0
        tvec[:] = 0
        tvec[2] = -100
        if failure == 'native_error':
            raise cv2.error('injected native refinement failure')
        return rvec, tvec

    def checked_vvs(xyz, pixels, k, distortion, rvec, tvec):
        np.testing.assert_array_equal(rvec, received_seed['rvec'])
        np.testing.assert_array_equal(tvec, received_seed['tvec'])
        np.testing.assert_array_equal(xyz, received_seed['xyz'])
        return native_vvs(xyz, pixels, k, distortion, rvec, tvec)

    monkeypatch.setattr(cv2, 'solvePnPRefineLM', bad_lm)
    monkeypatch.setattr(cv2, 'solvePnPRefineVVS', checked_vvs)
    candidate, evidence, _ = refine_pose(camera, xyz, camera._kps, rotation, translation, np.arange(len(xyz)))
    assert evidence['selected'] == 'vvs' and evidence['fallback_attempted']
    np.testing.assert_allclose(candidate['pose'], camera.pose, atol=1e-6)
    np.testing.assert_array_equal(rotation, original_r)
    np.testing.assert_array_equal(translation, original_t)
    mapping.check_integrity()


def test_refinement_cannot_replace_valid_seed_with_higher_cost(scene, monkeypatch):
    mapping, xyz, _ = scene
    camera = mapping.frames[-1]
    rotation = cv2.Rodrigues(camera.pose[:3, :3])[0]
    translation = camera.pose[:3, 3:4].copy()

    def worse_fit(xyz, pixels, k, distortion, rvec, tvec):
        tvec[0] += .01  # Still below the 3-pixel gate, but worse on the same data.
        return rvec, tvec

    monkeypatch.setattr(cv2, 'solvePnPRefineLM', worse_fit)
    candidate, evidence, _ = refine_pose(camera, xyz, camera._kps, rotation, translation, np.arange(len(xyz)))
    assert evidence['reason'] == 'lm_increased_cost'
    assert evidence['selected'] != 'lm'
    np.testing.assert_allclose(candidate['pose'], camera.pose, atol=1e-6)


def test_ransac_mask_does_not_authorize_invalid_pose(scene, monkeypatch):
    mapping, xyz, _ = scene
    camera = mapping.frames[-1]
    before = [frame.pose.copy() for frame in mapping.frames]

    def bad_initial(*args, **kwargs):
        return True, np.zeros((3, 1)), np.array([[0.], [0.], [-100.]]), np.arange(len(xyz))[:, None]

    def failed_refinement(*args, **kwargs):
        raise cv2.error('injected solver failure')

    monkeypatch.setattr(cv2, 'solvePnPRansac', bad_initial)
    monkeypatch.setattr(cv2, 'solvePnPRefineLM', failed_refinement)
    monkeypatch.setattr(cv2, 'solvePnPRefineVVS', failed_refinement)
    evidence = {}
    with pytest.raises(TrackingError, match='invalid refined PnP pose'):
        estimate_pose(camera, mapping.points, np.arange(len(xyz)), diagnostics=evidence, condition=False)
    assert evidence['ransac_inliers'] == len(xyz)
    assert evidence['refinement']['selected'] is None
    assert evidence['refined_inliers'] == 0
    for previous, current in zip(before, mapping.frames):
        np.testing.assert_array_equal(current.pose, previous)
    mapping.check_integrity()


def test_native_failure_can_retain_independently_validated_initial_pose(scene, monkeypatch):
    mapping, xyz, _ = scene
    camera = mapping.frames[-1]

    def failed_refinement(*args, **kwargs):
        raise cv2.error('injected solver failure')

    monkeypatch.setattr(cv2, 'solvePnPRefineLM', failed_refinement)
    monkeypatch.setattr(cv2, 'solvePnPRefineVVS', failed_refinement)
    candidate, evidence, _ = refine_pose(camera, xyz, camera._kps,
        cv2.Rodrigues(camera.pose[:3, :3])[0], camera.pose[:3, 3:4].copy(), np.arange(len(xyz)))
    assert evidence['selected'] == 'ransac_pose'
    np.testing.assert_allclose(candidate['pose'], camera.pose, atol=1e-10)
