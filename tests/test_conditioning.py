"""Independent camera oracles for changing world origin and monocular units."""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from frame import estimate_pose, refine_pose, TrackingError
from geometry import condition_points


def camera_problem():
    rng = np.random.default_rng(28)
    xyz = rng.uniform([-2, -1.5, 4], [2, 1.5, 10], (120, 3))
    rotation = cv2.Rodrigues(np.array([.14, -.19, .09]))[0]
    translation = np.array([-.4, .2, .1])
    local = xyz @ rotation.T + translation
    k = np.array([[510., 0, 320], [0, 495., 240], [0, 0, 1.]])
    # Derive image observations without production projection/normalization.
    pixels = np.column_stack((510 * local[:, 0] / local[:, 2] + 320,
                               495 * local[:, 1] / local[:, 2] + 240))
    return xyz, rotation, translation, SimpleNamespace(k=k, _kps=pixels, w=640, h=480)


@pytest.mark.parametrize('scale,shift', [
    (1., [0., 0., 0.]),
    (1., [1e9, -2e9, 3e9]),
    (1e-5, [0., 0., 0.]),
    (1e5, [1e9, 2e9, -3e9]),
    (1e-4, [1e6, -2e6, 3e6]),
])
def test_native_pnp_preserves_pose_and_outlier_rows_across_world_coordinates(scale, shift):
    xyz, rotation, translation, camera = camera_problem()
    camera._kps[:12] += 50  # Correspondence outliers stay the same in every world frame.
    shift = np.array(shift)
    changed = scale * xyz + shift
    points = [SimpleNamespace(id=i, point=point.copy()) for i, point in enumerate(changed)]
    evidence = {}
    cv2.setRNGSeed(0)
    pose, selected = estimate_pose(camera, points, np.arange(len(xyz)), diagnostics=evidence, condition=True)
    np.testing.assert_array_equal(selected, np.arange(12, 120))
    np.testing.assert_allclose(pose[:3, :3], rotation, atol=1e-6)
    # For X' = a*X + b, the equivalent pose is R'=R, t'=a*t-R*b.
    # Compare in original scene units so tiny/large scales cannot hide errors.
    recovered_translation = (pose[:3, 3] + pose[:3, :3] @ shift) / scale
    np.testing.assert_allclose(recovered_translation, translation, atol=1e-4)
    np.testing.assert_allclose(np.array([point.point for point in points]), changed, rtol=0, atol=0)
    assert evidence['conditioning']['scale_world'] > 0
    assert evidence['conditioning'] == evidence['refinement']['conditioning']


def test_existing_world_pose_is_converted_for_refinement_and_returned_in_world_units():
    xyz, rotation, translation, camera = camera_problem()
    scale, shift = 1e-3, np.array([5000., -2000., 3000.])
    changed = scale * xyz + shift
    # The seed is slightly wrong but already geometrically supported. The public
    # refiner must accept this world-space tvec, not misread it as local units.
    seed_rotation = cv2.Rodrigues(rotation)[0]
    seed_translation = (scale * (translation + [.002, -.002, .001]) - rotation @ shift).reshape(3, 1)
    before = seed_translation.copy()
    candidate, evidence, _ = refine_pose(camera, changed, camera._kps, seed_rotation,
                                          seed_translation, np.arange(len(xyz)), condition=True)
    assert evidence['selected'] is not None
    pose = candidate['pose']
    np.testing.assert_allclose(pose[:3, :3], rotation, atol=1e-6)
    np.testing.assert_allclose((pose[:3, 3] + pose[:3, :3] @ shift) / scale, translation, atol=1e-5)
    np.testing.assert_array_equal(seed_translation, before)


@pytest.mark.parametrize('scale', [1e-200, 1e200])
def test_conditioning_avoids_squaring_extreme_world_magnitudes(scale):
    # Only normalization is tested at these extreme units. Existing world-depth
    # thresholds and native/map precision limits still constrain actual SLAM.
    points = np.array([[1., 2., 3.], [2., -1., 4.], [-2., 1., -3.], [3., 3., 1.]]) * scale
    with np.errstate(all='raise'):
        result = condition_points(points)
    assert np.isfinite(result.points).all()
    np.testing.assert_allclose(result.points.mean(axis=0), 0, atol=1e-15)
    np.testing.assert_allclose(np.mean(np.sum(result.points ** 2, axis=1)), 1.)
    np.testing.assert_allclose((result.scale * result.points + result.centre) / scale, points / scale, atol=1e-14)


@pytest.mark.parametrize('kind', ['coincident', 'unrepresentable_spread', 'nan', 'infinity', 'overflowing_span'])
def test_invalid_conditioning_stops_before_native_solver(kind, monkeypatch):
    xyz, _, _, camera = camera_problem()
    if kind == 'coincident':
        xyz[:] = [1., 2., 3.]
    elif kind == 'unrepresentable_spread':
        xyz = np.full_like(xyz, 1e20) + 1.  # Variation was already lost in float64.
    elif kind == 'nan':
        xyz[0, 0] = np.nan
    elif kind == 'infinity':
        xyz[0, 0] = np.inf
    else:
        xyz[0, 0], xyz[1, 0] = -1e308, 1e308

    def should_not_run(*args, **kwargs):
        pytest.fail('Invalid 3D coordinates reached native RANSAC')

    monkeypatch.setattr(cv2, 'solvePnPRansac', should_not_run)
    evidence = {}
    points = [SimpleNamespace(id=i, point=point) for i, point in enumerate(xyz)]
    with pytest.raises(TrackingError):
        estimate_pose(camera, points, np.arange(len(points)), diagnostics=evidence, condition=True)
    assert evidence['status'] == 'rejected' and evidence['gate'] == 'conditioning'


def test_nonfinite_pixels_stop_before_native_solver(monkeypatch):
    xyz, _, _, camera = camera_problem()
    camera._kps[0, 0] = np.nan

    def should_not_run(*args, **kwargs):
        pytest.fail('Non-finite pixels reached native RANSAC')

    monkeypatch.setattr(cv2, 'solvePnPRansac', should_not_run)
    points = [SimpleNamespace(id=i, point=point) for i, point in enumerate(xyz)]
    with pytest.raises(TrackingError, match='finite pixel'):
        estimate_pose(camera, points, np.arange(len(points)))
