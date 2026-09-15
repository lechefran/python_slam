import cv2
import numpy as np
import pytest

from frame import recover_relative, TrackingError
from geometry import normalize, pose_rt, project, triangulate, triangulate_valid, valid_pose
from slam import camera_parameters


def test_two_camera_triangulation_with_rotation():
    rng = np.random.default_rng(7)
    xyz = rng.uniform([-1, -1, 4], [1, 1, 8], (100, 3))
    first = pose_rt(cv2.Rodrigues(np.array([.02, -.03, .01]))[0], [.1, .2, 0])
    second = pose_rt(cv2.Rodrigues(np.array([-.03, .06, .02]))[0], [-.7, .1, .1])
    rays = []
    for pose in [first, second]:
        camera = (pose[:3, :3] @ xyz.T).T + pose[:3, 3]
        rays.append(camera[:, :2] / camera[:, 2:3])
    homogeneous = triangulate(first, second, *rays)
    np.testing.assert_allclose(homogeneous[:, :3] / homogeneous[:, 3:4], xyz, atol=1e-10)
    # Translation of the world origin must not turn physical quality into a
    # radius-from-origin test through a homogeneous-coordinate threshold.
    shift = np.array([1000., 500, -800])
    shifted = []
    for pose in [first, second]:
        changed = pose.copy()
        changed[:3, 3] -= changed[:3, :3] @ shift
        shifted.append(changed)
    k = np.diag([500., 500, 1])
    recovered, good = triangulate_valid(*shifted, *rays, k)
    assert good.all()
    np.testing.assert_allclose(recovered, xyz + shift, atol=1e-6)


def test_depth_and_degenerate_geometry():
    k = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
    with np.errstate(divide='raise', invalid='raise'):
        uv, depth, valid = project(k, np.eye(4), [[0, 0, 3], [0, 0, -3], [0, 0, 0]])
        assert valid.tolist() == [True, False, False]
        np.testing.assert_allclose(uv[0], [320, 240])
    rays = np.array([[.1, .1], [.2, -.1]])
    _, good = triangulate_valid(np.eye(4), np.eye(4), rays, rays, k)
    assert not good.any()
    assert triangulate(np.eye(4), np.eye(4), [], []).shape == (0, 4)


def test_resize_and_calibration():
    w, h, k, _ = camera_parameters(1920, 1080)
    assert (w, h) == (1024, 576)
    np.testing.assert_allclose(k, [[280, 0, 512], [0, 280, 288], [0, 0, 1]])
    calibration = {'model': 'pinhole', 'width': 641, 'height': 479,
                   'K': [[500, 0, 300], [0, 510, 240], [0, 0, 1]]}
    w, h, k, _ = camera_parameters(641, 479, max_width=320, calibration=calibration)
    np.testing.assert_allclose(k, np.diag([w/641, h/479, 1]) @ calibration['K'])
    with pytest.raises(ValueError, match='dimensions'):
        camera_parameters(640, 480, calibration=calibration)
    with pytest.raises(ValueError):
        camera_parameters(640, 480, focal=float('nan'))


def test_essential_pose_direction_and_cheirality():
    rng = np.random.default_rng(42)
    xyz = rng.uniform([-2, -1.4, 4], [2, 1.4, 8], (200, 3))
    rotation = cv2.Rodrigues(np.array([.03, -.04, .01]))[0]
    translation = np.array([-.7, .08, .03])
    camera = (rotation @ xyz.T).T + translation
    old = xyz[:, :2] / xyz[:, 2:3]
    new = camera[:, :2] / camera[:, 2:3]
    cv2.setRNGSeed(0)
    pose, inliers = recover_relative(old, new, 500)
    assert valid_pose(pose) and inliers.sum() > 180
    np.testing.assert_allclose(pose[:3, :3], rotation, atol=1e-5)
    np.testing.assert_allclose(pose[:3, 3], translation / np.linalg.norm(translation), atol=1e-5)
    with pytest.raises(TrackingError):
        recover_relative(old[:4], new[:4], 500)
