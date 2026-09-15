"""Pure camera geometry. Poses map world coordinates into the camera (T_cw)."""

import numpy as np


def pose_rt(rotation, translation):
    """Build a float64 (4,4) rigid transform from R (3,3) and t (3,)."""
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = np.asarray(translation).reshape(3)
    return pose


def valid_pose(pose):
    """Check the SE(3) contract without silently repairing an invalid estimate."""
    pose = np.asarray(pose)
    return (pose.shape == (4, 4) and np.isfinite(pose).all()
            and np.allclose(pose[3], [0, 0, 0, 1], atol=1e-8)
            and np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-6)
            and abs(np.linalg.det(pose[:3, :3]) - 1) < 1e-6)


def add_one(points):
    """Append the homogeneous component to an (N,D) array."""
    points = np.asarray(points, dtype=np.float64)
    return np.column_stack((points, np.ones(len(points))))


def normalize(kinv, pixels):
    """Convert ideal/rectified pixels (N,2) to normalized camera rays (N,2)."""
    rays = add_one(np.asarray(pixels).reshape(-1, 2)) @ kinv.T
    return rays[:, :2] / rays[:, 2:3]


def denormalize(k, point):
    pixel = k @ np.r_[point, 1.0]
    return tuple(np.rint(pixel[:2] / pixel[2]).astype(int))


def project(k, pose, points):
    """Project world points (N,3); return pixels, camera depth, validity mask.

    Invalid pixels are NaN. Callers must apply the returned mask before search.
    """
    camera = add_one(np.asarray(points).reshape(-1, 3)) @ pose[:3].T
    depth = camera[:, 2]
    valid = np.isfinite(camera).all(axis=1) & (depth > 1e-9)
    pixels = np.full((len(camera), 2), np.nan)
    # Behind-camera points may land inside image bounds, so reject depth first.
    projected = camera[valid] @ k.T
    pixels[valid] = projected[:, :2] / projected[:, 2:3]
    valid &= np.isfinite(pixels).all(axis=1)
    return pixels, depth, valid


def triangulate(pose1, pose2, points1, points2):
    """Return homogeneous world points (N,4) from normalized observations (N,2)."""
    points1 = np.asarray(points1, dtype=np.float64).reshape(-1, 2)
    points2 = np.asarray(points2, dtype=np.float64).reshape(-1, 2)
    if points1.shape != points2.shape:
        raise ValueError('Triangulation observations must have equal shapes')
    if not len(points1):
        return np.empty((0, 4))
    # Each camera contributes two ray constraints. Mixing normalized rays with K
    # here, or reusing the first pose for the second ray, destroys the baseline.
    rows = np.stack((points1[:, 0, None] * pose1[2] - pose1[0],
                     points1[:, 1, None] * pose1[2] - pose1[1],
                     points2[:, 0, None] * pose2[2] - pose2[0],
                     points2[:, 1, None] * pose2[2] - pose2[1]), axis=1)
    return np.linalg.svd(rows)[2][:, -1, :]


def triangulate_valid(pose1, pose2, rays1, rays2, k, max_error=3.0, min_angle=1.0):
    """Return world XYZ and mask after two-view depth, parallax and pixel checks.

    min_angle is degrees; max_error is pixels in the processed image.
    """
    homogeneous = triangulate(pose1, pose2, rays1, rays2)
    valid = np.isfinite(homogeneous).all(axis=1) & (np.abs(homogeneous[:, 3]) > 1e-12)
    xyz = np.full((len(homogeneous), 3), np.nan)
    xyz[valid] = homogeneous[valid, :3] / homogeneous[valid, 3:4]
    # Rotate camera rays into the same world basis before measuring parallax.
    directions = []
    for pose, rays in [(pose1, rays1), (pose2, rays2)]:
        direction = add_one(np.asarray(rays).reshape(-1, 2)) @ pose[:3, :3]
        direction /= np.linalg.norm(direction, axis=1, keepdims=True)
        directions.append(direction)
        pixels, _, visible = project(k, pose, xyz)
        target = add_one(np.asarray(rays).reshape(-1, 2)) @ k.T
        target = target[:, :2] / target[:, 2:3]
        valid &= visible & (np.linalg.norm(pixels - target, axis=1) <= max_error)
    angle = np.degrees(np.arccos(np.clip(np.sum(directions[0] * directions[1], axis=1), -1, 1)))
    valid &= angle >= min_angle
    return xyz, valid
