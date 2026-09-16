"""Pure camera geometry. Poses map world coordinates into the camera (T_cw)."""

import numpy as np
from dataclasses import dataclass


@dataclass
class ConditionedPoints:
    """Temporary isotropic coordinates for a native pose solver, not a new map.

    X_local = (X_world - centre) / scale. Rotations are unchanged; translations
    must be converted because camera coordinates also scale by this positive s.
    """

    points: np.ndarray
    centre: np.ndarray
    scale: float
    method: str = 'centred_rms_radius'

    def world_translation(self, rotation, local_translation):
        """Convert t_local to t_world = s*t_local - R*centre, shape (3,1)."""
        return (self.scale * np.asarray(local_translation).reshape(3)
                - rotation @ self.centre).reshape(3, 1)

    def local_translation(self, rotation, world_translation):
        """Convert a world-to-camera seed to the centred/scaled solver frame."""
        return ((rotation @ self.centre + np.asarray(world_translation).reshape(3))
                / self.scale).reshape(3, 1)

    def metadata(self):
        return {'method': self.method, 'centre_world': self.centre.tolist(),
                'scale_world': self.scale, 'solver_coordinates': '(X_world - centre_world) / scale_world'}


def condition_points(points):
    """Centre finite world XYZ (N,3) and give it unit RMS distance from centre.

    Use an anchor and scaled offsets to avoid summing large world coordinates
    or squaring their absolute magnitudes. Keep one isotropic scale so this
    remains a camera similarity transform; per-axis whitening would change rays.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Pose conditioning requires finite, nonempty world XYZ (N,3)')
    with np.errstate(over='ignore', invalid='ignore', under='ignore'):
        offsets = points - points[0]
        extent = np.max(np.abs(offsets))
        if not np.isfinite(extent):
            raise ValueError('World coordinate span exceeds floating-point range')
        if extent == 0:
            raise ValueError('Coincident world points have no usable scale for pose fitting')
        centre = points[0] + extent * np.mean(offsets / extent, axis=0)
        centred = points - centre
        radius = np.max(np.abs(centred))
        if not np.isfinite(radius) or radius == 0:
            raise ValueError('World point spread cannot be represented for pose fitting')
        scale = float(radius * np.sqrt(np.mean(np.sum((centred / radius) ** 2, axis=1))))
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('World point scale cannot be represented for pose fitting')
        local = np.ascontiguousarray(centred / scale)
    if not np.isfinite(local).all():
        raise ValueError('Conditioned world points are non-finite')
    return ConditionedPoints(local, centre, scale)


def spatial_support(pixels, width, height):
    """Describe image support of (N,2) pixels without changing acceptance gates.

    A bounding box can be large because of a few isolated points. A 4x4 count
    grid, central-90% spans and the smaller covariance axis expose concentration
    and thin/diagonal bands. Empty sky/road cells are not assumed observable.
    """
    pixels = np.asarray(pixels, dtype=np.float64).reshape(-1, 2)
    if width <= 0 or height <= 0:
        raise ValueError('Spatial support requires positive image dimensions')
    normalized = pixels / [width, height]
    inside = np.isfinite(normalized).all(axis=1) & (normalized >= 0).all(axis=1) & (normalized < 1).all(axis=1)
    points = normalized[inside]
    counts = np.zeros(16, dtype=int)
    central_span = np.zeros(2)
    minor_std = 0.
    if len(points):
        cells = np.floor(points * 4).astype(int)
        counts = np.bincount(cells[:, 1] * 4 + cells[:, 0], minlength=16)
        central_span = np.percentile(points, 95, axis=0) - np.percentile(points, 5, axis=0)
        centered = points - points.mean(axis=0)
        covariance = centered.T @ centered / len(points)
        minor_std = float(np.sqrt(max(0., np.linalg.eigvalsh(covariance)[0])))
    weights = counts / len(points) if len(points) else np.zeros(16)
    # Inverse concentration equals the occupied-cell count for uniform support,
    # and tends toward one when nearly every observation lies in a single cell.
    return {'input_count': len(pixels), 'in_image_count': len(points),
            'outside_or_nonfinite_count': int((~inside).sum()),
            'grid_shape': [4, 4], 'grid_counts': counts.reshape(4, 4).tolist(),
            'occupied_cells': int(np.count_nonzero(counts)),
            'effective_cells': float(1. / np.dot(weights, weights)) if len(points) else 0.,
            'largest_cell_fraction': float(weights.max()),
            'central_90_span_fraction': central_span.tolist(),
            'minor_axis_std_fraction': minor_std}


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
