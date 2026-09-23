"""Synthetic camera/landmark observations independent of production projection."""

import numpy as np
import pytest
from scipy.spatial import cKDTree

from dmap import Map
from geometry import normalize
from point import Point


class Camera:
    pass


@pytest.fixture
def scene(request):
    rng = np.random.default_rng(4)
    xyz = rng.uniform([-1.5, -1, 4], [1.5, 1, 8], (60, 3))
    k = np.array([[500., 0, 320], [0, 510, 240], [0, 0, 1]])
    mapping = Map()
    for number in range(3):
        camera = Camera()
        camera.id = getattr(request, "param", [0, 1, 2])[number]
        camera.timestamp = number / 30
        camera.k = k
        camera.pose = np.eye(4)
        camera.pose[0, 3] = -number * 0.4
        camera.h, camera.w = 480, 640
        # Explicit camera coordinates provide an oracle for graph residuals.
        local = xyz + camera.pose[:3, 3]
        camera._kps = np.column_stack((500 * local[:, 0] / local[:, 2] + 320,
                                      510 * local[:, 1] / local[:, 2] + 240))
        camera.kps = normalize(np.linalg.inv(k), camera._kps)
        camera.des = rng.integers(0, 256, (len(xyz), 32), dtype=np.uint8)
        camera.pts = [None] * len(xyz)
        camera.kd = cKDTree(camera._kps)
        mapping.add_frame(camera)
    for index, location in enumerate(xyz):
        point = Point(mapping, location, [255, 0, 0])
        for camera in mapping.frames:
            point.add_observation(camera, index)
    return mapping, xyz, k
