"""Analytic views test selection independently of the production projector."""

import numpy as np
import pytest

from dmap import Map
from keyframes import KeyframePolicy
from point import Point


class Camera:
    pass


def scene(centre=.01, yaw=0., timestamp=.4, current_ids=range(80), scale=1.):
    rng = np.random.default_rng(18)
    xyz = rng.uniform([-1.5, -1., 4.], [1.5, 1., 8.], (120, 3)) * scale
    mapping = Map()
    for identifier, (x, angle, time) in enumerate(((0., 0., 0.), (.01, 0., .2), (centre, yaw, timestamp))):
        camera = Camera()
        camera.id, camera.timestamp = identifier, time
        c, s = np.cos(np.radians(angle)), np.sin(np.radians(angle))
        rotation = np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])
        camera.pose = np.eye(4)
        camera.pose[:3, :3] = rotation
        camera.pose[:3, 3] = -rotation @ np.array([x * scale, 0., 0.])
        camera.k = np.array([[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]])
        # Direct analytic camera coordinates avoid calling production project().
        local = (xyz - [x * scale, 0., 0.]) @ rotation.T
        camera.kps = local[:, :2] / local[:, 2, None]
        camera._kps = camera.kps * 500 + [320, 240]
        camera.pts = [None] * len(xyz)
        mapping.add_frame(camera)
    for identifier, location in enumerate(xyz):
        point = Point(mapping, location, [100, 150, 200])
        for camera in mapping.frames:
            if camera.id == 0 or (camera.id == 1 and identifier < 80) or (camera.id == 2 and identifier in current_ids):
                point.add_observation(camera, identifier)
    mapping.keyframes.initialize(mapping.frames[:2])
    points = [p for p in mapping.frames[2].pts if p is not None]
    return mapping, mapping.frames[2], points


def test_initialization_anchors_and_redundant_view():
    mapping, frame, points = scene()
    before = mapping.trajectory.pose_rows()
    decision = mapping.keyframes.consider(frame, points)
    assert not decision['selected'] and decision['reasons'] == ['redundant_view']
    assert decision['overlap'] == 1.
    assert decision['median_parallax_degrees'] < 1e-10
    assert mapping.keyframes.frames == mapping.frames[:2]
    assert mapping.trajectory.pose_rows() == before
    assert mapping.keyframes.summary()['affects_estimation'] is False


@pytest.mark.parametrize('scale', [.01, 1., 1e4])
def test_translation_parallax_is_scale_invariant(scale):
    mapping, frame, points = scene(centre=.4, scale=scale)
    decision = mapping.keyframes.consider(frame, points)
    assert decision['selected'] and decision['reasons'] == ['parallax']
    assert 3. < decision['median_parallax_degrees'] < 5.
    assert mapping.keyframes.frames[-1] is frame


def test_rotation_does_not_masquerade_as_parallax():
    mapping, frame, points = scene(yaw=15.)
    decision = mapping.keyframes.consider(frame, points)
    assert decision['reasons'] == ['rotation']
    assert decision['median_parallax_degrees'] < 1e-10
    assert decision['rotation_degrees'] == pytest.approx(15.)


@pytest.mark.parametrize(('kwargs', 'reasons'), [
    ({'current_ids': range(40, 120)}, ['overlap_drop']),
    ({'current_ids': range(50)}, ['overlap_drop', 'support_drop']),
    ({'timestamp': 1.2}, ['maximum_interval']),
    ({'centre': .4, 'timestamp': .25}, ['minimum_interval']),
    ({'current_ids': range(20), 'timestamp': 3.}, ['insufficient_verified_support']),
])
def test_support_overlap_and_time_gates(kwargs, reasons):
    mapping, frame, points = scene(**kwargs)
    decision = mapping.keyframes.consider(frame, points)
    assert decision['reasons'] == reasons
    assert decision['selected'] == (reasons[0] not in ('minimum_interval', 'insufficient_verified_support'))


def test_only_estimator_support_counts_and_recovery_requires_quality():
    mapping, frame, points = scene(timestamp=.25)
    decision = mapping.keyframes.consider(frame, points[:20], recovered=True)
    assert decision['reasons'] == ['insufficient_verified_support']
    decision = mapping.keyframes.consider(frame, points, recovered=True)
    assert decision['selected'] and decision['reasons'] == ['recovery']
    with pytest.raises(ValueError, match='advance'):
        mapping.keyframes.consider(frame, points)


def test_culled_bad_and_behind_camera_observations_do_not_supply_support():
    mapping, frame, points = scene(timestamp=3.)
    for point in points[:30]:
        point.remove_observation(frame)
    frame._kps[30:60] += 100
    decision = mapping.keyframes.consider(frame, points)
    assert decision['support'] == 20 and not decision['selected']
    for point in points[60:]:
        point.point[2] = -5
    assert mapping.keyframes.consider(frame, points)['support'] == 0


def test_unaccepted_frame_and_invalid_policy_rejected():
    mapping, frame, points = scene()
    mapping.frames.pop()
    with pytest.raises(ValueError, match='accepted'):
        mapping.keyframes.consider(frame, points)
    for changes in ({'min_support': 1}, {'min_support': 30.5}, {'min_overlap': 2},
                    {'min_interval_seconds': 2}, {'max_residual_pixels': float('nan')}):
        with pytest.raises(ValueError):
            KeyframePolicy(**changes)
