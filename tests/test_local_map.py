"""Live covisibility selection and native constrained-graph checks."""

from copy import copy

import numpy as np
import pytest


def selected_map(scene):
    mapping, _, _ = scene
    mapping.local_map_policy = 'shared'
    mapping.keyframes.initialize(mapping.frames[:2])
    return mapping


def test_shared_neighbors_rank_live_support_with_deterministic_ties(scene):
    mapping = selected_map(scene)
    local, evidence = mapping.select_local_frames(2)
    assert {f.id for f in local} == {1, 2}
    assert evidence['neighbors'] == [{'frame_id': 1, 'shared_landmarks': 60}]
    # A once-useful neighbor loses observations; scores must reflect the live
    # reciprocal links immediately, without an asynchronously stale cache.
    for point in mapping.points[:55]:
        point.remove_observation(mapping.frames[1])
    local, evidence = mapping.select_local_frames(2)
    assert {f.id for f in local} == {0, 2}
    assert evidence['neighbors'] == [{'frame_id': 0, 'shared_landmarks': 60}]
    assert len(mapping.select_local_frames(1)[0]) == 1


def test_disconnected_recent_keyframe_is_excluded(scene):
    mapping = selected_map(scene)
    points = list(mapping.points)
    for identifier in (3, 4):
        frame = copy(mapping.frames[2])
        frame.id, frame.timestamp, frame.pts = identifier, float(identifier), [None] * len(points)
        mapping.add_frame(frame)
        for index, point in enumerate(points):
            point.add_observation(frame, index)
        if identifier == 3:
            assert mapping.keyframes.consider(frame, points)['selected']
            unrelated = frame
    for point in points:
        point.remove_observation(unrelated)
    local, evidence = mapping.select_local_frames(3)
    assert {f.id for f in local} == {0, 1, 4}
    assert unrelated not in local
    assert [r['frame_id'] for r in evidence['neighbors']] == [1, 0]
    mapping.check_integrity()


def test_maturity_and_deleted_landmarks_do_not_inflate_scores(scene):
    mapping = selected_map(scene)
    mapping.landmark_maturity = True
    for point in mapping.points[:20]:
        point._set_state('active')
    assert mapping.select_local_frames(2)[1]['neighbors'][0]['shared_landmarks'] == 20
    for point in list(mapping.points[:15]):
        point.delete_point()
    assert mapping.select_local_frames(2)[1]['neighbors'] == []


@pytest.mark.parametrize('budget', [24, 60])
def test_native_shared_graph_retains_fixed_boundary_and_corrects_pose(scene, budget):
    mapping = selected_map(scene)
    anchors = [f.pose.copy() for f in mapping.frames[:2]]
    mapping.frames[-1].pose[0, 3] += .1
    result = mapping.optimize(local_window=1, iterations=15, max_points=budget)
    assert result.status == 'accepted', result
    assert result.local_map['local_frame_ids'] == [2]
    assert result.local_map['fixed_frame_ids'] == [0, 1]
    assert result.local_map['graph_points'] == budget and result.edges == 3 * budget
    assert result.after_chi2 < result.before_chi2 * .001
    np.testing.assert_allclose(mapping.frames[-1].pose[:3, 3], [-.8, 0, 0], atol=1e-5)
    for frame, expected in zip(mapping.frames[:2], anchors):
        np.testing.assert_array_equal(frame.pose, expected)
    for frame, record in zip(mapping.frames, mapping.trajectory.accepted):
        np.testing.assert_array_equal(frame.pose, record.T_cw)


def test_missing_baseline_is_skipped_without_writeback(scene):
    mapping = selected_map(scene)
    for point in mapping.points:
        point.remove_observation(mapping.frames[0])
    before = [f.pose.copy() for f in mapping.frames]
    history = mapping.trajectory.records
    result = mapping.optimize(local_window=1)
    assert result.status == 'skipped' and result.reason == 'component lacks a fixed nonzero baseline'
    assert history == mapping.trajectory.records
    for frame, pose in zip(mapping.frames, before):
        np.testing.assert_array_equal(frame.pose, pose)


@pytest.mark.parametrize('window', [0, -1, 1.5, True])
def test_invalid_window_rejected(scene, window):
    with pytest.raises(ValueError):
        selected_map(scene).select_local_frames(window)
