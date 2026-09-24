"""Retirement preserves geometry and both directions of observation ownership."""
from copy import copy

import numpy as np
import pytest


def retirement_scene(scene):
    mapping, _, _ = scene
    template = mapping.frames[-1]
    for identifier in range(3, 10):
        frame = copy(template)
        frame.id, frame.timestamp = identifier, float(identifier)
        frame.pts = [None] * len(template.pts)
        mapping.add_frame(frame)
        for index, point in enumerate(mapping.points):
            point.add_observation(frame, index)
    mapping.keyframes.initialize(mapping.frames[:2])
    mapping.keyframes.consider(mapping.frames[3], mapping.points)
    mapping.keyframes.consider(mapping.frames[6], mapping.points)
    mapping.trajectory.set_reference(4, 3)
    return mapping


def test_retire_keyframe_reanchors_and_preserves_all_pose_history(scene):
    mapping = retirement_scene(scene)
    frame = mapping.frames[3]
    before = mapping.trajectory.pose_rows()
    count = mapping.accepted_frame_count
    result = mapping.retire_frame(frame, recent_window=2)
    assert result['status'] == 'retired' and result['removed_observations'] == 60
    assert mapping.trajectory.pose_rows() == before
    assert mapping.accepted_frame_count == count and len(mapping.frames) == count - 1
    assert frame not in mapping.keyframes.frames and frame not in mapping.frames
    assert all(p is None for p in frame.pts)
    for point in mapping.points:
        assert frame not in point.frames and len(point.frames) == len(point.idx) == 9
        assert [f.id for f in point.frames[:2]] == [0, 1]
        assert [f.id for f in point.frames[-3:]] == [7, 8, 9]
    records = {r.frame_id: r for r in mapping.trajectory.records}
    assert records[3].retired and not records[3].is_keyframe
    assert records[4].reference_keyframe_id == records[3].reference_keyframe_id == result['replacement_id']
    root = records[result['replacement_id']]
    for identifier in (3, 4):
        np.testing.assert_allclose(np.asarray(records[identifier].T_cr) @ root.T_cw,
                                   records[identifier].T_cw, atol=1e-12)
    mapping.check_integrity()
    assert mapping.optimize().status == 'accepted'


@pytest.mark.parametrize('identifier', [0, 1, 6, 8, 9])
def test_anchor_latest_keyframe_and_recent_views_are_protected(scene, identifier):
    mapping = retirement_scene(scene)
    assert mapping.retire_frame(mapping.frames[identifier], recent_window=2)['reason'] == 'protected_or_recent'


def test_recovery_protection_and_essential_observations(scene):
    mapping = retirement_scene(scene)
    frame = mapping.frames[3]
    assert mapping.retire_frame(frame, protected=[frame], recent_window=2)['reason'] == 'protected_or_recent'
    point = mapping.points[0]
    point.remove_observation(mapping.frames[0])
    point.remove_observation(mapping.frames[1])
    before = mapping.trajectory.records
    assert mapping.retire_frame(frame, recent_window=2)['reason'] == 'essential_observation'
    assert mapping.trajectory.records == before and frame in mapping.frames


def test_post_commit_integrity_failure_rolls_back_every_link(scene, monkeypatch):
    mapping = retirement_scene(scene)
    frame = mapping.frames[3]
    before = mapping.trajectory.records
    frames, keys, slots = list(mapping.frames), list(mapping.keyframes.frames), list(frame.pts)
    pairs = [(list(p.frames), list(p.idx)) for p in mapping.points]
    original = mapping.check_integrity
    calls = 0
    def fail_after_commit():
        nonlocal calls
        calls += 1
        if calls == 2:
            raise ValueError('injected post-removal failure')
        original()
    monkeypatch.setattr(mapping, 'check_integrity', fail_after_commit)
    with pytest.raises(ValueError, match='injected'):
        mapping.retire_frame(frame, recent_window=2)
    assert mapping.trajectory.records == before
    assert mapping.frames == frames and mapping.keyframes.frames == keys and frame.pts == slots
    assert not mapping.retired_frame_ids
    assert [(p.frames, p.idx) for p in mapping.points] == pairs
    original()


def test_missing_common_witness_blocks_retirement(scene):
    mapping = retirement_scene(scene)
    frame = mapping.frames[3]
    # Every point still has many observers, but no one retained keyframe can
    # preserve all paths through this candidate. Refuse rather than guessing.
    mapping.points[0].remove_observation(mapping.frames[0])
    mapping.points[1].remove_observation(mapping.frames[1])
    mapping.points[2].remove_observation(mapping.frames[6])
    assert mapping.retire_frame(frame, recent_window=2)['reason'] == 'no_common_surviving_keyframe'
    assert frame in mapping.frames


def test_bounded_sweep_retires_old_storage_without_rewinding_history(scene):
    mapping = retirement_scene(scene)
    template = mapping.frames[-1]
    for identifier in range(10, 72):
        frame = copy(template)
        frame.id, frame.timestamp = identifier, float(identifier)
        frame.pts = [None] * len(template.pts)
        mapping.add_frame(frame)
        for index, point in enumerate(mapping.points):
            point.add_observation(frame, index)
    protected = mapping.frames[2]
    first = mapping.retire_redundant_frame(protected=[protected], max_checks=1)
    assert len(first['attempts']) == 1 and first['retired_frames'] == 0
    second = mapping.retire_redundant_frame(protected=[protected], max_checks=1)
    assert second['attempts'][0]['frame_id'] == 3
    assert second['attempts'][0]['status'] == 'retired'
    assert len(mapping.frames) == 71 and mapping.accepted_frame_count == 72
    assert len(mapping.trajectory.accepted) == 72
    assert protected in mapping.frames
    mapping.check_integrity()
