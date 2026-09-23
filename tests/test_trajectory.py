import numpy as np
import pytest

from slam import SLAM
from trajectory import Trajectory


def test_delayed_initialization_and_owned_pose():
    history = Trajectory()
    history.begin(0, 0.)
    history.finish(0, 'initializing', 'waiting')
    history.begin(1, .1)
    history.finish(1, 'initializing', 'low parallax')
    history.begin(2, .2)
    pose = np.eye(4)
    history.accept(0, 0., pose)
    history.accept(2, .2, pose)
    history.finish(2, 'initialized')
    pose[0, 3] = 99
    assert [r.frame_id for r in history.accepted] == [0, 2]
    assert history.records[0].status == 'initializing'
    assert history.records[1].T_cw is None
    assert history.accepted[0].T_cw[0][3] == 0
    rows = history.pose_rows()
    rows[0]['T_cw'][0][3] = 44
    assert history.accepted[0].T_cw[0][3] == 0
    with pytest.raises(ValueError, match='Duplicate'):
        history.begin(2, .2)


def test_native_corrections_and_frame_storage_independence(scene):
    mapping, _, _ = scene
    mapping.frames[2].pose[0, 3] += .1
    result = mapping.optimize(iterations=15)
    assert result.status == 'accepted'
    for frame, record in zip(mapping.frames, mapping.trajectory.accepted):
        np.testing.assert_array_equal(frame.pose, record.T_cw)
    before = mapping.trajectory.pose_rows()
    # Retiring feature storage later must not erase the already published path.
    mapping.frames.clear()
    assert mapping.trajectory.pose_rows() == before


def test_invalid_correction_is_atomic():
    history = Trajectory()
    history.accept(0, 0., np.eye(4))
    history.accept(1, .1, np.eye(4))
    before = history.records
    corrected = np.eye(4)
    corrected[0, 3] = 2
    with pytest.raises(ValueError):
        history.update_poses({0: corrected, 1: np.full((4, 4), np.nan)})
    assert history.records == before


def test_lost_frame_has_outcome_without_identity_pose(scene):
    mapping, _, k = scene
    tracker = SLAM(k)
    tracker.map = mapping
    tracker.reference = mapping.frames[-1]
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 3, .1)
    record = mapping.trajectory.records[-1]
    assert record.status == result.status == 'lost'
    assert result.keyframe is None and not mapping.keyframes.frames
    assert record.reason == result.reason
    assert record.T_cw is None
    assert len(mapping.trajectory.accepted) == 3


def camera_pose(angle, translation):
    """Analytic world-to-camera rotation/translation, deliberately noncommuting."""
    c, s = np.cos(angle), np.sin(angle)
    pose = np.eye(4)
    pose[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    pose[:3, 3] = translation
    return pose


def linked_history():
    history = Trajectory()
    root = camera_pose(.4, [2., -1., .5])
    relative = camera_pose(-.2, [.3, .8, -.1])
    history.accept(0, 0., root)
    history.mark_keyframe(0)
    history.accept(1, .1, relative @ root)
    history.set_reference(1, 0)
    return history, relative


def test_reference_correction_composition_and_original_pose_preservation():
    history, relative = linked_history()
    original = history.records[1].T_cw_initial
    corrected = camera_pose(-.6, [-4., 2., 1.])
    history.update_poses({0: corrected})
    child = history.records[1]
    np.testing.assert_allclose(child.T_cw, relative @ corrected, atol=1e-12)
    np.testing.assert_allclose(child.T_cr, relative, atol=1e-12)
    assert child.T_cw_initial == original
    assert child.submap_id == 0 and child.scale_status == 'arbitrary'
    # Repeated root updates use the fixed relative pose, not accumulated deltas.
    again = camera_pose(.9, [5., -2., .3])
    history.update_poses({0: again})
    np.testing.assert_allclose(history.records[1].T_cw, relative @ again, atol=1e-12)


def test_explicit_and_retained_camera_poses_override_propagation():
    history, _ = linked_history()
    before = history.records[1].T_cw
    root = camera_pose(-.3, [1., 2., 3.])
    history.update_poses({0: root}, independent_ids=[1])
    child = history.records[1]
    assert child.T_cw == before
    np.testing.assert_allclose(np.asarray(child.T_cr) @ root, before, atol=1e-12)
    root2, explicit = camera_pose(.7, [3., 4., 5.]), camera_pose(-.1, [-2., 1., 0.])
    history.update_poses({0: root2, 1: explicit})
    child = history.records[1]
    np.testing.assert_array_equal(child.T_cw, explicit)
    np.testing.assert_allclose(np.asarray(child.T_cr) @ root2, explicit, atol=1e-12)


def test_reanchor_preserves_world_pose_and_follows_only_new_reference():
    history, _ = linked_history()
    history.accept(2, .2, camera_pose(.8, [4., 3., 2.]))
    history.mark_keyframe(2)
    before = history.records[1]
    history.reanchor_dependents(0, 2)
    child = history.records[1]
    assert child.T_cw == before.T_cw and child.T_cw_initial == before.T_cw_initial
    assert child.reference_keyframe_id == 2
    history.update_poses({0: camera_pose(.1, [8., 0., 0.])})
    assert history.records[1].T_cw == before.T_cw
    new = camera_pose(-.5, [0., 3., 0.])
    history.update_poses({2: new})
    np.testing.assert_allclose(history.records[1].T_cw, np.asarray(child.T_cr) @ new, atol=1e-12)


def test_invalid_reference_cross_submap_and_failed_update_are_atomic():
    history, _ = linked_history()
    history.accept(2, .2, np.eye(4), submap_id=1)
    history.mark_keyframe(2)
    before = history.records
    for action in (lambda: history.set_reference(1, 2),
                   lambda: history.reanchor_dependents(0, 2),
                   lambda: history.set_reference(1, 1),
                   lambda: history.set_reference(0, 1),
                   lambda: history.set_reference(1, 999),
                   lambda: history.update_poses({0: np.eye(4), 999: np.eye(4)}),
                   lambda: history.update_poses({0: np.eye(4), 1: np.full((4, 4), np.nan)})):
        with pytest.raises(ValueError):
            action()
        assert history.records == before
    history.begin(3, .3)
    with pytest.raises(ValueError):
        history.set_reference(3, 0)


def test_promotion_detaches_old_reference():
    history, _ = linked_history()
    history.mark_keyframe(1)
    before = history.records[1]
    history.update_poses({0: camera_pose(.8, [5., 1., 0.])})
    assert history.records[1] == before
    assert before.is_keyframe and before.reference_keyframe_id is None and before.T_cr is None


def test_native_ba_corrects_dependent_record_without_a_mapping_frame(scene):
    mapping, _, _ = scene
    root = mapping.frames[2]
    root.pose[0, 3] += .1
    mapping.trajectory.update_poses({root.id: root.pose})
    mapping.trajectory.mark_keyframe(root.id)
    relative = camera_pose(.3, [.1, -.2, .05])
    mapping.trajectory.accept(3, .1, relative @ root.pose)
    mapping.trajectory.set_reference(3, root.id)
    before = mapping.trajectory.records[-1]
    result = mapping.optimize(iterations=15)
    assert result.status == 'accepted'
    child = mapping.trajectory.records[-1]
    assert child.T_cw != before.T_cw
    assert child.T_cw_initial == before.T_cw_initial
    np.testing.assert_allclose(child.T_cw, relative @ root.pose, atol=1e-12)
    for frame, record in zip(mapping.frames, mapping.trajectory.accepted):
        np.testing.assert_array_equal(frame.pose, record.T_cw)


def test_failed_native_ba_keeps_reference_history(scene, monkeypatch):
    import dmap
    mapping, _, _ = scene
    mapping.trajectory.mark_keyframe(0)
    mapping.trajectory.set_reference(2, 0)
    before = mapping.trajectory.records
    def fail():
        raise RuntimeError('injected native failure')
    monkeypatch.setattr(dmap.g2o, 'SparseOptimizer', fail)
    assert mapping.optimize().status == 'failed'
    assert mapping.trajectory.records == before
