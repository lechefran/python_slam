"""Observation evidence must not change poses, count unsearched points or grow unbounded."""

import json
from copy import copy
from pathlib import Path
import subprocess
import sys

import numpy as np

from observation_quality import ObservationQuality, write_quality_report
from slam import SLAM
from scripts.generate_demo import generate


def test_history_is_bounded_and_search_denominator_excludes_censored_checks():
    history = ObservationQuality()
    for i in range(100):
        assert history.record(i, i, 'search', 'accepted' if i % 2 else 'unmatched')
        assert not history.record(i, i, 'search', 'accepted')
    history.record(100, 100, 'search', 'unassessed')
    history.record(100, 20, 'cull', 'retained', 1.0, 5., 2)
    assert not history.record(101, 20, 'cull', 'retained', 1.0, 5., 2)
    result = history.summary()
    assert len(result['recent_events']) == 16
    assert result['assessed_searches'] == 100
    assert result['successful_reobservation_ratio'] == .5
    assert result['counts']['cull/retained'] == 1
    assert ObservationQuality().summary()['successful_reobservation_ratio'] is None


def test_invalid_measurements_are_json_null_and_never_mixed_with_added_residuals():
    history = ObservationQuality()
    history.record(1, 1, 'observation', 'added', 2., 5., 1)
    history.record(2, 2, 'observation', 'added', 4., 5., 2)
    history.record(3, 1, 'cull', 'rejected', 100., -5., 1)
    history.record(4, 4, 'search', 'unassessed', np.nan, np.inf)
    report = history.summary()
    assert report['recent_added_residual_px'] == {'samples': 2, 'median': 3., 'max': 4.}
    assert report['recent_events'][-1]['depth'] is None
    json.dumps(report, allow_nan=False)


def test_added_residuals_are_snapshots_and_duplicate_links_do_not_add_events(scene):
    mapping, _, _ = scene
    point = mapping.points[0]
    before = point.observation_quality.summary()
    point.add_observation(mapping.frames[-1], 0)
    assert point.observation_quality.summary() == before
    point.point += [0.1, 0, 0]
    assert point.observation_quality.summary() == before
    mapping.cull(50, max_error=100)
    samples = point.observation_quality.summary()['recent_events']
    assert samples[-1]['phase'] == 'cull' and samples[-1]['residual_px'] > 1
    assert samples[-1]['assessed_frame_id'] == 50


def test_searches_use_final_geometry_and_do_not_count_unsearched_points(scene):
    mapping, _, k = scene
    current = copy(mapping.frames[-1])
    current.id = 3
    current.pts = [None] * 60
    current.pts[0] = mapping.points[0]
    tracker = SLAM(k)
    tracker.map = mapping
    points = mapping.points
    points[3].point[2] = -1
    attempts = {points[0]: 0, points[1]: 1, points[2]: None, points[3]: None}
    tracker.assess_searches(current, attempts)
    assert [p.observation_quality.samples[-1].outcome for p in points[:4]] == [
        'accepted', 'rejected', 'unmatched', 'unassessed']
    assert points[4].observation_quality.counts['search/unmatched'] == 0
    totals = mapping.observation_summary()
    assert totals['assessed_searches'] == 3 and totals['successful_reobservation_ratio'] == 1 / 3
    tracker.assess_searches(current, attempts)
    assert mapping.observation_summary() == totals


def test_masked_and_outside_searches_are_unassessed(scene):
    mapping, _, k = scene
    current = copy(mapping.frames[-1])
    current.id, current.pts = 4, [None] * 60
    tracker = SLAM(k, feature_mask=np.zeros((480, 640), np.uint8))
    tracker.map = mapping
    tracker.assess_searches(current, {mapping.points[0]: None})
    assert mapping.points[0].observation_quality.samples[-1].outcome == 'unassessed'
    assert mapping.observation_summary()['assessed_searches'] == 0


def test_failed_tracking_leaves_individual_quality_histories_unchanged(scene):
    mapping, _, k = scene
    tracker = SLAM(k)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    before = [p.observation_quality.summary() for p in mapping.points]
    _, result = tracker.process(np.zeros((480, 640, 3), np.uint8), 3, .1)
    assert result.status == 'lost'
    assert before == [p.observation_quality.summary() for p in mapping.points]
    assert mapping.quality_events['tracking/lost_unassessed'] == 1


def test_recovery_proposal_records_only_winning_attempts_after_commit(scene):
    mapping, _, k = scene
    tracker = SLAM(k)
    tracker.map, tracker.reference = mapping, mapping.frames[-1]
    tracker.keyframes.add(mapping.frames[0])
    current = copy(mapping.frames[-1])
    current.id, current.timestamp = 150, 5.
    current.des = mapping.frames[0].des.copy()
    current.pts = [None] * 60
    before = [p.observation_quality.summary() for p in mapping.points]
    # A failed normal hypothesis may have left arbitrary staged entries. The
    # successful old-view proposal replaces that ledger without recording it.
    unrelated = object()
    attempts = {unrelated: None}
    proposal = tracker.recover_pose(current, quality_attempts=attempts)
    assert proposal is not None and unrelated not in attempts
    assert before == [p.observation_quality.summary() for p in mapping.points]
    current.pose, points, indices, _, _ = proposal
    mapping.add_frame(current)
    for point, index in zip(points, indices):
        point.add_observation(current, index)
    tracker.assess_searches(current, attempts)
    assert mapping.observation_summary()['assessed_searches'] == 60
    assert mapping.observation_summary()['successful_reobservation_ratio'] == 1
    mapping.check_integrity()


def test_retired_samples_survive_unlinking_and_are_bounded(scene):
    from point import Point
    mapping, _, _ = scene
    old = mapping.points[0]
    old.delete_point('outlier', assessed_frame_id=50)
    snapshot = mapping.retired_quality[-1]
    assert snapshot['point_id'] == old.id and snapshot['live_observations'] == 0
    assert snapshot['history']['counts']['observation/added'] == 3
    assert snapshot['history']['recent_events'][-1]['assessed_frame_id'] == 50
    old.delete_point()
    assert len(mapping.retired_quality) == 1
    for _ in range(100):
        Point(mapping, [0, 0, 5], [255, 0, 0]).delete_point()
    assert len(mapping.retired_quality) == 64
    mapping.check_integrity()


def test_streamed_export_round_trips_independent_json_reader(tmp_path, scene):
    mapping, _, _ = scene
    path = tmp_path / 'quality.json'
    write_quality_report(path, {'schema_version': 1}, iter(mapping.points), mapping.retired_quality, 30)
    report = json.loads(path.read_text())
    assert len(report['live_landmarks']) == 60
    assert report['live_landmarks'][0]['history']['counts']['observation/added'] == 3
    assert report['live_landmarks'][0]['age_source_frames'] == 30
    assert not path.with_name('quality.json.tmp').exists()


def test_cli_histories_preserve_exact_native_trajectory_and_protect_inputs(tmp_path):
    video = generate(tmp_path / 'demo.avi', 25)
    root = Path(__file__).resolve().parents[1]
    command = [sys.executable, str(root / 'slam.py'), str(video), '--headless', '--focal', '400']
    reports = []
    for enabled in (False, True):
        report = tmp_path / f'report-{enabled}.json'
        args = [*command, '--report', str(report), '--observation-history' if enabled else '--no-observation-history']
        if enabled:
            args += ['--quality-report', str(tmp_path / 'quality.json')]
        result = subprocess.run(args, capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stderr
        reports.append(json.loads(report.read_text()))
    assert reports[0]['poses'] == reports[1]['poses']
    assert reports[0]['observation_quality'] is None
    assert reports[1]['observation_quality']['assessed_searches'] > 0
    detail = json.loads((tmp_path / 'quality.json').read_text())
    assert len(detail['live_landmarks']) == reports[1]['landmarks']
    assert all(len(p['history']['recent_events']) <= 16 for p in detail['live_landmarks'])
    for suffix in (['--quality-report', str(video)],
                   ['--quality-report', str(tmp_path / 'same'), '--report', str(tmp_path / 'same')],
                   ['--quality-report', str(tmp_path / 'off.json'), '--no-observation-history']):
        bad = subprocess.run([*command, *suffix], capture_output=True, text=True, timeout=30)
        assert bad.returncode == 2
