import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from frame import estimate_pose, TrackingError
from scripts.generate_demo import generate
from scripts.benchmark_tracking import compare_baseline
from tracking_diagnostics import residual_summary


ROOT = Path(__file__).resolve().parents[1]


def test_baseline_coverage_compares_frame_identities_within_replay_horizon():
    def report(end, accepted, condition):
        rows = [{'frame_id': i, 'timestamp': i / 30, 'status': 'tracking', 'reason': '',
                 'features': 40, 'matches': 20, 'inliers': 12, 'added_points': 0, 'landmarks': 60}
                for i in range(end)]
        return {'video_sha256': 'same-video', 'camera': {}, 'environment': {},
                'configuration': {'condition_pnp': condition}, 'frames': rows,
                'poses': [{'frame_id': i} for i in accepted]}

    # A longer baseline has poses outside this prefix. Those do not count as
    # regressions; within the prefix, dropping frame 1 must remain explicit.
    compared = compare_baseline(report(3, [0, 2], True), report(5, [0, 1, 2, 4], False))
    assert compared['compatible_inputs'] and not compared['same_solver_configuration']
    coverage = compared['coverage_comparison']
    assert coverage['baseline_accepted_poses'] == 3 and coverage['retained_baseline_poses'] == 2
    assert not coverage['retains_all_baseline_frames']
    assert coverage['newly_lost_frame_ids'] == [1]
    assert coverage['newly_accepted_frame_ids'] == []


@pytest.mark.parametrize('condition', [False, True])
def test_narrow_band_inliers_explain_coverage_rejection(condition):
    """Many accurate matches in a thin horizontal band still fail camera coverage.

    Independently project noncoplanar world points. This isolates the dashcam's
    observed gate without requiring private video or relaxing pose validation.
    """
    rng = np.random.default_rng(17)
    xyz = rng.uniform([-2, -.06, 4], [2, .06, 8], (80, 3))
    pixels = np.column_stack((500 * xyz[:, 0] / xyz[:, 2] + 320,
                              500 * xyz[:, 1] / xyz[:, 2] + 240))
    frame = SimpleNamespace(k=np.array([[500., 0, 320], [0, 500., 240], [0, 0, 1.]]),
                            _kps=pixels, w=640, h=480)
    points = [SimpleNamespace(point=point, id=i) for i, point in enumerate(xyz)]
    evidence, trace = {}, {}
    cv2.setRNGSeed(0)
    with pytest.raises(TrackingError, match='image coverage'):
        estimate_pose(frame, points, np.arange(80), diagnostics=evidence, trace=trace, condition=condition)
    assert evidence['gate'] == 'image_coverage' and evidence['status'] == 'rejected'
    assert evidence['refined_inliers'] == 80 and evidence['ransac_inliers'] == 80
    assert evidence['span_fraction'][0] > .1 > evidence['span_fraction'][1]
    assert evidence['inlier_residual_px']['max'] < 1e-5
    assert trace['refined_rows'] == list(range(80))
    np.testing.assert_array_equal(trace['xyz'], xyz)
    np.testing.assert_array_equal(trace['pixels'], pixels)
    # The same measurements may guide search, but provisional acceptance must
    # explicitly report that it has not passed the required final coverage gate.
    cv2.setRNGSeed(0)
    estimate_pose(frame, points, np.arange(80), require_coverage=False, diagnostics=evidence, condition=condition)
    assert evidence['status'] == 'accepted' and evidence['coverage_required'] is False


def test_insufficient_map_support_preserves_failure_evidence(scene):
    mapping, _, _ = scene
    evidence, trace = {}, {}
    with pytest.raises(TrackingError, match='twelve'):
        estimate_pose(mapping.frames[-1], mapping.points[:4], np.arange(4),
                      diagnostics=evidence, trace=trace)
    assert evidence['gate'] == 'correspondence_count'
    assert evidence['input_count'] == 4 and trace['refined_rows'] == []
    assert residual_summary(np.array([1., 2., np.nan, np.inf])) == {
        'finite_count': 2, 'invalid_count': 2, 'median': 1.5, 'p95': 1.95, 'max': 2.0}


@pytest.mark.parametrize('condition, spatial', [(False, False), (True, False), (True, True)])
def test_benchmark_preserves_tracking_and_writes_replayable_evidence(tmp_path, condition, spatial):
    video = generate(tmp_path / 'demo.avi', 25)
    baseline = tmp_path / 'baseline.json'
    mode = ['--condition-pnp'] if condition else ['--no-condition-pnp']
    mode += ['--spatial-mapping'] if spatial else ['--no-spatial-mapping']
    plain = subprocess.run([sys.executable, str(ROOT / 'slam.py'), str(video), '--headless',
        '--focal', '400', '--max-frames', '25', '--report', str(baseline), *mode],
        capture_output=True, text=True, timeout=60, cwd=ROOT)
    assert plain.returncode == 0, plain.stderr
    output = tmp_path / 'benchmark'
    command = [sys.executable, '-m', 'scripts.benchmark_tracking', str(output), '--video', str(video),
               '--focus-start', '10', '--focus-end', '24', '--every', '7', '--focal', '400',
               '--baseline', str(baseline), *mode]
    instrumented = subprocess.run(command, capture_output=True, text=True, timeout=60, cwd=ROOT)
    assert instrumented.returncode == 0, instrumented.stdout + instrumented.stderr
    report = json.loads((output / 'report.json').read_text())
    assert report['configuration']['spatial_mapping'] == spatial
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['complete_window'] and summary['focus_frames'] == 15
    assert summary['baseline_comparison']['identical_frame_outcomes']
    expected_method = 'centred_rms_radius' if condition else 'none'
    assert report['frames'][-1]['diagnostics']['stages']['final_pnp']['conditioning']['method'] == expected_method
    # Identical horizon and seed allow a stronger test than counts: passive
    # diagnostics must preserve the actual native-optimized camera trajectory.
    assert report['poses'] == json.loads(baseline.read_text())['poses']
    index = json.loads((output / 'frames/index.json').read_text())
    ids = {row['frame_id'] for row in index['samples']}
    assert {10, 17, 24} <= ids and all(10 <= number <= 24 for number in ids)
    sample = index['samples'][-1]
    data = json.loads((output / 'frames' / sample['evidence']).read_text())
    image = cv2.imread(str(output / 'frames' / sample['image']))
    assert image.shape == (2 * (360 + 104), 2 * 640, 3)
    trace = data['trace']['final_pnp']
    xyz, pixels = np.array(trace['xyz']), np.array(trace['pixels'])
    pose = np.array(trace['T_cw'])
    # Reconstruct residuals independently from the saved world points and pose.
    local = xyz @ pose[:3, :3].T + pose[:3, 3]
    projection = local @ np.array(report['camera']['K']).T
    errors = np.linalg.norm(projection[:, :2] / projection[:, 2:] - pixels, axis=1)
    rows = trace['refined_rows']
    assert np.all(errors[rows] <= 3.)
    assert len(rows) == data['result']['diagnostics']['stages']['final_pnp']['refined_inliers']
    rerun = subprocess.run(command, capture_output=True, text=True, timeout=10, cwd=ROOT)
    assert rerun.returncode == 2 and 'already exists' in rerun.stderr
