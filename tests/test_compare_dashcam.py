"""Controlled comparisons preserve frame identities and reject confounded evidence."""

import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from scripts.compare_dashcam import compare_pair, final_metrics, outcome_hash
from scripts.generate_demo import generate
from test_camera_profile import bundle

ROOT = Path(__file__).resolve().parents[1]


def report(accepted=(0, 2)):
    rows = [{'frame_id': i, 'timestamp': i/30, 'status': 'tracking' if i in accepted else 'lost',
             'reason': '' if i in accepted else 'test loss', 'features': 40, 'matches': 20,
             'inliers': 12, 'added_points': 2, 'landmarks': 20, 'processing_seconds': .01,
             'diagnostics': {'stages': {'final_pnp': {'inlier_residual_px': {'median': i+.1, 'p95': i+.2},
                                                    'spatial_support': {'effective_cells': 5}}}}}
            for i in range(3)]
    return {'outcome': 'completed', 'decoded_frames': 3, 'accepted_poses': len(accepted),
            'video_sha256': 'video', 'frames': rows, 'poses': [{'frame_id': i, 'T_cw': np.eye(4).tolist()} for i in accepted],
            'configuration': {'seed': 0, 'width': 1024, 'report': 'different-path-allowed'},
            'environment': {key: 'test' for key in ('python', 'platform', 'numpy', 'opencv', 'g2opy')},
            'camera': {'K': np.eye(3).tolist(), 'distortion': [0]*5, 'source_size': [640, 360],
                       'processed_size': [640, 360], 'timestamp_source': 'test'},
            'feature_mask': {'source_sha256': None, 'effective_sha256': None}, 'elapsed_seconds': 1.}


def test_changed_calibration_is_allowed_but_frame_identity_regression_is_visible():
    a, b = report(), report((1, 2))
    b['camera']['K'][0][0] = 2.
    b['configuration']['calibration'] = 'candidate.json'
    b['feature_mask']['effective_sha256'] = 'rectification-border-changed'
    result = compare_pair(a, b, 3, 'video')
    assert result['valid'] and not result['same_processed_camera_geometry']
    assert result['coverage']['newly_unposed_frame_ids'] == [0]
    assert result['coverage']['newly_accepted_frame_ids'] == [1]
    assert result['effective_mask_changed']
    assert result['repeatability'] is None
    common = result['common_accepted_frame_metrics']
    assert common['baseline']['frames'] == common['candidate']['frames'] == 1
    assert common['baseline']['per_frame_inlier_median_px']['median'] == 2.1
    assert not compare_pair(a, b, 3, 'video', control=True)['valid']


@pytest.mark.parametrize('change', ['seed', 'video', 'timestamp', 'partial', 'environment', 'mask', 'missing_metrics', 'duplicate_pose'])
def test_invalid_comparisons_do_not_produce_quality_claims(change):
    a, b = report(), report()
    if change == 'seed': b['configuration']['seed'] = 42
    elif change == 'video': b['video_sha256'] = 'other'
    elif change == 'timestamp': b['frames'][1]['timestamp'] += .01
    elif change == 'partial': b['frames'].pop()
    elif change == 'environment': b['environment']['opencv'] = 'other'
    elif change == 'mask': b['feature_mask']['source_sha256'] = 'other'
    elif change == 'missing_metrics': b['frames'][0]['diagnostics'] = None
    elif change == 'duplicate_pose': b['poses'].append(b['poses'][0])
    result = compare_pair(a, b, 3, 'video')
    assert not result['valid'] and result['comparison'] is None
    assert result['reasons']


def test_empty_common_population_and_timing_independent_control_hash():
    a, b = report((0,)), report((2,))
    result = compare_pair(a, b, 3, 'video')
    assert result['valid']
    assert result['common_accepted_frame_metrics']['baseline']['per_frame_inlier_median_px'] == {
        'count': 0, 'median': None, 'p95': None, 'max': None}
    b = copy.deepcopy(a)
    b['frames'][0]['processing_seconds'] = 99
    b['elapsed_seconds'] = 99
    assert outcome_hash(a) == outcome_hash(b)
    b['poses'][0]['T_cw'][0][3] = 1
    assert outcome_hash(a) != outcome_hash(b)


def call(output, video, *args):
    return subprocess.run([sys.executable, '-m', 'scripts.compare_dashcam', str(output), '--video', str(video),
                           '--max-frames', '25', *args], cwd=ROOT, capture_output=True, text=True, timeout=100)


def test_native_control_abba_and_report_retention(tmp_path):
    video = generate(tmp_path/'video.avi', 25)
    output = tmp_path/'comparison'
    result = call(output, video, '--repeatability-control', '--focal', '400')
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads((output/'comparison.json').read_text())
    assert data['status'] == 'completed' and data['repeatability_passed']
    assert [r['arm'] for r in data['runs']] == ['baseline', 'candidate', 'candidate', 'baseline']
    assert len(data['pairs']) == 2
    assert data['pairs'][0]['baseline']['accepted_poses'] == 23
    retention = data['pairs'][0]['baseline']['landmark_retention']
    assert retention['created'] == retention['live'] + retention['retired']
    assert retention['live_fraction'] == retention['live']/retention['created']
    assert not list(output.rglob('*.png'))
    previous = (output/'comparison.json').read_bytes()
    assert call(output, video, '--repeatability-control').returncode == 2
    assert (output/'comparison.json').read_bytes() == previous


def test_profile_comparison_and_preflight_reject_wrong_source_size(tmp_path):
    video = generate(tmp_path/'video.avi', 25)
    profile, _, _ = bundle(tmp_path/'profile')
    output = tmp_path/'comparison'
    result = call(output, video, '--candidate-calibration', str(profile), '--repeats', '1', '--focal', '400')
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads((output/'comparison.json').read_text())
    assert data['mode'] == 'calibration_comparison'
    assert data['camera_provenance']['candidate']['profile']['provenance_status'] == 'hashes_verified'
    assert data['pairs'][0]['repeatability'] is None
    wrong = tmp_path/'legacy.json'
    wrong.write_text(json.dumps({'model':'pinhole', 'width':1920, 'height':1080,
                                'K':[[525,0,960],[0,525,540],[0,0,1]]}))
    result = call(tmp_path/'invalid', video, '--baseline-calibration', str(wrong), '--repeatability-control')
    assert result.returncode == 2 and 'dimensions' in result.stderr
    assert not (tmp_path/'invalid').exists()
