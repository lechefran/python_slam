"""Portable calibration provenance and declared-versus-decoded compatibility."""

import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

from calibration import main as calibrate, write_json
from camera_profile import (check_capture, create_profile, digest, load_calibration,
                            profile_id, read_json, settings, validate_calibration)
from scripts.generate_demo import generate
from slam import camera_parameters
from test_calibration import cli_args, render_checkerboards


ROOT = Path(__file__).resolve().parents[1]


def declaration():
    return settings({'schema_version': 1, 'camera_id': 'synthetic-01', 'lens_id': 'pinhole-400',
                     'mode_id': '640x360-30', 'fps': {'numerator': 30, 'denominator': 1},
                     'crop': {'x': 0, 'y': 0, 'width': 640, 'height': 360},
                     'stabilization': 'off', 'digital_zoom': 1, 'focus_mode': 'fixed', 'focus_setting': 'factory'})


def bundle(root):
    """A labelled synthetic provenance fixture; no real-camera fit is claimed."""
    root.mkdir(parents=True, exist_ok=True)
    camera = {'model': 'pinhole', 'width': 640, 'height': 360,
              'K': [[400., 0, 320], [0, 400., 180], [0, 0, 1.]], 'distortion': [0.] * 5,
              'created_utc': '2026-09-16T00:00:00+00:00', 'camera': 'synthetic fixture',
              'recording_mode': 'synthetic fixture mode', 'board': {'kind': 'synthetic'}, 'board_units': 'metres',
              'quality_status': 'needs_review', 'calibration_status': 'measured_unverified'}
    report = {'schema_version': 2, 'kind': 'camera_calibration_report', 'status': 'needs_review',
              'numpy_version': np.__version__, 'opencv_version': 'test fixture',
              'camera_model': {key: camera[key] for key in ('model', 'width', 'height', 'K', 'distortion')},
              'capture_settings': declaration(),
              **{key: camera[key] for key in ('created_utc', 'camera', 'recording_mode', 'board', 'board_units')}}
    report_path, path = root/'quality.json', root/'camera.json'
    write_json(report_path, report)
    camera['quality_report_sha256'] = hashlib.sha256(report_path.read_bytes()).hexdigest()
    profile = create_profile(camera, report, report_path, path, declaration())
    write_json(path, profile)
    return path, report_path, profile


def test_profile_relocation_formatting_and_geometry(tmp_path):
    path, report, original = bundle(tmp_path/'original')
    loaded, info, protected = load_calibration(path)
    assert info['provenance_status'] == 'hashes_verified'
    assert report.resolve() in protected
    shutil.copytree(path.parent, tmp_path/'moved')
    moved = tmp_path/'moved'/'camera.json'
    # Whitespace/key ordering and bundle location do not change profile identity.
    moved.write_text(json.dumps(original, sort_keys=True))
    reloaded, other_info, _ = load_calibration(moved)
    assert info['profile_id'] == other_info['profile_id']
    assert info['file_sha256'] != other_info['file_sha256']
    w, h, k, _ = camera_parameters(640, 360, max_width=320, calibration=reloaded)
    assert (w, h) == (320, 180)
    np.testing.assert_array_equal(k, [[200, 0, 160], [0, 200, 90], [0, 0, 1]])


def test_geometry_and_report_changes_are_detected(tmp_path):
    path, report_path, profile = bundle(tmp_path)
    changed = copy.deepcopy(profile)
    changed['K'][0][0] += 1
    path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match='content hash'):
        load_calibration(path)
    changed['profile_id'] = profile_id(changed)
    path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match='geometry does not match'):
        load_calibration(path)
    path.write_text(json.dumps(profile))
    report_path.write_text(report_path.read_text() + '\n')
    with pytest.raises(ValueError, match='report hash'):
        load_calibration(path)


def test_missing_sidecar_and_unknown_versions_fail(tmp_path):
    path, report, profile = bundle(tmp_path)
    report.rename(tmp_path/'renamed.json')
    with pytest.raises(OSError):
        load_calibration(path)
    profile['schema_version'] = 99
    with pytest.raises(ValueError, match='schema_version'):
        validate_calibration(profile)
    profile['schema_version'] = True
    with pytest.raises(ValueError, match='schema_version'):
        validate_calibration(profile)


def test_legacy_and_unknown_settings_are_not_certified(tmp_path):
    _, _, profile = bundle(tmp_path/'bundle')
    legacy = {key: profile[key] for key in ('model', 'width', 'height', 'K', 'distortion')}
    path = tmp_path/'legacy.json'
    path.write_text(json.dumps(legacy))
    value, info, _ = load_calibration(path)
    assert info['provenance_status'] == 'legacy_unverified'
    assert check_capture(value, 640, 360, 30)['status'] == 'legacy_unverified'
    result = check_capture(profile, 640, 360, 30)
    assert result['settings']['fps'] == 'matched_decoder_nominal'
    assert result['settings']['focus_mode'] == 'not_declared'
    assert result['status'] == 'incomplete'
    profile['capture_settings'] = settings({'schema_version': 1})
    assert check_capture(profile, 640, 360, 30, declaration())['status'] == 'incomplete'


def test_recording_mode_checks_and_fractional_fps(tmp_path):
    _, _, profile = bundle(tmp_path)
    assert check_capture(profile, 640, 360, 30, declaration())['status'] == 'matched_available_evidence'
    for key, value in [('stabilization', 'digital'), ('camera_id', 'other'), ('focus_mode', 'auto'),
                       ('digital_zoom', 2), ('crop', {'x': 1, 'y': 0, 'width': 640, 'height': 360})]:
        actual = declaration()
        actual[key] = value
        with pytest.raises(ValueError, match=key):
            check_capture(profile, 640, 360, 30, actual)
    with pytest.raises(ValueError, match='dimensions'):
        check_capture(profile, 1920, 1080, 30)
    profile['capture_settings']['fps'] = {'numerator': 30000, 'denominator': 1001}
    assert check_capture(profile, 640, 360, 29.97)['settings']['fps'] == 'matched_decoder_nominal'
    with pytest.raises(ValueError, match='frame rate'):
        check_capture(profile, 640, 360, 30)
    assert check_capture(profile, 640, 360, float('nan'))['settings']['fps'] == 'decoder_unavailable'


@pytest.mark.parametrize('bad', ['{"K":1,"K":2}', '{"number":NaN}', '{"number":Infinity}', '{"number":1e999}'])
def test_ambiguous_json_rejected(tmp_path, bad):
    path = tmp_path/'bad.json'
    path.write_text(bad)
    with pytest.raises(ValueError):
        read_json(path)


@pytest.mark.parametrize('field,value', [('fps', {'numerator': 30, 'denominator': 0}),
                                        ('digital_zoom', float('nan')), ('crop', [0, 0, 640, 360]),
                                        ('stabilization', 'maybe'), ('unknown_key', 'value')])
def test_invalid_settings(field, value):
    with pytest.raises(ValueError):
        settings({'schema_version': 1, field: value})


def test_real_offline_tool_produces_verifiable_bundle(tmp_path):
    render_checkerboards(tmp_path)
    metadata = tmp_path/'settings.json'
    metadata.write_text(json.dumps({'schema_version': 1, 'camera_id': 'board-fixture',
                                    'fps': {'numerator': 60000, 'denominator': 2002}}))
    assert calibrate(cli_args(tmp_path) + ['--camera-settings', str(metadata)]) == 0
    profile, info, _ = load_calibration(tmp_path/'camera.json')
    assert info['profile_id'] == profile_id(profile)
    assert profile['capture_settings']['fps'] == {'numerator': 30000, 'denominator': 1001}
    assert profile['quality_report'] == 'report.json'


def test_native_slam_profile_and_legacy_replays_agree_and_protect_provenance(tmp_path):
    path, quality, profile = bundle(tmp_path/'profile')
    video = generate(tmp_path/'demo.avi', 25)
    metadata = tmp_path/'settings.json'
    metadata.write_text(json.dumps(declaration()))
    def run(camera, report, *extra):
        return subprocess.run([sys.executable, str(ROOT/'slam.py'), str(video), '--calibration', str(camera),
                               '--headless', '--report', str(report), *extra], capture_output=True, text=True, timeout=60)
    legacy = tmp_path/'legacy.json'
    legacy.write_text(json.dumps({key: profile[key] for key in ('model', 'width', 'height', 'K', 'distortion')}))
    baseline, current = tmp_path/'baseline.json', tmp_path/'current.json'
    assert run(legacy, baseline).returncode == 0
    result = run(path, current, '--camera-settings', str(metadata))
    assert result.returncode == 0, result.stderr
    old, new = json.loads(baseline.read_text()), json.loads(current.read_text())
    assert old['poses'] == new['poses']
    assert old['landmarks'] == new['landmarks']
    assert new['camera']['profile']['profile_id'] == profile['profile_id']
    assert new['camera']['compatibility']['settings']['camera_id'] == 'matched_declaration'
    assert new['camera']['preprocessing']['scale_xy'] == [1., 1.]
    previous = quality.read_bytes()
    result = run(path, quality)
    assert result.returncode == 2 and 'must not overwrite' in result.stderr
    assert quality.read_bytes() == previous
    actual = declaration()
    actual['stabilization'] = 'digital'
    metadata.write_text(json.dumps(actual))
    mismatch = tmp_path/'mismatch.json'
    assert run(path, mismatch, '--camera-settings', str(metadata)).returncode == 1
    failed = json.loads(mismatch.read_text())
    assert failed['accepted_poses'] == 0
    assert 'stabilization' in failed['error']
    assert failed['camera']['profile']['profile_id'] == profile['profile_id']
