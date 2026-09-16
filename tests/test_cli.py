import json
from pathlib import Path
import subprocess
import sys

import pytest

from scripts.generate_demo import generate


ROOT = Path(__file__).resolve().parents[1]


def call(*args, env=None):
    return subprocess.run([sys.executable, str(ROOT / 'slam.py'), *map(str, args)],
                          capture_output=True, text=True, timeout=60, env=env)


def test_help_and_invalid_input(tmp_path):
    assert call('--help').returncode == 0
    result = call(tmp_path / 'missing.mp4', '--headless')
    assert result.returncode == 2 and 'does not exist' in result.stderr


def test_headless_video_report_and_no_gui_imports(tmp_path):
    video = generate(tmp_path / 'demo.avi', 25)
    report = tmp_path / 'report.json'
    result = call(video, '--headless', '--focal', '400', '--max-frames', 25, '--report', report)
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert data['decoded_frames'] == 25 and data['accepted_poses'] >= 10
    assert data['landmarks'] >= 20 and data['outcome'] == 'completed'
    assert any(row['ba'] and row['ba']['status'] == 'accepted' for row in data['frames'])
    probe = subprocess.run([sys.executable, '-c',
        'import slam, sys; assert "matplotlib.pyplot" not in sys.modules; assert "display" not in sys.modules'],
        cwd=ROOT, capture_output=True, text=True)
    assert probe.returncode == 0, probe.stderr


def test_invalid_frame_range_and_report_does_not_overwrite_input(tmp_path):
    video = generate(tmp_path / 'demo.avi', 2)
    assert call(video, '--headless', '--max-frames', 0).returncode == 2
    assert call(video, '--headless', '--report', video).returncode == 2
    result = call(video, '--headless', '--start-frame', 5)
    assert result.returncode == 1 and 'beyond' in result.stderr


@pytest.mark.parametrize('entrypoint', ['python-slam', 'python-slam-calibrate', 'python-slam-calibration-report', 'python-slam-camera-profile'])
def test_installed_entrypoint_away_from_checkout(tmp_path, entrypoint):
    command = Path(sys.executable).with_name(entrypoint)
    if not command.exists():
        pytest.skip('Run after installing the package to verify its entry point')
    # Running elsewhere prevents the source directory from masking a packaging
    # error where the executable exists but its Python modules were not installed.
    result = subprocess.run([str(command), '--help'], cwd=tmp_path,
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
