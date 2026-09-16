"""Mask coordinates and descriptor alignment, independent of pose acceptance."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from feature_mask import prepare_feature_mask, tint_exclusions
from frame import extract
from slam import SLAM
from scripts.generate_demo import generate


def png(tmp_path, pixels):
    path = tmp_path / 'mask.png'
    assert cv2.imwrite(str(path), pixels)
    return path


def test_source_resize_uses_full_pixel_support_and_preserves_intrinsics(tmp_path):
    source = np.full((6, 8), 255, np.uint8)
    source[2, 2] = 0
    path = png(tmp_path, source)
    mask, info = prepare_feature_mask(path, (8, 6), (4, 3))
    expected = np.full((3, 4), 255, np.uint8)
    expected[1, 1] = 0  # One excluded pixel contaminates its 2x2 area average.
    np.testing.assert_array_equal(mask, expected)
    assert info['source_sha256'] == hashlib.sha256(path.read_bytes()).hexdigest()
    k = np.array([[50., 0, 2], [0, 50, 1.5], [0, 0, 1]])
    tracker = SLAM(k, feature_mask=mask, mask_bottom=.34)
    combined = tracker.extraction_mask((3, 4))
    assert np.all(combined[1:] == 0)
    np.testing.assert_array_equal(mask, expected)
    np.testing.assert_array_equal(tracker.k, k)
    assert not combined.flags.writeable


def test_fractional_resize_does_not_lose_thin_exclusions(tmp_path):
    source = np.full((7, 11), 255, np.uint8)
    source[:, 5] = 0
    mask, _ = prepare_feature_mask(png(tmp_path, source), (11, 7), (4, 2))
    # Column 5 straddles output columns 1 and 2 at x=5.5; both are excluded.
    np.testing.assert_array_equal(mask, [[255, 0, 0, 255], [255, 0, 0, 255]])


def test_rectification_moves_mask_and_rejects_partial_border_support(tmp_path):
    source = np.full((6, 8), 255, np.uint8)
    source[:, 3] = 0
    xx, yy = np.meshgrid(np.arange(8, dtype=np.float32), np.arange(6, dtype=np.float32))
    maps = (xx + .5, yy)
    mask, _ = prepare_feature_mask(png(tmp_path, source), (8, 6), (8, 6), maps)
    expected = np.full((6, 8), 255, np.uint8)
    expected[:, [2, 3, 7]] = 0
    np.testing.assert_array_equal(mask, expected)
    border_only, _ = prepare_feature_mask(None, (8, 6), (8, 6), maps)
    assert np.all(border_only[:, -1] == 0) and np.all(border_only[:, :-1] == 255)
    assert prepare_feature_mask(None, (8, 6), (8, 6))[0] is None


@pytest.mark.parametrize('kind', ['color', 'alpha', 'gray', 'uint16', 'dimensions', 'corrupt'])
def test_invalid_mask_fails_explicitly(tmp_path, kind):
    pixels = np.full((6, 8), 255, np.uint8)
    if kind == 'color':
        pixels = np.repeat(pixels[:, :, None], 3, axis=2)
    elif kind == 'alpha':
        pixels = np.repeat(pixels[:, :, None], 4, axis=2)
    elif kind == 'gray':
        pixels[1, 1] = 128
    elif kind == 'uint16':
        pixels = pixels.astype(np.uint16)
    elif kind == 'dimensions':
        pixels = pixels[:5]
    path = png(tmp_path, pixels)
    if kind == 'corrupt':
        path.write_bytes(b'not a PNG')
    with pytest.raises(ValueError):
        prepare_feature_mask(path, (8, 6), (4, 3))


def test_post_pyramid_filter_keeps_descriptor_rows_aligned():
    mask = np.full((10, 10), 255, np.uint8)
    mask[:, 5:] = 0
    points = [cv2.KeyPoint(2., 2., 3.), cv2.KeyPoint(4.6, 3., 3.), cv2.KeyPoint(3., 4., 3.)]
    descriptors = np.repeat(np.arange(3, dtype=np.uint8)[:, None], 32, axis=1)
    detector = SimpleNamespace(detectAndCompute=lambda *args: (points, descriptors))
    pixels, kept = extract(np.zeros((10, 10, 3), np.uint8), detector, mask)
    np.testing.assert_array_equal(pixels, [[2, 2], [3, 4]])
    np.testing.assert_array_equal(kept, descriptors[[0, 2]])


def test_native_orb_respects_holes_and_empty_masks():
    image = np.random.default_rng(20).integers(0, 256, (240, 320, 3), dtype=np.uint8)
    mask = np.full((240, 320), 255, np.uint8)
    mask[70:170, 100:220] = 0
    pixels, descriptors = extract(image, mask=mask)
    assert len(pixels) > 100 and len(descriptors) == len(pixels)
    uv = np.floor(pixels + .5).astype(int)
    assert np.all(mask[uv[:, 1], uv[:, 0]] == 255)
    pixels, descriptors = extract(image, mask=np.zeros_like(mask))
    assert pixels.shape == (0, 2) and descriptors.shape == (0, 32)
    with pytest.raises(ValueError, match='dimensions'):
        SLAM(np.eye(3), feature_mask=mask).extraction_mask((120, 160))


def test_display_tint_changes_only_excluded_pixels_without_mutation():
    image = np.full((4, 5, 3), 80, np.uint8)
    mask = np.full((4, 5), 255, np.uint8)
    mask[2:] = 0
    result = tint_exclusions(image, mask)
    assert np.all(image == 80)
    np.testing.assert_array_equal(result[:2], image[:2])
    assert np.all(result[2:, :, 2] > image[2:, :, 2])


def test_cli_all_excluded_reports_no_map_and_protects_mask(tmp_path):
    video = generate(tmp_path / 'demo.avi', 3)
    path = png(tmp_path, np.zeros((360, 640), np.uint8))
    report = tmp_path / 'report.json'
    command = [sys.executable, str(Path(__file__).resolve().parents[1] / 'slam.py'), str(video),
               '--headless', '--feature-mask', str(path)]
    result = subprocess.run([*command, '--report', str(report)], capture_output=True, text=True, timeout=30)
    assert result.returncode == 2, result.stderr
    data = json.loads(report.read_text())
    assert data['outcome'] == 'completed' and data['accepted_poses'] == 0
    assert data['feature_mask']['excluded_fraction'] == 1
    assert all(row['features'] == 0 for row in data['frames'])
    original = path.read_bytes()
    collision = subprocess.run([*command, '--report', str(path)], capture_output=True, text=True, timeout=30)
    assert collision.returncode == 2 and 'overwrite' in collision.stderr
    assert path.read_bytes() == original
