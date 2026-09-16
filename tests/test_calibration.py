"""Independent camera-model fixtures and real OpenCV board detection checks."""

import hashlib
import json

import cv2
import numpy as np
import pytest

from calibration import Board, View, fit_camera, main, read_views
from slam import camera_parameters


SIZE = (960, 720)
K = np.array([[800., 0, 480], [0, 820, 360], [0, 0, 1]])
DIST = np.array([-.12, .035, .001, -.002, -.005])


def scene(count=20):
    """Known board-to-camera poses, with no production projection helpers."""
    rng = np.random.default_rng(6)
    xy = np.mgrid[:9, :6].T.reshape(-1, 2) * .025
    objects = np.column_stack((xy, np.zeros(len(xy)))).astype(np.float32)
    for i in range(count):
        ax, ay, az = rng.uniform(-.5, .5, 3)
        cx, sx, cy, sy, cz, sz = np.cos(ax), np.sin(ax), np.cos(ay), np.sin(ay), np.cos(az), np.sin(az)
        rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        rotation = rz @ ry @ rx
        translation = np.array([rng.uniform(-.19, .02), rng.uniform(-.15, .02), rng.uniform(.6, .9)])
        yield objects.copy(), rotation, translation


def project_independent(objects, rotation, translation, distortion=DIST):
    """Analytic Brown-Conrady projection, independent of cv2.projectPoints."""
    camera = objects @ rotation.T + translation
    x, y = (camera[:, :2] / camera[:, 2:]).T
    radius2 = x*x + y*y
    k1, k2, p1, p2, k3 = distortion
    radial = 1 + k1*radius2 + k2*radius2**2 + k3*radius2**3
    u = K[0, 0] * (x*radial + 2*p1*x*y + p2*(radius2 + 2*x*x)) + K[0, 2]
    v = K[1, 1] * (y*radial + p1*(radius2 + 2*y*y) + 2*p2*x*y) + K[1, 2]
    return np.column_stack((u, v)).astype(np.float32)


def synthetic_views():
    return [View(str(i), objects, project_independent(objects, rotation, translation))
            for i, (objects, rotation, translation) in enumerate(scene())]


def test_known_intrinsics_distortion_and_held_out():
    views = synthetic_views()
    k, distortion, report = fit_camera(views[:16], views[16:], SIZE)
    np.testing.assert_allclose(k, K, atol=.01)
    np.testing.assert_allclose(distortion, DIST, atol=.002)
    assert report['validation']['rms_px'] < .001
    assert sum(c['corners'] for c in report['validation']['spatial_residuals']) == 4 * 54
    assert len(report['fitting']['views']) == 16
    # Corrupt held-out observations only: intrinsics stay unchanged within
    # solver roundoff, while validation residuals reveal the damage. The native
    # solver's stopping tolerance is relative; use sub-micropixel agreement,
    # not bitwise equality across repeated floating-point decompositions.
    noisy = [View(v.name, v.objects, v.pixels.copy()) for v in views[16:]]
    rng = np.random.default_rng(4)
    for v in noisy:
        v.pixels += rng.normal(0, 2, v.pixels.shape).astype(np.float32)
    other_k, other_dist, other = fit_camera(views[:16], noisy, SIZE)
    np.testing.assert_allclose(other_k, k, rtol=0, atol=1e-7)
    np.testing.assert_allclose(other_dist, distortion, rtol=0, atol=1e-7)
    assert other['validation']['rms_px'] > 1
    assert any('Validation RMS' in message for message in other['warnings'])


def test_insufficient_degenerate_and_stationary_views():
    views = synthetic_views()
    with pytest.raises(ValueError, match='at least 8'):
        fit_camera(views[:7], views[16:], SIZE)
    with pytest.raises(ValueError, match='diversity'):
        fit_camera([views[0]] * 8, views[16:], SIZE)
    bad = View('bad', views[0].objects.copy(), views[0].pixels.copy())
    bad.objects[:, 1] = 0
    with pytest.raises(ValueError, match='degenerate'):
        fit_camera([bad] + views[1:16], views[16:], SIZE)
    bad.objects[0, 0] = np.nan
    with pytest.raises(ValueError, match='degenerate'):
        fit_camera([bad] + views[1:16], views[16:], SIZE)


def render_checkerboards(root):
    """Render known pinhole views of a 10x7 square board (9x6 inner corners)."""
    board = np.full((7*80, 10*80), 255, np.uint8)
    for row in range(7):
        for col in range(10):
            if (row + col) % 2 == 0:
                board[row*80:(row+1)*80, col*80:(col+1)*80] = 0
    outer = np.array([[-.025, -.025, 0], [.225, -.025, 0], [.225, .15, 0], [-.025, .15, 0]])
    source = np.float32([[0, 0], [800, 0], [800, 560], [0, 560]])
    for i, (_, rotation, translation) in enumerate(scene()):
        target = project_independent(outer, rotation, translation, np.zeros(5))
        transform = cv2.getPerspectiveTransform(source, target)
        image = cv2.warpPerspective(board, transform, SIZE, borderValue=180)
        folder = root / ('train' if i < 16 else 'validation')
        folder.mkdir(exist_ok=True)
        assert cv2.imwrite(str(folder / f'{i:02}.png'), image)


def cli_args(root):
    return ['--images', str(root/'train'), '--validation-images', str(root/'validation'),
            '--columns', '9', '--rows', '6', '--square-size', '.025',
            '--camera', 'synthetic', '--recording-mode', '960x720 no stabilization',
            '--output', str(root/'camera.json'), '--report', str(root/'report.json')]


def test_rendered_checkerboards_cli_and_slam_loader(tmp_path):
    render_checkerboards(tmp_path)
    assert main(cli_args(tmp_path) + ['--report-html', str(tmp_path/'review.html')]) == 0
    camera = json.loads((tmp_path/'camera.json').read_text())
    report = json.loads((tmp_path/'report.json').read_text())
    assert camera['calibration_status'] == 'measured_unverified'
    assert report['schema_version'] == 2
    assert camera['schema_version'] == 2
    assert report['camera_model']['K'] == camera['K']
    assert report['capture_summary']['fitting'] == {'detected': 16}
    assert 'Fitting versus held-out validation' in (tmp_path/'review.html').read_text()
    assert camera['quality_report_sha256'] == hashlib.sha256((tmp_path/'report.json').read_bytes()).hexdigest()
    assert report['status'] in ('checks_passed', 'needs_review')
    assert len(report['images']) == 20
    assert report['validation']['rms_px'] < .5
    np.testing.assert_allclose(camera['K'], K, atol=6)
    w, h, scaled, distortion = camera_parameters(*SIZE, max_width=480, calibration=camera)
    assert (w, h) == (480, 360)
    np.testing.assert_allclose(scaled, np.diag([.5, .5, 1]) @ camera['K'])
    assert len(distortion) == 5
    original = (tmp_path/'camera.json').read_bytes()
    with pytest.raises(SystemExit):
        main(cli_args(tmp_path))
    assert (tmp_path/'camera.json').read_bytes() == original


def test_charuco_partial_detection_preserves_ids():
    spec = Board('charuco', 7, 5, .04, .03)
    board, finder = spec.detector()
    image = board.generateImage((980, 700), marginSize=20)
    image[:, :280] = 255  # Occlude part of the board; IDs must still select correct XYZ.
    detected = spec.detect(image, (board, finder))
    assert detected is not None
    objects, pixels = detected
    corners, ids, _, _ = finder.detectBoard(image)
    assert 6 <= len(objects) < 24
    np.testing.assert_array_equal(objects, board.getChessboardCorners()[ids.ravel()])
    np.testing.assert_array_equal(pixels, corners.reshape(-1, 2))


def test_duplicate_and_size_mismatch_are_not_silently_dropped(tmp_path):
    spec = Board('checkerboard', 9, 6, .025)
    folder = tmp_path/'images'
    folder.mkdir()
    assert cv2.imwrite(str(folder/'a.png'), np.zeros((72, 96), np.uint8))
    assert cv2.imwrite(str(folder/'b.png'), np.zeros((72, 96), np.uint8))
    report = {'images': []}
    with pytest.raises(ValueError, match='Duplicate'):
        read_views(folder, spec, None, 'fitting', report, {}, None)
    assert cv2.imwrite(str(folder/'b.png'), np.zeros((73, 96), np.uint8))
    with pytest.raises(ValueError, match='Mixed source dimensions'):
        read_views(folder, spec, None, 'fitting', {'images': []}, {}, None)


def test_no_board_saves_failure_report_without_camera(tmp_path):
    rng = np.random.default_rng(1)
    for folder in ('train', 'validation'):
        (tmp_path/folder).mkdir()
        assert cv2.imwrite(str(tmp_path/folder/'blank.png'), rng.integers(0, 30, (72, 96), dtype=np.uint8))
    assert main(cli_args(tmp_path) + ['--report-html', str(tmp_path/'failure.html')]) == 2
    assert not (tmp_path/'camera.json').exists()
    report = json.loads((tmp_path/'report.json').read_text())
    assert report['status'] == 'failed'
    assert 'got 0 and 0' in report['error']
    assert all(row['status'] == 'board_not_found' for row in report['images'])
    assert 'Calibration failed' in (tmp_path/'failure.html').read_text()


@pytest.mark.parametrize('spec', [Board('checkerboard', 2, 6, .02),
                                   Board('checkerboard', 9, 6, float('nan')),
                                   Board('charuco', 7, 5, .04, .05),
                                   Board('charuco', 7, 5, .04, .03, 'not_a_dictionary')])
def test_invalid_board(spec):
    with pytest.raises(ValueError):
        spec.detector()


def test_rendered_charuco_calibration_cli(tmp_path):
    spec = Board('charuco', 7, 5, .04, .03)
    board, _ = spec.detector()
    source_image = board.generateImage((840, 600))
    outer = np.array([[0, 0, 0], [.28, 0, 0], [.28, .2, 0], [0, .2, 0]])
    source = np.float32([[0, 0], [840, 0], [840, 600], [0, 600]])
    for i, (_, rotation, translation) in enumerate(scene()):
        target = project_independent(outer, rotation, translation, np.zeros(5))
        image = cv2.warpPerspective(source_image, cv2.getPerspectiveTransform(source, target),
                                   SIZE, borderValue=180)
        folder = tmp_path/('train' if i < 16 else 'validation')
        folder.mkdir(exist_ok=True)
        assert cv2.imwrite(str(folder/f'{i:02}.png'), image)
    args = cli_args(tmp_path)
    args[args.index('--columns')+1] = '7'
    args[args.index('--rows')+1] = '5'
    args[args.index('--square-size')+1] = '.04'
    assert main(args + ['--board', 'charuco', '--marker-size', '.03']) == 0
    camera = json.loads((tmp_path/'camera.json').read_text())
    report = json.loads((tmp_path/'report.json').read_text())
    np.testing.assert_allclose(camera['K'], K, atol=8)
    assert report['validation']['rms_px'] < .5


def test_cross_split_duplicate_is_rejected(tmp_path):
    folder = tmp_path/'train'
    folder.mkdir()
    image = np.random.default_rng(2).integers(0, 255, (72, 96), dtype=np.uint8)
    cv2.imwrite(str(folder/'one.png'), image)
    report, seen = {'images': []}, {}
    spec = Board('checkerboard', 9, 6, .025)
    _, size = read_views(folder, spec, None, 'fitting', report, seen, None)
    with pytest.raises(ValueError, match='Duplicate'):
        read_views(folder, spec, None, 'validation', report, seen, size)


def test_output_collision_and_invalid_nonfinite_config(tmp_path):
    args = cli_args(tmp_path)
    args[args.index('--report')+1] = str(tmp_path/'camera.json')
    with pytest.raises(SystemExit):
        main(args)
    args = cli_args(tmp_path)
    args[args.index('--square-size')+1] = 'nan'
    with pytest.raises(SystemExit):
        main(args)
    assert not (tmp_path/'camera.json').exists()
    assert not (tmp_path/'report.json').exists()


def test_existing_html_is_preserved_before_fitting(tmp_path):
    path = tmp_path/'review.html'
    path.write_text('existing report')
    with pytest.raises(SystemExit):
        main(cli_args(tmp_path) + ['--report-html', str(path)])
    assert path.read_text() == 'existing report'
    assert not (tmp_path/'report.json').exists()
    assert not (tmp_path/'camera.json').exists()
