"""Offline pinhole calibration from raw, full-resolution board photographs.

Board coordinates are metres on Z=0; observations and residuals are source
pixels. Estimated per-view transforms map board coordinates into the camera.
This module needs only NumPy/OpenCV and never imports the SLAM or GUI runtime.
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from calibration_report import QUALITY_POLICY, assess_quality, finalize_report, write_html


@dataclass(frozen=True)
class Board:
    kind: str
    columns: int
    rows: int
    square_size: float
    marker_size: float | None = None
    dictionary: str = 'DICT_4X4_50'

    def validate(self):
        if self.kind not in ('checkerboard', 'charuco'):
            raise ValueError('Board must be checkerboard or charuco')
        if self.columns < 3 or self.rows < 3:
            raise ValueError('Board requires at least 3 columns and 3 rows')
        if not np.isfinite(self.square_size) or self.square_size <= 0:
            raise ValueError('Square size must be finite and positive, in metres')
        if self.kind == 'charuco':
            if (self.marker_size is None or not np.isfinite(self.marker_size)
                    or not 0 < self.marker_size < self.square_size):
                raise ValueError('ChArUco marker size must be positive and smaller than square size')
            if not self.dictionary.startswith('DICT_') or not hasattr(cv2.aruco, self.dictionary):
                raise ValueError('Unknown OpenCV ArUco dictionary')
        elif self.marker_size is not None:
            raise ValueError('Marker size only applies to ChArUco')

    def detector(self):
        self.validate()
        if self.kind == 'checkerboard':
            return None
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        board = cv2.aruco.CharucoBoard((self.columns, self.rows), self.square_size,
                                     self.marker_size, dictionary)
        return board, cv2.aruco.CharucoDetector(board)

    def detect(self, gray, detector):
        """Return aligned board XYZ (N,3) and raw pixel UV (N,2), or None."""
        if self.kind == 'checkerboard':
            found, corners = cv2.findChessboardCornersSB(
                gray, (self.columns, self.rows), flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not found:
                return None
            xy = np.mgrid[:self.columns, :self.rows].T.reshape(-1, 2)
            objects = np.column_stack((xy * self.square_size, np.zeros(len(xy))))
        else:
            board, finder = detector
            corners, ids, _, _ = finder.detectBoard(gray)
            if ids is None or len(ids) < 6 or board.checkCharucoCornersCollinear(ids):
                return None
            # IDs are essential: partial board detections do not have a fixed
            # row ordering, so select each known 3D corner by its detected ID.
            objects = board.getChessboardCorners()[ids.ravel()]
        return np.asarray(objects, np.float32), corners.reshape(-1, 2).astype(np.float32)


@dataclass
class View:
    name: str
    objects: np.ndarray
    pixels: np.ndarray


def read_views(folder, board, detector, split, report, seen, size):
    """Decode each source image once without resizing, cropping or rectifying."""
    folder = Path(folder)
    if not folder.is_dir():
        raise ValueError(f'Image directory does not exist: {folder}')
    files = sorted(p for p in folder.iterdir() if p.suffix.lower() in
                   ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'))
    if not files:
        raise ValueError(f'No supported images in {folder}')
    views = []
    for path in files:
        row = {'path': str(path.resolve()), 'split': split,
               'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        report['images'].append(row)
        # Ignore EXIF orientation: calibration must match the decoded video's
        # pixel axes, not a photo viewer's automatic display rotation.
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE | cv2.IMREAD_IGNORE_ORIENTATION)
        if gray is None:
            row['status'] = 'unreadable'
            raise ValueError(f'Cannot decode image: {path}')
        current = (gray.shape[1], gray.shape[0])
        if size is None:
            size = current
            report['source_size'] = list(size)
        if current != size:
            row['status'] = 'dimension_mismatch'
            raise ValueError(f'Mixed source dimensions: {path}: {current}, expected {size}')
        digest = hashlib.sha256(gray.tobytes()).hexdigest()
        row['decoded_sha256'] = digest
        if digest in seen:
            row['status'] = 'duplicate'
            raise ValueError(f'Duplicate decoded image: {path}; already used by {seen[digest]}')
        seen[digest] = str(path)
        observation = board.detect(gray, detector)
        if observation is None:
            row['status'] = 'board_not_found'
            continue
        objects, pixels = observation
        row.update(status='detected', corners=len(pixels),
                   object_points_m=objects.tolist(), image_points_px=pixels.tolist())
        views.append(View(str(path.resolve()), objects, pixels))
    return views, size


def residual_summary(views, poses, k, distortion, size):
    """Evaluate raw distorted-pixel residuals and a 4-column, 3-row error grid."""
    records, residuals, positions, view_ids = [], [], [], []
    for view, (rotation, translation) in zip(views, poses, strict=True):
        # OpenCV rvec/tvec is board-to-camera. Positive Z is required before
        # projection, even if an algebraic reprojection error appears small.
        camera_points = view.objects @ cv2.Rodrigues(rotation)[0].T + translation.reshape(1, 3)
        if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
            raise ValueError(f'Invalid board depth in {view.name}')
        projected = cv2.projectPoints(view.objects, rotation, translation, k, distortion)[0].reshape(-1, 2)
        delta = projected.astype(float) - view.pixels
        if not np.isfinite(delta).all():
            raise ValueError(f'Non-finite residuals in {view.name}')
        error = np.linalg.norm(delta, axis=1)
        records.append({'path': view.name, 'corners': len(error),
                        'rms_px': float(np.sqrt(np.mean(error ** 2))),
                        'median_px': float(np.median(error)),
                        'p95_px': float(np.percentile(error, 95)), 'max_px': float(error.max()),
                        'mean_residual_uv_px': delta.mean(axis=0).tolist(),
                        'residuals_uv_px': delta.tolist(),
                        'board_hull_fraction': float(cv2.contourArea(cv2.convexHull(view.pixels.astype(np.float32))) /
                                                     (size[0] * size[1])),
                        'rvec_board_to_camera': rotation.ravel().tolist(),
                        'tvec_board_to_camera_m': translation.ravel().tolist()})
        residuals.append(delta)
        positions.append(view.pixels)
        view_ids.append(np.full(len(error), len(records) - 1))
    delta, pixels = np.concatenate(residuals), np.concatenate(positions)
    error = np.linalg.norm(delta, axis=1)
    ids = np.concatenate(view_ids)
    cells = np.minimum((pixels / np.array(size) * [4, 3]).astype(int), [3, 2])
    grid = []
    for row in range(3):
        for col in range(4):
            selected = (cells[:, 0] == col) & (cells[:, 1] == row)
            grid.append({'column': col, 'row': row, 'corners': int(selected.sum()),
                         'views': int(len(np.unique(ids[selected]))),
                         'rms_px': float(np.sqrt(np.mean(error[selected] ** 2))) if selected.any() else None,
                         'mean_residual_uv_px': delta[selected].mean(axis=0).tolist() if selected.any() else None})
    # A dense board in one image is still just one view. Count each view once
    # per edge band; corner regions deliberately contribute to both adjacent edges.
    fraction = QUALITY_POLICY['edge_band_fraction']
    masks = {'left': pixels[:, 0] < size[0] * fraction,
             'right': pixels[:, 0] >= size[0] * (1 - fraction),
             'top': pixels[:, 1] < size[1] * fraction,
             'bottom': pixels[:, 1] >= size[1] * (1 - fraction)}
    edges = {edge: {'corners': int(selected.sum()), 'views': int(len(np.unique(ids[selected]))),
                    'rms_px': float(np.sqrt(np.mean(error[selected] ** 2))) if selected.any() else None}
             for edge, selected in masks.items()}
    return {'views': records, 'rms_px': float(np.sqrt(np.mean(error ** 2))),
            'median_px': float(np.median(error)), 'mean_residual_uv_px': delta.mean(axis=0).tolist(),
            'p95_px': float(np.percentile(error, 95)), 'max_px': float(error.max()),
            'occupied_grid_cells': sum(cell['corners'] > 0 for cell in grid),
            'grid_shape': [3, 4], 'spatial_residuals': grid, 'edge_coverage': edges}


def fit_camera(training, validation, size):
    """Fit five-coefficient pinhole intrinsics using training observations only.

    Validation solves only each board's six-DoF pose with frozen intrinsics.
    Its residual is a conditional camera-model check, not trajectory accuracy
    or a fully independent prediction of the board pose.
    """
    if len(training) < 8 or len(validation) < 3:
        raise ValueError(f'Need at least 8 fitting and 3 validation detections; got {len(training)} and {len(validation)}')
    for view in training + validation:
        if (view.objects.shape != (len(view.pixels), 3) or view.pixels.shape[1:] != (2,)
                or len(view.pixels) < 6 or not np.isfinite(view.objects).all()
                or not np.isfinite(view.pixels).all()
                or np.any(view.pixels < 0) or np.any(view.pixels >= size)
                or not np.allclose(view.objects[:, 2], 0)
                or np.linalg.matrix_rank(view.objects[:, :2] - view.objects[:, :2].mean(axis=0)) < 2):
            raise ValueError(f'Invalid or degenerate board observations: {view.name}')
    # Repeated stationary frames cannot constrain focal length and distortion.
    # This is only a gross diversity guard; tilt and field coverage are reported
    # separately and still require the user's inspection.
    signatures = [np.r_[v.pixels.mean(axis=0), np.ptp(v.pixels, axis=0)] / max(size) for v in training]
    distinct = []
    for signature in signatures:
        if all(np.linalg.norm(signature - previous) >= .02 for previous in distinct):
            distinct.append(signature)
    if len(distinct) < 3:
        raise ValueError('Insufficient board position/size diversity; capture varied views')
    result = cv2.calibrateCameraExtended(
        [v.objects.astype(np.float32) for v in training],
        [v.pixels.astype(np.float32) for v in training], size, None, None,
        flags=0, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 1e-9))
    rms, k, distortion, rotations, translations, std_intrinsics, _, _ = result
    if (not np.isfinite(rms) or not np.isfinite(k).all() or not np.isfinite(distortion).all()
            or not np.isfinite(std_intrinsics).all()
            or np.any(np.diag(k)[:2] < .05 * max(size))
            or np.any(np.diag(k)[:2] > 20 * max(size))
            or not 0 <= k[0, 2] < size[0] or not 0 <= k[1, 2] < size[1]):
        raise ValueError('Invalid or implausible camera fit; review board geometry and capture diversity')
    validation_poses = []
    for view in validation:
        ok, rotation, translation = cv2.solvePnP(view.objects, view.pixels, k, distortion,
                                               flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            raise ValueError(f'Cannot estimate validation board pose: {view.name}')
        validation_poses.append((rotation, translation))
    fit = residual_summary(training, list(zip(rotations, translations)), k, distortion, size)
    held_out = residual_summary(validation, validation_poses, k, distortion, size)
    normals = np.array([cv2.Rodrigues(r)[0][:, 2] for r in rotations])
    span = float(np.rad2deg(np.arccos(np.clip((normals @ normals.T).min(), -1, 1))))
    assessment = assess_quality(fit, held_out, span)
    return k, distortion.ravel(), {'fitting': fit, 'validation': held_out,
                                  'fitting_normal_span_degrees': span,
                                  'intrinsics_standard_deviations_opencv': std_intrinsics.ravel().tolist(),
                                  **assessment}


def write_json(path, payload):
    """Create new output exclusively, preserving any prior calibration/report."""
    text = json.dumps(payload, indent=2, allow_nan=False) + '\n'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as stream:
        stream.write(text)


def parser():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--images', required=True, type=Path, help='Directory of raw fitting photographs')
    cli.add_argument('--validation-images', required=True, type=Path, help='Separate held-out photograph directory')
    cli.add_argument('--board', choices=['checkerboard', 'charuco'], default='checkerboard')
    cli.add_argument('--columns', required=True, type=int, help='Inner corners for checkerboard; squares for ChArUco')
    cli.add_argument('--rows', required=True, type=int, help='Inner corners for checkerboard; squares for ChArUco')
    cli.add_argument('--square-size', required=True, type=float, help='Measured square side in metres')
    cli.add_argument('--marker-size', type=float, help='ChArUco marker side in metres')
    cli.add_argument('--dictionary', default='DICT_4X4_50')
    cli.add_argument('--camera', required=True, help='Camera/lens identifier')
    cli.add_argument('--recording-mode', required=True, help='Resolution, FPS, crop, zoom and stabilization settings')
    cli.add_argument('--output', required=True, type=Path, help='New camera JSON for slam --calibration')
    cli.add_argument('--report', required=True, type=Path, help='New detailed quality report JSON')
    cli.add_argument('--report-html', type=Path, help='Optional new standalone HTML quality review')
    return cli


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    # Refuse collisions before reading captures or fitting: a failed experiment
    # must never overwrite an input photograph or a previously usable camera.
    if args.output.resolve() == args.report.resolve():
        cli.error('Output and report must be different paths')
    for path in (args.output, args.report):
        if path.exists() or path.is_symlink():
            cli.error(f'Output already exists: {path}; choose a new path')
        if path.suffix.lower() != '.json':
            cli.error('Output and report must use .json extensions')
    if args.report_html is not None:
        if (args.report_html.exists() or args.report_html.is_symlink()
                or args.report_html.suffix.lower() != '.html'
                or args.report_html.resolve() in (args.output.resolve(), args.report.resolve())):
            cli.error('Choose a new, distinct .html report path')
    if not args.camera.strip() or not args.recording_mode.strip():
        cli.error('Camera and recording mode cannot be blank')
    board = Board(args.board, args.columns, args.rows, args.square_size, args.marker_size, args.dictionary)
    try:
        board.validate()
    except ValueError as exc:
        cli.error(str(exc))
    report = {'schema_version': 2, 'kind': 'camera_calibration_report', 'status': 'failed',
              'created_utc': datetime.now(timezone.utc).isoformat(),
              'opencv_version': cv2.__version__, 'numpy_version': np.__version__,
              'board': asdict(board), 'board_units': 'metres',
              'camera': args.camera, 'recording_mode': args.recording_mode,
              'images': [], 'warnings': [],
              'policy': {'minimum_fitting_views': 8, 'minimum_validation_views': 3,
                         **QUALITY_POLICY, 'opencv_calibration_flags': 0},
              'preprocessing': {'resize': False, 'crop': False, 'rectification': False,
                                'pixel_domain': 'raw_distorted_source', 'exif_orientation': 'ignored'},
              'validation_method': 'Frozen intrinsics; board pose fitted separately to each validation image',
              'intrinsics_standard_deviation_order': ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'p1', 'p2',
                                                      'k3', 'k4', 'k5', 'k6', 's1', 's2', 's3', 's4',
                                                      'tau_x', 'tau_y'],
              'limitations': ['Pinhole with k1,k2,p1,p2,k3; no fisheye or changing intrinsics',
                              'No automatic image removal or tuning against validation images',
                              'Fit is not certified calibration or SLAM accuracy evidence']}
    try:
        detector = board.detector()
        seen = {}
        training, size = read_views(args.images, board, detector, 'fitting', report, seen, None)
        validation, size = read_views(args.validation_images, board, detector, 'validation', report, seen, size)
        k, distortion, quality = fit_camera(training, validation, size)
        report.update(quality, status='needs_review' if quality['warnings'] else 'checks_passed')
        report['camera_model'] = {'model': 'pinhole', 'width': size[0], 'height': size[1],
                                  'K': k.tolist(), 'distortion': distortion.tolist(),
                                  'distortion_order': ['k1', 'k2', 'p1', 'p2', 'k3']}
        finalize_report(report)
        camera = {'schema_version': 1, 'model': 'pinhole', 'width': size[0], 'height': size[1],
                  'K': k.tolist(), 'distortion': distortion.tolist(),
                  'distortion_order': ['k1', 'k2', 'p1', 'p2', 'k3'],
                  'calibration_status': 'measured_unverified', 'created_utc': report['created_utc'],
                  'camera': args.camera, 'recording_mode': args.recording_mode,
                  'board': asdict(board), 'board_units': 'metres', 'quality_status': report['status'],
                  'quality_report': str(args.report.resolve()),
                  'quality_report_sha256': hashlib.sha256(
                      (json.dumps(report, indent=2, allow_nan=False) + '\n').encode()).hexdigest()}
        write_json(args.report, report)
        write_json(args.output, camera)
    except (ValueError, OSError, cv2.error) as exc:
        report.update(status='failed', error=str(exc))
        finalize_report(report)
        if not args.report.exists():
            try:
                write_json(args.report, report)
            except (OSError, ValueError):
                pass
        if args.report_html is not None:
            try:
                write_html(args.report_html, report)
            except (OSError, ValueError) as html_exc:
                print(f'HTML report could not be written: {html_exc}', file=sys.stderr)
        print(f'Calibration failed: {exc}', file=sys.stderr)
        return 2
    if args.report_html is not None:
        try:
            write_html(args.report_html, report)
        except (OSError, ValueError) as exc:
            print(f'Camera and JSON report saved, but HTML report failed: {exc}', file=sys.stderr)
            return 2
    print(f"Saved {args.output}: {report['status']}; fitting RMS {quality['fitting']['rms_px']:.3f}px; "
          f"validation RMS {quality['validation']['rms_px']:.3f}px")
    for warning in quality['warnings']:
        print(f'Review: {warning}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
