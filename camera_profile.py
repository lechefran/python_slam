"""Versioned pinhole profiles, capture declarations and reproducible provenance.

Intrinsics describe decoded raw source pixels. Profile hashes detect accidental
changes; they do not authenticate a camera or certify calibration accuracy.
"""

import argparse
import copy
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys

import numpy as np


SETTING_KEYS = {'schema_version', 'camera_id', 'lens_id', 'mode_id', 'fps', 'crop',
                'stabilization', 'digital_zoom', 'focus_mode', 'focus_setting'}


def read_json(path):
    """Reject ambiguous duplicate keys and non-standard NaN/Infinity JSON."""
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f'Duplicate JSON key: {key}')
            result[key] = value
        return result

    def nonfinite(value):
        raise ValueError(f'Non-finite JSON number: {value}')

    def finite_float(value):
        number = float(value)
        if not math.isfinite(number):
            nonfinite(value)
        return number

    raw = Path(path).read_bytes()
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=nonfinite,
                      parse_float=finite_float), hashlib.sha256(raw).hexdigest()


def digest(value):
    """Hash sorted, compact UTF-8 JSON; formatting and key order do not matter."""
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     ensure_ascii=True, allow_nan=False).encode()).hexdigest()


def settings(value):
    """Validate a capture declaration; null/unknown means not established."""
    if not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1:
        raise ValueError('Camera settings must be an object with schema_version 1')
    if value.keys() - SETTING_KEYS:
        raise ValueError(f'Unknown camera settings fields: {sorted(value.keys() - SETTING_KEYS)}')
    result = {key: None for key in sorted(SETTING_KEYS)}
    result.update(value)
    for key in ('camera_id', 'lens_id', 'mode_id', 'focus_setting'):
        if result[key] is not None and (not isinstance(result[key], str) or not result[key].strip()):
            raise ValueError(f'{key} must be nonblank text or null')
    for key, choices in [('stabilization', ('off', 'optical', 'digital', 'unknown')),
                         ('focus_mode', ('fixed', 'manual', 'auto', 'unknown'))]:
        if result[key] is None:
            result[key] = 'unknown'
        if result[key] not in choices:
            raise ValueError(f'Invalid {key}; expected one of {choices}')
    fps = result['fps']
    if fps is not None:
        if (not isinstance(fps, dict) or set(fps) != {'numerator', 'denominator'}
                or any(type(v) is not int or v <= 0 for v in fps.values())):
            raise ValueError('fps must contain positive integer numerator and denominator')
        factor = math.gcd(fps['numerator'], fps['denominator'])
        result['fps'] = {key: v // factor for key, v in fps.items()}
        try:
            rate = fps['numerator'] / fps['denominator']
        except OverflowError as exc:
            raise ValueError('FPS ratio must be finite') from exc
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('FPS ratio must be finite and positive')
    crop = result['crop']
    if crop is not None:
        if (not isinstance(crop, dict) or set(crop) != {'x', 'y', 'width', 'height'}
                or any(type(v) is not int for v in crop.values()) or min(crop['x'], crop['y']) < 0
                or min(crop['width'], crop['height']) <= 0):
            raise ValueError('crop must contain integer x/y >= 0 and width/height > 0')
    zoom = result['digital_zoom']
    if zoom is not None and (type(zoom) not in (int, float) or not math.isfinite(zoom) or zoom <= 0):
        raise ValueError('digital_zoom must be positive and finite or null')
    return result


def validate_calibration(value):
    """Validate source-pixel geometry for legacy files and versioned profiles."""
    if not isinstance(value, dict):
        raise ValueError('Calibration must be a JSON object')
    version = value.get('schema_version', 1)
    if type(version) is not int or version not in (1, 2):
        raise ValueError('Unsupported camera calibration schema_version')
    if value.get('model') != 'pinhole':
        raise ValueError('Only pinhole calibration is supported; rectify other lens models externally')
    if any(type(value.get(key)) is not int or value[key] <= 0 for key in ('width', 'height')):
        raise ValueError('Calibration dimensions must be positive integers')
    try:
        k = np.asarray(value.get('K'), dtype=float)
        dist = np.asarray(value.get('distortion', [0] * 5), dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('Calibration matrices must contain numeric values') from exc
    if (k.shape != (3, 3) or not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1]) > 1e-12 or abs(k[1, 0]) > 1e-12):
        raise ValueError('Calibration K must be a finite zero-skew pinhole matrix with positive focal lengths')
    if dist.ndim != 1 or len(dist) not in (4, 5, 8, 12, 14) or not np.isfinite(dist).all():
        raise ValueError('Invalid pinhole distortion coefficients')
    if version == 2:
        if value.get('kind') != 'slam_camera_profile' or value.get('pixel_domain') != 'raw_distorted_source':
            raise ValueError('Unsupported profile kind or pixel domain')
        if value.get('capture_settings') != settings(value.get('capture_settings')):
            raise ValueError('Profile capture settings must include all normalized fields')
        for key in ('camera', 'recording_mode', 'created_utc'):
            if not isinstance(value.get(key), str) or not value[key].strip():
                raise ValueError(f'Profile requires {key}')
        if datetime.fromisoformat(value['created_utc']).utcoffset() is None:
            raise ValueError('Profile creation time requires a timezone')
        provenance = value.get('provenance')
        if not isinstance(provenance, dict) or not isinstance(provenance.get('implementation_sha256'), dict):
            raise ValueError('Profile requires implementation provenance')
        if value.get('calibration_status') != 'measured_unverified':
            raise ValueError('Profile must retain measured_unverified calibration status')
        if value.get('quality_status') not in ('checks_passed', 'needs_review'):
            raise ValueError('Profile requires a successful, unverified quality report')
        if value.get('profile_id') != profile_id(value):
            raise ValueError('Camera profile content hash mismatch')
    return k, dist


def profile_id(profile):
    # The linked report's bytes are identified by SHA-256. Its filesystem
    # location is excluded so moving the profile/report bundle preserves identity.
    identity = {key: value for key, value in profile.items() if key not in ('profile_id', 'quality_report')}
    return 'sha256:' + digest(identity)


def create_profile(camera, report, report_path, output_path, capture_settings):
    """Create a profile from a successful fit; report hash covers its final bytes."""
    result = copy.deepcopy(camera)
    result.update(schema_version=2, kind='slam_camera_profile', pixel_domain='raw_distorted_source',
                  capture_settings=settings(capture_settings),
                  quality_report=os.path.relpath(report_path.resolve(), output_path.resolve().parent),
                  provenance={'python': platform.python_version(), 'numpy': report['numpy_version'],
                              'opencv': report['opencv_version'],
                              'implementation_sha256': {name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                                                        for name in ('calibration.py', 'calibration_report.py', 'camera_profile.py')}})
    result['profile_id'] = profile_id(result)
    validate_calibration(result)
    return result


def load_calibration(path):
    """Read one immutable snapshot; verify new profiles and their report sidecar."""
    path = Path(path)
    value, file_hash = read_json(path)
    validate_calibration(value)
    info = {'file_sha256': file_hash, 'schema_version': value.get('schema_version', 1),
            'profile_id': None, 'provenance_status': 'legacy_unverified'}
    protected = [path.resolve()]
    if value.get('schema_version') == 2:
        reference = value.get('quality_report')
        if not isinstance(reference, str) or not reference or Path(reference).is_absolute():
            raise ValueError('Profile quality_report must be a relative path')
        report_path = (path.parent / reference).resolve()
        report, report_hash = read_json(report_path)
        if report_hash != value.get('quality_report_sha256'):
            raise ValueError('Camera quality report hash mismatch')
        if (not isinstance(report, dict) or report.get('kind') != 'camera_calibration_report'
                or report.get('schema_version') != 2 or report.get('status') != value['quality_status']):
            raise ValueError('Profile quality report is incompatible or unsuccessful')
        model = report.get('camera_model', {})
        if not isinstance(model, dict):
            raise ValueError('Profile quality report has no camera model')
        if any(model.get(key) != value.get(key) for key in ('model', 'width', 'height', 'K', 'distortion')):
            raise ValueError('Profile geometry does not match the quality report')
        if report.get('capture_settings') != value['capture_settings']:
            raise ValueError('Profile capture settings do not match the quality report')
        for key in ('camera', 'recording_mode', 'board', 'board_units', 'created_utc'):
            if report.get(key) != value.get(key):
                raise ValueError(f'Profile {key} does not match the quality report')
        protected.append(report_path)
        info.update(profile_id=value['profile_id'], provenance_status='hashes_verified',
                    quality_report_sha256=report_hash, quality_status=value['quality_status'],
                    capture_settings=value['capture_settings'], calibration_status=value['calibration_status'])
    return value, info, protected


def check_capture(profile, width, height, decoder_fps, declared=None):
    """Compare decoded facts and optional user-declared camera settings.

    A decoder cannot infer physical lens/focus/stabilization settings. Matching
    declarations are labelled as declarations, never as sensor verification.
    """
    if (width, height) != (profile['width'], profile['height']):
        raise ValueError('Calibration dimensions do not match the decoded source image')
    result = {'dimensions': 'matched_decoded', 'decoder_nominal_fps':
              float(decoder_fps) if math.isfinite(decoder_fps) and decoder_fps > 0 else None,
              'settings': {}}
    expected = profile.get('capture_settings')
    if expected is None:
        result['status'] = 'legacy_unverified'
        return result
    actual = settings(declared) if declared is not None else None
    for key, wanted in expected.items():
        if key == 'schema_version':
            continue
        if wanted is None or wanted == 'unknown':
            result['settings'][key] = 'unknown_in_profile'
            continue
        if actual is not None and actual[key] is not None and actual[key] != 'unknown' and actual[key] != wanted:
            raise ValueError(f'Camera recording settings mismatch: {key}')
        if key == 'fps':
            fps = wanted['numerator'] / wanted['denominator']
            if result['decoder_nominal_fps'] is None:
                result['settings'][key] = 'decoder_unavailable'
            elif not math.isclose(decoder_fps, fps, rel_tol=1e-5, abs_tol=.001):
                raise ValueError(f'Camera frame rate mismatch: profile {fps:g}, decoder {decoder_fps:g}')
            else:
                result['settings'][key] = 'matched_decoder_nominal'
        else:
            result['settings'][key] = 'matched_declaration' if actual is not None and actual[key] == wanted else 'not_declared'
    result['status'] = 'incomplete' if any(v in ('unknown_in_profile', 'not_declared', 'decoder_unavailable')
                                         for v in result['settings'].values()) else 'matched_available_evidence'
    return result


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('profile', type=Path)
    args = cli.parse_args(argv)
    try:
        profile, info, _ = load_calibration(args.profile)
    except (ValueError, OSError, TypeError) as exc:
        cli.error(str(exc))
    print(json.dumps({'verification': info, 'source_size': [profile['width'], profile['height']],
                      'camera': profile.get('camera'), 'recording_mode': profile.get('recording_mode')}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
