"""Controlled camera-model comparison using sequential, isolated SLAM replays.

A comparison is evidence about tracking consistency, coverage and measured cost.
Without independent ground truth it is not evidence of trajectory accuracy.
"""

import argparse
from collections import Counter
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

from camera_profile import check_capture, load_calibration, read_json, settings

ROOT = Path(__file__).resolve().parents[1]
IGNORED_CONFIG = {'calibration', 'camera_settings', 'report', 'diagnostics_dir', 'quality_report'}


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    # A new run directory owns these files. Publish complete JSON after each
    # subprocess so interruption leaves previous runs and logs reviewable.
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def stats(values):
    data = np.array([v for v in values if v is not None and np.isfinite(v)], dtype=float)
    return {'count': len(data), 'median': float(np.median(data)) if len(data) else None,
            'p95': float(np.percentile(data, 95)) if len(data) else None,
            'max': float(data.max()) if len(data) else None}


def final_metrics(rows):
    """Summarize per-frame PnP statistics; do not pretend to pool corner residuals."""
    final = [(r.get('diagnostics') or {}).get('stages', {}).get('final_pnp', {}) for r in rows]
    return {'frames': len(rows),
            'per_frame_inlier_median_px': stats(m.get('inlier_residual_px', {}).get('median') for m in final),
            'per_frame_inlier_p95_px': stats(m.get('inlier_residual_px', {}).get('p95') for m in final),
            'effective_spatial_cells': stats(m.get('spatial_support', {}).get('effective_cells') for m in final),
            'largest_cell_fraction': stats(m.get('spatial_support', {}).get('largest_cell_fraction') for m in final),
            'minor_axis_std_fraction': stats(m.get('spatial_support', {}).get('minor_axis_std_fraction') for m in final),
            'inliers': stats(r['inliers'] for r in rows),
            'processing_seconds': stats(r['processing_seconds'] for r in rows)}


def summarize(report):
    frames = report['frames']
    accepted = {p['frame_id'] for p in report['poses']}
    gaps = []
    for row in frames:
        if row['status'] == 'lost':
            if gaps and row['frame_id'] == gaps[-1]['end'] + 1:
                gaps[-1]['end'] = row['frame_id']
            else:
                gaps.append({'start': row['frame_id'], 'end': row['frame_id']})
    stages = Counter((r.get('diagnostics') or {}).get('failure_stage') or 'unspecified'
                     for r in frames if r['status'] == 'lost')
    timings = [(r.get('diagnostics') or {}).get('timing_seconds', {}) for r in frames]
    return {'decoded_frames': len(frames), 'accepted_poses': len(accepted),
            'pose_coverage': len(accepted) / len(frames) if frames else None,
            'unposed_frame_ids': sorted({r['frame_id'] for r in frames} - accepted),
            'states': dict(Counter(r['status'] for r in frames)), 'lost_intervals_inclusive': gaps,
            'failure_stages': dict(stages),
            'failure_reasons': dict(Counter(r['reason'] for r in frames if r['status'] == 'lost')),
            'recovered_frame_ids': [r['frame_id'] for r in frames if r.get('recovered_from') is not None],
            'landmark_retention': report.get('landmark_retention'),
            'landmark_quality': report.get('landmark_quality'),
            'observation_quality': report.get('observation_quality'),
            'processing_seconds_all_frames': stats(r['processing_seconds'] for r in frames),
            'pipeline_elapsed_seconds': report['elapsed_seconds'],
            'stage_seconds': {key: stats(row.get(key) for row in timings)
                              for key in sorted({key for row in timings for key in row})},
            'accepted_frame_metrics': final_metrics([r for r in frames if r['frame_id'] in accepted])}


def compare_pair(baseline, candidate, expected_frames, video_hash, control=False):
    """Only pair evidence with identical video/time horizon and common policy.

    Different intrinsics change rectification, rays, associations and map gauge.
    Compare accepted frame identities and common-frame diagnostics, not raw
    trajectory matrices or landmark IDs across different camera models.
    """
    reasons = []
    for name, report in [('baseline', baseline), ('candidate', candidate)]:
        frames = report.get('frames', [])
        ids = [r['frame_id'] for r in frames]
        poses = report.get('poses', [])
        pose_ids = [p['frame_id'] for p in poses]
        if report.get('outcome') != 'completed' or ids != list(range(expected_frames)):
            reasons.append(f'{name}: incomplete or failed replay')
        if report.get('video_sha256') != video_hash:
            reasons.append(f'{name}: video hash mismatch')
        if report.get('decoded_frames') != len(frames):
            reasons.append(f'{name}: inconsistent frame count')
        if len(set(pose_ids)) != len(pose_ids) or not set(pose_ids) <= set(ids) or report.get('accepted_poses') != len(poses):
            reasons.append(f'{name}: inconsistent pose identities/count')
        if any(np.asarray(p.get('T_cw')).shape != (4, 4) or not np.isfinite(p['T_cw']).all() for p in poses):
            reasons.append(f'{name}: invalid pose matrix')
        if any(not r.get('diagnostics') for r in frames):
            reasons.append(f'{name}: tracking diagnostics missing')
        if not all(key in report.get('environment', {}) for key in ('python', 'platform', 'numpy', 'opencv', 'g2opy')):
            reasons.append(f'{name}: runtime environment evidence missing')
        if not all(key in report.get('camera', {}) for key in ('K', 'distortion', 'source_size', 'processed_size', 'timestamp_source')):
            reasons.append(f'{name}: camera evidence missing')
    config = lambda r: {k: v for k, v in r.get('configuration', {}).items() if k not in IGNORED_CONFIG}
    if config(baseline) != config(candidate):
        reasons.append('algorithm or preprocessing policy differs')
    if baseline.get('environment') != candidate.get('environment'):
        reasons.append('runtime environments differ')
    for key in ('source_size', 'processed_size', 'timestamp_source'):
        if baseline.get('camera', {}).get(key) != candidate.get('camera', {}).get(key):
            reasons.append(f'camera {key} differs')
    if baseline.get('feature_mask', {}).get('source_sha256') != candidate.get('feature_mask', {}).get('source_sha256'):
        reasons.append('source exclusion masks differ')
    if [r['timestamp'] for r in baseline.get('frames', [])] != [r['timestamp'] for r in candidate.get('frames', [])]:
        reasons.append('source timestamps differ')
    same_geometry = all(baseline.get('camera', {}).get(key) == candidate.get('camera', {}).get(key)
                        for key in ('K', 'distortion'))
    if control and not same_geometry:
        reasons.append('repeatability control camera geometry differs')
    if reasons:
        return {'valid': False, 'reasons': reasons, 'comparison': None}
    before = {p['frame_id'] for p in baseline['poses']}
    after = {p['frame_id'] for p in candidate['poses']}
    common = before & after
    coverage = {'retained': len(common), 'newly_unposed_frame_ids': sorted(before - after),
                'newly_accepted_frame_ids': sorted(after - before),
                'retains_every_baseline_pose': before <= after}
    fields = ('frame_id', 'timestamp', 'status', 'reason', 'features', 'matches', 'inliers', 'added_points', 'landmarks')
    differences = [a['frame_id'] for a, b in zip(baseline['frames'], candidate['frames'], strict=True)
                   if any(a[key] != b[key] for key in fields)]
    return {'valid': True, 'reasons': [], 'coverage': coverage,
            'same_processed_camera_geometry': same_geometry,
            'effective_mask_changed': baseline['feature_mask'].get('effective_sha256') != candidate['feature_mask'].get('effective_sha256'),
            'common_accepted_frame_metrics': {
                name: final_metrics([r for r in report['frames'] if r['frame_id'] in common])
                for name, report in [('baseline', baseline), ('candidate', candidate)]},
            'repeatability': {'identical_frame_outcomes': not differences, 'different_frame_ids': differences,
                              'identical_final_poses': baseline['poses'] == candidate['poses']} if control else None,
            'pipeline_elapsed_delta_seconds': candidate['elapsed_seconds'] - baseline['elapsed_seconds'],
            'baseline': summarize(baseline), 'candidate': summarize(candidate)}


def outcome_hash(report):
    """Exclude timing/provenance paths when checking repeated numerical outcomes."""
    fields = ('frame_id', 'timestamp', 'status', 'reason', 'features', 'matches', 'inliers', 'added_points', 'landmarks')
    evidence = {'frames': [{key: row[key] for key in fields} for row in report['frames']],
                'poses': report['poses'], 'landmark_retention': report.get('landmark_retention')}
    return hashlib.sha256(json.dumps(evidence, sort_keys=True, allow_nan=False).encode()).hexdigest()


def markdown(result):
    lines = ['# Controlled dashcam comparison', '', f"Status: **{result['status']}**", '',
             f"Mode: `{result['mode']}`. Accuracy remains unqualified without independent ground truth.", '',
             'Runs start at source frame 0 in fresh processes. Pair order alternates AB / BA.', '',
             '| Pair | Valid | Baseline poses | Candidate poses | Newly unposed | New poses |',
             '| --- | --- | --- | --- | --- | --- |']
    for i, pair in enumerate(result['pairs']):
        if pair['valid']:
            lines.append(f"| {i+1} | yes | {pair['baseline']['accepted_poses']} | {pair['candidate']['accepted_poses']} | "
                         f"{len(pair['coverage']['newly_unposed_frame_ids'])} | {len(pair['coverage']['newly_accepted_frame_ids'])} |")
        else:
            lines.append(f'| {i+1} | no | — | — | — | — |')
            lines.append('\nInvalid comparison: ' + '; '.join(pair['reasons']) + '\n')
    lines.extend(['', '## Interpretation', '',
                  '- JSON contains exact pose-coverage changes, loss intervals, residual/spatial metrics and timings.',
                  '- Residual statistics summarize per-frame values, not pooled pixel errors. Missing evidence stays null.',
                  '- Landmark retention and reobservation yield are bookkeeping measures, not static-scene confidence.',
                  '- Different calibrations also change rectification and effective masks. Those effects are part of the treatment.',
                  '- Timing is descriptive. Alternating order reduces order bias but does not isolate system load or certify a speedup.',
                  '- No ATE/RPE or raw cross-calibration pose-distance claims are made; map scales and origins can differ.'])
    if result.get('error'):
        lines.extend(['', 'Error: ' + result['error']])
    return '\n'.join(lines) + '\n'


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('output', type=Path, help='New experiment directory')
    cli.add_argument('--video', type=Path, default=ROOT/'sample_videos/GRMN2734.MP4')
    mode = cli.add_mutually_exclusive_group(required=True)
    mode.add_argument('--candidate-calibration', type=Path, help='Verified schema-2 candidate profile')
    mode.add_argument('--repeatability-control', action='store_true', help='Explicit same-camera control; no calibration-improvement claim')
    cli.add_argument('--baseline-calibration', type=Path)
    cli.add_argument('--baseline-settings', type=Path)
    cli.add_argument('--candidate-settings', type=Path)
    cli.add_argument('--feature-mask', type=Path)
    cli.add_argument('--max-frames', type=int, help='Bound the replay from frame zero; default entire clip')
    cli.add_argument('--repeats', type=int, default=2, help='Pairs; 2 gives AB then BA (default: 2)')
    cli.add_argument('--width', type=int, default=1024)
    cli.add_argument('--features', type=int, default=2000)
    cli.add_argument('--focal', type=float, default=525.)
    cli.add_argument('--mask-bottom', type=float, default=0.)
    cli.add_argument('--threads', type=int, default=1)
    cli.add_argument('--seed', type=int, default=0)
    args = cli.parse_args(argv)
    if (args.output.exists() or args.repeats < 1 or args.width < 1 or args.features < 30 or args.threads < 1
            or args.max_frames is not None and args.max_frames < 2
            or not np.isfinite(args.focal) or args.focal <= 0 or not np.isfinite(args.mask_bottom)
            or not 0 <= args.mask_bottom < 1 or not -2147483648 <= args.seed <= 2147483647):
        cli.error('Use a new output directory and valid positive run settings (at least 2 frames)')
    if args.repeatability_control and args.candidate_settings:
        cli.error('Repeatability control uses the baseline settings for both arms')
    candidates = {'baseline': (args.baseline_calibration, args.baseline_settings),
                  'candidate': (args.baseline_calibration, args.baseline_settings) if args.repeatability_control
                  else (args.candidate_calibration, args.candidate_settings)}
    protected = [args.video.resolve()]
    if args.feature_mask:
        protected.append(args.feature_mask.resolve())
    provenance = {}
    try:
        cap = cv2.VideoCapture(str(args.video))
        try:
            if not cap.isOpened():
                raise ValueError('Cannot open video')
            ok, image = cap.read()
            if not ok:
                raise ValueError('Cannot decode the first source frame')
            height, width = image.shape[:2]
            fps, count = cap.get(cv2.CAP_PROP_FPS), cap.get(cv2.CAP_PROP_FRAME_COUNT)
        finally:
            cap.release()
        count = int(count) if np.isfinite(count) and count > 0 else None
        if count is None and args.max_frames is None:
            raise ValueError('Unknown video length: supply --max-frames for an explicit comparison horizon')
        expected = min(count, args.max_frames) if count and args.max_frames else (count or args.max_frames)
        if expected < 2:
            raise ValueError('Video must contain at least two frames')
        for arm, (path, declaration_path) in candidates.items():
            if path is None:
                if declaration_path:
                    raise ValueError('Camera settings require a calibration profile')
                provenance[arm] = {'calibration_status': 'approximate', 'focal_source_px': args.focal}
                continue
            profile, info, paths = load_calibration(path)
            if arm == 'candidate' and not args.repeatability_control and profile.get('schema_version') != 2:
                raise ValueError('Candidate requires a schema-2 profile with verified provenance')
            declared = settings(read_json(declaration_path)[0]) if declaration_path else None
            compatibility = check_capture(profile, width, height, fps, declared)
            provenance[arm] = {'profile': info, 'compatibility': compatibility}
            protected.extend(paths)
            if declaration_path:
                protected.append(declaration_path.resolve())
        input_hashes = {str(p): sha256(p) for p in sorted(set(protected))}
    except (OSError, ValueError, cv2.error) as exc:
        cli.error(str(exc))
    sources = sorted([*ROOT.glob('*.py'), Path(__file__).resolve(), ROOT/'pyproject.toml'])
    source_hashes = {str(p): sha256(p) for p in sources}
    args.output.mkdir(parents=True)
    video_hash = input_hashes[str(args.video.resolve())]
    result = {'schema_version': 1, 'kind': 'controlled_dashcam_comparison', 'status': 'running',
              'mode': 'repeatability_control' if args.repeatability_control else 'calibration_comparison',
              'requested_pairs': args.repeats, 'expected_frames_per_run': expected,
              'video_sha256': video_hash, 'camera_provenance': provenance,
              'accuracy_status': 'unqualified: no independent ground truth', 'runs': [], 'pairs': []}
    manifest = {'schema_version': 1, 'input_sha256': input_hashes, 'source_sha256': source_hashes,
                'dependencies': {name: importlib.metadata.version(name) for name in ('numpy', 'scipy', 'g2opy')},
                'native_thread_environment': {name: os.getenv(name) for name in
                                              ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS')},
                'configuration': {k: str(v.resolve()) if isinstance(v, Path) else v for k, v in vars(args).items()},
                'expected_frames': expected, 'commands': [],
                'timing_scope': 'fresh subprocess wall time includes imports, report serialization and hashing; pipeline elapsed excludes startup/report export; per-frame processing excludes decode/resize/remap',
                'policy': 'All algorithm flags fixed equally; diagnostics without image overlays; sequential pairs with alternating order'}
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=ROOT, capture_output=True, text=True)
    manifest['git_revision'] = revision.stdout.strip() if revision.returncode == 0 else None
    diff = subprocess.run(['git', 'diff', 'HEAD', '--binary'], cwd=ROOT, capture_output=True)
    manifest['tracked_diff_sha256'] = hashlib.sha256(diff.stdout).hexdigest() if diff.returncode == 0 else None
    save(args.output/'manifest.json', manifest)
    try:
        for pair_index in range(args.repeats):
            reports = {}
            for arm in (('baseline', 'candidate') if pair_index % 2 == 0 else ('candidate', 'baseline')):
                if any(sha256(Path(p)) != h for p, h in {**input_hashes, **source_hashes}.items()):
                    raise ValueError('Input or implementation changed during the experiment')
                run_dir = args.output/f'pair-{pair_index+1:02d}-{arm}'
                run_dir.mkdir()
                report_path = (run_dir/'report.json').resolve()
                command = [sys.executable, str(ROOT/'slam.py'), str(args.video.resolve()), '--headless', '--diagnostics',
                           '--start-frame', '0', '--max-frames', str(expected), '--width', str(args.width),
                           '--features', str(args.features), '--focal', str(args.focal), '--mask-bottom', str(args.mask_bottom),
                           '--seed', str(args.seed), '--threads', str(args.threads), '--condition-pnp', '--spatial-mapping',
                           '--no-robust-pnp', '--recovery', '--no-landmark-maturity', '--observation-history',
                           '--report', str(report_path)]
                path, declaration_path = candidates[arm]
                if path:
                    command.extend(['--calibration', str(path.resolve())])
                if declaration_path:
                    command.extend(['--camera-settings', str(declaration_path.resolve())])
                if args.feature_mask:
                    command.extend(['--feature-mask', str(args.feature_mask.resolve())])
                manifest['commands'].append(command)
                save(args.output/'manifest.json', manifest)
                print(f'Pair {pair_index+1}/{args.repeats}: {arm}, {expected} frames', flush=True)
                start = time.perf_counter()
                with (run_dir/'run.log').open('w') as log:
                    process = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                wall = time.perf_counter() - start
                if any(sha256(Path(p)) != h for p, h in {**input_hashes, **source_hashes}.items()):
                    raise ValueError('Input or implementation changed during the experiment')
                report, report_hash = read_json(report_path)
                result['runs'].append({'pair': pair_index+1, 'arm': arm, 'returncode': process.returncode,
                                       'wall_seconds': wall, 'report': str(report_path), 'report_sha256': report_hash,
                                       'outcome_sha256': outcome_hash(report), 'environment': report['environment']})
                if process.returncode not in (0, 2):
                    raise ValueError(f'{arm} process failed with exit code {process.returncode}; inspect {run_dir}/run.log')
                reports[arm] = report
                save(args.output/'comparison.json', result)
            pair = compare_pair(reports['baseline'], reports['candidate'], expected, video_hash, args.repeatability_control)
            result['pairs'].append(pair)
            save(args.output/'comparison.json', result)
        result['status'] = 'completed' if all(p['valid'] for p in result['pairs']) else 'invalid'
        result['runtime_consistent_across_runs'] = len({json.dumps(r['environment'], sort_keys=True) for r in result['runs']}) == 1
        if not result['runtime_consistent_across_runs']:
            result['status'] = 'invalid'
        if result['status'] == 'completed':
            result['paired_pipeline_elapsed_delta_seconds'] = stats(p['pipeline_elapsed_delta_seconds'] for p in result['pairs'])
        if args.repeatability_control and result['status'] == 'completed':
            result['repeatability_passed'] = all(p['repeatability']['identical_frame_outcomes'] and
                                                 p['repeatability']['identical_final_poses'] for p in result['pairs']) and len(
                                                     {r['outcome_sha256'] for r in result['runs']}) == 1
            if not result['repeatability_passed']:
                result['status'] = 'repeatability_failed'
    except (OSError, ValueError, KeyError, TypeError, KeyboardInterrupt) as exc:
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
    save(args.output/'comparison.json', result)
    (args.output/'README.md').write_text(markdown(result))
    print(f"Comparison {result['status']}: {args.output/'README.md'}", flush=True)
    return 0 if result['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
