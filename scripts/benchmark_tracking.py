#!/usr/bin/env python3
"""Replay from frame zero and summarize an inclusive tracking-failure window."""

import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys

from slam import main as slam_main
from tracking_diagnostics import write_json


ROOT = Path(__file__).resolve().parents[1]


def summarize(report, start, end):
    """Keep failed estimates visible; accepted-pose counts alone hide rejection."""
    rows = [row for row in report['frames'] if start <= row['frame_id'] <= end]
    losses = [row for row in rows if row['status'] == 'lost']
    gates = Counter()
    methods, fallbacks = Counter(), Counter()
    for row in rows:
        for stage in ('provisional_pnp', 'final_pnp'):
            refinement = row['diagnostics']['stages'].get(stage, {}).get('refinement')
            if refinement:
                methods[f'{stage}/{refinement["selected"] or "rejected"}'] += 1
                if refinement['fallback_attempted']:
                    fallbacks[refinement['reason']] += 1
    for row in losses:
        evidence = row['diagnostics']
        stage = evidence.get('failure_stage', 'unknown')
        gate = evidence['stages'].get(stage, {}).get('gate', 'unknown')
        gates[f'{stage}/{gate}'] += 1
    return {'schema_version': 1, 'focus_start': start, 'focus_end': end,
            'decoded_frames': report['decoded_frames'], 'accepted_poses': report['accepted_poses'],
            'last_accepted_frame': report['poses'][-1]['frame_id'] if report['poses'] else None,
            'landmarks': report['landmarks'],
            'complete_window': [row['frame_id'] for row in rows] == list(range(start, end + 1)),
            'focus_frames': len(rows), 'states': dict(Counter(row['status'] for row in rows)),
            'first_lost_frame': losses[0]['frame_id'] if losses else None,
            'last_tracking_frame': next((row['frame_id'] for row in reversed(rows)
                                         if row['status'] == 'tracking'), None),
            'rejection_gates': dict(gates), 'reasons': dict(Counter(row['reason'] for row in losses)),
            'refinement_methods': dict(methods), 'refinement_fallback_reasons': dict(fallbacks),
            'video_sha256': report['video_sha256'], 'calibration_status': report['camera']['calibration_status'],
            'calibration_sha256': report.get('calibration_sha256'),
            'accuracy_status': 'unqualified: no independent ground truth',
            'timing_scope': 'processing includes diagnostic capture; artifact I/O separately timed; no speed claim'}


def compare_baseline(report, baseline):
    """Compare matching prefix outcomes, not poses later changed by different BA horizons."""
    config_keys = ('start_frame', 'width', 'focal', 'features', 'mask_bottom', 'seed', 'threads')
    compatible = (report.get('video_sha256') == baseline.get('video_sha256')
        and report.get('calibration_sha256') == baseline.get('calibration_sha256')
        and report['camera'] == baseline['camera']
        and all(report['configuration'].get(key) == baseline['configuration'].get(key) for key in config_keys))
    same_solver = (report['configuration'].get('condition_pnp', False)
                   == baseline['configuration'].get('condition_pnp', False))
    fields = ('timestamp', 'status', 'reason', 'features', 'matches', 'inliers', 'added_points', 'landmarks')
    previous = {row['frame_id']: row for row in baseline['frames']}
    differences = [{'frame_id': row['frame_id'],
                    'fields': [key for key in fields if row[key] != previous.get(row['frame_id'], {}).get(key)]}
                   for row in report['frames']]
    differences = [row for row in differences if row['fields']]
    # Compare frame identities as well as counts: extra later poses must not hide
    # losing frames the baseline tracked. Restrict both sets to the replay horizon.
    compared_ids = {row['frame_id'] for row in report['frames']}
    before = {pose['frame_id'] for pose in baseline['poses']} & compared_ids
    after = {pose['frame_id'] for pose in report['poses']} & compared_ids
    coverage = {'baseline_accepted_poses': len(before), 'current_accepted_poses': len(after),
                'retained_baseline_poses': len(before & after),
                'retains_all_baseline_frames': before <= after,
                'newly_lost_frame_ids': sorted(before - after),
                'newly_accepted_frame_ids': sorted(after - before)}
    return {'compatible_inputs': compatible, 'same_solver_configuration': same_solver,
            'coverage_comparison': coverage, 'compared_frames': len(report['frames']),
            'identical_frame_outcomes': compatible and not differences, 'differences': differences,
            'baseline_environment': baseline['environment'],
            'note': 'Compares listed per-frame outcomes; excludes timing and final optimized poses.'}


def write_table(path, report, start, end):
    fields = ['frame', 'status', 'reason', 'reference', 'reference_age', 'features', 'matches',
              'seed_inputs', 'seed_ransac', 'seed_refined', 'projected_in_image', 'projection_added',
              'final_inputs', 'final_ransac', 'final_refined', 'span_x', 'span_y',
              'inlier_median_px', 'inlier_p95_px', 'failure_stage']
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in report['frames']:
            if not start <= row['frame_id'] <= end:
                continue
            evidence = row['diagnostics']
            stages = evidence['stages']
            seed, search, final = (stages.get(key, {}) for key in ('provisional_pnp', 'projection_search', 'final_pnp'))
            span = final.get('span_fraction', [None, None])
            residual = final.get('inlier_residual_px', {})
            writer.writerow(dict(zip(fields, [row['frame_id'], row['status'], row['reason'],
                evidence['reference_frame_id'], evidence['reference_age_frames'], row['features'], row['matches'],
                seed.get('input_count'), seed.get('ransac_inliers'), seed.get('refined_inliers'),
                search.get('in_image'), search.get('added_matches'), final.get('input_count'),
                final.get('ransac_inliers'), final.get('refined_inliers'), *span,
                residual.get('median'), residual.get('p95'), evidence.get('failure_stage')])) )


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('output', type=Path, help='New output directory; existing runs are never overwritten')
    cli.add_argument('--video', type=Path, default=ROOT / 'sample_videos/GRMN2734.MP4')
    cli.add_argument('--focus-start', type=int, default=850)
    cli.add_argument('--focus-end', type=int, default=1000)
    cli.add_argument('--every', type=int, default=10)
    cli.add_argument('--calibration', type=Path)
    cli.add_argument('--focal', type=float, default=525)
    cli.add_argument('--condition-pnp', action=argparse.BooleanOptionalAction, default=True,
                     help='Centre/scale pose fitting (default); disable for the legacy reference')
    cli.add_argument('--baseline', type=Path, help='Optional earlier SLAM report for prefix outcome comparison')
    args = cli.parse_args(argv)
    if args.focus_start < 0 or args.focus_end < args.focus_start or args.every < 1:
        cli.error('Invalid inclusive focus range or sampling stride')
    if not args.video.is_file():
        cli.error(f'Video does not exist: {args.video}')
    if args.output.exists():
        cli.error('Output directory already exists; choose a new run name')
    baseline = json.loads(args.baseline.read_text()) if args.baseline else None
    args.output.mkdir(parents=True)
    # Replay every frame before the focus window: starting at 850 would create a
    # different map and fail to reproduce the accumulated state at the first loss.
    command = [str(args.video), '--headless', '--start-frame', '0', '--max-frames', str(args.focus_end + 1),
               '--width', '1024', '--features', '2000', '--mask-bottom', '0', '--seed', '0', '--threads', '1',
               '--focal', str(args.focal), '--report', str(args.output / 'report.json'),
               '--diagnostics-dir', str(args.output / 'frames'), '--diagnostics-start', str(args.focus_start),
               '--diagnostics-end', str(args.focus_end), '--diagnostics-every', str(args.every)]
    if args.calibration:
        command.extend(['--calibration', str(args.calibration)])
    command.append('--condition-pnp' if args.condition_pnp else '--no-condition-pnp')
    sources = [*ROOT.glob('*.py'), Path(__file__), ROOT / 'pyproject.toml']
    write_json(args.output / 'manifest.json', {'schema_version': 1, 'argv': command,
        'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted(sources)},
        'warmup_start_frame': 0, 'focus_range_inclusive': [args.focus_start, args.focus_end]})
    code = slam_main(command)
    report = json.loads((args.output / 'report.json').read_text())
    summary = summarize(report, args.focus_start, args.focus_end)
    if baseline is not None:
        summary['baseline_comparison'] = compare_baseline(report, baseline)
    write_json(args.output / 'summary.json', summary)
    write_table(args.output / 'window.csv', report, args.focus_start, args.focus_end)
    print(json.dumps(summary, indent=2))
    return code if code else (0 if summary['complete_window'] else 1)


if __name__ == '__main__':
    sys.exit(main())
