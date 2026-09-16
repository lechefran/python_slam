"""Calibration quality assessment and portable, offline HTML review.

Quality thresholds are review heuristics, not camera certification. This module
renders saved evidence without opening source images or rerunning calibration.
"""

import argparse
from collections import Counter
from html import escape
import json
from pathlib import Path
import sys

import numpy as np


QUALITY_POLICY = {
    'review_rms_px': 1.0, 'review_p95_px': 2.0, 'review_max_px': 5.0,
    'minimum_occupied_grid_cells': 9, 'minimum_views_per_cell': 2,
    'edge_band_fraction': .10, 'minimum_views_per_edge': 2,
    'minimum_normal_span_degrees': 10.0,
    'outlier_mad_multiplier': 3.0, 'outlier_sigma_factor': 1.4826,
    'outlier_rms_floor_px': .25,
    'validation_gap_ratio': 2.0, 'validation_gap_px': .25,
}


def assess_quality(fitting, validation, normal_span):
    """Flag residual tails, spatial gaps and validation degradation after fitting.

    Both summaries use source pixels. Median/MAD compares per-view RMS within
    each split so a large board does not dominate the outlier threshold.
    """
    policy = QUALITY_POLICY
    checks = []

    def check(code, split, flagged, observed, threshold, message, action, path=None):
        checks.append(dict(code=code, split=split, path=path,
                           status='review' if flagged else 'pass', observed=observed,
                           threshold=threshold, message=message, action=action))

    check('normal_span', 'fitting', normal_span < policy['minimum_normal_span_degrees'],
          normal_span, policy['minimum_normal_span_degrees'],
          'Fitting board normals span less than 10 degrees', 'Capture stronger horizontal and vertical tilts')
    for split, summary in [('fitting', fitting), ('validation', validation)]:
        check('rms', split, summary['rms_px'] > policy['review_rms_px'], summary['rms_px'],
              policy['review_rms_px'], f'{split.capitalize()} RMS exceeds 1 source pixel',
              'Inspect corner detections, blur, board flatness and lens-model suitability')
        check('grid_coverage', split, summary['occupied_grid_cells'] < 9,
              summary['occupied_grid_cells'], 9, f'Limited {split} image coverage: fewer than 9/12 occupied grid cells',
              'Capture the board across the full image, including corners')
        supported = sum(cell['views'] >= policy['minimum_views_per_cell'] for cell in summary['spatial_residuals'])
        summary['multiply_observed_grid_cells'] = supported
        check('grid_view_support', split, supported < 9, supported, 9,
              f'Limited {split} repeated spatial support: fewer than 9/12 cells seen in at least two views',
              'Add distinct tilted views across sparsely sampled image regions')
        for edge, values in summary['edge_coverage'].items():
            check(f'edge_{edge}', split, values['views'] < policy['minimum_views_per_edge'],
                  values['views'], policy['minimum_views_per_edge'],
                  f'{split.capitalize()} {edge} edge has fewer than two supporting views',
                  'Capture corners in this outer 10% image band; occupancy does not establish coverage of its whole length')
        rms = np.array([view['rms_px'] for view in summary['views']])
        median = float(np.median(rms))
        mad = float(np.median(np.abs(rms - median)))
        threshold = max(policy['outlier_rms_floor_px'], median +
                        policy['outlier_mad_multiplier'] * policy['outlier_sigma_factor'] * mad)
        summary['view_outlier_reference'] = {'median_rms_px': median, 'mad_rms_px': mad,
                                            'threshold_rms_px': threshold}
        for view in summary['views']:
            rules = [('rms', view['rms_px'], policy['review_rms_px']),
                     ('p95', view['p95_px'], policy['review_p95_px']),
                     ('max', view['max_px'], policy['review_max_px']),
                     ('relative_rms', view['rms_px'], threshold)]
            view['review_reasons'] = [name for name, value, limit in rules if value > limit]
            view['review_recommended'] = bool(view['review_reasons'])
            for name, value, limit in rules:
                check(f'view_{name}', split, value > limit, value, limit,
                      f'{split.capitalize()} image {name} residual exceeds its review threshold',
                      'Inspect this image and its corners; no observations were automatically removed', view['path'])
    # Require both an absolute and relative increase: tiny rounding-level fitting
    # residuals must not create alarming ratios for otherwise accurate validation.
    fit_rms, val_rms = fitting['rms_px'], validation['rms_px']
    gap = val_rms - fit_rms
    comparison = {'rms_gap_px': gap, 'rms_ratio': val_rms / fit_rms if fit_rms > 1e-9 else None}
    check('validation_gap', 'validation', gap > policy['validation_gap_px'] and
          val_rms > policy['validation_gap_ratio'] * fit_rms,
          comparison, {'gap_px': policy['validation_gap_px'], 'ratio': policy['validation_gap_ratio']},
          'Validation error is materially higher than fitting error',
          'Inspect recording-mode consistency and independent capture quality; do not tune against held-out images')
    return {'checks': checks, 'split_comparison': comparison,
            'warnings': [f"{c['message']}" + (f": {c['path']}" if c['path'] else '')
                         for c in checks if c['status'] == 'review']}


def finalize_report(report):
    """Summarize examined captures for successful and early-failure reports."""
    report['capture_summary'] = {
        split: dict(Counter(row.get('status', 'incomplete') for row in report['images'] if row['split'] == split))
        for split in ('fitting', 'validation')}
    report['review_images'] = [dict(split=split, path=view['path'], reasons=view.get('review_reasons', []))
                               for split in ('fitting', 'validation')
                               for view in report.get(split, {}).get('views', [])
                               if view.get('review_recommended')]


def html_report(report):
    """Render version 1/2 report data without scripts, network or source-file reads."""
    if (not isinstance(report, dict) or report.get('kind') != 'camera_calibration_report'
            or report.get('schema_version') not in (1, 2)
            or report.get('status') not in ('failed', 'needs_review', 'checks_passed')):
        raise ValueError('Expected a supported camera calibration report (schema 1 or 2)')

    def text(value):
        return escape(str(value), quote=True)

    def number(value):
        if value is None:
            return 'Not measured'
        return f'{float(value):.4g}'

    def table(headers, rows):
        return '<div class="scroll"><table><thead><tr>' + ''.join(f'<th>{text(h)}</th>' for h in headers) + \
            '</tr></thead><tbody>' + ''.join('<tr>' + ''.join(f'<td>{text(v)}</td>' for v in row) + '</tr>'
                                           for row in rows) + '</tbody></table></div>'

    parts = ['<!doctype html><html lang="en"><meta charset="utf-8">',
             '<meta name="viewport" content="width=device-width,initial-scale=1">',
             '<title>Calibration quality report</title><style>',
             'body{font:16px/1.5 system-ui,sans-serif;color:#172638;background:#f3f5f7;margin:0}',
             'main{max-width:1150px;margin:auto;padding:28px}section{background:white;padding:22px;margin:20px 0;border-radius:10px}',
             'h1,h2,h3{line-height:1.2}h1{margin-bottom:8px}.note{color:#46566b}',
             '.alert{border-left:5px solid #a84513;padding:12px;background:#fff1dc}',
             '.scroll{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:14px}',
             'th,td{text-align:left;padding:9px;border-bottom:1px solid #d7dfe6;overflow-wrap:anywhere;min-width:65px}',
             'th{background:#eaf0f5}.grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:6px}',
             '.cell{padding:10px;border:1px solid #b9c7d3;background:#e5f1ed;font-size:14px}',
             '.empty{background:#eceff2}.review{background:#ffe3bf}small{display:block}',
             'pre{white-space:pre-wrap;overflow-wrap:anywhere}summary{cursor:pointer;font-weight:600}',
             '@media(max-width:600px){main{padding:12px}section{padding:12px}.cell{padding:5px;font-size:12px}}',
             '@media print{body{background:white}main{max-width:none;padding:0}section{break-inside:avoid}.scroll{overflow:visible}}',
             '</style><main><h1>Calibration quality report</h1>',
             f'<p class="alert"><strong>{text(report["status"])}</strong> — '
             + ('No valid camera fit. ' if report['status'] == 'failed' else 'Measured, unverified camera fit. ') +
             'A passed check does not certify calibration or SLAM accuracy.</p>',
             f'<p>{text(report.get("camera", "Unknown camera"))} · {text(report.get("recording_mode", "Unknown mode"))}</p>',
             f'<p class="note">Created {text(report.get("created_utc", "unknown"))} · schema {report["schema_version"]}</p>']
    if report['status'] == 'failed':
        parts.append(f'<section><h2>Calibration failed</h2><p>{text(report.get("error", "Unknown failure"))}</p>'
                     '<p>No valid camera fit is established by this report. Capture examination may have stopped early.</p></section>')
    review_checks = [c for c in report.get('checks', []) if c['status'] == 'review']
    parts.append('<section><h2>Review findings</h2>')
    if review_checks:
        parts.append(table(['Split / image', 'Finding', 'Observed', 'Threshold', 'Suggested action'],
                           [(c['split'] + (' / ' + c['path'] if c.get('path') else ''), c['message'],
                             json.dumps(c['observed']), json.dumps(c['threshold']), c['action']) for c in review_checks]))
    elif report.get('warnings'):
        parts.append('<ul>' + ''.join(f'<li>{text(w)}</li>' for w in report['warnings']) + '</ul>')
    else:
        parts.append('<p>No recorded review findings. Failed or older reports may not contain all checks.</p>')
    parts.append('</section><section><h2>Fitting versus held-out validation</h2>')
    parts.append('<p>Residuals are predicted minus observed raw source pixels. RMS is the square root of mean squared '
                 '2D distance per corner. Validation freezes intrinsics but estimates each board pose from that image.</p>')
    available = [(split, report[split]) for split in ('fitting', 'validation') if split in report]
    parts.append(table(['Split', 'Views', 'RMS (px)', 'P95 (px)', 'Max (px)', 'Occupied cells'],
                       [(split, len(s['views']), number(s['rms_px']), number(s['p95_px']), number(s['max_px']),
                         f'{s["occupied_grid_cells"]}/12') for split, s in available]))
    comparison = report.get('split_comparison')
    if comparison:
        parts.append(f'<p>Validation − fitting RMS: {number(comparison["rms_gap_px"])} px. '
                     f'Ratio: {number(comparison["rms_ratio"])} (undefined for near-zero fitting RMS).</p>')
    if 'fitting_normal_span_degrees' in report:
        parts.append(f'<p>Fitting board-normal span: {number(report["fitting_normal_span_degrees"])} degrees.</p>')
    parts.append('</section>')
    for split, summary in available:
        parts.append(f'<section><h2>{text(split.capitalize())}: spatial support and residuals</h2>'
                     '<p>Image origin is top left. Each cell shows RMS, corner count and distinct supporting views. '
                     'Gray means no measurements; orange means RMS above 1 px or fewer than two views. '
                     'Other cells still require review.</p><div class="grid">')
        for cell in summary['spatial_residuals']:
            style = 'empty' if not cell['corners'] else ('review' if cell['rms_px'] > 1 or cell.get('views', 2) < 2 else '')
            parts.append(f'<div class="cell {style}"><strong>r{cell["row"]+1} c{cell["column"]+1}</strong>'
                         f'<small>{number(cell["rms_px"])} px RMS</small><small>{text(cell["corners"])} corners · '
                         f'{text(cell.get("views", "unknown"))} views</small>'
                         '<small>Mean (du,dv): ' +
                         (', '.join(number(v) for v in cell['mean_residual_uv_px'])
                          if cell.get('mean_residual_uv_px') is not None else 'Not measured') + '</small></div>')
        parts.append('</div>')
        if 'edge_coverage' in summary:
            parts.append('<h3>Outer 10% edge bands</h3>' + table(['Edge', 'Corners', 'Distinct views', 'RMS (px)'],
                         [(edge, v['corners'], v['views'], number(v['rms_px'])) for edge, v in summary['edge_coverage'].items()]))
        parts.append('<h3>Images (largest RMS first)</h3>' + table(
            ['Image', 'Corners', 'RMS / P95 / max (px)', 'Board hull / image', 'Review reasons'],
            [(v['path'], v['corners'], ' / '.join(number(v[key]) for key in ('rms_px', 'p95_px', 'max_px')),
              number(v.get('board_hull_fraction')), ', '.join(v.get('review_reasons', [])) or
              ('Review' if v.get('review_recommended') else 'None recorded'))
             for v in sorted(summary['views'], key=lambda row: row['rms_px'], reverse=True)]))
        parts.append('</section>')
    parts.append('<section><h2>Capture audit</h2><p>Examined images only; a failure can stop before all files are read.</p>')
    parts.append(table(['Split', 'Image', 'Detection status', 'Corners'],
                       [(r['split'], r['path'], r.get('status', 'incomplete'), r.get('corners', '—'))
                        for r in report.get('images', [])]))
    parts.append('</section><section><h2>Camera and provenance</h2>')
    model = report.get('camera_model')
    parts.append('<pre>' + text(json.dumps(model, indent=2) if model else 'Camera parameters not embedded in this report version') + '</pre>')
    names = report.get('intrinsics_standard_deviation_order', [])
    std = report.get('intrinsics_standard_deviations_opencv', [])
    if names and std:
        parts.append('<h3>OpenCV local standard deviations</h3><p>These are fit estimates, not calibrated confidence. '
                     'Fixed or unestimated parameters can have zero entries.</p>' +
                     table(['Parameter', 'Standard deviation', 'Units'],
                           [(name, number(value), 'pixels' if name in ('fx', 'fy', 'cx', 'cy') else
                             ('radians' if name in ('tau_x', 'tau_y') else 'dimensionless'))
                            for name, value in zip(names, std)]))
    parts.append('<details><summary>Policy, board and reproducibility metadata</summary><pre>' + text(json.dumps(
        {key: report.get(key) for key in ('policy', 'board', 'board_units', 'source_size', 'capture_settings',
                                         'camera_settings_sha256', 'execution', 'preprocessing',
                                         'opencv_version', 'numpy_version', 'validation_method')}, indent=2)) + '</pre></details>')
    parts.append('<h3>Limits</h3><ul>' + ''.join(f'<li>{text(item)}</li>' for item in report.get('limitations', [])) +
                 '</ul><p>Review flags never remove images or refit the camera automatically. Near-duplicate captures '
                 'can still overstate independent support. No metric SLAM scale or trajectory ground truth is supplied.</p></section></main></html>')
    return ''.join(parts)


def write_html(path, report):
    contents = html_report(report)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x', encoding='utf-8') as stream:
        stream.write(contents)


def main(argv=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('report', type=Path, help='Existing version 1 or 2 calibration report JSON')
    cli.add_argument('--output', type=Path, required=True, help='New standalone .html report')
    args = cli.parse_args(argv)
    if args.output.exists() or args.output.is_symlink() or args.output.suffix.lower() != '.html':
        cli.error('Choose a new .html output path; existing files are preserved')
    try:
        report = json.loads(args.report.read_text())
        write_html(args.output, report)
    except (OSError, ValueError, TypeError, KeyError, IndexError, AttributeError) as exc:
        print(f'Cannot render calibration report: {exc}', file=sys.stderr)
        return 2
    print(f'Saved {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
