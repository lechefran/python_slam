"""Quality diagnostics must reveal missing evidence without changing a fit."""

import copy
from html.parser import HTMLParser
import json

import numpy as np
import pytest

from calibration import View, residual_summary
from calibration_report import assess_quality, finalize_report, html_report, main


def measured_summary(errors):
    """Project a small board into the upper-left band with analytic pinhole UV.

    Each supplied (N,2) error is predicted minus observed, in source pixels.
    Different array entries represent different views, not independent corners.
    """
    views, poses = [], []
    k = np.array([[100., 0, 50], [0, 100, 50], [0, 0, 1]])
    for i, delta in enumerate(errors):
        n = len(delta)
        xy = np.column_stack((np.arange(n) % 10, np.arange(n) // 10)) * .001 - .44
        objects = np.column_stack((xy, np.zeros(n))).astype(np.float32)
        predicted = objects[:, :2].astype(float) * 100 + 50
        views.append(View(f'view-{i}.png', objects, (predicted - delta).astype(np.float32)))
        poses.append((np.zeros(3), np.array([0., 0., 1.])))
    return residual_summary(views, poses, k, np.zeros(5), (100, 100))


def test_signed_residuals_empty_cells_and_distinct_view_counts():
    summary = measured_summary([np.tile([3., 4.], (100, 1)), np.zeros((100, 2))])
    assert summary['rms_px'] == pytest.approx(np.sqrt(12.5), abs=1e-5)
    np.testing.assert_allclose(summary['mean_residual_uv_px'], [1.5, 2], atol=1e-5)
    occupied = summary['spatial_residuals'][0]
    assert occupied['corners'] == 200
    assert occupied['views'] == 2
    assert summary['edge_coverage']['left']['views'] == 2
    assert summary['edge_coverage']['right'] == {'corners': 0, 'views': 0, 'rms_px': None}
    assert summary['spatial_residuals'][-1]['rms_px'] is None
    assert summary['views'][0]['board_hull_fraction'] > 0
    np.testing.assert_allclose(summary['views'][0]['residuals_uv_px'], np.tile([3., 4.], (100, 1)), atol=1e-5)


def test_bad_corner_is_flagged_despite_low_global_rms():
    errors = np.zeros((100, 2))
    errors[0, 0] = 6
    fitting = measured_summary([np.zeros((100, 2))] * 8)
    validation = measured_summary([errors] + [np.zeros((100, 2))] * 3)
    result = assess_quality(fitting, validation, 25)
    assert validation['rms_px'] < 1
    view = validation['views'][0]
    assert view['review_recommended']
    assert 'max' in view['review_reasons']
    assert 'relative_rms' in view['review_reasons']
    assert any(c['status'] == 'review' and c['code'] == 'view_max' for c in result['checks'])
    assert len(validation['views']) == 4  # Nothing is removed to improve the score.


def test_dense_single_view_does_not_pass_spatial_support():
    fitting = measured_summary([np.zeros((100, 2))])
    validation = measured_summary([np.zeros((100, 2))])
    result = assess_quality(fitting, validation, 5)
    assert fitting['multiply_observed_grid_cells'] == 0
    codes = {c['code'] for c in result['checks'] if c['status'] == 'review'}
    assert {'grid_view_support', 'edge_left', 'edge_right', 'normal_span'} <= codes


def test_validation_gap_requires_absolute_and_relative_increase():
    fitting = measured_summary([np.zeros((100, 2))] * 8)
    validation = measured_summary([np.tile([.01, 0], (100, 1))] * 3)
    result = assess_quality(fitting, validation, 25)
    assert next(c for c in result['checks'] if c['code'] == 'validation_gap')['status'] == 'pass'
    validation = measured_summary([np.tile([.5, 0], (100, 1))] * 3)
    result = assess_quality(fitting, validation, 25)
    assert next(c for c in result['checks'] if c['code'] == 'validation_gap')['status'] == 'review'
    assert result['split_comparison']['rms_gap_px'] == pytest.approx(.5, abs=1e-5)


def report_fixture():
    fitting = measured_summary([np.zeros((100, 2))] * 8)
    validation = measured_summary([np.zeros((100, 2))] * 3)
    report = {'kind': 'camera_calibration_report', 'schema_version': 2, 'status': 'needs_review',
              'camera': '<script>alert("camera")</script>', 'images': [
                  {'split': 'fitting', 'path': '<img src=x onerror=alert(1)>.png', 'status': 'board_not_found'}],
              'fitting': fitting, 'validation': validation,
              **assess_quality(fitting, validation, 25)}
    finalize_report(report)
    return report


def test_html_contains_metrics_and_escapes_all_user_strings():
    report = report_fixture()
    before = copy.deepcopy(report)
    html = html_report(report)
    assert report == before  # Review must never rewrite the recorded evidence.
    assert '&lt;script&gt;' in html and '&lt;img src=x' in html
    assert '<script>' not in html and '<img ' not in html
    assert 'Not measured' in html
    assert 'Fitting versus held-out' in html
    assert 'Outer 10%' in html
    assert 'mean squared' in html
    class Tags(HTMLParser):
        def handle_starttag(self, tag, attrs):
            assert tag not in ('script', 'iframe', 'link', 'img')
            assert not any(key.startswith('on') for key, _ in attrs)
    Tags().feed(html)


def test_legacy_failure_report_and_exclusive_renderer_output(tmp_path):
    report = {'kind': 'camera_calibration_report', 'schema_version': 1, 'status': 'failed',
              'error': 'Need at least 8 fitting and 3 validation detections', 'images': []}
    source, output = tmp_path/'report.json', tmp_path/'review.html'
    source.write_text(json.dumps(report))
    assert main([str(source), '--output', str(output)]) == 0
    contents = output.read_text()
    assert 'Calibration failed' in contents and 'No valid camera fit' in contents
    assert 'Camera parameters not embedded' in contents
    with pytest.raises(SystemExit):
        main([str(source), '--output', str(output)])
    assert output.read_text() == contents


def test_legacy_success_renders_missing_new_fields_as_unknown():
    report = report_fixture()
    report['schema_version'] = 1
    for summary in (report['fitting'], report['validation']):
        del summary['edge_coverage']
        for cell in summary['spatial_residuals']:
            del cell['views']
    html = html_report(report)
    assert 'unknown views' in html
    assert 'Camera parameters not embedded' in html


def test_unsupported_report_fails_without_writing_html(tmp_path):
    source, output = tmp_path/'report.json', tmp_path/'review.html'
    source.write_text('{"kind":"camera_calibration_report","schema_version":99}')
    assert main([str(source), '--output', str(output)]) == 2
    assert not output.exists()


def test_zero_fitting_rms_ratio_is_null():
    fitting = measured_summary([np.zeros((100, 2))])
    validation = copy.deepcopy(fitting)
    # This tests the explicitly undefined ratio, independently of floating-point
    # projection roundoff in the synthetic camera fixture.
    fitting['rms_px'] = 0.0
    validation['rms_px'] = .01
    result = assess_quality(fitting, validation, 25)
    assert result['split_comparison']['rms_ratio'] is None
    json.dumps(result, allow_nan=False)
