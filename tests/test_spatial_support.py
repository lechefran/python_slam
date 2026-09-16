"""Spatial distribution, concentration and thin-band evidence in pixel coordinates."""

import numpy as np
import pytest

from geometry import spatial_support


def test_uniform_grid_support_is_independent_of_image_resolution():
    pixels = np.array([(x, y) for y in (45, 135, 225, 315) for x in (80, 240, 400, 560)])
    support = spatial_support(pixels, 640, 360)
    assert support == spatial_support(pixels * 2, 1280, 720)
    assert support['occupied_cells'] == 16 and support['effective_cells'] == 16
    assert support['largest_cell_fraction'] == 1 / 16
    assert support['grid_counts'] == [[1] * 4] * 4
    np.testing.assert_allclose(support['central_90_span_fraction'], [.75, .75])


def test_isolated_extremes_do_not_hide_concentrated_support():
    clustered = np.tile([320., 180.], (96, 1))
    extremes = np.array([[1, 1], [639, 1], [1, 359], [639, 359]])
    pixels = np.vstack((clustered, extremes))
    assert np.all(np.ptp(pixels, axis=0) / [640, 360] > .9)
    support = spatial_support(pixels, 640, 360)
    assert support['largest_cell_fraction'] == .96
    assert support['effective_cells'] < 1.1
    assert support['central_90_span_fraction'] == [0., 0.]


def test_diagonal_band_has_small_minor_axis_despite_large_bounding_box():
    positions = np.linspace(.1, .9, 80)
    pixels = np.column_stack((positions * 640, positions * 360))
    support = spatial_support(pixels, 640, 360)
    assert min(support['central_90_span_fraction']) > .7
    assert support['minor_axis_std_fraction'] < 1e-8


def test_empty_invalid_and_outside_pixels_are_explicit():
    support = spatial_support([[np.nan, 0], [640, 200], [-1, 3], [10, 360]], 640, 360)
    assert support['in_image_count'] == 0 and support['outside_or_nonfinite_count'] == 4
    assert support['effective_cells'] == support['minor_axis_std_fraction'] == 0
    assert spatial_support([], 640, 360)['input_count'] == 0
    with pytest.raises(ValueError):
        spatial_support([], 0, 360)
