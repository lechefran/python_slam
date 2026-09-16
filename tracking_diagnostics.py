"""Passive tracking evidence and annotated, processed-image snapshots.

No estimator is rerun here. Rejected candidate poses are diagnostic evidence,
never accepted trajectory poses. Coordinates are pixels unless labelled XYZ/T_cw.
"""

from dataclasses import asdict
import json
from pathlib import Path

import cv2
import numpy as np


def residual_summary(values):
    """Summarize finite pixel residuals; missing measurements remain null."""
    values = np.asarray(values)
    finite = values[np.isfinite(values)]
    return {'finite_count': len(finite), 'invalid_count': len(values) - len(finite),
            'median': float(np.median(finite)) if len(finite) else None,
            'p95': float(np.percentile(finite, 95)) if len(finite) else None,
            'max': float(np.max(finite)) if len(finite) else None}


def json_safe(value):
    """Represent invalid projections as null instead of non-standard JSON NaN."""
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, data):
    Path(path).write_text(json.dumps(json_safe(data), indent=2, allow_nan=False) + '\n')


def overlay(image, result, trace):
    """Show descriptor, seed-PnP, projection, and final-PnP filtering in BGR.

Each panel uses the same processed image. Green means geometric inlier, not
accepted camera: the final coverage gate may still reject that entire estimate.
"""
    height, width = image.shape[:2]
    panels = []
    stages = result.diagnostics['stages']

    def panel(title):
        canvas = np.zeros((height + 104, width, 3), np.uint8)
        canvas[104:] = image
        cv2.putText(canvas, title, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
        return canvas

    def label(canvas, text, row=1):
        cv2.putText(canvas, text, (12, 25 + row * 23), cv2.FONT_HERSHEY_SIMPLEX, .48,
                    (230, 230, 230), 1, cv2.LINE_AA)

    def dot(canvas, pixel, color, radius=2):
        if pixel is None or any(x is None for x in pixel):
            return
        u, v = pixel
        if np.isfinite([u, v]).all() and 0 <= u < width and 0 <= v < height:
            cv2.circle(canvas, (int(round(u)), int(round(v)) + 104), radius, color, 1, cv2.LINE_AA)

    first = panel(f'Frame {result.frame_id}: {result.status} | descriptor filtering')
    label(first, f'{result.features} features; {result.matches} unique matches to reference')
    label(first, 'Gray: ORB features; yellow: surviving matches', 2)
    label(first, f'Reference frame: {result.diagnostics["reference_frame_id"]}', 3)
    for pixel in trace.get('feature_pixels', []):
        dot(first, pixel, (130, 130, 130), 1)
    for pixel in trace.get('matches', {}).get('current_pixels', []):
        dot(first, pixel, (0, 255, 255))
    panels.append(first)

    def pnp_panel(name, title):
        canvas = panel(title)
        metrics = stages.get(name)
        sample = trace.get(name, {})
        if metrics is None:
            label(canvas, 'Not attempted: an earlier stage stopped tracking')
            return canvas
        label(canvas, f'Input {metrics["input_count"]}; RANSAC mask {metrics["ransac_inliers"]}; validated {metrics["refined_inliers"]}')
        refinement = metrics.get('refinement', {})
        method = refinement.get('selected') or refinement.get('inspected', 'legacy LM')
        label(canvas, f'{metrics["status"]}: {metrics["gate"]}; candidate={method}', 2)
        fraction = metrics.get('span_fraction')
        support = metrics.get('spatial_support')
        spatial_label = (f'; cells {support["occupied_cells"]}/16; peak {support["largest_cell_fraction"]:.0%}'
                         if support else '; green=inlier red=rejected')
        label(canvas, f'Span x/y: {fraction[0]:.3f}/{fraction[1]:.3f}' + spatial_label
              if fraction else 'Green: refined inlier; red: rejected/unresolved input', 3)
        if support:
            for division in range(1, 4):
                cv2.line(canvas, (width * division // 4, 104),
                         (width * division // 4, height + 103), (70, 70, 70), 1)
                cv2.line(canvas, (0, 104 + height * division // 4),
                         (width - 1, 104 + height * division // 4), (70, 70, 70), 1)
        kept = set(sample.get('refined_rows', []))
        for row, pixel in enumerate(sample.get('pixels', [])):
            dot(canvas, pixel, (0, 255, 0) if row in kept else (0, 0, 255), 3)
        bounds = metrics.get('inlier_bounds_px')
        if bounds:
            lo, hi = np.rint(bounds).astype(int)
            cv2.rectangle(canvas, (lo[0], lo[1] + 104), (hi[0], hi[1] + 104), (255, 255, 255), 1)
        return canvas

    panels.append(pnp_panel('provisional_pnp', 'Seed pose: previous-frame landmark matches'))
    third = panel('Map projection search: provisional pose, 5-pixel radius')
    metrics = stages.get('projection_search')
    if metrics is None:
        label(third, 'Not attempted: seed PnP was rejected')
    else:
        label(third, f'Candidates {metrics["candidates"]}; in image {metrics["in_image"]}; added {metrics["added_matches"]}')
        label(third, 'Magenta: projected landmarks; cyan: new descriptor matches', 2)
        for pixel in trace.get('projection_search', {}).get('visible_pixels', []):
            dot(third, pixel, (255, 0, 255))
        for index in trace.get('projection_search', {}).get('added_feature_indices', []):
            dot(third, trace['feature_pixels'][index], (255, 255, 0), 4)
    panels.append(third)
    panels.append(pnp_panel('final_pnp', 'Final pose: geometric inliers and image-coverage gate'))
    return np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))


class DiagnosticWriter:
    """Write sampled frames and status transitions inside an inclusive window."""

    def __init__(self, directory, start, end, every):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.start, self.end, self.every = start, end, every
        self.previous_status = None
        self.samples = []

    def captures(self, frame_id):
        return self.start <= frame_id <= self.end

    def write(self, image, result, trace):
        transition = self.previous_status != result.status
        self.previous_status = result.status
        if not self.captures(result.frame_id):
            return
        if (result.frame_id - self.start) % self.every and result.frame_id != self.end and not transition:
            return
        stem = f'frame-{result.frame_id:06d}'
        png = self.directory / f'{stem}.png'
        if not cv2.imwrite(str(png), overlay(image, result, trace)):
            raise OSError(f'Cannot write diagnostic image: {png}')
        write_json(self.directory / f'{stem}.json', {'schema_version': 1,
            'coordinate_contract': {'pixels': 'processed rectified image (u,v)',
                                    'xyz': 'world, arbitrary scale', 'T_cw': 'world to camera'},
            'snapshot_phase': 'tracking estimates before bundle adjustment and culling',
            'result': asdict(result), 'trace': trace})
        self.samples.append({'frame_id': result.frame_id, 'status': result.status,
                             'image': png.name, 'evidence': f'{stem}.json'})

    def finish(self):
        write_json(self.directory / 'index.json', {'schema_version': 1, 'start': self.start,
                   'end': self.end, 'every': self.every, 'samples': self.samples})
