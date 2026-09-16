"""Bounded, passive landmark evidence; never a pose weight or confidence score."""

from collections import Counter, deque
from dataclasses import asdict, dataclass
import json
from pathlib import Path

import numpy as np


def finite_or_none(value):
    return float(value) if value is not None and np.isfinite(value) else None


@dataclass(slots=True)
class QualitySample:
    assessed_frame_id: int
    observation_frame_id: int
    phase: str
    outcome: str
    residual_px: float | None
    depth: float | None
    feature_index: int | None


class ObservationQuality:
    """Keep 16 recent events plus lifetime counters, with no camera references.

    Search rates use accepted-pose, in-image, unmasked opportunities actually
    searched. Cull rechecks are separate measurements of historical observations.
    Rechecking an unchanged residual is not another independent observation.
    """
    __slots__ = ('samples', 'counts', 'last_search_frame', 'last_cull', 'first_seen', 'last_seen')

    def __init__(self):
        self.samples = deque(maxlen=16)
        self.counts = Counter()
        self.last_search_frame = None
        self.last_cull = None
        self.first_seen = None
        self.last_seen = None

    def record(self, assessed, observed, phase, outcome, residual=None, depth=None, index=None):
        residual, depth = finite_or_none(residual), finite_or_none(depth)
        if phase == 'search':
            if self.last_search_frame is not None and assessed <= self.last_search_frame:
                return False
            self.last_search_frame = assessed
        if phase == 'cull':
            signature = (observed, outcome, residual, depth, index)
            if self.last_cull == signature:
                return False
            self.last_cull = signature
        if phase == 'observation' and outcome == 'added':
            self.first_seen = observed if self.first_seen is None else min(self.first_seen, observed)
            self.last_seen = observed if self.last_seen is None else max(self.last_seen, observed)
        self.counts[f'{phase}/{outcome}'] += 1
        self.samples.append(QualitySample(int(assessed), int(observed), phase, outcome, residual, depth,
                                          int(index) if index is not None else None))
        return True

    def summary(self):
        accepted = self.counts['search/accepted']
        assessed = accepted + self.counts['search/rejected'] + self.counts['search/unmatched']
        residuals = [s.residual_px for s in self.samples
                     if s.phase == 'observation' and s.outcome == 'added' and s.residual_px is not None]
        return {'counts': dict(self.counts), 'assessed_searches': assessed,
                'successful_reobservation_ratio': accepted / assessed if assessed else None,
                'first_observed_frame_id': self.first_seen, 'last_observed_frame_id': self.last_seen,
                'recent_added_residual_px': {'samples': len(residuals),
                    'median': float(np.median(residuals)) if residuals else None,
                    'max': max(residuals) if residuals else None},
                'history_capacity': self.samples.maxlen, 'recent_events': [asdict(s) for s in self.samples]}


def measurement(point, frame, index):
    """Evaluate a committed observation with current T_cw; pixels and map units.

    This snapshot is not recalculated when BA later changes either state.
    Invalid projection values become null in evidence, not JSON NaN/Infinity.
    """
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        camera = frame.pose[:3, :3] @ point + frame.pose[:3, 3]
        pixel = frame.k @ camera
        residual = (np.linalg.norm(pixel[:2] / pixel[2] - frame._kps[index])
                    if np.isfinite(camera).all() and camera[2] > 0 else None)
    return finite_or_none(residual), finite_or_none(camera[2])


def write_quality_report(path, metadata, points, retired, current_id):
    """Stream one landmark at a time, then atomically publish the complete JSON.

    History is bounded per point, but the map is not globally bounded. Avoid
    materializing a second whole-map tree or giant JSON string during export.
    """
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    with temporary.open('w') as handle:
        handle.write('{\n')
        for key, value in metadata.items():
            handle.write(json.dumps(key) + ': ')
            json.dump(value, handle, allow_nan=False)
            handle.write(',\n')
        handle.write('"live_landmarks": [\n')
        for number, point in enumerate(points):
            if number:
                handle.write(',\n')
            json.dump(point.quality_snapshot(current_id), handle, allow_nan=False)
        handle.write('\n],\n"recent_retired_landmarks": ')
        json.dump(list(retired), handle, allow_nan=False)
        handle.write('\n}\n')
    temporary.replace(path)
