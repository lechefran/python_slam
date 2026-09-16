#!/usr/bin/env python3
"""Controlled occlusion/revisit using real dashcam frames, not a natural-route test."""

import argparse
from dataclasses import asdict
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np

from slam import SLAM, camera_parameters
from tracking_diagnostics import DiagnosticWriter, write_json


ROOT = Path(__file__).resolve().parents[1]


def replay(video, output, enabled):
    """Read source 0..119, inject 45 blank frames, then revisit source 30..59.

    Input timestamps remain monotonic at the source FPS. The revisit is an
    artificial discontinuity, explicitly distinct from natural dashcam motion.
    Reset both the map and native random seed for each comparison arm.
    """
    cv2.setRNGSeed(0)
    cv2.setNumThreads(1)
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError('Cannot open recovery benchmark video')
    writer = DiagnosticWriter(output / 'frames', 165, 194, 10)
    rows = []
    tracker = None
    start = time.perf_counter()
    try:
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not np.isfinite(fps) or fps <= 0:
            raise ValueError('Recovery benchmark requires valid source FPS')
        for number in range(195):
            source_id = number if number < 120 else number - 135 if number >= 165 else None
            if number == 165 and not capture.set(cv2.CAP_PROP_POS_FRAMES, 30):
                raise ValueError('Cannot seek to the recorded revisit')
            if source_id is not None:
                ok, image = capture.read()
                if not ok:
                    raise ValueError(f'Cannot decode source frame {source_id}')
                if tracker is None:
                    width, height, k, _ = camera_parameters(image.shape[1], image.shape[0])
                    tracker = SLAM(k, recovery=enabled)
                image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            else:
                image = np.zeros((height, width, 3), np.uint8)
            _, result = tracker.process(image, number, number / fps, diagnostics=True,
                                        capture_trace=number >= 165)
            row = asdict(result)
            row['source_frame_id'] = source_id
            rows.append(row)
            if number >= 165:
                writer.write(image, result, tracker.last_trace)
        tracker.map.check_integrity()
    finally:
        capture.release()
        writer.finish()
    report = {'recovery_enabled': enabled, 'elapsed_seconds': time.perf_counter() - start,
              'frames': rows, 'accepted_poses': len(tracker.map.frames),
              'recovered_frame_ids': [row['frame_id'] for row in rows if row['recovered_from'] is not None],
              'revisit_tracked_frames': sum(row['status'] == 'tracking' for row in rows[165:]),
              'poses': [{'frame_id': frame.id, 'T_cw': frame.pose.tolist()} for frame in tracker.map.frames],
              'K': k.tolist(), 'processed_size': [width, height], 'source_fps': fps}
    write_json(output / 'report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New output directory')
    parser.add_argument('--video', type=Path, default=ROOT / 'sample_videos/GRMN2734.MP4')
    args = parser.parse_args(argv)
    if args.output.exists() or not args.video.is_file():
        parser.error('Choose a new output directory and an existing video')
    args.output.mkdir(parents=True)
    with args.video.open('rb') as handle:
        digest = hashlib.file_digest(handle, 'sha256').hexdigest()
    sources = [*ROOT.glob('*.py'), Path(__file__)]
    write_json(args.output / 'manifest.json', {'video': str(args.video.resolve()), 'video_sha256': digest,
        'scenario': 'source 0..119; 45 black inputs; source 30..59; monotonic input clock',
        'seed': 0, 'threads': 1, 'robust_pnp': False,
        'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in sorted(sources)},
        'qualification': 'controlled recovery with recorded pixels, not natural-route accuracy'})
    summary = {}
    for label, enabled in [('disabled', False), ('enabled', True)]:
        target = args.output / label
        target.mkdir()
        report = replay(args.video, target, enabled)
        summary[label] = {key: report[key] for key in
                          ('accepted_poses', 'revisit_tracked_frames', 'recovered_frame_ids', 'elapsed_seconds')}
    write_json(args.output / 'summary.json', summary)
    print(summary)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
