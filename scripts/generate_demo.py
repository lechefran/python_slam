#!/usr/bin/env python3
"""Generate a deterministic textured scene for decode/tracking smoke checks."""

import argparse
from pathlib import Path
import cv2
import numpy as np


def generate(path, frames=30):
    """Render translated cameras viewing static textured patches at varied depths.

    Source size is 640x360, focal length 400 pixels. The analytic camera centres
    move 0.045 scene units per frame; video-only SLAM cannot recover that scale.
    """
    if frames <= 0:
        raise ValueError('frames must be positive')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(42)
    pixels = np.array([(u, v) for v in range(40, 330, 38) for u in range(40, 620, 38)])
    depth = rng.uniform(4, 9, len(pixels))
    points = np.column_stack(((pixels[:, 0] - 320) * depth / 400,
                              (pixels[:, 1] - 180) * depth / 400, depth))
    patches = rng.integers(30, 255, (len(points), 23, 23), dtype=np.uint8)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 360))
    if not writer.isOpened():
        raise RuntimeError('MJPEG video writer is unavailable')
    try:
        for number in range(frames):
            image = np.full((360, 640, 3), 20, dtype=np.uint8)
            # Independent analytic pinhole projection, not production helpers:
            # translate the camera centre right; world points move left in view.
            u = 400 * (points[:, 0] - number * 0.045) / points[:, 2] + 320
            v = 400 * points[:, 1] / points[:, 2] + 180
            for index in np.argsort(-depth):
                x, y = int(round(u[index])), int(round(v[index]))
                if 12 <= x < 628 and 12 <= y < 348:
                    image[y-11:y+12, x-11:x+12] = patches[index, :, :, None]
            writer.write(image)
    finally:
        writer.release()
    return path


if __name__ == '__main__':
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('output', type=Path)
    cli.add_argument('--frames', type=int, default=30)
    args = cli.parse_args()
    generate(args.output, args.frames)
