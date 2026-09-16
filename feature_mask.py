"""Static feature-centre exclusions in source and processed image coordinates."""

import hashlib
from pathlib import Path

import cv2
import numpy as np


def prepare_feature_mask(path, source_size, processed_size, maps=None):
    """Return a processed uint8 allow-mask and provenance; sizes are (width,height).

    Zero excludes a feature centre; 255 permits it. Propagate allowed support
    through the same area resize and linear rectification as the image. Any
    pixel mixing excluded source pixels or out-of-image border is excluded.
    This masks centres, not the full ORB descriptor patch at every pyramid level.
    """
    source = None
    digest = None
    if path is not None:
        raw = Path(path).read_bytes()
        if not raw.startswith(b'\x89PNG\r\n\x1a\n'):
            raise ValueError('--feature-mask must be a grayscale binary PNG')
        source = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
        if (source is None or source.dtype != np.uint8 or source.ndim != 2
                or not np.isin(source, [0, 255]).all()):
            raise ValueError('Feature mask must contain only 0 and 255 in one uint8 channel (no alpha)')
        if source.shape != (source_size[1], source_size[0]):
            raise ValueError('Feature mask dimensions must match the decoded source video')
        digest = hashlib.sha256(raw).hexdigest()
    mask = None
    if source is not None or maps is not None:
        if source is None:
            allowed = np.ones((processed_size[1], processed_size[0]), np.float32)
        else:
            allowed = cv2.resize(source.astype(np.float32) / 255., processed_size,
                                 interpolation=cv2.INTER_AREA)
        if maps is not None:
            allowed = cv2.remap(allowed, *maps, cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        mask = np.where(allowed >= 1.0 - 1e-6, 255, 0).astype(np.uint8)
    return mask, {'source_path': str(Path(path).resolve()) if path else None,
                  'source_sha256': digest, 'source_size': list(source_size),
                  'processed_size': list(processed_size), 'rectified': maps is not None,
                  'convention': '0 excludes feature centres; 255 allows',
                  'transform': 'INTER_AREA resize; INTER_LINEAR remap; require full allowed support'}


def tint_exclusions(image, mask):
    """Return a BGR display copy, tinting excluded pixels orange without mutation."""
    if mask is None:
        return image
    result = image.copy()
    excluded = mask == 0
    result[excluded] = (.6 * image[excluded] + .4 * np.array([0, 140, 255])).astype(np.uint8)
    return result
