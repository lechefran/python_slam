"""Classical ORB frontend with explicit failure and world-to-camera contracts."""

import cv2
import numpy as np
from scipy.spatial import cKDTree

from geometry import add_one, denormalize, normalize, pose_rt, valid_pose


class TrackingError(RuntimeError):
    """Insufficient or invalid visual evidence; the map must remain unchanged."""


class Frame:
    def __init__(self, img_map, img, k, frame_id=None, timestamp=0.0, detector=None, mask=None):
        """Prepare features without registering an unvalidated camera in the map."""
        self.k = np.asarray(k, dtype=np.float64)
        self.h, self.w = img.shape[:2]
        self.kinv = np.linalg.inv(self.k)
        self._kps, self.des = extract(img, detector, mask)
        self.kps = normalize(self.kinv, self._kps)
        self.pts = [None] * len(self.kps)
        self.pose = np.eye(4)
        self.id = img_map.next_frame_id if frame_id is None else frame_id
        self.timestamp = timestamp
        self.kd = cKDTree(self._kps)


def extract(img, detector=None, mask=None):
    """Return pixel positions (N,2) and aligned uint8 ORB descriptors (N,32)."""
    if img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
        raise ValueError('Expected a uint8 BGR image')
    detector = detector if detector is not None else cv2.ORB_create(nfeatures=2000)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Let ORB retain its orientation and pyramid metadata; descriptor computation
    # on manually constructed corners does not supply the full detector pipeline.
    keypoints, descriptors = detector.detectAndCompute(gray, mask)
    if descriptors is None or not keypoints:
        return np.empty((0, 2)), np.empty((0, 32), dtype=np.uint8)
    pixels = np.array([keypoint.pt for keypoint in keypoints], dtype=np.float64)
    return pixels, descriptors


def match_features(current, previous):
    """Return unique current/previous feature indices after binary descriptor gates."""
    if len(current.des) == 0 or len(previous.des) < 2:
        return np.empty(0, dtype=int), np.empty(0, dtype=int)
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(current.des, previous.des, k=2)
    candidates = [pair[0] for pair in pairs if len(pair) == 2
                  and pair[0].distance < 0.75 * pair[1].distance and pair[0].distance < 64]
    # Resolve competing matches by descriptor evidence, not incidental feature order.
    candidates.sort(key=lambda item: (item.distance, item.queryIdx, item.trainIdx))
    used = set()
    accepted = []
    for item in candidates:
        if item.trainIdx not in used:
            used.add(item.trainIdx)
            accepted.append((item.queryIdx, item.trainIdx))
    indices = np.array(accepted, dtype=int).reshape(-1, 2)
    return indices[:, 0], indices[:, 1]


def recover_relative(previous_rays, current_rays, focal, threshold_px=1.0):
    """Estimate T_current_previous from normalized (N,2) rays, up to scale.

    Robust fitting and cheirality masks refer to the same input correspondence rows.
    """
    if len(previous_rays) < 8:
        raise TrackingError('fewer than eight two-view correspondences')
    essential, mask = cv2.findEssentialMat(
        previous_rays, current_rays, np.eye(3), method=cv2.RANSAC,
        prob=0.999, threshold=threshold_px / focal)
    if essential is None or mask is None or not np.isfinite(essential).all():
        raise TrackingError('essential estimation failed')
    best = None
    # OpenCV can return multiple essential candidates. Select physical visibility,
    # never a trace heuristic or a manual translation sign chosen from the video.
    for candidate in essential.reshape(-1, 3, 3):
        count, rotation, translation, physical = cv2.recoverPose(
            candidate, previous_rays, current_rays, np.eye(3), mask=mask.copy())
        pose = pose_rt(rotation, translation)
        if valid_pose(pose) and (best is None or count > best[0]):
            best = count, pose, physical.ravel().astype(bool)
    if best is None or best[0] < 12:
        raise TrackingError('insufficient positive-depth support for initialization')
    return best[1], best[2]


def match(current, previous):
    """Compatibility helper returning current/previous indices and relative pose."""
    i, j = match_features(current, previous)
    pose, good = recover_relative(previous.kps[j], current.kps[i], min(current.k[0, 0], current.k[1, 1]))
    return i[good], j[good], pose


def estimate_pose(frame, points, indices, max_error=3.0, require_coverage=True):
    """Robust 3D-to-2D pose in map scale; return T_cw and accepted input rows."""
    xyz = np.asarray([point.point for point in points], dtype=np.float64).reshape(-1, 3)
    pixels = np.asarray(frame._kps[indices], dtype=np.float64)
    if len(xyz) < 12:
        raise TrackingError('fewer than twelve map correspondences')
    ok, rotation, translation, inliers = cv2.solvePnPRansac(
        xyz, pixels, frame.k, None, iterationsCount=200, reprojectionError=max_error,
        confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < 12:
        raise TrackingError('insufficient PnP inliers')
    selected = inliers.ravel()
    # Refine only robust inliers in pixel units. PnP returns world-to-camera,
    # which already agrees with the internal pose convention.
    rotation, translation = cv2.solvePnPRefineLM(xyz[selected], pixels[selected], frame.k, None, rotation, translation)
    pose = pose_rt(cv2.Rodrigues(rotation)[0], translation)
    from geometry import project
    projected, _, visible = project(frame.k, pose, xyz)
    good = visible & (np.linalg.norm(projected - pixels, axis=1) <= max_error)
    selected = np.flatnonzero(good)
    if not valid_pose(pose) or len(selected) < 12:
        raise TrackingError('invalid refined PnP pose')
    # Matches confined to a tiny patch cannot reliably constrain the full camera.
    spans = np.ptp(pixels[selected], axis=0)
    if require_coverage and (spans[0] < 0.1 * frame.w or spans[1] < 0.1 * frame.h):
        raise TrackingError('map inliers have insufficient image coverage')
    return pose, selected
