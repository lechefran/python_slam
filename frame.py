import cv2
import numpy as np
from scipy.spatial import KDTree
from skimage.measure import ransac
from skimage.transform import EssentialMatrixTransform
from skimage.transform import FundamentalMatrixTransform
import sys
import os

np.set_printoptions(suppress = True)

# class for the feature extractor using cv2 orbs
class Frame(object):
    def __init__(self, img_map, img, k):
        self.k = k
        self.w, self.h = img.shape[0:2]
        self.kinv = np.linalg.inv(self.k)
        self._kps, self.des = extract(img)
        self.kps = normalize(self.kinv, self._kps)
        self.pts = [None]*len(self.kps)
        self.pose = np.eye(4)
        self.kd = KDTree(self._kps)
        self.id = len(img_map.frames)

        img_map.frames.append(self)

def pose_rt(r, t):
    ret_val = np.eye(4)
    ret_val[:3, :3] = r
    ret_val[:3, 3] = t
    return ret_val

# transform [[x, y]] to [[x, y, 1]]
def add_one(x):
    return np.concatenate([x, np.ones((x.shape[0], 1))], axis = 1)

# function to extract features from an image frame
def extract(img):
    # find noteworthy points on grid to extract
    orb = cv2.ORB_create()
    features = cv2.goodFeaturesToTrack(np.mean(img, axis = 2).astype(np.uint8), 3000, qualityLevel = 0.01, minDistance = 7)
    key_points = [cv2.KeyPoint(x = f[0][0], y = f[0][1], _size = 20) for f in features]
    key_points, descriptors = orb.compute(img, key_points)
    return np.array([(kp.pt[0], kp.pt[1]) for kp in key_points]), descriptors

# function to do point matching between current and past frames
def match(frame1, frame2):
    # do point matching
    ret_val, idx1, idx2 = [], [], [] # return values, index 1, index 2
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(frame1.des, frame2.des, k = 2)

    # perform Lowe's Ratio test
    for m, n in matches:
        if m.distance < 0.75*n.distance:
            # append to frame points
            k1 = frame1.kps[m.queryIdx]
            k2 = frame2.kps[m.trainIdx]

            # less than 10% diagonal travel within orb distance
            if np.linalg.norm((k1-k2)) < 0.1*np.linalg.norm([frame1.w, frame1.h]) and m.distance < 32:
                # prevent this from becoming quadratic in runtime
                if m.queryIdx not in idx1 and m.trainIdx not in idx2:
                    idx1.append(m.queryIdx)
                    idx2.append(m.trainIdx)
                    ret_val.append((k1, k2))

    # prevent duplicates
    assert(len(set(idx1)) == len(idx1))
    assert(len(set(idx2)) == len(idx2))

    assert len(ret_val) >= 8
    ret_val = np.array(ret_val)
    idx1 = np.array(idx1)
    idx2 = np.array(idx2)

    # perform matrix fitting
    model, inliers = ransac((ret_val[:, 0], ret_val[:, 1]), FundamentalMatrixTransform,
                            min_samples = 8, residual_threshold = 0.001, max_trials = 100)
    # print(sum(inliers), len(inliers))
    print("Matches: %d -> %d -> %d -> %d" % (len(frame1.des), len(matches), len(inliers), sum(inliers)))
    rt = extractRT(model.params) # show matrix values
    return idx1[inliers], idx2[inliers], rt # return values

# function to denormalize a set of point values
def denormalize(k, pt):
    ret_val = np.dot(k, np.array([pt[0], pt[1], 1.0]))
    ret_val /= ret_val[2]
    return int(round(ret_val[0])), int(round(ret_val[1]))

# function to normalize a set of point values
def normalize(kinv, pts):
    return np.dot(kinv, transform_with_one(pts).T).T[:, 0:2]

def extractRT(parameters):
    W = np.mat([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype = float)
    s, v, d = np.linalg.svd(parameters)
    if np.linalg.det(s) < 0:
        s *= -1.0

    if np.linalg.det(d) < 0:
        d *= -1.0

    R = np.dot(np.dot(s, W), d)
    if np.sum(R.diagonal()) < 0:
        R = np.dot(np.dot(s, W.T), d)
    t = s[:, 2]

    if os.getenv("REVERSE") is not None:
        t *= -1

    return np.linalg.inv(pose_rt(R, t))
