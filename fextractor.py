#!/usr/bin/env python3

import cv2
import numpy as np
from skimage.measure import ransac
from skimage.transform import EssentialMatrixTransform
from skimage.transform import FundamentalMatrixTransform
np.set_printoptions(suppress = True)

# transform [[x, y]] to [[x, y, 1]]
def transform_with_one(x):
    return np.concatenate([x, np.ones((x.shape[0], 1))], axis = 1)

def extractRT(parameters):
    W = np.mat([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype = float)
    s, v, d = np.linalg.svd(parameters)
    # print(np.linalg.det(s))
    # print(np.linalg.det(d))
    assert np.linalg.det(s) > 0 # assert that value is never less that zero
    if np.linalg.det(d) < 0:
        d *= -1.0

    R = np.dot(np.dot(s, W), d)
    if np.sum(R.diagonal()) < 0:
        R = np.dot(np.dot(s, d.T), d)
    t = s[:, 2]
    RT = np.concatenate([R, t.reshape(3, 1)], axis = 1)
    print(RT)
    return RT
         
# class for the feature extractor using cv2 orbs
class FeatureExtractor(object):
    def __init__(self, k):
        self.k = k
        self.kinv = np.linalg.inv(self.k)
        self.orb = cv2.ORB_create(1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last = None

    def denormalize(self, pt, img):
        # print(self.kinv)
        ret_val = np.dot(self.k, np.array([pt[0], pt[1], 1.0]).T)
        # ret_val /= ret_val[2]
        return int(round(ret_val[0])), int(round(ret_val[1]))

    def normalize(self, pts):
        return np.dot(self.kinv, transform_with_one(pts).T).T[:, 0:2]

    def extract(self, img):
        # find noteworthy points on grid to extract
        features = cv2.goodFeaturesToTrack(np.mean(img, axis = 2).astype(np.uint8), 3000, qualityLevel = 0.01, minDistance = 5)
        key_points = [cv2.KeyPoint(x = f[0][0], y = f[0][1], _size = 20) for f in features]
        key_points, descriptors = self.orb.compute(img, key_points)

        # do point matching
        ret_val = []
        if self.last is not None:
            matches = self.bf.knnMatch(descriptors, self.last['des'], k = 2)
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    k1 = key_points[m.queryIdx].pt
                    k2 = self.last['kps'][m.trainIdx].pt
                    ret_val.append((k1, k2))
        
        # try and filter matches for cleaner point connections on image frame
        rt = None
        if len(ret_val) > 0:
            ret_val = np.array(ret_val)

            # normalize coordinates by finding center of the image frame using subtraction to reach 0
            ret_val[:, 0, :] = self.normalize(ret_val[:, 0, :])
            ret_val[:, 1, :] = self.normalize(ret_val[:, 1, :])

            model, inliers = ransac((ret_val[:, 0], ret_val[:, 1]), EssentialMatrixTransform,
                                    min_samples = 8, residual_threshold = 0.005, max_trials = 200)
            # print(sum(inliers), len(inliers)) 
            ret_val = ret_val[inliers]
            rt = extractRT(model.params)

        self.last = {'kps': key_points, 'des': descriptors}
        return ret_val, rt
