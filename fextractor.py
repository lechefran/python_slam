#!usr/bin/env python3

import cv2
<<<<<<< HEAD
import numpy as np
from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform

# class for the feature extractor using cv2 orbs
class FeatureExtractor(object):
    def __init__(self):
        self.orb = cv2.ORB_create(1000)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.last = None

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
            # matches = zip([key_points[m.queryIdx] for m in matches], [self.last['kps'][m.trainIdx] for m in matches])

        # try and filter matches for cleaner point connections on image frame 
        if len(ret_val) > 0:
            ret_val = np.array(ret_val)
            print(ret_val.shape)

            model, inliers = ransac((ret_val[:, 0], ret_val[:, 1]), FundamentalMatrixTransform,
                                    min_samples = 8, residual_threshold = 0.01, max_trials = 100)
            ret_val = ret_val[inliers] # clean up frame noise

        self.last = {'kps': key_points, 'des': descriptors}
        return ret_val


=======

# class for the feature extractor using cv2 orbs
class FeatureExtractor(object):
    gridX, gridY = 16 // 2, 12 // 2

    def __init__(self):
        self.orb = cv2.ORB_create(1000)
        self.bf = cv2.BFMatcher()
        self.last = None

    def extract(self, img):
        # find noteworthy points on grid to map
        akp = []
        sy = img.shape[0] // self.gridY
        sx = img.shape[1] // self.gridX
        for py in range(0, img.shape[0], sy):
            for px in range(0,img.shape[1], sx):
                img_area = img[py: py + sy, px: px + sx]
                kp = self.orb.detect(img_area, None)
                for p in kp: 
                    p.pt = (p.pt[0] + px, p.pt[1] + py)
                    akp.append(p)
        return akp # return frame with orb points
>>>>>>> 9148e1bd7e6fdce8774f44e3910a5622910c51b5
