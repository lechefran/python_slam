#!usr/bin/env python3

import cv2

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
