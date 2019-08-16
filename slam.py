#!/usr/bin/env python3

import cv2
import time
import numpy as np
from display import Display
from fextractor import FeatureExtractor

W = 1920 // 2
H = 1080 // 2

# intrinsic matrix
F = 200
K = np.array([[F, 0, W // 2], [0, F, H // 2], [0, 0, 1]])

# display and extractor objects
disp = Display("Display Window", W, H)
feature_extractor = FeatureExtractor(K)

def process_frame(img):
    img = cv2.resize(img, (W, H))
    matches, pose = feature_extractor.extract(img)
    if matches is None:
        return

    if pose is None:
        return

    for p1, p2 in matches:
        # u1, v1 = map(lambda x: int(round(x)), p1)
        # u2, v2 = map(lambda x: int(round(x)),p2)
        # u,v = map(lambda x: int(round(x)), p[0])
        u1, v1 = feature_extractor.denormalize(p1, img)
        u2, v2 = feature_extractor.denormalize(p2, img)

        cv2.circle(img, (u1, v1), color = (0, 255, 0), radius = 2)
        cv2.line(img, (u1, v2), (u2, v2), color = (255, 0, 0))

    # cv2.imshow('image', img)
    disp.paint(img)
    # print(img)
    print(f"{len(matches)} matches") # print the number of matches

def main():
    video = cv2.VideoCapture("video 1.mp4") # read in a mp4 file called video 1
    if video.isOpened() == False:
        print("Error, video file could not be loaded")

    while (video.isOpened()):
        ret, frame = video.read()
        if ret == True:
            process_frame(frame)
        else:
            break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
