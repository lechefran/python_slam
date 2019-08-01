#!usr/bin/env python3

import cv2
import time
import numpy as np
from display import Display
from fextractor import FeatureExtractor

W = 1920 // 2
H = 1080 // 2

# display and extractor objects
disp = Display("Display Window", W, H)
feature_extractor = FeatureExtractor()

def process_frame(img):
    img = cv2.resize(img, (W, H))
    kps = feature_extractor.extract(img)
    for p in kps:
        u, v = map(lambda x: int(round(x)), p.pt)
        cv2.circle(img, (u, v), color = (0, 255, 0), radius = 2)

    # cv2.imshow('image', img)
    disp.paint(img)
    print(img)

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
