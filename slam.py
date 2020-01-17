#!/usr/bin/env python3

import cv2
import time
import numpy as np
from display import Display
from frame import Frame, denormalize, match
import g2o # requires user to install additional requirements from readme
import pangolin # also requires user to install additional requirements from readme

# intrinsic matrix
W, H, F = 1920 // 2, 1080 // 2, 270
K = np.array([[F, 0, W // 2], [0, F, H // 2], [0, 0, 1]])

# display and extractor objects
disp = Display("Display Window", W, H)
frames = []

# function to process the image frame from the video: track and draw on
# obtained features and display back including matches
def process_frame(img):
    img = cv2.resize(img, (W, H))
    frame = Frame(img, K)
    frames.append(frame)

    if len(frames) <= 1: # make sure images actually exist
        return

    ret_val, rt = match(frames[-1], frames[-2])

    for pt1, pt2 in ret_val:
        u1, u2 = denormalize(K, pt1)
        v1, v2 = denormalize(K, pt2)
        cv2.circle(img, (u1, u2), color = (0, 255, 0), radius = 1)
        cv2.line(img, (u1, u2), (v1, v2), color = (255, 0, 255))

    disp.paint(img)

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
