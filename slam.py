#!/usr/bin/env python3

# import system specific parameters and functions library to link g2o and
# other additional files to this program
import sys

# lib directory in build path contains properly build g2o and pangolin
sys.path.append("./build/lib")

import cv2
import time
import numpy as np
from display import Display2D
from frame import Frame, denormalize, match, irt
import g2o # requires user to install additional requirements from readme
import pangolin

# intrinsic matrix
W, H, F = 1920 // 2, 1080 // 2, 270
K = np.array([[F, 0, W // 2], [0, F, H // 2], [0, 0, 1]])

# display and extractor objects
disp = Display2D("Display Window", W, H)
frames = []

# class for 3D points in an image frame 
class Point(object):
    # class constructor
    def __init__(self, location):
        self.location = location
        self.frames = []
        self.idx = []

    # class method to add a frame and index from video
    # feed to the Point object
    def add_observation(self, frame, index):
        self.frames.append(frame)
        self.idx.append(index)


# function to triangulate a 2D point into 3D space 
def triangulate_point(pose1, pose2, pts1, pts2):
    return cv2.triangulatePoints(pose1[:3], pose2[:3], pts1.T, pts2.T).T

# function to process the image frame from the video: track and draw on
# obtained features and display back including matches
def process_frame(img):
    img = cv2.resize(img, (W, H))
    frame = Frame(img, K)
    frames.append(frame)

    if len(frames) <= 1: # make sure images actually exist
        return

    # ret_val, rt = match(frames[-1], frames[-2])
    idx1, idx2, rt = match(frames[-1], frames[-2])
    frames[-1].pose = np.dot(rt, frames[-2].pose)

    # homogenous 3D coordinates 
    pts3d = triangulate_point(frames[-1].pose, frames[-2].pose, frames[-1].pts[idx1], frames[-2].pts[idx2])
    pts3d /= pts3d[:, 3:]

    # ignore all points tehcnically considered to be behind the camera
    good_pts3d = (np.abs(pts3d[:, 3]) > 0.005) & (pts3d[:, 2] > 0)

    # loop to create 3D points using points obtained from the image frames
    for i, p in enumerate(pts3d):
        if not good_pts3d[i]: # if point is not "good"
            continue
        pt = Point(p)
        pt.add_observation(frames[-1], idx1[i])
        pt.add_observation(frames[-2], idx2[i])

    # for pt1, pt2 in ret_val:
    for pt1, pt2 in zip(frames[-1].pts[idx1], frames[-2].pts[idx2]):
        u1, u2 = denormalize(K, pt1)
        v1, v2 = denormalize(K, pt2)
        cv2.circle(img, (u1, u2), color = (0, 255, 0), radius = 2)
        cv2.line(img, (u1, u2), (v1, v2), color = (255, 0, 255))

    disp.paint(img)

def main():
    # check that user has provided video as program parameter
    if len(sys.argv) < 2:
        print("Error: Please provide a video file as a parameter\nexit(-1)")
        exit(-1)

    video = cv2.VideoCapture(sys.argv[1]) # read in a mp4 file
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
