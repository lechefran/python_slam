#!/usr/bin/env python3

# import system specific parameters and functions library to link g2o and
# other additional files to this program
import sys

# lib directory in build path contains properly build g2o and pangolin
sys.path.append("./build/lib")

import os
import cv2
import time
import numpy as np
from display import Display2D
from frame import Frame, denormalize, match
from dmap import Map
from point import Point
import g2o # requires user to install additional requirements from readme
from multiprocessing import Process, Queue

# intrinsic matrix
W, H, F = 1920//2, 1080//2, 800
K = np.array([[F, 0, W//2], [0, F, H//2], [0, 0, 1]])

map3d = Map() # 3d map object
map3d.create_viewer()
disp = Display2D("Display Window", W, H) # 2d display window

# function to triangulate a 2D point into 3D space 
def triangulate_point(pose1, pose2, pts1, pts2):
    ret_val = np.zeros((pts1.shape[0], 4))
    pose1 = np.linalg.inv(pose1)
    pose2 = np.linalg.inv(pose2)

    for i, j in enumerate(zip(pts1, pts2)):
        temp = np.zeros((4, 4))
        temp[0] = j[0][0] * pose1[2] - pose1[0]
        temp[1] = j[0][1] * pose1[2] - pose1[1]
        temp[2] = j[1][0] * pose1[2] - pose1[0]
        temp[3] = j[1][1] * pose1[2] - pose1[1]
        _, _, vt = np.linalg.svd(temp)
        ret_val[i] = vt[3]

    return ret_val

# function to process the image frame from the video: track and draw on
# obtained features and display back including matches
def process_frame(img):
    img = cv2.resize(img, (W, H))
    frame = Frame(map3d, img, K)
    if frame.id == 0:
        return

    frame1 = map3d.frames[-1]
    frame2 = map3d.frames[-2]
    idx1, idx2, rt = match(frame1, frame2)
    frame1.pose = np.dot(rt, frame2.pose)

    for i, idx in enumerate(idx2):
        if frame2.pts[idx] is not None:
            frame2.pts[idx].add_observation(frame1, idx1[i])

    # homogenous 3D coordinates 
    # pts3d = triangulate_point(frame1.pose, frame2.pose, frame1.kps[idx1], frame2.kps[idx2])
    # pts3d /= pts3d[:, 3:]
    good_pts3d = np.array([frame1.pts[i] is None for i in idx1])

    # points locally in front of the camera
    pts_tri_local = triangulate_point(rt, np.eye(4), frame1.kps[idx1], frame2.kps[idx2])
    good_pts3d &= np.abs(pts_tri_local[:, 3]) > 0.005

    # ignore all points tehcnically considered to be behind the camera
    # unmatched_pts = np.array([frame1.pts[i] is None for i in idx1])
    # print("Adding %d points" % np.sum(unmatched_pts))
    # good_pts3d = (np.abs(pts3d[:, 3]) > 0.005) & (pts3d[:, 2] > 0) & unmatched_pts
    pts_tri_local /= pts_tri_local[:, 3:]
    good_pts3d &= pts_tri_local[:, 2] > 0

    # project to map view
    pts3d = np.dot(np.linalg.inv(frame1.pose), pts_tri_local.T).T
    print("Adding: %d points" % np.sum(good_pts3d))

    # loop to create 3D points using points obtained from the image frames
    for i, p in enumerate(pts3d):
        if not good_pts3d[i]: # if point is not "good"
            continue

        u, v = int(round(frame1._kps[idx1[i], 0])), int(round(frame1._kps[idx1[i], 1]))
        pt = Point(map3d, p, img[v, u])
        pt.add_observation(frame1, idx1[i])
        pt.add_observation(frame2, idx2[i])

    # for pt1, pt2 in ret_val:
    for pt1, pt2 in zip(frame1.kps[idx1], frame2.kps[idx2]):
        u1, u2 = denormalize(K, pt1)
        v1, v2 = denormalize(K, pt2)
        cv2.circle(img, (u1, u2), color = (0, 255, 0), radius = 2)
        cv2.line(img, (u1, u2), (v1, v2), color = (255, 0, 255))

    disp.paint(img) # 2D display

    # 3D map optimization
    if frame.id >= 4:
        error = map3d.PointMapOptimize()
        print("Optimize: %f units of error" % error)

    map3d.display() # 3D display

def main():
    debug_parameter = False # check if there was a debug system parameter

    # check that user has provided video as program parameter
    if len(sys.argv) < 3:
        print("Error: Please provide a video file as a parameter\nexit(-1)")
        exit(-1)
   
    if sys.argv[1] == "-t":
        sys.argv[1] = True
    elif sys.argv[1] == "-f":
        sys.argv[1] = False
    else:
        print("Unexpected Flag Error: Please provide a proper flag argument")
        exit(-1)

    video = cv2.VideoCapture(sys.argv[2]) # read in a mp4 file
    if video.isOpened() == False:
        print("Error, video file could not be loaded")

    if os.getenv("D3D") is not None:
        map3d.create_viewer()
    if os.getenv("D2D") is not None:
        disp = Display2D("Display Window", W, H) # 2d display window

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
