#!/usr/bin/env python3

# import system specific parameters and functions library to link g2o and
# other additional files to this program
import sys
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

map3d = Map() # 3d map object
disp = None

# function to triangulate a 2D point into 3D space
def triangulate(pose1, pose2, pts1, pts2):
    ret_val = np.zeros((pts1.shape[0], 4))

    for i, j in enumerate(zip(pts1, pts2)):
        temp = np.zeros((4, 4))
        temp[0] = j[0][0] * pose1[2] - pose1[0]
        temp[1] = j[0][1] * pose1[2] - pose1[1]
        temp[2] = j[1][0] * pose1[2] - pose1[0]
        temp[3] = j[1][1] * pose1[2] - pose1[1]
        _, _, vt = np.linalg.svd(temp)
        ret_val[i] = vt[3]

    return ret_val

def hamming_distance(a, b):
    r = (1 << np.arange(8))[:, None]
    return np.count_nonzero((np.bitwise_xor(a, b) & r) != 0)

# function to process the image frame from the video: track and draw on
# obtained features and display back including matches
def process_frame(img):
    start_time = time.time()
    img = cv2.resize(img, (W, H))
    frame = Frame(map3d, img, K)
    if frame.id == 0:
        return

     # print("\n*** frame %d ***" % (frame.id))

    frame1 = map3d.frames[-1]
    frame2 = map3d.frames[-2]
    idx1, idx2, rt = match(frame1, frame2)

    if frame.id < 5:
        frame1.pose = np.dot(rt, frame2.pose)
    else:
        velocity = np.dot(frame2.pose, np.linalg.inv(map3d.frames[-3].pose))
        frame1.pose = np.dot(velocity, frame2.pose)

    for i, idx in enumerate(idx2):
        if frame2.pts[idx] is not None:
            frame2.pts[idx].add_observation(frame1, idx1[i])

    # pose optimization
    pose_optimizer = map3d.optimize(local_window=1, fix_points=True)
    # print("Pose: %f" % pose_optimizer)

    # projection search
    projection_pts_count = 0
    if len(map3d.points) > 0:
        map_points = np.array([p.homogenous() for p in map3d.points])
        projs = np.dot(np.dot(K, frame1.pose[:3]), map_points.T).T
        projs = projs[:, 0:2]/projs[:, 2:]

        good_pts = (projs[:, 0] > 0) & (projs[:,0] < W) & (projs[:, 1] > 0) & (projs[:, 1] < H)

        for i, p in enumerate(map3d.points):
            if not good_pts[i]:
                continue;

            queue = frame1.kd.query_ball_point(projs[i], 5)
            for q in queue:
                if frame1.pts[q] is None:
                    o_dist = hamming_distance(p.orb(), frame1.des[q])
                    if o_dist < 32.0:
                        p.add_observation(frame1, q)
                        projection_pts_count += 1

    good_pts3d = np.array([frame1.pts[i] is None for i in idx1])
    pts3d = triangulate(frame1.pose, frame2.pose, frame1.kps[idx1], frame2.kps[idx2])
    good_pts3d &= np.abs(pts3d[:, 3]) > 0.005

    # homogeneous 3-D coordinates
    pts3d /= pts3d[:, 3:]

    # should reject points hehind the camera
    pts_tri_local = np.dot(frame1.pose, pts3d.T).T
    good_pts3d &= pts_tri_local[:, 2] > 0

    print("Adding: %d points" % np.sum(good_pts3d))

    # loop to create 3D points using points obtained from the image frames
    for i, p in enumerate(pts3d):
        if not good_pts3d[i]:
            continue

        u, v = int(round(frame1._kps[idx1[i], 0])), int(round(frame1._kps[idx1[i], 1]))
        pt = Point(map3d, p[0:3], img[v, u])
        pt.add_observation(frame1, idx1[i])
        pt.add_observation(frame2, idx2[i])

    # for pt1, pt2 in ret_val:
    for i1, i2 in zip(frame1.kps[idx1], frame2.kps[idx2]):
        pt1 = frame1.kps[i1]
        pt2 = frame2.kps[i2]
        u1, u2 = denormalize(K, pt1)
        v1, v2 = denormalize(K, pt2)

        # create circles, improve coloring
        if frame1.pts[i1] is not None:
            if len(frame1.pts[i1]) >= 5:
                cv2.circle(img, (u1, u2), color = (0, 255, 0), radius = 3)
            else:
                cv2.circle(img, (u1, u2), color = (0, 128, 0), radius = 3)
        else:
            cv2.circle(img, (u1, u2), color = (0, 0, 0), radius = 3)
        cv2.line(img, (u1, u2), (v1, v2), color = (255, 0, 255))

    if disp is not None:
        disp.paint(img) # 2D display

    # 3D map optimization
    if frame.id >= 4 and frame.id % 5 == 0:
        error = map3d.optimize()
        print("Optimize: %f units of error" % error)

    map3d.display() # 3D display
    end_time = time.time()
    print("Time: %.2f ms" % ((end_time - start_time) * 1000.0))
    print("Map: %d points, %d frames" % (len(map3d.points), len(map3d.frames)))

if __name__ == "__main__":
    debug_parameter = False # check if there was a debug system parameter

    # check that user has provided video as program parameter
    if len(sys.argv) < 2:
        print("Error: Please provide a video file as a parameter\nexit(-1)")
        exit(-1)

    map3d.create_viewer()
    video = cv2.VideoCapture(sys.argv[1]) # read in a mp4 file
    if video.isOpened() == False:
        print("Error, video file could not be loaded")

    # camera parameters
    W = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    F = float(os.getenv("F", "525"))
    K = np.array([[F, 0, W//2], [0, F, H//2], [0, 0, 1]])
    Kinv = np.linalg.inv(K)

    map3d.create_viewer()

    if W > 1024:
        downscale = 1024.0/W
        F *= downscale
        H = int(H * downscale)
        W = 1024
        print("using camera %dx%d with F %f" % (W, H, F))

    disp = Display2D("Display Window", W, H) # 2d display window

    while (video.isOpened()):
        ret, frame = video.read()
        if ret == True:
            process_frame(frame)
        else:
            break

    video.release()
    cv2.destroyAllWindows()
