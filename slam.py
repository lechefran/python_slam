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
import OpenGL.GL as gl
from multiprocessing import Process, Queue

# intrinsic matrix
W, H, F = 1920//2, 1080//2, 270
K = np.array([[F, 0, W//2], [0, F, H//2], [0, 0, 1]])

# display and extractor objects
disp = Display2D("Display Window", W, H)

# class for the Map object 
class Map(object):
    def __init__(self):
        self.frames = []
        self.points = []
        self.state = None
        self.queue = Queue()

        process = Process(target = self.viewer_thread, args = (self.queue,))
        process.daemon = True
        process.start()

    # Map display thread: keep updating display while queue is not empty
    def viewer_thread(self, queue):
        self.viewer_init(1024, 768)
        while 1:
            self.viewer_refresh(queue)

    # class method to initialize map viewer 
    def viewer_init(self, W, H):
        pangolin.CreateWindowAndBind('Map View', 640, 480)
        gl.glEnable(gl.GL_DEPTH_TEST)

        self.scam = pangolin.OpenGlRenderState(
                pangolin.ProjectionMatrix(W, H, 420, 420, W//2, H//2, 0.2, 1000),
                pangolin.ModelViewLookAt(0, -10, -8, 0, 0, 0, 0, -1, 0))
        self.handler = pangolin.Handler3D(self.scam)

        # create the interactive window 
        self.dcam = pangolin.CreateDisplay()
        self.dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -W/H)
        self.dcam.SetHandler(self.handler)

    # class method to refresh the viewer based on the contents of the queue
    def viewer_refresh(self, queue):
        if self.state is None or not queue.empty():
            self.state = queue.get()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(1.0, 1.0, 1.0, 1.0)
        self.dcam.Activate(self.scam)

        gl.glPointSize(10)
        gl.glColor3f(0.0, 1.0, 0.0)        
        pangolin.DrawCameras(self.state[0])

        gl.glPointSize(2)
        gl.glColor3f(0.0, 1.0, 0.0)
        pangolin.DrawPoints(self.state[1])

        pangolin.FinishFrame()

    # class method to display map 
    def display(self):
        poses, pts = [], []
        # include map frame poses into poses list
        for frame in self.frames:
            poses.append(frame.pose)

        # include map points int pts list
        for p in self.points:
            pts.append(p.point)

        self.queue.put((np.array(poses), np.array(pts)))

map3d = Map() # map object

# class for 3D points in an image frame 
class Point(object):
    # class constructor
    def __init__(self, img_map, location):
        self.point = location
        self.frames = []
        self.idx = []
        self.id = len(img_map.points)
        img_map.points.append(self)

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
    frame = Frame(map3d, img, K)
    if frame.id == 0:
        return

    frame1 = map3d.frames[-1]
    frame2 = map3d.frames[-2]
    idx1, idx2, rt = match(frame1, frame2)
    frame1.pose = np.dot(rt, frame2.pose)

    # homogenous 3D coordinates 
    pts3d = triangulate_point(frame1.pose, frame2.pose, frame1.pts[idx1], frame2.pts[idx2])
    pts3d /= pts3d[:, 3:]

    # ignore all points tehcnically considered to be behind the camera
    good_pts3d = (np.abs(pts3d[:, 3]) > 0.005) & (pts3d[:, 2] > 0)

    # loop to create 3D points using points obtained from the image frames
    for i, p in enumerate(pts3d):
        if not good_pts3d[i]: # if point is not "good"
            continue
        # print("new good point created")
        pt = Point(map3d, p)
        pt.add_observation(frame1, idx1[i])
        pt.add_observation(frame2, idx2[i])

    # for pt1, pt2 in ret_val:
    for pt1, pt2 in zip(frame1.pts[idx1], frame2.pts[idx2]):
        u1, u2 = denormalize(K, pt1)
        v1, v2 = denormalize(K, pt2)
        cv2.circle(img, (u1, u2), color = (0, 255, 0), radius = 2)
        cv2.line(img, (u1, u2), (v1, v2), color = (255, 0, 255))

    disp.paint(img) # 2D display
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

    video = cv2.VideoCapture(sys.argv[2]) # read in a mp4 file
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
