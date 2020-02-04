import numpy as np
from multiprocessing import Process, Queue
import g2o # requires user to install additional requirements from readme
import pangolin
import OpenGL.GL as gl
import sys

# lib directory in build path contains properly build g2o and pangolin
sys.path.append("./build/lib")

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

