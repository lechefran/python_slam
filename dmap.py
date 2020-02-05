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
        self.queue = None

    def create_viewer(self):
        self.queue = Queue()
        self.process = Process(target = self.viewer_thread, args = (self.queue,))
        self.process.daemon = True
        self.process.start()

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
                pangolin.ProjectionMatrix(W, H, 420, 420, W//2, H//2, 0.2, 10000),
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
        # check if queue even exists
        if self.queue is None:
            return

        poses, pts = [], []
        # include map frame poses into poses list
        for frame in self.frames:
            poses.append(frame.pose)

        # include map points int pts list
        for p in self.points:
            pts.append(p.point)

        self.queue.put((np.array(poses), np.array(pts)))

    # g2o graph optimizer for the 3d map
    def PointMapOptimize(self):
        # create the g2o optimizer
        optimizer = g2o.SparseOptimizer()
        graph_solver = g2o.BlockSolverSE3(g2o.LinearSolverCholmodSE3())
        graph_solver = g2o.OptimizationAlgorithmLevenberg(graph_solver)
        optimizer.set_algorithm(graph_solver)
        robust_kernel = g2o.RobustKernelHuber(np.sqrt(5.991))

        # add frames to the graph
        for f in self.frames:
            sbacam = g2o.SBACam(g2o.SE3Quat(f.pose[0:3, 0:3], f.pose[0:3, 3]))
            sbacam.set_cam(f.k[0][0], f.k[1][1], f.k[2][0], f.k[2][1], 1.0)

            v_se3 = g2o.VertexCam()
            v_se3.set_id(f.id)
            v_se3.set_estimate(sbacam)
            v_se3.set_fixed(f.id == 0)
            optimizer.add_vertex(v_se3)

        # add points to the frames
        for p in self.points:
            pt = g2o.VertexSBAPointXYZ()
            pt.set_id(p.id + 0x10000)
            pt.set_estimate(p.point[0:3])
            pt.set_marginalized(True)
            pt.set_fixed(False)
            optimizer.add_vertex(pt)

            for f in p.frames:
                edge = g2o.EdgeProjectP2MC()
                edge.set_vertex(0, pt)
                edge.set_vertex(1, optimizer.vertex(f.id))
                uv = f.kps[f.pts.index(p)]
                edge.set_measurement(uv)
                edge.set_information(np.eye(2))
                edge.set_robust_kernel(robust_kernel)
                optimizer.add_edge(edge)

            optimizer.set_verbose(True)
            optimizer.initialize_optimization()
            optimizer.optimize(20)

