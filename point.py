import numpy as np

# class for 3D points in an image frame
class Point(object):
    # class constructor
    def __init__(self, img_map, location, color):
        self.point = location
        self.frames = []
        self.idx = []
        self.color = np.copy(color)

        self.id = img_map.max_point
        img_map.max_point += 1
        img_map.points.append(self)

    def orb(self):
        des = []
        for f in self.frames:
            des.append(f.des[f.pts.index(self)])
        return des

    # class method to add a frame and index from video
    # feed to the Point object
    def add_observation(self, frame, index):
        frame.pts[index] = self
        self.frames.append(frame)
        self.idx.append(index)

    # class method to delete a point from a frame
    def delete_point(self):
        for f in self.frames:
            f.pts[f.pts.index(self)] = None
        del self

    def homogenous(self):
        return np.array([self.point[0], self.point[1], self.point[2], 1.0])
