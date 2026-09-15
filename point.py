"""World-space landmarks with reciprocal, one-per-frame observations."""

import numpy as np


class Point:
    def __init__(self, img_map, location, color):
        self.point = np.asarray(location, dtype=np.float64).copy()
        if self.point.shape != (3,) or not np.isfinite(self.point).all():
            raise ValueError('Landmark must be a finite world-space XYZ point')
        self.frames = []
        self.idx = []
        self.color = np.asarray(color, dtype=np.uint8).copy()  # RGB, not OpenCV BGR.
        self.map = img_map
        self.id = img_map.max_point
        img_map.max_point += 1
        img_map.points.append(self)
        self.deleted = False

    def orb(self):
        return [frame.des[index] for frame, index in zip(self.frames, self.idx)]

    def add_observation(self, frame, index):
        if self.deleted:
            raise ValueError('Cannot observe a deleted landmark')
        if not 0 <= index < len(frame.pts):
            raise IndexError('Observation feature index is out of bounds')
        if frame in self.frames:
            if self.idx[self.frames.index(frame)] == index and frame.pts[index] is self:
                return
            raise ValueError('Landmark already has an observation in this frame')
        if frame.pts[index] is not None:
            raise ValueError('Feature already belongs to another landmark')
        # Validate both sides before committing either link; BA reads this exact
        # stored index rather than searching an ambiguous feature-slot list.
        self.frames.append(frame)
        self.idx.append(int(index))
        frame.pts[index] = self

    def remove_observation(self, frame):
        if frame not in self.frames:
            return
        position = self.frames.index(frame)
        index = self.idx[position]
        if frame.pts[index] is not self:
            raise ValueError('Broken reciprocal observation')
        self.idx.pop(position)
        self.frames.pop(position)
        frame.pts[index] = None

    def delete_point(self):
        for frame in list(self.frames):
            self.remove_observation(frame)
        if self in self.map.points:
            self.map.points.remove(self)
        self.deleted = True

    def homogenous(self):
        return np.r_[self.point, 1.0]
