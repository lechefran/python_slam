"""Optional main-thread Matplotlib viewer; importing SLAM never loads this GUI."""

import time
import numpy as np


class Viewer:
    def __init__(self):
        import matplotlib
        import matplotlib.pyplot as plt
        if matplotlib.get_backend().lower() in ('agg', 'pdf', 'svg', 'ps', 'template', 'cairo'):
            raise RuntimeError('No interactive Matplotlib backend; use --headless or configure a desktop backend')
        self.plt = plt
        self.figure = plt.figure(figsize=(12, 6))
        self.image_axes = self.figure.add_subplot(121)
        self.map_axes = self.figure.add_subplot(122, projection='3d')
        self.image_artist = None
        self.last_draw = 0.0
        self.paused = False
        self.figure.canvas.mpl_connect('key_press_event', self.on_key)
        plt.show(block=False)

    def on_key(self, event):
        if event.key in ('escape', 'q'):
            self.close()
        elif event.key == ' ':
            self.paused = not self.paused

    @property
    def open(self):
        return self.plt.fignum_exists(self.figure.number)

    def update(self, image, frame, map3d, status, force=False):
        """Draw matching image/map snapshots; viewer coordinates never change poses."""
        if not self.open:
            return False
        now = time.perf_counter()
        if force or now - self.last_draw > 0.1:
            self.last_draw = now
            rgb = image[:, :, ::-1]
            if self.image_artist is None:
                self.image_artist = self.image_axes.imshow(rgb)
                self.image_axes.axis('off')
            else:
                self.image_artist.set_data(rgb)
            self.image_axes.set_title(f'Frame {frame.id}: {status}\nSpace: pause | Q/Esc: close')
            axes = self.map_axes
            axes.clear()
            if map3d.frames:
                # Camera position is the translation of T_wc, not of T_cw.
                centres = np.array([-f.pose[:3, :3].T @ f.pose[:3, 3] for f in map3d.frames])
                cuts = np.flatnonzero(np.diff([f.id for f in map3d.frames]) > 1) + 1
                for segment in np.split(centres, cuts):
                    axes.plot(*segment.T, color='tab:blue')
                axes.scatter(*centres[-1], color='orange', s=35)
            if map3d.points:
                stride = max(1, len(map3d.points) // 3000)
                points = map3d.points[::stride]
                xyz = np.array([p.point for p in points])
                axes.scatter(*xyz.T, c=np.array([p.color for p in points]) / 255.0, s=2)
            axes.set(xlabel='World X', ylabel='World Y', zlabel='World Z', title='Sparse 3D map — arbitrary scale')
            limits = np.array([axes.get_xlim(), axes.get_ylim(), axes.get_zlim()])
            centres = limits.mean(axis=1)
            radius = max(float(np.ptp(limits, axis=1).max()) / 2, 1)
            axes.set_xlim(centres[0] - radius, centres[0] + radius)
            axes.set_ylim(centres[1] - radius, centres[1] + radius)
            axes.set_zlim(centres[2] - radius, centres[2] + radius)
            axes.set_box_aspect((1, 1, 1))
            self.figure.canvas.draw_idle()
        self.plt.pause(0.001)
        while self.paused and self.open:
            self.plt.pause(0.05)
        return self.open

    def close(self):
        self.plt.close(self.figure)

    def hold(self):
        if self.open:
            self.plt.show(block=True)
