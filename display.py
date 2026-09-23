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
        self.feature_artists = []
        self.feature_legend = None
        self.show_features = True
        self.mask_artist = None
        self.show_mask = True
        self.last_draw = 0.0
        self.paused = False
        self.figure.canvas.mpl_connect('key_press_event', self.on_key)
        plt.show(block=False)

    def on_key(self, event):
        if event.key in ('escape', 'q'):
            self.close()
        elif event.key == ' ':
            self.paused = not self.paused
        elif event.key in ('m', 'M'):
            self.show_mask = not self.show_mask
            if self.mask_artist is not None:
                self.mask_artist.set_visible(self.show_mask)
            self.figure.canvas.draw_idle()
        elif event.key in ('o', 'O'):
            self.show_features = not self.show_features
            for artist in self.feature_artists:
                artist.set_visible(self.show_features)
            if self.feature_legend is not None:
                self.feature_legend.set_visible(self.show_features)
            # Change visibility immediately, including while paused or at EOF.
            self.figure.canvas.draw_idle()

    @property
    def open(self):
        return self.plt.fignum_exists(self.figure.number)

    def update(self, image, frame, map3d, status, force=False, mask=None):
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
                styles = [('#00d5ff', 'ORB feature'), ('#55ff55', 'Mapped ORB')]
                if map3d.landmark_maturity:
                    styles = [('#00d5ff', 'ORB feature'), ('#55ff55', 'Active landmark'),
                              ('#ff70ff', 'Candidate landmark')]
                self.feature_artists = [self.image_axes.scatter(
                    [], [], s=18, facecolors='none', edgecolors=color, linewidths=.7,
                    label=label, visible=self.show_features)
                    for color, label in styles]
                self.feature_legend = self.image_axes.legend(
                    loc='lower left', fontsize=8, facecolor='#111111',
                    labelcolor='white', framealpha=.75)
                self.feature_legend.set_visible(self.show_features)
            else:
                self.image_artist.set_data(rgb)
            if mask is not None:
                # A separate RGBA layer leaves both video pixels and ORB input
                # untouched. Its shape/extent is identical to the processed image.
                tint = np.zeros((*mask.shape, 4), dtype=np.uint8)
                tint[:, :, :3] = [255, 140, 0]
                tint[:, :, 3] = np.where(mask == 0, 100, 0)
                if self.mask_artist is None:
                    self.mask_artist = self.image_axes.imshow(tint, zorder=1)
                else:
                    self.mask_artist.set_data(tint)
                self.mask_artist.set_visible(self.show_mask)
            elif self.mask_artist is not None:
                self.mask_artist.set_data(np.zeros((*image.shape[:2], 4), np.uint8))
            # Use the detector's processed-image (u,v) pixels, never normalized
            # camera rays. Read live associations after culling; rings are a
            # display layer and do not alter the image or tracking inputs.
            mapped = np.array([point is not None and not point.deleted for point in frame.pts], dtype=bool)
            masks = (~mapped, mapped)
            if map3d.landmark_maturity:
                active = np.array([point is not None and point.state == 'active' for point in frame.pts], dtype=bool)
                masks = (~mapped, active, mapped & ~active)
            for artist, selected in zip(self.feature_artists, masks):
                artist.set_offsets(frame._kps[selected])
            self.image_axes.set_title(f'Frame {frame.id}: {status}\nSpace: pause | O: ORB | M: exclusions (orange) | Q: close', fontsize=10)
            axes = self.map_axes
            axes.clear()
            trajectory = map3d.trajectory.accepted
            if trajectory:
                # Camera position is the translation of T_wc, not of T_cw.
                poses = [np.asarray(record.T_cw) for record in trajectory]
                centres = np.array([-pose[:3, :3].T @ pose[:3, 3] for pose in poses])
                cuts = np.flatnonzero(np.diff([record.frame_id for record in trajectory]) > 1) + 1
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
