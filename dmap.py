"""Sparse map and native g2opy bundle adjustment; no display imports."""

from dataclasses import dataclass, asdict
from collections import Counter, deque
import time

import g2opy as g2o
import numpy as np

from geometry import pose_rt, project, valid_pose
from trajectory import Trajectory
from keyframes import MappingKeyframes


@dataclass
class OptimizationResult:
    status: str = 'skipped'
    reason: str = ''
    edges: int = 0
    iterations: int = 0
    before_chi2: float | None = None
    after_chi2: float | None = None
    seconds: float = 0.0

    def to_dict(self):
        # Failed native estimates can contain NaN costs; preserve the failure
        # status while keeping the report valid JSON for independent readers.
        return {key: None if isinstance(value, float) and not np.isfinite(value) else value
                for key, value in asdict(self).items()}


def camera_vertex(frame, identifier, fixed):
    """Convert internal T_cw to SBACam's camera-to-world estimate."""
    pose = np.linalg.inv(frame.pose)
    camera = g2o.SBACam(g2o.SE3Quat(pose[:3, :3], pose[:3, 3]))
    camera.set_cam(frame.k[0, 0], frame.k[1, 1], frame.k[0, 2], frame.k[1, 2], 1.0)
    vertex = g2o.VertexCam()
    vertex.set_id(identifier)
    vertex.set_estimate(camera)
    vertex.set_fixed(fixed)
    return vertex


class Map:
    def __init__(self, landmark_maturity=False, observation_history=True):
        self.frames = []
        self.trajectory = Trajectory()
        self.keyframes = MappingKeyframes(self)
        self.points = []
        self.max_point = 0
        self.next_frame_id = 0
        self.landmark_maturity = landmark_maturity
        self.landmark_counts = {'candidate': 0, 'active': 0, 'outlier': 0, 'retired': 0}
        self.maturity_events = {'promotions': 0, 'demotions': 0}
        self.observation_history = observation_history
        self.quality_events = Counter()
        self.retired_quality = deque(maxlen=64)

    def observation_summary(self):
        if not self.observation_history:
            return None
        accepted = self.quality_events['search/accepted']
        attempts = accepted + self.quality_events['search/rejected'] + self.quality_events['search/unmatched']
        return {'events': dict(self.quality_events), 'assessed_searches': attempts,
                'successful_reobservation_ratio': accepted / attempts if attempts else None,
                'history_capacity_per_landmark': 16, 'retired_sample_capacity': 64,
                'retained_retired_samples': len(self.retired_quality)}

    def maturity_summary(self):
        """Live candidate/active counts; outlier/retired counts are cumulative."""
        return {'enabled': self.landmark_maturity, **self.landmark_counts, **self.maturity_events}

    def add_frame(self, frame):
        if frame in self.frames or any(item.id == frame.id for item in self.frames):
            raise ValueError('Frame is already registered or its ID is duplicated')
        if not valid_pose(frame.pose):
            raise ValueError('Cannot register an invalid camera pose')
        self.trajectory.accept(frame.id, frame.timestamp, frame.pose)
        self.frames.append(frame)
        self.next_frame_id = max(self.next_frame_id, frame.id + 1)

    def check_integrity(self):
        """Validate reciprocal references without relying on disableable assertions."""
        points = set(self.points)
        frames = set(self.frames)
        for point in points:
            if point.point.shape != (3,) or not np.isfinite(point.point).all():
                raise ValueError('Invalid landmark coordinates')
            if point.deleted or point.state not in ('candidate', 'active') or len(point.frames) != len(point.idx) or len(set(point.frames)) != len(point.frames):
                raise ValueError('Invalid landmark observation history')
            for frame, index in zip(point.frames, point.idx):
                if frame not in frames or frame.pts[index] is not point:
                    raise ValueError('Broken landmark-to-frame link')
        for frame in frames:
            if not valid_pose(frame.pose) or not np.isfinite(frame._kps).all():
                raise ValueError('Invalid camera pose or measurements')
            for index, point in enumerate(frame.pts):
                if point is not None and (point not in points or frame not in point.frames
                                         or point.idx[point.frames.index(frame)] != index):
                    raise ValueError('Broken frame-to-landmark link')

    def cull(self, current_id, stale_after=20, max_error=5.0):
        """Retire weak stale points even when absent from the latest BA graph."""
        removed = 0
        observations = {}
        for point in self.points:
            for frame, index in zip(point.frames, point.idx):
                observations.setdefault(frame, []).append((point, index))
        # Batch projection by camera: residual semantics are identical, while
        # avoiding one NumPy matrix allocation for every historical observation.
        for frame, pairs in observations.items():
            pixels, depths, visible = project(frame.k, frame.pose, [p.point for p, _ in pairs])
            measured = frame._kps[[i for _, i in pairs]]
            errors = np.linalg.norm(pixels - measured, axis=1)
            rejected = ~visible | (errors > max_error)
            for (point, index), invalid, error, depth in zip(pairs, rejected, errors, depths):
                # Keep rejected historical checks and the latest live check;
                # replaying all old accepted residuals would swamp new evidence.
                if self.observation_history and (invalid or frame is point.frames[-1]):
                    point.record_quality(current_id, frame.id, 'cull', 'rejected' if invalid else 'retained',
                                         error, depth, index)
                if invalid:
                    point.remove_observation(frame, reason='culled', assessed_frame_id=current_id)
        for point in list(self.points):
            last_seen = max((frame.id for frame in point.frames), default=-1)
            if self.landmark_maturity:
                point.refresh_maturity()
            expired_candidate = (self.landmark_maturity and point.state == 'candidate'
                                 and point.born_frame_id is not None and current_id - point.born_frame_id > 30)
            if len(point.frames) < 2 or expired_candidate or (len(point.frames) <= 2 and current_id - last_seen > stale_after):
                point.delete_point('outlier' if len(point.frames) < 2 else 'stale', assessed_frame_id=current_id)
                removed += 1
        return removed

    def optimize(self, local_window=10, fix_points=False, verbose=False, iterations=10, max_points=600):
        """Refine a bounded local graph; commit only finite, non-worsening states.

        Edges measure rectified pixels with unit pixel information. The first two
        accepted cameras fix the arbitrary monocular frame and baseline scale.
        """
        start = time.perf_counter()
        result = OptimizationResult()
        if len(self.frames) < 2:
            result.reason = 'fewer than two accepted cameras'
            return result
        self.check_integrity()
        local = set(self.frames if local_window is None else self.frames[-local_window:])
        anchors = set(self.frames[:2])
        points = [p for p in self.points if len(p.frames) >= 2 and any(f in local for f in p.frames)
                  and (not self.landmark_maturity or p.state == 'active')]
        points.sort(key=lambda p: (-sum(f in local for f in p.frames), -p.frames[-1].id, p.id))
        points = points[:max_points]
        if not points:
            result.reason = 'no supported local landmarks'
            return result
        observations = {}
        for point in points:
            pairs = list(zip(point.frames, point.idx))
            # Retain local measurements and at most two old boundary observations;
            # unrelated historical cameras never enter this optimization graph.
            boundary = [(f, i) for f, i in pairs if f not in local][:2]
            observations[point] = [(f, i) for f, i in pairs if f in local] + boundary
        graph_frames = sorted({f for pairs in observations.values() for f, _ in pairs}, key=lambda f: f.id)
        fixed = {f for f in graph_frames if f in anchors or f not in local}
        support = {f: sum(f is obs for pairs in observations.values() for obs, _ in pairs) for f in graph_frames}
        if any(support[f] < 6 for f in graph_frames if f not in fixed):
            result.reason = 'insufficient edges for a free camera'
            return result
        # Require each connected camera component to retain fixed world/scale
        # context. Fixed landmarks already provide that context for pose-only BA.
        if not fix_points:
            adjacency = {f: set() for f in graph_frames}
            for pairs in observations.values():
                members = {f for f, _ in pairs}
                for f in members:
                    adjacency[f].update(members)
            remaining = set(graph_frames)
            while remaining:
                component, pending = set(), [next(iter(remaining))]
                while pending:
                    f = pending.pop()
                    if f not in component:
                        component.add(f)
                        pending.extend(adjacency[f] - component)
                remaining -= component
                fixed_centres = [-f.pose[:3, :3].T @ f.pose[:3, 3] for f in component & fixed]
                if len(fixed_centres) < 2 or max(np.linalg.norm(c - fixed_centres[0]) for c in fixed_centres) < 1e-9:
                    result.reason = 'component lacks a fixed nonzero baseline'
                    return result
        try:
            optimizer = g2o.SparseOptimizer()
            optimizer.set_algorithm(g2o.OptimizationAlgorithmLevenberg(
                g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3())))
            optimizer.set_verbose(verbose)
            # IDs are graph-local and disjoint, even if application frame IDs
            # exceed the old 65536 offset or become noncontiguous after loss.
            cameras = {}
            for identifier, frame in enumerate(graph_frames):
                vertex = camera_vertex(frame, identifier, frame in fixed)
                if not optimizer.add_vertex(vertex):
                    raise RuntimeError('Failed to add camera vertex')
                cameras[frame] = vertex
            vertices = {}
            edges = []
            for identifier, point in enumerate(points, start=len(cameras)):
                vertex = g2o.VertexPointXYZ()
                vertex.set_id(identifier)
                vertex.set_estimate(point.point)
                vertex.set_marginalized(True)
                vertex.set_fixed(fix_points)
                if not optimizer.add_vertex(vertex):
                    raise RuntimeError('Failed to add point vertex')
                vertices[point] = vertex
                for frame, index in observations[point]:
                    edge = g2o.EdgeProjectP2MC()
                    edge.set_vertex(0, vertex)
                    edge.set_vertex(1, cameras[frame])
                    edge.set_measurement(frame._kps[index])
                    edge.set_information(np.eye(2))
                    edge.set_robust_kernel(g2o.RobustKernelHuber(np.sqrt(5.991)))
                    if not optimizer.add_edge(edge):
                        raise RuntimeError('Failed to add observation edge')
                    edges.append(edge)
            result.edges = len(edges)
            optimizer.initialize_optimization()
            optimizer.compute_active_errors()
            result.before_chi2 = float(optimizer.active_chi2())
            result.iterations = int(optimizer.optimize(iterations))
            optimizer.compute_active_errors()
            result.after_chi2 = float(optimizer.active_chi2())
            new_poses = {f: np.linalg.inv(pose_rt(v.estimate().rotation().matrix(), v.estimate().translation()))
                         for f, v in cameras.items()}
            new_points = {p: np.asarray(v.estimate()).copy() for p, v in vertices.items()}
            valid = (result.iterations > 0 and np.isfinite(result.before_chi2)
                     and np.isfinite(result.after_chi2)
                     and result.after_chi2 <= result.before_chi2 + 1e-7
                     and all(valid_pose(pose) for pose in new_poses.values())
                     and all(np.isfinite(p).all() for p in new_points.values())
                     and all(np.allclose(new_poses[f], f.pose, atol=1e-8) for f in fixed))
            for point, pairs in observations.items():
                valid = valid and all(project(f.k, new_poses[f], [new_points[point]])[2][0] for f, _ in pairs)
            if not valid:
                result.status, result.reason = 'rejected', 'invalid or worsening optimizer result'
            else:
                # Publish both cameras and landmarks only after validating the
                # entire candidate state; rejection cannot partially move the map.
                # Trajectory snapshots follow the same accepted BA revision.
                self.trajectory.update_poses({f.id: p for f, p in new_poses.items() if f not in fixed})
                for frame, pose in new_poses.items():
                    if frame not in fixed:
                        frame.pose = pose
                if not fix_points:
                    for point, xyz in new_points.items():
                        point.point = xyz
                result.status = 'accepted'
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as exc:
            result.status, result.reason = 'failed', str(exc)
        result.seconds = time.perf_counter() - start
        return result
