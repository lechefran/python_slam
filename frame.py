"""Classical ORB frontend with explicit failure and world-to-camera contracts."""

import cv2
import numpy as np
from scipy.spatial import cKDTree

from geometry import ConditionedPoints, add_one, condition_points, denormalize, normalize, pose_rt, spatial_support, valid_pose
from robust_pose import HUBER_DELTA_PX, inspect_pose, robust_refine


class TrackingError(RuntimeError):
    """Insufficient or invalid visual evidence; the map must remain unchanged."""


class Frame:
    def __init__(self, img_map, img, k, frame_id=None, timestamp=0.0, detector=None, mask=None):
        """Prepare features without registering an unvalidated camera in the map."""
        self.k = np.asarray(k, dtype=np.float64)
        self.h, self.w = img.shape[:2]
        self.kinv = np.linalg.inv(self.k)
        self._kps, self.des = extract(img, detector, mask)
        self.kps = normalize(self.kinv, self._kps)
        self.pts = [None] * len(self.kps)
        self.pose = np.eye(4)
        self.id = img_map.next_frame_id if frame_id is None else frame_id
        self.timestamp = timestamp
        self.kd = cKDTree(self._kps)


def extract(img, detector=None, mask=None):
    """Return pixel positions (N,2) and aligned uint8 ORB descriptors (N,32)."""
    if img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
        raise ValueError('Expected a uint8 BGR image')
    detector = detector if detector is not None else cv2.ORB_create(nfeatures=2000)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Let ORB retain its orientation and pyramid metadata; descriptor computation
    # on manually constructed corners does not supply the full detector pipeline.
    keypoints, descriptors = detector.detectAndCompute(gray, mask)
    if descriptors is None or not keypoints:
        return np.empty((0, 2)), np.empty((0, 32), dtype=np.uint8)
    pixels = np.array([keypoint.pt for keypoint in keypoints], dtype=np.float64)
    if mask is not None:
        # Pyramid mask resampling can admit boundary centres. Apply one final
        # processed-pixel check to both arrays, keeping descriptor rows aligned.
        uv = np.floor(pixels + .5).astype(int)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < mask.shape[1]) & (uv[:, 1] >= 0) & (uv[:, 1] < mask.shape[0])
        allowed = np.zeros(len(pixels), dtype=bool)
        allowed[inside] = mask[uv[inside, 1], uv[inside, 0]] != 0
        pixels, descriptors = pixels[allowed], descriptors[allowed]
    return pixels, descriptors


def match_features(current, previous, diagnostics=None):
    """Return unique current/previous feature indices after binary descriptor gates."""
    if diagnostics is not None:
        diagnostics.update(query_features=len(current.des), train_features=len(previous.des),
                           knn_pairs=0, ratio_pass=0, distance_pass=0, unique_matches=0)
    if len(current.des) == 0 or len(previous.des) < 2:
        return np.empty(0, dtype=int), np.empty(0, dtype=int)
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(current.des, previous.des, k=2)
    candidates = [pair[0] for pair in pairs if len(pair) == 2
                  and pair[0].distance < 0.75 * pair[1].distance and pair[0].distance < 64]
    if diagnostics is not None:
        diagnostics.update(knn_pairs=sum(len(pair) == 2 for pair in pairs),
                           ratio_pass=sum(len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance for pair in pairs),
                           distance_pass=len(candidates))
    # Resolve competing matches by descriptor evidence, not incidental feature order.
    candidates.sort(key=lambda item: (item.distance, item.queryIdx, item.trainIdx))
    used = set()
    accepted = []
    for item in candidates:
        if item.trainIdx not in used:
            used.add(item.trainIdx)
            accepted.append((item.queryIdx, item.trainIdx))
    indices = np.array(accepted, dtype=int).reshape(-1, 2)
    if diagnostics is not None:
        diagnostics['unique_matches'] = len(indices)
    return indices[:, 0], indices[:, 1]


def recover_relative(previous_rays, current_rays, focal, threshold_px=1.0):
    """Estimate T_current_previous from normalized (N,2) rays, up to scale.

    Robust fitting and cheirality masks refer to the same input correspondence rows.
    """
    if len(previous_rays) < 8:
        raise TrackingError('fewer than eight two-view correspondences')
    essential, mask = cv2.findEssentialMat(
        previous_rays, current_rays, np.eye(3), method=cv2.RANSAC,
        prob=0.999, threshold=threshold_px / focal)
    if essential is None or mask is None or not np.isfinite(essential).all():
        raise TrackingError('essential estimation failed')
    best = None
    # OpenCV can return multiple essential candidates. Select physical visibility,
    # never a trace heuristic or a manual translation sign chosen from the video.
    for candidate in essential.reshape(-1, 3, 3):
        count, rotation, translation, physical = cv2.recoverPose(
            candidate, previous_rays, current_rays, np.eye(3), mask=mask.copy())
        pose = pose_rt(rotation, translation)
        if valid_pose(pose) and (best is None or count > best[0]):
            best = count, pose, physical.ravel().astype(bool)
    if best is None or best[0] < 12:
        raise TrackingError('insufficient positive-depth support for initialization')
    return best[1], best[2]


def match(current, previous):
    """Compatibility helper returning current/previous indices and relative pose."""
    i, j = match_features(current, previous)
    pose, good = recover_relative(previous.kps[j], current.kps[i], min(current.k[0, 0], current.k[1, 1]))
    return i[good], j[good], pose


def _pose_candidate(frame, xyz, pixels, rotation, translation, max_error, require_coverage):
    """Check one T_cw hypothesis against the same complete set of pixel observations.

    The RANSAC row mask is not a substitute for projecting its returned pose.
    Capped squared error gives every candidate the same scoring population;
    invalid depth costs the full cap instead of disappearing from the score.
    """
    from geometry import project
    from tracking_diagnostics import residual_summary

    finite = np.isfinite(rotation).all() and np.isfinite(translation).all()
    pose = pose_rt(cv2.Rodrigues(rotation)[0], translation) if finite else None
    valid = pose is not None and bool(valid_pose(pose))
    projected = np.full((len(xyz), 2), np.nan)
    visible = np.zeros(len(xyz), dtype=bool)
    if valid:
        projected, _, visible = project(frame.k, pose, xyz)
    errors = np.linalg.norm(projected - pixels, axis=1)
    selected = np.flatnonzero(visible & (errors <= max_error))
    spans = np.ptp(pixels[selected], axis=0) if len(selected) else np.zeros(2)
    fractions = spans / [frame.w, frame.h]
    score = np.full(len(xyz), max_error ** 2)
    score[visible] = np.minimum(errors[visible], max_error) ** 2
    gate = ('refined_pose' if not valid or len(selected) < 12 else
            'image_coverage' if require_coverage and np.any(fractions < .1) else 'accepted')
    metrics = {'status': 'accepted' if gate == 'accepted' else 'rejected', 'gate': gate,
        'pose_valid': valid, 'refined_inliers': len(selected), 'positive_depth': int(visible.sum()),
        'span_px': spans.tolist(), 'span_fraction': fractions.tolist(),
        'inlier_bounds_px': [pixels[selected].min(axis=0).tolist(), pixels[selected].max(axis=0).tolist()]
                           if len(selected) else None,
        'all_residual_px': residual_summary(errors), 'inlier_residual_px': residual_summary(errors[selected]),
        'clipped_cost_px2': float(score.sum())}
    metrics['spatial_support'] = spatial_support(pixels[selected], frame.w, frame.h)
    return {'pose': pose, 'projected': projected, 'rows': selected, 'metrics': metrics}


def _solver_points(xyz, condition):
    """Validate XYZ and choose centred solver coordinates or the legacy reference."""
    conditioned = condition_points(xyz)
    if condition:
        return conditioned
    return ConditionedPoints(np.ascontiguousarray(xyz), np.zeros(3), 1., method='none')


def refine_pose(frame, xyz, pixels, rotation, translation, ransac_rows, max_error=3.,
                require_coverage=True, condition=True, robust=True):
    """Refine a world-coordinate seed, optionally in a centred/scaled solver frame.

    This entry point retains its world-coordinate contract for callers with an
    existing pose. The RANSAC path enters the local helper directly so its seed
    never makes an unnecessary world/local round trip before refinement.
    """
    conditioning = _solver_points(xyz, condition)
    local_translation = conditioning.local_translation(cv2.Rodrigues(rotation)[0], translation)
    return _refine_conditioned_pose(frame, xyz, pixels, conditioning, rotation,
                                    local_translation, ransac_rows, max_error, require_coverage, robust)


def _refine_conditioned_pose(frame, xyz, pixels, conditioning, rotation, translation,
                             ransac_rows, max_error, require_coverage, robust=True):
    """Refit validated consensus at most twice, scoring every original observation.

    The native RANSAC mask belongs to an earlier hypothesis. After refinement,
    some of its rows may be outliers and other input rows may now agree. Refit
    that measured support, accepting only a strictly lower capped pixel cost
    under the unchanged physical and spatial gates. The cap scores proposals;
    each native refiner still minimizes ordinary squared error on its subset.
    The final bounded IRLS proposal uses a block-Huber objective instead.
    """
    candidate, evidence, candidates = _recover_conditioned_pose(
        frame, xyz, pixels, conditioning, rotation, translation, ransac_rows,
        max_error, require_coverage)
    # A coverage-rejected fit can still provide valid residual support. Refit
    # that actual support before abandoning the camera, but require the full
    # coverage gate again before accepting any result. Never loosen its value.
    if robust and evidence['selected'] is None and candidate['metrics']['gate'] == 'image_coverage':
        candidate, evidence, candidates = _recover_coverage(
            frame, xyz, pixels, conditioning, candidate, evidence, candidates, max_error)
    consensus = {'attempted_rounds': 0, 'accepted_rounds': 0, 'rounds': [],
                 'stop_reason': 'primary_rejected' if evidence['selected'] is None else 'disabled'}
    evidence['consensus'] = consensus
    if evidence['selected'] is None or conditioning.method == 'none':
        return _finish_pose(frame, xyz, pixels, candidate, evidence, candidates,
                            max_error, require_coverage, robust)

    previous_rows = ransac_rows
    for step in range(2):
        rows = candidate['rows']
        if np.array_equal(rows, np.sort(previous_rows)):
            consensus['stop_reason'] = 'stable_support'
            break
        previous_rows = rows
        # The checked T_cw is the public pose contract. Convert its translation
        # back to the temporary solver frame before refining the new subset.
        pose = candidate['pose']
        rvec = cv2.Rodrigues(pose[:3, :3])[0]
        tvec = conditioning.local_translation(pose[:3, :3], pose[:3, 3])
        proposed, refinement, proposals = _refine_seed_pose(
            frame, xyz, pixels, conditioning, rvec, tvec, rows, max_error, require_coverage)
        prefix = f'consensus{step}_'
        candidates.update({prefix + name: value for name, value in proposals.items()})
        consensus['attempted_rounds'] += 1
        improved = (refinement['selected'] is not None and
                    proposed['metrics']['clipped_cost_px2'] < candidate['metrics']['clipped_cost_px2'])
        consensus['rounds'].append({'input_count': len(rows), 'accepted': improved,
            'before_cost_px2': candidate['metrics']['clipped_cost_px2'],
            'proposed_cost_px2': proposed['metrics']['clipped_cost_px2'],
            'refinement_reason': refinement['reason']})
        if not improved:
            consensus['stop_reason'] = 'refit_rejected' if refinement['selected'] is None else 'cost_not_improved'
            break
        candidate = proposed
        evidence.update(selected=prefix + refinement['selected'], inspected=prefix + refinement['inspected'])
        consensus['accepted_rounds'] += 1
    else:
        consensus['stop_reason'] = 'round_limit'
    evidence['candidates'] = {name: item['metrics'] for name, item in candidates.items()}
    return _finish_pose(frame, xyz, pixels, candidate, evidence, candidates,
                        max_error, require_coverage, robust)


def _recover_coverage(frame, xyz, pixels, conditioning, candidate, evidence, candidates, max_error):
    """Try at most two actual-consensus fits for an otherwise physical seed.

    Intermediate fitting may have narrow image support; committing a camera
    may not. Each round must lower full-input capped squared cost, and an
    accepted recovery must pass the original complete geometric gates.
    """
    recovery = {'attempted_rounds': 0, 'accepted': False}
    evidence['coverage_recovery'] = recovery
    fitting = candidate
    for step in range(2):
        pose = fitting['pose']
        proposed, details, proposals = _refine_seed_pose(frame, xyz, pixels, conditioning,
            cv2.Rodrigues(pose[:3, :3])[0],
            conditioning.local_translation(pose[:3, :3], pose[:3, 3]),
            fitting['rows'], max_error, False)
        recovery['attempted_rounds'] += 1
        if (details['selected'] is None or proposed['metrics']['clipped_cost_px2']
                >= fitting['metrics']['clipped_cost_px2']):
            break
        fitting = proposed
        name = f'coverage{step}'
        checked = _pose_candidate(frame, xyz, pixels, cv2.Rodrigues(proposed['pose'][:3, :3])[0],
            proposed['pose'][:3, 3], max_error, True)
        candidates[name] = checked
        if checked['metrics']['status'] == 'accepted':
            candidate = checked
            evidence.update(selected=name, inspected=name)
            recovery['accepted'] = True
            break
    return candidate, evidence, candidates


def _finish_pose(frame, xyz, pixels, candidate, evidence, candidates,
                 max_error, require_coverage, robust):
    """Check final support conditioning and cautiously accept a robust proposal.

    Freeze the checked support for IRLS. Rank/conditioning are evaluated in
    normalized local coordinates even for the uncentred native reference mode.
    Reproject a proposal in world coordinates before it can replace the seed.
    """
    coverage_recovery = (robust and evidence['selected'] is None
                         and candidate['metrics']['gate'] == 'image_coverage')
    if evidence['selected'] is None and not coverage_recovery:
        return candidate, evidence, candidates
    rows, pose = candidate['rows'], candidate['pose']
    try:
        local = condition_points(xyz[rows])
        rotation = pose[:3, :3]
        translation = local.local_translation(rotation, pose[:3, 3]).ravel()
        new_r, new_t, report = robust_refine(frame.k, local.points, pixels[rows], rotation, translation, robust)
    except ValueError as exc:
        new_r, new_t = None, None
        report = {'enabled': robust, 'accepted_steps': 0, 'stop_reason': 'numerical_failure', 'error': str(exc)}
    report['selected'] = False
    report['coverage_recovery'] = coverage_recovery
    evidence['robust'] = report
    initial = report.get('initial')
    candidate['metrics']['pose_conditioning'] = initial
    # A deficient seed is not authorized merely because its pixels fit. A
    # full-rank but poorly conditioned seed remains visible with a warning;
    # no robust update is attempted in that case.
    if initial is None or initial['weighted']['rank'] < 6:
        candidate['metrics'].update(status='rejected', gate='pose_conditioning')
        evidence['selected'] = None
    elif new_r is not None and report['accepted_steps']:
        proposed = _pose_candidate(frame, xyz, pixels, cv2.Rodrigues(new_r)[0],
            local.world_translation(new_r, new_t), max_error, require_coverage)
        candidates['robust'] = proposed
        # Compare the SAME complete population with capped block-Huber loss.
        # Invalid depth receives the full cap, never a free zero residual.
        def cost(item):
            errors = np.linalg.norm(item['projected'] - pixels, axis=1)
            errors = np.minimum(np.where(np.isfinite(errors), errors, max_error), max_error)
            return float(np.where(errors <= HUBER_DELTA_PX, errors ** 2,
                2 * HUBER_DELTA_PX * errors - HUBER_DELTA_PX ** 2).sum())

        before, after = cost(candidate), cost(proposed)
        report.update(before_full_cost_px2=before, proposed_full_cost_px2=after)
        if proposed['metrics']['status'] == 'accepted':
            try:
                support = proposed['rows']
                # Newly admitted observations change the condition of the final
                # pose: inspect that actual support too, not just the IRLS rows.
                final_local = condition_points(xyz[support])
                final_t = final_local.local_translation(new_r, proposed['pose'][:3, 3]).ravel()
                checked, *_ = inspect_pose(frame.k, final_local.points @ new_r.T + final_t, pixels[support])
                proposed['metrics']['pose_conditioning'] = checked
            except (ValueError, np.linalg.LinAlgError):
                checked = None
            # Downweight marginal observations without silently discarding the
            # checked consensus used for projection search and map maintenance.
            support_preserved = bool(np.isin(rows, proposed['rows']).all())
            report['seed_support_preserved'] = support_preserved
            if (checked is not None and checked['weighted']['status'] == 'well_conditioned'
                    and support_preserved
                    and after < before - 1e-9):
                candidate = proposed
                evidence.update(selected='robust', inspected='robust')
                report['selected'] = True
        report['selection_reason'] = ('lower_full_huber_cost' if report['selected'] else
            'seed_support_changed' if report.get('seed_support_preserved') is False else 'proposal_rejected')
    evidence['candidates'] = {name: item['metrics'] for name, item in candidates.items()}
    return candidate, evidence, candidates


def _recover_conditioned_pose(frame, xyz, pixels, conditioning, rotation, translation,
                              ransac_rows, max_error, require_coverage):
    """Recover a failed centred fit with one independent seed on the same mask.

    EPnP's all-inlier fit can lose the hypothesis that produced its RANSAC mask.
    Local refiners may stay in that bad basin. SQPnP supplies a new starting
    pose, using only those same rows and the same centred coordinate system.
    Every proposal still faces the original world-depth/pixel/coverage checks.
    """
    arguments = (frame, xyz, pixels, conditioning)
    tail = (ransac_rows, max_error, require_coverage)
    candidate, evidence, candidates = _refine_seed_pose(
        *arguments, rotation, translation, *tail)
    recovery = {'attempted': False, 'reason': 'primary_validated' if evidence['selected'] else 'disabled'}
    evidence['seed_recovery'] = recovery
    if evidence['selected'] is not None or conditioning.method == 'none':
        return candidate, evidence, candidates

    recovery.update(attempted=True, reason='primary_rejected', solver='SQPNP',
                    input_count=len(ransac_rows))
    try:
        ok, new_rotation, new_translation = cv2.solvePnP(
            conditioning.points[ransac_rows], pixels[ransac_rows], frame.k, None,
            flags=cv2.SOLVEPNP_SQPNP)
    except cv2.error as exc:
        recovery.update(status='failed', native_error=str(exc))
        return candidate, evidence, candidates
    if not ok:
        recovery['status'] = 'failed'
        return candidate, evidence, candidates

    recovered, alternative, alternatives = _refine_seed_pose(
        *arguments, new_rotation, new_translation, *tail)
    # Keep both attempts visible; a rejected recovery must not erase the reason
    # the original pose failed. All trace poses remain world-to-camera.
    names = {name: 'sqpnp_pose' if name == 'ransac_pose' else 'sqpnp_' + name
             for name in alternatives}
    candidates.update({names[name]: value for name, value in alternatives.items()})
    recovery['status'] = 'accepted' if alternative['selected'] else 'rejected'
    if alternative['selected'] is not None:
        candidate = recovered
        evidence.update(selected=names[alternative['selected']], inspected=names[alternative['inspected']])
    evidence['candidates'] = {name: item['metrics'] for name, item in candidates.items()}
    return candidate, evidence, candidates


def _refine_seed_pose(frame, xyz, pixels, conditioning, rotation, translation,
                      ransac_rows, max_error, require_coverage):
    """Try LM once, with one VVS fallback if LM fails validation or worsens cost.

    All native XYZ/tvec inputs share the same local coordinates. Each returned
    candidate is converted back to world coordinates before the unchanged depth,
    pixel-error and coverage checks. OpenCV may mutate its seed arrays in place.
    """
    def world_candidate(rvec, local_tvec):
        if np.isfinite(rvec).all() and np.isfinite(local_tvec).all():
            with np.errstate(over='ignore', invalid='ignore'):
                world_tvec = conditioning.world_translation(cv2.Rodrigues(rvec)[0], local_tvec)
        else:
            world_tvec = np.full((3, 1), np.nan)
        return _pose_candidate(frame, xyz, pixels, rvec, world_tvec, max_error, require_coverage)

    candidates = {'ransac_pose': world_candidate(rotation, translation)}

    def attempt(name, solver):
        try:
            rvec, tvec = solver(conditioning.points[ransac_rows], pixels[ransac_rows], frame.k, None,
                               rotation.copy(), translation.copy())
            candidate = world_candidate(rvec, tvec)
        except cv2.error as exc:
            # A local refinement failure must not overwrite an independently
            # validated candidate or masquerade as a successfully refined pose.
            candidate = _pose_candidate(frame, xyz, pixels, np.full((3, 1), np.nan),
                                        np.full((3, 1), np.nan), max_error, require_coverage)
            candidate['metrics']['native_error'] = str(exc)
        candidates[name] = candidate
        return candidate['metrics']

    initial = candidates['ransac_pose']['metrics']
    lm = attempt('lm', cv2.solvePnPRefineLM)
    tolerance = max(1e-8, 1e-6 * initial['clipped_cost_px2'])
    reason = ('lm_rejected_' + lm['gate'] if lm['status'] != 'accepted' else
              'lm_increased_cost' if initial['status'] == 'accepted'
              and lm['clipped_cost_px2'] > initial['clipped_cost_px2'] + tolerance else 'lm_validated')
    fallback = reason != 'lm_validated'
    if fallback:
        attempt('vvs', cv2.solvePnPRefineVVS)
    eligible = [name for name, candidate in candidates.items()
                if candidate['metrics']['status'] == 'accepted']
    if not fallback:
        selected = 'lm'
    else:
        selected = min(eligible, key=lambda name: candidates[name]['metrics']['clipped_cost_px2']) if eligible else None
    # Preserve a useful rejected hypothesis for diagnostics even when no pose is
    # eligible. The caller raises TrackingError before this can reach the map.
    inspected = selected or min(candidates, key=lambda name: candidates[name]['metrics']['clipped_cost_px2'])
    return candidates[inspected], {'selected': selected, 'inspected': inspected,
        'fallback_attempted': fallback, 'reason': reason,
        'conditioning': conditioning.metadata(),
        'candidates': {name: candidate['metrics'] for name, candidate in candidates.items()}}, candidates


def estimate_pose(frame, points, indices, max_error=3.0, require_coverage=True,
                  diagnostics=None, trace=None, condition=True, robust=True):
    """Robust 3D-to-2D pose in map scale; return T_cw and accepted input rows."""
    xyz = np.asarray([point.point for point in points], dtype=np.float64).reshape(-1, 3)
    pixels = np.asarray(frame._kps[indices], dtype=np.float64)
    # Observe the existing solver once, including partial evidence on rejection.
    # Input rows always refer to the same world XYZ and processed-image pixels.
    if diagnostics is not None:
        diagnostics.update(status='rejected', gate='correspondence_count', input_count=len(xyz),
                           ransac_inliers=0, refined_inliers=0, positive_depth=0,
                           coverage_required=require_coverage, max_error_px=max_error)
    if trace is not None:
        trace.update(xyz=xyz.tolist(), pixels=pixels.tolist(), feature_indices=list(map(int, indices)),
                     landmark_ids=[int(p.id) for p in points], ransac_rows=[], refined_rows=[])
    if len(xyz) < 12:
        raise TrackingError('fewer than twelve map correspondences')
    if diagnostics is not None:
        diagnostics['gate'] = 'conditioning'
    try:
        conditioning = _solver_points(xyz, condition)
    except ValueError as exc:
        raise TrackingError(str(exc)) from exc
    if not np.isfinite(pixels).all():
        raise TrackingError('Pose fitting requires finite pixel observations')
    if diagnostics is not None:
        diagnostics['conditioning'] = conditioning.metadata()
    ok, rotation, translation, inliers = cv2.solvePnPRansac(
        conditioning.points, pixels, frame.k, None, iterationsCount=200, reprojectionError=max_error,
        confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
    if diagnostics is not None:
        diagnostics.update(gate='ransac_support', ransac_inliers=0 if inliers is None else len(inliers))
    if trace is not None and inliers is not None:
        trace['ransac_rows'] = inliers.ravel().tolist()
    if not ok or inliers is None or len(inliers) < 12:
        raise TrackingError('insufficient PnP inliers')
    candidate, refinement, candidates = _refine_conditioned_pose(frame, xyz, pixels, conditioning,
        rotation, translation, inliers.ravel(), max_error, require_coverage, robust)
    pose, selected, metrics = candidate['pose'], candidate['rows'], candidate['metrics']
    if diagnostics is not None:
        diagnostics.update(metrics, refinement=refinement)
    if trace is not None:
        trace.update(T_cw=pose.tolist() if pose is not None else None, refined_rows=selected.tolist(),
                     projected_pixels=candidate['projected'].tolist(),
                     refinement={name: {'T_cw': item['pose'].tolist() if item['pose'] is not None else None,
                                        'rows': item['rows'].tolist()} for name, item in candidates.items()})
    if metrics['gate'] == 'refined_pose':
        raise TrackingError('invalid refined PnP pose')
    if metrics['gate'] == 'image_coverage':
        raise TrackingError('map inliers have insufficient image coverage')
    if metrics['gate'] == 'pose_conditioning':
        raise TrackingError('invalid or rank-deficient PnP support')
    if diagnostics is not None:
        diagnostics.update(status='accepted', gate='accepted')
    return pose, selected
