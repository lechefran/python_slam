"""Bounded pose-only IRLS in centred coordinates; no map or viewer dependencies."""

import cv2
import numpy as np


HUBER_DELTA_PX = 1.5
RANK_RTOL = 1e-10
MAX_STEP_CONDITION = 1e6


def huber_blocks(residuals, delta=HUBER_DELTA_PX):
    """Return one weight per 2D observation and sum rho(||pixel error||²).

    Both pixel axes share w=min(1, delta/||e||): rotating image error axes must
    not change a landmark's influence. This is an M-estimator, not covariance.
    """
    norms = np.linalg.norm(residuals, axis=1)
    weights = np.ones(len(norms))
    tail = norms > delta
    weights[tail] = delta / norms[tail]
    cost = np.where(tail, 2 * delta * norms - delta ** 2, norms ** 2)
    return weights, float(cost.sum())


def linearize(k, camera):
    """Pixel Jacobian (N,2,6) for rotation about cloud centre and scaled motion.

    Y' = Exp(dw)(Y-pivot) + pivot + radius*dv. dw is radians; dv is translation
    in cloud RMS-radius units. This keeps the condition number independent of
    world origin and monocular scale, without independently whitening columns
    (which would hide weak directions). No normal equations are formed.
    """
    camera = np.asarray(camera, dtype=np.float64)
    if not len(camera) or not np.isfinite(camera).all() or np.any(camera[:, 2] <= 0):
        raise ValueError('Jacobian requires finite positive-depth support')
    offsets = camera - camera[0]
    mean_offset = offsets.mean(axis=0)
    pivot = camera[0] + mean_offset
    centered = offsets - mean_offset
    radius = float(np.sqrt(np.mean(np.sum(centered ** 2, axis=1))))
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError('Jacobian support has no representable spread')
    homogeneous = camera @ k.T
    pixels = homogeneous[:, :2] / homogeneous[:, 2:]
    # Quotient rule supports anisotropic intrinsics and skew, in pixel units.
    projection = (k[None, :2, :] - pixels[:, :, None] * k[None, 2:3, :]) / homogeneous[:, 2:, None]
    motion = np.zeros((len(camera), 3, 6))
    x, y, z = centered.T
    motion[:, 0, 1], motion[:, 0, 2] = z, -y
    motion[:, 1, 0], motion[:, 1, 2] = -z, x
    motion[:, 2, 0], motion[:, 2, 1] = y, -x
    motion[:, :, 3:] = radius * np.eye(3)
    jacobian = projection @ motion
    if not np.isfinite(jacobian).all() or not np.isfinite(pixels).all():
        raise ValueError('Non-finite projection Jacobian')
    return pixels, jacobian, pivot, radius


def spectrum(jacobian, weights):
    """Check the weighted design matrix itself, whose condition is not squared."""
    matrix = (jacobian * np.sqrt(weights)[:, None, None]).reshape(-1, 6)
    values = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.count_nonzero(values > values[0] * RANK_RTOL)) if len(values) else 0
    ratio = float(values[-1] / values[0]) if len(values) == 6 and values[0] > 0 else 0.
    condition = 1. / ratio if ratio > 0 else None
    status = ('rank_deficient' if rank < 6 else
              'ill_conditioned' if condition > MAX_STEP_CONDITION else 'well_conditioned')
    return {'rank': rank, 'singular_values': values.tolist(), 'condition_number': condition,
            'relative_smallest_singular_value': ratio, 'status': status,
            'rank_relative_tolerance': RANK_RTOL, 'step_condition_limit': MAX_STEP_CONDITION,
            'parameterization': 'camera_cloud_centred_rotation_radians_translation_rms_radius'}


def inspect_pose(k, camera, pixels):
    """Report weighted/unweighted observability on a stated, fixed support set."""
    if np.shape(pixels) != (len(camera), 2) or not np.isfinite(pixels).all():
        raise ValueError('Residual weighting requires finite aligned pixel observations')
    projected, jacobian, pivot, radius = linearize(k, camera)
    residuals = projected - pixels
    weights, cost = huber_blocks(residuals)
    metrics = {'input_count': len(camera), 'huber_delta_px': HUBER_DELTA_PX,
               'huber_cost_px2': cost, 'downweighted_count': int(np.count_nonzero(weights < 1)),
               'minimum_weight': float(weights.min()),
               'effective_observations': float(weights.sum() ** 2 / np.dot(weights, weights)),
               'unweighted': spectrum(jacobian, np.ones(len(camera))),
               'weighted': spectrum(jacobian, weights)}
    return metrics, residuals, jacobian, weights, pivot, radius


def robust_refine(k, xyz, pixels, rotation, translation, enabled=True):
    """Fit fixed support with at most 3 IRLS steps and 6 backtracking trials each.

    XYZ and translation use the same centred solver coordinates. Reject bad
    depth, deficient/ill-conditioned steps and nondecreasing Huber cost. Return
    a proposal only; the caller must check the full correspondence population.
    """
    rotation, translation = rotation.copy(), np.asarray(translation).reshape(3).copy()
    evidence = {'enabled': enabled, 'attempted_steps': 0, 'accepted_steps': 0, 'steps': []}
    try:
        initial, *_ = inspect_pose(k, xyz @ rotation.T + translation, pixels)
        evidence['initial'] = initial
        for _ in range(3):
            current, residuals, jacobian, weights, pivot, radius = inspect_pose(
                k, xyz @ rotation.T + translation, pixels)
            status = current['weighted']['status']
            if not enabled or status != 'well_conditioned' or not current['downweighted_count']:
                evidence['stop_reason'] = 'disabled' if not enabled else (
                    status if status != 'well_conditioned' else 'quadratic_region')
                break
            evidence['attempted_steps'] += 1
            root_weights = np.sqrt(weights)
            matrix = (jacobian * root_weights[:, None, None]).reshape(-1, 6)
            target = -(residuals * root_weights[:, None]).ravel()
            step, _, rank, _ = np.linalg.lstsq(matrix, target, rcond=RANK_RTOL)
            if rank < 6 or not np.isfinite(step).all():
                evidence['stop_reason'] = 'invalid_linear_step'
                break
            accepted = False
            for fraction in (1., .5, .25, .125, .0625, .03125):
                increment = cv2.Rodrigues(fraction * step[:3])[0]
                proposed_r = increment @ rotation
                proposed_t = increment @ (translation - pivot) + pivot + radius * fraction * step[3:]
                try:
                    proposed, *_ = inspect_pose(k, xyz @ proposed_r.T + proposed_t, pixels)
                except (ValueError, np.linalg.LinAlgError):
                    continue
                # An SVD rank check cannot infer accuracy; it only prevents a
                # locally unobservable pose update from entering the map.
                if (proposed['weighted']['status'] == 'well_conditioned'
                        and proposed['huber_cost_px2'] < current['huber_cost_px2'] - 1e-9):
                    rotation, translation = proposed_r, proposed_t
                    accepted = True
                    evidence['accepted_steps'] += 1
                    evidence['steps'].append({'before_cost_px2': current['huber_cost_px2'],
                        'after_cost_px2': proposed['huber_cost_px2'], 'step_fraction': fraction})
                    break
            if not accepted:
                evidence['stop_reason'] = 'no_descent'
                break
        else:
            evidence['stop_reason'] = 'step_limit'
        evidence['final'], *_ = inspect_pose(k, xyz @ rotation.T + translation, pixels)
    except (ValueError, np.linalg.LinAlgError, cv2.error) as exc:
        evidence.update(stop_reason='numerical_failure', error=str(exc))
        return None, None, evidence
    return rotation, translation, evidence
