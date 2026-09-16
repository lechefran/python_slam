# Robust residual weighting and pose conditioning

The subsequent [older-keyframe fallback](keyframe-recovery.md) adds recovery
after normal tracking fails. Add `--no-recovery` to reproduce the pre-recovery
measurements in this document.

## Scope

With `--robust-pnp`, PnP performs bounded block-Huber refinement after the existing
centered RANSAC, seed recovery and consensus fits. Jacobian checks always run on the actual
accepted support. Both provisional and final PnP use the checks; robust updates
run on final PnP after projection association has assembled the complete input
population. The provisional search seed stays on the established solver path. The change
preserves world-to-camera poses, pixel residual units and reciprocal map
observations. It introduces no dependency or model download.

The existing native bundle adjustment already uses a Huber kernel per pixel
edge. This change addresses the previously unweighted **pose-only** refiners;
it does not add bundle-adjustment rank tests or calibrated pose covariance.

## Residual weighting

For each selected landmark, the residual is a two-component pixel error
`e = projected_pixel - observed_pixel`. Its block norm is `r = ||e||`.
With `delta = 1.5` processed-image pixels:

- `rho(r²) = r²` when `r <= delta`;
- `rho(r²) = 2*delta*r - delta²` otherwise;
- the iterative least-squares weight is `w = min(1, delta/r)`, with `w=1`
  at zero error. Both pixel axes share the same weight.

This follows the [residual-block robust-loss formulation documented by Ceres](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#lossfunction).
The implementation uses existing NumPy/OpenCV dependencies, not Ceres. The
transition is an explicit pixel-scale engineering choice: it downweights
marginal support inside the existing 3-pixel acceptance radius. It is not an
estimated noise variance or a calibrated statistical confidence threshold.

The support set is frozen at the checked seed's inlier rows. Each of at most
three iteratively reweighted least-squares (IRLS) steps solves
`sqrt(W) J step = -sqrt(W) e`. The dense matrix has only six columns.
[`numpy.linalg.lstsq`](https://numpy.org/doc/stable/reference/generated/numpy.linalg.lstsq.html)
solves this directly with an explicit relative rank cutoff. Forming or inverting
`J.T @ W @ J` would square its condition number and is deliberately avoided.

Each step tries at most six fractions: 1, 1/2, 1/4, 1/8, 1/16 and 1/32.
It must preserve finite positive depth, pass the weighted condition check and
decrease fixed-support Huber cost. If all residuals are in the quadratic region,
the existing least-squares fit is retained. These are bounded refinement steps,
not a promise of convergence to the global optimum.

## Coordinate-aware conditioning

Checks use a centered, isotropically scaled cloud. For camera points `Y`, let
`pivot` be their centroid and `radius` their RMS distance from that centroid.
The local update is:

```text
Y_new = Exp(dw) (Y - pivot) + pivot + radius * dv
```

`dw` has rotation units (radians); `dv` is translation in cloud-radius units.
The pixel Jacobian is the pinhole projection derivative multiplied by
`[-skew(Y-pivot), radius*I]`. This makes the reported spectrum insensitive to
world-origin shifts and arbitrary monocular units. The six columns are not
independently normalized, which could conceal weak motion directions.

The singular values are computed on both `J` and `sqrt(W) J` for the **same
observations**. The condition number is `largest / smallest` singular value of
the design matrix, not its normal matrix. Its meaning depends on the declared
parameterization; it should not be compared directly to another solver's raw
Rodrigues/world-translation condition number.

| Condition | Action |
| --- | --- |
| Invalid/non-finite support or an initial condition-check failure | Reject the pose before map registration |
| Fewer than six singular directions above `1e-10 * largest` | Reject as numerically rank deficient |
| Full rank but condition number above `1e6` | Retain the previously validated seed with an explicit diagnostic; skip the robust update |
| Well-conditioned support | Permit bounded, independently validated robust proposals |

The cutoff values are conservative numerical policies, not claims of physical
observability or road accuracy. A planar cloud can constrain a local pose and
is not blanket-rejected. Collinear support fails the six-direction test.
Near-collinear support can remain full rank yet prevent refinement. Missing
calibration, moving vehicles, rolling shutter and local pose ambiguity can
still produce a well-conditioned but incorrect estimate.

## Final selection and failure isolation

A robust proposal is converted back to world coordinates and reprojected
against **all original PnP correspondences**. It must meet the existing finite
SE(3), positive-depth, 12-inlier, 3-pixel and applicable image-coverage gates.
Its final support is checked again, including any newly admitted rows.
Every seed inlier must remain an inlier: weighting may reduce a marginal
observation's influence, but may not silently discard the checked consensus
used by projection association and landmark maintenance.

The proposal must lower `sum rho(min(error, 3 px)²)` over that identical full
population. Invalid depth receives the full cap. Changing the fitting subset
cannot make observations disappear from this comparison. Unlike the preceding
ordinary consensus fits, this final selection uses capped Huber cost; ordinary
capped squared error may increase and remains reported separately.

Numerical failures, failed descent or invalid proposals leave the validated
seed untouched. If the seed itself is rank deficient or cannot be checked, it
is rejected; retaining an unchecked pose is not a fallback. No failed proposal
adds a camera or modifies landmark observations.

A fit rejected only for image coverage may still have sufficient physical
residual support. With robust PnP enabled, it receives at most two additional
ordinary consensus refits on its **actual** inliers. Each must lower the
full-input capped squared cost; intermediate narrow support is allowed only
inside fitting. A recovered pose must pass the original full coverage check
before entering the usual bounded consensus/robust path. The saved frame-1652
regression recovers in one such round; a truly narrow-band synthetic scene
remains rejected. A remaining coverage-rejected fit may receive the bounded
robust proposal too, but its seed cannot be retained as an accepted fallback.

## Use and diagnostics

Robust updates are **opt-in** because the full replay still loses four baseline
frames. `--robust-pnp` enables them; the default (also expressible as
`--no-robust-pnp`) retains conditioning checks and the preceding centered and
spatial-mapping solver. Reproducing older pre-spatial or uncentered results
also requires their respective disabling flags.

```sh
.venv-portability/bin/python -m scripts.benchmark_tracking output/robust-new-run \
  --video /Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4 \
  --focus-start 850 --focus-end 1799 --every 100 --robust-pnp \
  --baseline output/spatial-qualified-2026-09-15/report.json
```

With diagnostic capture enabled, each PnP stage records:

- `pose_conditioning`: input count, weighted/unweighted singular values, rank,
  condition number, policy thresholds and parameterization for the selected or inspected pose;
- Huber delta, cost, number downweighted, minimum weight and effective
  observations `(sum w)² / sum(w²)`;
- `refinement.robust`: initial/final fixed-support checks, attempted/accepted
  step counts, per-step costs/fractions, stop reason and final selection;
- `refinement.coverage_recovery`: attempted rounds and whether refitting actual
  support restored the unchanged coverage gate;
- `before_full_cost_px2` and `proposed_full_cost_px2` when a proposal is scored;
- `seed_support_preserved` and an explicit selection reason, including
  `seed_support_changed` when a lower-cost fit sacrifices existing support;
- candidate world poses and selected input rows in sampled numeric traces.

`refinement.robust.final` describes the fixed fitting rows; `pose_conditioning`
describes the actual returned support. They need not have equal counts. The
benchmark summary counts checks, selections, condition statuses and stop reasons
per stage and separates differing solver configurations in comparisons.

## Validation

Synthetic tests use independently projected known cameras and points. They
cover analytic-versus-finite-difference derivatives (including skew and unequal
focal lengths), rotationally invariant block weights, monotonic fixed-support
cost, origin/scale invariance, collinear/near-collinear/planar cases, invalid
depth/data, native baseline retention and failure before map commit.

In the deterministic 120-point noise/bias fixture, 24 observations have an added
`[2.7, 0.3]` pixel bias. Three robust steps reduce projection RMSE against known
noise-free truth from **0.3951 to 0.2752 pixels**. This qualifies the fixture,
not the uncalibrated road sequence. The saved frame-874 and frame-941 native
regressions remain part of the full suite.

Saved-input checks retain 114 inliers for frame 874 and 96 for frame 941.
Frame 874 accepts a robust update; its ordinary capped squared error changes
from 343.429 to 343.624 px² while its Huber objective decreases. Frame 941
retains the consensus fit (212.558 px²): the robust alternative would lose
seed correspondences. Weighted Jacobian condition numbers are about 13.58 and
8.03 respectively. These saved inputs describe earlier maps, not measurements
of the same frames in the new sequential replay.

## Development comparisons

The full clip is a development sequence, not held-out validation. Applying
robust updates in both provisional and final PnP initially retained 1,696 poses;
preserving seed correspondences increased that to 1,699 but still lost late
coverage. Moving updates after projection association retained 1,789 poses,
with seven losses at 1,652 and 1,712–1,717. These policies were not accepted as
the final coverage result. Conditioning gates did not cause those losses;
the failed fits had insufficient image coverage. Lower local robust cost alone
does not establish better sequential tracking or trajectory accuracy.

## Full dashcam result

Input: `/Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4`,
SHA-256 `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`.
Replay starts at frame zero and decodes all 1,800 frames. Both runs use width
1,024, ORB budget 2,000, bottom mask 0, source focal 525, seed 0 and one OpenCV
thread. Processed intrinsics remain approximate: `fx=fy=280`, `cx=512`, `cy=288`
at 1,024×576. No geometric acceptance threshold was widened.

| Measure | Previous spatial baseline | Final robust mode |
| --- | ---: | ---: |
| Accepted poses | 1,796 | 1,792 |
| Baseline frame IDs retained | 1,796 | 1,792 |
| Lost frames after initialization | 0 | 4 |
| Last accepted source frame | 1,799 | 1,799 |
| Final landmarks | 51,030 | 52,222 |

The four newly lost frames are **1,719, 1,760, 1,763 and 1,765**, all rejected
for insufficient image coverage. Frame 1,652 is recovered by refitting its
actual inliers. Final robust proposals are selected on **1,123 frames**. The
weighted condition check is full rank and below the step limit on all 1,794
inspected final fits (including the four rejected for coverage); its median
condition number is **8.48**, maximum **135.82**. This does not qualify accuracy.

The new default, with conditioning checks active and robust updates disabled,
reproduces **all 1,796 baseline pose IDs, every per-frame outcome/count and every
final optimized pose exactly**. Its final map has 51,030 landmarks. This separate
full replay took 499.69 seconds under the same uncontrolled timing conditions.

On the 1,790 common tracking frames, median effective image cells change from
3.457 to 3.483 and minor-axis spread from 0.04645 to 0.04722. Largest-cell
concentration slightly worsens (0.42265 to 0.42527), as does central-90% vertical
span (0.14881 to 0.14649). The distribution evidence is mixed; it is not a claim
that the map or trajectory is more accurate.

The final robust replay took **556.29 seconds**, including diagnostic capture
and overlapping validation runs. The previous baseline took 439.93 seconds in
its earlier run. Different map states and overlapping workloads prevent a
controlled performance comparison; no speed claim is made.

Ignored evidence: `output/robust-consensus-recovery-2026-09-15/` is the final
enabled replay, and `output/robust-reference-2026-09-15/` is the disabled-update
reference. Earlier investigations are retained under `output/robust-*` and
`output/robust-investigation/`, with input identity, settings and source hashes.
The final replay explicitly records `robust_pnp=true`; changing the CLI default
to off afterward does not change that tested mode. Only comments and the
default option value changed after its runtime snapshot.

### Package and regression checks

All **80 tests** pass on the local macOS arm64 environment, including native
g2opy and saved PnP regressions. Dependency consistency and whitespace checks
pass. A rebuilt installed command, launched from `/private/tmp`, processes
25 synthetic frames with **24 accepted poses** in both modes: **3,061 landmarks**
with the default and **3,062** with explicit `--robust-pnp`. Its imported
`frame`, `slam` and `robust_pose` module contents match the checkout. The test
suite also verifies that provisional PnP keeps robust updates disabled while
final PnP enables them when requested.

Native Linux execution, real camera calibration and held-out trajectory accuracy
remain unverified. A lower synthetic error, a full-rank Jacobian or retained
frame count does not establish real-road accuracy.
