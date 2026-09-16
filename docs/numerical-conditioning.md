# Numerical conditioning of pose fitting

The subsequent [robust PnP change](robust-pose.md) adds residual weighting and
Jacobian conditioning checks. Add `--no-robust-pnp` to the commands below to
isolate this earlier solver baseline.

## Status and use

The subsequent [spatial-support change](spatial-support.md) adds default landmark replenishment and extends the full replay to 1,796 poses. The measurements below isolate the preceding conditioning correction; use `--no-spatial-mapping` to reproduce its 1,685-pose baseline.

**Centering is enabled by default**, with bounded consensus refitting and recovery from an unusable RANSAC seed. On the requested 1,800-frame `GRMN2734.MP4` replay, this retains **all 1,180 frame IDs accepted by the preceding baseline**, adds **505**, and reaches **1,685 accepted poses**. This qualifies coverage on this clip and configuration; it does not establish real-road trajectory accuracy.

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --no-spatial-mapping --max-frames 1800 --report output/centred-run.json

.venv-portability/bin/python -m scripts.benchmark_tracking output/centred-benchmark \
  --focus-start 850 --focus-end 1799 --every 100 --no-spatial-mapping \
  --baseline output/conditioning-default-2026-09-15/report.json
```

Choose new output names for each benchmark. `--condition-pnp` remains an explicit positive alias; `--no-condition-pnp` selects the previous world-coordinate LM/VVS reference path. The benchmark accepts both flags. Python defaults are `SLAM(..., condition_pnp=True)` and `estimate_pose(..., condition=True)` / `refine_pose(..., condition=True)`.

Reports record `configuration.condition_pnp` and the actual solver-coordinate metadata. Baseline comparison separates identical video/camera/frontend inputs from the conditioning mode. It also compares accepted **frame identities within the common replay horizon**, so additional later poses cannot conceal losing earlier baseline coverage. Equal pose counts do not imply identical trajectories.

## Coordinate contract

Given world points `X_w`, compute their centre `c` and positive RMS radius `s`:

```text
X_local = (X_w - c) / s

X_camera = R * X_w + t_world
         = s * (R * X_local + t_local)

t_local = (R * c + t_world) / s
t_world = s * t_local - R * c
```

Rotations remain unchanged. The scale is isotropic; independently scaling x/y/z would change camera geometry. Intrinsics and image observations remain in processed pixels. This temporary 3D frame is distinct from the normalized 2D rays used by triangulation.

RANSAC, LM, VVS and SQPnP receive the same centered/scaled points and translation convention. RANSAC's seed passes directly to its initial refinement. Later consensus refits start from the independently checked world pose, converting its translation back into solver coordinates. The public refiner also accepts a world-coordinate seed. Native translation arrays have shape `(3,1)`, with independent copies for each attempt because OpenCV may mutate them.

Every candidate is converted back to **world-to-camera `T_cw`** before validation. Map landmarks, bundle adjustment, viewer data and exported report poses keep the original world coordinates and arbitrary monocular scale.

## Why centering alone was insufficient

The first centered implementation retained only 954 poses. Replaying 95 saved correspondence sets separated local fitting behavior from changes accumulated in the map. Centering reduced the aggregate capped pixel cost on these fixed inputs, but did not eliminate failed seeds or make the original RANSAC mask agree with the final refined pose.

Two corrections address those failures:

1. **Recover a rejected seed once.** Keep the existing EPnP RANSAC and its LM/VVS path. If none of their poses passes validation, run SQPnP on the **same RANSAC rows**, then refine and validate its new seed. Do not replace an already valid primary fit with an unrelated seed. Native errors, non-finite results and rejected recovery poses remain explicit failures.
2. **Refit the actual validated inliers.** After an accepted fit, reproject every original correspondence and collect the rows passing the existing depth and 3-pixel gates. If that set differs from the last fitting subset, refine it again from the checked pose. Accept the proposal only if it passes all required gates and strictly lowers the capped squared pixel cost across **all original inputs**. Stop when support stabilizes, cost fails to improve, validation fails, or two rounds have run.

This is bounded local optimization of the consensus set. The original mask may contain observations that no longer agree with the pose, and it may exclude observations that now do. Keeping that stale subset throughout refinement leaves useful evidence unused. A rejected refit retains the preceding valid candidate; it cannot overwrite the map or authorize a bad pose.

The cost cap is a **candidate scoring rule**, not the loss minimized by native LM/VVS. Those refiners still minimize ordinary squared reprojection error on their current subset. Invalid-depth observations cost the full cap, preventing a candidate from winning by hiding difficult points behind the camera. The two-round limit bounds extra fitting work; it is not a convergence guarantee.

[OpenCV's official PnP documentation](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html) describes the transform convention, SQPnP and the LM/VVS refinement methods. No new dependency or solver framework is introduced.

### Fixed-input regressions

The checked-in fixtures contain numeric world points, pixels, camera intrinsics and source-video identity, with no images or pose ground truth.

| Saved input | Prior centered behavior | Current centered behavior |
| --- | --- | --- |
| Frame 874, 129 correspondences | 107 RANSAC-mask rows, but zero validated rows for the returned pose, LM and VVS | SQPnP recovery, then two consensus refits: 114 validated rows; capped cost 343.43 px²; image spans 77.1% / 28.7% |
| Frame 941, 103 correspondences | Centered LM: 98 validated rows; capped cost 216.57 px² | Two consensus refits: 96 validated rows; capped cost 212.56 px²; image spans 77.9% / 11.8% |

The frame-941 fit illustrates why raw inlier count is not the sole selection criterion: the current fit has fewer inliers and lower total capped error while retaining the required spatial support. These fixed-input measurements differ from sequential replay, which builds a different accumulated map.

Diagnostics retain the chosen candidate, every candidate's validation metrics, `seed_recovery` status, and each consensus round's input count, before/proposed costs, acceptance and stopping reason. Sampled traces retain candidate world poses and their validated rows.

## Numerical and correctness safeguards

- Compute the center from scaled offsets relative to an existing point, avoiding sums of large absolute coordinates.
- Compute RMS radius through bounded intermediate values rather than squaring extreme magnitudes directly.
- Keep float64 throughout normalization and pose conversion; use contiguous native point inputs.
- Reject empty/non-finite inputs, coincident points, overflowed spans, unrepresentable spread and non-finite image observations before native fitting.
- Preserve the 12-inlier minimum, 3-pixel residual threshold, positive world-depth threshold, and 10% horizontal and vertical final image coverage.
- Keep provisional fitting distinct from final acceptance: a provisional pose can guide search without authorizing map insertion.
- Keep all candidate scores on the same original observation population and preserve independent seed copies.
- Bound recovery to one additional SQPnP seed and consensus optimization to two rounds. Skip both in legacy mode.

Independent synthetic cameras use a nonidentity rotation, 120 noncoplanar points and 12 fixed correspondence outliers. The default centered path preserves the same **108 true inlier rows** for all five world-coordinate changes below:

| World scale | Added world offset |
| --- | --- |
| 1 | `[0, 0, 0]` |
| 1 | `[1e9, -2e9, 3e9]` |
| `1e-5` | `[0, 0, 0]` |
| `1e5` | `[1e9, 2e9, -3e9]` |
| `1e-4` | `[1e6, -2e6, 3e6]` |

Tests compare rotation and translation in the original scene units, preserve inputs, and exercise world-seed conversion, refiner mutation/native failures, rejected recovery, consensus failure, fixed-input cost improvements, and coverage rejection. Arithmetic-only tests at `1e-200` and `1e200` units qualify the normalization helper, not the complete SLAM operating range. Both CLI modes must produce identical synthetic poses with diagnostics enabled or disabled within that mode.

## Full dashcam qualification

Input: `sample_videos/GRMN2734.MP4`, SHA-256 `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`; 1,800 frames replayed from frame zero. Settings: width 1,024, ORB 2,000, bottom mask 0, source focal 525, seed 0, OpenCV threads 1. The approximate processed intrinsics remain `fx=fy=280`, `cx=512`, `cy=288` at 1,024×576. No calibration file or independent trajectory truth is available.

| Measure | Previous uncentered baseline | First centered attempt | Centered consensus refitting |
| --- | ---: | ---: | ---: |
| Accepted poses | 1,180 | 954 | **1,685** |
| Baseline accepted frame IDs retained | 1,180 | Not qualified | **All 1,180** |
| Additional accepted frame IDs | — | — | **505** |
| Lost frames | 616 | 842 | **111** |
| Last accepted source frame | 1,198 | 959 | **1,709** |
| Lost frames within 850–1,000 | 4 | 43 | **1** |
| Final landmarks | 38,121 | 31,146 | **49,558** |

The new run's first loss is frame 990; tracking still ends before the video finishes. Increased coverage and retained points do not establish more accurate reconstruction. The clip was used during development and is **not a held-out validation sequence**. Broader default qualification needs independently calibrated/labelled sequences and Linux execution.

Ignored local evidence: baseline `output/conditioning-default-2026-09-15/`; experimental replay `output/centering-consensus-experiment/`; production replay `output/centering-qualified-2026-09-15/`; fixed-input investigation and prototype scripts `output/centering-investigation/`. Each replay records source hashes, input identity, settings and per-frame results. Diagnostic capture and overlapping validation affect timing; no speed improvement is claimed.

Other full-video probes failed to retain baseline coverage: always comparing an SQPnP alternative (179 poses), SQPnP recovery alone (958), AP3P hypothesis generation (2), VVS as primary refiner (930), and exact binary scaling (944). They informed the investigation and are not production modes. The retained method addresses the stale consensus directly while preserving the existing hypothesis generator and thresholds.

## Final verification

The production replay in `output/centering-qualified-2026-09-15/` matches every experimental per-frame outcome/count and every final optimized pose exactly. All 1,683 `tracking` frames report centered coordinates; the other two accepted poses are the initialization pair. Its final-fit selections are 1,418 second-round consensus fits, 256 first-round fits and 9 initial LM fits. The separate `coverage-comparison.json` verifies retention of all 1,180 baseline accepted frame IDs.

Production wall time was 367.03 seconds, versus 244.74 seconds for the previous default replay. Both are uncontrolled runs with different successful mapping workloads and overlapping checks. More tracking work and bounded extra refinements have a real runtime cost; this is not a speed improvement.

All **50 tests** pass in the local macOS environment; `pip check` and `git diff --check` pass. The rebuilt installed command ran from `/private/tmp` with no conditioning flag on 25 synthetic frames, returned 24 poses and 3,058 landmarks, and reported centered fitting. Native Linux execution remains unverified. The benchmark comparison helper gained explicit frame-set comparison after the production replay began; `coverage-comparison.json` was generated afterward from its unchanged report and the prior baseline.

## Remaining limitations

Centering cannot recover precision already lost in input coordinates or make degenerate geometry observable. Extreme origins still introduce rounding when converting back to world coordinates; very small map units still encounter the existing absolute depth threshold. This change does not normalize the bundle-adjustment graph.

Next work remains real calibration, distributed static-scene support, geometrically validated recovery beyond the 30-frame projection-recency limit, robust refinement losses and held-out trajectory evaluation. Do not infer covariance or road accuracy from this coverage benchmark.
