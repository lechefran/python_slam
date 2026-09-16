# Validated pose refinement

The subsequent [numerical-conditioning implementation](numerical-conditioning.md) enables centered fitting with bounded seed recovery and consensus refits by default. It retains all 1,180 accepted baseline frame IDs and adds 505 on the full dashcam replay. This document preserves the preceding uncentered refinement baseline.

## Frame-941 correction

The tracker now validates the actual pose returned by RANSAC, evaluates LM refinement on the same complete set of correspondences, and makes at most one VVS refinement attempt when LM fails the stage's acceptance gates or increases the capped reprojection cost relative to a valid initial pose. Each refiner receives its own copies of the original rotation/translation arrays. Rejected updates cannot overwrite the fallback seed or enter the map.

The **12-inlier**, **3-pixel residual**, **positive-depth**, **finite SE(3)** and **10% image width/height** requirements are unchanged. Provisional PnP still skips the final coverage requirement solely to guide map search. An independently validated RANSAC pose may be retained if refinement fails; this is recorded explicitly as `ransac_pose`, not as successful LM/VVS refinement. If every candidate is invalid, tracking remains lost.

No feature detector, RANSAC sampling settings, calibration, map-recovery age rule, or optimization backend changed in this correction.

## What the original evidence actually meant

The original frame-941 diagnostic snapshot contains **103 3D-to-2D inputs** for final PnP. Its RANSAC **mask** has 96 rows spanning 77.93% of image width and 11.78% of height. That row mask is not the same thing as the set of observations which pass reprojection checks under the returned pose.

Replaying the saved numerical inputs with seed 0 produces:

| Candidate | Validated inliers | Horizontal span | Vertical span | Capped squared error, px² | Existing final gate |
| --- | ---: | ---: | ---: | ---: | --- |
| Returned RANSAC pose | 33 | 6.38% | 6.70% | 735.57 | Reject: image coverage |
| LM refinement | 59 | 29.31% | 5.67% | 511.67 | Reject: image coverage |
| VVS refinement, same original seed/subset | 97 | 77.93% | 11.78% | 214.92 | Accept |

Thus LM improves the returned initial pose, but still finds a fit with inadequate spatial support. The prior description of “96 to 59 inliers” compared a RANSAC row mask with revalidated geometry. It did not demonstrate deterioration of an already-valid 96-inlier pose. The new diagnostics make this distinction explicit.

OpenCV offers both LM and VVS refinement. Its documentation describes LM's rotation update as a perturbation and VVS's update through the exponential map. They may converge differently from the same initial hypothesis; neither guarantees the true camera pose. The local frame-941 replay establishes the usefulness of the alternate attempt here. [OpenCV pose refinement](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html).

## Candidate validation and selection

For each candidate, independently project all input world points using `T_cw`. Reject non-finite transforms and nonpositive depth before residual filtering. Recompute the inlier row indices, image spans, and error statistics. The RANSAC fitting subset stays fixed across LM/VVS, while acceptance and scoring use the same **complete** input set.

Candidate cost is `sum(min(pixel_error, 3)**2)` for the default threshold, with invalid-depth/projection rows charged the full 9 px² penalty. This avoids comparing costs on different surviving subsets or letting invalid points disappear from the score. It is a robust ranking score, **not a robust loss inside the native refiners**, which still minimize ordinary reprojection error on the RANSAC subset.

LM remains the normal path when it passes validation and does not increase cost over a valid initial hypothesis beyond numerical tolerance (`max(1e-8, 1e-6 * initial_cost)`). Otherwise one VVS attempt starts from the original RANSAC pose. The lowest-cost eligible candidate is selected from those evaluated. Both native refiners use their existing bounded default iteration criteria. Native refinement errors and non-finite outputs are recorded; no unbounded retries or silent identity poses are introduced.

This guarantees the stated acceptance checks and candidate-selection policy. It does not prove observability, global optimality, static-scene consistency or calibrated pose uncertainty.

## New diagnostic fields

Each completed PnP stage now includes `refinement`:

- `selected`: `lm`, `vvs`, `ransac_pose`, or null if no candidate passes.
- `inspected`: candidate retained for diagnostics when the stage fails.
- `fallback_attempted` and `reason`: why an alternate refinement was needed.
- `candidates`: actual support, coverage, capped cost, residual distributions, pose validity and any native error for each evaluated hypothesis.

Numeric frame snapshots additionally retain each candidate's `T_cw` and revalidated row indices. Overlays explicitly label the RANSAC mask and the selected/inspected candidate. The benchmark summary counts refinement methods and fallback reasons. Snapshot coordinates remain the pre-BA map coordinates.

## Regression and real-video validation

The small [frame-941 fixture](../tests/fixtures/frame941_pnp.json) preserves the original XYZ/pixel inputs, camera matrix, frame number and source video hash. It contains no video frames and is a numerical regression, not ground-truth trajectory data. Run:

```sh
.venv-portability/bin/python -m pytest tests/test_refinement.py -q
```

Tests cover the native frame-941 fit, non-finite/native-error/behind-camera LM outputs, preservation of the original fallback seed despite in-place mutation, cost regression, rejection of an invalid returned RANSAC pose despite a large mask, and retention of an independently validated initial hypothesis. Existing thin-band tests still reject insufficient coverage; synthetic tests independently verify projection and pose correctness and diagnostic passivity.

The repaired 1,001-frame replay is in ignored `output/refinement-window-2026-09-15/`. With the same approximate calibration, image dimensions, ORB settings, seed and thread count as the earlier diagnostic run:

| Focus window 850–1,000 | Before | After |
| --- | ---: | ---: |
| Tracking frames | 86 | 147 |
| Lost frames | 65 | 4 |
| Accepted poses across all 1,001 processed frames | 932 | 993 |
| First loss inside the focus window | 923 | 892 |

The new window's earlier isolated loss at 892 is a regression to retain in evaluation. Frame 941 tracks with 139 accepted inliers and 37.18% vertical coverage in the new sequential run. That differs from the isolated 97-inlier replay because earlier accepted/refined poses change subsequent map geometry and associations. The new window run took 164.00 seconds versus 148.57 seconds for the earlier instrumented baseline; these are single, uncontrolled runs, not a performance claim.

### Full 1,800-frame run

The full run completed with **1,180 accepted poses**, **616 lost frames**, and **38,121 landmarks**, versus 932 / 864 / 30,371 in the original baseline. The last accepted frame moved from **940 to 1,198**; all frames from 1,199 onward remained lost. The 1,001-frame prefix exactly reproduces the new window run's outcomes and counts. Bundle adjustment accepted 230 updates and rejected 6. No acceptance threshold was widened.

Full-run wall time was **233.99 seconds**, versus 178.30 seconds in the earlier run. The workload now includes more accepted poses/BA solves and diagnostic capture, and these are uncontrolled single runs; the result is not a speed improvement. Calibration remains approximate and real-road accuracy remains unqualified.

Local evidence: `output/refinement-full-report.json`, `output/refinement-full-frames/`, and `output/refinement-comparison.json`. Reproduce using the direct CLI command from the [diagnostic guide](tracking-diagnostics.md), adding `--no-condition-pnp --no-spatial-mapping`, changing `--max-frames` to `1800` and choosing fresh output paths.

All **26 regression tests** pass in the clean macOS environment; dependency consistency and diff checks pass. The rebuilt installed command was also run from `/private/tmp` on 25 synthetic frames with diagnostics enabled, exiting 0 with 23 accepted poses and the new refinement evidence present. Linux execution remains unverified.

## Additional refinements to evaluate next

1. **Numerical conditioning of the 3D inputs.** Implemented and enabled by default with bounded consensus refits, origin/scale tests and consistent solver/world conversion. [Current qualification](numerical-conditioning.md) retains all baseline dashcam frame IDs; calibrated, held-out accuracy and Linux execution remain pending.
2. **Static-scene and spatially distributed support.** The earlier overlay contains inliers on the vehicle ahead and near the horizon. Evaluate landmark persistence, spatial balancing and dynamic-object contamination with labelled evidence. Low residual alone does not establish a stationary landmark. Avoid treating image-box span as a complete conditioning test.
3. **Robust refinement weights and uncertainty.** Compare noise-aware/robust residual weighting and pre/post-refinement conditioning against this classical baseline on held-out sequences. A capped candidate-ranking score does not implement these weights or provide covariance.
4. **Recovery after sustained loss.** The 30-frame projection-candidate expiry rule remains. A separate bounded keyframe-relocalization change must recover geometrically validated support after that window; refinement cannot recover absent correspondences.

Real camera calibration and independent trajectory truth remain required before claims of real-road accuracy. These recorded-video results measure tracking coverage and internal consistency only.

## Centered consensus follow-up

The default now combines centered/scaled fitting, bounded failed-seed recovery and up to two validated consensus refits. It retains all 1,180 baseline accepted frame IDs on the full requested dashcam replay and adds 505. [Current implementation and qualification](numerical-conditioning.md) supersede the earlier opt-in conditioning decision; the measurements above remain the historical uncentered refinement baseline.
