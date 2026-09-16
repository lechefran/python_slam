# Spatial support and landmark replenishment

The subsequent [robust PnP change](robust-pose.md) adds residual weighting and
Jacobian checks. Use `--no-robust-pnp` when reproducing this earlier benchmark.

## Result and scope

Spatial landmark replenishment is enabled by default alongside centered pose fitting. On the requested full dashcam replay it preserves **all 1,685 previously accepted frame IDs**, adds **111**, and reaches **1,796 accepted poses**. Tracking continues through the final source frame, 1,799, with no lost frames after initialization.

The change targets missing map support outside dense image bands. It retains the existing ORB detector, 2,000-feature budget, descriptor thresholds, PnP settings, calibration, and geometric acceptance gates. It does not classify objects as stationary or establish real-road trajectory accuracy.

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --max-frames 1800 --report output/spatial-run.json

.venv-portability/bin/python -m scripts.benchmark_tracking output/spatial-benchmark \
  --focus-start 850 --focus-end 1799 --every 100 \
  --baseline output/spatial-reference-2026-09-15/report.json
```

Use fresh output paths. `--no-spatial-mapping` selects the preceding 1,685-pose centered baseline. `--spatial-mapping` is an explicit positive alias. Python callers can use `SLAM(..., spatial_mapping=False)` for the reference behavior. To reproduce the earlier uncentered 1,180-pose baseline, disable both spatial mapping and conditioning.

## What was missing

The old image-coverage gate uses the extrema of accepted pixel coordinates. A large bounding box can be supported by very few isolated observations: in the preceding run at frame 1,704, 37 of 38 validated points occupy one quarter-height image band.

The broader extraction pool already contains features beyond the main inlier band. For example, at frame 850 the original 2,000 features have about 6.01 effective occupied cells, but the 240 pose inliers have only 3.11. Changing extraction alone did not resolve this loss of usable map support.

The ordinary mapping pass triangulates against the most recent accepted view at least 0.15 seconds older. Some distant features lack sufficient parallax at that separation. An additional older view can yield usable two-view geometry without changing the camera calibration or lowering the parallax threshold.

## Focused mapping change

After a camera passes final PnP validation and the ordinary mapping pass:

1. Count its associated landmarks in a **4×4 image grid**.
2. Identify cells with unused image features and fewer than **four associated landmarks**. Empty image regions do not create a requirement to invent features.
3. Select at most **one additional accepted view**, the most recent one at least **0.45 seconds older**, excluding the ordinary triangulation partner.
4. Apply the existing unique ORB/Hamming matching rules. Consider only feature slots unassociated in both views and belonging to an under-supported current-image cell.
5. Check the existing two-camera geometry: finite homogeneous conversion, positive depth in both cameras, at least **1° parallax**, and at most **3 pixels reprojection error** in either image.
6. Reserve geometrically valid proposals in descriptor-evidence order until each selected cell reaches four landmarks. Insert through the existing reciprocal-observation path.

The longer time separation is an opportunity for more parallax, not proof of it. Coincident cameras still produce no new points. A single cell cannot consume the entire pass: the additional support target is bounded by 16 cells × 4 points, and existing associations reduce that allowance. Existing landmarks and poses are not moved by this helper; the normal guarded bundle adjustment may subsequently optimize them.

The extra pass executes only after an accepted pose. It cannot turn a failed PnP estimate into a committed camera, silently reset the map, or populate cells by relaxing geometric checks. It performs one extra descriptor search at most; accepted proposals reuse the existing triangulation/insertion path, including a second small geometry check before insertion.

Per-frame `spatial_replenishment` diagnostics record the chosen reference ID, candidate count, geometry-pass count, inserted points and skip/evaluation status. Its work is included in the existing triangulation stage timer.

## Better support diagnostics

`geometry.spatial_support` measures finite, in-image pixels in normalized image coordinates. It is descriptive and does not add a hard acceptance gate:

| Field | Meaning |
| --- | --- |
| `grid_counts`, `occupied_cells` | Counts and occupancy of a 4×4 grid, indexed by image row then column |
| `largest_cell_fraction` | Fraction of observations in the most populated cell |
| `effective_cells` | `1 / sum(p_cell²)`; equals the occupied-cell count for a uniform distribution and approaches one for a dominant cluster |
| `central_90_span_fraction` | Horizontal/vertical 95th-minus-5th percentile spans; a few extrema cannot inflate this as easily as a bounding box |
| `minor_axis_std_fraction` | Square root of the smaller eigenvalue of the normalized 2D population covariance; exposes thin diagonal bands as well as horizontal ones |
| `outside_or_nonfinite_count` | Explicit count of inputs excluded from the image-support calculation |

No one metric is a complete observability or uncertainty test. Grid boundaries can split a thin band across cells; covariance can respond to extreme points. Read concentration, robust span and the minor axis together. A well-distributed set can still contain moving objects or incorrect associations.

Reports include feature support and each inspected pose candidate's inlier support. Sampled PnP overlays draw the grid and show occupied-cell and peak-concentration labels alongside the existing bounding box. CSV output includes concentration, central spans and minor-axis spread. Benchmark comparisons summarize **the same accepted frame IDs in both runs**, preventing a changed evaluation population from masquerading as a spatial improvement.

## Full dashcam evidence

Input: `sample_videos/GRMN2734.MP4`, SHA-256 `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`. Replay: frames 0–1,799; width 1,024; ORB 2,000; bottom mask 0; source focal 525; seed 0; OpenCV threads 1. Processed intrinsics are still the approximate 1,024×576 matrix with `fx=fy=280`, `cx=512`, `cy=288`.

| Measure | Centered baseline | Spatial replenishment |
| --- | ---: | ---: |
| Accepted poses | 1,685 | **1,796** |
| Baseline accepted frame IDs retained | 1,685 | **All 1,685** |
| Lost frames | 111 | **0** |
| Last accepted source frame | 1,709 | **1,799** |
| Final landmarks | 49,558 | **51,030** |

Initialization still needs two usable views. The initial reference at frame 0 is retained when frame 5 initializes the map; frames 1–4 have no accepted pose. All frames from 6 onward track successfully in this run.

Spatial comparison uses the **1,683 common tracking frames**; the initialization pair is excluded. Medians below come from the final PnP inliers on those same source frame IDs:

| Support measure | Baseline | Spatial replenishment |
| --- | ---: | ---: |
| Effective cells | 3.311 | **3.498** |
| Largest-cell fraction | 43.33% | **41.82%** |
| Central-90% horizontal span | 47.85% | **53.14%** |
| Central-90% vertical span | 13.02% | **15.52%** |
| Minor-axis standard deviation | 3.718% | **4.748%** |

These are tracking-coverage and image-distribution results, not calibrated uncertainty or trajectory accuracy. The inspected overlays still contain support on vehicles and near the hood. The clip was used during development; independent static-scene labels, real calibration and held-out trajectories remain necessary for accuracy qualification.

### Experiments retained as evidence, not application options

A global grid-balanced ORB selector improved feature spread but retained only 1,239 poses. It also displaced coarse pyramid features: a frame-850 extraction probe kept 29 level-7 features instead of the original 118. Balancing within the pyramid allocations improved inlier-distribution metrics on common frames, but retained only 947 poses. Both feature-selector experiments were removed from the application; their ignored experiment scripts/results remain available locally.

This investigation is consistent with the importance of per-level feature allocation in [OpenCV's ORB interface](https://github.com/opencv/opencv/blob/4.x/modules/features2d/include/opencv2/features2d.hpp) and spatial feature distribution in [ORB-SLAM3's extractor configuration](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/Monocular/EuRoC.yaml). The retained correction changes landmark replenishment, without replacing the native ORB frontend.

## Validation and artifacts

- Native synthetic cameras demonstrate that distant points rejected at the shorter baseline can be reconstructed from the older accepted view, without moving existing poses/points or breaking reciprocal observations.
- Zero-baseline data creates no landmarks despite time separation; twelve valid proposals in one empty cell insert only four.
- Distribution tests cover uniform occupancy, resolution invariance, outlier-inflated bounding boxes, diagonal thin bands, empty input and explicitly excluded pixels.
- CLI regression tests exercise the legacy solver, centered fitting, and centered fitting with spatial replenishment; diagnostic capture must preserve poses within each mode.
- A full diagnostics-only replay reproduces every preceding centered outcome/count and every optimized pose exactly.

Ignored evidence: `output/spatial-reference-2026-09-15/`, `output/spatial-replenishment-experiment/`, `output/spatial-qualified-2026-09-15/`, and `output/spatial-investigation/`. Reports preserve source hashes, environment and input/configuration identity. Additional matching/triangulation and successful mapping have runtime costs; no speed improvement is claimed. Native Linux execution remains unverified.

### Final verification

The production replay exactly reproduces every experimental per-frame outcome/count and every final optimized pose. Its feature counts also match the original frontend on all 1,800 input frames. The extra pass inserted **1,460 points across 837 frames**, with a maximum of **nine new points in one frame**; 1,786 frames evaluated the older reference and eight early tracking frames had none available.

Single-run wall times were **385.06 seconds** for the diagnostics-equipped centered reference and **439.93 seconds** for spatial replenishment. These runs include different amounts of successful mapping and overlap other checks; they do not establish a controlled performance ratio. The extra work is a cost, not a speed claim.

All **59 tests** pass, along with dependency consistency and diff checks. The installed command runs from outside the checkout with both defaults enabled; its runtime module contents match the final source files. The only runtime edit after the full replay was an explicit boolean dtype for an empty feature mask, covered by a dedicated regression. The installed synthetic smoke run also exercises the final package.

## Follow-up work

- Validate static-scene support explicitly; image distribution alone does not remove hood, vehicle or other moving-object contamination.
- Obtain real calibration and held-out trajectory/landmark truth before treating the improved coverage as improved reconstruction accuracy.
- Evaluate uncertainty and spatial information, rather than promoting the descriptive grid statistics into arbitrary hard gates.
- Profile the extra triangulation search on longer sequences; the existing historical-map resource and sustained-loss recovery limitations remain.
