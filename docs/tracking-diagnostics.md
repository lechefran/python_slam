# Tracking-loss investigation

## Follow-up: validated pose refinement

[The refinement correction](pose-refinement.md) adds independently checked pre/post-fit candidates and a bounded VVS fallback. It also corrects the interpretation of the original frame-941 evidence: the 96-row RANSAC mask does not imply that the returned pose has 96 validated inliers. That pose has only 33; LM reaches 59 and the alternate VVS fit reaches 97 on the saved input. The measurements below remain the original diagnostic baseline, before that correction.

## Reproduce the accumulated failure

Run from the repository root in an installed environment:

```sh
.venv-portability/bin/python -m scripts.benchmark_tracking output/tracking-investigation
```

The default video is `sample_videos/GRMN2734.MP4`. The command processes source frames **0–1,000 inclusive**, then summarizes **850–1,000 inclusive**. Starting a fresh tracker at frame 850 would build a different map; it does not reproduce the accumulated failure. Capture is sampled every 10 frames, plus status transitions and the last frame. All processed frames retain scalar diagnostic metrics in `report.json`, including warm-up and unsampled frames.

Use a new directory for each experiment. Optional benchmark arguments:

- `--video PATH`: a different input clip; its SHA-256 is recorded.
- `--focus-start N --focus-end N --every N`: change the inclusive analysis window and capture stride; warm-up still starts at zero.
- `--calibration PATH` or `--focal N`: use supplied pinhole calibration or an explicitly approximate source focal length.
- `--baseline PATH`: compare per-frame outcomes with a prior report from the same input/calibration/configuration. An incompatible comparison is labelled, not interpreted as regression evidence.

The baseline settings are explicitly fixed at width 1,024, 2,000 ORB features, no bottom mask, seed 0, and one OpenCV worker. The script overrides `SEEK`/`F` defaults. Exit 0 means a map initialized and the full requested window was processed, even if frames were lost. The summary reports losses separately. A short clip that ends before the requested window completes returns nonzero.

For direct CLI use with different tracker settings:

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --start-frame 0 --max-frames 1001 \
  --report output/custom-tracking-report.json \
  --diagnostics-dir output/custom-tracking-frames \
  --diagnostics-start 850 --diagnostics-end 1000 --diagnostics-every 10
```

## Evidence files

| File | Contents |
| --- | --- |
| `manifest.json` | Exact runner arguments, source SHA-256 hashes, warm-up and focus window |
| `report.json` | Video/calibration hashes, environment, effective camera/configuration, accepted poses and per-frame evidence |
| `summary.json` | Window completeness, states, first loss/last tracking frame, rejection stages/gates, optional baseline comparison |
| `window.csv` | One row per focus frame, including failed frames and partial estimator results |
| `frames/index.json` | Captured frame IDs, status, image and evidence filenames |
| `frames/frame-NNNNNN.png` | Four panels showing descriptor matches, seed PnP, map search, and final PnP |
| `frames/frame-NNNNNN.json` | Feature/match pixels, PnP input XYZ/pixels and IDs, RANSAC/refined row indices, candidate transforms and projections |

All generated evidence stays under ignored `output/`. There is no new model download, feature algorithm, or threshold change. The benchmark script runs from the checkout; the installed application also includes the diagnostic module.

## Interpret the measurements

**Descriptor filtering:** query/train feature counts, two-neighbor KNN support, ratio-test survivors, distance-gate survivors, and one-to-one matches. A descriptor match is not necessarily a mapped 3D landmark or a geometric inlier.

**Provisional PnP:** previous-frame matches with an established landmark become 3D-to-2D inputs. Evidence records input count, RANSAC support, refined support, positive-depth projection count, residual median/p95/max, image bounding box and x/y spans. This candidate is permitted to guide projection search without passing image coverage. It is not committed to the map.

**Projection search:** records eligible recent landmarks, positive-depth projections, in-image projections, projections with nearby/available features, and new descriptor associations. Counts refer to candidate landmarks, not necessarily distinct nearby pixels. A failed provisional estimate prevents this stage from running; missing evidence means “not attempted,” not zero visible landmarks.

**Final PnP:** runs on the retained seed inliers plus new map associations. It must pass the existing 12-inlier, positive-depth, 3-pixel residual and 10%-of-width/height span requirements. The metrics preserve supporting inliers even when the coverage gate rejects the whole pose. The older top-level `inliers` field continues to count accepted support, so it remains zero on a lost frame; inspect `diagnostics.stages.final_pnp.refined_inliers` for a rejected estimate's support.

**Coordinate and time contracts:** XYZ is in world coordinates at arbitrary monocular scale; `T_cw` maps world into camera; pixels use the processed, rectified image. Numeric snapshots are captured before that frame's bundle adjustment and culling, so later optimized map poses may differ. A rejected transform is labelled as candidate evidence, not part of the accepted trajectory. Invalid projections serialize as JSON null. Green overlay points mean geometric inliers, not acceptance of the camera. Residual statistics exclude invalid values and record their count.

Core processing time includes collecting diagnostic statistics/snapshots. `artifact_write` measures sampled JSON/PNG generation separately and is present in the main report, after writing a sample. Both add overhead. Neither run is an accuracy measurement or a controlled speed comparison. Individual timers cover extraction, descriptor matching, each PnP stage, projection search, triangulation, and combined optimization/culling; they are not a complete profiler of decoding and memory.

## Measured dashcam findings — 2026-09-15

Local run: `output/tracking-diagnostics-2026-09-15/`, using the default benchmark settings and `--baseline output/runtime-repair/dashcam-full.json`. Input SHA-256: `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`. Environment: macOS 27 arm64, Python 3.14.7, NumPy 2.5.3, OpenCV 5.0.0, g2opy 2.3.0. Intrinsics remain approximate; no ground truth was supplied.

The run processed **1,001 frames**, with **932 accepted poses** and **30,371 landmarks**. The 151-frame focus window contains **86 tracking / 65 lost** frames. It wrote **20 overlay/evidence pairs**. Every per-frame outcome/count in the 1,001-frame prefix matches the prior run exactly; an additional comparison also found the entire accepted pose list identical. In this case that comparison is meaningful because the earlier 1,800-frame run accepted no poses after frame 940. Total instrumented wall time was 148.57 seconds, including 1.52 seconds of artifact writing; this is not a speed comparison.

### 1. Initial rejection is spatial support, despite many features and low residuals

At source frame **923**, timestamp **30.7974 seconds**:

- ORB detected 1,985 features; descriptor filtering retained 996 unique matches.
- Only 115 matches referred to landmarks in the accepted reference frame 922.
- Seed PnP retained 83 refined inliers. Projection search saw 782 in-image candidates and added 49 matches.
- Final PnP used 132 inputs and retained **123 refined inliers**; median residual **1.03 px**, p95 **2.69 px**.
- Their bounding box spans **54.24% of width / 9.33% of height**. The existing 10% height gate rejects the pose. This is 53.75 vertical pixels against a 57.6-pixel minimum.

The frame-923 overlay visibly places much of the supporting evidence near the horizon and on the leading vehicle. Feature extraction is still productive. A count-only diagnosis would miss the spatial concentration. The low residual measures internal agreement; it does not prove correct camera motion or static landmarks.

### 2. Refinement narrows support before sustained loss

Frames 924–935 resume tracking. Frames 936–939 are rejected for coverage; frame 940 briefly succeeds. At **941**, the seed estimate has 82 refined inliers spanning 77.87% / 11.78% of image width/height. Projection search adds 21 matches, giving 103 final inputs.

The final RANSAC set has **96 rows** spanning **77.93% / 11.78%**. After LM refinement and the existing positive-depth/residual check, **59 rows** remain, spanning only **29.31% / 5.67%**. That final candidate fails coverage. These before/after spans can be independently recomputed from the saved `ransac_rows`, `refined_rows` and pixels. They identify a concrete stage to investigate; they do not establish why the optimizer concentrates on that subset.

Across all 65 lost frames in the window:

| Rejection stage / gate | Frames |
| --- | ---: |
| Final PnP / image coverage | 49 |
| Provisional PnP / fewer than 12 map correspondences | 9 |
| Provisional PnP / RANSAC support | 5 |
| Provisional PnP / refined pose support or validity | 1 |
| Final PnP / RANSAC support | 1 |

All 49 coverage rejections fail the vertical-span gate; 31 also fail horizontal span. There is no feature-extraction crash or native-optimizer exception in this run.

### 3. A stale reference and candidate expiry limit recovery

After frame 940, no further pose is accepted in this window. Each failed frame retries descriptor matching against frame 940. Its map-correspondence count falls from **88 at frame 941** to **12 at frame 1,000** while ordinary image features remain plentiful.

Projection candidates must have an accepted observation within 30 source frames. There are 871 eligible unseeded candidates at 941, 91 at 970, and **zero at 971**. No point can satisfy the age rule once the latest accepted observation at 940 is 31 frames old. Search therefore cannot broaden the seed support after this boundary, even when provisional PnP still succeeds. By frame 984 some attempts cannot collect 12 map inputs at all. This is a verified consequence of the current last-reference/recency policy, not evidence that the whole stored map is empty.

### What this establishes and the next focused fixes

The immediate rejection gate, the refinement-stage reduction in spatial support, and the later expiry of all projection candidates are reproduced and observable. The investigation does **not** yet prove whether inaccurate intrinsics, moving-object landmarks, weak depth geometry, or solver sensitivity is the principal upstream cause. Visual concentration on a vehicle is evidence to investigate, not a validated dynamic-object label.

Next implementation work should:

1. Inspect static-scene versus vehicle-associated support and pre/post-refinement geometry, using the captured frame-941 correspondences. Preserve the coverage safeguard while testing better spatially distributed map support and robust refinement.
2. Add a bounded recovery search against suitable accepted keyframes/landmarks that can still operate after 30 frames of loss. Require independent geometric validation before updating the map; do not simply accept the stale seed.
3. Evaluate with real camera calibration when available, then repeat the same warm-up, focus window and full clip. Require fewer losses **and** maintained geometric validity; more accepted poses alone are not sufficient.

This change adds diagnostics and regression coverage only; it does not claim to fix tracking loss or complete the broader CAM-08/QA-02 acceptance criteria.

## Regression checks

The tests independently construct a noncoplanar point cloud whose image observations fill a thin horizontal strip. All 80 points can have excellent reprojection agreement and still fail final image coverage; diagnostics must retain those inliers and the failed gate. A separate insufficient-support case checks early failure evidence. Blank-frame integration checks that failed estimates keep the map unchanged and later stages remain absent.

A deterministic 25-frame synthetic video is processed with diagnostics disabled and enabled. Per-frame outcomes and the complete native-optimized pose list must agree exactly on the same platform/configuration. Saved XYZ, pixels and `T_cw` are independently reprojected to verify the recorded inlier rows. The tests also validate capture range/stride, image output, and refusal to overwrite a previous benchmark directory.

Run `.venv-portability/bin/python -m pytest -q` for the complete suite. The diagnostic implementation passed all **19 tests** on the local macOS environment, with `pip check` and `git diff --check` also passing. After rebuilding the package, the installed `python-slam` command ran from `/private/tmp` on 25 synthetic frames with diagnostics enabled, exited 0 and retained 23 poses. Linux execution remains unverified.
