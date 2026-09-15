# SLAM improvement TODO

Detailed implementation review and development backlog, **2026-09-15**.

## Runtime repair update

The first repair pass now runs in the local macOS environment, including a clean dependency installation, native optimizer tests, a headless synthetic/video pipeline, and an observed native Matplotlib window. Setup, supported Linux targets, CI/container instructions, and precise validation limits are in [README.md](README.md) and [runtime validation](docs/runtime.md).

Implemented work includes parser/import/API repairs, optional visualization, resized camera intrinsics, two-camera triangulation and cheirality checks, robust map-based PnP, reciprocal observations, guarded native BA and culling, CLI/reporting, and regression tests. The full requested dashcam run decoded 1,800 frames with 932 accepted poses; later tracking loss remains explicitly reported. Linux wheel resolution passed, but Linux execution and remote CI are not yet verified.

The detailed findings and line numbers below preserve the **pre-repair review snapshot**, not the present source. The unchecked items describe their full acceptance scope; many are only partially addressed by this first repair. In particular, real calibration/accuracy, sustained-loss recovery, long-run resource budgets, 2D mapping and MAP-08 point-data interchange remain pending. Consult the repair notes before treating an old defect description as still reproducible.

## Tracking investigation update

The next diagnostic pass is implemented: [reproduction and findings](docs/tracking-diagnostics.md). It replays from frame zero, retains per-stage evidence on failed frames, exports sampled filtering overlays/numeric snapshots, and compares the focus window against the prior baseline.

- [x] Reproduce frames 850–1,000 with the accumulated map: all 1,001 prefix outcomes/counts matched the prior dashcam run.
- [x] Identify rejection stages: frame 923 retains 123 geometric inliers but fails vertical image coverage (9.33% versus 10% required).
- [x] Trace sustained loss: frame 941 loses spatial support during final PnP refinement; after frame 940 remains the last accepted reference, all projection candidates expire at frame 971.
- [x] Add an independent thin-band synthetic regression and verify diagnostics preserve synthetic native-optimized poses exactly.
- [ ] Improve spatially distributed, static-scene pose support and investigate refinement sensitivity; measure results without relaxing safeguards solely to increase pose counts.
- [ ] Add geometrically validated recovery that can search useful accepted keyframes/landmarks beyond the normal 30-frame projection-recency window.

This is partial progress on CAM-08 and QA-02. Upstream calibration/dynamic-object causes, recovery behavior, held-out accuracy and the full QA protocol remain unresolved.

## Pose refinement update

The [frame-941 correction](docs/pose-refinement.md) validates returned PnP poses independently of RANSAC masks, checks LM updates against fixed-input geometric support/cost, and tries one VVS refinement when needed. The original 96 RANSAC rows supported only 33 points under the returned pose; LM reached 59, and VVS reaches 97 on the same saved inputs while passing the unchanged coverage gate.

- [x] Preserve independent seed copies, reject invalid/native-failed candidates, and record which validated candidate is selected.
- [x] Add native frame-941 regression data and synthetic tests for failure isolation, cost regression and actual-pose validation.
- [x] Replay from frame zero: losses in frames 850–1,000 decrease from 65 to 4, with an earlier isolated failure at 892 explicitly retained as a regression.
- [x] Complete the full 1,800-frame comparison: 1,180 accepted poses versus 932; tracking ends at frame 1,198 instead of 940. Later loss, increased wall time and unqualified accuracy remain documented limitations. All 26 tests pass.
- [x] Implement local coordinate centring/scaling with origin/scale-invariance tests across PnP and world-pose conversion; now enabled by default with bounded consensus refinement.
- [x] Restore baseline dashcam coverage with centering: recover rejected seeds on their existing RANSAC rows, then refit validated consensus at most twice under unchanged gates. The full replay retains every one of the 1,180 baseline accepted frame IDs and adds 505, totaling 1,685.
- [ ] Extend conditioning qualification to calibrated, held-out sequences and native Linux execution; the supplied development clip does not establish trajectory accuracy.
- [ ] Evaluate spatial/static-scene weighting and robust refinement losses on labelled and held-out data; the candidate cost cap is not an optimizer loss or a covariance estimate.

The 30-frame projection expiry and broader recovery work remain unchanged. Full details and qualification limits are in the linked correction report.

## Numerical conditioning update

[Implementation and evidence](docs/numerical-conditioning.md) cover a centered/isotropically scaled solver frame through RANSAC, LM, VVS and bounded SQPnP seed recovery, with world-pose conversion before acceptance. Synthetic origin/scale cases preserve all 108 true inlier rows. Inputs and map coordinates retain their contracts; unrepresentable/non-finite inputs fail before native fitting.

The first centered experiment regressed to 954 poses. The correction addresses stale consensus: reproject all original observations, refit the validated rows at most twice, and accept only lower full-input capped cost under the existing depth, residual and coverage gates. A rejected primary fit can receive one independent SQPnP seed using the same RANSAC mask. Both changes run in centered coordinates.

**Centering is now the default.** The full requested dashcam replay reaches 1,685 accepted poses, retains **every** baseline accepted frame ID, adds 505, and extends the last accepted frame from 1,198 to 1,709. There are 111 lost frames; real-road accuracy and Linux execution remain unqualified. `--no-condition-pnp` retains the previous solver path for comparisons. See the linked report for fixed-input regressions, diagnostic fields, validation and remaining limitations.

## Purpose and scope

Improve this small Python monocular SLAM application in four connected areas: camera positioning, feature/landmark and graph-edge calculations, 2D mapping, and 3D mapping. Correctness, measured accuracy, usable operation, and sustained performance take precedence over adding algorithms.

This review covers the actual working-tree implementations of [slam.py](slam.py), [frame.py](frame.py), [point.py](point.py), [dmap.py](dmap.py), and [display.py](display.py), at base commit `5a58b6ed9b4df757f6632490db978ac3c944b933`, branch `improve-slam`. File/line references below describe this snapshot and will move as fixes land. Application source was not changed during the review.

**Status at the initial review (superseded by the repair update above): the application did not run in the available environment.** It also contained independently reproducible mathematical and observation-bookkeeping defects. Establish a runnable, geometrically correct baseline before reporting trajectory accuracy or speed improvements.

At that initial review, local `AGENTS.md` described a later macOS port, headless flags, dependency locks, and tests absent from the checkout. The renderer was SDL/Pangolin; `tests/` and `scripts/` contained no Python source; the referenced documentation and manifests were absent. Old generated files under `output/` were not evidence that the reviewed revision worked.

### Navigation

- [Evidence and real-video baseline](#evidence-and-real-video-baseline)
- [Priorities and implementation order](#priorities-and-implementation-order)
- [Coordinate and data contracts](#coordinate-and-data-contracts)
- [1. Runtime and reproducible setup](#1-runtime-and-reproducible-setup)
- [2. Camera positioning and initialization](#2-camera-positioning-and-initialization)
- [3. Image features, correspondences, and edges](#3-image-features-correspondences-and-edges)
- [4. Landmark geometry and map structure](#4-landmark-geometry-and-map-structure)
- [5. Bundle adjustment and graph edges](#5-bundle-adjustment-and-graph-edges)
- [6. Genuine 2D map creation](#6-genuine-2d-map-creation)
- [7. 3D map presentation, export, and usability](#7-3d-map-presentation-export-and-usability)
- [8. Performance and resource use](#8-performance-and-resource-use)
- [9. Relocalization, loop closure, and sensor extensions](#9-relocalization-loop-closure-and-sensor-extensions)
- [10. Tests and benchmark protocol](#10-tests-and-benchmark-protocol)
- [11. Current libraries, standards, and research options](#11-current-libraries-standards-and-research-options)
- [Definition of done](#definition-of-done)

### Terms that matter for this review

- **2D image overlay:** keypoints and match lines drawn on a video frame. This is what `Display2D` currently displays. It is not a persistent 2D map.
- **2D map:** a persistent trajectory and/or landmarks expressed in a declared plane, with origin, orientation, scale status, and map revision.
- **Occupancy map:** estimates of occupied, free, and unknown space. Sparse monocular keypoints alone do not establish free space or obstacle completeness.
- **3D map:** currently intended to be sparse world-space landmarks plus camera poses. A sparse cloud is not a dense surface, mesh, or complete reconstruction.
- **Image edge:** an intensity boundary or fitted line segment. **Optimization edge:** a measurement connecting state variables in the solver. The purple lines in the current image overlay are temporal correspondences, not extracted physical edges.
- **Bundle adjustment (BA):** jointly refining camera poses and 3D landmarks against their image observations.
- **Monocular scale:** translation is in arbitrary map units unless a separate measurement supplies metric scale. A fixed numerical baseline does not establish metres.

## Evidence and real-video baseline

### Checks actually performed

| Check | Observed result | What this establishes |
| --- | --- | --- |
| AST parse of all five modules | `dmap.py:169: IndentationError: unexpected indent`; other modules parse | There is a source-level blocker; not a dependency issue |
| Unmodified application launched on the requested video | Exit 1 at `display.py:1`, `ModuleNotFoundError: No module named 'sdl2'` | Application processes **zero video frames** in this environment |
| `.venv/bin/python -m pytest -q` | No tests ran; exit 5 | There is no executable regression suite in this checkout |
| `.venv/bin/python -m pip check` | No broken requirements; local pip-cache warning | Installed distributions are dependency-consistent; required application imports can still be missing |
| Installed native `g2opy` import and symbol checks | Import succeeds; current point/solver names differ from source | A small compatibility correction is feasible to investigate; convergence remains untested |
| Native `SBACam` known-point projection | Agrees with `K @ T_cw[:3]` when constructed from `T_wc` | Current inversion direction at this boundary should be preserved and regression-tested |
| Isolated, unchanged `triangulate()` on known cameras | Three different visible points reconstructed at the origin; changing only `pose2` has no effect | `slam.py:28–29` ignores the second camera |
| Isolated `Point` operations | Duplicate observations accepted; competing point overwrites a slot; deletion can raise `ValueError` | Reciprocal observation integrity is not enforced |
| Blank-image feature extraction | `TypeError` because corners are `None` | Ordinary feature loss is a crash path |
| Textured-image/real-video keypoint construction | OpenCV rejects `_size`; requires `size` | Frontend has an additional version-compatibility blocker |
| Pose-decomposition API probe | `np.mat` is absent in installed NumPy | Another runtime blocker after imports are fixed |

The native projection check used `T_cw` with x-translation `-1`, intrinsics `(500, 500, 320, 240)`, and world point `(1.5, 0.3, 5)`. Both the explicit projection and `SBACam` give `(370, 270)` pixels. This is a projection test, not a successful BA run.

**Observed environment, not a recommended lockfile:** macOS 27.0 arm64; Python 3.14.7; NumPy 2.5.3; `opencv-python` 5.0.0.93 / OpenCV runtime 5.0.0; SciPy 1.18.1; scikit-image 0.26.0; g2opy 2.3.0; Matplotlib 3.11.1; pytest 9.1.1; PyAV 18.1.0. Modules `g2o`, `pangolin`, `sdl2`, and `OpenGL` are unavailable. Installed `g2opy` exports `VertexPointXYZ` and `LinearSolverEigenSE3`, but not the source's `VertexSBAPointXYZ` or `LinearSolverCholmodSE3`.

### Required real-life dashcam workload

Use this file for every applicable implementation comparison:

```text
/Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4
```

| Property | Verified value |
| --- | --- |
| File size | 125,829,120 bytes |
| SHA-256 | `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c` |
| Codec and source dimensions | H.264, 1920 × 1080 |
| Decoded frames | 1,800 |
| Frame rate / stream time base | `30000/1001` fps; `1/30000` seconds |
| Stream duration | 60.06 seconds; last frame timestamp 60.026633 seconds |
| Timestamp inspection | No missing timestamps; all intervals positive and approximately 0.033366667 seconds |
| Current resize output | 1024 × 576 |
| Sampled source frames | 0, 359, 719, 1079, 1439, 1799 |
| GFTT corners at those resized frames | 642, 707, 606, 1019, 726, 735; these are pre-descriptor corners, not reliable map observations |

A single full PyAV decode took approximately **11.88 seconds** in the review probe. This includes no SLAM, no image conversion for tracking, no optimization, and no GUI. It is not application throughput or an established performance baseline.

The six inspected images show daytime highway driving, vehicles near the camera, a broad road plane, distant features, road barriers, overhead structures, and a visible dashboard/hood. These observations motivate the dashcam-specific tasks below. They do not prove that every difficult motion case or a loop closure appears in the clip. No matching calibration, synchronized ground-truth trajectory, readout-time metadata, or metric reference was established during this review.

Review artifacts are local and ignored: [probe script](output/implementation-review-2026-09-15/probe_dashcam.py), [probe data](output/implementation-review-2026-09-15/probe.json), and [sampled images](output/implementation-review-2026-09-15/dashcam-contact-sheet.jpg). These links will not exist in a fresh clone; the stable results and required protocol are recorded here. Loading both PyAV and OpenCV in the probe emitted duplicate macOS AVFoundation-class warnings; decoding still completed. Do not infer that mixed decoder imports are production-safe from this one run.

## Priorities and implementation order

All implementation checkboxes start unchecked. Review findings are not completed fixes.

| Priority | Meaning | Release gate |
| --- | --- | --- |
| **P0** | Cannot run, invalid geometry, or corruptible map state | Resolve before believing a generated trajectory or map |
| **P1** | Required for stable and explainable monocular operation | Resolve before claiming a useful real-video baseline |
| **P2** | Substantial usability, map capability, or measured efficiency improvement | Develop against a stable baseline |
| **P3** | Larger SLAM extensions or research experiments | Admit only with suitable data and demonstrated benefit |

Recommended order:

1. **Runnable foundation:** RUN-01–04; current native projection test; tiny deterministic test scene.
2. **Correct geometry and state:** CAM-01–05, FEAT-01/03/05/06, MAP-01–03, BA-01/02/05/07.
3. **Stable map-based tracking:** CAM-06–08, FEAT-02/04/07, MAP-04/05, BA-03/04/06; first complete dashcam report.
4. **Usable map outputs:** 2D-01–03, MAP-06–08, VIEW-01–04, QA-01–03.
5. **Measured speed and duration:** PERF-01–04, tested against the same dashcam configuration and longer held-out inputs.
6. **Persistent SLAM:** SLAM-01/02, then evidence-driven P3 feature/sensor/dense-mapping experiments.

The most urgent defects are the parser/import/API blockers, incorrect triangulation, stale resize calibration, ambiguous pose recovery, unsafe depth division, duplicate associations, unvalidated solver writeback, and unreachable stale-landmark culling. Installing a newer matcher would leave all of these problems in place.

## Coordinate and data contracts

Keep these visible in code docstrings, calibration files, tests, and exported metadata:

```text
Image                 shape (height, width, channels), OpenCV BGR, access image[v, u]
Pixel keypoints       shape (N, 2), values (u, v), pixels in processed image
Normalized keypoints  shape (N, 2), calibrated rays (x/z, y/z), dimensionless
ORB descriptors       shape (N, 32), uint8 for the chosen ORB configuration
Frame associations    N feature slots, each empty or one landmark
Landmark              X_w shape (3,), finite coordinates in the world/map frame
Internal pose         T_cw shape (4, 4), world -> camera
Viewer/export pose    T_wc = inverse(T_cw), camera -> world
Camera centre         C_w = -R_cw.T @ t_cw
Pinhole projection    lambda * [u, v, 1] = K @ T_cw[:3] @ [X_w, 1]
Pose composition      T_current_world = T_current_previous @ T_previous_world
```

Use float64 for reference geometry and optimization interfaces until profiling and numerical tests justify another choice. Test `R.T @ R ≈ I`, `det(R) ≈ +1`, finite translation, and homogeneous bottom row. A 4×4 array alone does not establish a valid rigid transform.

The projection equation above applies to ideal/rectified pinhole pixels. Raw distorted pixels require the selected lens model's distortion projection, as described in CAM-03.

The first camera's axes may define the initial world. That does not establish gravity, compass direction, or a road plane. For interoperability, document explicit conversion between camera optical right/down/forward and body forward/left/up axes. REP 103 and REP 105 are useful conventions for axes and separating continuous local odometry from globally corrected map poses; adopting them does not require ROS. [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst).

## 1. Runtime and reproducible setup

### RUN-01 — Repair parser and installed-API blockers [P0]

- [ ] Correct `dmap.py:168` (`for f in p.frames:g`) so the module parses. Verify the intended residual loop rather than just suppressing the exception.
- [ ] Replace removed `np.mat` use at `frame.py:96` with an ndarray and explicit matrix multiplication, and replace `_size` at `frame.py:42` with the supported OpenCV keypoint API.
- [ ] Audit actual native binding exports before changing imports, vertex classes, or solver constructors. Use a small compatibility boundary for the chosen supported g2opy version, not scattered `try/except` fallbacks or absolute `sys.path` patches.
- **Acceptance:** every source parses; importing geometry and the map backend succeeds in a clean documented environment; a real native known-point projection and constrained optimization execute. Syntax-only and import-only checks must remain labelled separately.

### RUN-02 — Make visualization optional and publish a tested environment [P0]

- [ ] Decouple SDL/Pangolin/OpenGL imports from geometry and optimization. `slam.py:10–15` and `dmap.py:5–6` currently make a GUI stack mandatory even for offline evaluation.
- [ ] Select one supported initial viewer path on macOS. A main-thread Matplotlib implementation is a reasonable simple option already installed here; measure its limits before selecting a larger viewer dependency.
- [ ] Add `pyproject.toml` with Python support bounds and core, viewer, and evaluation dependencies; produce a tested platform lock/constraints file. Avoid installing both OpenCV wheel variants that provide the same `cv2` namespace.
- [ ] Document native wheel availability and a clean install/import/test procedure. Preserve the existing MIT project license and record third-party licenses before distributing dependencies or models.
- **Acceptance:** headless operation works without GUI packages; documented installation works from an empty environment; `pip check` passes; native solver execution passes. A developer's pre-existing `.venv` is not the release artifact.

Use the current standardized `[project]` metadata format rather than hardcoding setup state in source. [Python packaging specification](https://packaging.python.org/en/latest/specifications/pyproject-toml/).

### RUN-03 — Introduce a validated CLI and explicit run state [P1]

- [ ] Replace direct `sys.argv` handling at `slam.py:142–185` with `argparse`: input, calibration, processed width, start frame/time, max frames, headless, hold-final-view, output directory, seed, and logging level.
- [ ] Validate input availability, decode success, dimensions, focal values, frame bounds, and output paths before creating a viewer. A failed `VideoCapture` currently prints a message and continues into invalid configuration.
- [ ] Put map/configuration/state in a small SLAM runner object or explicit arguments. Eliminate hidden dependence on global `map3d`, `W`, `H`, `K`, and `disp` in tests and repeat runs.
- [ ] Define precedence among configuration file, CLI, and legacy `F`/`SEEK`. Deprecate manual `REVERSE` translation-sign selection when geometric pose disambiguation is implemented.
- [ ] Return meaningful success, invalid-input, initialization-failure, and processing-failure statuses. Save a partial report with the last valid frame on controlled failure.
- **Acceptance:** `--help`, missing file, invalid calibration, zero frames, bounded run, EOF, and interruption behave predictably without windows in headless mode. Two runs in one interpreter do not share map state.

### RUN-04 — Reconcile repository documentation with executable behavior [P1]

- [ ] Replace historical Ubuntu-only setup instructions with verified supported commands and explain that the current estimator is a sparse monocular prototype.
- [ ] Add a small deterministic synthetic-camera fixture and actual checked-in tests. Remove claims that absent CLI flags, test files, or dependency locks already exist.
- [ ] Keep README capability descriptions and its link to this backlog current. Keep sample videos, captures, model weights, native build trees, and benchmark outputs ignored, with small manifests committed when appropriate.
- **Acceptance:** a fresh checkout can follow the README to install, test, and run a synthetic example; its reported capabilities match the implementation. Existing local outputs are never used as a substitute for that check.

## 2. Camera positioning and initialization

### CAM-01 — Make transform direction and pose validity executable contracts [P0]

- **Finding:** `frame.py:95–112`, `slam.py:50–56`, and `dmap.py:109–153` rely on implicit direction conventions and multiple inversions.
- [ ] Name relative transforms by source/destination and document the order passed into the estimator. `match(current, previous)` can legitimately require inversion of an estimated current-to-previous transform; the inverse is not by itself a bug.
- [ ] Centralize checked composition, inverse, projection, and camera-centre extraction. Never plot `T_cw[:3,3]` as the camera's world position.
- [ ] Store pose validity/state separately from the matrix. Identity at initialization must not silently represent a failed estimate later.
- **Acceptance:** known translation plus nonidentity rotation yields the correct composed pose, camera centre, projection, and viewer orientation. Include forward/backward movement and round trips through the actual g2opy boundary.

### CAM-02 — Correct calibration after every resize/crop [P0]

- **Finding:** `slam.py:160–161` creates `K`/`Kinv` before `slam.py:166–170` changes image dimensions. Updating scalar `F` afterward never updates the matrix used by tracking.
- [ ] Derive the processed camera matrix from the actual horizontal/vertical resize factors and crop offsets, then compute its inverse. Account for integer dimension rounding and the chosen pixel-centre convention.
- [ ] For this clip with the default `F=525`, the current code would retain `fx=fy=525, cx=960, cy=540` while images become 1024×576. Simple proportional scaling would instead give `fx=fy=280, cx=512, cy=288`; **these numbers correct resizing of the guess, not actual camera calibration**.
- [ ] Carry calibration alongside image dimensions; reject a mismatched calibration file instead of silently applying it.
- **Acceptance:** a known 3D point projects to the corresponding resized/cropped pixel under a reference image-coordinate transform; normalized rays agree before/after preprocessing. Test non-square inputs and rounded output heights.

### CAM-03 — Add real calibration and a consistent distortion path [P1]

- **Finding:** `slam.py:158–160` assumes one focal length and a centred principal point; there is no distortion model or calibration input.
- [ ] Define a versioned calibration file: source dimensions, `fx/fy/cx/cy`, lens model, distortion coefficients, calibration date/provenance, and recording mode. Reject non-finite or implausible matrices.
- [ ] Provide checkerboard/ChArUco capture and offline fitting instructions, with varied board orientations and coverage of image edges. Preserve board dimensions and units; test on held-out calibration images.
- [ ] Choose pinhole versus fisheye from the lens and residual evidence. Use either rectified-image pixels with the rectified matrix or consistently undistorted normalized points; do not undistort twice or mix raw and rectified residuals.
- [ ] Precompute image-remap tables when rectifying images. Save valid-pixel masks and the exact matrix for the output view. Verify whether digital stabilization/cropping changes the camera model over time.
- **Acceptance:** a calibration report includes per-view and spatial residuals, outlier images, held-out performance, and processed-image metadata. An approximate `F` run is visibly and machine-readably marked uncalibrated.

The OpenCV calibration model explains intrinsics/distortion and the ChArUco workflow supports partial board observations. Treat both as implementation references, not evidence that the dashcam has been calibrated. [Calibration model](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html), [ChArUco calibration](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html).

### CAM-04 — Replace heuristic essential-pose recovery [P0]

- **Finding:** `frame.py:78–82` fits `FundamentalMatrixTransform` on normalized coordinates and then decomposes it like an essential matrix. This can estimate an epipolar relation, but the estimator does not enforce all calibrated-essential constraints. `extractRT()` selects rotation from trace and translation sign from `REVERSE`, without checking point depth in both cameras.
- [ ] Use a calibrated essential-matrix estimator on a declared coordinate domain; evaluate all returned models where the binding can return multiple candidates.
- [ ] Recover rotation and translation direction through cheirality: finite triangulated points must lie in front of both views. Preserve the robust-estimator and pose-recovery masks through all feature indices.
- [ ] Verify essential singular-value structure, proper rotation, finite translation, spatial support, and sufficient triangulation quality. Return an explicit failure/degeneracy reason when candidates are ambiguous.
- [ ] Keep any compatibility implementation testable against a trusted library and analytic synthetic scenes. Do not fix the sign using the expected driving direction.
- **Acceptance:** noncoplanar noisy points with controlled outliers recover the known pose up to monocular scale; pure rotation, insufficient support, and ambiguous planar cases do not create confident translation or map points.

### CAM-05 — Introduce a deliberate monocular initialization state machine [P1]

- **Finding:** `slam.py:45–56` assumes the second image and subsequent early frames can immediately establish motion; `frame.py:72` treats eight matches as an assertion.
- [ ] Use `UNINITIALIZED`, `INITIALIZING`, `TRACKING`, `LOST`, and optionally `RELOCALIZING`; log every transition and its evidence.
- [ ] Retain candidate initialization frames until adequate baseline, inlier distribution, positive-depth support, and parallax exist. Compare homography/essential evidence using compatible noise models; road-plane dominance and pure rotation need explicit handling.
- [ ] Require a usable initial landmark set and successful constrained refinement before committing the first map. Choose and record an arbitrary scale gauge consistently.
- [ ] Treat far-away features near the forward-motion epipole as weak depth evidence even if their descriptors match well. Use longer-baseline keyframes when adjacent dashcam frames provide little parallax.
- **Acceptance:** stationary/rotation-only clips wait without fabricating a map; a known translating scene initializes correctly; failure leaves clean state and can retry. Report initialization delay in both source frames and seconds on the dashcam.

### CAM-06 — Track established 3D landmarks with robust PnP [P1]

- **Finding:** after frame 4, `slam.py:55–56` extrapolates constant velocity, then attempts pose-only BA from propagated observations. New projection associations at `slam.py:79–87` are added after that optimization and never refine the current pose in the same stage.
- [ ] Predict a pose from recent valid states; gather local-map 3D-to-2D correspondences; run robust PnP; refine accepted inliers; search for additional projections; refine again if evidence warrants it.
- [ ] Require enough well-distributed, geometrically informative inliers and bounded residuals. A solver's minimum sample count is not a tracking acceptance criterion.
- [ ] Use existing map scale when estimating pose. Do not concatenate independently unit-normalized essential translations as if they had a common distance scale.
- [ ] Keep two-view estimation for initialization and clearly separated fallback decisions. A fallback that loses scale consistency must not silently extend the same map.
- **Acceptance:** synthetic speed changes, turns, and injected outliers are recovered from established landmarks; projection matches contribute to final pose refinement; failure does not publish a predicted pose as observed success.

OpenCV exposes PnP RANSAC and LM/VVS refinement; its documented DLS/UPNP options fall back to EPnP, so they should not be selected as supposed upgrades. Verify overloads in the installed build. [PnP reference](https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html).

### CAM-07 — Make prediction timestamp-aware and diagnose dashcam imaging effects [P1/P3]

- [ ] Preserve each decoded frame's source index and presentation timestamp. Use a monotonic host clock only for processing duration; do not conflate playback time with processing time.
- [ ] Scale a tested motion model by elapsed source time; reset it across seeks, loss, or long gaps. Accept a valid initial timestamp of zero, handle duplicate/nonmonotonic timestamps explicitly, and distinguish skipped input from failed tracking.
- [ ] Record residuals by image row, exposure/blur indicators, and camera mode to investigate rolling shutter or stabilization. Do not label normal undistortion as rolling-shutter correction.
- [ ] Gate full rolling-shutter motion optimization on identified sensor readout and demonstrated error. It belongs in a separate experiment after the central-projection baseline works.
- **Acceptance:** analytic constant-velocity/constant-twist trajectories sampled at different intervals produce consistent predictions with matching initialization; acceleration cases have separately measured prediction errors. Seeks reset state; variable-frame-rate fixtures preserve exact associations. For the provided clip, source timing is verified approximately uniform, but the data model must not assume every future file is.

PyAV represents timestamps using PTS and stream/frame time bases; retain rational timing when available rather than rounding every frame to an assumed integer FPS. [PyAV time documentation](https://pyav.org/docs/stable/api/time.html).

### CAM-08 — Make tracking loss and pose quality observable [P1]

- **Finding:** `Frame.__init__` appends an identity-pose frame before tracking succeeds (`frame.py:21–25`). There is no explicit lost state, usable-inlier threshold, or rollback.
- [ ] Build candidate frame state and commit its map mutations only after acceptance. Retain failed frame timestamps/status for reporting without putting them in the valid-pose chain.
- [ ] Track inlier count/ratio, image coverage, reprojection percentiles, landmark age/depth spread, and relevant conditioning indicators. Distinguish quality scores from calibrated covariance.
- [ ] Stop adding landmarks during loss; attempt controlled recovery, then start a separately identified submap if necessary. Never connect unrelated coordinate systems with a straight trajectory line.
- **Acceptance:** blank/blurred frames, a sudden cut, low texture, and solver rejection produce clear states; no invalid observation/pose is left behind; the next valid frame does not extrapolate from failed identity poses.

## 3. Image features, correspondences, and edges

### FEAT-01 — Handle empty features and preserve row alignment [P0]

- **Finding:** `frame.py:41–44` assumes corners/descriptors exist; empty keypoints become shape `(0,)`, incompatible with downstream `(N,2)` geometry.
- [ ] Use an explicit BGR-to-gray conversion and validate supported image shape/dtype. Return stable empty arrays: `(0,2)` keypoints and the chosen descriptor width/dtype.
- [ ] Correct `frame.py:16` to assign `self.h, self.w = img.shape[:2]`. The current width/height labels are reversed; their use in a diagonal happens to hide the error, but other consumers must receive the actual dimensions.
- [ ] Construct positions from the keypoints returned by `ORB.compute`, because descriptor extraction can discard inputs. Assert equal keypoint, descriptor, and association lengths.
- [ ] Build/query the KD-tree only under its documented empty-data contract; treat an empty result as lack of observations.
- **Acceptance:** blank, tiny, non-square, blurred, grayscale-input-policy, and textured images either produce aligned arrays with correct dimensions or a documented input error. No ordinary no-feature frame raises an uncontrolled exception.

### FEAT-02 — Establish a reproducible classical feature baseline [P1]

- **Finding:** `frame.py:40–43` creates ORB anew and feeds fixed-size GFTT keypoints. The constructed keypoints do not provide detected ORB orientation/pyramid metadata; an isolated supported-keyword probe returned `angle=-1`, `octave=0`.
- [ ] Compare the current GFTT-plus-descriptor strategy with configured `ORB.detectAndCompute`. Record pyramid levels, scale factor, orientation, feature cap, FAST threshold, patch size, and border handling.
- [ ] Enforce spatial coverage using a grid/quadtree or balanced per-region quotas. Avoid collecting most usable features on a nearby vehicle or one textured sign.
- [ ] Retain response, octave, angle, and feature uncertainty where used by matching and BA. Reuse detector instances when safe.
- **Acceptance:** run a controlled rotation/scale fixture and the fixed dashcam ranges; report verified tracks, coverage, pose availability, and latency. A higher corner count alone is not a successful upgrade.

### FEAT-03 — Make descriptor matching safe, unique, and efficient [P0/P1]

- **Finding:** `frame.py:51–54` assumes every KNN result has two neighbors; `frame.py:63` performs repeated list-membership scans despite its performance comment.
- [ ] Guard missing descriptors, fewer than two training descriptors, short KNN results, and descriptor shape/dtype mismatches.
- [ ] Keep ratio and absolute-distance thresholds configurable. Evaluate mutual consistency or a deterministic best-match assignment; do not allow two accepted current features to compete silently for one previous feature.
- [ ] Use sets/masks for uniqueness and retain original query/train indices through every filter. Define stable tie-breaking for equal descriptor distances.
- **Acceptance:** zero/one/two descriptors, repeated descriptors, ties, and known matching permutations preserve index alignment and uniqueness. Measure runtime against feature count instead of assuming a loop is linear.

### FEAT-04 — Correct residual units and robust estimation settings [P1]

- **Finding:** `frame.py:61` compares normalized ray displacement to a pixel diagonal. At 1024×576, the nominal limit is approximately 117.5 normalized units, so the intended image-motion gate is ineffective. `frame.py:78–79` fixes the RANSAC budget to 100 trials without an explicit reproducibility policy.
- [ ] Express displacement gates in pixels using `_kps`, or derive a consistent normalized/angular threshold. Document every threshold's units and image-resolution scaling.
- [ ] Specify residual type, confidence, iteration limit, random seed, inlier support, and model-validity checks. Handle absent models, all-false masks, and too few post-fit inliers explicitly.
- [ ] A/B test OpenCV USAC/MAGSAC where the installed Python overload supports the intended model. Sort correspondences before PROSAC; do not assume every robust flag is supported by every function.
- **Acceptance:** resizing a synthetic input preserves the intended acceptance behavior; outlier sweeps establish a failure curve; repeated seeded runs are comparable. Use a computed/adaptive RANSAC budget rather than an unexplained universal iteration count.

MAGSAC does not eliminate the need for a useful noise/termination threshold, and parallel USAC can affect reproducibility. [OpenCV USAC tutorial](https://docs.opencv.org/4.13.0/de/d3e/tutorial_usac.html).

### FEAT-05 — Reject invalid projections before dividing [P0]

- **Finding:** `slam.py:69–73` divides by depth before checking it and filters only pixel bounds. A landmark behind the camera can project into the image and be accepted.
- [ ] Transform candidate points into the current camera; check finite coordinates and strictly positive, numerically safe depth before division.
- [ ] Apply finite pixel coordinates, valid-image/rectification mask, and image bounds afterward. Use a mask that stays aligned with landmark IDs.
- [ ] Use local-map visibility, viewing direction, predicted feature scale, and depth range to reject unsuitable candidates before descriptor comparison.
- **Acceptance:** positive/negative/zero depth, NaN/Inf points, edge pixels, and empty maps are handled without invalid divisions or false observations. Positive and negative depth points projecting to the same pixel must receive different visibility decisions.

### FEAT-06 — Enforce one point-to-feature association per frame [P0]

- **Finding:** `slam.py:80–87` breaks only the descriptor loop. One landmark can attach to several current-frame features; already-observed landmarks can be searched again.
- [ ] Exclude landmarks already observed in the frame; rank all viable candidates and commit at most one mutually compatible association.
- [ ] Stop the landmark search once an association is accepted; reject conflicts with previous propagation or another landmark. Use a documented descriptor representative and optional second-best margin.
- [ ] Keep association proposal separate from insertion so pose/outlier validation can reject candidates without corrupting the map.
- **Acceptance:** a synthetic cluster of similar descriptors produces no duplicate point/frame observations, no slot overwrites, and deterministic associations independent of incidental list order where ties are explicitly resolved.

### FEAT-07 — Reject dashboard features and reduce dynamic-object contamination [P1/P2]

- **Finding:** no mask or dynamic-consistency logic exists. The inspected dashcam frames include a stationary-in-image dashboard and independently moving traffic.
- [ ] Support a calibration/preprocessing-aware static exclusion mask for the hood/dashboard, overlays, and invalid rectification borders. Avoid a hardcoded crop that invalidates intrinsics.
- [ ] Monitor feature distribution and spatial motion consistency; use robust geometry plus persistent track history to reject moving-object observations and delay their landmark promotion.
- [ ] Evaluate optional semantic masks only after the classical baseline is measured. A detected vehicle is not always moving; a visually static track is not necessarily part of the world map.
- [ ] Test ablations with/without masks on identical frames and parameter settings. Quantify retained static scene support as well as rejected tracks.
- **Acceptance:** accepted landmarks are less concentrated on the dashboard/nearby moving vehicles without destroying useful coverage; pose availability and held-out accuracy do not regress. Report mask provenance and exact coordinate transform.

### FEAT-08 — Add physical line/edge features only as a measured SLAM extension [P3]

- **Finding:** there is no line detector or line-landmark model. `cv2.line` at `slam.py:127` merely visualizes matches between two images.
- [ ] First label the existing overlay accurately. If point tracking fails on low-texture structured scenes, prototype OpenCV line-segment detection in a separate optional frontend.
- [ ] Define image-line coordinates, endpoint ordering, line descriptor/matcher, uncertainty, and 3D line representation. Use geometric line reprojection constraints rather than treating arbitrary endpoints as independently observed fixed 3D points.
- [ ] Handle repeated lane markings, partial occlusion, changing visible endpoints, and line-direction ambiguity. Road lines can move in the image even on a static road and do not by themselves fix scale.
- **Acceptance:** point-only versus point-plus-line comparisons show better pose/map metrics on held-out structured scenes and acceptable runtime. Do not implement line BA simply to draw more edges on the video.

## 4. Landmark geometry and map structure

### MAP-01 — Make observation insertion/removal atomic and reciprocal [P0]

- **Finding:** `Point.add_observation` (`point.py:24–27`) overwrites feature slots and appends duplicates. `orb`, deletion, and BA search with `frame.pts.index(point)` despite already storing indices.
- [ ] Define an observation record keyed by frame ID with feature index and optional quality; each landmark has at most one observation in a frame, and each feature slot references at most one landmark.
- [ ] Reject or explicitly reconcile conflicting insertion; make exact repeated insertion idempotent. Update the frame and landmark together.
- [ ] Read descriptors and measurements through stored indices, validating the reciprocal link. Remove observations from both sides and remove a deleted point from the map explicitly; `del self` does not do this.
- [ ] Define landmark merging as a separate operation with conflict resolution, not silent overwriting.
- **Acceptance:** duplicates, competing landmarks, deletion in any order, rejected frames, fusion, and culling preserve all links. Repeated deletion is either harmless or a clearly documented error, never a partial mutation.

### MAP-02 — Fix two-camera triangulation [P0]

- **Finding:** all four DLT rows use `pose1` at `slam.py:25–29`. The `pose2` argument is unused.
- [ ] Build rows 1–2 from `P1=T_c1w[:3]` and observation 1, rows 3–4 from `P2=T_c2w[:3]` and observation 2 when input points are normalized. If using pixels, use `K1 @ P1` and `K2 @ P2` instead.
- [ ] Preserve correspondence ordering explicitly. Use a small correct scalar implementation as an oracle before selecting batched NumPy SVD or OpenCV triangulation.
- [ ] Separate triangulation output from geometric acceptance; return aligned validity/reason data.
- **Acceptance:** known cameras with both rotation and translation recover known finite noncoplanar points; changing the second camera changes the solution appropriately. Include tests that fail if either pose or correspondence order is ignored.

### MAP-03 — Replace the homogeneous-w heuristic with physical quality gates [P0]

- **Finding:** `slam.py:91–98` marks small homogeneous denominators but divides every row anyway; only the current camera's depth is checked. There is no parallax or two-view reprojection test.
- [ ] Check all homogeneous components and safe denominator magnitude before dehomogenization; divide only valid rows.
- [ ] Require finite 3D position, positive depth in both cameras, adequate ray angle/conditioning, and bounded reprojection errors in both views.
- [ ] Replace `abs(w)>0.005` as an alleged geometry-quality test. For an SVD unit vector, it acts roughly like a radius limit near 200 arbitrary world units and changes under a translated world origin; it is not a parallax criterion.
- [ ] Validate feature availability in both frames before constructing a new point. Do not insert a point that has passed only one view's checks.
- **Acceptance:** points at infinity, tiny baseline, pure rotation, negative depth in either view, large residual, and NaN/Inf are rejected with reasons; valid scene geometry is invariant to world-frame translation/rotation and consistent scale changes.

### MAP-04 — Add landmark maturity, uncertainty, and quality history [P1]

- [ ] Distinguish candidate, active, outlier, and retired landmarks. Delay promotion until sufficient independent observations and baseline support exist.
- [ ] Track observation count, successful re-observation ratio, age, parallax, residual history, representative descriptor, viewing direction, and scale/depth range.
- [ ] Prefer stronger-baseline triangulation partners rather than creating every point from immediately adjacent frames. Consider inverse-depth candidates only if measured long-range conditioning problems justify the added representation.
- [ ] Treat uncertainty estimates honestly: heuristic confidence is not a statistical covariance. Verify any propagated uncertainty on controlled noisy scenes.
- **Acceptance:** weak newborn points cannot dominate pose estimation; well-observed distant points are not deleted solely for being distant; reported map quality includes retention and support rather than just cloud size.

### MAP-05 — Separate frames from keyframes and maintain a local map [P1]

- **Finding:** every `Frame` is stored forever and participates in graph setup (`frame.py:23–25`, `dmap.py:109`). There is no keyframe decision or covisibility structure.
- [ ] Retain a lightweight timestamped pose/status trajectory for all input frames; keep full descriptors/observations only for keyframes and the bounded tracking cache.
- [ ] Store each accepted non-keyframe's reference keyframe ID, relative transform, and submap/scale context so later keyframe corrections can update its map trajectory. Reanchor these records before culling their reference keyframe; preserve a separate original odometry history if needed.
- [ ] Insert keyframes based on overlap, parallax, tracking support, scene change, and elapsed time. A fixed every-N-frames rule can be a starting baseline, not the sole criterion.
- [ ] Build a small covisibility structure from shared validated landmarks; choose local keyframes/points and required fixed boundary cameras from it.
- [ ] Cull redundant keyframes without deleting the only observations that constrain a landmark or disconnecting the local graph. Use stable IDs independent of list positions.
- **Acceptance:** a long synthetic sequence bounds active descriptor/solver state while preserving trajectory history and graph connectivity; corrected keyframes update dependent trajectory records, and reanchoring during culling preserves their intended poses. Dashcam coverage/accuracy is compared before and after the policy change.

### MAP-06 — Define the sparse 3D map product and its quality metadata [P2]

- [ ] Export finite validated world-space points, camera/keyframe poses, support count, quality, point IDs, and scale/frame metadata. Include whether coordinates are raw local odometry or globally corrected map output.
- [ ] Display only supported landmarks by default, with optional debug layers for candidates/outliers. Keep display downsampling separate from estimator landmark culling.
- [ ] Add geometric consistency checks: depth/parallax distributions, residuals by image region, negative-depth fraction, track lifetime, and map fragmentation.
- [ ] Avoid automatic surface meshing of the sparse dashcam cloud. Dense reconstruction requires trustworthy depth, view coverage, and separate evaluation of holes, moving objects, and scale consistency.
- **Acceptance:** exported clouds contain no invalid coordinates; their reprojections agree with the stored observations; quality filters change the displayed subset without silently changing the underlying solver map.

### MAP-07 — Add versioned map and trajectory persistence [P2]

- [ ] Define a small explicit schema for calibration, run provenance, coordinate conventions, scale status, keyframes, landmarks, observation indices, and non-keyframe reference/relative-pose records.
- [ ] Use array formats such as NPZ plus JSON metadata for the first implementation; avoid opaque pickle as an interoperability contract. Validate shapes, references, version, and finiteness on load.
- [ ] Record checksums and write outputs atomically. Preserve submap IDs and correction versions; reject mixing observations or poses from different map revisions.
- [ ] Distinguish a view-only point-cloud export from a resumable map containing descriptors/observations. Loading a cloud alone cannot provide relocalization.
- **Acceptance:** save/load preserves geometry and reciprocal links; malformed or mismatched calibration files fail clearly; replaying a loaded map does not require the original decoder or GUI backend.

### MAP-08 — Export portable 2D/3D point data for other applications [P2]

- **Requested behavior:** another application can read an exported file, use the numerical point data, and recreate the same 2D map and 3D point cloud without running this SLAM program or accessing the original video.
- **Dependencies:** MAP-06/07 establish validated map snapshots and metadata; 2D-02/03 define the map plane; VIEW-04 supplies the export/report interface. Share these implementations rather than introducing competing export schemas.
- [ ] Define and document a versioned, self-contained JSON exchange file containing metadata plus `points_2d` and `points_3d` collections. Support exporting either collection or both; state explicitly which are present. Keep this public interchange schema independent of Python classes, pickle, g2opy, and the viewer backend.
- [ ] Export actual numerical map coordinates with stable string point IDs and submap IDs. Use `x,y` for planar map points and `x,y,z` for world-space landmarks; image feature pixels `(u,v)` must be a separately named optional observation collection. Preserve the 2D-to-3D point-ID correspondence where a planar point is a projection of a landmark.
- [ ] Include everything needed to interpret and recreate the geometry: schema version, map ID/revision, coordinate-frame name, axis directions/handedness, units, scale status, point counts, bounds, and export selection/filter settings. For 2D projections, include the world-space plane origin and basis vectors and document the projection equation. For matrices, specify direction, dimensions, and storage order.
- [ ] Include available RGB colors, observation/support counts, landmark state, and quality values with documented ranges and meanings. Make optional/missing attributes explicit; do not invent confidence or metric units. Preserve adequate floating-point precision, reject NaN/Infinity and duplicate IDs, and keep unrelated submaps distinct unless a validated alignment is supplied.
- [ ] Take all exported collections from one immutable, accepted map revision so concurrent BA or loop correction cannot mix old 2D points with new 3D geometry. Export full validated map data by default; make any filtered/downsampled selection explicit and record it. Save atomically and define behavior for empty maps and partial runs.
- [ ] Offer simple companion formats for consumers that need them: 2D CSV with a metadata JSON sidecar and 3D PLY with a metadata/attribute sidecar where needed. Document each format's retained and omitted fields; the self-contained JSON remains sufficient to recreate both available point layers.
- [ ] Add proposed CLI options for output path, `2d`/`3d`/`both`, format, and optional selection settings. These options must work headlessly and produce actionable errors for unsupported formats, unavailable map layers, and unwritable paths.
- [ ] Publish the schema, a small synthetic sample file, and an independent reader example that uses ordinary file/JSON parsing and does not import the SLAM application's geometry or serialization helpers. Demonstrate loading coordinates into another application's data structures and plotting both map layers using only exported values and metadata.
- [ ] Extend the required `GRMN2734.MP4` benchmark to export and independently reload both layers once its prerequisite mapping work runs. Record file size, write/read duration, point counts, invalid/dropped records, and numerical differences, alongside the run's calibration and scale status.
- **Acceptance:** an independent consumer recreates the exported 2D point locations and 3D cloud with the same IDs, colors, axes, scale labels, and declared filtering. Synthetic non-axis-aligned plane tests verify that projecting exported 3D landmarks reproduces their exported 2D counterparts within the documented serialization tolerance. Empty files/maps, unknown schema versions, malformed coordinates, and submap separation have explicit tests. Recreating the map must not depend on the source video, this application's runtime, or hidden local configuration.
- **Representation limit:** 2D projection discards height; a 2D-only export cannot recover the original 3D cloud. Export both collections when both maps must be recreated. Point-data interchange recreates the recorded geometry; resuming SLAM still requires the richer descriptors, observations, and state specified in MAP-07.

## 5. Bundle adjustment and graph edges

### BA-01 — Verify native pose conversion and solver capabilities [P0]

- **Finding:** source uses unavailable g2o names (`dmap.py:98,126`); the pose inversion itself is supported by the isolated installed-SBACam projection check.
- [ ] Retain a native boundary test comparing explicit projection with `SBACam`, including nonidentity rotation, unequal `fx/fy`, and off-centre principal point.
- [ ] Choose the installed sparse solver/point vertex through a documented supported API. Confirm constructors, estimate accessors, edge-error access, and object ownership from the exact binding.
- [ ] Verify that `EdgeProjectP2MC` is used as the intended two-coordinate monocular measurement; the `set_cam(..., 1.0)` baseline argument is not a measured physical baseline for this video.
- **Acceptance:** an actual small graph reduces error from a perturbed state and leaves fixed states unchanged; no stubbed binding is accepted as native-solver validation.

### BA-02 — Remove graph-ID collisions and document gauge constraints [P0/P1]

- **Finding:** frame IDs can collide with point IDs offset by `0x10000` (`dmap.py:115,121–127`) on a sufficiently long sequence. First two frames are fixed (`dmap.py:117`).
- [ ] Allocate graph-local IDs from disjoint maps rather than a magic offset. Check every vertex/edge insertion result and reference.
- [ ] Document the monocular gauge: global rigid frame and one scale degree of freedom must be fixed by a valid anchor/baseline or equivalent constraint.
- [ ] Distinguish a practical initial fixed-pair policy from measured scale. Fixing two entire poses can also freeze initialization error; evaluate the anchor policy deliberately before relaxing it.
- **Acceptance:** large logical frame IDs cannot collide with landmark vertices; a known graph has no unconstrained gauge; anchor states remain fixed; arbitrary-unit maps never claim metric scale from the anchor alone.

### BA-03 — Build a genuinely local and sufficiently constrained graph [P1]

- **Finding:** `local_window` limits movable cameras, but `dmap.py:109` still adds all historical camera vertices. Active points add all their observations.
- [ ] Select local keyframes, their useful landmarks, and only fixed boundary cameras needed to constrain those observations. Preserve necessary boundary evidence rather than simply truncating every observation to a recent window.
- [ ] Detect disconnected components, points with insufficient support, and cameras with too few informative edges before optimization.
- [ ] Implement a lightweight pose-only graph for tracking instead of rebuilding the entire historical graph each frame. Bound BA iterations or wall-time budget and report early termination.
- **Acceptance:** graph size follows the selected local problem, not total recording duration; reference scenes retain comparable optimized geometry; the graph cannot silently optimize an unsupported current camera.

### BA-04 — Use consistent noise weights and safe robust-kernel ownership [P1]

- **Finding:** all edges have identity information (`dmap.py:139`), and one Huber object is attached to every edge (`dmap.py:101,140`). Shared native ownership safety is unverified, not a reproduced crash.
- [ ] Weight pixel residuals with documented inverse observation covariance, for example calibrated octave-dependent uncertainty where justified. Keep residual, squared Mahalanobis error, and information-matrix units consistent.
- [ ] Use one robust kernel per edge unless the exact binding explicitly supports shared ownership. Exercise repeated graph creation/destruction under the native runtime.
- [ ] Treat `sqrt(5.991)` as a convention tied to a two-dimensional normalized residual model, not a universal pixel threshold. Calibrate noise and gate values using controlled perturbations.
- **Acceptance:** changing declared observation noise produces the expected weighting; graph teardown remains stable; robust-loss settings, raw pixel residuals, and retained edges are reported separately.

### BA-05 — Validate optimizer results before committing them [P0/P1]

- **Finding:** `dmap.py:145–163` optimizes and writes poses/points back without checking success, finite results, valid rotations, residual change, or usable constraints. Pose-only mode returns `None`; BA returns an aggregate chi-square value.
- [ ] Return a structured result: attempted/skipped/failed, reason, iteration count, timing, vertices/edges, before/after residual summaries, and rejected observations.
- [ ] Optimize a candidate state; validate finite points/poses, valid rotations, fixed-state preservation, and objective behavior on comparable observations before applying it atomically.
- [ ] Retain the last valid state on numerical failure and expose failure to tracking. Do not convert failed optimization into an identity or unchecked prediction.
- **Acceptance:** empty/underconstrained graphs, invalid estimates, solver exceptions, and failed convergence leave the map intact. Successful synthetic refinement improves the same fixed evaluation observations.

### BA-06 — Separate robust refinement, observation rejection, and final refinement [P1]

- [ ] Perform robust optimization, inspect each observation's normalized residual and depth, deactivate/remove outlying observations, then refine the accepted set when useful.
- [ ] Remove a bad observation without automatically deleting a landmark that still has adequate independent support.
- [ ] Report residual distributions on retained edges **and** a fixed evaluation set, along with rejected counts. Deleting difficult measurements can lower a mean without improving pose accuracy.
- [ ] Reevaluate landmark quality after camera/point updates; keep tracking rejection thresholds and BA rejection thresholds explicit.
- **Acceptance:** injected bad observations are rejected while a well-supported landmark survives; the graph remains constrained; before/after metrics state whether the measurement set changed.

### BA-07 — Repair stale-point culling and invalid-residual handling [P0/P1]

- **Finding:** points outside the local graph are retained at `dmap.py:159–162` before the stale condition at `:164` is checked. A point with no observation in the local window cannot reach that stale test. NaN mean errors also fail an ordinary `>5` comparison.
- [ ] Move lifecycle-based retirement outside graph-membership-dependent writeback. Define age from frame/time IDs and distinguish recent support from merely existing historical observations.
- [ ] Handle empty observations, missing vertices, unsafe depth, and non-finite residuals explicitly. Use robust per-observation statistics rather than one unexplained arithmetic mean.
- [ ] Apply retirement through the reciprocal deletion API and preserve diagnostics. Avoid deleting well-observed archived landmarks needed for later relocalization merely to reduce active-map size.
- **Acceptance:** inactive weak points retire, useful archived points follow a documented policy, NaN geometry never survives accidentally, and no deleted point remains in a frame slot or active graph.

## 6. Genuine 2D map creation

### 2D-01 — Add a persistent top-down trajectory view [P2]

- **Finding:** `display.py:13–21` only copies an image into an SDL surface; there is no map-coordinate 2D renderer.
- [ ] Implement a separate top-down view from accepted camera centres `C_w`, with heading, current position, keyframes, and a persistent path.
- [ ] Choose an explicit display plane/basis, equal axis scaling, origin, axis labels, and scale status. A first-camera horizontal projection may be offered as such without calling it gravity-aligned ground.
- [ ] Break paths at lost intervals, seeks, new submaps, and unconnected coordinate systems. Visually distinguish an unmeasured prediction from an accepted pose.
- [ ] Provide fit-map, follow-camera, zoom/pan, and reset controls without changing map geometry.
- **Acceptance:** known straight and turning synthetic trajectories render with correct handedness, positions, and heading; a lost section is visibly discontinuous; no meter scale appears for an arbitrary-scale map.

### 2D-02 — Estimate or configure a ground-aligned map frame [P2]

- [ ] Define a rigid map-to-ground/display transform and its provenance. If estimating a road plane, select plausible static road evidence, reject outliers, and track support/uncertainty.
- [ ] Do not fit a plane indiscriminately to all landmarks: barriers, vehicles, signs, and the dashboard violate that assumption. A locally planar road can change grade and banking.
- [ ] Project points into a declared planar basis, e.g. `x2d=e1·(X-origin)`, `y2d=e2·(X-origin)`; use a separate height coordinate for filtering.
- [ ] A calibrated camera height or other real constraint may establish scale only under validated assumptions. Record the source and uncertainty and test violations; plane fitting alone does not provide metres.
- **Acceptance:** synthetic slopes/banks and known camera mounting transforms preserve orientation and height; poorly supported plane estimates are suppressed or labelled uncertain rather than constantly rotating the map.

### 2D-03 — Add a sparse landmark map layer with correct semantics [P2]

- [ ] Project mature 3D landmarks into the chosen map plane; support height/quality filters, current local-map highlighting, and spatial binning for display.
- [ ] Label the layer as sparse observations. A dense-looking cluster of features is not a probability of occupancy, and blank regions are unknown.
- [ ] Keep plane definition, scale, origin, and correction revision in exported 2D map metadata. Rebuild affected layers after BA/loop corrections rather than accumulating stale copies.
- [ ] Expose this numerical 2D point layer through the portable exchange file defined in MAP-08, retaining links to its source 3D landmarks.
- **Acceptance:** a known synthetic 3D scene projects into the correct 2D coordinates; changing display resolution preserves geometry; global corrections update trajectory and landmark layers consistently.

### 2D-04 — Gate occupancy/free-space mapping on sufficient evidence [P3]

- [ ] Before implementing an occupancy grid, specify sensor/ray observations, uncertainty, inverse sensor model, cell size in known units or explicitly arbitrary units, and occupied/free/unknown semantics.
- [ ] Define grid origin pose, width/height, resolution, row-major indexing, frame/submap/revision, and value encoding. Test world-to-cell and cell-to-world conversion at negative coordinates and exact boundaries before adding updates or exports.
- [ ] Sparse visible corners do not justify marking every ray as reliably traversable free space or claiming complete obstacles. Evaluate multi-view depth or additional depth sensing only with a clear SLAM mapping requirement.
- [ ] If evidence becomes adequate, test bounded log-odds updates, unknown-cell handling, occlusion, dynamic-object expiry, and rebuilding/submap transformation after pose correction.
- **Acceptance:** independently constructed scenes verify ray updates and unknown regions; no occupancy or road-safety claim is made from the current sparse cloud. Keep route planning and driving-control features outside this backlog.

## 7. 3D map presentation, export, and usability

### VIEW-01 — Fix GUI ownership, exit handling, and cleanup [P1]

- **Finding:** `dmap.py:20–30` starts an infinite daemon viewer process; `:53–54` uses `Queue.empty()` as a synchronization check; `display.py:16–17` calls `exit(0)` inside drawing. Cleanup in `slam.py:184–185` is bypassed on exceptions.
- [ ] Give the viewer an explicit ownership/lifecycle contract. On macOS, validate a main-thread GUI implementation or the actual native process model used by the selected backend.
- [ ] Use bounded latest-state delivery with nonblocking draining; avoid relying on `Queue.empty()` for correctness. Add quit/EOF messages, liveness checks, close/join behavior, and timeout-based termination only as a fallback.
- [ ] Release capture, GUI resources, and workers in `finally`/context-managed cleanup. Handle keyboard interruption, window close, EOF, initialization failure, and solver failure.
- **Acceptance:** all exit paths leave no child process or camera/file handle; closing either view stops or detaches according to documented behavior. Test an actual desktop window; an Agg screenshot is not a native GUI lifecycle test.

### VIEW-02 — Provide useful 2D/3D diagnostics and playback controls [P2]

- [ ] Show current tracking state, frame/time, calibrated/uncalibrated status, scale status, keypoint/match/inlier/landmark counts, residual summary, and processing rate.
- [ ] Add pause/resume, single-step, restart, frame/time seek, and hold-at-EOF. Define seek as reset/replay or loading a valid checkpoint; never splice old map state onto unrelated timestamps.
- [ ] Add a concise legend separating raw features, propagated matches, projection matches, accepted landmarks, and rejected observations. Make debug layers toggleable and use readable labels/color choices.
- [ ] Separate playback rate from processing throughput. Display source frames that correspond to the shown pose; tag asynchronous map snapshots with source frame and map revision.
- **Acceptance:** a user can identify initialization, loss, and successful tracking and inspect a problematic frame without reading terminal output; pause/seek never pairs an old pose with a new image.

### VIEW-03 — Correct pose/cloud rendering and color conventions [P2]

- **Finding:** `dmap.py:61–67` draws the last camera twice in different colors and never draws the full trajectory. `slam.py:108` stores BGR colors; `dmap.py:92` divides by 256, with no explicit RGB conversion.
- [ ] Render a tested camera frustum convention, distinct current camera/keyframes, and continuous valid trajectory segments. Remove the duplicate final-camera draw.
- [ ] Convert BGR samples to the renderer/exporter's declared RGB order and normalize uint8 channels by 255 where float colors are required. Check the SDL surface format instead of assuming channel layout.
- [ ] Handle empty clouds/pose lists with stable shapes, filter invalid display geometry, and compute sensible fit bounds without letting one outlier hide the entire map.
- [ ] Keep all viewer transforms outside estimator state. Add axes, origin, quality filters, and a scale-status label.
- **Acceptance:** a synthetic red/blue color chart is correct in image, 3D view, and export; known pose rotations point the frustum correctly; empty/one-point/outlier-heavy maps remain usable.

### VIEW-04 — Export trajectories, point clouds, and structured reports [P2]

- [ ] Export camera-to-world trajectory with original timestamps, validity/submap IDs, and an explicit quaternion ordering. TUM text uses `timestamp tx ty tz qx qy qz qw`; matrix-only KITTI exports need a separate timestamp/status sidecar where required.
- [ ] Add PLY point-cloud export with RGB and a sidecar for calibration, scale, frame, quality, and map revision. Keep a richer native map format for resumability.
- [ ] Expose the self-contained 2D/3D point-data exchange format from MAP-08 through the same export interface and independently verify that another application can recreate both map layers.
- [ ] Save per-frame JSONL diagnostics and an aggregate JSON report with schema version, completion status, rejected-reason counts, and runtime provenance.
- [ ] Keep invalid/lost intervals explicit; do not fill missing poses with identity or fabricate smooth interpolation in evaluation outputs. Save output only under the requested run directory.
- **Acceptance:** export/import round trips preserve camera centres/orientations and timestamps; a separate evaluator can load the result; partial runs and uncalibrated maps cannot be mistaken for successful metric trajectories.

[TUM trajectory format](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats) is an interoperability reference; a format-compliant file is not evidence that its trajectory is accurate.

## 8. Performance and resource use

### PERF-01 — Instrument the complete processing path [P1]

- **Finding:** `slam.py:42,138–139` reports one `time.time()` duration for `process_frame`; video decoding happens outside it and asynchronous viewer completion is not measured.
- [ ] Use `perf_counter_ns` or equivalent monotonic timing for decode, resize/rectification, feature extraction, matching, robust estimation, map search, pose refinement, triangulation, graph construction, BA, map update, serialization, and rendering.
- [ ] Report median/p95/p99/max, total run wall time, processed/decoded/tracked frames, queue age, and peak process-tree memory. Separate initialization, steady tracking, BA-trigger frames, and GUI runs.
- [ ] Record hardware, power mode where known, thread counts, library builds, resolution, seeds, and exact configuration. Include decoder and viewer costs in end-to-end claims.
- **Acceptance:** stage times explain the measured total within documented overlaps; GUI throughput is not inferred from enqueue time; skipped/lost frames cannot inflate reported tracking FPS.

### PERF-02 — Remove demonstrated algorithmic hot spots [P2]

- **Source candidates, not measured bottleneck rankings:** per-frame detector/matcher creation; repeated inverse calibration; repeated list membership and `.index`; Python SVD loop; whole-map projection; all-observation descriptor scans; full-history graph construction and snapshot serialization.
- [ ] Profile the corrected baseline on the required video before ranking these changes. Use Python profiling plus explicit native-stage wall timers; Python call counts alone do not explain native solver cost.
- [ ] Cache invariant calibration/detector state; use stored observation indices and descriptor representatives; batch projection/triangulation after matching reference outputs.
- [ ] Search a bounded local map with a grid/KD-tree and informed visibility gates. Benchmark OpenCV/native Hamming batches or NumPy bit-count approaches against the existing helper before adopting them.
- [ ] Avoid retaining every full-resolution frame or full descriptor history. Downsample rendering independently of estimation; avoid rebuilding identical display arrays unnecessarily.
- **Acceptance:** every optimization reports unchanged or explicitly explained numerical behavior, before/after stage latency, memory, and map/tracking metrics on identical inputs. Do not assume GPU acceleration improves a small CPU-bound workload.

### PERF-03 — Separate tracking, mapping, and display work only when needed [P2]

- [ ] Begin with deterministic single-threaded estimation and optional display. Introduce a mapping worker only if BA latency is a demonstrated obstacle.
- [ ] Define snapshot ownership, versioned map updates, atomic publication, and conflict policy for observations changed while BA runs. Reject stale optimization results rather than overwriting newer tracking state.
- [ ] Bound queues and choose explicit backpressure/drop policies. Dropping a display snapshot is different from dropping input frames or silently skipping map updates.
- [ ] Control OpenCV/BLAS/solver worker counts to avoid oversubscription. Test the platform's multiprocessing start method and avoid unnecessary array copying/pickling.
- **Acceptance:** slow viewer/BA simulations keep input and pose timestamps aligned, bound memory, and shut down cleanly. Single-threaded and concurrent runs produce comparable accepted states within declared numerical tolerance.

### PERF-04 — Define sustainable resource and latency budgets [P2]

- [ ] Treat this clip's approximately 33.367 ms frame interval as a **candidate real-time budget**, not current achieved speed. Set a supported resolution/hardware target after measuring the correct baseline.
- [ ] Set explicit caps for active keyframes, active landmarks, decoder buffering, pending BA, and display snapshots. Archive long-term state separately with a documented memory/disk tradeoff.
- [ ] Repeat full-clip runs in one process and test longer independent videos for retained memory, graph-size growth, native leaks, and worker cleanup. Looping this 60-second clip is a stress fixture, not new accuracy evidence.
- [ ] Measure slow-path latency and startup/vocabulary/model loading as well as steady-state averages. Warm-cache and cold-start results should be labelled.
- **Acceptance:** active estimator/queue memory stays within the declared policy, repeated runs release resources, and deadline misses are reported. A bounded active map can coexist with a growing archived trajectory; distinguish them in the memory report.

## 9. Relocalization, loop closure, and sensor extensions

### SLAM-01 — Recover localization against existing keyframes [P2]

- **Finding:** there is no relocalization or map reuse path; current failure handling cannot recover a valid pose.
- [ ] After keyframe persistence and tracking states exist, add descriptor/place-recognition candidate retrieval followed by geometrically verified 3D-to-2D PnP and local refinement.
- [ ] Require sufficient distributed inliers and multi-frame consistency before resuming the old map. Appearance similarity alone is insufficient for repeated barriers, lane markings, or similar highway signs.
- [ ] Reset motion prediction after recovery; record the gap and submap/reference identity. Keep a new-map path distinct from recovered localization.
- **Acceptance:** a synthetic/held-out revisit after forced loss recovers the correct map; visually similar wrong places are rejected; false-positive recovery rate and time-to-recover are reported.

### SLAM-02 — Add geometrically verified loop closure and map correction [P3]

- [ ] Build loop candidates from persistent keyframes with temporal exclusion and repeated confirmation; estimate a geometrically consistent relation and reject inconsistent matches.
- [ ] For monocular maps, account for scale drift with a validated Sim(3) correction where appropriate. Optimize a pose graph, fuse duplicate landmarks safely, and run suitable local/global refinement.
- [ ] Update affected landmarks, keyframes, saved poses, and 2D/3D map layers under one map revision. Retain a distinction between continuous odometry and globally corrected map poses.
- [ ] Define rollback for rejected corrections and tests against false loops. Keep expensive global work separate from current-frame tracking.
- **Acceptance:** a ground-truth loop sequence improves globally aligned drift without local discontinuity being hidden or observations becoming inconsistent. The requested highway clip is not established to contain a loop, so add a genuine revisit dataset rather than asserting loop-closure coverage from it.

ORB-SLAM3 is a useful architecture and comparison reference for keyframes, map reuse, and place recognition; its implementation is a separate C++ system with its own dependencies and GPLv3 license. Evaluate it externally before considering integration. [ORB-SLAM3 paper](https://arxiv.org/abs/2007.11898), [official repository](https://github.com/UZ-SLAMLab/ORB_SLAM3).

### SLAM-03 — Gate metric scale and additional sensors on real inputs [P3]

- [ ] Keep the monocular baseline first-class. Introduce stereo, IMU, wheel, or GNSS measurements only when actual synchronized recordings, calibration, and an evaluation objective exist.
- [ ] For visual-inertial work, specify time offset, camera-to-IMU extrinsics, noise/bias model, gravity/scale initialization, and sensor-rate handling. A dashcam video filename does not imply IMU/GNSS access.
- [ ] For GNSS or known-height scale constraints, model uncertainty and coordinate conversion; do not overwrite visual estimates with unverified metadata.
- [ ] Ensure comparisons use equivalent sensor modalities. A stereo/inertial system's metric accuracy is not an apples-to-apples monocular result.
- **Acceptance:** controlled sensor fixtures verify units, timing, and observability; real synchronized data demonstrates benefit before this becomes part of the main pipeline. Navigation, route recommendation, and vehicle control remain outside scope.

## 10. Tests and benchmark protocol

### QA-01 — Build a layered, independent regression suite [P0/P1]

- [ ] **Geometry:** projection/normalization, pose inverse/composition, rotation validity, resize/crop/distortion, essential recovery, pure rotation/low parallax, DLT, both-view depth, and reprojection.
- [ ] **Frontend:** empty/tiny inputs, one-neighbor KNN, duplicate descriptors, mask transforms, feature-row alignment, correspondence uniqueness, and residual-unit scaling.
- [ ] **Map integrity:** duplicate/conflicting insertion, point fusion, reciprocal deletion, failed-frame rollback, keyframe removal, archive/active transitions, and save/load.
- [ ] **Native solver:** projection convention, constrained noisy graph convergence, fixed states, invalid/empty/disconnected graph handling, graph IDs, robust-kernel lifecycle, and atomic failure recovery.
- [ ] **Application:** invalid input, CLI validation, bounded headless synthetic run, report/export round trip, EOF, interruption, seek/reset, and missing optional viewer packages.
- [ ] **Viewer:** actual macOS window lifecycle, controls, coordinate/color checks, and frame/pose synchronization. Keep screenshot tests separate from native-window evidence.
- **Acceptance:** synthetic truth is generated independently from the production math being tested. Use nonidentity rotations and noncoplanar points; include noise/outliers/degenerate scenes. A test that generates expectations with the same broken helper cannot validate that helper.

### QA-02 — Make the dashcam benchmark repeatable and mandatory where applicable [P1]

- [ ] Add a benchmark harness and a versioned run manifest; use the exact path and hash recorded above. Do not commit the video or generated reports.
- [ ] Store revision plus working-tree diff/hash, video identity, source frame/time range, calibration hash and quality status, processed dimensions, crop/mask hashes, seed, dependency/native build information, device, thread counts, and algorithm configuration.
- [ ] Define zero-based, end-exclusive frame ranges. Suggested initial runs: `[0,300)` smoke, `[0,1800)` full clip. Freeze additional scenario ranges after inspecting motion/support diagnostics.
- [ ] Use the full clip for release regression. If tuning on part of it, record that fact; a different portion of the same highway clip is correlated data and does not replace an independent held-out sequence.
- [ ] Preserve source PTS; compare identical decoded inputs and preprocessing. Run a fixed-seed reproducibility check plus several recorded seeds for stochastic robustness, and at least three repeated timing runs when making performance claims.
- [ ] Report all outcomes, including failed initialization, dropped frames, tracking loss, restarts/submaps, and incomplete runs. Do not average only successful segments without reporting coverage.
- **Acceptance:** another developer can reproduce the run from its manifest and input hash; the comparison script rejects mismatched critical inputs or clearly labels a deliberate change.

#### Measurements to save for every applicable change

| Area | Required measurements | Interpretation limits |
| --- | --- | --- |
| Input/timing | Decoded/attempted frames, source timestamps, decode failures, skipped frames | Decoder throughput is not SLAM throughput |
| Initialization | First valid map frame/time, inlier support, parallax, rejected reasons | Fast initialization can still be wrong |
| Tracking | Accepted-pose coverage, loss count/duration, longest valid segment, recovery latency | Predictions and interpolated poses are not accepted visual tracking |
| Correspondences | Extracted/descriptor counts, ratio-filtered matches, geometric/PnP inliers, spatial coverage, track lengths | Raw match count is not accuracy |
| Geometry | Both-view positive-depth rate, triangulation rejection reasons, parallax, pixel residual median/p95/p99, invalid values | Low residual alone can coexist with drift or wrong scale |
| Map | Candidate/mature/retired points, observation count, lifespan, keyframes, active/archive counts, submaps | More points need not produce a better map |
| Solver | Graph size, iterations/status, construction/solve time, comparable before/after residuals, outliers | Reduced error after deleting edges is not automatically better estimation |
| Performance | End-to-end wall time/FPS, stage median/p95/p99/max, warm/cold start, GUI queue age | Include decode and asynchronous work in end-to-end claims |
| Resources | Peak process-tree RSS, active map/cache/queue sizes, repeated-run memory trend | One minute cannot establish indefinite stability |
| Truth-based accuracy | APE/ATE, RPE, scale/drift and dataset-specific metrics when synchronized truth exists | Unavailable for this dashcam until matching truth/reference is supplied |

#### Current executable commands

These work as checks now; some intentionally report the current blockers:

```sh
# Reports zero collected tests in the current checkout.
.venv/bin/python -m pytest -q
.venv/bin/python -m pip check

# Current application entry point; presently fails before decoding input.
.venv/bin/python slam.py sample_videos/GRMN2734.MP4

# Local review probe; ignored artifact, not part of a fresh checkout.
.venv/bin/python output/implementation-review-2026-09-15/probe_dashcam.py
```

#### Proposed commands after RUN-03 and QA-02 are implemented

**The following flags/scripts do not exist in this checkout.** They describe the intended interface and must be verified in the README when implemented:

```sh
# Calibrated, bounded, real-video smoke test.
.venv/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --calibration calibration/dashcam.json --headless \
  --start-frame 0 --max-frames 300 --width 1024 --seed 0 \
  --output-dir output/bench/grmn2734-smoke

# Full required clip; the harness repeats runs and records manifests.
.venv/bin/python scripts/benchmark.py \
  --video sample_videos/GRMN2734.MP4 \
  --calibration calibration/dashcam.json --max-frames 1800 \
  --width 1024 --seeds 0 1 2 --repeats 3 \
  --output-dir output/bench/grmn2734-full
```

`calibration/dashcam.json` above is a proposed input, not an existing or fabricated calibration. Until measured calibration exists, allow an explicitly labelled approximate-camera smoke mode; its outputs cannot establish real-road accuracy.

### QA-03 — Add independent ground-truth accuracy evaluation [P1/P2]

- [ ] Use held-out public driving sequences, such as KITTI odometry, with the same monocular input mode, known intrinsics, and correct camera/ground-truth extrinsics. Use separate controlled/loop sequences from TUM or EuRoC to test cases missing from this clip.
- [ ] Export accepted `T_wc` poses with timestamps; verify frame correspondence, quaternion order, and reference-camera identity before calculating errors.
- [ ] Declare association tolerance and alignment: SE(3) for known consistent metric scale, Sim(3) when a monocular scale fit is required. Avoid independent per-segment alignment that conceals drift; report exactly what was fitted.
- [ ] Report translation/rotation relative error over several intervals, global aligned trajectory error, coverage, and scale drift. Never substitute reprojection error for absolute pose accuracy.
- [ ] Use `evo` as an evaluation tool while preserving official dataset protocols for published comparisons. KITTI's official odometry evaluation uses 100–800 m segments, translation percentage, and rotation degrees/metre; do not label a generic aligned APE result as that score.
- **Acceptance:** a known synthetic trajectory perturbation gives the expected metric behavior; ground-truth frame conversion is tested; comparable sensor modes and identical held-out parameters are documented.

[evo](https://github.com/MichaelGrupp/evo), [alignment options](https://github.com/MichaelGrupp/evo/wiki/evo_traj), [KITTI odometry protocol](https://www.cvlibs.net/datasets/kitti/eval_odometry.php), [EuRoC](https://projects.asl.ethz.ch/datasets/euroc-mav/), and [TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) provide complementary tooling/data. EuRoC body-frame ground truth needs conversion to the evaluated camera frame. RGB-D/stereo/IMU inputs must not be silently used in the monocular baseline.

### QA-04 — Add small, useful continuous checks and change gates [P1]

- [ ] Run syntax/static checks, focused pytest tests, and dependency consistency on the supported Python/runtime matrix. Include at least one job with the real native optimizer; isolate optional GUI tests.
- [ ] Keep synthetic fixtures small and deterministic. Public-data evaluations and the local dashcam run can be documented/manual jobs when input distribution is impractical.
- [ ] Add a comparison report that checks pose coverage, failure counts, residuals, accuracy where available, latency, and memory together. Set regression tolerances only after baseline variability is measured.
- [ ] Introduce one algorithmic change per experiment. Separate formatting, dependency upgrades, geometry fixes, and parameter tuning so a reviewer can attribute changes.
- **Acceptance:** a proposed improvement includes the affected unit/native tests, the required dashcam comparison when applicable, held-out evidence for accuracy claims, and an explicit list of unverified behavior.

## 11. Current libraries, standards, and research options

Sources were checked on **2026-09-15**. These are candidates and references, not a claim that one stack is universally best or that every listed release supports this Mac. Pin and test selected versions. The local OpenCV runtime is 5.0.0, while the fetched official `/4.x` pages resolve to 4.13.0; verify runtime signatures before translating those examples. Likewise, development documentation is not proof that a released wheel supports Python 3.14.

| Choice | Recommendation for this project | Prerequisites and checks | Primary source |
| --- | --- | --- | --- |
| NumPy + OpenCV + SciPy | Retain as the classical CPU baseline; use native operations after fixing contracts | Stable supported APIs, descriptor dtype/distance, coordinate units, numerical equivalence, native build metadata | [NumPy migration guide](https://numpy.org/doc/stable/numpy_2_0_migration_guide.html), [OpenCV](https://docs.opencv.org/4.13.0/), [SciPy cKDTree](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.cKDTree.html) |
| g2opy / g2o | Keep behind a small tested optimizer boundary first | Verify current vertex/solver names, camera convention, kernel ownership, ABI and actual graph convergence | [g2opy package](https://pypi.org/project/g2opy/), [g2o](https://github.com/RainerKuemmerle/g2o) |
| Ceres Solver or GTSAM | Evaluate only if the existing backend blocks required residuals or inference | Separate migration benchmark; manifold/gauge/Jacobian tests; Python/native distribution; no simultaneous wholesale rewrite | [Ceres modeling](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html), [GTSAM](https://github.com/borglab/gtsam) |
| Matplotlib | Practical first simple viewer candidate | Main-thread desktop validation, bounded redraw rate, headless import separation, correct pose/color conventions | [Interactive guide](https://matplotlib.org/stable/users/explain/figure/interactive_guide.html) |
| Open3D | Evaluate for richer cloud viewing, filtering, selection, and I/O | Check released arm64 wheels before adopting. Retrieved stable 0.19 docs list Python 3.8–3.12; development docs list 3.10–3.14. Keep estimation filtering separate | [Stable install](https://www.open3d.org/docs/release/getting_started.html), [development install](https://www.open3d.org/docs/latest/getting_started.html) |
| PyAV / FFmpeg | Evaluate as a timestamp-aware decoder/export layer | Verify time bases and stream selection; avoid accidental dual decoder native-library conflicts on macOS; do not count decode-only timing as SLAM | [PyAV time](https://pyav.org/docs/stable/api/time.html) |
| evo + TUM/KITTI formats | Optional evaluation/export tooling | Explicit transform direction, timestamps, alignment, coverage, and protocol. evo is GPL-3.0-or-later; record license when packaging tools | [evo repository](https://github.com/MichaelGrupp/evo) |
| ORB-SLAM3 / DBoW2 | Architectural and external baseline; vocabulary-based retrieval candidate | Keyframes and map persistence first; geometric verification still required. ORB-SLAM3 is GPLv3 with C++ dependencies; its old tested Ubuntu setup is not verified Apple Silicon support | [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3), [DBoW2](https://github.com/dorian3d/DBoW2) |
| LightGlue with an appropriate extractor | Optional learned-matching experiment after baseline correctness | Specify pixels, descriptor dtype/distance, confidence, image scaling/color/range, weights/hash, memory/device/runtime. LightGlue code/weights are Apache-2.0; SuperPoint has separate restrictions | [LightGlue](https://github.com/cvg/LightGlue), [SuperPoint license](https://github.com/magicleap/SuperPointPretrainedNetwork/blob/master/LICENSE) |
| MASt3R-SLAM | Separate modern dense monocular SLAM research comparator | Upstream specifies CUDA/PyTorch/Python 3.11 setup; no established compatibility with this Mac environment. Code is CC BY-NC-SA 4.0; pretrained components have separate terms. Verify online/causal input access in any comparison | [MASt3R-SLAM](https://github.com/rmurai0610/MASt3R-SLAM), [license](https://github.com/rmurai0610/MASt3R-SLAM/blob/main/LICENSE.md) |
| Kalibr | Offline calibration/readout research when camera or IMU data warrants it | Rolling-shutter tool is a calibration aid, not a drop-in motion compensator; documented rolling-shutter path has sensor-scope limitations | [Kalibr rolling-shutter calibration](https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration) |
| ROS REP 103/105 | Adopt explicit axis/unit and frame-separation conventions for interoperability | No mandatory ROS runtime; mark arbitrary monocular units and unobserved gravity explicitly | [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst) |

### LIB-01 — Require evidence before promoting an experimental frontend/backend [P2/P3]

- [ ] Keep a selectable classical ORB baseline with frozen evaluation settings.
- [ ] Define an adapter contract for coordinates, image preprocessing, descriptor dtype/dimension/distance, match indices, score meaning, timestamps, device, and failure behavior. A floating-point learned descriptor must not use ORB Hamming thresholds.
- [ ] Record exact code and weight hashes, upstream source, licenses, runtime environment, cold-start cost, peak memory, and inference device. Keep model downloads explicit and weights outside commits.
- [ ] Evaluate the same dashcam frames plus independent ground-truth sequences. Compare full pose coverage, drift, latency, and memory; published CUDA timings are not measurements of this Mac.
- [ ] Separate offline methods that inspect future frames from causal online SLAM. Do not call a visually compelling dense reconstruction a real-time localization improvement without measuring those properties.
- **Acceptance:** the experiment improves a stated SLAM objective beyond measured baseline variability, has an acceptable runtime/distribution path, and preserves the classical fallback. No PyTorch/training infrastructure is needed for the P0/P1 repairs.

## Definition of done

An implementation task is complete only when its behavior is implemented and the relevant evidence is recorded:

1. The original defect or missing behavior has a concrete reproduction or acceptance scenario.
2. Coordinate direction, units, shapes, scale status, and observation ownership remain consistent across producers and consumers.
3. Appropriate deterministic geometry/bookkeeping tests and real native-backend checks pass.
4. The specified dashcam clip is used for applicable frontend, pose, map, performance, and runtime comparisons, with manifest and failure/coverage reporting.
5. Accuracy claims use appropriate independent ground truth; this uncalibrated/unreferenced dashcam alone supports robustness/performance observations, not absolute positioning accuracy.
6. Measured improvements include their costs and limits: memory, latency, missing poses, retained observations, hardware, and untested modes.
7. Documentation and CLI examples describe what exists; artifacts carry provenance; optional research dependencies do not become mandatory without evidence.

**First recommended deliverable:** a runnable headless program with corrected resize calibration, two-camera triangulation, checked pose recovery, reciprocal observations, and a small native regression suite. Then produce the first complete, explicitly calibrated-or-approximate report on all 1,800 frames of `GRMN2734.MP4` before tuning performance or adding larger SLAM features.
