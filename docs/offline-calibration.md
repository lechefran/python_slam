# Offline camera calibration

Use `python-slam-calibrate` (installed package) or `python -m calibration`
(from this checkout) to fit a **pinhole** camera from checkerboard or ChArUco
photographs. It runs headlessly with the existing NumPy/OpenCV dependencies;
it does not import g2opy or the display backend.

The output works directly with `slam.py --calibration`. This is the offline
fitting part of CAM-03, not a measured calibration for `GRMN2734.MP4`.

## Prepare captures

1. Use a rigid, flat printed board. Measure its square side in **metres** after
   printing; do not rely on a printer's nominal scale. For ChArUco, also measure
   marker side length and record the exact dictionary. Old ChArUco layouts may
   differ: this tool uses the current OpenCV layout, with no legacy-pattern mode.
2. Keep the actual camera/lens, focus, zoom, recording resolution, crop and
   stabilization settings consistent with the intended video. Export raw video
   frames if photo mode has different optics or processing. Do not resize,
   rectify or crop images before calibration. Variable digital stabilization
   can invalidate the assumption of one fixed camera model.
3. Aim for 20–30 sharp fitting views, with different distances, horizontal and
   vertical tilts, and board positions covering the centre **and edges**.
   Avoid motion blur, glare, a bent board and many adjacent stationary frames.
4. Reserve a separate capture burst of 6–10 varied images for validation.
   Place it in a separate directory **before fitting**. Do not repeatedly tune
   the model using those validation images and then call them held out.
5. Supported files are PNG, JPEG, BMP and TIFF directly inside each directory,
   sorted by filename. PNG exported from the original recording is preferred.
   EXIF display rotation is ignored; all decoded image dimensions must agree.
   Videos are not directly ingested by this first version.

The minimum is **8 detected fitting views and 3 detected validation views**.
These are operating safeguards, not proof of sufficient calibration geometry.
The tool detects exact duplicates of decoded grayscale pixels, including
across splits; it cannot detect all near-duplicates or shared capture sessions.

## Checkerboard example

A board with **10 × 7 squares** has **9 × 6 inner corners**. The arguments below
count inner corners, and assume measured 25 mm squares:

```sh
.venv/bin/python -m calibration \
  --images output/calibration-captures/fit \
  --validation-images output/calibration-captures/validation \
  --board checkerboard --columns 9 --rows 6 --square-size 0.025 \
  --camera 'camera-and-lens-identifier' \
  --recording-mode '1920x1080 29.97fps; fixed focus; crop/zoom/stabilization settings' \
  --output output/camera-v1.json \
  --report output/camera-v1-report.json
```

A checkerboard must be fully detected. The detector uses OpenCV's sector-based
checkerboard corner detector (`findChessboardCornersSB`).

## ChArUco example

For ChArUco, columns and rows count **squares**, not inner corners. A 7 × 5 board
with 40 mm squares, 30 mm markers and the `DICT_4X4_50` dictionary uses:

```sh
.venv/bin/python -m calibration \
  --images output/calibration-captures/fit \
  --validation-images output/calibration-captures/validation \
  --board charuco --columns 7 --rows 5 \
  --square-size 0.04 --marker-size 0.03 --dictionary DICT_4X4_50 \
  --camera 'camera-and-lens-identifier' \
  --recording-mode '1920x1080 29.97fps; fixed focus; crop/zoom/stabilization settings' \
  --output output/camera-charuco-v1.json \
  --report output/camera-charuco-v1-report.json
```

Partial ChArUco boards are supported when at least six non-collinear corners
are detected. IDs preserve correspondence to the correct physical board corners.
Use the current OpenCV board generator and confirm the printed layout matches
its dimensions and dictionary. See the official [ChArUco calibration workflow](https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html).

## What is fitted and evaluated

- The fitted model is zero-skew pinhole, independent `fx/fy/cx/cy`, with five
  Brown–Conrady coefficients in OpenCV order: `k1, k2, p1, p2, k3`.
- Only fitting observations enter `calibrateCameraExtended`. There is no automatic
  image removal, outlier refitting or model selection against the validation set.
- Board coordinates are metres on `Z=0`; each reported rotation/translation maps
  **board coordinates into camera coordinates**. They are not the SLAM trajectory.
- Each validation image estimates its own board pose with **frozen intrinsics and
  distortion**. The score checks camera-model consistency conditional on that
  estimated pose; it is not an independent pose prediction or SLAM accuracy test.
- Residuals compare predicted distorted pixels with original detected pixels at
  source resolution. RMS means `sqrt(mean(du² + dv²))`, weighted per corner.
- The report includes per-image RMS, 95th-percentile and maximum error, plus a
  4-column × 3-row spatial grid of counts, RMS and mean signed residual vectors.
  Empty cells contain `null`, not zero error. More corners in a cell mean more
  weight; occupied cells alone do not prove adequate geometric diversity.
- Intrinsic standard deviations are OpenCV's local fit estimates with their
  parameter order recorded. They are not calibrated confidence in real-road
  positioning, and fixed/unestimated parameters can have zero entries.

### Failure and review policy

Malformed configuration, unreadable images, mixed dimensions, duplicate images,
insufficient detections, degenerate points, grossly repeated board positions,
invalid depth, or non-finite/implausible fits fail. Gross plausibility checks
require the principal point inside the source image and focal lengths between
0.05 and 20 times its longest dimension. These broad guards are not physical
lens identification. No camera JSON is produced for those failures; a diagnostic
report is saved for failures after configuration validation when possible.

A successful fit has `checks_passed` or `needs_review` in its report. Review is
recommended if either split exceeds 1 pixel RMS, occupies fewer than 9 of 12
image cells, or fitting board normals span less than 10 degrees. Individual
images over 1 pixel RMS are flagged. These thresholds are documented heuristics,
not universal calibration acceptance limits. Warnings do not silently discard
images or prevent saving an inspectable fit. Exit code 0 means files were
written; exit code 2 means failure.

Every output camera is labelled **`measured_unverified`**, even when checks pass.
Inspect residual patterns and validate against independent captures before use.
A low average error can hide poor coverage, a wrong lens model or changing
intrinsics. Fisheye fitting, rolling-shutter estimation and time-varying camera
models are outside this initial tool.

## Output and SLAM integration

New camera profiles and quality reports both use version 2. Legacy camera files remain readable; see [camera profiles](camera-profiles.md). The camera retains the existing runtime keys
`model`, `width`, `height`, `K`, and `distortion`. Additional metadata records
camera/recording mode, board geometry and units, UTC fitting time, quality status,
and the report path/SHA-256. The report stores dependency versions, source-file
and decoded-pixel hashes, detected correspondences, fitted board poses and the
quality metrics. Keep the report alongside the camera profile.

Output files must be new `.json` paths. Existing files, symlinks and identical
camera/report paths are refused so a failed experiment cannot replace an earlier
profile. All input images are processed at their original resolution; the
existing SLAM loader performs its usual resize and distortion remapping later.
The loader labels any supplied calibration `provided`, not certified.

```sh
.venv/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --calibration output/camera-v1.json --headless \
  --report output/calibrated-dashcam-v1.json
```

Run this only with captures matching that dashcam's camera and recording mode.
The road video alone is not calibration-board data. Calibration does not supply
monocular metric scale or ground-truth trajectory accuracy.

## Validation evidence

The focused tests cover independently projected cameras with known nonzero
radial/tangential distortion, held-out perturbations, actual detection/fitting
of rendered checkerboards and ChArUco boards, partial ChArUco ID alignment,
loader/resize compatibility, duplicate and dimension rejection, failed fits,
and preservation of existing output files. The fixtures establish implementation
behavior; real camera accuracy still requires physical calibration captures.

### Local qualification (2026-09-16)

The installed console command was exercised from `/private/tmp`, away from the
checkout, with 16 rendered fitting images and 4 validation images. It produced
0.069 px fitting RMS and 0.085 px validation RMS, while correctly retaining a
`needs_review` warning for limited validation-image coverage. A separate import
probe confirmed calibration does not load SLAM, g2opy or the GUI modules.
Artifacts: `output/offline-calibration-installed-2026-09-16/` (ignored).

For the required real-video check, 12 fitting images at source frames
0, 150, …, 1650 and 4 validation images at 75, 525, 975, 1425 were decoded from
`sample_videos/GRMN2734.MP4` at 1920×1080. All 16 were reported as
`board_not_found`; the command exited 2, saved a failure report and produced no
camera file. This is a **negative-input check**, not a calibration or trajectory
benchmark. Artifacts: `output/offline-calibration-dashcam-2026-09-16/` (ignored).

The installed command is available in this workspace at
`.venv-portability/bin/python-slam-calibrate`. The full regression suite passes **147 tests**, including both installed
entry points. Both existing environments pass `pip check`; `.venv` retains its original desktop OpenCV installation. Native
validation was performed on macOS with OpenCV 5.0.0. Linux execution and
real-camera calibration quality have not been established by these tests.

## Quality report review (schema 2)

New calibration profiles now use schema 2 with the same camera-model fields plus verified provenance; see [camera profiles](camera-profiles.md).
The **quality report is now schema 2**; it retains the version 1 fields and adds
structured review checks, per-corner signed residuals, distinct-view spatial
support, edge-band coverage, capture summaries, and a fitting/validation
comparison. The report embeds the fitted camera parameters and explicitly states
that detection/fitting used raw source pixels without resize, crop or rectification.
The camera file's report SHA-256 covers the final report, including these checks.

### Open an offline HTML review

Add an optional HTML output to a normal fitting command:

```sh
.venv-portability/bin/python -m calibration \
  --images output/calibration-captures/fit \
  --validation-images output/calibration-captures/validation \
  --board checkerboard --columns 9 --rows 6 --square-size 0.025 \
  --camera 'camera-and-lens-identifier' \
  --recording-mode '1920x1080 29.97fps; fixed focus; crop/zoom/stabilization settings' \
  --output output/camera-v2.json --report output/camera-v2-report.json \
  --report-html output/camera-v2-review.html
```

Or render a **saved version 1 or 2** report without reopening images or refitting:

```sh
.venv-portability/bin/python -m calibration_report \
  output/camera-v2-report.json --output output/camera-v2-review.html
```

The installed equivalent is `python-slam-calibration-report`. All HTML paths must
be new files; existing outputs are preserved. The HTML is self-contained, with
no JavaScript, remote resources or source-image dependencies. Camera identifiers,
paths and other supplied text are escaped. The JSON remains the complete numeric
evidence, including file hashes and detected point coordinates.

The HTML contains:

- An overall status, actionable review findings and their observed values/thresholds.
- Fitting versus held-out RMS, P95, maximum residual and board-normal span.
- Separate spatial grids with residuals, corner counts and **distinct view counts**.
- Edge-band support and images ranked by RMS, with individual review reasons.
- A capture audit, including images where a board was not found.
- The camera model, named OpenCV local standard deviations with units, policy,
  board dimensions, preprocessing information and qualification limits.

A failed fit can also write an HTML report explaining the failure and examined
captures. Early failure may leave some images unexamined; the audit is explicitly
partial. No-board failures do not acquire camera estimates. If optional HTML
writing fails after the camera and JSON are saved, the command exits 2 and states
that those two files remain available; render the saved JSON separately.

### Review policy and interpretation

All quality checks are **post-fit diagnostics**. They never remove observations,
change optimization weights, refit intrinsics or select a model using held-out data.
Any flagged check sets the report to `needs_review`. Successful camera files
remain `measured_unverified`; `checks_passed` is not a certificate.

| Check | Review condition | Meaning / next action |
| --- | --- | --- |
| Overall or individual-image RMS | >1 source pixel | Inspect board/corner quality, blur and lens-model suitability |
| Individual-image residual tail | P95 >2 px or max >5 px | A few bad corners can be hidden by a small global average |
| Image RMS relative to its split | >max(0.25 px, median + 3 × 1.4826 × MAD) | Inspect unusually poor views; not an automatic rejection |
| Occupied spatial cells | <9 of 12 | Capture board corners across more of the image |
| Repeated spatial support | <9 cells with at least 2 distinct views each | Many corners in one photograph do not provide multiple viewpoints |
| Each outer edge band | <2 views containing corners in that band | Add captures near that edge; bands occupy the outer 10% of width/height |
| Fitting board-normal span | <10 degrees | Capture stronger horizontal and vertical tilts |
| Validation gap | Validation RMS >2× fitting RMS **and** gap >0.25 px | Investigate capture/mode mismatch or poor generalization; retain an independent validation set |

MAD is the median absolute deviation of **per-image RMS**, calculated separately
for fitting and validation. Its 1.4826 scale factor is a conventional robust scale
estimate; these heuristic flags are not hypothesis tests or calibrated confidence.
The 0.25 px floor avoids flagging meaningless roundoff differences. The validation
RMS ratio is `null` if fitting RMS is at most 1e-9 px; its absolute gap remains
available. Both gap conditions prevent an enormous ratio alone from flagging
otherwise tiny errors.

Spatial grids have 4 columns and 3 rows, with image origin at the top left. Empty
cells have `null` residuals, **not zero error**. Signed residual vectors use
`predicted_uv - detected_uv`, in source pixels, aligned with each view's detected
corner order. Edge corner regions contribute to both adjacent edge bands.
A view counts once per cell/band, regardless of how many corners it contributes.
Near-duplicate images can still exaggerate independent support. Band occupancy
does not imply that its entire length was observed.

The per-image `board_hull_fraction` is the convex hull area of detected corners
in pixels squared divided by source image area. It includes space between
corners and must not be read as the fraction of pixels directly measured.

The underlying standard-deviation conventions follow [OpenCV's calibration
reference](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html). These are local
fit estimates, not a replacement for independent captures or trajectory truth.
No new runtime dependency or SLAM tracking/mapping policy is introduced.

### Quality report qualification

The full regression suite passes **158 tests** in `.venv-portability`, including
all three installed entry points. Both existing environments pass `pip check`.

The same 16/4 rendered checkerboard image split retains 0.069 px fitting RMS and
0.085 px held-out RMS. Compared with the previous tool's saved camera, the largest
intrinsics change was 9.1e-13 and the largest distortion change was 1.5e-13
(numerical roundoff). The richer report records 11 review findings, primarily
limited repeated spatial support and edge coverage. It does not reinterpret the
low fitting error as evidence of complete image coverage.

The installed report command rendered the new JSON from `/private/tmp`; a saved
version 1 report also rendered successfully. Report hashes and packaged module
contents were checked. HTML tests check content, text escaping, empty/missing
measurements, failure reports and preservation of existing files. No browser
surface was available for visual layout inspection in this session.

The 16 previously sampled `GRMN2734.MP4` frames were reprocessed: 12 fitting and
4 validation images had no board detections. The new JSON and HTML correctly
report failure, with no camera output. This remains a negative-input test, not a
real-camera calibration or SLAM accuracy result.

Artifacts (ignored): `output/calibration-quality-2026-09-16/` and
`output/calibration-quality-dashcam-2026-09-16/`. The synthetic HTML example is
`output/calibration-quality-2026-09-16/review-final.html`.
