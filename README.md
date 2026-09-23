# Python SLAM

A small classical **sparse monocular SLAM prototype**: ORB image features, calibrated two-view initialization, map-based PnP tracking, triangulated 3D landmarks, and native g2opy bundle adjustment. It runs headlessly or with a main-thread Matplotlib image/3D viewer.

Camera poses are world-to-camera transforms. Translation and map coordinates have **arbitrary scale**. There is no loop closure, general relocalization, dense reconstruction, or persistent top-down map yet. The image panel is a tracking view, not a 2D map. Planned improvements and point-data interchange are in [TODO.md](TODO.md).

Trajectory history is stored independently of mapping frames, with reference transforms that preserve keyframe corrections; see [trajectory records](docs/trajectory-records.md). [Mapping keyframes](docs/keyframe-selection.md) are selected with explicit geometric/support evidence; frame retirement is not yet enabled.

## Setup on macOS or Linux

Use **CPython 3.11–3.14** and a fresh virtual environment. The initial binary-install target is Apple Silicon macOS and 64-bit glibc Linux (x86-64/ARM64, glibc 2.28 or newer). This is not a guarantee for every Linux distribution, architecture, or Python build. Alpine/musl, 32-bit systems, and older glibc need a separately validated native build or the container below.

```sh
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install '.[test,viewer]'
.venv/bin/python -m pip check
.venv/bin/python -m pytest -q
```

For a server without a desktop, install `'.[test]'` or just `.`. The core uses `opencv-python-headless`; **do not install another OpenCV wheel into the same environment**, because they share `cv2`. Matplotlib supplies the optional GUI. No SDL, Pangolin, PyOpenGL, CHOLMOD installation, or manual Python path changes are needed.

On Debian/Ubuntu, install `python3-venv` if venv support is missing. A desktop viewer requires an interactive Matplotlib backend; `python3-tk` is an option for Tk on distribution Python. A headless Linux server requires no display server. See [platform and validation notes](docs/runtime.md) for tested versus unverified environments and native dependency references.

## Reproducible synthetic smoke run

```sh
.venv/bin/python scripts/generate_demo.py output/demo.avi --frames 30
.venv/bin/python slam.py output/demo.avi --focal 400 --headless \
  --max-frames 30 --report output/demo-report.json
```

This fixture verifies decoding, tracking, map insertion, and native optimization. It does not establish real-road accuracy. Its known focal length is **400 source-image pixels**; pass that value as shown.

## Run the dashcam clip

```sh
.venv/bin/python slam.py sample_videos/GRMN2734.MP4 --headless \
  --max-frames 1800 --report output/dashcam-report.json
```

The clip is local and ignored by Git. Without `--calibration`, the source focal length defaults to **525 pixels** and the principal point to the image centre. These are approximate values, not a measured calibration for this dashcam. Intrinsics are scaled with the actual processed dimensions, capped at 1024 pixels wide by default.

The initial repaired full-clip run decoded 1,800 frames and retained 932 accepted camera poses; tracking was lost in the latter part. It completed processing and reported the loss rather than inventing poses. See [measured results and remaining limits](docs/runtime.md#validation-results). Runtime success is not a claim of accurate or continuous localization.

For the desktop image/3D view:

```sh
.venv/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --max-frames 120 --hold --report output/viewer-report.json
```

Press **Space** to pause/resume and **Q/Esc** to close. The Matplotlib toolbar supports view manipulation. `--hold` keeps the final map open; close it to finish writing the run report. Quit, EOF, and processing errors release the video and viewer resources.

## Camera calibration

Use the headless [offline calibration tool](docs/offline-calibration.md) to fit a
camera from checkerboard or ChArUco photographs, with a separate held-out image
set and a detailed quality report. Run `python-slam-calibrate --help` after
installation, or `.venv/bin/python -m calibration --help` in this checkout.
Physical captures must match the camera and recording mode; the dashcam road
clip alone is not calibration input.

Add `--report-html output/calibration-review.html` for a standalone visual review
of errors, edge coverage and flagged images. To review an existing JSON without
refitting, use `python-slam-calibration-report report.json --output review.html`.
See the [quality report guide](docs/offline-calibration.md#quality-report-review-schema-2)
for threshold meanings and qualification limits.

New calibrations are [reproducible camera profiles](docs/camera-profiles.md), with
structured `--camera-settings`, stable content IDs and verified quality-report
provenance. Inspect one with `python-slam-camera-profile camera.json`. SLAM checks
decoded dimensions/FPS and supplied recording settings, and records unverified
fields explicitly. Legacy calibration files remain supported.

Use the [controlled dashcam comparison](docs/controlled-dashcam-comparison.md)
to replay identical inputs with baseline/candidate camera models in fresh
processes, compare exact pose coverage and common-frame diagnostics, and retain
run provenance. Without a measured dashcam profile, use its explicitly labelled
repeatability-control mode.

Supply a JSON file describing the **decoded source image**, for example this schema illustration (values below are not a calibration for `GRMN2734.MP4`):

```json
{
  "model": "pinhole",
  "width": 640,
  "height": 360,
  "K": [[400, 0, 320], [0, 400, 180], [0, 0, 1]],
  "distortion": [0, 0, 0, 0, 0]
}
```

```sh
.venv/bin/python slam.py path/to/video.mp4 --calibration path/to/camera.json \
  --headless --report output/calibrated-run.json
```

The runner validates the source dimensions and pinhole matrix, rescales the intrinsics, and rectifies images with the supplied distortion coefficients. Estimation and BA then use rectified pixels. Calibration files describe the actual recording mode, including any crop or stabilization. Fisheye and time-varying camera models are not implemented; rectify those inputs externally with matching calibration. A supplied calibration is labelled `provided`, not automatically certified accurate.

## Useful options and result semantics

Run `.venv/bin/python slam.py --help` for the complete interface. The installed equivalent is `.venv/bin/python-slam`.

| Option | Meaning |
| --- | --- |
| `--headless` | No viewer module or GUI backend is imported |
| `--start-frame N` | Skip exactly N decoded source frames; start a fresh map |
| `--max-frames N` | Bound the number of frames processed |
| `--width N` | Maximum processed width; preserve aspect ratio with rounded height |
| `--focal F` | Approximate focal length in source pixels when calibration is absent |
| `--calibration FILE` | Validated pinhole calibration input |
| `--features N` | ORB feature cap (default 2000) |
| `--condition-pnp` / `--no-condition-pnp` | Centered pose fitting with bounded consensus refits is enabled by default; disable for the legacy reference |
| `--spatial-mapping` / `--no-spatial-mapping` | Replenish sparse image cells from a longer accepted-camera baseline; enabled by default |
| `--robust-pnp` / `--no-robust-pnp` | Opt-in block-Huber pose refinement; conditioning checks are always active. Default off pending dashcam coverage qualification |
| `--recovery` / `--no-recovery` | Recover failed tracking against a bounded archive of older accepted views; enabled by default |
| `--mask-bottom FRACTION` | Optional fixed exclusion mask for hood/dashboard; default 0 |
| `--feature-mask FILE.png` | Source-resolution binary grayscale mask: black excludes feature centres, white allows them; combined with the bottom strip |
| `--landmark-maturity` / `--no-landmark-maturity` | Opt-in candidate validation and active-only pose/recovery/BA support; default off pending coverage qualification |
| `--observation-history` / `--no-observation-history` | Bounded passive landmark evidence; enabled by default |
| `--quality-report FILE` | Detailed JSON with individual observation histories and a bounded retired-landmark sample |
| `--seed N`, `--threads N` | OpenCV random seed and worker count; defaults 0 and 1 |
| `--report FILE` | JSON environment/configuration, input hashes, frame outcomes, BA results and accepted `T_cw` poses |
| `--diagnostics-dir DIR` | New directory for sampled tracking overlays and numeric evidence; use with `--report` |
| `--diagnostics-start N`, `--diagnostics-end N` | Inclusive source-frame capture window (defaults 850–1,000); does not skip warm-up frames |
| `--diagnostics-every N` | Sample every N frames within the window; also capture status transitions and the final frame |
| `--hold` | Keep the desktop view open at the end |

`F` and `SEEK` provide legacy defaults overridden by explicit CLI options. `REVERSE` is ignored with a notice: translation sign comes from two-view cheirality.

Frames are reported as `initializing`, `initialized`, `tracking`, or `lost`. Only accepted poses are inserted into the map; the initialization reference is also retained once the initial pair succeeds. Failed normal tracking can recover against a bounded archive of older views, with `recovered_from` identifying the selected keyframe. Failed recovery retains the last valid reference and map scale. Inspect `states`, `pose_coverage` and `recovered_frames` in reports, not only exit status.

The desktop dashcam view shows ORB features as small rings: **cyan** for detected
features and **green** for features associated with live 3D landmarks. These use
the tracker's actual pixel locations on corners and textured edges; they are
not a separate edge detector. Press **O** to hide/show the markers, including
while paused; **Space** pauses and **Q/Esc** closes the viewer. The overlay is
display-only and does not change tracking or the source video.

With `--landmark-maturity`, new landmarks remain candidates until a later
accepted observation and adequate parallax support promotion. The viewer uses
**magenta** for candidates and **green** for active landmarks. A limited initial
bootstrap avoids requiring mature landmarks before the first map-based pose.
See [landmark maturity](docs/landmark-maturity.md) for lifecycle rules,
diagnostics, and qualification limits. This option is off by default.
The experimental dashcam run accepts only three poses, versus the unchanged
1,796-pose default, because mature support loses image coverage at frame 7.
Use the default policy for normal dashcam runs.

[Observation quality histories](docs/observation-quality.md) record insertion
residuals, assessed re-observation searches, later culling checks and retirement
without changing tracking decisions. Aggregate counts appear in normal reports;
`--quality-report output/quality.json` saves individual bounded histories. Lost
frames and unsearched points are not counted as landmark matching failures.

### Configurable exclusion masks

Use `--feature-mask mask.png` to exclude the dashboard, timestamps, or other
fixed image regions from feature extraction. The file must be a single-channel
8-bit PNG containing only 0 (black, exclude) and 255 (white, allow), with the
same dimensions as the decoded source video. Images are not cropped and
intrinsics are unchanged. The mask follows image resizing and rectification;
pixels containing excluded or invalid border contributions are also excluded.
`--mask-bottom` applies an additional strip in the final processed image.

The viewer and diagnostic images tint exclusions orange. **M** toggles that
display layer, including while paused; it does not enable or disable masking.
Reports include the mask hash, coordinate convention and excluded fraction.
See [mask creation, behavior and measured results](docs/exclusion-masks.md).

Exit codes: **0** means processing completed/was closed and a map was initialized (tracking gaps may exist); **1** means processing failed; **2** means invalid arguments or no map initialized; **130** means interrupted. GUI hold/pause time is included in total wall time; per-frame processing excludes decoding, resizing/rectification, and rendering. Do not label either number alone as real-time SLAM throughput.

## Diagnose tracking loss

From the repository root, replay the dashcam from frame zero and inspect frames 850–1,000:

```sh
.venv-portability/bin/python -m scripts.benchmark_tracking output/tracking-investigation
```

Use `.venv/bin/python` instead if that is your installed environment. The output directory must be new. The command records input/configuration identities, source hashes, all frame results, a focus-window CSV and summary, and sampled PNG/JSON evidence. Add `--baseline output/runtime-repair/dashcam-full.json` when that local earlier report exists. Diagnostics observe the existing estimators; failed candidate poses stay outside the accepted trajectory.

See [tracking investigation](docs/tracking-diagnostics.md) for reproduction, evidence definitions, measured findings, and remaining uncertainty.

PnP refinement now validates its initial and refined hypotheses and can try one VVS fallback when LM fails or worsens the candidate cost. See [pose-refinement behavior and evidence](docs/pose-refinement.md); coverage and residual acceptance thresholds remain unchanged.

[Numerical conditioning](docs/numerical-conditioning.md) documents centered fitting, seed recovery and consensus refits. [Spatial support](docs/spatial-support.md) adds bounded landmark replenishment and concentration diagnostics. Together they retain every baseline accepted frame and reach 1,796 poses on the requested 1,800-frame dashcam replay, with no tracking loss after initialization. Real-road accuracy remains unqualified.

[Robust pose fitting](docs/robust-pose.md) adds residual weighting during optimization and checks the weighted/unweighted pose Jacobian. Conditioning checks are always active; `--robust-pnp` opts into weighted updates. Before older-keyframe recovery, its full dashcam replay retained 1,792 of the baseline's 1,796 poses. With recovery it accepts 1,793, but newly loses frame 1,762 relative to that earlier robust run. Weighted updates remain off by default while these regressions are unresolved.

[Older-keyframe recovery](docs/keyframe-recovery.md) adds a bounded fallback after normal tracking fails. The default full dashcam replay preserves all 1,796 baseline poses exactly. A controlled revisit using the supplied dashcam pixels recovers across a 135-frame keyframe gap and tracks all 30 revisit frames, versus seven without recovery. This is separate from natural-route accuracy qualification.

## Linux container and CI

```sh
docker build -t python-slam .
docker run --rm \
  -v "$PWD/sample_videos:/data:ro" -v "$PWD/output:/output" \
  python-slam /data/GRMN2734.MP4 --max-frames 1800 --report /output/run.json
```

Create `output/` before mounting it. The Debian-based image runs headlessly. Docker and an actual Linux runtime were unavailable on the repair host, so this recipe is provided for validation on Linux, not represented as a locally tested image.

The checked-in GitHub Actions workflow runs real native tests and a synthetic headless clip on Ubuntu and macOS, Python 3.11–3.14. It has not been run remotely as part of this local change. GUI tests require a desktop session separately.

## Development

Keep math-heavy code documented with shapes, units, coordinate direction, and the reason for numerical safeguards. Tests use independently generated camera observations and exercise actual native g2opy projection/convergence. Videos, benchmark reports, and virtual environments stay outside commits.

The original project is MIT licensed; selected dependencies have their own licenses. See [LICENSE](LICENSE).
