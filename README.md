# Python SLAM

A small classical **sparse monocular SLAM prototype**: ORB image features, calibrated two-view initialization, map-based PnP tracking, triangulated 3D landmarks, and native g2opy bundle adjustment. It runs headlessly or with a main-thread Matplotlib image/3D viewer.

Camera poses are world-to-camera transforms. Translation and map coordinates have **arbitrary scale**. There is no loop closure, general relocalization, dense reconstruction, or persistent top-down map yet. The image panel is a tracking view, not a 2D map. Planned improvements and point-data interchange are in [TODO.md](TODO.md).

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
| `--mask-bottom FRACTION` | Optional fixed exclusion mask for hood/dashboard; default 0 |
| `--seed N`, `--threads N` | OpenCV random seed and worker count; defaults 0 and 1 |
| `--report FILE` | JSON environment/configuration, input hashes, frame outcomes, BA results and accepted `T_cw` poses |
| `--hold` | Keep the desktop view open at the end |

`F` and `SEEK` provide legacy defaults overridden by explicit CLI options. `REVERSE` is ignored with a notice: translation sign comes from two-view cheirality.

Frames are reported as `initializing`, `initialized`, `tracking`, or `lost`. Only accepted poses are inserted into the map; the initialization reference is also retained once the initial pair succeeds. After loss the tracker retries the last accepted view; there is no place-recognition recovery or silent restart into a different scale. Inspect `states` and `pose_coverage` in reports, not only exit status.

Exit codes: **0** means processing completed/was closed and a map was initialized (tracking gaps may exist); **1** means processing failed; **2** means invalid arguments or no map initialized; **130** means interrupted. GUI hold/pause time is included in total wall time; per-frame processing excludes decoding, resizing/rectification, and rendering. Do not label either number alone as real-time SLAM throughput.

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
