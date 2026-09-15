# Runtime repair and platform validation

## Supported installation target

The application is pure Python around native NumPy/OpenCV/SciPy/g2opy wheels. It no longer imports legacy `g2o`, SDL, Pangolin or PyOpenGL. GUI code is optional and uses Matplotlib in the main thread.

Target CPython 3.11–3.14, Apple Silicon macOS, or 64-bit glibc Linux. Linux binary availability depends on Python ABI, CPU architecture and glibc, not just the distribution's name. glibc 2.28+ is the documented initial Linux target; Alpine/musl and older/32-bit systems are not qualified. The Dockerfile supplies a Debian/Python 3.12 headless environment for hosts able to run it.

`g2opy==2.3.0` is pinned because its concrete API is tested (`VertexPointXYZ`, `LinearSolverEigenSE3`, `SBACam`, `EdgeProjectP2MC`). Other compatible version bounds are in `pyproject.toml`; the exact clean macOS environment is recorded in `requirements-macos.lock`. Install dependencies in a fresh environment so full and headless OpenCV wheels never collide.

References: [g2opy distributions](https://pypi.org/project/g2opy/), [OpenCV wheel variants](https://pypi.org/project/opencv-python/), [Matplotlib event-loop integration](https://matplotlib.org/stable/users/explain/figure/interactive_guide.html). Wheel resolution is a packaging check; only execution on the target OS can validate runtime compatibility.

## Validation results

Repair host: macOS 27.0 arm64, CPython 3.14.7. All results below belong to this repair, not older files left in `output/`.

- Created a clean `.venv-portability` from the declared package and extras. Core, GUI, test dependencies and the installed entry point built successfully. Both the original `.venv` and clean environment pass `pip check`.
- All 16 regression tests passed in the clean installed environment. The original environment passed the 15 core tests; the installed-entry-point check requires package installation. Native tests cover calibrated rotated-camera triangulation, essential pose direction, positive depth, resized intrinsics, PnP outlier rejection, observation conflicts/deletion, stale-point removal, large graph IDs, fixed-anchor BA convergence, and failure handling. The installed command also ran a 30-frame synthetic clip successfully from outside the checkout.
- The synthetic CLI test decodes 25 frames, initializes/tracks, inserts landmarks, and executes accepted native BA. It also checks help, invalid paths/frame limits, input overwrite rejection and headless import isolation.
- Native macOS window observed on a 30-frame synthetic clip: camera image, labelled 3D cloud, camera trajectory and final held state rendered. Pressing Q closed the app; the process exited 0 and wrote its report. This validates native rendering, final hold and Q cleanup. Pause/resume, toolbar interactions, Linux GUI backends and all interruption paths were not manually qualified.
- Linux x86-64 CPython 3.12 core dependencies resolved to manylinux wheels using pip's cross-platform dry-run. This does **not** execute Linux code. Docker/Podman and a Linux host were unavailable; the new Ubuntu/macOS CI matrix is pending its first remote run.

### Required dashcam run

Input: `sample_videos/GRMN2734.MP4`; SHA-256 `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`.

Command:

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --max-frames 1800 --report output/runtime-repair/dashcam-full.json
```

Configuration: seed 0, one OpenCV thread, 2000 ORB features, no exclusion mask, 1024×576 processed images, default **approximate** source focal length 525. No calibration or ground truth was supplied.

The full run decoded **1,800 frames**, retained **932 accepted poses**, and ended with **30,371 landmarks**, exiting 0. Frame outcomes were 5 initializing, 1 initialized, 930 tracking, and 864 lost; the earlier initialization reference accounts for the extra accepted pose. First loss occurred at source frame 923, with intermittent recovery through frame 940; subsequent frames stayed lost. BA accepted 177 solves and rejected 9 candidate updates. Recorded total wall time was **178.30 seconds**; this is a single run and overlapped a small GUI validation run, so it is not a controlled performance comparison. The original code failed before decoding any frame; there is no valid original SLAM throughput to compare against. The JSON records failed frames rather than inserting identity poses. No accuracy claim follows from these counts or from a plausible cloud.

Local ignored evidence: `output/runtime-repair/dashcam-full.json`, `gui.json`, and `linux-x86_64-py312-dependencies.json`. The synthetic video generator and regression tests are checked in; real footage and generated outputs are not.

## What changed

- Fixed the parser/import/API blockers and removed mandatory graphics dependencies from the core.
- Added a validated CLI, fresh per-run state, bounded execution, camera input/rectification, deterministic seed/thread settings, status reports and cleanup.
- Corrected intrinsics after resize, transform conventions, two-camera DLT, cheirality selection and checks before homogeneous/depth division.
- Kept a classical ORB frontend with aligned empty outputs, unique descriptor matches and explicit evidence failures. Essential estimation initializes a scale; robust PnP then tracks that same map scale.
- Added conservative initialization support, provisional pose/map search followed by full acceptance, and older-view triangulation to replenish landmarks under small inter-frame parallax.
- Enforced reciprocal observation insertion/removal. Native BA uses graph-local IDs, selected local/boundary observations, distinct robust kernels, and validated atomic writeback. Culling is independent of graph membership and batches projection by camera.
- Added portable packaging, regression tests, Linux container instructions and a cross-platform CI workflow.

## Remaining limitations

This is a working baseline, not completion of every task in TODO.md. Recovery after sustained loss, held-out trajectory accuracy, real calibration, dynamic-object rejection, formal uncertainty/degeneracy handling, keyframe/long-term memory budgets, persistent 2D mapping and portable point-data export remain further work. It retains historical frame features and mature points, so memory and some work still grow with sequence length. BA checks support/connectivity and finite, non-worsening results; these are not a full observability or convergence proof.

Current numeric thresholds are explicit baseline settings, not universal SLAM standards: 1-pixel essential RANSAC threshold converted to normalized units, 1-degree triangulation angle, 3-pixel reprojection/PnP gates, 60 initial points, 12 PnP inliers, and 10% image spans. The larger initialization support and older triangulation partner address observed immediate tracking loss; they are not tuned/qualified on held-out data. Validate camera geometry and scene coverage before changing these values.
