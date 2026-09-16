# Configurable feature exclusion masks

## Purpose and scope

Exclude fixed regions such as a dashboard, hood or camera timestamp before ORB
feature extraction. Masking is opt-in with `--feature-mask`; existing unmasked
operation remains available. Invalid rectification borders are automatically
excluded whenever calibration rectification is active.

This is a static feature-centre mask. It does not classify moving vehicles,
certify stationary scene points, remove already-created landmarks, or prevent
an allowed keypoint's descriptor patch from spanning an excluded region.
Pyramid-scale patch protection and temporal landmark quality remain separate
work. The mask is fixed for a run; changing it requires starting a new run.

## File format and creation

Use a **single-channel, uint8 PNG** at the decoded source video resolution.
Allowed pixels must be **255 (white)** and excluded pixels **0 (black)**.
Color, alpha, 16-bit, intermediate gray values, unreadable files and mismatched
dimensions are rejected explicitly. Black polygons, rectangles and holes may
be painted with any tool that preserves this format; disable antialiasing.

For example, generate a 1920x1080 mask with a bottom strip and an optional
polygon. Coordinates refer to the original video, before resize/rectification:

```python
import cv2
import numpy as np

mask = np.full((1080, 1920), 255, dtype=np.uint8)
mask[840:, :] = 0  # Example only: inspect your own recording before choosing it.
# Arbitrary fixed regions can also be painted black:
# polygon = np.array([[20, 20], [300, 20], [300, 70], [20, 70]], np.int32)
# cv2.fillPoly(mask, [polygon], 0)
assert cv2.imwrite("mask.png", mask)
```

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --feature-mask mask.png --hold

.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --feature-mask mask.png --headless --report output/masked-report.json \
  --diagnostics-dir output/masked-frames --diagnostics-start 0 --diagnostics-end 300
```

No custom mask is installed as the default. An overly broad mask can remove the
vertical support needed to accept a pose. Entirely black masks are valid and
produce zero features; a CLI run that cannot initialize returns exit code 2.

## Coordinate and extraction contract

1. Verify mask dimensions against the first decoded source image. Hash the
   exact PNG bytes for provenance.
2. Convert the allow-mask to floating point [0,1] and resize with `INTER_AREA`,
   using the same integer dimensions as the image. Any mixed allowed/excluded
   support is rejected, preserving thin exclusions when downsampling.
3. If calibration rectifies the image, apply the same inverse remap with linear
   interpolation and zero border support. Pixels whose sampling footprint
   touches excluded pixels or the exterior are excluded. Floating-point values
   within 1e-6 of full support count as allowed.
4. Combine with `--mask-bottom`, which retains its processed-image convention:
   exclude rows starting at `int(height * (1 - fraction))`. Exclusions combine
   by union; neither mask can re-enable pixels forbidden by the other.
5. Pass the final binary mask to ORB. Recheck returned feature centres against
   the processed mask using `floor(pixel + 0.5)` to catch pyramid-boundary
   leakage. Filter pixel and descriptor rows together before constructing
   normalized rays, feature slots or the KD-tree.

The final mask is cached once per tracker and read-only. Its dimensions cannot
silently change. Intrinsics, source pixels, projection thresholds and pose
acceptance gates are unaffected by masking. Matching, projection association,
triangulation and older-keyframe recovery all consume the same filtered
features, so they cannot reintroduce excluded current-image feature centres.

## Display and reporting

- The viewer shows exclusions as an orange transparent layer. **M** toggles its
  visibility; **O** independently toggles ORB markers. Both work while paused.
- Saved diagnostic panels also tint the excluded pixels orange. Rendering
  uses copies/layers and does not alter extraction images or geometry.
- Reports include top-level `feature_mask` metadata: source path/hash,
  source/processed sizes, transform convention, rectification state, bottom
  fraction, effective excluded fraction and SHA-256 of row-major processed
  uint8 mask bytes. Sizes and convention accompany this hash.
- Extraction diagnostics report the excluded fraction. Feature traces contain
  the surviving centres, not hypothetical detections inside the mask.
- Benchmark comparison reports `same_feature_mask` separately from video,
  camera and solver compatibility. Source files should be retained with the
  run when reproducing custom masks.

## Validation

Tests cover source dimensions and malformed formats, integer and fractional
resizing, an independently specified half-pixel rectification shift and border
support, union with the bottom strip, post-pyramid descriptor alignment, native
ORB with holes/all-black masks, non-mutating display tint and CLI failure/report
behavior. See the benchmark results below for this recording's measured scope.

The macOS arm64 regression suite passes **108 tests**. Native ORB also produced
identical keypoint/descriptor arrays with an all-white mask and no mask. A
40-frame native `macosx` viewer check on the supplied video verified excluded
feature centres, unchanged source images, independent M/O toggles while paused,
and map integrity. The rebuilt installed command was run outside the checkout
on a 25-frame synthetic clip with a 100x100 corner exclusion plus a 5% bottom
strip: it completed with 23 accepted poses and 2,056 landmarks. The installed
command also rejected a source-size mismatch explicitly. Installed module bytes
match the checkout, and `pip check` passes. Linux execution was not requalified.

## Dashcam experiment

Use `/Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4` with the
existing approximate intrinsics (source focal 525), width 1024, 2,000 ORB budget,
seed 0, one OpenCV thread, centering/spatial mapping/recovery enabled and robust
weighting disabled. Both runs start from zero and decode all 1,800 frames.

The example mask excludes **source rows 840–1079** of the 1920x1080 recording,
22.22% of the image, and nothing else. This lower-image/dashboard example was
chosen before measuring outcomes. It is **not a recommended preset**.

| Measure | No custom mask | Example lower-image mask |
| --- | ---: | ---: |
| Decoded frames | 1,800 | 1,800 |
| Accepted poses | 1,796 | 2 |
| Lost frames after initialization | 0 | 1,795 |
| Final landmarks | 51,030 | 80 |

The unmasked run preserves every baseline per-frame outcome/count and every
final optimized pose **exactly**, compared with
`output/keyframe-recovery-default-2026-09-15/report.json`. All 17,904 feature
centres in the saved masked snapshots were independently checked against the
known processed boundary (row 448); none lie in the excluded region.

The masked run accepts only **two poses**, frames **0 and 4**, and retains 80
landmarks. Frames 5–10 fail image coverage; the remaining failures predominantly
have insufficient map correspondences. It records 1,795 lost frames and no
recoveries. This demonstrates a severe coverage regression with this region
choice under unchanged geometric safeguards. It does not establish which
baseline features were physically stationary or whether either map is accurate.
The application does not automatically apply this example mask or relax gates
to compensate for it. Further masking experiments need better scene support,
camera calibration and independent accuracy evidence.

Ignored evidence lives in `output/exclusion-masks-2026-09-16/`: source mask,
mask description, full reports/manifests, numeric feature traces, diagnostic
images, and `viewer-frame-000030.png`. Mask PNG SHA-256:
`8aafb2e2e4204a3c691454dc48845c8d717ef13b537e6cc277284c4dd659f7ee`.
Video SHA-256:
`9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`.
`qualification.json` records report hashes and exact baseline comparisons.
Single-run elapsed times were 454.72 seconds unmasked and 101.15 seconds masked,
with overlapping test/benchmark workloads.

Reproduce with new output directories:

```sh
.venv-portability/bin/python -m scripts.benchmark_tracking output/mask-baseline \
  --focus-start 0 --focus-end 1799 --every 300
.venv-portability/bin/python -m scripts.benchmark_tracking output/mask-comparison \
  --focus-start 0 --focus-end 1799 --every 300 --feature-mask mask.png \
  --baseline output/mask-baseline/report.json
```

The benchmark is a coverage/behavior comparison, not a speed or accuracy claim.
In particular, a quickly failing tracker does less mapping work; the failed
masked run's shorter elapsed time cannot be interpreted as a performance win.
