# Recovery against older keyframes

## Behavior

Recovery is enabled by default and runs **only after normal map-based tracking
fails**. Use `--no-recovery` for the earlier last-reference-only behavior. The
existing robust-weighting option remains separate and opt-in.

Normal tracking keeps its 30-source-frame landmark recency limit. Recovery can
use older landmarks associated with a selected historical view. It estimates
the current camera in the existing world frame and scale; it does not reset
the map, accumulate a new unit translation, or use an identity-pose fallback.

This is bounded map-based recovery, not loop closure, global map correction or
a full place-recognition system. The staged pattern—candidate retrieval, PnP,
projection search, then pose validation—is also visible in the primary
[ORB-SLAM2 tracking implementation](https://github.com/raulmur/ORB_SLAM2/blob/master/src/Tracking.cc).
This implementation uses its own conservative gates and a small archive; it
does not import ORB-SLAM2's vocabulary, thresholds or backend.

## Archive and search bounds

- Keep at most **64** recovery representatives, sampled from committed cameras
  at least **0.5 seconds** apart. Initialization seeds the first representative.
- When full, remove the interior representative with the smallest neighboring
  time interval. Preserve the first and newest representatives and gradually
  thin crowded periods.
- Store references to existing frame objects. BA pose changes and culled
  feature-to-landmark links remain authoritative; no detached XYZ/pose cache
  can silently become stale.
- Exclude the current normal reference and views less than 0.5 seconds older
  than the input. Search only live landmark descriptors in the remaining views.
- Explicitly remap compact descriptor rows to original feature slots after
  culling. Keep unique current features and unique landmarks aligned.
- Rank candidates by mapped descriptor-match count, breaking ties by recency.
  Try PnP on at most the **three** strongest candidates with at least 30 matches.

These representatives serve recovery only. Mapping, bundle adjustment and
triangulation retain their existing scheduling. Bounding this archive does not
bound the application's full historical map memory. Descriptor retrieval is
still a linear scan of at most 64 views, not an indexed visual-word database.

## Validation before commit

For each shortlisted view:

1. Estimate a provisional `T_cw` from its matched live world XYZ and current
   pixel observations, using existing centered RANSAC/refinement and conditioning
   checks. No old camera pose is accepted as the current pose.
2. Project the chosen view's remaining live landmarks, using its stored
   descriptors. Only this recovery search bypasses the normal recency limit.
   Keep positive depth, image bounds, 5-pixel search radius and the existing
   Hamming/ratio filters.
3. Re-estimate from the expanded correspondences, with the original final
   finite-SE(3), positive-depth, 3-pixel residual, image-span and Jacobian gates.
4. Require at least **30 original descriptor correspondences** and at least
   **50% of the original mapped descriptor matches** to survive that final fit.
   Projected additions cannot satisfy this original-support requirement.
5. Commit the camera and only its validated reciprocal observations once.
   Set it as the next normal tracking reference. Defer new-point triangulation
   until a subsequent normal tracking frame; the recovery frame adds no new
   landmarks. Existing scheduled guarded BA/culling can still run.

Each candidate starts independently. Tracking/native PnP failures reject that
candidate and retain the existing map. If all candidates fail, the frame remains
lost with its original tracking-failure reason. Blank/low-feature inputs avoid
archive matching entirely. Failed estimates never enter the recovery archive.

The original-support thresholds are conservative operating policies, not
calibrated false-recognition probabilities. Repeated structures or moving
objects can still produce convincing matches. Held-out negative/revisit data,
real calibration and independent trajectory truth are still required.

## Diagnostics and use

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --report output/recovery-run.json

.venv-portability/bin/python -m scripts.benchmark_tracking output/recovery-comparison \
  --focus-start 850 --focus-end 1799 --every 100 \
  --baseline output/robust-reference-2026-09-15/report.json

.venv-portability/bin/python -m scripts.benchmark_recovery output/recovery-revisit
```

Every frame result includes nullable `recovered_from`; the report also includes
`recovered_frames`. Recovered frames retain status `tracking`, so consumers can
continue to handle a single accepted-tracking state. The CLI prints recovery
events even between its normal progress intervals.

With diagnostics enabled, `stages.recovery` records archive size, scanned views,
mapped match counts, attempted keyframe IDs/ages, candidate PnP/projection
evidence, rejection reasons, original inlier support and the chosen view.
`normal_tracking_failure` preserves the rejected normal path. After successful
recovery, top-level PnP/projection stages and numeric traces describe the pose
actually committed. Normal-stage timings still describe the normal attempt;
the separate `recovery` timer covers retrieval and all recovery candidates.

Sampled overlays label the recovered keyframe, and every successful recovery
inside the requested diagnostic window is captured even between regular sample
intervals. Failed recovery retains the original normal-stage rejection evidence.

## Validation

Synthetic tests project known nonidentity cameras independently of production
geometry. They exercise matching and native PnP on landmarks last seen 148
frames earlier; recovery returns the known pose and does not mutate the map
until commit. Other checks cover:

- archive capacity, temporal spacing and preserved endpoints;
- culled descriptor-slot alignment and deleted landmarks;
- unique correspondences, wrong geometry and too-small geometric consensus;
- native solver failures and the three-candidate limit;
- one-time commit, reciprocal observation integrity and reference changes;
- unchanged map/reference on failure and no recovery calls during healthy tracking;
- enabled/disabled operation with and without diagnostic capture.

On this macOS arm64 environment, `.venv-portability/bin/python -m pytest -q`
passes **95 tests**, including recovery capture outside the sampling stride
without a status transition. The rebuilt wheel's installed CLI was exercised
from `/private/tmp` in both recovery modes on a 25-frame synthetic clip: both
completed with 24 accepted poses, 3,061 landmarks and identical final poses.
Installed runtime module bytes match the checkout, and `pip check` passes.
This verifies headless execution on this host; Linux execution and native GUI
behavior were not requalified by this change.

### Controlled test using recorded dashcam pixels

The reproducible helper reads source frames 0–119, injects 45 blank inputs, and
then revisits source frames 30–59 at input frames 165–194. Input time stays
monotonic at the source FPS. This is an **artificial revisit**, not the natural
route, and has no independent physical trajectory truth. A separate decode
check confirms that seeking reproduces source frames 30–59 pixel-for-pixel.

| Measure | Recovery disabled | Recovery enabled |
| --- | ---: | ---: |
| Accepted poses over 195 inputs | 123 | 146 |
| Tracked revisit frames out of 30 | 7 | 30 |
| First tracked revisit input | 188 | 165 |

Input 165 recovers from keyframe **30**, a gap of **135 frames / 4.5045 seconds**,
with **89 of 94 original descriptor matches** surviving. All 45 blank inputs
remain lost; the successful recovery inserts no new points. Subsequent inputs
resume ordinary tracking. Single-run times were 20.56 and 23.82 seconds with
overlapping benchmarks; these are not controlled performance ratios.

Ignored evidence is in `output/keyframe-recovery-revisit-2026-09-16/`, including
the video hash, source hashes, per-input source-frame mapping, poses and overlays.
The source is `/Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4`,
SHA-256 `9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`.
The test uses the existing approximate intrinsics and default 2,000 ORB budget.

### Full natural dashcam replay

Both modes decode all 1,800 source frames from frame zero, using width 1,024,
the existing approximate focal default of 525 source pixels, 2,000 ORB features,
no bottom mask, seed 0 and one OpenCV thread. Centering and spatial replenishment
remain enabled. Compare against the same mode before older-keyframe recovery:

| Measure | Default | Opt-in robust weighting |
| --- | ---: | ---: |
| Accepted poses before recovery | 1,796 | 1,792 |
| Accepted poses with recovery | 1,796 | 1,793 |
| Successful recoveries | 0 | 1 |
| Lost frames after initialization | 0 | 3 |
| Retained earlier accepted frame identities | 1,796 / 1,796 | 1,791 / 1,792 |

The default replay preserves all original per-frame outcome/count fields and
final optimized poses **exactly**; healthy tracking never invokes recovery.
It retains 51,030 landmarks. Initial source frames 1–4 remain unposed, as before.

Robust mode recovers frame **1,760** against keyframe **1,740**. It also tracks
1,765 subsequently, but newly loses **1,762** compared with its earlier run.
Its final lost frames are **1,719, 1,762 and 1,763**, with 51,927 landmarks.
Four frames attempt recovery, totaling six candidate fits. This does **not**
qualify robust weighting as a default: increasing the count alone does not
preserve the previous accepted frame identities. Robust weighting stays opt-in.

Ignored artifacts:

- `output/keyframe-recovery-default-2026-09-15/`, compared with
  `output/robust-reference-2026-09-15/report.json`;
- `output/keyframe-recovery-robust-2026-09-15/`, compared with
  `output/robust-consensus-recovery-2026-09-15/report.json`.

Each contains the original report and benchmark manifest. Companion
`recovery-summary.json` files were generated after completion with explicit
report hashes, recovery counts and frame-identity comparisons; they do not
replace the original evidence. Single-run elapsed times were 488.76 and 496.02
seconds under overlapping workloads. No speed or real-road accuracy improvement
is claimed. Held-out natural revisits and false-recovery rates remain unqualified.
