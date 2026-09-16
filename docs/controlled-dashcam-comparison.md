# Controlled dashcam comparison

`python -m scripts.compare_dashcam` runs paired camera-model experiments from
source frame zero. It fixes algorithm settings, uses fresh processes, verifies
inputs and implementation hashes, and produces a reviewable JSON/Markdown report.
The default input is `sample_videos/GRMN2734.MP4`.

**This does not establish trajectory accuracy.** The supplied road clip has no
independent ground-truth trajectory or measured matching camera profile in this
checkout. Approximate-camera repeatability is a separate qualification from a
measured-calibration comparison.

## Compare an approximate baseline with a measured profile

First obtain board captures matching the camera and recording mode, and produce
a [camera profile](camera-profiles.md). Inspect its quality report before using it.
Then run:

```sh
.venv-portability/bin/python -m scripts.compare_dashcam \
  output/dashcam-calibration-comparison \
  --candidate-calibration output/my-camera/camera.json \
  --candidate-settings output/this-video-settings.json
```

The baseline defaults to the explicitly approximate source focal 525 model.
To compare two supplied calibrations, add `--baseline-calibration` and optionally
`--baseline-settings`. Candidate calibration must be a schema-2 profile with a
verified quality-report sidecar. A verified hash is not certified camera accuracy;
`needs_review` profiles and incomplete recording declarations remain visible in
`camera_provenance`. Resolution/FPS/settings mismatches detected by the profile
loader fail before the experiment starts. Legacy baseline files remain supported.

By default this replays the entire video **four times**, sequentially: baseline,
candidate, candidate, baseline (two pairs, `--repeats 2`). This alternating order
reduces first-run/cache order bias. It does not eliminate background load, thermal
changes, decoder caching, or native solver variability. Close competing workloads
when gathering timing evidence. Use more pairs when assessing repeatability;
do not infer a speedup from a single timing difference.

For a bounded run through frame 999, including the historical frame-941 region:

```sh
.venv-portability/bin/python -m scripts.compare_dashcam \
  output/dashcam-calibration-first-1000 \
  --candidate-calibration output/my-camera/camera.json \
  --candidate-settings output/this-video-settings.json \
  --max-frames 1000 --repeats 1
```

A bounded run qualifies only that prefix. Both arms still start at frame zero;
starting at frame 941 would initialize a different map and would not reproduce
the accumulated state at that frame. If the requested limit exceeds a known
video length, the expected horizon is the shorter full video. Unknown-length
inputs require an explicit limit and must decode that entire limit.

## Same-camera repeatability control

When no matching measured calibration is available, exercise the harness honestly:

```sh
.venv-portability/bin/python -m scripts.compare_dashcam \
  output/dashcam-repeatability \
  --repeatability-control --max-frames 1000 --repeats 1
```

This applies the same baseline camera to both arms. It must never be labelled a
calibration improvement. If a baseline profile/settings file is supplied, both
arms use it. Repeatability checks compare per-frame outcomes, final poses and
landmark retention across **all** runs, excluding timings and output paths.

## What is controlled

- Identical video SHA-256, source frame identities/timestamps, replay horizon,
  processed dimensions, runtime environment and algorithm configuration.
- Seed 0, one OpenCV thread, maximum width 1024, 2,000 ORB features, bottom mask 0,
  and source focal 525 by default. These common settings have CLI overrides.
- Centered PnP, spatial replenishment, older-view recovery and observation history
  enabled; robust PnP and landmark maturity disabled in both arms. These are
  explicitly fixed for the current calibration baseline, not inherited CLI defaults.
- Optional `--feature-mask` applies the same **source mask** to both arms. Changed
  calibration may legitimately change rectification, invalid borders and the
  effective mask; the comparison reports this as part of the calibration treatment.
- Tracking diagnostics are collected in both arms with the new `slam.py
  --diagnostics` flag, without image overlays or detailed landmark-history exports.
  The flag also works independently with the normal SLAM CLI.
- Each arm launches the same interpreter in a new subprocess. All runs are
  sequential. Commands, source hashes (including uncommitted implementation),
  Git revision, tracked diff hash, dependencies and native-thread environment
  variables are recorded. Untracked implementation files are covered by source
  hashes even though Git's tracked diff does not contain them.
- Video, profile, provenance, settings, mask and implementation files are hashed
  before/after each run. Changing them invalidates the experiment. The source
  captures used for fitting are not reopened; their hashes remain in the profile's
  linked quality report.

The output directory must be new. An interruption or failed subprocess preserves
completed evidence/logs and marks the experiment failed. No partial run is
presented as a comparable complete horizon. A completed replay that tracks fewer
than two poses is still useful negative evidence and can be compared; its low
coverage remains explicit.

## Outputs and interpretation

- `manifest.json`: exact commands, controlled settings, expected horizon, input
  and code fingerprints, implementation revision and timing definitions.
- `pair-NN-baseline/` and `pair-NN-candidate/`: full SLAM `report.json` and `run.log`.
- `comparison.json`: per-pair validation, exact coverage changes, metrics and
  descriptive timing deltas. Each run includes its report hash and outcome hash.
- `README.md`: a compact result table and interpretation limits.

Comparison rejects failed/truncated horizons, inconsistent pose IDs/counts,
changed source/timestamps, mismatched environments/policies, different source
masks and missing diagnostics. Invalid pairs expose reasons and no quality
comparison. Exit 0 means a comparable experiment completed, **not** that the
candidate improved tracking. Exit 1 means invalid/interrupted experiment or a
failed repeatability control; invalid command/preflight inputs exit 2.

Coverage is compared by **accepted source frame ID**, not just totals. A candidate
that loses frame 941 and gains another frame must not appear to preserve coverage.
Lost intervals are listed separately from initialization frames without poses.
Recovery events and failure reasons/stages remain visible.

Residual and spatial metrics are reported for each arm's accepted frames and for
the intersection of accepted frame IDs. A P95 over per-frame P95 values is not a
pooled residual percentile. Missing measurements are `null`, never zero error.
Even on common frames, different cameras can select different features, depths
and rectified pixels; smaller residuals alone do not prove more accurate geometry.

Landmark metrics include total created, live, retired and live fraction, plus
existing maturity/observation-history aggregates. Live fraction is lifetime
bookkeeping, not an age-matched survival curve or static-scene probability.
Landmark IDs do not establish correspondence between independently built maps.

Timing has three distinct scopes:

1. Subprocess wall time includes startup, imports, hashing and report export.
2. SLAM pipeline elapsed includes decode/preprocessing/tracking, but excludes
   startup before its timer and report serialization/hash work after it.
3. Per-frame and stage timings describe tracker processing with diagnostics;
   they exclude video decode, resizing and remapping.

The report deliberately does not calculate cross-calibration matrix-distance,
ATE or RPE scores. Monocular maps can have different scales/origins. A measured
trajectory-accuracy comparison requires independent, synchronized ground truth
and an explicit alignment/evaluation protocol.

## Local validation

The regression suite passes **187 tests** on the macOS development host. Synthetic
native replays cover AB/BA ordering, repeatability, schema-2 candidate profiles,
wrong-resolution preflight failures, provenance preservation, exact pose identity
changes and empty common-frame populations. Dependency checks pass; the installed
SLAM module matches the source used for testing. Linux execution and real-road
accuracy are not established by these tests.

An initial real-video attempt was deliberately interrupted because it overlapped
the test workload. Its output is retained under
`output/controlled-dashcam-1000-2026-09-16/` with status `failed`; it is excluded
from qualification. The clean run uses
`output/controlled-dashcam-qualified-2026-09-16/` after tests/builds finished.

### Real-video control result (source frames 0–999)

The clean same-camera control completed with **identical per-frame outcomes,
final optimized poses and landmark-retention counts** in both fresh processes:

| Measurement | Baseline arm | Control arm |
| --- | ---: | ---: |
| Decoded frames | 1,000 | 1,000 |
| Accepted poses | 996 | 996 |
| Tracking-loss frames | 0 | 0 |
| Created landmarks | 47,874 | 47,874 |
| Live landmarks | 31,143 | 31,143 |
| Retired landmarks | 16,731 | 16,731 |
| Pipeline elapsed, seconds | 218.790 | 218.814 |

Frame 941 is tracked in both arms. There are no newly unposed or newly accepted
frame IDs between arms. The 1,000-frame outcome/count prefix also matches the
previous `output/observation-history-enabled-2026-09-16/report.json` exactly.
Final poses were **not** compared against that longer historical run because
later bundle adjustment changes earlier poses; the equal-length control poses
were compared exactly with each other.

There are 994 accepted frames with final-PnP residual evidence (the initial
map's two poses lack that stage). Their median per-frame inlier median is
1.1913 processed pixels, and median effective spatial-cell count is 3.5557 in
both arms. These are tracking diagnostics, not real-road positioning errors.

This was one A/B pair using the existing approximate focal 525 model. It qualifies
the harness and the diagnostics on this prefix, **not** a measured-calibration
improvement, full 1,800-frame coverage, long-run map bounds or a speedup. Synthetic
tests additionally exercise the default AB/BA order and profile-comparison path.

See `output/controlled-dashcam-qualified-2026-09-16/README.md`,
`comparison.json`, `manifest.json` and `prior-prefix-check.json` for the retained
evidence. A real measured-profile comparison remains pending matching physical
camera captures/profile.
