# Observation quality history

## Behavior

Observation history is enabled by default and is **passive**: it records what
the tracker did without changing correspondence selection, pose weights,
acceptance thresholds, maturity promotion, recovery ranking or culling policy.
Use `--no-observation-history` to disable its storage and processing overhead.
The experimental `--landmark-maturity` mode remains a separate, default-off
option; collecting history does not enable active-only tracking.

Each landmark retains **16 recent events** and lifetime counters. A map retains
at most **64 recently retired landmark snapshots**, plus aggregate event counts
that include retired landmarks. This limits additional history per landmark;
it does not bound the application's total historical map or report size.

## What the events mean

| Phase / outcome | Evidence |
| --- | --- |
| `observation/added` | A reciprocal observation was actually inserted; stores its residual and camera depth at insertion |
| `search/accepted` | A searched existing landmark was associated with the accepted current camera |
| `search/rejected` | A correspondence was proposed but not committed, despite a usable final projection |
| `search/unmatched` | Projection search ran but found no accepted descriptor proposal, with a usable final projection |
| `search/unassessed` | The final projection is invalid, outside the image, or excluded by the processed mask |
| `cull/retained`, `cull/rejected` | A later geometric check of an existing observation using current map geometry |
| `observation/culled`, `observation/removed` | An observation was explicitly unlinked; this does not erase its prior measurements |
| `retirement/*` | The landmark was retired or rejected; records its terminal reason |

Residuals are Euclidean distances in **processed rectified pixels**. Depth is
camera Z in **arbitrary monocular map units**. NaN, Infinity and unavailable
residuals are encoded as JSON `null`; finite negative depth remains visible as
evidence of invalid geometry. They are not replaced with zero error.

Every event distinguishes `observation_frame_id` from `assessed_frame_id`.
For example, a frame-20 observation may be rechecked after optimization at
frame 100. Stored insertion residuals remain historical snapshots; later BA
does not rewrite them as if the improved geometry had been available earlier.
For newly triangulated points, the older observation is assessed at creation,
not retroactively claimed as a measurement known at its source frame.

Culling records rejected historical checks and the latest live observation's
check. It does not append every old retained residual on every culling pass.
An identical consecutive culling sample is deduplicated. Cull counts therefore
describe recorded checks, **not all optimizer edges or independent sightings**.

## Re-observation denominator

The lifetime successful-reobservation ratio is:

```text
search/accepted / (search/accepted + search/rejected + search/unmatched)
```

It is `null` if no eligible search has been assessed. There is at most one
search result per landmark per input frame, even if propagation, projection
search and candidate validation encounter the same point. Initial triangulation
observations are not counted as successful re-observation opportunities.

Search attempts are staged locally until a camera is accepted. They include
mapped descriptor proposals and landmarks actually searched by projection.
The accepted pose then determines whether unsuccessful attempts have positive
depth, in-image projection and allowed mask coordinates. A committed observed
feature is itself evidence of successful association even if its reprojection
lies just outside an image/mask boundary. An unseen/unsearched landmark does not
receive a fabricated miss. Occlusion is not modelled, so this is an operational
search yield, not a calibrated visibility or static-scene probability.

Failed tracking frames leave individual histories unchanged. The aggregate
`tracking/lost_unassessed` counter records these frames separately. When recovery
succeeds, only the winning recovery candidate's staged searches are assessed;
failed normal tracking and rejected recovery hypotheses cannot penalize points.
Calling recovery to obtain a proposal does not itself commit history.

## Reports and inspection

Per-frame and final trajectory reports include aggregate `observation_quality`.
For individual landmarks, request the separate detailed file:

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --report output/run.json --quality-report output/quality.json
```

The detailed file has `schema_version: 1` and
`kind: landmark_observation_quality`. It includes the video hash, camera/mask
metadata, configuration, environment, completion status, aggregate counts,
live landmarks and the bounded recently retired sample. Each point includes:

- stable point ID, creation frame, age in source frames and current live support;
- maturity state, or `unassessed` when maturity classification is disabled;
- historical first/last observed frame IDs, which survive observation removal;
- lifetime event counters and the explicitly defined re-observation ratio;
- recent added-observation residual count, median and maximum;
- the bounded event history, with assessment/observation frame IDs, outcomes,
  residuals, depth and feature indices.

Residual summaries use only finite `observation/added` values still in the
16-event window. They exclude culling and search samples and explicitly report
their sample count. Other event types can evict those residual samples; a null
summary is missing retained evidence, not zero error. Retirement snapshots
describe state at retirement, including zero remaining reciprocal links.
Bulk link cleanup during retirement contributes one retirement event; it does
not inflate the individual observation-removal counters.

The detailed exporter streams one point at a time to a temporary file and
publishes it by rename after completion. It does not allocate a second
whole-map JSON tree. CLI validation prevents reports or their temporary paths
from overwriting video, calibration, masks, or each other. A quality report
requires history to be enabled. Aggregate counts include all recorded events;
the most recent retired sample is not a complete deletion journal.

This is a diagnostic schema, not the planned MAP-08 2D/3D map interchange format.
It contains no calibrated covariance or dynamic-object classification. Its
evidence can support future quality-based policies, which need separate
regression/accuracy qualification before altering tracking.

## Validation

Tests cover bounded storage, per-frame search deduplication, unknown ratios,
phase-separated residual summaries, JSON null handling, immutable insertion
snapshots, duplicate observation links, final-pose/mask eligibility, unsearched
and lost-frame behavior, culling timestamps, retired samples, export round trips,
CLI path protection and exact native synthetic trajectory equality with history
enabled/disabled. Full dashcam comparisons are recorded below.

The regression suite passes **133 tests** on macOS arm64. The rebuilt installed
CLI was run outside the checkout with maturity enabled, both with and without
history: both 25-frame synthetic runs finish with 23 accepted poses and 2,053
landmarks, with identical optimized poses and maturity counters. The detailed
file round-trips through an independent JSON reader and matches the aggregate
report. Installed module bytes match the checkout; dependency checks pass.
No GUI change was needed. Linux execution and real-road accuracy were not
requalified by these checks.

### Full supplied dashcam replay

Both comparison arms decode all 1,800 frames of
`/Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4`, from frame
zero, with the existing approximate source focal 525, width 1024, 2,000 ORB
features, seed 0, one OpenCV thread, no exclusion mask, maturity/robust weighting
off and recovery/centering/spatial replenishment enabled.

| Measure | History disabled | History enabled |
| --- | ---: | ---: |
| Accepted poses | 1,796 | 1,796 |
| Lost frames after initialization | 0 | 0 |
| Final live landmarks | 51,030 | 51,030 |
| Exact final poses and original per-frame outcomes match baseline | Yes | Yes |

The baseline is `output/landmark-maturity-default-2026-09-16/report.json`.
Recording history preserves every accepted frame identity and every final
optimized pose exactly, rather than merely matching the pose count.

Recorded evidence includes 506,030 observation insertions, 346,398 successful
searched re-observations, 1,870,697 unmatched searches, 51,152 rejected proposals,
and 1,355 unassessed searches. The operational search yield is **15.27%** over
2,268,247 assessed searches. This includes broad map-projection searches and
association competition: it is not the PnP inlier fraction, scene-static
probability, or trajectory accuracy. There are 16,297 recorded culling
rejections/removals, 27,780 stale retirements and 1,006 outlier retirements.

The full detailed export contains all **51,030 live landmark records** and
**64 retired samples**. An independent line-by-line audit verified unique live
IDs, event histories of at most 16 entries, and assessment frames no earlier
than their observation frames. Its size is **169,664,614 bytes** (169.7 MB).
Detailed export is optional; bounded per-point history does not make a large
map's full export small.

Single-run elapsed times were 480.32 seconds with history disabled and 535.62
seconds enabled, under overlapping workloads. These include decode, processing
and sampled diagnostic artifacts, but exclude final JSON report/export writing.
They expose added bookkeeping cost; repeated isolated runs and memory profiling
are still needed for a performance qualification. History can be disabled when
that diagnostic cost is undesirable.

Ignored artifacts:

- `output/observation-history-enabled-2026-09-16/`: report, source manifest,
  overlays and detailed `quality.json`;
- `output/observation-history-disabled-2026-09-16/`: comparison arm;
- `output/observation-history-qualification-2026-09-16.json`: baseline/report
  hashes, exact-pose comparisons and export audit.

Video SHA-256:
`9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`.
