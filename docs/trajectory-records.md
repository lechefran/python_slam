# Trajectory records and corrections

`Map.trajectory` owns lightweight records independently of `Map.frames`:

- Each processed source frame retains ID, timestamp, original status/reason,
  and an optional accepted world-to-camera pose (`T_cw`, 4×4).
- Initialization can later accept an earlier reference without rewriting its
  original processing outcome. Lost/uninitialized records have no fabricated pose.
- Accepted records retain their **first accepted pose** (`T_cw_initial`) separately
  from subsequent corrections. This is insertion history, not an independently
  estimated odometry trajectory: later tracking can already use corrected maps.
- Records contain immutable numeric tuples and scalar metadata, never images,
  descriptors, observations, or Frame references.

## Reference transforms

Selected mapping keyframes are independent roots (`is_keyframe: true`). Each
accepted non-keyframe processed by SLAM references the latest selected keyframe
and stores `T_cr`, mapping reference-camera coordinates into its own camera:

```
T_cr = T_cw @ inverse(T_reference_world)
T_cw = T_cr @ T_reference_world
```

The multiplication order is intentional for world-to-camera transforms. Both
records must have accepted poses and matching submap/scale context. This runtime
uses submap 0 with arbitrary monocular scale; it does not infer metres or merge
unrelated maps. References must be keyframes, so dependencies have one level and
cannot form cycles. Promoting a record to keyframe removes its reference link.

## Atomic corrections and retained frames

`Trajectory.update_poses()` validates the entire proposed correction before
publishing it. Explicitly supplied camera estimates take precedence over relative
propagation and refresh their `T_cr` against the corrected reference. Historical
records without an independent camera estimate retain `T_cr` and follow the root.

Current BA still independently constrains retained mapping frames. It passes
those IDs as `independent_ids`: fixed/out-of-graph frames keep their accepted
poses, and their relative transforms are refreshed when a reference changes.
This prevents keyframe propagation from moving cameras outside the validated BA
solution. Once a frame expires from the cache or is retired, its record follows its
reference without retaining descriptors or observations. Invalid or
failed optimization leaves the trajectory unchanged.

`reanchor_dependents(old_id, new_id)` atomically moves direct dependencies between
accepted keyframes in the same context, recomputing relative transforms while
preserving world poses exactly. This prepares for future keyframe retirement;
it does not itself delete a keyframe, change the selection list, or remove map
observations. Reanchoring may use a later accepted keyframe. Cross-submap links,
self-links, unaccepted references and non-keyframe roots are rejected.

## Reports and scope

The existing JSON `poses` list and viewer continue to use corrected `T_cw` values.
The additive `trajectory` report section (schema 1) includes all source records,
reference IDs/transforms, keyframe flags, initial poses and submap/scale labels.
It documents transform direction and initial-pose semantics explicitly. This is
correction provenance, not a resumable map serialization format.

[Keyframe selection](keyframe-selection.md) and bounded non-keyframe storage
are implemented (bounded storage is opt-in); opt-in [guarded retirement](frame-retirement.md) can also
remove redundant keyframes after reanchoring. [Shared-landmark BA selection](local-map-selection.md)
is now opt-in; the temporal baseline remains the default. Total-map memory bounds and Sim(3)/loop corrections remain separate work. Direct registered-ID mutation
or pose mutation outside the validated map-update path is unsupported.

## Validation

Analytic rotated-camera tests cover composition order, repeated corrections,
explicit/retained-pose precedence, promotion, same-map reanchoring without pose
jumps, cross-map rejection and atomic failure. Native BA verifies propagation to
a dependent record with no mapping Frame and unchanged history on solver failure.
CLI tests independently reconstruct relative poses from the serialized report.

Qualification (2026-09-23, macOS arm64): **211 tests pass** and both virtual
environments pass dependency consistency checks. Full `GRMN2734.MP4` replay:
1,800 processed frames, 1,796 accepted poses, no tracking losses, 51,030 landmarks,
360 keyframes and 1,436 linked non-keyframes. Frame outcomes, keyframe decisions
and final optimized poses match the previous keyframe-selection baseline exactly.
An independent reader reconstructs every linked pose from its exported relative
transform and reference; maximum absolute matrix-element difference is
`3.55e-15`. This checks serialization/composition consistency, not road accuracy.

Ignored `output/trajectory-corrections-2026-09-23/` contains the command,
source/video hashes, report, log and independent comparison. Tests overlapped
part of replay, so no timing comparison is claimed. Native Linux execution and
native GUI operation were not qualified in this change. Frame retirement remains
disabled; tests cover reference propagation to records without retained frames.

## Bounded feature storage

With experimental `--tracking-cache-size N`, full frames survive only as
selected mapping keyframes or cached views. The default retains all accepted
frames because the bounded policy currently regresses dashcam coverage:

- `--tracking-cache-size N` keeps the last N successfully accepted current
  frames (minimum 2; qualification uses 64). Failed input frames cannot evict the last valid
  tracking reference. The initialization pair remains as permanent keyframes.
- Recovery keeps its existing separately bounded archive of up to 64 views;
  `--no-recovery` removes this allowance. Thus retained non-keyframes are bounded
  by **N + 64** (or N without recovery), with overlap counted only once.
- After pose/keyframe/recovery decisions, expiry unlinks both sides of each
  observation and retires landmarks left with fewer than two measurements.
  Maturity and quality history use the existing observation-removal path.
- Expired cameras keep timestamp, status/reason, initial/current pose and relative
  reference transform in the trajectory. Subsequent keyframe corrections propagate
  to these records without keeping a Frame object. Lost frames retain no pose.
- `frame_storage` reports cache capacities, retained keyframes/non-keyframes,
  released full frames and trajectory record count. Guarded keyframe retirement
  remains a separate opt-in policy; its ID list counts only that policy's removals.

The bound applies to cached full frames, not total memory: selected keyframes,
landmarks, lightweight trajectory records and report diagnostics can still grow.
The current in-flight frame (and a pending initializer before map creation) also
uses temporary feature storage. Small caches can shorten triangulation baselines
and alter BA constraints; changing the capacity requires a coverage comparison.

### Bounded-storage qualification (2026-09-24)

Full `GRMN2734.MP4` comparison at source frames 0–1799, width 1024,
approximate source focal 525, seed 0, one OpenCV thread, temporal local BA,
recovery enabled and `--tracking-cache-size 64`:

| Measure | Retain-all baseline | Bounded storage |
| --- | ---: | ---: |
| Timestamped trajectory records | 1,800 | 1,800 |
| Accepted poses | 1,796 | 1,701 |
| Lost frames | 0 | 95 |
| Retained full frames | 1,796 | 419 |
| Selected keyframes | 360 | 342 |
| Live landmarks | 51,030 | 11,595 |

Bounded storage retains 77 cached non-keyframes, below the 128-view combined
allowance. Coverage regresses, including sustained late loss after frame 1709;
therefore this remains **opt-in**. Expiry changes landmark support and fixed BA
measurements; identifying which losses require better selection or constraint
preservation is follow-up work, not a reason to relax pose acceptance thresholds.
No real-road accuracy or total-memory bound is established. Tests overlapped
replay, so elapsed time is not a controlled performance comparison.

Ignored `output/frame-cache-2026-09-24/` contains full reports, replay logs and
independent trajectory/composition checks. Synthetic tests cover bounded frame
lifetime (including garbage collection), observation integrity, weak-landmark
removal, invalid-reference rejection, lost-frame cache preservation, and pose
corrections after expiry. **251 tests pass**, and dependency consistency passes.
Two bounded replays produce identical final poses; independent relative-pose
composition differs by at most `3.55e-15`. Native Linux and GUI operation remain
unqualified.
