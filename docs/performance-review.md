# Performance and simplification review — 2026-09-24

Reviewed the tracking/feature frontend, map and observation ownership, local BA,
trajectory corrections, retirement, and viewer consumers. Changes are limited to
`dmap.py`; solver parameters, tracking gates, graph selection policies, and
retirement defaults remain unchanged.

## Findings addressed

1. **Repeated camera-support scans (P2).** BA scanned every selected observation
   once per graph camera. The 300-frame dashcam profile counted 15,693,181 inner
   checks. Build camera-to-point membership once and reuse its lengths for camera
   support. This changes counting work from cameras × observations to observations.
2. **Global candidate search for a local problem (P2).** Temporal BA scanned all
   landmarks and repeatedly searched their histories to establish local support.
   After integrity validation, local frame slots already provide exactly those
   landmarks. One Counter now supplies both the candidate set and local support
   counts. The existing total ordering (support, recency, stable ID) preserves
   selected points; shared-policy current-camera priority is also unchanged.
3. **Repeated observation-history lookup (P2).** Reverse integrity checks searched
   each landmark's frame list for every mapped feature. The forward pass already
   verifies each declared observation owns its frame slot and uses unique frames.
   Counting actual occupied slots then detects extra/dangling reverse links in
   linear work. Explicit index-type/range checks preserve rejection of invalid
   and negative indices. Extra slots, missing links, foreign points, duplicated
   observations and swapped slots have corruption regressions.
4. **Scalar optimized-geometry validation (P2).** Each graph edge projected a
   single point through the same camera. Validate the identical camera/point pairs
   in batches using the existing projection helper and unchanged finite/depth
   gates. No optimized coordinates or residual thresholds are altered.
5. **Temporary observation lists (P3).** Partition local observations and the
   earliest two boundary observations in one pass, retaining their original order.
   The same camera membership table now serves support counts and depth checks.

The changes deliberately avoid persistent caches or new ownership indexes, which
would add synchronization requirements to culling, retirement and rollback.

## Remaining priorities

- Native ORB descriptor matching dominated the baseline prefix profile:
  `knnMatch` consumed 20.716 of 54.131 profiler seconds. Reducing matching work
  needs a separate correspondence/coverage experiment; changing match candidates
  is not a behavior-preserving cleanup.
- Full observation-integrity checking and culling still visit historical map
  state. This pass removes redundant searches, not those correctness checks.
- Retained map/trajectory storage still grows. The guarded retirement policy
  removed no frames on the previous dashcam qualification; this review does not
  claim a memory bound or silently weaken its protections.

## Validation scope

Use the ignored `output/performance-review-2026-09-24/` artifacts for source
snapshots, profiles and comparison reports. The baseline is an unchanged source
snapshot from the start of this review. Prefix profiling uses source frames
`[0,300)`, the same approximate camera, seed 0 and one OpenCV thread. Baseline and
candidate profiles run separately without concurrent tests or replays. A single
profile pair measures instrumented work; it does not establish production FPS,
real-time performance, or held-out trajectory accuracy.

### Paired 300-frame profile

| Instrumented measurement | Baseline | Updated |
| --- | ---: | ---: |
| Total profile seconds | 54.131 | 49.009 |
| BA cumulative seconds (59 calls) | 10.779 | 5.858 |
| Integrity cumulative seconds | 2.211 | 2.135 |
| Projection calls, whole run | 186,518 | 19,696 |
| Python/native profiler call count | 61,305,951 | 42,730,255 |

BA cumulative time fell about 46% in this instrumented prefix; overall profiler
time fell about 9%. Integrity validation itself improved only slightly here;
its main benefit is removing repeated history searches as tracks grow. Exact
frame outcomes, selected graphs, final poses and trajectory metadata agree.
Treat the timing figures as a single controlled profile pair, not a general
speedup guarantee. `profile-comparison.json` contains the unrounded results.

### Regression results

- **239 tests pass**, including native optimization, retirement rollback,
  trajectory correction, both local-map policies and eight observation-link
  corruption cases. Dependency consistency passes in both local environments.
- Full requested dashcam replay: **1,800 frames, 1,796 accepted poses, zero
  tracking losses, 51,030 live landmarks**. Every compared per-frame outcome,
  final pose, trajectory record and BA result (excluding duration) exactly
  matches the saved pre-change baseline. `full-comparison.json` records this.
- The full replay overlapped tests during startup, so its elapsed time is not
  used for a performance comparison. Native Linux/GUI operation and real-road
  trajectory accuracy were not newly qualified by this review.
