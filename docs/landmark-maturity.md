# Landmark maturity

## Status and use

`--landmark-maturity` opts into a candidate/active lifecycle and active-only
pose estimation, older-keyframe recovery and bundle adjustment. It is currently
**off by default** because active-only tracking regresses on the supplied dashcam.
Use
`--no-landmark-maturity` for the established tracking policy.

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --landmark-maturity --headless --report output/maturity.json \
  --diagnostics-dir output/maturity-frames --diagnostics-start 0 --diagnostics-end 60
```

This implements the maturity portion of MAP-04. It does not establish that a
landmark is stationary, estimate statistical covariance, or provide a full
landmark descriptor/visibility/uncertainty model. Robust pose weighting and
custom exclusion masks remain separate options.

## Lifecycle

| State | Meaning | Eligible to estimate poses / enter BA? |
| --- | --- | --- |
| Candidate | Triangulated but not independently supported by enough accepted views | No, except the explicit initial bootstrap |
| Active | Passed the maturity support checks | Yes |
| Outlier | Removed because fewer than two live observations survived geometric culling | No; all links removed |
| Retired | Removed for staleness, expired candidacy or explicit deletion | No; all links removed |

A new triangulated point starts as a **candidate**, with its creation source
frame ID. Promotion requires three distinct accepted frame IDs, including a
frame later than creation. Repeating the same observation cannot increase its
support. The selected support is the earliest and two latest live observations;
all three must have positive finite depth and at most **3 pixels** reprojection
error. Their maximum pairwise world-ray angle must be at least **1 degree**.

These conservative thresholds are operating policy, not calibrated confidence.
The three selected views bound each geometric check; they do not certify every
historical observation. The earliest-view choice can reject a point that some
other triplet would support. Age is in source frames, not seconds.

Camera centres come from `C_w = -R_cw.T @ t_cw`. The comparison uses normalized
`X_w - C_w` rays, so translating the world origin or changing monocular scale
does not change the parallax test. Large-coordinate normalization avoids
squaring world magnitudes. Camera poses and point coordinates are never changed
by a maturity check.

## Bootstrap and independent candidate validation

The initial two-view map has no three-view landmarks yet. Only its initial
triangulated points receive a bootstrap flag. While the map contains exactly
those two accepted cameras, these points may support normal PnP under all the
existing depth, residual, image-coverage and conditioning gates. After the first
successful map-based camera is committed, that exception ends. It is never
available to older-keyframe recovery and is not reopened after later loss.

After each accepted pose, the tracker validates candidate correspondences
against that **fixed committed pose**:

1. Propose descriptor matches to candidates in the preceding reference.
2. Search remaining candidate projections with the normal 30-source-frame
   recency limit, 5-pixel radius and descriptor gates.
3. Reserve every already-committed feature/landmark association; maintain one
   point per feature and one feature per point.
4. Attach only positive-depth observations with <=3-pixel residuals, then
   reassess maturity. Newly promoted points become available for later poses.

Candidate checks do not change the camera estimate or its PnP inlier count.
Failed poses do not attach observations, promote candidates, or advance culling.
A successfully recovered pose may validate candidates in the same way, but
recovery itself uses active points only and creates no new landmarks.

## Culling and bundle adjustment

Active points can become candidates again if supporting observations disappear
or the selected views fail the maturity geometry checks. Scheduled culling
rechecks maturity after BA, using the latest live poses/XYZ; classification is
not based on detached coordinate caches. BA includes only active points when
the mode is enabled, preserving the existing anchored graph checks.

Candidates more than **30 source frames past creation** retire at scheduled
culling, even if repeatedly observed without enough geometric support. A point
demoted long after birth can therefore retire immediately at that culling pass.
The existing weak-stale-point rule remains in force. Supported active points
are not removed solely because they are old or distant. Retirement unlinks
both frame and landmark observations, is idempotent, and prevents reuse by
recovery archives. Culling remains scheduled on accepted cameras; this does
not add background map mutation during tracking loss.

## Diagnostics and viewer

- With maturity enabled, per-frame and final reports expose `landmark_quality`:
  live `candidate`/`active` counts, cumulative `outlier`/`retired` counts, and
  promotion/demotion event counts. Re-promotions count as new events, not new
  unique landmarks. The live counts sum to `landmarks`.
- `stages.landmark_selection` identifies bootstrap versus active-only PnP
  inputs. `candidate_validation` separates its proposals, accepted and rejected
  observations from pose inliers. Its processing time is recorded separately.
- Each live `Point` retains its creation frame and current maturity evidence:
  reason, selected frame IDs, parallax and maximum checked residual when valid.
  This is a current bounded assessment, not an unbounded residual history or
  successful-reobservation probability.
- The ORB overlay shows **green active landmarks** and **magenta candidates**
  in this mode. Cyan remains an unassociated ORB feature. O toggles all markers.
- With maturity disabled, maturity classification is not evaluated and report
  quality is null. The viewer retains the earlier mapped/unmapped distinction.

## Validation

Synthetic checks cover promotion, duplicate observations, insufficient parallax,
bad depth/residuals, creation timing, demotion, atomic/idempotent retirement,
world-origin/scale invariance, expiry, bootstrap scope, candidate-only recovery
and BA rejection, native active-point recovery and BA, and unchanged map state
after failed poses. An independent synthetic process test proves both native
PnP passes use 50 active points while 10 candidates are validated/promoted only
after that camera is accepted; its inlier count remains 50.

The 25-frame synthetic smoke initializes a two-view map, promotes 77 points on
the first map-based pose and finishes with 23 accepted cameras. This is a
bootstrap/lifecycle check, not evidence of real-road accuracy.

The full regression suite passes **123 tests** on macOS arm64. A native
`macosx` viewer check on 12 dashcam frames verified the candidate/active pixel
locations, independent colors and pause-toggle behavior, unchanged input pixels
and poses, and reciprocal map integrity. The rebuilt installed command was
checked outside the checkout; its runtime module bytes match the source.
Dependency consistency passes. Linux execution and held-out accuracy remain
unqualified.

### Full dashcam comparison

Both runs decode all 1,800 frames of
`/Users/francissy/Documents/python_slam/sample_videos/GRMN2734.MP4` from zero,
using source focal 525 (approximate calibration), width 1024, 2,000 ORB features,
seed 0, one OpenCV thread, no exclusion mask, recovery/centering/spatial mapping
enabled and robust weighting disabled.

| Measure | Default policy | Maturity enabled |
| --- | ---: | ---: |
| Accepted poses | 1,796 | 3 |
| Lost frames after initialization | 0 | 1,793 |
| Final live landmarks | 51,030 | 135 |
| Final classified candidates / active | Not assessed | 83 / 52 |

The default run preserves **all per-frame outcomes/counts and final optimized
poses exactly** against `output/exclusion-masks-2026-09-16/default/report.json`.

The experimental run accepts source frames **0, 5 and 6**. Initialization creates
77 candidates at frame 5. The frame-6 bootstrap pose promotes 52 points, with
83 candidates remaining after new triangulation. Frame 7 has 36 active
descriptor correspondences, but its final pose fails the unchanged image-coverage
gate. Subsequent tracking/recovery never recovers. Candidate validation requires
an accepted pose, so those new candidates cannot gain later support during loss.
With no further accepted cameras, scheduled culling also does not advance.

This is a **failed coverage qualification**, not an accuracy improvement. The
policy therefore stays opt-in. Next work must address the transition from
bootstrap to spatially adequate established support, with independent validation
and held-out data. It should not promote points solely to satisfy this clip's
pose count or weaken geometric gates. Maturity also does not prove that the
surviving points are static scenery.

Ignored artifacts are `output/landmark-maturity-default-2026-09-16/`,
`output/landmark-maturity-enabled-2026-09-16/` and
`output/landmark-maturity-viewer-2026-09-16/`.
`output/landmark-maturity-qualification-2026-09-16.json` records report hashes,
baseline comparison and validation scope. The video SHA-256 is
`9026415d4c494c5f831d7c5578259668906516e8d27a581723d1fae01ea82e3c`.
Single-run elapsed times were 463.06 seconds (default) and 96.21 seconds
(experimental) under overlapping workloads. Early loss eliminates most mapping
work, so the latter is not a performance improvement.
