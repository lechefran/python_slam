# Mapping keyframe selection

`Map.keyframes` selects accepted views for future local-map management. It is
separate from `SLAM.keyframes`, the existing bounded recovery archive.
Selection is enabled, deterministic, and currently **does not change estimation**:
all accepted mapping frames remain retained, and tracking, triangulation,
recovery candidates and BA graph membership keep their existing policies.

## Decision policy (version 1)

Both already-validated initialization anchors are selected. For later accepted
frames, selection runs after scheduled BA/culling and requires at least 30 live,
positive-depth, <=3-pixel observations from the landmarks used in that frame's
final pose estimate. New triangulated points cannot supply their own insertion
support. Failed/uninitialized inputs never enter selection.

Compare against the last selected mapping keyframe. A minimum interval of 0.15
source seconds suppresses dense insertion; validated recovery can bypass this
interval, but cannot bypass the support gate. After that gate, select for any of:

- Median rotation-compensated parallax >=1 degree, with >=20 shared observations.
- Camera rotation >=10 degrees (view change, not evidence of translation).
- Shared support below 70% of the reference's verified support at insertion.
- Current verified support below 70% of the same insertion support count.
- One source second since the last selected keyframe (temporal fallback).
- Geometrically accepted recovery into the existing map.

These are explicit initial heuristics, not calibrated confidence or optimized
thresholds. The temporal fallback can select a stationary view; it bounds spacing
between supported views, not the total number of keyframes or memory use. Very
weak accepted views can exceed the interval without qualifying for insertion.

## Geometry and evidence

Parallax compares measured normalized camera rays after rotating each into world
coordinates with `R_cw.T`. Pure camera rotation therefore does not produce
translational parallax. Translation scale never enters the angle. Ray measurements
are rechecked against live poses/landmarks using rectified pixel reprojection and
positive depth. Missing shared support produces a null parallax, never zero.

Overlap uses *reference support at insertion* as the denominator. Culled or lost
reference observations therefore reduce retained support; it is not an
occlusion-aware visibility probability or a moving-object classifier. The current
count comes only from pre-triangulation pose-estimator landmarks, revalidated
after BA. With experimental maturity enabled, this inherits the estimator's
active/bootstrap policy; selection does not promote candidates.

`FrameResult.keyframe` records selected/rejected reasons, reference ID, source
interval, verified/current/shared support, overlap, support ratio, median parallax
and rotation. Earlier initialization-reference decisions are in the aggregate
`mapping_keyframes.insertions` report (the earlier processing result is preserved).
The aggregate includes policy thresholds, schema version, selected count and
`affects_estimation: false`. Insertion evidence describes the decision at that
time; selected Frame references remain live through subsequent BA/culling.

## Next boundaries

Selection establishes a distinct subset; it does not yet provide covisibility,
relative non-keyframe anchoring, frame retirement, or bounded keyframe storage.
Those changes need separate geometry and recovery checks before selected frames
can replace the current estimator inputs. An identical trajectory in this step
checks behavior preservation, not accuracy improvement.

## Qualification (2026-09-23)

- **204 tests pass**, including analytic translation/scale invariance, pure
  rotation, overlap/support decline, temporal gates, recovery, observation
  removal/residual/depth rejection, CLI reporting and failed-frame exclusion.
- Full `GRMN2734.MP4`: **360 selected mapping keyframes**, 1,800 processed frames,
  1,796 accepted poses, no tracking losses and 51,030 live landmarks. Every
  compared frame outcome/count and final optimized pose matches the previous
  trajectory-records baseline exactly; frame 941 tracks successfully.
- Two initialization anchors plus 358 subsequent selections. All subsequent
  selections include overlap decline; 192 also include parallax and 15 include
  support decline. All 1,436 rejected accepted views hit the minimum interval.
  This clip therefore selects at the earliest permitted intervals: thresholds
  are not demonstrated optimal, and held-out diversity/resource qualification
  remains necessary before using this subset to restrict estimator inputs.
- Selection IDs are unique, ordered, and correspond to accepted poses. Every
  non-initialization selection has >=30 verified pose-estimator observations.
- Wheel build and isolated installation/import pass; dependency consistency
  passes. Ignored `output/keyframe-selection-2026-09-23/` contains source/video
  fingerprints, the exact command, report, log and independent comparison.
- Tests overlapped replay, so elapsed time is not performance evidence. Native
  Linux execution, GUI operation and real-road trajectory accuracy were not
  qualified in this change.
