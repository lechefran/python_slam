# Trajectory records

`Map.trajectory` owns lightweight records independently of `Map.frames`:

- One record per processed source frame: ID, source timestamp, original status
  and reason, and an optional accepted 4×4 world-to-camera pose (`T_cw`).
- Failed/uninitialized inputs have no pose. Successful two-view initialization
  can later attach a pose to its earlier reference, preserving that reference's
  original processing outcome.
- Pose values are immutable tuples, copied from mapping frames. Records retain
  no images, keypoints, descriptors, landmark links, or Frame references.
- Accepted bundle-adjustment writeback publishes the corrected trajectory poses.
  Rejected/failed optimization leaves the published trajectory unchanged.
- JSON `poses` and the viewer path read this trajectory. Existing report keys,
  pose order, world-to-camera convention and arbitrary scale are unchanged.
  The viewer continues to break the path across missing source frame IDs.

This is the first MAP-05 slice. All accepted mapping frames are still retained;
feature selection, recovery and optimization policies are unchanged. It does not
reduce memory yet. Keyframe selection, relative reference poses/reanchoring,
non-keyframe retirement and bounded feature storage remain separate follow-ups.
Before implementing retirement, every path that corrects a pose must publish the
same correction to the trajectory, and non-keyframes must follow reference-pose
corrections. Direct mutations of registered frame IDs are unsupported.

## Validation

Tests cover delayed initialization, loss without identity poses, independent
record ownership, atomic invalid-correction rejection, and native BA correction
publication. The large-ID native solver fixture assigns IDs before registration.

Qualification (2026-09-23, macOS arm64): **191 tests pass**, both virtual
environments pass `pip check`, and the wheel builds and imports from an isolated
installation directory. Full `GRMN2734.MP4` replay: 1,800 processed frames,
1,796 accepted poses, no lost frames, and 51,030 live landmarks. All compared
per-frame outcomes/counts and final optimized poses match the saved full-clip
observation-history baseline exactly; frame 941 tracks successfully.
Evidence: ignored `output/trajectory-records-2026-09-23/` contains the command,
source/video hashes, report, log and independent comparison. Tests overlapped
part of replay, so timing is not used for a performance claim. Real-road accuracy,
native Linux execution and native GUI operation were not qualified in this change.
