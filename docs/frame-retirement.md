# Guarded frame retirement

Enable the experimental policy with `--retire-frames`; the default is
`--no-retire-frames`. Use the temporal local-map baseline when evaluating it
independently of the experimental shared-landmark BA policy.

After an accepted BA result, every 20 accepted frames, the scheduler checks at
most eight old candidates and retires at most one. A monotonic accepted-frame
counter schedules BA and retirement; shrinking retained storage cannot shift
the optimization cadence or re-enter the initialization bootstrap.

## Required safeguards

- Never retire either initialization anchor, the latest 64 retained frames,
  frames less than one source second old, the current tracking reference, a
  recovery-archive frame, the latest mapping keyframe, or a camera participating
  in the current BA local/boundary selection.
- Every affected landmark retains its first two and last three observations.
  This protects fixed boundary evidence, descriptor recency, and the views used
  by maturity checks. Retirement never deletes a landmark to meet its budget.
- One surviving mapping keyframe must observe **every** affected landmark with
  positive depth and <=3-pixel reprojection error. This common witness preserves
  graph connections previously passing through the candidate. Empty-observation
  cameras need only a valid same-context replacement reference.
- Reanchor all dependent trajectory records, then demote and link the retired
  camera's own record to the replacement. Preserve current and first-accepted
  world-to-camera poses; references must remain in the same submap/scale context.
- Prepare new frame/landmark observation lists and trajectory updates before
  publishing. Check reciprocal observation integrity before and after removal;
  roll back every staged change if the post-removal check raises.
- Only after success, record observation-removal quality events and retirement
  metadata. First/latest maturity evidence is unchanged, so removal does not
  manufacture a promotion or new observation.

The direct `Map.retire_frame` API takes `protected` camera references: callers
outside the SLAM scheduler must include any frontend/recovery consumers that
still need those frames. The scheduler supplies these protections automatically.
The method removes ownership from map/keyframe/landmark collections; it does not
clear image-feature arrays held by an unrelated external caller.

## Reporting and limits

`frames[].retirement` records bounded attempts, refusal reasons, replacement IDs
and removal counts. `frame_retirement` summarizes retained storage and retired
source IDs. Trajectory records retain every accepted camera, with `retired: true`
where applicable. Mapping-keyframe `count`/`retained_frame_ids` describe live
storage; `selected_count`/`insertions` preserve selection history.

This is conservative redundancy removal, **not a hard memory cap**. Frames with
unique useful observations remain retained. Recovery-archive entries may become
eligible only after archive thinning releases them. No aggressive graph thinning,
landmark deletion, loop closure, or scale correction is introduced. Retired poses
follow their surviving references on subsequent accepted BA corrections.

## Qualification (2026-09-24)

The full **230-test suite passes**. A subsequent bounded-sweep regression also
passes (10 focused retirement tests): it removes an old keyframe from a 72-frame
synthetic map, preserves all 72 trajectory records, and leaves the protected
recovery camera intact. Other tests cover pose-preserving reanchoring, native BA
after removal, essential-observation refusal and post-commit rollback.

Both full `GRMN2734.MP4` replays retain 1,796 poses and 51,030 landmarks with no
tracking losses. Enabling retirement preserves every compared frame outcome,
final pose and BA scheduling frame exactly. Exported relative poses reconstruct
within `3.55e-15` maximum matrix-element difference.

**No dashcam frames were retired.** Of 656 bounded candidate checks, 572 were
blocked by essential first/latest observations and 84 by protected/recent state.
The clip therefore provides behavior-preservation evidence, not a storage or
performance improvement. Do not relax those guards solely to obtain a removal
count on this clip. More general retirement requires separately validated
replacement-baseline and visibility evidence.

Ignored `output/frame-retirement-{baseline,enabled}-2026-09-24/` contains exact
commands, source/video hashes, reports and logs; the enabled directory contains
an independent comparison. Replays overlapped tests, so timing is not compared.
Native Linux/GUI execution and real-road accuracy were not qualified here.
