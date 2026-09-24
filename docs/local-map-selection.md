# Local maps from shared landmarks

Run the experimental backend policy with:

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --headless --max-frames 1800 --seed 0 --threads 1 \
  --local-map-policy shared --report output/shared-map.json
```

The default remains `--local-map-policy temporal`, preserving the existing
recent-frame BA baseline. The new policy changes the actual BA graph; it is not
just selection metadata. It does not change projection search, triangulation,
recovery retrieval, mapping-keyframe insertion, or frame-retirement policies.

## Selection and constraints

- Seed from the latest accepted frame's live landmarks with at least two
  observations. With maturity enabled, only active landmarks supply scores.
- Count shared landmarks for each retained mapping keyframe. Require six shared
  observations; rank by descending count, then newer stable frame ID.
- Select the current frame plus at most nine neighbors with the default
  `local_window=10`. Non-keyframes do not become neighbors merely by recency.
  `local_window=None` explicitly removes the local-camera cap.
- Gather candidate points from those views. Prefer points seen in the current
  camera, then support within the selected neighborhood, recency and stable ID.
  The existing 600-point budget remains in force.
- Include each selected point's local observations plus up to two earliest
  outside observations as fixed boundary constraints. Boundary cameras may be
  non-keyframes; removing that evidence would change the map's constraints.
- Preserve the first two initialization cameras as fixed if present. Require
  >=6 edges per free graph camera and a fixed nonzero baseline in each connected
  component (unless landmarks themselves are fixed). Skip unsupported graphs;
  never silently switch policy or fabricate an accepted update.
- Keep existing native-result validation and atomic pose/landmark/trajectory
  publication. Only geometrically accepted corrections reach the map.

Covisibility counts are recomputed from reciprocal live observations on each BA
attempt; deletion/culling cannot leave a stale cached edge. Shared observations
are existing accepted map evidence, not independently calibrated static-scene
confidence. The six-landmark neighbor threshold is an initial graph-support
heuristic, not a new pose-acceptance gate.

`frames[].ba.local_map` records policy, seed, neighbor IDs/shared counts, selected
local IDs, fixed boundary IDs, graph camera/point counts and point budget. Graph
counts describe actual participating vertices; some selected views may have no
remaining edges after point-budget selection. Skipped attempts retain available
selection evidence and their existing rejection reason.

This bounds the selected movable neighborhood and point budget, not total stored
map memory. Fixed boundary-camera count depends on the selected observations.
[Guarded retirement](frame-retirement.md) is a separate opt-in policy. Covisibility-based
tracking search and long-sequence resource qualification remain pending.

## Validation

Tests cover deterministic shared-count ranking, disappearing observations,
disconnected recent keyframes, maturity/deletion filtering, camera/point budgets,
fixed boundary preservation, native convergence, and failed-constraint rollback.

## Dashcam qualification (2026-09-24)

**221 tests pass**; both environments pass dependency checks. Paired full
`GRMN2734.MP4` replays used identical source frames, timestamps, approximate camera,
seed and tracking configuration. Runs overlapped, so timing is not compared.

| Result | Temporal baseline | Shared-landmark policy |
| --- | ---: | ---: |
| Processed frames | 1,800 | 1,800 |
| Accepted poses | 1,796 | 1,780 |
| Lost frames | 0 | 16 |
| Live landmarks | 51,030 | 52,435 |
| Accepted BA attempts | 349 | 337 |
| Rejected BA attempts | 10 | 17 |
| Constraint skips | 0 | 2 |
| Median / maximum graph cameras | 81 / 153 | 96 / 164 |
| Maximum selected local cameras / points | 10 / 600 | 10 / 600 |

The temporal run exactly reproduces the prior trajectory-corrections baseline's
frame outcomes, keyframe decisions and final poses. Frame 941 tracks in both.
The shared policy newly loses frames 947, 952, 1029, 1086, 1090, 1146, 1285, 1334,
1677, 1678, 1693, 1695, 1699, 1701, 1702 and 1709, with no newly accepted frames.
Fifteen losses cite insufficient inlier image coverage; one cites an invalid
refined PnP pose. BA safely skips frames 1774 and 1779 because the selected graph
lacks a fixed nonzero baseline.

**Keep shared selection experimental.** It is neither a demonstrated coverage
nor efficiency improvement: boundary cameras increase despite the local-view
cap. Before promotion, investigate the selected support/boundary geometry on
independent scenes, preserve baseline frame identities under unchanged tracking
safeguards, and qualify held-out calibrated accuracy. More landmarks do not
establish better reconstruction.

Ignored `output/local-map-{temporal,shared}-2026-09-24/` contains commands,
source/video fingerprints, reports and logs; the shared directory also contains
an independent `comparison.json`. Native Linux, GUI behavior and real-road
trajectory accuracy were not qualified in this change.
