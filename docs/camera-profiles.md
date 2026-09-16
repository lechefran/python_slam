# Reproducible camera profiles

Offline calibration now exports **schema 2 camera profiles**. They retain the
pinhole geometry expected by SLAM and add structured recording settings,
portable quality-report references and content verification. Existing unversioned
and schema 1 pinhole files still load, labelled `legacy_unverified` in provenance.
No profile for the actual dashcam has been measured by this work.

## Declare the camera and recording mode

Copy [camera-settings.example.json](camera-settings.example.json), then fill in
values you actually know. The example is a template, not the measured settings
of `GRMN2734.MP4`. `null` and `unknown` explicitly mean unestablished information.

| Field | Meaning |
| --- | --- |
| `schema_version` | Settings format version; currently integer `1` |
| `camera_id` | Your stable identifier for the physical camera |
| `lens_id` | Lens or camera/lens assembly identifier |
| `mode_id` | Stable recording-mode identifier; distinct modes need distinct identifiers |
| `fps` | Nominal recording rate as positive integer numerator/denominator, e.g. 30000/1001; fractions are reduced on import |
| `crop` | Declared fixed capture crop: integer `x,y,width,height` in pixels of the uncropped recording raster, before resizing; null means unknown. This is metadata, not a request to crop images |
| `stabilization` | `off`, `optical`, `digital`, or `unknown` |
| `digital_zoom` | Positive magnification factor; use 1 for known no digital zoom, null if unknown |
| `focus_mode` | `fixed`, `manual`, `auto`, or `unknown` |
| `focus_setting` | Setting/position identifier meaningful to that camera, or null |

Unknown keys, duplicate JSON keys, non-finite numbers and malformed settings are
rejected. Decoded width/height are obtained from the calibration images, not
inferred from `mode_id` or a description. Camera and recording-mode free-text
labels remain available, but runtime matching uses the structured declarations.
Changing stabilization, focus or crop can change the camera model; recording
metadata does not make a time-varying camera compatible with fixed intrinsics.

## Fit and save a profile bundle

```sh
.venv-portability/bin/python -m calibration \
  --images output/calibration-captures/fit \
  --validation-images output/calibration-captures/validation \
  --board checkerboard --columns 9 --rows 6 --square-size 0.025 \
  --camera 'camera-and-lens-name' --recording-mode 'recording-mode-description' \
  --camera-settings output/camera-settings.json --seed 0 --threads 1 \
  --output output/my-camera/camera.json \
  --report output/my-camera/quality.json \
  --report-html output/my-camera/quality.html
```

The settings file is optional for compatibility with earlier calibration commands.
Omitting it produces a profile with explicitly unknown settings. Default OpenCV
seed is 0 and thread count is 1. The quality report records these values, solver
iteration/epsilon settings, normalized capture settings and the settings-file
SHA-256. These controls support repeatable experiments; native solver/platform
changes can still alter floating-point results.

The profile records:

- The fitted `K`, distortion, decoded source size and `raw_distorted_source`
  pixel domain. Processing continues to resize and rectify through the existing
  runtime geometry path.
- Camera/mode labels, structured settings, board geometry/units and UTC fit time.
- Python, NumPy and OpenCV versions, plus SHA-256 of the calibration, reporting
  and profile implementation modules used to create the artifact.
- A **relative** quality-report path and SHA-256 of its exact bytes. The quality
  report contains the source-image hashes and detected correspondences.
- `profile_id`, a `sha256:` identifier of the profile contents excluding its own
  ID and the report's filesystem location. Canonicalization uses Python sorted
  compact ASCII JSON with non-finite values forbidden; it is not an implementation
  of RFC 8785. Numeric representation changes such as `1` to `1.0` can change the ID.

Keep `camera.json` and `quality.json` together. Moving/copying the bundle preserves
its ID. Formatting or reordering keys in the profile also preserves its ID,
while its separate raw-file hash changes. The report hash intentionally covers
exact bytes: reformatting the report is detected as a provenance change. If the
report is relocated separately, edit only its relative reference in the profile;
the content ID remains valid if the referenced bytes are identical.

A new calibration run is a new artifact: creation time, report bytes and possibly
numerical results change its ID. The ID identifies an experiment artifact; it
is not a claim that every refit produces identical bytes. Hashes detect accidental
changes and wrong pairings, not malicious forgery or camera authenticity.

## Inspect and verify

```sh
.venv-portability/bin/python -m camera_profile output/my-camera/camera.json
# Installed equivalent:
.venv-portability/bin/python-slam-camera-profile output/my-camera/camera.json
```

Verification checks supported schema/model, finite geometry, the profile ID, the
linked report hash and agreement between report/profile geometry, settings,
quality status, board, labels and creation time. Missing or incompatible reports
fail clearly. Source captures are not reopened during profile verification;
retain them if you need to repeat the fit. A profile's recorded implementation
hash describes the fitting code; it does not require the current runtime to be
the same revision.

Successful verification is reported as `hashes_verified`. Calibration remains
**`measured_unverified`**, and the quality report may still say `needs_review`.
Neither successful hashing nor compatible capture declarations certify accuracy.
Legacy files are not silently upgraded: rerun calibration with the original
captures to produce a new profile with provenance.

## Use with SLAM

```sh
.venv-portability/bin/python slam.py sample_videos/GRMN2734.MP4 \
  --calibration output/my-camera/camera.json \
  --camera-settings output/this-video-settings.json \
  --headless --report output/profile-run.json
```

Use the actual video's declared settings; do not simply copy a different camera's
profile declarations to make a comparison pass. Runtime checks:

1. Verify the profile/report bundle before tracking.
2. Require decoded source dimensions to equal calibration dimensions.
3. If profile FPS is known, compare it with OpenCV's **nominal decoder FPS** using
   relative tolerance 1e-5 or absolute tolerance 0.001 FPS. This accepts decoder
   rounding of 30000/1001 to 29.97, but rejects 30 versus 29.97. Missing decoder FPS
   is recorded as unavailable; this does not validate variable-rate timestamps.
4. Compare any supplied known camera/mode/lens/crop/stabilization/zoom/focus/FPS
   declarations with known profile values. A known disagreement fails before
   a pose is accepted. Unknown values do not become matches.

The decoder cannot verify physical camera identity, lens, focus or stabilization.
Matching supplied settings are labelled `matched_declaration`, separately from
`matched_decoded` dimensions and `matched_decoder_nominal` FPS. Missing declarations
or unknown profile fields yield `incomplete`; processing is allowed with a notice
and a detailed compatibility record. If all fields have matching available
evidence, status is `matched_available_evidence`, which still includes declarations.
With a legacy file the status remains `legacy_unverified`.

Trajectory and landmark-quality reports carry the profile ID, file/report hashes,
quality status, normalized declarations and their file hash, per-field
compatibility, original and processed intrinsics, resize factors, interpolation
and rectification details. Reports cannot overwrite profile provenance or settings
inputs. The calibration-file hash refers to the exact snapshot loaded at startup,
not a later reread of a possibly changed file.

The tracking benchmark helper also accepts `--camera-settings`. Its existing
baseline comparator remains strict about full camera metadata and file hashes;
formatting/provenance changes can therefore make a baseline ineligible even when
profile IDs or geometry match. Profile identity is recorded separately for audit.

## Qualification

Tests cover profile relocation and reformatting, content/report tampering, missing
provenance, legacy loading, malformed settings and JSON, unknown information,
fractional frame rates, source-size and recording-mode mismatches, native SLAM
replay equivalence and protecting report provenance from output overwrites.
Real camera quality and Linux execution remain unverified by these macOS tests.

### Local results (2026-09-16)

The full `.venv-portability` regression suite passes **175 tests**, including all
four installed entry points. Both existing environments pass `pip check`. The
installed wheel matches the final implementation modules.

- A profile from the existing 16/4 rendered checkerboard split retains fitting
  RMS 0.069 px and validation RMS 0.085 px. Its quality remains `needs_review`.
  The installed verifier successfully checked the bundle from `/private/tmp`.
- A 25-frame native synthetic SLAM replay produces identical final poses and
  landmark counts with a version 2 profile and equivalent legacy intrinsics.
  A deliberately mismatched stabilization declaration fails with no accepted pose.
- Before/after replay of source frames 0–99 of `GRMN2734.MP4`, using the same
  **approximate legacy focal 525** model in both runs, preserves every per-frame
  outcome/count and final pose exactly: 96 accepted poses and 2,425 landmarks.
  This is a bounded compatibility check, not a full-route or accuracy qualification.
- Applying the 960×720 synthetic profile to the 1920×1080 dashcam input is rejected
  before tracking. No synthetic profile is treated as measured dashcam calibration.

Artifacts are ignored under `output/camera-profiles-2026-09-16/`; the comparison
record is `qualification.json`. No tracking, landmark selection or optimizer
policy changed in this work. Runtime camera metadata and validation changed.
