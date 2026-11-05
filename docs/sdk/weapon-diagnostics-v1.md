# Native weapon diagnostics V1

The optional `WeaponDiagnosticsV1` sidecar carries selected-weapon **referee**
diagnostics. The SDK produces images, measurements, navigation, control,
propulsion, fuzes and physical effects. The Viewer selects, formats and draws
those values. It does not execute a parallel sensor, estimator, controller or
damage simulation. No Agent permissions, actor methods or gateway reports are
added. WorldLayout v9 and fsrec v8 remain unchanged.

## Native producer and Viewer seam

After a successful normal world publication, call
`weaponDiagnosticWorldIdentityV1(world,pid,incarnation,frame)`. This reads the
existing WorldHeader and commits outputs together only for a live compatible
publisher with a completed frame. Create the separate diagnostic publisher
with the same world, actual current PID and incarnation. Mapping names include
all three: old readers never observe reconstructed atomic objects belonging
to a replacement publisher. Active duplicate identities are refused, owner
close/process death invalidates readers, and POSIX names are unlinked at close.

The producer reads `selection(currentFrame,request)`, then independently
resolves the requested weapon ID **and generation** in native state. This
display request cannot fire, guide or alter a weapon. It includes an original
world-frame epoch and correlation, expires after 300 frames, and supports a
zero ID/generation pair to clear selection. Multiple viewers use last-request
wins selection rather than blocking native simulation.

Allocate `WeaponDiagnosticFrameV1` on the **heap**. The maximum RF domain makes
the full frame too large for routine stack use. Stamp exact publisher identity,
worldFrame, simulation time, selected identity and request acknowledgement.
A roster can be published before selection. Each subsystem retains original
sample, processing, delivery and validity epochs, source ID/frame, availability
and native cause. Future delivery, oversized data, invalid identities and
reordered publication are refused without replacing the last complete frame.

Three frame slots use atomic integer payload words and guard counters. Payload
release/acquire ordering avoids a plain-object copy race. Readers make at most
eight attempts and require an exact native frame. Failure leaves their output
unchanged; the GUI separately marks retained data historical. Selection has
its own bounded atomic request channel. Neither channel enters modeled DMS or
actor reports.

## Native units and missing values

| Data | Native meaning and availability |
|---|---|
| Thermal pixels | Float32 diagnostic encoding of **electrons/pixel/exposure**, never irradiance. Expected/measured response remain separate. |
| Detector policy | Exposure s, full-well electrons, ADC bits, detection sigma, read-noise electrons and dark-current electrons/s come from resolved native configuration. |
| Thermal mask | Requires a segmentation-available flag. A receiver exporting features but no mask must not present zero labels as a processed empty mask. Labels are native regions, not target truth IDs. |
| Centroid | Delivered native pixel coordinates, angular variance rad^2 and tracking flags; absent centroid remains unavailable. Row zero is image top. |
| RF cells | Native `Signal::Snapshot` linear power **relative to mean thermal noise/filter**, not watts. Exact native dimensions and peak cells are retained. |
| CFAR | Local noise and threshold matrices have separate flags. The client does not derive a substitute CFAR detector. Native cell/peak/hit counts and wall processing times are separate statistics. |
| RF axes | Native folded range/delay and Doppler metadata. A bistatic delay-equivalent axis is labeled explicitly and cannot become passive target range. |
| Navigation | Hamilton body-to-ECEF quaternion, ECEF position/velocity, measured body gyro rad/s, specific force m/s^2, estimated body biases, 15/21-state native covariance and original IMU/GNSS epochs. |
| GNSS | Fitted, currently aided and ever-sampled are distinct flags. Unknown last GNSS is not a copied NaN or an asserted zero-time measurement. A known old epoch survives an outage. |
| Control | Requested/achieved acceleration and moments have independent flags. Actual fin demand/response, stops, rate and failures remain separate from allocator diagnostics. TVC requires an actual configured nozzle. |
| Guidance/support | Current native phase/decision and original seeker capture/processing/delivery have separate stamps. LOS is the admitted navigation-resolved measured direction; its derivative is ECEF unit-direction/s. Range, range rate, variance and Tgo/Tactive remain separately available. |
| Weapon link | Exact fit direction and channel liveness are separate. Accepted fusion state epoch, original source sample/receive times, radio arrival and expiry remain distinct, including lost/stale inputs. Source track/service IDs are not referee target IDs. Actual native channel counters and radio-resource epochs have their own availability. |
| Propulsion | Native actual engine thrust N, optional fuel flow kg/s and fuel kg, native motor age, pulse command/actual ignition, inlet/combustor and valve data. Phase names do not assert ignition. |
| Effects | Actual fuze arming/burst, warhead topology, source fragment states and victim component impacts. Hit count, penetrated energy, continuous health and removed mass have separate flags. Impact energy cannot imply zero health/death. |
| Native events | Original serials, epochs, IDs, values, units and causes. No hit or success is inferred from proximity or an animation. |

Motor-clock and commanded-pulse-policy availability are explicit. Zero cannot
stand in for an unobserved engine clock. Engine thrust scalars and their sum
are labeled as such; `netThrustN` is shown only with NetAxialForce availability
and means the native total body +X propulsion force, including native ram and
inventory reactions. The client never substitutes gross engine sum for that
net force. Controller-command time is separate from the actual fin-state epoch.

For navigation the full covariance order is dp,dv,dtheta_ECEF,dba_body,
dbg_body,[held accelerometer white error,held gyro white error], with packed
row stride `errorStateCount`. Optional true-own reference is explicitly
Referee comparison and never a navigation correction.

Bounds are 128 roster entries, 128x128 thermal pixels, RF up to 2048 rows and
256 filters with **131072 cells total**, 64 peaks, 32 fins, four engines,
128 displayed actual fragments, 64 damage rows and 128 events. They are
transport bounds, not equipment parameters. Image/spectrum overflow is an
explicit CapacityExceeded status; the transport never silently resamples.
Bounded native fragment/component subsets preserve their full source counts.

## Drawing and companion recording

`WeaponDiagnosticsGui` is an additional ImGui command. The application sets
the actual mirrored world identity/frame/time and calls `update()` on the
render thread. A changed selection must receive its exact ID/generation and
request acknowledgement before previous-weapon data can appear as new data.
Fullscreen scene, Earth imagery, terrain and all existing effects remain on.

The non-collapsible panel draws native detector pixels/optional masks and
centroid, RF spectrum/peaks/optional CFAR data, attitude triad and covariance,
actual actuator response, propulsion, physical fragment positions, component
impacts and event causes. Palette transforms change display colors only.
Expected/measured electrons and a native or clearly labeled sample min/max
display range are explicit. Native row strips keep each draw list inside
16-bit ImGui index limits without dropping cells. A bounded 256-frame graph
uses actual observed timestamps and values, retaining unavailable gaps.

`WeaponDiagnosticsRecorderV1` writes optional `.wdiag` companions with a
separate header and CRC32 per compact record. Only actual counted arrays and
flagged optional arrays are stored; maximum-capacity unused zeros are omitted.
Replay checks sizes, schema, publisher identity, frame/time ordering and CRC.
Modified, truncated or oversized records fail explicitly. CRC is corruption
detection, not authentication.

Fsrec v8 has simulation time but no publisher sequence. The companion reader
therefore supports an **exact** simulation-time join and rewind. An exact time,
entity ID and generation can recur in another recording; they cannot establish
that two files came from the same capture.

The native World invalidates `<recording>.wdiag.binding-v1.json` before
truncating a new recording. Failure to invalidate refuses that recording and
leaves its previous files untouched; simulation continues. At destruction,
the existing flight and diagnostic writers close with checked flush/close
results. Only successful completion of both writers publishes a
`fsim.weapon-diagnostic-capture-binding.v1` manifest. It binds their original
publisher identity, intended byte lengths and SHA-256 digests. A diagnostic
append failure discards its writer and cannot publish a completed binding.
The small closed temporary manifest is published last by an atomic no-replace
hard link; occupied destinations, including dangling symlinks and competing
publishers, are refused. Filesystems without hard-link support leave joined
diagnostics unavailable. Exact owned temporary paths are cleaned up.

The Viewer uses the additive `BoundWeaponDiagnosticsReplayV1` sibling. It
verifies both complete file hashes once before joining and retains the original
raw reader's per-record CRC/schema/epoch validation. It establishes the capture
watch before loading the actual flight file and guards it again afterward;
replacing both files during startup cannot join old in-memory flight with a
new diagnostic capture. Before and after every
open/read/seek/rewind, it checks the regular-file sizes and modification times
and the exact bounded completion-marker bytes. Ordinary overwrite or marker
invalidation closes the bound reader and clears the displayed diagnostic frame,
including while paused at one native time. Reads commit caller output only
after all checks succeed. This is content provenance, not sender authentication
against a malicious edit that deliberately preserves size and modification time.
Streaming hashing keeps one 64 KiB block, with a default diagnostic admission
bound of 1 TiB per file and a 4 KiB manifest; it adds no limit to legacy flight
recording and performs no new simulation sampling or random draw.

Copying or renaming a flight file, companion and its adjacent binding manifest
preserves admission because hashes bind bytes rather than absolute paths. The
default missing, legacy unbound, incomplete, mutated or mismatched companion
leaves flight replay usable and diagnostics unavailable. An explicitly requested
invalid companion reports an error. Raw fsrec v8 and live World IPC formats,
the original raw diagnostic reader, SDK layouts and plugin vtables are unchanged.
Missing times remain unavailable; the Viewer does not interpolate missing
images or reconstruct another weapon's unrecorded data.

SHA-256 uses the unchanged MIT-licensed PicoSHA2 v1.0.1 source pinned at
`161cb3fc4170fa7a3eca9e582cebd27cc4d1fe29`; source URLs and hashes are retained
in the vendored provenance document. It requires only the C++ standard library.
Both `sdk` and `viewer` install components include
`share/flightsim/licenses/PicoSHA2-LICENSE.txt`.

Source fixtures include a real foreign same-time pair with reused entity IDs,
generations and publisher fields, matching/copy/rename cases, incomplete close
and intended-length refusals, malformed/deep/comment-obscured manifests,
occupied/dangling/concurrent completion publication, ordinary post-open
mutation and invalidation, actual World close/re-record capture, and GPU-free
Viewer admission/clearing. The 2026-11-03 serial run built the Viewer and all
three focused Release targets with strict warnings. Release passed 17 contract
cases/629 assertions, 8 GPU-free UI cases/135 assertions, and 3 actual World
cases/539 assertions; Headless passed the same contract and World cases.
Detailed native negative witnesses retain the old raw/time-only foreign join
and the old load-before-binding startup race alongside matching positives and
the guarded refusals. The initial platform fixture failure is retained:
UCRT64's `create_symlink` returned ENOSYS. Creating real NTFS links with the
supported Windows API then verified dangling-marker refusal, exact invalidation
and symlink-input refusal without skipping or weakening those checks.

Both generated transport install rules were executed for their actual `sdk`
and `viewer` components and installed the byte-identical MIT notice. This is
license-rule evidence, not full installed-consumer acceptance. Current logs,
JUnit reports, negative witnesses and bound source/product hashes are under
`build/architecture-reviews/sim-m03-production-20261103/replay-binding/`.
Fullscreen presentation, complete installed consumers and full-program campaign
acceptance remain separate integration gates.

Focused native source cases cover exact units-bearing values, identity and
correlation, capacity/malformed refusal, freshness, publisher replacement,
atomic torn-frame stress, compact records, exact-time seek and tampering.
GPU-free ImGui cases exercise native image/spectrum/covariance drawing and
replay mismatch. Actual native execution, producer integration and complete
fullscreen presentation are separate root-owned verification gates.

## Live thermal channel display selection

`WeaponThermalBandV1` is an optional, additive display sidecar. It advertises
zero, one or two existing image channels for the selected entity and generation.
Its mapping name includes the world, publisher PID and publisher incarnation.
A second owner cannot replace a live mapping. Both advertisement and request
carry a completed world frame; future data and data older than 300 frames are
unavailable. Invalid requests leave the previous accepted request unchanged.
This is local diagnostic transport, not an actor permission or authentication
mechanism. Multiple diagnostic readers share one display selection.

The normal diagnostic panel offers a channel selector only when both channels
are advertised for the exact selected identity. It refreshes the display request
every 100 published frames. A changed world, incarnation or weapon clears the
local selection. The published capture's cause identifies its actual channel;
the requested channel is not an acknowledgement. An older publisher without
this optional sidecar continues to supply its existing V1 diagnostic frame.

The native publisher copies the selected existing image into the unchanged
V1 thermal payload, preserving the original exposure time. No sensor operation,
simulation step, tracking update or random draw is performed by this selection.
World layout, diagnostic V1 frames and fsrec v8 remain unchanged. A retained
terminal image keeps its original epoch and removed status. Once its native
source is removed, only the retained published image remains available.

Companion recording stores the channel actually published at that frame.
Replay disables live channel requests and never reconstructs an unrecorded
channel. GPU-free tests use synthetic transport fixtures to verify channel
requests, incarnation changes and unchanged replay pixels. These tests do not
establish sensor fidelity or fullscreen visual acceptance.

Focused reproduction uses the existing UCRT64 launcher from the repository root:

```powershell
$env:TMP='D:/ENV/tmp'
$env:TEMP='D:/ENV/tmp'
$env:TORCH_EXTENSIONS_DIR='D:/ENV/tmp'
./fsim.cmd --preset ucrt64-release build --target fsim flightsim-viewer fsim_weapon_diagnostics_test fsim_weapon_thermal_band_test fsim_weapon_diagnostics_ui_test -j 8
$env:PATH='D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin;'+$env:PATH
ctest --preset ucrt64-release -R '^(weapon_diagnostics|weapon_thermal_band|weapon_diagnostics_ui)$' --no-tests=error --timeout 120 --output-on-failure
./fsim.cmd --preset ucrt64-headless build --target fsim_weapon_diagnostics_test fsim_weapon_thermal_band_test -j 8
ctest --preset ucrt64-headless -R '^(weapon_diagnostics|weapon_thermal_band)$' --no-tests=error --timeout 120 --output-on-failure
```

The 2026-11-02 focused execution records three passing Release entries and two
passing Headless entries. Original failing tests, current JUnit reports,
independent display review and a source/product hash manifest are under
`build/architecture-reviews/sim-m03-resume-20261102/thermal-band-*`.

A retained diagnostic selection whose source Entry no longer exists marks every
previously Available runtime component stamp Removed, just as a selected terminal
Entry does. Its original generation, source frame, sample epoch, finite-fin values
and limits remain historical values; Available never authorizes dereferencing an
already removed native FDM. The native frameSkip1/4 removal fixture preserves and
checks those values. This does not modify a weapon end state or effects outcome.
