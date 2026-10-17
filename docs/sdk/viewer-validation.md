# Native viewer validation

Date: 2026-10-12. Checkout: `main`, starting commit `c2641e9cf51d2f84befa96d6a1d0b592029898d4`,
with the viewer corrections described below. This record covers the native Vulkan/VSG viewer and offscreen
Vision on this machine. The earlier [engagement delivery record](engagement-validation.md) remains headless
SDK and browser-client evidence; the measurements in [the viewer guide](viewer.md#the-frame-rate) retain
their original date and build.

## Environment and build

- Windows, NVIDIA GeForce RTX 5070 Ti, driver 596.36, 16,303 MiB reported device memory.
- MSYS2 UCRT64 GCC 16.2, CMake 4.4.3, Ninja 1.13.2 and CPython 3.13.12 under `D:/ENV`.
- `ucrt64-release`, `FSIM_WARNINGS_AS_ERRORS=ON`, bundled NVIDIA Flow and the vendored VSG stack.
- `TMP`, `TEMP` and `TORCH_EXTENSIONS_DIR` point to `D:/ENV/tmp`; Hangar's Matplotlib cache is also under
  `D:/ENV/tmp`.
- Every native viewer acceptance run uses `--fullscreen`; captures are checked at 3840 x 2160.
  Vision cameras and RCS use offscreen GPU targets, and scene-graph tests have no viewer window.
- Offline maps use `assets/maps` and `assets/config/viewer.json`. Missing cached subtiles can produce map
  warnings; this run does not establish worldwide offline coverage.
- Khronos Vulkan validation layers are unavailable on this machine. GPU execution does not establish a
  clean validation-layer run or acceptance on another driver/GPU.

The dependency and renderer builds completed with these entry points:

```powershell
cmake -S deps --preset deps-ucrt64
cmake --build build/deps-ucrt64 --parallel 12
.\fsim.cmd --preset ucrt64-release configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe -DFSIM_WARNINGS_AS_ERRORS=ON
.\fsim.cmd --preset ucrt64-release build --parallel 12
```

Raw build logs: `D:/ENV/tmp/viewer-deps-configure.log`, `viewer-deps-build.log`,
`viewer-render-configure.log` and `viewer-render-build.log`.

## Reproduced defects and corrections

| Defect | Reproduction and resulting behavior |
| --- | --- |
| Boundary documentation omitted optional Vision exports | The initial renderer selection passed 53 of 54 tests; `sdk_boundary` alone failed. The committed page had been rendered from a headless build. Rendering with both DLLs restores 68 Vision entries: 28 C++, 25 C and 15 Python, for 1,785 classified entries with zero errors. Both render and headless boundary gates then passed. |
| Loading stores after the first Vision render could crash | The public GPU regression rendered an empty F-16C, loaded four Mk 82 stores, then rendered again. Before the correction, the second render exited with access violation `-1073741819` in VSG descriptor binding. New scene resources now request deferred offscreen compilation before drawing. |
| New same-model entities retained another entity's pylons | Fullscreen live captures showed pylons on a fresh empty F-16C after slot reuse, and again after a same-name World restart. With dynamic compilation repaired first, the depth regression still found three unexpected silhouette pixels. Carriage history is now reset when the entity identity changes, including cached model reuse; release on the original aircraft retains its pylons. The corrected Vision lifecycle regression passed. |
| Replay reconstruction could apply stores to the wrong entity | Replay now applies each frame's vehicle-table changes before that frame's store changes. Entity identity drives carriage/section restoration, and replay looping clears live/visible metadata and trails before reconstructing the beginning. |
| Screenshots read the previous presented frame | Accelerated replay at factor 50 showed the old loaded F-16C at the clean replacement's time. The recording contains the new entity and explicit empty inventory. A fullscreen regression warmed up red, presented blue, and captured red before the correction. Readback now selects the current presented swapchain image; blue and green captures both match their latest frame at 3840 x 2160. |
| Screenshot failure returned success | A fullscreen CLI run with a nonexistent output parent exited 0 without producing a PNG. The CLI now checks readback/save results and exits 1 for failed single or sequence captures, after its normal shutdown. A valid output still exits 0 and decodes at 3840 x 2160. Plume and performance consumers remove prior images and require new output before measuring it. |
| Mission monitor mislabeled partial capabilities | The footer previously described every `(MS-n)` marker as not modeled, despite partial native strike support. It now states that the marker means partial or unimplemented SDK support. Published capability strings and support status are unchanged. |
| Strike camera subjects could exceed prepared Viewer capacity | A native scene requires carrier, weapon and target slots, but a smaller `--capacity` could leave the weapon outside the prepared batch. Startup now rejects the capacity before window creation; the live check also validates the prepared slot bound. The harness checks exit 2 and the explicit capacity error without a capture-ready marker. |
| Rejected Showcase options retained old success artifacts | A seeded recording and sidecar remained after an unsupported CLI option. The generator now invalidates both outputs before validating options, and the black-box regression checks that rejected input cannot leave them behind. |
| Private strike capture header had the wrong module owner | The full delivery suite rejected `StrikeCapture.h` as a headless app header. The shared-directory ownership policy now explicitly assigns it to Viewer, like `DemoAutopilot.h`. Existing dependency edges and checks remain intact. |

These corrections are private renderer lifecycle changes. Public C++/C/Python interfaces, ABI layouts, IPC
layouts, permission checks, SDK simulation order and aircraft/store model data are unchanged. The regression
uses the public World and Vision interfaces, checking loaded geometry, original-aircraft pylon retention,
removal and a fresh same-model entity's silhouette against its clean baseline.

Raw evidence: `D:/ENV/tmp/viewer-render-tests.log`, `viewer-lifecycle-depth-red-test.log`,
`viewer-lifecycle-red-debug.log`, `viewer-lifecycle-pylon-red-test.log` and
`viewer-lifecycle-green-test.log`, `viewer-screenshot-red-test.log` and `viewer-screenshot-green-test.log`.
The initial fullscreen live sequence and SDK inventory events are in
`D:/ENV/tmp/viewer-acceptance-fullscreen-before/`.

## Viewer correction baseline

These checks establish the viewer-correction baseline before replacement of the scripted missile showcase.
Results are recorded only after execution and evidence review. A prepared command, recording or test list
does not count as a pass.

| Check | Result |
| --- | --- |
| Final warnings-as-errors build | Passed, including the new screenshot regression and restaged native/Python runtimes. Final log: `D:/ENV/tmp/viewer-complete-build.log`. |
| Full `ucrt64-release` CTest suite | Passed: 650/650, zero failures and zero CTest skips, 992.86 seconds. Includes actual Vision, Vision C ABI, RCS, Flow calibration and the fullscreen screenshot regression, plus SDK, architecture, Hangar, launcher and installed-consumer checks. Log: `D:/ENV/tmp/viewer-full-tests.log`. |
| Fullscreen live SDK lifecycle | Passed, exit 0; all five 3840 x 2160 captures decoded and visually inspected. Loaded Mk 82 stores, consented release of two stores into three live entities, reset/reload, clean replacement entity ID 4 in the reused slot, and same-name World restart with clean entity ID 1 were observed. Empty inventories agree with the absence of old pylons on both fresh aircraft. Final evidence: `D:/ENV/tmp/viewer-acceptance-fullscreen-final/`, including `live-events.json`. |
| Fullscreen demo and replay scene matrix | All nine processes exited 0 and all 23 PNGs decoded at 3840 x 2160; 13 representative captures were visually inspected across all nine cases. Scenes cover the maintained launcher demo, native F-16C release and clean replacement, native F-22A bay/release and clean replacement, fleet, reheat, ground strike and scripted hit. Final evidence: `D:/ENV/tmp/viewer-acceptance-scenes-final/matrix.json`, images and model logs beside it. |
| Four-scene 4K fullscreen performance gate | **Failed**, exit 1: reheat passed; fleet, strike and hit failed the unchanged gate. Measurements are below. Evidence: `D:/ENV/tmp/viewer-performance.log` and `D:/ENV/tmp/viewer-acceptance-perf/current.json`. |
| Replay loop and skipped-frame reconstruction | Passed in native fullscreen runtime. Factor-50 captures at simulation time 9 show clean replacement geometry for both F-16C and F-22A. A separate public-SDK recording creates a second aircraft at time 5; live replay statistics show one aircraft initially, two before the end, and one again after looping at wall time 10.1 and 11.1. Evidence: `D:/ENV/tmp/viewer-acceptance-fullscreen-final/replay-loop.json`, its log and PNG. Manual timeline seeking remains unverified. |

The native live lifecycle uses a published SDK World: loaded F-16C, consented release, reset/reload,
remove/recreate with empty inventory, and a new World with the same name. The native F-16C and F-22A recordings
in `D:/ENV/tmp/viewer-acceptance-recordings/` are generated by public SDK Mission calls and normal simulation
steps. They exercise release/bay state, reset and clean entity replacement; they do not replace weapon flight
with viewer-side physics.

The fleet, reheat and ground-strike fixtures exercise rendering of SDK Worlds. At this baseline, `hit` used a
scripted showcase recording with scheduled missile motion and detonation; its image and frame timing are
rendering evidence, not native guidance or fuze acceptance.

### Performance before native showcase replacement

Runs use 3840 x 2160 fullscreen, offline maps and active NVIDIA Flow. No builds, CTest runs or screenshot
readbacks run alongside them. The target is unchanged: after five seconds of warmup, each one-second window
must reach at least 158.4 fps, and no frame may exceed 12.5 ms.

| Scene | Measured windows | Mean fps | Lowest window fps | Worst frame ms | Frames over 12.5 ms | Gate |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Fleet | 35 | 160.0 | 159.9 | 17.81 | 2 | Failed: slow frames |
| Ground strike | 21 | 159.1 | 150.5 | 13.14 | 2 | Failed: rate and slow frames |
| Scripted hit | 29 | 159.2 | 145.2 | 18.89 | 4 | Failed: rate and slow frames |
| Reheat | 21 | 160.0 | 159.9 | 7.09 | 0 | Passed |

These are baseline measurements, not a before/after optimization comparison. The dated rates in the viewer
guide remain historical evidence. This strict gate was not satisfied for three scenes; no threshold was
relaxed and no performance or material redesign is included in this correction.

The four raw logs confirm Flow initialization and contain no model parse/build failures. They do contain
missing-subtile warnings (1,828 / 4,359 / 7,322 / 5,180 for fleet / strike / hit / reheat) and one
`missing DescriptorBinding for vertexInputRates` warning per scene. The slowest frames spend most of their
time in `recordAndSubmit`; these logs do not establish the cause of those stalls or a connection to the warnings.

### Material and shadow scope

Aircraft GLBs, material generation and the lighting shaders retain their existing bytes. The inspected F-16C
and F-22A GLBs each contain one base-color image and no normal texture or metallic/roughness texture; their
paint uses fixed metallic and roughness factors. This run does not upgrade their material detail or establish
finished visual quality.

`world/Sky.cpp` creates sun, sky and ground directional lights without VSG shadow settings. Aircraft
self-shadowing and aircraft-to-ground shadow casting are not enabled through that path. `world/Clouds.cpp`
has its separate cloud shadow maps and ground composite. Surface shading and cloud shadows must not be
reported as accepted aircraft cast shadows.

## Native guidance replacement

The maintained showcase, Hangar generator, plume-check recordings and performance `hit` fixture now use
`native-engagement-v1`. The old tool's hand-written missile motion, fin/thrust animations, scheduled
detonations, repeated target alignment and forced breakup have been removed. The generator rejects legacy
scene fields instead of silently reinterpreting a mechanical fixture. Ten ignored legacy showcase JSON and
recording files under the five missile output directories were removed.

World owns the launcher and target flight, measured radar track/designation, explicit strike consent,
JSBSim missile flight, seeker guidance, fuze and damage. Capture retains the native default `dt=1/120` and
`frameSkip=4`; the default recording rate is 30 Hz. Lower supported integer divisors select native snapshots
without interpolation or changing the guidance cadence. A generic encounter can miss, expire or produce
nonfatal damage. A burst alone is not accepted as a guided hit by the performance fixture.

The black-box regression failed against the old tool, then passed against both maintained native presets.
It decodes the actual recording and checks measured designation, explicit consent, fuel consumption,
acceleration, target flight, native detonation/damage and recorded event timing/position. Repeated runs are
deterministic. Captures at 30 and 15 Hz retain identical native guidance/fuze/damage events and positions at
shared timestamps; unsupported rates and legacy fields fail without retaining old success artifacts.

The earlier head-on scene (target heading 270 degrees, seed 17) has release at 13.033333 seconds and detonation/damage at 17.633333 seconds;
maximum missile speed is 670.252806 m/s. Its target is damaged without native crew loss. The scene's geometry and
aircraft speeds are assumed acceptance inputs, not calibration against a real engagement.

Focused evidence: `D:/ENV/tmp/native-showcase-cpp-red.log`,
`native-showcase-integer-rate-headless-test.log`, `native-showcase-final-render-test.log`,
`native-showcase-probes/`, `native-showcase-missiles-module-green.log` and
`viewer-screenshot-failure-red/result.json`, `viewer-screenshot-failure-green/result.json`.

## Continuous live strike acceptance

The maintained `tools/validation/viewer_perf/strike_acceptance.py` prepares one native scene, validates its preflight
recording, then starts one fullscreen viewer and one live SDK World. The current `hit.json` fixture uses a
target heading of 210 degrees. This is distinct from the earlier 270-degree head-on fixture above; the
earlier damage without crew loss remains attached to that scene. The new geometry is an assumed
acceptance input. It does not force damage, detonation or destruction, or change missile guidance parameters.

```powershell
.\fsim.cmd --preset ucrt64-release python tools/validation/viewer_perf/strike_acceptance.py --build build/ucrt64-release --scene tools/validation/viewer_perf/hit.json --out D:/ENV/tmp/native-strike-acceptance
```

The output directory contains the exact scene, preflight and live native recordings with their engagement
sidecars, raw logs, two PNGs and a success summary. Preflight supplies precise SDK event anchors; it does not
authorize the live target transition. The live viewer requires a matching native weapon detonation and actual
fragment Hit on the expected target, with the expected store, entity slots and event time. Completion also
requires the matching native CrewLost event; its time and target slot are checked in the live IPC stream. It follows the
carrier before release, the actual missile throughout flight, and the target only after that Hit. Missing IDs,
early destructive effects, mismatched native events or a missed capture fail the check.

Exactly two frames are requested: `strike_launch_plus_1s.png` at actual launch + 1.0 s and
`strike_hit_plus_0.3s.png` at actual Hit + 0.3 s. The diagnostic viewer uses authoritative native snapshots
without display interpolation, and synchronous Flow for the recorded event clock. A capture can occur at the
requested native time or at most one 30 Hz World step later (33.3 ms); logs and the summary report the actual
snapshot time rather than relabeling a later frame as exact. The viewer stays open after the second capture,
continuing through the full default 40 s. There is no per-second image series or viewer restart between phases.

The harness decodes both native recordings, checks the complete cadence and healthy pre-hit target state,
requires actual native crew loss, and compares live event evidence with preflight. It also checks
camera order, exactly two event-relative captures, native wall duration and 3840 x 2160 PNG dimensions. These
automated checks support review; the continuous fullscreen flight and both actual images still require visual
inspection. The diagnostic capture run is not a performance benchmark.

The fresh 2026-10-12 run passed on the UCRT64 release build and GPU listed above. One fullscreen viewer
remained attached to the same native World for the complete 40 s encounter. The native process took
40.0301 wall seconds and the viewer 41.9923 seconds; both exited 0. The 1,200 live native frames match the
preflight event evidence, with `damage.destroyed=true` (native crew incapacitation), zero pre-hit destructive effects and 86 matching
hit fragments. This is SDK damage following an external missile strike, not an aircraft self-explosion.

| Phase | Actual native time s | Followed entity |
| --- | ---: | --- |
| Carrier acquired | 0.466666667 | Slot 0, entity ID 1 |
| Actual release | 13.033333333 | Missile slot 2, entity ID 3 |
| Actual fragment Hit | 19.533333333 | Target slot 1, entity ID 2 |

Exactly two PNGs decoded at 3840 x 2160 and both were visually inspected: launch + 1.0 s at
14.033333333 s while following the missile, and Hit + 0.3 s at 19.833333333 s while following the struck
target. Both captures landed on their exact requested native ticks in this run; the tool's maximum
one-step tolerance remains as documented above. The viewer continued to native time 40.0 s after both
captures, without switching windows or restarting the encounter.

Fresh evidence is in `D:/ENV/tmp/native-strike-acceptance-20261012/`: `result.json`, `viewer.log`,
`live.log`, `live-readback.log`, `live.fsrec`, its engagement sidecar and the two named PNGs. Native readback
reports `nativeFrames=1200 preHitDestructiveEffects=0 matchingHitFragments=86 nativeCameraSubjects=3
captures=2 duration_s=40`. This two-image visual review does not expand the earlier scene matrix's count
of individually inspected images.

At this acceptance stage, the focused release tests `strike_capture`, `showcase_native` and `engagement_native`
passed (3/3); the maintained Python missile module passed 20/20. The full 650-test result above remains the
earlier viewer-correction baseline. The later delivery run and ownership correction are recorded below.

### Post-hit flight interpretation

`DamageEvent.destroyed` retains its existing SDK meaning: native crew loss. It is not a structural breakup,
loss-of-lift or terrain-impact flag. The acceptance report now separates `crew_lost`, engine failure, control-channel
loss, torn panels, separated sections, `crashed` and `airborne_at_end`. A confirmed hit and crew incapacitation
must not be described as a confirmed airframe breakup or crash.

The 210-degree fixture records one seized engine and three torn panels, with no separated sections or native
crash through simulation time 40 s. Thrust and turbine core speed are zero after the hit, while recorded SDK
ControlInputs stay fixed. The remaining airframe retains lift; the aircraft model's internal FBW continues
operating. Airspeed decreases from approximately 249.9 to 221.2 m/s and altitude from 5,971.2 to 5,953.2 m over
the remaining 20.47 s. In this model the outcome is an incapacitated, decelerating glide, still airborne at the
end of the encounter. A screenshot at hit + 0.3 s cannot establish its later trajectory.

Native readback checks that modeled engine failures reach later flight snapshots, that unused engine slots
cannot prove a failure, that recorded SDK commands remain fixed after crew loss, and that the target has
samples through the full duration. Synthetic regressions distinguish same-tick pre-damage snapshots from
later integration, missing target coverage, crew loss without a live event, and actual crash flags. The report
does not claim that physical FCS surfaces stop moving or that engine seizure disables electrical/hydraulic
power. Structural/control and power-system fidelity remain separate model-validation work; this delivery does
not force a hit aircraft to tumble, break up or crash.

## Final delivery verification (2026-10-12)

This verification includes the capacity and stale-artifact corrections, live CrewLost evidence and post-hit
flight audit. Release and headless builds use tests enabled, warnings-as-errors enabled and CPython 3.13
from D:/ENV. No concurrent native build or external Viewer ran alongside the suite or live acceptance.

| Check | Current result |
| --- | --- |
| Final Release and headless builds | Passed. Logs: `D:/ENV/tmp/viewer-strike-delivery-20261012/audit-final-build.log` and `headless-final-build.log`. |
| Full Release CTest run | 652/653 passed, zero CTest skips, 945.31 seconds. The only failure was `architecture_boundary`: the private strike header lacked explicit Viewer ownership. All remaining checks, including Vision, RCS, Flow, Python, installed consumers and the fullscreen screenshot regression, passed. |
| Architecture after ownership correction | Release 2/2 and headless 2/2 passed. Only the policy file registration and its README changed after the full run; dependency edges, runtime sources and binaries did not change. The complete 653-test suite was not repeated after this metadata correction. |
| Final headless compatibility/native selection | 11/11 passed in 85.48 seconds: strike capture, native showcase/engagement, SDK boundary, C ABI, control allocation, three adapter gates, Python SDK and installed consumer. |
| Final continuous fullscreen strike | Passed: native exit 0 and Viewer exit 0; 40.0311 and 41.9120 wall seconds respectively. One World produced 1,200 native frames through simulation time 40.0 s. Both 3840 x 2160 captures were decoded and visually inspected. |

The final strike releases at 13.033333333 s, records the target Hit and live CrewLost at 19.533333333 s,
and captures at 14.033333333 s and 19.833333333 s. Camera order is carrier slot 0/entity 1, weapon
slot 2/entity 3, then target slot 1/entity 2. Native readback reports zero pre-hit destructive effects,
86 matching hit fragments and 614 post-hit target samples. Preflight and live outcome reports agree.
The outcome is crew incapacitation, one engine failure and three torn panels, with no separated sections
or native crash and the target still airborne at the end; the flight interpretation above applies.

Raw evidence is under `D:/ENV/tmp/viewer-strike-delivery-20261012/`: `release-full-tests.log`,
`release-architecture-final.log`, `headless-architecture-final.log` and `headless-focused.log`.
The `final-strike/` directory contains the result, native recordings/sidecars, capacity rejection,
native readback, live Viewer log and the two named captures. This run does not remeasure the 4K performance
gate or change the material/shadow limitations recorded above.

## HC-07 verification (2026-10-17)

The current Release build (main `66dc43d`, the binaries HC-11's full suites tested) was run again in one GUI slot at
3840 x 2160 fullscreen. The [HC-07 record](../architecture/validation/hc-07-viewer-and-joint.md) has the evidence.

| Case | Result |
| --- | --- |
| The continuous live strike (`strike_acceptance.py`) | Passed, and the same as this page's final run: release 13.033 s, hit 19.533 s, captures at 14.033 and 19.833 s, the same outcome |
| The replay frames of the four scenes; the demo; a world replaced under the same name | Passed and inspected |
| The demo's pause, step, time factor and quit, through the window's key input | 15/15 checks passed; this is input posted to the Viewer's window, not a human operator |
| The 4K performance gate, run twice | **Failed** for fleet, strike and hit, as in the baseline above; reheat passed (HC-45). Other applications were open, so the machine was not idle |

Mouse, the timeline slider, window resize and a human-operated session remain unverified, as do the combat Viewer
run (HC-06) and the performance gate.

## Reproduction and limits

Use the maintained preset launcher for native viewer checks, retaining the fullscreen argument:

```powershell
.\fsim.cmd --preset ucrt64-release viewer --demo --fullscreen --offline --screenshot D:/ENV/tmp/viewer-demo.png --screenshot-after 5
python tools/validation/viewer_perf/viewer_perf.py --build build/ucrt64-release --out D:/ENV/tmp/viewer-acceptance-perf --label current
ctest --preset ucrt64-release
```

The screenshot parent directory must already exist. Wait for the native process to exit, read its log,
decode each PNG, confirm its dimensions, and inspect the actual rendered scene. A GUI executable launched
directly from PowerShell can return before the process finishes; use the maintained launcher or a subprocess
that waits for completion.

The performance target remains 160 fps at 3840 x 2160: after five seconds of warmup, every second must reach
at least 99% of the target (158.4 fps), and no frame may exceed two refresh intervals (12.5 ms). A completed benchmark is not
necessarily a passing gate. Screenshots pause rendering for readback and are excluded from timing runs.

CLI-driven screenshots and replay checks do not establish manual keyboard, mouse, timeline-slider or
window-resize acceptance. This record does not claim CI execution, other GPUs, uncached map regions or
high-fidelity physical calibration of the native engagement or plume effects.
