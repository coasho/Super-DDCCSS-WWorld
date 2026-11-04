# SIM-M03 native navigation integration continuation

**Goal:** Resume the existing SIM-M03 production implementation and resolve its
recorded native navigation integration failure without weakening its predicates.

**Architecture:** Keep SDK simulation authority, native strapdown execution and
the existing versioned diagnostic sidecar. Simulation control, sensor sampling
and presentation publication have distinct clocks.

**Specification:** The existing [physical completion plan](2026-11-02-sim-m03-physical-completion.md)
and [engineering contract](../../sdk/sim-m03-engineering-contract.md) govern this continuation.
SIM-M01 and SIM-M02 are accepted within the user's confirmed scope.

## Exclusive window

Main HEAD: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.
No native build/test or Viewer process was active at slot acquisition.
The current owner holds the serial native slot for this continuation.
Do not create branches/worktrees or access `.codex` directories.
Preserve all inherited changes; initial hashes and raw failure are retained at
`build/architecture-reviews/sim-m03-resume-20261102/`.

Exact initial write allowlist:

- This plan.
- `tests/simulation/test_weapon_navigation_integration.cpp`: native World
  integration observation and regression only.
- `docs/research/sim-m03/README.md`: append current verified evidence after checks.

Any production correction requires a recorded extension with exact files and
symbols before editing. Do not change public layouts, native outcomes, sensor
constants, original scene or original control/navigation tolerances.

### Receiver admission correction window

Fresh staging reproduced a production refusal before release. The original
descriptor contains physical receiver bindings; `configureSeeker` validates a
normalized scalar descriptor but then checks the original binding count.
The extended exclusive source allowlist is
`src/simulation/mission_systems/weapon/Seeker.cpp`, only `configureSeeker` binding
validation. `weapon_sensor_engineering` independently reproduced 13 failing
cases before the correction. Preserve exact mode/variant and unknown-binding
refusals; use the already validated normalized descriptor for scalar checks.

After that correction the unchanged integration assertions exposed a second
boundary: the World advances in simulation time while presentation publication
is wall-clock throttled. The fixture was reading a valid earlier frame. Use
`publishNow` to snapshot the exact asserted boundary, and additionally require
the selected generation/request/time and unchanged IMU/covariance on repeated
publication. Keep the original exact navigation/control epochs and tolerances.

## Tasks

- [x] Read the existing contracts, implementation and archived checkpoint106;
  reproduce the unchanged failing native integration through `fsim.cmd`.
- [ ] Diagnose selection, generation, navigation and publication timestamps at
  the failing boundary; preserve the original navigation availability assertion.
- [ ] Apply the smallest correction at the demonstrated failing boundary.
- [ ] Rebuild and run the original ten focused CTest entries with
  `--no-tests=error --timeout 180`; retain raw logs and discovered counts.
- [ ] Review the change and expand integration verification only where changed
  behavior or failures require it. Record current evidence and pending full
  regression, installed, model-fleet and actual Viewer gates honestly.

## Complete-package continuation

The user's subsequent instruction requires completion of the entire SIM-M03
package. This integration repair is the first task, not a narrowed delivery.
The complete-quality and physical-completion plans remain binding. Independent
read-only reviews cover sensing, navigation/control/propulsion and effects plus
delivery; root integrates all corrections and executes every native/GUI gate.

### Thermal feature implementation window

The thermal implementer owns the complete source-backed receiver correction:
continuous subpixel optical response before detector integration, visibility-
correct attribution, and MICA IR's explicitly fitted dual-waveband execution.
Exact initial paths are the existing private thermal receiver and scene types
under `src/simulation/mission_systems/components/{implementations/seeker,interfaces}`,
`components/support/ReceiverConfigurationV1.{h,cpp}`, the thermal integration in
`components/implementations/seeker/EngineeringSeekerV3.{h,cpp}`, focused thermal
tests, `assets/stores/mica-ir.json`, and their method documentation.
The worker must enumerate concrete filenames before editing and hand off tests
before implementation. No worker may build, run native tests or a Viewer.
WorldEngagements.cpp, Seeker.cpp and shared registrations remain root-owned.
Preserve old descriptor behavior and private/public vtable/layout contracts;
new functionality uses additive private/versioned interfaces where needed.

### Compatibility fixture maintenance window

Root owns `tests/simulation/fixtures/weapon_v2_assets.py`, the four generated
`tests/simulation/fixtures/weapon-v2/*.json` files and their `manifest.json`.
The retained V2 profiles must formally omit physical V3 receiver bindings and
parameters, retain their four V2 provider IDs and original immediate-observation
assertions. Review semantic deltas before replacing generated fixtures; keep
the strict current-input/output/generator hash verification unchanged. Native
V3 production acceptance and full-model execution remain separate mandatory
gates. No acceptance predicate is weakened by a fixture refresh.

### Root conservation correction window

Root additionally owns `src/simulation/flight/PassiveRolleronV1.cpp`, only the
transmitted wheel-bearing impulse in `advance`,
`tests/simulation/test_passive_rolleron.cpp`,
`tests/simulation/test_weapon_pulse_energy.cpp`, and
`cmake/JsbsimPatches.cmake`, only the opted-in commanded-pulse usable-fuel
admission/reservation path. Preserve the original impulse tables, all legacy
rocket modes and public/private layouts. Add independent RED conservation tests
before production edits. Thermal-worker files are outside the passive-rolleron
target's build inputs; no shared CMake edits occur in that window.

### Dual-band integration and physical scene window

Root exclusively owns `components/registration/SeekerProvidersV1.cpp`,
`weapon/Seeker.cpp`, `runtime/world/WorldEngagements.cpp`,
`tests/simulation/test_weapon_sensor_engineering.cpp` and
`tests/simulation/test_weapon_sensor_native.cpp` for additive dual-band
admission, both on-rail and released thermal scenes, and transparent native
test-proxy forwarding. The thermal worker owns only its separately recorded
files. Every channel must execute its own band-integrated radiance, detector
and noise from the same native scene/estimated pose; actor data never gain
referee identities or truth. Existing single-channel modes remain unchanged.
The same World scene file is root-owned for RF ground-patch propagation:
surface echoes require both actual path visibility and excess losses.

### M03 targeted-build runtime staging

Root owns `tests/CMakeLists.txt` for one M03 runtime-staging dependency and its
exact M03 executable dependencies. Actual execution found a stale
`tests/libJSBSim.dll` shadowing the freshly rebuilt `bin/libJSBSim.dll` after
a targeted build; a PATH change cannot override Windows executable-directory
DLL precedence. The normal M03 build targets must stage current native runtime
and cards before tests. Preserve all generic tests and their existing targets.

### Intact-airframe thermal geometry correction

The original native AIM-9M fixture reproduced a real acquisition failure:
one admitted aircraft sample, zero thermal patches, live finite target/launcher.
`VehicleDamage` is allocated lazily by `ensureDamage`, while both thermal scene
formation and projected extent incorrectly required that allocation. Root owns
`World.h` only for private `seekerGeometryV1` declaration and
`WorldEngagements.cpp` for its implementation/consumption. Intact geometry comes
from the immutable admitted airframe; damaged bodies retain their actual
attached geometry. A sensor read must neither manufacture damage nor mutate
damage RNG/history. Preserve the original scene and acquisition predicate.

## Review focus

### Authored-fit evidence tool window

The fit-tool owner exclusively owns new
`tools/validation/weapon_quality/{fits.py,test_fits.py}`,
`audit.py` only for retaining the authored station count, and new
`docs/sdk/weapon-authored-fits.md`. Root retains campaign/report/CMake integration
and all SDK/native execution. The derived population is 35 configurations,
46 authored loadouts, 214 station bindings and 215 physical stores including
the explicitly authored A-10C dual rail; counts are not acceptance evidence.
Armed fit execution must not be replaced with unarmed carriage. Per-fit referee
integration fixtures remain separate from the existing actual actor/C2 fleet
gate. No new rotor flight exclusions or M10 payload behavior are introduced.

Check one/four-step batching, requested entity and generation, publisher frame
freshness, exact navigation/control epochs, and non-consuming diagnostic reads.
Reading a prior valid snapshot must not be mistaken for a fresh simulation
sample. Publication must never advance the physical simulation.

## Continuation rulings and current evidence

### Dual-band native diagnostic presentation window

Root owns new `foundation/transport/WeaponThermalBandV1.{h,cpp}` and its
transport CMake registration; `World.h`, `WorldWeaponDiagnostics.cpp`,
`WorldWeaponSensorDiagnostics.cpp`; `visualization/ui/WeaponDiagnosticsGui.{h,cpp}`;
new `tests/contracts/test_weapon_thermal_band.cpp`; existing navigation
integration and GPU-free diagnostic UI tests; their test registrations; and
`docs/sdk/weapon-diagnostics-v1.md`. MICA's two native images must be inspectable
through the normal diagnostic window. Use an additive display-only channel
selector sidecar bound to publisher/incarnation/entity/generation/frame. Keep
the entire existing WorldLayout, fsrec and diagnostic V1 frame layouts unchanged.
Switching channels copies the actual detector data and preserves its exposure
epoch; it never resamples, changes tracking or advances simulation. Replay
shows only the channel actually recorded. Old readers retain their V1 contract.

### Native countermeasure receiver evidence window

The countermeasure evidence owner exclusively owns
`tests/simulation/test_weapon_sensor_native.cpp`, new
`tools/validation/weapon_quality/{countermeasures.py,test_countermeasures.py}`,
and new `docs/sdk/weapon-countermeasure-receiver-validation.md`. Root transfers
the native sensor test file after the intact-geometry/RF corrections and retains
all production source, CMake, campaign/report and native execution. Preserve
the original three cadence scenes and their output paths. A separate complete
24-model scene must prove actual native dispensation reaches the physical
receiver and changes detector charge/RF cells through paired native-kernel
counterfactuals, not merely reduce inventory. Keep source identities outside
measurement/association and do not require deception or a hit. Use distinct
raw output files and strict missing/duplicate/source-epoch admission controls.

The M03 runtime-staging dependency includes both current JSBSim and SDK DLLs,
so targeted C++/C99 M03 tests cannot load stale neighboring runtime copies.

Independent source/native-limit review found default-fuel complete-loadout
overweight in A-10C external tanks and C-17A pallets. Use one predeclared
`fuel_fraction=0.5` for every fit scene. Preserve the full authored loads,
native mass limits and functional predicates. Source-only capacity arithmetic
validates all 44 loadouts with declared finite MTOW; the two J-10A loadouts
retain unavailable declared limits rather than fabricated limit proofs. This
plans fuel before functional fit execution; it is not outcome-dependent model
fitting. Receipt validation must bind the policy and exact source/installed
inputs, current controller epochs and complete ripple-body histories.

Thermal feature review round 1: measured-association-history and independent
support findings addressed; scoped SPEC/QUALITY review passes with exact hashes
and original RED retained. Current focused native execution passed four CTest
entries: navigation integration (5 cases/29,471 assertions), sensor engineering
(18/1,188), thermal receiver (20/7,953), and pulse energy (6/611). These overlap
and are not a summed program total. Intact AIM-9M acquisition passed the original
native scene; RF hill-path ARH/SARH validation passed in 85.52 seconds. Full
current regression, installed/fleet/fit execution, actual Viewer evidence and
reproducible delivery remain mandatory.

The additive diagnostic display continuation passes scoped SPEC/QUALITY review
after two independently identified P2 repairs. Future channel requests now fail
without replacing the accepted request. Valid removed thermal captures draw all
retained pixels with their original removed status and sample epoch. Original
RED and fresh Release/Headless GREEN JUnit reports are retained with the display
review and current source/product hash receipt. Release passes three CTest
entries (existing diagnostic contract 7/107; channel contract 2/50; GPU-free UI
7/112). Headless repeats the two contract entries. The normal SDK and Viewer
targets link. No fullscreen or installed-consumer acceptance is inferred.
Whole-program SIM-M03 acceptance remains unestablished. The exclusive display
source/build window is released after this scoped evidence handoff.
