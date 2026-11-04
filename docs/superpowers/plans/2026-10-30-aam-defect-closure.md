# All-AAM defect closure

User assignment: repair every defect exposed by the 2026-10-30 all24 AAM
campaign, preserving the frozen scenarios and native outcome semantics.

Baseline: `main`, `004e1f7455e9a19af231e2d795f65f2bf4c5f4ba`. The checkout
contains substantial pre-existing changes. Preserve them; do not stage or
replace them wholesale. No branches, worktrees, profile memory or excluded
directories are used.

## Acceptance and execution order

1. Diagnose the first departure of MICA EM/IR, PL-10, R-77M and IRIS-T using
   native histories, signed force/moment derivatives, actuator dynamics and
   mass/CG/inertia. Correct the initiating fault through the normal generator
   and SDK control path; never suppress recorded incidence/load or force hits.
2. Review every card against its actual seeker, navigation, support, propulsion,
   control, fuze and damage implementation. Implement missing mechanisms with
   explicit source/assumption provenance and independent mechanism checks.
3. Rebuild generated products using normal Hangar entrypoints. Rerun the
   unchanged 48 encounters, full per-model telemetry/fin audits and additional
   support-loss, off-axis, maneuver, countermeasure and propulsion checks.
4. Rerun all24 live fullscreen 1x complete-scene cases in one GUI slot. Preserve
   the original report as before evidence and publish a separate after report.
5. Update the existing fidelity guide and status with measured before/after
   results and each remaining external-data gate. Missing exact-model public
   measurements cannot be converted into calibrated results by simulated fits.

## Initial exclusive feature window

The first feature owns the physical-control diagnosis and repair. Exact source
allowlist: `src/simulation/mission_systems/weapon/Autopilot.{cpp,h}`,
`tools/models/hangar/hangar/storecontrol.py`,
`tools/models/hangar/tests/test_storecontrol.py`,
`tests/simulation/test_missile_control_native.cpp`, and the existing focused
autopilot tests identified during diagnosis. Generated store XML/cards/tables
may change only through their normal generator, with explicit per-model hashes.
Evidence and diagnostic scripts are confined to
`build/architecture-reviews/aam-closure-20261030/`.

Initial symbols: `cascade`, `finCommands`, `ControlAxis`, `AutopilotInput`,
`storecontrol.describe/install`, and their focused fixtures. Further feature
windows must extend this list before their first production write. Generic
gateway/schema/MA/radar work and unrelated Viewer changes are outside scope.

The reproduced first fault is upstream of control: all six wingless AAMs with
negative Mach lift factors use the first strake as a fictitious main wing in
`mach._tail_downwash`. The second strake receives almost 100% downwash, and the
tail fins about 75%. Scaling that correction by the first strake's supersonic
slope drives downwash above one and reverses the entire aircraft's lift and
rate damping. Extend the initial allowlist to
`tools/models/hangar/hangar/aero/mach.py` (`_tail_downwash`, `mach_effects`),
`tools/models/hangar/tests/test_mach_topology.py`, and the normal generated
products of all24 AAMs. Preserve the existing fully coupled VLM base; do not
substitute a fitted scalar or clip negative output tables.

No native configure/build/test/Viewer process was active at the initial process
check. The root owns the serial native slot for this assignment; all native
build inputs are frozen while a build or test runs. TMP, TEMP and
TORCH_EXTENSIONS_DIR use `D:/ENV/tmp`. GUI execution is sequential.

## Progress

- [x] Read original campaign, fidelity guide, status and project-local Hangar skill.
- [x] Establish the initiating physical-control fault with reproducible evidence.
- [x] Repair and verify the complete all24 control slice.
- [ ] Close remaining modeled-mechanism defects and per-model checks.
- [x] Complete fresh all24 native and visual diagnostic acceptance.

## Finite-actuator control follow-up

The first new-mechanism campaign is retained at `final/` as a failed attempt:
PL-10 still oscillates after Mach 2.6 and enters reverse flow. The small-angle
moment slope changes from stable to unstable across this region, and measured
nonlinear loads amplify the finite-actuator lag. The old instantaneous inner
inversion does not assign the actuator pole. Add a separately identified
`ActuatorAwareCascadeAutopilotV1` with an analytic three-state angle/rate/actual
surface feedback solve, retaining existing engineering response-time inputs.
Preserve the original provider and unversioned controller arithmetic. Require
same-frame actual controls, reject uncontrollable/nonfinite solves, and derive
the feasible curvature again when the existing incidence limit is reached.
No card maximum-g, fuze radius, gain or engagement condition is tuned.

Additional exact allowlist: the new class's `.h/.cpp` in
`src/simulation/mission_systems/components/implementations/autopilot/`, its
registration and CMake source entry, `weapon/Autopilot.{h,cpp}`,
`weapon/PlantAdapter.cpp`, `tools/models/hangar/hangar/components.py`, the
all24 cards' autopilot implementation identity and normal generated outputs,
and focused autopilot/composition/provider tests and source index. The new
provider uses native autopilot implementation code 2; existing code 1 and
public interfaces remain supported. Verify independently with finite-lag
linear plants and the unchanged native PL-10 encounters before all24 reruns.

The first actuator-aware-only trial repaired head-on but still departed in
tail pursuit. It is retained in its raw run log. PL-10 therefore also requires
local incidence/Mach Jacobians: zero-incidence moment slopes differ materially
from the emitted nonlinear loads even at four degrees. Extend `storecontrol`
and its independent emitted-XML force/moment finite-difference tests. Native
metadata version 2 requires these local derivatives; the World refuses old
version-1 derivatives for this selected provider. Only PL-10 selects the new
controller. Other currently stable profiles retain their existing controller.

Add `tools/validation/missile/propulsion_controls.py` for native four-model
nozzle conservation/jam checks and Meteor positive-airflow/no-fuel checks.
The shared Mach topology fix also changes wingless sibling-store generation.
The unchanged-output MK82 regression correctly exposed stale published sibling
XML. Extend the generated-product allowlist to the other 29 stores' normal
Hangar aero/build outputs; retain every numerical fit/configuration and repeat
the independent MK82/A-22 retained-reference checks. No new calibration or
outside-source repair is authorized by this dependency synchronization.

Fresh replay caught version-2 PL-10 fin telemetry being rejected by a
version-1-only native read gate. Extend the exact allowlist to
`src/simulation/runtime/world/WorldWeaponControls.cpp` and the existing native
control fixture: admit the selected native metadata version while retaining
the unchanged public fin schema, stable IDs, values and renderer assertions.
Wrong/missing versions must still refuse. Retain the failed replay artifact.
The legacy numerical-equivalence test now explicitly selects retained legacy
providers for all53 stores. Current TVC/ducted/local-derivative mechanisms are
covered by separate source/native checks; their intentional physics changes
must not be described as byte-identical legacy behavior.

Final generator review also requires typed ducted designs to refuse a missing
or incompatible propulsion profile. Scaffolding/migration must preserve already
selected TVC, finite-actuator and ducted-cycle declarations instead of silently
replacing them with generic defaults. The legacy Meteor equivalence fixture uses
the actual old untyped motor input. Current valid generated products must remain
byte-identical; no further native physics or visual outcomes change.

The final report audit also corrects the lifetime filter to compare recorded
World time with launch/end World times, never local vehicle age. A synthetic
slot-reuse regression must exclude pre-launch and post-terminal samples. Apply
this filter equally to the raw before/after recordings, retaining the original
report unchanged. This is a statistics repair, not a native outcome change.

## Verified diagnostic handoff

Main remains `004e1f7455e9a19af231e2d795f65f2bf4c5f4ba`; unrelated initial
working-tree changes are preserved. The feature remains uncommitted. Exact
source/product fingerprints and current native executable hashes are in
`build/architecture-reviews/aam-closure-20261030/delivery.json`.

Current evidence: 60 Python tests, 12 distinct current native gates (11 passes
plus a targeted fixture retest), all48 unchanged native scenes, all24 complete
fullscreen1x live cases with96 inspected native-resolution captures, and83,436
actual fin angles through production VehicleVisuals with zero hinge/rate/stop
violations. All53 valid normal generated products remain byte-identical after
the final input/migration safeguards; all24 frozen card/XML and native binary
hashes agree. The additional AIM-120C event-relative strike check preserves
native fragment Hit, crew loss and post-hit flight semantics. PL-10 does not
satisfy that separate crew-loss predicate and is not promoted into a pass.

The [issue ledger](../../../build/architecture-reviews/aam-closure-20261030/closure.json)
explicitly remains incomplete for equipment closure: exact-model measured
calibration, GPS/fitted weapon-link/commanded pulse mechanisms and full-envelope
acceptance are pending. The existing feedback contract requires its reviewed
versioned extension and owner handoff; no generic gateway or actor truth
contract was changed. Native configure/build/test and GUI slots are released
at this handoff; no background simulation is retained.

## Measured-seeker fleet window

After the control-only campaign has frozen its generated products, promote the
remaining 20 cards to the existing independent measured-seeker families and
measured PN. This removes Cartesian target position/velocity from terminal IR
and SARH steering. ARH retains its admitted geometric range/radial measurement.
Each family keeps its own association/lifecycle behavior and each card keeps
its existing explicit numeric inputs and capability declaration. This is a
software mechanism repair, not calibrated detector, AESA, GPS or link fidelity.

Additional exact allowlist: `src/simulation/mission_systems/weapon/Seeker.{h,cpp}`
(`configureSeeker`, the private supported-variant table),
`src/simulation/mission_systems/weapon/Composition.cpp`
(`configureCompositionV1` variant/card identity check),
`tests/simulation/test_weapon_seeker_models.cpp`, the `seeker`/`guidance`
descriptors in the 20 currently legacy AAM cards, and their normal generated
store products. The original four variant bindings remain unchanged. Unknown
variants, mismatched family/variant/card, unsupported hardware fidelity bindings
and unpaired measured guidance must still refuse atomically. Public signatures,
layouts, vtables and permissions are unchanged.

The serial control-only campaign retains the old compiled executable and
unchanged card descriptors until its 48 frozen cases finish. The subsequent
final campaign must use the rebuilt current executable and current products.

Preserve authored inertial/launcher-update capabilities when promoting IR/SARH.
Extend this window to `components/support/MeasuredSeekerV2.{h,cpp}` (admitted
navigation seed, timestamped propagation, reset),
`components/registration/SeekerProvidersV1.cpp` (native initial seed), and
`runtime/world/WorldEngagements.cpp` (allow the existing admitted launcher
estimate for every profile explicitly enabling updates). Paths are relative
to `src/simulation/`. The estimate stays separate from terminal bearing samples.
No new truth path, link performance claim or public/vtable slot is introduced.

Also include `components/implementations/seeker/ActiveRadarSeekerV2.{h,cpp}`:
its existing reset vtable slot must clear activation/navigation as well as
bearing history. Include `tools/validation/viewer_perf/aam_campaign.py`:
report current outcomes rather than hard-coded statements that particular
models always depart or miss; do not rewrite the frozen before report.
Include `components/implementations/seeker/{ImagingIrSeekerV2,SemiActiveRadarSeekerV2}.cpp`:
invalid navigation frames must not overwrite their last accepted cached sensor
result. Add a repeated-epoch regression after a rejected future navigation fix.

## Ducted-rocket mechanism window

Meteor requires a real distinction between an onboard-oxidizer solid booster
and an atmosphere-fed, throttled sustainer. The manufacturer confirms this
mechanism but publishes no admitted cycle map. Use a separately identified
engineering thermodynamic model with explicit nonzero input uncertainty;
never label it measured Meteor propulsion or fit it to encounter outcomes.

Exact additional allowlist: `tools/models/hangar/hangar/{ductedrocket,components,storebodies,storestages,missiles}.py`,
`tools/models/hangar/tests/test_ductedrocket.py`,
`src/simulation/mission_systems/weapon/PlantAdapter.cpp`,
`src/simulation/mission_systems/components/registration/PropulsionProvidersV1.cpp`,
the two burning-state assignments in
`src/simulation/runtime/world/WorldEngagements.cpp`, Meteor's authored TOML/card
and normal generated engine/XML products. Preserve original launch mass,
launch CG/inertia and authored booster. JSBSim integrates both engine forces,
finite valve motion and fuel/mass consumption; no external guidance force is
introduced. The new propulsion adapter requires its distinct native identity
and exactly two engines. Existing solid-rocket cards retain code 1 and one engine.

References: [MBDA's Meteor mechanism](https://www.mbda-systems.com/meteor-missile-and-f-35a-one-step-closer-operational-capability),
[NASA ramjet thrust](https://www.grc.nasa.gov/www/BGH/ramth.html),
[NASA ramjet operation](https://www.grc.nasa.gov/WWW/k-12/airplane/ramjet.html)
and the existing JSBSim 1.3.1 rocket, nozzle and finite-actuator implementation.
Cycle approximation: captured air, finite fuel flow, total enthalpy,
pressure recovery, a fully expanded equivalent nozzle and net momentum thrust.
No measured inlet/combustor/nozzle map is claimed.
Include the concrete
`components/implementations/propulsion/JsbsimDuctedRocketPropulsionV1.{h,cpp}`,
its one registration in `src/simulation/mission_systems/CMakeLists.txt`,
`tests/simulation/test_payload_plant_adapters.cpp`, and the new provider's
source-index entry in `tools/validation/payload_models/report.py`.

## Finite thrust-vector mechanism window

Provide an explicitly assumed equivalent thrust-vector actuator for the
manufacturer-confirmed AIM-9X Block I, IRIS-T and MICA EM/IR mechanisms.
Keep actual aerodynamic fins, their limits/rates and the existing controller.
JSBSim's native nozzle orientation applies the physical thrust force and its
moment about current CG. The controller must use combined signed control
effectiveness and same-frame aerodynamic/propulsion feedback; adding a second
unaccounted torque is forbidden. No attitude overwrite or ideal steering force.

Exact additional allowlist: `tools/models/hangar/hangar/thrustvector.py`,
`storecontrol.py`, `components.py`, `tools/models/hangar/tests/test_thrustvector.py`,
`src/simulation/mission_systems/weapon/Autopilot.h` (private configuration only),
`PlantAdapter.cpp`, `src/simulation/runtime/world/World.{h,cpp}` (private reads,
signed nozzle effectiveness and feedback), the four cards' autopilot
descriptors and normal XML outputs. The versioned optional autopilot binding
requires explicit finite vector-angle/rate inputs; missing/malformed native
metadata refuses rather than retaining fin-only control. Existing public
structures, plugin vtables, fin IDs, telemetry and legacy defaults remain intact.

The equivalent vector limit/rate are engineering assumptions with nonzero
uncertainty, not recovered jet-vane or nozzle hardware curves. Manufacturer
mechanism references: [MBDA MICA datasheet](https://www.mbda-systems.com/sites/mbda/files/2024-07/2022%20MICA%20datasheet.pdf)
and [Diehl IRIS-T](https://new.diehl.com/defence/en/products/guided-missiles/iris-t2).

The native contract selection exposed a pre-existing R-73 declaration mismatch:
its authoritative component fuze uses 9.720294101747088 m, while an absent
canonical radius falls back to a heuristic from the subsequently sourced
8 kg warhead (9.82842712474619 m). Preserve the actual component radius and
native outcomes. Add the existing radius as an explicit canonical
`fuze_radius_m` alias in `assets/stores/r-73.json`; do not change either used
fuze radius or the manufacturer's warhead mass. Add
`tests/simulation/test_payload_composition.cpp` to the allowlist: require
exactly seven autopilot roles only for the four explicitly declared vector
profiles, five for all others, and validate their actual vector configuration.
The retained-value contract and its numerical assertions remain unchanged.
The three native stale-asset refusals also exposed an incomplete fixture:
`tests/simulation/test_payload_composition_native.cpp` copied only XML/engines,
omitting the now-required source-bound VLM carriage JSON. Include that fixture
path; copy its genuine generated table before corrupting the native fingerprint.
Retain every refusal, inventory/history and identity assertion unchanged.

Upstream references for the control investigation: JSBSim's official
[reference frames](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/)
and [force/moment frames](https://github.com/JSBSim-Team/jsbsim/wiki/Force-and-Moment-Frames-of-Reference),
and Johns Hopkins APL's
[Missile Flight Control Systems](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf).
These establish conventions and control principles; they do not supply
exact-variant equipment calibration.
