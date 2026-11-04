# SIM-M03 production closure

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.

The user confirms accepted SIM-M01/M02 scope and assigns complete SIM-M03
engineering delivery for all maintained models and applicable components.
Retain the existing complete-quality and final-delivery contracts, original
scenarios and tolerances, native outcome semantics and separate equipment
accuracy ledger. Existing uncommitted changes are preserved.

## Exclusive ownership and evidence

Live process inspection found no competing native configure/build/test or
Viewer process. Root holds the exclusive native and GUI slots. Independent
read-only physical and delivery audits follow the complete-quality plan; they
cannot edit inputs or execute native/GUI work.

Initial write allowlist: this plan and new raw diagnostic/evidence files under
`build/architecture-reviews/sim-m03-production-20261103/`. Before a production
edit, extend this record with exact paths/symbols and retain inherited bytes
and hashes in that evidence directory. No generic gateway, radar or external
worker ownership is transferred. No `.codex` access or other worktree is allowed.

## Observed failures to reproduce and repair

The inherited final integration selection failed two of 60 entries:
`engagement_native` timed out at its original 300-second bound and
`showcase_native` failed its unchanged positive-detonation predicate. The actual
5 km AIM-120C recording reports 682.4287793607305 m closest approach with
intermittent acquisition and no terminal event. These are implementation
failures, not accepted engineering results or equipment calibration limits.

1. Capture current receiver, navigation, measured guidance and actual fin
   execution in the unchanged scene; identify and independently reproduce the
   causal defect before changing production behavior.
2. Repair the production mechanism while retaining input values, seeds, scenes,
   timing, tolerances and outcomes. Recheck all affected models and cadences.
3. Run complete normal generation, current full regressions, installed public
   interfaces, authored fits/fleet, complete-scene fullscreen 1x presentation
   and exact-source reproduction using the existing campaign.
4. Resolve independent review findings, bind source/product/research/evidence
   bytes and deliver only after every required engineering gate passes.

## RF geometry repair window

Delegate `m03_physical_audit` exclusively owns
`tests/simulation/test_weapon_radio_receiver.cpp` for an independent spherical
diagonal-ray regression, then
`src/simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.cpp`
only for the two monopulse-angle inverse transformations and
`docs/sdk/weapon-radio-receiver.md` for their coordinate convention. Root runs
the unchanged native slot, first records the failing regression, then authorizes
the production correction. Inherited bytes/hashes are retained in
`rf-geometry/window.json`. No card, waveform, threshold or detector noise changes.

## Physical factory navigation initialization

Delegate `m03_delivery_audit` exclusively owns
`tests/simulation/test_weapon_sensor_engineering.cpp` for the actual physical
MICA-IR factory regression and
`src/simulation/mission_systems/components/registration/SeekerProvidersV1.cpp`
only for applying the authorized initial navigation seed after successful
physical receiver configuration. Root runs RED before the production correction
and GREEN afterward. Inherited bytes/hashes: `factory-navigation/window.json`.
Retain reset behavior, original authored receiver policy and absence semantics.

## Measurement-memory continuity

After both delegate write windows close, root owns
`tests/simulation/test_weapon_sensor_engineering.cpp`,
`components/implementations/seeker/EngineeringSeekerV3.cpp` under the existing
mission-systems source directory, and `docs/sdk/weapon-sensing-engineering.md`
only for retaining the measured angular track identity/filter and finite pointing
through a missed exposure inside the existing authored memory interval. A missed
exposure remains no detection; it gains no attribution, measurement epoch or
tracking flag. Expired memory cannot seed a reacquisition. Add an independent
gap/reacquisition/expiry regression before correcting production. Snapshot:
`measurement-memory/window.json`. No gains, fields, thresholds, periods or scenes
change. Legacy V2 semantics remain unchanged.

The continuity correction also owns the private nonvirtual
`MeasuredSeekerV2::forgetTrack` helper in `components/support/MeasuredSeekerV2.h`
to expire only angular association without resetting navigation, active-radar
phase or random streams. Only the V3 wrapper invokes it at memory expiry;
retained V2 sample behavior is unchanged. Snapshot: `extended-window.json`.

## Time-armed physical proximity scene

Delegate `m03_physical_audit` owns `src/simulation/runtime/world/World.h` only
for private Entry per-candidate fuze histories, `World.cpp` only for clearing
those histories at existing reset/terminal transitions, `WorldEngagements.cpp`
only for explicit V1 physical proximity scene collection, earliest captured
trigger selection and cleanup, new
`tests/simulation/test_weapon_proximity_native.cpp`, and
`docs/sdk/weapon-fidelity-validation.md` for the corrected contract. Root owns
only the corresponding target/test registration in `tests/CMakeLists.txt` and
all native execution. Snapshot: `proximity-scene/window.json`.

First record actual native failure with an unavailable main seeker, a captured
armed encounter inside the unchanged sphere, outside/prearming/jettison controls
and deterministic candidate ordering. Preserve the existing provider vtable,
legacy V0 behavior, native captured paths, per-candidate generation/lifetime,
time arming, single initiation/delay, contact/terrain/life ordering and radius.
The selected V1 model is an explicitly assumed time-armed ideal spherical CPA
detector. Its previously undeclared current-main-seeker prerequisite is removed;
this does not reconstruct hardware electronics or assert that every real fuze
is independent of guidance enable commands. Navy Fundamentals sections15.2 and
17.7.3.2 distinguish these functions and real enable policies:
<https://man.fas.org/dod-101/navy/docs/fun/part14.htm> and
<https://man.fas.org/dod-101/navy/docs/fun/part16.htm>.
Physical fuze scene truth remains outside guidance and actor observations.

## Moving-origin fragment angular impulse

After the current native gate releases its inputs, delegate `m03_effects_review`
owns `src/simulation/mission_systems/damage/Damage.cpp` only for the scene
crossing angular moment, `Damage.h` only for the scene convention comment,
`src/simulation/runtime/world/World.h` only for private damage application
signatures, `WorldDamage.cpp` only for those application wrappers, torque
reference selection and scene call sites, `tests/simulation/test_damage.cpp`
for independent angular-boost/captured-motion/batch tests, and
`docs/sdk/sim-m03-effects-quality.md` for the convention and numerical limit.
Snapshot: `moving-origin-moment/window.json`.

First reproduce the Galilean angular defect before correcting production. The
scene kernel must return moment about the instantaneous moving structural
origin; scene application must not subsequently apply the historical fixed-world
origin translation again. Preserve the old synchronous/external moment contract,
all public layouts/vtables, source field/energy/impulse, random draws and original
scenes. No fixture tolerance or physical coefficient changes are allowed.

## Measured terminal inertial coast

Root owns `components/implementations/seeker/EngineeringSeekerV3.cpp/.h` and
`components/implementations/guidance/MeasuredPnGuidanceV2.cpp/.h` under
`src/simulation/mission_systems/`, `tests/simulation/test_weapon_sensor_engineering.cpp`
and `docs/sdk/weapon-sensing-engineering.md` for explicit V3 terminal inertial
coast. Snapshot: `terminal-coast/window.json`.

The unchanged30km native case retains its actual29.223m miss. Its captured trace
shows RF loss at the physical750m transmit eclipse, then a return from Lost to
Midcourse after the existing0.12s association memory. That both restores launch
loft and discards the relative geometry measured by the onboard ARH receiver.
First reproduce these two behaviors independently. Reconstruct an onboard target
navigation estimate only from actually available range, range rate, filtered
LOS/rate and the same-epoch estimated own pose; retain original capture epoch and
normal delayed delivery. Passive sensors gain no range. Association memory and
the physical eclipse stay unchanged; no current detection or attribution is
invented. A private V3 terminal-phase latch prevents loft restart. Preserve V2
behavior, reset/new-track semantics and all public layouts/vtables. No card,
gain, threshold, timing, scene or acceptance sphere changes are authorized.

The terminal continuation additionally owns `World.cpp` only at its existing
pose-reset boundary, new private `components/interfaces/GuidanceStateResetV1.h`
and `tests/simulation/test_weapon_proximity_native.cpp` for resetting the new
optional guidance state without altering authority/tasks/C2. It also owns
`tests/simulation/test_engagement_native.cpp` only for extending its existing
opt-in referee trace with an observation-only measured-guidance provider wrapper.
The wrapper must delegate exactly once and return unchanged output; it is never
active without the pre-existing FSIM_ENGAGEMENT_TRACE option. Preserve original
scenes, timings, assertions and native outcomes. Snapshot: `extended-window.json`.

## Finite full-channel radar track fit

The unchanged30km diagnostic reproduces the prior14.806m miss exactly with the
observation-only wrapper. Its last actual RF sample has35.68m/s target velocity
error; during eclipse the relative prediction error grows at the same rate.
Root additionally owns new private
`src/simulation/mission_systems/components/support/RadarTrackFitV1.h` and its
method cases in the existing sensor-engineering test. Reuse the accepted native
tracking linear algebra without modifying its owner. Fit actual ARH range/LOS
position pseudo-measurements and Doppler jointly over the existing authored
association-memory interval, in the same estimated own-navigation coordinates.
Weights use actual receiver angular variance, the existing declared angular
noise floor and actual waveform range/Doppler quantization. No new fitted
coefficient, source identity, truth channel, longer memory or hardware capability
is introduced. Filtered angular state feeds both guidance and inertial coast;
the admitted scalar range/range-rate channels retain their original values.
Validate exact constant-velocity geometry under own maneuver/common bias,
observability, invalid/reordered input, and actual physical receiver/reset before
retrying the unchanged30km gate. This is a finite-window engineering estimator,
not an OEM reconstruction or a posterior-covariance accuracy claim.

Primary guidance-filter context: Palumbo et al., Guidance Filter Fundamentals,
JHU/APL29(1),2010,pp60–70,
<https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_Guidance.pdf>.
The joint weighted linear fit and its finite-window assumptions are derived
explicitly in the method guide; no paper's results are claimed as SDK results.


The finite-window prototype was rejected after its unchanged30km native gate
regressed from14.806m to37.3304m (sampled closest37.4543m). Its exact source,
method tests and hash manifest are archived under
`build/architecture-reviews/sim-m03-production-20261103/rejected-window-fit/`;
raw physical traces remain under `terminal-coast/`. No accepted test/scene,
noise, gate, radius or coefficient was modified. Method algebra passing did not
establish noisy terminal quality. Production selection returns to the preceding
measured geometry/coast and phase/reset repairs while a source-admitted support
velocity estimator is independently specified and verified.


## Admitted support transverse-velocity prior

Root continues the same terminal-coast source/test/document window. At an actual
physical ARH range/range-rate capture, preserve own-navigation-correlated target
position and use delivered support velocity only transversely, replacing its
radial component with actual Doppler plus own estimated radial velocity. Use
only spec.launcherUpdates, frame.supported and the deliveredInput-backed frame
estimate, its original finite nonfuture fused state epoch, monotonic adopted
epoch and the existing acquisition cue angle. No covariance, new resolution-
based sigma gate, truth identity or longer lifetime is inferred. Preserve raw
angular guidance and passive behavior. No RF capture means no coast refresh.
Contact replacement/reset clears prior history. An unobservable wrong transverse
prior remains an explicit possible error, never a forced truth correction.

Independent method RED: support-red.log records 1 case,1534 assertions,76
failures. The first implementation's original complete suite left only two
new fixture bookkeeping failures: the inconsistent prior was introduced before
any rate-bearing sample existed. Moving that test-only transition from0.04s to
0.08s exercises both supported and rejected captures without changing any
physical/acceptance scene. support-green.log records30 cases,7024 assertions
passed. Original30km physical gate remains independently required.


## Complete authored explosive effects execution

Independent effects review found the existing CJ-10A installed receipt proves
only continued coordinate-guided flight, not terminal fuze/warhead execution.
Root delegates m03_effects_review an exclusive new-file window for
`tests/simulation/test_weapon_effects_population.cpp`,
`tools/validation/weapon_quality/effects_population.py` and its admission tests,
and the effects guide only. The new fixture must independently enumerate every
actual explosive card and exercise the selected authored scene emission,
assessment/native damage. Add an orthogonal real armed CJ-10A native terminal
path and inert control without changing the original cruise or other scenes.
Root owns target registration/CMake and report/campaign receipt joins after
reviewed handoff. The delegate runs no native build/test; root retains that slot.
No shared simulation/provider source write is granted by this window.


The first support-coast run retained an actual13.8162m miss (sampled16.6329m).
Independent matched-epoch trace analysis found RMS true late LOS rate0.0191rad/s
versus finite bearing-derived0.1472rad/s, with repeated35g reversals that the
unchanged finite plant cannot follow. No frame-map defect was found. Root extends
the same supported-capture estimator to an explicit prior-derived kinematic
angular rate P_perp(v_prior-v_own)/range, retaining measured direction/range and
original source epochs. Doppler cancels from this transverse equation; it must
not be called an independent measured angular rate or optimal covariance fusion.
No coefficients, finite actuators, scenes or acceptance predicates change. The
unsupported/passive bearing tracker remains independently required. The exact
rate-method RED records1case1594assertions60failures before this extension.
Independent authority review accepted this disclosed dependency and required
wrong-prior, missing/unsupported/inconsistent/reordered-source and epoch controls.


## Expanded native verification and effects admission

The supported-rate original30km native gate passed at2.67138m with exact one/two
worker event determinism. Full sensor methods passed30cases7084assertions.
The current expanded Release invocation discovered102 CTest entries:101passed,
1failed in690.41s. The failed engagement entry preserves three failures:
5km functional damage after an actual1.38667m detonation and damage event, plus
two stale test helper assumptions that compare named equipment selection IDs
directly with underlying mechanism IDs. Root corrects only that identity decode
through the existing equipment::mechanism mapping, leaving physical assertions
and both retainedV2/currentV3 fixture branches unchanged.

Root now owns the exact effects target/runtime/admission registration in
`tests/CMakeLists.txt` and `cmake/ArchitectureChecks.cmake`, plus new mandatory
source-epoch/receipt joins in `tools/validation/weapon_quality/campaign.py` and
`report.py` and their README. Every applicable explosive methods/native row must
bind actual52card/preset executions and8CJ10 terminal controls; a broad flight
receipt no longer substitutes for this component evidence. Static parses pass.

The first registered effects run built successfully and preserved2native case
failures in220097assertions: R-27ER real-world fuel-hole predicate and a CJ10
fixture coordinate error (absolute instead of shared origin). Raw first-run
material/source tables are archived in `effects-population-first-raw/`. Fixture
repairs remain delegated solely to the new fixture/collector, without changing
old scenes or authored materials. The native5km functional failure remains
required; an observational diagnostic will establish actual component/section
outcomes before any production correction is proposed.


## Finite rod born inside a closed skin

Independent R27ER actual-card RED exposed a concrete geometric distinction:
finiteLineContact clips occupancy to t>=0, and its skin caller mistakes a line
already wholly inside the box for an entry at birth; a nonexistent THOR skin
crossing may consume it before the real overlapping fuel component. Root
delegates m03_effects_review `tests/simulation/test_damage.cpp` only for an
independent rod-interior/true entry/exit regression, then after root recordsRED,
`src/simulation/mission_systems/damage/Damage.cpp` only for geometric skin-crossing
selection and the effects guide. Snapshot `rod-interior/source-before.json`.
Do not change authored materials, ray semantics, source topology, actual fuel
geometry, energy/momentum or assertions. Root alone runs native gates. The
existing all-card fuel-hole predicate remains unchanged.


The root observational test window additionally uses only the existing TRACE
option to enable the current publisher/IPC source selection reader and dump
bounded real retained fragments/component energy plus full target state before
the unchanged functional assertion. There is no Viewer, resampling, time-step,
RNG, source or physical predicate change. Exact proposal/hash preserved in
`effects-observation/manifest.json`; original non-TRACE execution is retained.
IR first-release telemetry is added to the existing failing native case to
confirm whether pending on-rail packet attribution loses its separate scoring
map. A later repair, if reproduced, owns World.cpp only at that same provider
handoff and moves existing attribution history alongside the provider; no
referee identity may enter sensor/guidance inputs.


## Typed structural skin mass

Independent source inspection found a declared/implemented type mismatch:
Hangar SKIN_SHARE is documented as a fraction of structure, but skin_kg uses
whole section mass including engines, systems and gear. Root delegates
m03_effects_review tools/models/hangar/hangar/damage.py only for typed structural
skin allocation and independent Hangar tests (exact test path recorded at
handoff), plus the effects guide. Derive per-section structure from existing
mass_model.items kind==structure and existing breakup point weights. Preserve
SKIN_SHARE=.35, all geometry, section/whole mass/CG/inertia, strength and flight
coefficients. First require heavy nonstructure and relocated structure controls;
retain RED/GREEN logs. No native generation/build execution by delegate.
Snapshot skin-structure-mass/before.json. Root regenerates all designs and runs
native/production gates after reviewed handoff; this is not equipment calibration.


## Authored skin versus ownership caps

Rod correction passes6cases67assertions and the complete selected scene-effects
population passes35cases11241assertions. The authored effects rerun advances
through all R27 variants but preserves R77M fuel-hole failure; all4CJ10 armed/
inert/frame-skip cases now execute the unchanged physical terminal predicates.
Source inspection found body panel area is lateral only, yet the old damage
box query charges every artificial axial ownership cut as a skin. The existing
immutable contact geometry already distinguishes real skin and ownership-cap
triangles. Root delegates m03_physical_audit the exact Damage.cpp closed-body
skin query and independent test_damage.cpp partition/point/rod/interior/boundary
controls plus effects guide, firstRED then root-gated implementation. Reuse the
maintained contact meshes, preserve original array identities and skin laws,
never add a physical bulkhead at an ownership cut or expose a torn cap as new
homogeneous armour. No new public layout, provider vtable, material coefficient,
source geometry or acceptance predicate change is granted. Legacy absent-mesh
synthetic boxes retain their explicit existing closed-box semantics.


## Finite liquid transit (new additive private method)

Primary FAA research and code inspection confirm empty capacity fuel boxes are
currently perfect fragment absorbers, independently of actual contents. This
cannot be certified as current-content finite liquid shielding. Root owns the
complete correction and delegates only independent pure method primitives in
new damage/FluidTransitV1.h and tests/simulation/test_fluid_transit.cpp to
m03_delivery_audit. These files do not change existing Target, SceneVictim,
Airframe layouts or provider vtables. Required inputs remain explicit: density,
liquid drag coefficient, actual contents/occupancy and relative material motion.
No equipment Cd is inferred from the air coefficient or calibrated to a hit.

The primitives cover volume-conserving hydrostatic box clipping/free-surface
inversion; segment immersion; exact point quadratic-drag motion/energy/momentum;
and consistent uniform-line force/generalized acceleration/moment/power over
only its actually immersed interval. Empty liquid means zero liquid shielding.
Independent analytic/quadrature/boost/partial/invalid-state controls precede
production integration. Native captured tank contents, detached state and the
chronological scene kernel integration remain a subsequent root-owned exclusive
shared-source window after authored-skin handoff. No unimplemented primitive or
research note counts as production completion.


Authored-skin GREEN passes the unchanged4REDcases plus a nonconvex4crossing
control, together with rod regressions; the selected whole scene-effects set
passes40cases11447assertions. Current Release exact26 authored effects and all
4CJ10 armed/inert/cadence cases pass in15.93s. The original5km functional gate
passes without scene/source/material coefficient changes: actual engine loss,
5scontinued finite flight, speed249.752->152.975m/s and971.527mtravel. Original
miss1.38667m remains identical under the observation-only publisher. These are
focused current receipts, not a complete-program/full-regression assertion.

The IR metadata handoff also passes both original cases after actualRED
tracking1/holding0 -> GREEN tracking1/holding2 at the identical first delivery.
The independent on-rail source review is hash-bound in its new receipt.

A further required distinction remains under implementation: capacity fuel
probe boxes currently absorb all fragment mass even when native tank contents
are zero. The new primary research describes finite liquid drag and required
partial-immersion/current-content/geometry inputs. Current vulnerability fitting
also shrinks geometric boxes without conserving their nominal capacity volume;
no fake density or unimplemented liquid model may be certified from that data.


## Liquid occupancy producer and frozen seam

The current all35 source audit finds32 liquid models and149 native tank IDs;
skua/cf2/iris are explicitly liquid-inapplicable. Old vulnerability probes are
not capacity-conserving material containers. Root delegates m03_effects_review
new `tools/models/hangar/hangar/liquid.py`, new
`tools/models/hangar/tests/test_liquid.py`, normal producer calls only in
`tools/models/hangar/hangar/pipeline.py:Design._components` and
`tools/models/hangar/hangar/rotorcraft/__init__.py:Rotorcraft.model`, plus new
`docs/research/sim-m03/liquid-occupancy-method.md`. Existing flight XML, native
tank points, whole/section mass, aero and geometry remain unchanged.

Use already installed manifold3d Mesh64/Manifold Boolean operations to certify
cells against the delivered closed neutral GLB boundary; ordinary source/mesh
validation remains required. Approximate SDF sign/distance cannot certify a
whole cell. Manifold upstream supports oriented closed mesh inputs and explicit
error status; its topological guarantees do not prove smooth-CAD/OEM accuracy.
No new installation is required for the producer's explicit dependency path.

Frozen versioned JSON seam: `<model>.liquid-v1.json`,
`schema=fsim.liquid-occupancy.v1`, `design`,
`frame=body_origin_forward_right_down_m`, `sources` exact hashes,
`tanks` with native `index`, `capacity_kg`, `density_kg_m3`,
`anchor_body_m`, and `cells` with stable integer `id`, `tank`,
`low_m[3]`, `high_m[3]`, `volume_m3`. Required producer checks include per-tank
capacity/rho volume, certified containment/exclusions, disjoint spatial cells,
finite numerical budget, deterministic bytes and geometric placement uncertainty.
Do not change density to fit capacity. Partial allocations are real clipped
slabs. No liquid produces explicit reason and empty tanks/cells.

The delegate receives the exclusive geometry-method test slot for Python/
Manifold unit fixtures only until handoff; no CMake, native World or all-model
production runs. Root runs no competing native build/test during this interval.
Root retains all current-source complete generation, all-model geometry preflight,
installation/data copying, native sidecar reading, contents capture, scene
integration, full regression and Viewer acceptance ownership.


## Native liquid material scene and actual contents

Root delegates m03_physical_audit new private damage/LiquidSceneV1.h (new types
and additive scene function), Damage.cpp only for the new optional liquid-aware
scene execution, independent new tests/simulation/test_liquid_scene.cpp and
new docs/research/sim-m03/liquid-scene-method.md. Preserve every old layout,
provider vtable and old pure assess/assessSceneV1 behavior. New input recipients
bind existing victim identity/generation, disjoint capacity-cell geometry, native
tank/component/section identities, explicit density/Cd and frozen body-frame
free-surface planes for actual current quantities. Root supplies native pose,
current tank/partial-fill captures and validated producer data separately.

The additive law must traverse liquid chronologically, preserve downstream
contacts, handle empty/full/partial volumes and distributed rod immersion, and
account fragment mass, momentum, deposited relative energy and moving-origin
angular impulse. Cell subdivisions are not tank walls or new penetration work.
Required sources missing/invalid reject explicitly; no fullstop/proxy/density
fallback. Use explicit finite numerical budgets/convergence controls, preserve
legacy nonfluid paths and current actual-skin/rod fixes. Native execution waits
for the separate geometry method slot to release and remains root-owned.

Root exclusively owns World.h private Matter fuel snapshots/cache bindings,
World.cpp reset/separation handoff, WorldDamage.cpp capture/interpolation/material
construction and new WorldLiquidMaterials.cpp plus registrations/native producer
parser fixtures. These disjoint source paths are not delegated. Current native
fuel amounts, detached-piece ownership and original capture epochs are authority;
capacity never substitutes for a measured amount.


## Delivered breakup cap topology

Actual unchanged F16 GLB preflight rejects centre and left-wing section solids.
The raw edge audit finds29 and31 unmatched single-incidence edges respectively;
exact welding and zero-displacement Manifold merge do not repair them. Source
whole-airframe closure is not section-cap closure. Extend the occupancy owner
window to `tools/models/hangar/hangar/shape/breakup.py` only its cap traversal/
triangulation and independent new `tools/models/hangar/tests/test_breakup_caps.py`.
First reproduce a branch/touching-loop edge omission independently; every real
boundary edge must be closed exactly once, orientation preserved, without moved
vertices, removed source skins, convex proxies or waived manifold checks. The
normal model-key invalidation seam may be proposed separately if its current
source digest does not already cover the cap implementation. Actual regenerated
F16 section and all-model closed-boundary gates remain root-owned.


The root state-integration window additionally owns State.h/State.cpp only for
an additive nonvirtual applyLiquidEnergyV1 method and independent new
`tests/simulation/test_liquid_state.cpp`. It consumes actual positive deposited
fuel energy with zero additional physical punctures/hits, using the unchanged
existing exponential energy/pressure ignition hazard. Original apply behavior,
layouts, random-stream ownership and all coefficients remain unchanged. Root
World calls it only for opted-in liquid assessments; initial puncture rows retain
their original application. Subsequent drag energy must not be ignored or
converted into fabricated component hits merely to trigger old ignition code.

## Current liquid integration and independent fixture windows

At unchanged main HEAD e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f, root
owns new WorldLiquidMaterials.h/.cpp, private World captures/cache/reset and
WorldDamage synchronous/causal wiring, runtime CMake, State additive energy and
its test, and test target registration. New material data require complete
native tank/owner/capacity coverage and the same structural origin. Current
quantities are captured at the encounter epoch, never substituted by capacity.
The causal interval uses its start capture and a quasi-static body-frame fill
plane, not an unresolved slosh claim. Required invalid input refuses assessment.

Delegate m03_delivery_audit owns only new test_liquid_materials.cpp for independent
parser/freeze/current-fill/ownership fixtures. Delegate m03_physical_audit owns
only additions to test_liquid_scene.cpp and test_fluid_transit.cpp for explicit
half/double liquid coefficient analytic sensitivity. Neither runs native gates.
Root compiles only after their source handoffs and after the separate geometry
method test slot releases. No public structs or provider vtables change.

Root also owns exact packaging additions in cmake/StageDesigns.cmake and
cmake/RunPackageConsumer.cmake: copy the declared liquid-v1 sidecar beside the
existing damage/flight/model products. A missing required sidecar remains a
native creation refusal, not a legacy fallback.

Root adds WorldWeaponEffectsDiagnostics.cpp captureWeaponImpactDiagnosticsV1 to
its exact liquid integration window: retain actual positive continuation work
with hits=0, without incrementing physical hit counts or totalDamageCount. The
energy-only row must not disappear from referee diagnostics. Independent native
production binding/effect observation remains required before acceptance.

The first normal regenerated F16 model fails strictly because inferred closed
centre/left-wing owner solids intersect; see liquid-occupancy/f16-normal-model.log.
No liquid sidecar was accepted or installed. Root requested read-only geometric
measurement before proposing an allocation correction. Closed section cap solids
are estimated interiors; real liquid cells must still be independently disjoint,
inside their own native owner, clear of equipment, and capacity-conserving at
physical density. No filled-volume or hull predicate has been waived.

## Authoritative unsplit boundary and reduced-scene RED

Native focused build succeeds after strict compiler repairs. First current run
liquid-native-first.log/xml passes pure fluid and material binding, but scene
methods fail 5/15 cases (27/2328 assertions). The original expected analytic
attenuation/downstream work remains unchanged. Delegate m03_physical_audit now
owns the exclusive focused native build/test slot and Damage.cpp liquid-only
correction; root freezes those inputs until its handoff.

Actual cap overlap measurement exposes both positive inferred-owner intersections
and a negative signed inter-owner Boolean volume. Official Manifold documentation
separates guaranteed integer topology from geometric epsilon-validity; NoError
alone cannot certify cap interiors. Extend m03_effects_review source window to
model3d.py only an additive nonrendered GLB extras original unsplit hull accessor
record, liquid.py authoritative containment/global actual-cell exclusion, their
independent tests and the method document. Preserve all existing exterior meshes,
scene nodes and flight/mass products. No synthetic seam welding is an authority.
The actual unsplit hull is required for cell containment; inferred caps constrain
owner placement only. Prior allocated CELL volume, not whole estimated owner
interior, is excluded across owners. Capacity/density, native owner, equipment
exclusion, finite budgets and strict negative/nonfinite Boolean refusal remain.
Geometry execution waits for the native slot handoff; no concurrent gates.

Delegate m03_delivery_audit now owns only new test_liquid_native.cpp for actual
World producer-data/current-content/continuation/reset/child binding acceptance.
It may read relevant native fixtures but has no runtime slot, implementation or
product writes. Root retains CMake, normal generation and execution ownership.
Root also owns new native-liquid-binding.md documentation for exact activation,
capture/unknown semantics, child origin/contents, physical uncertainty and the
separate required verification gates; no synthetic gate substitutes for native.

The focused liquid scene repair now passes16 cases/2361 assertions in
liquid-debug-final.log/xml. Entry schedule and inverse-contact time commit the
represented reached boundary; original raw AABB planes are clipped in long
double so shared-cell intersections match exactly. No geometric gap tolerance
is introduced; an independent real0.2m dry-gap control passes. Every earlier
failed expected value and numerical tolerance remains. Geometry execution slot
is now handed to m03_effects_review for its26-method authoritative-boundary
verification. Broader nonliquid regression and actual production World tests
remain required.

Root extends its exact normal-production integration window to
hangar/engineering.py generate (normal forced model generation and byte-bound
visual/liquid products) and hangar/runtime_products.py explicit runtime asset
copy. Current engineering.generate only forces flight tables/XML and auxiliary
damage, so it cannot certify the newly required normal-model liquid products.
The complete roster must produce them, then verify installed exact bytes. Store
visual products retain their source design directory while their flight XML
retains the existing stores/<design> runtime directory.

## Native section ownership instead of invalid cap interiors

The second normal regenerated F16 run fails strictly at a negative
Cell/interior intersection volume (f16-authoritative-model.log). The authoritative
unsplit exterior is present, but its intersection with an inferred cap solid
cannot repair an epsilon-invalid cap input. No current liquid product is admitted.
Source inspection identifies the existing actual section/dry-mass assignment:
damage._sections_of/_masses use breakup.classify on material points. That frozen
classifier, rather than inferred drawing-cap volume, defines native ownership.

Extend the occupancy owner's window to a conservative interval certificate for
an entire AABB under the existing exact classifier and its precedence. Use the
original valid unsplit hull for physical containment, plus equipment exclusions;
no cap solid participates in material Booleans. Unknown/mixed owner cells refine
rather than borrow another section. All-points interval bounds must account for
roundoff and clipped/mirrored planform expressions; corner/centroid sampling is
insufficient. A singleton possible-owner set admits a box; real cells remain
strictly disjoint and each native tank conserves complete capacity at density.
Independent thin-region/cross-owner/owner-shortage RED controls precede repair.
This changes an incorrect inferred-cap predicate to the already established
native mass/section contract, not a reduced acceptance population or guessed
new tank owner. Visual caps remain an explicitly inferred drawing interior.

Normal-production changes now force normal model writers for all88 designs,
include their GLB/manifest and all35 liquid sidecars in source/install byte
manifests, retain existing store flight-vs-visual namespaces, and refuse reuse
without a completed normal-model receipt. Existing model checks participate in
the generation gate. Root requested read-only independent production-path review
from m03_physical_audit. Runtime copier and package verifier prune excluded names
before recursive descent; no forbidden tree is a source or package input.

Independent production review found a real stale-section XML hole. Extend root's
exact write window to pieces.py:write and new test_piece_publication.py. The
writer retires only its known PARTS section XML when current piece_xml returns
None, verifies its resolved owned namespace and rejects redirected paths; it
preserves neighboring/user files and directories. engineering.generate derives
and compares the exact current piece contents instead of trusting existence.
Backend-resume closure is also under correction; an old geometry output cannot
be resumed merely because Python source hashes still match.

Root owns only the additive required Manifold3D package entry in
bootstrap/dependencies.lock.json and its Python import probe in bootstrap/project.py.
Official PyPI3.5.4 metadata retained in liquid-environment verifies Python>=3.9,
cp313 Windows x64 wheel, SHA25676975aa4ebd18dcb0481e43dac3ed70d78066e9ad047e73a5f7dbc0df1553cf4,
and NumPy>=1.26.0b1 forPython>=3.12; the existing locked2.4.6 satisfies it.
No toolchain/dependency version changed, no installer executed, and no general
bootstrap ownership is transferred. Clean prerequisites must include this actual
new production dependency rather than relying on an unexplained existing binary.

Delegate m03_physical_audit owns only new method_backends.py and its independent
new test for actual selected installed-package/mesher/Python/compiler backend
byte inventories. Metadata versions alone are insufficient, and the current
D:/ENV dependency overlay lacks several distribution-info directories. Snapshot
must use actual module origins, bounded full selected trees/dependency roots and
explicit OS/runtime/lazy-loading limits, prune forbidden paths before metadata,
and reject redirected roots. Root wires this receipt into forced generation and
resume. No native or model input writes are delegated.

Fresh native method receipt liquid-native-methods-green.log/xml passes3/3CTest,
with33Catch cases/3162 assertions (fluid8/329, scene+state16/2361, binding9/472).
New actualWorldfixture compiles strictly; it has not run without its required
real liquid product. New piece-publication negative controls pass2/2.

The unchanged300s actual F16 occupancy attempt failed screening budget. A bounded
20s non-admission cost probe (f16-baseline-cost.json) found17.78s hull intersection/
difference work versus0.753s owner intervals; centre104cells complete, left-wing
3382screened/1653pending/0accepted. Prior-cell CSG construction cost0.0109s, so it
was not the measured main bottleneck. The producer owner will refine indeterminate
owner boxes before geometry and prioritize certified candidates; actual hull/
equipment/capacity/disjoint checks and300s budget remain unchanged. A distinct
singleton native-owner certificate proves cross-owner partition, but final
independent global positive-AABB-overlap refusal remains mandatory.

Root independently fetched and SHA-verified the official1,036,021-byte3.5.4 wheel
under D:/ENV/cache/fsim-m03 and proved its actual manifold.pyd byte-identical to
the current runtime (binarySHA296d7bcd16ee6ad5596d29a510925994cf933987753870c5140d351009d20d12).
No environment was replaced or installed. Exact receipt:
liquid-environment/manifold-wheel-binding.json.

Root's normal-entrypoint window includes only fsim.ps1 default Matplotlib/XDG
cache roots under D:/ENV and disabled bytecode writes. Campaign sets these roots
explicitly. The new method closure imports plotting backends, so a default
profile cache must not be created on C: during normal production/verification.
Existing launcher arguments, preset/toolchain selection and user overrides
remain; no UI or simulation behavior changes.

Corrected the earlier README claim that the existing D:/ENV Python overlay had
a shipped complete-byte manifest: current inspection found none and missing
NumPy distribution metadata. The new actual-origin selected-backend inventory,
not that unsupported older sentence, is the required resume/final proof. Wheel
provenance, installed actual method bytes and installed SDK products remain
separate explicit checks. Final report now reruns complete liquid admission
and requires every generated normal-model row's current backend snapshot.

Actual backend closure now passes all12 synthetic negative controls and a live
snapshot+verify(true):3649files/206195565bytes, SHAa2ae8d22f464159f5142b19424369b68ec228e3a09c04649ffc0f4485f6431f1,
first snapshot31.438s (this clock excludes the subsequent verification). Raw
method-backends-first.log and method-backends-actual-first.log/json. Root has
wired the receipt into each generation row, resume and final88-row comparison.
The code uses only actual-container metadata; foreign global site-packages
metadata cannot substitute for a missing overlay record.

A second optimization attempt failed300.210s in centre after103767screened and
9240microscopic cells totaling0.000317925m3. It exposed certified-priority queue
starvation rather than capacity shortage. The bounded independent sphere fixture
first fails at4096screened (fairness-red.log), then passes with depth fairness
and certified priority only within a depth (fairness-green.log,32methods total).
All original geometric/capacity predicates remain. A new actual F16 read-only
probe is running under the same300s budget and frozen sources. No liquid product
or actualWorld native test is yet admitted.

## Physical capacity boundary correction

Read-only independent review identifies another real defect in the new liquid
path: initial capacity occupancy is currently admitted as a bladder puncture.
A point or whole rod born strictly inside has crossed no wall; drag work can
start immediately but hole area/hits must remain zero until a true capacity
UNION exterior-face crossing. Capacity-cell cuts cannot act as walls. No
pressure-wave/bladder-failure solver justifies a birth hole.

Delegate m03_physical_audit owns only Damage.cpp liquid puncture admission,
independent new point/rod/shared-face/partial-neighbor/disconnected-union RED
fixtures in test_liquid_scene.cpp and private header/method wording. It first
captures RED under an explicitly handed-over native slot, then implements exact
exposed AABB face patches with same-TK neighbor coverage removed, retaining the
one-TK continuation budget and all energy/impulse/solid/skin behavior. Lazy
per-assessment cached faces avoid a full65k-cell surface rebuild. No geometric
gap tolerance, fake hit, hole or future event is authorized.

Root retains test_liquid_native.cpp and will correct its unexecuted inside-cell
one-hit premise only after the physical RED proof; later energy-only diagnostic
rows are already supported without count mutation. Root/geometry/native slots
remain serial. Delivery agent reviews production/admission integration read-only.
Backend ancestry caching is only a proposed performance change; its measured
cost, identical snapshot bytes and new race controls are required before use.

Independent production/admission review found three concrete gaps. Root/assigned
validator fixes now require the independently derived complete GLB/manifest/
applicable-liquid roster in both hashed generation and installed maps; matching
omissions cannot pass. Reuse independently derives the same mandatory identities.
Retired source XML must also retire only its exact staged generated counterpart;
Python staging correction is saved, matching CMake and controls wait for native
slot release. No recursive directory removal is used.

Root owns new backend_epoch.py/test_backend_epoch.py and campaign/report wiring:
campaign records selected interpreter, dependency overlay, Release mesher and
D:/ENV caches. Production receives those explicit paths. Final report captures a
fresh backend snapshot in a bounded child with a sanitized parent environment,
not the report shell's imported packages/Python paths. Source identity is checked
inside that child. Sanitized-parent/error/timeout/redirect controls are authored
but not yet run while the physical native slot is active.

Physical bladder RED is real:6cases139assertions34failures across5 cases, ordinary
entry/true-face control already passes. Root corrected only its unexecuted native
inside-cell premise to zero wall hits/holes, retaining positive wet work and
reaction. Physical owner additionally tests zero-net-force rotating material:
nonzero dissipated energy or torque is a real fluid encounter even if net
momentum cancels; it must not create a bladder wall hole. This changes only new
liquid event admission after its independent RED, with actual load law unchanged.

Bladder focused final strict build and three current native gates pass41cases/
3344assertions: fluid8/329, scene24/2543, binding9/472. Exact evidence
liquid-bladder-final.log/xml; source hashes and distinctions are in the owned
liquid-scene-method.md. Actual World binding, wider regression and final visual
acceptance have not run this epoch.

The actual localized F16 geometry/capacity probe passes141.758s with4tanks/
14134cells, within unchanged300s. Independent actual-material AABB audit then
finds positive actuator overlaps: flaperon left/right5.55940033e-5/5.51273282e-5m3,
leading-edge actuators0.00114406510/0.00113980239m3. No current-engine/crew AABB
overlap was found. This epoch is NOT equipment-clearance admitted.

Extend the occupancy owner window narrowly to current real damage nonfuel solid
component AABB exclusions, canonical origin/components source binding (avoids
whole-damage annotation hash cycles), independent shifted-frame/control-overlap
RED and collector source checks. Capacities, density, native owner, numerical
budgets and original hull predicates remain. Rotor products still have no fake
damage recipient. Root awaits the corrected actual audit before normal delivery.

Current independent product-admission20methods, sanitized-backend-environment
4methods, and both-route staged-piece-retirement1method pass (25 total) in their
respective admission-products-current.log/backend-environment-first.log/
runtime-piece-epoch-first.log. They prove source/install namespace and freshness
rules; they are not physical or all-model execution evidence.

## Scope clarification requested by the user

The user asks whether M03 includes fuel tanks. Original allocation row49 names
seeker/autopilot/fuze/lethality fidelity; tank perforation/leak/fire are existing
victim damage consequences. A complete all-aircraft spatial tank-occupancy/free-
surface implementation is ROOT'S chosen repair for the perfect-absorber defect,
not a separately enumerated original task or a user-imposed requirement. Do not
present this implementation choice as the user's formal scope. It does not
transfer/reopen the accepted M02 flight fuel-capacity/mass/transfer authority.
Keep the complete M03 objective and its required native/public/fleet/visual gates
in view rather than treating new oil geometry as the entire task.

## Shared physical scene reach correctness

Independent source/actual-card analysis proves the old150m burst-centred cull is
not a valid broad phase for the current1s point/rod field. Exact implemented
free-air law gives AIM120C D(1)=210.8969m and R77M228.4766m at declared air density
1.225/Cd1.3; gravity perturbation over1s is bounded4.903325m. Both current filters
(WorldDamage.cpp victim capture and causal continuation) can omit reachable
geometry. Documenting a software cull does not make it equipment lethality or a
user acceptance waiver. No lifetime expansion is necessary for this correction.

Delegate m03_physical_audit owns only new test_fragment_reach_native.cpp for
synchronous/shared and actual released/retained source chronological regression,
original external-adapter range controls and negative nonintersection/late cases.
Root owns WorldDamage.cpp conditional legacy-only cull, nearest diagnostic target
initialization and CMake registration after RED. Existing scene kernel's current
chord/geometry broad phase remains physical authority. Normal F16 model generation
currently holds execution; no competing native build/test is authorized yet.

Normal fsim.ps1 hangar f16c model now completes successfully in246.1s with the
actual required liquid sidecar, full structural/part/size checks and no reduced
visual branch. Source sidecar4,989,000bytes, normal log
liquid-occupancy/f16-control-clear-normal-model.log. First actualWorld native gate
fails3/3 cases on a TEST premise: it required a closed contact mesh for analytic
lifting-surface quads (and internal components), though these retain actual
native analytic geometry. Raw liquid-world-first.log/xml retained; this is not
a production physical pass. Root fixture now checks real quad triangles and
conservative native component boxes, retains closed body mesh requirements and
ignores explicitly artificial cut faces; exact safe AABB pruning avoids testing
all distant triangle sets. No outcome/energy/impulse tolerance is changed.

Root's existing Damage.h window corrects only the stale triangleNeighbors
comment: artificial ownership cuts never constitute authored physical skin,
matching the already verified authored-skin production correction.

Current actualWorld native second run fails49 assertions across3cases in34s;
liquid-world-second.log/xml retained. A concrete integration bug: root mistook
`totalDamageCount` (observed RECORD count, >=retained damageCount) for wall hit
count, causing energy-only frames to be rejected by the unchanged validator and
leaving stale birth snapshots. Producer now counts actual positive-work records;
physical hitCount/hole area remain zero. No diagnostic validator is relaxed.

Child physical reset also captures a real native zero fuel point mass before
lazy damage-state recreation. Root capture now reads the EXISTING enumerated
native inertia pointmass0 value, proving each nonnegative owned tank zero only
when the aggregate is exactly measured zero. Positive/unknown aggregate cannot
be provisionally split. The established damage(child)==nullptr after reset is
preserved. No authored capacity replaces missing current data.

Source review confirms a new reduced-method physical gap: liquidDerivative
applies ambient-air drag over fully submerged material before adding liquid.
Delegate physical owner pauses reach-regression authoring (no file yet) and
prepares independent full-wet point/half-wet rod/dry/zero-content RED controls;
production remains unchanged until RED. Root retains World/observer/native
fixture ownership. New fixture clearance at skip4 also remains unresolved;
current failure must not be represented as native binding closure.

Reach RED on unchanged production is captured: fragment-reach-red.log/xml,
3cases2353assertions20failures across shared synchronous/causal cases; original
external-adapter case passes. All prescribed source/native forecast and negative
geometry checks execute without resampling. Root corrects only shared finite
scene dispatch: keep150m for !sharedScene, remove original-burst distance from
causal continuation, initialize nearest diagnostic candidate to infinity and
reject nonfinite distance. Actual existing material-path/geometry broad phase,
legacy external behavior, force laws and1s window remain unchanged. GREEN and
wider regression are pending; documentation describes the corrected semantics,
not a completed verification claim.

## Resumed native reset repair window

Main remains `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`. Root owns the
exclusive native slot and extends the exact source allowlist to
`src/simulation/flight/JsbsimModel.cpp:saveInitialProperties` only for restoring
existing loaded point-mass weight/location properties on physical reset, plus
`tests/simulation/test_liquid_native.cpp` for direct native reset assertions.
Existing World capture/observer, CMake and this journal windows remain owned by
root. Independent agents perform read-only reviews; no competing build writes.

Current `liquid-world-third.log/xml`: 3 cases, 2 passed, 1 failed, 1,554,031
assertions, 77.02 seconds. Full/partial/empty native captures and parent reset
pass; detached reset refuses missing actual per-tank contents. Independent
review identifies the native cause: `saveInitialProperties` excludes tied
point-mass nodes and FGMassBalance::InitModel does not restore their loaded
weights/locations. The transferred positive child fuel point mass survives
reset, so the measured-zero capture fallback correctly refuses it. Restore
loaded properties rather than substituting zero or splitting an unknown
positive aggregate. Existing immediate damage(child)==nullptr stays required.

Current `fragment-reach-after-cull.log/xml`: 3 cases and 2,393 assertions pass
in 73.38 seconds. Full current regression and final production campaign remain
required; these focused results do not establish package completion.

Direct native point-mass RED (`liquid-reset-pointmass-red.log`) confirms
462.5998944992164752 and 462.96676165535302516 lb survive the two child resets.
Restoring only existing loaded point-mass weights/XYZ locations fixes the
original child encounter: `liquid-reset-pointmass-green.log`, one case and
1,355,977 assertions pass. No current quantity is fabricated; the generated
piece's actual loaded fuel weight is zero. Full three-case rerun is pending.

## Tangential rod material partition repair window

Independent effects review finds closed adjacent capacity cells can double
charge rod material lying along their common face. Existing crossing-rod tests
do not cover that geometry. Delegate `m03_physical_audit` owns a complete narrow
repair across `Damage.cpp:liquidDerivative` interval/material admission,
`tests/simulation/test_liquid_scene.cpp` for exact seam/split/order/negative
controls, and `docs/research/sim-m03/liquid-scene-method.md` for interface policy.
Capture RED before production edits; preserve coefficients, integration budgets,
original scenarios and tolerances. No global capacity overlap waiver. Root
releases the serial native build/test slot for this focused repair and freezes
all native build inputs until the delegate's handoff. Delegate must return exact
source hashes and raw bounded focused verification; no GUI use authorized.

The same complete seam window includes exact point chord work attribution and
ownership-aware horizons: independent review's four-cell, two-medium analytic
counterexample failed before the horizon correction. Current delegate handoff
`liquid-owner-horizon-final.log/xml`: fluid8/329, scene32/3714, binding9/472,
49 cases/4,515 assertions pass. Independent current-patch review is pending.
Root reacquires the exclusive native slot; delegate native inputs are frozen.

## Retained-store reset compatibility window

Root additionally owns only `StoreFlightBinding.h/.cpp:invalidateMasses`,
`World.cpp:resetVehicle` station mass/cache reconciliation,
`tests/simulation/test_mass_change.cpp` retained inventory/loaded mass reset
controls, and `effects_population.py:SOURCE_PATHS` for these physical inputs.
The flight model now restores loaded authored point masses. The Host inventory
is intentionally retained by Vehicle reset, so a matching revision must not
leave its actual loaded station weight at the authored empty value. Capture
RED first. Preserve inventory, actor permissions, task/authority and optional
custom-model mass support. Reapply current weights before an empty native
mass transaction so derived mass/CG/J are current without an artificial reset
impulse/CG shift; verify both airborne and ground behavior against a fresh
same-loadout native model. This is compatibility repair, not new store policy.

Current evidence-admission corpus: 119 tests pass in9.834s
(`current-admission-corpus.log`). Current independent coverage retains53 stores,
35 airframes,24 AAM and26 explosives (`current-coverage.json`). These checks
do not establish full production, native or presentation acceptance.

Retained-store RED: `store-reset-red.log`,1case448assertions35failures; an
unchanged loaded station loses500.00841063530236852lb/226.8kg on reset. Current
`store-reset-green.log` passes all448 assertions in the same airborne/nonzero-
rate and ground scenarios, including immediate/next-step pose/velocity. Delivery
review accepts the exact source patch; no generic inventory/authority contract
changed. Focused combined current integration (`native-material-integration-current`)
passes all3 liquid-native cases and all3 fragment-reach cases. The26-explosive
fixture fails immediately on AIM120C because shared material assessment returns
invalid; CJ10 terminal case passes. Retain the unchanged hit/hole requirement
and authored source; this is not an equipment calibration waiver.

Delegate m03_physical_audit owns a complete follow-up only in Damage.cpp liquid
integration/admission internals, liquid scene tests/method doc and a new bounded
real-frame regression under test_liquid_scene.cpp if needed. Diagnose the actual
authored AIM120C/F16 refusal before any force-law change. Temporary diagnostic
instrumentation may identify the refusal but must be removed from final source.
Root transfers the exclusive native slot and freezes all build/model inputs.
The actual26-model fixture/scenario/source/thresholds remain unchanged. Any
needed additional exact file/symbol must be recorded before implementation.

Read-only current-byte delivery review47 identities passes in
`source-review-delivery-resumed.json`; this is static scope only. Root records
an independent documentation repair window for m03_delivery_audit:
`docs/research/sim-m03/baseline/sensing-reference-map.md` and
`docs/research/sim-m03/baseline/model-capability-atlas.md` only, adding explicit
frozen2026-11-02 baseline notices and correcting relative repository-source
links. Preserve all historical results, numeric/hash fields and companion JSON
bytes. These two Markdown files are not inputs to the active focused native
target; all native/model source inputs remain frozen for the physical owner.
Current journal is excluded from static final review until its final freeze.

The documentation owner additionally owns only
`docs/sdk/equipment-model-names.md:Verification scope` to qualify the historical
naming-migration AIM9M countermeasure failure as that dated review's evidence,
not an assertion that its exact source failure persists after later repairs.
No model identities, schema/ABI contracts, historical outcome or current
all-population acceptance requirement may be changed. This Markdown file is
not an input to the active authored-effects native target.

Root extends the physical owner's diagnostic-only allowlist to
`tests/simulation/test_weapon_effects_population.cpp` for a TEMPORARY separately
named duplicate exact-card case isolating MAGIC2's later native refusal. The
original26 loop, authored data and all predicates stay unchanged. Remove the
diagnostic duplicate and verify the original source bytes before final full26
verification. Diagnostic single-card runs never count as the complete roster.

Root extends the documentation owner's exact allowlist to
`docs/research/sim-m03/retained-files.json`, and new immutable copies at
`artifacts/sim-m03/research/baseline-originals/sensing-reference-map.md` and
`artifacts/sim-m03/research/baseline-originals/model-capability-atlas.md`.
Read only the index's explicitly named project original files, verify their
recorded old hashes before archiving, retain original hash/byte identities and
record maintained-view changes/current retained hashes. Do not silently replace
original evidence identity. Additionally correct ONLY the TP1078 NTRS locator
in `docs/sdk/missile-public-source-search.md` and
`docs/superpowers/plans/2026-10-29-missile-calibration.md`: TP1078 is19780004098;
19770021141 belongs to CR2836 and its correctly labelled entries stay unchanged.
No calibration data, accepted M02 scope or outcome changes are authorized.

Documentation owner may additionally correct ONLY the native fixture name in
`docs/sdk/weapon-countermeasure-receiver-validation.md` to match the actual
`Every configured AAM receives an actual native countermeasure in its physical detector`
test/collector identifier. No native inputs, predicates or results may change.

The physical owner's temporary exact-card observational duplicate may isolate
any of the CURRENT26 authored explosive cards (including now R27ER), selected
only by a validated current-card identifier. It must use the same actual source,
scenario and original predicates, leave the original26 loop untouched, and be
removed before final source verification (original fileSHA fb9cb5df6ccef36399d3ad0e73850298524b03c5f465970fee733eae059070a7).
This grant prevents repeated full-prefix diagnostics; single-card observations
are never complete-population acceptance. Current full gate passed17cards before
R27ER's material refusal, not all26. Root has not loosened a native predicate.

Independent shared review identifies a real piston thermal source defect:
the native GetEGT value is reset-only, while getExhaustGasTemp_degF and its
actual egt-degf property expose the evolving exhaust temperature. Root records
the NEXT thermal repair allowlist `JsbsimModel.cpp:nativeThermalSnapshotV1`
and `tests/simulation/test_variant_actuation_native.cpp:[native-thermal]`.
No writes to these active native build inputs occur before the physical owner's
slot release. Require actual powered heat/cool/reset RED/GREEN against the live
native property, not against the same stale getter or artificial EGT writes.

Next root WorldDamage.cpp window additionally repairs only detonate's retained
airframes_ iterator across applyDamage/separation: keep shared Airframe ownership
before a callback can insert a new piece type and rehash the map. The physics,
scenario and outcome predicates stay unchanged. No active native input is
modified before the physical owner's slot handoff.

## Additive replay capture-binding preparation

Independent review proves a current same-time unrelated .fsrec/.wdiag pair can
be joined without capture identity. Delegate m03_effects_review owns the complete
capture-binding feature. Its FIRST disjoint write window permits ONLY NEW,
currently unreferenced files:
`src/foundation/transport/WeaponDiagnosticCaptureBindingV1.h`,
`src/foundation/transport/WeaponDiagnosticCaptureBindingV1.cpp`, and
`src/foundation/transport/third_party/picosha2/{picosha2.h,LICENSE,provenance.json}`.
Transport's CMake source list is explicit; these files are not active native
build/test inputs. Do not edit existing headers/sources, tests, CMake or perform
any native/GUI execution until root grants the subsequent integration window.
Vendor unchanged official PicoSHA2 v1.0.1 commit161cb3fc4170fa7a3eca9e582cebd27cc4d1fe29
and preserve MIT attribution/pinned byte hashes. This first window adds no
runtime dependency, format/ABI/layout change or acceptance claim.

The complete later feature must bind actual closed fsrec/wdiag bytes with
checked SHA256/lengths, invalidate completion before new capture truncation,
finalize atomically only after both checked stream closes, and reject a foreign
same-time companion. Preserve fsrecV8/shared frames/WorldIPC and legacy flight
replay; unbound diagnostics are unavailable. Recording close/error paths and
World/Viewer integration receive separate exact ownership after native handoff.

Physical owner's complete liquid primitive window now additionally includes
`FluidTransitV1.h:rodLoads` and `tests/simulation/test_fluid_transit.cpp`.
Actual R27ER source37 exposes a positive immersed interval whose endpoints are
adjacent doubles (.84047982744282568,.84047982744282579); the quadrature has no
representable interior sample and refuses. Capture independent uniform/varying
velocity RED, then use an endpoint integral with a conservative explicit
affine-velocity Lipschitz truncation bound, included in the unchanged global
quadrature error budget. Retain positive tiny work rather than dropping or
epsilon-merging the interval. No coefficients/tolerances/count budgets change.
The prior primitive owner has released all source/review slots; no concurrent
writes or native execution are authorized outside the physical owner's slot.

Physical final handoff: original unchanged26-card native gate passes224,102
assertions in182.0563718s (unchanged300s bound), with actual Hit and positive
hole for every card. Focused fluid10/375,scene38/3841,binding9/472 pass57cases/
4,688 assertions. `liquid-production-authored-final.log` and
`liquid-material-methods-final.log/xml` retain exact current evidence; method
doc records all REDs and model/control invariants. Temporary fixtures/diagnostics
are removed and original populationfileSHA fb9cb5df6ccef36399d3ad0e73850298524b03c5f465970fee733eae059070a7
is restored. This is focused acceptance, not full M03 completion.

Root NOW reacquires the serial native slot and activates the previously recorded
piston thermal and WorldDamage iterator windows. Remaining3 full-World/reach/
population CTests are transferred for a merged rerun after queued integration,
not skipped. Replay owner may repair ONLY unreferenced new binding helper bytes;
no existing foundation/World/Viewer/CMake/test edits before its next handoff.

Thermal regression setup corrections are retained: first compile used a const
write handle; first run used wrong case/hyphen native property spellings. These
are fixture setup errors, not production RED. Valid native properties are
`propulsion/engine/egt-degF` and `propulsion/magneto_cmd`. Valid-fixture RED
reports25 failures/180assertions, including593-696K live snapshot mismatch.
The live-getter repair makes every heat/cool/native-property comparison pass;
one NEW reset premise assumed instant altitude-ambient equilibrium. Source
ResetToIC initializes exhaust from its loaded inlet before altitude IC, so
the regression now compares an independent fresh stopped native plant at the
same IC, preserving that native transient. The1e-10K live/fresh equality bound,
actual heat/cool predicates and production plant are unchanged.

Root thermal current native gate passes2cases/184assertions
(`piston-thermal-final.log`) against actual powered/cooling/reset piston property,
fresh stopped native plant, retained rocket-unavailable and eight-engine B52
controls. WorldDamage retains shared Airframe before applyDamage and uses it
after piece map insertion; no event/physics predicate changes.

Independent final kernel review requests an ACTUAL RED for a new two-tank
right-limit corner: a point immediately departs A tangentially under gravity,
crosses a real dry gap and later enters B. A must receive zero early work even
when B keeps outer broadphase potential true. Root transfers the native slot
back to the physical owner for the exact already-owned scene test/kernel seam.
Do not change production before the counterexample proves a defect. If it is
real, fix temporal ownership without epsilon/gap merging or tolerance/budget
changes; re-run focused methods and original26 native gate on final bytes.
Replay helper preparation remains unreferenced/disjoint; existing integration
and full-World native3 still wait for the next explicit slot handoff.

Replay owner's disjoint NEW-file preparation window additionally permits
`BoundWeaponDiagnosticsReplayV1` in the already granted new binding header/cpp.
It wraps the unchanged raw reader through its own private state, validates the
closed file pair once, then checks ordinary input/marker changes around each
read using bounded metadata/marker receipts. Invalidated capture becomes
unavailable without touching caller output. No members/signatures of an old
reader class, CMake, tests, World or Viewer are changed in this preparation.
Existing-source integration and all runtime negative controls still wait for
the serial native slot handoff. This additional new code is not compiled yet.

## Normal model population and disjoint replay integration window

Main remains e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f. Final right-limit
kernel passes58cases/4772assertions and ALL26 original cards/224102assertions
in181.1565144s under the unchanged300s bound. Source/fixture bytes and raw
failures are bound in the method doc; the restored original native stderr is
byte-identical to the preceding complete run. This is not final program
acceptance; native3/broad/installed/fullscreen/source-export gates remain.

Root owns execution for model-only preflight through the35 actual normal
Hangar CLI model writers, using the explicit current-coverage.json roster,
eight disjoint workers/one native thread each and bounded900s child processes.
Diagnostic runner `build/architecture-reviews/sim-m03-production-20261103/normal-model-population.py`
binds outcomes/raw logs/current GLB,manifest,liquid products and selected method
bytes. This does not replace the mandatory forced88-design production campaign.
No configure/build/test/Viewer may run while model products are being written.

The replay owner's SECOND complete integration write window may proceed
concurrently ONLY in these disjoint non-model inputs:
`src/foundation/transport/Recording.h/.cpp` checked nonvirtual finish/close;
`WeaponDiagnosticsV1.h/.cpp` diagnostic recorder checked completion only;
the existing new CaptureBindingV1 header/cpp and pinned third-party provenance;
`src/foundation/transport/CMakeLists.txt` source/private include/license install;
`src/simulation/runtime/world/World.cpp` capture startup/destruction only;
`World.h` private capture bookkeeping only if needed, never installed API layout;
`WorldWeaponDiagnostics.cpp` capture creation/failure lifecycle only;
`src/applications/viewer/main.cpp` bound companion replay selection only;
`tests/contracts/test_weapon_diagnostics.cpp`, new
`tests/contracts/test_weapon_diagnostic_binding.cpp`,
`tests/simulation/test_weapon_effects_diagnostics.cpp` actual paired World fixture,
`tests/visualization/test_weapon_diagnostics_ui.cpp`, `tests/CMakeLists.txt`, and
`docs/sdk/weapon-diagnostics-v1.md` only for this capture binding feature.
NO native configure/build/test/GUI execution until root ends model preflight
and explicitly transfers the serial slot. No model/asset/producer/.py edits.
Preserve fsrecV8/shared-frame/publicSDK/provider layouts and old flight replay.
Record real closed-writer failures, actual pair identity and unavailable legacy
diagnostics; no substitute simulation, new samples or native event retuning.
Use actual existing install components for MIT notice in SDK/Viewer products.

During the35-writer phase, physical reviewer may run ONE isolated pure geometry
counterexample under `D:/ENV/tmp` using existing NumPy/Manifold and current
read-only breakup.py. This is not a CMake/nativeSDK/World/model-writer gate:
no configure/compile/staging/Viewer, no maintained source/model/cache changes,
no full aircraft generation. One thread and the existing eight generation
workers stay within the16-thread reservation. Preserve raw synthetic RED/input
and source hash. Any actual model replay or producer repair still waits until
the35 current writers join, so their sealed method bytes remain unchanged.

Additional isolated read-only geometry diagnostic: physical reviewer may execute
ONE RQ4B volume-only query (existing NumPy/Manifold,1thread,60s bound) reading
current `aircraft/rq4b/{rq4b.toml,rq4b.xml,lrn1015.dat,rq4b.glb,rq4b.damage.json}`
and its explicit existing calibration inputs/current Hangar imports. No pack,
airframe/meshkit build, pipeline/model writer, configure/compile/SDK/GUI or
project/cache writes. Output/raw hashes only in current evidence/DENVtmp;
hash input bytes before/after. Compare native required4.905m3 per wing against
actual hull-intersected necessary-owner volume before/after exclusions. This
distinguishes an implementation bound error from an actual admitted geometry/
capacity inconsistency without borrowing fuel identities or scaling density.

## First complete model preflight outcome and next exclusive gates

All35 normal CLI writers joined after2683.3343131s; selected producer method
hashes unchanged.23pass/12fail: caps(E3G,EA18G,F22A), RQ4B owner-space conflict,
U2S geometry screening budget, and900s DIAGNOSTIC-parent timeouts(C17A,F35A,
J20A,MiG29A,Rafale,Su27S,Su57). This is a complete diagnostic population,
not accepted production. Native plant/capacity/density/geometry thresholds were
not changed to turn these into passes. RQ4 volume-only query independently
proves4.905m3 required vs4.724352/4.729007 before exclusions; it proves the
chosen skin-point-owner restriction cannot fit, not that a disclosed inferred
whole-tank root vessel is physically impossible.

Replay owner now receives the EXCLUSIVE serial configure/build/test slot for
the already recorded capture-binding integration, first its required native
negative controls and actual pair lifecycle, then current GUI-free presentation
and bounded Viewer build. No other native configure/build/test/model/Viewer
execution until handoff. Producer and model asset bytes remain frozen during
these gates. Root does not modify shared native/CMake paths concurrently.

Physical reviewer may prepare ONLY NEW diagnostic/profiler scripts under
D:/ENV/tmp/current evidence and inspect current Python/source/data. No actual
aircraft replay, producer repair or model writes until replay gate handoff.
Root will then grant exact caps/RQ4/remaining timing repair windows. Preserve
the synthetic32/32 cap diagonal RED and actual negative capacity proof.

Capture binding complete scoped handoff: both presets native gates pass,
Release17/629contract +8/135GUI-freeUI +3/539actualWorld, Headless17/629+
3/539. Viewer strict build and independent17-file source review pass; original
same-time/startup failure witnesses and UCRT symlink-fixture correction remain
retained. Scoped receipt `replay-binding/acceptance.json` binds the final peer
review and licence-only install proof. Full program/SDK install/fullscreen
acceptance is NOT inferred. Root reacquires native slot for current broad
material/engagement/reset/regression execution.

Root now builds current fsim_tests/native material/reach/population/engagement/
sensor/proximity targets with all current SDK inputs frozen. Physical reviewer
receives next SOURCE write window only in
`tools/models/hangar/hangar/shape/breakup.py:caps`,
`tools/models/hangar/tests/test_breakup_caps.py`, new DENV/evidence probe scripts,
and `docs/research/sim-m03/liquid-occupancy-method.md` cap-method clarification.
Fix the actual independently proven32-case cap-only diagonal collision while
preserving every original skin triangle/edge, strict winding/incidence checks
and original-hull volume authority. Preserve original function bytes/probe
before-after outcomes. No model or geometry process until root's current C++
build finishes; no source asset/product publication during native staging.
After root explicit release, actual EA18G/E3G/F22A observational extraction may
write ONLY DENV scratch files, retain exact rejected section/edge incidence,
then verify repaired caps without new simulation or compiled-binary changes.

The requested root native build reached all target links at unchanged main
HEAD e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f. Root now runs the unchanged
three native material/reach/population CTest gates with their 300 s bound.
The current moving-origin fixture produced a retained RED: 1 case, 164
assertions, four failed scene rate bounds. Its birth component was fuel,
although its analytic premise explicitly requires instantaneous solid
absorption. Root's exclusive test-only repair window is
`tests/simulation/test_damage.cpp:nativeMomentApplication`: select the actual
crew solid at birth, retaining all impulse, rate/moment bounds, legacy controls,
cadence and turning cases. Real finite-time fluid transport is independently
verified by the unchanged fluid/native material fixtures; no production
coefficient or tolerance changes are authorized by this fixture correction.

The physical owner may now execute the already approved pure cap suite and
three actual EA18G/E3G/F22A observational probes, sequentially, one CPU/thread,
with writes confined to D:/ENV scratch. This does not publish model products
or change SDK inputs/binaries during root's native tests. The exact reviewed
caps replacement d2f196f3216fa7fccf5efb26bf6e97cacb93358e4707824634d45d5a28ca0bc3
may use guarded atomic replacement after a mapped-source Windows truncation
refusal; preserve the original 38ed550b0d0e8017b688c101f9664080f5b00cf8ad10aa2b620ed59b4de46c21
bytes and failure evidence. No permission, attribute or security changes.

## RQ4 inferred root-vessel source window

Owner `/root/m03_delivery_audit`, on main HEAD
`e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`, implements the reviewed explicit
inferred-vessel proposal. Exact source/symbol allowlist:

- NEW `aircraft/rq4b/rq4b.liquid-geometry-v1.json`: bounded declared TK0/1
  original-wing root-strip eligibility and sealed-module assumptions only.
- NEW `tools/models/hangar/hangar/liquid_vessels.py`: strict optional declaration,
  original source-wing eligibility/certification and centroid discrepancy methods.
- `tools/models/hangar/hangar/liquid.py`: per-tank candidate/actual-slab
  eligibility, independent whole-hull/exclusion/nonoverlap, typed receipts.
- `tools/models/hangar/hangar/engineering.py:generation_inputs`: optional
  declaration presence/hash for each recursively visited design only.
- NEW `tools/models/hangar/tests/test_liquid_vessels.py` and
  `test_liquid_vessel_inputs.py`: independent declaration/geometry/assignment
  and normal-generation input epoch controls, authored before implementation.
- `tools/validation/weapon_quality/liquid_population.py`: independent typed
  eligibility/declaration/source-role/centroid checks; all old numeric/native
  ownership and all35 roster checks retained.
- NEW `tools/validation/weapon_quality/test_liquid_vessel_policy.py`:
  independent collector omission/stale/forged eligibility controls.
- `src/simulation/runtime/world/WorldLiquidMaterials.cpp`: private supported
  policy-marker validation only, RESERVED until root explicitly releases its
  active native build-input freeze; no C++ writes during that build.
- NEW `tests/simulation/test_liquid_vessels_native.cpp`: actual normal RQ4
  source/child-root-cell work, native kg/ownership/departure and reset controls;
  source-only and unregistered until root integration/native slot grant.
- NEW `docs/research/sim-m03/rq4-inferred-vessels.md`: method, evidence limits,
  coarse sealed-module fracture and native lump/actual-cell centroid discrepancy.
- This journal: this ownership record and bounded feature handoff only.

No accepted RQ4 flight TOML/XML/capacity/anchor/CG/inertia, native tank identity,
public layout/vtable, physical owner's breakup cap files, CMake, existing native
fixtures or unrelated sources are writable. No configure/build/native/model
process or generated aircraft product publication is authorized in this source
window. Root retains the serial native slot. Exact source policy and original
source-wing/hull/equipment clipping must be measured before claiming capacity
closure; original negative proof is retained. The final current all-model,
installed, exported and fullscreen program gates remain independent and pending.

## Current native handoffs and isolated geometry observations

Root's current three native material gates passed without changed predicates:
`liquid_native_binding` 3 cases / 1,554,105 assertions / 78.19 s;
`fragment_reach_native` 3 / 2,393 / 73.66 s;
`weapon_effects_population` 2 / 224,798 / 182.67 s, including all26 original
authored explosives and the existing terminal controls. Raw full CTest output
was archived as `current-native-materials.raw.log` before another CTest could
overwrite LastTest.log; its SHA256 is
`4f5a0ee00aba468647064c63f1b364ef4339839b371a2dee4e249bcf3aa12eb8`.
The three-entry JUnit hash is
`9ce31b110632e63f71061d2582ab81cd60d412f028e321b63879260054edde51`.
The corrected solid-birth moving-origin fixture passes the SAME164 assertions;
independent review `source-review-native-moment-solid-fixture.json` records the
unchanged eight worlds, containment, native rate bounds and legacy control.

Five of six additional native gates passed (navigation integration/policy,
sensor engineering, proximity and retained V2 batched contact). The combined
12-case `engagement_native` exceeded root's180 s diagnostic default. This is
not the established full-program default: `sim_m02_closure.py` uses1200 s.
Root reruns the unchanged binary/scenes under that existing1200 s condition,
with full passed/failed output limits and exact pre-run source+binary snapshot
`current-native-guidance-standard-epoch.json`. Preserve the180 s interruption
and its raw log. Do not change a functional predicate or native outcome.

The unchanged `engagement_native` rerun completed successfully: all12 native
cases pass in267.13 s under the established1200 s condition. Exact raw output
`current-native-guidance-standard.raw.log` has SHA256
`fd06402685bab878787ede19651af38ed0f847f4caf283cfdc200bf2c79bf634`.
This closes that scoped execution gate, preserving the earlier180 s diagnostic
interruption. The current direct scene/effect fixture also passes40 cases /
11,447 assertions; `current-scene-effects.log` SHA256 is
`089c1abea36f14e8654c28970980585c7743270d370473650ac1a61f0efac442`.
These native receipts do not replace current all-model production, complete
both-preset regression, installed consumers or actual fullscreen presentation.

The RQ4 owner now receives the previously reserved C++ parser/new-native-test
SOURCE write window. Root will not compile during this source handoff. The
running diagnostic uses the frozen pre-change binary, SDK libraries and model
assets; its exact old parser/source bytes were archived before this release.
Final acceptance requires a later rebuild and fresh full campaign.

The effects owner may run ONE cloned U2S normal-entrypoint phase observation,
one CPU/thread, unchanged900 s diagnostic parent bound and unchanged liquid
limits, with all outputs in D:/ENV only. Snapshots before/after imports bind the
loaded method epoch; a concurrent later producer edit is explicitly marked as
epoch-changing diagnostic evidence, never final production admission. The
physical owner separately runs the already approved sequential cap observations.
These scratch processes cannot publish project assets or alter native inputs.

Root's next normal RQ4 publication window consumes the owner's frozen11-file
handoff and unchanged main HEAD. The normal CLI is `hangar rq4b model`, one
geometry thread, with all native configure/build/test/GUI slots idle. It owns
only normal generated `aircraft/rq4b/rq4b.glb`, its manifest, damage/liquid
sidecars, normal model cache/report and the existing generated RQ4 piece flight
products. It does not hand-edit flight XML/TOML or source geometry. The new
`normal-model-one.py` diagnostic runner records the actual command, bounded
child-tree cleanup, nine method hashes before/after, raw output and products.
Its900 s parent bound is diagnostic; the existing300 s liquid packing controls
remain unchanged. No other source/model writer may change these input bytes.

Root integration source window at the same main HEAD:
`tests/CMakeLists.txt`, only new `fsim_liquid_vessels_native_test` executable,
`liquid_vessels_native` CTest entry and its membership in the existing mandatory
M03 runtime-DLL staging helper. Use the actual source/card/flight paths and
existing session/Catch2 owners, a nonzero three-case population, serial native
execution and bounded300 s property. No configure/build/test until the normal
RQ4 writer joins. This source registration is not a producer input or a native
execution and changes no file consumed by that writer.

## External tank native fuel functions population window

Main HEAD e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f, branch main verified.
Owner /root/m03_effects_review receives source-only complete feature scope:
NEW tests/simulation/test_external_tank_population.cpp,
NEW tools/validation/weapon_quality/external_tanks.py,
NEW tools/validation/weapon_quality/test_external_tanks.py,
NEW docs/research/sim-m03/external-tank-functions.md.
No existing simulation, card/fit/TOML/generated flight XML or shared build input
writes. Root owns later CMake/campaign/report registration and exclusive native
execution. Preserve original test_impact fuel/fire thresholds. Derive all19
actual maintained tank cards independently; bind actual per-model native fuel
transfer, remaining fuel, release mass/CG and native pointmass/ground-fire
controls. PTB1150 lacks a real authored fit: any generic mechanically compatible
mount or standalone native fixture must be labeled diagnostic, never OEM.
This closes the existing applicable fuel functions only; carried/free fragment
vulnerability and SIM-M10 store/task/MDF completion are not inferred.
Before authoring, one <=60s read-only U2S original-hull/old-epoch-classifier
necessary-volume query may run against the finished DENV clone, one CPU/thread,
no packing/model/SDK execution, with unchanged input hashes and DENV-only output.

### RQ4 inferred-vessel source handoff (not production acceptance)

Owner `/root/m03_delivery_audit` releases the source window at unchanged main
HEAD `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`. All eleven owned feature
source/config/test/document inputs below are frozen for root integration.

| Path | SHA256 |
| --- | --- |
| `aircraft/rq4b/rq4b.liquid-geometry-v1.json` | `476d524b8c018c152bdc922a4b9d5899b6fc638d2daf911576712f44728ab82a` |
| `tools/models/hangar/hangar/liquid_vessels.py` | `2554778aebf258f6339679eabb1e053c92311394f74d8e487a4563d741ac26c7` |
| `tools/models/hangar/hangar/liquid.py` | `c4ef2d09d3c9b3e5ff3a758b0c08d079d20c4ee35cf3d79da915080c33d11601` |
| `tools/models/hangar/hangar/engineering.py` | `c7fc5d7b1d95be88ecb28ee7ec654851be705e752644c849a76ddf6da930493f` |
| `tools/models/hangar/tests/test_liquid_vessels.py` | `fc027d0c98fc1822fb2153718a5a4ea707e9ff4a8572672f740d3699b5dbf88f` |
| `tools/models/hangar/tests/test_liquid_vessel_inputs.py` | `ae86e268a3d3220b415329c6e0519098be0108964c5b7480faff1170ce41c9fd` |
| `tools/validation/weapon_quality/liquid_population.py` | `d83e9e1683dc5105e0d8a2b2caf7461d40c11bd3b7842391badfc12ce5421e93` |
| `tools/validation/weapon_quality/test_liquid_vessel_policy.py` | `cf503b1df29dd6759d2c171b1c4dcc7b6194be6b567f7979800f742d0b19cf38` |
| `src/simulation/runtime/world/WorldLiquidMaterials.cpp` | `585bd63df70636445c38e1dba693af338817e4de98f79e6ac2f4530bb4199a13` |
| `tests/simulation/test_liquid_vessels_native.cpp` | `5ba8cb1e418f91f85620acb795008183d2c7e07d3d289d65b6b17fd00abe445b` |
| `docs/research/sim-m03/rq4-inferred-vessels.md` | `2de6227a2cf2ef60047fa92363627e678a9270fa4a1d55fd7aa29c0d41318f0d` |

The explicit source declaration admits only native TK0/TK1, their own original
uninflated wing root half-strips and the declared theoretical sealed-module
attachment. Restricted TK2/TK3 allocate first. Every final slab is certified
per TK, original hull/exclusions remain mandatory, prior actual capacity cells
are subtracted and exact positive-interior AABB checks plus the final global
sweep reject overlap. Native structural singleton ownership is explicitly false
for this inferred product, with a separate required vessel eligibility receipt.
Full and reference-up half-fill actual spatial centroid/bounds/native-lump
mismatch are reported. No accepted mass/flight input, identity or public layout
was changed. The accepted RQ4 TOML remains SHA256
`719034f2ed9c281ca9fb6e9f4ade01f8093220cca198655d90af36c82203d35c`.
Reversing only the generation_inputs optional-policy bind reconstructs its
exact inherited source SHA256
`22b3302710ff231328c05efb159cbcfeb9753bd578ede326fa712373b0513b97`.

The RQ4 sidecar requires the original thirteen liquid source roles plus eight
new exact roles: inferred_vessel_declaration, inferred_vessel_method,
inferred_vessel_geometry, inferred_vessel_mesher,
inferred_vessel_section_method, inferred_vessel_airfoil_method,
inferred_vessel_airfoil_coordinates and inferred_vessel_mesher_binary.
Other liquid products retain thirteen and no-liquid products seven roles.
The collector independently checks the exact role set/current hashes and
original LRN input; it does not rerun Manifold or admit equipment calibration.

Authorized pure-method execution: twelve tests pass in 1.555 s, raw
`build/architecture-reviews/sim-m03-production-20261103/rq4-vessels-method-fourth.log`.
This covers strict declaration, own-side/per-TK/final-slab geometry, restricted
allocation, real hull/equipment/nonoverlap controls, actual reference half-fill
first moments, loaded/current foil consistency, exact foil roles, optional
source presence/hash and independent generation reuse refusal. The initial
nine-test/one-error raw first log and overlap-observation log are retained:
Manifold preserved an exactly coplanar four-vertex zero-volume contact as
nonempty. The new per-TK path retains the Boolean status/subtraction but uses
exact original AABB endpoints for the existing positive-interior criterion;
no tolerance, capacity or density was changed. Earlier second/third focused
logs are historical runs, not additional acceptance.

Three new native cases are source-authored, unregistered and UNEXECUTED:
`[liquid-vessels-native]` has actual attached root work and whole-TK departure,
all translated owned cells/native kg, child-only later work and empty-child
physical reset for frameSkip 1/4 and workers 1/2; `[liquid-vessels-policy]`
uses the actual produced document positive plus nine malformed/unsupported/
borrowed/downgrade controls. The actual World diagnostic uses real releases,
source IDs/generations, ordinal one, no blast, preselected root geometry outside
old left fuel probes, unchanged 220 m/s flight/5 m/s relative point velocity,
zero born-inside wall hits and the original 2e-5 N s reaction bound. It is a
material fixture, not an authored weapon lethality or actor-consent claim.
Its geometry dependencies are explicit; absent valid produced root geometry
fails rather than substituting a probe, changing a scene or declaring success.

No configure/build/native/GUI or normal model writer/publication was run by this
owner. The new native source reuses isolated independent geometry/analytic
oracles from the established F16 material fixture in its own translation unit;
no existing native fixture was edited. Root owns CMake registration, first real
normal RQ4 generation/publication, actual native runs, all35 collection and
installed/source-export/visual integration. Root-source containment/capacity,
new native outcomes and complete SIM-M03 acceptance remain pending those gates.
The original 4.905 versus 4.724 m3 negative proof is retained, not relabelled.

Root integration source window, main HEAD unchanged:
`cmake/ArchitectureChecks.cmake` only for the external-tank and vessel-policy
standard-library controls and a configured model-method suite; NEW
`tools/validation/weapon_quality/model_methods.py` bounded unittest entry with
explicit source/build/dependency paths and no accepted skip/empty population;
`tools/validation/weapon_quality/report.py` only to require those actual CTest
names in both complete-preset inventories. Existing gates/identifiers/rosters
and the approved28 exclusions remain unchanged. The authoring-method suite is
registered when the existing explicit Python dependency path is configured;
the complete campaign supplies that path and cannot accept its missing test.
Root owns the serial native slot after the external-feature handoff. This
source window changes no Hangar producer or current pure geometry replay input.

Independent method-entry review found that aggregate nonzero loading alone
would admit one selected module with zero tests. Root's narrow correction
window additionally owns `test_acceptance.py` only for synthetic required-module
admission controls. Count each fixed module separately before composing the
suite, reject an empty module/list or loader error, and retain the actual92
current method cases without a hardcoded passing count.

The required-module controls execute23 current admission tests successfully,
including the three empty-module/list/nonempty-module controls. The negative
source path is the existing `tools/validation/weapon_quality/test_acceptance.py`;
no additional negative-test file was introduced. Both configured presets had
already passed the three registered method entries, including the92-method
population; the repaired loader requires another current invocation before
final evidence admission.

Finite delivery ownership decision: preserve the inherited workspace deletion
of `docs/superpowers/plans/2026-10-18-sim-m02-aero-convergence.md` without restoring
or staging it. This is accepted M02 historical plan content, outside M03's
feature-owned change list; an ordinary HEAD-based M03 source export therefore
retains its original HEAD bytes. The missing generated
`aircraft/stores/aim9m/aero/vlm-carriage-c7f3d5fa305817d5a6d7d93431fa4a2cfa5a9571d10caa1ac9b27ab784f6db05.json`
is a retirement candidate belonging to the generated AIM9M product namespace.
The current card binds only `vlm-carriage-9a6a1248f1d14d234ed09662b81c6c7c0367ffb3818e36758c5dbb10405fcabe.json`;
normal runtime staging explicitly retires noncurrent owned carriage grids.
Include only this generated retirement in M03's reviewed deletion set, subject
to final forced-production references/product reconciliation. No new deletion
or restoration command was executed by this decision.

The real normal RQ4 d1cf attempt joined at68.4399927 s BEFORE liquid packing:
`model3d.finish` could not truncate the mapped rq4b.glb after its existing
bounded retries. This does not reclassify the393 s earlier pack failure or the
current whole-U2 deadline failures. The now independently reviewed atomic GLB
feature preserves the three legacy helpers and all GLB/manifest serialization
bytes, passes13 actual mapped-reader/failure controls, and is admitted only as
per-file publication. Root adds `test_glb_publication` to the configured required
model-method modules; all current method cases must run again after the ongoing
owner-process source window freezes. The actual normal RQ writer still must
retest both publication and its full liquid/native products.

Root's actual normal RQ4 model gate joined at393.8259606 s with unchanged nine
producer hashes and failed in the left-wing pack screening-budget branch.
No liquid product was published; the diagnostic log alone does not distinguish
screen-count from clock exhaustion or TK2 from TK0. Preserve that failure.
The physical owner now owns only `liquid.py:pack`, a new exact-equivalence test
and the search-method paragraph, retaining every geometric/control predicate,
exploration ordering and fresh final-slab audit. No normal asset writer or
native execution is authorized by that source window.

Root registered and built the new RQ native target. The first strict build
exposed a test-only misuse of optional.get(); the checked optional's address
replaced it, preserving every fixture predicate. Current native parser
e061f6020ce0c696c12cdc9f3c357316486bf330c4e86324adaa47d6c7bb739f
and test b02631f4f9fa8aa403e6da2bd30de59639627e26cd4bb7e693432f0d0226a7f3
compile successfully. The three RQ native cases require a real produced liquid
product and remain unexecuted. Both reviewed native-policy/ownership deltas are
recorded in the independent closed review, including genuine legacy absence.

Freshly rebuilt showcase and material binding pass4/4 CTest entries. Raw output
`current-native-showcase.raw.log` SHA256
f89900616e45170df4ff3525d93bac9d221d27a7f88f07c066e3e84d4d50e4ef
retains the actual focus/positive-Hit, functional damage/continued/unthreatened
flight, deterministic recording/legacy-refusal and9/472 material-binding proof.
These are current scoped native observations, not whole-model or visual
acceptance. No Viewer scenario ran in this window.

Root's next exact integration source window is `tests/CMakeLists.txt` only:
new `fsim_external_tank_population_test` / `external_tank_population`, native
session/JSBSim/Catch2 owners, actual source/runtime macros,300 s serial test,
per-preset raw destination and existing mandatory M03 runtime-DLL staging.
No carrier/source fit, card, generated flight input or original impact fixture
is writable. After registration root may transfer the exclusive native slot
to the complete external-tank feature owner for its own real first gates.

### External tank mandatory campaign join window

Main HEAD: `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`; branch main.
After both scoped native populations passed (one case / 4,422 assertions per preset),
the external-tank feature owner may edit only
`tools/validation/weapon_quality/campaign.py`, `report.py`, and `README.md`
for the mandatory external19 source-epoch / current both-preset receipt join.
The existing external-tank collector, its negative tests and method document
remain in the same feature window. No shared native source or CMake changes.
Preserve every other campaign command, applicability, installed, visual and
source-review requirement. Root owns final shared registration and full campaign.

### Closed atomic GLB publication window

Main HEAD `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`, branch main.
The effects feature owner owns `tools/models/hangar/hangar/files.py` only for a
NEW atomic staged binary/text stream helper; preserve existing open_write and
atomic_write_text behavior. Own `model3d.py::finish` only for per-file closed
publication of unchanged GLB/manifest bytes; preserve geometry/serialization.
Own NEW `tools/models/hangar/tests/test_glb_publication.py` and NEW
`docs/research/sim-m03/glb-publication.md`. Pure DENV fixtures on one CPU only;
no native/model/project-asset execution or CMake edits. Raw RQ normal publication
failure is not a packing result; prior packing failures remain independent.
Snapshots are retained in `glb-publication/before.json` and exact before files.

### Mandatory producer controls integration

Main HEAD remains `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`, branch main.
Root owns only `tools/validation/weapon_quality/model_methods.py::MODULES`
to require `test_producer_publication` and `test_liquid_owner_processes`
alongside the existing twelve modules. Each module must contain actual tests;
the existing load-error, empty-module, skip and expected-failure refusals remain.
Run the complete configured method entry only after both producer windows freeze
and receive independent review. No execution pass is claimed by registration.

### Measured packing and resource follow-up

The immutable104-file DENV diagnostic package retains liquid producer
`c2efff18db6e677d63cda0a7a38a4fd33b9c2acd64c67fc2358abba22bf65952`
and resource helper `9192ca2000bf5e2b3485d5a79b65f790c5d86bc49a344d94ab9147f5d91aafd2`.
Its old startup boundary and production job integration were explicitly not
admitted by these measurements. All original geometric/depth/count/time limits
remained unchanged; no diagnostic wrote project model products.

On verified distinct physical cores (logical CPUs0,2), full U2 production of
liquid geometry returned in267.198 s within the original300 s packing budget:
five tanks,62,400 cells,880,445 screened cells and124,695 assignment edges;
all five capacity residuals are zero. The complete left-wing cell and tank
payloads exactly match the retained original serial left-wing result. The
same frozen producer on logical CPUs0,1, which share a physical core on this
host, reached the300 s limit after40,009 emitted/audited cells and returned no
complete product. Both searches reached the same880,445 screening and124,695
edge counts. Concurrent authorized RQ work and OS load are recorded; these are
operational observations, not a controlled attribution of every timing delta
to SMT. The paired evidence is
`D:/ENV/tmp/m03-u2-owner-full-20261103-smt01/comparison.json`, SHA256
`26cb34f1a8ce2892bd7a6f23327721ef4544f03e2da9c1486721e1715eb440b1`.

The full RQ inferred-vessel diagnostic refused the original shared packing
deadline. Restricted TK2/TK3 closed in14.903/22.462 s with1,233/1,211 cells;
TK0 exhausted its262.197 s remaining budget with1.3884633577 m3 certified
against2.875 m3 required. TK1 did not start. No complete RQ product was saved.
The retained receipt SHA256 is
`f5b501056b052cd98d645a37b68c650c86a72230417969989195cb8224bd74fc`.
An explicitly interrupted60 s TK0 profile measured3.12058229947 m3 net
available after prior-cell and necessary-region clips, a positive0.2455823 m3
margin, without proving successful certified packing. Cache fallback was zero;
original interior Boolean materialization dominated the measured work. Its
separate receipt SHA256 is
`d82a1e2ed091a6bf2f2bdf56afd11bbda43afab64b5000d7af73f4a8a2b481c5`.
The original negative remains authoritative until an unchanged-control complete
producer and actual normal/native gates pass.

The startup/job repair subsequently passed83 method cases, including19 process
controls, with actual slow-import/backpressure cancellation, repeated exact
warmed handle non-growth and nested2CPU-job/single-CPU-worker checks. Independent
review `source-review-liquid-owner-processes-repaired.json` SHA256
`11cbf200792a6bb7f593f02a70483cce47be942b53fa8fc4bf937e5ac35a1e99`
closes its two earlier source findings. Default topology ordering is a separate
resource-only window; inferred RQ packing remains a separate geometric problem.

The verified physical-core-first default selection subsequently passed89 liquid
controls and its independent three-file source review,
`source-review-liquid-cpu-topology-current.json` SHA256
`a130ca1304a38c5a2e1a0e2f0023625966db023e5bb5a756ea2f4d0acd096319`.
Explicit CPU identity/order is preserved, and the actual no-override resource
scope selects CPUs0,2 on this host then restores the original mask. These are
resource observations; fresh normal U2 production remains required.

Root's next exact method-entry window requires NEW
`test_liquid_vessel_packing` in `model_methods.py::MODULES`, alongside the
existing fourteen modules. Before SHA and scope are recorded in
`model-method-inferred-pool-window.json` before the selector edit. The load,
per-module nonzero, failure and skip guards are unchanged. Execute all fifteen
required modules only after the inferred-pool implementation/dependencies freeze
and receive independent review; registration alone claims no passing execution.

### Private liquid derived-state finiteness repair

Independent review found that individually finite disjoint capacity cells can
overflow a binary64 aggregate, and finite tolerance inputs can overflow their
derived acceptance bound. The original two independent regressions fail6 of25
assertions before the repair. Root's exact window is retained in
`liquid-volume-overflow/window.json`; only `Damage.cpp::prepareLiquids` and
three corresponding tests in `test_liquid_scene.cpp` changed. Reject nonfinite
aggregate wet/capacity volume after each addition and nonfinite derived tolerance.
Keep the original positive-contents/missing-capacity refusal before any map
lookup that inserts a default value. A separately retained intermediate RED
catches that ordering mistake even when the missing contents are below numeric
volume tolerance; it is corrected without loosening the original contract.

Both current Release and Headless strict rebuilt method populations pass the
same61 native cases /4,805 assertions per preset: fluid10/375, scene42/3,958,
material binding9/472. These are two build variants, not122 distinct cases.
Independent source review `source-review-liquid-volume-overflow-current.json`
SHA256 `f4aad9b5961ff1e0a058d50ff0fc057453acb5b81ea590b654be947bcb0722ca`
closes the scoped finding. Original integration laws, numerical controls,
ordinary input behavior and original scene assertions remain unchanged. Full
program, generated model and visual acceptance remain separate required gates.

### Complete whole-file producer publication closure window

Main HEAD `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`, branch main.
Effects owner owns `files.py::replace/_publish_closed` for common guarded
publication with unchanged legacy refusal/deadline behavior. Preserve
`atomic_write_text`, `open_write` and the already reviewed native rename.
The finite source path list under `tools/models/hangar/hangar/` is
`pipeline.py` (`write_text`, `Design.save/build` complete JSON/engine/main XML
contexts), `damage.py::write`, `pieces.py::write`, `rotorcraft/heli.py::_write`,
`rotorcraft/__init__.py` save/hover contexts, `storestages.py::main` report JSON,
and `model3d.py::refresh_engine_lines`. Only complete whole-file write contexts
and necessary imports change; preserve serialization, unchanged shortcuts and
return values. Own NEW `tests/test_producer_publication.py` and publication
contract `docs/research/sim-m03/glb-publication.md`. Before bytes/hashes are in
`producer-publication/before.json`. Pure DENV fixture execution on one CPU only.
No model/native/build/GUI or CMake execution; no liquid/engineering/resources or
occupancy-doc writes. Separate current `fuel-transit-method.md` review read-only.

### Current normal U2 and configured mandatory-method integration

The current normal launcher source-tree `u2s model` invocation passed in
352.4384975 seconds with all eighteen captured producer hashes unchanged.
Its five tank ledgers close10.8975 m3 with62,400 cells; authored TOML/XML,
calibration and paint bytes are unchanged. Receipt
`normal-u2-current/followup.json`, SHA256
`0372388603c500cc524c49595bfdc34242ce58de0ffec8c5c6c8217bdede60a0`,
is source-tree model publication evidence, not an installed/all-model claim.

The configured mandatory method entry now requires fifteen modules. The
original failed run revealed that owned child processes lacked configured
Python import roots and that two mocked TOML fixtures accidentally invoked
real source-backed worker reconstruction. The runner propagates exactly its
four configured import roots; only those two mocked fixtures reserve one lane.
Production parallel fixtures and their original assertions remain required.
Both Release and Headless now pass all three selected CTest entries, including
161 mandatory method cases each without skips or expected failures. Joined
receipt `current-required15-both-presets-receipt.json`, SHA256
`7dd360b1fd8117cdcf5debc573e3a8cc32adf7cd7262f04a96c60b3d5a4dcdf4`,
binds the raw original logs and current sources. Future producer edits require
fresh method verification; this is not whole-program completion.

### Complete native liquid population and unsectioned receiver window

Independent coverage review found that static35-model/149-tank admission and
ordinary fleet loading do not establish per-tank production freeze, transit
and reset. Thirty positive models/141 tanks lack those dedicated assertions.
UH1H/UH60 are positive standalone liquid products, not inapplicable: the
structural-only recipient path currently skips their absent damage Airframes.
Preserve the approved28 rotor flight deferrals and implement liquid-only
receivers without inventing structural geometry or a duplicate fragment field.

Delivery owner exclusively owns `WorldLiquidMaterials.{h,cpp}`, material
receiver portions of `WorldDamage.cpp`, an additive private liquid-only victim
input/overload in `LiquidSceneV1.h`, internal optional structural adaptation in
`Damage.cpp` (existing layouts and finite preparation guards preserved), and
one private `World.h` overload declaration only if required. It also owns a
new native all-model fixture, its additive registration in
`tests/CMakeLists.txt`, and mandatory campaign/report population joining in
`tools/validation/weapon_quality/{campaign.py,report.py,liquid_population.py}`
plus focused new collector controls/docs. Exact before bytes and added symbols
are recorded by the owner before edits. Root retains the serial native slot.
All recipients must participate in one globally chronological material kernel.

### Geometry-directed inferred-vessel refinement window

The retained strict RQ shared-pool search does not close the required capacity;
none of its timeout/refusal diagnostics is a complete product. A finite exact
affine-wedge prototype shows that a facet-bounded vertical column proposal
plus slope-sensitive XY residual splitting avoids the old isotropic-cell
explosion. Its independent proof/control artifact is
`rq-shared-search-refinement/inscribed-column-analytic-proposal.json`, SHA256
`6aa802aeb2939e15c8b1b1d1aa3ff5cc6577f95912efcbbd0f40131baf6876cc`.
This is proposal evidence only, not an actual RQ measurement.

Effects owner exclusively owns private proposal/search/cache helpers in
`liquid_vessels.py`, additive `test_liquid_vessel_packing.py` controls, and
`rq4-inferred-vessels.md`. Original meshes, capacities, physical densities,
native ownership, target resolution, edge/depth/count/cache/time limits and
fresh final geometric checks remain unchanged. Any sufficient plane proposal
still requires the actual original strict audits; unsupported/nonconvex/cavity
cases fall back, and nonfinite/negative kernel results remain refusals. Root
reviews finite source deltas and bounded analytic/actual-subset observations
before authorizing another complete RQ run. No model/native/build/GUI slot is
included in this producer source window.

### Large reviewed-population delivery repair

Root's exact two-file before window is `delivery-large-population/window.json`.
An ordinary independent repository with384 long generated-product identities
plus the retained CRLF/binary/deletion population reproduces actual Windows
`WinError206` in the old explicit-path patch command (1 case/1 error).
`delivery.py::prepare` now diffs the HEAD-seeded, reviewed-updates-only private
index with bounded existing exclusion pathspecs, and independently requires its
NUL-delimited changed-file set to equal the exact reviewed delta. It still
exports exact immutable blobs and preserves the real index and unrelated work.

All15 pure delivery cases pass in18.459 seconds; receipt
`delivery-large-population/receipt.json`, SHA256
`582d6dcd7ca462564c8cfad0c54bb6c49365e1956b0435e3c8b43e2baa976d7c`,
binds original RED and final raw logs. Independent two-file review
`source-review-delivery-large-population-current.json`, SHA256
`51736510dbc384381bbe637de6dfe7feb359ff2cce40f6821a1b1e127bcd3e06`,
confirms all17 original test/helper method ASTs and all other production
functions unchanged. This closes the demonstrated tool-scale blocker only;
complete actual source/package replay and fullM03 admission remain required.
