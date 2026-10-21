# SIM-M03 five-kilometre production repair

Baseline: main `92ae5ed16295b001592db30eae0928a6dcc00e10`, clean at entry.
The user assigns root-cause isolation and SDK production repair of both fixed
5 km hit failures, followed by regression verification and commit. Scenes,
equipment values, control gains, fuze bounds and hit criteria remain unchanged.

Root owns the exclusive source-write/native configure/build/test slot and the
single Viewer slot. Prior parallel SIM-M03 authorization continues as read-only
control and damage reviews; neither reviewer writes or executes native tools.
No external numerical directory, other worktree or excluded directory is read.
Use the maintained UCRT64 launcher and D:/ENV temporary directories.

### Final integration subwindows

Root delegates only `WorldDamage.cpp::separatedAt/launchPiece` and
`test_mass_change.cpp` to the independent reviewer for loaded-debris momentum
correction and a regressions-first conservation fixture. A separate window
owns only World native capture epochs/cutoffs in `preStep`, `step`, Entry
bookkeeping and `finishEngagements` cutoff derivation; other World edits remain
root-owned. The fuze reviewer owns the three native engagement/showcase test
and evidence paths to add actual functional-loss and continued-flight gates.
No worker runs native builds/tests. All input owners must freeze before root
executes the serial gates.

## Exact initial path and symbol allowlist

* `src/simulation/runtime/world/WorldDamage.cpp::detonate` and
  `WorldEngagements.cpp::finishEngagements`: opt-in, non-mutating diagnostic
  capture of exact burst fraction, axes/velocity, target origin/axes/velocity,
  native paths and assessment/RNG inputs. No diagnostic draw may consume RNG or
  feed simulation. Remove temporary diagnostics before final delivery unless a
  separately reviewed diagnostic contract justifies retention.
* `tests/simulation/test_engagement_native.cpp::options/engagement`: opt-in native
  recording for the existing scene, without changing execution or assertions.
* `tests/simulation/test_damage.cpp`, `test_weapon_guidance.cpp`: independent
  analytical regressions before any demonstrated damage/fuze production repair.
* `src/simulation/mission_systems/damage/Damage.cpp`,
  `weapon/{Fuze,MotionHistory}.h`: only independently demonstrated geometric,
  frame or timing defects, after a recorded RED invariant. A hypothesis is not
  authorization to change behavior, and a lack of fragments alone proves no bug.
* This plan, new `docs/sdk/sim-m03-five-km-repair.md`, the current SIM-M03 status,
  allocation/carryover and delivery records: exact causes, evidence and limits.
* Ignored `build/architecture-reviews/sim-m03-five-km-*`: native recordings,
  diagnostic/replay helpers, logs and immutable baseline evidence.

Expand any necessary source allowlist before its first edit. Preserve public
layouts, vtables, permissions and existing outcome meanings. Do not synthesize
damage, replace the scenes, bypass failures or relax their requirements.

The exact existing helper regression file is `tests/simulation/test_fuze.cpp`.
Root adds a same-outer-interval, unchanged-linear-motion subdivision invariant
there before any `MotionHistory.h` behavior change. A redundant target-only
sample must not alter the stated closest-approach result. Changing World
decision cadence or adding future prediction is outside this numerical repair.

## Demonstrated contact-geometry window

Both independent reviews identify showcase contact panel 44 as a damage-only
midpoint box extruded over the first metre of the fuselage. The captured nose
point is body `(7.99999924, .01993252, .14092219)`; the authored tip at body x=8
has zero width and z in `[.33,.37]`. The point is outside the authored skin.
This proves premature contact, not successful later fragment damage.

The contact owner receives an exclusive vertical feature window over
`src/simulation/mission_systems/damage/{Damage.h,Damage.cpp}` (new private
versioned contact geometry and reader only), `weapon/ContactFuze.{h,cpp}`,
`tests/simulation/test_contact_fuze.cpp`, `tools/models/hangar/hangar/damage.py`,
`tools/models/hangar/tests/test_damage.py` and `docs/sdk/weapon-contact-fuze.md`.
Root retains World diagnostics and all native build/test execution. Add RED
tapered-body fixtures first, then wait for root's run before native repair.
Reuse the existing authored `Body::skin` geometry. Existing damage boxes,
fragment/THOR/blast inputs and all equipment numbers stay unchanged. Preserve
initial-inside suppression, immutable section membership and detached-body
exclusion. Broad-phase bounds alone cannot authorize physical contact. Invalid
explicit geometry must fail closed rather than revert to a box hit.

Root owns generated-product integration after source handoff. Only tracked
`aircraft/<authored-design>/<same-name>.damage.json` may gain the new contact
metadata, through the maintained generator from unchanged TOMLs. Preserve every
pre-existing JSON value and all XML/GLB/motor/card bytes. Record the exact output
list and before/after proof before committing. No hand-editing runtime products.

## Execution

## Resumed physical-damage window

On 2026-10-20 the user rejects Hit-only completion and assigns research,
production repair and actual state/flight verification before commit. Main HEAD
remains `92ae5ed16295b001592db30eae0928a6dcc00e10`; the earlier geometry/CPA
changes remain uncommitted and feature-owned. Root retains the serial native
build/test and single GUI slots. Research and explicitly assigned contact
geometry work may proceed in parallel, with disjoint write windows.

The production contact response will use a declared frictionless, zero-
restitution rigid-body impulse, not a ballistic penetration or calibrated
equipment model. It must use captured native mass, full inertia, point velocity
and a genuine exposed face normal; conserve linear/angular momentum and never
create kinetic energy. Local root loading uses the existing damage capacities,
with the same parent/debris impulse redistribution as blast. No impact is
counted as a fragment. Internal ownership caps cannot supply exterior normals.
Analytical and native RED evidence precedes each repair.

Additional exact root allowlist:

* `src/simulation/flight/{FlightModel.h,JsbsimModel.h,JsbsimModel.cpp}`:
  additive optional rigid-mass snapshot interface, preserving the primary
  FlightModel vtable and existing public layouts.
* `src/simulation/runtime/world/{World.h,World.cpp,WorldEngagements.cpp,WorldDamage.cpp}`:
  captured mass/inertia epochs, contact response and localized native damage;
  correct any demonstrated post-impulse debris-state epoch defect.
* `src/simulation/mission_systems/damage/{Impact.h,Impact.cpp,State.h,State.cpp}`:
  pure contact mechanics and existing structural-load feedback.
* `tests/simulation/{test_damage.cpp,test_engagement_native.cpp,test_contact_fuze_native.cpp}`,
  `tests/integration/test_showcase_native.cpp`: independent conservation,
  actual state/physical response and existing unchanged-scene acceptance.
* Native showcase source discovered from the maintained CMake target, only
  additive referee evidence of damage state and continued flight response.
* `weapon/MaterialPoint.h`, `tests/simulation/test_impact.cpp`,
  `tests/simulation/test_mass_change.cpp`, `tests/CMakeLists.txt` and
  `tools/validation/showcase/main.cpp`: material-point/conservation regressions,
  registration and native SDK showcase evidence.

The user subsequently requires model repairs and completion of model-card
parameters, including control algorithms as MS payload configuration. Root owns
`assets/stores/aim-120c.json` (all card writes, including combined warhead/control
inputs) and source/provenance documentation. Values must come from identified
public evidence or a recorded engineering derivation with uncertainty, never
reverse fitting to these encounters. The unchanged-scene prohibition does not
freeze demonstrably defective model data; every intentional parameter correction
must have its own independent rationale and before/after record.

The control owner receives an exclusive vertical write window over
`weapon/{Guidance.h,Guidance.cpp,Autopilot.h,Autopilot.cpp}`,
`tools/models/hangar/hangar/storecontrol.py`, its directly relevant generator
tests, `tests/simulation/{test_weapon_guidance.cpp,test_missile_autopilot.cpp}`
and a new control-configuration document. It implements a versioned card-to-
algorithm configuration seam and signed actuator-stop normalization. Root alone
integrates that seam in World release/native control and writes cards/generated
assets. No primary FlightModel vtable, generic MA facade or existing public
layout changes are authorized.
The explicit profile also carries native actuator lag/rate; the maintained
generator must consume them and the controller must validate the actual plant
lag rather than silently assuming configured dynamics. Existing values are
retained as documented engineering assumptions, not equipment measurements.
The control owner's generator window additionally includes only
`tools/models/hangar/hangar/storebodies.py::files`'s `storecontrol.apply` call
and its directly associated input fingerprint, to forward the existing card.

The fuze owner receives `weapon/{Fuze.h,MotionHistory.h,FuzeDelay.h}`,
`tests/simulation/test_fuze.cpp` and a new fuze-model document. It adds an
explicit V1 turning-point CPA law (an approaching interval endpoint is not a
physical CPA), plus a pure delayed-initiation lifecycle. It must preserve legacy
unversioned numerical contracts and all arming/lifetime/native-cutoff exclusions.
The control owner alone adds/validates the agreed Spec fields; root alone binds
material sensor points, delayed native epochs and contact arbitration in World.

The warhead owner receives `damage/{Damage.h,Damage.cpp}` (warhead model and
assessment only; retain validated contact geometry), new
`tests/simulation/test_warhead_model.cpp` and its own warhead-model document.
The V1 finite cylindrical side-case is a declared generic analytical model:
radial ejection, exact annular casing volume, Gurney speed, equivalent cubes and
integer mass conservation. It replaces game angular weights in this explicit
profile without claiming calibrated WDU-41 fragmentation. Root owns CMake and
all model-card writes/integration.

The contact geometry owner has the exclusive vertical write window over
`damage/{Damage.h,Damage.cpp}` (contact metadata only),
`weapon/ContactFuze.{h,cpp}`, `test_contact_fuze.cpp`,
`tools/models/hangar/hangar/damage.py`, its damage tests and the contact-fuze
document. It adds exposed-face provenance and exact point/normal output, with
RED cases for internal ownership cap ties. Root does not edit those paths
until frozen handoff. Generated JSON integration remains root-owned.

Burst placement is investigated separately. The current guided burst is at
moving CG despite the internal contract naming a warhead centre. No source
supplies an authored AIM-120C warhead station. Do not move it to the tip,
launch CG or a guessed station to improve damage. Any additive fixed-point
contract must retain explicit availability/provenance and same-epoch point
velocity; absent equipment data remains an acknowledged acceptance limitation.

Both original scenes, seeds, equipment, control gains, fuze limits and existing
hit assertions remain. Additional acceptance must inspect actual damage state
and native consequences, not promote Hit to functional destruction. A target
need not be arbitrarily forced to explode, crash or suffer crew loss. Fullscreen
replays must continue after the encounter and show the authoritative result.

### Native loaded-piece precision follow-up

The current nine-case native CTest selection exposes an additional RED:
`sweptDamageUsesEncounterPositions` has no in-flight weapon, so full native
engagement capture is inactive and its finite-fraction damage query lacks
historical CG/atmosphere. Root pauses native builds/tests and assigns the
capture owner only `World.cpp::{preStep,step}` and the history-access/capture
guards in `WorldEngagements.cpp` / `WorldDamage.cpp`, plus the directly relevant
native fixture. Synchronize decision endpoints even without a weapon; retain
full native-substep capture only when required. Do not substitute current CG,
retain stale histories or change the existing assertion, scene or equipment.
All native execution resumes only after the owner freezes this window.

Root grants the independent reviewer a bounded write window over the optional
`NativePositionSourceV1` sidecar in `flight/FlightModel.h`, its JsbsimModel
implementation/declaration, `WorldDamage.cpp::launchPiece` and the loaded-piece
fixture in `test_mass_change.cpp`. The sidecar uses the model's actual native
ellipsoid and coordinate convention; it adds no primary FlightModel virtual.
Canonical SDK unit helpers avoid redundant multiply/divide conversions.

The generator window additionally includes only numeric round-trip precision
inside `hangar/pieces.py::piece_xml` mass-balance output and its direct test.
Root regenerates the 123 tracked `aircraft/*/pieces/*/*.xml` products; the
recorded generation proof requires every byte outside `<mass_balance>` to stay
identical. The precise output list and hashes are in
`build/architecture-reviews/sim-m03-pieces-generation.json`.

The loaded-piece angular oracle uses the frame translating with the initial
CG, retaining the original .001 Nms bound and separate .001 Ns linear check.
It additionally checks total mass and first moment to 1e-8 kg and 1e-8 m.
The old Earth-frame residual remains diagnostic: Earth-coordinate rounding
multiplied by common bulk momentum must not be mistaken for recoil error.
No spring, scene, physical capacity, control gain or acceptance hit assertion
changes. The temporary pre-assessment JSON trace is removed after exact RNG
and ray reconstruction; archived diagnostic inputs remain outside production.

- [x] Reproduce both failures on the unchanged binary, preserve baseline outputs
  and show the user's requested missile-follow native replay.
- [ ] Capture exact terminal inputs and unchanged RNG evidence; isolate a
  demonstrated cause for each failure with independent failing invariants.
- [ ] Repair the SDK production implementation and obtain independent review.
- [ ] Verify the unchanged positive-hit scenes, negative controls, worker/cadence
  invariants and appropriate current Headless/Release/public/full regression.
- [ ] Update truthful acceptance evidence, inspect the owned staged diff, commit
  on main and release source/build/GUI slots.

### Current frozen handoff, still unaccepted

Main remains `92ae5ed`; nothing is staged or committed. Both complete build
targets pass. Current scoped CTest is Release 112/114 and Headless 112/113, zero
skips; the remaining failures are unchanged showcase Hit and 6 m side-burst
component Hit. Native engagement has 12/12 cases passing, including the added
no-weapon historical-CG invariant. Hangar's 30 tests retain the independently
confirmed two pre-existing C172/Skua generation-drift failures. The initial
single-process 105-case tag run hit its 180 s bound and is not passing evidence;
the later per-case selection excludes only the hidden manual benchmark already
absent from the maintained CTest inventory.

The loaded-piece translating-frame oracle retains .001 Ns/.001 Nms bounds,
separate mass/centroid checks and the nonpassing Earth-frame diagnostic.
The obsolete angular-zone sum is replaced by explicit V1 empty-zone/exact-mass
requirements after a captured RED; original hit/component requirements remain.
Independent exact ray and shielding reconstructions establish the current
model limitations without proving calibrated equipment adequacy or supplying
a fitted correction. Current acceptance cannot be inferred from mechanical
nose loss, crew loss or a fragment Hit.

The current frozen binary has completed fullscreen native and showcase runs,
including actual post-contact flight. All 198 source/test/card/product hashes
still match. All writers are frozen, native/GUI slots are released, and the
owned background processes have exited. Complete current full-suite,
installed-consumer validation, the remaining physical-model gates and commit
remain pending. No external numerical delivery has been inspected or imported.

### Payload component configuration audit

The user extends the SIM-M03 requirement to variant-specific seeker configuration
and replaceable native component adapters/plugins, and asks whether the same
complete route exists for seeker, warhead, propulsion and other payload parts.
At unchanged main `92ae5ed`, root opens a documentation-only audit window over
this plan and `docs/sdk/payload-model-composition-audit.md`. Inspect explicitly
relevant StoreCard parsing, World construction, weapon/damage models, existing
registration interfaces and Hangar generation. Preserve all frozen production
inputs and report current behavior separately from the required future seam.
This window runs no native tests and creates no new public ABI or plugin vtable.
