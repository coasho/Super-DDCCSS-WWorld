# Missile flight validation (SIM-M02)

<!-- AAM qbar observation correction begin -->
AAM qbar observation correction, 2026-11-01: the explicitly approved fixture
now reads the first evaluated native force sample at exactly matched Mach,
alpha, beta and actual fin positions, while completing the original public
step. Initial conditions, commands and the `1e-8` bound are preserved; the
observation boundary changed. Corrected native control cases pass 13/13 in
both headless and Release. Related registered headless gates pass 7/7 and
Release native gates pass 7/7; the installed public consumer passes 1/1 after
refreshing an older staged Python SDK DLL using the existing package target.
The first failed install receipt is retained. All24 AAMs pass the common
six-load pressure comparison in each preset: 3,454 observed native evaluations
per preset, 48 complete observed/unobserved terminal-state comparisons, and no
state difference. The two roster runs took 14.787 and 14.765 seconds. These are
sampling and noninterference checks, not whole-envelope validation. The former
terminal comparison still fails for 23/24 designs per preset and is retained
separately; no force coefficient or flight step was altered to fit it. Normal
`run.ps1` Release headless entry also loaded the current AAM product and
completed finite flight. Independent read-only review found no blocking source
defect; its two provenance/wording findings were closed. Exact commands, current
DLL/product/source hashes, actual counts and raw paths are in the
[AAM receipt](../data/aam-qbar-observation-20261101.json).
This closes the scoped qbar observation defect; it does not claim formal
SIM-M02 closure or supersede the retained rotor/full-regression failures.
<!-- AAM qbar observation correction end -->

<!-- SIM-M02 functional acceptance correction begin -->
Current functional correction, 2026-11-01: SIM-M02 is not formally accepted. All88 designs were regenerated from current geometry/cards; the common engineering matrix has88 engineering-complete-uncalibrated and0 blocked designs, separately from functional acceptance. The retained native repairs cover body quadrature, physical rotor inertia, passive tail damping, mechanical hub work, capture damping with unity cruise gain, complete-cascade response/lookahead, waypoint transit, landing-position capture and momentum-aware terminal hover. Original headless control fixtures: 6/6, 498 assertions passed and0 failed. Current Release original selection: 6/9 cases, 18 failed assertions; pre-observation-correction native AAM: 12/13; installed package consumer: 1 executed. Original scenes, assertions and thresholds are unchanged. RPM-window survival and fall preconditions remain unclosed; the qbar result is superseded by the approved observation correction above; missing exact-model measurements are not the reason. Normal run.ps1 Release entry loaded the current DLLs and exact regenerated UH-1H XML, refused a poisoned exact XML, and flew a real-time orbit-to-hover diagnostic at3840x2160. This is runtime/entrypoint evidence, not whole-fleet physical or complete visual acceptance. All-table integrity checks cover7067 XML tables and3611335 cells; 1165 consistency predicates are not physical tests. Generation/install took1139.453s; the separately timed all88 basic-use pass took812.276s. That basic pass predates final native-only control changes and is not relabeled as a fresh final-DLL run. The earlier full headless attempt was interrupted with753/867 completed records and11 failures; no final full headless/Release pass is claimed. Current source/products, matrix, full-table inventory, raw failures, source review and replay patch: `build/architecture-reviews/sim-m02-functional-resolution-20261101/`.
<!-- SIM-M02 functional acceptance correction end -->

## Shared all-design engineering execution

Force normalization requires matched aerodynamic state as well as matched
actual actuator position. The relation is `F=qbar*S*C(Mach,alpha,beta,...)`.
Two different-density plants generally evolve to different Mach and incidence
over multiple internal native steps. Equal fin positions alone therefore do
not imply equal `F/qbar` after a complete public step. On 2026-11-01 the user
approved correcting the fixture's observation boundary to its first evaluated
native force sample. Initial conditions, commands, complete public-step
propagation and the `1e-8` numerical bound are preserved. This is an explicitly
changed observation boundary, not a claim that the old terminal comparison now
passes. The original 12/13 suite and terminal discrepancy remain in
`build/architecture-reviews/aam-qbar-resolution-20261101/original-before.*`.
The generic [native recorder](../../tools/validation/native_flight_probe/README.md#matched-state-aam-pressure-observation)
retains every internal sample, checks matched first-sample aerodynamic state,
compares all six normalized fin loads for all24 AAMs and verifies observed versus
unobserved terminal state identity. No force law or generated table is fitted
to the former end-step number. The current compact evidence receipt is
`docs/data/aam-qbar-observation-20261101.json`. This bounded sampling result
does not close rotorcraft or whole-program SIM-M02 acceptance.

Both `engineering-complete` and `engineering-complete-uncalibrated` are accepted
completion states. They use identical engineering checks; the latter lacks a
completed optional measurement-calibration record. Missing measurements do not
cause `blocked`. Required but unavailable components remain `required`,
`present=false`, with a failed check and blocked design. Only an explicit project
configuration declaring a role not installed permits `not_applicable` and a
passing check; absent implementation/declaration never implies that state.

The [shared engineering campaign](../hangar-system.md#sim-m02-aerodynamic-engineering-completeness-gate)
applies the same generation, reference, native and installed-consumer checks to
all 88 designs, including the complete 24-AAM subset. It produces per-table
method/source/version/basis and finite grid evidence, rather than using a
missile-specific measurement-data admission rule. Native dynamic validation is
sampling: 210 requested conditions per design, compared at realized states after
one native step. Request/realization deltas are reported. It does not validate
all envelope knots, controls or rates dynamically. Separate static checks cover
every emitted coefficient cell and native XML table, including grid/finite,
boundary and provenance checks. The speed/mass World-reconstruction checks are
repeatability smoke checks, not complete native-state equivalence proofs.
Every sampled row records and independently reloads the actual native rotational
integrator and internal timestep before and after its step. A raw native IC
reset restores Euler, so the consumer restores the generated selector before
advancing simulation. Old rows lacking that evidence cannot certify execution
under the declared numerical configuration. Legacy terrain-strike energy keeps
its fixed-tick incoming state/mass; precise contact initiation uses its separate
substep snapshot, without changing the original impact assertions.

The [current production directory](../../build/architecture-reviews/sim-m02-delivery-20261031/production/index.html)
contains all 88 rows, source/method/product fingerprints and raw generation logs.
Its generation entry point does not launch tests or flight sweeps. All writers
join before the single SDK/Python installation. `campaign-timing.json` records
aggregate production wall time and phase costs. A separate basic-usability pass
loads every design through both the native and installed SDK, observes short
flight/control/reset/release execution and records the actual loaded DLL.
These basic flights are separate from the historical 210-condition sampling
campaign described above; historical sample totals are not current results.

The [delivery report](../../build/architecture-reviews/sim-m02-delivery-20261031/delivery.md)
separates full-table production integrity, current basic execution, internal
consistency predicates, source review and actual clean HEAD-plus-patch application.
The dated basic-use campaign did not run a new long full regression. The current task requires actual functional closure with the original scenarios and assertions. No older full-suite result is relabeled as
a fresh pass. Generic armed unguided contact effects and a distinct
inertial-waypoint turbofan composition now execute native fuzes/warheads.
Terrain initiation requires an explicit opt-in component binding and uses the
same native contact epoch for arming and geometry. A declared configuration
without a terminal seeker does not obtain hidden tracking or target truth;
its inertial waypoint is frozen from an authorized launch designation.
These implementations require fresh common-matrix and complete regression
evidence before any whole-SIM-M02 completion claim.

The production correction uses one reflection-preserving section-polar method
for all designs: symmetric default positive/negative lift limits, simultaneous
smooth clipping, paired symmetric peak updates and reflected negative flap
increments. Authored asymmetric limits remain explicit. It changes physical
table values, including cambered sections and hinged controls; all 88 products
must therefore be regenerated. The contact solver uses the actual translational
and rotational integration coefficients and derivative histories for version2
models, retaining the original legacy branch. No model-name branch or fitted
force multiplier is introduced.

Explicit, versioned fixed internal JSBSim substeps resolve the small-inertia /
high-rate limiting case without changing the SDK outer clock. The default is
one step, preserving existing force-sampling semantics. Derived resolution
follows the same geometry/inertia/damping/stiffness calculation for every design;
the source declaration, selected value and recommendation are recorded. Every
substep preserves first native terrain contact. Six-component control and rate
tables retain full floating-point precision, and explicit quasi-steady zero
alpha-dot derivatives are labeled estimated. Neither numerical resolution nor
a successful force comparison is an equipment calibration.

The earlier native propulsion/reference gates below retain their original
predicates. They must be rerun against current products when reporting current
results; their dated totals are historical evidence, not the current campaign's
test count. Optional exact-model measurement gaps remain in the calibration
ledger and do not set the engineering status.

## Aerodynamic construction and runtime evidence

The single maintained [Hangar system guide](../hangar-system.md) covers shared
VLM construction, JSBSim generation and actual runtime verification. Its
[SIM-M02 aerodynamic engineering-completeness gate](../hangar-system.md#sim-m02-aerodynamic-engineering-completeness-gate)
is the acceptance rule for every design. VLM is the baseline; bounded CFD and
labeled engineering estimates are admissible. Every model uses the same
generic process; there are no per-model waivers. Measured exact-model data is
optional calibration, not a SIM-M02 entry requirement. Solver convergence and
agreement between estimators are not measured equipment accuracy.

Earlier partial table/runtime receipts retain their dated scope; they do not
qualify the current fleet table. The [current status](../status.md) records the
remaining fleet and runtime gates. Historical limited-grid flights and withdrawn
experiments retain their original scope in the guide and raw receipts. This
reference retains the separate release, motor, powered-flight and
software-validation contract.

## Continuation scope, 2026-10-17

Baseline: main `57bc4c78c58f52b952126371319db9879131ff8a`. The working tree was clean.
The user assigned the complete SIM-M02 continuation to Codex; SIM-M01 remains
Claude's. Work stays on main, without another checkout. This supersedes the
earlier first-slice stop point, but does not change the physical acceptance
criteria in [the simulation plan](../architecture/modules/simulation/plan.md#5-functional-roadmap-retained-after-structural-work)
or [MS-7c](../architecture/modules/simulation/adr-30-mission-systems.md).

The goal is reproducible aerodynamic and propulsion validation of generated
missile models used by the SDK. Existing import, metadata and reset work is a
prerequisite, not physical calibration. The first representative remains the
existing AIM-120C-7 game model. Its motor phases, specific impulse, internal mass
distribution and detailed geometry remain assumptions until independently
admitted evidence establishes them. Cross-checking the same assumed input in two
programs verifies their implementation agreement, not the real equipment.

### Write ownership

| Package | Exclusive paths and symbols |
| --- | --- |
| Release ownership prerequisite | `src/simulation/api/World.cpp::World::launchWeapon`, `python/fsim/world.py::World.launch_weapon`, related cases in `tests/compatibility/test_sdk.cpp` and `tests/compatibility/python/test_engagement.py` |
| Motor input prerequisite | `tools/models/hangar/hangar/motorinputs.py`, `storebodies.py::motor_of`, related cases in `tools/models/hangar/tests/test_missiles.py` |
| Independent flight reference | New `tools/validation/missile/` scripts, fixtures and tests; this document |
| Solver reference campaign | Temporary input/output and executable records under `build/architecture-reviews/sim-m02/`; tools and archives under `D:/ENV/physics/` |
| Pulse timing correction | `cmake/JsbsimPatches.cmake` generated `FGRocket::Calculate` coordinate selection; new synthetic `tests/simulation/data/rocket_pulse/` fixtures and case in `tests/simulation/test_vehicle_pool.cpp`; `storebodies.py` pulse table emission; generator-owned `aircraft/stores/{pl15,r37m,r77m}/Engines/*_motor.xml`; source-change notice in `THIRD_PARTY_NOTICES.md` |
| Released-motor ignition correction | `src/simulation/runtime/world/World.cpp::World::{preStep,resetVehicle}` weapon control-cascade eligibility and reset inputs; related public SDK armed/inert release regression in `tests/compatibility/test_sdk.cpp`; `tools/models/hangar/hangar/storestages.py::_burn` explicit actuator request and related isolated test in `tools/models/hangar/tests/test_missiles.py`; existing `engagement_native` acceptance. Added after that unchanged end-to-end test reproduced unlit released AIM-120C motors at the same baseline HEAD |
| Final integration | Related standard-library validation registrations in `cmake/ArchitectureChecks.cmake`, `tools/validation/testing/scopes.toml`, and links/status in `docs/{hangar,status}.md`, `docs/architecture/modules/simulation/ms-work-allocation.md` after reviewing the final implementation |

No radar source/card, guidance/warhead calibration or public ABI layout is in
this window. Only the root agent runs native SDK configure/build/test commands;
build inputs are frozen while those commands run. Offline reference solvers run
with bounded threads. Every command uses `D:/ENV/tmp` for temporary files.

On 2026-10-18 the user closed this implementation window for delivery and
assigned the three numerical investigations (mesh sensitivity, panel closure
and solver convergence) to Claude in a separate research directory. Codex
performs independent acceptance. That handoff contains only those numerical
problems and their required inputs; it authorizes no SDK or equipment-parameter
changes and does not close the remaining physical acceptance criteria.

### Execution and acceptance

1. Add failing ownership and motor-input regressions, then correct the native
   and Python release boundary and unsafe derived motor values. Preserve valid
   current generated outputs and local numeric launcher IDs.
2. Pin actual RocketPy versions, executable/archive hashes, licenses and support
   requirements for software comparison. CFD is optional and bounded by the
   whole-campaign timing report; no historical solver run is a prerequisite.
3. Implement a repeatable comparison of the generated native missile's powered
   flight and RocketPy with matched declared inputs. Record trajectory, thrust,
   mass and residuals with explicit tolerances. Invalid/missing dependencies,
   stale models and incomplete comparisons fail explicitly.
4. Generate every design through the same shared pipeline from current geometry
   and card. Use VLM first; bounded CFD and labeled engineering estimates are
   admissible where a table needs them. Every table records method, source or
   derivation, envelope coverage and measured/estimated/simulated basis.
5. Run the generic completeness audit, aggregate generation/validation timing report,
   native flight/performance checks and installed-consumer checks. Record
   measured calibration separately; a missing measured dataset alone is not a
   SIM-M02 block.

Reference reports must distinguish software agreement, numerical convergence,
source admission and real-equipment validation. No passing comparison is allowed
to upgrade any other category implicitly. The span discrepancy recorded in
allocation section 10.6 remains open until the exact source pages are reconciled.

The fleet comparison found the consumed-mass table could skip the complete 2 N
coast in three declared pulse motors: its mass interval is smaller than one
preceding high-thrust step. The correction adds the private versioned table name
`fsim/ignition-time-thrust-v1`, evaluated against the existing native engine's
ignition-relative burn time. Other table names retain consumed-mass semantics.
No new member, virtual method or public signature is added. The generator selects
this form only for declarations with a zero-thrust phase, retaining their current
2 N coast and assumed grain/impulse inputs. It emits an explicit zero after the
last pulse. These generated pulse tables require the matching patched native
runtime; the comparison and installed consumers verify that pairing. Reset,
delayed ignition and ignition persistence through coast are
native fixture requirements. This is a schedule correction, not motor calibration.
The clock retains upstream semantics under starvation: it advances only during
non-starved burning, so it is active burn time rather than unconditional wall
time. The marked fixture also checks fuel-unavailable shutdown and refilled reset.

## Results

Final software verification: 2026-10-18, on the same main baseline with the
feature changes below. Historical aerodynamic research retains its separate
evidence; it is not a SIM-M02 entry gate.

The dated software comparison below passed across all 24 authored AAM models.
The current complete fleet audit and aggregate runtime evidence are linked at the
top of this document. The former two required-component gaps preceded the
generic armed-store composition repair; they are historical results, not the
current matrix. Whole-program acceptance still requires the unchanged full
regression gates and independent review. Exact-model measured data and real
motor curves are optional calibration evidence; they are
not SIM-M02 entry requirements. The changes are based on the baseline above and
are recorded independently of the historical results in allocation sections
10.1-10.6.

| Delivered behavior | Current evidence |
| --- | --- |
| Reject a launcher object from another native World before dispatch | Native and Python regressions first reproduced release/state changes; local numeric IDs and wrappers of the same native World still work |
| Reject unsafe derived motor inputs | Overflow, nonpositive remaining structure/inertia, grain installation and collapsed raw/serialized consumed-mass knots fail before XML generation; focused generator selection passed 26/26 in 0.582 s, including explicit Hangar burn ignition and refusal cleanup |
| Preserve pulse coast timing | The synthetic native regression first failed 145 assertions. The marked/unmarked table fixture now covers 60/120 Hz, delayed ignition, coast, terminal zero, starvation, reset and fresh replay |
| Ignite actual released weapons | The unchanged end-to-end engagement test exposed an aircraft neutral command overwriting launch throttle. The public armed/inert regression first failed 32 assertions; preserving weapon inputs and their requested reset state restores native ignition without load-time burning. The three related release/eligibility/engagement cases pass, including worker determinism, powered/coast and inert jettison |
| Generate the timing correction through Hangar | Only PL-15, R-37M and R-77M motor XML changed. A census of 53 generated store models (101 files) found no other generated differences |
| Compare powered flight independently | 48/48 vacuum/air cases passed, comprising 96 native runs and 16,592 common time samples across all 24 AAM models |
| Preserve public integration | Final headless selection passed 22/22 in 64.77 s, including the installed C++/C99/Python consumer (25.90 s). Separate final reset/authority checks passed 2/2 in 1.16 s. Installed model hashes and loaded-library provenance match |
| Verify the Release configuration | Final Release build and related selection passed 24/24 in 39.97 s, including the requested engine-state reversal on reset and the existing authority/task/C2 preservation case |

The generator selection includes current authored-design and generated-output
controls. The related CTest selection covers the three native rocket cases, both
new release-ownership cases, armed/inert release and reset, Python engagement/stores/weapons modules, C ABI and
SDK boundary checks, reference guards, architecture checks and contract/scope
checkers. These are current focused checks, not a full-suite or Viewer claim.

The fleet's worst residuals over both native timesteps are **0.052202 kg** mass,
**1.285336 m/s** axial speed and **4.157134 m** altitude; maximum vacuum-impulse
discrepancy is **0.184871 percent**. Every case also passed the declared timestep
sensitivity condition. These values describe the matched central-gravity
campaign below, under assumed game inputs. They do not establish real-equipment
accuracy, Earth-trajectory agreement or control-response fidelity.

Raw logs, full trajectories and explicitly listed source/library SHA-256 values
are retained under `build/architecture-reviews/sim-m02/`: `fleet-flight-final.json`,
`fleet-flight-final.log`, `generator-final.log`, `final-headless-tests.log`,
`released-ignition-green.log`, `reset-authority-final.log`, `burn-native.log`,
`release-final-build.log` and `release-final-tests.log`. The two actual Hangar
burn checks (AIM-120C and PL-15) also passed. Earlier diagnostic failures remain in separate reports;
they are not included in the passing fleet result. The report hashes actual
loaded native DLLs against the explicitly requested build and records the Python
interpreter, RocketPy/NumPy/SciPy versions, selected numerical sources, authored
design/card, production generated files and temporary instrumented files. This
is a declared provenance scope, not a complete dependency closure.

### Powered-flight reference conditions (frozen before comparison)

The initial diagnostic comparison is axial unguided flight: an already airborne, vertically
oriented body at 8,000 m and 250 m/s, equator, no wind, terrain or guidance.
Cases use vacuum and a spatially constant atmosphere sampled from the native
initial state. The native World keeps its own gravity and rotating ellipsoid;
RocketPy uses the initial local gravity with its flat local flight frame. These
approximations bound this short axial comparison; lateral/control responses are
not validated by it. Both engines use the same declared grain, vacuum impulse,
nozzle area, ignition buildup and axial drag table. RocketPy's grain burns
radially at fixed length, matching the generated JSBSim cylindrical grain.

RocketPy receives axial drag only; its optional fin/nose stability model is not
constructed in this reference. Its static-margin warnings remain in the raw log.
The comparison establishes no aerodynamic stability or lateral-response result.

At each complete common sample through burnout plus 0.25 s, absolute residual
limits are 0.10 kg mass, 2.0 m/s axial speed and 10.0 m altitude. Total vacuum
impulse must agree within 0.5 percent. Native timesteps are 1/240 and 1/480 s;
both must pass, and the finer run's maximum speed discrepancy may exceed the
coarse run's by at most 0.1 m/s. No channel may be empty, truncated, nonfinite,
misaligned or silently interpolated across missing data. These limits test the
implementation under declared game inputs; they do not admit a real motor.

### Matched central-gravity campaign (frozen before its first execution)

The broader diagnostic run exposed the first comparison's environment limits on
longer burns, separately from the pulse-schedule bug. A second explicitly selected
case, `--spherical-reference --native-hz 480 960`, matches the environment in both
programs: a nonrotating sphere of radius 6,378,137 m, central gravitational
parameter 398,600,441,800,000 m3/s2 and zero J2. The temporary native model adds a
standard JSBSim `planet` element; RocketPy uses `mu / (radius + altitude)^2`.
Production World/Earth configuration and production assets are unchanged. This
isolates the axial comparison from Earth rotation and mismatched gravity rather
than admitting a real trajectory. The same residual/impulse limits and step
sensitivity condition apply; no limit is widened. Both constant atmospheres and
the complete declared motor interval are retained. Exact instrumentation and
input hashes are included in the report. The earlier rotating-Earth results,
including their failures, remain separate evidence.

### Aerodynamic reference outcome

See the [retained aerodynamic research cases](../hangar-system.md#11-historical-evidence-and-retained-lessons)
for numerical outcomes, failed controls and their evidence roots. The
[method](../hangar-system.md#5-solver-research-and-controlled-numerical-changes)
and [physical-admission boundaries](../hangar-system.md#8-sampling-interpolation-spatial-accuracy-and-physical-scope)
are maintained there; successful propulsion comparison does not close them.

### Reproducible entrypoints

Run from the repository root after the project-local setup/build in the
[Hangar system guide](../hangar-system.md#2-whole-system-setup-build-run-and-stop).
The reference safeguard uses only Python's standard library. The matched
powered-flight comparison additionally needs the pinned optional RocketPy
environment; a software-only SDK build does not require offline CFD tools.

```powershell
.\run.ps1 -Mode python -Preset ucrt64-headless -Arguments @('tools/validation/missile/test_reference.py')

# One model, under the matched reference conditions.
$OneModel = @('tools/validation/missile/crosscheck.py', '--model', 'aim120c',
  '--spherical-reference', '--native-hz', '480', '960',
  '--native-bin', 'build/ucrt64-headless/bin',
  '--output', 'build/architecture-reviews/sim-m02/aim120c-flight.json')
.\run.ps1 -Mode python -Preset ucrt64-headless -Arguments $OneModel

# All authored AAMs; the emitted card is checked against its exact design.
$Models = @(Get-ChildItem -LiteralPath assets/stores -Filter '*.json' | ForEach-Object {
  $Card = Get-Content -LiteralPath $_.FullName -Raw | ConvertFrom-Json
  if ($Card.class -eq 'aam') { '--model'; $Card.model.Substring(7) }
})
$Fleet = @('tools/validation/missile/crosscheck.py') + $Models + @(
  '--spherical-reference', '--native-hz', '480', '960',
  '--native-bin', 'build/ucrt64-headless/bin',
  '--output', 'build/architecture-reviews/sim-m02/fleet-flight.json')
.\run.ps1 -Mode python -Preset ucrt64-headless -Arguments $Fleet
```

The flight CLI invalidates an earlier passing report before starting and records
ordinary native/numerical failures explicitly. Missing runtime dependencies,
different native DLLs, stale generated assets, ignored atmosphere/gravity
overrides and incomplete/nonfinite histories cannot produce a passing report.

### Remaining SIM-M02 acceptance

SIM-M02 is accepted only when the generic gate has been run over the whole
declared roster and reports the following for every design, without per-model
waivers:

- **Structural completeness:** six-degree-of-freedom state, mass/inertia/CG,
  aero, propulsion, control/actuator, seeker/guidance/fuze/damage and
  release/reset components are present; not-applicable entries are explicit.
- **Theoretical complexity completeness:** all six aero components and the
  derivatives, cross-coupling, rate and alpha-dot terms required by the declared
  envelope; compressibility and control terms are not omitted.
- **Factual computation completeness:** every table is regenerated from the
  current geometry and card through the shared pipeline, covers the declared
  grid, contains finite values, has no silent missing row or column, and records
  interpolation/extrapolation bounds.
- **Provenance:** every table records method, public source or derivation,
  version and measured/estimated/simulated basis. AI knowledge is not a source;
  public papers, government publications and open-source tools are cited when
  used. Leak or classified inputs are excluded.
- **Generic validation:** one reference suite and the same acceptance checks run
  for every design. Public datasets and VLM/CFD comparisons test method
  behavior; they are not a per-model data-entry requirement.
- **Runtime:** report aggregate generation/validation wall time and phase costs,
  including all reporting groups. Twenty minutes is a performance target, not
  a hard admission failure. Bound subprocess/CFD work explicitly; disclose any
  operational timeout and fallback. Never hide total cost by splitting batches.
- **Installed-product evidence:** native flight/performance and installed
  consumer checks load the generated product, not a stale asset.
- **Fleet status:** every design is `engineering-complete`,
  `engineering-complete-uncalibrated` or `blocked` for a missing required
  dimension/component/table or an explicit operational timeout. Missing measured data alone
  is not a block.

Measured calibration is a separate optional label and does not gate SIM-M02.
The software fixes and powered-flight agreement do not by themselves close the
completeness audit.

Generator/schema and native runtime checks establish software consistency,
separately from the
[SIM-M02 engineering-completeness gate](../hangar-system.md#sim-m02-aerodynamic-engineering-completeness-gate).
Measured calibration remains a separate optional label.

### Numerical research provenance

Retained numerical lessons are documented in the
[Hangar consolidated lessons](../hangar-system.md#consolidated-lessons).
The original coarse negative-drag failure remains visible; delivery review and
model acceptance have separate meanings. Dated raw receipts preserve the
original evidence; they do not reinstate the retired solver acceptance path.

#### Regeneration and complete-suite follow-up, 2026-10-21/22

Historical software evidence is under
`build/architecture-reviews/numerics-integration-20261021/final/`: all 53 stores
regenerated, Release 910/910 and Headless 836/836 passed without skips. Their
inventories matched JUnit exactly; 1,057 frozen inputs and four sets of 813
runtime files retained their identities. Those dated totals are not current
CFD admission or the current fleet rerun. The former ten aircraft ceiling and
damping failures are addressed by the current Hangar work; consult
[status](../status.md) for its measured outcome.

The reusable software corrections remain in the core and their regressions:

- Physical reset clears crew incapacitation while preserving authority, tasks
  and MA bindings. The real-crew-loss fixture checks the original trajectory.
- Cached atmospheric property bindings sample fresh values and validity on
  every boundary, retry missing bindings and invalidate on reset. The original
  allocation and control-digest checks remain intact.
- Source attribution uses declared categories without changing physical
  parameter objects. Motor fixtures supply the current card boundary while
  preserving the existing validation assertions.
- Live fixtures publishing the same fixed world share a CTest resource lock;
  their original scenes, assertions and timeouts remain unchanged.
- RCS products bind source/model/recipe bytes. All 36 tables passed their
  original verification after three recomputations; this establishes byte
  identity and numerical consistency, not electromagnetic accuracy.

The pre-consolidation document and its retained-lesson checklist are archived
as `missile-validation-before-consolidation-98.md` and
`document-consolidation-98.json` in the Hangar evidence root. Original failed
receipts and parameter hashes remain there or in the dated integration root.

### Aerodynamic method and source controls

The [2026-10-22 research case](../hangar-system.md#2026-10-22-conservation-gradients-thermodynamics-and-generated-models)
retains the wall-force defect, conservative wall/gradient corrections, exact
disabled replays, sphere/enthalpy/AUSM/face-EOS controls and remaining accuracy
failures. Their numerical/operator proofs do not admit equipment coefficients.


<!-- SIM-M02 typed-source and replay contracts -->
An authored engine fixes propulsion applicability. A versioned air-breathing
store source with a missing, unavailable, not-applicable or incompatible
propulsion selection is rejected; generation cannot silently discard its
engine. Single-turbofan v1 rejects mixed rocket hardware. Legacy numerical
compatibility and explicit new-type contracts together cover every store;
rocket propellant and fed air-breathing fuel retain the same canonical mass
checks. No physical flight scenario, assertion or tolerance is replaced.

Full verification records every discovered headless/Release CTest case and
nested unittest method separately. An ordinary source export has no Git
metadata. Its original Git-only skip remains visible; the unchanged original
fixture may execute once on main only after its project code and declared input
bytes match the tested export. External execution is recorded separately and
never counted as multiple CTest executions. Uncovered skips remain failures.
The test-reference evaluator may reuse immutable arrays and per-state
interpolation work only when exact recorded-state equivalence is demonstrated.


<!-- SIM-M02 current generated verification begin -->
Current production delivery: 88 engineering-complete-uncalibrated, 0 engineering-complete and 0 blocked designs. All 31 fixed wings, four rotorcraft and 53 stores (including all24 AAMs) use the same source-driven method and component rules. Aggregate production generation/install: 1045.836 seconds; current basic usability: 813.329 seconds. Current basic execution covers 88 native and 88 installed consumers, 2112 short-flight observations and 10 core/package fixtures (0 failed, 0 skipped). Production integrity inventories 16776 logical/serialized table entries and 3611335 XML coefficient cells. These are separately counted from document consistency predicates. This dated basic-use result does not establish formal SIM-M02 acceptance; no full regression or full-envelope dynamic coverage is claimed. Exact-model measurement calibration remains optional. Evidence: `build/architecture-reviews/sim-m02-delivery-20261031/`.
<!-- SIM-M02 current generated verification end -->
