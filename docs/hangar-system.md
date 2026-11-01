# Hangar: one VLM airframe workflow

<!-- SIM-M02 final closure begin -->
SIM-M02 is **closed for the user-authorized 31 fixed-wing and 53 store functional scope** on 2026-11-01. All 4 rotor functional populations are explicitly deferred at lowest priority, not passed. The common engineering matrix still covers all 88 designs: **88 engineering-complete-uncalibrated, 0 blocked**. Missing equipment measurements are optional calibration limits, not missing components. Required but unavailable components remain required, absent, failed and blocked; the native armed-store implementations do not use an inert-release waiver.

The current clean-source execution covers all 840 selected headless CTest cases, followed by the exact 2 approved repair rechecks (79 nested methods). The first headless run's 2 failures are retained. Independent current Release execution passes **914/914**, with the exact 28 authorized rotor-only cases excluded from both preset inventories. All 16 fleet shards retain all 31 fixed wings. Additional current evidence is 24/24 matched-state AAM observations (3,454 native evaluations), 48/48 powered axial comparisons at both 480/960 Hz, and 84 native plus 84 installed consumers with 2,016 short-flight observations, 106 released bodies and 10/10 component/package fixtures. These distinct populations are not added into a fabricated single test total.

Exhaustive static checks cover 7,067 native XML tables, 3,611,335 coefficient cells, 522,331 axis knots and 16,776 inventory entries. Every table retains method/source/version/basis, finite-value/grid and interpolation/extrapolation checks. The 1,165 applicability/check/status consistency predicates are **not physical tests**. Native short-flight, pressure and axial checks are sampling/method comparisons, not full-envelope equipment validation. CFD tables: 0; newly measured aerodynamic tables: 0.

Current-source all-design generation, mandatory-component recovery, installation, basic use and full-table auditing take **2,845.286 seconds in total**. This exceeds the 20-minute aspiration; no under-20-minute or historical aggregate speedup is claimed. The dominant measured job is C-17A at 1,290.243 seconds, including 926.975 seconds for fresh damage/detached models. Fresh preparation avoids repeating cold generation in each regression fixture, bounded immutable caches preserve exact inputs, and both native integration resolutions reuse one identical-state numerical reference per atmosphere/design. Complete and interrupted regression/attempt times and end-to-end elapsed time are separately recorded, not hidden by batch partitions.

The root normal `run.ps1` entry and the clean-source entry each complete the unchanged 40-second native encounter at 1x, fullscreen 3840x2160. The SDK produces launch, acquisition, contact detonation and damage; rendered captures preserve those outcomes. The scene executable statically links `fsim::core` and was rebuilt from current source; actual JSBSim DLL identity is recorded separately from the root public Python SDK's actual libfsim/JSBSim DLL identities. All 439 generated products match the accepted manifest in source, staged data and the installed package. This is concrete entrypoint and package evidence, without claiming all-design visual or equipment calibration acceptance.

The [final receipt](data/sim-m02-final-closure-20261101.json), [88-design matrix](data/sim-m02-engineering-completeness.csv), and `build/architecture-reviews/sim-m02-final-closure-20261101/delivery.md` identify the exact commands, original failures, independent reviews, current counts, source/product hashes, complete installed SDK, one HEAD-based exact-byte patch and successful patch replay. This dated closure supersedes the historical SIM-M02 nonacceptance entries below; those original failures remain available as history. Rotor fitness remains in its separate authorized deferral ledger.
<!-- SIM-M02 final closure end -->

<!-- SIM-M02 authorized closure boundary begin -->
The superseding 2026-11-01 user instruction defers **all rotor functional
issues at lowest priority**. This SIM-M02 functional closure covers 31 fixed-wing
and 53 store designs. The 4 rotor designs retain their all-design engineering
component/table records; their flight fitness is deferred, not passed.
The [versioned deferred-scenario ledger](data/sim-m02-deferred-rotor-issues-20261101.json) names 28 rotor-only native
flight cases and 4 rotor-only Python methods. Mixed fixtures retain fixed-wing
conditions, World clock advances, assertions and thresholds through the
explicit tests-only `sim-m02-nonrotor-v1` population scope. All 16 fleet shards
remain and must collectively cover exactly 31 maintained fixed wings. Generic
API/lifecycle/layout/mathematical checks may include rotor types without
claiming rotor flight acceptance. Default test behavior remains all-family.
All 88 source/component/table applicability rules remain unchanged; no required
role becomes not_applicable. Closure execution reconciles the complete CTest
inventory minus the exact deferred native cases, separately accounts for
nested deferred methods/platform skips, and binds current source/binaries.
Use `tools/validation/aerodynamics/sim_m02_closure.py --preset PRESET --output
OUTPUT --powershell POWERSHELL_PATH --production PRODUCTION --git-source-root MAIN_CHECKOUT`
after building the preset and forcing all-design generation. The generation
preflight binds current authored/optional profile inputs, runtime products and
intermediate cache keys, and rejects unequal bin/tests/Python DLL copies.
HEAD-tracked historical output inputs and all explicitly consumed profile files
remain immutable during regression; other new derived reports/caches are
recorded separately from source files.
<!-- SIM-M02 authorized closure boundary end -->

<!-- SIM-M02 functional acceptance correction begin -->
Historical pre-deferral functional correction, 2026-11-01: SIM-M02 is not formally accepted. All88 designs were regenerated from current geometry/cards; the common engineering matrix has88 engineering-complete-uncalibrated and0 blocked designs, separately from functional acceptance. The retained native repairs cover body quadrature, physical rotor inertia, passive tail damping, mechanical hub work, capture damping with unity cruise gain, complete-cascade response/lookahead, waypoint transit, landing-position capture and momentum-aware terminal hover. Original headless control fixtures: 6/6, 498 assertions passed and0 failed. Current Release original selection: 6/9 cases, 18 failed assertions; native AAM: 12/13; installed package consumer: 1 executed. Original scenes, assertions and thresholds are unchanged. RPM-window survival, fall preconditions and the qbar matched-state premise remain unclosed; missing exact-model measurements are not the reason. Normal run.ps1 Release entry loaded the current DLLs and exact regenerated UH-1H XML, refused a poisoned exact XML, and flew a real-time orbit-to-hover diagnostic at3840x2160. This is runtime/entrypoint evidence, not whole-fleet physical or complete visual acceptance. All-table integrity checks cover7067 XML tables and3611335 cells; 1165 consistency predicates are not physical tests. Generation/install took1139.453s; the separately timed all88 basic-use pass took812.276s. That basic pass predates final native-only control changes and is not relabeled as a fresh final-DLL run. The earlier full headless attempt was interrupted with753/867 completed records and11 failures; no final full headless/Release pass is claimed. Current source/products, matrix, full-table inventory, raw failures, source review and replay patch: `build/architecture-reviews/sim-m02-functional-resolution-20261101/`.
<!-- SIM-M02 functional acceptance correction end -->

## SIM-M02 aerodynamic engineering-completeness gate

The common body integrator uses nodal trapezoidal weights for distributed
potential/crossflow forces, friction and their first moments. End nodes receive
half their adjacent interval; using `gradient(x)` as quadrature weights
overcounts a constant distribution. This changes the shared generated tables
and requires fresh generation for every design. Source/method hashes identify
the implementation without changing the public table or SDK schemas.

Rotor generation receipts also bind `hover.toml` and, when declared,
`tail-damping.json`. Missing or stale mandatory source-derived native
components fail the common structural check. They cannot be relabeled
not-applicable because generation or implementation is unavailable.

The executable common gate is `hangar engineering campaign`. It enumerates the
authored roster and requires exactly 31 fixed-wing designs, four rotorcraft and
53 stores. A selection uses the identical rules; the full gate requires all 88.
The measured-calibration catalog is not modified by this command.

Generation always recomputes the aerodynamic grids. Geometry fitting may reuse
an identical signed-distance query only within the lifetime of its immutable
native scene: the cache key is all 24 bytes of the float64 point, with no
rounding or spatial approximation. Distance and material outputs are retained
together; FIFO capacity is 65,536 points per scene, and scans above 4,096 points
bypass it. Small inputs are snapshotted, and evaluation/closure on the same
scene serialize. No generated product or source cache substitutes for force
generation. Per-design receipts report native query counts; hits include exact
duplicates within one request, and totals include closed scenes only.

`--scheduling-profile` optionally accepts the versioned
`fsim.sim-m02.scheduling-profile.v1` observed wall-cost map. Its absence keeps
the original ordering. It changes only longest-first ordering within the same
AAM, fixed-wing, other-store and rotorcraft priority cohorts. The complete
roster, all numerical inputs and all checks remain identical. The supplied
profile is explicit, covers all 88 designs, records its source run/hardware,
and is hashed in timing and batch evidence. It contains no physical fit values.

`--geometry-threads` reserves threads only for the fresh damage/geometry phase;
its default is one. The generation worker count times this reservation cannot
exceed 16. Table/BLAS work remains one thread per worker and native consumer
execution remains serial. An eight-worker/two-thread geometry run therefore
keeps the same maximum 16-thread reservation as a sixteen-worker/one-thread
run. Nested phase reservations cannot enlarge their parent and restore the
previous policy on exit. Common native union queries use bounded inline
temporary storage for small operator graphs, preserving every bound, child
evaluation and original fit predicate.

### Control sizing and reproducible functional execution

The versioned `fsim.store-yaw-control-idealizations.v1` control-design method
retains the nominal shared-store table prediction and adds an alternative
attached potential-pressure recovery calculated by the same body-force
integrator. It preserves finite endpoints, geometry, CG moment transfer and
nominal shared-table fin loads. The alternative holds the PG induced flow at
its explicitly recorded recovery Mach. Recovery Mach is limited to the common
subsonic PG domain; above that domain the correction is held as an explicitly
estimated continuation while the nominal full-Mach table remains scheduled.
The more adverse station derivative sizes the controller across its declared
loadouts, including store multiplicity. Native nominal force coefficients
are unchanged by selecting this control-design alternative.

This is an estimated set of two design idealizations, not a measured bound on
all real separation states. [NACA TR184](https://ntrs.nasa.gov/citations/19930091249),
printed pp15-17, distinguishes ideal flow and real rear recovery;
[NASA CR145251](https://ntrs.nasa.gov/citations/19770023140), printed pp18-19,
describes the nominal separation treatment. Generation receipts record the
distinct evaluated design conditions, both derivatives, selection, winning
store/count per station, assumptions, source hashes and method version. No
per-model scale or modified reference datum is used.

Generation input manifests bind the actual geometry, referenced airfoil files,
calibration/identification, stored flight/performance/ground inputs, payload and
station cards, recursively carried-store sources, and public method data.
Optional absence is recorded, so adding an input invalidates reuse. Table cache
publication is atomic; a concurrent reader sees a complete table. Production
still forces every design's table generation. Subsequent regression may reuse
only current-key generated intermediate tables, while native consumers load
the regenerated runtime XML/carriage products. Runtime publication refreshes
staged DLLs from the declared source directories and re-resolves transitive
imports. Functional preflight requires equal native core hashes in bin, tests
and Python, binds all current products/inputs, and rejects source, runtime or
CMake/compile-command drift during execution.

The current authorized functional workflow separates all 88 production from
84-design execution. First prepare declared source-bound components with
`tools/validation/aerodynamics/engineering_components.py --output BUILD_OUTPUT`.
It visits declarations, not design names. A stale required damping reference
uses the existing native bootstrap trim and identification, preserves the
acquisition XML/engine files and controller identification, and requires an
airborne, noncrashed, nondiverged, unsupported-by-contact operating point. This
is simulated component preparation, not rotor manoeuvre or fitness acceptance.
Then force `hangar engineering production`, run `engineering_basic.py` with
`--functional-scope sim-m02-nonrotor-v1 --consumer-execution batched-v1`, audit
with `engineering_production_delivery.py`, and execute both complete preset
inventories through `sim_m02_closure.py --production PRODUCTION ...`. The
unrestricted all-family combined campaign remains available for future rotor
functional work; its historical results are not the current closure evidence.

The user-approved signed-control-load correction preserves all 16 original
alpha/beta/deflection cases and the 1e-10 line. A controlled-minus-neutral drag
increment can be negative when a fin unloads; it is not an independent passive
body. The check reconciles its force projection with complete-model drag
change including separately emitted Mach/ground circulation terms. The
existing whole-model nonpositive-work check remains unchanged. No force clamp,
geometry change or invented drag is used to satisfy the withdrawn incremental
passivity predicate; its original failure remains in the full-run evidence.

The source freeze separately records fixture-built Cargo target outputs,
including copied runtime DLL hashes checked against the current preset. Only
non-input, untracked outputs under the documented target path are classified
as derived. Maintained Rust sources/configuration and any tracked target files
remain protected. Test execution, internal consistency predicates and source
review are reported separately.

### Completion and component applicability

`engineering-complete` and `engineering-complete-uncalibrated` both complete
the engineering gate. The latter means the same engineering checks passed
without a completed optional measurement-calibration record; it is an accepted
completion state, not a blocker. `engineering-complete` additionally records
completed optional calibration. A partial historical calibration ledger does
not automatically establish that claim. `blocked` means an actual missing or
invalid required dimension/component/table, or an explicit operational generation/validation timeout. The 20-minute target is reported for the whole campaign and is not a hard admission failure.
Missing measurement data by itself never causes `blocked`.

Applicability is a project design declaration, independent of implementation
capacity. The shared component/check truth table is:

| Explicit source declaration | Applicability | Present | Check |
|---|---|---|---|
| Installed role, implementation available | required | true | pass |
| Installed role, implementation unavailable | required | false | fail |
| Project configuration explicitly does not install the role | not_applicable | false | pass |

The last row requires the explicit versioned project component declaration and
reason. A missing provider, missing declaration, or wording such as "inert
release only" cannot create that configuration. There is no operator waiver or
per-design solver branch. Every published row checks applicability/presence
against its check outcome. Armed roles remain required even while unavailable.
The native composition now supports unguided armed contact effects and a
no-terminal-seeker inertial-waypoint turbofan design. An explicit versioned
terrain-contact binding selects native STRUCTURE contact; its absence preserves
legacy aircraft-contact-only initiation. Generation never invents applicability.

```powershell
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
& .\tools\launch\fsim.ps1 --preset ucrt64-release build --target fsim python_package
& .\run.ps1 -Mode hangar -Arguments @('engineering', 'campaign',
  '--output', 'build/architecture-reviews/sim-m02-replay-20261031/final',
  '--generation-workers', '16',
  '--prefix', 'D:/ENV/sim-m02-replay-20261031')
```

The command declares reporting groups of eight, ordered AAM, fixed-wing, other
stores, then rotorcraft. One bounded queue (default eight, explicitly sixteen
in the clean replay) forces generation from current geometry/card, with one
nested native thread per design. Every generation worker joins before one serial SDK and
Python installation. Native and installed short flights then run serially.
The installed process also performs 210 requested native sampling conditions
per design (two altitudes, three Mach fractions, seven alpha and five beta
values). These are sampling at realized states after a native step, not
full-angle or full-envelope dynamic validation. Request/realization deltas are
recorded. All emitted array cells and XML tables receive static finite, shape,
grid, domain/boundary and source/method/version/basis checks.

`campaign-timing.json` reports the entire generation, installation, validation
and publication wall time. Reporting groups do not divide or excuse aggregate
runtime. The comparison baseline is 2,151.704 seconds, including 1,312.165 seconds
generation and 839.539 seconds serial installation/validation. Twenty minutes is
a disclosed performance target, no longer a hard acceptance failure. The
operational whole-campaign process limit is 7,200 seconds; timed-out child trees
are terminated and the parent restores any XML modified by the negative-load
probe before releasing the serial slot. No quick grid or CFD branch is used.
The installed consumer preloads the installed SDK DLL so its own regenerated
`share/flightsim` tree is authoritative. Reconstruction checks observe speed
and mass repeatability; they do not prove equality of every native subsystem.

Each receipt-relative test reference `engineering-aero-v1.json` contains the complete coefficient
arrays, derived body CA/CN/CY arrays, table inventory, source/derivation, basis,
version, input/generator hashes, interpolation and continuation boundaries.
It is stored in the evidence directory beside `generation.json` and verified
by `reference_bundle` name/hash. It is not an SDK runtime aircraft asset.
Native physics loads installed XML, carriage, damage and detached-body products;
the external reference supplies expected test values only. This keeps the
existing store-builder runtime-file contract intact. Damage data and detached
models are regenerated from current geometry and included in product hashes.
Carriage tables receive exact cell/grid comparisons with the shared reference.
`matrix.json`, `matrix.csv` and `index.html` summarize every design; each row
links generation, native and installed receipts and exact command logs. Table
cache hits cannot satisfy the campaign. Body-only designs retain the same table
interface with zero lifting panels and explicit estimated body aerodynamics.

`engineering_clean.py` owns one serial clean replay through configure/build,
forced generation, complete headless/Release CTest runs and full-table reloads.
It requires a fresh installed prefix. Its `clean-pipeline.json` reports the
entire tool wall, including full regression, separately from campaign timing.
The historical baseline omitted fresh damage/piece generation and full
regression; that workload difference must accompany every runtime comparison.
CTest preserves full method output and reconciles exact Hangar loader/execution
IDs. CPU cases retain reviewed parallelism; shared-cache writers, CMake fixtures
and GPU cases are serial. The three measured heavy meshing classes reserve
eight CPU slots and eight native threads; ordinary cases retain one thread.

Wake interpolation uses exact cache keys. The old near-angle cache could
reuse the previous condition's operator and make nonlinear results depend on
traversal order. Principal alpha/beta inputs follow the declared table domain.
The shared solver preserves the original wake-angle formula, including its
behavior outside principal angles, and keeps one bounded velocity/rate response
memo, reusing identical tensor contractions.
No lattice density, knot, nonlinear solve, iteration budget or acceptance
threshold is removed or relaxed. Strict cache invalidation and reversed-order
six-axis tests use exact array equality. Historical table deltas from correcting
the old cache are reported separately; they are not called numerical equivalence.

The common writer retains double precision, six coefficients for every control
and rotation channel, and all six alpha-dot entries. The four additional
alpha-dot derivatives are explicit quasi-steady zero estimates. Tail downwash
lag requires a real upstream wing. Mach knots include the declared endpoint;
transonic interpolation can retain the 1.2 engineering anchor outside a shorter
declared domain. Angle/control/Mach/height lookups clamp to their end knots;
rotation and alpha-dot derivatives continue linearly. This does not claim a
fully coupled unsteady tensor model.

Numerical integration is also generated through `integration.py`. Its default
is the identity (one internal step), preserving existing per-tick force sampling.
An explicit `[integration] version = 1, mode = "derived"` declaration selects
fixed internal JSBSim substeps from dry inertia and aerodynamic damping/stiffness
bounds at the declared maximum Mach and ISA sea-level density. The same
derivation and checks apply to every design; there is no model-name branch.
The bundle records both selected and recommended resolution. The existing
120 Hz SDK clock and public interfaces are unchanged. The
constant internal dt is set **before** loading FCS filters; runtime dt switching
would invalidate their coefficients and multistep history. This is numerical
resolution, not a force scale or equipment fit. The bundle records the complete
derivation and selected value. More than 4,096 selected substeps is rejected.
Every internal step retains its first native terrain contact, before a later
spring rebound can erase it. Enabling derived resolution is a versioned numeric
choice, with a default identity; it is never an aerodynamic gain or data fit.

Fixed-wing primary servos retain their authored rate and receive explicit lag
and physical stops where missing. The shared rotor command shaper v1 uses
2 normalized units/s and normalized stops before the existing native
motor/swashplate response. Its additional lag is explicitly zero: the physical
response states already provide lag, and a second guessed 0.1 s pole destabilized
the unchanged UH-1H performance fixture. The common audit requires those existing
response filters when accepting this composed representation. A standalone servo
without the physical response defaults to an estimated 0.1 s pole. Zero additional
lag is metadata, not a JSBSim `<lag>0</lag>` element: that element freezes its
first sample because JSBSim expects a pole rate. These are common engineering
assumptions, not measured servo responses. No per-design fitting knob is added.

The standalone consumer runs with Python `-I` outside the source/build tree,
checks every newly generated product and actual loaded DLL hash, and rejects a
poisoned copy of the exact runtime XML before restoring its original bytes.
Its identical flight/reference checks include all six force/moment components,
mass/inertia/CG, engine thrust/mass flow, World-reconstruction speed/mass repeatability and native
store release when applicable. Physical rotor reaction loads are included
explicitly in the force/moment oracle. `Vehicle.reset` comparisons remain
diagnostic because that API preserves activities/overrides; full native-state
reset uses a reconstructed owned World. Short untrimmed traces and instantaneous
drag power do not establish measured trim, climb, top speed or equipment limits.
This is installed-product provenance isolation, not actor-only process isolation
or installed MA public-endpoint joint acceptance.

Method references: [MIT AVL 3.40 User Primer](https://web.mit.edu/drela/Public/web/avl/avl_doc.txt)
for VLM/quasi-steady/rotation/PG assumptions (the project uses its own solver),
and [JSBSim frames](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/)
for coordinate conventions. Formula-specific derivations and exact project
source hashes accompany every table. Neither source is exact-model measurement
evidence. No CFD result or AI-generated value is labeled measured.

The [88-row component matrix](data/sim-m02-engineering-completeness.csv) and
[delivery report](../build/architecture-reviews/sim-m02-replay-20261031/delivery.md)
record final statuses and current counts. The historical two armed-store component blocks have new native implementations;
fresh common checks, rather than changed labels, decide their current status. Whole-tool runtime, full static
table/value counts, native sampling and consistency checks are separate from
fresh full regression and independent review. Short untrimmed observations do
not remeasure trimmed maximum-performance profiles.

Current rule, 2026-10-31: one common engineering-completeness gate covers every
design. VLM is the baseline; bounded CFD and labeled public/open-source
engineering estimates are admissible. Exact-model measurements are optional
calibration, not an admission requirement. Report the aggregate production wall
time and phase costs without hiding them through partitioning. The current user
instruction removes a time-based admission gate.

The gate has two required parts.

**Theoretical complexity completeness.** A design must carry every state and
component its equations of motion require: six-degree-of-freedom rigid-body
state; mass, inertia and CG variation; aero six components CA, CN, CY, Cl, Cm
and Cn; Mach, alpha, beta, control-deflection, rate and alpha-dot coverage;
propulsion thrust, mass flow, nozzle and boost/sustain/coast phases; actuator
lag, rate and limits, mixer and autopilot; and, where applicable, seeker,
guidance, fuze, warhead, damage, release and reset. A missing required component
is a block; a labeled engineering estimate is not.

**Factual computation completeness.** Every table must be generated for the
current geometry and card through the shared pipeline, cover the declared
envelope, contain finite values without silent missing rows or columns, declare
interpolation and extrapolation bounds, and record method, source or derivation,
version and `measured`, `estimated` or `simulated` basis. AI knowledge may assist
derivation but is not itself a source. Public papers, government publications
and open-source tools must be cited when used. Agreement between estimators does
not create measured accuracy.

All designs pass the same process; there are no per-model acceptance waivers.
A design is `blocked` only for a missing required dimension, component or table,
or for an explicit operational generation/validation timeout. Missing measured data
alone is not a block. The measured-calibration ledger (`calibrated`, `blocked`,
`historical`) and `public_data.json` per-model labels remain separate evidence
and do not set SIM-M02 acceptance.

Run VLM first. A CFD attempt is optional; if it cannot finish inside the budget,
record the timeout and fall back to VLM or a labeled engineering method. Never
start an unbounded solver campaign or a second background poller. Historical
solver failures are not acceptance criteria and do not forbid a bounded retry.

## Normal entrypoints

Windows x64, pinned CPython 3.13.12 and UCRT64 GNU tools are supported.
From the repository root:

```powershell
.\setup.ps1
.\build.ps1 -Models all -Jobs 16 -MemoryLimitMiB 8192
.\run.ps1 -Mode demo -Background -Arguments @('--fullscreen', '--offline')
.\stop.ps1
```

`setup` detects exact compatible tools and Python packages first, reuses them
read-only, and downloads only missing content from the version/hash lock at
`tools/bootstrap/dependencies.lock.json`. Project-installed dependencies,
source archives, overlays, logs and products stay inside the project. Existing
tools may be reused from `D:/ENV`; compilation scratch uses `D:/ENV/tmp`.
No external CFD solver is installed or required.

`build` compiles the geometry helper, runs every selected design through
`aero -> calibration configuration -> build`, then builds/stages the complete
Headless SDK and Vulkan Viewer. `-Models @('f16c','aim120c')` selects a subset;
`-Plan` displays commands; `-Test` runs the existing complete CTest inventories
with bounded timeouts and zero-test rejection. There are no precision tiers.

`run` supports `viewer`, `demo`, `headless`, `hangar` and `python` modes.
`stop` checks project root, recorded PID, executable, creation time and job
identity before terminating that project's process tree. It does not kill
other Python, Viewer or system processes by name.

```powershell
.\run.ps1 -Mode hangar -Arguments @('list')
.\run.ps1 -Mode hangar -Arguments @('f16c','aero','build','verify')
.\run.ps1 -Mode hangar -Arguments @('aim120c','aero','build','verify','fly')
```

A normal product build avoids optional remeshing and long performance/flight
campaigns. Request `geometry`, `model`, `fly`, `performance` or `calibrate`
explicitly when changing their inputs. `calibration` configuration records
existing/admitted references; `calibrate` runs the public-reference SDK loop and
is not silently run by the product build. Old `storedrag`/`storemodels` CLI
names are compatibility aliases for the same `stores aero build` operation.

## Inputs, products and ownership

Author geometry, physical controls, mass, CG, inertia and propulsion in
`aircraft/<name>/<name>.toml`; cite source dimensions and mark assumptions.
Structural coordinates are metres, x aft, y right, z up. Mirrored surfaces
describe their right half. Cards reference those designs, never an alternative
force model. All 88 designs use the shared airframe aerodynamic path: 31
fixed-wing aircraft, four rotorcraft and 53 stores (24 AAMs, one cruise missile, one bomb,
19 tanks, six pods and two cargo stores).

Aircraft XML is `aircraft/<name>/<name>.xml`; released-store XML is
`aircraft/stores/<name>/<name>.xml`. Store cards are `assets/stores/*.json`.
Each store has a content-addressed `aero/vlm-carriage-<sha256>.json` beside its
released XML. The same base six-axis, rate, ground-effect and Mach data feed
carriage and released flight. Stowed carriage uses neutral fins; released
flight uses actual individually identified finite actuators. An absent or
wrong-source binding fails rather than falling back to a scalar drag table.
Pylon/rail/lug hardware loads remain separate from the store's own loads.
Controller gain scheduling differentiates the same VLM store forces/moments
and shifts them to the carrier CG, rather than reading retired scalar fields.
Rotor thrust, blade dynamics and motor/engine behavior remain physical
propulsion inputs; their airframe aerodynamics use the common writer.

`out/` contains source-keyed tables and stage reports. A source/solver change
invalidates the aerodynamic cache; calibration/writer changes normally require
only cheap publication. XML generation is pure: inspecting a modified card or
a test fixture cannot publish it to production. Normal `build` publishes cards
and shared tables only after writing its XML. Runtime package staging copies
only declared products and prunes obsolete owned carriage-table hashes.

Keep four mass ledgers disjoint: empty mass, fixed payload, internal fuel and
station stores. Component placement distributes empty mass without adding it
again. Crew/passengers/permanent equipment/ammunition/baggage are fixed loads;
a separable tank, pod, weapon or cargo is a store. Stores spawn on empty
stations only when loaded. MTOW admission, external fuel transfer, release CG
and inertia remain SDK-owned. Do not put removable stores into fixed payload.

The SDK is the sole simulation authority. Viewer meshes, effects, RCS and
damage geometry are separate products, never substitutes for flight forces.
Gear/wheel contact geometry, finite hinge axes/stops and the force-reference
origin must stay consistent with the same source dimensions. Geometry helpers
retain resource caps, double-precision source checks and reusable gear plans;
visual mesh simplification is not aerodynamic geometry validation.

## Shared method and retained limits

The force path is `aero/model.py -> aero/tables.py -> aero/mach.py -> jsbsim.py`.
`airframe.py` connects every design to that path. It handles a genuine body-only
design without inventing a wing. Store mass/motor/contact writers add no
independent aerodynamic law. Native `StoreAerodynamicsV1` evaluates the same
published tables for attached stores, including angular local flow and the
force/moment arm to the carrier CG.

| Part | Common implementation and scope |
|---|---|
| Surface interaction | Coupled horseshoe VLM, induced velocities, viscous decambering and actual physical fin/hinge geometry. |
| Section response | Thin-airfoil/Glauert relations, generic Reynolds/thickness and DATCOM-style trends, Viterna separated/reversed-flow continuation. |
| Body/profile drag | Shared geometry/section/body engineering correlations. They are not a resolved viscous flow solution. |
| Leading-edge vortex lift | Shared geometry relation influenced by earlier NASA F16 and delta-wing fits; no aircraft-name branch. Those fitted constants are not universal experimental truth. |
| Rates and controls | All six force/moment rate arrays; signed per-fin increments from the same solver, with actual finite native actuator positions, lag and stops. |
| Mach/ground effect | Shared transformations and linear/compressibility/wave-drag relations over each source's declared domain; rotor propulsion is separate. |
| Calibration | Source-fitted positive `base_drag_scale` and `extra_drag_area_m2`; optional wave-drag efficiency, Korn factor, throttle ratio and propeller pitch in a separate per-design file. Historical overrides remain unverified. |

The drag decomposition preserves the solver's actual circulation-induced
component `CDi`. Only this component receives the induced Mach/ground-effect
factor. Section, body and separated-flow drag remain intact. Applying a
negative global `CL^2` correction to all drag had produced aerodynamic thrust
at high angle/Mach; JSBSim's previous-frame `aero/cl-squared` made that worse.
Current XML uses current-state circulation data, including control and rate
increments, and computes rate normalization from current velocity before
JSBSim pre-functions run. This is a shared algorithm correction, not a
per-model clamp or adjusted acceptance tolerance.

Numerical full-angle/Mach coverage does not imply wind-tunnel accuracy there.
VLM assumes thin lifting surfaces and is strongest in attached subsonic flow.
Transonic shocks, high-Mach viscous interactions, post-stall/spin behavior,
forebody/fin shielding and low-Re separation bubbles are not resolved. Section
Reynolds numbers are built at the authored analysis condition. Propeller
slipstream and exact engine maps remain limited; a top-speed propulsion fit
cannot certify low-altitude thrust or six-axis aerodynamic loads. Current
store products extend through Mach 5 using the shared corrections, not a
supersonic CFD solution. See the [AVL author documentation](https://web.mit.edu/drela/Public/web/avl/avl_doc.txt).
The retained method references include Katz/Plotkin, Glauert,
Mukherjee/Gopalarathnam, DATCOM, Raymer, Polhamus and Viterna/Corrigan.

## Calibration and measured errors

All 88 designs are required calibration subjects. Configuration files, SDK
loads and table consistency do not complete calibration. The public
[error report](data/hangar-vlm-errors.html), [CSV](data/hangar-vlm-errors.csv)
and [source/measurement JSON](data/hangar-vlm-errors.json) report external
reference comparisons. Run the complete loop through the normal entrypoint:

Complete-roster campaigns execute stores first, fixed-wing aircraft next and
rotorcraft last. This execution order does not omit any of the 88 models.

```powershell
.\run.ps1 -Mode hangar -Arguments @('calibrate','--model','all','--require-all')
```

`hangar.reference_campaign` is the common core for aircraft, rotorcraft and
stores; `MODEL calibrate` routes to it. Its versioned `public_data.json` accounts
for every model: dated searches, public sources, page/table, units, conditions,
configuration, predetermined calibration/holdout partitions and missing-data
reasons. PDF hashes identify the originals; admitted extracts ship with the
source, so execution does not require network access. SDK objectives and
finite-difference sensitivities fit bounded physical parameters. Holdouts do
not enter the optimizer, bounds or convergence. Parameters freeze before fresh
SDK holdouts decide retention: store candidates require at least a 2% relative
NRMSE decrease and a physically justified change. Retention is separate from
the 10% fidelity KPI; a retained 2--10% gain must not be reported as that KPI.
Unchanged fits preserve their configuration
and profiles. Normal products
and changed flight/hover/performance profiles are regenerated in the same loop.
Design, calibration, XML and loaded native DLL hashes accompany measurements.
Each measurement also records its controlled fixture XML hash and actual
conditions. Pressure-altitude references are positioned using the SDK's
atmosphere, rather than equating pressure and geometric height. C172 retains
the authored automatic mixture schedule, not a hand-optimized peak-RPM
mixture; its reported climb error includes that propulsion/control limitation.

The Gauss-Newton step uses the pinned SciPy 1.18.1 bounded-variable
least-squares solver. Hard clipping an unconstrained coupled step can stop
before the admissible optimum; an active parameter must leave the remaining
parameters free to refit. On the original MK82 Salisbury training observations,
this correction changed extra drag area from 0.0032734290746024466 to
0.0029271695759419 m2 and Korn kappa from 0.9333841661382474 to 0.95;
wave-drag efficiency stayed at the original 1.2 lower bound. Parameter limits,
data and partitions did not change. The six independent Melbourne holdouts
improved from 13.4933% to 10.2781% CD NRMSE; maximum absolute CD error fell
from 0.0894325 to 0.0719437. Receipt:
`build/hangar-calibration/closure/mk82-bounded-fit/results.json`.
This applies to clean configuration 1 at zero incidence, not its other
fuze/lug configurations or full-envelope coefficients.

Seven models currently have scoped numerical observations, including the newly
retrieved A-22 wind-tunnel data. A calibrated row requires a
parameter diff and a matching original-reference baseline that the current
SDK holdout beats. Blocked rows carry dated negative searches, a specific gap
and a scoped estimate with nonzero assumed uncertainty. Historical rows do
not establish a new calibration. `--require-all` returns code 2 for unresolved calibration;
execution failures are separate. An all-model run with unknowns is incomplete.
No current result certifies full-envelope matched-equipment accuracy.

| Model | Calibrated quantity and held-out comparison | Public evidence |
|---|---|---|
| C172P | Climb rate; different altitude/temperature points held out | Cessna POH figures 5-1/5-5; common flight-test campaign |
| CF2 | Total static rotor thrust versus RPM; alternating bench points held out | Bitcraze 2015 measurements; not body-drag validation |
| F15C, F16C | Zero-lift drag at Mach 0.6/1.3; Mach 0.9 held out | NASA TP3414 figure 23; flight-derived graphical summary, CD0 reading uncertainty 0.001; original control/Reynolds conditions incomplete |
| IRIS+ | Bare-airframe drag fit; drag, lift/side force and three moments retested | NASA TM-2018-219758 raw tables C58a/C59a; disjoint runs, different dynamic pressure, common tunnel |
| MK82 | Clean zero-incidence drag | DSTO-TR0554 tables 4/5; Salisbury S1 calibration versus independent Melbourne facility |
| A-22 | Bare maximum-height container CD; 1/6 physical model fitted, 1/8 and 1/4 models held out | ASD-TR-61-326 / AD0270535, Fig.27, printed p.42; shared wind tunnel, separate scale models |

Fighter CD0 solves zero actual SDK lift, not alpha=0. Its digitized readings
cannot support sub-percent physical precision. The F/A18 comparison in TP3414
does not establish the authored C variant; the older F16A-derived model is also
not direct F16C validation. MK82 geometry/Reynolds differences remain disclosed.
Iris uses test-specific rotor/camera removal; force signs, moment origin and
inch-pound units are explicit. Small near-zero signals can produce large
relative errors: absolute residuals remain in the receipt. C172 performance and
CF2 rotor thrust do not identify all six airframe coefficients.

On 2026-10-30, the shared writer gained a positive base-drag shape multiplier:
`CD_base_corrected = base_drag_scale * CD_base + extra_drag_area_m2 / S`.
It leaves the source tables, Mach increments, lift and moments unchanged;
its default is exactly one. The original 11 IRIS+ C58a training observations
fit `base_drag_scale=2.0621546067304823` and
`extra_drag_area_m2=0.017937512013538322 m2` (previously one and
`0.021237061101274827 m2`). Independent SDK worlds retested the unchanged
11 C59a drag holdouts: NRMSE decreased from 22.1635% to 21.2105%, and maximum
absolute drag-area error from 0.00679243 to 0.00602276 m2. This is a modest
bare-frame drag improvement, not a lift/moment or powered-flight improvement.
The parameter is a fitted correction, not a published measurement. Receipt:
`build/hangar-calibration/closure/iris-shape-fit/results.json`; its conditions,
raw measurements and partition remain in the source catalog for normal replay.

The 2026-10-30 store follow-up retains two changes. MK82 candidate 2 evaluates
the existing body-friction correlation at Reynolds number `M*a*L/nu`, replacing
only the compressibility-only increment already counted by the writer. The
fixed SDK reference atmosphere is a simulated input, not measured tunnel air.
The original six Melbourne holdouts decrease from 7.4365246% to 7.2434702% CD
NRMSE (2.5960% relative); maximum absolute CD error increases from 0.0433508 to
0.0447980. Lift, moments and full-envelope fidelity are not improved claims.

The original A-22 report was recovered after the earlier Citeseer fetch failed.
Its Fig.27 gives bare-container CD 0.802, 0.950 and 0.804 at scales 1/8, 1/6
and 1/4. Printed pp.25--27 and 40 specify a 52 x 43 x 60 inch box, 5.20 psf,
67.6 ft/s, and Reynolds numbers 275000, 366000 and 549000. The card/source now
describe that maximum-height bare configuration, replacing assumed dimensions.
`store_tunnel.py` scales body geometry/reference quantities and uses the normal
table/XML writer; solver speed `Re*NU0/L` matches the source Reynolds. SDK density
is derived from `2q/V^2`; the unreported temperature is declared as an assumed
288.15 K with ideal-gas pressure. The tunnel fixture clears runway contacts and
reapplies atmosphere after reset. Imposed flow is checked against actual SDK q
and density. Model mass scales as s^3 and inertia as s^5 solely for the fixture.
No observed coefficients enter these transformations.

Only the 1/6 observation fits `base_drag_scale`, from 1 to 0.9500315508356607.
The two other original models remain held out: CD NRMSE falls from 24.5713241%
to 18.3121492%, a 25.4735% relative decrease. This meets the store KPI for these
scaled bare-container measurements. The reference's scale-to-scale dispersion
and unknown total measurement uncertainty remain; it is not full-scale airdrop,
parachute, carriage-interference or moment validation. Production table/XML/card
publication and SDK carriage/release checks are separate from tunnel validation.

The seven specifically requested source decisions, their dates, queries,
URLs/report numbers, page locators and gaps are stored once at
`public_data.json:store_campaign_20261030.requested_seven_source_reviews`.
FPU-11/A's CF-18 trial supplies qualitative measured agreement with predictions,
not numerical measured holdouts or established part-number equivalence. FPU-8/A's
NATOPS Fig.11-2 p.11-8 supplies installed DI values without identifying this
entry as measured versus estimated. RAND tables A.2/A.4 remain historical
configuration data for AIM-120/610-gal and 370-gal/ALQ-184, respectively; missing
Mach/Re/DI normalization must not be invented. NASA TM87634 is a 370-gal ground
vibration test; GAO/NSIAD-90-168 is an ALQ-184 EW readiness report. Neither measures
store aerodynamics. There is a second A-class store: A-22; no absence exception
is claimed. Reproduce each retained gain without refitting:

```powershell
.\run.ps1 -Mode python -Arguments @('-m','hangar.store_reference_gain','--model','mk82','--evidence','tools/models/hangar/hangar/public_data.json','--out','build/hangar-calibration/mk82-retest.json')
.\run.ps1 -Mode python -Arguments @('-m','hangar.store_reference_gain','--model','cdsa22','--evidence','tools/models/hangar/hangar/public_data.json','--out','build/hangar-calibration/a22-retest.json')
.\run.ps1 -Mode hangar -Arguments @('mk82','verify','fly')
.\run.ps1 -Mode hangar -Arguments @('cdsa22','verify','fly')
```

The remote `opencode/sim-m03` branch was reviewed at
[`e55e7fa`](https://github.com/coasho/Super-DDCCSS-WWorld/blob/e55e7fa2fb230e164df8cb42958844876d8aa9fb/docs/sdk/missile-public-data-matrix.md).
Its 24-AAM fact table and primary/secondary references are allowed as a
component-identity, seeker-field and provenance reference, reconciled into the
same catalog with the commit and fact-table hash. Its actual corrections cover
component identity and seeker fields. All 24 card drag areas remain explicit
engineering estimates at that commit; aerodynamic calibration uses only
admitted measurement cases. The range-derived R60 seeker estimate is recorded
as an engineering estimate. No combat fixtures or unrelated branch fixes are
merged by the aerodynamic calibration operation.

Published card revisions must also reach authored geometry before regeneration.
The existing `hangar missiles MODEL` authoring path reconciled IRIS-T, Magic 2,
Meteor and PL15 dimensions, including Magic 2's 90 kg launch mass. Their VLM
tables, carriage products and XML are rebuilt from those sources; dimensions
and mass do not become aerodynamic error observations. Existing canonical-source
and release-mass tests retain their original thresholds.

Remaining data work is explicit: UH60 has real flight/hover reports whose
weight, CG, configuration and power channels still need matching; AIM7M has
an identified DREV TM-1999-050 reference whose original numerical report was
not retrieved. MiG29A and R73 searches found scaled-model experiments with
unmatched Reynolds/geometry/configurations. These are usable research leads,
not evidence that no experiments exist and not completed calibration.

The four historical RAND configuration fits (AIM120C, ALQ184, 370/610-gallon
tanks) remain historical overrides, not new SDK fits. [RAND tables A.2/A.4](https://www.rand.org/content/dam/rand/pubs/monograph_reports/2005/MR1216.pdf)
lack complete operating/carrier-interference conditions. Normal build preserves
their configuration; the old table-only fitter and performance-only fitting
entrypoints no longer calibrate products. Other retained overrides are not newly
validated data. Unknown errors stay null, never zero or a table residual.
The fighter throttle ratios inherited by a CD0 fit also remain historical
parameters; a drag measurement does not calibrate engine thrust. F15C/F16C
bounded refits did not reduce their 17.1655%/12.1191% holdout errors. Their
configuration and XML bytes were restored to their exact pre-refit hashes
(`build/hangar-calibration/closure/no-gain-reversion.json`). No improvement is
claimed for those attempts.

All 24 AAM inputs were forcibly resolved again and their XML/carriage tables
republished. Reproduce the full source/card/cache/table binding manifest with:

```powershell
.\run.ps1 -Mode python -Arguments @('-m','hangar.aero_evidence','--out','docs/data/hangar-aam-bindings.json')
```

The shared publisher checks actual card dimensions before stamping provenance;
changing a card cannot merely rebind its new dimensions to unchanged old
geometry. The manifest uses SHA-256 for file contents and records the existing
16-character SHA-1 input key separately. Geometry, card and table hashes are
different objects: each must match its own recorded reference. Hash agreement
and fresh regeneration establish input consistency, not physical accuracy.

The 35 aircraft carry source/method-bound `flight-calibration.json` records so
runtime profiles survive a clone without `out/`. Changed-model data are rerun
before export and normal publication; stale records are rejected. UH1H's
original three fuel-order breaches now measure zero after replacing the
independent truncated torque expression with upstream's force-work accounting
`Q=Q_profile-R*(T*lambda+H*mu)`. The original frozen three speed columns and
the complete 585-run performance sweep were remeasured with unchanged check
tolerances. This repairs a model consistency defect; it does not establish
real UH1H fuel-flow accuracy. The Army flight manual's temperature, gross
weight, rotor RPM and configuration must be matched before admitting fuel
data. The torque change's transient rotor-speed regression remains unresolved
and is retained in the regression logs. The separate experimental fourfold
native rate-integral change was withdrawn; production controller gains are
unchanged. UH60's short-window inversion was removed by the shared 30 s level
averaging window, spanning hold cycles like the existing climb sampler.
Fixed-wing
method fingerprints exclude unused rotor Python methods; the validated
dependency-scope migration retains the original global fingerprint and SDK
observations, without claiming a new flight measurement.

The original motor flight check remains `fastest - initial_speed` between
0.45 and 1.02 times ideal rocket delta-v, with its original trajectory and
timing. A preceding candidate replaced that observable with an integral of
SDK thrust/mass; that candidate is rejected for acceptance. Propulsive impulse
and trajectory speed gain are different quantities. An impulse comparison
cannot erase a failure of the original trajectory check. The retained raw
R-27ET diagnostic gained 174.932 m/s (about 0.326 of ideal); this remains a
negative result against the original lower bound, not a guided-flight
qualification or a passing motor-flight gate.

Every authored design now explicitly opts into `[integration] version = 2,
mode = "derived"`. The shared writer estimates fixed internal resolution from
the six-component table moments and forces transferred from AERORP to the hull
of source dry-CG/tank/point-mass positions, plus the dry-inertia torque-free
Jacobian over the declared reduced-rate reference interval. This is an estimated
neutral-control numerical domain, not a nonlinear global-stability certificate.
Native rotational rates use upstream Adams-Bashforth 3 (property
`simulation/integrator/rate/rotational = 4`), selected after load and after reset,
before `RunIC` initializes derivative history. The public outer clock is
unchanged. Absent numerical configuration and version1 assets retain their previous
integration behavior. Version2's optional integer `refinement` is bounded1..8,
defaults to1, and changes only numerical resolution; it exists for reproducible
step-refinement diagnostics. All88 delivery configurations use the same default1.
The local primary sources are `third_party/jsbsim/src/models/FGPropagate.h`,
`FGPropagate.cpp` and `third_party/jsbsim/src/FGFDMExec.cpp`.
The corresponding upstream [rotational integrator implementation](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGPropagate.cpp)
and [native inertia-product frame convention](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGMassBalance.cpp)
are method references; local source hashes identify the actual runtime inputs.
The native sampling consumer restores the receipt-declared selector immediately
after its raw initial-condition reset, before advancing any step. It validates
and records the actual selector and internal timestep before and after every
sample. The native timestep comes from the generated receipt, rather than
dividing the public World decision period by the model's substep count.
Legacy `TerrainStrikeSourceV1` retains its incoming fixed-tick epoch and mass;
the separate `TerrainContactSourceV1` retains precise first-contact substep
timing for native fuzes. Internal refinement cannot change the legacy
observation boundary. Capturing its state once per fixed tick also avoids
repeating full-state extraction at each internal integration step.
Physical-fin control derivative metadata uses method2: the exact positive
directional derivative of the emitted piecewise-linear neutral interpolant at
each evaluated metadata grid point in free air,
including signed mixer gains, stops and Mach corrections. The explicit
`control-derivative-direction=1` records which side of a neutral knot is used;
the negative directional derivative is separately computable and need not be
equal. The emitted angle/Mach schedule interpolates these values; it is an
approximation between knots, and excludes ground effect, CG moment transfer
and actuator transients. This metadata does not change aerodynamic tables or
their interpolation.
The ducted-cycle negative axial load is projected using the upstream native
wind/body matrix, including the negative authored DRAG/LIFT conventions.
Inverse-transform regression checks cover nonzero alpha and beta.
Primary definitions are upstream
[FGAuxiliary::UpdateBodyMatrices](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGAuxiliary.cpp)
and [FGAerodynamics::Run](https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGAerodynamics.cpp);
the source manifest binds the local implementation used in each replay.

The previous identical-discretization optimization and the later numerical
correction have separate timing/equality records. Changed integration metadata
and internal work cannot be marketed as an identical-runtime optimization.

## Verification and reproduction

On 2026-10-30, normal `build.ps1 -Models all` completed its 12 SDK/Viewer and
product stages. The forced 88-model base-table regeneration took 980.25 s;
the latest cache-assisted aero stage took 78.55 s. Current receipts live under
`build/hangar-calibration/closure/`. The complete 53-store SDK verify/release
campaign measured 780 checks in 117.37 s before the latest MK82 refit; that
refit's own verify/release stages were rerun before its independent holdouts.
The focused evidence/configuration/shared-writer/missile/store suite passed
128 tests in 38.14 s. These checks prevent regressions; their counts are not
equipment fidelity measurements. The expanded rotor suite retains unresolved
transient/control cases. Its experimental rate-integral modification was
withdrawn; earlier passing native totals do not qualify these changes.

The public-reference campaign accounts for all 88 models and independently
remeasures admitted cases through the SDK. Seven have scoped source comparisons;
the latest store-only round retains MK82 and A-22. The complete fleet is not
calibrated, so `--require-all` correctly returns incomplete. Each blocked row
includes its individual source review and estimate limitations. The original
28 historical/configuration overrides remain unverified. Rebuilding a table
or passing SDK/table checks does not change that evidence category.

Restoration evidence before the public-reference refits:
`build/architecture-reviews/hangar-vlm-restoration/`. Current reference receipts:
`build/hangar-calibration/`. The historical cold shared-model rebuild took
830.67 s with 16 single-thread workers and an 8192 MiB scheduler reservation.
These timings describe this machine, not guarantees or hard memory limits.
Current reference/runtime receipts supersede older regression totals.

A source export began in an empty directory with no copied native build
products or aerodynamic caches. Setup, both SDK/Viewer builds and normal run
entrypoints completed after calibration-persistence repairs and replays. All
88 generated XML files are byte-identical to the main directory. The package
SDK passed 88/88 models and 11,411 additional cases through `run.ps1`; the
Viewer rendered an airborne F16C and exited after its screenshot. A subsequent
live project job was stopped successfully by `stop.ps1`. The final cached full
build took 150.27 s; this is not the initial cold-build time. The receipt at
`build/architecture-reviews/hangar-vlm-restoration/reproduction-evidence/receipt.json`
retains the initial interruption and repair replays, not a single-attempt claim.

To diagnose implementation wiring separately after a normal build:

```powershell
.\run.ps1 -Mode python -Arguments @('tools/validation/aerodynamics/vlm_sdk.py','--model','all','--full-grid','--jobs','4','--native-bin','build/ucrt64-release/bin','--out','build/vlm-sdk.json')
.\run.ps1 -Mode python -Arguments @('tools/validation/aerodynamics/reference_sdk.py','--native-bin','build/ucrt64-release/bin','--out','build/vlm-reference.json')
.\run.ps1 -Mode python -Arguments @('-m','hangar.calibration','--out','build/vlm-calibration.json')
.\run.ps1 -Mode python -Arguments @('tools/validation/aerodynamics/error_inventory.py','--sdk','build/vlm-sdk.json','--calibration','build/vlm-calibration.json','--reference','build/vlm-reference.json','--out','build/vlm-errors.json','--csv','build/vlm-wiring-errors.csv')
```

Run store `verify fly` before the inventory if burn diagnostics are absent.
The SDK check selects and hashes the actual loaded DLL, checks XML/calibration
hashes, uses writable initial conditions, covers all authored angle nodes at
four bounded Mach samples, and exercises signed actual actuator positions.
The original 0.002 coefficient wiring tolerance remains. Isolated child
processes retain load failures without losing the rest of the roster. Reference
fixtures hold controls neutral explicitly; they are distinct from production
control-law acceptance.

## Consolidated lessons

This list replaces the former Hangar manual/build log and the cancelled
numerical-convergence/campaign notes; historical numbers are not current gates.

1. Preserve the four mass ledgers, MTOW admission, physical motor/grain/nozzle,
   geometry/CG/reference axes and release inertia; do not tune geometry or CG
   to hide a flight failure.
2. Keep vortex-lattice chordwise load placement and actual wing/body
   interference. Earlier F16 pitching-moment corrections generalized these
   mechanisms; they did not justify copying F16 lift tables to other models.
   Preserve authored variant geometry when fixing a test: R77M retains its
   conventional tail ([Army ODIN](https://odin.t2com.army.mil/WEG/Asset/70822f0f2e8de125f5a54a9c42a022ba));
   a manufactured grid-fin fixture tests grid rendering separately.
3. Account for separated/profile and circulatory drag separately. Iterative
   convergence or finite output alone cannot admit negative wind drag.
4. Record panel connectivity, native index maps, wake-root attachment and
   active-wake containment independently. Old VSPAERO closure/wake patches
   were research controls, not a supported production solver dependency.
5. Distinguish algebra/schema checks, solver convergence, mesh convergence,
   SDK wiring, performance fitting and independent equipment accuracy. Old
   C172/F16 agreement and 48-case RocketPy reports retain their dated scope;
   do not copy those percentages/counts into a new full-fleet claim.
6. Generate all force axes from one source, resolve store resources through
   the same package resolver as released XML, and verify the loaded DLL and
   component fingerprints. A freshly linked DLL does not refresh Python's
   private DLL or staged cards automatically.
7. XML subtree replacement must be structural. Preserve physics subtrees,
   actuators and component identities; keep text generation free of publication
   side effects. Use enough numeric precision for fin stops and Mach grids.
8. Cache deterministic source-keyed geometry/tables; bound worker/thread
   counts; reuse gear plans; write raw per-model logs and propagate child
   failures. Reuse products only when their input identities match.
9. Install only missing pinned dependencies, keep temporary data bounded,
   prune only owned obsolete hashes and stop only recorded project processes.
   A source export must omit tool environments, caches and retired evidence.
10. Preserve fresh World/reset/ignition semantics and resource byte provenance.
    Historical reset/RCS work remains in its owning SDK contracts; it is not
    aerodynamic accuracy evidence.
11. Publish a store card, its content-addressed carriage table and released XML
    together into already-built SDK data before retiring owned old hashes.
    Mixing an old packaged card with new source XML caused an actual SDK load
    failure. Normal Hangar publication now synchronizes both SDK presets and
    their Python packages; calibration does not require a private workaround.
12. Preserve unchanged calibration-file bytes during inventory generation.
    Metadata-only rewrites must not invalidate source-bound portable flight
    records. Physical method/source changes still invalidate them.
13. Fly authored load-factor controls at every SDK step; a 0.5-second recording
    cadence is not a control cadence. Performance trials reuse bounded slots,
    stop when complete and omit combat payloads while preserving identical
    mass, metrics, propulsion, flight-control and aerodynamic XML subtrees.
    F15C/F16C sampling measured 34.94/31.26 seconds after this correction;
    the original performance tolerances are unchanged.
14. Publish rotor performance bounds within both the tested request and the
    SDK speed actually achieved; settling residuals and transient overshoots
    must not extend the bound. Average level fuel/power over several hold
    cycles; a five-second window produced a false UH60 fuel-order inversion.
    The original fleet actuator acceptance still uses its final principal-angle
    difference. A signed accumulated-roll diagnostic cannot replace that
    predicate; any angle-wrapping disagreement remains an explicit failure.

The SU2 installation/campaign/source branch and old scalar/slender/bluff store
force branches have been removed from normal execution. Some retired archives
and research-data directories, plus the disposable reproduction checkout,
remain because automatic approval rejected their
deletion with `blocked by policy`; they are excluded from source exports and
never read by Hangar. Do not report their bytes as reclaimed. The current
cleanup receipt (`build/architecture-reviews/hangar-vlm-restoration/cleanup-current.json`)
lists eight blocked targets totalling 8.879 GiB, including 3.845 GiB from the
reproduction checkout, rather than repeating the old 70 GiB inventory.


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


<!-- SIM-M02 historical generated verification begin -->
Current production delivery: 88 engineering-complete-uncalibrated, 0 engineering-complete and 0 blocked designs. All 31 fixed wings, four rotorcraft and 53 stores (including all24 AAMs) use the same source-driven method and component rules. Aggregate production generation/install: 1045.836 seconds; current basic usability: 813.329 seconds. Current basic execution covers 88 native and 88 installed consumers, 2112 short-flight observations and 10 core/package fixtures (0 failed, 0 skipped). Production integrity inventories 16776 logical/serialized table entries and 3611335 XML coefficient cells. These are separately counted from document consistency predicates. This dated basic-use result does not establish formal SIM-M02 acceptance; no full regression or full-envelope dynamic coverage is claimed. Exact-model measurement calibration remains optional. Evidence: `build/architecture-reviews/sim-m02-delivery-20261031/`.
<!-- SIM-M02 historical generated verification end -->
