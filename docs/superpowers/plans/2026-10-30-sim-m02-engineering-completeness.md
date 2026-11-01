# SIM-M02 engineering completeness implementation window

Assignment: the user assigns the complete 88-design SIM-M02 package and sole
ownership of its shared paths for this window. Base main HEAD:
`004e1f7455e9a19af231e2d795f65f2bf4c5f4ba`. No commit or push is authorized.
Existing working-tree changes are retained. No other worktrees or excluded
directories are inputs.

Write allowlist: `tools/models/hangar/hangar/engineering*.py`,
`tools/models/hangar/hangar/__main__.py`, `aero/tables.py`, `aero/mach.py`,
`verify.py` and `jsbsim.py` under that same package; shared normal generation
hooks in `airframe.py`, `pipeline.py`, `rotorcraft/` and `storestages.py` only
where required; `cmake/StageDesigns.cmake`; generated XML/Engines/aero products
under `aircraft/<design>/` and `aircraft/stores/<design>/`; normal generated
store-card aerodynamic bindings under `assets/stores/`;
`tests/test_hangar_engineering.py`; the four assigned documentation files;
this plan; evidence under `build/architecture-reviews/sim-m02-engineering-20261030/`.
The normal runtime publication allowlist also includes `runtime_products.py`
to stage the generated engineering bundle alongside its exact XML epoch.
The maintained generated roster summary is
`docs/data/sim-m02-engineering-completeness.csv`; its source is the complete
campaign `matrix.csv`, with detailed receipts retained under the evidence path.
The unchanged high-rate preflight reproduced native integration divergence in
the small-inertia limiting case. Extend this window to the private
`src/simulation/flight/JsbsimModel.cpp::{load,step}` implementation and shared
`hangar/integration.py` generated numerical-integration metadata. No public
headers, object layouts, signatures or World clock contract change. The main
HEAD remains the base recorded above. Native writes/builds remain exclusive.
Tests live in the established `tools/models/hangar/tests/test_engineering.py`.
The independent native envelope/performance consumer is maintained at
`tools/validation/aerodynamics/engineering_sdk.py`; it consumes the immutable
campaign receipts and installed products, without changing the generation
inputs or the existing coefficient-error threshold.
The final native-table inventory audit uses
`tools/validation/aerodynamics/engineering_inventory.py` and its focused tests.
It follows the bundled JSBSim `FGTable` leaf-dimension inference, including
internal tables without independentVar declarations, and records numeric axis
bounds and strict finite/rectangular/monotonic checks for every XML table.
Unchanged AAM regressions exposed two integration issues: changing the default
one-tick force sampling semantics, and losing contact inside a substep loop.
The numerical-resolution input is therefore explicit/versioned with identity
default; the common derivative-based derived mode is recorded in
`aircraft/cf2/cf2.toml`. Extend the write allowlist to that source declaration,
`tests/simulation/test_vehicle_pool.cpp` and its synthetic integration-contact
fixture. Existing assertions and thresholds remain unchanged. Each native
substep must preserve the first terrain strike before later motion can erase it.
The unchanged all-AAM low-speed/full-command fixture also requires derived
resolution for `aircraft/asraam/asraam.toml` (18 recommended substeps). This is
the same versioned integration input, with identity default and no name-based
solver branch. Extend the authored-source allowlist to this declaration.
Native World, Agent, schemas, ABI and reference observations are outside this
write window. Source component definitions are inspected without changing their
fidelity or measured-data claims.

1. Complete the common coefficient schema and declared Mach boundary; retain
   the normal geometry/card -> VLM -> common writer path.
2. Implement one roster-driven generation/audit/reference/runtime campaign.
   Record method, derivation, version, basis, interpolation and extrapolation
   for every table; validate shapes, finite values and required components.
3. Declare stable bounded batches before execution, with a non-configurable
   1,200 second subprocess deadline including generation and validation. Record
   failures and partial results rather than hiding timeout or missing tables.
4. Regenerate all 88 without a table cache hit. Validate native flight and
   force/performance traces and installed package provenance with identical
   checks and no per-name exemptions. Install only through the existing preset.
5. Publish the complete matrix, actual timings, commands, logs, product hashes,
   limitations and final diff. Exact-model calibration remains a separate ledger.

The native configure/build/test slot is held serially by this assignment; no
competing CMake or Viewer process was found when the window began. Every build
sets TMP, TEMP and TORCH_EXTENSIONS_DIR to `D:/ENV/tmp`. A batch uses one worker
entry and raw file logging; no additional background poller is created.

Final regression found that a guessed extra 0.1 s command pole double-counted
the native rotor/motor response and failed two unchanged UH-1H performance
conditions. Bounded, test-only chain instrumentation isolates that pole; finite
rate/stops without the second pole retain all nine original held conditions.
Keep response lag in the native plant, record zero additional front-end lag,
and require actual existing lag filters in the composed-component audit.
JSBSim's zero pole is not identity, so zero additional lag is explicit metadata
with the lag element omitted. No original fixture predicate is changed.
Archive the earlier complete campaign, freeze the repaired generator and force
the complete 88-design process again, preserving AAM/fixed-wing/store/rotor order.

## Component applicability correction window, 2026-10-31

Main HEAD remains `004e1f7455e9a19af231e2d795f65f2bf4c5f4ba`.
The user assigns this follow-up: fix applicability/check contradictions using
one rule for all designs; investigate native bomb effects first; never infer
non-applicability from missing implementation. The exclusive serial native
slot is acquired for this window; no competing CMake, CTest or Viewer process
was found. Preserve all unrelated dirty files; no staging, commit or push.

Write allowlist: `hangar/engineering.py` and
`tools/models/hangar/tests/test_engineering.py`; the four assigned acceptance
documents; generated roster CSV; normal regenerated aircraft/store products and
card aerodynamic bindings. Evidence and bounded drivers are under
`build/architecture-reviews/sim-m02-applicability-20261031/` with an isolated
installed prefix under `D:/ENV/sim-m02-applicability-20261031`.

Native investigation identifies missing ballistic armed-release composition:
`weapon/Composition.cpp::configureCompositionV1` requires an AAM seeker and
control; `World.cpp` only constructs guided execution for authored AAMs;
`WorldEngagements.cpp` requires an armed guidance instance for contact burst.
Existing fuze/warhead adapters alone do not supply that missing lifecycle.
Do not bind AAM providers to unavailable armed-store roles to manufacture completion.
Keep unavailable required components blocked until native execution exists.

The previous follow-up used 12 reporting batches and a hard 1,200-second batch
budget. That timing criterion is superseded by the aggregate-performance window
below; the component truth table and complete-roster requirement remain in force.

## Replay and aggregate-performance correction window, 2026-10-31

Baseline main HEAD: `004e1f7455e9a19af231e2d795f65f2bf4c5f4ba`.
The user assigns the entire SIM-M02 shared-path write window. The serial native
slot is held by this task; no competing CMake, Ninja or Viewer process was found.
No branch, worktree, commit or push is authorized. Unrelated changes are retained.

The write allowlist adds `aero/tables.py`, `engineering.py`,
`engineering_runtime.py`, `tools/validation/aerodynamics/engineering_sdk.py`,
their method tests and reproducible delivery tooling. The existing source/product
and acceptance-document allowlist remains in force. New evidence belongs under
`build/architecture-reviews/sim-m02-replay-20261031/` and tools/caches/installations
under `D:/ENV`. A clean source archive plus one patch will be replayed without a
Git branch or worktree. Read-only independent review is explicitly requested.

The latest acceptance replaces the previous hard 1,200-second campaign gate:
report aggregate generation plus validation wall time, before/after comparison,
all static table coverage, native sampling separately, and fresh full headless
and Release regression. A 20-minute overrun must be visible, not hidden by
partitioning. Operational subprocess limits remain explicit safety bounds.

Independent scheduling review extends the correction to `CMakeLists.txt`,
`cmake/SerialVerification.cmake`, `python/CMakeLists.txt` and
`tests/CMakeLists.txt`: retain read-only CPU parallelism, serialize shared-cache,
configure/build/install and GPU fixtures, and record discovered slot properties.
The public ABI, physical assertions, reference inputs and test thresholds stay
unchanged. Exact Hangar method IDs and actual counts are reconciled from full
CTest output instead of repeating the complete suite in a serial process.
Failed/interrupted diagnostic attempts remain separate evidence and cost.

## Full closure continuation, 2026-10-31

Main HEAD remains `004e1f7455e9a19af231e2d795f65f2bf4c5f4ba`.
The user requires closure rather than an open delivery. The main task remains
the sole writer. The frozen `build/r` export and its running serial native gates
are immutable; main-source changes are not inputs to that running build.
The write allowlist additionally covers shared component applicability,
ballistic/cruise armed-store composition, native release/fuze/damage/reset,
rotor controller allocation and native own-plant adapters, Hangar source-design
round trips and deterministic damage backend selection, plus their regression
fixtures and acceptance documentation. Public SDK signatures/layouts/vtables,
enum values, native outcome semantics and physical acceptance limits stay fixed.
No alternate aerodynamic scale or reference-data change is authorized.

The same closure window includes `tools/CMakeLists.txt`, the bounded native
recorder in `tools/validation/native_flight_probe/`, private
`RotorAllocationV1.h` and its control regression fixture, typed
`turbofanstore.py`/`storecontacts.py`, `assets/payloads/h6k-store-station.json`,
and the explicitly estimated cruise source configuration. Native body, wind,
structural and CG references must remain dimensionally distinct. Optional
terrain-fuze initiation defaults to identity unless a validated source binding
selects it. The user subsequently required the original stress scenarios and
assertions to remain unchanged. All proposed fixture changes were reverted;
their diagnostic evidence does not replace those frozen acceptance cases.

The performance continuation retains main HEAD
`004e1f7455e9a19af231e2d795f65f2bf4c5f4ba` and the same sole writer. Its exact
additional paths/symbols are `hangar/shape/meshkit.py` (`Probe`,
`probe_cache_stats`), `native/sdf.cpp` (`Union::below`), `hangar/resources.py`
(`threads`, `thread_reservation`), `hangar/engineering.py` (`generate`,
`generation_order`, `campaign`), the corresponding cache/resource/scheduling
tests, and `tools/validation/aerodynamics/engineering_clean.py`,
`engineering_verify.py`, `engineering_report.py` plus the explicit versioned
scheduling profile. Hangar paths above are relative to `tools/models/hangar/`.
The changes preserve every fitting point, native distance/material result,
grid, numerical tolerance and physical regression assertion. Native geometry
thread reservations are explicit and the worker-times-reservation product is
bounded at 16; defaults retain the original ordering and one-thread policy.

A proposed earth-fixed horizontal integral and paired path-feedforward change
was tested against the unchanged 148-case flight diagnostic suite and rejected:
it increased failing cases from five to ten. The proposal, friend reader and
hypothesis test were removed. Its before/after logs remain diagnostic evidence;
the final source preserves the original controller coordinate convention.

The same performance window also covers the test-only reference evaluator in
`hangar/engineering_runtime.py`, its sampling caller
`tools/validation/aerodynamics/engineering_sdk.py`, and standalone evaluator
regressions under `tools/validation/aerodynamics/`. Optimizations may reuse
immutable numeric table storage and per-state interpolation stencils/Mach
factors only. Every operation used to evaluate a coefficient, every sample,
expected value, tolerance and failure predicate must remain unchanged. The
entire realized-state corpus must compare bit-for-bit against the preceding
reference implementation before accepting the optimization.

The replay prerequisite closure includes the existing root `README.md` unchanged,
which documents the supplied bootstrap/Hangar entrypoints and is an explicit
input to the original Git-checkout compatibility fixture. The fixture itself is
unchanged. The ordinary export retains its skip; the verification runner may
execute that same original method on main only after its source dependencies and
its declared fixture inputs match the tested export byte-for-byte. Original
skip counts and external execution remain separately reported. Any unexecuted
or uncovered skip rejects whole acceptance. This changes no aerodynamic model
admission rule, original scenario, expected value or tolerance.

The typed-store continuation covers `components.migration`/`motor`,
`turbofanstore.inputs`, `ComponentsTest` and `StoresTest.test_flight_models`.
Authored air-breathing hardware requires a compatible available versioned
selection; unsupported/mixed hardware cannot silently become unpowered or
not-applicable. Legacy numerical equivalence remains strict on every compatible
source, while new typed sources require missing-selection refusal and complete
deterministic engine/nozzle/feed/tank/governor artifacts. The complete 53-store
partition and 24-AAM assertion remain enforced. Canonical mass/fuel checks retain
the existing 0.02 kg tolerance. Original rotorcraft, pattern and fleet scenarios
and physical assertions remain unchanged. The first unfiltered 858/932-record
run retains eight failures per preset; two structural type assumptions are
corrected separately from the six remaining original physical failures.

Independent acceptance review then found additional predicate substitutions in
that candidate: accumulated fleet roll instead of final principal-angle roll,
propulsive impulse instead of trajectory speed gain, reduced carriage magnitude
checks, and altered control-force/derivative/incremental-work predicates. The
candidate is rejected for whole-program acceptance. The restoration window
includes `tests/simulation/test_fleet.cpp`, `test_store_drag.cpp`,
`hangar/storestages.py`, and Hangar `test_carriage.py`, `test_missiles.py`,
`test_storecontrol.py`, plus the common guide and evidence tools. Original Python
acceptance functions are restored exactly to the baseline AST; the fleet fixture
is restored byte-for-byte. The retired internal Munk/half-fin fields stay removed,
while their physical directional-stability range/sign checks are reapplied to
the actual common six-component table loads. Routing/complete-model passivity
checks remain additive and cannot replace an original failed predicate. Run a
new all-design forced campaign and unfiltered headless/Release regression from
the resulting HEAD-plus-patch export. Preserve every rejected attempt and its
aggregate clock; do not reuse its selected or substituted predicates as current
acceptance.

The source-supported numerical correction additionally owns
`hangar/integration.py`, `src/simulation/flight/JsbsimModel.cpp`, numerical-option
sections of all88 authored TOMLs, the existing engineering method tests,
`hangar/engineering_runtime.py` and `engineering_burn.py`/`engineering_verify.py`.
All designs opt into the same version2 source-derived fixed resolution and
upstream AB3 rotational-rate integrator. Absent configuration and version1
assets preserve their existing behavior. The estimate includes native CG force
transfer and the source reduced-rate gyro Jacobian; no aerodynamic coefficient,
geometry, mass, controller gain or physical acceptance threshold is changed.
An optional bounded refinement1..8 has identity default1 and is used only for
documented numerical convergence; final delivery has no design exception.
The unchanged original R77M burn case passes at refinements1/2/4; the later
all-design campaign and complete regression remain mandatory. Original motor
flight runs for every authored motor as a separate complete applicability
roster, with actual loaded numerical method/timestep and generated XML hashes.
