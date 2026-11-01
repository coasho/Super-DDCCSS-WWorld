# Simulation module plan

<!-- SIM-M02 final closure begin -->
SIM-M02 is **closed for the user-authorized 31 fixed-wing and 53 store functional scope** on 2026-11-01. All 4 rotor functional populations are explicitly deferred at lowest priority, not passed. The common engineering matrix still covers all 88 designs: **88 engineering-complete-uncalibrated, 0 blocked**. Missing equipment measurements are optional calibration limits, not missing components. Required but unavailable components remain required, absent, failed and blocked; the native armed-store implementations do not use an inert-release waiver.

The current clean-source execution covers all 840 selected headless CTest cases, followed by the exact 2 approved repair rechecks (79 nested methods). The first headless run's 2 failures are retained. Independent current Release execution passes **914/914**, with the exact 28 authorized rotor-only cases excluded from both preset inventories. All 16 fleet shards retain all 31 fixed wings. Additional current evidence is 24/24 matched-state AAM observations (3,454 native evaluations), 48/48 powered axial comparisons at both 480/960 Hz, and 84 native plus 84 installed consumers with 2,016 short-flight observations, 106 released bodies and 10/10 component/package fixtures. These distinct populations are not added into a fabricated single test total.

Exhaustive static checks cover 7,067 native XML tables, 3,611,335 coefficient cells, 522,331 axis knots and 16,776 inventory entries. Every table retains method/source/version/basis, finite-value/grid and interpolation/extrapolation checks. The 1,165 applicability/check/status consistency predicates are **not physical tests**. Native short-flight, pressure and axial checks are sampling/method comparisons, not full-envelope equipment validation. CFD tables: 0; newly measured aerodynamic tables: 0.

Current-source all-design generation, mandatory-component recovery, installation, basic use and full-table auditing take **2,845.286 seconds in total**. This exceeds the 20-minute aspiration; no under-20-minute or historical aggregate speedup is claimed. The dominant measured job is C-17A at 1,290.243 seconds, including 926.975 seconds for fresh damage/detached models. Fresh preparation avoids repeating cold generation in each regression fixture, bounded immutable caches preserve exact inputs, and both native integration resolutions reuse one identical-state numerical reference per atmosphere/design. Complete and interrupted regression/attempt times and end-to-end elapsed time are separately recorded, not hidden by batch partitions.

The root normal `run.ps1` entry and the clean-source entry each complete the unchanged 40-second native encounter at 1x, fullscreen 3840x2160. The SDK produces launch, acquisition, contact detonation and damage; rendered captures preserve those outcomes. The scene executable statically links `fsim::core` and was rebuilt from current source; actual JSBSim DLL identity is recorded separately from the root public Python SDK's actual libfsim/JSBSim DLL identities. All 439 generated products match the accepted manifest in source, staged data and the installed package. This is concrete entrypoint and package evidence, without claiming all-design visual or equipment calibration acceptance.

The [final receipt](../../../data/sim-m02-final-closure-20261101.json), [88-design matrix](../../../data/sim-m02-engineering-completeness.csv), and `build/architecture-reviews/sim-m02-final-closure-20261101/delivery.md` identify the exact commands, original failures, independent reviews, current counts, source/product hashes, complete installed SDK, one HEAD-based exact-byte patch and successful patch replay. This dated closure supersedes the historical SIM-M02 nonacceptance entries below; those original failures remain available as history. Rotor fitness remains in its separate authorized deferral ledger.
<!-- SIM-M02 final closure end -->

<!-- SIM-M02 authorized closure boundary begin -->
The superseding 2026-11-01 user instruction defers **all rotor functional
issues at lowest priority**. This SIM-M02 functional closure covers 31 fixed-wing
and 53 store designs. The 4 rotor designs retain their all-design engineering
component/table records; their flight fitness is deferred, not passed.
The [versioned deferred-scenario ledger](../../../data/sim-m02-deferred-rotor-issues-20261101.json) names 28 rotor-only native
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

Aerodynamic evidence policy and the Hangar-to-runtime workflow are
maintained in the [Hangar system guide](../../../hangar-system.md). This document
retains its architectural, ownership and milestone contracts.

The [SIM-M02 engineering-completeness gate](../../../hangar-system.md#sim-m02-aerodynamic-engineering-completeness-gate)
supersedes the dated solver selections and acceptance assignments below.
VLM is the baseline; bounded CFD and labeled engineering estimates are
admissible, and measured exact-model data is optional calibration.

## 1. Responsibility and completion criteria

Simulation is the sole owner of the native World clock and physical execution: flight, controls, physical disturbances, sensor measurements, modeled communication delivery, release, weapon guidance, fuzes, collisions, damage and native Agent/C2 permission enforcement.

Its implementation destination is src/simulation. It consumes neutral contracts and named foundation services; it imports no training or visualization implementation. It can run, reset an ordinary simulation and expose existing APIs without a GPU or an ML framework.

Two completion tracks are independent:

- Structural completion: the module is correctly owned, acyclic, headless and compatible after relocation.
- Model completion: the original physical-model and performance criteria have their own achieved evidence.

A directory move does not close a model-fidelity gap.

## 2. Current inventory

| Area | Current source | Destination | Existing responsibility |
| --- | --- | --- | --- |
| Flight | src/sim | simulation/flight | JSBSim integration, vehicle state and native stepping |
| Control | src/control | simulation/flight_control | Activities, grants, execution and flight-control arbitration |
| Physical effects | src/effects | simulation/environment | Wind, noise, latency/sensed-state errors and force/moment effects |
| Mission Systems | src/ms | simulation/mission_systems | Host, payload state/BIT, RF/RCS, radar/tracking, weapon/damage primitives |
| Generic communication | src/comm | simulation/communications/network | Network/UDP and communication primitives |
| Modeled DMS | src/session/Dms* | simulation/communications/dms | RF-aware modeled delivery, queues and provenance |
| Native services | MaCore, AgentRules, MaServiceBindings, MaReports | simulation/runtime/services | Deterministic native permissions, C2 tasks/authority and reports |
| World | src/session/World* and native scenario | simulation/runtime/world | Authoritative assembly, state lifecycle and dispatch |
| Store binding | src/session store-binding helpers | simulation/runtime/stores | Carriage, mass/drag and released-model ownership |
| Public simulation facade | selected src/sdk World/Agent/scenario files | simulation/api | Existing C++ simulation API implementation below training |

Shared IPC/recording mechanisms belong in foundation/transport; their stable state/wire definitions belong in contracts. Public API headers remain include/fsim.

## 3. Required internal boundaries

### Flight and control

FDM integration receives explicit native commands and returns authoritative state. Control algorithms remain below World orchestration. Source relocation preserves dt, frame skip, operation order, control defaults, registry behavior, control-allocation semantics and seeded effects.

Tests must compare the same seed, recipe, input commands and native cadence. A changed observation wrapper is not permission to adjust flight gains or aircraft cards.

### Mission Systems

Models consume admitted world/sensor facts and return native measurements or outcomes. World owns entity/authority lookup, ordered store selection and transactional changes. Stateless eligibility/geometry helpers stay with their domain model.

Keep public native meanings: release activity completion is not impact; detonation is not a kill; crew loss is not every form of mission kill; removal is distinct from destruction. Actor reports cannot use referee-only identities or state.

### Native services and communications

The deterministic native MaCore, DMS and Agent permissions remain simulation services. They are not a production MA planner/BT runtime. Cross-platform information arrives through modeled sensors/links; shared process memory does not broaden an Agent's view.

Keep modeled DMS separate from OS named pipes, shared-memory publication and OMS wire transport. Each has different authority, delivery and lifetime contracts.

The generic network target and modeled_dms target are distinct. Modeled DMS consumes the MS RF propagation implementation; declare modeled_dms -> ms and modeled_dms -> comm explicitly. The runtime consumes modeled_dms. Do not move Dms.cpp into the generic comm target while omitting its RF dependency.

### Construction and scenarios

Separate native World options and scenario values from facade mapping and training recipes. Preserve explicit mapping completeness and legacy scenario admission. Native runtime cannot import VecEnv or MissionBatch merely to read scenario values.

### Output

World publishes or records immutable output through foundation transport. Presentation readers, focus, capture rate and recording configuration must not change authoritative state or RNG. Keep diagnostics separate from permitted actor reports.

## 4. Structural work packages

| ID | Prerequisite | Deliverable | Acceptance |
| --- | --- | --- | --- |
| SIM-A | ARCH-01 ownership policy | Per-file ownership of World, services, DMS, physical effects and public facade | No unowned/overlapping sources; current symbols mapped |
| SIM-B | ARCH-02 neutral contracts | Separate scenario/options/state values from SDK/training aggregates | Public layouts unchanged; dependency-negative fixtures |
| SIM-C | ARCH-03 foundation migration | Flight/control/effects/MS leaves moved without algorithm changes | Existing numerical/domain regressions and headless graph |
| SIM-D | SIM-B/C | Runtime/world/services/stores ownership split and relocation | Native scenario/replay/determinism parity |
| SIM-E | SIM-D | Lower private simulation_api for World/Agent/recording and construction mapping; native scenario execution in runtime; thin public applyScenario adapter remains in final assembly under HC-15 | No training dependency; one exported definition; old-header consumers; adapter relocation in ARCH-07 |
| SIM-F | SIM-E and M8 generic handoff release | Training consumes the bridge without a reverse edge | Complete native/Python graph and installed consumers |
| SIM-G | ARCH-10 | Final simulation source/package acceptance | Current full regression plus required installed provenance |

Exact cross-domain sequencing and write allowlists are in the [migration roadmap](../../migration/roadmap.md).

## 5. Functional roadmap retained after structural work

These are separate feature packages with their own model inputs, provenance, tolerances and acceptance. No new implementation is claimed by this plan.

The [Mission Systems work allocation](ms-work-allocation.md) assigns the next model and integration packages to complete vertical owners. It retains the original MS acceptance criteria and separates them from MA-training M0-M8.

| ID | Original scope | Current delivered basis | Remaining output |
| --- | --- | --- | --- |
| SIM-M01 | MS-4b/MS-5/MS-6 refinements | Radar, tracking, detection/signal processing and engagement modes | Sourced measurement-error floors/glint/boresight; tracker/fusion work; signal-level track/cued/STT and processing limits |
| SIM-M02 | MS-7c | Generated missile geometry/JSBSim and native powered flight | Generic engineering-completeness acceptance for all designs: complete six-degree-of-freedom/aero/propulsion/control components, finite full-envelope tables, provenance, installed-product evidence and disclosed aggregate production timing; measured exact-model calibration is separate and optional; RocketPy motor/powered-flight software comparison and input provenance |
| SIM-M03 | MS-7d | Native radar/IR guidance, seekers, fuzes and damage using stated assumptions | Calibrated seeker/sensor and fin/autopilot response, documented fuze/lethality fidelity and repeatable encounter evidence |
| SIM-M04 | MS-8 | Advisory launch-zone queries | Flyout-derived DLZ/WEZ with published-performance comparison and explicit uncertainty |
| SIM-M05 | MS-9b | RWR contacts, active-seeker emissions and native chaff/flare behavior | Full ESM/electronic attack and calibrated countermeasure/sensor interaction |
| SIM-M06 | MS-10 | Existing rendering infrastructure and limited native IR engagement | Physical IR signatures/transmission, IRST and targeting sensors, defined image/measurement contracts |
| SIM-M07 | MS-11 | Generic DMS/C2 links and payload status/BIT | Physical terminal/PNT/weather/SAR/SMTI and remaining weapon/payload models; do not relabel existing links absent |
| SIM-M08 | MS-12 | Native SDK commands and modeled messaging | External OMS/LA-CAL adapter, versioned schema mapping, service status/heartbeats and protocol/harness acceptance |
| SIM-M09 | MS-7f and original Mission Systems M1 | A narrower native engagement path | Published range/flight-time/no-escape comparisons, measured 4v4 real-time cases and the specified autonomous 1v1 Viewer loop |
| SIM-M10 | Documented simulation backlog | Existing aircraft/store/RCS/damage generation | Loaded-store RCS, bomb ballistic validation, model/inertia/ground-impact gaps in individually scoped packages |

The maintained [MS roadmap](adr-30-mission-systems.md), [engagement contract](../../../sdk/engagement.md), [current status](../../../status.md) and [backlog](../../../backlog.md) define the existing facts. Historical solver/tool installation is preparation, not proof that calibrated models were delivered.

### Model package procedure

For each SIM-M package:

1. Freeze exactly which model/family and acceptance criterion is being completed.
2. Record source data, licenses, assumptions, uncertainty and admitted operating envelope.
3. Define deterministic reference cases and numerical/performance tolerances before changing behavior.
4. Change source designs/cards/generators, never derived XML or a client-side physics substitute.
5. Exercise unit physics, integrated World execution, SDK permission paths and event meaning.
6. Compare solver/reference outputs and record residuals, rather than claiming realism from screenshots.
7. Update model capability/support states only for verified scope.
8. Run appropriate compatibility/installed/data-provenance gates and visual evidence if model presentation changed.

MS-12 is an external protocol adapter above simulation APIs. It cannot make the simulation core depend on a network server, generated UI or a training runtime.

## 6. Compatibility and data gates

- Preserve existing World/Vehicle/Agent/Referee public signatures, layouts and errors.
- Keep old Vehicle.reset semantics; complete mission resets belong to the training contract.
- Preserve event ordering, terminal record/history behavior, store bookkeeping and authority ownership.
- Keep CPython types and C ABI sized-copy rules stable.
- Keep Hangar design-to-generator-to-runtime ownership; account for geometry changes invalidating RCS and damage products.
- Installed consumers load only their installed DLL/data prefix; poisoned/missing data must not fall back silently to source.
- Preserve publication/recording layouts and legacy readers.
- Archive baseline hashes before a migration phase; record intentional behavior changes only in separate model packages.

## 7. Verification and exit

Use the existing launcher/presets and the [acceptance matrix](../../validation/acceptance.md). Discover actual test inventories, run nonempty bounded selections and record the exact HEAD/diff.

Required structural coverage includes flight/control, RF/radar/track, stores/native engagement/damage/countermeasures, Agent/C2/DMS ownership, scenario mapping, output parity, installed consumers and old binaries. A headless simulation build cannot link Vulkan/VSG/Flow.

A structural handoff lists moved symbols/files, remaining old-path compatibility shims, target/import graph, preserved exports/types, tests and pending model gates. The final simulation plan remains open until all selected SIM-M criteria are either achieved with evidence or explicitly deferred by the user; structural acceptance alone does not make that claim.


<!-- SIM-M02 historical generated verification begin -->
Current production delivery: 88 engineering-complete-uncalibrated, 0 engineering-complete and 0 blocked designs. All 31 fixed wings, four rotorcraft and 53 stores (including all24 AAMs) use the same source-driven method and component rules. Aggregate production generation/install: 1045.836 seconds; current basic usability: 813.329 seconds. Current basic execution covers 88 native and 88 installed consumers, 2112 short-flight observations and 10 core/package fixtures (0 failed, 0 skipped). Production integrity inventories 16776 logical/serialized table entries and 3611335 XML coefficient cells. These are separately counted from document consistency predicates. This dated basic-use result does not establish formal SIM-M02 acceptance; no full regression or full-envelope dynamic coverage is claimed. Exact-model measurement calibration remains optional. Evidence: `build/architecture-reviews/sim-m02-delivery-20261031/`.
<!-- SIM-M02 historical generated verification end -->
