# Mission Systems work allocation

Date: 2026-10-17. Planning baseline: main `5c80892` with the architecture/terminology/review documentation being published together. This allocation starts MS planning; it makes no new implementation or model-acceptance claim.

## 1. Scope and authority

The [simulation plan](plan.md#5-functional-roadmap-retained-after-structural-work) defines SIM-M01 to SIM-M10. The [original MS roadmap](adr-30-mission-systems.md#8-stages) defines their physical-model goals and original Mission Systems M1. Those remain the completion criteria. Directory migration, a native missile engagement and training M0-M8 core do not complete these physical-model milestones.

SDK remains the simulation authority. Real MA planners, behavior trees, blackboards and DRL training belong in the upper-layer project. SDK examples are minimal runnable demonstrations. This plan does not transfer Claude's existing generic gateway, composition, batch, compatibility and installed-consumer ownership in the [training allocation](../../../superpowers/plans/2026-10-12-ma-system-training-work-allocation.md).

Each owner delivers its complete feature: native behavior, required public C++/C99/Python interfaces, generated-model inputs, examples, meaningful regressions, documentation and acceptance evidence. Do not divide work by language or implementation layer. The other agent reviews the handoff independently; review does not transfer ownership.

## 2. Responsibilities

| Existing package | Planned lead | Complete feature and acceptance focus |
| --- | --- | --- |
| SIM-M01: radar errors, processing and tracking/fusion | Claude | One radar family and one named processing gap at a time; sourced error/processing limits, native measurement and tracking behavior, legal reports, public usage and repeatable fixtures. Retain MS-4b/MS-5/MS-6 distinctions |
| SIM-M02: MS-7c missile aerodynamics and propulsion | Codex | First representative missile model, geometry and motor provenance, SU2/VSPAERO cross-checks and RocketPy powered-flight comparison, generator integration and native flight evidence |
| SIM-M03: MS-7d seeker, autopilot, fuze and lethality fidelity | Codex | Calibrated seeker/control response and explicit fuze/lethality assumptions; consume accepted radar/IR inputs and preserve native outcome meanings |
| SIM-M04: MS-8 flyout DLZ/WEZ | Codex | Flyout-derived envelopes with uncertainty and published-performance comparison; consume accepted missile models rather than treating advisory formulas as calibrated flyout |
| SIM-M05: MS-9b ESM and electronic attack | Claude | One warning/EA/countermeasure interaction family per package, modeled signal paths and legal platform reports; calibrated assumptions and encounter evidence |
| SIM-M06: MS-10 physical optical/IR sensors | Codex | Physical signatures/transmission and one IRST/targeting sensor family with defined measurement/image contracts; visualization consumes outputs |
| SIM-M07: MS-11 remaining payloads | Claude | Split terminals/PNT, IFF, weather/SAR/SMTI and other payload families before opening a write window. Each family includes discovery/control/BIT/reports and its own model evidence; existing DMS/C2 delivery is not reimplemented |
| SIM-M08: MS-12 OMS adapter | Claude | Versioned schema mapping, external transport adapter, service/subsystem status and protocol fixtures; keep external transport distinct from modeled RF delivery |
| SIM-M09: MS-7f and original MS M1 scenarios | Claude | Complete SDK example and encounter validation through accepted models: published range/flight-time/no-escape comparisons, measured 4v4 and the specified autonomous 1v1 Viewer loop. Native model corrections remain with their package owners |
| SIM-M10: stores/RCS/ballistic/model backlog | Codex | Individually scoped generated-store/RCS, ballistic, inertia and impact refinements with data provenance and native regression evidence |

These are planning leads, not simultaneous blanket write windows. Unstarted families remain pending. Fixes discovered in a consumed model return to its owner; no second implementation is created in an adapter, example or Viewer.

## 3. First wave and dependencies

The current handoff opens row 1's planning/input freeze only. Rows 2A-4 are the subsequent route; implementation starts only after the user relays a specific package and its exclusive path/build window is recorded.

| Order | Owner | First concrete delivery | Required stop point |
| --- | --- | --- | --- |
| 1: freeze the two first-package inputs | Codex / Claude, each for its owned package | Codex selects one missile and freezes geometry, motor data, admitted conditions, comparison sources and numerical tolerances. Claude selects one radar and one remaining processing/error/tracking gap, with the same source/criterion discipline | Record exact package scope, current HEAD and path/symbol allowlist before source edits. Missing data stay unavailable or explicitly assumed; do not manufacture calibration evidence |
| 2A: SIM-M02 first-model slice | Codex | Reproducible input-to-generator-to-native-model pipeline and independent solver/reference comparison for the selected operating envelope | Independent review of model provenance, assumptions, generated outputs and changed numerical behavior; stop before extending to a fleet |
| 2B: SIM-M01 first-radar slice | Claude | One named radar capability with native behavior, public usage, sourced limits and positive/negative encounter fixtures | Independent review; stop before adding another mode/family or changing missile calibration |
| 3: next weapon stages | Codex | SIM-M03, then SIM-M04, consuming the accepted model and sensor slices | Separate seeker/control/fuze and launch-zone acceptance; do not infer calibration from a working engagement |
| 4: complete encounter | Claude | SIM-M09 consumes accepted model packages and legal SDK control paths | Native events, reports and Viewer evidence must agree; detonation, damage, crew loss and mission success remain distinct. Preserve ADR-30's IFF-family prerequisite before any 2v2/4v4; the original 1v1 does not need that extra prerequisite |

The first-wave features can be developed independently only where their exact write allowlists are disjoint. Shared World, Agent, adapters, contracts, CMake and boundary files have one writer at a time. No native build/test or GUI slot is shared concurrently.

For SIM-M02, AIM-120C is an existing encounter-fixture candidate, not a promise that unpublished real parameters are available. Freeze the chosen model only after its admitted sources and uncertainty are recorded. Historical solver installation is preparation, not evidence that its model has been calibrated. Read upstream support matrices before any toolchain work; environments and caches remain under D:/ENV.

## 4. Separate HC-06 integration track

HC-06 remains a training integration prerequisite in the [carryover register](../../validation/handoff-carryover-register.md). It must not be confused with the physical-model packages above or treated as proof that all native equipment behavior is absent.

| Feature | Owner | Deliverable and boundary |
| --- | --- | --- |
| Authorization-profile and native extension inputs | Codex | Complete the independently usable profile loading/activation feature and freeze its typed native semantics and evidence fixtures; define the supplied registration/ownership seam and versioned outcome/criterion inputs |
| Generic gateway extension integration | Claude | Consume the accepted inputs; register the five request families, refresh legal reports/receipts and preserve existing service binding, tick/lifetime, deduplication and authority behavior |
| Full composition/batch/joint acceptance | Claude | Continue full M6/M7/M8 only when the relevant extension and outcome contracts are supplied; retain blocked cases and installed-consumer provenance |

The [extension seam](../../../sdk/mission-training.md#15-extension-seam-and-pending-gates) and [equipment contract](../../../sdk/ma-engagement-contract.md) govern those inputs. Existing schemas and Agent equipment calls are supplied. In particular, countermeasure dispensing and launch-zone queries already have public C++/Python paths in the [engagement API](../../../sdk/engagement.md); do not assign their reimplementation because an older registry note still calls parity proposed.

Before changing that registry, reconcile documentary operation names with current APIs while retaining frozen request kinds, schema fields and native refusal meanings. Do not mark `runtime_installed` true until the actual runtime handlers and registrations exist and are accepted. Neither a release receipt nor a detonation alone is a valid kill/reward reduction.

## 5. Execution, evidence and retained work

For every active package, record the current main HEAD, exact source/symbol allowlist, admitted data, fixtures, expected changes and stop point. Model implementation remains under `src/simulation/mission_systems`; generated model sources/generators retain their documented ownership. Do not hand-edit generated aircraft/store XML. New private dependencies must follow the accepted architecture and public ABI remains compatible.

Run the changed feature's quick checks during development. Reuse independently accepted evidence only with its dated source and limits. Any later full run requires closed review findings, passing relevant checks and a frozen final code state; test selection alone never authorizes execution. One native configure/build/test slot and one complete GUI scenario slot remain exclusive.

Keep all discovered limitations in the carryover register with an owner and closure condition. HC-07 still has HC-45 performance, HC-06 combat inputs, HC-02 J6, human/uncaptured interaction gates; HC-52 is a nonblocking resize-helper follow-up. HC-47 to HC-51 are closed within their stated scope. MS planning does not close or silently absorb those items, reopen HC-11, start renderer optimization, or claim a full-program acceptance.

After each complete vertical package, release shared write/build slots and stop for independent acceptance. Do not begin the next model family merely because a plan lists it.
