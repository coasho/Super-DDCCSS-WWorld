# Mission Systems work allocation

Date: 2026-10-17. Initial planning baseline: main `5c80892`; source and implementation coverage rechecked at main `6170ec6`. This allocation starts MS planning; it makes no new implementation, protocol-conformance or model-acceptance claim.

## 1. Scope and authority

The [simulation plan](plan.md#5-functional-roadmap-retained-after-structural-work) defines SIM-M01 to SIM-M10. The [original MS roadmap](adr-30-mission-systems.md#8-stages) defines their physical-model goals and original Mission Systems M1. Those remain the completion criteria. Directory migration, a native missile engagement and training M0-M8 core do not complete these physical-model milestones.

SDK remains the simulation authority. Real MA planners, behavior trees, blackboards and DRL training belong in the upper-layer project. SDK examples are minimal runnable demonstrations. This plan does not transfer Claude's existing generic gateway, composition, batch, compatibility and installed-consumer ownership in the [training allocation](../../../superpowers/plans/2026-10-12-ma-system-training-work-allocation.md).

Each owner delivers its complete feature: native behavior, required public C++/C99/Python interfaces, generated-model inputs, examples, meaningful regressions, documentation and acceptance evidence. Do not divide work by language or implementation layer. The other agent reviews the handoff independently; review does not transfer ownership.

### 1.1 Mandatory public-data fidelity, open-source reuse and fleet coverage

These requirements apply to **every MS payload family and actual equipment model/variant**, including the project extensions in 6.3. They are completion gates for both owners and all SIM-M packages, not optional improvements after the first radar or missile slice.

- **Build from real public evidence.** Identify the actual fitted equipment and vehicle variant. Prefer manufacturer/operator/government technical publications, then original engineering literature and independently corroborated secondary sources. For each model input, record the exact source/version and location, units, operating/measurement conditions, provenance, derivation where applicable, uncertainty and whether it is published, derived or assumed. A source establishing a band, equipment name or installation does not establish a particular frequency, power or performance value; a general textbook supports a method, not the actual equipment fit or its specific parameters. Where public data is missing, derive a physically justified bound or explicitly retain an uncertain assumption/unknown; never invent a published value. Seek better evidence and validate its effect before closing the model.
- **Pursue the highest defensible simulation fidelity.** Model the payload's relevant physical mechanisms, operating modes, timing/scheduling, measurement errors, environmental effects, resource limits and interactions with other native systems as fully as public evidence and validated methods permit. Freeze quantitative reference conditions and tolerances before implementation; test uncertainty/sensitivity and failure/unavailable cases. Separate calibration inputs from independent validation cases and report residuals and extrapolation limits. Any reduced-order approximation or performance tradeoff must state its lost behavior, evidence and remaining closure condition; a working interface, a familiar equipment name or visual similarity cannot close a fidelity gate.
- **Prefer mature open-source implementations.** Before writing a new model or solver, evaluate relevant established open-source projects for physical scope, validation evidence, maintenance, license, numerical behavior, determinism, runtime cost and integration compatibility. Prefer adopting or adapting a suitable proven implementation over rebuilding it. Record the candidate/version, evaluation, selected reuse and any reason a custom implementation is necessary in this document's package handoff/evidence. Reuse may be an offline reference/generator or a runtime component as appropriate; analysis tools need not become simulation-core dependencies. Validate the integration against upstream reference cases as well as the SDK use case; reuse alone does not establish accuracy. Preserve the accepted dependency direction, SDK simulation authority and complete feature ownership.
- **Cover every current vehicle with sufficient real equipment models.** Independently enumerate the project's currently supported vehicle types and configured variants from its maintained design/support/packaging roster, not only from existing payload fits. Reconcile supported stock types/aliases explicitly; a visual stand-in is not a payload fit. For every vehicle, map its actual carried MS equipment and modes to sourced model/variant cards, implemented behavior, owner and acceptance evidence. Add missing equipment models and functional coverage as necessary. Recount when the fleet changes. The present 35-airframe fit catalogue is a dated starting point, not a permanent denominator or proof that every real installation is already listed.
- **Measure functional and variant coverage.** Common physics engines and genuinely shared equipment may be reused across vehicles, but each claimed real model/variant needs justified parameters and supported modes, with per-fit SDK execution and provenance evidence. Do not substitute a generic radar, cloned defaults, an empty/status-only card or an unconfirmed model designation for an accepted actual-model implementation. Explicitly distinguish confirmed not-fitted equipment, unverified installation/model identity, missing public data and unfinished simulation. Unknown or unfinished entries remain open and prevent a claim of complete fleet coverage; they cannot be relabeled not fitted merely to remove them from the denominator.

Before a model package starts, its owner freezes the public inputs, open-source evaluation and affected vehicle/model coverage rows here. Before overall MS acceptance, every current vehicle/equipment/mode row must have its actual model and required public-interface, physical and integration evidence accepted, or a specifically user-approved scope change recorded. Success of a representative first slice is not fleet-wide acceptance. This requirement opens no additional implementation window and does not silently expand the admitted environment/entity scope.

## 2. Responsibilities

| Existing package | Planned lead | Complete feature and acceptance focus |
| --- | --- | --- |
| SIM-M01: radar errors, processing and tracking/fusion | Claude | One radar family and one named processing gap at a time; sourced error/processing limits, native measurement and tracking behavior, legal reports, public usage and repeatable fixtures. Retain MS-4b/MS-5/MS-6 distinctions |
| SIM-M02: MS-7c missile aerodynamics and propulsion | Codex | First representative missile model, geometry and motor provenance, SU2/VSPAERO cross-checks and RocketPy powered-flight comparison, generator integration and native flight evidence |
| SIM-M03: MS-7d seeker, autopilot, fuze and lethality fidelity | Codex | Calibrated seeker/control response and explicit fuze/lethality assumptions; consume accepted radar/IR inputs and preserve native outcome meanings |
| SIM-M04: MS-8 flyout DLZ/WEZ | Codex | Flyout-derived envelopes with uncertainty and published-performance comparison; consume accepted missile models rather than treating advisory formulas as calibrated flyout |
| SIM-M05: MS-9b ESM, COMINT and electronic attack | Claude | One warning/collection/EA/countermeasure interaction family per package, modeled signal paths and legal platform reports; calibrated assumptions and encounter evidence |
| SIM-M06: MS-10 passive and active optical/IR sensors | Codex | Physical signatures/transmission, IRST/targeting sensors and the separately identified AO capabilities below, with defined measurement/image contracts; visualization consumes outputs |
| SIM-M07: MS-11 remaining MS families and services | Claude | Separate terminal/relay/gateway, PNT, weather, SAR, SMTI, air sampling, cargo, refuel, system deployment, counter-space and tactical-order packages, plus native MS plan/status/fault services. IFF is a separate project extension. Apply the explicit dispositions below before opening a write window; existing DMS/C2 delivery is not reimplemented |
| SIM-M08: MS-12 OMS adapter | Claude | Versioned schema mapping, external transport adapter, service/subsystem status and protocol fixtures; keep external transport distinct from modeled RF delivery |
| SIM-M09: MS-7f and original MS M1 scenarios | Claude | Complete SDK example and encounter validation through accepted models: published range/flight-time/no-escape comparisons, measured 4v4 and the specified autonomous 1v1 Viewer loop. Native model corrections remain with their package owners |
| SIM-M10: guns, remaining weapon/store models and model backlog | Codex | Explicit gun firing/ammunition/projectile and air-to-ground weapon packages, plus individually scoped generated-store/RCS, ballistic, inertia and impact refinements. Consume existing release/flight behavior and retain data provenance and native regression evidence |

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

## 6. A-GRA MS source coverage and payload allocation

### 6.1 Pinned source, applicability and evidence rules

This crosswalk was checked against the original **MS L1 Interface Volume v6.0a, 31 July 2026**, and its companion schema at upstream commit `aaabf2cd5c341002e6c62d1448460b55b5634931`:

- [MS Interface Volume](https://github.com/open-arsenal/a-gra/blob/aaabf2cd5c341002e6c62d1448460b55b5634931/Documentation/ASK_6.0a_MS_Interface_Volume.pdf), SHA-256 `e8cfe2e09ea41c14c663b9a7622ab85f2a31ad56856ea0cace14cec2093c72a7`.
- [A-GRA 6.0a schema](https://github.com/open-arsenal/a-gra/blob/aaabf2cd5c341002e6c62d1448460b55b5634931/Schema/A-GRA_MessageDefinitions_v6_0_a.xsd), SHA-256 `9efa174eda548bf27c8ca4afaaba12c6dca4cc259d799bcc4a098df968cfc38d`.

Page references below are the PDF's printed page numbers, which equal its physical page numbers. Section 1.2 contains **67 numbered subsections in ten packages**: 14 communications, 5 contingencies, 4 COP, 2 fusion, 10 DMS, 2 plan behaviors, 1 PNT, 11 sensor tasking, 5 status and 13 weapon employment. The six DMS subsections 1.2.5.5-1.2.5.10 specify crosscutting protocol requirements rather than additional sequence diagrams. Table 1-65, pp.134-139, contains **139 distinct message types: 101 UCI 2.3 and 38 A-GRA extensions**. The earlier ADR-30 summary of 128 (99 + 29) is not the inventory for this pinned source; use section 8 below for this allocation.

The source's interface-compliance requirements (Table 1-64, p.133, and section 1.3.1) bind Core plus the other Mission Use Cases (MUCs) applicable to the platform. A Core capability/status message form is a required protocol obligation even where the SDK has no fitted physical payload. Supporting a message form does not authorize advertising nonexistent hardware or pretending its physical model is complete. Conversely, a family already implemented natively is not absent merely because its OMS mapping is pending. A platform profile must explicitly record fitted hardware, admitted modes, unsupported functionality and the associated MUCs.

Coverage here means that every source obligation has a named owner, current disposition and closure condition. It does not mean that every family is implemented, that every aircraft carries every payload, or that all global UCI schema types are MS requirements. The source includes AirSample, AO, CargoDelivery, COMINT, CommRelay, CounterSpace, Refuel, SystemDeployment and TacticalOrder capability/status forms in Tables 1-46 and 1-65; these are not dismissible as unrelated schema entries. Other types appearing only in shared schema/appendix taxonomies do not automatically become required MS payloads.

Implementation evidence at `6170ec6` is [the SDK family/support inventory](../../../sdk/mission-systems.md), [capability construction](../../../../src/simulation/mission_systems/Cards.cpp), [operational support overrides](../../../../src/simulation/mission_systems/Host.cpp), [model construction](../../../../src/simulation/mission_systems/Model.cpp), [stores](../../../../src/simulation/mission_systems/HostStores.cpp) and [engagement permissions/reports](../../../sdk/engagement.md). A declared `PayloadTask` enum or capability name is scaffolding unless its public execution path and native model exist.

Every delivery below has these common acceptance conditions:

1. **Model and fit:** satisfy 1.1's public-data, open-source and fleet-coverage gates; identify the actual aircraft/payload, admitted operating envelope, source/license, assumptions, uncertainties and measurable tolerances before implementation. Unsupported or not-fitted cases remain explicit. Freeze any required new environment/entity scope before admitting it; ADR-30's current no-ground-entities boundary remains in force.
2. **Native execution:** exercise the legal SDK command path, measurements/outcomes, power/BIT/failure states, resource consumption, cancellation and deterministic seeded behavior, including negative and unavailable-data cases. Preserve actor/referee visibility and native event meanings.
3. **Public feature:** the native feature owner supplies the required C++/C99/Python feature, documentation and meaningful regressions together, preserving ABI and installed-data provenance. Do not duplicate the model in an adapter, Viewer or training wrapper.
4. **Protocol:** Claude's SIM-M08 adapter consumes the accepted native feature and demonstrates schema-valid messages plus the extra fields required by the MS use-case tables, original identity/correlation, direction, state progression, scheduling, periodic/change-driven publication and refusal behavior. Schema validation alone does not satisfy a use-case table. A local native call is not an accepted OMS exchange.
5. **Model evidence and integration:** compare physical outputs against independently admitted references/solvers and frozen tolerances; then run the related nonempty fast checks. SIM-M09 consumes accepted models for encounter, measured real-time and Viewer evidence. Neither successful protocol exchange nor a screenshot proves calibration. Full suites remain subject to section 5's frozen-final-state gate.

### 6.2 Payloads and MS capability families

`Declared` means native discovery/power/status/BIT exists without a complete operational model. `Partial` identifies useful delivered behavior with named remaining work. `Absent` means no corresponding dedicated native family/model has been delivered. **All external MS message mappings below remain SIM-M08 acceptance work**, including mappings for already operational native features. Physical/model leads in this table own complete vertical features; Claude owns the separate complete external-adapter feature.

| Payload / capability and original anchor | Modes / data contract to freeze | Current implementation at 6170ec6 | Vertical lead / package | Specific acceptance condition, in addition to 6.1 |
| --- | --- | --- | --- | --- |
| AMTI / `air_radar`; 1.2.8.1/.4/.8, pp.78-84/92-93; Core capability/status and AMTI MUC | `AMTI_CapabilityEnum`: `VOLUME`, `CUED`, `TRACK`; subcapabilities `NAS`, `AAS`, `ACTIVE_TRACK_UPDATE`, `AUTO_SUBCAPABILITY_SELECT`. SDK RWS/velocity/TWS/surveillance/acquisition/STT and illumination are separately named native modes | Partial: detection-level search/acquisition/tracking, signal processing and STT illumination exist. Signal-level CUED/TRACK/STT and velocity combinations are refused; sourced error floors/glint/boresight and fusion refinements remain | Claude / SIM-M01 | Test every advertised mode at each processing level; field-of-regard, frequency, clutter/measurement errors, track initiation/update/loss and scheduling must agree with discovery. Illumination remains an explicit SDK extension, not a fabricated AMTI enum |
| ESM / RWR / `esm`; 1.2.8.2/.5/.9, pp.79-80/84-87/93-94; Core declarations and ESM MUC | `CapabilityInitiationEnum` and `ESM_SubCapabilityEnum` in 6.3; acquisition, direction finding, identification, geolocation and measurement are distinct functions | Partial: measured emitter contacts, search/lock/illumination warning and active-seeker emissions exist; full collection/geolocation and calibrated receiver behavior remain | Claude / SIM-M05 | Compare received signals, detection/bearing uncertainty, association/classification, single/multiple-receiver geolocation and unavailable signals against admitted cases. Resolve the volume/schema discrepancy in 9 before freezing wire commands |
| Passive optical / `passive_optical`; 1.2.8.3/.6/.10 and 1.2.9.3, pp.80-81/87-90/94-96/103-104; PO MUC | MS-specific `SEARCH_VOLUME` and `TRACK`; EO/IR, FLIR/IRST/targeting hardware fit. Additional schema imaging/search/tracking modes are listed separately in 6.3 | Declared; no complete physical PO payload model. Existing native IR missile seekers and rendering are separate delivered features | Codex / SIM-M06 | Source signatures, atmosphere/transmission, aperture/bands/FOR, detection/track uncertainty and permitted measurement/image output. Validate search/track and required periodic component/lens/MDF/output configuration; accept extra modes individually |
| Active optical / AO; Tables 1-46/1-65, pp.100/134; Core declaration/status forms | `AO_CapabilityEnum`: `ILLUMINATION`, `DESIGNATION`, `TARGET_MARKING`, `LADAR`, `RANGING` | Absent as a dedicated family; passive EO/IR or radar illumination does not supply AO | Codex / SIM-M06 | Freeze a sourced fitted optical emitter/receiver and each admitted function; validate beam/pointing/occlusion, range or designation/marking evidence and any modeled weapon coupling. Core codec/status coverage is required independently; full AO commands are an explicitly selected extension |
| SAR / `sar`; Core declarations, pp.138; no dedicated SAR command sequence in 1.2.8 | `SAR_CapabilityEnum`: `SPOT`, `STRIP`, `AREA`; `SAR_SubCapabilityEnum`: `SAR`, `ISAR` | Declared, no functional collection/imagery model | Claude / SIM-M07 | For each admitted mode, freeze geometry, aperture/motion, resolution/swath/error and image metadata/reference targets. Retain honest status for unimplemented modes; a terrain image does not prove all SAR/ISAR functions |
| SMTI / `smti`; Core declarations, p.138; no dedicated SMTI command sequence in 1.2.8 | `SMTI_CapabilityEnum`: `AREA`, `SPOT`, `CUED`; subcapabilities `MMTI`, `GMTI`, `MIXED_BALANCED`, `MIXED_MMTI_DOMINANT`, `MIXED_GMTI_DOMINANT` | Declared, no functional surface detection/tracking model; required target/environment inputs are not admitted by the existing no-ground-entities scope | Claude / SIM-M07 | Freeze the admissible target source and any scope decision first; validate clutter, minimum detectable velocity, false alarms, detection/tracking errors and mode-specific reports. Keep functional modes pending if prerequisites are absent; do not substitute aircraft truth |
| Weather radar / `weather_radar`; 1.2.8.11, pp.96-97; Core declarations, Weather Radar observation MUC | `WeatherRadarCapabilityEnum`: `SECTOR`; radar observations are separate from external weather-service reports | Declared, no physical weather-radar measurement model | Claude / SIM-M07 | Use an admitted weather field and sourced sensor response; verify sector/FOR, attenuation/return units, missing observations and WeatherRadarObservation publication independently of WeatherMT |
| Electronic attack / `electronic_attack`; Core declarations, p.135 | `CapabilityInitiationEnum`; technique activation `PREEMPTIVE`/`REACTIVE`, pointing `LOCATION`/`BEARING`/`TRACK`/`OMNI`; see 6.3 | Declared, no full operational jammer model | Claude / SIM-M05 | Freeze one supported technique and source/receiver/link interaction; verify power/band/geometry, activation/cancel/resource limits and measured effect. Do not count the native chaff/flare dispenser as full EA |
| COMINT; Core declarations, p.134 | `CapabilityInitiationEnum` and `COMINT_SubCapabilityEnum` in 6.3 | Absent as a dedicated capability; RWR/ESM contacts and decoded DMS traffic are not COMINT | Claude / SIM-M05 | Admit one sourced receiver/collection mode; verify modeled signal availability, bearing/identity/location or permitted collection output and unavailable cases. Do not manufacture intercepted message contents |
| AirSample; Core declarations, p.134 | `AirSampleCapabilityEnum`: `SPECTROMETER`, `SAMPLE` | Absent; no fitted sampler or sampled-environment model admitted | Claude / SIM-M07 | Core codec/status coverage plus explicit fit/profile disposition. Physical completion requires a sourced sampler, environment/sample input, timing/volume/units/uncertainty and contamination/empty/failure cases; otherwise retain the physical model as pending |
| Communication terminal / `comm_terminal`; 1.2.1, pp.18-40 | Waveform `CommCapabilityEnum` and `CommDataFormatEnum` in 6.3; terminal plans, direct commands, precedence, statistics and activities | Declared; modeled RF-aware DMS/C2 delivery exists, physical terminal-specific controls and quality model remain | Claude / SIM-M07 | Admit each fitted waveform/format, bandwidth/latency/range/queue and failure/failover model. Verify terminal-plan activation, configured settings and measured link statistics; an enum does not implement every waveform |
| Communications relay / CommRelay; Core declarations, pp.134-135 | `CommCapabilityEnum`; relay routing/availability and resource limits | No dedicated CommRelay capability/model; existing link delivery is a foundation | Claude / SIM-M07 | Freeze the fitted relay and supported networks; verify reachable paths, relay delay/capacity, routing/failover and no direct delivery across an unavailable modeled path. Keep protocol discovery distinct from physical relay completion |
| Gateway, including DMS; 1.2.1.3/.6/.12-.14 and 1.2.5 | Terminal/network association; ingress/egress, directed/topic routing, IER/distribution/rate groups, plans/effectivities/overrides | Partial: generic modeled DMS/C2 queues and reports exist; full source gateway-plan semantics and DDS/OMS protocol evidence remain | Claude / SIM-M07 native gateway feature; Claude / SIM-M08 external adapter | Run all applicable communication/DMS rows in 7, including plan activation/override, route/filter/QoS/expiry and endpoint changes. Existing modeled delivery and local pipe HMAC do not establish DDS protocol compliance |
| PNT / `pnt`; 1.2.7.1, pp.76-78; PNT MUC | Status/settings plus `MA_PositionReportDetailedMT`: position/time, orientation/rates/acceleration, magnetic heading, wander angle and air data; no dedicated MS capability-mode enum | Declared; ownship/navigation reports exist, receiver/navigation-solution quality model remains | Claude / SIM-M07 | Validate source frame/units/time, configured publication frequency, error/validity/age/outage behavior and all use-case-required fields. Exact World state is not proof of a modeled navigation solution |
| Refuel; Core declarations, p.138 | `RefuelCapabilityEnum`: `BOOM`, `DROGUE`, `UNDEFINED` | Absent as an MS refuel capability; existing external-tank fuel transfer is not aerial refueling | Claude / SIM-M07 | Core codec/status plus explicit platform/profile fit. Operational acceptance requires admitted tanker/receiver hardware, connection/envelope, conservation/transfer limits and disconnect/failure cases; flight-control prerequisites remain explicit |
| CargoDelivery; Core declarations, p.134 | `CargoDeliveryCapabilityEnum`: `EXTERNAL_SLING_LOAD`, `INTERNAL_BAY`; subcapabilities `STATIONARY_LAND`, `HOVER` | Absent; store carriage/release is not a complete cargo delivery capability | Claude / SIM-M07 | Core codec/status plus explicit fit/profile decision. Freeze supported payload attachment, mass/drag/load transfer, placement/release and failure outcomes; any landing/hover/environment prerequisite is separately admitted |
| SystemDeployment; Core declarations, p.139 | `SystemDeploymentEnum`: `CARRY`, `RELEASE`, `TOW`, `LAUNCH`, `RETRACT`, `RECOVER` | Absent as a dedicated capability; weapon release alone is insufficient | Claude / SIM-M07 | Core codec/status plus a named deployable system and each admitted operation; verify attachment/lifetime, mass/forces, sequencing, recoverability and refusal/failure. Unadmitted operations remain pending |
| CounterSpace; Core declarations, p.135 | `CS_CapabilityType.CapabilityType`: `CapabilityInitiationEnum`; `CS_KindEnum`: `EA`, `ES`, `SA`, `OTHER` | Absent; no counter-space/space-environment physical profile admitted | Claude / SIM-M07 physical profile/disposition; Claude / SIM-M08 message forms | Implement the required Core forms without advertising hardware. A physical package requires an explicit scope/environment decision, sourced subsystem and measurable effect; no space model is opened by this plan |
| TacticalOrder; Core declarations, p.139; service capability rather than a hardware payload | `AIR_TO_AIR_ROLE`, `AIR_TO_SURFACE_ROLE`, `CLEAR_AREA`, `GO_TO_VOICE`, `LAUNCH_ALERT_SYSTEMS`, `LOITER`, `MUST_FLY`, `ORBIT`, `RECALL`, `REFUEL`, `RETURN_TO_BASE` | Existing native C2 task/role behavior is partial basis, not accepted TacticalOrder protocol coverage | Claude / SIM-M07 | Freeze the mapping of each advertised order to an accepted native activity/permission path, state/cancellation/refusal and unsupported outcomes. Do not implement an upper-layer MA planner in the SDK |
| Strike/store stations, including missile families; 1.2.10, pp.109-133 | `MA_StrikeCapabilityType.CapabilityType` is `StoreType` (type/category/variant), not a strike-mode enum; inventory, prepare, consent, release, abort, Standard/Emergency jettison, flyout; radar/IR seeker and weapon variants are native model choices | Partial: inventory/loadouts, native consent/strike/jettison/released flight, AAM guidance/fuzes/damage and advisory zones exist | Codex / SIM-M02 propulsion/aero; SIM-M03 seeker/control/fuze/lethality/feedback; SIM-M04 DLZ/WEZ; SIM-M10 store lifecycle/model refinements | Preserve the existing complete paths. Accept independent calibrated flight/seeker/fuze/damage models, actor-permitted weapon feedback, and the separate DLZ/WEZ contracts and all 13 weapon exchanges in 7. Training extension integration remains section 4 |
| Gun / `gun`; Strike store category `GUN`, not a separate MS MMS family | Single/burst/continuous operation only where supported by the sourced gun; ammunition/projectiles/recoil/impact and mass accounting are native model contracts | Declared/status only; Strike remains NotImplemented. TaskKind Fire is not a submitted, executed gun feature | Codex / SIM-M10 | Deliver a named gun/ammunition model, safe refusal and fire/stop timing, conserved ammunition/mass, sourced projectile trajectory/dispersion and native collision/damage evidence; no phantom release or repeated-command firing |
| Air-to-ground weapons; Strike store categories, not 13 new weapon types inferred from the 13 sequences | Guided/unguided stores and supported fuze/target/reference modes must be named per model; `StoreCategoryEnum` is listed in 6.3 | Partial: Mk 82 release/native JSBSim flight exists; ballistic validation remains. CJ-10A is an inert body, not an operational powered/guided cruise missile | Codex / SIM-M10 ballistic/store/model feature; SIM-M02/03 for any separately admitted powered/guided model | Reuse release/flight. Validate trajectory/dispersion, arming/fuze/terrain-relative impact and event meaning against frozen data. A functional guided weapon requires accepted propulsion/seeker/control inputs. Do not silently add ground targets, SAMs, ships or ground emitters |
| Countermeasures / `countermeasures`; native equipment mapped through EA/strike-related contracts as appropriate | Existing single-cartridge chaff/flare dispensing; programmed quantities/salvo timing and future decoy types are separately planned, not executed PayloadTask features; no invented MS-specific dispense enum | Partial: native inventory, dispensing, motion/expiry and seeker competition exist even though general fsim.ms.ea remains unimplemented | Claude / SIM-M05 | Preserve existing SDK calls; calibrate material/signature/kinematics and seeker interaction, verify depletion/expiry and legal reports. Register the existing training request seam under HC-06; do not reimplement dispensing |

The absent or not-admitted families above remain named work, with an owner and closure condition, rather than being hidden under “other payloads.” Profile decisions can defer a physical model but cannot claim model completion or silently delete required Core message support.

### 6.3 Exact mode taxonomies and project extensions

The following are schema choices, not a promise that the SDK supports every value. Each future fit records a supported subset; unavailable values must not be advertised as operational. The MS volume mandates the narrower use-case behavior where specified, especially PO search/track. These lists prevent a broad family label from concealing unassigned modes.

| Exact schema field/type | Complete relevant choice set | Allocation / limitation |
| --- | --- | --- |
| `CapabilityInitiationEnum` used by ESM, COMINT, EA and CS capability types | `AUTO`, `MANUAL` | Initiation taxonomy, not an ESM search/track enum. Collection/technique functions are separate fields |
| `ESM_SubCapabilityEnum` | `ACQUISITION`, `DIRECTION_FINDING`, `EMITTER_TYPE_IDENTIFICATION`, `SPECIFIC_EMITTER_IDENTIFICATION`, `SINGLE_SUBSYSTEM_GEOLOCATION`, `MULTI_SUBSYSTEM_GEOLOCATION`, `FINE_FREQUENCY_MEASUREMENT`, `PRI_MEASUREMENT`, `MODULATION_MEASUREMENT`, `SCAN_MEASUREMENT`, `PULSE_DATA_COLLECTION`, `SPECTRAL_DENSITY_REPORTING` | Claude / SIM-M05; each selected function needs its own measurement/uncertainty contract |
| `COMINT_SubCapabilityEnum` | `ACQUISITION`, `DIRECTION_FINDING`, `EMITTER_TYPE_IDENTIFICATION`, `SPECIFIC_EMITTER_IDENTIFICATION`, `SINGLE_SUBSYSTEM_GEOLOCATION`, `MULTI_SUBSYSTEM_GEOLOCATION`, `FINE_FREQUENCY_MEASUREMENT`, `MODULATION_MEASUREMENT`, `SCAN_MEASUREMENT`, `DATA_COLLECTION`, `NETWORK_IDENTIFICATION`, `DEMODULATION_AUDIO`, `DEMODULATION_DATA`, `IQ_FORWARDING`, `SPECTRAL_DENSITY_REPORTING`, `OTHER` | Claude / SIM-M05; signal collection is separate from having access to decoded platform DMS data |
| `PO_CapabilityEnum` | `SPOT_IMAGE`, `AREA_IMAGE`, `LINE_SWEEP`, `FULL_MOTION_VIDEO`, `FOCUS_SWEEP`, `SEARCH_VOLUME`, `SEARCH_CUED`, `SEARCH_ATR`, `TRACK_SINGLE`, `TRACK_MULTI`, `CALIBRATE`, `SEARCH`, `TARGET_CLASSIFICATION`, `TRACK`, `AUTO_CUE`, `PERSISTENT_IMAGING`, `ANOMALY_DETECTION`, `VISUAL_MAGNITUDE_MEASUREMENT`, `SOB_CORRELATION` | Codex / SIM-M06; SEARCH_VOLUME/TRACK are the MS-specific MUC requirements, other choices are individually admitted extensions |
| `CommCapabilityEnum` | `FALCON`, `5G`, `MBITR`, `AEHF`, `JTRS`, `BOWMAN`, `C_BAND`, `CDL`, `CMF`, `DSCS`, `EHF_SATCOM`, `EPLRS`, `HAVE_QUICK`, `HF`, `HNW`, `IFDL`, `INMARSAT`, `IP`, `IRIDIUM`, `KA_SATCOM`, `KU_SATCOM`, `L_BAND`, `LINK4`, `LINK11`, `LINK16`, `LINK22`, `MADL`, `MF`, `MILSTAR`, `MUOS`, `S_BAND`, `S_BAND_SATCOM`, `SADL`, `SATURN`, `SINCGARS`, `SRW`, `TCDL`, `TTNT`, `UHF`, `UHF_SATCOM`, `VDL`, `VHF`, `VMF`, `TDRSS`, `FLTSATCOM`, `NCCT`, `VORTEX`, `WGS`, `WNW`, `X_BAND`, `PTW`, `STARLINK`, `OTHER` | Claude / SIM-M07 terminals/relay; SIM-M08 maps only the accepted fitted subset. Schema presence alone supplies no waveform implementation/calibration |
| `CommDataFormatEnum` | `IP`, `AIS`, `CSEL`, `PRODUCT`, `AFAPD`, `UCI`, `LINK_11`, `LINK_16`, `LINK_22`, `CMF`, `VMF`, `IFDL`, `MADL`, `STANAG_4607`, `USMTF`, `JREAP_C`, `OTHER` | Claude / SIM-M07/08; native RF delivery and external data format are separate contracts |
| `StoreCategoryEnum` | `AIR`, `SEA_SURFACE`, `SEA_SURFACE_SAMSSM`, `SEA_SUBSURFACE`, `GROUND`, `GUN` | Codex weapon packages; the schema's sea/ground categories do not override the current scenario/entity boundary |
| EA technique fields | `EA_ActivationEnum`: `PREEMPTIVE`, `REACTIVE`; `EA_PointingEnum`: `LOCATION`, `BEARING`, `TRACK`, `OMNI` | Claude / SIM-M05; `EA_ModeEnum` is attached to `JammerEmitterIdentityType.JamMode`, not directly to EA_CapabilityType |

| Additional project feature, outside this MS volume's dedicated MMS/interaction inventory | Current implementation | Lead / acceptance |
| --- | --- | --- |
| IFF interrogator/transponder | No dedicated native family. IFF appears as a cooperative sensor example and in shared capability taxonomies, but there is no dedicated IFF MMS row or IFF command/activity sequence in this MS volume | Claude / SIM-M07. Freeze the fitted interrogator/transponder, query/reply/identity/quality and modeled RF/report path. Required by the project before 2v2/4v4 acceptance; not a prerequisite for the original 1v1. Weapon “Confirm Threat ID” is not this feature |
| Real-beam ground-map / terrain-avoidance radar, `ground_map_radar` | Native status-only family; not the SAR capability and no matching dedicated MS MMS capability in this volume | Claude / SIM-M07 if explicitly admitted. Freeze sourced fit/FOR/terrain measurements and a separately named extension; otherwise retain status-only support |
| Tank and pod carriage | Existing fuel transfer, mass/drag and jettison; pod sensing/jamming remains separate | Codex / SIM-M10 retains carriage/model behavior; the complete active payload feature stays with its AO/PO/EA owner. A carried pod does not prove that its sensor/jammer operates |

Carrier is an applicability label on `SubsystemStatusMT` and `SystemStatusMT`, not proof of an implemented aircraft-carrier environment. Claude / SIM-M07/08 owns the explicit platform-profile disposition and status mapping; any physical carrier capability requires a separate admitted scenario/environment prerequisite.

## 7. Complete MS interaction and service register

Each row corresponds to one numbered subsection of the pinned MS volume. Page is the subsection's first printed page. **External wire realization is planned for Claude / SIM-M08 in every row**; no row claims current OMS/UCI conformance. The semantic lead column identifies the owner of the complete native feature consumed by that adapter. Existing basis must be reused. Conditional MUCs are recorded where the source names them; section 8 preserves the message-level applicability even when it differs from a subsection's broader description.

### 7.1 Communications Service (14)

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.1.1 / 18 | Communications Lost | Modeled link loss exists; terminal precedence/failover and wire status remain | Claude / SIM-M07 | Degrade one link, switch to an admitted redundant link, then lose all links; publish availability reasons/precedence and SystemStatus LOST only for the all-unavailable case |
| 1.2.1.2 / 19 | Configure Comm Terminal | Terminal declared; direct configuration/activity pending; Additional Comms Capabilities MUC | Claude / SIM-M07 | Accepted command identifies its activity; rejected command reports a reason; settings affect only the addressed fitted terminal |
| 1.2.1.3 / 21 | Configure Gateway | Modeled DMS exists; direct gateway command/activity pending; Additional Comms Capabilities | Claude / SIM-M07 | Accept/reject configuration, publish active gateway activity and demonstrate its ingress/egress effect |
| 1.2.1.4 / 23 | Establish Comm Terminal | Links exist; CommSystem/Asset/Capability/Schedule descriptions not accepted | Claude / SIM-M07 | Every described CommAssetID resolves; publish complete fitted system/capability and scheduled allocation information |
| 1.2.1.5 / 25 | Maintain Comm Terminal | Existing delivery counters are partial basis; full waveform/activity status pending | Claude / SIM-M07 | Publish actual connection, common/waveform settings and link statistics while operational; updates agree with modeled delivery |
| 1.2.1.6 / 26 | Override Data Dissemination Plan | Plan/effectivity/override feature pending; Additional Comms Capabilities | Claude / SIM-M07 | Temporary override accept/reject/cancel, processing reason, activation status and gateway activities remain correlated; removing it restores the admitted plan behavior |
| 1.2.1.7 / 28 | Publish Comm Team Report | Native team/link reports are basis; source message mapping pending | Claude / SIM-M07 | Join/leave and empty team cases report the correct SystemID/EndpointLabel set |
| 1.2.1.8 / 29 | Publish Network Endpoint Availability | Native reachability is basis; source endpoint publication pending | Claude / SIM-M07 | Publish on membership/reachability change, including no available endpoints; do not expose unreachable peers |
| 1.2.1.9 / 30 | Receive Communication Error Report | Native refusal/drop reasons are basis; explicit report mapping pending | Claude / SIM-M07 | Error report ID, affected SystemID, source and ErrorDetail identify the actual fault |
| 1.2.1.10 / 31 | Request Communication Asset Status | Asset estimation request/result pending; Additional Comms Capabilities | Claude / SIM-M07 | Correlate the requested CommAssetID and return admitted estimated position/status, not referee truth |
| 1.2.1.11 / 32 | Update Comm Terminal Plan | Terminal plan, triggers and activation pending; Additional Comms Capabilities | Claude / SIM-M07 | Store referenced plan, apply effectivity/configuration, report accepted/in-progress/terminal or rejected activation and honor triggers |
| 1.2.1.12 / 34 | Update Comms Gateway Status | Full gateway capability/activity publication pending; Additional Comms Capabilities | Claude / SIM-M07 | Every active gateway's capability/status and activity accurately describe configured over-the-air ingress/egress |
| 1.2.1.13 / 35 | Update Data Dissemination Plan | IER/distribution/rate groups and activation pending; Additional Comms Capabilities | Claude / SIM-M07 | Exercise filters, directions, topic/directed destinations, wrapping, rate groups and linked effectivities; invalid references or unavailable routes fail explicitly |
| 1.2.1.14 / 39 | Update Data Dissemination Plan Activation Status | Plan activation/override state publication pending; Additional Comms Capabilities | Claude / SIM-M07 | Periodic and change-driven status identifies active effectivities and override modifications |

### 7.2 Contingencies (5)

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.2.1 / 40 | Intra-Vehicle Comms Failure_MS | SDK lifetime/link refusal exists; service heartbeat/query timeout mapping pending | Claude / SIM-M07 | Withhold and restore service status, handle the explicit status request, and expose timeout/recovery. The upper-layer MA chooses its contingency response |
| 1.2.2.2 / 41 | Logging Failure | General failure/BIT exists; logging-service fault sequence pending | Claude / SIM-M07 | Inject a real logging-service failure, run/report BIT and identify completed BIT plus failure reason |
| 1.2.2.3 / 42 | MS Malformed Message Response | Native validation exists; source notification/corrected-query lifecycle pending | Claude / SIM-M07 | ACTIVE notification references malformed type/ID/fields; corrected data is correlated; CLOSED follows resolution. Context-invalid but schema-valid input is covered |
| 1.2.2.4 / 43 | Payload Management Failure | Payload failure state exists; full subsystem/fault publication pending | Claude / SIM-M07 | Failed deployment/management changes subsystem status; supported optional fault reporting includes severity, state and description |
| 1.2.2.5 / 44 | Sensor Failure | Payload failure/BIT exists; source service/fault detail mapping pending | Claude / SIM-M07 | Stop the affected sensor outputs, report service state and, if supported, the capability/service/subsystem/component fault attribution |

### 7.3 COP (4) and Data Fusion (2)

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.3.1 / 46 | MS Comms Status Updates to COP | Native link status exists; source periodic subsystem update pending | Claude / SIM-M07 | Comms SubsystemID/state and link changes are correct at the configured publication period |
| 1.2.3.2 / 46 | MS Sensor AMTI Status Updates to COP | Radar availability/activity exists; AMTI MUC mapping pending | Claude / SIM-M01 | Capability/status/activity agree on identity, availability and schedule; no unavailable mode is offered to COP |
| 1.2.3.3 / 49 | MS Sensor PO Status Updates to COP | PO declared; operational configuration/activity pending; PO MUC | Codex / SIM-M06 | Publish accepted PO capability, availability and scheduling state and react to loss/power/BIT changes |
| 1.2.3.4 / 51 | MS Weapon Status Updates to COP | Inventory/store/activity basis exists; Store Management MUC mapping pending | Codex / SIM-M10 | Empty and occupied stations, compatible/installed stores, CapabilityID and weapon availability/state agree without inventing a store in an empty station |
| 1.2.4.1 / 54 | Distribute Single Sensor Track Data to Perform Local Fusion | Native measurements/tracks exist; full OMR/Entity source and association mapping pending | Claude / SIM-M01 | MeasurementID associations and kinematic/identity source SystemID/SubsystemID/CapabilityID remain traceable; only permitted sensor/link data enter fusion |
| 1.2.4.2 / 56 | Receive Processed Track | Native tracking/report basis exists; fuller fusion and Entity/EntityLost publication pending | Claude / SIM-M01 | Merge admitted sources with identity/kinematics/uncertainty/provenance; publish new/changed/lost tracks and preserve unknown/stale states |

### 7.4 Decentralized Messaging Service (10)

| Source / page | Original interaction / requirement | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.5.1 / 59 | Receive Message Subscription Status | Modeled subscriptions are basis; source status and plan/manual distinction pending | Claude / SIM-M07 | Report startup/manual subscriptions periodically/on change; keep plan-driven subscriptions traceable to gateway activity |
| 1.2.5.2 / 60 | Receive UCI Wrapped Message | Modeled DMS delivery exists; external wrapper codec pending | Claude / SIM-M07 | Preserve original sender, message type, priority, destination, receive time and expiry; decode hexBinary UTF-8 XML and honor WrapIngressMessages |
| 1.2.5.3 / 61 | Send UCI Wrapped Message | Queues/latency/costs exist; source routing/status/wrapper mapping pending | Claude / SIM-M07 | Specific-only sends require a destination; test topic/direct routing, permitted routes, priority, expiry, processing progression and failure without duplicate execution |
| 1.2.5.4 / 67 | Subscribe to Message Topic | Native subscribe/delivery basis exists; full source command/filter lifecycle pending | Claude / SIM-M07 | Add/change/delete by SubscriptionID, required topic, optional priority/filter/originator and correlated status; no unintended publisher leakage |
| 1.2.5.5 / 69 | Quality of Service | Modeled reliability/queues do not establish external DDS QoS | Claude / SIM-M08 | Declare every topic's QoS; exercise source reliability defaults and overrides, delivery/expiry and reconnect keep-last behavior without replaying commands as new actions |
| 1.2.5.6 / 69 | Security | Source-named DDS peer credentials and wire protection are not implemented by SDK Agent permissions or local pipe HMAC | Claude / SIM-M08 | Demonstrate the source-required DDS peer admission/credentials and encrypted transport with valid/invalid peers and wire-protection fixtures. Record DDS plugin/profile compatibility explicitly |
| 1.2.5.7 / 70 | Serialization and Data Encoding | Native payload bytes exist; IDL/CDR/EXI protocol acceptance pending | Claude / SIM-M08 | MA_DataPayloadWrapper IDL/CDR with schema-informed strict EXI, source Table 1-31 options and decimal/date/UUID rules; roundtrip and malformed/precision cases, not JSON substitution |
| 1.2.5.8 / 71 | System Discovery | SDK service/link identities exist; RTPS discovery acceptance pending | Claude / SIM-M08 | Unique configured SystemIDs, SPDP/SEDP dynamic discovery and the source-required static/peer-list configurations work in constrained networks |
| 1.2.5.9 / 71 | Topic Publication and Subscription | Native topics are basis; external topic/IDL rules pending | Claude / SIM-M08 | Default topic equals encoded message type, wrapper type agrees, directed SystemID topic naming and any admitted plan override are interoperable |
| 1.2.5.10 / 72 | Transport Protocols | Generic networking exists; external DMS transport acceptance pending | Claude / SIM-M08 | Demonstrate both UDP and TCP through the selected DDS implementation with source-appropriate delivery/QoS; keep physical RF delivery in native DMS |

### 7.5 Mission Plan Behaviors (2) and PNT (1)

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.6.1 / 72 | Receive Plans | Native task/mission basis exists; all five source plan forms are not accepted | Claude / SIM-M07 | Receive ActivityPlan, RouteActivityPlan, RoutePlan, Task and TaskPlan with NEW/UPDATED ObjectState and stable identities; no SDK upper-layer planner is introduced |
| 1.2.6.2 / 74 | Verify Plan Patch Is Possible - MS | Native request validation exists; mission/task/route-activity/activity/action patch validation exchanges pending | Claude / SIM-M07 | Validate each PlanPart through its corresponding command/status; reject invalid references/resources without applying a failed patch |
| 1.2.7.1 / 76 | Request PNT Navigation Data | Ownship/nav data exists; full PNT status/settings and detailed publication pending; PNT MUC | Claude / SIM-M07 | Query subsystem status, change publication frequency and publish all required orientation/heading/wander/rate/acceleration/air-data fields with admitted quality/time semantics |

### 7.6 Sensor/Vehicle Tasking (11)

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.8.1 / 78 | Advertise Sensor AMTI Capability | Operational radar/discovery exists; full source advertisement pending; AMTI MUC | Claude / SIM-M01 | Unique CapabilityIDs, CAPABILITY_COMMAND, true mode/subcapability, frequency/options and component FOR match supported execution |
| 1.2.8.2 / 79 | Advertise Sensor ESM Capability | Native receiver exists; full ESM advertisement pending; ESM MUC | Claude / SIM-M05 | Unique IDs, command interface and source-required FOR/options agree with accepted receiver functions; resolve source/schema mode mismatch before wire freeze |
| 1.2.8.3 / 80 | Advertise Sensor PO Capability | Discovery only; PO MUC operational advertisement pending | Codex / SIM-M06 | Advertise admitted SEARCH_VOLUME/TRACK, command interface and required FOR; don't claim unimplemented imagery/track modes |
| 1.2.8.4 / 81 | Command Sensor AMTI Capability | Native search/track commands exist; full source scheduler/status exchange pending; AMTI MUC | Claude / SIM-M01 | NEW/UPDATE/CANCEL, temporal windows/ranking, VOLUME AirVolume or TRACK EntityID, RECEIVED then accept/reject/cancel and ordered estimated activities |
| 1.2.8.5 / 84 | Command Sensor ESM Capability | Passive warning receiver exists; full scheduled ESM capability command pending; ESM MUC | Claude / SIM-M05 | Acquisition volume or geolocation entity input, valid windows, cancellation, command progression and ordered activities; use schema-valid fields after resolving section 9 |
| 1.2.8.6 / 87 | Command Sensor PO Capability | Full PO execution/scheduler pending; PO MUC | Codex / SIM-M06 | One Pointing.Volume, SEARCH_VOLUME AirVolume or TRACK EntityID, temporal windows, NEW/UPDATE/CANCEL and correlated activity/state/timing |
| 1.2.8.7 / 90 | Publish External Weather Reports | Weather environment exists; offboard weather service request/update behavior pending | Claude / SIM-M07 | DataUpdateRequest query WEATHER, received/processing/COMPLETED plus WeatherMT on success; FAILED with no fabricated data on unavailable service |
| 1.2.8.8 / 92 | Report Sensor AMTI Capability State | Native radar state/activity exists; AMTI MUC publication pending | Claude / SIM-M01 | Periodic/change-driven availability and ordered activities share advertised IDs; unusable sensors cannot accept commands |
| 1.2.8.9 / 93 | Report Sensor ESM Capability State | Native receiver status exists; full ESM activity schedule/publication pending; ESM MUC | Claude / SIM-M05 | Activity identity, state, schedule, estimates and capability linkage agree with accepted command/cancellation |
| 1.2.8.10 / 94 | Report Sensor PO Capability State | Declared PO status; operational PO state/activity pending; PO MUC | Codex / SIM-M06 | Periodic/change-driven availability/activity and estimates remain consistent through power/BIT/failure/cancel |
| 1.2.8.11 / 96 | Report Weather Radar Capability Observations | Weather radar declared; observation model/publication pending; Weather Radar MUC | Claude / SIM-M07 | Publish measured sector-weather observations from the admitted sensor model, with units/validity and unavailable cases distinct from external weather reports |

### 7.7 Status Service (5)

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.9.1 / 97 | Mission Systems Capability Heartbeat | Native supported-family discovery/status exists; all source forms/cadence pending | Claude / SIM-M07 | Cover every family in Tables 1-46/1-65 and 6.2, honest fit/availability and configurable periodic status; no false Supported claim for an absent model |
| 1.2.9.2 / 101 | MS Responds to Query for Capabilities | Native discovery exists; source query progression/results pending | Claude / SIM-M07 | Correlate request, optional intermediate states, completed result/native-message choice and failure reason |
| 1.2.9.3 / 103 | Publish Periodic Passive Optical Sensor Configuration | PO discovery only; required detailed configuration/publication pending; PO MUC | Codex / SIM-M06 | Publish source-required domain pairing, pointing/package operation, components/lens, MDF, outputs/FOR, track range/control and ATR fields; schema-optional is not use-case-optional |
| 1.2.9.4 / 105 | Receive Periodic Health, Status, & BIT Information | Native payload power/BIT/failure exists; service/subsystem periodic/configuration protocol pending | Claude / SIM-M07 | Configured periodic subsystem/service/BIT, active/completed tests and faults; only advertised service configuration parameters may be changed |
| 1.2.9.5 / 107 | Request Mission System Health, Status & BIT Information | Native status/BIT commands exist; request/command status mapping pending | Claude / SIM-M07 | Address the actual subsystem/service ID; acknowledge and report requested status/BIT with processing state and explicit invalid/unavailable cases |

### 7.8 Weapon Employment (13)

These are thirteen interactions, not thirteen weapon types. TaskID, ActivityID, CapabilityID, request IDs, native store indices and released weapon entity IDs are distinct; the adapter preserves their actual mapping. “Fire Command Exchange” is the general Strike task sequence, not a specification of gun physics. “Confirm Threat ID” consumes identification evidence, not an IFF interrogation sequence.

| Source / page | Original interaction | Current basis and remaining semantics | Semantic lead | Specific acceptance |
| --- | --- | --- | --- | --- |
| 1.2.10.1 / 109 | Abort Release Sequence | Native cancel/abort exists; TaskID/cancel/status wire lifecycle pending | Codex / SIM-M10 | Cancel original TaskID, report correlated command/task/store states, reject invalid/late cancellation and preserve an already released weapon |
| 1.2.10.2 / 111 | Confirm Threat ID | Measured tracks/designations exist; StandardIdentity/ThreatType custody/report mapping incomplete | Claude / SIM-M01 | Preserve sourced identity/threat evidence and unknown/conflicting cases across updates; upper-layer MA evaluates engagement criteria without referee truth |
| 1.2.10.3 / 112 | Exchange DLZ | Native scalar advisory zone query exists; calibrated DLZ and asynchronous exchange pending | Codex / SIM-M04 | ResultsInNativeMessage TRUE, request/status/original DLZ_RequestID correlation, accepted flyout-derived inputs/output and unavailable model cases |
| 1.2.10.4 / 113 | Exchange WEZ | Scalar advisory zones do not implement the WEZ exchange | Codex / SIM-M04 | Distinguish StaticWEZ, PlannedStaticWEZ, PredictedWEZ, PlannedPredictedWEZ; store/FOV/azimuth inputs and per-azimuth minimum, turn-and-run, intercept-probability, guidance, optimal and aero range outputs |
| 1.2.10.5 / 115 | Fire Command Exchange | Native strike/activity/receipts and release IDs exist; full Strike task-command/status trace pending | Codex / SIM-M10 | Define/update Strike task, select CapabilityID, command TaskID, accept/reject, publish progress and confirm release. Consume configured legal consent; no repeated release on retry |
| 1.2.10.6 / 118 | Jettison Release Sequence | Native selected/all Standard/Emergency jettison and power-down phase exist | Codex / SIM-M10 | Select capable stores, preserve Standard versus Emergency sequencing, correlate assignment/ack/progress/store state. Do not claim actual MDF zeroization without a delivered MDF model |
| 1.2.10.7 / 120 | Mission Store Release Initialization Sequence | Native target/store type/quantity/index selection and preparation exist | Codex / SIM-M10 | RequiredWeapon and quantity select valid installed capabilities, Task traceability resolves each CapabilityID and store status reflects preparation |
| 1.2.10.8 / 121 | Mission Store Status Exchange | Native store/strike/activity reports exist; full publication lifecycle pending | Codex / SIM-M10 | Include empty capabilities and availability reasons, initial activity in every state, repeat until DISABLED/FAILED/DELETED as specified, and retain release-versus-impact meaning |
| 1.2.10.9 / 124 | Receive Changes to Stores Primary Mission Plan Data | Weapon MDF/service configuration feature not delivered; conditional source functionality | Codex / SIM-M10 | Advertise configurable parameters/MDF, query/change only those parameters or ActivateMDF, and publish correlated result plus updated capability/status; otherwise explicitly report unsupported fit/profile |
| 1.2.10.10 / 126 | Receive Weapon Flyout State | World/referee telemetry exists; actor feedback explicitly unavailable without modeled feedback; Store Management MUC | Codex / SIM-M03 | Assessment accept/reject and permitted periodic MA_Entity feedback with engagement/time-to-active/intercept; stop at LOST/SOFT_LOST/END_OF_FLIGHT. Never forward referee truth as actor feedback |
| 1.2.10.11 / 128 | Release Consent Sequence | Native role-based consent/request/activity correlation exists; complete authorization profiles remain HC-06 | Codex / HC-06 native input feature | Correlate task/activity/final authorization and independently configured C2 designation, approved-response or arming criteria; Claude consumes accepted profiles for adapter/gateway integration |
| 1.2.10.12 / 130 | Send Weapon Entity Data | Measured-track/designation/DMS/guidance paths exist; full fused-entity import/update/loss mapping pending | Claude / SIM-M01 fused report feature; Codex / SIM-M03 weapon consumption | Accepted fused Entity ID/source/status/identity/kinematics updates reach the weapon through modeled supported paths; lost/stale/unknown input is explicit. Two distinct complete features meet at a frozen report contract |
| 1.2.10.13 / 131 | Store Inventory and Descriptions Exchange | Native inventory/station/loadout compatibility exists; full per-store/rack identity and verification mapping pending | Codex / SIM-M10 | Installed-store CapabilityID ties to carriage/rack and type/variant/mnemonic/power/loadout; verify/update inventory. Loading acceptance alone is not safe-separation evidence |

### 7.9 Interface compliance requirements outside individual payloads

| Source requirement, Table 1-64 p.133 | Owner / current disposition | Closure condition |
| --- | --- | --- |
| MA-L1-006 / MA-L1-007 | Claude / SIM-M08; full message/sequence compliance pending | Select explicit platform MUC profile and prove Core plus all applicable entries in 7-8; no silent omission of unsupported forms |
| MA-L1-008 | Claude / SIM-M08; platform ASB exchange pending | The external MA-MS integration uses the platform-hosted ASB; native SDK calls remain available independently |
| MA-L1-009 / MA-L1-010 / MA-L1-011 / MA-L1-012 | Claude / SIM-M08; DDS/RTPS/IDL/topic acceptance pending | Interoperable DMS peers use DDS, required discovery, MA_DataPayloadWrapper IDL and matching message/topic naming as required by 7.4 |
| MA-L1-019 | Claude / SIM-M08 consuming SIM-M07 validation | Exercise context-malformed messages, notification, corrected query and closure in 1.2.2.3 |
| MA-L1-025 | Claude / SIM-M08; EXI acceptance pending | UTF-8 XML/hexBinary on the MA-MS boundary and specified EXI in the OTA wrapper; Table 1-31 settings/precision fixtures pass |

## 8. Complete original MS minimum message set

The following is the exact Table 1-65 inventory, including wrapped PDF identifiers joined without added spaces. `in`/`out` are relative to MS. MUC codes: **C** Core; **AC** Additional Comms Capabilities; **AMTI**, **ESM**, **PO**, **PNT** retain their source names; **SM** Store Management; **CA** Carrier; **WR** Weather Radar. Codes on one row are overlapping applicability, not additional message types. `MA_` names are A-GRA Custom Extensions; all other rows are UCI 2.3. Every listed type was found in the pinned XSD.

For **each row**, the external message feature owner is **Claude / SIM-M08**, current complete wire acceptance is **pending**, and closure requires 6.1 condition 4 plus the referenced semantic family/service and all applicable rows in 7. Native completion remains with the family/feature owner in 6-7. This explicitly assigns Core message forms for absent physical families without claiming those models exist. A profile must record applicability; it cannot remove a Core form just because the payload is not fitted.

Inventory checks: 139 unique names; 101 UCI + 38 A-GRA; directions 39 in / 96 out / 4 inout. The six source pages contain 22 / 25 / 18 / 20 / 28 / 26 rows. MUC memberships overlap: Core 103, AC 19, SM 7, AMTI 5, PO 5, ESM 4, PNT 4, CA 2, WR 1.

<!-- BEGIN PINNED MS MMS INVENTORY -->
| No. | Exact message type | Direction | MUC | PDF page | Native semantic feature / exchange |
| --- | --- | --- | --- | --- | --- |
| 1 | `ActionPlanValidationCommandMT` | in | C | 134 | Plan patch validation (7.5) |
| 2 | `ActionPlanValidationCommandStatusMT` | out | C | 134 | Plan patch validation (7.5) |
| 3 | `ActivityPlanValidationCommandMT` | in | C | 134 | Plan patch validation (7.5) |
| 4 | `ActivityPlanValidationCommandStatusMT` | out | C | 134 | Plan patch validation (7.5) |
| 5 | `AirSampleCapabilityMT` | out | C | 134 | AirSample (6.2) |
| 6 | `AirSampleCapabilityStatusMT` | out | C | 134 | AirSample (6.2) |
| 7 | `AMTI_ActivityMT` | out | AMTI | 134 | AMTI (6.2/7.6) |
| 8 | `AMTI_CapabilityStatusMT` | out | C, AMTI | 134 | AMTI (6.2/7.6) |
| 9 | `AMTI_CommandMT` | in | AMTI | 134 | AMTI (6.2/7.6) |
| 10 | `AMTI_CommandStatusMT` | out | AMTI | 134 | AMTI (6.2/7.6) |
| 11 | `AO_CapabilityMT` | out | C | 134 | AO (6.2) |
| 12 | `AO_CapabilityStatusMT` | out | C | 134 | AO (6.2) |
| 13 | `AssessmentRequestMT` | in | SM | 134 | Permitted weapon flyout feedback (7.8) |
| 14 | `AssessmentRequestStatusMT` | out | SM | 134 | Permitted weapon flyout feedback (7.8) |
| 15 | `CargoDeliveryCapabilityMT` | out | C | 134 | CargoDelivery (6.2) |
| 16 | `CargoDeliveryCapabilityStatusMT` | out | C | 134 | CargoDelivery (6.2) |
| 17 | `COMINT_CapabilityMT` | out | C | 134 | COMINT (6.2) |
| 18 | `COMINT_CapabilityStatusMT` | out | C | 134 | COMINT (6.2) |
| 19 | `CommAssetEstimationRequestMT` | in | AC | 134 | Terminal/assets/status (7.1) |
| 20 | `CommAssetEstimationRequestStatusMT` | out | AC | 134 | Terminal/assets/status (7.1) |
| 21 | `CommAssetMT` | out | C | 134 | Terminal/assets/status (7.1) |
| 22 | `CommRelayCapabilityMT` | out | C | 134 | CommRelay (6.2) |
| 23 | `CommRelayCapabilityStatusMT` | out | C | 135 | CommRelay (6.2) |
| 24 | `CommScheduleAllocationMT` | out | C | 135 | Terminal/assets/status (7.1) |
| 25 | `CommSystemMT` | out | C | 135 | Terminal/assets/status (7.1) |
| 26 | `CommTerminalActivityMT` | out | AC, C | 135 | Terminal/assets/status (7.1) |
| 27 | `CommTerminalCapabilityMT` | out | C | 135 | Terminal/assets/status (7.1) |
| 28 | `CommTerminalCapabilityStatusMT` | out | C | 135 | Terminal/assets/status (7.1) |
| 29 | `CommTerminalCommandMT` | in | AC | 135 | Terminal/assets/status (7.1) |
| 30 | `CommTerminalCommandStatusMT` | out | AC | 135 | Terminal/assets/status (7.1) |
| 31 | `CommTerminalPlanActivationCommandMT` | in | AC | 135 | Terminal/assets/status (7.1) |
| 32 | `CommTerminalPlanActivationCommandStatusMT` | out | AC | 135 | Terminal/assets/status (7.1) |
| 33 | `CommTerminalPlanMT` | in | AC | 135 | Terminal/assets/status (7.1) |
| 34 | `CounterSpaceCapabilityMT` | out | C | 135 | CounterSpace (6.2) |
| 35 | `CounterSpaceCapabilityStatusMT` | out | C | 135 | CounterSpace (6.2) |
| 36 | `DataUpdateRequestMT` | in | C | 135 | External weather request/result (7.6) |
| 37 | `DataUpdateRequestStatusMT` | out | C | 135 | External weather request/result (7.6) |
| 38 | `DLZ_MT` | out | C | 135 | WEZ / DLZ request and result (7.8) |
| 39 | `DLZ_RequestMT` | in | C | 135 | WEZ / DLZ request and result (7.8) |
| 40 | `DLZ_RequestStatusMT` | out | C | 135 | WEZ / DLZ request and result (7.8) |
| 41 | `EA_CapabilityMT` | out | C | 135 | EA (6.2) |
| 42 | `EA_CapabilityStatusMT` | out | C | 135 | EA (6.2) |
| 43 | `EntityLostMT` | inout | C | 135 | Sensor/fused entities (7.3); weapon input (7.8) |
| 44 | `EntityMT` | inout | C | 135 | Sensor/fused entities (7.3); weapon input (7.8) |
| 45 | `ESM_ActivityMT` | out | ESM | 135 | ESM (6.2/7.6) |
| 46 | `ESM_CapabilityMT` | out | C, ESM | 135 | ESM (6.2/7.6) |
| 47 | `ESM_CapabilityStatusMT` | out | C | 135 | ESM (6.2/7.6) |
| 48 | `ESM_CommandMT` | in | ESM | 136 | ESM (6.2/7.6) |
| 49 | `ESM_CommandStatusMT` | out | ESM | 136 | ESM (6.2/7.6) |
| 50 | `GatewayCapabilityMT` | out | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 51 | `GatewayCapabilityStatusMT` | out | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 52 | `MA_ActivityPlanMT` | in | C | 136 | Plan reception (7.5) |
| 53 | `MA_AMTI_CapabilityMT` | out | C, AMTI | 136 | AMTI (6.2/7.6) |
| 54 | `MA_CommAvailableEndpointsMT` | out | C | 136 | Comms endpoints/team/errors (7.1) |
| 55 | `MA_CommErrorReportMT` | out | C | 136 | Comms endpoints/team/errors (7.1) |
| 56 | `MA_CommTeamReportMT` | out | C | 136 | Comms endpoints/team/errors (7.1) |
| 57 | `MA_DataDisseminationPlanActivationCommandMT` | in | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 58 | `MA_DataDisseminationPlanActivationCommandStatusMT` | out | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 59 | `MA_DataDisseminationPlanActivationStatusMT` | out | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 60 | `MA_DataDisseminationPlanMT` | in | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 61 | `MA_DataDisseminationPlanOverrideRequestMT` | in | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 62 | `MA_DataDisseminationPlanOverrideRequestStatusMT` | out | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 63 | `MA_EntityMT` | out | SM | 136 | Permitted weapon flyout feedback (7.8) |
| 64 | `MA_FaultMT` | out | C | 136 | Fault/notification/correction (7.2) |
| 65 | `MA_GatewayActivityMT` | out | AC | 136 | Gateway/plans/effectivities (7.1/7.4) |
| 66 | `MA_GatewayCommandMT` | in | AC | 137 | Gateway/plans/effectivities (7.1/7.4) |
| 67 | `MA_GatewayCommandStatusMT` | out | AC | 137 | Gateway/plans/effectivities (7.1/7.4) |
| 68 | `MA_MessagingSubscriptionSettingCommandMT` | in | C | 137 | DMS subscription/routing/wrapping (7.4) |
| 69 | `MA_MessagingSubscriptionSettingCommandStatusMT` | out | C | 137 | DMS subscription/routing/wrapping (7.4) |
| 70 | `MA_MessagingSubscriptionStatusMT` | out | C | 137 | DMS subscription/routing/wrapping (7.4) |
| 71 | `MA_MissionPlanValidationCommandMT` | in | C | 137 | Plan patch validation (7.5) |
| 72 | `MA_MissionPlanValidationCommandStatusMT` | out | C | 137 | Plan patch validation (7.5) |
| 73 | `MA_PositionReportDetailedMT` | out | PNT | 137 | PNT status/settings/data (7.5) |
| 74 | `MA_RouteActivityPlanMT` | in | C | 137 | Plan reception (7.5) |
| 75 | `MA_RoutePlanMT` | in | C | 137 | Plan reception (7.5) |
| 76 | `MA_RxDataPayloadMT` | out | C | 137 | DMS subscription/routing/wrapping (7.4) |
| 77 | `MA_StrikeActivityMT` | out | C, SM | 137 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 78 | `MA_StrikeCapabilityMT` | out | C, SM | 137 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 79 | `MA_SystemNotificationMT` | in | C | 137 | Fault/notification/correction (7.2) |
| 80 | `MA_TaskCommandMT` | in | C | 137 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 81 | `MA_TaskCommandStatusMT` | out | C | 137 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 82 | `MA_TaskMT` | in | C | 137 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 83 | `MA_TaskPlanMT` | in | C | 137 | Plan reception (7.5) |
| 84 | `MA_TaskStatusMT` | inout | C | 137 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 85 | `MA_TxDataPayloadCommandMT` | in | C | 137 | DMS subscription/routing/wrapping (7.4) |
| 86 | `MA_TxDataPayloadCommandStatusMT` | out | C | 138 | DMS subscription/routing/wrapping (7.4) |
| 87 | `MA_WEZ_MT` | out | C | 138 | WEZ / DLZ request and result (7.8) |
| 88 | `MA_WEZ_RequestMT` | in | C | 138 | WEZ / DLZ request and result (7.8) |
| 89 | `MA_WEZ_RequestStatusMT` | out | C | 138 | WEZ / DLZ request and result (7.8) |
| 90 | `ObservationMeasurementReportMT` | out | C | 138 | Sensor/fused entities (7.3); weapon input (7.8) |
| 91 | `PO_ActivityMT` | out | PO | 138 | PO (6.2/7.6-7.7) |
| 92 | `PO_CapabilityMT` | out | PO | 138 | PO (6.2/7.6-7.7) |
| 93 | `PO_CapabilityStatusMT` | out | PO | 138 | PO (6.2/7.6-7.7) |
| 94 | `PO_CommandMT` | in | PO | 138 | PO (6.2/7.6-7.7) |
| 95 | `PO_CommandStatusMT` | out | PO | 138 | PO (6.2/7.6-7.7) |
| 96 | `QueryDataRequestMT` | in | C | 138 | Capability query (7.7); malformed correction (7.2) |
| 97 | `QueryDataRequestStatusMT` | out | C | 138 | Capability query (7.7); malformed correction (7.2) |
| 98 | `RefuelCapabilityMT` | out | C | 138 | Refuel (6.2) |
| 99 | `RefuelCapabilityStatusMT` | out | C | 138 | Refuel (6.2) |
| 100 | `RouteActivityPlanValidationCommandMT` | in | C | 138 | Plan patch validation (7.5) |
| 101 | `RouteActivityPlanValidationCommandStatusMT` | out | C | 138 | Plan patch validation (7.5) |
| 102 | `SAR_CapabilityMT` | out | C | 138 | SAR (6.2) |
| 103 | `SAR_CapabilityStatusMT` | out | C | 138 | SAR (6.2) |
| 104 | `ServiceConfigurationChangeRequestMT` | in | C | 138 | Service configuration (7.7); weapon MDF (7.8) |
| 105 | `ServiceConfigurationChangeRequestStatusMT` | out | C | 138 | Service configuration (7.7); weapon MDF (7.8) |
| 106 | `ServiceConfigurationParamsDataRequestMT` | in | C | 138 | Service configuration (7.7); weapon MDF (7.8) |
| 107 | `ServiceConfigurationParamsDataRequestStatusMT` | out | C | 138 | Service configuration (7.7); weapon MDF (7.8) |
| 108 | `ServiceConfigurationParamsMT` | out | C | 138 | Service configuration (7.7); weapon MDF (7.8) |
| 109 | `ServiceStatusDataRequestMT` | in | C | 138 | Service/subsystem/BIT (7.2/7.7) |
| 110 | `ServiceStatusDataRequestStatusMT` | out | C | 138 | Service/subsystem/BIT (7.2/7.7) |
| 111 | `ServiceStatusMT` | out | C | 138 | Service/subsystem/BIT (7.2/7.7) |
| 112 | `SMTI_CapabilityMT` | out | C | 138 | SMTI (6.2) |
| 113 | `SMTI_CapabilityStatusMT` | out | C | 138 | SMTI (6.2) |
| 114 | `StoreManagementCommandMT` | in | C | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 115 | `StoreManagementCommandStatusMT` | out | C | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 116 | `StoreManagementStatusMT` | out | SM | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 117 | `StrikeCapabilityStatusMT` | out | C, SM | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 118 | `StrikeConsentRequestMT` | out | C | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 119 | `StrikeConsentRequestStatusMT` | in | C | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 120 | `StrikeSettingsCommandMT` | in | C | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 121 | `StrikeSettingsCommandStatusMT` | out | C | 139 | Strike/stores/consent (7.8); Task forms also 7.5 |
| 122 | `SubsystemBIT_CommandMT` | in | C | 139 | Service/subsystem/BIT (7.2/7.7) |
| 123 | `SubsystemBIT_CommandStatusMT` | out | C | 139 | Service/subsystem/BIT (7.2/7.7) |
| 124 | `SubsystemBIT_StatusMT` | out | C | 139 | Service/subsystem/BIT (7.2/7.7) |
| 125 | `SubsystemSettingsCommandMT` | in | PNT | 139 | PNT status/settings/data (7.5) |
| 126 | `SubsystemStatusDataRequestMT` | in | PNT, C | 139 | Subsystem/COP/PNT/Carrier status (6.3/7.3/7.5/7.7) |
| 127 | `SubsystemStatusDataRequestStatusMT` | out | PNT, C | 139 | Subsystem/COP/PNT/Carrier status (6.3/7.3/7.5/7.7) |
| 128 | `SubsystemStatusMT` | out | CA, C | 139 | Subsystem/COP/PNT/Carrier status (6.3/7.3/7.5/7.7) |
| 129 | `SystemDeploymentCapabilityMT` | out | C | 139 | SystemDeployment (6.2) |
| 130 | `SystemDeploymentCapabilityStatusMT` | out | C | 139 | SystemDeployment (6.2) |
| 131 | `SystemStatusMT` | inout | C, CA | 139 | Link-loss/system/Carrier status (6.3/7.1) |
| 132 | `TacticalOrderCapabilityMT` | out | C | 139 | TacticalOrder (6.2) |
| 133 | `TacticalOrderCapabilityStatusMT` | out | C | 139 | TacticalOrder (6.2) |
| 134 | `TaskPlanValidationCommandMT` | in | C | 139 | Plan patch validation (7.5) |
| 135 | `TaskPlanValidationCommandStatusMT` | out | C | 139 | Plan patch validation (7.5) |
| 136 | `WeatherMT` | out | C | 139 | External weather request/result (7.6) |
| 137 | `WeatherRadarCapabilityMT` | out | C | 139 | Weather radar (6.2/7.6) |
| 138 | `WeatherRadarCapabilityStatusMT` | out | C | 139 | Weather radar (6.2/7.6) |
| 139 | `WeatherRadarObservationMT` | out | WR | 139 | Weather radar (6.2/7.6) |
<!-- END PINNED MS MMS INVENTORY -->

## 9. Open source alignment and completion decisions

These items stay in this allocation so that the original source, capability plan and future feature acceptance cannot disagree silently. They are planning obligations, not newly completed code or an active implementation window.

| Item | Owner / package | Resolution / acceptance condition |
| --- | --- | --- |
| Dated ADR-30 MMS count differs from the pinned original | Claude / SIM-M08 consumes this corrected inventory | Use 139 / 101 + 38 and all exact rows in 8 for this source version. Preserve the old count only as dated history until its separate documentation reconciliation; do not use 128 as the current acceptance denominator |
| ESM advertisement/command prose and XSD use different capability taxonomies | Claude / SIM-M05 semantic feature and SIM-M08 adapter | MS 1.2.8.2 says SEARCH_VOLUME/TRACK, but ESM_CapabilityType uses CapabilityInitiationEnum and a separate ESM_SubCapabilityEnum. The command prose also mixes CapabilityType/SubCapabilityType, and Table 1-39 names PO_CapabilityCommandType. Freeze a schema-valid mapping with explicit use-case intent and a recorded source clarification; never invent an enum or silently modify the frozen schema |
| Abort spelling and abbreviated AMTI type references in prose | Claude / SIM-M08, consuming Codex native abort semantics | Resolve CANCELLED/CANCELED against the actual wire type; advertisement uses MA_AMTI_CapabilityMT although some prose says AMTI_CapabilityMT. Fixtures use the exact XSD types/values and retain source aliases as documentary discrepancies |
| Core declaration support versus physical fit | Claude / SIM-M08 message support; model owner in 6.2 | Every Core form has codec/status evidence; a platform fit/profile records absent/unsupported hardware honestly. Any physical not-fitted/deferred disposition is explicit, with prerequisites and closure conditions, rather than an implicit “other payloads” omission |
| Unadmitted environments and families | Claude / SIM-M07 profile decisions; Codex for its named optical/weapon model inputs | Air sampling, cargo/refuel/deployment, counter-space, SMTI target sources and Carrier prerequisites remain pending until admitted. Retain the current no-ground-entities boundary and do not infer operational capability from a schema category |
| IFF and other SDK extensions versus original MS requirements | Claude / SIM-M07, with SIM-M09 consuming accepted IFF | Track IFF and ground-map features separately from the 67/139 source denominators. Preserve the project's IFF-before-multiaircraft gate and original 1v1 scope |
| Completion evidence by family, mode, message and platform profile | Native feature owner; Claude / SIM-M08 protocol; Claude / SIM-M09 encounter integration | Close only named supported modes with model/public/protocol evidence and explicit remaining modes. Update status/carryover records when an actual feature is handed off; this planning expansion closes no HC item or physical-model milestone |
| Fleet-wide real-equipment coverage and public-data fidelity | Every native model owner in 2/6.2; Claude / SIM-M09 consolidates accepted integration evidence | Apply 1.1 to all MS payloads. Reconcile the independent current vehicle/variant roster with actual installations, missing model identities, parameters, modes and functional evidence. Keep missing/unknown/unfinished equipment open until accepted or an explicit user scope decision; do not infer complete coverage from fit-file presence |
| Mature open-source evaluation before custom modeling | Each complete feature owner | Record candidate project/version, validation and integration/license suitability, selected reuse or justified custom work before implementing the model. The current document is the record; no separate planning document is required |
| Existing radar fidelity/coverage gaps at 6170ec6 | Claude / SIM-M01 air radars and SIM-M07 remaining radar families | The fit audit found 24 radar-family cards: 17 operational air-radar models and 7 weather/SAR/ground-map identity/status cards without numerical models. The air-radar cards contain 253 assumed, 45 published and 27 derived parameter entries; these labels are not a fidelity score. APG-79/APG-81 currently share identical numerical parameters; APG-73, Captor-M, N036, Type 1473 and Type 1475 have no per-card validation entries. Source/variant-specific justification and independent validation remain required; placeholder cards, class calibration or names alone cannot close these gaps |

## 10. First SIM-M02 input and reproducibility slice

Baseline: main `14a90f4`, 2026-10-17. Owner: Codex. The first candidate is the existing `aim120c` design and `aim-120c` card, labelled AIM-120C-7. This slice supplies generic evidence tooling and truthful batch results before any physical-model calibration. It changes no aerodynamic, motor, guidance, fuze, damage or generated-model values and does not close SIM-M02.

| Item | Frozen scope / acceptance |
| --- | --- |
| Source/symbol write allowlist | New `tools/models/hangar/hangar/provenance.py` capture/verify/CLI; `tools/models/hangar/hangar/storestages.py` batch child-result aggregation/reporting; existing `tools/models/hangar/tests/test_stores.py` metadata/child-failure regressions; this document's package evidence |
| Input/artifact record | Deterministic SHA-256 records over explicitly listed repository-relative input and artifact files; both sets must be nonempty. Verification requires the independently supplied expected path sets, rejects missing/changed/dropped records and unresolved card citations, and is read-only. It does not infer complete transitive dependencies or fetch/archive remote citations; those remain explicit input-freeze work |
| Admission limit | Existing card source labels do not establish variant-specific calibration. Separate official identity/installation references, secondary descriptions, assumptions and explicitly identified game references. Metadata integrity never promotes game values or a general publication to a real-equipment measurement |
| Existing report defect | A child exit of 1 plus old passing stage JSON can currently yield batch exit 0. Every nonzero or missing child result must fail the batch and be explicit in any new report, even when cached stage reports pass |
| Shared paths | Radar paths, World/Agent/adapters/public ABI, CMake, other shared documents and generated TOML/XML/GLB are outside this write window. Claude's SIM-M01 slice has a separate radar-only allowlist; native configure/build/test slots remain serial |
| Fast checks | Synthetic provenance/CLI and failed-child regressions, selected existing generator consistency checks, and document checks only. No full Hangar label, native simulation, solver campaign, full suite or GUI run |
| Stop point | Stop for independent acceptance of the input/evidence tooling. Keep absent public motor/geometry details, independent validation cases, actual solver versions/hashes and the historical missing mesher explicitly pending; no calibrated-flight or complete-fleet claim |

### 10.1 Evidence and reuse disposition

The generic evidence utility uses Python's standard-library SHA-256, JSON and path handling; it introduces no physics implementation or runtime dependency. The accepted offline-tool direction is retained rather than replacing it with a new solver:

| Existing implementation / candidate | This slice's decision | Remaining readiness evidence |
| --- | --- | --- |
| Existing JSBSim model/runtime | Preserve the current implementation and all generated bytes | This slice executes no native flight and supplies no new flight-calibration evidence |
| [SU2](https://github.com/su2code/SU2) and [OpenVSP/VSPAERO](https://github.com/OpenVSP/OpenVSP) | Retain as separate offline comparison candidates; no installation, rebuild or model run | Historical preparation does not pin a current executable. Record actual paths, source/build versions, hashes, compatible mesh/input workflow and upstream reference-case evidence before use. Neither executable was discovered on this session's PATH; that is not proof it is absent elsewhere |
| [RocketPy](https://github.com/RocketPy-Team/RocketPy) | Retain as an offline reference candidate; do not implement another trajectory solver | Package metadata for D:/ENV/python/3.13 reports 1.13.0. Import/runtime/reference-case compatibility and model-input admission remain unverified; the gmsh Python module was not found in that interpreter. No toolchain was changed |

The first real-file probe freezes **13 explicitly selected inputs and 3 existing artifacts**. Input paths are the card `assets/stores/aim-120c.json`; `aircraft/aim120c/aim120c.toml` and `paint.toml`; `tools/models/hangar/hangar/{missiles,storeaero,storebodies,storedrag,storemodels,storestages,pipeline,files,provenance}.py`; and `tools/models/hangar/hangar/geometry/aircraft.py`. Artifacts are `aircraft/stores/aim120c/aim120c.xml`, `Engines/aim120c_motor.xml` and `Engines/aim120c_nozzle.xml` under that same model directory. This is a declared file-integrity scope, not a claim to capture every transitive dependency, external package, solver binary or validation dataset.

The independently supplied path lists are retained at `build/architecture-reviews/ms-first-slice/expected-files.json`; the API and CLI snapshots are in the same temporary evidence directory. Original pre-category snapshot digest, superseded by 10.2: `069236839c8f59cf3e470144604aa905657eef06252a20f9d7b3a551dcd8140b`. Verification must receive the frozen expected lists, not derive them from an untrusted snapshot. The card has 8 entries labelled published and 7 assumed; the snapshot deliberately reports `source_assessment=not_assessed`. Its labels, especially explicitly identified game references, have not been promoted to admitted real-world measurements.

Reproduction from the repository root, using the independently frozen list file (captures current bytes; later input edits change the digest):

```powershell
$env:PYTHONPATH = "$PWD/tools/models/hangar"
$files = Get-Content build/architecture-reviews/ms-first-slice/expected-files.json -Raw | ConvertFrom-Json
$options = @('--root', '.', '--manifest', 'build/architecture-reviews/ms-first-slice/aim120c-cli-provenance.json',
             '--card', 'assets/stores/aim-120c.json')
foreach ($path in $files.inputs) { $options += @('--input', $path) }
foreach ($path in $files.artifacts) { $options += @('--artifact', $path) }
& 'D:/ENV/python/3.13/python.exe' -B -m hangar.provenance capture @options
& 'D:/ENV/python/3.13/python.exe' -B -m hangar.provenance verify @options
```

The batch regression reproduced the old false success for child exit 1 with a stale passing report. The corrected batch fails for nonzero/missing child results, exposes child exit codes and `batch_passed` in JSON, and labels cached stage values as historical in Markdown. New metadata regressions cover tampered/missing files, independently expected sets, empty groups, card identity/citation errors, ambiguous/excluded paths and CLI outcomes. Exclusion probes are mocked and access no excluded directory. Tests remain in the existing registered StoresTest class; no shared CMake edit is needed.

Focused verification on the final code passed 13 tests in 0.728 s: the 10 new metadata/batch tests and three existing card/design/generator consistency tests. The real-file API and CLI capture/verify results match the digest above. The repository architecture checker passed with no Python cycles or layout exceptions. No native build, physical solver, GUI or full suite was run.

Acceptance of this slice is limited to provenance and batch-result correctness. Source admission, solver provenance, independent comparison data, meaningful physical tolerances and actual model calibration remain open for the next input handoff; this record is not a claim that SIM-M02 or its first calibrated model is complete.

### 10.2 Continued catalogue and source-metadata audit

Continued at main `3cfcbe8` on 2026-10-17 under the user's instruction to continue. Codex's additional write scope is the existing source dictionaries of the 25 cards listed below, this section, `hangar/provenance.py` source-category validation/reporting and regressions in the existing `StoresTest` class. Card values, runtime behavior, native source/ABI, generated designs and Claude's radar files remain outside this metadata change. The file list is `assets/stores/{aim-120c,aim-120d,aim-7m,aim-9m,aim-9x,asraam,cj-10a,iris-t,magic-2,meteor,mica-em,mica-ir,pl-10,pl-12,pl-15,pl-8,r-27er,r-27et,r-27r,r-27t,r-37m,r-60m,r-73,r-77m,super-530d}.json`.

Source metadata now has an explicit `evidence_kind`: `government_publication`, `manufacturer_publication`, `operator_publication`, `research_publication`, `secondary_reference`, `game_reference` or `unclassified`. This describes a publication category, not correctness or admission of a particular claim. Legacy unclassified sources remain unclassified; unknown category spellings fail metadata validation. A parameter's historical `basis=published` is not a real-equipment admission flag: the evidence report must retain its cited source category, including `game_reference`. Native card parsing and all numerical values remain unchanged. Newly added manufacturer references in this slice establish identity only; they do not establish variant-specific measurements or calibration.

#### Independent catalogue denominator

The authored roster is **35 aircraft**, independently derived from tracked `aircraft/<name>/<name>.toml` designs without a store card and checked against [Hangar's maintained roster](../../../hangar.md) and `tools/models/hangar/rebuild_all.py`. The 53 store designs are separate: 88 design TOMLs in total. The aircraft IDs are:

```text
a10c b52h c130j c172 c17a cf2 e3g e7a ea18g ec130h f15c f16c f22a
f35a fa18c gripen h6k iris j10a j20a kc135r kc46a mig29a mirage2000
rafale rc135w rq4b skua su25 su27s su57 typhoon u2s uh1h uh60
```

All 35 have payload-fit files. Their 45 loadouts reference **19 distinct AAM cards across 17 aircraft**, or **30 aircraft/card pairs**. The table below records existing software configuration only, not independent confirmation of an installation or an accepted physical model. The five other AAM cards have no loadout reference: `aim-120d`, `r-27er`, `r-27et`, `r-27t`, `r-37m`. Every referenced store ID resolves, and the 53 store cards have unique design/card/model identities.

| Authored aircraft | Existing AAM card references |
| --- | --- |
| a10c | aim-9m |
| ea18g | aim-120c |
| f15c | aim-120c, aim-9m |
| f16c, f22a | aim-120c, aim-9x |
| f35a | aim-120c |
| fa18c | aim-120c, aim-7m, aim-9m |
| gripen | iris-t, meteor |
| j10a | pl-12, pl-8 |
| j20a | pl-10, pl-15 |
| mig29a, su27s | r-27r, r-73 |
| mirage2000 | magic-2, super-530d |
| rafale | mica-em, mica-ir |
| su25 | r-60m |
| su57 | r-77m |
| typhoon | aim-120c, asraam |
| b52h, c130j, c172, c17a, cf2, e3g, e7a, ec130h, h6k, iris, kc135r, kc46a, rc135w, rq4b, skua, u2s, uh1h, uh60 | No AAM loadout recorded; this is not a sourced not-fitted decision |

`h6k` also references `cj-10a`, outside the AAM column. That card describes a cruise-missile identity but is stored as `class=bomb` and explicitly represents an inert released object; no functioning cruise-missile model is inferred. Keep this identity in the inventory rather than excluding it by runtime class.

The packaging denominator is wider than the authored roster: `cmake/Install.cmake` ships the stock JSBSim aircraft tree. A read-only count found **64 modern `fdm_config` definitions in 59 directories**, plus two legacy/template `FDM_CONFIG` files. Some are test/released objects, so 64 is not a count of accepted operational aircraft. The explicit pending list below is relative to `third_party/jsbsim/aircraft/`; a bare name means `<name>/<name>.xml`:

```text
737 787-8 A320 A4 B17 B747 Boeing314 C130 Camel Concorde DHC6 F450
F4N/{F4N-jsbsim.xml,F4N.xml} F80C J246 J3Cub L17 L410 MD11 OV10
Pterosaur SGS Short_S23 Shuttle Submarine_Scout T37 T38 X15 XB-70 ZLT-NT
ah1s ball ballx c172p c172r c172x c182 c310 dr1 f104 f15 f16
f22/{f22.xml,yf22.xml} fokker100/{fokker100.xml,fokker70.xml,xf100.xml}
fokker50 global5000 minisgs mk82 p51d pa28 paraglider pc7 pogo-jsbsim
sgs126 sgs233 t6texan2 weather-balloon
wrightFlyer1903/{wrightFlyer1903.xml,wrightFlyer1905.xml} x24b
Legacy/template: aircraft_template.xml; blank/blank.xml
```

Every stock entry still needs an explicit SDK applicability/identity disposition; file presence does not prove loadability or payload acceptance. The declared visual aliases are `C130 -> c130j`, `c172x/c172p/c172r -> c172`, `f15 -> f15c`, `f16 -> f16c`, `f22 -> f22a`. [Resource ownership](../../../resources.md) does not grant fit inheritance through a visual alias. `aircraft/models.txt` contains the six aliases other than `C130`; the declaration/manifest discrepancy remains recorded below. No stock fit or accepted equipment installation is invented here.

#### Existing source-quality inventory

Before the metadata additions, the 24 AAM cards contained 356 parameter entries: 165 labelled published, 184 assumed and 7 derived. These are declaration counts, not accuracy scores. Eighteen published-labelled entries cite an explicitly described War Thunder reference; those source records now carry `game_reference`. Only five AAM cards cite government records in their parameters. All local source keys resolve, but resolution alone establishes neither variant applicability nor measurement accuracy.

| Card identity | Existing source-quality/coverage issue retained for review |
| --- | --- |
| aim-120c (AIM-120C-7) | Family references do not establish every C-7-specific claim; game reference remains unpinned |
| aim-120d | Family/D subvariant applicability unresolved; no current loadout reference |
| aim-7m | Preserve M identity separately from family-level descriptions and other Sparrow variants |
| aim-9m | One existing M-card parameter cites an X fact-file; applicability remains unverified |
| aim-9x | Block unspecified; two listed historical references are unused by parameters |
| asraam | Previously encyclopedia-only; identity reference does not admit numerical inputs |
| iris-t | Previously encyclopedia-only; do not conflate air-launched and SL variants |
| magic-2 | Family/variant and source applicability require explicit evidence |
| meteor | Two encyclopedia language editions were not independent primary measurement evidence |
| mica-em, mica-ir | Keep RF/IR identities distinct; a shared family page is not independent model validation |
| pl-10, pl-12, pl-15, pl-8 | Existing records are encyclopedia/game references; export identities are not automatic aliases |
| r-27er, r-27et, r-27r, r-27t | Family references do not make all four variants interchangeable; only r-27r has a current loadout reference |
| r-37m | Family/variant applicability unresolved; no current loadout reference |
| r-60m, r-73 | Preserve exact identities; existing game data is not real-equipment evidence |
| r-77m | Family article does not establish this specific model or equate it to export names |
| super-530d | Keep D separate from Super 530 F and the older R530 family |
| cj-10a | Secondary family-derived identity; inert runtime representation, not a working missile |

The 18 game citations point to a mutable directory on `master`, without a pinned commit/file. Wikipedia entries use revision locators, but language editions do not create independent engineering evidence. Existing government links were blocked with HTTP 403 in the browsing check; their present contents and individual claims were not verified. Do not interpret adding an unused primary source as increasing the number of admitted parameter inputs.

#### Identity references added without numerical admission

The following 22 source records were added to 22 cards. Existing source text and every non-source card field are preserved. The three remaining cards (`aim-7m`, `aim-9m`, `aim-9x`) already list original government identity references; those references still do not accept every existing parameter or unspecified production block. These are bibliographic/identity results, not a physical-model completion count.

| Cards | Reference / locator | Evidence boundary |
| --- | --- | --- |
| aim-120c, aim-120d | [Raytheon AMRAAM](https://www.rtx.com/raytheon/what-we-do/air/amraam-missile), product identity and image captions | Manufacturer publication; family identity, with C-7 and D-3 names in captions. D-3 does not silently replace D |
| asraam | [MBDA ASRAAM](https://www.mbda-systems.com/products/air-dominance/asraam), product heading | Manufacturer identity only |
| meteor | [MBDA METEOR](https://www.mbda-systems.com/products/air-dominance/meteor), product heading | Manufacturer identity only |
| mica-em, mica-ir | [MBDA MICA](https://www.mbda-systems.com/products/air-dominance/mica-family/mica), RF/IR names | Manufacturer publication distinguishes those identities; no numerical calibration |
| iris-t | [Diehl guided-missile catalogue](https://new.diehl.com/defence/en/products/guided-missiles), IRIS-T heading | Keep IRIS-T distinct from the separately listed IRIS-T SL |
| magic-2 | [French Ministry, Mirage 2000 D](https://www.defense.gouv.fr/en/node/8342), Magic II name | Original government page indexed text; full-page retrieval failed in this session. Not acceptance of this project's Mirage 2000C fit |
| super-530d | [ECPAD DIA 28 04 32 17](https://imagesdefense.gouv.fr/fr/deux-avions-de-combat-mirage-2000-sur-la-piste-de-la-ba-base-aerienne-d-al-ahsa-armes-de-missiles-matra-super-530-d.html), archive caption | Historical identity, not engineering measurements |
| r-27r, r-27er, r-27t, r-27et | [FOI-R--4991--SE](https://www.foi.se/rest-api/report/FOI-R--4991--SE), printed p48 | Government-published research names the variants; secondary-reporting basis, not firsthand manufacturer confirmation |
| r-77m, r-37m | Same FOI report, printed pp50 and 51 respectively | Research identity discussion only; no export-name equivalence or current installation accepted |
| r-60m, r-73 | [Slovak MoD, Kempoviny, 25 June 2008](https://www.mod.gov.sk/data/att/106495.pdf), printed p4 | Historical government equipment-list identities only |
| pl-8, pl-10, pl-12 | [Original CCTV report, 2025-03-23](https://military.cctv.com/2025/03/23/ARTIhBJ4g3KjFFJg4PnYWwpg250323.shtml), names in expert segment | Original journalism, classified as secondary engineering evidence; no production subtype or fitted configuration accepted |
| pl-15 | [Original CCTV report, 2025-08-31](https://news.cctv.com/2025/08/31/ARTIrCa6IvtwxFIiXOTLUX3h250830.shtml), named identity | No PL-15E equivalence inferred |
| cj-10a | [Original Xinhua report, 2019-10-03](https://www.xinhuanet.com/politics/70zn/2019-10/03/c_1125070191.htm), Rocket Force parade context | **Conflicting fit evidence:** land-based context does not establish H-6K air launch or KD-20/CJ-20 equivalence. The source note explicitly records this caution |

A [MOD-hosted PL-10/PL-15 story](https://eng.mod.gov.cn/xb/News_213114/TopStories/4816016.html) credits Global Times/Sina; hosting does not turn it into operator technical evidence. The [CPPCC transcript](https://www.cppcc.gov.cn/zxww/2017/12/16/ARTI1513309235664454.shtml) names PL8B in a question, not an accepted generic PL-8 subtype. The [PL-10E exhibition report](https://news.cctv.com/2016/11/03/ARTI61olfmGGnZhnw7a82mTo161103.shtml) establishes an E-suffix name only. These leads are recorded here rather than silently substituted for existing card identities.

#### Carryover dispositions from this audit

| Item | Owner | Current disposition / closure condition |
| --- | --- | --- |
| Publication category absent from machine-readable source records | Codex metadata slice | Categories added to all 25 card source dictionaries; provenance output retains each cited category and rejects unknown spellings. Game references remain explicitly game references; this closes categorization only |
| Mutable game URLs and uncaptured remote publications | Codex input/evidence ownership | Open. Existing links remain historical references rather than immutable, independently admitted technical inputs |
| Exact variant identity versus family/export names | Codex catalogue ownership | Open where noted above. Preserve identity-specific evidence and its publication type; no automatic alias or parameter transfer |
| Five AAM cards without loadout references | Codex catalogue ownership; shared integration by recorded handoff | Open installation/configuration coverage, not proof of missing or installed real hardware |
| CJ-10A runtime class, inert representation and H-6K identity conflict | Codex catalogue ownership; shared integration by recorded handoff | Open. Keep the existing runtime identifiers and numerical fields unchanged while recording the conflicting identity evidence; no working cruise-missile claim |
| Stock definitions outside the authored fit roster | Joint MS integration owner, through an exclusive shared-file handoff | Open for explicit per-definition applicability, identity and fit disposition; do not drop shipped models or count test objects as operational aircraft |
| C130 visual declaration absent from generated alias manifest | Resource owner, through an exclusive shared-file handoff | Open resource catalogue discrepancy. Does not authorize payload-fit inheritance or a missile implementation change |
| Physical calibration and functional acceptance | SIM-M02 remains unaccepted | This audit supplies no new flight/solver/engagement or performance evidence. Bibliographic identity, metadata integrity and a present XML file do not close that milestone |

#### Reproducibility and focused verification

The three new source-category regressions first failed on the preceding utility/card state: missing category output and absent catalogue classifications. They then passed with the metadata changes. On the final card/code state, the selected metadata, batch-result and unchanged-generator checks passed **16/16 in 0.770 s**. The boundary-document checks passed **22/22 in 0.088 s**; the architecture plan, repository architecture and MA contract/document checkers also passed. Independent source/metadata and fleet-inventory reviews found no outstanding findings in this audit. Existing tests remain in `StoresTest`; no CMake registration, native build, GUI or full suite was changed or run. All 25 generated XML hashes were also compared to HEAD and matched.

Read-only capture/verify also covered all 25 card identities. The aggregate file-integrity scope freezes 26 explicit inputs (the 25 cards and the provenance utility) and 25 existing generated XML files. The independently frozen lists are `build/architecture-reviews/ms-first-slice/catalogue-expected-files.json`, with `catalogue-file-provenance.json` and `catalogue-source-audit.json` beside it. Aggregate digest: `666755e0274d7158b65860b36f2a042cf8a14cc4bda549e13d3ce2a416d41046`. The scoped audit compared each card against HEAD and confirmed all non-source fields identical. This scope does not claim complete generator dependencies or remotely archived scientific evidence.

The original first-slice snapshot correctly failed verification after its card/tool inputs changed. The refreshed 13-input/3-artifact capture is `aim120c-source-kinds-provenance.json` in that evidence directory, digest `4fd04a1c9dc712a987076ae80d137d8ce7c6989ae374ffe3cf2c634ae556cbed`. Its source assessment remains `not_assessed`. The separate catalogue source audit retains all 18 game-cited metadata entries without admitting them as real-equipment data.

### 10.3 SIM-M02 generator integration

Implementation continues at main `541cde6` under the user's explicit instruction to follow the original game-simulation plan. The immediate code dependency is the shared path from an offline coefficient dataset to the generated JSBSim model: currently the aero report and the XML generator independently recompute the component build-up, so importing a solver report cannot change the generated flight model. This implementation adds an optional, explicitly selected derivative-table input with matching geometry/reference checks, shared report/generator selection, exact-grid validation and deterministic generated output. Unselected models retain their existing output. Synthetic fixtures verify plumbing; they are not solver calibration or first-model acceptance.

The additional write allowlist is `tools/models/hangar/hangar/{storetables,storeaero,storebodies,storestages,motorinputs}.py`, `tools/models/hangar/hangar/geometry/aircraft.py`, the existing `tools/models/hangar/tests/{test_stores,test_missiles}.py` classes and this section. No current model's numerical constants or generated XML may be hand-edited. Radar/native shared files and CMake remain outside the window. Internal implementation tasks have disjoint paths: one task owns only `motorinputs.py` and the motor-validation tests in `test_missiles.py`; the main task integrates consumers and the table pipeline after that helper is delivered. Native execution remains serial; use only focused quick tests for these changes.

The continued catalogue-development window additionally permits `tools/models/hangar/hangar/catalogue.py` and catalogue regressions in the existing `StoresTest` class. It supplies a read-only structural audit of authored vehicles, explicit fit/loadout references, store/card/design/generated-model identities and shipped stock definitions. Unused cards, missing applicability decisions and visual-only aliases remain explicit; neither structural integrity nor an alias admits an equipment installation or a physical calibration. No fit, numerical card, generated model, native interface or radar file is changed by this checker.

#### Implemented input and generation contract

`[store] aero_table = "relative/input.json"` explicitly selects the offline input, relative to that design's directory. Without the field, the existing component build-up and generated formatting remain unchanged. An invalid selected file fails before model output is written. Both `storestages.aero` and `storebodies.files` now select through `storeaero.coefficients`; the report and generated flight model therefore consume the same input.

The strict `fsim.store-aero-derivatives.v1` document contains exactly `schema`, `card`, `geometry_sha256`, `reference`, `producer`, `mach`, `coefficients` and `crossflow`. Its geometry digest covers `x`, `r`, `fins`, `cg`, `length` and `diameter`. Reference area, diameter and moment coordinate must match the geometry in the declared `body-from-first-station` frame. The producer declaration has nonempty `name`, `version` and `case` strings; actual executable identity and reference-case acceptance remain separate evidence requirements.

The Mach grid must have at least two finite, nonnegative, strictly increasing coordinates. All eight arrays (`ca`, `cna`, `cma`, `cyb`, `cnb`, `cmq`, `cnr`, `clp`) must cover that exact grid. The five finite crossflow quantities are `normal`, `pitch`, `side`, `yaw` and `turn_m5`. Duplicate/unknown JSON fields, missing channels, booleans in numeric fields, nonfinite values, wrong card/geometry/reference and nonlocal/excluded paths fail explicitly. This input represents the generator's reduced derivative parameterization: it does not introduce a general angle/control/Reynolds grid or independently establish an operating envelope.

Imported tables, crossflow values and normalization/reference quantities retain 17 significant digits in generated output. The build check compares the complete coordinates and every channel, and binds the report to the raw input SHA-256 embedded in the generated model. Changing the input with a stale aero report fails the build checks; refreshing the aero stage restores the matching report. The diagnostic at Mach 0.6 is unavailable when that point falls outside the supplied grid. Unit conversions must remain finite before output: independent review found overflow in the turning coefficient and lossy reference formatting; focused regressions also cover reference-area and reference-coordinate overflow. The fixes preserve the unselected models' output.

Motor declarations now share one validation path in the geometry, generated-model and propulsion-stage consumers. Malformed present declarations fail, including falsey malformed declarations, nonfinite or boolean fields and invalid phase shapes. A genuinely absent motor remains unpowered. A valid phase-free visual declaration now reports no simulated propulsion instead of crashing when the generator returns no motor. Two additional stage regressions reproduced the crash and validation bypass before the fix, then passed in 0.017 s. No current motor parameter was calibrated or changed.

#### Focused implementation evidence

The integration regressions exercise synthetic input through actual temporary TOML/card files, `pipeline.Design`, aero reporting, generation and stale-input detection. Negative tests reproduced the previous ignored-input behavior, the motor consumer bypass, reference precision loss and conversion overflow before their respective fixes. The initial focused selection passed **39/39 in 5.544 s**, including provenance/batch regressions, input/generator regressions, motor validation and existing consistency checks for all 53 generated store flight models. The final combined selection, including the subsequent propulsion-stage and catalogue corrections, is recorded in 10.4. This is a selected test set, not the complete Hangar label.

A serial headless SDK probe then loaded two generated, synthetic unpowered fixtures and advanced each for one step. The measured normalized axial values matched the independent expected values, **0.10495502684046443** and **0.20990703061481802**; fixture mass also matched its input within tolerance. This demonstrates that changed imported coefficients reach the native runtime. The bounded launcher finished in 1.416 s; no native build, GUI, physical solver campaign or full suite was run. The reproducible probe and raw log are `build/architecture-reviews/ms-first-slice/native_table_check.py` and `native-table-check.log`. Synthetic-fixture evidence does not close an actual equipment model's calibration gate.

The 10.2 first-model provenance snapshot is historical after these generator inputs change. A fresh capture/verify uses the independently supplied input list extended with `storetables.py` and `motorinputs.py`: **15 inputs and 3 existing artifacts**, final digest `8942e90efbd504b577544f6533d95ee04c03496265bc2d354a4bb03023914555`. The lists and snapshot are `generator-expected-files.json` and `aim120c-generator-provenance.json` under the same temporary evidence directory. This remains an explicitly bounded input set. Existing generated XML remains unchanged, and a fresh comparison against HEAD confirmed all 25 edited cards differ only in their source dictionaries. The 10.2 aggregate card/source snapshot has a narrower declared scope and must not be treated as a generator-dependency snapshot.

#### Toolchain and remaining model gates

The bounded local lookup confirmed RocketPy 1.13.0 package metadata and its Python >=3.10 declaration, but did not execute a RocketPy reference case. It found no SU2, VSPAERO or mesher executable in the specifically checked tool directories; this is not a whole-machine absence claim. Upstream distributions provide an acquisition path without speculative compiler changes: [SU2 8.5.0 release](https://github.com/su2code/SU2/releases/tag/v8.5.0), [OpenVSP 3.54.0 archive/API](https://openvsp.org/api_docs/3.54.0/index.html) and [Gmsh 4.15.2 package](https://pypi.org/project/gmsh/4.15.2/). SU2 has a Windows OpenMP archive, OpenVSP has a Windows/Python 3.13 archive, and Gmsh publishes a Windows AMD64 wheel. No archive was acquired, installed or run in this window; runtime support and reference cases remain unverified.

SIM-M02 remains open for variant-specific admitted inputs, immutable source/executable records, independent comparison cases and actual first-model validation. Fleet identity/installation decisions recorded in 10.2 also remain open. The implemented generator and catalogue tools support game asset development and expose these limits without silently treating assumptions or file presence as accepted model fidelity.

### 10.4 Executable catalogue integrity and coverage reporting

`hangar/catalogue.py` implements the read-only standard-library audit `audit(root)` and `python -m hangar.catalogue --root . [--json]`. The deterministic `fsim.store-catalogue.v1` result follows authored aircraft through exact fit identities, station definitions, explicit loadouts, store cards, unique store designs, generated flight models and referenced engine/thruster files. It also inventories the shipped stock definitions independently, with legacy/template definitions separate. It never constructs `pipeline.Design`, generates models, substitutes compatible stores or inherits a fit through a visual alias.

Broken or ambiguous identities, missing referenced files, malformed input shapes, undeclared/duplicate station assignments, duplicate stock identifiers, malformed visual offsets, dangling visual targets and invalid engine/thruster XML types produce structural errors and CLI exit 3. Only a structurally intact graph returns exit 0; text and JSON retain `source_assessment=not_assessed`, `installation_assessment=not_assessed` and explicit pending coverage. Directory exclusions are pruned before descent, and linked/junction inputs are refused rather than followed.

The real-file result is **35 authored aircraft, 53 store cards/designs/generated models, 45 loadouts, 47 explicitly referenced store identities, 64 modern stock definitions and 2 separately inventoried legacy/template definitions**. All 48 generated engine/thruster references resolve with supported XML types. There are zero structural issues. Repeat audit and JSON roundtrip produce the same result, retained at `build/architecture-reviews/ms-first-slice/catalogue-audit.json`.

| Pending inventory | Current result | Owner / closure condition |
| --- | --- | --- |
| Authored equipment installation/fidelity | 35 unassessed fit records | Each complete payload owner; independent installation/variant evidence and functional acceptance are still required |
| Aircraft without an explicit loadout | 14: `c172`, `cf2`, `e3g`, `e7a`, `ec130h`, `iris`, `kc135r`, `kc46a`, `rc135w`, `rq4b`, `skua`, `u2s`, `uh1h`, `uh60` | Catalogue/integration owner; add supported explicit configurations or an accepted not-fitted/applicability disposition |
| Cards unused by explicit loadouts | 6: `aim-120d`, `ptb-1150`, `r-27er`, `r-27et`, `r-27t`, `r-37m` | Catalogue/integration owner; resolve applicability and supported explicit configurations without silently assigning them to a vehicle |
| Shipped stock applicability | 64 modern and 2 legacy/template records | Joint integration owner; per-definition SDK applicability and equipment disposition remain open |
| Visual alias registration | `C130 -> c130j` declaration is absent from `aircraft/models.txt` | Resource owner through its shared-file handoff; preserve the separate fit boundary |

These categories total **122 pending records**, not 122 newly discovered code defects. The existing H-6K/CJ-10A identity and inert-representation caution in 10.2 remains open and is retained in the all-store inventory. Catalogue integrity cannot close that functional or source-applicability gate.

Nineteen synthetic catalogue tests cover missing links, duplicate/unsafe identities, malformed JSON/TOML/XML shapes, station validation, aliases without fit inheritance, empty denominators, excluded-path pruning, stock ancillary XML and CLI outcomes. Independent review found two further issues: a dangling visual target was only pending, and the first pruning-test fixture wrongly created excluded temporary directories. The target now fails structurally, with a regression that failed before the fix. The pruning test now injects synthetic directory-entry names and guards filesystem calls entirely through mocks. Earlier reported test runs included the erroneous fixture writes; the final run uses only the corrected mock fixture.

The final related selection passed **60/60 in 6.116 s** after those corrections, including all new catalogue tests, table/provenance/batch checks, propulsion-stage validation and existing generated-model consistency controls. Its exact selection is reproducible with `build/architecture-reviews/ms-first-slice/quick_checks.py`; raw output is `quick-checks.log`. Independent read-only reviews closed the generator conversion and catalogue findings. Repository architecture checks passed with no Python cycles or layout exceptions; boundary-document checks passed 22/22 in 0.103 s. No full Hangar/native suite, new native build, GUI or performance campaign was run.

### 10.5 Imported-input validity and complete build evidence

Continuation baseline: main `442f75c`, 2026-10-17. Owner: Codex. The implementation window is limited to `tools/models/hangar/hangar/{storetables,storebodies,storestages}.py`, new related regressions in the existing `tools/models/hangar/tests/test_stores.py` class and this document. `storetables.payload_digest` supplies a shared, validated fingerprint of the selected input payload so finite-but-altered cached values also fail comparison. Temporary reproduction scripts/logs remain under `build/architecture-reviews/ms-first-slice/`. Current equipment parameters, generated equipment models, native/public ABI, CMake and radar paths remain outside this window.

This slice closes imported-model false success when the aero report is missing, has the wrong kind/shape or contains nonfinite/boolean expected data. It also makes the selected table's inclusive Mach bounds and current range-validity state queryable through private generated-model properties. The range flag describes input-domain availability and does not establish physical accuracy. Legacy unselected generation remains unchanged. Regressions use synthetic game assets; only related fast checks and, if needed, one bounded serial headless SDK probe are allowed. No full suite, GUI or real-equipment performance calibration is part of this slice.

#### Implementation and dispositions

Imported generation now embeds two content identifiers: the original input-file SHA-256 and a canonical selected-payload SHA-256. `storetables.payload_digest` validates all selected reference/crossflow scalars, launch CG, the complete Mach grid and eight coefficient arrays, plus the exact input/producer metadata shapes. Area and diameter must be positive; numeric fields reject booleans/nonfinite values; input paths are checked lexically without opening the report's path. Numbers are normalized to the emitted double representation. Normal report bookkeeping is excluded from the digest. The build independently hashes its cached payload, so even finite altered values or changed producer declarations cannot retain matching evidence simply by copying the file SHA.

| Finding | Delivered behavior / evidence |
| --- | --- |
| Missing, empty, unsupported-kind or malformed JSON report skipped imported checks or crashed | The generated import marker requires a supported body report; failures carry an actionable `run aero again` check |
| Nonfinite/boolean expected array values could yield a false zero residual | Expected arrays and coordinates are validated before numerical comparison; complete payload validation also covers scalars and metadata |
| Missing metadata or altered finite scalars could pass with the current source SHA | The payload fingerprint binds all selected values, declared source metadata and launch reference; malformed or changed payloads fail acceptance |
| CLI acceptance not independently demonstrated | The actual Hangar entrypoint returns 1 for `build` with missing evidence, then 0 after `aero build` on a consistent synthetic fixture; every child-process launch is blocked during this test |

Imported models additionally expose `fsim/model/imported-aero/selected`, `mach-min`, `mach-max` and `mach-in-range`. These are named JSBSim pre-functions; the range predicate uses inclusive `ge`/`le` comparisons against the supplied endpoints. It is updated on aerodynamic evaluation and should be queried after the corresponding native step. Paused or otherwise unevaluated state retains the last evaluated flag. Upstream endpoint-clamped table lookup behavior remains unchanged; the diagnostic exposes axis coverage rather than suppressing forces or admitting a calibrated physical envelope. The implementation uses the existing [JSBSim function binding](../../../../third_party/jsbsim/src/math/FGFunction.cpp) and [model pre-function evaluation](../../../../third_party/jsbsim/src/math/FGModelFunctions.cpp); no new runtime solver or dependency was introduced.

#### Acceptance evidence for this software slice

The new negative regressions first reproduced absent/wrong-kind report false success, nonfinite/boolean cache acceptance, malformed input-metadata crashes and finite-but-altered scalar acceptance. The final selected set passed **70/70 in 6.680 s**, including all 23 offline-input tests and existing provenance, catalogue, motor and all-store generator compatibility controls. All 53 existing generated store-model outputs remain compatible. Raw output is `build/architecture-reviews/ms-first-slice/sim-m02-validity-quick-checks.log`; the retained `quick_checks.py` reproduces that related selection.

After confirming the serial native slot was idle, one headless probe created three separate synthetic unpowered Worlds and advanced each once. For a supplied Mach interval `[0.25, 0.75]`, queried native Mach values **0.029730591381542555**, **0.4410775444930083** and **0.8615547603183129** returned flags **0, 1 and 0** respectively; selected/min/max also matched. The bounded process finished in **1.461 s**. Script and raw log are `native_domain_check.py` and `native-domain-check.log` in the same temporary evidence directory. Exact endpoint equality was not separately exercised natively; the inclusive comparison elements are covered by the generator regression and upstream semantics.

Both independent read-only reviews closed the scoped report, payload-binding and diagnostic findings. The architecture checker passed without Python cycles or layout exceptions. The preceding 15-input/3-artifact generator snapshot correctly failed verification after these source edits; the new capture/verify is `aim120c-validity-provenance.json`, digest `2549657e1bea601a5e145c9ad02904e54b3d689b7a7e73ad9f0d36a9c3a8ea3e`, using the same independently supplied `generator-expected-files.json`. Its scope remains file integrity only. SIM-M02's actual equipment inputs, independent physical comparisons and first-model calibration gates remain open under 10.3.

### 10.6 Propulsion runtime reset

Continuation baseline: main `090f318`, 2026-10-17. Owner: Codex. The exclusive native write/build window covers `cmake/{JsbsimPatches,Install}.cmake`, `src/simulation/flight/JsbsimModel.cpp`, `tests/simulation/test_vehicle_pool.cpp`, the new synthetic fixture `tests/simulation/data/rocket_reset/` (airframe, solid/liquid engine and nozzle XML), `THIRD_PARTY_NOTICES.md` and this document. Changed runtime symbols are limited to the generated JSBSim `FGRocket::ResetToIC` override, clearing residual force/moment vectors in `FGThruster::ResetToIC`, and excluding rockets from `FGPropulsion::GetSteadyState`'s aircraft-engine warm-up loop. `JsbsimModel::engineFuelFlowKgS` reads oxidizer flow through its existing tied native property rather than the concrete rocket subclass header; JSBSim's own rocket constructors use its generated header consistently. Vendored sources, public SDK signatures/layouts, aircraft equipment parameters, guidance and radar remain unchanged. Temporary logs stay in `build/architecture-reviews/ms-first-slice/`.

The runtime issue under verification is that a used rocket restores its tanks on reset while retaining rocket-owned burn/impulse history. The implementation must reset per-run state through the existing JSBSim patch mechanism, preserve configured values and force calculations, and match a fresh synthetic model before and after a short replay. A second reproduced runtime issue is the aircraft-engine warm-up loop accumulating solid-motor burn history before simulation time advances; powered load/reset must instead leave the motor unburned until actual stepping. The bounded `FGRocket::Calculate` initial-state branch also refreshes a function-valued observable at the new conditions without advancing burn/consumption. Related quick gates cover solid and liquid engines, powered and unpowered reset, and ordinary aircraft reset controls. No full suite, GUI or performance campaign is authorized in this window.

Public external-dimension review also found conflicting span descriptions: the [NAVAIR AMRAAM page](https://www.navair.navy.mil/product/AMRAAM) lists C/D span as 19 in, whereas the indexed Raytheon C-7 brochure in the [government-hosted appendix](https://media.defense.gov/2020/Jul/14/2002456822/-1/-1/0/JAX-FEIS-OEIS-VOL2-APPENDICES.PDF) lists wing/fin spans of 17.5/17.6 in. Direct retrieval of that appendix returned HTTP 403, so its page/table has not been visually verified. Exact geometry remains open to Codex: obtain a readable primary publication and reconcile variant and measurement definitions before changing the current model. No value is selected merely to produce an asset diff. The separate cross-World launcher-object issue is also pending a release-interface ownership window; it does not block the bounded propulsion-state correction.

#### Delivered runtime behavior and acceptance

| Finding | Disposition / evidence |
| --- | --- |
| Rocket reset refilled tanks but retained burn time, consumption, impulse and flow | Generated `FGRocket::ResetToIC` calls the base reset and clears only run history. The original regression failed 602 assertions before the fix |
| Initial-state pass reused a nozzle's previous force | Generated `FGThruster::ResetToIC` clears native force/moment vectors; the existing body-force refresh computes the body caches. The state-only rocket fix still failed 238 assertions until this was corrected |
| Aircraft warm-up burned a solid motor before simulation time advanced | `FGPropulsion::GetSteadyState` skips each rocket while preserving other engines' warm-up. The startup regression failed 8 assertions before this correction |
| Function-valued parameter retained a previous evaluated value | Independent review identified direct and cached pre-function cases; both synthetic forms reproduced load/reset failures. The suspended initial-state pass refreshes pre-functions and then the observable without accumulating a burn |
| Liquid starvation coverage was not guaranteed | Regression now explicitly requires both liquid tanks empty and vacuum thrust zero before reset. New ignition also checks reported combined flow against actual fuel-plus-oxidizer tank loss per step |
| Generated subclass header leaked into the flight module's include roots | Source/configured gates rejected the attempted include root. It was removed; the adapter uses the existing tied native oxidizer getter, with no architecture-checker exception or rule change |

Two maintained synthetic lifecycle cases are registered through the existing `fsim_tests` discovery, with four small fixture XML files under simulation test data. They exercise solid/liquid history, an unpowered reset, idle after a prior burn, fresh-model replay, starvation, repeated reset, powered startup and a function-valued input. Input numbers describe no equipment. No model coefficient, motor formula, guidance or equipment configuration was calibrated in this window. The generated patch header adds an override of an existing virtual slot and no data members; public SDK signatures and layouts are unchanged. The installed source-change notices include all four new generated patch files.

The final selection passed **9/9 in 10.96 s**: the two synthetic cases, C172 reset, violent-history reset, AH-1S reuse, the existing five-aircraft public SDK reset/reuse comparison, source/configured architecture gates and `ma_contract`. Checker unit gates had also passed earlier; no checker was modified. Both independent read-only audits closed their scoped findings, including the named pre-function dependency. Native runs used one serial slot, discovered nonzero counts, `--no-tests=error` and 30/60 s per-test timeouts.

The public Python SDK probe passed after a used model reset, six idle steps and twelve newly powered steps against a fresh World. It checked the actually loaded `libfsim.dll` and `libJSBSim.dll` paths and SHA-256 values against the current build. Its first attempt loaded older staged libraries and is not acceptance evidence; the existing `python_package` target refreshed them before the successful probe, which was repeated on the final dependency-refresh fix. Raw logs and `rocket_sdk_check.py` remain under `build/architecture-reviews/ms-first-slice/` (`rocket-final-quick.log`, `rocket-sdk-check.log`, with the earlier red/fix logs retained separately). Only incremental headless targets and these related quick gates ran; no full suite, Release build, GUI or performance campaign ran.

`GetSteadyState` now deliberately contributes no rocket force, including when called for trim; it is not a steady-state rocket solver. Actual integration still uses the existing native calculations. Constant and function-valued synthetic inputs are covered; first-equipment physical calibration and independent engineering comparisons remain open under 10.3. The external-geometry conflict above and the foreign-World launcher-object issue (`src/simulation/api/World.cpp: World::launchWeapon`, `python/fsim/world.py: World.launch_weapon`) remain explicit Codex carryovers. The latter must reject a Vehicle owned by another native World before release, without changing either World's state, while preserving local numeric-ID calls. These open items are not represented as accepted model fidelity.

### 10.7 Powered-flight comparison and aerodynamic experiment

Superseding delivery instruction, 2026-10-18: publish the current partial
implementation, then provide Claude a standalone numerical research directory
covering only mesh sensitivity, panel closure and solver convergence. Codex
performs independent acceptance of those deliveries. The complete-program
assignment below is dated history; the research handoff carries no SDK,
equipment-parameter or other model-package implementation authority. Remaining
physical criteria stay open.

Continuation baseline: main `57bc4c78c58f52b952126371319db9879131ff8a`,
2026-10-17; initial tree clean. The user assigned the complete SIM-M02
continuation to Codex and retained SIM-M01 with Claude. The exclusive path/symbol
allowlist, physical acceptance criteria, actual inputs, commands and evidence
are recorded in [Missile flight validation](../../../sdk/missile-flight-validation.md).
That record supersedes the first-slice stop point, without admitting unavailable
equipment data or transferring radar/generic MA ownership.

The foreign-World carryover in 10.6 is closed: the C++ release facade checks
native World identity before dispatch; Python rejects foreign Vehicle handles.
New regressions reproduce the previous release/state mutation and preserve
same-native-World wrappers and local numeric IDs. Motor construction additionally
rejects finite inputs whose derived impulse/geometry overflows, impossible
residual structure/inertia/grain installations, and collapsed raw or serialized
consumed-mass knots before generating XML.

The independent RocketPy campaign exposed a native pulse defect: a single
high-thrust step can cross the tiny consumed-mass interval of an assumed 2 N
coast, triggering the next pulse early. A private versioned table name selects
the existing engine BurnTime coordinate; unmarked tables retain their previous
mass coordinate. No public signature, member layout or vtable changes. Only the
generator-owned PL-15, R-37M and R-77M motor files changed, through Hangar; the
other 98 generated files in the 53-store census remain identical. A synthetic
native fixture covers delayed ignition, zero coast, terminal shutdown,
starvation/reset, 60/120 Hz and fresh replay, with an unmarked legacy control.

An additional end-to-end native gate reproduced unlit released AIM-120C motors:
the empty aircraft control cascade replaced launch throttle with neutral before
the first actual motor step. The old warm-up had masked that missing command
preservation. `World::preStep` now retains launch inputs for Weapon entries and
continues all native guidance/effects/physics; weapon reset selects the requested
engine state without changing aircraft controls or authority/task/C2 semantics.
The public regression first failed 32 assertions; both engine requests and their
reversal are covered for AIM-120C/PL-15 at 60/120 Hz. Hangar `_burn` also requests
ignition explicitly. Actual AIM-120C/PL-15 burn checks passed; the unchanged
end-to-end engagement gate now passes, including worker determinism and coast.

The maintained powered-flight runner verifies current generated bytes, loaded
DLL provenance and declared reference conditions, then compares every finite,
aligned sample. The matched nonrotating-sphere campaign passed **48/48 cases**
across all 24 AAMs (vacuum and constant air, 480/960 Hz): 96 native runs and
16,592 samples. Worst residuals are **0.052202 kg**, **1.285336 m/s** and
**4.157134 m**; worst impulse difference is **0.184871 percent**. These compare
the same game inputs in independent implementations, not real weapon performance.
The earlier rotating-Earth/flat-reference diagnostic failures remain separate.

SU2 8.5.0, OpenVSP 3.54.0/VSPAERO 7.2.2 and Gmsh 4.15.2 were pinned under
`D:/ENV`, with support/license/hash records and upstream execution checks.
Actual AIM-120C authored-geometry trials **failed acceptance**: SU2's converged
first-order fields failed drag/symmetry checks and second-order restarts missed
the residual limit; VSPAERO had lifting wakes and met printed linear-solver
targets but produced unphysical near-surface loads. Mesh quality, sampled geometry
differences and far-wake diagnostics are retained explicitly. The runner rejects
these results and always keeps full model admission separate. No aero dataset
was imported, and no damping/crossflow channel was invented.

The subsequent numerical diagnosis corrected near-singular nose representations
and introduced a guarded full mirrored CFD mesh without imposing flow symmetry
boundaries. Experimental second-order +/-2-degree cases converged and were
antisymmetric, but coarse/fine CA and CM changed 126.2/71.9 percent. VSP's native
point nose repaired the isolated body; the complete assembly retained unmatched
fin edges and nonlinear nonconvergence. A separate literal Cp integration remains
a diagnostic and does not replace failed near-surface edge-force loads. The
maintained CLI now requires matching complete nonlinear histories in addition
to inner linear convergence, physical gates and independent residuals. Its final
actual run still returns `failed_static_comparison`, `model_accepted=false`.
The combined safeguards passed **44/44**; the final actual mesh independently
passed the corrected topology guards, including rejection of folded adjacent
tetrahedra. Full details and the 24-file final source/tool/case hash manifest are
in the linked validation record and `aero/current-run/` evidence directory.

Current focused evidence: generator **26/26 in 0.582 s**; final headless CTest
**22/22 in 64.77 s**, including the installed C++/C99/Python equipment consumer
(25.90 s); final reset/authority selection **2/2 in 1.16 s**; release/eligibility/
engagement selection **3/3 in 60.09 s**; final Release **24/24 in 39.97 s**.
These final software checks completed on 2026-10-18. Both presets rebuilt the changed native targets and
Python package. Independent review confirmed all 122 unique input/model/DLL
hashes and the listed numerical sources in the final fleet report. Logs and full
trajectories are under `build/architecture-reviews/sim-m02/`; this is not a full
regression, Viewer or performance campaign. Full SIM-M02 remains open for the
primary-source geometry/motor admission, credible aerodynamic convergence and
independent agreement, dynamic/viscous/control data and variant/fleet physical
acceptance listed in the validation record.

## 11. SIM-M03 independent correctness, 2026-10-20

Baseline: main `0d4f25a5df3fdd23696e696644cbd0ee32ab23af`, initially clean.
The user assigned the complete SIM-M03 continuation while Claude continues the
three SIM-M02 numerical investigations in the standalone AeroNumericsLab
directory. The user subsequently confirmed that dependent work must await that
handoff. Exact independent-feature file/symbol ownership is recorded in the
[continuation plan](../../../superpowers/plans/2026-10-20-sim-m03-independent-correctness.md).

The [weapon fidelity validation](../../../sdk/weapon-fidelity-validation.md)
records these implemented corrections: consistent on-rail/released IR reach,
body-axis seeker field of regard, invalid guidance input rejection without state
mutation, unknown timing estimates, fuze sweeps restricted to the armed/live
interval, target-only fragment momentum transfer, and legal target visibility
through direct C++/C99/Python Agent strike-status getters. Existing native
algorithms are reused; no equipment input, generated model, public layout or
generic schema is changed. An independent read-only review found no actionable
code defect within this scope.

Current Headless evidence is 9/9 core CTest cases, 8/8 public/installed CTest
cases and 20 damage-model Catch2 cases (4,814 assertions). The core selection
includes native engagement/determinism/terminal-event controls and the 17-case
guidance and 11-case temporal-fuze executables. Installed consumers report the
actually loaded C++/C99/Python libraries, modules and data roots. Exact commands,
logs, timing and provenance paths are in the linked record. This is focused
software evidence; no new full regression, render-enabled Release or Viewer
campaign is claimed.

The final `ma_contract` also passes. A supplementary selector self-check fails
one of 101 internal cases on an unchanged, pre-existing SIM-M02 CMake-fixture
routing gap. The baseline-rule probe and owner/closure are recorded as HC-53 in
the [carryover register](../../validation/handoff-carryover-register.md#sim-m03-audit-follow-up-2026-10-20).
Overall repository-check success is not claimed.

Full SIM-M03 remains open for accepted SIM-M02 dynamics/control inputs, admitted
radar/IR/seeker/actuator and fuze/lethality references, independent per-variant
validation and fleet coverage. Modeled weapon return transport/assessment and
accepted fused weapon input also remain open at the existing owner seams.
At that initial handoff, idealized servo/uplink, decoy timing and post-separation damage geometry
limits are recorded explicitly. A solver research folder, static convergence or
a passing native engagement cannot replace those acceptance gates.

## 12. SIM-M03 finite-control priority, 2026-10-20

Baseline: main `85e7d251c5de702de5d2ec66115a0886bd0702cd`, after the committed
HC-53 correction. The user prioritised actual missile surface control while
Claude continues the independent SIM-M02 investigation. The
[finite-control plan](../../../superpowers/plans/2026-10-20-finite-missile-controls.md)
records exact implementation, generator, regression and shared-path ownership;
one root owner executes the serial native build/test gates.

The [mechanism and evidence](../../../sdk/missile-control-validation.md) replace
guidance's ideal aerodynamic cancellation/force/moment injection with a bounded
surface-command autopilot and generator-owned finite individual-fin actuators.
Geometric fin forces and their same-pressure-centre moments enter JSBSim's
normal aerodynamic integration. The 24 current AAM models are regenerated;
motor/nozzle bytes, generic MA ownership and public layouts remain unchanged.
The three-axis position report derives from actual fins; individual-fin residual
details remain available as model properties and need separate Viewer evidence.

This closes the missing execution mechanism, not physical model acceptance.
The explicitly estimated actuator/controller constants and control-effectiveness
build-up retain accepted SIM-M02 and measured per-variant response prerequisites.
At this finite-control handoff HC-54/HC-55, calibrated sensing/lethality and
modeled/fused feedback remained open; the subsequent review below refines them.

## 13. SIM-M03 continued physical-chain review, 2026-10-20

Baseline: main `16da711bf59665a538919f2abb2dfda2ab11278f`. The user authorizes
continued parallel audit and independent repairs while Claude continues SIM-M02.
The [plan](../../../superpowers/plans/2026-10-20-sim-m03-physical-chain-audit.md)
records root shared-path/serial-build ownership and three disjoint worker paths.
The [audit](../../../sdk/sim-m03-physical-chain-audit.md) records corrections,
source-linked remaining approximations, required owners/inputs and current gates.

No generic feature or external numerical-model ownership transfers. Dynamics and
control calibration awaits the accepted SIM-M02 handoff; sensor, weapon-link,
vulnerability and detached-body prerequisites retain their separate owners.
