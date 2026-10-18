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
