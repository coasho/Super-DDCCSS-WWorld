# Current capabilities and milestone status

Progress reviewed: 2026-10-16, `main` at `d344e2b`. M7 core is accepted; M8 generic is implemented, corrected after its independent review, and awaits independent acceptance (below); prior full-suite results retain their dated scope.
First maintenance pass reviewed: 2026-10-12, `2b83f24` (implementation baseline `32e78a6`).
Second maintenance pass: `f8cfec9`; Architecture A1: `6652584`; Architecture A2: `34e5516`; Architecture A3: `e79f268`;
Architecture A4: `e708b87`; Architecture A5: `2efea05`; Architecture A6: `b57947d`, based on `39be3c1`.
Current development and delivery use `main`, including both maintenance passes and Architecture A1 through A6.
Dated verification records retain their original commits, execution context and acceptance scope.

This is the entry point for what exists now, which delivery it belongs to, and what remains open. The first
maintenance pass consolidated documentation and development guidance, corrected outdated descriptions, and retired
the obsolete tactical prototype and orphan visual/compiler artifacts. The second pass aligns build/run/test presets
and repairs standalone SDK packaging and consumer acceptance. Architecture A1 starts the third pass by enforcing
native module dependencies, separating the Mission Systems C adapter and assigning stateless launch eligibility
to the weapon module. A2 retires private MA consent bookkeeping and separates C/Python native DMS/C2 adapters
while retaining public history and original state-transition timing. A3 separates C control/VI and Python native
control/Mission handlers, retaining shared handles, errors and registration. A4 separates C Scenario/Environment
and Python World/Scenario/Environment handlers while retaining core runtime ownership. A5 separates C/Python
VecEnv/Recording handlers, retaining C++ wrappers and coupled type/buffer callbacks. A6 starts World state-owner
decomposition with private store-to-flight-model bindings, retaining lifecycle and simulation order. Public interfaces, ABI layouts,
physics and aircraft model bytes remain unchanged.

Use the [documentation index](README.md) to find reference material, the [development guide](development.md) for
build and contribution procedures, and the [resource guide](resources.md) before editing generated assets.

## How to read status

- **Implemented** means the current source provides the named behavior. It does not imply that every aircraft,
  configuration, hardware target, or physical performance claim has passed acceptance.
- **Recorded delivery** means a named validation record reports completed checks on its stated build. It is not a
  fresh test run on the reader's checkout.
- **Partial** means an implementation exists but the original stage's scope or acceptance criteria remain open.
- **Planned** means an accepted objective or candidate exists without evidence here that it has been delivered.
- **Historical** identifies dated decisions, plans, measurements, and migration records. Their dates and scope
  remain attached to the original evidence.

API contracts belong in `docs/sdk/`; accepted architectural constraints remain in their ADRs. This page summarizes
their current relationship. A newer implementation or demonstration does not silently close every older roadmap
criterion. Update this page and the affected reference together when a capability changes.

## Capability map

| Area | Current state | Contract and evidence | Limits and remaining work |
| --- | --- | --- | --- |
| Native simulation and SDK | Implemented: caller-driven `World` stepping, vehicles, JSBSim flight, environments, effects, scenarios, recording, C++/C/Python interfaces and vectorized environments. | [SDK index](sdk/README.md), [world](sdk/world.md), [scenarios](sdk/scenarios.md), [VecEnv](sdk/vecenv.md). | Public interfaces are under `include/fsim`; `src/` is internal. Test success is scoped to its configuration and cases. |
| Vehicle Interface and Flight Autonomy | Implemented control contracts, activities, grants, performance/navigation reports, routes, patterns, launch and recovery. Support and applicability vary by aircraft. | [Control API](sdk/control.md), [Vehicle Interface](architecture/modules/simulation/vehicle-interface.md), [Flight Autonomy record](architecture/modules/simulation/flight-autonomy.md), [testbench](../src/applications/testbench/README.md). | Retain the open control and stability findings in the [backlog](backlog.md). An advertised capability or a successful process exit is not blanket full-load flight acceptance. |
| Agent, MA core, range and referee separation | Implemented agent handles, authority checks, measured designations, messaging and process-isolation interfaces; the ADR records AB-1 through AB-6. | [Agent API](sdk/agents.md), [ADR-31](architecture/design/agent-boundary.md), generated [boundary table](sdk/boundary.md). | Truth snapshots remain range/referee interfaces. New exports require classification and boundary checks; original engagement-scoring acceptance remains distinct from API access classification. |
| Mission Systems host and payload data | Implemented discovery, payload state/BIT, capabilities, activities, grants, cards and aircraft fits. | [Mission Systems API](sdk/mission-systems.md), [ADR-30](architecture/modules/simulation/adr-30-mission-systems.md). | A card or installed payload does not prove its whole physical model exists. Per-capability support and per-aircraft fits remain relevant. |
| RF, RCS, radar and tracking | Implemented RF propagation, mesh-derived RCS tables, detection radar, trackers and signal processing. RWS/TWS/STT and ACM presets are present. The roadmap records MS-1 through MS-6 and MS-4b deliveries. | [Mission Systems API](sdk/mission-systems.md), [dated MS progress](architecture/modules/simulation/adr-30-mission-systems.md), `tests/simulation/test_radar_modes.cpp`, `tests/simulation/test_signal.cpp`, `tests/simulation/test_track.cpp`. | Signal-level processing has a different cost from detection-level processing. Loaded-store RCS and other model gaps remain in the backlog. |
| Aircraft and store generation | Implemented Hangar designs, models, JSBSim files, manifests, carriage and the recorded store build. | [Resource ownership](resources.md), [Hangar](hangar.md), [store build record](hangar-stores-build.md). | Source inputs, generated designs and runtime products have different editing rules. Reproducible generation does not establish calibrated real-world flight performance. |
| Native missile engagements | Implemented release, native motor/flight state, radar and infrared seeker observations, guidance, countermeasures, fuzes, damage, terminal records and replayable events. Recorded delivery at the implementation baseline. | [Engagement contract](sdk/engagement.md), [delivery validation](sdk/engagement-validation.md), `tests/simulation/test_engagement_native.cpp`. | Guidance, fin response, seekers, decoys and launch-zone parameters are game estimates. This is not completion of the original high-fidelity MS-7c/MS-7d/MS-7f criteria. |
| Launch zones and countermeasures | Implemented advisory launch-zone queries and native chaff/flare inventory, releases, motion and expiry. | [Engagement contract](sdk/engagement.md), `tests/simulation/test_countermeasure_core.cpp`, `tests/simulation/test_weapon_guidance.cpp`. | Advisory envelopes do not guarantee intercepts or satisfy MS-8's original published-range acceptance. Direct dispenser APIs do not mean every electronic-attack capability is implemented. |
| Warning receivers and other sensors | ESM/RWR contacts are implemented. Other payload families retain their own support states and staged modeling work. | [Mission Systems API](sdk/mission-systems.md), `src/simulation/mission_systems/esm/`, [remaining MS stages](architecture/modules/simulation/adr-30-mission-systems.md). | Native seeker and dispenser support is not a claim that full IRST imagery, IFF, jamming, all guns, PNT, or the OMS bridge is complete. |
| Native viewer, effects and vision | Optional Vulkan/VSG viewer and vision modules exist, with terrain, aircraft/store visuals and engagement effects. Native fullscreen lifecycle and screenshot corrections have passed focused GPU checks. | [Viewer](sdk/viewer.md), [vision](sdk/vision.md), [current viewer validation](sdk/viewer-validation.md), [Hangar effects](hangar-effects.md). | The viewer-correction baseline failed the 4K performance gate for three of four scenes; performance of the replacement native hit fixture has not been remeasured. Aircraft materials retain existing detail; aircraft cast shadows are not enabled. Current acceptance is scoped by the linked record; older GPU observations retain their original hardware and build scope. |
| Testbench | Flight Autonomy, platform and stability scenarios remain active. The obsolete `tactics.bvr` prototype and its independent Python simulation have been removed. | [Testbench guide](../src/applications/testbench/README.md), `src/applications/testbench/fsim_testbench/registry.py`. | Native engagement acceptance is provided by SDK tests and its delivery record; no replacement testbench combat scenario is introduced here. |

## Milestones

Use the domain-qualified names below in status reports. Existing `M1` identifiers in historical documents are
preserved, but never use an unqualified `M1` to select work.

| Name used here | Original scope | Current disposition |
| --- | --- | --- |
| **Platform M0-M4** | Foundation milestones in the [system architecture record](architecture/design/platform-baseline.md): simulation, terrain, SDK/RL interfaces, viewer, vision and external training. | Simulation, SDK/VecEnv, viewer and camera implementations exist with dated progress records. The original generic `ext/env_server` proposal is not an implemented module. Original throughput goals and target matrices are not asserted complete or freshly verified. |
| **Platform M5** | Public v1 release, packaging, licensing, benchmarks and frozen interfaces. | Open release milestone. No local `v*` release tag was present at review; packaging code alone is not public release acceptance. |
| **Mission Systems MS-1 to MS-6, MS-4b** | Host/data, RF, RCS, radar, trackers, signal processing and engagement radar modes. | Recorded implementations and dated stage evidence exist. Outstanding fixes or model limits remain open individually. |
| **Mission Systems MS-7 / MS-8** | Store/weapon lifecycle, high-fidelity flight models, seekers, guidance, lethality, validation and launch zones. | Partial against the original criteria: stores, assets, damage and a native game-assumption engagement path exist; higher-fidelity calibration and published-performance acceptance remain open. See B-36 and related [backlog](backlog.md) items. |
| **Mission Systems M1** | The autonomous end-to-end loop in [ADR-30](architecture/modules/simulation/adr-30-mission-systems.md), including the original modeling, sensor, scoring and demonstration acceptance. | Keep open as the original acceptance target. The native-engagement delivery supplies a narrower working path; it does not close every M1 dependency or authorize the next stage automatically. |
| **Native engagement delivery** | SDK ownership of game-assumption missile engagements and countermeasures; native integration and client-state consumption. | Recorded local delivery: [contract](sdk/engagement.md), [validation record](sdk/engagement-validation.md), [design](architecture/modules/simulation/native-engagement-design.md). Its browser-client results do not cover the retired testbench prototype or a rebuilt native viewer. |
| **Agent boundary AB-1 to AB-6** | Classification, scoped handles, designations, mission data, messaging/authority, and process isolation. | Recorded completed migration in [ADR-31](architecture/design/agent-boundary.md). Later engagement records do not by themselves prove every original referee/reward-scoring requirement. |
| **First maintenance pass** | README/navigation, current capability and milestone status, development guidance, resource-generation ownership and removal of confirmed obsolete content. | Completed and locally verified below. Retired the tactical prototype, deleted orphan visual/compiler artifacts and corrected descriptions/tests/report wording. No architecture refactor, launcher/packaging repair, model regeneration or additional feature milestone is included. |
| **Second maintenance pass** | Consistent configure/build/run/test/dist presets, first-build entry, independent SDK runtime data and installed-consumer acceptance. | Completed and locally verified below: 535 functional CTest cases, real launch/build/dist commands and independent installed consumers. Native module architecture and simulation models are unchanged. |
| **Architecture A1** | Executable native dependency boundaries, Mission Systems C adapter separation and weapon-owned launch eligibility. | Implemented and locally verified below: 538 functional cases plus three architecture checks; 1,338 DLL export names match the baseline. Internal ownership changes preserve public interfaces and simulation behavior. The broader architecture queue remains open. |
| **Architecture A2** | MA private consent retirement and C/Python native DMS/C2 adapter separation. | Committed as `34e5516` and locally verified below: 547 functional cases plus three architecture checks, unchanged exports and installed-consumer/distribution acceptance. Public history remains queryable; aggregate history reclamation is deferred. |
| **Architecture A3** | C control/VI and Python control/Mission adapter separation. | Committed as `e79f268` and locally verified below against the [A3 design](architecture/migration/history/2026-10-12-architecture-adapters-design.md): 548 functional cases plus three architecture checks, unchanged exports/runtime descriptors and installed-consumer/distribution acceptance. |
| **Architecture A4** | C Scenario/Environment and Python World/Scenario/Environment adapter separation. | Committed as `e708b87` and locally verified below against the [A4 design](architecture/migration/history/2026-10-12-architecture-world-adapters-design.md): 549 functional cases plus three architecture checks, unchanged public exports/native identities and installed-consumer/distribution acceptance. |
| **Architecture A5** | C/Python VecEnv and Recording adapter separation with retained C++ and runtime type owners. | Committed as `2efea05` and locally verified below against the [A5 design](architecture/migration/history/2026-10-12-architecture-batch-recording-design.md): 550 functional cases plus three architecture checks, unchanged exports/native identities and installed-consumer/distribution acceptance. |
| **Architecture A6** | World store-to-flight-model binding state ownership. | Committed as `b57947d` and pushed at the user's request. Locally verified below against the [A6 design](architecture/migration/history/2026-10-12-architecture-store-binding-design.md): all 596 headless CTest cases, including 40 Hangar cases, exact eight-variant baseline replay, unchanged exports/native identities and installed-consumer/distribution acceptance. |
| **MA training M0** | Generic contracts and compatibility baseline of the [MA training plan](superpowers/plans/2026-10-12-ma-system-training-readiness.md). | Contract frozen: the [mission training contract](sdk/mission-training.md), its [schema](sdk/ma-contract.schema.json) and fixtures, the [composition vocabulary](sdk/ma-composition.md) and the dated [compatibility baseline](sdk/ma-compatibility-baseline.md) (599 of 599 headless cases at the baseline; C++, C99 and Python consumers frozen). M0 itself changed no SDK header, export, adapter or behaviour. |
| **MA training M1** | Explicit MA-service binding: which Agent of a platform owns each command capability and each side of its DMS. | Implemented in C++ (`fsim/MAService.h`), C (ABI 1.72) and Python, additive: configured ownership is the same whatever order the Agents were made or bound in, overlapping ownership and foreign controllers are refused, and a platform with no binding behaves as before. A binding takes back what stood in its way - another controller's grant, the default policy's among them, and policy activities that are not their capability's owner's, flight and Mission Systems alike - and a bound platform stays under grants; the forms that take the Agent itself refuse another World's (C ABI 1.74) ([contract](sdk/mission-training.md#9-service-binding), [agents.md](sdk/agents.md#ma-services)). |
| **MA training noncombat M2** | Legal reports with their provenance: navigation, own tracks, passive contacts, link tracks, peers, tasks and authority. | Implemented as Agent calls in C++ (`fsim/MAReports.h`), C (ABI 1.73) and Python, additive: each report is the Agent's existing read with its source, sample and receive times, age and validity; unknown, stale, invalid, unavailable and not-applicable are kept apart, a peer's position report and its heartbeat each keep the delivery they came by (ABI 1.74), and nothing the referee alone knows enters one ([contract](sdk/mission-training.md#legal-report-calls-noncombat-m2), [agents.md](sdk/agents.md#legal-reports)). Verified by `ctest -L ma-training`, which includes the installed C++, C99 and Python consumers. The internal mission episode owner (M4) has passed independent acceptance. The public gateway is implemented as M5 core and awaits independent acceptance; engagement extension handlers, full M5 and full joint acceptance remain open. |
| **MA equipment access/reports** | Own-platform dispenser inventory/release, own-designation advisory launch zones, stores/strikes/inventory and ingress-delivered consent reports. | Additive C++/C/Python interfaces (C ABI additions 1.75); native activity/release correlations with true target IDs redacted and no weapon telemetry. [Equipment verification](sdk/engagement.md#equipment-interface-verification-2026-10-14) records headless/Release builds, focused regression, installed consumers, frozen ABI and protected-input checks. Mission gateway handlers and full joint acceptance remain pending. |
| **M4 native-state fixture input** | Reusable same-build setup for simultaneous damage, live weapon/decoy, store consumption, pending task/consent/message and removal state. | [Fixture and verification](sdk/native-state-reset-fixture.md) supplied on 2026-10-15 with deterministic fresh/replay and legacy-reset controls, and applied to the M4 episode reset by `mission_episode`; M4 independent acceptance completed at `54b7fb9`; M5 core is implemented and awaiting acceptance. |
| **MA training M5 core** | Public mission episodes: `MissionEpisode`, `MissionEndpoint`, `MissionReferee`; one immutable legal snapshot per service per decision tick; submit validates and queues, step dispatches in service-id/request-id order through each service's own Agent; deduplication, retention, native receipts; noncombat criteria and referee documents; strict actor isolation. | **Implemented, awaiting independent acceptance** in C++ (`fsim/MissionEpisode.h`), C (`fsim/fsim_mission_env.h`, ABI 1.76) and Python (`fsim.mission`, `fsim.isolation.MissionEndpointProcess`); see the M5 core verification below. Full M5 remains pending: no extension definition or handler is installed, so every extension key and request kind is refused. The independent review's four runtime defects and the `gateway.close` checker condition were corrected in `57a846d`, which passed independent review. M6 noncombat is independently accepted; M7 core is implemented and awaiting acceptance (below); M8, joint and Viewer acceptance are not started. |
| **MA training M6 noncombat** | Composition over the mission gateway: two policy components, a BT executive and a planner sharing one snapshot per tick; deterministic arbitration; selected requests through the bound endpoint; explicit component lifecycle. | **Implemented in `84768ea`, independently accepted** as the example `examples/training/ma_composition` ([composition](sdk/ma-composition.md#7-the-example-as-implemented-m6-noncombat)), on the installed public Python SDK alone; see the M6 noncombat verification below. Full M6 composition scenarios consume full M5 and remain pending. |
| **MA training M8 generic** | Installed consumers and joint integration: recorded joint chains replayed through public endpoints; generic joint cases; installed C++, C99 and Python consumers and a strictly isolated actor writing one trace; compatibility held to the frozen baseline. | **Implemented, corrected after the independent review of `d344e2b`, awaiting independent acceptance**; the [validation record](sdk/ma-training-validation.md) lists each joint case's evidence. Delivered: J1, J2, J4, J7, J6 (one exact gap pending: speed resumption after an override's cancellation) and the generic parts of J3, J8 and J10. Pending: J5, J9, the combat parts of J3, J8 and J10, and complete Viewer acceptance - full M8 and the program's completion consume them. |
| **MA training M7 core** | Batches over independent episode owners with autoreset; per-role flags kept apart from `episode_done`/`reason`; final documents by principal; a lost platform; explicit communication/resource costs (`ma.costs.v1`); critic truth apart from actor views; a dependency-free Python training adapter. | **Implemented, independently accepted at `0cf0150`** in C++ (`fsim/MissionBatch.h`), C (`fsim/fsim_mission_env.h`, ABI 1.77) and Python (`fsim.mission.MissionBatch`, `fsim.training`); [section 18](sdk/mission-training.md#18-batches-and-training-m7-core); see the M7 core verification below. The independent review of `5272180` found five defects; all were corrected (`fc59959`, `0cf0150`), and M7 core is **independently accepted at `0cf0150`**. Rewards are pending (none is installed); full M7 consumes accepted extension outcomes/criteria and full M6. |
| **MA training M4** | Independent mission episodes: one World per episode, rebuilt whole on reset; generation-bound children; section 8 seed streams. | Implemented in `8254d7e`, `c507978` and `54b7fb9` as the internal native owner `fsim::env::MissionEpisode` ([contract](sdk/mission-training.md#4-ownership-generation-lifetime-and-move)), not exported; **independently accepted at `54b7fb9`**. The `mission_episode` gate applies the supplied native-state fixture to the episode's own reset - field-for-field equal to a fresh episode and an independent World of the same seed and recipe, nothing normalised, before and after replay - and covers stale, closed, busy and moved handles, disposal from inside a callback, children opened during a reset, published metadata under concurrent reads, terrain construction inputs, worker-count and threaded-schedule invariance, independence of other episodes, diagnostic names and the unchanged legacy vehicle reset. The public episode family, `ma.config.v1` parsing, snapshots and the gateway are M5 core (above). |

## SDK simulation/interface compatibility checks (2026-10-15)

The [boundary contract](architecture/design/simulation-interface-boundaries.md) defines native simulation
and Viewer interface compatibility checks accepted before the M5 export. Public C++ World and episode construction share one internal mapper;
new public/native/episode options require an explicit mapping decision. A scenario's legacy VecEnv section is
refused instead of silently ignored. No production decision system, training algorithm, public layout, permission,
physical model or Viewer presentation behavior was added or changed.

Public-header checks reject private native types even through forward declarations; exact existing World facade
seams remain compatible. Absolute state/control/transport layout guards and literal FSREC v1-v7 samples protect
old array strides. Diagnostic parity uses the actual IPC reader and the supplied fieldwise native capture across
reset, publication, recording, focus requests and reconnects. Its private-state observability limits remain those of
the fixture; mission payload/radar component checks pin strides rather than every internal member.

| Recorded verification at `08e3afe` | Result |
| --- | --- |
| UCRT64 headless build, warnings as errors | Passed |
| Focused headless gates, including three installed-consumer gates | 20/20 passed in 243.30 s |
| New simulation/interface gate | 1,314 assertions in 7 cases passed |
| Architecture negative fixtures | 51/51 passed |
| Compile-only mutation probes | Two original inputs accepted; added public/native/episode options and synchronized C/C++ state growth all refused |
| Frozen old-header C++, C99 and Python consumers on the newly installed headless DLL | 5/5 passed |
| UCRT64 Release build, warnings as errors, and focused gates | Passed; 16/16 gates in 82.01 s |

Raw logs, compiler probes and scope checks are under `D:/ENV/tmp/simulation-interface-boundary-20261015/`.
These are focused boundary results, not a fresh full-suite or fullscreen Viewer/GPU acceptance. Simulation model
fidelity, Viewer materials/shadows/performance, M5 actor isolation and full J1-J10 acceptance retain their open scope.
M4 and this bounded prerequisite are accepted. M5 core was implemented after its prerequisites were verified at `08e3afe` (below).

## MA training M5 core (2026-10-15, awaiting independent acceptance)

The public mission episode family over the accepted M4 owner: `fsim/MissionEpisode.h`, `fsim/fsim_mission_env.h`
(ABI 1.76, no change to `fsim_c.h`), `fsim.mission` and `fsim.isolation.MissionEndpointProcess` /
`RemoteMissionEndpoint` ([contract](sdk/mission-training.md)). Prerequisites were verified first at `08e3afe`; the
new regressions failed against placeholder implementations before the gateway was written. Strict `ma.config.v1` /
`ma.actions.v1` reading, one frozen snapshot per service per tick, dispatch in service-id/request-id order through each
service's own Agent, deduplication and retention, native receipts, noncombat criteria, referee documents and the
actor process boundary are implemented; every extension key and request kind is still refused (full M5 pending).

| Recorded M5 core verification (implementation committed as `6e8b4aa`) | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| Full headless suite | 616/616 passed in 972.23 s |
| Release focused gates (M5 core, M4, boundary/architecture, contract checkers, three installed-consumer gates, C ABI, `python_sdk`) | 21/21 passed in 187.87 s |
| New gates | `ma_actions` (7 cases), `ma_observation` (6 cases), `mission_c_abi`, `test_mission` (5 cases, every written document held to the schema and the checker's rules through every generic operation of the frozen fixtures) |
| `sdk_boundary` on the Release DLLs and package | 1,894 entry points, 0 errors; `boundary.md` re-rendered |
| Exports against the frozen pre-M1 baseline | 0 removed from `libfsim` (headless, Release) and `libfsim_vision` |
| Frozen baseline verify; old-header C++, C99 and three Python consumers on the new installed library | 66 artifacts unchanged; 5/5 exit 0 |
| Installed C++, C99 and Python consumers of the episode family, with an isolated actor | Passed; headers and `mission.py` byte-identical in the provenance manifest |

Implementation logs are under `D:/ENV/tmp/m5/logs/`. The handoff flagged a contract clarification: `gateway.close`'s own receipt
is `accepted` with `native` null, and `tools/ma_contract/check.py` exempts kind `gateway` from the native-answer rule
([section 12](sdk/mission-training.md#12-mareceiptsv1-acknowledgments-and-receipts)). Pending: independent M5 core
acceptance, full M5 (extension definitions and handlers), the first-package acceptance, M6-M8, and joint and Viewer
acceptance.

## MA training M5 core independent review (2026-10-15)

**Acceptance requires corrections.** Independent review of `6e8b4aa` reproduced four runtime defects using the
configured CPython 3.13 and public SDK, with each probe in a separate bounded subprocess. These findings concern
SDK reset serialization, handle lifetime and actor/referee data visibility.

| Contract | Observed result | Required correction |
| --- | --- | --- |
| Public reset serialization (`src/sdk/MissionEpisode.cpp`) | While `reset(11)` was constructing a World, `reset(22)` returned Busy, yet the first reset's snapshot reported local episode index 22. Shared minting metadata is written before the native boundary; reset also releases that boundary before reading its result key. | Serialize metadata preparation, full reset and result-key capture together. A refused call must change nothing. |
| Active Python call lifetime (`python/src/_native_mission_env.c`) | Closing an episode from another thread during a GIL-released reset produced a native access violation, exit `3221225477` (`0xC0000005`). | Retain the active native handle until reset/step returns; synchronize close and avoid reading a mutable handle after releasing the GIL. |
| Actor argument validation (`python/fsim/isolation.py::_carried`) | Actual `ma.truth.v1` and `ma.outcome.v1` JSON strings were refused, but parsed dictionaries reached an actor through constructor arguments and nested call arguments. | Check a dictionary's schema before recursively checking its members; preserve the referee-document exclusion in every supported representation. |
| Endpoint ownership (`MissionEndpointProcess` construction/rebind) | Supplying non-diagnostic episode A with diagnostic episode B's endpoint served B's snapshot. Rebinding A's process to B's same-service endpoint did likewise. | Validate endpoint episode ownership, diagnostic status and generation at construction and every rebind. |

The transport-only `gateway.close` receipt may correctly be `accepted`, closed and `native: null`; it makes no
simulation call. Its checker exception needs a narrower condition: the current checker also accepts `pending`,
`executing`, `completed`, `failed` and `canceled` gateway-close receipts without a native answer. Regression cases
must distinguish the intended accepted/closed representation from those unsupported states.

Independent evidence is under `D:/ENV/tmp/m5-independent-review-20261015/`: `review_probe.py`, `review_actor.py`,
the four probe logs, build logs and focused test logs. The initial serial Headless selection discovered 23 gates:
22 passed; `mission_package_consumer` detected that this terminology update's `isolation.py` had not yet been
staged into the Python package. Headless and Release were then rebuilt to synchronize the package. The affected
consumer passed its separate rerun (1/1, 33.65 s). All 23 selected gates therefore have passing results across those
two runs, while the four independent runtime probes still reproduce the defects above. This is not a new full-suite
or Viewer acceptance run.

Claude retains the complete M5 core correction package across implementation, language interfaces, regression
tests, installed consumers and acceptance evidence. Independent M5 core and first-package acceptance remain open;
M6-M8 and full extension/joint/Viewer acceptance retain their existing prerequisites.

## MA training M5 core correction (2026-10-15, independently accepted)

All four review findings were reproduced first with the reviewer's probes on the `6e8b4aa` build (logs
`01-repro-*`), then held by new regressions that failed before the correction (`02-red-python`, `04-red-native`):

| Finding | Correction | Regression |
| --- | --- | --- |
| Reset serialization | `env::MissionEpisode::reset(local, ResetHooks)` (additive; the one-argument reset is unchanged) runs the public facade's `prepare` and `complete` hooks under the reset's own boundary: the generation's minting inputs, the rebuild and the returned key are one operation. A reset refused busy runs neither hook; its error says busy. | `ma_actions` (C++), `mission_c_abi` (C99, a second thread), `test_mission` lifetime probe |
| Active Python call lifetime | An episode and its handles share one lifetime record in `_native_mission_env.c`: reset and step take the native handle under the GIL and count themselves without it; `close()` marks the record closed at once - every handle answers "closed" - and the last running call releases the native episode on return. | `test_mission` (`close_reset`, `close_step`, `overlap_reset`, each in a bounded subprocess) |
| Parsed referee documents | `fsim.isolation._carried` checks a mapping's `schema` before its members and checks text and bytes as what they parse to, at any depth. | `test_mission`; installed Python consumer |
| Endpoint ownership | `MissionEndpointProcess` construction and `rebind` require the episode's own endpoint (`MissionEpisode._owns`, by its native handle), the same service, its current generation, and an open handle; a diagnostic episode stays refused. | `test_mission`; installed Python consumer |
| `gateway.close` receipts | The checker accepts only `queued`, `rejected` with a gateway reason, or `accepted` and closed - each with `native` null - for kind `gateway`; other kinds keep the native-answer rule. | `ma_contract_checker` (`test_gateway_close_receipts`) |

The reviewer's four probes now: refuse parsed truth and outcome documents (`TypeError`), refuse another episode's
endpoint (`ValueError`), keep the running reset's local index 11 while the second reset is refused busy, and close
during a reset without a fault (`07-probe-*`).

| Current verification (correction source; uncommitted at run time, committed unchanged) | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| Affected headless gates (M5 core, M4, boundary/architecture, contract checkers, two installed-consumer gates, C ABI, `python_sdk`) | 19/19 passed in 167.16 s; `mission_package_consumer` rerun with the extended Python consumer 1/1 |
| Affected Release gates (the same, with `equipment_package_consumer` and `ma_engagement_checker`) | 21/21 passed in 195.75 s |
| Full headless suite | 616/616 passed in 938.32 s |
| New or extended cases | `ma_actions` 8 cases, `mission_c_abi` (busy reset on a second thread), `test_mission` 10 cases (three in bounded subprocesses), `ma_contract_checker` 28 cases |
| Exports against the frozen pre-M1 baseline | 0 removed; identical to `6e8b4aa` (the correction exports nothing new) |
| Frozen baseline verify; old-header C++, C99 and three Python consumers on the new installed library | 66 artifacts unchanged; 5/5 exit 0 |

Correction logs are under `D:/ENV/tmp/m5fix/logs/`. Pending: independent M5 core acceptance of this correction, full M5
(extension definitions and handlers), the first-package acceptance, M6-M8, and joint and Viewer acceptance.

## MA training M6 noncombat composition (2026-10-15, independently accepted)

Started from `57a846d` after the M5 core correction passed independent review. First, the installed Python consumer's
isolation refusals were made specific: with fsim.isolation's referee-data check removed, the old assertions did not
pass falsely in this build, but the referee documents reached the actor's process and the call failed only through the
actor's own argument error (`D:/ENV/tmp/m6/logs/01-weak-refusal.log`). The consumer now sends through `echo(value)`,
checks that ordinary data crosses, and requires the isolation `TypeError` with no further call reaching the actor; with
the check removed it fails as it should (`02-consumer-*`).

`examples/python/ma_composition` ([composition](sdk/ma-composition.md#7-the-example-as-implemented-m6-noncombat)):
two policy components, a BT executive and a planning component sharing one read-only snapshot per tick; a deterministic
coordinator (configured owner, priority, component id); selected requests only, through the lead service's endpoint;
SDK refusals to their originator; per-generation reset of blackboard, BT state, planner and recurrent placeholder
memory, RNG streams, queues and request/activity mappings. It uses the installed public `fsim` package alone; no SDK
interface, native code, BT or ML dependency was added.

| Current verification | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| Affected headless gates (`python_sdk`, installed-consumer gates, contract, boundary and architecture checks, M5 core and M4 gates) | 13/13 passed in 153.97 s |
| Affected Release gates (the same with `equipment_package_consumer`) | 14/14 passed in 184.06 s; `python_sdk` 245 cases |
| Full headless suite | 616/616 passed in 955.73 s |
| `test_ma_composition` | 12 cases; a mutation check plants 14 single defects in a copy of the example (two reads per tick, a snapshot per component, double ingestion, reversed tie-break, owners ignored, foreign cancel, losers submitted, feedback to all, periods ignored, activity map, BT state or RNG kept across generations, a refused request retried, truth as feedback): every one fails the tests, the unmutated copy passes (`05-mutants.log`) |
| Installed composition example (`mission_package_consumer`) | Two runs from a copy on the installed package: byte-identical traces, the reset generation equal to a fresh episode, installed package and library |

Logs are under `D:/ENV/tmp/m6/logs/`. Pending: independent M6 noncombat acceptance; full M6 composition scenarios
(they consume full M5); M7-M8; joint (J1 is shown here for noncombat only) and Viewer acceptance.

## MA training M7 core (2026-10-15, awaiting independent acceptance)

Started from `84768ea` after M6 noncombat was accepted. The failing regressions came first: `mission_training` did not
compile and `test_mission_training` failed to import (`D:/ENV/tmp/m7/logs/02-red-native.log`, `03-red-python.log`).

[Section 18](sdk/mission-training.md#18-batches-and-training-m7-core) of the contract: `fsim::MissionBatch` keeps one
public `MissionEpisode` per slot (its own World, task/judge, gateway, seed streams and local-episode count; unique
`env_id`s), steps slots over a worker pool and offers `Disabled` (an ended slot holds: nothing of it is dispatched or
stepped), `NextStep` (the next step only resets) and `SameStep` autoreset. Final snapshots stay the actor's; final
outcome, truth and the new referee document `ma.costs.v1` ([schema](sdk/ma-training.schema.json); the frozen contract
schema is unchanged) stay the referee's. `MissionEpisode::removePlatform` uses the World's own removal. Python adds
`fsim.mission.MissionBatch` and the standard-library `fsim.training` adapter (lifecycle hooks; actor, lifecycle and
critic streams; section 8 seeds). No reward is computed: none is installed. All additions are additive; no export was
removed.

| Current verification | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| `mission_training` | 8 cases, 126 assertions: slot independence and variable topology; workers 1/2/4 x `NextStep`/`SameStep`/`Disabled` with slots of different lengths give identical documents; a removed wing ends its role while teammates continue; a team end closes every role, the C2 role among them, and `Disabled` holds the ended generation; `NextStep` resets before any request of the ended generation (no receipt, activity or message carried over); `SameStep` finals distinct from the new generation; cost counts; wing snapshots byte-equal under a hidden-truth change |
| `mission_c_abi`, `test_mission_training` | The batch C handle (options, errors, flags, cross-kind refusal, finals, costs, removal); 5 Python cases (status/errors, finals, the costs schema, seed vectors, adapter hooks and streams) |
| Affected headless gates (contract, boundary and architecture checks, M4/M5/M7 gates, `c_abi`, `python_sdk`, installed consumers) | 17/17 passed in 263.77 s |
| Affected Release gates (the same with `equipment_package_consumer`) | 18/18 passed in 297.55 s; `python_sdk` 250 cases |
| Full headless suite | 617/617 passed in 1015.64 s |
| Installed consumers (`mission_package_consumer`) | C++, C99 and Python each run a two-slot `SameStep` batch from the installed packages; the Python pass drives it through `fsim.training.TrainingAdapter` |
| Boundary and exports | 1953 entry points classified, 0 errors; `libfsim.dll` 0 removed (headless and Release), `libfsim_vision.dll` unchanged |
| Compatibility baseline | `baseline.py verify`: 66 artifacts, 0 changed or missing; the baseline's old-header C++, C99 and Python consumers (`agent.py`, `links.py`, `c2.py`) pass against the current installed package |
| Fixed workload (`tools/ma_contract/mission_workload.py`, Release, 8 two-ship slots, 20-tick episodes, 60 steps, 30 World steps per tick) | Construction under 1 ms; reset of 8 slots 4.62 s with 1 worker, 1.69 s with 2, 1.49 s with 4; 46.7, 122.5 and 156.6 decision ticks/s (1,400, 3,675 and 4,697 World steps/s); the same digest for every worker count and over 24 rebuilds; private commit after each rebuild bimodal (about 32-39 MB or 74-78 MB, the allocator keeping one block or not), its maximum 76.4 MB in the first half and 77.8 MB in the second; a 48-rebuild run showed no upward trend |

Logs are under `D:/ENV/tmp/m7/logs/`; the workload's records are `D:/ENV/tmp/m7/workload-release*.json`. Pending:
independent M7 core acceptance; rewards and combat outcome reduction (no definition installed); old-generation
release and damage checks, which need combat extensions; full M7 (accepted extension outcomes/criteria and full M6);
M8; joint and Viewer acceptance.

## MA training M7 core review correction (2026-10-15, awaiting independent acceptance)

The independent review of `5272180` (`build/m7-review-5272180/review.md`) found five defects. Each was reproduced first
with the review's probes on the unchanged build (`D:/ENV/tmp/m7fix/logs/01-repro-*.log`), and the new Python
regressions failed there (`02-red-python.log`, `03-red-isolation.log`):

1. **Batch calls overlapped** (P1). Python releases the GIL for `reset`, `reset_slot` and `step`, and the batch's own
   statuses, reset counts, pending resets and final documents were outside any boundary: a second step during a
   `SameStep` rebuild returned without stepping, and readers saw torn state. Now every batch call but `size` holds the
   whole batch for its whole operation and a conflicting call fails busy at once (C++ `std::runtime_error`, C
   `FSIM_ERROR`, Python `fsim.Error`), changing and reading nothing; a step is refused whole when a slot was never reset.
2. **`ma.costs.v1` crossed to actor processes** (P2). It is now among the referee documents strict isolation refuses
   (parsed, nested, as text or bytes, in constructor or call arguments), and a batch handle is refused like an episode.
3. **Ended roles kept acting under the adapter** (P2). A role that ends while its episode goes on ticks no more and
   leaves its final snapshot once - the one it ended in; each system still gets one `begin_episode` and one
   `end_episode` per generation (a reset that cuts a generation short ends it first; `TrainingAdapter.reset_slot` resets
   a `"disabled"` slot with its hooks).
4. **The M6 system could not run under the adapter** (P2). `begin_episode` now gets an `EpisodeContext`: the former
   mapping, plus `episode_key`, `service_id`, `service_seed` and `component(id)` with section 8's seeds from
   `MissionBatch.base_seed(slot)`. `examples/python/ma_composition/batch.py` runs the unchanged reference system and
   scripted C2 under the adapter; `run.py` and the installed gate require each generation, with `"same_step"` and
   `"next_step"`, to equal the example's direct run of the same local episode.
5. **Native publication cost no bytes** (P2). `dms.publish_native` now costs each published message at its native
   encoding (`comm::encode`: one `PositionReport`, `SystemStatusReport` or `EntityReport`); refusals, identical
   retries and Data Dissemination Plan publications cost nothing.

| Current verification | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| `mission_training` | 12 cases (154 or 155 assertions: one check depends on which thread wins a race), the 8 earlier ones and: calls from two threads meeting on one batch (every whole call counted, every status read whole, refusals seen), a step refused whole while a reset holds the batch, native publication bytes equal to the delivered `byte_length` with a refusal and a retry, and dissemination outside every cost. The two concurrency cases passed 20 repeated runs |
| Python | `test_mission_training` 11 cases (the concurrency case passed 10 repeated runs), `test_mission` with costs documents refused; 24 mission cases in all; `python_sdk` 256 cases |
| Review probes on the corrected build | The status read during a running step fails busy; costs refused parsed, nested and as text; the lost wing stops at 2 ticks with one final snapshot; publication costs 88 bytes; the M6 system runs (`07-probe-*.log`) |
| Affected headless gates | 17/17 passed in 286.98 s |
| Affected Release gates (with `equipment_package_consumer`) | 18/18 passed in 318.84 s |
| Full headless suite | 617/617 passed in 1033.02 s |
| Installed consumers (`mission_package_consumer`, both presets) | The C++, C99 and Python batch passes; costs documents refused before the isolated actor, ordinary data still crossing; the installed composition example identical over two runs and after a reset, and under the training adapter with `"same_step"` and `"next_step"` (two generations each, equal to the direct runs) |
| Boundary and exports | 1960 entry points classified, 0 errors (the committed `boundary.toml`/`boundary.md` checked with the committed tool); `libfsim.dll` 0 removed, `libfsim_vision.dll` unchanged |
| Compatibility baseline | `baseline.py verify`: 66 artifacts, 0 changed or missing; the old-header C++, C99 and Python consumers pass |
| Fixed workload (Release, as above) | The same digest as before for 1, 2 and 4 workers and over 24 rebuilds; private commit bounded (maxima 76.5 and 77.7 MB by half). Today 47.3, 82.8-88.3 and 118.3-129.6 decision ticks/s with 1, 2 and 4 workers; the A/B with the uncorrected `MissionBatch.cpp` on the same machine measured the same (85.2-88.3 and 124.4-129.6), so the multi-worker figures below the first record (122.5 and 156.6) are the machine's, not the hold's |

Logs are under `D:/ENV/tmp/m7fix/logs/`. Pending: independent acceptance of M7 core with this correction; rewards and
combat outcome reduction; old-generation release and damage checks (combat extensions); full M7; M8; joint and Viewer
acceptance.

## MA training M7 core issue 1 completed (2026-10-16, independently accepted at `0cf0150`)

The review of `fc59959` (`build/m7-review-fc59959/review.md`) accepted issues 2-5 and found issue 1 partial: endpoints
and referees opened before a step take only their episode's boundary, and the batch's transition took that boundary
several times - the native step, then the outcome, the final documents and the autoreset. A call of such a handle
between them made the step fail busy after the native tick, with the slot's metadata half changed; the slot then
stayed ended and never reset, and its final outcome was missing. Reproduced first with the review's probe in both
modes (`D:/ENV/tmp/m7fix2/logs/01-repro-*.log`), and measured on the unchanged build: about one step in 40 met such a
call under an actor's thread, each time leaving the slot stuck (`03-red-rate-old.log`). There the new Python adapter
case failed in its first run (`02-red-python.log`), while the direct cases, at 8 and 10 steps then, did not meet the
window; they now run 200 steps per mode, which the unchanged build passes in fewer than 1 in 100 runs at that rate.

The correction makes each slot's transition one operation. The M4 owner gains `env::EpisodeHold`: an episode's
boundary taken once - at once or not at all - under which steps, judge reads and resets run with the episode's own
semantics and no other call enters. A batch `step` and `reset` take every slot's hold before anything changes, so a
preopened handle's call makes them fail busy whole, and theirs fail busy meanwhile. Under the hold a slot's native
step, outcome, final documents and autoreset run together, and its status and final documents are committed only
once all of it is read; a SameStep reset that fails after its episode ended leaves the slot ended with its reset
pending, which the next step performs first. The training adapter raises a busy batch step with nothing recorded and
runs it again on its next step without ticking its systems twice; what a step produced is read whole before any
stream or hook changes. No public signature, export or document changed (`MissionEpisode` names `MissionBatch` a friend:
no layout change).

| Current verification | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| `mission_episode` | The new hold case: steps, judge reads and a reset under one hold while a service's, the referee's and the range's calls fail busy; an unheld hold and a moved-from episode do nothing |
| `mission_training` | 14 cases, 162 assertions: the earlier 12 and an actor's thread reading and submitting through preopened endpoints and referees against 200 steps each of one-tick SameStep and NextStep batches - every step whole or busy with nothing changed, every ended generation's final outcome, observation, truth and costs whole and its own, NextStep's reset alone with the new generation at time 0, nothing stuck afterwards. The whole Release binary passed 8 repeated runs. (A first version of the actor could keep an endpoint without its referee when opening the referee met the batch held, and dereferenced the missing referee: one Release gate run segfaulted. The actor now opens both or neither; its first handles are opened before any step) |
| `mission_c_abi` | The same with C handles on a Windows thread (reads and submissions; first handles opened before any step), 100 steps per autoreset mode - 470 to 553 steps refused busy per mode in a run, every one with nothing changed; 5 repeated Release runs passed |
| Python | `test_mission_training` 14 cases: 200 steps per mode against an actor's thread, and the adapter under the same calls for 61 steps (each system ticked once per decision tick, one lifecycle and critic record per step, one hook pair and final snapshot per generation); 5 repeated runs passed. 27 mission cases in all; `python_sdk` 259 cases |
| Affected headless gates | 17/17 passed in 589.85 s |
| Affected Release gates (with `equipment_package_consumer`) | 18/18 passed in 614.88 s |
| Full headless suite | 617/617 passed in 1040.65 s |
| Boundary and exports | 1960 entry points classified, 0 errors, unchanged; `libfsim.dll` 1456 exports, 0 removed, none added by this correction; `libfsim_vision.dll` unchanged |
| Compatibility baseline | `baseline.py verify`: 66 artifacts, 0 changed or missing; the old-header C++, C99 and Python consumers pass |

Logs are under `D:/ENV/tmp/m7fix2/logs/`. Pending: independent acceptance of M7 core; rewards and combat outcome
reduction; old-generation release and damage checks (combat extensions); full M7; M8; joint and Viewer acceptance.

## MA training M8 generic (2026-10-16, awaiting independent acceptance)

Started from `0cf0150` after M7 core was accepted. The [validation record](sdk/ma-training-validation.md) is the
evidence manifest; in short:

- **Joint chains.** `tools/ma_contract/joint_chain.py author` ran two scenarios once through the public Python
  endpoints and recorded every submission (`tests/fixtures/mission_training/joint`): the joint chain (J2, J4, J6, J7,
  29 ticks) and the lossy chain (J3's messages on a net losing 60% of attempts, 17 ticks). Replays are deterministic.
- **`ma_joint`** (new gate, C++): 8 cases, 438 assertions, on the replays - J2 P2P perception, J3's messages and
  resubmissions, J4 typed tasks and an unauthorized role, J6 control ownership, J7 report aging, lost link and package
  change, and J8's lifecycle (a reset midway equals a fresh episode; interleaved episodes keep their own traces).
- **J10, generic.** `mission_package_consumer` now has its installed C++, C99 and Python consumers replay both chains,
  Python also with the wing's actor strictly isolated: the four traces of each chain are byte-identical, on both
  presets, and equal to a build-tree replay. `test_ma_joint` (Python) checks the same and the isolated proxy's
  refusals (range members, referee handles and documents, earlier generations).
- **Compatibility.** `tools/ma_contract/baseline.py compare` (new action) holds the current builds to the frozen
  baseline: 19 comparisons, 0 failures - no export, vtable or Python name removed; 267 C++ types, 120 C structs, 1,133
  enumerators and 50 vtables unchanged. It fails as it should on a doctored baseline.

| Current verification | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| Affected headless gates (with `ma_joint`) | 18/18 passed in 609.02 s |
| Affected Release gates (the same with `equipment_package_consumer`) | 19/19 passed in 707.93 s |
| Full headless suite | 618/618 passed in 1086.22 s |
| `python_sdk` | 261 cases |
| Baseline | `verify` 66 artifacts unchanged; old-header consumers pass; `compare` 0 failures |

Logs are under `D:/ENV/tmp/m8/logs/`. Pending: independent M8 generic acceptance; J5 and J9; the combat parts of J3,
J8 and J10 (they consume the extension handlers, outcome reduction and their fixtures, none installed); complete
Viewer acceptance (the 2026-10-12 record is a diagnostic baseline); a published seed/aspect matrix with the combat
scenes. Full M8 and the program's completion require all of them.

## MA training M8 generic review correction (2026-10-16, awaiting independent acceptance)

The independent review of `d344e2b` (`docs/architecture/modules/training/reviews/2026-10-16-m8-generic-d344e2b.md`)
found five acceptance-harness and evidence defects; no simulation-runtime regression. Each regression came first and
failed on the unchanged harness (`D:/ENV/tmp/m8fix/logs/01-red-baseline.log`, `04-red-installed-data.log`):

1. **Python contracts.** `baseline.py compare` compared only names. It now holds each frozen description to its
   contract - signatures and defaults, enumerator values, record fields in order, member kinds and signatures,
   exception bases, constants, runtime owners - and admits additions alone. 620 (headless) and 629 (Release)
   descriptions keep their contracts.
2. **Fresh, required artifacts.** Missing artifacts were skipped and a reused work directory could be read. Every
   artifact of each selected build is now required, the work directory (marked as the comparison's own) is cleared
   first, and the baseline is never written. `ma_baseline_checker` (new gate, 15 cases) holds both rules: each kind of
   contract change and a missing DLL, package or extension, an incomplete selected Release build, a reused work
   directory and the baseline as work directory fail; additions pass.
3. **Installed aircraft data.** The Python replay cleared `FSIM_AIRCRAFT_PATH`, so a development library could fly the
   source tree's F-16C: with the installed `f16c.xml` corrupted, or missing, the episode was still built. The Python
   consumer, its isolated actor and the composition example now fly the installed package's aircraft; the gate hashes
   every installed F-16C file in both packages against the source and requires the corrupted-data run to be refused
   (it is, on both presets). The missing-file run is recorded: a development library still finds its source copy
   (the resolver's intended order, unchanged).
4. **J8 preconditions.** The reset case asserted a nonempty task list while the task was already completed. A new case
   establishes an `executing` task, a message still on a 2.5 s link and a `queued` gateway request before the reset,
   then shows for 8 ticks that none of them survives and that every snapshot, truth and costs equal a fresh
   episode's; the chain case is renamed to what it shows.
5. **J6 evidence.** A new case shows C2's control designation revoking HSA in both of the lead's services and giving it
   back, and C2's speed override held by both services and binding the lead's own request (+25.3 m/s in 12 s without
   it, -12.8 m/s under it); the native binding cases of `ma_service_binding` are cited. One gap stays pending: whether
   the MA's own speed resumes after C2 cancels an override.

The manifest also states the platform/service counts exactly (five platforms, five services on four of them).

| Current verification | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| `ma_joint` | 10 cases, 696 assertions |
| Affected headless gates (with `ma_baseline_checker` and `ma_service_binding`) | 20/20 passed in 663.31 s |
| Affected Release gates (the same with `equipment_package_consumer`) | 21/21 passed in 683.43 s |
| Full headless suite | 619/619 passed in 1066.28 s |
| `baseline.py compare` (both presets) | 19 comparisons, 0 failures, every required artifact captured |
| Baseline | `verify` 66 artifacts unchanged; old-header consumers pass |

Logs are under `D:/ENV/tmp/m8fix/logs/`. Pending as before: independent M8 generic acceptance; J5, J9, the combat parts
of J3, J8 and J10; complete Viewer acceptance; a published seed/aspect matrix; J6's speed resumption after an override.

## Architecture ARCH-02 neutral contracts (2026-10-16, awaiting independent acceptance)

The second package of the architecture roadmap. The record is
`docs/architecture/validation/arch-02-neutral-contracts.md`, with its dated baseline; both are committed. No numerical,
RNG, timestep or default behavior changed, and ARCH-03 has not started.

- **Contracts:** `fsim_contracts` (`src/contracts`, header-only) holds definitions both sides of a boundary include,
  with their names, namespaces and layouts unchanged:
  - the shared-memory, camera-segment and recording layouts;
  - the stream batch (`sim::SnapshotBatch`);
  - the native construction values (`session::WorldOptions`).
- **Transport:** `ipc` links the contracts, core and platform, and no simulation module, so the `ipc -> sim`
  exception is removed.
- **Public headers:** all 41 headers are classified by owner domain and kind. The checker holds module and header
  includes to the domain rules. Three existing includes are exact exceptions: `ui` on `Capability.h` and
  `MissionSystems.h` until ARCH-06, and `VecEnv.h` on `fsim_c.h` until ARCH-07.
- **Scenario (A-03):** the runtime no longer includes the training-owned `Scenario.h`.
  - The new public `fsim/ScenarioDocument.h` holds the document values and their reading/writing; `Scenario.h`
    includes it, so the API is unchanged.
  - The runtime applies its own `session::ScenarioRecipe`.
  - `vecEnvOptions` moved to training (`src/env/ScenarioVecEnv.cpp`), as the same exported symbol.
- **Regressions, added before includes changed:**
  - the recording writer's framing and fields at literal offsets, captured from the unchanged writer;
  - a contracts-only build test (`contracts`);
  - 12 architecture fixtures.

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings; every render consumer compiled |
| ABI/Python comparison with the frozen baseline (both builds) | 19 comparisons, 0 failures; 1,456 exports, layouts, enumerators and vtables unchanged |
| Frozen old-header consumers on the new installed SDK and package | C++, C99 and 3 Python consumers pass |
| Architecture gate; `test_check.py` | Accepted (20 modules, 374 files); 122 cases passed |
| Targeted CTest | Headless 46/46 (103.99 s); Release 15/15 (94.60 s); no full suite |

Found, not changed here: the recording writer copies unspecified padding bytes into files (readers are unaffected).

**HC-00 and HC-01 corrections (2026-10-16).** These are separate corrections, from the independent ARCH-02 review and
the handoff carryover register (`docs/architecture/validation/`); the ARCH-02 record's section 9 has the details.
- **HC-00:** every file under `include/fsim` is classified and scanned whatever its suffix. An include of an
  unclassified public file, an unscanned private include and a non-text public file each fail explicitly. Six
  fixtures were added; the five negative ones fail on `6c0de98`.
- **HC-01:** the recording writer writes every record's padding as zero, with no size, offset, version or member
  change. The same world is now byte-identical. A regression with garbage-filled source structs failed on the unfixed
  writer; the frozen file is pinned by digest; old-recording reads pass.
- **ABI:** the frozen-baseline compare is 19/0 on both builds.
- **Targeted CTest:** headless 17/17, Release 6/6; no full suite.

Observed, not changed: the shared-memory publisher copies `VehicleState` padding into the segment (HC-10, transport
owner).

Pending: independent ARCH-02 acceptance; the full-suite run at the next regression milestone; ARCH-03 onward.

## Architecture ARCH-03 foundation and simulation leaves (2026-10-16, awaiting independent acceptance)

The third package of the architecture roadmap is a relocation only; the record is
`docs/architecture/validation/arch-03-foundation-and-leaves.md`, with its dated baseline. ARCH-04 has not started.

- **Moves:** `platform`, `core`, `io` and the transport (`ipc`) are in `src/foundation`. `sim`, `control`, `effects`,
  `comm` and `ms` are in `src/simulation` (`flight`, `flight_control`, `environment`, `communications/network`,
  `mission_systems`). That is 195 files, moved as renames.
- **Unchanged:** target names, aliases, namespaces, link graph and registration.
- **Includes and references:** 591 literal includes in 257 files name the destination paths. Source comments,
  maintained documents and the selector's scope patterns follow them; dated evidence keeps its historical paths.
- **Policy:** the moved modules list no legacy directories. A stale legacy directory now fails the architecture gate,
  with a fixture that fails on `565ef3e`.
- **HC-09's comment item:** the `WorldLayout.h` padding comment no longer understates the recording's framing padding.

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings |
| Fast tier: every test measured under 5 s, plus the architecture, contracts, layout, C ABI, Vision, capture, showcase, screenshot and installed-package gates | Headless 568/568 (98 s); Release 633/633 (118 s) |
| Frozen-baseline compare, both builds | 19 comparisons, 0 failures; the export list is byte-identical to the pre-move capture |
| Architecture gate; fixtures; selector tests | Accepted, the same counts and exceptions; 129; 49 |

No full suite ran; that remains open as HC-11. Register HC-11 to HC-13 record this package's newly reported items:
- HC-11: the full-suite run is still owed;
- HC-12: per-target include-root narrowing is undecided;
- HC-13: moved leaves have no targeted test scopes.

**HC-12 correction: include-path narrowing (2026-10-16, awaiting independent acceptance).** Roadmap task 5 is
implemented; section 7 of the ARCH-03 record has the details.
- **Views:** each of the ten migrated targets (`contracts` and the nine moved) exports a generated include view of its
  own headers instead of the shared `src` root. It compiles with its own headers and its declared providers' only.
- **Unchanged:** include spelling, targets, aliases, links and the installed package.
- **Consumers:** two non-migrated consumers (`flightsim` and the strike-capture test) had borrowed the root and now
  declare it themselves.
- **Fixtures:**
  - ten `include_view_<module>` compile probes: 28 allowed headers compile, and 23 headers of unrelated modules are not
    found but compile once the shared root is restored;
  - the probe checker's six cases;
  - the gate's new `broad-include-root` rule, with a fixture that fails on `30bc5ef`.
- **Evidence:**
  - both builds 0 warnings;
  - every object file byte-identical to `30bc5ef`'s (498 headless, 545 Release);
  - frozen-baseline compare 19/0, with all captures identical;
  - relevant tests 18/18 on each preset;
  - 130 checker cases and 49 selector cases.

No full suite ran (HC-11). Non-migrated targets keep the shared root until their packages move them (HC-14).

**HC-12 ancestor-root correction (2026-10-16, awaiting independent acceptance).** This closes the gap the `e7ee67e`
recheck found: the gate and the compile check both accepted an ancestor of `src`, such as the repository root.
- **Gate:** `broad-include-root` rejects `src` and every ancestor of it, resolved, so `src/.`, `src/..` and relative
  spellings are caught.
- **Compile check:** the include view check refuses any root that resolves to `src` or an ancestor of it.
- **Probes:** each forbidden header is also probed in its `src/...` spelling (46 spellings of 23 headers).
- **Regressions:**
  - the gate fixture rejects the repository root, `src/..` and `../../..`. The `e7ee67e` checker accepts all three;
  - the probe checker's 13 cases cover the repository root, `src/..`, the repository's parent, and copied `src`
    trees that only the compile probes can reject.
- **Unchanged:** no target declaration or source changed; objects are still byte-identical.
- **Tests:** related tests 16/16 on each preset; 130 checker cases and 49 selector cases. No full suite (HC-11). The
  ARCH-03 record's section 7 has the details.

## Architecture ARCH-04 simulation runtime, services and lower facade (2026-10-16, awaiting independent acceptance)

The fourth package of the architecture roadmap is a relocation and target split. The record is
`docs/architecture/validation/arch-04-simulation-runtime.md`, with its dated baseline and include environments. ARCH-05
has not started.

- **Runtime:** session moved to `src/simulation/runtime`, split by the state each file owns: `world/` (the World, its
  member files and the native scenario), `services/` (MaCore, consent, service bindings, reports, Agent rules) and
  `stores/` (store-to-flight binding). The target is still `fsim_session`.
- **Modeled DMS:** `Dms.h/.cpp` form `fsim_modeled_dms` (`src/simulation/communications/dms`), over MS RF
  propagation, beside the generic network.
- **Lower facade:** `fsim_simulation_api` (`src/simulation/api`) holds the World, Agent and recording implementations
  and the public-to-native options mapping. Training (`fsim_env`) and the SDK consume it; the SDK links it whole and
  exports its classes, each defined once.
- **`applyScenario`:** it stays in the SDK assembly, because its public header also declares training's
  `vecEnvOptions` (decision HC-15).
- **Unchanged:** targets, aliases, namespaces, registration and exports. Every first-party object is unchanged in
  code, data and symbols (287 headless, 335 Release; 21 moved).

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings |
| Frozen-baseline compare, both builds; frozen C++/C99/Python consumers on the new installed SDK | 19 comparisons, 0 failures, all captures identical; five consumers exit 0 |
| Fast tier (ARCH-03's tests) plus the new link gate and include-view probes | Headless 572/572; Release 637/637 |
| Related gates outside the tier: MA services, C2/DMS, episodes, facade, store binding, engagement | 27/27 |
| Architecture gate; fixtures; selector tests | Accepted (22 modules); 131; 49 |

No full suite ran (HC-11). Register HC-15 asks for a decision on `applyScenario`'s placement; HC-14's remaining
shared roots are now `env`, `sdk`, the visualization targets, applications, tests and tools.

## Architecture ARCH-05 training environment and mission extraction (2026-10-16, awaiting independent acceptance)

The fifth package of the architecture roadmap is a relocation and target split. The record is
`docs/architecture/validation/arch-05-training-extraction.md`, with its dated baseline and include environments.
ARCH-06 has not started.

- **`fsim_env`:** keeps its name. It is now in `src/training/native`, with the legacy VecEnv in `vecenv/` and the
  native mission episode owner in `episodes/runtime/`.
- **Mission implementation:** configuration, strict JSON, gateway, batches and the public mission facade leave the
  SDK assembly. They are the new `fsim_training_mission` (`mission/`, `batch/`, `episodes/api/`), which uses the World
  and Agent through `simulation_api`. The SDK links it whole and exports its classes, each defined once; no
  training-to-SDK-to-training cycle remains.
- **Unchanged:** every first-party object in code, data and symbols (288 headless, 336 Release; 11 moved), and
  exports, vtables, Python inventories and frozen consumers. No Python source changed.

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings |
| Frozen-baseline compare; frozen C++/C99/Python consumers on the new installed SDK | 19 comparisons, 0 failures, all captures identical; five consumers exit 0 |
| Installed M6 composition traces (`mission_package_consumer`, both presets) | Byte-identical to the pre-move traces |
| Fast tier plus the new link gate and include-view probes | Headless 575/575; Release 640/640 |
| Related training gates outside the tier, including the mission stress cases and the three preopened-handle regressions | 32/32 |
| Architecture gate; fixtures; selector tests | Accepted (23 modules); 132; 49 |

No full suite ran (HC-11).

## Architecture ARCH-06 visualization core, effects and source separation (2026-10-16, awaiting independent acceptance)

The sixth package of the architecture roadmap. The record is `docs/architecture/validation/arch-06-visualization.md`,
with its dated baseline and include environments. ARCH-07 has not started.

- **Visualization:** now in `src/visualization`:
  - `render`;
  - `scene` (`fsim_world`), with the visual effects in `vfx/effects`;
  - the optional Flow solver in `vfx/flow`;
  - `ui`;
  - the new `vision_core`, which renders read-only vehicle frames offscreen.
- **Read-only inputs:** the scene and UI read contracts. The family-code range and the constraint bits the UI names are
  now stated on the wire contract, and the producer asserts them. The `world -> sim` and `ui -> sim` exceptions and
  the UI's two public-header exceptions are removed.
- **Vision adapter:** `fsim_vision.dll` is `src/adapters/vision`, which reads the World into frames.
- **Viewer and demo:** the Viewer is `src/applications/viewer`. The demo's simulation is the demo application's
  (`fsim_demo_publisher`), which the Viewer executable composes for `--demo` through the exact, temporary exception
  `viewer -> demo` (until ARCH-08; HC-16).

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings |
| Object identity against `58f9297` | All unchanged except the restructured `Vision.cpp` and Viewer `main.cpp` (and two new files) |
| Vision cameras' images | Byte-identical to the pre-change run |
| Live native strike, fullscreen 3840 x 2160, before and after | Both pass with the same native outcome and capture times; the replay and the demo render within run-to-run noise |
| Frozen-baseline compare; frozen C++/C99/Python consumers | 19/0, all 22 captures identical (Vision DLL included); five consumers exit 0 |
| Fast tier plus the new probes; Flow calibration | Headless 576/576; Release 646/646; 1/1 |
| Architecture gate; fixtures; selector tests | Accepted (25 modules); 133; 49 |

No full suite ran (HC-11), and the 4K performance gate was not remeasured; the Viewer validation record's dated limits
stand.

## HC-07 mouse, replay and resize (2026-10-17, awaiting independent acceptance)

HC-07's remaining interaction cases were verified through input posted to the Viewer's own window, on the current
Release build at 3840 x 2160. The [HC-07 record](architecture/validation/hc-07-viewer-and-joint.md#mouse-replay-and-resize-2026-10-17) has the details.

| Case | Result |
| --- | --- |
| Mouse: rotate, the right button, the wheel, space + drag (locked and free), the distance limits, the ground | Passed: 20 actions, the pause flag after each, frame differences, and panel readings against what each must do |
| Wheel with a detached camera | Zooms towards the pointer, measured by registering the frames |
| Replay: space, `.`, `[`, `]`, home, the timeline slider, the pause box, the step button, the loop at the end | Passed, from panel readings; steps exact from the first frame |
| Window resize, minimise and restore, maximise and restore, 4:3 | Passed: every frame its client area, none stretched or blank, the demo still simulating |

HC-51 is new and nonblocking: the Viewer guide's mouse description differs from the Viewer's wheel, space + drag
and nearest distance. HC-07 stays open:
- HC-45's performance gate (optimization deferred);
- HC-06's combat cases;
- HC-02's J6;
- a human-operated session.

No full suite or performance scene ran.
The documentation checks pass: the planning checker (58 documents), the local-link audit (1,446 links in 113
files, none broken) and `ma_contract`, `architecture_boundary` and `sdk_guard_checker` (3/3).

## HC-47 helper checks, version 2 (2026-10-17, awaiting independent acceptance)

The [recheck](architecture/validation/hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b) found
eight invalid records that HC-47's helper checks still accepted. Version 2 of the helpers, in
`D:/ENV/tmp/hc47b/helpers/`, keeps the runs and corrects the checks:
- **Measurements:** each must be present, not null and correctly typed, including both segment results, the publisher
  lists and both phases' incarnations. A probe that cannot tell now gives null.
- **Order:** the reconnect's log must attach again before any of the replacement's frames.

| Check (pure: no Viewer, no process) | Result |
| --- | --- |
| The review's eight records | Version 1 accepts all eight; version 2 rejects all eight |
| Self-tests | Demo: 47 negatives, each rejected for its own reason, and the valid control passes. Reconnect: 54 and 1 |
| Valid controls | Both saved run outcomes pass version 2. The order check passes the three real reconnect logs, and rejects each once its reattach is moved last |
| Probes | On captured `tasklist` output and under stubs, version 1 reads a failed query as no process and an access-denied segment as absent; version 2 reads both as null. A missing segment reads as absent |
| All | 149 results, all as expected |

Version 2's run path has not run against a Viewer, since no GUI run was in scope. The saved outcomes were measured with
version 1's probes. No full suite or performance scene ran.
The documentation checks pass: the planning checker (58 documents), the local-link audit (1,434 links in 113
files, none broken) and `ma_contract`, `architecture_boundary` and `sdk_guard_checker` (3/3).
The [HC-07 record](architecture/validation/hc-07-viewer-and-joint.md#version-2-checks-2026-10-17) and its
[summary](architecture/validation/hc-07-evidence.json) have the details.

## HC-47 to HC-50 corrections (2026-10-17, awaiting independent acceptance)

The findings of the [HC-07](architecture/validation/hc-07-review-2026-10-17.md) and
[HC-45](architecture/validation/hc-45-review-2026-10-17.md) reviews are corrected. HC-45 stays open, its optimization
deferred; no performance scene was rerun.

| Finding | Correction | Check |
| --- | --- | --- |
| HC-47: the HC-07 summary gave the demo's exit and cleanup, and the staged data's match, as constants; the reconnect helper asserted nothing | Fail-closed helpers measure and assert the demo's exit, publisher, segment cleanup and frame, and the reconnect's published identities, log order, frames and cleanup. The summary reads saved outcomes; the original demo run's unrecorded exit and cleanup are now null | 14 and 13 negative controls each fail, as they must; one run of each passes on the current Release Viewer; the staged data is still HC-11's |
| HC-48: an unqualified "no regression" | Narrowed: the same three scenes fail before the migration; the current Viewer's strike minima are 1.4-1.5 fps lower; neither a regression nor equivalence is established | Documentation |
| HC-49: a GPU gap bridged a frame that was not read | `GpuTimer` measures a gap only between frames read one after the other, reports `n/a` otherwise and counts the unread frames | The review's probe gives `n/a` (was 7 ms); five new `[gpu-timer]` cases; related tests 74/74 Release, 6/6 headless |
| HC-50: separately selected frame statistics joined, and hypotheses stated as causes | The slow-frame table separates its four selections and labels the leads as hypotheses; the Viewer guide says what a gap can and cannot show | Documentation |

`tests/CMakeLists.txt` changed, so the selector plans both presets' full suites; as directed, they did not run and stay
pending. The Release inventory is now 785 tests. Four staged Release artifacts (the Viewer, the Vision DLL twice and
`_vision.pyd`) differ from the HC-11-tested ones; HC-11 stays accepted for its frozen source.
The documentation checks pass: the planning checker (58 documents), the local-link audit (1,418 links in 113
files, none broken) and `ma_contract`, `architecture_boundary` and `sdk_guard_checker` (3/3).
The [HC-45 record](architecture/validation/hc-45-performance-comparison.md#5-the-reviews-corrections-2026-10-17) and the
[HC-07 record](architecture/validation/hc-07-viewer-and-joint.md#hc-47-correction-2026-10-17) have the details.

## HC-45 performance comparison (2026-10-17, awaiting independent acceptance)

The [HC-45 record](architecture/validation/hc-45-performance-comparison.md) has the details. HC-45 stays open: by the
user's decision, no performance change was made.

| Check | Result |
| --- | --- |
| Idle host | CPU 1-6 %, GPU 0-1 % before each run |
| Current Viewer against `7948dc4`'s, from before the migration, on the same scenes, publishers, recordings, assets and settings (A B A B) | Both fail fleet (two ~17.7 ms frames), strike (lowest second 144-147 fps at the impact) and hit (5-12 frames over 12.5 ms), and both pass reheat. Two rounds establish neither a regression nor equivalence: the current Viewer's strike minima are 1.4-1.5 fps lower (HC-48) |
| New `--stats` line `gpu worst` (each second's slowest GPU frame and the GPU's gaps); the related Release tests | 68/68; after the HC-49 fix, 74/74, and 6/6 headless |

Hypotheses, not demonstrated causes (HC-50): GPU work or presentation pacing in fleet's long gaps (tile uploads, for
one), a full cloud march in hit, NVIDIA Flow's solver sharing the GPU in the fire's aftermath, and the fire's shading
and breakup pieces within the strike's pass groups at the impact.

## HC-07 Viewer and joint verification (2026-10-17, awaiting independent acceptance)

The [HC-07 record](architecture/validation/hc-07-viewer-and-joint.md) has the details. HC-07 stays open.

| Case | Result |
| --- | --- |
| Live fullscreen strike, replay frames, demo, reconnection to a replaced world | Passed at 3840 x 2160 on the HC-11-tested binaries |
| Demo pause, step, time factor and quit through the window's key input | 15/15 |
| 4K performance gate (160 fps), two runs | **Failed** for fleet, strike and hit; reheat passed (HC-45) |
| Combat joint cases, combat Viewer run, seed/aspect matrix | **Not run:** their prerequisites are not supplied (HC-06) |

## HC-11 final regression (2026-10-17, awaiting independent acceptance)

Both required full suites ran once on one frozen, fingerprinted source snapshot, and both passed. The
[HC-11 record](architecture/validation/hc-11-final-regression.md) has the details.

| Verification | Result |
| --- | --- |
| Preconditions | Review findings closed (HC-38 to HC-43, HC-20); related fast checks 29/29 headless and 35/35 Release |
| Snapshot | HEAD `f1bb879` plus 58 differing paths, digest `e26241d67a879d4e`; both build trees fingerprinted |
| `ucrt64-headless` full suite | 717/717 passed, 1,123 s |
| `ucrt64-release` full suite | 780/780 passed, 1,138 s; the renderer tests ran |
| Snapshot after each run | Unchanged |

Pending (HC-07): the fullscreen Viewer scenarios, manual controls, the 4K performance gate and joint acceptance.

## ARCH-10 HC-38 to HC-43 correction (2026-10-17, awaiting independent acceptance)

Corrects the six items of the ARCH-10 review; section 11 of the
[ARCH-10 record](architecture/validation/arch-10-final-acceptance.md) has the details.

- **Configured-graph gate:**
  - SYSTEM includes no longer bypass the ownership checks (HC-38);
  - the reply must name the requested build, and a disabled branch's modules must be absent (HC-42);
  - it needs CMake 4.2 or later (HC-43, the user's decision). Native builds keep CMake 3.25, and with an older CMake
    the gate fails with an actionable diagnostic, leaving its acceptance pending.
- **Test selection:**
  - every CMake file both configures read plans and escalates both presets at any depth, while render-only CMake
    files stay on Release (HC-39, completing HC-20);
  - the architecture gates are required on each preset (HC-40).
- **Provenance:** a verifiable source manifest against a stated base replaces the stale one (HC-41). Both inventories
  hold 67 Python SDK tests.

| Verification | Result |
| --- | --- |
| Related tests, after a build of both presets | Headless 29/29, Release 35/35 |
| Checker suites | Configured 37, selector 101, architecture 144 |

No full suite or GUI scenario ran.

## Architecture ARCH-10 final three-domain acceptance (2026-10-17, awaiting independent acceptance)

The tenth and last package of the architecture roadmap. The
[ARCH-10 record](architecture/validation/arch-10-final-acceptance.md) has the details. It reports migration progress: the
full suites (HC-11) and the Viewer and joint gates (HC-07) are pending, so the three-domain architecture is not yet
declared complete.

- **Configured build graph:** each preset's own CMake File API code model is now held to the source graph
  (`architecture_configured`; its checker's cases, controls included, are now 37). `ucrt64-headless` configures 20 modules and 67 links, the
  renderer modules absent with the renderer off; `ucrt64-release` configures all 28 modules and all 97 source edges.
- **No exception left:** the policy lists no module, public, Python or layout exception. The launchers'
  implementation is the declared `tools/launch` category (HC-25).
- **Test selection:** an escalation applies to the presets its own path plans (HC-19). Shared build inputs plan and
  escalate both presets (HC-20). The package root's eager imports stay a dependency, by decision (HC-04).
- **Planning graph:** it uses the policy's module names and directories, and its checker enforces them (HC-24).
- **Records:** the final baseline and the provenance (toolchain, GPU, inventories, artifact and data hashes) are in
  `docs/architecture/validation/`.

| Verification | Result |
| --- | --- |
| Source, configured (both presets), Python and planning graphs | No unknown owner, forbidden edge or cycle |
| Related tests (architecture, documentation, selector) | Headless 29/29, Release 35/35 |
| Compatibility and installed consumers (C ABI, SDK boundary, packages, launchers) | Headless 7/7, Release 7/7 |

The full suites (717 headless and 780 Release tests) and the GUI scenarios did not run.

## HC-33 authority registration (2026-10-17, awaiting independent acceptance)

Completes ARCH-09's authority gate, which the independent review at `4d7a9be` left open (HC-33). The
[ARCH-09 record](architecture/validation/arch-09-architecture-authority.md#10-hc-33-correction-2026-10-17) has the
details. ARCH-10 has not started.

- **Registered in place:** three architecture documents outside `docs/architecture` now have a register row and a
  direct entry in the architecture index, each with a category, owner and status:
  - ADR-26, the control architecture: current authority, canonical at its path;
  - ADR-27, rotorcraft: a retained domain reference;
  - the 2026-10-11 build/run and installed-SDK packaging plan: historical evidence.
- **Bodies:** none moved or changed.

| Verification | Result |
| --- | --- |
| Planning checker; local Markdown links in the repository | Accepted (50 documents, 25 register rows); 1,303 links, none broken |
| Related documentation tests | Headless 3/3 |

No native build or full suite ran (HC-11).

## Architecture ARCH-09 architecture authority relocated (2026-10-17, awaiting independent acceptance)

The ninth package of the architecture roadmap. The record is
`docs/architecture/validation/arch-09-architecture-authority.md`. ARCH-10 has not started.

- **Relocated:** the registered architecture documents moved once into `docs/architecture`, with their history:
  - the platform baseline (formerly `docs/FlightSim_System_Architecture_and_Design.md`), the agent boundary and the
    simulation interface boundaries into `design/`;
  - ADR-30 mission systems, the Vehicle Interface, flight autonomy and the native engagement design into
    `modules/simulation/`;
  - the A1-A6 maintenance designs and plans into `migration/history/`.
- **Redirects:** the old paths of the first five keep a redirect, because code comments and generated data still name
  them. Every Markdown link follows the move.
- **Deferred:** the MA training design, plan and allocation stay in place until the M8 handoff.
- **Index:** every registered document has a category, owner and status in the architecture index.

| Verification | Result |
| --- | --- |
| Local Markdown links in the repository; planning checker | 1,278 links, none broken; 49 documents reachable from the index |
| Related tests (documentation, SDK principals, MA contract) | Headless 6/6, Release 6/6 |

No full suite ran (HC-11); no build input changed.

## HC-32 timer-handle correction (2026-10-17, awaiting independent acceptance)

Corrects the leak the ARCH-08 recheck at `1dffece` found; the ARCH-08 record's section 17 has the details.

- **HC-32:** `platform::sleepUntil()`'s per-thread Windows timer is now owned by a thread-local object that closes it as
  its thread ends. A thread that slept no longer leaves a handle behind, including each simulation thread the demo
  publisher starts at a hold. The timer's behavior is unchanged.

| Verification | Result |
| --- | --- |
| New regressions on the unchanged code; after the fix | 30 threads that slept: 115 to 145 handles, then steady. 30 real holds and resumes: 122 to 152, then 116 to 116 |
| Related tests | Release 12/12, with the demo publisher's parity, steps, lifecycle and schedules; headless gates 5/5; 5/5 repeats |

No full suite ran (HC-11).

## HC-28, HC-30 and HC-31 demo publisher correction (2026-10-17, awaiting independent acceptance)

Corrects the items the ARCH-08 recheck at `ecf58a3` left open; the ARCH-08 record's section 15 has the details.

- **HC-28:** each hold joins and restarts the runner's thread, so a step it was taking is published before any request
  is handed over. Every later step is a requested one; no timing is involved.
- **HC-30:** requests made while the demo runs, or pending through a resume, are taken at the next hold, one step each.
  The control block acknowledges the requests taken (`stepsTaken`, version 2).
- **HC-31:** the `demo publisher` selector scope requires its publisher tests on Release, and its schedules, probe and
  contract tests on both presets. A plan on an inventory without them fails before any test runs.

| Verification | Result |
| --- | --- |
| Schedules against a model of the runner, including the review's delayed-work case and 2,000 random schedules; mutations | Every request is one requested step; removing the join reproduces the review's failure |
| Real runner in-process; process tests on Release | Exact step counts after each hold; the publisher cases pass 5/5 when repeated |
| Selector | New cases fail on HEAD's selector (22 sub-tests), then all 82 pass; targeted plan: headless 9/9, Release 72/72 |

No full suite ran (HC-11).

## HC-28 and HC-29 demo publisher correction (2026-10-17, awaiting independent acceptance)

Corrects the two defects of the ARCH-08 review; the ARCH-08 record's section 13 has the details.

- **HC-28:** the demo publisher takes every single-step request, one at a time while the simulation is held: 20
  requests made at once are 20 steps, the in-process demo's to the bit. A resume drops the requests not yet taken.
- **HC-29:** the Viewer's demo link closes the publisher's process handle whatever the child's state, including after a
  failed start.

| Verification | Result |
| --- | --- |
| New regressions on the unchanged code; after the fix | `demo_publisher_steps` and `demo_publisher_lifecycle` fail (1 step for 20 requests; 3 leaked handles); then pass, 5/5 each when repeated |
| Selector's targeted plan | Headless 8/8; Release 71/71, every renderer test included |

No full suite ran (HC-11).

## Architecture ARCH-08 applications, tests, examples and packaging (2026-10-17, awaiting independent acceptance)

The eighth package of the architecture roadmap. The record is
`docs/architecture/validation/arch-08-applications-tests-and-packaging.md`, with its dated baseline and include
environments. ARCH-09 has not started.

- **Applications:** `src/applications` holds the headless runner, the Viewer, the demo and the Python testbench
  (moved from `testbench`).
  - `flightsim-viewer --demo` launches the demo publisher `flightsim-demo`, a process of its own. It attaches to its
    world read-only and drives it through the demo control block (HC-16).
  - The Viewer takes the control levels' names from the world-stream contract (HC-17.2).
  - No shared `src` include root remains (HC-14).
- **Tests, examples and tools:** 609 files moved into categories:
  - tests: contracts, simulation, training, visualization, compatibility and integration;
  - examples: simulation, training and visualization;
  - tools: models, data and validation, beside architecture and launch.

  CTest names, executables, launcher commands and package layouts are unchanged. The architecture gate holds the
  categories, with `tools/launch` as its one exception (HC-25).

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings |
| Object identity against ARCH-07 | Every library and application object unchanged but the Viewer's entry point. Moved tests differ only by their embedded path, or by reviewed edits |
| Frozen-baseline compare; Python identity | 19/0, exports and layouts byte-identical; 494 and 501 names, 0 differ |
| Frozen consumers on the installed SDK and package; installed provenance | 5 and 5 exit 0; no installed file names the checkout |
| Demo | 240/240 steps identical to the in-process demo; the GUI demo runs on the publisher and leaves no process |
| Related fast tests | Headless 624/627, then the 3 fixed 3/3; Release 688/688 |
| Architecture gate; fixtures; planning checker | Accepted (layout: 598 files in 13 categories); 141; accepted |

No full suite ran (HC-11).

## HC-21 P056 and HC-23 (2026-10-17, awaiting independent acceptance)

Completes ARCH-07's P056 and resolves HC-23; the ARCH-07 record's section 13 has the details.

- **P056:** `fsim.isolation`'s codec is `fsim._adapters.codec`, and its actor-side proxies with the actor process's loop
  are `fsim._adapters.actor`. Neither imports a public module. `fsim.isolation` keeps its Range-side hosts and checks,
  and its public names keep it as their owner. The other owner's uncommitted edits to the module stay uncommitted and
  in place.
- **HC-23:** the native `training_vision` reservation waits for VIS-F and stays planned. The gate accepts a module
  plan's package as a planned phase.

| Verification | Result |
| --- | --- |
| Gate; fixtures | Accepted, no Python cycle; 135 |
| Python identity, both presets | 0 differ |
| Frozen-baseline compare; frozen consumers on the installed package | 19/0; 5/5 |
| Related fast tests | Headless 22/22, then 26/26; Release 6/6 |

No full suite ran (HC-11).

## Architecture ARCH-07 SDK assembly and Python import cleanup (2026-10-16, awaiting independent acceptance)

The seventh package of the architecture roadmap. The record is
`docs/architecture/validation/arch-07-sdk-and-python.md`, with its dated baseline and include environments. ARCH-08
has not started.

- **SDK assembly:** `src/adapters/sdk`:
  - `cpp/` holds the C++ compatibility facade: the VecEnv pimpl and `applyScenario`'s delegation (HC-15);
  - `c/` holds the C ABI adapters and their private headers. `fsim.cpp` was split between the two.

  The assembly compiles with its own headers and its linked modules' views, no longer the `src` root.
- **Extension modules:** `fsim._native` is in `src/adapters/sdk/python` and `fsim._vision` in
  `src/adapters/vision/python`. Their shared `fsim_py.h` and stable-ABI settings are `fsim_python_abi`, and CPython is
  the imported target `cpython::abi3`.
- **Python:** the implementation behind the public modules is in `fsim._training`, `fsim._simulation` (shared records,
  agent and referee factories), `fsim._visualization` (cameras) and `fsim._contracts`. Public names keep their owners.
  Both deferred import cycles are removed, and the camera core has no VecEnv: `fsim.vision` composes `VisionBatch`.
- **Public headers:** `fsim/fsim_vecenv_handle.h` holds the C handle `VecEnv.h` shares with the C ABI. The last public
  exception is removed; `VecEnv.h` no longer includes `fsim_c.h` (HC-22).
- **Not done:** P056 (`fsim.isolation`'s codec and proxy): the module carries another owner's uncommitted edits (HC-21).

| Verification | Result |
| --- | --- |
| Builds (headless, Release), warnings as errors | 0 warnings |
| Object identity against `74efb49` | All unchanged (20 and 21 moved), except the split `fsim.cpp`. Function by function it differs only on three throw paths, which call `fsim_last_error()` out of line |
| Frozen-baseline compare; exports and layouts against the pre-change capture | 19/0; byte-identical |
| Python public identity, both presets | 494 and 501 names, 0 differ |
| Frozen C++/C99/Python consumers on the installed SDK, and on the installed package | 5 and 5 exit 0 |
| Related fast tests | Headless 621/622 with the corrected SDK probe, then 5/5 rerun; Release 682/682 |
| Architecture gate; fixtures; selector tests; planning checker | Accepted (28 modules, no Python cycle, no public exception); 134; 74; accepted |

No full suite ran (HC-11).

## HC-13 shared renderer scope (2026-10-16, awaiting independent acceptance)

The `5f8c139` recheck's last P2: the shared `render, vision and viewer` scope (public Vision header, Python wrapper,
assets) asked for the `render` label on Release but did not require it, so generic tests passed an inventory without
renderer tests. The ARCH-06 record's section 12 has the details.

- **Requirement:** any `release` scope selecting the `render` label (render-only or shared), or the compile-database
  evidence, makes the Release plan require and select every renderer test; generic Release scopes are unchanged.
- **Headless:** `render_only` still alone decides routing; the shared paths keep their headless selection.
- **Regressions:** shader, header and wrapper negatives over tiers, mixed escalation and run lifecycles; valid,
  post-build and generic-scope controls; the recheck's probe on the real headless inventory.

| Verification | Result |
| --- | --- |
| The recheck's probe, real headless inventory as Release | HEAD: ten plans exit 0, two runs start their runners; corrected: all twelve exit 3, nothing run |
| Real Release control | 69 of 769 targeted with all 59 render tests; headless 10 |
| `testing_scope_checker`, both presets | 74/74 and 74/74 |

No product source changed and no full suite ran (HC-11).

## HC-13 renderer-coverage requirement (2026-10-16, awaiting independent acceptance)

The `9e66811` recheck's remaining P2: a renderer change's `ucrt64-release` plan passed on an inventory without renderer
tests, through the render scope's generic names. The ARCH-06 record's section 11 has the details.

- **Requirement:** a path only the renderer builds makes the Release plan select every `render`-labelled test; an
  inventory without one is a `no-renderer-coverage` error in every tier, for rules and compile-database evidence alike.
- **Lifecycle:** checked on the inventory listed after any build, before any runner starts (exit 3).
- **Regressions:** a realistic negative fixture (`sdk_boundary` and the Python Vision tests, no renderer test) over
  tiers, mixed escalation, evidence, plan-only and run lifecycles; controls for the post-build refresh and a valid
  inventory; the review's probe on the real headless inventory.

| Verification | Result |
| --- | --- |
| The review's probe, real headless inventory as Release | HEAD exit 0 (runs start); corrected exit 3 in all six cases, nothing run |
| Real Release control | 64 of 769 targeted, all 59 render tests |
| `testing_scope_checker`, both presets | 69/69 and 69/69 |

No product source changed and no full suite ran (HC-11).

## HC-13 Release selection for renderer changes (2026-10-16, awaiting independent acceptance)

The ARCH-06 review's P2: a scene, UI or Flow change was planned on `ucrt64-headless` alone, which does not compile it,
so its selection passed without the change being built or tested. The correction is in the test selector; the ARCH-06
record's section 10 has the details.

- **Render-only roots:** the `render-only roots` scope claims everything only the renderer builds, nested CMake files
  included. A change there plans `ucrt64-release` alone and selects the `render`-labelled tests, which are all 59 tests
  only the renderer builds, plus `sdk_boundary` and the Python Vision tests.
- **Compile-database evidence:** independently, any path the presets' compile databases show is compiled only in
  `ucrt64-release` plans that preset.
- **Fail-closed:** a missing Release inventory or a failed Release build is an error, not a headless pass.
- **Regressions:** pure plans, lifecycle and real-build consistency cases (`testing_scope_checker`, 62 cases). On the
  review's probe, HEAD's selector passes UI, scene and Flow changes on headless alone; the corrected one plans them on
  Release.

| Verification | Result |
| --- | --- |
| `testing_scope_checker`, both presets | 62/62 and 62/62 |
| Real inventories, planned only: UI, scene, Flow, nested CMake file, render backend | `ucrt64-release` alone, 64 of 769 tests |
| `render`-labelled tests on `ucrt64-release` | 59/59 |

No product source changed and no full suite ran (HC-11).

## HC-10 shared-memory padding (2026-10-16, awaiting independent acceptance)

The carryover register's HC-10 is verified and corrected; section 10 of the ARCH-02 record
(`docs/architecture/validation/arch-02-neutral-contracts.md`) has the details.
- **Measured:** a generated probe compiled every member's offset and size for each record the world publisher writes.
  Only `VehicleState` (bytes 391 and 604-607) and `MissionRangeDoppler` (bytes 52-55) have padding.
- **Confirmed:** a raw shared-memory regression publishes records with poisoned padding and maps the segment as a
  viewer does. On the unfixed publisher all 14 padding bytes failed: the caller's padding reached local readers.
- **Corrected:** the publisher zeroes those bytes inside its seqlock window. One shared padding definition in the layout
  contract serves both the publisher and the recording writer. Layouts, member values, publication semantics,
  recording bytes (frozen digest unchanged) and exports are unchanged.
- **Verification:** both builds with 0 warnings; targeted CTest headless 21/21 and Release 9/9; frozen-baseline
  compare 19/0. No full suite.

## Architecture ARCH-01 ownership and policy baseline (2026-10-16, awaiting independent acceptance)

The first package of the architecture roadmap (`docs/architecture/migration/roadmap.md`, in the untracked planning
package) enforces source ownership and the
three-domain dependency policy on the current tree. No domain source moved. The
[ARCH-01 record](architecture/validation/arch-01-ownership-baseline.md) holds the details and the dated
[baseline](architecture/validation/arch-01-baseline.json) taken at `7948dc4`. The architecture planning package
under `docs/architecture` is otherwise untracked: only its ARCH-01 record and baseline are committed here.

- **Policy v2** (`tools/architecture/policy.json`): each native module has a domain, a kind (contract, implementation,
  final assembly, application) and its future destinations; 11 planned owners reserve theirs. The three current
  reverse edges are exact exceptions naming the package that removes them: `ipc -> sim` (ARCH-02), `world -> sim` and
  `ui -> sim` (ARCH-06).
- **Checker**: direction rules on links and includes; longest-prefix ownership with `duplicate-owner`,
  `planned-owner` and domain-root unowned files; `duplicate-source`; `unused-allowance`; stale exceptions; strict
  policy validation. `python_imports.py` checks the Python package: cycles with deferred edges (the two current ones
  are ARCH-07 exceptions), facade backedges from private packages, optional eager imports, unregistered dynamic
  imports and undeclared subpackages.
- **Baseline** (`tools/architecture/baseline.py`): the actual native link, include and external graphs, owners and
  hashes of 327 sources and 40 public headers, the Python imports, and the declared package outputs.
- **Configured-build reconciliation**: the CMake File API procedure for after the moves is defined in
  `tools/architecture/README.md`; nothing was configured.

| Verification | Result |
| --- | --- |
| `check.py` on the repository | Accepted; 19 modules, 367 files; 3 listed exceptions in use; Python 18 modules, 0 eager cycles |
| `test_check.py` | 90 cases passed (51 existing native, 37 new domain/ownership/Python, 2 baseline) |
| Planning document checker | Accepted; 12 documents, links valid |
| Targeted CTest tier (`scope.py --run`) | 5/5 passed in 11.59 s; no build, no full suite |

**Review correction (2026-10-16).** The independent review of `9b6b645`
(`docs/architecture/validation/arch-01-review-2026-10-16.md`) found five P2 issues. All are corrected, each with a
focused regression that fails on `9b6b645`:
1. Exact file claims may no longer override a reserved or foreign destination (`file-claim` input error).
2. Imported and aliased import functions are dynamic-import sites; other uses are unsupported forms.
3. Package-initializer edges join the eager closure, without inventing facade cycles.
4. A legacy Python exception must close its own cycle.
5. The baseline records every declared extension input and link: the helper-added `src/fsim_py.h` and the
   unresolved `${FSIM_PYTHON3_DLL}`, with hashes and branches.

The baseline was refreshed at `9b6b645`; source hashes are unchanged.

| Correction verification | Result |
| --- | --- |
| `test_check.py` | 102 cases passed (12 added) |
| `check.py` on the repository | Accepted; the same 3 exceptions and 2 legacy Python cycles |
| Targeted CTest tier (`scope.py --run`) | 5/5 passed in 13.68 s; no build, no full suite |

**Recheck correction (2026-10-16).** The recheck at `9e0ecc2` closed findings 3-5 and left two gaps; both are
closed now. Each focused regression fails on `9e0ecc2`.
- Path identity: ownership compares case-folded paths, so `src/Training/...` meets the reserved, foreign and
  domain-root rules. Only the policy's own spelling passes (`noncanonical-path`, input errors). The legacy shared
  claims are kept.
- Wildcard imports: a wildcard import from `importlib` or `builtins` is an unsupported dynamic-import form, refused
  even at a registered site.

The baseline was refreshed at `9e0ecc2`; only its HEAD and working-tree list changed.

| Recheck verification | Result |
| --- | --- |
| `test_check.py` | 110 cases passed (8 added; the 6 negative ones fail on `9e0ecc2`) |
| `check.py` on the repository; planning document checker | Accepted; accepted (13 documents) |
| Targeted CTest tier (`scope.py --run`) | 5/5 passed in 14.97 s; no build, no full suite |

Pending: independent ARCH-01 acceptance (ARCH-02 not started); ARCH-02 onward; configured File API reconciliation once moves begin.

## MA training M8 correction and change-scoped testing (2026-10-16, awaiting independent review)

The independent review of `3802e7a` and the uncommitted test framework
(`docs/architecture/modules/training/reviews/2026-10-16-m8-3802e7a-test-scoping.md`) left two M8 items partial and
four selector findings open. Each correction has its own pure regression first:

1. **Python contracts.** A record may no longer gain fields - a record is unpacked by position, so a new shape is a new
   record type - and an added parameter's kind and default are read outside quotes and brackets: `(a, b=1, *, c: '=')`
   over `(a, b=1)` is refused, and so is an added `/`. A check on real functions confirms each admitted addition takes
   every old call and each refused one breaks one. All 2,326 frozen signatures classify; no frozen record changed.
   `ma_baseline_checker` has 17 cases.
2. **Installed data.** The expected runtime inventory is what the staging rules give for the flown F-16C and the
   stores (113 files), each required in both packages by hash. The Python consumer checks it before making any
   World, in its normal run and in its probes: a missing `f16c.xml`, a missing `Engines/f16c_engine0.xml` and a
   corrupted `f16c.xml` are refused (exit 4), each naming the installed path; with the presence check alone, the
   corrupted file is refused by the SDK (exit 3). The development resolver is unchanged.
3. **Selector, fail closed (T1).** An unclaimed path, an empty selection and no changes exit 3 with their reasons and
   paths (`--json`). A scope may add tests without claiming paths (`claims = false`), so an unimported `python/fsim`
   module stays unclaimed.
4. **Dependency closure (T2).** Every `python_sdk` test whose module imports a changed file - through modules, shared
   test fixtures and the scripts it runs - is selected, stress cases included. Test sources map to targets through
   `compile_commands.json`, including local headers and CMake-driven executables. Data files map through the sources
   that name them. Legacy `src/env` code escalates to the full suite; the mission episode owner keeps its own scope.
5. **Inventory lifecycle (T3).** The Python test glob is `CONFIGURE_DEPENDS`. With `--build`, `scope.py` builds first,
   lists each inventory afresh and resolves names to `-I` numbers just before running, then requires that exactly the
   selected tests ran (exit 4). Mocked lifecycles and a tiny independent CMake project show a module added after
   configuring being registered and selected by its new number.
6. **Excluded directories (T4).** Source discovery prunes `.codex` (and `.git`, `__pycache__`) before descending; a
   recording test shows no excluded directory is entered.

Measurement: a test's passing duration is kept apart from a failed or timed-out run's. The estimate is a labelled
lower bound: `RUN_SERIAL` tests are added, and the rest are packed by `PROCESSORS`. Acceptance commands now select by label
(`docs/development.md`, `docs/sdk/mission-training.md`): `python_sdk` is one CTest test per module, and the contention
cases of `mission_training` and `mission_c_abi` run as their `_stress` tests; `-L ma-training` still selects all of them.

| Current verification | Result |
| --- | --- |
| Pure tiers (`test_scope.py`, `test_baseline.py ContractTest`) | 43 and 10 cases passed, about 2 s |
| Targeted headless tier through `scope.py --run --build` (`testing_scope_checker`, `ma_baseline_checker`, `mission_package_consumer`) | 3/3 passed; 68 s wall with the build |
| Measurement of the uncommitted snapshot before this correction | full headless 690/690 in 1,121 s; Release `python_sdk`, `ma-training` and `testing` sets 90/91, `testing_scope_checker` failing on selector files being edited during that run |
| Release targeted tier (the same three tests, after a Release build with 0 warnings) | 3/3 passed in 55.66 s |
| Documentation checks (`ma_contract`, `ma_contract_checker`, `architecture_boundary`, `sdk_guard_checker`) | 4/4 passed |
| Full headless suite, run once on the frozen revision (19 files fingerprinted before and verified unchanged after) | 690/690 passed in 1227.65 s; build 0 warnings |

Pending as before: independent M8 generic acceptance; J5, J9, the combat parts of J3, J8 and J10; complete Viewer
acceptance; J6's speed resumption after an override. Logs: `D:/ENV/tmp/testing/logs/` (`r2`, `r3`, `final`).

The third maintenance pass includes bounded A1,
[A2 lifecycle/adapter](architecture/migration/history/2026-10-12-architecture-lifecycle-design.md) and
[A3 control/Mission](architecture/migration/history/2026-10-12-architecture-adapters-design.md) and
[A4 World/Scenario/Environment](architecture/migration/history/2026-10-12-architecture-world-adapters-design.md) and
[A5 VecEnv/Recording](architecture/migration/history/2026-10-12-architecture-batch-recording-design.md) and
[A6 store-binding slices](architecture/migration/history/2026-10-12-architecture-store-binding-design.md).
The remaining [architecture queue](architecture/migration/history/2026-10-11-architecture-maintenance-design.md#subsequent-work)
includes further World state-owner decomposition, a public-history retention contract, retained C World and Python core/type/buffer
ownership, finer control ownership, operational geometry, weapon observations, viewer/platform
boundaries and DLL export ownership. A2 retains map traversal,
controller mappings and post-removal history; it does not bound aggregate State storage or unanswered ended-strike
requests. These candidates require their own acceptance; architecture maintenance does not close a flight,
sensor or weapon-model milestone.

## Known workflow and reporting gaps

The first pass recorded these workflow gaps. The second pass repairs the first three; Python SDK availability remains
an environment-dependent condition. A1 makes architecture acceptance independent of SDK binding discovery.

| Workflow | Current behavior | Acceptance and limits |
| --- | --- | --- |
| Launcher bootstrap/configuration | Explicit `--preset` precedes `FSIM_PRESET`, then `ucrt64-headless`. Configure/build/run/test/dist and Testbench use the selected build. First build/dist configure automatically; help/where work before binaries exist. | `launcher_contract` exercises the real Windows batch entry, argument preservation, failures, first build and configuration isolation. Testbench viewer/profile regressions run inside `python_sdk`. |
| Independent SDK data contract | SDK installs stock JSBSim, designed aircraft/store models, cards, damage/pieces, notices and applicable vision resources. Exports describe the actual relocatable `bin` and `share/flightsim` layout. | `package_consumer` verifies complete runtime-file hashes, installed DLL locations, stock/custom/store flight and poisoned-XML rejection. The development resolver's source fallback remains; probes prove use of existing installed models. Vision resources are checked without GPU rendering. |
| Direct `dist` dependencies and consumers | `dist` builds all enabled installed targets, selects present components and removes inactive components' obsolete generated files. Only viewer maps are retained as reusable data. | `dist_components` builds a small independent fixture directly through dist, checks disabled-component cleanup, CMake exports and cleanup ownership. CTest/CI register both package gates. |
| Optional Python checks | Python discovery can return from `python/CMakeLists.txt` before registering Python, Hangar and `sdk_boundary` checks. | Check that required tests actually exist in the configured tree before interpreting a green CTest summary. |
| Native architecture checks | Three standard-library checks register independently of the Python SDK, using Python 3.11+ or `FSIM_CHECK_PYTHON`. | Repository imports and declaration policy, module checker fixtures and C guard scanner fixtures are exercised. Configure warns when no supported interpreter is available; that configuration has no architecture acceptance. |

## Cleanup decisions

- Removed the old `tactics.bvr` suite and its independent Python sensor/missile/kill-scoring implementation. The
  testbench no longer offers that scene. Native SDK encounter tests remain the engagement acceptance route.
- Removed the unused root `comp.spv`, three agent-process PNGs and seven unreferenced README/viewer screenshots.
  Referenced design/source photographs and dated control/performance evidence remain attached to their scope.
- Corrected strike/store capability descriptions to name the remaining calibrated weapon and launch-zone fidelity
  rather than claiming native guidance, seekers and fuzes do not exist. `Partial` support remains intentional.
- Replaced the maintained showcase, plume-check and viewer `hit` scene's scripted missile motion with SDK
  release and native JSBSim flight. Their scenes use observations and consent; SDK guidance, seekers, fuzes and
  damage determine the outcome. Missing support fails instead of falling back to supplied poses or timed bursts.
  Dated scripted-fixture images and performance records keep their original scope; they are not evidence for
  the replacement scenes.
  The optional native `--live` run publishes and paces the same scene for continuous fullscreen observation;
  captured frames alone do not establish full-flight acceptance. Completed live runs are scoped by the
  [viewer validation record](sdk/viewer-validation.md).
- Clarified the generated effects audit: zero missing entries is a result under its current exclusions, not proof
  that every physical effect exists. Rotorcraft exhaust/wake gaps remain B-59 in the [backlog](backlog.md).

## Verification records and update rules

### First maintenance verification, 2026-10-11

Local checks for the change committed as `2b83f24`:

| Check | Result |
| --- | --- |
| `cmake --build --preset ucrt64-headless -j 10` | Passed; native libraries, tests and Python package restaged |
| `ctest --preset ucrt64-headless -j 10 --timeout 3600` | All 572 tests passed, including 40 Hangar and 18 fleet-labeled cases; elapsed 886.00 seconds |
| Documentation links and commands | Checked all 22 changed/new Markdown files, local file links and anchors; no missing targets. All 13 PowerShell command blocks parsed |
| Development skill | Both frontmatter validations passed; canonical and discovery copies are byte-identical |
| Testbench catalogue | 89 scenarios remain; FA recovery and all 35 stability cases remain registered; no Tactics suite/scenario |
| Generated effects report | Regenerated with its owner; existing inventory rows and totals unchanged. Audit rules and scripted Python fixture behavior unchanged |
| Change scope | Diff formatting clean; no changes under `aircraft/`, `assets/stores/` or `third_party/` |

Raw build and test logs are local artifacts at `D:/ENV/tmp/project-cleanup-build.log` and
`D:/ENV/tmp/project-cleanup-tests.log`. This maintenance check did not rebuild the Vulkan viewer, perform GPU
visual acceptance, reproduce the external browser-client checks, or install/test standalone SDK components.

### Second maintenance verification, 2026-10-11

Local checks for the change committed as `f8cfec9`, based on `2b83f24`, using UCRT64 Release headless and
CPython 3.13.12. All compiler temporary directories use `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| `.\fsim.cmd --preset ucrt64-headless configure -DFSIM_PYTHON_EXECUTABLE:FILEPATH=D:/ENV/python/3.13/python.exe` and `build -j 10` | Passed; typed CMake flags preserved and current Python entry metadata regenerated |
| `.\fsim.cmd --preset ucrt64-headless test -LE hangar -j 10 --timeout 3600` | All 535 functional cases passed, including 18 fleet cases, Python SDK, SDK boundary, launcher and both package gates; elapsed 346.02 seconds |
| `launcher_contract` | 79 checks passed through real CMD/PowerShell entry points; includes empty/quoted/metacharacter arguments, preset precedence, first-build bootstrap, process failures and stale Python entry invalidation using real CMake |
| `package_consumer` | SDK-only install under a prefix containing spaces; 1,230 runtime-file hashes, installed fsim/JSBSim DLL provenance, stock/custom flight, four Mk 82 stores and native weapon stepping passed. All three poisoned-model probes rejected with exit 42; restored positive run passed |
| `dist_components` | Direct dist compiled an initially unbuilt example; disabling components removed obsolete executables/wheels, retained viewer map data and rejected cleanup outside the owned distribution tree. Independent CMake exports matched actual fixed paths with nondefault GNUInstallDirs cache values |
| Actual launch commands | Headless simulator stepped two C172s; direct configured Python created/stepped an SDK world; synchronous Testbench `fa1.support` passed |
| `.\fsim.cmd --preset ucrt64-headless dist -j 10` | Passed; SDK, tools, examples and Python trees plus wheel produced. Actual SDK distribution matched all 1,230 consumer manifest entries and tested runtime DLL bytes |
| Documentation and review | Nine changed/new Markdown files, local links/anchors, PowerShell blocks, preset JSON and CI YAML checked; skill copies matched. Independent review passed after export-layout and stale-entry fixes |
| Scope | No native implementation/public-header, aircraft, store-source or third-party changes; no architecture refactor |

Logs are `D:/ENV/tmp/preset-sdk-configure.log`, `preset-sdk-build.log`, `preset-sdk-functional-tests.log`,
`preset-sdk-headless-run.log`, `preset-sdk-python-run.log`, `preset-sdk-testbench.log` and `preset-sdk-dist.log`.
Installed-consumer logs and `runtime-data.sha256` are under `build/ucrt64-headless/package-consumer`.
The unchanged 40 Hangar cases retain the first pass's full-suite evidence and were not rerun in this pass.
The renderer/GPU, render SDK installation, CPack ZIP and remote GitHub workflow were not exercised. Headless
package acceptance does not close Platform M5's public-release or GPU target matrix.

### Architecture A1 verification, 2026-10-11

Fresh local checks for A1, based on `f8cfec9`, using UCRT64 Release headless and
CPython 3.13.12. Compiler temporary directories remain under `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| Headless configure and `build -j 10` through `fsim.cmd` | Passed; libraries, native tests and Python package rebuilt |
| `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600` | All 538 functional cases passed, including 18 fleet cases, Python/principal checks, launcher and both installed-package gates; elapsed 325.27 seconds |
| `.\fsim.cmd --preset ucrt64-headless test -L architecture --output-on-failure` | All three checks passed in 2.94 seconds: repository policy covers 19 modules and 332 source/header files, with 33 module fixtures and 22 C guard scanner test methods |
| Architecture checks with `FSIM_BUILD_PYTHON=OFF` | All three passed in 5.24 seconds in an independent headless configuration. This establishes registration without Python SDK bindings; the independent configuration was not used for a native build |
| World launch characterization and focused native acceptance | Before extraction, two new World cases passed with 443 assertions. After extraction, all 17 selected cases passed in 65.97 seconds, covering launch eligibility, stores, quaternion/JSBSim comparisons, C ABI and native engagement/worker determinism |
| Adapter and ABI comparison | The extracted 710-line MS block contains 78 exported definitions and is byte-identical to its baseline. Existing error/guard/copy helpers are unchanged; all 1,338 rebuilt DLL export names, including 466 C exports, match the captured baseline |
| `.\fsim.cmd --preset ucrt64-headless dist -j 10` | Passed; distribution refreshed from the rebuilt SDK. All 1,230 runtime manifest entries and both fsim/JSBSim DLL bytes match the independent installed consumer |
| Independent review | Three checker gaps were reproduced and fixed: target include-search imports, qualified world-handle inputs and non-enforcing guard macros. Final negative fixtures and repository checks passed; no remaining actionable finding in the reviewed A1 scope |
| Documentation | Eight changed/new Markdown files, 164 local links and five anchors checked with no errors; all 11 PowerShell command blocks parsed and development skill copies remained byte-identical |
| Scope | No changes under public `include/`, aircraft, store-source or third-party paths; no generated model changes |

The functional and architecture runs cover 541 distinct CTest cases. Raw logs are local artifacts at
`D:/ENV/tmp/architecture-a1-configure.log`, `architecture-a1-build.log`, `architecture-a1-functional-tests.log`,
`architecture-a1-architecture-checks.log`, `architecture-a1-focused-tests.log` and
`architecture-a1-no-python-sdk-checks.log`. Distribution output is in `D:/ENV/tmp/architecture-a1-dist.log`;
manifest, DLL and scope comparison results are in `D:/ENV/tmp/architecture-a1-final-verification.json`.
Adapter comparison evidence is in
`D:/ENV/tmp/architecture-a1-sdk-extraction-check.json`; installed-consumer logs and runtime hashes are under
`build/ucrt64-headless/package-consumer`.

The first functional command through the batch wrapper failed before CTest started because CMD interpreted the
combined label expression's pipe. The successful recorded run invoked CTest directly with the same preset and
selection; the initial shell failure remains in `D:/ENV/tmp/architecture-a1-functional-tests-shell.log`.
Unchanged Hangar generation retains the first pass's 40-case evidence. Renderer/GPU behavior, render SDK
installation, CPack ZIP and remote CI were not exercised. The dependency gate scans renderer source declarations
in a headless checkout; that is source acceptance, not a renderer build.

### Architecture A2 verification, 2026-10-12

Fresh local checks for A2, committed as `34e5516`, collected during A2 validation and based on A1
`6652584`, using UCRT64 Release headless, warnings as errors and CPython 3.13.12. Compiler temporary directories
remain under `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| Headless `build -j 10` through `fsim.cmd` | Passed; native libraries/tests and both Python adapter translation units compiled and the Python package restaged |
| `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600` | All 547 functional cases passed, including 18 fleet cases, C ABI, Python agent/isolation/thread/buffer checks, native engagement determinism, launcher and both package gates; elapsed 325.12 seconds |
| Architecture gates | All three passed in 2.51 seconds; repository policy covers 19 modules and 334 source/header files, with 33 module fixtures and 22 C guard scanner test methods |
| MA baseline characterization | Before MA edits, three public cases passed with 105 assertions and one borrowed-storage case passed with four assertions. They cover removal/reset history, C2 settlement, finite/NaN overrides, lost communication, slot reuse, first answers and authorized late replies |
| MA storage and public acceptance | The new ledger tests first failed for the absent implementation. After integration, all six internal cases passed with 4,134 assertions, including 512 answer and 512 retirement/reuse cycles; all three public characterization cases passed with 105 assertions |
| C adapter comparison | The 659-line DMS/C2 block with 70 exported definitions is byte-identical to the A1 baseline. The original-file remainder, three conversions and shared guard body are unchanged; only the guard declaration gains an unused-helper annotation |
| Python adapter comparison | All 49 moved handler/converter bodies, six shared helpers, 14 registration tables, WorldObject layout, type registration and 23 existing busy checks match the baseline. No GIL phase is added. All 47 native descriptors retain their names, qualified names, owners, types and documentation; seven compatibility tests passed before extraction, after rebuilding and after final distribution staging |
| Exports and protected scope | All 1,338 SDK DLL export names, including 466 C exports, match A1. The Python native module still exports only `PyInit__native`. SHA-256 checks match all 3,204 captured public-header, Python object-model, aircraft/store/generated, third-party and boundary-inventory files |
| Installed consumer and `.\fsim.cmd --preset ucrt64-headless dist -j 10` | Both package gates passed in the functional suite. Distribution refreshed successfully; all 1,230 runtime manifest entries and both fsim/JSBSim DLL bytes match the tested installed consumer. Distributed `_native.pyd` matches the rebuilt, tested module |
| Independent review and documentation | Complete A2 diff independently reviewed with no actionable findings. Six changed/new Markdown files, 151 local links and six anchors checked with no errors; all five PowerShell blocks parsed. Development skill copies remain byte-identical |

The functional and architecture runs cover 550 distinct CTest cases. Raw logs are local artifacts at
`D:/ENV/tmp/architecture-a2-build.log`, `architecture-a2-functional-tests.log`,
`architecture-a2-architecture-tests.log`, `architecture-a2-ma-characterization.log`,
`architecture-a2-ma-borrowed-characterization.log`, `architecture-a2-ma-ledger-red.log`,
`architecture-a2-ma-ledger-green.log`, `architecture-a2-ma-public-green.log`,
`architecture-a2-python-compatibility.log`, `architecture-a2-python-dist-compatibility.log` and
`architecture-a2-dist.log`. C extraction evidence is in `architecture-a2-c-links-extraction-check.json`;
Python source/runtime comparison evidence is under `D:/ENV/tmp/a2-python-native`.
Final export, scope and distribution comparison results are in
`D:/ENV/tmp/architecture-a2-final-verification.json`. Installed-consumer logs and runtime hashes are under
`build/ucrt64-headless/package-consumer`.

Two initial pre-refactor fixture runs exposed assumptions about message arrival order and cancel success:
network jitter can reorder simultaneous replies, and successful cancellation returns `Canceled`. The fixtures
were corrected before production edits; their original failure logs remain as
`architecture-a2-ma-characterization-baseline-fixture-failure.log` and
`architecture-a2-ma-characterization-baseline-cancel-fixture-failure.log` under `D:/ENV/tmp`.
Final formatting normalized one new Python source file's trailing newline; distribution rebuilt that module,
and the literal body checks plus seven compatibility tests passed again.

A2 reclaims answered private consent records and releases their storage when an entity is removed. It retains
public State/history, borrowed strings, controller mappings and unanswered ended-strike requests needed for late
answers. Retired entries still occupy the map; settled entries skip maintenance, but traversal remains a map scan.
No aggregate history bound or performance improvement was measured. Unchanged Hangar generation retains the first
pass's 40-case evidence and was not rerun. Renderer/GPU behavior, render SDK installation, CPack ZIP and remote CI
were not exercised. This architecture slice does not close a flight, sensor, weapon or public-release milestone.

### Architecture A3 verification, 2026-10-12

Fresh local checks for A3, subsequently committed as `e79f268`, were collected during A3 validation
against A2 commit `34e5516`.
The [A3 design](architecture/migration/history/2026-10-12-architecture-adapters-design.md) and
[implementation plan](architecture/migration/history/2026-10-12-architecture-adapters.md) record the extraction seams and scope.
All commands used the existing Windows x64 UCRT64 headless Release preset and CPython 3.13.12, with compilation
temporary directories under `D:/ENV/tmp` and warnings treated as errors. No dependency was installed.

| Check | Result |
| --- | --- |
| Baseline characterization | Standalone C adapter consumer passed 1/1 and six new Python domain cases passed against A2 before production extraction. Missing-owner extraction checks failed as expected before the new files existed |
| Headless build through `fsim.cmd` | Passed; the C control and Python control/Mission translation units link into the existing binaries. After normalizing two EOF blank lines, the affected Python sources rebuilt and the package was restaged before the full suite |
| `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` | Passed 548/548 in 323.33 s, including control/Mission, agent/principal, MA lifecycle, isolation/thread/buffer, determinism, fleet and both package gates |
| Focused C/Python and architecture acceptance | C ABI, Python SDK, SDK boundary and new C compatibility consumer passed 4/4 in 82.72 s. Three architecture checks passed in 2.29 s |
| C extraction comparison | All 233 exported control/VI definitions, 55 local helpers, five layout assertions, 157 first principal guards and 32 source-authorization macros match A2. The parameter helper and original-file remainder retain their bodies/linkage; prior Mission/DMS owners and shared handle/error support are unchanged |
| Python source/runtime comparison | Control has 112 World and 13 module handlers; Mission has 37 World and 12 module handlers. Moved bodies/macros, eight shared helpers, 14 registration initializers, four type specs, PyInit and WorldObject layout match A2. All 229 World descriptors and 32 module functions retain their names, owners, qualified names, types and documentation. Existing busy/GIL/buffer/reference operations retain their positions |
| Exports and protected scope | All 1,338 SDK DLL export names match A2; the Python native module exports only `PyInit__native`. SHA-256 checks match all 3,514 captured public/object-model, resource, native domain, third-party and prior adapter files |
| Installed consumer, dist and distributed Python | Both package gates passed. Dist refreshed; all 1,230 runtime manifest entries and fsim/JSBSim DLL bytes match the tested installed SDK consumer. Distributed Python matches the staged package, and both binding compatibility suites passed 13/13 against dist |
| Independent review and documentation | Complete C/Python/test diff independently reviewed with no actionable regression. Seven changed/new Markdown files, 158 local links, six anchors and five PowerShell blocks checked. English content, exact file endings and diff scope passed; development skill copies remain byte-identical |

The functional and architecture runs cover 551 distinct CTest cases. Raw logs are under `D:/ENV/tmp`:
`architecture-a3-build.log`, `architecture-a3-final-build.log`, `architecture-a3-focused-tests.log`,
`architecture-a3-architecture-tests.log`, `architecture-a3-functional-tests.log`,
`architecture-a3-c-characterization-baseline.log`, `architecture-a3-python-characterization-baseline.log`,
`architecture-a3-c-extraction-check.log`, `architecture-a3-python-source-check.log`,
`architecture-a3-runtime-check.log`, `architecture-a3-dist.log` and `architecture-a3-python-dist-compatibility.log`.
The final export/scope/distribution results are in `architecture-a3-final-verification.json`; source/runtime
baselines are in `a3-baseline` and `a3-c-control`. Installed-consumer evidence remains under
`build/ucrt64-headless/package-consumer`.

Four initial C fixture failures were corrected before production edits: STARTING is not a requestable target;
activities return newest live entries before ended entries; controller zero is an agent's own identity; and
agent_identity returns boolean one on success. Their original logs remain as
`architecture-a3-c-characterization-fixture-{state,order,claims,identity}-failure.log` under `D:/ENV/tmp`.
Root also corrected initial test registration by separating the standalone main from the Catch SDK target,
restoring all existing test discovery. The EOF formatting failure is retained in `architecture-a3-formatting-failure.log`.

A3 preserves direct/guarded error clearing, null/short-result side effects, command readback and borrowed lifetime
distinctions. Mission's existing busy-check policy and simulation/domain behavior are unchanged. The working tree
was left uncommitted for review, then committed at the user's request. Unchanged Hangar generation retains the first
pass's 40-case evidence and was not rerun. Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This slice does not close
the separate physics, sensor, weapon or public-release milestones.

### Architecture A4 verification, 2026-10-12

Fresh checks were collected for A4, against committed A3 `e79f268`.
The [A4 design](architecture/migration/history/2026-10-12-architecture-world-adapters-design.md) and
[implementation plan](architecture/migration/history/2026-10-12-architecture-world-adapters.md) record the selected ownership
and stopping point. All builds used the existing UCRT64 headless Release preset, warnings as errors and CPython
3.13.12, with TMP/TEMP/TORCH_EXTENSIONS_DIR under `D:/ENV/tmp`. No dependencies were installed.

| Check | Result |
| --- | --- |
| A3 characterization before extraction | Public C consumer passed 1/1 and six new Python cases passed 6/6. A final C assertion refinement checks borrowed string content instead of unspecified pointer identity; its A3 baseline passed again. Both missing-owner source verifiers failed as expected before the new files existed |
| Headless build through `fsim.cmd` | Passed with warnings as errors; both new C owners and all three new Python owners compiled and linked into the existing SDK/module, and Python staging refreshed |
| Focused C/Python and architecture acceptance | C ABI, Python SDK, SDK boundary and both C adapter compatibility gates passed 5/5 in 83.69 s. Three architecture gates passed in 2.32 s |
| Full applicable functional suite | `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` passed 549/549 in 325.02 s, including 18 fleet cases, principal/isolation/thread/buffer checks, lifecycle, native engagement determinism, launcher and both package gates |
| C source comparison | Six Scenario and eight Environment exports are literal extractions, including all five moved first principal guards. World retains 56 definitions and control retains 228. Original remainders and prior Mission/DMS owners, Handles, CAbiSupport and CParameterSupport remain byte-identical to A3 |
| Python source/runtime comparison | The 23 World, four Scenario and eight Environment callbacks, field metadata and shared layouts retain their bodies. Core remainder, eight shared helpers, 14 registration initializers, four type specs and PyInit match A3. All 246 native descriptors across World/Scenario/VecEnv/Recording, 32 module functions and four type layouts/flags match; busy/GIL/buffer operations retain their original positions |
| Exports, protected scope and inventory | All 1,338 SDK export names match A3; Python exports only PyInit__native. SHA-256 matches all 3,692 protected files. The exact 22-file diff scope passes. All 591 A3 CTest names remain; the new standalone compatibility gate brings registration to 592, including 40 unchanged Hangar cases |
| Installed consumer, dist and distributed Python | Both package gates passed. Dist refreshed successfully; all 1,230 resource manifest entries and both SDK/JSBSim DLL bytes match the tested installed consumer. Distributed native/object-model bytes match staging, and all three binding compatibility suites passed 19/19 against the distributed module |
| Independent review and documentation | Complete C/Python/test diff independently reviewed with no actionable regression. English content, local links/anchors, PowerShell examples, exact new-file endings and diff checks pass; development skill copies remain byte-identical |

The functional and architecture runs cover 552 distinct CTest cases. Raw logs are under `D:/ENV/tmp`:
`architecture-a4-build.log`, `architecture-a4-focused-tests.log`, `architecture-a4-architecture-tests.log`,
`architecture-a4-functional-tests.log`, `architecture-a4-c-characterization-baseline.log`,
`architecture-a4-python-characterization-baseline.log`, `architecture-a4-characterization-final-baseline.log`,
`architecture-a4-dist.log` and `architecture-a4-python-dist-compatibility.log`. Final scope/export/resource results
are in `architecture-a4-final-verification.json`. Baselines are under `a4-baseline`; literal extraction evidence
and expected missing-owner failures are under `a4-c-adapters` and `a4-python-adapters`. Installed-consumer logs and
runtime hashes remain under `build/ucrt64-headless/package-consumer`.

No baseline behavior fixture or post-extraction acceptance failed. Existing native Scenario tests cover a later
command error retaining earlier creation/environment side effects; the new C consumer separately covers success,
borrowed strings, versioned options, truncated IDs and principal refusal. Scenario's borrowed C strings, Python
type ownership, partial application, Environment error/NaN distinctions and existing busy/GIL behavior remain
unchanged. Session World state ownership is unchanged. Python runtime construction/destruction, agent
creation/sealing, state buffers, scenario application, VecEnv/Recording and public object identities remain in
their established owners.

A4 was left uncommitted for review, then committed as e708b87 at the user's request after fresh source/runtime,
scope/export/distribution, documentation and focused architecture/C verification.
Unchanged Hangar generation retains first-pass evidence and was not rerun.
Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This adapter slice does not close
separate physics, sensor, weapon, history-retention or public-release milestones.

### Architecture A5 verification, 2026-10-12

Fresh checks were collected for A5, against committed A4 `e708b87`.
The [A5 design](architecture/migration/history/2026-10-12-architecture-batch-recording-design.md) and
[implementation plan](architecture/migration/history/2026-10-12-architecture-batch-recording.md) record the selected split and
retained ownership. Builds used the existing UCRT64 headless Release preset with warnings as errors and CPython
3.13.12. Compilation TMP/TEMP/TORCH_EXTENSIONS_DIR remained under `D:/ENV/tmp`; no dependencies were installed.

| Check | Result |
| --- | --- |
| A4 characterization before extraction | New public C consumer passed 1/1 in 0.30 s; six Python cases passed 6/6 in 0.206 s. Both missing-owner source verifiers failed as expected before production moves |
| Headless build through `fsim.cmd` | Passed with warnings as errors. New C/Python owners linked into the existing binaries, C++ wrappers remained in fsim.cpp and the Python package restaged |
| Focused C/Python and architecture acceptance | C ABI, Python SDK, SDK boundary and all three C adapter compatibility gates passed 6/6 in 83.51 s. Three architecture gates passed in 2.41 s |
| Full applicable functional suite | `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` passed 550/550 in 324.74 s, including 18 fleet cases, principals, thread/buffer/lifecycle checks, native engagement determinism, launcher and both package gates |
| C source comparison | All 14 VecEnv and nine Recording C definitions retain their original bodies. VecEnv's three local helpers and opaque layout match A4; C++ Impl/methods/toOptions/span/toResult, lastError, log/version/registry and original C linkage are retained. Recording's layout/assertions and borrowed views are preserved. World recording/lifecycle guards remain core; only exposed EOF blank lines were normalized |
| Python source/runtime comparison | All 11 VecEnv and five Recording callbacks, option metadata, names_list and exact private layouts match A4. Core retains three coupled callbacks, 14 registration initializers, four type specs, eight shared helpers and PyInit. All 246 descriptors, 32 module functions and four type layouts/flags match; existing GIL/busy/buffer/reference operations retain their positions |
| Exports, protected scope and inventory | All 1,338 SDK export names match A4; Python exports only PyInit__native. SHA-256 matches all 3,701 protected files. Exact 22-file diff scope passes. All 592 A4 CTest names remain; the new standalone consumer brings registration to 593, including 40 unchanged Hangar cases |
| Installed consumer, dist and distributed Python | Both package gates passed. Dist refreshed; all 1,230 runtime manifest entries and fsim/JSBSim DLL bytes match the tested installed consumer. Staged/distributed native and public Python wrappers, including VecEnv/Recording, match. All four binding compatibility suites passed 25/25 against distributed Python |
| Independent review and documentation | Complete adapter/test diff independently reviewed with no actionable regression, including independent expected/frozen hash checks. English content, links/anchors, PowerShell examples, exact new-file endings and diff checks pass; development skill copies remain byte-identical |

The functional and architecture runs cover 553 distinct CTest cases. Raw logs are under `D:/ENV/tmp`:
`architecture-a5-build.log`, `architecture-a5-c-characterization-baseline.log`, `a5-binding-baseline-final.log`,
`architecture-a5-focused-tests.log`, `architecture-a5-architecture-tests.log`, `architecture-a5-functional-tests.log`,
`architecture-a5-c-source-check.log`, `architecture-a5-python-source-check.log`, `architecture-a5-runtime-check.log`,
`architecture-a5-dist.log` and `architecture-a5-python-dist-compatibility.log`. Final scope/export/resource results
are in `architecture-a5-final-verification.json`. Baselines and exact source expectations are under `a5-baseline`,
`a5-c-adapters` and `a5-python-adapters`. Installed-consumer logs and runtime hashes remain under
`build/ucrt64-headless/package-consumer`.

The first Python fixture used World's nine pointer words as VecEnv's size; A4's VecEnv actually has eight. It was
corrected before production moves; the first failure remains in `a5-binding-baseline-first.log`, followed by
corrected/final baseline logs. Committed tests use standard temporary-directory discovery; local runs select
`D:/ENV/tmp` through the environment. C extraction exposed two extra EOF newlines in World; only those bytes were
normalized before rebuilding. The original formatting failure remains in `architecture-a5-formatting-failure.log`,
with pre-normalization expected bytes/metadata retained by the C verifier. Post-extraction acceptance had no failures.

VecEnv's original void-return guard remains distinct from the shared int-return guard; direct LastError retention
and guarded success clearing are unchanged. The cached borrowed World and zero-copy VecEnv/Recording views retain
their owner lifetime. C++ VecEnv wrappers and shared LastError remain core. Python buffers/borrowed World/sample
callbacks, all runtime type pointers and registration remain core; other callbacks have focused owners. Public
headers/object models, native simulation/domain sources and resource bytes remain unchanged.

A5 was left uncommitted for review, then committed as `2efea05` at the user's request after fresh source/runtime,
scope/export/distribution and documentation checks. Eight focused ABI, boundary, adapter and architecture gates
passed again before commit. Unchanged Hangar generation retains first-pass evidence and was not rerun.
Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This adapter slice does not close
separate physics, sensor, weapon, history-retention or public-release milestones.

### Architecture A6 verification, 2026-10-12

Fresh checks were collected for A6, against A5 delivery baseline `39be3c1`.
The [A6 design](architecture/migration/history/2026-10-12-architecture-store-binding-design.md) and
[implementation plan](architecture/migration/history/2026-10-12-architecture-store-binding.md) describe the first World state-owner
slice. Builds used the existing UCRT64 headless Release preset, warnings as errors and CPython 3.13.12, with
TMP/TEMP/TORCH_EXTENSIONS_DIR under `D:/ENV/tmp`. No dependencies were installed.

| Check | Result |
| --- | --- |
| A5 characterization before extraction | Three new production-path cases passed with 2,655 assertions after correcting the fixture's initial Host revision assumption. Eight worker/frameSkip/drag variants recorded reset, reuse, between-step inventory edits, external fuel, tick boundaries and asymmetric release |
| Source extraction | The missing-owner verifier failed before implementation. After extraction, all five owned production files and complete World/CMake remainders match explicit baseline-derived expectations; WorldDamage remains unchanged |
| Headless build | Passed with warnings as errors; the new private owner compiles into the existing session target, without new libraries or external dependencies |
| Post-extraction characterization and replay | The same three cases and 2,655 assertions passed. All eight before/after traces match exactly: 67,553 lines, 2,641,139 bytes, SHA-256 `2d6e061a2dd7b7b84f0ec02ba32d83ff6b1e6ec9a80b99204861d731b7f32df5`. State fields use explicit scalar encodings, including float speedbrake, without structure padding |
| ABI, principal, adapter and architecture gates | All eight passed in 7.41 s: C ABI, SDK boundary, three C adapter compatibility consumers and three architecture checks |
| Full applicable functional suite | `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` passed 553/553 in 325.59 s, including 18 fleet cases, lifecycle/principal/thread/buffer checks, native engagement determinism, Python, launcher and both package/installed-consumer gates |
| Exports, protected scope and inventory | SHA-256 matches all 3,720 protected files; all 1,338 SDK export names, sole Python `PyInit__native` export, 246 native descriptors, 32 module functions and four type layouts match A5. The exact 13-file diff scope passes. All 593 existing CTest names remain, with three new store-binding cases bringing registration to 596 |
| Installed consumer, dist and distributed Python | Both package gates passed in the functional suite. `cmake --build --preset ucrt64-headless --target dist -j 10` passed; all 1,230 runtime manifest entries and SDK/JSBSim DLL bytes match the tested installed consumer. Staged/distributed native and public Python wrapper bytes match; all four binding compatibility suites passed 25/25 against the distributed module |
| Independent review and documentation | Complete source/test diff independently reviewed with no actionable findings. Six changed/new Markdown files, 175 local links and five anchors pass; all five PowerShell blocks parse. English text, new-file endings and diff checks pass; development skill copies remain byte-identical |

Raw logs and replay files are under `D:/ENV/tmp`: `architecture-a6-build.log`,
`architecture-a6-characterization-baseline.log`, `architecture-a6-characterization-baseline-corrected.log`,
`architecture-a6-characterization-after.log`, `architecture-a6-replay-baseline.txt`, `architecture-a6-replay-after.txt`,
`architecture-a6-focused-tests.log`, `architecture-a6-functional-tests.log`, `architecture-a6-dist.log` and
`architecture-a6-python-dist-compatibility.log`. Final scope/export/resource results are in
`architecture-a6-final-verification.json`; documentation checks are in `architecture-a6-doc-verification.log` and
`architecture-a6-doc-commands.log`. The functional and architecture runs cover 556 distinct CTest cases.
Baselines are under `a6-baseline`;
explicit extraction expectations, original bytes and RED/GREEN evidence are under `a6-owner`. Independent source
review is in `architecture-a6-independent-review.md`. Installed-consumer evidence remains under
`build/ucrt64-headless/package-consumer`.

The first fixture incorrectly expected zero fresh Host revisions; Host::bind advances storeRevision. Its 18 failed
assertions were corrected against unchanged A5 source by comparing original/fresh Host revisions, then the baseline
passed. The first failure remains in its log. No production initialization or reset behavior was changed to pass;
post-extraction acceptance had no failures.

StoreFlightBinding owns Entry's station property handles, mass/drag revisions, between-step carried-mass stamp,
cached drag points/qbar and load-report revision. It borrows model property nodes and receives Host/model inputs
explicitly, retaining no World, Host or model pointer. World retains model/Entry lifetime, aircraft station-name
cache, traversal, permission checks, loadTick, control publication and release/damage mass transactions.
Separate store/fuel revisions, lazy qbar acquisition, refused-load synchronization, non-finite weight-room return,
fuel dt/threshold/draw/refuel order and force summation remain unchanged. Loaded reset retains inventory and
binding state; same-aircraft reuse creates a fresh Entry/Host/binding around the retained model, including its
original pre-synchronization mass transient.

A6 was initially left uncommitted for review, then delivered at the user's request. This initial acceptance retained
the first pass's unchanged Hangar generation evidence and did not rerun it.
Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This slice does not close the
remaining World state-owner work or separate physics, sensor, weapon, history-retention and public-release milestones.

### Architecture A6 post-push validation, 2026-10-12

The user requested delivery followed by fresh validation. Implementation commit
`b57947d75ea85e998e0af40fa9daf7828b5f9cd5` was pushed to origin;
an independent remote-head query matched local HEAD at that delivery.
The checkout was clean. The existing UCRT64 headless Release build retained warnings as errors and CPython 3.13.12.
TMP/TEMP/TORCH_EXTENSIONS_DIR and Matplotlib's configuration/cache directory were under `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| Before commit/push | `fsim.cmd` build passed; 11 focused CTest cases passed in 7.22 s. Literal extraction, protected files, exports/runtime identities, exact scope and documentation checks passed |
| Build after push | `.\fsim.cmd --preset ucrt64-headless build -j 10` passed |
| Full registered headless suite | `ctest --preset ucrt64-headless -j 10 --timeout 3600 --output-on-failure` passed 596/596 in 887.51 s: 553 functional cases, three architecture checks and all 40 Hangar classes. This includes 18 fleet cases, native engagement determinism, lifecycle/principals, Python, launcher and both installed-consumer/package gates |
| Fresh production-path replay | All three store-binding cases passed with 2,655 assertions. Eight worker/frameSkip/drag variants reproduce the original 67,553-line, 2,641,139-byte baseline exactly, with the same SHA-256 recorded above |
| Distribution after the full suite | `.\fsim.cmd --preset ucrt64-headless dist -j 10` passed. All 1,230 runtime manifest entries and SDK/JSBSim DLL bytes match the freshly tested installed consumer; distributed Python native/wrapper bytes match staging |
| Distributed Python compatibility | All four binding compatibility suites passed 25/25 in 1.195 s against the actual distributed native module |
| Protected scope and documentation | All 3,720 protected hashes, 1,338 SDK export names, sole Python export, 246 descriptors, 32 functions and four type layouts match the A5 baseline. The committed A6 slice retains its exact 13-file scope. Six Markdown files, 175 local links, five anchors and five PowerShell blocks pass |

The Hangar run validates existing geometry/manifests, mass, carriage, effects, vulnerability and temporary-copy
generation/startup cases. It does not rebuild all canonical aircraft assets or validate GPU rendering.
Post-push logs are under `D:/ENV/tmp`: `architecture-a6-delivery-build.log`,
`architecture-a6-delivery-full-tests.log`, `architecture-a6-delivery-characterization.log`,
`architecture-a6-delivery-dist.log` and `architecture-a6-delivery-python-dist-compatibility.log`. Remote identity is in
`architecture-a6-delivery-push.json`; the fresh trace is `architecture-a6-delivery-replay-after.txt`. Committed-scope
results are in `architecture-a6-delivery-final-verification.json`; documentation results are in
`architecture-a6-delivery-doc-verification.log` and `architecture-a6-delivery-doc-commands.log`.

### Main integration, 2026-10-12

At the user's request, the existing `main` was fast-forwarded to the fully validated `03f512f` delivery, bringing
both maintenance passes and Architecture A1 through A6 into `main`. Remote main was an ancestor with no divergent
commits. Before the workflow documentation update, main's complete tree was identical to `03f512f`:
`29fc31b2513fd9e36be5e8b7ac2d15baad8d8c21`.

Only this status page and the [development workflow](development.md) change after integration. Source, tests,
public headers, Python, build configuration and resources retain the exact validated tree. The 596/596 full-suite
and 25/25 distributed Python results above remain applicable; they are not represented as new runs on main.
Fresh ABI, boundary, adapter and architecture gates passed 8/8 in 7.27 seconds. Protected source/resource hashes,
exports, Python identities and distribution provenance match the recorded A6 delivery. Both changed Markdown
files, 82 local links, four anchors and five PowerShell blocks pass, together with source identity and diff checks.
Logs are under `D:/ENV/tmp`: `architecture-a6-main-focused-tests.log`,
`architecture-a6-main-protected-verification.log`, `architecture-a6-main-doc-verification.log` and
`architecture-a6-main-doc-commands.log`.
Future implementation, documentation, commits and pushes use the existing `main`; another branch or worktree
requires an explicit user request.

### Earlier delivery evidence

The [native-engagement validation record](sdk/engagement-validation.md) reports a repaired functional suite, an
unchanged model suite, Python and boundary checks, native encounters, and an external browser integration. It also
records the initial allocation failure and the limits of the final checks. Preserve that distinction rather than
rewriting it as one uninterrupted green run.

The checked-in [CI workflow](../.github/workflows/ci.yml) configures a Windows UCRT64 headless build, runs CTest, and
rebuilds store/effect outputs plus one aircraft. The second maintenance pass adds launcher, distribution and SDK-only
component-install consumer acceptance through the same preset entry point. It does not build the Vulkan viewer.
Its header records that it had not yet run on GitHub when written; no remote run was inspected for this maintenance work.

When updating status:

1. Name the implementation revision, build/preset and the commands actually run.
2. Keep source inspection, recorded checks and fresh measurements separate.
3. Update the SDK reference and affected current-status statements together; mark replaced plans as historical.
4. Close a backlog/stage item only against its own acceptance criteria and evidence.
5. Keep agent/range/referee access, ABI compatibility, deterministic behavior and resource provenance explicit.
