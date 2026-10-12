# MA System Integration and Training Readiness Design

Status: Accepted architecture. MA contract/API/environment implementation remains pending.
Date: 2026-10-12.
Repository: Super-DDCCSS-WWorld, main.
Implementation baseline: baf476f841970f1f3c90b8de0bf630e8a5478738, separately verified and pushed to main. The source audit began at c2641e9cf51d2f84befa96d6a1d0b592029898d4; the delivered baseline contains the reviewed Viewer/native engagement work. See the [implementation plan](../plans/2026-10-12-ma-system-training-readiness.md), [vertical work allocation](../plans/2026-10-12-ma-system-training-work-allocation.md) and [Viewer evidence](../../sdk/viewer-validation.md).

## 1. Purpose and accepted ownership

Prepare the SDK for complete Mission Autonomy systems and their training, including local mission decisions, peer coordination and C2 interaction. An MA may contain several learned actors, behavior trees, planners, deterministic algorithms, internal services and shared state. Neither an aircraft nor an SDK Agent implies exactly one learned actor.

The user explicitly selected the following ownership:

- The formal MA decision runtime belongs in an upper-layer project.
- This repository supplies permission-scoped SDK interfaces, simulation and training environments, minimal composition examples and acceptance evidence.
- The SDK remains the sole authority for platform execution, communication delivery, guidance, fuzes, collisions and damage.
- Conversation is Chinese. Maintained files, code, tests, UI text and commit messages are English.

The intended result is training readiness, not a trained operational policy, a universal MA runtime or formal ASK compliance.

## 2. Current evidence and compatibility constraints

These findings were checked against the current source. They are not new runtime test results:

1. Agent interfaces mask sensor truth and enforce own-platform commands. C2 tasks, peer reports, shared tracks, DMS, approval, direct commands, lost-link handling and packages already exist. Extend and integrate them; do not rebuild the transport.
2. C++ and Python Agent surfaces omit some actor-safe capabilities already accepted by the C adapter: own passive contacts, countermeasure inventory/dispense and own-designation launch-zone queries.
3. StrikeStatus describes release activity. Done does not mean impact or mission completion. Global weapon/engagement records contain referee truth; destroyed currently means actual crew loss.
4. Current VecEnv has a single World and a single Task object for the batch, flight-oriented actions and observations, and vehicle-oriented reset. It is not an independently isolated combat-episode environment.
5. The existing vehicle reset contract deliberately preserves MA authority, tasks, overrides and C2 activity ownership. tests/test_agents.cpp contains an explicit preservation test. Full episode reset must be a different operation.
6. MaCore state and DMS inboxes are platform/system-owned. Agent.receive consumes the shared inbox. Multiple internal components cannot independently drain that inbox and expect identical observations.
7. MaCore::designate currently chooses a platform controller by agent traversal order. Multiple handles are not sufficient to specify multiple MA-service capability ownership.
8. Current Showcase uses internal World and range-supplied consent. Its real native flyout/viewer evidence does not exercise the entire installed Agent/P2P/C2 path.

Preserve existing signatures, class sizes, public plugin vtables, enum numbers, errors, observation/action shapes, legacy defaults and reset semantics. New capabilities use additive, versioned interfaces. Existing binaries must continue to work with the new library.

## 3. Terms, identities and ownership

| Term | Owner and meaning |
| --- | --- |
| Episode | One independently initialized battlefield, its clock, RNG streams, entities, links, tasks and referee |
| Platform/System | Aircraft or C2 communication node; owns platform equipment, system identity, DMS and authority state |
| MA System | Upper-layer mission decision system associated with one or more explicitly configured service endpoints |
| MA Service | An explicitly bound SDK-facing service/controller identity on a platform or C2 node |
| Component | Upper-layer actor, behavior-tree executive, planner, rule module or internal service |
| DRL Actor | A trainable component; actor count is independent of platform and MA-service count |
| SDK Agent | A permission-scoped endpoint, not an MA implementation or neural-network model |
| Referee | Truth access for task outcome, reward, termination, curriculum and a centralized critic |

Identify objects with separate fields: env_id, episode_generation, local_episode_index, system_id, ma_service_id, controller_id, component_id and request_id. VI/MS activity IDs, C2 command/task/consent IDs and DMS message IDs retain their separate namespaces.

Internal components may share legally obtained reports, derived estimates and private MA state through ordinary internal interfaces within the same onboard/platform boundary. A common upper-layer wrapper cannot share a blackboard across aircraft or C2 nodes to bypass the links. Peer MAs and C2 systems exchange information only through their own sensors or simulated links. Internal sharing never expands the platform's permissions or introduces referee truth.

## 4. Responsibilities and dependency direction

| Responsibility | Location |
| --- | --- |
| Tactical target/task/weapon/time selection, peer negotiation, task execution planning | Upper-layer MA |
| Behavior trees, blackboards, actor scheduling and internal arbitration | Upper-layer MA |
| Identity, RBAC, correlated approval and authorized direct-command enforcement | Existing deterministic MA core in SDK |
| VI execution, control loops, grants, protection and revocation | Existing platform FA |
| Sensing/tracking, stores, release, weapon autonomy and communication equipment | Existing Mission Systems |
| Cross-entity time, motion, collisions, fuzes and damage transactions | Existing World/session simulation |
| Independent episode ownership, actor-safe snapshots and command dispatch | New environment layer over existing session/SDK boundaries |
| Reward, team outcome, termination and centralized critic inputs | Range/referee training side |
| Cameras, visual effects and capture scheduling | Viewer/acceptance tools |

The environment may invoke only existing permission-scoped execution paths for actor actions. It must not assign consent directly, select tactical targets, reproduce weapon physics or expose Range through an actor endpoint.

World extraction is limited to concrete ownership seams needed for endpoint binding or episode lifecycle. Seeker calibration, aircraft cards, fuze/damage parameters, control gains, materials and shadows are outside this plan.

## 5. Selected isolation approach

Three approaches were considered:

| Approach | Trade-off |
| --- | --- |
| Extend the existing flight VecEnv in place | Small apparent surface change, but mixes legacy flight shapes, mutable plugin state and combat reset semantics |
| Add battlefield partitions to the existing shared World | Potential throughput benefit, but requires correct partitioning across sensors, seekers, damage, links, entities, events and recording |
| Add independent mission episodes, initially one World per episode | Clear correctness boundary and fresh-reset reference; higher setup/memory cost can be measured |

Select independent mission episodes. Preserve flight VecEnv as the existing flight-training API.

A mission episode owns its World, platform/C2 handles, local task judge, report/message/event cursors and RNG context. The first reset implementation destroys and reconstructs that episode World from a deterministic scenario recipe. Immutable assets may be reused; mutable Host, Guidance, Damage, DMS, MaCore and task objects are not shared between episodes.

Use seed streams derived from base_seed, stable env_id, local_episode_index and stable stream/service/component identifiers. Another environment's reset count, worker schedule or component registration order must not change a stream.

A reset produces a new generation. Old endpoint handles, target/activity/request references and deferred upper-layer intents are rejected by generation checks. Old process connections are closed or explicitly rebound. A small closed/generation control block outlives the World while child handles exist; parent destruction closes it before releasing the World. Child access fails before dereferencing an expired World. Moving a live parent preserves its identity. Existing Vehicle.reset and MA retirement/history contracts remain unchanged.

Configuration includes an explicit worker budget, publication and recording settings. The initial training baseline uses one worker per World and publication off. Batch concurrency has a bounded global budget. Diagnostic publication names and recording paths include env ID and generation so two episodes do not collide or overwrite each other.

The known crewLost omission in legacy vehicle reset is documented separately. Fresh mission-world reconstruction prevents its carryover here; changing legacy reset behavior would require a separate behavior-fix contract and tests.

## 6. MA service binding

New training scenarios explicitly configure:

- A service ID and its Range-minted controller on each platform/system.
- The service assigned each command capability.
- One DMS ingress owner and one outward communication owner per platform/system.
- Activity ownership and the permitted update/cancel path.

For a single composite MA, one gateway identity is sufficient for many internal components. For multiple MA services on one platform, capabilities are allocated explicitly; sharing reports does not grant command ownership.

The configured path must not depend on Agent creation order. Duplicate exclusive bindings are rejected at initialization. C2 designations and FA revocation apply to the configured owners.

Keep the legacy unconfigured path compatible. New mission episodes always require explicit binding; legacy last-created-controller behavior is documented as a compatibility limitation rather than normalized silently.

## 7. Actor-safe reports and equipment parity

Snapshots contain only reports available through the bound Agent:

- Navigation and own-platform reports.
- Local detections/tracks and received/fused link tracks, without truth identifiers.
- Contacts, inventory, release activities and launcher-known equipment/support state.
- Peer reports, arrived messages and own send statuses.
- C2 tasks, authorization outcomes, grants, direct-command ownership, overrides, lost-link and package state.
- Public mission data intentionally delivered to that MA.

Every report carries source/provenance, sample time, receive/report time, age, availability and validity. Preserve unknown, stale and unavailable values instead of substituting truth or zero.

Post-release weapon telemetry is exposed only if an actual modeled equipment report path supplies it. Initial v1 may legitimately report flyout details as unavailable. Do not synthesize seeker lock, holding entity, true missile position, terminal miss distance or kill confirmation from global referee records. Additional weapon telemetry equipment is a separate feature.

Add own-platform contacts, countermeasure inventory/dispense and own-designation launch-zone methods to C++/Python Agent with the same C permission rules. Keep global weapon, decoy, seeker-truth and engagement-event APIs referee-only.

## 8. Versioned training contract

Use a new API family rather than changing VecEnvPlugins virtual interfaces. New SDK headers and native adapters own this family. v1 uses validated UTF-8 JSON documents for heterogeneous observations/actions across C++, C and Python; this is a local SDK contract, not an ASK/UCI wire format. JSON construction runs at decision boundaries, never inside the native control loop. Use the existing native JSON facilities and a schema document; do not introduce a generic serialization framework.

Encode all 64-bit identity/correlation fields as unsigned decimal strings. Unknown numeric reports use JSON null plus validity/availability, never NaN. Binary DMS data uses bounded base64, with limits checked against decoded bytes and existing DMS limits. Omitted optional action fields explicitly retain the existing native hold/default semantics; unknown report values are not silently accepted as numeric commands.

### Observation document

ma.observation.v1 contains episode key, service/system identity, tick, simulation time, legal report collections, messages, authority/tasks and command receipts. Collections include counts, validity/availability and timestamps. Action masks may depend only on the actor's reported knowledge and explicit authority.

Freeze one gateway snapshot per decision tick. All internal components using that gateway see the same snapshot. Re-reading it is non-consuming. The gateway ingests DMS once; legacy Agent.receive remains consuming.

Observation construction never accepts VehicleState, Referee or arbitrary TaskState.custom/lastReward. Referee data has a separate API/document and is not attached to actor infos.

### Action document

ma.actions.v1 contains episode generation, service_id, tick and a list of requests. Each request has request_id, kind, operation, parameters and optional upper-layer component provenance. The service is authenticated by the endpoint; a document cannot select a new sender, source or controller.

Supported families:

| Family | Existing execution path |
| --- | --- |
| Flight NEW/UPDATE/CANCEL | Agent VI command/activity APIs |
| Sensor operations and local/link designation | Agent Mission Systems and designation APIs |
| Strike and countermeasure | Agent submitStrike/cancelMission and safe equipment APIs |
| P2P send/publish/subscribe | Agent DMS APIs; existing native-topic restrictions |
| MA task accept/reject/progress/end | Agent c2Tasks/acceptTask/rejectTask/reportTask |
| C2 task/control/override/consent/package | Role-scoped Agent C2 APIs |
| Policy activation, where configured | New typed authorized policy message |
| Wait | Advance with no newly submitted request |

Each kind has a fixed parameter schema and maps to one SDK operation. Missing required fields, unknown kinds, non-finite numbers, wrong roles, expired generations and unauthorized targets are rejected explicitly. Target references are own local/link designations, not arbitrary entity truth IDs.

Requests are edge-triggered. Within one episode/service, retrying the same request ID and identical body returns its original submission receipt without repeating side effects; reusing that ID with a different body is rejected. IDs are monotonically issued. Completed-request retry receipts/tombstones use a configured retention window; older retries are rejected, never re-executed. Pending/executing native correlations remain until explicit completion, cancellation or timeout, independently of that window. A configured maximum in-flight count produces an explicit capacity refusal rather than losing live associations.

Upper-layer traces own proposed/arbitrated decisions. SDK receipts distinguish queued submission, native accepted/rejected, pending, executing and terminal states, preserving VI/MS/DMS/C2 reasons and native correlation IDs. The SDK does not attest to an unsubmitted internal proposal or choose which MA component wins arbitration.

### Timing

The scenario fixes physics dt, native frameSkip and an integer number of native World steps per MA decision boundary. Components may have different decision periods in the upper layer. Freeze the snapshot, run components, arbitrate, enqueue requests, close the decision tick, dispatch selected native operations, advance physics, then build the next snapshot.

submit validates and queues; it does not execute native commands immediately. Range.step closes the current tick and dispatches by stable service ID then request ID. A missing endpoint submits no new action; accepted activities continue under native semantics. Late, future and stale-tick submissions are rejected. Reset/step/submission have one serialized Range scheduling boundary; concurrent callers receive explicit busy/lifetime failure rather than racing the World. Proxy arrival order cannot determine command ownership.

Timeouts, report aging and mission windows use simulation time. Viewer wall-clock throttling and captures do not affect training.

## 9. Bounded authorization profile

Keep current RBAC/default strike policy behavior when no new profile is configured.

Introduce training.authorization.v1 for the new mission path. Initial supported criteria are:

- Existing authenticated role/permission and configured strike-authorizer membership.
- Valid own designation and eligible specified stores.
- Policy-selected auto-approve, auto-reject or required approval.
- Request validity interval in simulation time and its episode/activity association.
- Explicitly configured C2 principals allowed to activate a preloaded profile.

Unknown criteria and unavailable required evidence reject profile activation. Do not pretend to implement IFF/hostility matrices, geographic rules or full ASK weapons policy when their observation/model inputs do not exist.

Profiles are loaded by Range; authorized activation/replacement travels through typed DMS messages. Profile changes, cancellation and expiry have explicit effects on pending requests. Approval answers match exactly one pending request, activity, profile revision and episode; late, duplicate or unrelated answers cannot authorize a new strike. Revalidate required legal designation, store availability and any configured equipment evidence at authorization/release commit after delay. Preserve a release transaction once validly committed.

Document implemented, unsupported and project-assumed semantics separately from normative ASK requirements. Do not change existing role-bit values or default role permission tables.

## 10. Upper-layer composition example

Provide a small Python example directory, not an SDK MA framework:

- A gateway obtains one legal snapshot and distributes it internally.
- At least two deterministic policy-component fixtures, a small behavior-tree executive and a planning function produce intents.
- A reference coordinator uses explicit resource ownership, priority and stable tie-breaks to arbitrate.
- Only selected intents are submitted through the gateway. SDK refusals remain authoritative.
- reset/begin_episode/tick/end_episode hooks clear component memory, blackboard, BT status, planner state, internal queues and activity mappings.
- Component-specific RNG streams use stable IDs.

The example demonstrates replacement points for DRL components and different component rates. It imports only installed public fsim interfaces. It does not need PyTorch, a BT package or an RL framework.

Sharing a gateway makes internal components one SDK audit principal. Upper-layer logs retain component decisions and arbitration provenance. Distinct MA services are represented only when Range explicitly configures separate endpoint ownership.

## 11. Batch, roles, outcomes and training

Batch independent MissionEpisodes; do not assume actor_count equals num_vehicles. A C2 endpoint has no flight model. The SDK batch indexes service/role endpoints; upper-layer adapters index their internal components/actors.

Each episode has its own task-judge instance and mutable state. Return episode_done/reason separately from per-role active, terminated, truncated, final_observation and bootstrap eligibility. Individual loss may leave the team episode running. Ending a team episode produces explicit closure for every stopped role.

Next-step autoreset is processed before any old-episode action or physics step. Same-step autoreset preserves final observations and results separately from the new episode's initial snapshot. Upper-layer lifecycle hooks run before a component can act in the new generation.

Reward definitions are versioned and referee-owned. Distinguish release, detonation, actual damage, crew loss, physical incapacitation, collision, friendly damage and mission success. Mission success is a configured task criterion, not an alias for StrikeStatus Done or an explosion image.

Support explicit centralized-training/decentralized-execution records: actor observation/intent/receipt streams and referee/critic streams have separate schemas and access paths. The same-process interface is an accidental-access boundary; evaluation requiring stronger isolation uses a process proxy that exports only the actor endpoint.

Reuse the existing AgentProcess side-channel guard. Strict isolated episodes disable truth publication/recording and refuse Range/referee handles. A diagnostic opt-in for viewer/recording is explicitly labeled non-isolated. Strict isolated trace comparison and fullscreen visual/FSREC acceptance are separate runs; neither is presented as proving the other's boundary.

## 12. Joint acceptance matrix

| ID | Required result |
| --- | --- |
| J1 Composition | Two policy components, BT executive and planner share one legal snapshot; deterministic conflict resolution; one inbox ingestion; only selected actions execute |
| J2 P2P perception | Lead shares a measured track; radar-off wing sees it only after actual delivery, then creates its own legal link designation |
| J3 Peer coordination | Task/resource proposal and acknowledgment survive retransmission without duplicate releases or tasks |
| J4 Typed C2 task | MA accepts/rejects and reports progress/completion; unauthorized role has no execution effect |
| J5 Strike approval | Approved, denied, delayed, expired, canceled and late consent follow the correlated request; approved strike uses native physics/damage |
| J6 Control ownership | C2 direct command/override and FA revocation constrain all MA components/services; cancellation/restoration has the correct owner |
| J7 Link/package change | Report aging, lost-link and restoration follow simulated time; an MA proposes a leader, authorized package changes use protocol messages |
| J8 Episode lifecycle | Finish with weapons, decoys, messages, tasks and policy memory pending; next episode equals a fresh same-seed episode; independent episodes do not interfere |
| J9 Outcome validity | Miss, countermeasure, nonfatal hit, crew loss, collision and friendly damage use actual referee outcomes; no timed self-destruction |
| J10 Public consumers | Installed C++, C99 and Python run the joint chain without internal headers or source-tree DLL/data fallback; isolated actor path has equivalent decisions/results |

Use deterministic fixtures first, then a fixed published seed/aspect matrix. A selected deterministic hit scene is an acceptance fixture, not evidence of general policy quality or calibrated weapon performance.

## 13. Compatibility, delivery and finite exit

Each work package has focused failure-driven tests, a native/binding review and an independent acceptance record. Freeze an old-header consumer binary before implementation and run it against the new library. Preserve existing symbol names/layouts and test supported short C struct sizes.

Use existing ucrt64-headless and ucrt64-release presets. All environments/caches stay under D:/ENV; TMP, TEMP and TORCH_EXTENSIONS_DIR use D:/ENV/tmp. No new branch or worktree; implementation uses main with bounded, separately verified commits.

All viewer tests use fullscreen and finish the full scenario. Strike visual acceptance uses carrier before launch, continuous missile focus after launch and target focus after actual damage. Capture exactly launch + 1.0 s and actual hit + 0.3 s. Never add periodic screenshots or camera changes that influence physics. Preflight referee evidence remains acceptance-only.

Completion requires J1-J10, legacy compatibility, current full headless regression, installed consumer provenance and the full-scenario viewer acceptance. Document measurements and limitations; do not reuse older test totals as current evidence.

At that point stop this architecture program and integrate the real MA system in the upper-layer project.

Deferred work: production MA/BT/planning framework, DRL algorithms/trained policies, distributed rollout infrastructure, shared-World partitions, snapshot/world-pool optimization, new weapon telemetry equipment, full ASK/OMS wire compliance, cryptographic identity, model calibration, materials/shadows and unrelated file splitting.

## References

- [Agent boundary and training principles](../../agent-boundary.md).
- [Public Agent interfaces](../../sdk/agents.md).
- [Existing flight VecEnv](../../sdk/vecenv.md).
- [Native engagement contract](../../sdk/engagement.md).
- [Current viewer validation](../../sdk/viewer-validation.md).
- [CPython 3.13 Limited API and Stable ABI](https://docs.python.org/3.13/c-api/stable.html). Retain the existing non-free-threaded abi3 target and Py_LIMITED_API 0x030B0000; no new Python API version floor is introduced.
- [JSON Schema draft 2020-12](https://json-schema.org/draft/2020-12), used for the documented local contract dialect.
- Local ASK 6.0a VI, MS, C2 and Peer volumes in D:/Projects/Cpp/oms-ref/a-gra/Documentation. These are reference material, not redistributed SDK content.
