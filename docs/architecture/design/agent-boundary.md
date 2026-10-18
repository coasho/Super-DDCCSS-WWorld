# ADR-31: Agents, information and authority - the boundary between mission autonomy, the platform and the range

| | |
| --- | --- |
| Status | **Accepted** (the owner, 2026-10-07; section 9). AB-1 to AB-6 done (the migration of section 8 complete) |
| Scope | System-wide: the SDK (C++, the C ABI, Python), the VI (ADR-28/29), Mission Systems (ADR-30), communication (ADR-22/25), the batch layer (`VecEnv`), the environment server (ADR-17) |
| Owner's direction (2026-10-07, summarized in SDK terms) | The boundary between MA, the VI/MS SDK, platform internals and the scenario/referee defines what a DRL policy may observe and control. Ground truth, scenario authority and platform-internal capabilities remain outside the Agent interface; MA decision logic belongs in the upper layer. P2P/C2 information follows modeled sensor/link delivery, and native C2 role, task and grant rules apply in both in-process and process-proxy execution. |
| Reference | A-GRA ASK 6.0a (D:/Projects/Cpp/oms-ref/a-gra): the VI, C2 and Peer interface volumes, the MA L1 compliance document. Read for semantics; nothing of them is in this repository |

## 1. Why now

The platform grew its interfaces one capability at a time, each through the same `World` and `Vehicle` objects. Every
call a mission autonomy (MA) needs is there, but so is every call a scenario, a referee or the platform itself needs, and
nothing tells them apart. A trained policy is shaped by whatever its observation and action plumbing can reach; unintended truth access
found after training invalidates the result. MS-7 is about to add guided weapons and an engagement demo, and later
stages add datalinks and C2: the boundary has to exist before they are built on the current shape.

## 2. What the code does today (audit, 2026-10-07)

| # | Finding | Where |
| --- | --- | --- |
| L1 | One object for every role. `Vehicle` offers MA's calls (VI and MS commands, reports) beside scenario calls (`loadStore`, `loadLoadout`, `failPayload`, `reset`, `addEffect`), configuration (`setAllowed`, `revokeControl`, `setCapabilityPrecedence`, `setControlMode`, `setProtection`), internals (`controls()`, `use()`) and truth (`state()`) | `include/fsim/World.h` |
| L2 | Identity is declared, not given. A caller picks its `Source` (`Policy`, `Autopilot`, `Override`) and its `ControllerId`; a policy can claim `Override` and pre-empt anything, or name another controller | `Capability.h` (`Source`, `Caller`); [flight-autonomy.md](../modules/simulation/flight-autonomy.md) 4.12: "Controllers, like sources, are declared, not authenticated" |
| L3 | Truth inside MA's reports. A radar's detections and tracks carry `truth`, the entity that made them ("for scoring"), in the same struct MA reads | `MissionSystems.h` (`AmtiDetection::truth`, `AmtiTrack::truth`); `python/fsim/world.py` |
| L4 | Other aircraft named by entity id. A strike's target, a behaviour's target, a must fly's `Entity` location, a reference frame following a vehicle and a weapon's record name the simulation's entity; a pursuit, an evasion or a must fly given one flies on that aircraft's true state | `StrikeCommand::target`, `BehaviorCommand::target`, `MustFlyLocation::Entity`, `FrameSpec`, `WeaponRecord::target` |
| L5 | The batch layer's observations are built from truth. `ObservationBuilder::build` receives the vehicle's true `VehicleState`; the built-in `state` observation equals the navigation solution only because navigation is perfect today | `VecEnvPlugins.h` |
| L6 | The message fabric trusts its callers. `Network::send` takes a `Message` whose `from` the caller sets; any node's inbox is readable by anyone holding the `Network`; protocols get the world's truth (`WorldView`) | `Comm.h` |
| L7 | World queries that are the referee's: every vehicle's state, the weapons list, the radio path between any two entities | `World.h` (`radioPath`, `weapons`, vehicle lists) |
| L8 | Platform plug-ins receive truth. Controllers, behaviours, effects and protocols receive `WorldView` (every vehicle's state). An MA implemented as a platform plug-in receives internal access beyond the Agent interface | `Control.h` (`WorldView`, `ControlContext::state`), `ControllerRegistry.h` |
| L9 | One process. A Python or C++ MA in the trainer's process may import Range interfaces. A process proxy keeps Range-owned objects in the parent process and exposes only Agent calls; operating-system file/process access requires separate experiment configuration | - |
| L10 | C2's designations are a direct call. `setAllowed` (AUT-07, "C2 designations") is called by whoever holds the `Vehicle`; there is no C2, no message and no check of who may designate | `World.h`; [flight-autonomy.md](../modules/simulation/flight-autonomy.md) appendix, AUT-07 |
| L11 | Mission data is world-global. Operational points, zones, lines and volumes and reference frames are kept by the world: what one MA defines, every MA reads and flies to (found by AB-1) | `World::setOpZone` and the rest, `World::createFrame` |

L1-L11 identify SDK access findings from this dated audit. Current service binding, Agent permissions, data
visibility and actor process interfaces use the [SDK access terminology](../../sdk/agents.md#sdk-access-terminology).
The A-GRA source terminology in section 3 retains its original meaning; local pipe HMAC checks and SDK service
bindings are described separately.

What is already right and stays: A-GRA's split of MA and FA behind the VI (ADR-28); grants per capability and named
controllers (4.12); FA's behaviours kept as manoeuvres, never tactics (D14); store selection and release mechanics in
Mission Systems, the decision to strike in MA (MS-7a); `ControlContext::sensed` beside `state`.

## 3. A-GRA's control-authority model (verified 2026-10-07)

The owner asked for C2's authority to follow A-GRA's actual model. What the ASK 6.0a volumes say:

| # | A-GRA | Source |
| --- | --- | --- |
| A1 | FA is always the primary controller of every flight capability, as the aircraft's safety-critical system; MA is at most a secondary controller (CAPABILITY_SECONDARY). FA may revoke MA's secondary control at any time | VI 1.2.2.8-1.2.2.9; C2 1.2.7.23 |
| A2 | C2 never addresses FA. The C2-MA interface is off-board (over the air), carried by Mission Systems' Decentralized Messaging Service (DMS) in UCI wrappers; MA authenticates what it receives before processing it | C2 1.1 |
| A3 | C2 decides the control paradigm: it authorizes or revokes MA's command of specific flight capabilities (MA_FlightCapabilityMT, then a ControlRequestMT naming MA the controller and FA the controllee), which MA forwards to FA | C2 1.2.7.23; VI 1.2.2.9 |
| A4 | Direct C2: C2 may send live flight commands (HSA/CSA, waypoint following) to MA, invoking a capability MA has advertised; MA accepts or rejects each, and an accepted one flies under MA's secondary control, with FA's activity reports, until control is taken from MA or MA relinquishes it. Formations go by the task path, not as flight commands | C2 1.2.7.7-1.2.7.9 |
| A5 | Planned C2 (plans and data: activate, deactivate, update, send) and responsive C2 (trigger-and-response rules C2 sets in advance) are the other two paradigms | C2 1.1 |
| A6 | Authority is role-based (RBAC), not a hierarchy. A C2 system declares its role (OperatorMT); roles (Admin, QB, AVC, LRE, Observer, or mission-made ones) carry their permissions in MA_AuthorizationMT: which plans they may activate, which requirements (tasks, actions, capabilities, responses) they may execute, which data they may modify or activate, which requests, teams and settings. Mission planning loads the configuration; only an Admin may change it in flight. Mission Systems authenticates a C2 system when it joins the network | C2 1.2.9, 1.2.9.4, 1.2.9.9 |
| A7 | MA enforces it: a message from a C2 system its role does not authorize is answered with the unauthorized response and not acted on. Among authorized C2 systems, MA executes the last command received; it executes a C2 command or rejects it, and does not ask C2 to approve C2's own command | C2 1.2.9 (intro), 1.2.9.5, 1.2.9.9; MA L1 compliance, C2 requirements |
| A8 | Approval policy: each autonomous action MA itself originates is AUTO_APPROVE, AUTO_REJECT or REQUIRES_APPROVAL (MA_ApprovalPolicyMT), routed to the roles that approve it (MA_ApprovalAuthorityMT). By default the QB role approves every strike MA requests | C2 1.2.9.9 |
| A9 | Release consent: MA enforces the weapons authorization policy, compiled from the engagement criteria C2 or the mission plan sent it, and asks the authorizing C2 for final consent (StrikeConsentRequestMT, answered by its status) | C2 1.2.15.6 |
| A10 | Overrides: C2 may set or clear a platform's or a package's speed, altitude, formation or IFF override, optionally until a time or within zones; MA accepts or rejects it | C2 1.2.12.1 |
| A11 | Lost communication: C2 configures in advance what MA does when it hears nothing for a threshold (an alert, a contingency plan's activation) | C2 1.2.4.3 |
| A12 | Peers: no peer has custody of another. A package's leader checks a C2 command's authorization and relays what its members must do; members accept messages only from their leader, or from a C2 whose role authorizes them. Peer traffic also goes over DMS, each MA authenticating what it receives | Peer 1.1, 1.2.11 |

So, on the owner's question: C2 never takes FA's control and never becomes a primary controller, and nothing happens
automatically. **Explicit control transfer exists in two forms, both through MA:** C2 designating which flight
capabilities MA may command (A3), and C2 commanding the flight itself through MA (A4) - each only where C2's role
authorizes it (A6, A7).

## 4. The principals

Every call into the simulation is made by exactly one of five principals. They are roles in the software, not objects
in the simulated world.

| Principal | Is | May |
| --- | --- | --- |
| **Range** | The experiment's author: a scenario file, a test, a training harness's setup. It also stands for mission planning: it loads each MA's mission data, its RBAC configuration and its nets | Build and change the world: create entities and C2 nodes, load stores, set the weather, inject faults, configure nets and roles; create agents; step the clock |
| **Referee** | The experiment's judge: scoring, rewards, termination, a centralised critic, logging | Read everything, including truth and the labels that tie a track to the entity behind it. Change nothing |
| **Platform** | The simulated equipment of one entity: its flight model, FA behind the VI, its Mission Systems host and payloads, its terminals and DMS, its weapons once released | Read truth only as its physics needs it (a radar's echo comes from the target's true position); expose to its MA only what its models produce |
| **MA core** | The part of A-GRA's MA that applies rules (A6-A12): it checks delivered sender identity and role permissions, forwards C2's control designations to FA, executes authorized direct commands and overrides, applies the approval and weapons authorization policies, runs C2's lost-communication responses and answers unauthorized requests. The SDK World owns this runtime with entity-keyed state | Apply native authority rules on behalf of the bound service; target/task/course-of-action decisions belong in the upper layer |
| **Agent** | A permission-scoped SDK endpoint minted by Range for one platform or C2 entity. Upper-layer scripts, planners, learned actors, human operators or a composite MA use its reports and commands | Read what its bound platform/system and MA core report to it, and act through their permitted interfaces (section 6) |

Current [MaCore state](../../../src/simulation/runtime/services/MaCore.h) is keyed by entity; [World](../../../src/simulation/runtime/world/World.h) owns the runtime.
An upper-layer MA may contain multiple actors, behavior trees, algorithms and internal services behind an endpoint.
Explicit configured service/capability/DMS ownership is the delivered M1 additive interface described in the
[MA integration design](../../superpowers/specs/2026-10-12-ma-system-training-readiness-design.md); creating more Agent handles alone does not configure it.
Its identities, ownership rules and principals are in the
[mission training contract](../../sdk/mission-training.md#9-service-binding), and the binding itself is implemented
(`fsim/MAService.h`, `fsim_world_bind_ma_service`, `World.bind_ma_service`): on a platform the range binds, grants, C2
effects and each side of the DMS follow the bindings. On a platform it binds nothing on, a C2 control designation's
grants, a C2 override applied to a live HSA and a lost-communication plan activation still go to the platform's most
recently created Agent; that legacy behaviour is unchanged.
An Agent's reports also come with their provenance (`fsim/MAReports.h`, `fsim_agent_report`, `agent.report_*`): the
values its reads already give, each with its source, sample and receive times, age and validity, and nothing the
referee alone knows ([legal report calls](../../sdk/mission-training.md#legal-report-calls-noncombat-m2)).

A C2 node is a principal-bearing node like a platform: a compute and communications node with a position and
communication endpoints (the owner, 2026-10-07), not a ground vehicle, so D7 (no ground entities) is unaffected. A C2
node may be off-board (a ground station, a remote operator) or carried by an aircraft (the E-3G, the E-7A). Its agent
reaches MAs only through messages over the simulated links.

## 5. Decisions

### B1. A handle per principal, the agent's minted by the range

- `World` remains the range's object (every existing program keeps working, as a range).
- `Referee` is a read-only view the range hands out: truth, labels, every report of every platform.
- `Agent` is created by the range for one platform or C2 node (`range.createAgent(vehicle, name)`) and handed to the
  agent's program. It offers only the calls in section 6, each scoped to its platform. No call on an agent returns a
  `Vehicle`, a `World`, a `Network` or a `Referee`.
- The same split in C++, the C ABI (an opaque agent handle with its own `fsim_agent_*` functions) and Python
  (`fsim.Agent`).

### B2. Identity comes from the handle

- An agent's commands carry `Source::Policy` and the `ControllerId` its handle was minted with; it cannot choose
  either. `Override` belongs to the range (and to a human operator principal, when one exists).
- A direct C2 command executed by the MA core flies under the same MA's secondary control (A4) and its activity
  records the C2 system and role it came from.
- An agent may hold several controllers (A-GRA's several MA services on one vehicle) only as the range minted them.
- Grants (ADR-28 VI-7, 4.12) keep their meaning; who holds one becomes a fact the platform knows, not a claim.

### B3. Truth in, measurements out

- Everything an agent reads is produced by its platform's models from truth through a modelled process: a sensor, the
  navigation solution, a link. Where the model is perfect today (navigation), the agent still reads it through the
  model's interface, so a later error model reaches it with no change to the agent.
- Truth labels leave the agent's structs: `AmtiDetection::truth` and `AmtiTrack::truth` move to the referee
  (`referee.trackTruth(platform, track)`), which is what scoring uses.
- An agent's clock is the simulation time its platform keeps (an on-board clock); the range may later model its error.

### B4. Agents name things only by their own identifiers

- Other aircraft are named by the agent's track numbers (its platform's tracks), or by network track numbers received
  over a link - never by entity id. A strike's target, an intercept's or a behaviour's target become a track
  reference; the platform resolves it to whatever its tracks estimate, and a pursuit flies on that estimate, not on the
  other aircraft's true state.
- A released weapon is its own platform's: its agent reads it in its strike's status (its id is the weapon's own
  identity, as its vehicle's is - A-GRA's WeaponID - and names nothing an agent may command or read the truth of).
- Plans, activities, stores, payloads, terminals, C2 systems and peers are named by their A-GRA identifiers (SystemID,
  OperatorRoleID, PackageID ...), as the range assigned them.

### B5. Agents act only through commands, their MA core and their own terminals

- VI commands (ADR-28/29), Mission Systems commands (ADR-30), messages sent through their MA core over their
  platform's DMS (B7). Nothing else changes the world on an agent's behalf.
- Plug-ins (controllers, behaviours, effects, protocols, media) are platform equipment and the range's to register.
  An agent cannot register one, and MA logic must not be written as one: a plug-in sees truth (L8).

### B6. No MA decision logic in the SDK

The platform and the MA core execute choices; they never make the mission's. Neither may choose among targets, tracks,
threats, tasks or courses of action. They may carry out a choice given them, apply the rules A-GRA requires of an MA
(A6-A12), and run what A-GRA places in FA or Mission Systems: envelope protection, contingencies and lost-link
procedures, store selection for a strike, a released weapon's own seeker, guidance and fuze. Each new capability is
checked against this rule when it is designed; existing ones were (D14: behaviours are manoeuvres; MS-7a: the strike's
decision is MA's).

### B7. Between platforms, only messages over simulated links

- Whatever one agent learns about another platform - its position, its tracks, its fuel, its intentions - arrives as a
  message through the communication model, or through its own sensors. There is no other path, in process or not.
- Each platform and C2 node has a DMS over its terminals (Mission Systems' comms payloads, ADR-30), on the nets the
  range assigns. A message's sender is stamped by the network from the sending terminal and its system's identity;
  the Agent cannot choose a different sender, read another terminal's traffic, or send on a net its platform is not on.
  Net membership models A2/A6 system admission: Range mission configuration assigns each system its identity and role.
- Delivery goes through the medium: range and terrain masking by the radio path (MS-2), latency, loss, capacity and
  slots, and later jamming (ADR-30's EA). A lost link loses the messages.
- What travels is A-GRA's C2 and peer messages as native types (tasks, task status, plans, flight commands, control
  designations, overrides, approval requests and answers, strike consent, shared vehicle state, tracks, package
  management), plus free-form agent messages (for learned communication) with a size limit, under the same delivery
  model.
- A track received over a link joins the receiver's picture through its own fusion (ADR-30's Data Fusion), so what an
  agent knows of the air is always what its platform has sensed or been told.

### B8. C2 authority as A-GRA has it

The range loads each MA's RBAC configuration (roles, authorizations, approval policies and authorities) and each C2
node's role, as mission planning does. In flight, everything C2 does reaches the subordinate's MA core as messages over
the links, and the MA core enforces it:

| C2 sends | The MA core | The agent |
| --- | --- | --- |
| Anything its role does not authorize | Answers it unauthorized (A7); nothing else happens | Is told it was refused |
| Control designations (A3) | Forwards them to FA, which sets which capabilities MA may command (today's `setAllowed`, now reachable only this way and by the range) | Reads its allowed capabilities and where they came from |
| A direct flight command (A4) | Executes it under its MA's secondary control, answering accepted or rejected (FA's own rejections included); the last authorized command wins (A7) | Sees the activity, marked as C2's (B9) |
| Overrides (A10), engagement criteria and weapons authorization (A9), approval policies (A8), lost-communication responses (A11) | Holds and applies them | Reads them; its own actions are checked against them |
| A task, a plan, a request (A5) | Delivers it to the agent | Decides: accepts, rejects with a reason, reports progress, plans the execution |
| An approval or a strike consent answer | Applies it to the request that asked | Learns the outcome |

- **Local autonomy:** within its allowed capabilities, overrides and policies, the agent decides. What it originates and
  the approval policy says needs approval (a strike, by default) is sent to the approving role by the MA core and waits
  for the answer, which can be delayed or lost with the link.
- **Lost link:** the MA core keeps the last designations, overrides and policies it received and runs C2's configured
  lost-communication response (A11); FA's own lost-link rules stay FA's (ADR-29's CTG-09, FA-16).
- **Teams:** a package leader's agent relays C2's team commands to its members (A12); a member's MA core accepts from its
  leader or from an authorized C2 only. Leader election and the package's internal tasking are the agents' decisions.

### B9. Grants stay MA's; C2 controls through MA

A-GRA's grants are between an MA and the FA of its own vehicle, MA always secondary (A1). A C2 node never holds a
subordinate's grants and never commands its FA. C2's control of a subordinate's flight is A3 and A4 through the
subordinate's MA core, with the link's delay and loss and its role's authorization. While a direct C2 command
flies a capability, the agent's own commands on it are refused until C2 cancels it, it completes, or C2's designations
change (section 9, item 5).

### B10. Training on the boundary (DRL)

- **Actor input:** an observation is built from an `Agent` only. `ObservationBuilder` takes an agent view, never a
  `VehicleState`; the built-in observations are rebuilt on the navigation solution (the same numbers, as the
  platform last reported them: AB-3).
- **Reward, termination, curricula, a centralised critic:** the referee's, and may use truth. Centralised training with
  decentralised execution is allowed and explicit: the critic's input is labelled referee data.
- **Actions:** an `ActionMapper` issues commands through an agent.
- **Several agents:** a multi-agent `VecEnv` gives each agent its own observation and action; agents share nothing
  outside messages (B7). A C2 policy is an agent like any other, at a C2 node.
- An experiment's record names, per agent, what it observed and through which interface (the audit, B11).

### B11. Enforcement in three tiers (the owner, 2026-10-07)

| Tier | What it guarantees | How |
| --- | --- | --- |
| 1. Types | An agent written against an `Agent` cannot reach truth, the range, its MA core's rules or another platform by accident | The handles (B1); truth removed from agent structs (B3); identifiers (B4) |
| 2. Runtime checks | What an agent did is known, and range-only calls fail from it | Each agent call recorded with the agent's identity; a sealed run refuses range calls after it starts unless made on the range's handle; a boundary test classifies every SDK function (C++, C, Python) by principal and fails on an unclassified new one |
| 3. Actor process interfaces | Range-owned simulation objects stay in the parent process; the actor proxy serves only Agent methods | AB-6 uses a local named pipe with an HMAC connection check and typed JSON; ADR-17's proposed shared-memory server was never built. The MA core stays in the simulation process. Operating-system file/process access is configured by the experiment runner. The later OMS bridge (ADR-30, MS-12) is planned to carry A-GRA messages across the same SDK role boundary |

Tiers 1 and 2 hold in every run; tier 3 is used for evaluation and for any result that claims decentralised execution.
An isolated agent's run must reproduce the in-process run to the bit, which is how the boundary itself is tested.

## 6. The agent's interface

| Area | The agent may | Through |
| --- | --- | --- |
| Own ship | Its navigation solution and its quality; its fuel, its stores, its health as its platform reports them | VI navigation report and state data; MS stores, status, BIT |
| Flight | Discover capabilities; request and release its grants; command (NEW, UPDATE, CANCEL), plans, tasks; read its activities and their reports, C2's direct commands among them | VI (ADR-28/29) |
| Sensors and weapons | Discover payloads; command them; read their detections, tracks, statuses; strike (target by track), abort, jettison; follow its weapons' flyout as reported. A strike's consent goes through the approval and weapons authorization policies (A8, A9) | MS (ADR-30), its MA core |
| Communication | Send and receive A-GRA's C2 and peer messages and free-form messages, through its MA core and DMS; read the link picture its platform fused | B7 |
| Authority | Read its allowed capabilities, overrides, policies and engagement criteria and where each came from; as a C2 agent, send what its role authorizes | B8 |
| Mission data | Read what the range loaded before the mission: plans, airfields, zones, known threat data, its RBAC configuration | A mission data load, recorded with the experiment |

Everything else - entity states, other platforms' reports, the world's lists, the radio path between arbitrary
entities, labels, faults, configuration - is the range's or the referee's.

## 7. Between platforms, worked out

| An agent wants | It gets it from |
| --- | --- |
| Where a wingman is | The wingman's shared vehicle state messages, as delivered; else its own radar's track of it |
| What a wingman's radar sees | Track messages the wingman sends, as delivered, fused into its own picture |
| What an AWACS sees | The same, from the AWACS's DMS |
| Whether a wingman has missiles left | Only if the wingman's agent sends a status message saying so |
| What C2 wants | Tasks, plans and commands from C2, as delivered and as C2's role authorizes |
| Whether it may fire | Its weapons authorization policy and engagement criteria, as last received, and the consent answer to its request |
| An adversary's intentions | Nothing but what its sensors show |

## 8. Migration

Stages, each committed and runnable when its tests pass, every existing flight unchanged:

- **AB-1, classification:** every SDK function in C++, the C ABI and Python assigned to range, referee, platform, MA
  core or agent in a table; the boundary test; the docs say which calls an agent may use.
- **AB-2, handles and identifiers:** `Agent` and `Referee` handles minted by the range; source and controller from the
  handle; truth labels moved to the referee; targets as track references (strike, behaviours, must flies, frames);
  weapons reported by strike and serial; mission data each MA's own; no agent object leading back to the world (L1-L6,
  L11, as AB-1's table lists them). Before MS-7c continues (the owner, 2026-10-07), so seekers and guidance are built on
  track references.
- **AB-3, the batch layer on the boundary:** observations from agents, rewards from the referee, a multi-agent
  `VecEnv`; the built-in tasks fly and reward identically (their observations: the platform's report, see Progress).
- **AB-4, links:** DMS per platform and C2 node, nets assigned by the range, stamped senders, agent send and receive,
  shared vehicle state and tracks over the existing link model, then over the comms payloads as ADR-30 builds them.
- **AB-5, C2 and peers:** C2 nodes; RBAC loaded by the range; the MA core's authorization, control designations,
  direct commands, overrides, approval and weapons authorization policies, strike consent over the link, lost
  communication; packages and their leaders.
- **AB-6, isolation:** agents in their own processes over the environment server; the in-process and isolated runs
  identical to the bit.

MS-7's demo (MS-7f) runs on AB-2's handles: a range program builds the engagement and judges it; a separate agent
program holds only its agent. Each later demo with more than one platform uses AB-4/AB-5's links and authority.

### AB-4 in detail: systems, nets and the DMS

A-GRA's model (MS volume 1.2.5; C2 and Peer volumes 1.2): every off-board message between MAs and C2 goes through the
sender's and the receiver's Decentralized Messaging Service. MA hands its DMS a message (`MA_TxDataPayloadCommandMT`)
with optional destinations by SystemID; the DMS publishes it on its topic - named by the message's type - or sends it
to each destination's own topic with reliable delivery, reporting the transmission's status
(`MA_TxDataPayloadCommandStatusMT`); the receiving DMS passes what arrives to MA (`MA_RxDataPayloadMT`) with the
sender's SystemID in its header, once even when it came by two paths, for the topics MA subscribed to
(`MA_MessagingSubscriptionSettingCommandMT`). What a platform publishes and how often is its Data Dissemination Plan.
Peers exchange vehicle state (`MA_PositionReportMT`: position, domain and ground velocity, calibrated airspeed, Mach),
heartbeats (`SystemStatusMT` with its tail number) and their tracks, fused or not (`EntityMT`, 1.2.5.4).

Here:

- **Systems.** Every platform and C2 node that communicates is a system with a SystemID the range assigns (mission
  planning), clear of entity ids, and a tail number; one the range did not name gets the next free id when it first
  joins a net. Agents and messages name systems by SystemID alone; the referee maps a SystemID to its entity.
- **C2 nodes.** A compute and communications node with a position (section 9, item 2): no flight model, no VI or MS.
  The range makes one, puts it on nets, and may make an agent for it - whose calls are its DMS's, its clock, the
  terrain and mission data. Its authority over others is AB-5's.
- **Nets.** The range makes each net (a radio network: AB-4 gives it the existing link model's terms - range, latency,
  jitter, loss - and adds the radio path's line of sight over the terrain and the earth's curvature, MS-2, and a data
  rate shared by its members) and puts systems on it. A message travels on every net its sender shares with a receiver;
  the first arrival is delivered, later copies are not (A-GRA's content filtering). MS-11's terminals replace these
  terms with link budgets, slots and jamming; the interface stays.
- **The DMS.** Sending: a topic, its payload, and optionally destinations. Without destinations it is published: one
  attempt to each system subscribed to the topic. With them it is sent reliably to each: retried while the link is
  down, up to the net's reliable timeout; its status per destination (pending, delivered, failed) is the sender's to
  read. A sender's queue on a net is its share of the net's rate: a message that would wait longer than the net allows
  is dropped. Every message's source is stamped by the DMS from the sending system - an agent sends only as its own
  system, only on nets its system is on, and reads only its own system's inbox.
- **What travels.** Free-form messages - an agent's own bytes on topics of its own, up to the net's size - for learned
  communication; and A-GRA's native types, built by the MA core from its platform's own reports, never from truth and
  never by the agent: position reports (from its own ship's report), heartbeats, and its radar's tracks (each with its
  system and track number). The range's Data Dissemination Plan publishes them periodically; an agent may also send
  them when it chooses. C2's types (tasks, designations, commands, approvals) come with AB-5.
- **The picture.** A receiving system keeps the latest position report of each peer and each received track (by its
  source system and track number), dropped once stale. Its agent reads them, and may designate a received track as a
  target (B4: a network track), flown on its last report carried forward by its velocity - until it lapses.
- **Delivery's physics** reads truth (positions for range and line of sight), as a radar's echo does; nothing of it
  reaches an agent but what arrives. Its random draws have their own stream, so a world without nets flies as before
  to the bit.
- **The referee** maps SystemIDs to entities, a received track to the entity behind its source track, and counts each
  net's traffic: sent, delivered, dropped by range, by line of sight, by loss and by queue.
- **The old fabric** (`comm::Network`, its nodes addressed by vehicle id, its media, protocols and bridges) stays the
  range's: external processes and hardware join through it. An agent's messages move from it to the DMS (L6's last
  part: no entity id in what an agent sends or receives).

Parts: AB-4a, systems, C2 nodes, nets and the DMS with free-form messages; AB-4b, the native types, the Data
Dissemination Plan and the picture, received tracks as designations. Each in C++, the C ABI and Python.

### Progress

- **AB-1, classification (2026-10-07).** Every entry point of the SDK - 1,270: 539 C++ and 389 C exported by the DLLs,
  342 Python - belongs to one principal in [sdk/boundary.toml](../../sdk/boundary.toml), rendered as
  [sdk/boundary.md](../../sdk/boundary.md). The surface is read, not declared: `tools/validation/boundary/boundary.py` takes the DLLs'
  export tables (objdump, c++filt) and imports the Python package (and the calls a world and a vehicle bind per
  instance). ctest's `sdk_boundary` fails on an entry point no rule classifies, on a rule that classifies nothing, on a
  name two rules list, on a call of the role-mixing objects (`World`, `Vehicle`, activities, the network, the cameras)
  classified by a pattern rather than by its name, and on a page out of date. Each agent call that reaches what this
  record keeps from agents carries its access finding: L1 on 3 calls, L2 on 67, L3 on 6, L4 on 69, L5 on 3, L6 on 6, L11 on 33.
  Two access findings were identified: a must fly to an `Entity` and a frame following a vehicle (L4), and world-global mission
  data (L11).

- **AB-2 in four parts.** AB-2a, the handles (closes L1-L3, L5, L6, and L4 by refusal); AB-2b, targets by track (a
  pursuit, a must fly, a strike at a track, flown on its estimate; weapons reported by strike and serial); AB-2c, each
  MA's own mission data (L11); AB-2d, the audit of an agent's calls and the sealed run (tier 2).
- **AB-2a, the handles (2026-10-07).** The range makes an agent for a live aircraft (`World::createAgent`,
  `fsim_agent_create`, `world.create_agent`); the world mints its controller (from 0x40000001). The agent holds its own
  objects - `fsim::Agent`, `fsim.Agent`, an `fsim.AgentActivity` without its world or vehicle - and its C handle is an
  `fsim_world*` in an agent's mode: every C function an `fsim_world*` reaches opens with its guard (refused, or scoped
  to its own vehicle and activities), generated from AB-1's classification and checked against it by `sdk_boundary`,
  so even the objects behind `fsim.Agent` answer nothing else. One set of rules (`simulation/runtime/services/AgentRules`) for the C ABI and
  the C++ `Agent`:
  - **As itself (L2):** the policy's source and its own controller; options or a caller claiming another are refused,
    0 is its own. The per-step `command_*` entry points (the default policy's, without options) are the range's.
  - **Its own vehicle (L1):** another vehicle's calls and activities refused; nothing returned leads to the world.
  - **No entity ids (L4):** a behaviour's or must fly's target and a strike's target naming another aircraft refused -
    by the world itself for any command carrying an agent's controller, whoever makes it (`World::agentRefusal`);
    a frame following another aircraft reads empty. The range's own scripts may still fly on truth.
  - **Consent is C2's (A9):** an agent's strike asks for release consent; it cannot give its own.
  - **Reports, not truth (L3, L5):** detections and tracks with `truth` 0; the whole sensed state is the referee's, and
    an agent reads its own ship from a new report (`ownship`: its navigation and air data through its sensor effects,
    without what no platform reports).
  - **Its own node (L6):** sends stamped from its vehicle's node; its own inbox alone.
  - **The referee** (`fsim::Referee`, `fsim.Referee`): every vehicle's truth, the weapons, radio paths and cross
    sections, the entity behind a track or detection, the agents made.

  Tests: the C++ SDK's (7 cases), the C ABI's (an agent's handle refused the range's calls, scoped, as itself), the
  Python SDK's; `sdk_boundary` (1,497 entry points, every guard). L11 (mission data) stays open until AB-2c. Guide:
  [sdk/agents.md](../../sdk/agents.md); example: `examples/simulation/python/agent.py`.

- **AB-2b, targets by track (2026-10-07).** A vehicle designates one of its radar's tracks as a target (A-GRA's target
  designation, C2 1.2.15.10): `designate(payload, track)` gives a target id from 0x80000001, never an entity's. A
  behaviour's, a must fly's and a strike's target may be it; the world refreshes each designation's estimate from its
  track as each step begins (predicted to then), and a behaviour following it reads that estimate - position, height,
  velocity, its heading and climb - as the target's state, never the aircraft behind it. A dropped track lapses its
  designation, and the behaviour has no target. An agent names its own vehicle's designations alone (the world's
  check, `agentRefusal`); a strike's weapons carry the designation as their target, which the referee resolves
  (`designationTruth`). L4's capability is back for agents, on estimates. C ABI 1.58 (`fsim_vehicle_designate`,
  `_undesignate`, `_designations`, `fsim_world_designation_truth`). Tests: designated, flown on an estimate within 2 km
  of the bandit and not its state, another's designation refused, a must fly over it accepted, struck, undesignated.
  A/B against AB-2a (both built alike, 12 interleaved rounds): a step with 15 aircraft pursuing one by entity +0.11 %,
  with a per-step UPDATE on 16 -0.78 %, three cheap C calls +0.92 % (the state lookup's one compare), all within noise.

- **AB-2c, each MA's own mission data (2026-10-07).** Operational points, zones, lines and volumes, and reference
  frames, are each an owner's: 0, the range's common mission data - every MA reads and names it, the range alone writes
  it - or a vehicle's, its MA's own, which no other vehicle sees. The world keeps them by owner and id; a vehicle sees
  its own, then the common (its own shadowing a common one of the same id), in its reads and in what its commands name,
  through a view of its own (each host's SessionView). Through the C ABI the handle is the owner: an agent's handle reads
  and writes its own, the range's the common; the range loads an MA's own through its agent before handing it over.
  An MA's frame may follow itself or its own designations, never another aircraft, and a common frame following
  another aircraft is not there for a vehicle an MA flies (L4). L11 is closed: every access finding in AB-1's register now counts
  no agent call. Tests: two MAs' zones of one id kept apart, shadowing and removal, must flies naming what each sees, an
  MA's frames (its own, not another aircraft's, not another MA's), the range's frame following another aircraft hidden
  from an MA's vehicle; the C ABI's handles as owners; the Python SDK's. A/B against AB-2b (12 interleaved rounds): 15
  aircraft flying routes in a frame following a 16th -0.06 %, pursuit 0.00 %, a per-step UPDATE -0.14 %, cheap C calls
  +0.02 % - all within noise.

- **AB-2d, the audit and the sealed run (2026-10-07).** Tier 2. Every call an agent makes is counted by name in its
  record - through its C handle (each guard counts it, and each refusal) and through `fsim::Agent` (each member) - and
  the referee reads it (`Referee::audit`, `fsim_world_agent_audit`, `ref.audit()`). The range may seal its handle
  (`fsim_world_seal`, `World.seal()` in Python): the run goes on through the handle the seal gives, and the sealed one,
  with every vehicle and activity got from it, answers nothing but its release - code that kept it, an agent's by
  mistake, can no longer change the world or read its truth. The C++ World is the program's own and is not sealed: a C++
  MA runs as the range's code, in its process (tier 3, AB-6, is what keeps one out). The guards' flags sit beside the
  world reference, merged in one (`restricted`), and a restricted handle's guard runs out of line (cold): the range's
  handle pays one predictable branch a call. A/B against AB-2c (12 interleaved rounds, repeated): steps with per-step
  UPDATEs, pursuits and routes in a moving frame within noise (+0.30 %, 0.00 %, -0.06 %); three cheap C calls +5.7 %
  (0.29 ns): the calls that had no guard - an agent's scope-free calls, `fsim_world_time` among them - now have one, as
  a sealed handle must refuse them too (the two flags read apart cost +10.1 %, the guards inline +6.6 %). Tests: the
  audit's counts and refusals (C++, the C ABI, Python), the sealed handle refused and its successor going on, an
  agent's handle across the seal.
- **AB-3, the batch layer on the boundary (2026-10-07).** Closes L5. `VecEnv` makes an agent per vehicle as it builds
  the batch (each in the referee's `agents()`): its actions are issued as that agent's controller (`World::command(id,
  command, controller)`, the per-step path - an update of the live activity, else a NEW as the agent's, a behaviour or a
  must fly naming another aircraft by its entity id refused), and its observation is built from an `AgentView` alone
  (`ObservationBuilder::build(agent, task, out)`: its own ship as its navigation and air data report it - `OwnShip`,
  which gains the throttle, also in the C ABI 1.60 and Python - its clock, its radar's tracks without `truth`). Tasks
  keep the true state: rewards, termination and a centralised critic are the referee's (B10). Equivalence, the built-in
  tasks and the four action levels over 300 steps with auto-resets (3 envs x 2 vehicles each): rewards, flags and the
  flights unchanged to the bit - with the view fed the true state instead, every digest (observations, rewards, flags,
  final observations) equals AB-2d's; with the report, the observations move by the report's age alone: the report is
  the one the flight controls flew the step's last FDM step on, the state as that step began (1/120 s before the true
  state at the step's end). That is what an agent can know, and with sensor effects the only honest input: a test puts
  altitude noise on one of four vehicles - its observation moves, the others' stay to the bit, every reward stays.
  A/B against AB-2d (12 interleaved rounds, 16 envs x 1 c172x, one worker, 240 steps): a step at the actuator,
  attitude and velocity levels within noise (+0.05 %, +0.17 %, +0.09 %; each side's spread about 1 %). Tests: the
  batch's agents (one each, distinct controllers, their activities theirs), the noisy vehicle's observation against
  its view, the plugin task's rewards from the true state with the observation within the report's age, the own
  ship's throttle (C++, the C ABI, Python).

- **AB-4a, systems, C2 nodes, nets and the DMS (2026-10-07).** Every platform and C2 node that communicates is a system
  named by a SystemID the range assigns (else the next free from 0x53000001), never an entity id; the referee maps
  them (`systemEntity`). C2 nodes (`createC2Node`, ids from 0x60000001) are positions with communication endpoints and
  may have an agent. Nets (`createNet`, `joinNet`) carry a message when the receiver is in range and in line of sight
  - the radio path's (P.528's mode and the terrain's masking, MS-2), worked out per pair each second - and the loss
  draw passes, after the sender's share of the net's rate (the members' even split: an estimate), latency and jitter;
  the defaults are a Link 16 net's published range, band and rate, the rest estimates. Each system's DMS publishes on a
  topic (one attempt per subscriber) or sends reliably to SystemIDs (retried each `retryS` until `reliableTimeoutS`,
  each destination's status kept), stamps the sender, delivers each message once whatever nets carried it, and keeps
  the subscriptions (every topic by default). Delivery has a random stream of its own and runs only when a net exists.
  An agent's messages moved from the vehicle node (`Agent::send`/`inbox` now take and give DMS messages; the C
  `fsim_comm_*` calls are the range's) - L6's last part, an entity id in what an agent sent or received, is closed.
  ABI 1.61: `fsim_world_create_net`, `_join_net`, `_assign_system`, `_create_c2_node`, the agent's `fsim_dms_*`, the
  referee's `fsim_world_system_entity`, `_net_stats`. Equivalence: with no net, the fleet and every test as before;
  the VecEnv digests equal AB-3's. A/B against AB-3 (12 interleaved rounds each): the C calls, per-step UPDATEs, pursuits, routes in a moving frame and
  VecEnv steps at three levels within noise (-0.55 % to +0.20 %; each side's spread 1-2 %). Tests: stamped senders and SystemIDs, refusals (no net,
  a native topic, too large, an unknown destination), published and reliable sends and their statuses, a C2 node's
  agent, the earth's curvature masking two low aircraft 400 km apart (retried, failed at the timeout), a net out of
  range, subscriptions, two nets delivering once, a sender's share dropping what waits too long (C++, the C ABI,
  Python).

- **AB-4b, A-GRA's native messages and the picture (2026-10-07).** Position reports (`MA_PositionReportMT`: the own
  ship as its navigation reports it), heartbeats (`SystemStatusMT`: the tail number) and radar tracks (`EntityMT`: the
  radar's numbering and estimate, never the entity) travel as fixed layouts through the DMS. The MA core builds them
  from the platform's own reports - an agent's free-form bytes may not take a native topic - and publishes them on
  each system's Data Dissemination Plan, which the range sets (`setDissemination`), or when the agent asks
  (`publishPosition`, `publishStatus`, `publishTracks`). A receiving system's picture keeps each peer's last position
  report and heartbeat and each received track (by source system, radar and number), dropped after the plan's
  timeouts (30 s and 12 s by default: estimates). A received track can be designated (`designateLink`: B4's network
  track), flown on its source's last report carried forward by its velocity, lapsing when the picture drops it; the
  referee names the entity behind one (`linkTrackTruth`, and `designationTruth` for its designation). Fusing received
  tracks with the platform's own is ADR-30's Data Fusion, not yet built. Found while running the example: an inbox
  holding only the last world step's arrivals lost what an MA deciding once a second did not read in time - what
  arrives now waits until taken (`receive()`; `inbox()` looks), up to 1,024 (an estimate), the oldest dropped and
  counted (`overflowed`). ABI 1.62. Example: `examples/simulation/python/links.py` - a two-ship, a C2 node and a bandit; the
  wingman, its radar off, is committed by C2's message on the lead's track and pursues it (the referee: its target is
  the bandit). Tests: the plan's reports and heartbeats in a wingman's picture (near the lead's true position), the
  lead's tracks received - the bandit's among them by the referee - a native topic refused to an agent's bytes, a
  received track designated and pursued and lapsing when the lead leaves the net, an inbox waiting until taken (C++,
  the C ABI, Python). A/B against AB-4a (12 interleaved rounds each): the C calls, per-step UPDATEs, pursuits,
  routes in a moving frame and VecEnv steps at three levels within noise (-0.23 % to +0.37 %); the VecEnv digests
  equal AB-3's.

- **AB-5 in three parts.** AB-5a, RBAC and the MA core's authorization, control designations and direct commands;
  AB-5b, C2's tasks to the agent, overrides, the approval policy and release consent over the link; AB-5c, lost
  communication and teams (a package's leader relaying C2's commands to its members).
- **AB-5a, RBAC, control designations, direct commands (2026-10-07).** Each platform's MA core (`session::MaCore`) takes
  the C2 messages its DMS received - topics 10 to 49 go to it, never to the agent's inbox - and authorizes each by the
  role its sender declared (OperatorMT) in the platform's RBAC configuration, which the range loads (A-GRA's five roles
  by default: A-GRA describes them and leaves the exact permission table to its model, so the sets are estimates from
  its descriptions). An unknown role is rejected; a command its sender's role does not permit, or from a sender that
  declared none, is answered unauthorized and not acted on. A declaration is broadcast as the role's systems
  (MA_OperatorRoleMT). Control designations (MA_FlightCapabilityMT with ControlRequestMT) are forwarded to the flight
  controls as A-GRA has them: the control mode Granted, the modes not designated revoked, control of the designated ones
  - and of the platform's own levels, behaviours and effectors, which C2 does not designate - granted to the MA's
  agent. Direct commands (MA_FlightCommandMT: HSA/CSA, waypoints, a cancel) are flown under MA's secondary control as
  C2's - source Autopilot, a controller per C2 system - so the agent's own commands on their axes are refused
  `authority_held` by the platform's arbitration, naming C2's activity, and the agent may not update or cancel it
  (section 9, item 5); a mode C2 revoked is not flown for C2 either. Each command is answered (accepted, rejected,
  unauthorized), and a direct command's end (completed, failed, canceled) reported, back over the link; C2 reads its
  commands' statuses, the agent the authority over it (who declared which role, the designation, C2's commands). ABI
  1.63. Tests: unauthorized before a role and from an Observer, an unknown role rejected, a QB's command flown and held
  against the agent (its own HSA authority_held naming it, its cancel refused), C2's cancel and the agent flying again,
  an AVC's designation (HSA granted, waypoints revoked for the agent and for C2) (C++, the C ABI, Python). Example:
  `examples/simulation/python/c2.py` - a CRC (QB) takes an F-16C south for 40 s while its MA's own commands are held, then gives it
  back; an observer post's command is answered unauthorized. A/B against AB-4b (12 interleaved rounds each): within
  noise (-0.70 % to +0.08 %); the VecEnv digests equal AB-3's.

- **AB-5b, tasks, overrides, release consent (2026-10-07).** Tasks (MA_TaskMT: patrol, strike, reconnaissance, escort,
  transit; a place, a window, a network-track target, remarks) reach the MA's agent through its MA core after the
  sender's role is checked (a strike task needs the weapons permission too); the agent decides - accepts, or rejects with
  a code of its own - then reports progress and completion or failure, each answer going back to C2 with its percent.
  Overrides (MA_PlatformOverrideCommandMT: speed or altitude, until canceled or expired) are held by the MA core and
  flown by the agent's commands: its live HSA updated at once, its next HSA and routes given the overriding values
  (A-GRA leaves how to the implementation; zones not yet). Release consent (C2 1.2.15.6) follows the platform's strike
  policy, part of its RBAC configuration: requires-approval (A-GRA's default) sends a StrikeConsentRequest - the
  strike, its store and count, its target as a track, the launcher's position - to the systems whose declared role
  approves strikes (QB), asked again when one declares if none had; the first authorized answer releases or ends it;
  another role's, or an unasked system's, is unauthorized; auto-approve and auto-reject answer without asking. A
  platform on no net still waits for the range's `strikeConsent`, so every earlier program and test runs as before.
  The MS host lists its strikes awaiting consent (`awaitingConsent`), so the per-step check costs nothing without one.
  Found and fixed on the way: Python's `Referee.weapons()` called the World's `weapons` property. ABI 1.64. Tests: a
  patrol accepted, its progress and completion back to C2, a report before acceptance refused, a strike task from an
  AVC unauthorized, a rejection with its code; an altitude override on the live HSA and the next, its expiry; a strike's
  consent request to the QB alone, an AVC's answer refused, the QB's grant releasing it; auto-reject ending one
  `consent_denied` (C++, the C ABI, Python). The C2 example adds an altitude override its MA's own commands fly until it
  expires. A/B against AB-5a (12 interleaved rounds each): within noise (-0.37 % to +0.31 %); the VecEnv digests equal
  AB-3's.

- **AB-5c, lost communication and teams (2026-10-07).** C2 configures the lost-communication contingency
  (MA_PlanningFunctionSettingsCommandMT's CommsLost trigger with its response, folded into one message): the MA core
  watches when it last heard anything from a system that declared a role to it (the DMS keeps each receiver's last
  delivery from each sender; C2's heartbeats come from its dissemination plan) and, past the threshold, raises the
  contingency - an alert counted, and the route plan the MA uploaded under the given id prepared and activated as its
  own (A-GRA's approved response plan) - re-armed once heard again. Teams (Peer 1.1, 1.2.11): C2 (the package
  permission) sends each member and the leader the team; a member's MA core then accepts what the leader sends as C2's
  (A-GRA: C2's commands relayed by the leader), alongside authorized C2, until dissolved; a leader cannot remake its
  team; leader election and the division of work stay the agents'. ABI 1.65. Tests: the alert and the plan activated
  after C2 leaves the net, re-armed when it returns; a peer's command unauthorized before a team, its leader's accepted
  after, a stranger's refused, the leader's attempt to remake it refused, the dissolution ending it (C++, the C ABI,
  Python). Not yet: approval of autonomous actions other than strikes, role configuration in flight, overrides within
  zones. A/B against AB-5b (12 interleaved rounds each): the C calls, UPDATEs, pursuits and
  routes in a moving frame -0.57 % to -0.37 %; the VecEnv steps first +0.6 % to +1.3 %, then, the two builds swapped
  between the run directories, +0.05 %, -0.06 %, -0.50 % - the difference followed the directory, not the code (with
  no net neither the DMS nor the MA core runs). The VecEnv digests equal AB-3's.

- **AB-6, isolation (2026-10-07).** Tier 3: an MA runs in a process of its own (`fsim.isolation.AgentProcess`), holding
  only a channel to the range's process through which its agent's calls are made - `fsim.Agent`'s and, on the
  activities it holds, `fsim.AgentActivity`'s, exactly `boundary.toml`'s agent calls of those classes (a test compares
  them). B11 named ADR-17's shared-memory server for the channel; it was never built, and a local named pipe with an HMAC connection check
  carrying JSON serves: an agent call costs about 45 us there (0.9 us in one process), some 20,000 a second -
  ample for evaluation - and its messages are data the range checks, never code it runs (nothing is unpickled; only
  fsim's own named tuples, enums and batch commands are rebuilt; an activity is named by id and must be one the agent
  holds). Lockstep: the range asks one MA at a time and serves its calls in order, so the isolated run is the
  in-process run to the bit - `examples/simulation/python/links.py --isolated` against the same engagement in one process:
  every vehicle's final state, the net's traffic and the audit of every call identical. Exceptions cross both ways
  (an agent call's as in one process; the MA's own as `AgentError` with its traceback). A world that is published (its
  shared-memory segment) or being recorded (its file, now reported by `World::recording`, `fsim_world_recording`,
  `World.recording`; ABI 1.66) is refused unless the range says otherwise: any local process could read the truth
  there. Python MAs alone for now - the DRL path; the protocol is language-neutral, and a C or C++ client is a later
  piece. Tests: the links engagement in-process and isolated to the bit; an isolated MA's identity, its attempts to
  reach the world refused, an HSA and its activity used again, a named tuple and bytes exact, a platform refusal and
  its own error crossing, invalid method requests and unowned activity handles answered with errors; types
  outside fsim never rebuilt; a published world refused; the callable-method list against the boundary table. A/B against AB-5c
  (12 interleaved rounds each; the VecEnv cases with the builds swapped between the run directories): within noise
  (-0.32 % to +0.17 %).

## 9. Decisions

Taken (the owner, 2026-10-07):

1. **Enforcement:** tiers 1 and 2 in every run; tier 3 (process isolation) for evaluation.
2. **C2 nodes:** a C2 node is a compute and communications node with a position and communication endpoints, not a
   ground vehicle.
3. **Remote control:** as A-GRA has it, verified against the volumes (section 3): no automatic direct control; C2
   controls a subordinate's flight only by designating MA's capabilities and by direct commands through MA, where its
   role authorizes it.
4. **Order:** AB-1 and AB-2 now, before MS-7c continues.
5. **A direct C2 command and the agent's own commands on one capability.** A-GRA has MA execute an authorized C2
   command (the last command received wins) and does not say whether MA's own autonomy may then replace it. While a
   direct C2 command flies a capability, the agent's own commands on it are refused (`authority_held`, naming the C2
   system) until C2 cancels it, it completes, or C2's designations change. The agent may still reject a C2 *task*,
   with a reason.
6. **Existing programs** keep working as range programs, with the docs marking which calls an agent may use.
