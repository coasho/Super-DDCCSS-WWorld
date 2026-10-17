# Agents and the referee

An **agent** is a mission autonomy's decisions bound to one aircraft - a script, a planner, a trained policy. A
**referee** is a read-only view of everything, truth among it - for scoring, rewards and a centralised critic. Both are
made by the **range**: whoever holds the `World` (ADR-31, [agent-boundary.md](../agent-boundary.md)). Which entry
point belongs to whom is in [boundary.md](boundary.md).

Current capability and milestone scope is tracked in [status](../status.md). Native missile engagement access,
designation requirements and range-only observations are defined in [engagement](engagement.md).

```python
w = fsim.World("engagement")
viper = w.create_vehicle("viper", "jsbsim:f16c", ...)
bandit = w.create_vehicle("bandit", "jsbsim:mig29a", ...)
blue = w.create_agent(viper, "blue-1")    # hand this - not w, not viper - to the MA's code
ref = w.referee()                          # hand this to the reward or the critic

act = blue.submit_hsa(heading_rad=1.2, speed=240.0, altitude_m=7000.0)   # as itself: act.controller == blue.controller
blue.submit_hsa(heading_rad=1.2, source=fsim.Source.OVERRIDE)           # fsim.Error: an agent commands as itself
blue.amti_tracks(0)[0].truth                                            # 0 - the entity behind a track is the referee's
ref.track_truth(viper.id, 0, blue.amti_tracks(0)[0].id)                 # bandit.id
w.step()                                                                 # the range steps the world
```

## What an agent is

- **Made by the range** for a live aircraft: `World::createAgent(vehicle, name)` (C++), `fsim_agent_create` (C),
  `world.create_agent(vehicle, name)` (Python). The world mints its controller (from `0x40000001`): its identity.
- **Its own objects:** `fsim::Agent` and `fsim.Agent` offer an agent's calls alone - its own ship through its reports
  (navigation, state data, mass), discovery, its grants, NEW, its activities (UPDATE, CANCEL, activity commands, a
  route's branches), tasks and route plans, its payloads (state, BIT, search, its detections and tracks), its stores
  (strike, jettison, their status), messages through its system's DMS (AB-4), its clock, the terrain database and mission data. Nothing
  they return leads back to the world, a vehicle, the network or the referee: an activity is an `AgentActivity`,
  without its world or vehicle.
- **Its handle refuses the rest:** in C an agent's handle is an `fsim_world*` in an agent's mode, and the library
  itself - every C function, checked by `sdk_boundary` - refuses it what is not an agent's call (`FSIM_NOT_ALLOWED`, or
  nothing: NULL, "", 0, NaN; `fsim_last_error()` says why) and scopes the rest to its own vehicle. `fsim.Agent` is
  built on such a handle, so even the objects behind it answer nothing else.

## Targets by track

An agent names another aircraft only through its own radar's tracks: it **designates** one (A-GRA's target
designation), and the designation's id - from `0x80000001`, never an entity's - is what its behaviours, must flies
and strikes name.

```python
track = min(blue.amti_tracks("radar"), key=lambda t: t.range_m)
target = blue.designate("radar", track.id)       # 0 if the radar does not hold that track now
blue.submit_behavior("pursuit", target=target)   # flown on the track's estimate
ref.designation_truth(target)                    # the referee: the entity behind it
blue.undesignate(target)
```

- **Flown on the estimate:** as each step begins the world reads the track - predicted to that time - and a behaviour
  following the designation reads it as the target's state: its position, height, velocity, the heading and climb of
  its velocity. Never the aircraft behind it: the track's errors are the pursuit's.
- **It lapses with its track:** a track the radar drops ends the designation; the behaviour has no target, as one whose
  target was removed (a pursuit holds its heading and speed).
- **Its own:** a designation is its vehicle's; another vehicle's agent may not name it. The range may designate too
  (`Vehicle::designate`), and its own scripts may still name entities.
- **A strike** may name its designation: its weapons' records carry it as their target (native missile seekers are cued by
  it), and the referee resolves it.

## Mission data

Operational points, zones, lines and volumes, and reference frames, are each an owner's:

- **The range's common mission data:** what the range sets through the World - every MA reads it and names it in its
  commands; only the range changes it.
- **An MA's own:** what it sets through its agent (`agent.set_op_zone`, `create_frame` ...), and what the range loads
  for it the same way before handing the agent over (its mission data load). No other MA sees it, reads it or can name
  it: two MAs may keep a zone 2 each, different ones.
- **What a vehicle sees** - in its reads and in its commands' names - is its own, then the common: its own shadows a
  common one of the same id, and removing its own uncovers it. It cannot remove the common.
- **Frames:** its own may be fixed, moving, or follow itself or one of its designations - never another aircraft (that
  is its true pose). A common frame following another aircraft is not there for a vehicle an MA flies.

```python
w.set_op_zone(fsim.OpZone(1, "ellipse", ...))   # the range: common - every MA reads it
blue.set_op_zone(fsim.OpZone(2, "ellipse", ...)) # blue's own: red does not see it
blue.op_zones()                                  # zones 1 and 2
red.op_zone(2)                                   # None
blue.submit_must_fly(location="op_zone", target=2)  # (named as it sees it)
```

## The rules an agent is held to

| Rule | What is refused (NotAllowed) |
| --- | --- |
| As itself (L2) | options or a caller claiming another source than the policy's, or another controller than its own (0 means its own) |
| Its own vehicle (L1) | another vehicle's calls, another vehicle's activities (an activity of another reads empty) |
| No entity ids (L4) | a behaviour's target, a must fly's `Entity`, a strike's target naming another aircraft by its entity id, or another vehicle's designation: an agent names its own designations (targets by track). A frame following another aircraft reads empty |
| Consent is C2's (A9) | a strike giving its own release consent: it asks; the MA core routes approval through configured C2 roles over the links. For a platform outside a net, the range supplies consent directly. See [C2 authority](#c2-authority) |
| Reports, not truth (L3, L5) | the entity behind a detection or a track (`truth` is 0); the whole sensed state (an agent reads its navigation report and state data) |
| Its own system (L6) | sending as another system, reading another's inbox, the range's message fabric (`fsim_comm_*`); a native message type's topic (the MA core's) |
| The first entry points | the per-step `command_*` calls (the default policy's, without options): an agent submits and updates as itself |

The world checks commands from an agent's controller again, whoever makes them: a behaviour, must fly or strike naming
another aircraft by entity id, or a command on another vehicle, is refused even through the range's own `Vehicle`
with the agent's controller in its options. The range's own scripts - controller 0 - may still fly on truth: a
scripted adversary is the range's.

## The audit and the sealed run

- **The audit:** every call an agent makes - through its C handle (and so `fsim.Agent`) or `fsim::Agent` - is counted
  by name in the agent's record, and so is each refusal. The referee reads it (`Referee::audit`, `ref.audit()`,
  `fsim_world_agent_audit`): what each agent touched, through which interface, and what it was refused - for an
  experiment's record, or to catch an MA's code asking for what it should not.
- **The sealed run:** before handing agents over, the range may seal its handle: `range = w.seal()` (C:
  `fsim_world_seal`). The World returned is the range's from then on - the same world, its agents going on - and the
  old one answers nothing, nor do vehicles and activities got from it: code that kept them, an agent's by mistake, can no
  longer change the world or read its truth. Keep the new one out of agents' reach. The C++ World remains unsealed;
  MA code must receive its scoped Agent rather than the range's World. [Process isolation](#isolation) is implemented
  by `fsim.isolation.AgentProcess` for runs requiring the stronger boundary.

```python
blue = w.create_agent(viper, "blue-1")
range_ = w.seal()                       # from now on, the range's World
range_.step()                           # w.step() raises fsim.Error: sealed
range_.referee().audit()                # [(vehicle, controller, "blue-1", [("fsim_vehicle_submit_mode", 2, 1), ...])]
```

## Links

Between platforms there are only messages over simulated links (ADR-31, B7; AB-4). A-GRA's model: each platform and
C2 node has a Decentralized Messaging Service, and messages go DMS to DMS.

- **Systems:** every platform and C2 node that communicates is a system, named by the SystemID the range assigned it
  (`world.assign_system(entity, system, tail_number)`, else the next free one when it first joins a net) - never an
  entity id. An agent reads its own (`agent.system`); the referee maps one to its entity (`system_entity`).
- **C2 nodes:** `world.create_c2_node(name, latitude_deg, longitude_deg, altitude_m)` - a compute and communications node
  with a position, no flight model. `world.create_agent(node)` gives it an agent: its DMS, its clock, the terrain and
  mission data. Its authority over others uses the implemented [C2 authority](#c2-authority) rules (AB-5).
- **Nets:** `world.create_net(name, **spec)` and `world.join_net(entity, net)` - the range's (mission planning). A net
  carries a message when the receiver is within its range, in line of sight (the radio path over the earth's curvature
  and the terrain, MS-2), and its loss draw passes, after its sender's share of the net's data rate, the latency and
  the jitter. The defaults are a Link 16 net's published range (300 NM), band and rate (57.6 kbit/s); latency, jitter,
  queue, retry and size limits are estimates (`fsim/Comm.h`). MS-11's terminals replace these terms.
- **Sending** (`agent.send(topic, data, destinations=())`): without destinations it is published - one attempt to each
  system subscribed to its topic; with them it goes reliably to each, retried until the net's timeout, its status per
  destination in `agent.transmission(message)`. The DMS stamps the sender's SystemID: an agent sends as its own system
  alone, on its own nets. Its topics start at `fsim.TOPIC_FREE_FORM` (1000); below are A-GRA's native types, which the
  MA core sends from its platform's own reports (below). A message larger than its nets allow is refused.
- **Receiving:** what reaches it - each message once, however many nets carried it, with its sender's SystemID, topic,
  net and times - waits in its inbox until it takes it: `agent.receive()` takes them, `agent.inbox()` looks. Up to 1,024
  wait (an estimate of a DMS's buffer); then the oldest go, and the referee counts them (`overflowed`). An MA that
  decides once a second reads everything that arrived meanwhile. It receives every published topic unless it chose
  (`subscribe_all(False)`, `subscribe(topic)`); what is sent to it always arrives.
- **A-GRA's native messages** (Peer 1.2.5.4, 1.2.6.2, 1.2.6.3): position reports (its own ship as its navigation
  reports it), heartbeats (its tail number) and its radar's tracks (each an entity report: its numbering and estimate,
  never the entity). The MA core builds them from the platform's own reports - an agent cannot send one of its own
  bytes - and publishes them on the range's Data Dissemination Plan (`world.set_dissemination(entity,
  position_period_s=1, status_period_s=10, tracks_period_s=2)`), or when the agent asks (`publish_position`,
  `publish_status`, `publish_tracks`, optionally to destinations).
- **The picture:** a receiving system keeps each peer's last position report and heartbeat (`agent.peers()`) and each
  track it received (`agent.link_tracks()`: its source system, the source radar's numbering, the report's estimate and
  time), dropped once older than its plan keeps them (30 s and 12 s by default: estimates). A received track can be a
  target: `agent.designate_link(source, payload, track)` - a network track (B4), flown on the source's last report
  carried forward by its velocity, lapsing when the picture drops it. The referee names the entity behind one
  (`link_track_truth`).
- **The referee:** `net_stats(net)` - each net's transmissions, deliveries, duplicates, and the attempts lost to range,
  line of sight, loss and a full queue.

```python
net = w.create_net("l16-a")                          # the range: a net, who is on it, what each publishes on its own
for e in (viper, wingman, c2):
    w.join_net(e, net)
w.set_dissemination(viper, position_period_s=1.0, tracks_period_s=2.0)
lead, wing = w.create_agent(viper, "blue-1"), w.create_agent(wingman, "blue-2")
lead.send(1001, b"bandit 270 30")                    # published: each system subscribed
w.step(30)
[(m.source, m.data) for m in wing.receive()]         # [(lead.system, b"bandit 270 30")] - a SystemID, not an entity id
t = wing.link_tracks()[0]                            # (the lead searching) its radar track, as its last report said
target = wing.designate_link(t.source, t.payload, t.track)
wing.submit_behavior("pursuit", target=target)       # flown on the report, carried forward
```

`examples/simulation/python/links.py` runs a two-ship, a C2 node and a bandit: the lead's MA searches, its MA core shares the
tracks; the C2 node's MA watches the picture and sends the wingman a "commit" naming a track; the wingman's MA, its
radar off, designates the received track and pursues it.

The range's own fabric (`World.network()`, `fsim_comm_*`: nodes by vehicle id, media, protocols, UDP bridges) stays
the range's - external processes and hardware join through it; an agent's handle is refused it.

## Isolation

The handles (tier 1) and the audit and seal (tier 2) keep an agent's code from reaching the world by accident; a process
of its own keeps it out even if it tries (ADR-31, B11 tier 3; AB-6) - for evaluation, and any result that claims
decentralised execution.

```python
import fsim.isolation
w = fsim.World("eval", publish=False)                  # (published or recorded: refused - see below)
...
agent = w.create_agent(viper, "blue-1")
with fsim.isolation.AgentProcess(w, agent, "my_policy:Policy", path=["policies"]) as ma:
    for _ in range(steps):
        ma.step()                                       # Policy(agent).step() in its process; its calls come back here
        w.step(30)
    ma.call("summary")                                  # any of its methods
```

- **Its process holds one thing:** a channel - a named pipe, authenticated - through which its `agent` (a
  `fsim.isolation.RemoteAgent`) makes `fsim.Agent`'s calls, and an activity's calls on the activities it holds. The
  range serves each in its own process on the real agent. Nothing else is reachable: no world, vehicle, referee or
  network object exists in the MA's process. What it may call is exactly `boundary.toml`'s agent calls of
  `fsim.Agent` and `fsim.AgentActivity` (a test checks it).
- **Data, never code:** messages are JSON with typed values - numbers (exact: shortest round-trip form), strings,
  bytes, lists, tuples, dicts, fsim's named tuples, enums and batch commands, NumPy arrays of numbers, an activity by
  id. Nothing is unpickled on either side; the range rebuilds fsim's own types alone.
- **The same run to the bit:** the range asks one MA at a time and serves its calls in order, so the world sees the
  same calls in the same order as with the MA in its own process: `examples/simulation/python/links.py --isolated` flies the
  same engagement as without, to the last bit of every state, the net's traffic and the audit (a test checks it).
- **Errors cross:** an agent call's exception is raised in the MA's process as it would be in one (fsim.Error,
  fsim.Rejected, MissionRejected, a builtin); the MA's own uncaught exception reaches the range as
  `fsim.isolation.AgentError`, with its traceback.
- **Side channels:** a published world's shared-memory segment and a recording's file hold every vehicle's truth and
  any local process may read them: `AgentProcess` refuses a world that is published or recorded
  (`allow_side_channels=True` overrides it). Files the range keeps elsewhere, and the machine itself, are the
  experimenter's to keep from the MA (another user, a container).
- **Its cost:** about 45 us an agent call over the channel (0.9 us in one process): some 20,000 calls a second.

C and C++ MAs run at tiers 1 and 2 (their handle and `fsim::Agent`); the channel's protocol - JSON over a
length-prefixed byte stream, method names as `fsim.Agent`'s - is the one a client in another language would speak.

## C2 authority

What C2 may do to a platform (ADR-31, B8 and section 9; AB-5) is A-GRA's: C2 never addresses the flight controls; it
reaches a platform's MA over the links, and the MA core - A-GRA's rules, applied for the agent - authorizes each message
by the role its sender declared.

- **Roles (RBAC):** each platform's configuration is the range's (`world.set_rbac(entity, roles)`, mission planning):
  roles and what each may send (`fsim.PERMISSION_FLIGHT`, `_CONTROL`, `_TASK`, `_WEAPONS`, `_OVERRIDE`, `_SETTINGS`,
  `_PACKAGE`, `_ROLES`). By default A-GRA's five: Admin, QB, AVC (flight and sensors, not weapons), LRE (launch and
  recovery), Observer (nothing). A-GRA describes them and leaves the exact table to its model: the default sets are
  estimates. A C2 system declares its role (`c2.declare_role(fsim.ROLE_QB, "CRC LEAD")`); an MA's agent reads who is who
  (`agent.authority().operators`).
- **Authorized or not:** a command whose sender's role does not permit it - or who declared none - is answered
  `fsim.C2_UNAUTHORIZED` and nothing happens. Among authorized C2 systems the last command wins.
- **Control designations** (`c2.designate_control(ma, modes)`, a bit per A-GRA flight mode): the modes MA may command;
  the MA core forwards them to the flight controls - the vehicle's control mode becomes Granted, the other modes are
  revoked, and control of the designated ones (and of the platform's own levels, behaviours and effectors, which C2 does
  not designate) is granted to the MA's agent. A C2 direct command on a revoked mode is rejected too.
- **Direct commands** (`c2.command_hsa(ma, ...)`, `c2.command_route(ma, points)`, `c2.cancel_command(ma, id)`): the MA
  core flies them as C2's, under MA's secondary control. While one flies, the agent's own commands on its axes are
  refused `authority_held` (naming C2's activity) and it may not update or cancel it - until C2 cancels it, it ends, or
  C2's designation changes (section 9, item 5). Its status - accepted or rejected, then completed, failed or canceled -
  goes back to C2 (`c2.command_statuses()`), and the agent sees it in `agent.authority().commands`.

```python
w.set_rbac(viper, w.rbac(viper))                      # (A-GRA's defaults, as mission planning loaded them)
boss = w.create_agent(crc, "crc")                     # a C2 node's agent
boss.declare_role(fsim.ROLE_QB, "CRC LEAD")
order = boss.command_hsa(blue.system, heading_rad=math.pi, speed=230.0, altitude_m=6500.0)
...                                                   # C2's status: accepted; the agent's own HSA: authority_held
boss.cancel_command(blue.system, order)               # the agent flies again
```

`examples/simulation/python/c2.py` runs it: a CRC takes an F-16C south for 40 s, its MA held, then gives it back and sets an
altitude override its MA's own commands fly; an observer post's command is answered unauthorized.

- **Tasks** (`c2.send_task(ma, fsim.TASK_PATROL, latitude_rad=..., text=...)`; a target as a network track): the agent's
  to decide - `agent.c2_tasks()`, then `accept_task` or `reject_task(source, id, code)` with a reason of its own, then
  `report_task` (its progress, its completion or failure). Each answer goes back to C2 (`command_statuses()`, topic
  `fsim.TOPIC_TASK`, with the percent). A strike task needs the weapons permission besides the task permission.
- **Overrides** (`c2.set_override(ma, fsim.OVERRIDE_ALTITUDE, 7500.0, expiry_s=...)`, or its cancel): the agent's HSA
  and routes fly the overriding speed or altitude in place of their own - its live HSA at once, its next commands as it
  gives them - until C2 cancels it or it expires. The agent reads them (`agent.overrides()`).
- **Release consent:** a strike asks for consent as the platform's policy says (`world.set_strike_policy`, the
  range's): by default the systems whose declared role approves strikes (a QB) are asked over the link
  (`c2.consent_requests()`: the strike, its store and count, its target track, the launcher's position), and the first
  answer (`c2.answer_consent(ma, request, granted)`) releases it or ends it (`consent_denied`); another role's answer is
  unauthorized. `STRIKE_AUTO_APPROVE` and `STRIKE_AUTO_REJECT` answer without asking. A platform on no net waits for the
  range's `strike_consent`, as before.
- **Lost communication** (`c2.set_lost_comms(ma, threshold_s, plan_id)`, C2 1.2.4.3): hearing nothing from any system
  that declared a role to it for the threshold - C2's heartbeats are its dissemination plan's - the MA core raises a
  contingency alert and, given a plan, activates the route plan the MA uploaded under that id, as its own. The agent
  reads it (`agent.comms()`: configured, lost, since when, alerts). Heard again, it is armed for the next loss.
- **Teams** (`c2.form_package(id, leader, members)`, `dissolve_package`; Peer 1.1, 1.2.11): a member's MA core accepts
  what its team's leader sends as C2's - C2's commands relayed - besides authorized C2; a leader cannot remake its team.
  Who leads and how a team divides its work are the agents' decisions.

## MA services

A platform may carry several agents - several MA services, each a controller of its own. By themselves they are not
configured: any of them may command any capability, any of them may take the shared inbox, and a C2 control
designation's grants, a C2 override on a live HSA and a lost-communication plan go to whichever agent was made last.
A **binding** makes the ownership explicit (`fsim/MAService.h`; [contract](mission-training.md#9-service-binding)):

```python
flight, sensors = w.create_agent(viper, "flight"), w.create_agent(viper, "sensors")
w.bind_ma_service(flight, 1, ("fsim.guidance.hsa", "fsim.guidance.route"), owns_dms_ingress=False, owns_dms_egress=True)
w.bind_ma_service(sensors, 2, ("fsim.ms.amti:radar",), owns_dms_ingress=True, owns_dms_egress=False)
w.ma_services()                                  # [MAService(service_id=1, ...), MAService(service_id=2, ...)]
flight.submit_hsa(heading_rad=1.2)               # its capability: flown
sensors.submit_hsa(heading_rad=2.0)              # fsim.Error: not_granted - not its capability
sensors.request_control("fsim.guidance.hsa")     # fsim.Rejected: not_allowed
sensors.send(1001, b"contact")                   # fsim.Error: the DMS egress is the flight service's
sensors.receive()                                # the inbox: the ingress owner's alone (flight.receive() == [])
```

- **The range's:** `bindMAService(world, binding, &error)` (C++), `fsim_world_bind_ma_service` (C),
  `world.bind_ma_service(agent, service_id, command_capabilities, owns_dms_ingress, owns_dms_egress)` (Python). It
  names a controller the world minted for one of the platform's agents and creates none. `maServiceBindings`,
  `fsim_world_ma_service`, `world.ma_services()` read the bindings back in ascending service id.
- **By the agent itself:** two worlds mint the same vehicle ids and controller numbers, so an agent is told from
  another world's only by the world that made it. `bindMAService(world, agent, service_id, capabilities, ingress,
  egress, &error)` (C++), `fsim_world_bind_ma_service_agent(world, agent_handle, &binding)` (C) and Python's
  `bind_ma_service` take the agent, read its platform and controller from it, and refuse an agent of another world
  and anything that is not an agent - nothing changed.
- **Ownership is the bindings':** it does not depend on the order the agents were made in, or the order they were
  bound in. Command capabilities are named by their discovery ids, or `"*"` for all of them; a capability, the DMS
  ingress and the DMS egress each have at most one owner on a platform, and a binding that overlaps another - or names
  a controller that is not one of the platform's own, or a capability the platform does not offer - is refused with
  the reason, and nothing changes.
- **What it uses:** a bound platform is under `ControlMode.GRANTED` and stays there (`set_control_mode` to another:
  refused). Only an owner is granted its capability - not another agent, not the default policy (controller 0), not
  any other controller - so only it commands it; another service reads an activity but may not update or cancel it
  (`authority_held`). C2's designation, overrides and lost-communication plan go to the owners.
- **What stood before gives way:** a grant another controller held when a binding is made, and any policy activity
  that is not its capability's owner's - flown under a grant or, on an open platform, without one - is taken back by
  the platform: the grant revoked, that capability's policy activities ended `revoked`, flight and Mission Systems
  alike, an unowned capability's too. The revoke is of the capability: it ends every controller's policy activities
  of it, so the newly bound service's own go on only on a capability no other controller held or acted on. Bind every
  service of a platform before any of them acts and nothing is taken from anyone.
- **Reports are shared:** every agent of the platform reads its navigation, sensors, picture, tasks and authority
  alike. Sharing a report grants no command.
- **Not bound, not changed:** a platform the range binds nothing on behaves exactly as before. Removing a platform
  ends its bindings; the legacy `reset` keeps them.

`examples/training/python/ma_services.py` runs it: one aircraft, two services, the agents made in either order.

## Legal reports

An agent's reads say what it holds. The report calls say the same things with where each came from, when, and how old
it is (`fsim/MAReports.h`; [contract](mission-training.md#legal-report-calls-noncombat-m2)) - what a policy needs to
tell a fresh measurement from an old delivery:

```python
nav = blue.report_navigation()                   # Report(availability="available", meta, (OwnShip, NavigationReport))
nav.meta                                         # ReportMeta(source="own_navigation", validity="valid", ...)
for item in blue.report_link_tracks(stale_after_s=6.0).items:
    item.meta.system, item.meta.net              # who reported it, over which net
    item.meta.sample_time_s                      # when its source measured it
    item.meta.receive_time_s, item.meta.age_s    # when the link delivered it; how old it is now
    item.meta.validity                           # "stale" once older than 6 s; never without a threshold
    item.value                                   # the LinkTrack itself, as blue.link_tracks() gives it
boss.report_navigation()                         # Report("not_applicable", None, None): a C2 node has none
```

- **Seven reports:** `reportNavigation`, `reportTracks`, `reportContacts`, `reportLinkTracks`, `reportPeers`,
  `reportTasks`, `reportAuthority` (C++); `fsim_agent_report(world, entity, FSIM_REPORT_*, ...)`, which gives the
  metadata item for item beside the existing getters (C); `report_navigation()` ... `report_authority()` (Python).
- **A peer is two messages:** its position report and its heartbeat may come by different nets at different times.
  `report_peers()` gives the provenance of the position each peer holds - the net that report came by and when it
  was delivered - and `report_peer_heartbeats()` (`reportPeerHeartbeats`, `FSIM_REPORT_PEER_HEARTBEATS`) that of its
  last heartbeat, index for index. Neither borrows the other's. `PeerReport.net` and `received_s` themselves are
  unchanged: those of whichever message was heard last.
- **Nothing more than it may know:** the values are those of `ownship`, `amti_tracks`, `contacts`, `link_tracks`,
  `peers`, `c2_tasks` and `authority`, under the same rules - no entity behind a track or a contact, no other
  platform's reports, nothing from the referee. A report its links did not deliver is not there, and one its picture
  dropped is gone.
- **Unknown stays unknown:** a time its source does not state is NaN in C++ and C and None in Python (a task's sample
  time, a peer heard only by its heartbeat); a quantity a sensor does not measure stays NaN (a warning receiver's
  range). Navigation a sensor of its own calls invalid is passed on as reported and marked `invalid`.
- **Availability:** `available`; `unavailable` once its platform is gone; `not_applicable` for what a C2 node does not
  have. Every agent of a platform reads these alike, bound or not.
- **Reading takes nothing.** `agent.contacts(payload)` (`Agent::contacts`) is new with these: the agent's read of its
  payloads' contacts, `truth` 0.

`examples/training/python/ma_reports.py` runs it: a lead sharing its radar's tracks with a wingman, and what each then holds.

## Equipment reports

`countermeasures()`, `dispense(kind)` and `launch_zone(store, designation)` provide the existing own-platform
equipment operations. `report_stores()`, `report_strikes()`, `report_countermeasures()` and `report_consents()` add
availability and provenance without exposing weapon truth. A new strike report's target is an own designation or
zero, and its released weapon IDs are native correlations only. Delivered-consent reports honor DMS ingress
ownership; legacy consent reads keep their platform-shared behavior. See the [equipment contract](engagement.md#agent-equipment-access-and-reports)
for language equivalents, timing, unavailable values and refusal semantics.

## The referee

`fsim::Referee` / `fsim.Referee`: every vehicle's true state (and sensed state), the vehicles, the weapons, radio
paths and radar cross sections between any two, the entity behind a radar track or detection (`trackTruth`,
`detectionTruth`), the agents made. It changes nothing. Its answers are the simulation's knowledge: keep them out of
an agent's observation (a centralised critic may use them in training, as ADR-31's B10 allows).

## Not yet

- Mission episodes: one World per episode, immutable per-tick snapshots and queued requests through an endpoint of
  its own. Their contract is frozen and not built: see the [mission training contract](mission-training.md) and
  [composing an MA system](ma-composition.md). [MA services](#ma-services) and [legal reports](#legal-reports) are
  the parts that are.
- A C or C++ MA in a process of its own (the isolation channel's client in those languages); approval of autonomous
  actions other than strikes (A-GRA's approval
  policy for plans and responses), role configuration in flight (an Admin's), overrides within zones; fusion of received
  tracks
  with its own (ADR-30's Data Fusion) and the comms payloads' link budgets (MS-11).

## In a batch

A `VecEnv` makes an agent per vehicle (AB-3): observations from its `AgentView` (its own ship as reported, its clock,
its radar's tracks), actions as its controller, rewards from the referee
([vecenv.md](vecenv.md#agents-and-the-referee)). Each appears in `Referee::agents()` like one the range made.
