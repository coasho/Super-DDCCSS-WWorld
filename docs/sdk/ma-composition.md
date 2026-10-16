# Composing an MA system over the mission gateway

Status: **frozen contract; noncombat example implemented (M6 noncombat), awaiting independent acceptance**. Generic M0
froze this vocabulary and these example-only interfaces on 2026-10-12 with the
[mission training contract](mission-training.md). The example is `examples/python/ma_composition`
([section 7](#7-the-example-as-implemented-m6-noncombat)). Full composition scenarios consume full M5 (combat extensions)
and are not part of it.

This page is about the upper layer. The formal MA runtime - its actors, behaviour trees, planners, blackboards,
internal services and arbitration - belongs to the upper-layer project, not to the SDK. What the SDK will ship is a
small Python example that shows where such a runtime attaches and what it must reset. It adds no SDK plug-in
interface, no vtable, no dependency on a BT, planning or learning package, and nothing a production system must
inherit from.

## 1. Who is who

| Thing | Count and owner | SDK identity |
| --- | --- | --- |
| Platform or C2 node | The scenario's | An entity id and a SystemID |
| MA service | The configuration's `services` | `service_id`, one Agent, one controller, one endpoint |
| MA system | The upper layer's; uses one or more services | None of its own |
| Component | The upper layer's: an actor, a BT executive, a planner, a rule module, an internal service | `component_id`: provenance only |
| DRL actor | A trainable component | None of its own |

One platform may carry several services, each with explicit capabilities
([service binding](mission-training.md#9-service-binding)). One service may front any number of components: to the
SDK they are one principal, and its audit and receipts name the service. A C2 node's service has no aircraft. No
count follows from another.

## 2. The gateway rule

A gateway is the upper-layer object that holds one endpoint. Per decision tick it:

1. reads the service's snapshot once (`ma.observation.v1`) - reading again returns the same bytes;
2. hands that one snapshot, unchanged, to every component it runs this tick;
3. collects their proposals, arbitrates, and submits the selected requests in one `ma.actions.v1` document;
4. gives each component the receipts of its own requests on later ticks.

Messages are in the snapshot of the service that owns DMS ingress, ingested once by the SDK. A gateway never calls
the consuming `Agent.receive`, and no component drains an inbox.

Inside one platform's boundary, components may share what its services legally read, their own estimates and their
private state. A wrapper that runs several platforms' or C2 nodes' systems in one process keeps their blackboards
apart: what one platform knows reaches another only as a delivered message or a sensor report. Sharing a gateway
never widens what its service may command, and never brings in referee data.

## 3. Example-only interfaces

Proposed for `examples/python/ma_composition/contracts.py`. They are conventions of the example, written against the
installed public `fsim` package alone:

    class MAComponent:
        component_id = 0                  # stable, unique within its service
        period_ticks = 1                  # it proposes on ticks where tick % period_ticks == 0
        def reset(self, context): ...     # a new generation: forget everything
        def propose(self, snapshot): ...  # -> a list of proposals (kind, operation, parameters, resources, priority)
        def feedback(self, receipts): ... # the receipts of the requests this component originated

    class MASystem:
        def begin_episode(self, context): ...  # before any component acts in a generation
        def tick(self, endpoint): ...          # read once, run components, arbitrate, submit once
        def end_episode(self, outcome): ...    # the role's permitted mission feedback

`context` carries the episode key, the service's identity and the seeds of
[the seed streams](mission-training.md#8-seed-streams): the service seed, and for each component its component seed.
`snapshot` is the parsed observation, treated as read-only. `outcome` in `end_episode` is what the range chooses to
tell that role - a done flag and a reason, for instance - never `ma.truth.v1` and never another role's view.

## 4. Arbitration and provenance

The reference coordinator of the example is deterministic and deliberately small:

- every proposal names the resources it would own (flight axes of the platform, a radar, DMS egress, a C2 task);
- a configured owner per resource wins; otherwise the higher configured priority; otherwise the lower
  `component_id`;
- selected proposals get `request_id`s in a strictly increasing sequence the gateway owns for its service and
  generation, and carry `component.component_id`;
- losing proposals are logged, not submitted.

Three records stay separate: what components proposed, what the coordinator selected and why, and what the SDK
answered. Only the last is an SDK receipt. The SDK does not attest to a proposal it never received, does not choose
between components, and its refusals are final: a coordinator may re-plan, it may not retry around FA, MS or C2
authority. A refused or failed request is fed back to the component that originated it.

## 5. Lifecycle

At every new generation, before any component proposes, the system clears: the blackboard, each BT's execution
state, planner state and any recurrent-policy memory, component random streams (re-seeded from the new context),
internal queues, the request-id sequence, and every map from requests to native activities, targets, tasks and
messages. Identifiers of the old generation are never sent again; the SDK would refuse them
(`stale_generation`), and a late intent computed from an old snapshot is dropped by the gateway before that.

Components may run at different periods. The gateway still reads one snapshot per tick and submits one document per
tick; a component that does not run this tick proposes nothing, and what it started continues natively.

## 6. What M6 must show

The example is accepted (J1 of the design) when, on the installed SDK: two policy components, a BT executive and a
planning function share one snapshot object per tick; an inbox is ingested once; conflicting proposals resolve the
same way on every run; only selected requests execute; refusals reach their originators; and a reset leaves no state
of the previous generation. Those are M6's gates, not this page's: this page fixes the words and the seams they are
written against.

## 7. The example as implemented (M6 noncombat)

    cd examples/python
    fsim python -m ma_composition.run [--ticks N] [--trace trace.json]

| File | What it is |
| --- | --- |
| `contracts.py` | Section 3's `MAComponent` and `MASystem`, a `Proposal`, the episode and component contexts with section 8's seed streams computed from the snapshot's episode key, and `frozen`: the tick's snapshot made read-only |
| `components.py` | Two policy fixtures - `NavigationPolicy` (steers on an HSA activity of its own; its last headings stand for a recurrent policy's memory) and `CoordinationPolicy` (reports to C2 over DMS with bytes from its own seeded stream; asks once for a radar its service does not own) - the BT component `TaskExecutive` (accepts a received C2 task, reports progress, then completion) and the planning component `RoutePlanner` |
| `bt.py` | A four-node behaviour-tree executive (condition, action, sequence, selector) whose execution state a reset clears |
| `planner.py` | A pure planning function: the same inputs give the same route |
| `coordinator.py` | Section 4's coordinator: configured owner, then priority, then the lower `component_id`, then proposal order; a live activity's resources stay its component's; an update or cancel addresses only its own component's activity |
| `system.py` | The reference system behind one endpoint: one read and one frozen snapshot per tick for every component due, the service's messages onto the blackboard once, one submission per tick with `component.component_id`, each changed receipt to its originator alone, and the four separate records - proposals, decisions, submissions, SDK receipts |
| `scenario.py`, `run.py` | The range: the contract's two-ship configuration with a C2 node, the system behind the lead's flight service, a scripted C2 using its own endpoint, and a run that compares a reset generation with a fresh episode |

Periods: navigation and the BT every tick, coordination every second, the planner every fourth. In the reference run
the planner's route wins the flight axes over navigation (priority), keeps them while its activity lives
(`held_by`), and re-plans its own route by update; coordination's cancel of that route is never sent
(`not_activity_owner`); navigation's report on the coordination topic loses to its configured owner despite a higher
priority; the radar request is refused by the SDK (`capability_not_bound`) and goes back to coordination alone, which
does not ask again. Nothing in the example decides for the SDK: it submits what it selected and keeps what the SDK
answered.

`python/tests/test_ma_composition.py` (in `python_sdk`) holds these to the SDK under test; `mission_package_consumer`
runs the example twice from a copy against the installed Python package and requires identical traces, the same trace
after a reset as in a fresh episode, and the installed package and library.
