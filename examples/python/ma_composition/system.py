"""The reference MA system of the example (docs/sdk/ma-composition.md, sections 2, 4 and 5): one endpoint, one snapshot
per tick shared by every component, one submission per tick, and the lifecycle that clears all of it per generation.

Three records stay apart: what components proposed, what the coordinator decided and why, and what the SDK answered
(its receipts, fed back to the component that originated each request). Only the last is the SDK's."""
import hashlib
import json

from .contracts import MASystem, frozen
from .coordinator import Coordinator

LIVE = ("queued", "pending", "executing")
STARTS = {("flight", "new"), ("sensor", "set_state"), ("sensor", "search")}
PERMITTED_OUTCOME = {"episode_done", "reason"}


def plain(value):
    """A frozen snapshot value back as plain data."""
    if hasattr(value, "items"):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, tuple):
        return [plain(v) for v in value]
    return value


class ReferenceSystem(MASystem):
    def __init__(self, service_id, components, owners=None):
        ids = [c.component_id for c in components]
        if len(set(ids)) != len(ids) or any(i <= 0 for i in ids):
            raise ValueError("component ids are positive and unique within their service")
        self.service_id = str(service_id)
        self.components = sorted(components, key=lambda c: c.component_id)
        self.coordinator = Coordinator(owners)
        self.context, self.outcome = None, None
        self._clear()

    def _clear(self):
        """Everything of a generation: blackboard, queues, the request-id sequence, every request/activity mapping and
        the provenance records."""
        self.blackboard = {"messages": [], "duplicates": 0}
        self._seen_messages = set()
        self.next_request = 1
        self.origin = {}           # request id -> (component id, proposal index, tick, proposal)
        self.activity_owners = {}  # activity id -> component id
        self.delivered = {}        # request id -> what its originator was last told
        self.records = {"snapshots": [], "proposals": [], "decisions": [], "submissions": [], "receipts": []}
        self.reads = 0
        self.last_snapshot = None
        self.outcome = None

    # --- lifecycle --------------------------------------------------------------------------------------------------

    def begin_episode(self, context):
        """A new generation: clear the system's state, then reset every component with its own context - before any
        component may act in it."""
        self._clear()
        self.context = context
        for c in self.components:
            c.reset(context.component(c.component_id))

    def end_episode(self, outcome):
        """The role's permitted mission feedback - a done flag and a reason - never referee truth."""
        extra = set(outcome) - PERMITTED_OUTCOME
        if extra:
            raise ValueError("an MA system is told its permitted mission feedback alone, not %s" % sorted(extra))
        self.outcome = dict(outcome)

    # --- one decision tick ------------------------------------------------------------------------------------------

    def tick(self, endpoint):
        """Read once, hand that one snapshot to every component due this tick, arbitrate, submit once."""
        text = endpoint.observation_json()
        self.reads += 1
        snapshot = frozen(json.loads(text))
        if self.context is None or plain(snapshot["episode"]) != self.context.episode_key:
            # (a snapshot of another generation: nothing computed from it may be sent - begin_episode first)
            raise RuntimeError("composition: a snapshot of another generation; begin_episode before its first tick")
        self.last_snapshot = snapshot
        tick = int(snapshot["tick"])
        body = {k: v for k, v in plain(snapshot).items() if k != "episode"}
        self.records["snapshots"].append([tick, hashlib.sha256(json.dumps(body, sort_keys=True).encode()).hexdigest()])
        self._ingest(snapshot)
        self._feedback(snapshot, tick)

        proposals = []
        for c in self.components:
            if tick % c.period_ticks:
                continue
            for index, p in enumerate(c.propose(snapshot)):
                proposals.append((c.component_id, index, p))
                self.records["proposals"].append([tick, c.component_id, index, p.kind, p.operation, plain(p.parameters),
                                                  list(p.resources), p.priority])

        requests = []
        for cid, index, p, selected, reason in self.coordinator.arbitrate(proposals, self._held(snapshot), self.activity_owners):
            request_id = None
            if selected:
                request_id = self.next_request
                self.next_request += 1
                self.origin[str(request_id)] = (cid, index, tick, p)
                requests.append({"request_id": str(request_id), "kind": p.kind, "operation": p.operation,
                                 "parameters": plain(p.parameters), "component": {"component_id": str(cid)}})
            self.records["decisions"].append([tick, cid, index, reason, request_id])

        document = {"schema": "ma.actions.v1", "episode": self.context.episode_key, "service_id": self.service_id,
                    "tick": snapshot["tick"], "requests": requests}
        ack = json.loads(endpoint.submit_json(json.dumps(document)))
        submission = ack["submission"]
        self.records["submissions"].append([tick, submission["accepted"], submission["reason"],
                                            [[r["request_id"], r["status"], r["reason"]] for r in submission["requests"]]])
        return snapshot

    def _ingest(self, snapshot):
        """The service's messages, ingested once by the SDK into this one snapshot, onto the blackboard once."""
        for m in snapshot["messages"]["items"]:
            key = (m["source_system_id"], m["message_id"])
            if key in self._seen_messages:
                self.blackboard["duplicates"] += 1
                continue
            self._seen_messages.add(key)
            self.blackboard["messages"].append(plain(m))

    def _feedback(self, snapshot, tick):
        """Each receipt that changed goes to the component that originated its request, and to nobody else."""
        due = {}
        for r in snapshot["receipts"]:
            origin = self.origin.get(r["request_id"])
            if origin is None:
                continue
            cid = origin[0]
            native = r["native"] or {}
            told = (r["state"], r["closed"], native.get("status"), r["gateway_reason"])
            if self.delivered.get(r["request_id"]) == told:
                continue
            self.delivered[r["request_id"]] = told
            activity = (native.get("ids") or {}).get("activity_id")
            if (r["kind"], r["operation"]) in STARTS and activity and activity != "0":
                self.activity_owners[activity] = cid
            due.setdefault(cid, []).append(plain(r))
            self.records["receipts"].append([tick, r["request_id"], cid, r["state"], r["closed"], native.get("status"), r["gateway_reason"]])
        for c in self.components:
            if c.component_id in due:
                c.feedback(due[c.component_id])

    def _held(self, snapshot):
        """Resources held by a live activity, by the component whose request started it."""
        held = {}
        for r in snapshot["receipts"]:
            origin = self.origin.get(r["request_id"])
            if origin and (r["kind"], r["operation"]) in STARTS and r["state"] in LIVE and not r["closed"]:
                for resource in origin[3].resources:
                    held[resource] = origin[0]
        return held

    # --- what a run leaves -------------------------------------------------------------------------------------------

    def state(self):
        """The system's and every component's state, as plain data."""
        return {"blackboard": {"messages": len(self.blackboard["messages"]), "duplicates": self.blackboard["duplicates"]},
                "next_request": self.next_request, "origin": sorted(self.origin), "activity_owners": dict(sorted(self.activity_owners.items())),
                "components": {str(c.component_id): c.state() for c in self.components}}

    def trace(self):
        """The generation's provenance and end state: identical inputs give an identical trace."""
        return {"records": json.loads(json.dumps(self.records)), "state": json.loads(json.dumps(self.state(), sort_keys=True)),
                "reads": self.reads, "outcome": self.outcome}
