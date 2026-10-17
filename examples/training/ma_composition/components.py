"""The example's components (docs/sdk/ma-composition.md): two noncombat policy fixtures, a behaviour-tree executive and a
planning component. Deterministic stand-ins - each marks the place a learned or engineered component would take -
proposing `ma.actions.v1` request bodies from the one snapshot of the tick, never anything else."""
import base64
import random

from .bt import FAILURE, RUNNING, SUCCESS, Action, Condition, Executive, Selector, Sequence
from .contracts import MAComponent, Proposal
from .planner import bearing_rad, plan_route

LIVE = ("queued", "pending", "executing")


def position(snapshot):
    """The own ship's latitude and longitude as its navigation reports it, or None."""
    navigation = snapshot["reports"]["navigation"]
    own = navigation["own_ship"]
    if navigation["availability"] != "available" or own is None or own["latitude_rad"] is None:
        return None
    return own["latitude_rad"], own["longitude_rad"]


def refusal(receipt):
    """Why the SDK refused a request (its gateway reason or the native reason), or None."""
    if receipt["state"] != "rejected":
        return None
    native = receipt["native"] or {}
    return receipt["gateway_reason"] or native.get("reason") or native.get("status") or "rejected"


class NavigationPolicy(MAComponent):
    """Policy fixture 1: steer toward a goal on an HSA activity of its own, at every tick. Its memory - the last headings
    it chose - stands for a recurrent policy's state."""

    component_id, period_ticks = 1, 1

    def __init__(self, goal):
        self.goal = goal
        self.reset(None)

    def reset(self, context):
        self.seed = context.seed if context else None
        self.activity, self.commanded, self.proposed = None, None, None
        self.memory, self.refusals = [], []

    def propose(self, snapshot):
        here = position(snapshot)
        if here is None:
            return []
        heading = round(bearing_rad(here[0], here[1], *self.goal), 4)
        self.memory = (self.memory + [heading])[-4:]
        self.proposed = heading
        out = []
        if self.activity is None:
            out.append(Proposal("flight", "new", {"mode": "hsa", "heading_rad": heading, "speed": 230.0, "altitude_m": 7000.0},
                                ("flight",), 1))
        elif self.commanded is None or abs(heading - self.commanded) > 0.05:
            out.append(Proposal("flight", "update", {"mode": "hsa", "activity_id": self.activity, "heading_rad": heading},
                                ("flight",), 1))
        if snapshot["tick"] == "3":  # (a high priority does not beat a resource's configured owner)
            out.append(Proposal("dms", "send", {"topic": "1001", "destination_system_ids": ["1392508931"], "byte_length": 1,
                                                "data_b64": "AQ=="}, ("dms:1001",), 9))
        return out

    def feedback(self, receipts):
        for r in receipts:
            if r["kind"] != "flight":
                continue
            if r["operation"] == "new" and r["state"] in LIVE and r["native"]:
                self.activity, self.commanded = r["native"]["ids"]["activity_id"], self.proposed
            elif r["operation"] == "new" and r["closed"]:
                self.activity = None
            elif r["operation"] == "update" and r["state"] == "accepted":
                self.commanded = self.proposed
            if refusal(r):
                self.refusals.append([r["request_id"], refusal(r)])

    def state(self):
        return {"seed": self.seed, "activity": self.activity, "commanded": self.commanded, "proposed": self.proposed,
                "memory": list(self.memory), "refusals": list(self.refusals)}


class CoordinationPolicy(MAComponent):
    """Policy fixture 2: reports to C2 over its DMS every other tick (its bytes from its own seeded stream), asks once for
    the radar its service does not own, and - once - tries to cancel a route another component flies."""

    component_id, period_ticks = 2, 2

    def __init__(self, c2_system):
        self.c2_system = c2_system
        self.reset(None)

    def reset(self, context):
        self.seed = context.seed if context else None
        self.service_id = context.service_id if context else None
        self.rng = random.Random(self.seed)
        self.sent, self.radar, self.refusals = 0, None, []

    def propose(self, snapshot):
        data = bytes(self.rng.randrange(256) for _ in range(4))
        out = [Proposal("dms", "send", {"topic": "1001", "destination_system_ids": [self.c2_system], "byte_length": 4,
                                        "data_b64": base64.b64encode(data).decode("ascii")}, ("dms:1001",), 1)]
        if self.radar is None:
            out.append(Proposal("sensor", "set_state", {"payload": "0", "state": "operate"}, ("radar",), 1))
        if snapshot["tick"] == "6":
            for a in snapshot["reports"]["activities"]["items"]:
                if a.get("capability") == "fsim.guidance.route" and a["owner_service_id"] == self.service_id and a["state"] in ("pending", "active"):
                    out.append(Proposal("flight", "cancel", {"activity_id": a["activity_id"]}, ("flight",), 5))
        return out

    def feedback(self, receipts):
        for r in receipts:
            if r["kind"] == "dms" and r["state"] == "completed":
                self.sent += 1  # (delivered to C2, as its receipt says)
            if r["kind"] == "sensor" and refusal(r):
                self.radar = refusal(r)  # (refused: not proposed again - no retry around the SDK's authority)
            if refusal(r):
                self.refusals.append([r["request_id"], refusal(r)])

    def state(self):
        return {"seed": self.seed, "rng": self.rng.getstate()[1][:4] if self.seed is not None else None, "sent": self.sent,
                "radar": self.radar, "refusals": list(self.refusals)}


class TaskExecutive(MAComponent):
    """The behaviour-tree executive: accepts a C2 task it received, then reports its progress until completed."""

    component_id, period_ticks = 3, 1

    def __init__(self):
        self.tree = Executive(Selector(
            "handle-c2-task",
            Sequence("accept-received", Condition("task-received", self._received), Action("propose-accept", self._accept)),
            Sequence("report-progress", Condition("task-accepted", self._accepted), Action("propose-report", self._report)),
            Action("idle", lambda bb: SUCCESS)))
        self.reset(None)

    def reset(self, context):
        self.seed = context.seed if context else None
        self.tree.reset()
        self.waiting, self.progress, self.refusals = {}, {}, []
        self.blackboard = {}

    @staticmethod
    def _key(task):
        return "%s:%s" % (task["from_system_id"], task["task_id"])

    def _received(self, bb):
        bb["task"] = next((t for t in bb["tasks"] if t["state"] == "received" and self._key(t) not in self.waiting), None)
        return bb["task"] is not None

    def _accept(self, bb):
        task = bb["task"]
        self.waiting[self._key(task)] = "accept"
        bb["proposals"].append(Proposal("task", "accept", {"from_system_id": task["from_system_id"], "task_id": task["task_id"]},
                                        ("c2_task:" + self._key(task),), 3))
        return RUNNING

    def _accepted(self, bb):
        bb["task"] = next((t for t in bb["tasks"] if t["state"] == "accepted" and self._key(t) not in self.waiting), None)
        return bb["task"] is not None

    def _report(self, bb):
        task, key = bb["task"], self._key(bb["task"])
        percent = min(100, self.progress.get(key, 0) + 50)
        self.progress[key] = percent
        self.waiting[key] = "report"
        parameters = {"from_system_id": task["from_system_id"], "task_id": task["task_id"],
                      "state": "completed" if percent >= 100 else "accepted"}
        if percent < 100:
            parameters["percent"] = float(percent)
        bb["proposals"].append(Proposal("task", "report", parameters, ("c2_task:" + key,), 3))
        return RUNNING if percent < 100 else SUCCESS

    def propose(self, snapshot):
        self.blackboard = {"tasks": snapshot["reports"]["tasks"]["items"], "proposals": []}
        self.tree.tick(self.blackboard)
        return list(self.blackboard["proposals"])

    def feedback(self, receipts):
        for r in receipts:
            if r["kind"] != "task" or not r["closed"]:
                continue
            if refusal(r):
                self.refusals.append([r["request_id"], refusal(r)])  # (the task stays waiting: not retried)
            else:
                # answered: the next snapshot shows the task in its new state
                for key, answer in list(self.waiting.items()):
                    if answer in ("accept", "report"):
                        del self.waiting[key]

    def state(self):
        return {"seed": self.seed, "tree": self.tree.state(), "waiting": dict(sorted(self.waiting.items())),
                "progress": dict(sorted(self.progress.items())), "refusals": list(self.refusals)}


class RoutePlanner(MAComponent):
    """The planning component: every fourth tick, a fresh route to the goal from the pure planner - a new route, or an
    update of the one it flies."""

    component_id, period_ticks = 4, 4

    def __init__(self, goal):
        self.goal = goal
        self.reset(None)

    def reset(self, context):
        self.seed = context.seed if context else None
        self.activity, self.plans, self.refusals = None, 0, []

    def propose(self, snapshot):
        here = position(snapshot)
        points = plan_route(here[0] if here else None, here[1] if here else None, self.goal[0], self.goal[1], 7000.0, 3)
        if not points:
            return []
        self.plans += 1
        if self.activity is None:
            return [Proposal("flight", "new", {"mode": "route", "end": "loiter", "waypoints": points}, ("flight",), 2)]
        return [Proposal("flight", "update", {"mode": "route", "activity_id": self.activity, "end": "loiter", "waypoints": points},
                         ("flight",), 2)]

    def feedback(self, receipts):
        for r in receipts:
            if r["kind"] == "flight" and r["operation"] == "new":
                if r["state"] in LIVE and r["native"]:
                    self.activity = r["native"]["ids"]["activity_id"]
                elif r["closed"]:
                    self.activity = None
            if refusal(r):
                self.refusals.append([r["request_id"], refusal(r)])

    def state(self):
        return {"seed": self.seed, "activity": self.activity, "plans": self.plans, "refusals": list(self.refusals)}
