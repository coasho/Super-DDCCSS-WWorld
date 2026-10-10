"""Agents and the referee (ADR-31, docs/agent-boundary.md; docs/sdk/boundary.md).

An :class:`Agent` is a mission autonomy's decisions bound to one aircraft - what an MA may read of its own platform and
do with it, as itself. The range (whoever holds the :class:`fsim.World`) makes one with ``world.create_agent(vehicle,
name)`` and hands it to the MA's code; nothing an agent returns leads back to the world, a vehicle, the network or the
referee. Its handle is the native world's in an agent's mode: the library itself refuses an agent's handle every call
that is not an agent's, scopes it to its own vehicle and commands as its own controller - so even the objects behind
this class answer nothing else. Its commands are the policy's, from a controller of its own: a ``source`` or
``controller`` claiming another is refused, it addresses its own vehicle's activities alone, names no other aircraft by
its entity id (targets by track come with AB-2b), asks for release consent rather than giving it, and reads its
detections and tracks without the entity behind them (``truth`` 0).

A :class:`Referee` is a read-only view of everything, truth among it: for scoring, rewards and a centralised critic.
"""
import math
import types

from .world import (Activity, ActiveOverride, C2Authority, C2Task, CommandStatus, CommsState, ConsentRequest, DmsReceived,
                    DmsTransmission, LinkTrack, NetStats, PeerReport, Vehicle, World)

# The Vehicle calls an agent makes on its own vehicle (docs/sdk/boundary.toml's agent principal), by name.
AGENT_VEHICLE_CALLS = (
    "name", "type", "role", "id",
    # discovery
    "capabilities", "capability_status", "support", "support_table", "availability", "capability_limits", "performance",
    "performance_profile", "capability_precedence", "control_mode", "vehicle_default", "protection",
    # grants
    "request_control", "release_control", "control_status", "control_revision",
    # NEW
    "submit", "submit_behavior", "submit_hsa", "submit_route", "submit_pattern", "submit_marshall", "submit_intercept",
    "submit_must_fly", "submit_curve", "submit_support", "submit_batch", "behavior_finished",
    # its activities, tasks and plans
    "activities", "commanded", "store_task", "command_task", "cancel_task", "remove_task", "task_status", "tasks",
    "publish_plan", "plan_command", "remove_plan", "plan_status", "plans", "plan", "validate_plan", "airfields",
    "airfield",
    # own ship
    "ownship", "navigation_report", "navigation", "set_recovery", "clear_recovery", "set_qnh", "qnh", "state_data",
    "mass_kg",
    # Mission Systems
    "payloads", "payload", "mission_capabilities", "mission_capability_status", "set_payload_state", "payload_bit",
    "cancel_mission", "mission_activity", "mission_activities", "amti_search", "radar_status", "amti_detections",
    "designate", "undesignate", "designations",
    "amti_tracks", "stores", "strike", "jettison", "strike_status",
)
# The World calls an agent makes: its clock, the terrain, its activities by id, mission data read.
AGENT_WORLD_CALLS = (
    "time", "terrain", "activity", "last_command_details", "activity_progress", "activity_setpoint", "intercept_status",
    "end_points", "update", "op_points", "op_point", "op_zones", "op_zone", "op_lines", "op_line", "op_volumes",
    "op_volume", "frame_point",
    # its own mission data (AB-2c): the agent's handle writes its own
    "set_op_point", "remove_op_point", "set_op_zone", "remove_op_zone", "set_op_line", "remove_op_line", "set_op_volume",
    "remove_op_volume", "create_frame", "remove_frame",
)
# An activity's members an agent reads and commands (not its world or vehicle).
AGENT_ACTIVITY_CALLS = (
    "update", "update_route", "command_branch", "update_must_fly", "update_curve", "append", "cancel", "disable",
    "enable", "reset", "delete", "change_rank", "unassign", "setpoint", "intercept_status", "end_points", "info",
    "progress", "state", "live", "clamped", "command_id", "controller", "deferred", "endurance", "id", "level",
    "overridden", "source",
)


def _agents(value, controller=None):
    """What an agent's call returns, as the agent may hold it: an activity without its world or vehicle - its
    controller the agent's own, which the library commanded it as."""
    if isinstance(value, Activity):
        if controller is not None:
            value.controller = controller
        return AgentActivity(value)
    if isinstance(value, list) and value and any(isinstance(v, Activity) for v in value):
        return [_agents(v, controller) for v in value]
    return value


def _delegate(cls, holder, source, name):
    member = vars(source).get(name)
    if isinstance(member, (property, types.MemberDescriptorType, types.GetSetDescriptorType)):
        prop = property(lambda self: _agents(getattr(getattr(self, holder), name)), doc=member.__doc__)
        setattr(cls, name, prop)
        return
    if member is None:  # (bound per instance: World.step and the like are not an agent's)
        raise AttributeError("%s.%s is not a member to delegate" % (source.__name__, name))

    def call(self, *args, **kwargs):
        return _agents(getattr(getattr(self, holder), name)(*args, **kwargs), getattr(self, "controller", None))

    call.__name__ = name
    call.__qualname__ = "%s.%s" % (cls.__name__, name)
    call.__doc__ = member.__doc__
    setattr(cls, name, call)


class AgentActivity:
    """An activity as its agent holds it: what it flies, its progress, UPDATE, CANCEL and the activity commands - not
    the world or the vehicle behind it."""

    __slots__ = ("_a",)

    def __init__(self, activity):
        self._a = activity

    def __repr__(self):
        return "AgentActivity(%d, %r)" % (self._a.id, self._a.level)

    def __eq__(self, other):
        return isinstance(other, AgentActivity) and other._a.id == self._a.id

    def __hash__(self):
        return hash(self._a.id)


for _n in AGENT_ACTIVITY_CALLS:
    _delegate(AgentActivity, "_a", Activity, _n)


class Agent:
    """A mission autonomy's view of its own aircraft and its commands, as a controller of its own (ADR-31).

    Made by the range: ``world.create_agent(vehicle, name)``. ``agent.vehicle_id`` and ``agent.controller`` are its
    identity, minted by the world."""

    __slots__ = ("_world", "_vehicle", "vehicle_id", "controller", "agent_name")

    def __init__(self, range_world, vehicle, name=""):
        vid = vehicle.id if isinstance(vehicle, Vehicle) else int(vehicle)
        handle = range_world._h.create_agent(vid, name)
        self._world = World(range_world.name, _handle=handle)  # (the agent's handle: the library answers an agent alone)
        self._vehicle = Vehicle(self._world, vid)
        self.vehicle_id, self.controller, self.agent_name = handle.agent_identity()

    def __repr__(self):
        return "Agent(%r, vehicle %d, controller %#x)" % (self.agent_name, self.vehicle_id, self.controller)

    # --- messages (AB-4): its system's DMS - stamped with its own SystemID, other systems named by theirs -------------
    @property
    def system(self):
        """Its SystemID (0 until the range puts it on a net)."""
        return self._world._h.dms_system(self.vehicle_id)

    def nets(self):
        """The nets it is on: [(net id, name)]."""
        return self._world._h.dms_nets(self.vehicle_id)

    def send(self, topic, data, destinations=()):
        """Publish (no destinations: to every system subscribed to the topic, one attempt each) or send reliably to each
        destination SystemID: its message number. Its own topics from fsim.TOPIC_FREE_FORM (1000); raises fsim.Error when
        refused (on no net, a native type's topic, too large, a destination no net of its reaches)."""
        return self._world._h.dms_send(self.vehicle_id, int(topic), bytes(data), [int(d) for d in destinations])

    def subscribe(self, topic):
        return self._world._h.dms_subscribe(self.vehicle_id, int(topic), True)

    def unsubscribe(self, topic):
        return self._world._h.dms_subscribe(self.vehicle_id, int(topic), False)

    def subscribe_all(self, on=True):
        """Receive every published topic (the default), or only those subscribed to. What is sent to it always arrives."""
        return self._world._h.dms_subscribe_all(self.vehicle_id, bool(on))

    def subscriptions(self):
        """([topics subscribed to], whether it receives every topic)."""
        return self._world._h.dms_subscriptions(self.vehicle_id)

    def inbox(self):
        """What reached it and waits for it, each once, oldest first (fsim.DmsReceived: its sender's SystemID as
        stamped). Up to 1024 wait (an estimate); then the oldest go."""
        return [DmsReceived(*r) for r in self._world._h.dms_inbox(self.vehicle_id)]

    def receive(self):
        """Take what waits: inbox(), emptied."""
        got = self.inbox()
        if got:
            self._world._h.dms_inbox_take(self.vehicle_id, len(got))
        return got

    def transmission(self, message):
        """A send's status (fsim.DmsTransmission): each destination's [(SystemID, fsim.DELIVERY_*)], how many it reached."""
        t = self._world._h.dms_transmission(self.vehicle_id, int(message))
        return None if t is None else DmsTransmission(*t)

    # --- A-GRA's native messages and the picture (AB-4b) --------------------------------------------------------------
    def _publish(self, topic, payload, destinations):
        first, count = self._world._h.dms_publish(self.vehicle_id, topic, payload, [int(d) for d in destinations])
        return list(range(first, first + count))

    def publish_position(self, destinations=()):
        """Its position report - its own ship as its navigation reports it, built by its MA core: its message numbers."""
        return self._publish(2, 0, destinations)

    def publish_status(self, destinations=()):
        """Its heartbeat (its tail number)."""
        return self._publish(1, 0, destinations)

    def publish_tracks(self, payload=0xFFFF, destinations=()):
        """Its radar's tracks, each an entity report (payload 0xFFFF: every radar's): their message numbers."""
        return self._publish(3, int(payload), destinations)

    def peers(self):
        """Its picture's peers (fsim.PeerReport): each one's last position report and heartbeat, as they arrived."""
        return [PeerReport(*p) for p in self._world._h.dms_peers(self.vehicle_id)]

    def link_tracks(self):
        """The tracks it received over its links (fsim.LinkTrack): their source system's numbering, never the entity."""
        return [LinkTrack(*t) for t in self._world._h.dms_link_tracks(self.vehicle_id)]

    def designate_link(self, source, payload, track):
        """Designate a received track as a target (a network track): its target id (0: not held now), flown on its last
        report carried forward."""
        return self._world._h.designate_link(self.vehicle_id, int(source), int(payload), int(track))

    # --- C2 authority (AB-5a) ------------------------------------------------------------------------------------------
    def authority(self):
        """The authority over its platform, as its MA core keeps it (fsim.C2Authority): who declared which role
        ([(system, role_id, role name)]), C2's designation, and the direct commands C2 flies on it ([(activity, system,
        command_id)]) - while one flies, its own commands on those axes are refused (authority_held)."""
        return C2Authority(*self._world._h.agent_authority(self.vehicle_id))

    def declare_role(self, role_id, callsign="", destinations=()):
        """As a C2 system: declare its role (OperatorMT) to the MAs given (none: to every system on its nets)."""
        return self._world._h.c2_declare_role(self.vehicle_id, int(role_id), callsign, [int(d) for d in destinations])

    def designate_control(self, ma, modes):
        """As a C2 system: the flight modes an MA may command (1 << flight mode each): its command id."""
        return self._world._h.c2_designate_control(self.vehicle_id, int(ma), int(modes))

    def command_hsa(self, ma, heading_rad=None, course_rad=None, speed=None, speed_reference=None, altitude_m=None,
                    altitude_reference=None):
        """As a C2 system: a direct HSA/CSA command (None leaves a field out): its command id."""
        f = [math.nan if v is None else float(v) for v in (heading_rad, course_rad, speed, speed_reference, altitude_m,
                                                           altitude_reference)]
        return self._world._h.c2_command_hsa(self.vehicle_id, int(ma), *f)

    def command_route(self, ma, points):
        """As a C2 system: a direct waypoint command, [(latitude_rad, longitude_rad, altitude_m, speed or None)] (1 to 16):
        its command id."""
        return self._world._h.c2_command_route(self.vehicle_id, int(ma), [(p[0], p[1], p[2], math.nan if len(p) < 4 or p[3] is None
                                                                           else p[3]) for p in points])

    def cancel_command(self, ma, command_id):
        """As a C2 system: cancel a direct command it sent."""
        return self._world._h.c2_cancel_command(self.vehicle_id, int(ma), int(command_id))

    def command_statuses(self):
        """Its commands' last statuses, as the MAs answered (fsim.CommandStatus; state fsim.C2_*)."""
        return [CommandStatus(*s) for s in self._world._h.c2_command_statuses(self.vehicle_id)]

    # --- tasks, overrides, release consent (AB-5b) ------------------------------------------------------------------------
    def c2_tasks(self):
        """As an MA: C2's tasks for it to decide on (fsim.C2Task; state fsim.TASK_*)."""
        return [C2Task(*t) for t in self._world._h.agent_c2_tasks(self.vehicle_id)]

    def accept_task(self, source, task_id):
        return self._world._h.agent_answer_task(self.vehicle_id, int(source), int(task_id), 1, 0, math.nan)

    def reject_task(self, source, task_id, code=0):
        """Reject a received task, with a reason code of its own (sent to C2)."""
        return self._world._h.agent_answer_task(self.vehicle_id, int(source), int(task_id), 2, int(code), math.nan)

    def report_task(self, source, task_id, state=1, percent=math.nan):
        """An accepted task's progress (state fsim.TASK_ACCEPTED, its percent) or its end (TASK_COMPLETED, TASK_FAILED)."""
        return self._world._h.agent_answer_task(self.vehicle_id, int(source), int(task_id), int(state), 0, float(percent))

    def overrides(self):
        """As an MA: C2's overrides in force (fsim.ActiveOverride; kind fsim.OVERRIDE_*) - its HSA and routes fly them."""
        return [ActiveOverride(*o) for o in self._world._h.agent_overrides(self.vehicle_id)]

    def send_task(self, ma, type=0, priority=0, latitude_rad=math.nan, longitude_rad=math.nan, altitude_m=math.nan, radius_m=math.nan,
                  start_s=math.nan, end_s=math.nan, target=None, text=""):
        """As a C2 system: a task for an MA's agent (type fsim.TASK_*; a target a network track (source, payload, track)):
        its task id."""
        src, pay, trk = target if target else (0, 0, 0)
        return self._world._h.c2_send_task(self.vehicle_id, int(ma), int(type), int(priority), float(latitude_rad), float(longitude_rad),
                                           float(altitude_m), float(radius_m), float(start_s), float(end_s), int(src), int(pay), int(trk),
                                           text)

    def set_override(self, ma, kind, value=0.0, expiry_s=math.nan, cancel=False):
        """As a C2 system: an override (fsim.OVERRIDE_SPEED m/s, OVERRIDE_ALTITUDE m) until canceled or expiry_s, or its
        cancel: its command id."""
        return self._world._h.c2_set_override(self.vehicle_id, int(ma), int(kind), bool(cancel), float(value), float(expiry_s))

    def consent_requests(self):
        """As an approving C2 system: the strikes' consent requests it received (fsim.ConsentRequest)."""
        return [ConsentRequest(*c) for c in self._world._h.c2_consents(self.vehicle_id)]

    def answer_consent(self, ma, request_id, granted=True):
        """Answer a consent request it holds: the strike released, or ended."""
        return self._world._h.c2_answer_consent(self.vehicle_id, int(ma), int(request_id), bool(granted))

    # --- lost communication and teams (AB-5c) ------------------------------------------------------------------------------
    def comms(self):
        """As an MA: the lost-communication contingency C2 configured and whether it is raised, and its team
        (fsim.CommsState)."""
        return CommsState(*self._world._h.agent_comms(self.vehicle_id))

    def set_lost_comms(self, ma, threshold_s, plan_id=0):
        """As a C2 system: hearing nothing from C2 for threshold_s, the MA raises a contingency alert - and activates the
        route plan it holds by plan_id (0: the alert alone). Its command id."""
        return self._world._h.c2_set_lost_comms(self.vehicle_id, int(ma), float(threshold_s), int(plan_id))

    def form_package(self, package_id, leader, members):
        """As a C2 system: a team - its members accept what its leader sends as C2's. Its command id."""
        return self._world._h.c2_package(self.vehicle_id, int(package_id), int(leader), [int(m) for m in members], False)

    def dissolve_package(self, package_id, systems):
        return self._world._h.c2_package(self.vehicle_id, int(package_id), 0, [int(m) for m in systems], True)

    def designation_source(self, target):
        """A designation's source system (0: its own radar's track)."""
        return self._world._h.designation_source(self.vehicle_id, int(target))


for _n in AGENT_VEHICLE_CALLS:
    _delegate(Agent, "_vehicle", Vehicle, _n)
for _n in AGENT_WORLD_CALLS:
    _delegate(Agent, "_world", World, _n)
del _n


class Referee:
    """Everything, read only, truth among it: for scoring, rewards and a centralised critic (ADR-31's referee).

    ``world.referee()``. Its answers are the simulation's knowledge, never an agent's: keep them out of an agent's
    observation."""

    __slots__ = ("_world",)

    def __init__(self, world):
        self._world = world

    @property
    def time(self):
        return self._world.time

    def vehicles(self):
        """Every vehicle's id."""
        return list(self._world.vehicle_ids())

    def state(self, vehicle):
        """A vehicle's true state (fsim.VehicleState)."""
        return self._world.vehicle(vehicle).state

    def sensed(self, vehicle):
        """A vehicle's state through its sensor effects."""
        return self._world.vehicle(vehicle).sensed

    def weapons(self):
        """Every weapon released, in flight or ended (World.weapons)."""
        return self._world.weapons

    def radio_path(self, a, b, **options):
        return self._world.radio_path(self._world.vehicle(a), self._world.vehicle(b), **options)

    def radar_cross_section(self, observer, target, **query):
        return self._world.radar_cross_section(self._world.vehicle(observer), self._world.vehicle(target), **query)

    def environment(self):
        return self._world.environment()

    def track_truth(self, vehicle, payload, track):
        """The entity behind a radar track now (0: a false alarm, or a track it no longer has)."""
        for t in self._world.vehicle(vehicle).amti_tracks(payload):
            if t.id == track:
                return t.truth
        return 0

    def detection_truth(self, vehicle, payload, serial):
        """The entity that made a radar detection (0: a false alarm, or one it no longer keeps)."""
        for d in self._world.vehicle(vehicle).amti_detections(payload, serial):
            if d.serial == serial:
                return d.truth
        return 0

    def designation_truth(self, target):
        """The entity behind a designation's track now (0: lapsed, a false alarm, none)."""
        return self._world._h.designation_truth(int(target))

    def audit(self):
        """The audit (AB-2d): every agent's calls - (vehicle, controller, name, [(call, made, refused), ...]) - in the order the
        range made them; a call is a C function's name (what fsim.Agent's calls reach)."""
        return self._world._h.agent_audit()

    # --- systems and nets (AB-4) ------------------------------------------------------------------------------------
    def system_entity(self, system):
        """The entity behind a SystemID (0: none)."""
        return self._world._h.system_entity(int(system))

    def entity_system(self, entity):
        return self._world._h.entity_system(entity.id if isinstance(entity, Vehicle) else int(entity))

    def nets(self):
        """The nets' ids (1 .. count)."""
        return list(range(1, self._world._h.net_count() + 1))

    def link_track_truth(self, source, payload, track):
        """The entity behind a received track: its source radar's track's now (0: none, a false alarm)."""
        return self._world._h.link_track_truth(int(source), int(payload), int(track))

    def net_stats(self, net):
        """A net's traffic (fsim.NetStats)."""
        return NetStats(*self._world._h.net_stats(int(net)))

    def agents(self):
        """The agents the range made: (vehicle, controller, name)."""
        return [(a.vehicle_id, a.controller, a.agent_name) for a in self._world._agents]
