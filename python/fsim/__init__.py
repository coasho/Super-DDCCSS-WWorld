"""fsim: the flight simulation platform's Python SDK.

The same platform as the C++ SDK - libfsim.dll, JSBSim vehicles, the
multi-level control stack, effects, communication, the batch layer and
cameras - through a native module built on the C ABI, with the cost of
Python kept out of the way:

* results are numpy arrays that *view* the platform's memory: nothing is
  copied per step, and a VecEnv step is one native call;
* many-vehicle operations are one call (``World.states``, ``World.command``);
* the GIL is released while the platform simulates or renders.

    import fsim
    world = fsim.World("my-experiment")
    red = world.create_vehicle("red-1", latitude_deg=37.62, longitude_deg=-122.38,
                               altitude_msl_m=1500, heading_deg=90, airspeed_ms=60)
    red.command_velocity(airspeed_ms=65, vertical_speed_ms=2, heading_rad=0.0)
    for _ in range(3000):
        world.step()
        s = red.state                 # live, zero-copy: s.altitude_msl_m, s.euler_rad[0], ...

See docs/sdk/python.md.
"""
import os as _os
import sys as _sys

_HERE = _os.path.dirname(_os.path.abspath(__file__))

# The platform's DLLs live in bin/ beside this file (a package) or wherever
# FSIM_DLL_DIR points (a development tree). Python 3.8+ no longer searches
# PATH for an extension module's dependencies, so they are named here.
_dll_directories = []
if _sys.platform == "win32":
    for _dir in (_os.environ.get("FSIM_DLL_DIR"), _os.path.join(_HERE, "bin")):
        if _dir and _os.path.isdir(_dir):
            _dll_directories.append(_os.add_dll_directory(_dir))

from . import _native  # noqa: E402

Error = _native.Error
HOLD = _native.HOLD

_LOG_LEVELS = {"trace": 0, "debug": 1, "info": 2, "warn": 3, "warning": 3, "error": 4, "off": 5}


def set_log_level(level):
    """The platform's own log, which goes to stderr: "trace", "debug", "info",
    "warning", "error" or "off". Quiet by default in Python ("warning"); the
    FSIM_LOG_LEVEL environment variable sets the starting level."""
    value = _LOG_LEVELS[level.lower()] if isinstance(level, str) else int(level)
    _native.set_log_level(value)
    vision = _sys.modules.get(__name__ + "._vision")
    if vision is not None:
        vision.set_log_level(value)


set_log_level(_os.environ.get("FSIM_LOG_LEVEL", "warning"))

from ._state import (  # noqa: E402
    ControlInputs,
    RecordedSample,
    VehicleState,
    control_inputs_dtype,
    recorded_sample_dtype,
    vehicle_state_dtype,
)
from .world import (  # noqa: E402
    MissionActivity,
    MissionCapability,
    MissionRejected,
    AcmPattern,
    AmtiMode,
    AmtiTask,
    AmtiDetection,
    AmtiTrack,
    Contact,
    ContactKind,
    EmitterMode,
    RadarStatus,
    RadioPath,
    RadarCrossSection,
    WeaponRecord,
    WeaponPhase,
    CountermeasureInventory,
    DecoyState,
    EngagementEvent,
    LaunchZone,
    StoreInfo,
    StrikeStatus,
    PayloadInfo,
    PayloadState,
    COMMAND_DEFAULTS,
    COMMAND_FIELDS,
    SETPOINT_DEFAULTS,
    SETPOINT_FIELDS,
    SUPPORT_FIELDS,
    SUPPORT_KINDS,
    MODE_FIELDS,
    MODE_KINDS,
    Activity,
    ActivityBasis,
    ActivityFlag,
    ActivityInfo,
    ActivityProgress,
    ActivityWait,
    Adjustment,
    BatchCommand,
    Finding,
    Rank,
    RequirementKind,
    TaskState,
    TaskStatus,
    PlanCommand,
    PlanCommandResult,
    PlanExecution,
    PlanState,
    PlanStatus,
    InterceptMethod,
    InterceptStatus,
    SegmentStatus,
    PlanValidationResult,
    PathMetadata,
    PointMetadata,
    PointSource,
    RoutePlan,
    Airfield,
    OpPoint,
    OpZone,
    ZoneShape,
    ZoneVertex,
    OpLine,
    LineVertex,
    OpVolume,
    VolumeShape,
    Runway,
    RunwayCoordinates,
    RunwayPoint,
    TimeCriticality,
    TimeWindow,
    Validation,
    AltitudeReference,
    ActivityState,
    Availability,
    AvailabilityInfo,
    Axis,
    Capability,
    CommandedState,
    Contingency,
    ControlMode,
    DirectionReference,
    ControlStatus,
    BezierSegment,
    NurbsSegment,
    CurveZ,
    WaypointType,
    EndBehavior,
    EndPoint,
    EndPointKind,
    Endurance,
    ClimbOptimization,
    Energy,
    Envelope,
    FrameOffsets,
    FrameOrigin,
    FrameRotation,
    HoldContext,
    HoldEntry,
    HoldTurn,
    LIMITS,
    Level,
    LimitStatus,
    MagneticField,
    Message,
    DmsReceived,
    DmsTransmission,
    NetStats,
    PeerReport,
    LinkTrack,
    Role,
    MAService,
    Report,
    ReportCollection,
    ReportItem,
    ReportMeta,
    C2Authority,
    CommandStatus,
    C2Task,
    CommsState,
    TOPIC_LOST_COMMS,
    TOPIC_PACKAGE,
    ActiveOverride,
    ConsentRequest,
    TASK_PATROL,
    TASK_STRIKE,
    TASK_RECONNAISSANCE,
    TASK_ESCORT,
    TASK_TRANSIT,
    TASK_OTHER,
    TASK_RECEIVED,
    TASK_ACCEPTED,
    TASK_REJECTED,
    TASK_COMPLETED,
    TASK_FAILED,
    OVERRIDE_SPEED,
    OVERRIDE_ALTITUDE,
    STRIKE_AUTO_APPROVE,
    STRIKE_AUTO_REJECT,
    STRIKE_REQUIRES_APPROVAL,
    TOPIC_TASK,
    TOPIC_OVERRIDE,
    TOPIC_CONSENT_ANSWER,
    TOPIC_CONSENT_REQUEST,
    PERMISSION_FLIGHT,
    PERMISSION_CONTROL,
    PERMISSION_TASK,
    PERMISSION_WEAPONS,
    PERMISSION_OVERRIDE,
    PERMISSION_SETTINGS,
    PERMISSION_PACKAGE,
    PERMISSION_ROLES,
    ROLE_ADMIN,
    ROLE_QB,
    ROLE_AVC,
    ROLE_LRE,
    ROLE_OBSERVER,
    C2_ACCEPTED,
    C2_REJECTED,
    C2_UNAUTHORIZED,
    C2_COMPLETED,
    C2_FAILED,
    C2_CANCELED,
    TOPIC_OPERATOR,
    TOPIC_CONTROL_DESIGNATION,
    TOPIC_FLIGHT_COMMAND,
    TOPIC_OPERATOR_ROLE,
    TOPIC_COMMAND_STATUS,
    TOPIC_SYSTEM_STATUS,
    TOPIC_POSITION_REPORT,
    TOPIC_ENTITY,
    TOPIC_FREE_FORM,
    DELIVERY_PENDING,
    DELIVERY_DELIVERED,
    DELIVERY_FAILED,
    MustFlyLocation,
    NavigationReport,
    NavigationSettings,
    Parameter,
    PatternKind,
    Performance,
    PerformanceProfile,
    ProfileAcceleration,
    ProfileExcessPower,
    ProfileOrientation,
    ProfilePoint,
    ProfileRates,
    Projection,
    ProtectionMode,
    RangePolicy,
    Rejected,
    RouteLoiter,
    RouteState,
    RoutePath,
    PathType,
    RouteBranch,
    Comparison,
    RouteTerminator,
    PathTerminator,
    Scenario,
    SUPPORT_FEATURES,
    Source,
    SpeedOptimization,
    SpeedReference,
    StateData,
    OwnShip,
    Support,
    SupportInfo,
    TerrainPoint,
    TurnType,
    Vehicle,
    VehicleDefault,
    Waypoint,
    World,
    decimal_year,
    magnetic_field,
)
from .vecenv import VecEnv  # noqa: E402
from .agent import Agent, AgentActivity, Referee  # noqa: E402
from . import agra  # noqa: E402,F401
from .recording import Recording  # noqa: E402
from ._threads import torch_threads  # noqa: E402

__version__, abi_version = _native.version()


def tasks():
    """Task ids a VecEnv can be given (built-ins and registered ones)."""
    return _native.registered_ids(_native.REGISTRY_TASK)


def observations():
    """Observation ids a VecEnv can be given."""
    return _native.registered_ids(_native.REGISTRY_OBSERVATION)


def actions():
    """Action ids a VecEnv can be given."""
    return _native.registered_ids(_native.REGISTRY_ACTION)


_plugins = []  # loaded for good: the registries hold their code


def load_plugin(path):
    """Load a C++ plugin: a DLL that links the SDK and registers tasks,
    observations, actions or controllers from its static initialisers
    (examples/python/plugin). They land in the registries of the libfsim.dll
    this package runs, so a VecEnv created afterwards can name them. Build it
    with the platform's toolchain (MSYS2 UCRT64 GCC) against this version of
    the SDK: the plugin interface is C++.

    Returns the ids the plugin added, as a dict of "tasks", "observations"
    and "actions" (an id it replaced is not new, and loading the same DLL
    again adds nothing). The plugin stays loaded until the process ends."""
    import ctypes

    before = tasks(), observations(), actions()
    _plugins.append(ctypes.CDLL(_os.path.abspath(_os.fspath(path))))
    after = tasks(), observations(), actions()
    return {kind: sorted(set(a) - set(b)) for kind, b, a in zip(("tasks", "observations", "actions"), before, after)}


__all__ = [
    "Agent",
    "AgentActivity",
    "Referee",
    "OwnShip",
    "MissionActivity",
    "MissionCapability",
    "MissionRejected",
    "AcmPattern",
    "AmtiMode",
    "AmtiTask",
    "AmtiDetection",
    "AmtiTrack",
    "Contact",
    "ContactKind",
    "EmitterMode",
    "RadarStatus",
    "RadioPath",
    "RadarCrossSection",
    "WeaponRecord",
    "WeaponPhase",
    "CountermeasureInventory",
    "DecoyState",
    "EngagementEvent",
    "LaunchZone",
    "StoreInfo",
    "StrikeStatus",
    "PayloadInfo",
    "PayloadState",
    "COMMAND_DEFAULTS",
    "COMMAND_FIELDS",
    "SETPOINT_DEFAULTS",
    "SETPOINT_FIELDS",
    "ControlInputs",
    "Error",
    "HOLD",
    "Level",
    "Message",
    "DmsReceived",
    "DmsTransmission",
    "NetStats",
    "PeerReport",
    "LinkTrack",
    "Role",
    "MAService",
    "Report",
    "ReportCollection",
    "ReportItem",
    "ReportMeta",
    "C2Authority",
    "CommandStatus",
    "C2Task",
    "CommsState",
    "TOPIC_LOST_COMMS",
    "TOPIC_PACKAGE",
    "ActiveOverride",
    "ConsentRequest",
    "TASK_PATROL",
    "TASK_STRIKE",
    "TASK_RECONNAISSANCE",
    "TASK_ESCORT",
    "TASK_TRANSIT",
    "TASK_OTHER",
    "TASK_RECEIVED",
    "TASK_ACCEPTED",
    "TASK_REJECTED",
    "TASK_COMPLETED",
    "TASK_FAILED",
    "OVERRIDE_SPEED",
    "OVERRIDE_ALTITUDE",
    "STRIKE_AUTO_APPROVE",
    "STRIKE_AUTO_REJECT",
    "STRIKE_REQUIRES_APPROVAL",
    "TOPIC_TASK",
    "TOPIC_OVERRIDE",
    "TOPIC_CONSENT_ANSWER",
    "TOPIC_CONSENT_REQUEST",
    "PERMISSION_FLIGHT",
    "PERMISSION_CONTROL",
    "PERMISSION_TASK",
    "PERMISSION_WEAPONS",
    "PERMISSION_OVERRIDE",
    "PERMISSION_SETTINGS",
    "PERMISSION_PACKAGE",
    "PERMISSION_ROLES",
    "ROLE_ADMIN",
    "ROLE_QB",
    "ROLE_AVC",
    "ROLE_LRE",
    "ROLE_OBSERVER",
    "C2_ACCEPTED",
    "C2_REJECTED",
    "C2_UNAUTHORIZED",
    "C2_COMPLETED",
    "C2_FAILED",
    "C2_CANCELED",
    "TOPIC_OPERATOR",
    "TOPIC_CONTROL_DESIGNATION",
    "TOPIC_FLIGHT_COMMAND",
    "TOPIC_OPERATOR_ROLE",
    "TOPIC_COMMAND_STATUS",
    "TOPIC_SYSTEM_STATUS",
    "TOPIC_POSITION_REPORT",
    "TOPIC_ENTITY",
    "TOPIC_FREE_FORM",
    "DELIVERY_PENDING",
    "DELIVERY_DELIVERED",
    "DELIVERY_FAILED",
    "RecordedSample",
    "Recording",
    "Scenario",
    "VecEnv",
    "Vehicle",
    "VehicleState",
    "World",
    "abi_version",
    "actions",
    "control_inputs_dtype",
    "load_plugin",
    "observations",
    "recorded_sample_dtype",
    "set_log_level",
    "tasks",
    "torch_threads",
    "vehicle_state_dtype",
]
