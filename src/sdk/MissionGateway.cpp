// The mission gateway of one generation (MissionGateway.h; docs/sdk/mission-training.md, sections 10 to 13).

#include "sdk/MissionGateway.h"

#include "fsim/MAService.h"
#include "sdk/MissionJson.h"
#include "session/MaCore.h"
#include "session/World.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace fsim::mission {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kHalfPi = 1.5707963267948966, kPi = 3.141592653589793;

// --- requests: one generic operation each, read whole at submit ------------------------------------------------------

enum class Op : std::uint8_t {
    FlightNew, FlightUpdate, FlightCancel,
    SensorState, SensorSearch, SensorCancel, Designate, DesignateLink, Undesignate,
    DmsSend, DmsPublish, DmsSubscribe, DmsUnsubscribe, DmsSubscribeAll,
    TaskAccept, TaskReject, TaskReport,
    C2DeclareRole, C2DesignateControl, C2CommandHsa, C2CommandRoute, C2CancelCommand, C2SendTask, C2SetOverride, C2SetLostComms,
    C2FormPackage, C2DissolvePackage,
    GatewayClose,
    Unsupported, ///< a kind no installed definition handles (section 15)
};
enum class Mode : std::uint8_t { Hsa, Route, Behavior };

/// One request in native form: what its one SDK call takes. Read at submit, so that a request that is queued can be made.
struct Operation {
    Op op = Op::Unsupported;
    Mode mode = Mode::Hsa;
    std::optional<bool> interrupt;
    control::HsaCommand hsa;
    control::RouteCommand route;
    std::vector<control::Waypoint> waypoints;
    std::string behavior;
    std::uint32_t target = 0; ///< a designation's target id
    std::map<std::string, double> params;
    std::uint64_t id = 0;     ///< activity_id, task_id, command_id, request_id (gateway.close), plan_id
    std::uint16_t payload = 0xFFFF;
    ms::PayloadState state = ms::PayloadState::Standby;
    ms::AmtiCommand amti;
    std::uint64_t track = 0;
    comm::SystemId system = comm::kNoSystem; ///< source, addressed MA, sender of a task, package leader
    comm::DmsMessage message;
    int report = 0;           ///< position, status, tracks
    bool enabled = false;
    std::uint32_t number = 0; ///< a task answer's code, a role id, a modes mask, a package id
    comm::ReceivedTask::State taskState = comm::ReceivedTask::Accepted;
    double value = kNaN;      ///< a task's percent, a lost-communication threshold
    std::string text;         ///< a callsign
    std::vector<comm::SystemId> systems;
    comm::TaskOrder task;
    comm::OverrideCommand override;
};

std::string at(const std::string& path, std::size_t i) { return path + "[" + std::to_string(i) + "]"; }

std::vector<comm::SystemId> systemList(const core::Json& v, const std::string& path, std::size_t minItems, std::size_t maxItems) {
    std::vector<comm::SystemId> out;
    const auto& list = arrayOf(v, path, minItems, maxItems);
    for (std::size_t i = 0; i < list.size(); ++i) out.push_back(static_cast<comm::SystemId>(idOf(list[i], at(path, i), 32, true)));
    return out;
}
std::uint16_t payloadOf(Fields& p, std::string_view name) { return static_cast<std::uint16_t>(idOf(p.required(name), p.path(name), 16)); }
comm::SystemId systemOf(Fields& p, std::string_view name) { return static_cast<comm::SystemId>(idOf(p.required(name), p.path(name), 32, true)); }
void number(Fields& p, std::string_view name, double& out, Bounds b = {}) {
    if (const core::Json* v = p.optional(name)) out = numberOf(*v, p.path(name), b);
}
/// A number or null (null: left out).
void nullable(Fields& p, std::string_view name, double& out, Bounds b = {}) {
    if (const core::Json* v = p.present(name)) out = numberOf(*v, p.path(name), b);
}
void choice(Fields& p, std::string_view name, double& out, std::initializer_list<const char*> names) {
    if (const core::Json* v = p.optional(name)) out = static_cast<double>(enumOf(*v, p.path(name), names));
}

#define FSIM_SPEED_REFERENCES {"true_airspeed", "calibrated_airspeed", "ground_speed", "mach"}
#define FSIM_ALTITUDE_REFERENCES {"msl", "above_ground", "ellipsoid", "barometric"}

/// flight.new and flight.update's parameters (flight_command and its operation's refinements).
void flightCommand(Fields& p, Operation& o, bool update) {
    o.mode = static_cast<Mode>(enumOf(p.required("mode"), p.path("mode"), {"hsa", "route", "behavior"}));
    if (update) o.id = idOf(p.required("activity_id"), p.path("activity_id"), 64, true);
    else if (p.has("activity_id")) malformed(p.path("activity_id"), "a new command names no activity");
    if (const core::Json* v = p.optional("interrupt")) {
        if (update) malformed(p.path("interrupt"), "an update does not interrupt");
        o.interrupt = boolOf(*v, p.path("interrupt"));
    }
    switch (o.mode) {
    case Mode::Hsa:
        number(p, "heading_rad", o.hsa.headingRad);
        number(p, "course_rad", o.hsa.courseRad);
        number(p, "speed", o.hsa.speed, atLeast(0.0));
        choice(p, "speed_reference", o.hsa.speedReference, FSIM_SPEED_REFERENCES);
        number(p, "altitude_m", o.hsa.altitudeM);
        choice(p, "altitude_reference", o.hsa.altitudeReference, FSIM_ALTITUDE_REFERENCES);
        choice(p, "speed_optimization", o.hsa.speedOptimization, {"long_range_cruise", "max_endurance"});
        choice(p, "direction_reference", o.hsa.directionReference, {"true_north", "magnetic_north"});
        break;
    case Mode::Route: {
        choice(p, "projection", o.route.projection, {"great_circle", "rhumb"});
        if (const core::Json* v = p.optional("repeat")) o.route.repeat = boolOf(*v, p.path("repeat")) ? 1.0 : 0.0;
        choice(p, "end", o.route.end, {"continue", "loiter"});
        if (const core::Json* v = p.optional("start")) o.route.start = static_cast<double>(integerOf(*v, p.path("start"), 0, 255));
        const auto& points = arrayOf(p.required("waypoints"), p.path("waypoints"), 1, 256);
        for (std::size_t i = 0; i < points.size(); ++i) {
            Fields w(points[i], at(p.path("waypoints"), i));
            control::Waypoint wp;
            wp.latitudeRad = numberOf(w.required("latitude_rad"), w.path("latitude_rad"), within(-kHalfPi, kHalfPi));
            wp.longitudeRad = numberOf(w.required("longitude_rad"), w.path("longitude_rad"), within(-kPi, kPi));
            number(w, "altitude_m", wp.altitudeM);
            choice(w, "altitude_reference", wp.altitudeReference, FSIM_ALTITUDE_REFERENCES);
            number(w, "speed", wp.speed, atLeast(0.0));
            choice(w, "speed_reference", wp.speedReference, FSIM_SPEED_REFERENCES);
            choice(w, "turn", wp.turn, {"fly_by", "fly_over"});
            if (const core::Json* v = w.optional("waypoint_id")) wp.id = idOf(*v, w.path("waypoint_id"));
            w.close();
            o.waypoints.push_back(wp);
        }
        break;
    }
    case Mode::Behavior: {
        const std::string& b = stringOf(p.required("behavior"), p.path("behavior"), 63);
        if (!lowerIdentifier(b, 63)) malformed(p.path("behavior"), "a behaviour is [a-z][a-z0-9_]*");
        o.behavior = b;
        if (const core::Json* v = p.optional("target_id")) {
            if (update) malformed(p.path("target_id"), "a behaviour's update carries its parameters alone, never a target");
            o.target = static_cast<std::uint32_t>(idOf(*v, p.path("target_id"), 32, true));
        }
        if (const core::Json* v = p.optional("params")) {
            Fields params(*v, p.path("params"));
            if (params.size() > 32) malformed(p.path("params"), "more than 32 parameters");
            for (const auto& [name, value] : v->asObject()) {
                if (!lowerIdentifier(name, 63)) malformed(params.path(name), "a parameter's name is [a-z][a-z0-9_]*");
                o.params[name] = numberOf(*params.optional(name), params.path(name));
            }
        }
        break;
    }
    }
    p.close();
}

/// A request's parameters as its operation defines them (section 11). Malformed for an unknown operation of a known
/// kind, a missing or unknown parameter, or a value out of its range.
Operation readOperation(const std::string& kind, const std::string& operation, const core::Json& parameters, const std::string& path,
                        std::size_t maxMessageSchemaBytes = 65536) {
    Operation o;
    if (kind != "flight" && kind != "sensor" && kind != "dms" && kind != "task" && kind != "c2" && kind != "gateway") {
        if (!parameters.isObject()) malformed(path, "an object is required");
        return o; // (an extension's kind: its parameters are its definition's, refused per request until installed)
    }
    Fields p(parameters, path);
    const auto unknown = [&]() { malformed(path, "unknown operation " + kind + "." + operation); };
    if (kind == "flight") {
        if (operation == "new") o.op = Op::FlightNew, flightCommand(p, o, false);
        else if (operation == "update") o.op = Op::FlightUpdate, flightCommand(p, o, true);
        else if (operation == "cancel") o.op = Op::FlightCancel, o.id = idOf(p.required("activity_id"), p.path("activity_id"), 64, true), p.close();
        else unknown();
        return o;
    }
    if (kind == "sensor") {
        if (operation == "set_state") {
            o.op = Op::SensorState;
            o.payload = payloadOf(p, "payload");
            static const ms::PayloadState states[] = {ms::PayloadState::Off, ms::PayloadState::Standby, ms::PayloadState::Operate};
            o.state = states[enumOf(p.required("state"), p.path("state"), {"off", "standby", "operate"})];
        } else if (operation == "search") {
            o.op = Op::SensorSearch;
            o.payload = payloadOf(p, "payload");
            o.amti.payload = o.payload;
            o.amti.mode = static_cast<ms::AmtiMode>(
                enumOf(p.required("mode"), p.path("mode"), {"range_while_search", "velocity_search", "track_while_scan", "surveillance", "single_target_track"}));
            if (const core::Json* v = p.optional("task")) o.amti.task = static_cast<ms::AmtiTask>(enumOf(*v, p.path("task"), {"volume", "cued", "track"}));
            if (const core::Json* v = p.optional("track_id")) o.amti.track = idOf(*v, p.path("track_id"));
            number(p, "azimuth_centre_rad", o.amti.azimuthCentreRad);
            number(p, "azimuth_half_width_rad", o.amti.azimuthHalfWidthRad, above(0.0));
            number(p, "elevation_centre_rad", o.amti.elevationCentreRad);
            number(p, "elevation_half_width_rad", o.amti.elevationHalfWidthRad, above(0.0));
            if (const core::Json* v = p.optional("bars")) o.amti.bars = static_cast<int>(integerOf(*v, p.path("bars"), 1, 8));
            number(p, "range_scale_m", o.amti.rangeScaleM, above(0.0));
            if (const core::Json* v = p.optional("signal_level")) o.amti.signalLevel = boolOf(*v, p.path("signal_level"));
            o.amti.illuminate = false; // (never a request of this version)
        } else if (operation == "cancel") {
            o.op = Op::SensorCancel;
            o.id = idOf(p.required("activity_id"), p.path("activity_id"), 64, true);
        } else if (operation == "designate") {
            o.op = Op::Designate;
            o.payload = payloadOf(p, "payload");
            o.track = idOf(p.required("track_id"), p.path("track_id"));
        } else if (operation == "designate_link") {
            o.op = Op::DesignateLink;
            o.system = systemOf(p, "source_system_id");
            o.payload = payloadOf(p, "payload");
            o.track = idOf(p.required("track_id"), p.path("track_id"));
        } else if (operation == "undesignate") {
            o.op = Op::Undesignate;
            o.target = static_cast<std::uint32_t>(idOf(p.required("target_id"), p.path("target_id"), 32, true));
        } else {
            unknown();
        }
        p.close();
        return o;
    }
    if (kind == "dms") {
        if (operation == "send") {
            o.op = Op::DmsSend;
            o.message.topic = static_cast<comm::Topic>(idOf(p.required("topic"), p.path("topic"), 32));
            const std::int64_t length = integerOf(p.required("byte_length"), p.path("byte_length"), 0, 65536);
            const std::string& data = stringOf(p.required("data_b64"), p.path("data_b64"), 87384);
            const auto bytes = decodeBase64(data);
            if (!bytes) malformed(p.path("data_b64"), "not canonical base64 (RFC 4648 section 4, padded, unused bits zero)");
            if (bytes->size() != static_cast<std::size_t>(length)) malformed(p.path("byte_length"), "not the decoded length");
            if (bytes->size() > maxMessageSchemaBytes) malformed(p.path("data_b64"), "decodes to more than 65,536 bytes");
            o.message.bytes = *bytes;
            if (const core::Json* v = p.optional("destination_system_ids")) o.message.destinations = systemList(*v, p.path("destination_system_ids"), 0, 64);
        } else if (operation == "publish_native") {
            o.op = Op::DmsPublish;
            o.report = static_cast<int>(enumOf(p.required("report"), p.path("report"), {"position", "status", "tracks"}));
            if (const core::Json* v = p.optional("payload")) o.payload = static_cast<std::uint16_t>(idOf(*v, p.path("payload"), 16));
            if (const core::Json* v = p.optional("destination_system_ids")) o.systems = systemList(*v, p.path("destination_system_ids"), 0, 64);
        } else if (operation == "subscribe" || operation == "unsubscribe") {
            o.op = operation == "subscribe" ? Op::DmsSubscribe : Op::DmsUnsubscribe;
            o.message.topic = static_cast<comm::Topic>(idOf(p.required("topic"), p.path("topic"), 32));
        } else if (operation == "subscribe_all") {
            o.op = Op::DmsSubscribeAll;
            o.enabled = boolOf(p.required("enabled"), p.path("enabled"));
        } else {
            unknown();
        }
        p.close();
        return o;
    }
    if (kind == "task") {
        if (operation == "accept") o.op = Op::TaskAccept;
        else if (operation == "reject") o.op = Op::TaskReject;
        else if (operation == "report") o.op = Op::TaskReport;
        else unknown();
        o.system = systemOf(p, "from_system_id");
        o.id = idOf(p.required("task_id"), p.path("task_id"), 64, true);
        if (o.op == Op::TaskReject) o.number = static_cast<std::uint32_t>(integerOf(p.required("code"), p.path("code"), 0, 4294967295LL));
        if (o.op == Op::TaskReport) {
            static const comm::ReceivedTask::State states[] = {comm::ReceivedTask::Accepted, comm::ReceivedTask::Completed, comm::ReceivedTask::Failed};
            o.taskState = states[enumOf(p.required("state"), p.path("state"), {"accepted", "completed", "failed"})];
            nullable(p, "percent", o.value, within(0.0, 100.0));
        }
        p.close();
        return o;
    }
    if (kind == "gateway") {
        if (operation != "close") unknown();
        o.op = Op::GatewayClose;
        o.id = idOf(p.required("request_id"), p.path("request_id"), 64, true);
        p.close();
        return o;
    }
    // c2
    if (operation == "declare_role") {
        o.op = Op::C2DeclareRole;
        o.number = static_cast<std::uint32_t>(idOf(p.required("role_id"), p.path("role_id"), 32, true));
        o.text = boundedString(p.required("callsign"), p.path("callsign"), 27);
        if (const core::Json* v = p.optional("destination_system_ids")) o.systems = systemList(*v, p.path("destination_system_ids"), 0, 64);
    } else if (operation == "designate_control") {
        o.op = Op::C2DesignateControl;
        o.system = systemOf(p, "ma_system_id");
        o.number = static_cast<std::uint32_t>(integerOf(p.required("modes_mask"), p.path("modes_mask"), 0, 4294967295LL));
    } else if (operation == "command_hsa") {
        o.op = Op::C2CommandHsa;
        o.system = systemOf(p, "ma_system_id");
        number(p, "heading_rad", o.hsa.headingRad);
        number(p, "course_rad", o.hsa.courseRad);
        number(p, "speed", o.hsa.speed, atLeast(0.0));
        choice(p, "speed_reference", o.hsa.speedReference, FSIM_SPEED_REFERENCES);
        number(p, "altitude_m", o.hsa.altitudeM);
        choice(p, "altitude_reference", o.hsa.altitudeReference, FSIM_ALTITUDE_REFERENCES);
    } else if (operation == "command_route") {
        o.op = Op::C2CommandRoute;
        o.system = systemOf(p, "ma_system_id");
        const auto& points = arrayOf(p.required("points"), p.path("points"), 1, 16);
        for (std::size_t i = 0; i < points.size(); ++i) {
            Fields w(points[i], at(p.path("points"), i));
            control::Waypoint wp;
            wp.latitudeRad = numberOf(w.required("latitude_rad"), w.path("latitude_rad"), within(-kHalfPi, kHalfPi));
            wp.longitudeRad = numberOf(w.required("longitude_rad"), w.path("longitude_rad"), within(-kPi, kPi));
            wp.altitudeM = numberOf(w.required("altitude_m"), w.path("altitude_m"));
            number(w, "speed", wp.speed, atLeast(0.0));
            w.close();
            o.waypoints.push_back(wp);
        }
    } else if (operation == "cancel_command") {
        o.op = Op::C2CancelCommand;
        o.system = systemOf(p, "ma_system_id");
        o.id = idOf(p.required("command_id"), p.path("command_id"), 64, true);
    } else if (operation == "send_task") {
        o.op = Op::C2SendTask;
        o.system = systemOf(p, "ma_system_id");
        o.task.type = static_cast<std::uint32_t>(enumOf(p.required("type"), p.path("type"), {"patrol", "strike", "reconnaissance", "escort", "transit", "other"}));
        if (const core::Json* v = p.optional("priority")) o.task.priority = static_cast<std::uint32_t>(integerOf(*v, p.path("priority"), 0, 4294967295LL));
        nullable(p, "latitude_rad", o.task.latitudeRad, within(-kHalfPi, kHalfPi));
        nullable(p, "longitude_rad", o.task.longitudeRad, within(-kPi, kPi));
        nullable(p, "altitude_m", o.task.altitudeM);
        nullable(p, "radius_m", o.task.radiusM, atLeast(0.0));
        nullable(p, "start_s", o.task.startS);
        nullable(p, "end_s", o.task.endS);
        if (const core::Json* v = p.present("target")) {
            Fields t(*v, p.path("target"));
            o.task.targetSource = systemOf(t, "source_system_id");
            o.task.targetPayload = static_cast<std::uint32_t>(idOf(t.required("payload"), t.path("payload"), 32));
            o.task.targetTrack = idOf(t.required("track_id"), t.path("track_id"));
            t.close();
        }
        if (const core::Json* v = p.optional("text")) {
            const std::string& text = boundedString(*v, p.path("text"), 63);
            std::copy(text.begin(), text.end(), o.task.text);
        }
    } else if (operation == "set_override") {
        o.op = Op::C2SetOverride;
        o.system = systemOf(p, "ma_system_id");
        o.override.kind = static_cast<std::uint32_t>(enumOf(p.required("override"), p.path("override"), {"speed", "altitude"}));
        bool cancel = false;
        if (const core::Json* v = p.optional("cancel")) cancel = boolOf(*v, p.path("cancel"));
        const core::Json* value = p.optional("value");
        if (value) o.override.value = numberOf(*value, p.path("value"));
        nullable(p, "expiry_s", o.override.expiryS);
        if (!value && !cancel) malformed(p.path(), "either a value or cancel true");
        o.override.cancel = cancel ? 1u : 0u;
    } else if (operation == "set_lost_comms") {
        o.op = Op::C2SetLostComms;
        o.system = systemOf(p, "ma_system_id");
        o.value = numberOf(p.required("threshold_s"), p.path("threshold_s"), atLeast(0.0));
        if (const core::Json* v = p.optional("plan_id")) o.id = idOf(*v, p.path("plan_id"));
    } else if (operation == "form_package") {
        o.op = Op::C2FormPackage;
        o.number = static_cast<std::uint32_t>(idOf(p.required("package_id"), p.path("package_id"), 32, true));
        o.system = systemOf(p, "leader_system_id");
        o.systems = systemList(p.required("member_system_ids"), p.path("member_system_ids"), 0, 16);
    } else if (operation == "dissolve_package") {
        o.op = Op::C2DissolvePackage;
        o.number = static_cast<std::uint32_t>(idOf(p.required("package_id"), p.path("package_id"), 32, true));
        o.systems = systemList(p.required("system_ids"), p.path("system_ids"), 1, 17);
    } else {
        unknown();
    }
    p.close();
    return o;
}

const char* dmsReasonName(comm::DmsReason r) noexcept {
    switch (r) {
    case comm::DmsReason::None: return "none";
    case comm::DmsReason::NoSystem: return "no_system";
    case comm::DmsReason::ReservedTopic: return "reserved_topic";
    case comm::DmsReason::TooLarge: return "too_large";
    case comm::DmsReason::UnknownSystem: return "unknown_system";
    case comm::DmsReason::NotOwner: return "not_owner";
    }
    return "unknown";
}
const char* commandStatusName(control::CommandStatus s) noexcept {
    switch (s) {
    case control::CommandStatus::Accepted: return "accepted";
    case control::CommandStatus::Rejected: return "rejected";
    case control::CommandStatus::Canceled: return "canceled";
    case control::CommandStatus::Valid: return "valid";
    }
    return "rejected";
}
const char* answerName(std::uint32_t state) noexcept {
    switch (state) {
    case comm::CommandStatus::Accepted: return "accepted";
    case comm::CommandStatus::Rejected: return "rejected";
    case comm::CommandStatus::Unauthorized: return "unauthorized";
    case comm::CommandStatus::Completed: return "completed";
    case comm::CommandStatus::Failed: return "failed";
    case comm::CommandStatus::Canceled: return "canceled";
    }
    return "rejected";
}
const char* taskStateName(comm::ReceivedTask::State s) noexcept {
    switch (s) {
    case comm::ReceivedTask::Received: return "received";
    case comm::ReceivedTask::Accepted: return "accepted";
    case comm::ReceivedTask::Rejected: return "rejected";
    case comm::ReceivedTask::Completed: return "completed";
    case comm::ReceivedTask::Failed: return "failed";
    }
    return "received";
}
const char* deliveryName(comm::Delivery d) noexcept {
    return d == comm::Delivery::Delivered ? "delivered" : d == comm::Delivery::Failed ? "failed" : "pending";
}
const char* taskTypeName(std::uint32_t t) noexcept {
    static const char* const names[] = {"patrol", "strike", "reconnaissance", "escort", "transit", "other"};
    return t < 6 ? names[t] : "other";
}

} // namespace

// --- records ----------------------------------------------------------------------------------------------------------

enum class State : std::uint8_t { Queued, Rejected, Accepted, Pending, Executing, Completed, Failed, Canceled };

namespace {
const char* stateName(State s) noexcept {
    static const char* const names[] = {"queued", "rejected", "accepted", "pending", "executing", "completed", "failed", "canceled"};
    return names[static_cast<int>(s)];
}
/// A native activity's state, as a receipt follows it (section 12).
State followed(control::ActivityState s) noexcept {
    switch (s) {
    case control::ActivityState::Pending:
    case control::ActivityState::Disabled: return State::Pending;
    case control::ActivityState::Active: return State::Executing;
    case control::ActivityState::Completed: return State::Completed;
    case control::ActivityState::Failed: return State::Failed;
    case control::ActivityState::Canceled:
    case control::ActivityState::Deleted: return State::Canceled;
    }
    return State::Pending;
}
bool ended(State s) noexcept { return s != State::Queued && s != State::Pending && s != State::Executing; }
} // namespace

struct Gateway::Native {
    const char* domain = "";
    std::string status;
    std::optional<std::string> reason;
    std::optional<std::uint32_t> code;
    std::vector<std::pair<const char*, std::uint64_t>> ids; ///< in the schema's order; written as identities
    std::optional<std::uint32_t> messageCount;
};

struct Gateway::Record {
    enum class Follow : std::uint8_t { None, ViActivity, MsActivity, Transmission, C2Answers, C2FirstAnswer };
    std::uint64_t id = 0;
    std::string kind, operation, body;
    bool hasComponent = false;
    std::uint64_t componentId = 0;
    std::optional<std::string> label;
    Operation op;
    std::uint64_t submitted = 0, updated = 0;
    State state = State::Queued;
    bool closed = false;
    const char* gatewayReason = nullptr;
    std::optional<Native> native;
    Follow follow = Follow::None;
    std::uint64_t followId = 0;      ///< the activity, message or command followed
    comm::SystemId ma = 0;           ///< a C2 command's addressee
    comm::Topic topic = 0;           ///< and its topic
    bool answered = false;           ///< a C2 command's status was seen once
    std::vector<std::uint64_t> messages; ///< the DMS messages it sent
};

struct Gateway::Service {
    const ServiceConfig* config = nullptr;
    Agent agent;
    std::uint32_t platform = 0;
    bool c2Node = false;
    std::map<std::uint64_t, Record> records; ///< by request id
    std::uint64_t highest = 0;               ///< the highest request id used in this generation
    std::optional<std::string> submission;   ///< the latest submission member
    std::vector<comm::DmsReceived> messages; ///< this tick's, taken once at its freeze
    std::string observation;                 ///< the frozen snapshot
    std::size_t open() const {
        return static_cast<std::size_t>(std::count_if(records.begin(), records.end(), [](const auto& r) { return !r.second.closed; }));
    }
};

// --- construction -----------------------------------------------------------------------------------------------------

Gateway::Gateway(std::shared_ptr<const Config> config, Minted minted)
    : config_(std::move(config)), facade_(std::move(minted.facade)), generation_(minted.generation), localEpisodeIndex_(minted.localEpisodeIndex) {
    Writer k;
    k.beginObject().key("env_id").id(config_->envId).key("episode_generation").id(generation_).key("local_episode_index").id(localEpisodeIndex_).endObject();
    key_ = k.take();
    const std::vector<std::uint32_t> nodes = facade_->c2Nodes();
    const Referee referee = facade_->referee();
    for (const std::string& name : config_->platforms) {
        Platform p;
        p.name = name;
        for (const std::uint32_t node : nodes)
            if (const auto spec = referee.c2Node(node); spec && spec->name == name) p.entity = node, p.c2Node = true;
        if (!p.entity) p.entity = facade_->vehicle(name).id();
        platforms_.push_back(p);
    }
    for (std::size_t i = 0; i < config_->services.size(); ++i) {
        auto s = std::make_unique<Service>();
        s->config = &config_->services[i];
        s->agent = minted.agents.at(i);
        s->platform = s->agent.vehicle();
        s->c2Node = std::find(nodes.begin(), nodes.end(), s->platform) != nodes.end();
        services_.push_back(std::move(s));
    }
    criteria_.resize(config_->criteria.size());
    freeze();
}

Gateway::~Gateway() = default;

bool Gateway::serves(std::uint64_t serviceId) const noexcept {
    return std::any_of(services_.begin(), services_.end(), [&](const auto& s) { return s->config->serviceId == serviceId; });
}
Gateway::Service& Gateway::service(std::uint64_t serviceId) {
    for (auto& s : services_)
        if (s->config->serviceId == serviceId) return *s;
    throw std::invalid_argument("mission episode: no configured service " + std::to_string(serviceId));
}
const Gateway::Service& Gateway::service(std::uint64_t serviceId) const { return const_cast<Gateway*>(this)->service(serviceId); }

// --- submit: validate and queue ---------------------------------------------------------------------------------------

std::string Gateway::submit(std::uint64_t serviceId, std::string_view text) {
    Service& s = service(serviceId);
    const Limits& limits = config_->limits;
    if (text.size() > limits.maxDocumentBytes) malformed("ma.actions.v1", "larger than max_document_bytes");
    const core::Json doc = parseStrict(text, "ma.actions.v1");
    Fields f(doc, "ma.actions.v1: $");
    const core::Json& schema = f.required("schema");
    if (!schema.isString() || schema.asString() != "ma.actions.v1") malformed(f.path("schema"), "\"ma.actions.v1\" is required");
    Fields key(f.required("episode"), f.path("episode"));
    const std::uint64_t envId = idOf(key.required("env_id"), key.path("env_id"));
    const std::uint64_t generation = idOf(key.required("episode_generation"), key.path("episode_generation"), 64, true);
    const std::uint64_t local = idOf(key.required("local_episode_index"), key.path("local_episode_index"));
    key.close();
    const std::uint64_t addressed = idOf(f.required("service_id"), f.path("service_id"), 64, true);
    const std::uint64_t tick = idOf(f.required("tick"), f.path("tick"));
    const auto& list = arrayOf(f.required("requests"), f.path("requests"), 0, 1024);
    if (list.size() > limits.maxRequestsPerDocument) malformed(f.path("requests"), "more requests than max_requests_per_document");
    struct Incoming {
        std::uint64_t id;
        std::string kind, operation, body;
        bool hasComponent = false;
        std::uint64_t componentId = 0;
        std::optional<std::string> label;
        Operation op;
    };
    std::vector<Incoming> incoming;
    for (std::size_t i = 0; i < list.size(); ++i) {
        const std::string path = at(f.path("requests"), i);
        Fields r(list[i], path);
        Incoming in;
        in.id = idOf(r.required("request_id"), r.path("request_id"), 64, true);
        in.kind = stringOf(r.required("kind"), r.path("kind"), 31);
        in.operation = stringOf(r.required("operation"), r.path("operation"), 31);
        if (!lowerIdentifier(in.kind, 31)) malformed(r.path("kind"), "a kind is [a-z][a-z0-9_]*");
        if (!lowerIdentifier(in.operation, 31)) malformed(r.path("operation"), "an operation is [a-z][a-z0-9_]*");
        const core::Json& parameters = r.required("parameters");
        in.op = readOperation(in.kind, in.operation, parameters, r.path("parameters"));
        core::Json body = core::Json::Object{};
        body.set("kind", in.kind).set("operation", in.operation).set("parameters", parameters);
        if (const core::Json* c = r.optional("component")) {
            Fields comp(*c, r.path("component"));
            in.hasComponent = true;
            in.componentId = idOf(comp.required("component_id"), comp.path("component_id"));
            if (const core::Json* l = comp.optional("label")) in.label = stringOf(*l, comp.path("label"), 63);
            comp.close();
            body.set("component", *c);
        }
        r.close();
        if (!incoming.empty() && in.id <= incoming.back().id) malformed(f.path("requests"), "request ids do not strictly increase");
        in.body = canonical(body);
        incoming.push_back(std::move(in));
    }
    f.close();

    Writer w;
    w.beginObject().key("tick").id(tick);
    const char* reason = nullptr;
    if (envId != config_->envId || generation != generation_ || local != localEpisodeIndex_) reason = "stale_generation";
    else if (tick < tick_) reason = "late_tick";
    else if (tick > tick_) reason = "future_tick";
    else if (addressed != serviceId) reason = "wrong_service";
    w.key("accepted").boolean(reason == nullptr).key("reason");
    reason ? w.string(reason) : w.null();
    w.key("requests").beginArray();
    if (!reason) {
        for (Incoming& in : incoming) {
            w.beginObject().key("request_id").id(in.id);
            const auto found = s.records.find(in.id);
            if (found != s.records.end()) {
                // the record answers: its own receipt, no effect repeated - or another body under its id
                if (found->second.body == in.body) w.member("status", "duplicate").key("reason").null();
                else w.member("status", "rejected").member("reason", "request_id_reuse");
                w.endObject();
                continue;
            }
            if (in.id <= s.highest) {
                w.member("status", "rejected").member("reason", "retry_expired").endObject();
                continue;
            }
            s.highest = in.id;
            Record rec;
            rec.id = in.id, rec.kind = std::move(in.kind), rec.operation = std::move(in.operation), rec.body = std::move(in.body);
            rec.hasComponent = in.hasComponent, rec.componentId = in.componentId, rec.label = std::move(in.label);
            rec.submitted = rec.updated = tick_;
            const char* refused = nullptr;
            if (in.op.op == Op::Unsupported) refused = "unsupported_kind";
            else if (in.op.op == Op::DmsSend && in.op.message.bytes.size() > limits.maxMessageBytes) refused = "message_too_large";
            else if (s.open() >= limits.maxInFlightRequests) refused = "capacity";
            rec.op = std::move(in.op);
            if (refused) {
                rec.state = State::Rejected, rec.closed = true, rec.gatewayReason = refused;
                w.member("status", "rejected").member("reason", refused);
            } else {
                w.member("status", "queued").key("reason").null();
            }
            w.endObject();
            s.records.emplace(rec.id, std::move(rec));
        }
    }
    w.endArray().endObject();
    s.submission = w.take();
    return receiptsOf(s);
}

std::string Gateway::receipts(std::uint64_t serviceId) const { return receiptsOf(service(serviceId)); }
const std::string& Gateway::observation(std::uint64_t serviceId) const { return service(serviceId).observation; }

// --- dispatch: each request as its service's own Agent ----------------------------------------------------------------

bool Gateway::owns(const Service& s, const std::string& capability) const {
    return maCapabilityOwner(*facade_, s.platform, capability) == s.agent.controller();
}

std::optional<std::uint64_t> Gateway::ownerService(std::uint32_t platform, const std::string& capability) const {
    const control::ControllerId owner = maCapabilityOwner(*facade_, platform, capability);
    if (!owner) return std::nullopt;
    for (const auto& s : services_)
        if (s->platform == platform && s->agent.controller() == owner) return s->config->serviceId;
    return std::nullopt;
}

namespace {

Gateway::Native nativeOf(const char* domain, std::string status) {
    Gateway::Native n;
    n.domain = domain, n.status = std::move(status);
    return n;
}

/// The command capability a flight request commands, as the platform discovers it.
std::string flightCapability(const Agent& agent, const Operation& o) {
    if (o.mode == Mode::Hsa) return "fsim.guidance.hsa";
    if (o.mode == Mode::Route) return "fsim.guidance.route";
    for (const control::CapabilityDescriptor& d : agent.capabilities())
        if (d.behavior == o.behavior) return d.id;
    // (the native registry's naming of a behaviour no descriptor offers: not a capability the platform has)
    return "user.guidance." + o.behavior;
}

/// A payload's Mission Systems capability instance of a type ("" when it has none).
std::string missionCapability(const Agent& agent, std::uint16_t payload, ms::CapabilityType type) {
    for (const ms::CapabilityInfo& c : agent.missionCapabilities())
        if (c.payload == payload && c.type == type) return c.id;
    return {};
}

} // namespace

void Gateway::dispatch(Service& s, Record& r) {
    using Follow = Record::Follow;
    const Operation& o = r.op;
    r.updated = tick_ + 1;
    const auto refuse = [&](const char* why) {
        r.state = State::Rejected, r.closed = true, r.gatewayReason = why;
    };
    const auto oneShot = [&](bool done, Native n) {
        r.native = std::move(n);
        r.state = done ? State::Accepted : State::Rejected;
        r.closed = true;
    };
    if (o.op == Op::GatewayClose) {
        // the transport's own: the request named stops being followed, in its last known state; nothing native
        const auto target = s.records.find(o.id);
        if (target == s.records.end() || target->first >= r.id || target->second.closed) return refuse("unknown_request");
        target->second.closed = true, target->second.follow = Follow::None, target->second.updated = tick_ + 1;
        r.state = State::Accepted, r.closed = true;
        return;
    }
    if (!s.agent.valid()) return refuse("service_not_alive");
    const bool flightOrSensor = o.op <= Op::Undesignate;
    if (flightOrSensor && s.c2Node) return refuse("not_applicable");
    const bool ingress = o.op == Op::DmsSubscribe || o.op == Op::DmsUnsubscribe || o.op == Op::DmsSubscribeAll;
    const bool egress = !flightOrSensor && !ingress;
    if (ingress && !s.config->ownsDmsIngress) return refuse("dms_ingress_not_owned");
    if (egress && !s.config->ownsDmsEgress) return refuse("dms_egress_not_owned");

    switch (o.op) {
    case Op::FlightNew:
    case Op::FlightUpdate:
    case Op::FlightCancel: {
        if (o.op == Op::FlightNew && !owns(s, flightCapability(s.agent, o))) return refuse("capability_not_bound");
        control::CommandResult res;
        if (o.op == Op::FlightNew) {
            control::CommandOptions options;
            options.commandId = r.id;
            if (o.interrupt) options.interrupt = *o.interrupt;
            if (o.mode == Mode::Hsa) res = s.agent.submit(o.hsa, options);
            else if (o.mode == Mode::Route) res = s.agent.submit(o.route, Span<const control::Waypoint>(o.waypoints.data(), o.waypoints.size()), options);
            else res = s.agent.submit(control::BehaviorCommand{o.behavior, o.target, o.params, {}}, options);
        } else if (o.op == Op::FlightUpdate) {
            if (o.mode == Mode::Hsa) res = s.agent.update(o.id, o.hsa);
            else if (o.mode == Mode::Route) res = s.agent.update(o.id, o.route, Span<const control::Waypoint>(o.waypoints.data(), o.waypoints.size()));
            else res = s.agent.update(o.id, control::BehaviorCommand{o.behavior, 0, o.params, {}});
        } else {
            res = s.agent.cancel(o.id);
        }
        Native n = nativeOf("vi", commandStatusName(res.status));
        if (res.reason != control::Reason::None) n.reason = control::reasonName(res.reason);
        n.code = static_cast<std::uint32_t>(res.reason);
        n.ids = {{"activity_id", res.activity}, {"other_activity_id", res.other}};
        if (o.op == Op::FlightNew) n.ids.emplace_back("command_id", r.id);
        if (o.op != Op::FlightNew) return oneShot(res.status == control::CommandStatus::Accepted || res.status == control::CommandStatus::Canceled, std::move(n));
        r.native = std::move(n);
        if (res.status != control::CommandStatus::Accepted) {
            r.state = State::Rejected, r.closed = true;
            return;
        }
        r.state = State::Pending, r.follow = Follow::ViActivity, r.followId = res.activity;
        activityOwners_[res.activity] = s.config->serviceId;
        return;
    }
    case Op::SensorState:
    case Op::SensorSearch:
    case Op::SensorCancel: {
        if (o.op != Op::SensorCancel) {
            const std::string capability =
                missionCapability(s.agent, o.payload, o.op == Op::SensorState ? ms::CapabilityType::Status : ms::CapabilityType::Amti);
            if (capability.empty() || !owns(s, capability)) return refuse("capability_not_bound");
        }
        ms::CommandResult res;
        ms::CommandOptions options;
        options.commandId = r.id;
        if (o.op == Op::SensorState) res = s.agent.submit(ms::StateCommand{o.payload, o.state}, options);
        else if (o.op == Op::SensorSearch) res = s.agent.submit(o.amti, options);
        else res = s.agent.cancelMission(o.id);
        Native n = nativeOf("ms", commandStatusName(res.status));
        if (res.reason != ms::Reason::None) n.reason = ms::reasonName(res.reason);
        n.code = static_cast<std::uint32_t>(res.reason);
        n.ids = {{"activity_id", res.activity}};
        if (o.op == Op::SensorCancel) return oneShot(res.status == control::CommandStatus::Accepted || res.status == control::CommandStatus::Canceled, std::move(n));
        r.native = std::move(n);
        if (res.status != control::CommandStatus::Accepted) {
            r.state = State::Rejected, r.closed = true;
            return;
        }
        r.state = State::Pending, r.follow = Follow::MsActivity, r.followId = res.activity;
        activityOwners_[res.activity] = s.config->serviceId;
        return;
    }
    case Op::Designate:
    case Op::DesignateLink:
    case Op::Undesignate: {
        if (o.op == Op::Designate) {
            const std::string capability = missionCapability(s.agent, o.payload, ms::CapabilityType::Amti);
            if (capability.empty() || !owns(s, capability)) return refuse("capability_not_bound");
        }
        bool done = false;
        std::uint32_t target = 0;
        if (o.op == Op::Undesignate) done = s.agent.undesignate(o.target), target = o.target;
        else target = o.op == Op::Designate ? s.agent.designate(o.payload, o.track) : s.agent.designateLink(o.system, o.payload, o.track), done = target != 0;
        Native n = nativeOf("designation", done ? (o.op == Op::Undesignate ? "undesignated" : "designated") : "not_held");
        if (done) n.ids = {{"target_id", target}};
        return oneShot(done, std::move(n));
    }
    case Op::DmsSend: {
        comm::DmsReason why = comm::DmsReason::None;
        const std::uint64_t message = s.agent.send(o.message, &why);
        if (!message) {
            Native n = nativeOf("dms", "refused");
            n.reason = dmsReasonName(why), n.code = static_cast<std::uint32_t>(why);
            return oneShot(false, std::move(n));
        }
        r.messages = {message};
        Native n = nativeOf("dms", o.message.destinations.empty() ? "published" : "pending");
        n.code = 0;
        n.ids = {{"message_id", message}};
        if (o.message.destinations.empty()) return oneShot(true, std::move(n));
        r.native = std::move(n);
        r.state = State::Pending, r.follow = Follow::Transmission, r.followId = message;
        return;
    }
    case Op::DmsPublish: {
        comm::DmsReason why = comm::DmsReason::None;
        std::vector<std::uint64_t> sent;
        if (o.report == 2) {
            sent = s.agent.publishTracks(o.payload, o.systems, &why);
        } else {
            const std::uint64_t m = o.report == 0 ? s.agent.publishPosition(o.systems, &why) : s.agent.publishStatus(o.systems, &why);
            if (m) sent.push_back(m);
        }
        if (sent.empty() && (o.report != 2 || why != comm::DmsReason::None)) {
            Native n = nativeOf("dms", "refused");
            n.reason = dmsReasonName(why), n.code = static_cast<std::uint32_t>(why);
            return oneShot(false, std::move(n));
        }
        r.messages = sent;
        Native n = nativeOf("dms", "published");
        n.code = 0;
        if (!sent.empty()) n.ids = {{"message_id", sent.front()}};
        n.messageCount = static_cast<std::uint32_t>(sent.size());
        return oneShot(true, std::move(n));
    }
    case Op::DmsSubscribe:
    case Op::DmsUnsubscribe:
    case Op::DmsSubscribeAll: {
        const bool done = o.op == Op::DmsSubscribe     ? s.agent.subscribe(o.message.topic)
                          : o.op == Op::DmsUnsubscribe ? s.agent.unsubscribe(o.message.topic)
                                                       : s.agent.subscribeAll(o.enabled);
        return oneShot(done, nativeOf("dms", done ? "done" : "refused"));
    }
    case Op::TaskAccept:
    case Op::TaskReject:
    case Op::TaskReport: {
        const bool done = o.op == Op::TaskAccept   ? s.agent.acceptTask(o.system, o.id)
                          : o.op == Op::TaskReject ? s.agent.rejectTask(o.system, o.id, o.number)
                                                   : s.agent.reportTask(o.system, o.id, o.taskState, o.value);
        return oneShot(done, nativeOf("task", done ? "answered" : "not_held_or_decided"));
    }
    default: break;
    }

    // c2: a typed message through the service's DMS, its answers followed as the native picture keeps them
    comm::DmsReason why = comm::DmsReason::None;
    std::uint64_t command = 0;
    bool sent = false;
    Follow follow = Follow::None;
    comm::Topic topic = 0;
    switch (o.op) {
    case Op::C2DeclareRole: sent = s.agent.declareRole(o.number, o.text, o.systems, &why); break;
    case Op::C2DesignateControl:
        command = s.agent.designateControl(o.system, o.number, &why), follow = Follow::C2FirstAnswer, topic = comm::topic::kControlDesignation;
        break;
    case Op::C2CommandHsa: command = s.agent.commandHsa(o.system, o.hsa, &why), follow = Follow::C2Answers, topic = comm::topic::kFlightCommand; break;
    case Op::C2CommandRoute:
        command = s.agent.commandRoute(o.system, Span<const control::Waypoint>(o.waypoints.data(), o.waypoints.size()), &why);
        follow = Follow::C2Answers, topic = comm::topic::kFlightCommand;
        break;
    case Op::C2CancelCommand: command = s.agent.cancelCommand(o.system, o.id, &why); break;
    case Op::C2SendTask: command = s.agent.sendTask(o.system, o.task, &why), follow = Follow::C2Answers, topic = comm::topic::kTask; break;
    case Op::C2SetOverride: command = s.agent.setOverride(o.system, o.override, &why), follow = Follow::C2FirstAnswer, topic = comm::topic::kOverride; break;
    case Op::C2SetLostComms:
        command = s.agent.setLostComms(o.system, o.value, o.id, &why), follow = Follow::C2FirstAnswer, topic = comm::topic::kLostComms;
        break;
    case Op::C2FormPackage: command = s.agent.formPackage(o.number, o.system, o.systems, &why); break;
    case Op::C2DissolvePackage: command = s.agent.dissolvePackage(o.number, o.systems, &why); break;
    default: return refuse("unsupported_kind"); // (not reached: every generic operation is above)
    }
    if (o.op != Op::C2DeclareRole) sent = command != 0;
    if (!sent) {
        Native n = nativeOf("c2", "refused");
        n.reason = dmsReasonName(why), n.code = static_cast<std::uint32_t>(why);
        return oneShot(false, std::move(n));
    }
    Native n = nativeOf("c2", "sent");
    if (command) n.ids.emplace_back("command_id", command);
    if (o.op == Op::C2SendTask) n.ids.emplace_back("task_id", command);
    if (o.system && o.op != Op::C2FormPackage) n.ids.emplace_back("from_system_id", o.system);
    if (follow == Follow::None) return oneShot(true, std::move(n));
    r.native = std::move(n);
    r.state = State::Pending, r.follow = follow, r.followId = command, r.ma = o.system, r.topic = topic;
}

// --- after the World stepped: native statuses --------------------------------------------------------------------------

void Gateway::refresh(Service& s, Record& r) {
    using Follow = Record::Follow;
    const auto evicted = [&]() {
        // the native picture no longer holds its status: closed in its last known state, nothing claimed
        if (r.native) r.native->status = "status_evicted";
        r.closed = true, r.follow = Follow::None;
    };
    switch (r.follow) {
    case Follow::None: return;
    case Follow::ViActivity: {
        const auto a = s.agent.activity(r.followId);
        if (!a) return evicted();
        r.state = followed(a->state);
        r.native->status = control::activityStateName(a->state);
        r.native->reason = a->reason != control::Reason::None ? std::optional<std::string>(control::reasonName(a->reason)) : std::nullopt;
        r.native->code = static_cast<std::uint32_t>(a->reason);
        break;
    }
    case Follow::MsActivity: {
        const auto a = s.agent.missionActivity(r.followId);
        if (!a) return evicted();
        r.state = followed(a->state);
        r.native->status = control::activityStateName(a->state);
        r.native->reason = a->reason != ms::Reason::None ? std::optional<std::string>(ms::reasonName(a->reason)) : std::nullopt;
        r.native->code = static_cast<std::uint32_t>(a->reason);
        break;
    }
    case Follow::Transmission: {
        const auto t = s.agent.transmission(r.followId);
        if (!t) return evicted();
        bool pending = false, failed = false;
        for (const auto& [system, delivery] : t->destinations) pending = pending || delivery == comm::Delivery::Pending, failed = failed || delivery == comm::Delivery::Failed;
        if (!pending) {
            r.state = failed ? State::Failed : State::Completed;
            r.native->status = failed ? "failed" : "delivered";
        }
        break;
    }
    case Follow::C2Answers:
    case Follow::C2FirstAnswer: {
        const comm::CommandStatus* status = nullptr;
        const std::vector<comm::CommandStatusEntry> statuses = s.agent.commandStatuses();
        for (const comm::CommandStatusEntry& e : statuses)
            if (e.from == r.ma && e.status.commandId == r.followId && e.status.topic == r.topic) status = &e.status;
        if (!status) {
            if (r.answered) evicted();
            return; // (never answered: pending until its service closes it)
        }
        r.answered = true;
        const std::uint32_t st = status->state;
        const bool refused = st == comm::CommandStatus::Rejected || st == comm::CommandStatus::Unauthorized;
        if (r.follow == Follow::C2Answers && refused && r.state == State::Executing) return; // (a later cancel's answer)
        r.native->status = answerName(st);
        r.native->code = status->reason;
        if (status->activity) {
            auto& ids = r.native->ids;
            if (std::none_of(ids.begin(), ids.end(), [](const auto& id) { return std::string_view(id.first) == "activity_id"; }))
                ids.insert(ids.begin(), {"activity_id", status->activity});
        }
        if (refused) r.state = State::Rejected;
        else if (r.follow == Follow::C2FirstAnswer) r.state = State::Accepted;
        else if (st == comm::CommandStatus::Accepted) r.state = State::Executing;
        else r.state = st == comm::CommandStatus::Completed ? State::Completed : st == comm::CommandStatus::Failed ? State::Failed : State::Canceled;
        break;
    }
    }
    if (ended(r.state)) r.closed = true, r.follow = Follow::None;
}

void Gateway::beforeStep(session::World&, std::uint64_t tick) {
    tick_ = tick;
    for (auto& s : services_)
        for (auto& [id, r] : s->records)
            if (r.state == State::Queued) dispatch(*s, r);
}

void Gateway::afterStep(const session::World&, std::uint64_t tick) {
    tick_ = tick;
    const std::uint64_t retention = config_->limits.completedRequestRetentionTicks;
    for (auto& s : services_) {
        for (auto& [id, r] : s->records) {
            if (r.closed) continue;
            const State state = r.state;
            const std::string status = r.native ? r.native->status : std::string();
            refresh(*s, r);
            if (r.state != state || r.closed || (r.native && r.native->status != status)) r.updated = std::max(r.updated, tick);
        }
        // retained for its ticks after it closed, then forgotten: its id is expired from then on
        for (auto it = s->records.begin(); it != s->records.end();)
            it = it->second.closed && tick > it->second.updated + retention ? s->records.erase(it) : std::next(it);
    }
    judge();
    freeze();
}

// --- documents --------------------------------------------------------------------------------------------------------

namespace {

void writeRecord(Writer& w, const Gateway::Record& r) {
    w.beginObject().key("request_id").id(r.id).member("kind", r.kind).member("operation", r.operation);
    if (r.hasComponent) {
        w.key("component").beginObject().key("component_id").id(r.componentId);
        if (r.label) w.member("label", *r.label);
        w.endObject();
    }
    w.key("submitted_tick").id(r.submitted).key("updated_tick").id(r.updated).member("state", stateName(r.state)).key("closed").boolean(r.closed);
    w.key("gateway_reason");
    r.gatewayReason ? w.string(r.gatewayReason) : w.null();
    w.key("native");
    if (!r.native) {
        w.null();
    } else {
        const Gateway::Native& n = *r.native;
        w.beginObject().member("domain", n.domain).member("status", n.status).key("reason");
        n.reason ? w.string(*n.reason) : w.null();
        w.key("reason_code");
        n.code ? w.integer(*n.code) : w.null();
        w.key("ids").beginObject();
        for (const auto& [name, value] : n.ids) {
            w.key(name).id(value);
            if (std::string_view(name) == "message_id" && n.messageCount) w.key("message_count").integer(*n.messageCount);
        }
        if (n.messageCount && std::none_of(n.ids.begin(), n.ids.end(), [](const auto& id) { return std::string_view(id.first) == "message_id"; }))
            w.key("message_count").integer(*n.messageCount);
        w.endObject().endObject();
    }
    w.endObject();
}

void writeMeta(Writer& w, const ma::ReportMeta& m) {
    w.key("meta").beginObject().member("source", ma::sourceName(m.source));
    if (m.system) w.key("system_id").id(m.system);
    if (m.net) w.key("net_id").id(m.net);
    if (m.payload != ma::kNoPayload) w.key("payload").id(m.payload);
    w.member("validity", ma::validityName(m.validity));
    w.key("sample_time_s").number(m.sampleTimeS).key("receive_time_s").number(m.receiveTimeS).key("age_s").number(m.ageS);
    w.endObject();
}

/// A collection's envelope; `items` writes its items when it is available.
template <typename Items>
void collection(Writer& w, const char* name, ma::Availability availability, std::size_t count, Items items) {
    const bool available = availability == ma::Availability::Available;
    w.key(name).beginObject().member("availability", ma::availabilityName(availability)).key("count").integer(available ? static_cast<std::int64_t>(count) : 0);
    w.key("items").beginArray();
    if (available) items();
    w.endArray().endObject();
}

/// The availability of a part with no report call of its own (section 10).
ma::Availability partAvailability(bool alive, bool applicable, bool owner = true) {
    if (!alive) return ma::Availability::Unavailable;
    if (!applicable) return ma::Availability::NotApplicable;
    return owner ? ma::Availability::Available : ma::Availability::NotOwner;
}

/// A time a report states, `when` its snapshot's: its meta as an own or delivered report's (section 10).
ma::ReportMeta linkMeta(comm::SystemId from, double sampleS, double receiveS, double now, double staleAfterS) {
    ma::ReportMeta m;
    m.source = ma::Source::Link, m.system = from, m.sampleTimeS = sampleS, m.receiveTimeS = receiveS;
    m.ageS = std::isfinite(sampleS) ? now - sampleS : kNaN;
    m.validity = std::isfinite(m.ageS) && std::isfinite(staleAfterS) && m.ageS > staleAfterS ? ma::Validity::Stale : ma::Validity::Valid;
    return m;
}

} // namespace

std::string Gateway::receiptsOf(const Service& s) const {
    Writer w;
    w.beginObject().member("schema", "ma.receipts.v1").key("episode").raw(key_).key("service_id").id(s.config->serviceId).key("tick").id(tick_);
    w.key("submission");
    s.submission ? w.raw(*s.submission) : w.null();
    w.key("receipts").beginArray();
    for (const auto& [id, r] : s.records) writeRecord(w, r);
    w.endArray().endObject();
    return w.take();
}

std::string Gateway::observe(Service& s) {
    Agent& a = s.agent;
    const bool alive = a.valid();
    const double now = facade_->time();
    ma::ReportOptions options;
    options.staleAfterS = config_->staleAfterS;
    Writer w;
    w.beginObject().member("schema", "ma.observation.v1").key("episode").raw(key_);
    w.key("service").beginObject().key("service_id").id(s.config->serviceId).member("name", s.config->name);
    w.member("platform_kind", s.c2Node ? "c2_node" : "vehicle").key("platform_id").id(s.platform).key("controller_id").id(a.controller());
    w.key("system_id").idOrNull(a.system()).key("alive").boolean(alive).endObject();
    w.key("tick").id(tick_).key("time_s").number(now);
    w.key("reports").beginObject();

    // navigation: its own ship and endurance, as its navigation reports them
    {
        const ma::Report<ma::Navigation> nav = a.reportNavigation(options);
        const bool available = nav.availability == ma::Availability::Available;
        w.key("navigation").beginObject().member("availability", ma::availabilityName(nav.availability));
        writeMeta(w, nav.meta);
        w.key("own_ship");
        if (!available) {
            w.null();
        } else {
            const OwnShip& o = nav.value.ownShip;
            w.beginObject().key("time_s").number(o.timeS).key("latitude_rad").number(o.latitudeRad).key("longitude_rad").number(o.longitudeRad);
            w.key("altitude_msl_m").number(o.altitudeMslM).key("altitude_agl_m").number(o.altitudeAglM);
            w.key("velocity_ned_ms").vector3(o.velocityNedMs).key("euler_rad").vector3(o.eulerRad);
            w.key("angular_rate_body_rad_s").vector3(o.angularRateBodyRadS).key("acceleration_body_ms2").vector3(o.accelerationBodyMs2);
            w.key("airspeed_true_ms").number(o.airspeedTrueMs).key("airspeed_calibrated_ms").number(o.airspeedCalibratedMs).key("mach").number(o.mach);
            w.key("alpha_rad").number(o.alphaRad).key("beta_rad").number(o.betaRad).key("load_factor").number(o.loadFactor).key("fuel_kg").number(o.fuelKg);
            w.key("on_ground").boolean(o.onGround).key("gnss_valid").boolean(o.gnssValid).key("air_data_valid").boolean(o.airDataValid);
            w.key("throttle").number(o.throttle).endObject();
        }
        w.key("endurance");
        if (!available) {
            w.null();
        } else {
            const control::NavigationReport& e = nav.value.endurance;
            w.beginObject().member("energy", control::energyName(e.energy)).key("fuel_kg").number(e.fuelKg).key("remaining").number(e.remaining);
            w.key("capacity").number(e.capacity).key("percent").number(e.percent).key("consumption").number(e.consumption);
            w.key("endurance_s").number(e.enduranceS).key("endurance_unbounded").boolean(std::isinf(e.enduranceS));
            w.key("reserve").number(e.reserve).key("playtime_s").number(e.playtimeS).key("playtime_unbounded").boolean(std::isinf(e.playtimeS));
            w.key("return_distance_m").number(e.returnDistanceM).key("return_tas_ms").number(e.returnTasMs).key("return_consumption").number(e.returnConsumption);
            w.member("contingency", control::contingencyName(e.contingency)).key("starved").boolean(e.starved).endObject();
        }
        w.endObject();
    }

    // capabilities: each command capability by its discovery id, as the platform offers it for command
    {
        std::vector<control::CapabilityDescriptor> descriptors;
        std::vector<ms::CapabilityInfo> missions;
        if (alive && !s.c2Node) descriptors = a.capabilities(), missions = a.missionCapabilities();
        descriptors.erase(std::remove_if(descriptors.begin(), descriptors.end(),
                                         [](const control::CapabilityDescriptor& d) { return !(d.interactions & control::kCommand) || d.kind == control::CapabilityKind::Status; }),
                          descriptors.end());
        const auto item = [&](const std::string& id, const char* domain, const char* availability, const char* reason) {
            const control::ControlStatus cs = a.controlStatus(id);
            w.beginObject().member("capability", id).member("domain", domain).key("owner_service_id");
            const auto owner = ownerService(s.platform, id);
            owner ? w.id(*owner) : w.null();
            w.member("availability", availability).member("reason", reason).key("allowed").boolean(cs.allowed);
            w.key("granted").boolean(cs.granted && cs.holder == a.controller()).endObject();
        };
        collection(w, "capabilities", partAvailability(alive, !s.c2Node), descriptors.size() + missions.size(), [&] {
            for (const control::CapabilityDescriptor& d : descriptors) {
                const control::CapabilityStatus st = a.capabilityStatus(d.id);
                item(d.id, "vi", control::availabilityName(st.availability), control::reasonName(st.reason));
            }
            for (const ms::CapabilityInfo& c : missions) {
                const ms::CapabilityStatus st = a.missionCapabilityStatus(c.id);
                item(c.id, "ms", control::availabilityName(st.availability), ms::reasonName(st.reason));
            }
        });
    }

    // sensors
    {
        const std::vector<ms::PayloadInfo> payloads = alive && !s.c2Node ? a.payloads() : std::vector<ms::PayloadInfo>{};
        collection(w, "sensors", partAvailability(alive, !s.c2Node), payloads.size(), [&] {
            for (const ms::PayloadInfo& p : payloads) {
                w.beginObject().key("payload").id(p.index).member("name", p.name);
                if (p.card && *p.card) w.member("card", p.card);
                w.member("family", ms::familyName(p.family)).member("state", ms::payloadStateName(p.state)).key("radar");
                if (p.family != ms::Family::AirRadar) {
                    w.null();
                } else {
                    const ms::RadarStatus r = a.radarStatus(p.index);
                    w.beginObject().key("searching").boolean(r.searching).key("activity_id").idOrNull(r.activity);
                    w.member("mode", ms::amtiModeName(r.mode)).member("task", ms::amtiTaskName(r.task));
                    w.key("designated_track_id").idOrNull(r.designatedTrack).key("tracks").integer(r.tracks).key("tentative_tracks").integer(r.tentativeTracks);
                    w.endObject();
                }
                w.endObject();
            }
        });
    }

    // tracks, contacts, link tracks, peers: the legal report calls, with their provenance
    {
        const auto tracks = a.reportTracks(options);
        collection(w, "tracks", tracks.availability, tracks.items.size(), [&] {
            for (const auto& [meta, t] : tracks.items) {
                w.beginObject().key("track_id").id(t.id).key("payload").id(t.payload);
                writeMeta(w, meta);
                w.key("updates").integer(t.updates).key("latitude_rad").number(t.latitudeRad).key("longitude_rad").number(t.longitudeRad);
                w.key("height_m").number(t.heightM).key("velocity_ned_ms").vector3(t.velocityNedMs).key("position_sigma_m").number(t.positionSigmaM);
                w.key("velocity_sigma_ms").number(t.velocitySigmaMs).key("range_m").number(t.rangeM).key("azimuth_rad").number(t.azimuthRad);
                w.key("elevation_rad").number(t.elevationRad).key("range_rate_ms").number(t.rangeRateMs).endObject();
            }
        });
        const auto contacts = a.reportContacts(options);
        collection(w, "contacts", contacts.availability, contacts.items.size(), [&] {
            for (const auto& [meta, c] : contacts.items) {
                w.beginObject().key("contact_id").id(c.id).key("payload").id(c.payload);
                writeMeta(w, meta);
                w.member("kind", ms::contactKindName(c.kind)).member("mode", ms::emitterModeName(c.mode)).key("warning").boolean(c.warning);
                w.key("first_time_s").number(c.firstTimeS).key("azimuth_rad").number(c.azimuthRad).key("elevation_rad").number(c.elevationRad);
                w.key("bearing_sigma_rad").number(c.bearingSigmaRad).key("range_m").number(c.rangeM).key("range_rate_ms").number(c.rangeRateMs);
                w.key("latitude_rad").number(c.latitudeRad).key("longitude_rad").number(c.longitudeRad).key("height_m").number(c.heightM);
                w.key("strength_db").number(c.strengthDb).key("frequency_hz").number(c.frequencyHz).key("size_m").number(c.sizeM).endObject();
            }
        });
        const auto links = a.reportLinkTracks(options);
        collection(w, "link_tracks", links.availability, links.items.size(), [&] {
            for (const auto& [meta, l] : links.items) {
                w.beginObject().key("source_system_id").id(l.source).key("net_id").id(l.net).key("payload").id(l.report.payload).key("track_id").id(l.report.track);
                writeMeta(w, meta);
                w.key("updates").integer(l.report.updates).key("latitude_rad").number(l.report.latitudeRad).key("longitude_rad").number(l.report.longitudeRad);
                w.key("height_m").number(l.report.heightM).key("velocity_ned_ms").vector3(l.report.velocityNedMs);
                w.key("position_sigma_m").number(l.report.positionSigmaM).key("velocity_sigma_ms").number(l.report.velocitySigmaMs).endObject();
            }
        });
        const auto peers = a.reportPeers(options);
        collection(w, "peers", peers.availability, peers.items.size(), [&] {
            for (const auto& [meta, p] : peers.items) {
                w.beginObject().key("system_id").id(p.system).key("net_id").id(p.net);
                writeMeta(w, meta);
                w.key("tail_number");
                p.tailNumber.empty() ? w.null() : w.string(p.tailNumber);
                w.key("position");
                if (!p.hasPosition) {
                    w.null();
                } else {
                    const comm::PositionReport& q = p.position;
                    w.beginObject().key("time_s").number(q.timeS).key("latitude_rad").number(q.latitudeRad).key("longitude_rad").number(q.longitudeRad);
                    w.key("altitude_msl_m").number(q.altitudeMslM).key("velocity_ned_ms").vector3(q.velocityNedMs).key("heading_rad").number(q.headingRad);
                    w.key("airspeed_true_ms").number(q.airspeedTrueMs).key("airspeed_calibrated_ms").number(q.airspeedCalibratedMs).key("mach").number(q.mach);
                    w.endObject();
                }
                w.endObject();
            }
        });
    }

    // designations, activities, nets, transmissions
    {
        const std::vector<Designation> designations = alive && !s.c2Node ? a.designations() : std::vector<Designation>{};
        collection(w, "designations", partAvailability(alive, !s.c2Node), designations.size(), [&] {
            for (const Designation& d : designations)
                w.beginObject().key("target_id").id(d.target).key("payload").id(d.payload).key("track_id").id(d.track).key("valid").boolean(d.valid)
                    .key("source_system_id").idOrNull(d.source).endObject();
        });
        std::vector<control::ActivityRecord> flight;
        std::vector<ms::ActivityRecord> mission;
        std::vector<control::CapabilityDescriptor> descriptors;
        std::vector<ms::CapabilityInfo> instances;
        if (alive && !s.c2Node) flight = a.activities(), mission = a.missionActivities(), descriptors = a.capabilities(), instances = a.missionCapabilities();
        flight.erase(std::remove_if(flight.begin(), flight.end(), [](const control::ActivityRecord& r) { return r.id == 0; }), flight.end());
        mission.erase(std::remove_if(mission.begin(), mission.end(), [](const ms::ActivityRecord& r) { return r.id == 0; }), mission.end());
        const auto owner = [&](std::uint64_t activity) {
            const auto it = activityOwners_.find(activity);
            it == activityOwners_.end() ? w.null() : w.id(it->second);
        };
        collection(w, "activities", partAvailability(alive, !s.c2Node), flight.size() + mission.size(), [&] {
            for (const control::ActivityRecord& r : flight) {
                w.beginObject().member("domain", "vi").key("activity_id").id(r.id);
                if (r.capability < descriptors.size()) w.member("capability", descriptors[r.capability].id);
                w.member("state", control::activityStateName(r.state)).member("reason", control::reasonName(r.reason));
                w.key("controller_id").id(r.controller).key("command_id").id(r.commandId).key("owner_service_id");
                owner(r.id);
                w.key("start_time_s").number(r.startTime).key("end_time_s").number(r.endTime).key("percent").number(r.progress.percent).endObject();
            }
            for (const ms::ActivityRecord& r : mission) {
                w.beginObject().member("domain", "ms").key("activity_id").id(r.id);
                if (r.capability < instances.size()) w.member("capability", instances[r.capability].id);
                w.key("payload").id(r.payload).member("state", control::activityStateName(r.state)).member("reason", ms::reasonName(r.reason));
                w.key("controller_id").id(r.controller).key("command_id").id(r.commandId).key("owner_service_id");
                owner(r.id);
                w.key("start_time_s").number(r.startTime).key("end_time_s").number(r.endTime).key("percent").number(r.percent).endObject();
            }
        });
        const std::vector<comm::NetInfo> nets = alive ? a.nets() : std::vector<comm::NetInfo>{};
        collection(w, "nets", partAvailability(alive, true), nets.size(), [&] {
            for (const comm::NetInfo& n : nets)
                w.beginObject().key("net_id").id(n.id).member("name", n.name).key("max_payload_bytes").integer(n.spec.maxPayloadBytes).endObject();
        });
        std::vector<std::pair<std::uint64_t, comm::DmsTransmission>> sends;
        if (alive && s.config->ownsDmsEgress)
            for (const auto& [id, r] : s.records)
                for (const std::uint64_t m : r.messages)
                    if (const auto t = a.transmission(m)) sends.emplace_back(id, *t);
        collection(w, "transmissions", partAvailability(alive, true, s.config->ownsDmsEgress), sends.size(), [&] {
            for (const auto& [request, t] : sends) {
                w.beginObject().key("message_id").id(t.message).key("request_id").id(request).key("topic").id(t.topic).key("time_sent_s").number(t.timeSent);
                w.key("published").boolean(t.published).key("reached").integer(t.reached).key("queue_dropped").boolean(t.queueDropped);
                w.key("destinations").beginArray();
                for (const auto& [system, delivery] : t.destinations)
                    w.beginObject().key("system_id").id(system).member("delivery", deliveryName(delivery)).endObject();
                w.endArray().endObject();
            }
        });
    }

    // authority, tasks, overrides, command statuses
    {
        const ma::Report<C2Authority> authority = a.reportAuthority(options);
        const bool available = authority.availability == ma::Availability::Available;
        const C2Authority& c = authority.value;
        w.key("authority").beginObject().member("availability", ma::availabilityName(authority.availability));
        writeMeta(w, authority.meta);
        w.key("operators").beginArray();
        if (available)
            for (const C2Authority::Operator& op : c.operators)
                w.beginObject().key("system_id").id(op.system).key("role_id").id(op.roleId).member("role", op.role).member("callsign", op.callsign).endObject();
        w.endArray();
        w.key("designated").boolean(available && c.designated).key("designated_modes_mask").integer(available ? c.designatedModes : 0);
        w.key("designated_by_system_id").idOrNull(available ? c.designatedBy : 0);
        w.key("designated_time_s").number(available && c.designated ? c.designatedS : kNaN);
        w.key("commands").beginArray();
        if (available)
            for (const C2Authority::Command& cmd : c.commands)
                w.beginObject().key("activity_id").id(cmd.activity).key("system_id").id(cmd.system).key("command_id").id(cmd.commandId).endObject();
        w.endArray();
        const bool configured = available && c.lostCommsSet;
        w.key("lost_comms").beginObject().key("configured").boolean(configured).key("threshold_s").number(configured ? c.lostCommsThresholdS : kNaN);
        w.key("plan_id").idOrNull(configured ? c.lostCommsPlan : 0).key("lost").boolean(available && c.lost);
        w.key("lost_since_s").number(available && c.lost ? c.lostSinceS : kNaN).key("alerts").integer(available ? c.alerts : 0).endObject();
        w.key("package");
        if (!available || !c.packageId) {
            w.null();
        } else {
            w.beginObject().key("package_id").id(c.packageId).key("leader_system_id").idOrNull(c.leader).key("member_system_ids").beginArray();
            for (const comm::SystemId m : c.members) w.id(m);
            w.endArray().endObject();
        }
        w.endObject();

        const auto tasks = a.reportTasks(options);
        collection(w, "tasks", tasks.availability, tasks.items.size(), [&] {
            for (const auto& [meta, t] : tasks.items) {
                w.beginObject().key("from_system_id").id(t.from).key("task_id").id(t.order.taskId);
                writeMeta(w, meta);
                w.member("state", taskStateName(t.state)).key("percent").number(t.percent);
                const comm::TaskOrder& o = t.order;
                w.key("order").beginObject().member("type", taskTypeName(o.type)).key("priority").integer(o.priority);
                w.key("latitude_rad").number(o.latitudeRad).key("longitude_rad").number(o.longitudeRad).key("altitude_m").number(o.altitudeM);
                w.key("radius_m").number(o.radiusM).key("start_s").number(o.startS).key("end_s").number(o.endS).key("target");
                if (!o.targetSource) w.null();
                else
                    w.beginObject().key("source_system_id").id(o.targetSource).key("payload").id(o.targetPayload).key("track_id").id(o.targetTrack).endObject();
                w.member("text", std::string(o.text, strnlen(o.text, sizeof o.text))).endObject().endObject();
            }
        });
        const std::vector<comm::ActiveOverride> overrides = alive && !s.c2Node ? a.overrides() : std::vector<comm::ActiveOverride>{};
        collection(w, "overrides", partAvailability(alive, !s.c2Node), overrides.size(), [&] {
            for (const comm::ActiveOverride& o : overrides)
                w.beginObject().key("from_system_id").id(o.from).member("override", o.kind == comm::OverrideCommand::Speed ? "speed" : "altitude")
                    .key("value").number(o.value).key("expiry_s").number(o.expiryS).endObject();
        });
        const std::vector<comm::CommandStatusEntry> statuses = alive ? a.commandStatuses() : std::vector<comm::CommandStatusEntry>{};
        collection(w, "command_statuses", partAvailability(alive, true), statuses.size(), [&] {
            for (const comm::CommandStatusEntry& e : statuses) {
                w.beginObject().key("from_system_id").id(e.from).key("command_id").id(e.status.commandId);
                writeMeta(w, linkMeta(e.from, e.status.timeS, e.receivedS, now, config_->staleAfterS));
                w.key("topic").id(e.status.topic).member("state", answerName(e.status.state)).key("reason_code").integer(e.status.reason);
                w.key("activity_id").idOrNull(e.status.activity).key("percent").number(e.status.percent).endObject();
            }
        });
    }
    w.endObject(); // reports

    collection(w, "messages", partAvailability(alive, true, s.config->ownsDmsIngress), s.messages.size(), [&] {
        for (const comm::DmsReceived& m : s.messages)
            w.beginObject().key("message_id").id(m.message).key("source_system_id").id(m.source).key("topic").id(m.topic).key("net_id").id(m.net)
                .key("time_sent_s").number(m.timeSent).key("time_delivered_s").number(m.timeDelivered).key("byte_length").integer(static_cast<std::int64_t>(m.bytes.size()))
                .member("data_b64", encodeBase64(m.bytes.data(), m.bytes.size())).endObject();
    });
    w.key("receipts").beginArray();
    for (const auto& [id, r] : s.records) writeRecord(w, r);
    w.endArray().endObject();
    return w.take();
}

// --- the referee's documents ------------------------------------------------------------------------------------------

namespace {
/// Over the ground, on the mean Earth sphere (WGS-84's mean radius): the distance a region's radius is measured in.
double groundDistanceM(double lat1, double lon1, double lat2, double lon2) noexcept {
    constexpr double kR = 6371008.8;
    const double dLat = lat2 - lat1, dLon = lon2 - lon1;
    const double h = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2) * std::sin(dLon / 2);
    return 2.0 * kR * std::asin(std::min(1.0, std::sqrt(h)));
}
} // namespace

void Gateway::judge() {
    const Referee referee = facade_->referee();
    const double now = facade_->time();
    const auto entity = [&](const std::string& name) -> std::uint32_t {
        for (const Platform& p : platforms_)
            if (p.name == name) return p.entity;
        return 0;
    };
    for (std::size_t i = 0; i < config_->criteria.size(); ++i) {
        const Criterion& c = config_->criteria[i];
        Judged& j = criteria_[i];
        bool holds = false;
        switch (c.kind) {
        case Criterion::Kind::TickLimit: holds = tick_ >= c.maxTicks; break;
        case Criterion::Kind::RegionReached: {
            const VehicleState* st = referee.state(entity(c.platform));
            const bool inside = st && groundDistanceM(st->latitudeRad, st->longitudeRad, c.latitudeRad, c.longitudeRad) <= c.radiusM &&
                                st->altitudeMslM >= c.altitudeMinM && st->altitudeMslM <= c.altitudeMaxM;
            if (inside && !j.inside) j.insideSinceS = now;
            j.inside = inside;
            holds = inside && now - j.insideSinceS >= c.dwellS - 1e-9;
            break;
        }
        case Criterion::Kind::C2TaskState: {
            const comm::SystemId from = referee.entitySystem(entity(c.fromPlatform));
            if (const auto* tasks = facade_->impl().maCore().tasks(entity(c.platform)); tasks && from)
                for (const comm::ReceivedTask& t : *tasks) holds = holds || (t.from == from && t.state == c.state);
            break;
        }
        }
        if (holds && !j.satisfied) j.satisfied = true, j.tick = tick_;
    }
}

void Gateway::freeze() {
    for (auto& s : services_) {
        // each message is ingested once, by its platform's ingress owner, into this snapshot alone
        s->messages.clear();
        if (s->config->ownsDmsIngress && s->agent.valid()) s->messages = s->agent.receive();
        s->observation = observe(*s);
    }
    const Referee referee = facade_->referee();
    const double now = facade_->time();

    // outcome: section 13's rule on the criteria as judged
    bool failure = false, anySuccess = false, allSuccess = true, truncated = false;
    for (std::size_t i = 0; i < config_->criteria.size(); ++i) {
        const Criterion& c = config_->criteria[i];
        const bool sat = criteria_[i].satisfied;
        if (c.outcome == Criterion::Outcome::Failure) failure = failure || sat;
        if (c.outcome == Criterion::Outcome::Success) anySuccess = true, allSuccess = allSuccess && sat;
        if (c.outcome == Criterion::Outcome::Truncated) truncated = truncated || sat;
    }
    const char* reason = failure ? "failure" : anySuccess && allSuccess ? "success" : truncated ? "truncated" : nullptr;
    Writer o;
    o.beginObject().member("schema", "ma.outcome.v1").key("episode").raw(key_).key("tick").id(tick_).key("time_s").number(now);
    o.key("episode_done").boolean(reason != nullptr).key("reason");
    reason ? o.string(reason) : o.null();
    o.key("criteria").beginArray();
    static const char* const kinds[] = {"tick_limit", "region_reached", "c2_task_state"};
    static const char* const outcomes[] = {"success", "failure", "truncated"};
    for (std::size_t i = 0; i < config_->criteria.size(); ++i) {
        const Criterion& c = config_->criteria[i];
        o.beginObject().member("id", c.id).member("kind", kinds[static_cast<int>(c.kind)]).member("outcome", outcomes[static_cast<int>(c.outcome)]);
        o.key("satisfied").boolean(criteria_[i].satisfied).key("satisfied_tick");
        criteria_[i].satisfied ? o.id(criteria_[i].tick) : o.null();
        o.endObject();
    }
    o.endArray().key("roles").beginArray();
    const bool decided = reason && std::string_view(reason) != "truncated";
    for (const auto& s : services_) {
        const bool alive = s->agent.valid();
        const bool terminated = !alive || decided;
        const bool cut = reason && !terminated;
        o.beginObject().key("service_id").id(s->config->serviceId).key("active").boolean(alive && !reason).key("terminated").boolean(terminated);
        o.key("truncated").boolean(cut).key("final_observation").boolean(terminated || cut).key("bootstrap").boolean(cut).endObject();
    }
    o.endArray().endObject();
    outcome_ = o.take();

    // truth: every entity, every service's identity, every net's traffic
    Writer t;
    t.beginObject().member("schema", "ma.truth.v1").key("episode").raw(key_).key("tick").id(tick_).key("time_s").number(now);
    t.key("entities").beginArray();
    const std::vector<std::uint32_t> vehicles = referee.vehicles();
    const std::vector<std::uint32_t> nodes = referee.c2Nodes();
    for (const std::uint32_t id : vehicles) {
        const VehicleState* st = referee.state(id);
        t.beginObject().key("entity_id").id(id).member("name", facade_->vehicle(id).name()).member("kind", "vehicle").key("alive").boolean(true);
        t.key("system_id").idOrNull(referee.entitySystem(id)).key("state");
        if (!st) {
            t.null();
        } else {
            t.beginObject().key("latitude_rad").number(st->latitudeRad).key("longitude_rad").number(st->longitudeRad).key("altitude_msl_m").number(st->altitudeMslM);
            t.key("velocity_ned_ms").vector3(st->velocityNedMs).key("euler_rad").vector3(st->eulerRad).key("airspeed_true_ms").number(st->airspeedTrueMs);
            t.key("on_ground").boolean(st->onGround).endObject();
        }
        t.endObject();
    }
    for (const Platform& p : platforms_) // (a configured platform that is gone: listed, not alive)
        if (!p.c2Node && std::find(vehicles.begin(), vehicles.end(), p.entity) == vehicles.end() && p.entity)
            t.beginObject().key("entity_id").id(p.entity).member("name", p.name).member("kind", "vehicle").key("alive").boolean(false)
                .key("system_id").idOrNull(referee.entitySystem(p.entity)).key("state").null().endObject();
    for (const std::uint32_t id : nodes) {
        const auto spec = referee.c2Node(id);
        t.beginObject().key("entity_id").id(id).member("name", spec ? spec->name : std::string()).member("kind", "c2_node").key("alive").boolean(true);
        t.key("system_id").idOrNull(referee.entitySystem(id)).key("state").null().endObject();
    }
    for (const Platform& p : platforms_)
        if (p.c2Node && std::find(nodes.begin(), nodes.end(), p.entity) == nodes.end())
            t.beginObject().key("entity_id").id(p.entity).member("name", p.name).member("kind", "c2_node").key("alive").boolean(false)
                .key("system_id").idOrNull(referee.entitySystem(p.entity)).key("state").null().endObject();
    t.endArray().key("services").beginArray();
    for (const auto& s : services_)
        t.beginObject().key("service_id").id(s->config->serviceId).member("name", s->config->name).key("entity_id").id(s->platform)
            .key("controller_id").id(s->agent.controller()).endObject();
    t.endArray().key("nets").beginArray();
    for (const comm::NetInfo& n : referee.nets()) {
        const comm::NetStats st = referee.netStats(n.id);
        t.beginObject().key("net_id").id(n.id).member("name", n.name).key("transmissions").id(st.transmissions).key("delivered").id(st.delivered);
        t.key("duplicates").id(st.duplicates).key("out_of_range").id(st.outOfRange).key("masked").id(st.masked).key("lost").id(st.lost);
        t.key("queue_dropped").id(st.queueDropped).key("failed").id(st.failed).key("overflowed").id(st.overflowed).endObject();
    }
    t.endArray().endObject();
    truth_ = t.take();
}

} // namespace fsim::mission
