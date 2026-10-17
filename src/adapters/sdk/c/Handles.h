#pragma once

// The C ABI's opaque handles (shared by the translation units of fsim.dll).

#include "fsim/PerformanceProfile.h"
#include "fsim/Capability.h"
#include "fsim/World.h"
#include "fsim/fsim_c.h"
#include "simulation/runtime/services/AgentRules.h"
#include "LastError.h"
#include "simulation/runtime/world/World.h"

#include <atomic>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

struct fsim_world {
    std::shared_ptr<fsim::session::World> owned; ///< null when the world belongs to something else (a VecEnv)
    fsim::session::World& world;
    /// AB-2d: sealed (fsim_world_seal), this handle answers nothing but its own release: the range goes on through the
    /// handle the seal gave it, which no code holding this one has. (Beside `world`, as `agent`: the guards read
    /// them together.)
    bool restricted = false; ///< an agent's or a sealed handle: the one flag every guard reads (the range's passes on it)
    bool sealed = false;
    bool successor = false; ///< the handle a seal gave: freed with fsim_world_destroy, even over a world it does not own
    /// ADR-31: set, this handle is an agent's - bound to one vehicle, commanding as its own controller; every C function
    /// says first whose call it is (FSIM_NOT_AGENT, FSIM_AGENT_*), and an agent's handle answers only its own. (Beside
    /// `world`, which every call reads: its guard costs no other cache line.)
    std::optional<fsim::session::AgentIdentity> agent;
    std::unique_ptr<fsim::World> object;          ///< the C++ view handed out by fsim_world_object(), made on demand
    std::unordered_set<std::string> strings;      ///< strings handed out (capability ids, parameter names): stable until the world goes
    std::vector<fsim::ms::WeaponRecord> weapons;  ///< the weapons as fsim_world_weapon_count last took them
    std::vector<fsim::ms::EngagementEvent> combatEvents;
    std::vector<fsim::ms::DecoyState> decoys;
    fsim::control::CommandResult last;             ///< the last answer to a NEW, UPDATE or CANCEL (fsim_last_command_detail)
    uint32_t lastVehicle = 0;                      ///< the vehicle that gave it: its details (World::commandDetails) are the last command's
    std::vector<fsim::control::Waypoint> waypoints; ///< a route's, as given (reused: a route updated every step allocates nothing once it has room)
    std::vector<fsim::control::BezierSegment> segments; ///< a curve's, as given (reused likewise)
    std::vector<fsim::control::NurbsSegment> nurbs;     ///< a curve's as A-GRA's schema gives them (likewise)
    std::vector<fsim::control::RouteLoiter> loiters;    ///< a route's loiters (likewise)
    std::vector<fsim::control::RouteState> states;      ///< a route's planned states (likewise)
    std::vector<fsim::control::RoutePath> paths;        ///< a route's paths (likewise)
    std::vector<fsim::control::RouteBranch> branches;   ///< a route's conditional branches (likewise)
    std::vector<fsim::control::RouteTerminator> terminators; ///< a route's civil path terminators' data (likewise)
    /// An activity's setpoint as last read back (fsim_activity_get_setpoint): what its arrays point into.
    struct Readback {
        fsim::control::Setpoint setpoint;
        std::vector<double> fields;
        std::vector<fsim_waypoint> waypoints;
        std::vector<fsim_bezier_segment> segments;
        std::vector<fsim_nurbs_segment> nurbs;
        std::vector<fsim_route_loiter> loiters;
        std::vector<fsim_route_state> states;
        std::vector<fsim_route_path> paths;
        std::vector<fsim_route_branch> branches;
        std::vector<fsim_route_terminator> terminators;
        fsim_behavior_command behavior{};
        std::vector<const char*> names;
        std::vector<double> values;
        std::vector<fsim_position_command> points;
    } readback;
    /// A route plan as last read back (fsim_vehicle_get_plan): what its metadata's arrays and texts point into (its route's
    /// arrays: `readback`'s).
    struct PlanReadback {
        fsim::control::RoutePlan plan;
        std::vector<fsim_point_metadata> points;
        std::vector<fsim_path_metadata> paths;
    } planReadback;
    /// An airfield as last read back (fsim_vehicle_get_airfield, _at): what its runways and ICAO code point into.
    struct AirfieldReadback {
        fsim::control::Airfield airfield;
        std::vector<fsim_runway> runways;
    } airfieldReadback;
    /// An operational zone as last read back (fsim_world_get_op_zone, _at): what its vertices and holes point into.
    struct ZoneReadback {
        std::vector<fsim_zone_vertex> vertices;
        std::vector<std::vector<fsim_zone_vertex>> holes;
        std::vector<const fsim_zone_vertex*> holePointers;
        std::vector<uint32_t> holeSizes;
    } zoneReadback;
    /// An operational line as last read back (fsim_world_get_op_line, _at): what its vertices point into.
    std::vector<fsim_line_vertex> lineReadback;
    /// The performance profile last asked for (fsim_vehicle_performance_profile): what its arrays point into.
    fsim::control::PerformanceProfile profile;
    /// An MA service's capabilities as last read back (fsim_world_ma_service): what its array points into.
    std::vector<const char*> serviceCapabilities;

    const char* intern(const std::string& s) { return strings.insert(s).first->c_str(); }

    explicit fsim_world(const fsim::session::WorldOptions& o) : owned(std::make_shared<fsim::session::World>(o)), world(*owned) {}
    explicit fsim_world(fsim::session::World& borrowed) : world(borrowed) {}
    /// An agent's handle on a world: it keeps the world alive where the world is a program's own (a VecEnv's is its own).
    fsim_world(const fsim_world& range, fsim::session::AgentIdentity identity)
        : owned(range.owned), world(range.world), restricted(true), agent(std::move(identity)) {}
    /// The range's handle a seal gives (AB-2d): the same world.
    struct Successor {};
    fsim_world(const fsim_world& before, Successor) : owned(before.owned), world(before.world), successor(true) {}
};

namespace fsim::sdk {

using session::AgentIdentity;
using session::agentActivity;
using session::agentCaller;
using session::notAgents;

/// Why a sealed handle answers nothing (AB-2d).
inline std::string sealedHandle(const char* call) {
    return std::string(call) + ": a sealed handle - the range goes on through the handle fsim_world_seal gave it (ADR-31)";
}
/// Refuse a call: fsim_last_error says why.
inline bool refuse(const fsim_world* w, const char* call) {
    lastError() = w->sealed ? sealedHandle(call) : notAgents(call);
    return true;
}
/// Count an agent's call in its audit (AB-2d): `call`, the function's index (session::auditCall).
inline void audit(const fsim_world* w, std::uint16_t call, bool refused) { w->world.audit(w->agent->record, call, refused); }

/// An agent's handle declaring a source and controller (the _by forms): the policy's and its own, else refused; 0 is
/// rewritten to its own controller.
inline bool agentClaims(const fsim_world* w, int& source, uint32_t& controller, const char* call) {
    if (!w || !w->agent || agentCaller(*w->agent, source, controller)) return false;
    lastError() = std::string(call) + ": an agent commands as itself - the policy's source and its own controller (ADR-31)";
    return true;
}
inline bool agentClaims(const fsim_world* w, uint32_t& controller, const char* call) {
    int source = 0;
    return agentClaims(w, source, controller, call);
}
/// An agent's handle giving options that claim another source or controller.
inline bool agentClaims(const fsim_world* w, const fsim_command_options* o, const char* call) {
    if (!w || !w->agent || !o) return false;
    int source = o->source;
    uint32_t controller = o->struct_size >= offsetof(fsim_command_options, controller) + sizeof(o->controller) ? o->controller : 0;
    return agentClaims(w, source, controller, call);
}
inline bool agentClaims(const fsim_world* w, const fsim_ms_options* o, const char* call) {
    if (!w || !w->agent || !o) return false;
    int source = o->struct_size >= sizeof(uint32_t) * 2 ? o->source : 0;
    uint32_t controller = o->struct_size >= sizeof(uint32_t) * 3 ? o->controller : 0;
    return agentClaims(w, source, controller, call);
}
/// An agent's handle on a platform with configured MA services (docs/sdk/mission-training.md, section 9), calling on a
/// DMS side another service owns: refused, counted in its audit, fsim_last_error saying whose it is.
inline bool notDmsOwner(const fsim_world* w, bool egress, const char* call) {
    if (!w || !w->agent) [[likely]]
        return false;
    if (egress ? session::agentDmsEgress(*w->agent, w->world) : session::agentDmsIngress(*w->agent, w->world)) return false;
    w->world.auditRefusal(w->agent->record, session::auditCall(call));
    lastError() = std::string(call) + ": the platform's DMS " + (egress ? "egress" : "ingress") + " is another configured MA service's";
    return true;
}
/// An agent's handle addressing another vehicle's activity (a batch's: each).
inline bool outsideAgentActivity(const fsim_world* w, uint64_t activity, const char* call) {
    if (!w || !w->agent || agentActivity(*w->agent, activity, &w->world)) [[likely]]
        return false;
    lastError() = notAgents(call);
    return true;
}

} // namespace fsim::sdk

// ADR-31: the first statement of every C function an fsim_world* reaches says whose call it is - checked by
// tools/validation/boundary/boundary.py against docs/sdk/boundary.toml. The trailing argument is what a refused call returns. The
// range's handle passes at the cost of one predictable branch; an agent's handle has each call counted in its audit
// (AB-2d), refused or not; a sealed handle is refused everything but its release.
/// What a guard asks of a restricted handle (an agent's, or a sealed one).
namespace fsim::sdk {
enum class Guard : std::uint8_t { NotAgent, Call, Vehicle, Activity };
/// A restricted handle's guard, out of line and cold so the range's handle pays one predictable branch: the call counted
/// in an agent's audit (`call` its index, given on its first call), refused - true - where it is not an agent's, is out of
/// the agent's scope (`scope`: a vehicle or an activity), or the handle is sealed.
[[gnu::cold, gnu::noinline]] inline bool guarded(const fsim_world* w, const char* func, Guard guard, std::uint64_t scope,
                                                std::atomic<std::uint16_t>& call) {
    if (w->sealed) return refuse(w, func);
    std::uint16_t index = call.load(std::memory_order_relaxed);
    if (index == 0xFFFFu) call.store(index = session::auditCall(func), std::memory_order_relaxed);
    const bool refused = guard == Guard::NotAgent ||
                         (guard == Guard::Vehicle && static_cast<std::uint32_t>(scope) != w->agent->vehicle) ||
                         (guard == Guard::Activity && !agentActivity(*w->agent, scope, &w->world));
    audit(w, index, refused);
    return refused && refuse(w, func);
}
/// A refusal of an agent's claims (FSIM_AGENT_AS), counted.
[[gnu::cold, gnu::noinline]] inline void refusedClaim(const fsim_world* w, const char* func, std::atomic<std::uint16_t>& call) {
    std::uint16_t index = call.load(std::memory_order_relaxed);
    if (index == 0xFFFFu) call.store(index = session::auditCall(func), std::memory_order_relaxed);
    w->world.auditRefusal(w->agent->record, index);
}
} // namespace fsim::sdk

/// A guard of a kind: the range's handle passes on one flag; a restricted one goes out of line.
#define FSIM_GUARD_(w, kind, scope, ...)                                                                                    \
    do {                                                                                                                    \
        if ((w) && (w)->restricted) [[unlikely]] {                                                                          \
            static std::atomic<std::uint16_t> fsim_call_{0xFFFFu};                                                          \
            if (::fsim::sdk::guarded((w), __func__, ::fsim::sdk::Guard::kind, (scope), fsim_call_)) return __VA_ARGS__;     \
        }                                                                                                                   \
    } while (0)
/// Not an agent's (the range's, the referee's, the platform's, the MA core's): an agent's handle is refused.
#define FSIM_NOT_AGENT(w, ...) FSIM_GUARD_(w, NotAgent, 0u, __VA_ARGS__)
/// An agent's, needing no scope (its world's time, the terrain, what it reads and writes of its own).
#define FSIM_AGENT_CALL(w, ...) FSIM_GUARD_(w, Call, 0u, __VA_ARGS__)
/// An agent's, on its own vehicle (`id`) alone.
#define FSIM_AGENT_VEHICLE(w, id, ...) FSIM_GUARD_(w, Vehicle, static_cast<std::uint64_t>(id), __VA_ARGS__)
/// An agent's, on its own vehicle's activities alone.
#define FSIM_AGENT_ACTIVITY(w, activity, ...) FSIM_GUARD_(w, Activity, static_cast<std::uint64_t>(activity), __VA_ARGS__)
/// After its guard: a call on its platform's DMS ingress (its inbox, its subscriptions) or egress (its sends), refused an
/// agent that is not that side's configured owner. `...` is what the refused call returns.
#define FSIM_MA_INGRESS(w, ...)                                                                                             \
    do {                                                                                                                    \
        if (::fsim::sdk::notDmsOwner((w), false, __func__)) return __VA_ARGS__;                                            \
    } while (0)
#define FSIM_MA_EGRESS(w, reason, ...)                                                                                      \
    do {                                                                                                                    \
        if (::fsim::sdk::notDmsOwner((w), true, __func__)) {                                                               \
            if (reason) *(reason) = FSIM_DMS_NOT_OWNER;                                                                     \
            return __VA_ARGS__;                                                                                             \
        }                                                                                                                   \
    } while (0)
/// Any handle's, sealed or not: its own release, its identity.
#define FSIM_AGENT_RELEASE(w) ((void)(w))
/// An agent's handle: a declared source and controller must be its own (`controller` 0 becomes its own; the
/// arguments are rewritten), or options must claim nothing else. A refusal is counted in its audit.
#define FSIM_AGENT_AS(w, ...) FSIM_AGENT_CLAIMS_((w), __VA_ARGS__)
#define FSIM_AGENT_CLAIMS_(w, ...) FSIM_AGENT_PICK_(__VA_ARGS__, FSIM_AGENT_AS3_, FSIM_AGENT_AS2_, )(w, __VA_ARGS__)
#define FSIM_AGENT_PICK_(a, b, c, name, ...) name
#define FSIM_AGENT_AS3_(w, source, controller, fail)                                                                       \
    do {                                                                                                                    \
        if (::fsim::sdk::agentClaims((w), (source), (controller), __func__)) {                                              \
            static std::atomic<std::uint16_t> fsim_call_{0xFFFFu};                                                          \
            ::fsim::sdk::refusedClaim((w), __func__, fsim_call_);                                                           \
            return fail;                                                                                                    \
        }                                                                                                                   \
    } while (0)
#define FSIM_AGENT_AS2_(w, claim, fail)                                                                                     \
    do {                                                                                                                    \
        if (::fsim::sdk::agentClaims((w), (claim), __func__)) {                                                             \
            static std::atomic<std::uint16_t> fsim_call_{0xFFFFu};                                                          \
            ::fsim::sdk::refusedClaim((w), __func__, fsim_call_);                                                           \
            return fail;                                                                                                    \
        }                                                                                                                   \
    } while (0)
