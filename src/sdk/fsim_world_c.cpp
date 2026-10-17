// C ABI for the World / vehicle object model (design 9.8) over session::World.

#include "fsim/BuiltinEffects.h"
#include "fsim/Comm.h"
#include "fsim/Recording.h"
#include "fsim/Scenario.h"
#include "fsim/fsim_c.h"

#include "simulation/flight_control/Catalog.h"
#include "fsim/Magnetic.h"
#include "foundation/core/Log.h"
#include "sdk/CAbiSupport.h"
#include "sdk/CParameterSupport.h"
#include "sdk/Handles.h"
#include "sdk/LastError.h"
#include "simulation/runtime/world/Scenario.h"
#include "simulation/runtime/world/World.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <exception>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

// The C state struct must mirror fsim::VehicleState exactly.
static_assert(sizeof(fsim_vehicle_state) == sizeof(fsim::sim::VehicleState), "fsim_vehicle_state layout differs from VehicleState");
static_assert(offsetof(fsim_vehicle_state, rotation_body_to_ecef) == offsetof(fsim::sim::VehicleState, rotationBodyToEcef), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_count) == offsetof(fsim::sim::VehicleState, engineCount), "layout");
static_assert(offsetof(fsim_vehicle_state, speedbrake_position) == offsetof(fsim::sim::VehicleState, speedbrakePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, throttle_position) == offsetof(fsim::sim::VehicleState, throttlePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, on_ground) == offsetof(fsim::sim::VehicleState, onGround), "layout");
static_assert(offsetof(fsim_vehicle_state, crashed) == offsetof(fsim::sim::VehicleState, crashed), "layout");
static_assert(offsetof(fsim_vehicle_state, engine_rpm) == offsetof(fsim::sim::VehicleState, engineRpm), "layout");
static_assert(offsetof(fsim_vehicle_state, nozzle_position) == offsetof(fsim::sim::VehicleState, nozzlePosition), "layout");
static_assert(offsetof(fsim_vehicle_state, leading_edge_flap_rad) == offsetof(fsim::sim::VehicleState, leadingEdgeFlapRad), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_count) == offsetof(fsim::sim::VehicleState, wheelCount), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_compression_m) == offsetof(fsim::sim::VehicleState, wheelCompressionM), "layout");
static_assert(offsetof(fsim_vehicle_state, wheel_speed_ms) == offsetof(fsim::sim::VehicleState, wheelSpeedMs), "layout");
static_assert(FSIM_MAX_WHEELS == fsim::sim::VehicleState::kMaxWheels, "layout");
static_assert(sizeof(bool) == 1, "bool must be one byte for the C mirror of VehicleState");
static_assert(sizeof(fsim_control_inputs) == sizeof(fsim::ControlInputs), "fsim_control_inputs layout differs from ControlInputs");
static_assert(offsetof(fsim_control_inputs, brake_right) == offsetof(fsim::ControlInputs, brakeRight), "layout");

namespace {



double param(const std::vector<std::pair<std::string, double>>& p, const char* name, double fallback) {
    for (const auto& [k, v] : p)
        if (k == name) return v;
    return fallback;
}

fsim::sim::InitialConditions initialOf(const fsim_vehicle_spec& s) {
    fsim::sim::InitialConditions ic;
    ic.latitudeDeg = s.latitude_deg;
    ic.longitudeDeg = s.longitude_deg;
    ic.altitudeMslM = s.altitude_msl_m;
    ic.headingDeg = s.heading_deg;
    ic.pitchDeg = s.pitch_deg;
    ic.rollDeg = s.roll_deg;
    ic.airspeedTrueMs = s.airspeed_ms;
    ic.onGround = s.on_ground != 0;
    ic.fuelFraction = s.fuel_fraction;
    return ic;
}

/// The caller's spec whole: what its struct_size does not reach is as fsim_vehicle_spec_init leaves it (a caller built
/// before the field was there). False: none, or one shorter than the first there was.
bool wholeSpec(const fsim_vehicle_spec* given, fsim_vehicle_spec& whole) {
    if (!given || given->struct_size < offsetof(fsim_vehicle_spec, fuel_fraction)) return false;
    fsim_vehicle_spec_init(&whole);
    std::memcpy(&whole, given, std::min<std::size_t>(given->struct_size, sizeof(whole)));
    whole.struct_size = sizeof(whole);
    return true;
}

const fsim_vehicle_state* asC(const fsim::sim::VehicleState* s) noexcept { return reinterpret_cast<const fsim_vehicle_state*>(s); }


} // namespace

extern "C" {


FSIM_API void fsim_world_options_init(fsim_world_options* o) {
    if (!o) return;
    std::memset(o, 0, sizeof(*o));
    o->struct_size = sizeof(*o);
    o->name = "default";
    o->dt = 1.0 / 120.0;
    o->frame_skip = 4;
    o->pin_workers = 1;
    o->capacity = 256;
    o->publish = 1;
    o->publish_interval_s = 1.0 / 60.0;
    o->terrain_zoom = 15;
}

FSIM_API void fsim_vehicle_spec_init(fsim_vehicle_spec* s) {
    if (!s) return;
    std::memset(s, 0, sizeof(*s));
    s->struct_size = sizeof(*s);
    s->type = "jsbsim:c172x";
    const fsim::sim::InitialConditions ic;
    s->latitude_deg = ic.latitudeDeg;
    s->longitude_deg = ic.longitudeDeg;
    s->altitude_msl_m = ic.altitudeMslM;
    s->heading_deg = ic.headingDeg;
    s->airspeed_ms = ic.airspeedTrueMs;
    s->control_divider = 1;
    s->fuel_fraction = ic.fuelFraction;
}


FSIM_API int fsim_world_create(const fsim_world_options* options, fsim_world** out) {
    // (a caller's struct before 1.46 ends at record_interval_s)
    constexpr std::size_t kBeforeOffline = offsetof(fsim_world_options, record_interval_s) + sizeof(double);
    if (!options || !out || options->struct_size < kBeforeOffline) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create: bad arguments");
    *out = nullptr;
    return guard("fsim_world_create", [&] {
        fsim::session::WorldOptions o;
        if (options->name) o.name = options->name;
        o.dt = options->dt;
        o.frameSkip = options->frame_skip;
        o.workers = options->workers;
        o.pinWorkers = options->pin_workers != 0;
        o.seed = options->seed;
        o.capacity = options->capacity;
        o.publish = options->publish != 0;
        o.publishIntervalSeconds = options->publish_interval_s;
        if (options->jsbsim_root) o.jsbsimRoot = options->jsbsim_root;
        o.terrain = options->terrain != 0;
        if (options->terrain_url) o.terrainUrl = options->terrain_url;
        o.terrainZoom = options->terrain_zoom ? options->terrain_zoom : 15u;
        if (options->record_path) o.recordPath = options->record_path;
        o.recordIntervalSeconds = options->record_interval_s;
        if (options->struct_size >= offsetof(fsim_world_options, terrain_offline) + sizeof(int32_t))
            o.terrainOffline = options->terrain_offline != 0;
        if (options->struct_size >= offsetof(fsim_world_options, store_drag) + sizeof(int32_t)) o.storeDrag = options->store_drag != 0;
        *out = new fsim_world(o);
        return FSIM_OK;
    });
}

FSIM_API void fsim_world_destroy(fsim_world* world) {
    FSIM_AGENT_RELEASE(world); // (an agent frees its own handle, a sealed one its own; the world goes with the range's)
    if (world && !world->owned && !world->agent && !world->successor) return; // a borrowed handle (fsim_vecenv_world) belongs to its owner
    delete world;
}

FSIM_API int fsim_agent_create(fsim_world* world, uint32_t vehicle, const char* name, fsim_world** agent) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !agent) return FSIM_INVALID_ARGUMENT;
    *agent = nullptr;
    const std::string n = name ? name : "";
    const auto controller = world->world.mintAgent(vehicle, n);
    if (controller == 0) return fail(FSIM_INVALID_ARGUMENT, "fsim_agent_create: vehicle " + std::to_string(vehicle) + " is not a live aircraft");
    *agent = new fsim_world(*world, fsim::sdk::AgentIdentity{vehicle, controller, n, world->world.agents().size() - 1});
    return FSIM_OK;
}

FSIM_API int fsim_world_seal(fsim_world* world, fsim_world** range) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !range) return FSIM_INVALID_ARGUMENT;
    *range = new fsim_world(*world, fsim_world::Successor{});
    world->sealed = world->restricted = true;
    return FSIM_OK;
}

FSIM_API void fsim_agent_audit_init(fsim_agent_audit* audit) {
    if (!audit) return;
    std::memset(audit, 0, sizeof *audit);
    audit->struct_size = sizeof *audit;
    audit->name = "";
}

FSIM_API uint32_t fsim_world_agent_count(const fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world ? static_cast<uint32_t>(world->world.agents().size()) : 0u;
}

FSIM_API int fsim_world_agent_audit(fsim_world* world, uint32_t index, fsim_agent_audit* out, fsim_audit_call* calls, uint32_t max) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out || (max && !calls) || index >= world->world.agents().size()) return FSIM_INVALID_ARGUMENT;
    const auto& a = world->world.agents()[index];
    const std::vector<std::string> names = fsim::session::auditNames();
    out->vehicle = a.vehicle, out->controller = a.controller, out->name = world->intern(a.name);
    uint32_t made = 0;
    for (std::size_t i = 0; i < a.calls.size(); ++i) {
        if (a.calls[i] == 0 && a.refused[i] == 0) continue;
        if (made < max)
            calls[made] = fsim_audit_call{world->intern(i < names.size() ? names[i] : std::string("?")), a.calls[i], a.refused[i]};
        ++made;
    }
    out->call_count = made;
    return FSIM_OK;
}

FSIM_API void fsim_ownship_init(fsim_ownship* o) {
    if (!o) return;
    std::memset(o, 0, sizeof *o);
    o->struct_size = sizeof *o;
    o->load_factor = 1.0, o->gnss_valid = 1, o->air_data_valid = 1;
}

FSIM_API int fsim_vehicle_ownship(const fsim_world* world, uint32_t id, fsim_ownship* out) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* s = world && out ? world->world.sensedState(id) : nullptr;
    if (!s || out->struct_size < offsetof(fsim_ownship, throttle)) return FSIM_INVALID_ARGUMENT;
    const fsim::OwnShip o = fsim::session::ownShipOf(*s);
    out->time_s = o.timeS;
    out->latitude_rad = o.latitudeRad, out->longitude_rad = o.longitudeRad, out->altitude_msl_m = o.altitudeMslM;
    out->altitude_agl_m = o.altitudeAglM;
    for (int i = 0; i < 3; ++i) {
        out->velocity_ned_ms[i] = o.velocityNedMs[i], out->euler_rad[i] = o.eulerRad[i];
        out->angular_rate_body_rad_s[i] = o.angularRateBodyRadS[i], out->acceleration_body_ms2[i] = o.accelerationBodyMs2[i];
    }
    out->airspeed_true_ms = o.airspeedTrueMs, out->airspeed_calibrated_ms = o.airspeedCalibratedMs, out->mach = o.mach;
    out->alpha_rad = o.alphaRad, out->beta_rad = o.betaRad, out->load_factor = o.loadFactor, out->fuel_kg = o.fuelKg;
    out->on_ground = o.onGround ? 1 : 0, out->gnss_valid = o.gnssValid ? 1 : 0, out->air_data_valid = o.airDataValid ? 1 : 0;
    if (out->struct_size >= offsetof(fsim_ownship, throttle) + sizeof(out->throttle)) out->throttle = o.throttle;
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_designate(fsim_world* world, uint32_t id, uint32_t payload, uint64_t track, uint32_t* target) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !target || payload > 0xFFFFu || !world->world.vehicleState(id)) return FSIM_INVALID_ARGUMENT;
    *target = world->world.designate(id, static_cast<std::uint16_t>(payload), track);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_undesignate(fsim_world* world, uint32_t id, uint32_t target) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !world->world.undesignate(id, target)) return FSIM_INVALID_ARGUMENT;
    return FSIM_OK;
}

FSIM_API void fsim_designation_init(fsim_designation* d) {
    if (!d) return;
    std::memset(d, 0, sizeof *d);
    d->struct_size = sizeof *d;
}

FSIM_API int fsim_vehicle_designations(const fsim_world* world, uint32_t id, fsim_designation* out, uint32_t max, uint32_t* count) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !count || (max && !out)) return FSIM_INVALID_ARGUMENT;
    const auto list = world->world.designations(id);
    for (std::size_t i = 0; i < list.size() && i < max; ++i) {
        fsim_designation_init(&out[i]);
        out[i].target = list[i].id, out[i].payload = list[i].payload, out[i].track = list[i].track, out[i].valid = list[i].valid ? 1 : 0;
    }
    *count = static_cast<uint32_t>(list.size());
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_designation_truth(const fsim_world* world, uint32_t target) {
    FSIM_NOT_AGENT(world, 0);
    const auto* d = world ? world->world.designation(target) : nullptr;
    if (!d || !d->valid) return 0;
    // (a network track's: its source system's radar track - AB-4b)
    const uint32_t holder = d->source ? world->world.dms().entityOf(d->source) : d->owner;
    for (const auto& t : world->world.amtiTracks(holder, d->payload))
        if (t.id == d->track) return t.truth;
    return 0;
}

FSIM_API int fsim_agent_identity(const fsim_world* agent, uint32_t* vehicle, uint32_t* controller, const char** name) {
    FSIM_AGENT_RELEASE(agent);
    if (!agent || !agent->agent) return 0;
    if (vehicle) *vehicle = agent->agent->vehicle;
    if (controller) *controller = agent->agent->controller;
    if (name) *name = agent->agent->name.c_str();
    return 1;
}

FSIM_API void fsim_set_last_error(const char* message) { fsim::sdk::lastError() = message ? message : ""; }

} // extern "C"

fsim::World* fsim_world_object(fsim_world* world) {
    FSIM_NOT_AGENT(world, nullptr); // (the C++ World is the range's)
    if (!world) return nullptr;
    if (!world->object) world->object.reset(fsim::World::borrowed(world->world));
    return world->object.get();
}

extern "C" {

FSIM_API int fsim_world_step(fsim_world* world, uint32_t steps) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_world_step", [&] { world->world.step(steps); return FSIM_OK; });
}

FSIM_API double fsim_world_time(const fsim_world* world) {
    FSIM_AGENT_CALL(world, std::numeric_limits<double>::quiet_NaN());
    return world ? world->world.simTime() : 0.0;
}
FSIM_API double fsim_world_step_seconds(const fsim_world* world) {
    FSIM_NOT_AGENT(world, std::numeric_limits<double>::quiet_NaN());
    return world ? world->world.dt() * world->world.frameSkip() : 0.0;
}
FSIM_API uint64_t fsim_world_vehicle_steps(const fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world ? world->world.vehicleSteps() : 0;
}
FSIM_API int fsim_world_published(const fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world && world->world.published() ? 1 : 0;
}

FSIM_API int fsim_world_recording(const fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world && world->world.recording() ? 1 : 0;
}

FSIM_API int fsim_world_focus_viewer(fsim_world* world, uint32_t id) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_focus_viewer: bad arguments");
    if (!world->world.focusViewer(id)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_focus_viewer: no live vehicle with that id, or the world is not published");
    return FSIM_OK;
}

FSIM_API int fsim_world_create_vehicle(fsim_world* world, const fsim_vehicle_spec* spec, uint32_t* id) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    fsim_vehicle_spec whole;
    if (!world || !id || !wholeSpec(spec, whole)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_create_vehicle: bad arguments");
    spec = &whole;
    *id = 0;
    return guard("fsim_world_create_vehicle", [&] {
        fsim::session::VehicleSpec s;
        if (spec->name) s.name = spec->name;
        if (spec->type) s.type = spec->type;
        s.initial = initialOf(*spec);
        if (spec->model) s.model = spec->model;
        s.controlDivider = spec->control_divider;
        *id = world->world.createVehicle(s);
        return *id ? FSIM_OK : fail(FSIM_LOAD_FAILED, "fsim_world_create_vehicle: failed to create '" + s.name + "' (" + s.type + ")");
    });
}

FSIM_API int fsim_world_remove_vehicle(fsim_world* world, uint32_t id) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    return world->world.removeVehicle(id) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "no vehicle with id " + std::to_string(id));
}

FSIM_API int fsim_world_reset_vehicle(fsim_world* world, uint32_t id, const fsim_vehicle_spec* spec) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim_vehicle_spec whole;
    if (spec && !wholeSpec(spec, whole)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_reset_vehicle: bad arguments");
    return guard("fsim_world_reset_vehicle", [&] {
        bool ok;
        if (spec) {
            const auto ic = initialOf(whole);
            ok = world->world.resetVehicle(id, &ic);
        } else {
            ok = world->world.resetVehicle(id);
        }
        return ok ? FSIM_OK : fail(FSIM_ERROR, "fsim_world_reset_vehicle: reset failed for id " + std::to_string(id));
    });
}

FSIM_API uint32_t fsim_world_find_vehicle(const fsim_world* world, const char* name) {
    FSIM_NOT_AGENT(world, 0);
    return world && name ? world->world.find(name) : 0;
}

FSIM_API uint32_t fsim_world_vehicle_count(const fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world ? static_cast<uint32_t>(world->world.vehicleCount()) : 0;
}

FSIM_API uint32_t fsim_world_vehicle_ids(const fsim_world* world, uint32_t* ids, uint32_t capacity) {
    FSIM_NOT_AGENT(world, 0);
    if (!world) return 0;
    const auto all = world->world.vehicleIds();
    if (ids)
        for (uint32_t i = 0; i < capacity && i < all.size(); ++i) ids[i] = all[i];
    return static_cast<uint32_t>(all.size());
}

FSIM_API const char* fsim_vehicle_name(const fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, "");
    const auto* i = world ? world->world.info(id) : nullptr;
    return i ? i->name.c_str() : "";
}

FSIM_API const char* fsim_vehicle_type(const fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, "");
    const auto* i = world ? world->world.info(id) : nullptr;
    return i ? i->type.c_str() : "";
}

FSIM_API const fsim_vehicle_state* fsim_vehicle_state_ptr(const fsim_world* world, uint32_t id) {
    FSIM_NOT_AGENT(world, nullptr);
    return world ? asC(world->world.vehicleState(id)) : nullptr;
}

FSIM_API const fsim_vehicle_state* fsim_vehicle_sensed_ptr(const fsim_world* world, uint32_t id) {
    FSIM_NOT_AGENT(world, nullptr); // (ADR-31, L5: a whole state, truth-only fields among it - an agent reads its reports)
    const auto* s = world ? world->world.sensedState(id) : nullptr;
    return s ? asC(&s->state) : nullptr;
}

FSIM_API int fsim_vehicle_get_property(fsim_world* world, uint32_t id, const char* path, double* value) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    auto* m = world && path && value ? world->world.model(id) : nullptr;
    if (!m) return FSIM_INVALID_ARGUMENT;
    auto h = m->property(path);
    if (!h.valid()) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown property ") + path);
    *value = h.get();
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_set_property(fsim_world* world, uint32_t id, const char* path, double value) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    auto* m = world && path ? world->world.model(id) : nullptr;
    if (!m) return FSIM_INVALID_ARGUMENT;
    auto h = m->property(path);
    if (!h.valid()) return fail(FSIM_INVALID_ARGUMENT, std::string("unknown property ") + path);
    std::string error;
    if (!h.trySet(value, error)) return fail(FSIM_ERROR, std::string("set ") + path + ": " + error);
    return FSIM_OK;
}


FSIM_API int fsim_world_gather_states(const fsim_world* world, const uint32_t* ids, uint32_t count, int sensed, fsim_vehicle_state* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || (count && (!ids || !out))) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_gather_states: bad arguments");
    uint32_t unknown = 0;
    for (uint32_t i = 0; i < count; ++i) {
        const fsim::sim::VehicleState* s = nullptr;
        if (sensed) {
            const auto* z = world->world.sensedState(ids[i]);
            s = z ? &z->state : nullptr;
        } else {
            s = world->world.vehicleState(ids[i]);
        }
        if (s) {
            std::memcpy(&out[i], s, sizeof(fsim_vehicle_state));
        } else {
            std::memset(&out[i], 0, sizeof(fsim_vehicle_state));
            if (!unknown) unknown = ids[i] ? ids[i] : ~0u;
        }
    }
    if (unknown) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_gather_states: no vehicle with id " + std::to_string(unknown));
    fsim::sdk::lastError().clear();
    return FSIM_OK;
}



FSIM_API int fsim_vehicle_add_effect(fsim_world* world, uint32_t id, const char* effectId, const char* const* names, const double* values, uint32_t count) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !effectId) return FSIM_INVALID_ARGUMENT;
    const auto p = params(names, values, count);
    const std::string eid = effectId;
    if (!fsim::effects::createBuiltinEffect(eid, p)) return fail(FSIM_INVALID_ARGUMENT, "unknown effect " + eid);
    if (id == 0) {
        world->world.addEffectToAll([eid, p] { return fsim::effects::createBuiltinEffect(eid, p); });
        return FSIM_OK;
    }
    return world->world.addEffect(id, fsim::effects::createBuiltinEffect(eid, p)) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "no vehicle with id " + std::to_string(id));
}

FSIM_API int fsim_vehicle_clear_effects(fsim_world* world, uint32_t id) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    world->world.clearEffects(id);
    return FSIM_OK;
}

FSIM_API int fsim_comm_create_node(fsim_world* world, uint32_t address) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || address == 0 || address == fsim::comm::kBroadcast) return FSIM_INVALID_ARGUMENT;
    world->world.network().createNode(address, 0);
    return FSIM_OK;
}

FSIM_API int fsim_comm_send(fsim_world* world, uint32_t from, uint32_t to, uint32_t channel, uint32_t format, const void* bytes, size_t length) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED); // (the range's fabric: an agent's messages go through its DMS, AB-4)
    if (!world || !world->world.network().node(from)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::Message m;
    m.from = from;
    m.to = to;
    m.channel = channel;
    m.payload.format = format;
    if (bytes && length) m.payload.bytes.assign(static_cast<const uint8_t*>(bytes), static_cast<const uint8_t*>(bytes) + length);
    world->world.network().send(std::move(m));
    return FSIM_OK;
}

FSIM_API uint32_t fsim_comm_inbox_count(const fsim_world* world, uint32_t node) {
    FSIM_NOT_AGENT(world, 0);
    const auto* n = world ? world->world.network().node(node) : nullptr;
    return n ? static_cast<uint32_t>(n->inbox().size()) : 0;
}

FSIM_API int fsim_comm_inbox_get(const fsim_world* world, uint32_t node, uint32_t index, fsim_message* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    const auto* n = world && out ? world->world.network().node(node) : nullptr;
    if (!n || index >= n->inbox().size()) return FSIM_INVALID_ARGUMENT;
    const auto& m = n->inbox()[index];
    out->from = m.from; out->to = m.to; out->channel = m.channel; out->format = m.payload.format;
    out->time_sent = m.timeSent; out->time_delivered = m.timeDelivered;
    out->bytes = m.payload.bytes.data(); out->length = m.payload.bytes.size();
    return FSIM_OK;
}


FSIM_API int fsim_comm_set_medium(fsim_world* world, const char* mediumId, const char* const* names, const double* values, uint32_t count) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !mediumId) return FSIM_INVALID_ARGUMENT;
    const std::string id = mediumId;
    const auto p = params(names, values, count);
    if (id == "ideal") {
        world->world.network().setMedium(std::make_unique<fsim::comm::IdealMedium>());
        return FSIM_OK;
    }
    if (id == "link") {
        auto link = std::make_unique<fsim::comm::LinkModel>();
        link->rangeM = param(p, "range_m", link->rangeM);
        link->latencyS = param(p, "latency_s", link->latencyS);
        link->jitterS = param(p, "jitter_s", link->jitterS);
        link->lossProbability = param(p, "loss_probability", link->lossProbability);
        world->world.network().setMedium(std::move(link));
        return FSIM_OK;
    }
    return fail(FSIM_INVALID_ARGUMENT, "unknown medium " + id);
}

FSIM_API int fsim_comm_attach_protocol(fsim_world* world, uint32_t node, const char* protocolId, const char* const* names, const double* values, uint32_t count) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !protocolId || !world->world.network().node(node)) return FSIM_INVALID_ARGUMENT;
    const std::string id = protocolId;
    const auto p = params(names, values, count);
    if (id == "beacon") {
        world->world.network().attach(node, std::make_unique<fsim::comm::BeaconProtocol>(param(p, "period_s", 1.0), static_cast<uint32_t>(param(p, "channel", 1.0))));
        return FSIM_OK;
    }
    return fail(FSIM_INVALID_ARGUMENT, "unknown protocol " + id);
}

FSIM_API int fsim_comm_attach_udp_bridge(fsim_world* world, uint32_t node, uint16_t local_port, const char* remote_host, uint16_t remote_port) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !world->world.network().node(node)) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_comm_attach_udp_bridge", [&] {
        std::string error;
        auto transport = fsim::comm::createUdpTransport(local_port, remote_host ? remote_host : "", remote_port, &error);
        if (!transport) return fail(FSIM_ERROR, "fsim_comm_attach_udp_bridge: " + error);
        world->world.network().attach(node, std::make_unique<fsim::comm::BridgeProtocol>(std::move(transport)));
        return static_cast<int>(FSIM_OK);
    });
}

} // extern "C"
