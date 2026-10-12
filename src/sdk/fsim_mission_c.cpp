// C ABI for Mission Systems over session::World.

#include "fsim/fsim_c.h"

#include "sdk/CAbiSupport.h"
#include "sdk/Handles.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

// --- Mission Systems (ABI 1.49; ADR-30) --------------------------------------------

namespace {

const fsim::ms::Host* msHost(const fsim_world* world, uint32_t id) noexcept { return world ? world->world.missionSystems(id) : nullptr; }

/// Mission Systems options; from an agent's handle, the agent's source and controller (ADR-31).
fsim::ms::CommandOptions msOptions(const fsim_world* world, const fsim_ms_options* o) noexcept {
    fsim::ms::CommandOptions out;
    if (o && o->struct_size >= sizeof(uint32_t) * 2) out.source = static_cast<fsim::control::Source>(std::clamp(o->source, 0, 2));
    if (o && o->struct_size >= sizeof(uint32_t) * 3) out.controller = o->controller;
    if (o && o->struct_size >= sizeof(fsim_ms_options)) out.commandId = o->command_id;
    if (world && world->agent) out.source = fsim::control::Source::Policy, out.controller = world->agent->controller;
    return out;
}

int msResult(const fsim::ms::CommandResult& r, fsim_ms_result* out) {
    if (!out) return FSIM_OK;
    fsim_ms_result c;
    fsim_ms_result_init(&c);
    c.status = static_cast<int32_t>(r.status);
    c.reason = static_cast<int32_t>(r.reason);
    c.activity = r.activity;
    c.command_id = r.commandId;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

int msActivity(const fsim::ms::ActivityRecord& a, fsim_ms_activity_info* out) {
    fsim_ms_activity_info c;
    fsim_ms_activity_info_init(&c);
    c.id = a.id;
    c.capability = a.capability;
    c.payload = a.payload;
    c.source = static_cast<int32_t>(a.source);
    c.controller = a.controller;
    c.command_id = a.commandId;
    c.state = static_cast<int32_t>(a.state);
    c.reason = static_cast<int32_t>(a.reason);
    c.start_time = a.startTime, c.end_time = a.endTime, c.percent = a.percent;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

} // namespace

FSIM_API void fsim_payload_info_init(fsim_payload_info* info) {
    if (info) *info = fsim_payload_info{}, info->struct_size = sizeof *info;
}
FSIM_API void fsim_ms_capability_info_init(fsim_ms_capability_info* info) {
    if (info) *info = fsim_ms_capability_info{}, info->struct_size = sizeof *info;
}
FSIM_API void fsim_ms_options_init(fsim_ms_options* options) {
    if (options) *options = fsim_ms_options{}, options->struct_size = sizeof *options;
}
FSIM_API void fsim_ms_result_init(fsim_ms_result* result) {
    if (result) *result = fsim_ms_result{}, result->struct_size = sizeof *result;
}
FSIM_API void fsim_ms_activity_info_init(fsim_ms_activity_info* info) {
    if (info) *info = fsim_ms_activity_info{}, info->struct_size = sizeof *info, info->end_time = info->percent = std::nan("");
}

FSIM_API uint32_t fsim_vehicle_payload_count(const fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, 0);
    const auto* h = msHost(world, id);
    return h ? static_cast<uint32_t>(h->payloads().size()) : 0;
}

FSIM_API int fsim_vehicle_payload(const fsim_world* world, uint32_t id, uint32_t index, fsim_payload_info* out) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* h = msHost(world, id);
    if (!h || index >= h->payloads().size() || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_payload: bad arguments");
    const fsim::ms::PayloadInfo& p = h->payloads()[index];
    fsim_payload_info c;
    fsim_payload_info_init(&c);
    c.index = p.index, c.name = p.name, c.card = p.card, c.designation = p.designation, c.summary = p.summary;
    c.family = static_cast<int32_t>(p.family), c.state = static_cast<int32_t>(p.state), c.count = p.count, c.source = p.source;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API uint32_t fsim_vehicle_ms_capability_count(const fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, 0);
    const auto* h = msHost(world, id);
    return h ? static_cast<uint32_t>(h->capabilities().size()) : 0;
}

FSIM_API int fsim_vehicle_ms_capability(const fsim_world* world, uint32_t id, uint32_t index, fsim_ms_capability_info* out) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* h = msHost(world, id);
    if (!h || index >= h->capabilities().size() || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_ms_capability: bad arguments");
    const fsim::ms::CapabilityInfo& m = h->capabilities()[index];
    fsim_ms_capability_info c;
    fsim_ms_capability_info_init(&c);
    c.index = m.index, c.payload = m.payload, c.type = static_cast<int32_t>(m.type), c.id = m.id;
    c.support = static_cast<int32_t>(m.support), c.stage = m.stage;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_vehicle_ms_capability_status(const fsim_world* world, uint32_t id, const char* capability, int32_t* availability,
                                               int32_t* reason, double* next_available_s) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !capability) return FSIM_INVALID_ARGUMENT;
    const auto s = world->world.missionCapabilityStatus(id, capability);
    if (availability) *availability = static_cast<int32_t>(s.availability);
    if (reason) *reason = static_cast<int32_t>(s.reason);
    if (next_available_s) *next_available_s = s.nextAvailableS;
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_payload_state(fsim_world* world, uint32_t id, uint32_t payload, int32_t state, const fsim_ms_options* options,
                                        fsim_ms_result* result) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    FSIM_AGENT_AS(world, options, FSIM_NOT_ALLOWED);
    if (!world || state < 0 || state >= static_cast<int32_t>(fsim::ms::PayloadState::Count) || payload > 0xFFFFu)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_payload_state: bad arguments");
    const fsim::ms::StateCommand c{static_cast<std::uint16_t>(payload), static_cast<fsim::ms::PayloadState>(state)};
    return msResult(world->world.submit(id, c, msOptions(world, options)), result);
}

FSIM_API int fsim_vehicle_payload_bit(fsim_world* world, uint32_t id, uint32_t payload, const fsim_ms_options* options, fsim_ms_result* result) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    FSIM_AGENT_AS(world, options, FSIM_NOT_ALLOWED);
    if (!world || payload > 0xFFFFu) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_payload_bit: bad arguments");
    return msResult(world->world.submit(id, fsim::ms::BitCommand{static_cast<std::uint16_t>(payload)}, msOptions(world, options)), result);
}

FSIM_API int fsim_ms_activity_cancel(fsim_world* world, fsim_activity_id activity, const fsim_ms_options* options, fsim_ms_result* result) {
    FSIM_AGENT_ACTIVITY(world, activity, FSIM_NOT_ALLOWED);
    FSIM_AGENT_AS(world, options, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    const fsim::ms::CommandOptions o = msOptions(world, options);
    return msResult(world->world.cancelMission(static_cast<uint32_t>(activity >> 32), fsim::control::Caller{o.source, o.controller}, activity), result);
}

FSIM_API int fsim_ms_activity_get(const fsim_world* world, fsim_activity_id activity, fsim_ms_activity_info* out) {
    FSIM_AGENT_ACTIVITY(world, activity, FSIM_NOT_ALLOWED);
    const auto* h = msHost(world, static_cast<uint32_t>(activity >> 32));
    const fsim::ms::ActivityRecord* a = h ? h->activity(activity) : nullptr;
    if (!a || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_ms_activity_get: no such activity");
    return msActivity(*a, out);
}

FSIM_API uint32_t fsim_vehicle_ms_activity_count(const fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, 0);
    const auto* h = msHost(world, id);
    return h ? static_cast<uint32_t>(h->activities().size()) : 0;
}

FSIM_API int fsim_vehicle_ms_activity(const fsim_world* world, uint32_t id, uint32_t index, fsim_ms_activity_info* out) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* h = msHost(world, id);
    if (!h || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_ms_activity: bad arguments");
    const auto all = h->activities();
    if (index >= all.size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_ms_activity: no activity at that index");
    return msActivity(all[index], out);
}

FSIM_API int fsim_vehicle_payload_fail(fsim_world* world, uint32_t id, uint32_t payload, int failed) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || payload > 0xFFFFu) return FSIM_INVALID_ARGUMENT;
    const auto r = world->world.failPayload(id, static_cast<std::uint16_t>(payload), failed != 0);
    return r == fsim::ms::Reason::None ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, std::string("fsim_vehicle_payload_fail: ") + fsim::ms::reasonName(r));
}

FSIM_API const char* fsim_payload_family_name(int family) {
    return family >= 0 && family < static_cast<int>(fsim::ms::Family::Count) ? fsim::ms::familyName(static_cast<fsim::ms::Family>(family)) : "";
}
FSIM_API const char* fsim_payload_state_name(int state) {
    return state >= 0 && state < static_cast<int>(fsim::ms::PayloadState::Count) ? fsim::ms::payloadStateName(static_cast<fsim::ms::PayloadState>(state)) : "";
}
FSIM_API const char* fsim_ms_capability_type_id(int type) {
    return type >= 0 && type < static_cast<int>(fsim::ms::CapabilityType::Count) ? fsim::ms::capabilityTypeId(static_cast<fsim::ms::CapabilityType>(type)) : "";
}
FSIM_API const char* fsim_ms_reason_name(int reason) {
    return reason >= 0 && reason < static_cast<int>(fsim::ms::Reason::Count) ? fsim::ms::reasonName(static_cast<fsim::ms::Reason>(reason)) : "";
}
FSIM_API const char* fsim_ms_reason_description(int reason) {
    return reason >= 0 && reason < static_cast<int>(fsim::ms::Reason::Count) ? fsim::ms::reasonDescription(static_cast<fsim::ms::Reason>(reason)) : "";
}

// --- Weapons in flight and the radio path (ABI 1.50; ADR-30 MS-2) ----------------------

FSIM_API void fsim_weapon_launch_init(fsim_weapon_launch* launch) {
    if (launch) *launch = fsim_weapon_launch{}, launch->struct_size = sizeof *launch, launch->type = "jsbsim:mk82";
}
FSIM_API void fsim_weapon_info_init(fsim_weapon_info* info) {
    if (!info) return;
    *info = fsim_weapon_info{}, info->struct_size = sizeof *info;
    info->end_time_s = info->end_latitude_rad = info->end_longitude_rad = info->end_height_m = info->end_speed_ms = std::nan("");
    info->miss_m = std::nan("");
    info->time_to_go_s = info->time_to_active_s = std::nan("");
    info->seeker = "";
}
FSIM_API void fsim_radio_path_options_init(fsim_radio_path_options* options) {
    if (!options) return;
    const fsim::ms::RadioPathOptions d;
    *options = fsim_radio_path_options{}, options->struct_size = sizeof *options;
    options->frequency_hz = d.frequencyHz, options->polarization = d.polarization, options->time_percent = d.timePercent;
    options->rain_rate_mmh = d.rainRateMmH, options->rain_height_m = d.rainHeightM, options->itm = d.itm ? 1 : 0;
}
FSIM_API void fsim_radio_path_init(fsim_radio_path* path) {
    if (path) *path = fsim_radio_path{}, path->struct_size = sizeof *path;
}

FSIM_API int fsim_world_launch_weapon(fsim_world* world, uint32_t launcher, const fsim_weapon_launch* launch, uint32_t* id) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (id) *id = 0;
    if (!world || !launch || launch->struct_size < sizeof(fsim_weapon_launch)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_launch_weapon: bad arguments");
    fsim::ms::WeaponLaunch l;
    if (launch->type && *launch->type) l.type = launch->type;
    if (launch->name) l.name = launch->name;
    for (int i = 0; i < 3; ++i) l.offsetBodyM[i] = launch->offset_body_m[i], l.ejectionBodyMs[i] = launch->ejection_body_ms[i];
    const uint32_t made = world->world.launchWeapon(launcher, l);
    if (!made) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_launch_weapon: no weapon released (an unknown launcher, or its model did not load: see the log)");
    if (id) *id = made;
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_weapon_count(fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    if (!world) return 0;
    world->weapons = world->world.weapons();
    return static_cast<uint32_t>(world->weapons.size());
}

FSIM_API int fsim_world_weapon(const fsim_world* world, uint32_t index, fsim_weapon_info* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out || index >= world->weapons.size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_weapon: bad arguments");
    const fsim::ms::WeaponRecord& w = world->weapons[index];
    fsim_weapon_info c;
    fsim_weapon_info_init(&c);
    c.id = w.id, c.launcher = w.launcher;
    auto& strings = const_cast<fsim_world*>(world)->strings;
    c.name = strings.insert(w.name).first->c_str();
    c.type = strings.insert(w.type).first->c_str();
    c.launch_time_s = w.launchTimeS, c.end_time_s = w.endTimeS, c.end = static_cast<int32_t>(w.end);
    c.end_latitude_rad = w.endLatitudeRad, c.end_longitude_rad = w.endLongitudeRad, c.end_height_m = w.endHeightM, c.end_speed_ms = w.endSpeedMs;
    c.store = strings.insert(w.store).first->c_str(); // (ABI 1.56: copied where the caller's struct_size reaches)
    c.station = w.station, c.jettisoned = w.jettisoned ? 1 : 0, c.activity = w.activity, c.target = w.target;
    c.phase = static_cast<int32_t>(w.phase);
    c.seeker = strings.insert(w.seeker).first->c_str();
    c.holding = w.holding, c.miss_m = w.missM;
    c.burning = w.burning, c.tracking = w.tracking, c.supported = w.supported;
    c.time_to_go_s=w.timeToGoS, c.time_to_active_s=w.timeToActiveS;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_vehicle_role(const fsim_world* world, uint32_t id, int32_t* role, uint32_t* launcher) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* i = world ? world->world.info(id) : nullptr;
    if (!i) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_role: unknown vehicle");
    if (role) *role = static_cast<int32_t>(i->role);
    if (launcher) *launcher = i->launcher;
    return FSIM_OK;
}

FSIM_API int fsim_world_radio_path(const fsim_world* world, uint32_t from, uint32_t to, const fsim_radio_path_options* options,
                                   fsim_radio_path* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radio_path: bad arguments");
    fsim::ms::RadioPathOptions o;
    if (options) {
        if (options->struct_size < sizeof(fsim_radio_path_options)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radio_path: options too small");
        o.frequencyHz = options->frequency_hz, o.polarization = options->polarization, o.timePercent = options->time_percent;
        o.rainRateMmH = options->rain_rate_mmh, o.rainHeightM = options->rain_height_m, o.itm = options->itm != 0;
    }
    if (!(o.frequencyHz > 0.0)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radio_path: frequency_hz must be above 0");
    const auto p = world->world.radioPath(from, to, o);
    if (!p) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radio_path: unknown vehicle");
    fsim_radio_path c;
    fsim_radio_path_init(&c);
    c.code = p->code, c.via_itm = p->viaItm ? 1 : 0, c.mode = p->mode, c.terrain_masked = p->terrainMasked ? 1 : 0;
    c.ground_range_m = p->groundRangeM, c.slant_range_m = p->slantRangeM;
    for (int i = 0; i < 2; ++i) c.elevation_rad[i] = p->elevationRad[i], c.azimuth_rad[i] = p->azimuthRad[i];
    c.clearance_m = p->clearanceM;
    c.free_space_db = p->freeSpaceDb, c.basic_db = p->basicDb, c.gas_db = p->gasDb, c.terrain_db = p->terrainDb, c.rain_db = p->rainDb;
    c.total_db = p->totalDb;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API const char* fsim_role_name(int role) { return fsim::ms::roleName(static_cast<fsim::ms::Role>(role == 1 ? 1 : 0)); }
FSIM_API const char* fsim_weapon_end_name(int end) {
    return end >= 0 && end <= static_cast<int>(fsim::ms::WeaponEnd::Expired) ? fsim::ms::weaponEndName(static_cast<fsim::ms::WeaponEnd>(end)) : "";
}
FSIM_API const char* fsim_weapon_phase_name(int phase) {
    return fsim::ms::weaponPhaseName(static_cast<fsim::ms::WeaponPhase>(phase));
}

FSIM_API void fsim_engagement_event_init(fsim_engagement_event* out) {
    if (!out) return;
    std::memset(out,0,sizeof *out); out->struct_size=sizeof *out; out->kind=out->reason="";
}
FSIM_API void fsim_decoy_state_init(fsim_decoy_state* out) {
    if (!out) return;
    std::memset(out,0,sizeof *out); out->struct_size=sizeof *out;
}
FSIM_API void fsim_launch_zone_init(fsim_launch_zone* out) {
    if (!out) return;
    std::memset(out,0,sizeof *out); out->struct_size=sizeof *out;
}
FSIM_API uint32_t fsim_world_engagement_count(fsim_world* world,uint64_t since) {
    FSIM_NOT_AGENT(world,0);
    if (!world) return 0;
    world->combatEvents=world->world.engagementEvents(since);
    return static_cast<uint32_t>(world->combatEvents.size());
}
FSIM_API int fsim_world_engagement(const fsim_world* world,uint32_t index,fsim_engagement_event* out) {
    FSIM_NOT_AGENT(world,FSIM_NOT_ALLOWED);
    if (!world || !out || index>=world->combatEvents.size()) return fail(FSIM_INVALID_ARGUMENT,"engagement: bad index");
    const auto& event=world->combatEvents[index];
    fsim_engagement_event c; fsim_engagement_event_init(&c);
    c.serial=event.serial; c.time_s=event.timeS;
    c.kind=const_cast<fsim_world*>(world)->intern(event.kind); c.reason=const_cast<fsim_world*>(world)->intern(event.reason);
    c.entity=event.entity; c.launcher=event.launcher; c.target=event.target; c.destroyed=event.destroyed;
    std::copy_n(event.positionEcef,3,c.position_ecef); std::copy_n(event.velocityEcef,3,c.velocity_ecef);
    return copyOut(c,out)?FSIM_OK:FSIM_INVALID_ARGUMENT;
}
FSIM_API uint32_t fsim_world_decoy_count(fsim_world* world) {
    FSIM_NOT_AGENT(world,0);
    if (!world) return 0;
    world->decoys=world->world.decoys(); return static_cast<uint32_t>(world->decoys.size());
}
FSIM_API int fsim_world_decoy(const fsim_world* world,uint32_t index,fsim_decoy_state* out) {
    FSIM_NOT_AGENT(world,FSIM_NOT_ALLOWED);
    if (!world || !out || index>=world->decoys.size()) return fail(FSIM_INVALID_ARGUMENT,"decoy: bad index");
    const auto& decoy=world->decoys[index]; fsim_decoy_state c; fsim_decoy_state_init(&c);
    c.id=decoy.id; c.owner=decoy.owner; c.flare=decoy.flare; c.age_s=decoy.ageS;
    std::copy_n(decoy.positionEcef,3,c.position_ecef);
    return copyOut(c,out)?FSIM_OK:FSIM_INVALID_ARGUMENT;
}
FSIM_API int fsim_world_dispense(fsim_world* world,uint32_t vehicle,int32_t flare) {
    FSIM_AGENT_VEHICLE(world,vehicle,FSIM_NOT_ALLOWED);
    if (!world || (flare!=0 && flare!=1)) return fail(FSIM_INVALID_ARGUMENT,"dispense: invalid kind");
    const auto reason=world->world.dispense(vehicle,flare!=0);
    return reason==fsim::ms::Reason::None?FSIM_OK:fail(FSIM_INVALID_ARGUMENT,fsim::ms::reasonName(reason));
}
FSIM_API int fsim_world_countermeasures(const fsim_world* world,uint32_t vehicle,uint32_t* chaff,uint32_t* flare) {
    FSIM_AGENT_VEHICLE(world,vehicle,FSIM_NOT_ALLOWED);
    if (!world || !chaff || !flare || !world->world.alive(vehicle)) return fail(FSIM_INVALID_ARGUMENT,"countermeasures: unknown vehicle");
    const auto inventory=world->world.countermeasures(vehicle); *chaff=inventory.chaff; *flare=inventory.flare; return FSIM_OK;
}
FSIM_API int fsim_world_seeker_target(const fsim_world* world,uint32_t vehicle,const char* store,uint32_t* target) {
    FSIM_NOT_AGENT(world,FSIM_NOT_ALLOWED);
    if (!world || !store || !target || !world->world.alive(vehicle)) return fail(FSIM_INVALID_ARGUMENT,"seeker: unknown vehicle");
    *target=world->world.seekerTarget(vehicle,store); return FSIM_OK;
}
FSIM_API int fsim_world_launch_zone(const fsim_world* world,uint32_t vehicle,const char* store,uint32_t target,fsim_launch_zone* out) {
    FSIM_AGENT_VEHICLE(world,vehicle,FSIM_NOT_ALLOWED);
    if (!world || !store || !out || !world->world.alive(vehicle)) return fail(FSIM_INVALID_ARGUMENT,"launch zone: invalid launcher");
    // An agent may use only its own designation, never another aircraft's true state.
    if (world->agent) {
        const auto* designation=world->world.designation(target);
        if (!designation || designation->owner!=vehicle) return fail(FSIM_NOT_ALLOWED,"launch zone: own designation required");
    }
    if (!world->world.vehicleState(target)) return fail(FSIM_INVALID_ARGUMENT,"launch zone: unknown target");
    const auto zone=world->world.launchZone(vehicle,store,target); fsim_launch_zone c; fsim_launch_zone_init(&c);
    c.maximum_m=zone.maximumM; c.no_escape_m=zone.noEscapeM; c.minimum_m=zone.minimumM;
    return copyOut(c,out)?FSIM_OK:FSIM_INVALID_ARGUMENT;
}

// --- AMTI (ABI 1.52; ADR-30 MS-4) -------------------------------------------------------

FSIM_API void fsim_amti_command_init(fsim_amti_command* command) {
    if (!command) return;
    const fsim::ms::AmtiCommand d;
    *command = fsim_amti_command{}, command->struct_size = sizeof *command;
    command->mode = static_cast<int32_t>(d.mode), command->bars = d.bars;
    command->azimuth_centre_rad = d.azimuthCentreRad, command->azimuth_half_width_rad = d.azimuthHalfWidthRad;
    command->elevation_centre_rad = d.elevationCentreRad, command->range_scale_m = d.rangeScaleM;
    command->task = static_cast<int32_t>(d.task), command->elevation_half_width_rad = d.elevationHalfWidthRad;
}

FSIM_API int fsim_amti_command_acm(fsim_amti_command* command, int pattern) {
    if (!command || command->struct_size < sizeof(fsim_amti_command) || pattern < 0 ||
        pattern > static_cast<int>(fsim::ms::AcmPattern::VerticalScan))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_amti_command_acm: bad arguments");
    const fsim::ms::AmtiCommand d = fsim::ms::acmCommand(0, static_cast<fsim::ms::AcmPattern>(pattern));
    command->mode = static_cast<int32_t>(d.mode), command->bars = d.bars, command->task = static_cast<int32_t>(d.task);
    command->azimuth_centre_rad = d.azimuthCentreRad, command->azimuth_half_width_rad = d.azimuthHalfWidthRad;
    command->elevation_centre_rad = d.elevationCentreRad, command->elevation_half_width_rad = d.elevationHalfWidthRad;
    command->range_scale_m = d.rangeScaleM, command->signal_level = 0, command->track = 0;
    return FSIM_OK;
}
FSIM_API void fsim_amti_detection_init(fsim_amti_detection* detection) {
    if (detection) *detection = fsim_amti_detection{}, detection->struct_size = sizeof *detection;
}
FSIM_API void fsim_radar_status_init(fsim_radar_status* status) {
    if (status) *status = fsim_radar_status{}, status->struct_size = sizeof *status;
}

FSIM_API int fsim_vehicle_amti_search(fsim_world* world, uint32_t id, const fsim_amti_command* command, const fsim_ms_options* options,
                                      fsim_ms_result* result) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    FSIM_AGENT_AS(world, options, FSIM_NOT_ALLOWED);
    // (a caller from before ABI 1.55 has no signal_level: its struct ends before it)
    if (!world || !command || command->struct_size < offsetof(fsim_amti_command, signal_level) || command->payload > 0xFFFFu)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_amti_search: bad arguments");
    fsim::ms::AmtiCommand c;
    c.payload = static_cast<std::uint16_t>(command->payload);
    if (command->mode < 0 || command->mode > static_cast<int32_t>(fsim::ms::AmtiMode::SingleTargetTrack))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_amti_search: unknown mode");
    c.mode = static_cast<fsim::ms::AmtiMode>(command->mode);
    c.bars = command->bars;
    c.azimuthCentreRad = command->azimuth_centre_rad, c.azimuthHalfWidthRad = command->azimuth_half_width_rad;
    c.elevationCentreRad = command->elevation_centre_rad, c.rangeScaleM = command->range_scale_m;
    if (command->struct_size >= offsetof(fsim_amti_command, signal_level) + sizeof command->signal_level) c.signalLevel = command->signal_level != 0;
    // (a caller from before ABI 1.70 has none of MS-4b's: a volume's search)
    if (command->struct_size >= offsetof(fsim_amti_command, elevation_half_width_rad) + sizeof command->elevation_half_width_rad) {
        if (command->task < 0 || command->task > static_cast<int32_t>(fsim::ms::AmtiTask::Track))
            return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_amti_search: unknown task");
        c.task = static_cast<fsim::ms::AmtiTask>(command->task);
        c.track = command->track, c.illuminate = command->illuminate != 0, c.elevationHalfWidthRad = command->elevation_half_width_rad;
    }
    return msResult(world->world.submit(id, c, msOptions(world, options)), result);
}

FSIM_API int fsim_vehicle_radar_status(const fsim_world* world, uint32_t id, uint32_t payload, fsim_radar_status* out) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* h = msHost(world, id);
    if (!h || !out || payload > 0xFFFFu || !h->radar(static_cast<std::uint16_t>(payload)))
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_radar_status: no such radar");
    const fsim::ms::RadarStatus s = h->radarStatus(static_cast<std::uint16_t>(payload));
    fsim_radar_status c;
    fsim_radar_status_init(&c);
    c.searching = s.searching ? 1 : 0, c.activity = s.activity, c.mode = static_cast<int32_t>(s.mode), c.bars = s.bars;
    c.rotodome = s.rotodome ? 1 : 0, c.electronic = s.electronic ? 1 : 0;
    c.beam_azimuth_rad = s.beamAzimuthRad, c.beam_elevation_rad = s.beamElevationRad, c.frame_period_s = s.framePeriodS;
    c.azimuth_centre_rad = s.azimuthCentreRad, c.azimuth_half_width_rad = s.azimuthHalfWidthRad;
    c.elevation_centre_rad = s.elevationCentreRad, c.range_scale_m = s.rangeScaleM;
    c.azimuth_limit_rad = s.azimuthLimitRad, c.elevation_limit_rad = s.elevationLimitRad;
    c.instrumented_range_m = s.instrumentedRangeM, c.frequency_hz = s.frequencyHz;
    c.beamwidth_azimuth_rad = s.beamwidthAzimuthRad, c.beamwidth_elevation_rad = s.beamwidthElevationRad;
    c.dwells = s.dwells, c.detections = s.detections;
    c.false_alarms = s.falseAlarms, c.tracks = s.tracks, c.tentative_tracks = s.tentativeTracks;
    c.signal_level = s.signalLevel ? 1 : 0;
    c.task = static_cast<int32_t>(s.task), c.designated_track = s.designatedTrack;
    c.acquiring = s.acquiring ? 1 : 0, c.illuminating = s.illuminating ? 1 : 0, c.track_memory_s = s.trackMemoryS;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API void fsim_amti_track_init(fsim_amti_track* track) {
    if (track) *track = fsim_amti_track{}, track->struct_size = sizeof *track;
}

FSIM_API int fsim_vehicle_amti_tracks(const fsim_world* world, uint32_t id, uint32_t payload, fsim_amti_track* out, uint32_t max,
                                      uint32_t* count) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !count || payload > 0xFFFFu || (max && (!out || out[0].struct_size < sizeof(uint32_t)))) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_amti_tracks", [&]() -> int {
        const auto list = world->world.amtiTracks(id, static_cast<std::uint16_t>(payload));
        const uint32_t stride = max ? out[0].struct_size : 0;
        for (std::size_t i = 0; i < list.size() && i < max; ++i) {
            const fsim::ms::AmtiTrack& t = list[i];
            fsim_amti_track c;
            fsim_amti_track_init(&c);
            c.payload = t.payload, c.id = t.id, c.time_s = t.timeS, c.last_update_s = t.lastUpdateS, c.updates = t.updates;
            c.truth = world->agent ? 0 : t.truth; // (ADR-31: the referee's)
            c.latitude_rad = t.latitudeRad, c.longitude_rad = t.longitudeRad, c.height_m = t.heightM;
            for (int k = 0; k < 3; ++k) c.velocity_ned_ms[k] = t.velocityNedMs[k];
            c.position_sigma_m = t.positionSigmaM, c.velocity_sigma_ms = t.velocitySigmaMs;
            c.range_m = t.rangeM, c.azimuth_rad = t.azimuthRad, c.elevation_rad = t.elevationRad, c.range_rate_ms = t.rangeRateMs;
            for (int k = 0; k < 6; ++k) c.state_ecef[k] = t.stateEcef[k];
            for (int k = 0; k < 36; ++k) c.covariance_ecef[k] = t.covarianceEcef[k];
            auto* slot = reinterpret_cast<fsim_amti_track*>(reinterpret_cast<char*>(out) + i * stride);
            slot->struct_size = stride;
            copyOut(c, slot);
        }
        *count = static_cast<uint32_t>(list.size());
        return FSIM_OK;
    });
}

// --- Contacts (ABI 1.71; ADR-30 MS-9a) --------------------------------------------------

FSIM_API void fsim_contact_init(fsim_contact* contact) {
    if (contact) *contact = fsim_contact{}, contact->struct_size = sizeof *contact;
}

FSIM_API int fsim_vehicle_contacts(const fsim_world* world, uint32_t id, uint32_t payload, fsim_contact* out, uint32_t max, uint32_t* count) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !count || payload > 0xFFFFu || (max && (!out || out[0].struct_size < sizeof(uint32_t)))) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_contacts", [&]() -> int {
        const auto list = world->world.contacts(id, static_cast<std::uint16_t>(payload));
        const uint32_t stride = max ? out[0].struct_size : 0;
        for (std::size_t i = 0; i < list.size() && i < max; ++i) {
            const fsim::ms::Contact& t = list[i];
            fsim_contact c;
            fsim_contact_init(&c);
            c.payload = t.payload, c.id = t.id, c.kind = static_cast<int32_t>(t.kind), c.mode = static_cast<int32_t>(t.mode);
            c.warning = t.warning ? 1 : 0;
            c.truth = world->agent ? 0 : t.truth; // (ADR-31: the referee's)
            c.first_time_s = t.firstTimeS, c.time_s = t.timeS;
            c.azimuth_rad = t.azimuthRad, c.elevation_rad = t.elevationRad, c.bearing_sigma_rad = t.bearingSigmaRad;
            c.range_m = t.rangeM, c.range_rate_ms = t.rangeRateMs;
            c.latitude_rad = t.latitudeRad, c.longitude_rad = t.longitudeRad, c.height_m = t.heightM;
            c.strength_db = t.strengthDb, c.frequency_hz = t.frequencyHz, c.size_m = t.sizeM;
            auto* slot = reinterpret_cast<fsim_contact*>(reinterpret_cast<char*>(out) + i * stride);
            slot->struct_size = stride;
            copyOut(c, slot);
        }
        *count = static_cast<uint32_t>(list.size());
        return FSIM_OK;
    });
}

FSIM_API const char* fsim_contact_kind_name(int kind) {
    return kind >= 0 && kind < static_cast<int>(fsim::ms::ContactKind::Count) ? fsim::ms::contactKindName(static_cast<fsim::ms::ContactKind>(kind)) : "";
}
FSIM_API const char* fsim_emitter_mode_name(int mode) {
    return mode >= 0 && mode < static_cast<int>(fsim::ms::EmitterMode::Count) ? fsim::ms::emitterModeName(static_cast<fsim::ms::EmitterMode>(mode)) : "";
}

FSIM_API int fsim_vehicle_amti_detections(const fsim_world* world, uint32_t id, uint32_t payload, uint64_t since, fsim_amti_detection* out,
                                          uint32_t max, uint32_t* count) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !count || payload > 0xFFFFu || (max && (!out || out[0].struct_size < sizeof(uint32_t)))) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vehicle_amti_detections", [&]() -> int {
        const auto list = world->world.amtiDetections(id, static_cast<std::uint16_t>(payload), since);
        const uint32_t stride = max ? out[0].struct_size : 0;
        for (std::size_t i = 0; i < list.size() && i < max; ++i) {
            const fsim::ms::AmtiDetection& d = list[i];
            fsim_amti_detection c;
            fsim_amti_detection_init(&c);
            c.payload = d.payload, c.serial = d.serial, c.time_s = d.timeS, c.truth = world->agent ? 0 : d.truth; // (ADR-31: the referee's)
            c.range_m = d.rangeM, c.azimuth_rad = d.azimuthRad, c.elevation_rad = d.elevationRad, c.range_rate_ms = d.rangeRateMs;
            c.snr_db = d.snrDb, c.latitude_rad = d.latitudeRad, c.longitude_rad = d.longitudeRad, c.height_m = d.heightM;
            auto* slot = reinterpret_cast<fsim_amti_detection*>(reinterpret_cast<char*>(out) + i * stride);
            slot->struct_size = stride; // (the caller's header's size: only the first is read)
            copyOut(c, slot);
        }
        *count = static_cast<uint32_t>(list.size());
        return FSIM_OK;
    });
}

FSIM_API const char* fsim_amti_mode_name(int mode) {
    if (mode < 0 || mode > static_cast<int>(fsim::ms::AmtiMode::SingleTargetTrack)) return "";
    return fsim::ms::amtiModeName(static_cast<fsim::ms::AmtiMode>(mode));
}

FSIM_API const char* fsim_amti_task_name(int task) {
    if (task < 0 || task > static_cast<int>(fsim::ms::AmtiTask::Track)) return "";
    return fsim::ms::amtiTaskName(static_cast<fsim::ms::AmtiTask>(task));
}

// --- Radar cross section (ABI 1.51; ADR-30 MS-3) ---------------------------------------

FSIM_API void fsim_rcs_query_init(fsim_rcs_query* query) {
    if (!query) return;
    *query = fsim_rcs_query{}, query->struct_size = sizeof *query;
    query->frequency_hz = 9.5e9, query->transmit_polarization_rad = 0.0, query->receive_polarization_rad = std::nan("");
}
FSIM_API void fsim_radar_cross_section_init(fsim_radar_cross_section* out) {
    if (!out) return;
    *out = fsim_radar_cross_section{}, out->struct_size = sizeof *out, out->sigma_m2 = out->fluctuation = std::nan("");
}

FSIM_API int fsim_world_radar_cross_section(const fsim_world* world, uint32_t observer, uint32_t target, const fsim_rcs_query* query,
                                            fsim_radar_cross_section* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radar_cross_section: bad arguments");
    fsim::ms::RcsQuery q;
    if (query) {
        if (query->struct_size < sizeof(fsim_rcs_query)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radar_cross_section: query too small");
        q.frequencyHz = query->frequency_hz, q.transmitPolarizationRad = query->transmit_polarization_rad, q.receivePolarizationRad = query->receive_polarization_rad;
    }
    if (!(q.frequencyHz > 0.0)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radar_cross_section: frequency_hz must be above 0");
    const auto r = world->world.radarCrossSection(observer, target, q);
    if (!r) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_radar_cross_section: unknown vehicle (or the two in one place)");
    fsim_radar_cross_section c;
    fsim_radar_cross_section_init(&c);
    c.modelled = r->modelled ? 1 : 0, c.sigma_m2 = r->sigmaM2, c.fluctuation = r->fluctuation;
    c.aspect_azimuth_rad = r->aspectAzimuthRad, c.aspect_elevation_rad = r->aspectElevationRad;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

// --- Stores and weapon employment (ABI 1.56; ADR-30 MS-7a) ------------------------

namespace {

std::vector<std::uint16_t> storeList(const uint32_t* stores, uint32_t count, bool& ok) {
    std::vector<std::uint16_t> out;
    ok = true;
    for (uint32_t i = 0; stores && i < count; ++i) {
        if (stores[i] > 0xFFFFu) ok = false;
        out.push_back(static_cast<std::uint16_t>(stores[i]));
    }
    return out;
}

} // namespace

FSIM_API void fsim_store_info_init(fsim_store_info* info) {
    if (info) *info = fsim_store_info{}, info->struct_size = sizeof *info;
}
FSIM_API void fsim_strike_command_init(fsim_strike_command* command) {
    if (!command) return;
    const fsim::ms::StrikeCommand d;
    *command = fsim_strike_command{}, command->struct_size = sizeof *command;
    command->quantity = d.quantity, command->ripple_s = d.rippleS;
}
FSIM_API void fsim_jettison_command_init(fsim_jettison_command* command) {
    if (command) *command = fsim_jettison_command{}, command->struct_size = sizeof *command;
}
FSIM_API void fsim_strike_status_init(fsim_strike_status* status) {
    if (status) *status = fsim_strike_status{}, status->struct_size = sizeof *status;
}

FSIM_API uint32_t fsim_vehicle_store_count(fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, 0);
    const auto* h = msHost(world, id);
    return h ? static_cast<uint32_t>(h->stores().size()) : 0u;
}

FSIM_API int fsim_vehicle_store(const fsim_world* world, uint32_t id, uint32_t index, fsim_store_info* out) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    const auto* h = msHost(world, id);
    if (!h || !out || index >= h->stores().size()) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_store: no such store");
    const fsim::ms::StoreInfo& s = h->stores()[index];
    fsim_store_info c;
    fsim_store_info_init(&c);
    c.index = s.index, c.payload = s.payload, c.station = s.station;
    c.station_name = s.stationName, c.store = s.store, c.mnemonic = s.mnemonic, c.store_class = s.storeClass, c.carriage = s.carriage;
    c.powerable = s.powerable ? 1 : 0, c.jettisonable = s.jettisonable ? 1 : 0, c.flyable = s.flyable ? 1 : 0;
    c.status = static_cast<int32_t>(s.status), c.mass_kg = s.massKg, c.weapon = s.weapon;
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_vehicle_load_store(fsim_world* world, uint32_t id, uint32_t payload, int32_t station, const char* store, int32_t count,
                                     int32_t* reason) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || payload > 0xFFFFu) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_load_store: bad arguments");
    const auto r = world->world.loadStore(id, static_cast<std::uint16_t>(payload), station, store ? store : "", count);
    if (reason) *reason = static_cast<int32_t>(r);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_load_loadout(fsim_world* world, uint32_t id, const char* loadout, int32_t* reason) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !loadout) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_load_loadout: bad arguments");
    const auto r = world->world.loadLoadout(id, loadout);
    if (reason) *reason = static_cast<int32_t>(r);
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_strike(fsim_world* world, uint32_t id, const fsim_strike_command* command, const fsim_ms_options* options,
                                 fsim_ms_result* result) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    FSIM_AGENT_AS(world, options, FSIM_NOT_ALLOWED);
    if (!world || !command || command->struct_size < sizeof(fsim_strike_command) || command->payload > 0xFFFFu)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_strike: bad arguments");
    fsim::ms::StrikeCommand c;
    c.payload = static_cast<std::uint16_t>(command->payload);
    c.store = command->store ? command->store : "";
    c.quantity = command->quantity;
    bool ok = true;
    c.stores = storeList(command->stores, command->store_count, ok);
    if (!ok) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_strike: a store index out of range");
    c.target = command->target;
    c.consent = command->consent != 0;
    c.rippleS = command->ripple_s;
    return msResult(world->world.submit(id, c, msOptions(world, options)), result);
}

FSIM_API int fsim_vehicle_jettison(fsim_world* world, uint32_t id, const fsim_jettison_command* command, const fsim_ms_options* options,
                                   fsim_ms_result* result) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    FSIM_AGENT_AS(world, options, FSIM_NOT_ALLOWED);
    if (!world || !command || command->struct_size < sizeof(fsim_jettison_command) || command->payload > 0xFFFFu)
        return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_jettison: bad arguments");
    fsim::ms::JettisonCommand c;
    c.payload = static_cast<std::uint16_t>(command->payload);
    bool ok = true;
    c.stores = storeList(command->stores, command->store_count, ok);
    if (!ok) return fail(FSIM_INVALID_ARGUMENT, "fsim_vehicle_jettison: a store index out of range");
    c.emergency = command->emergency != 0;
    return msResult(world->world.submit(id, c, msOptions(world, options)), result);
}

FSIM_API int fsim_strike_consent(fsim_world* world, uint64_t activity, int32_t granted, int32_t* reason) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return fail(FSIM_INVALID_ARGUMENT, "fsim_strike_consent: bad arguments");
    const auto r = world->world.strikeConsent(activity, granted != 0);
    if (reason) *reason = static_cast<int32_t>(r);
    return FSIM_OK;
}

FSIM_API int fsim_strike_status_get(const fsim_world* world, uint64_t activity, fsim_strike_status* out, uint32_t* stores, uint32_t max_stores,
                                    uint32_t* weapons, uint32_t max_weapons) {
    FSIM_AGENT_ACTIVITY(world, activity, FSIM_NOT_ALLOWED);
    const auto s = world ? world->world.strikeStatus(activity) : std::nullopt;
    if (!s || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_strike_status_get: not a strike or jettison");
    fsim_strike_status c;
    fsim_strike_status_init(&c);
    c.phase = static_cast<int32_t>(s->phase), c.jettison = s->jettison ? 1 : 0, c.target = s->target;
    c.store_count = static_cast<uint32_t>(s->stores.size()), c.weapon_count = static_cast<uint32_t>(s->weapons.size());
    for (uint32_t i = 0; stores && i < max_stores && i < s->stores.size(); ++i) stores[i] = s->stores[i];
    for (uint32_t i = 0; weapons && i < max_weapons && i < s->weapons.size(); ++i) weapons[i] = s->weapons[i];
    return copyOut(c, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API double fsim_vehicle_mass_kg(const fsim_world* world, uint32_t id) {
    FSIM_AGENT_VEHICLE(world, id, std::numeric_limits<double>::quiet_NaN());
    return world ? world->world.massKg(id) : std::nan("");
}

FSIM_API const char* fsim_store_status_name(int status) {
    return status < 0 || status > static_cast<int>(fsim::ms::StoreStatus::PoweringDown) ? "" : fsim::ms::storeStatusName(static_cast<fsim::ms::StoreStatus>(status));
}
FSIM_API const char* fsim_strike_phase_name(int phase) {
    return phase < 0 || phase > static_cast<int>(fsim::ms::StrikePhase::Done) ? "" : fsim::ms::strikePhaseName(static_cast<fsim::ms::StrikePhase>(phase));
}
