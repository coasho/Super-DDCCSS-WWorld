// C ABI for DMS links and C2 authority over session::World.

#include "fsim/fsim_c.h"

#include "sdk/CAbiSupport.h"
#include "sdk/Handles.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <optional>
#include <vector>

extern "C" {

// --- Systems, nets and the DMS (ABI 1.61; ADR-31, AB-4) ---------------------------------------------------------------

namespace {
fsim::comm::NetSpec netSpecOf(const fsim_net_spec& c) {
    fsim::comm::NetSpec s;
    if (c.name) s.name = c.name;
    s.rangeM = c.range_m, s.frequencyHz = c.frequency_hz, s.lineOfSight = c.line_of_sight != 0;
    s.latencyS = c.latency_s, s.jitterS = c.jitter_s, s.lossProbability = c.loss_probability;
    s.dataRateBps = c.data_rate_bps, s.maxQueueS = c.max_queue_s, s.reliableTimeoutS = c.reliable_timeout_s, s.retryS = c.retry_s;
    s.maxPayloadBytes = c.max_payload_bytes, s.headerBytes = c.header_bytes;
    return s;
}
void netSpecToC(const fsim::comm::NetInfo& n, fsim_net_spec& c) {
    const auto& s = n.spec;
    c.name = n.name.c_str();
    c.range_m = s.rangeM, c.frequency_hz = s.frequencyHz, c.line_of_sight = s.lineOfSight ? 1 : 0;
    c.latency_s = s.latencyS, c.jitter_s = s.jitterS, c.loss_probability = s.lossProbability;
    c.data_rate_bps = s.dataRateBps, c.max_queue_s = s.maxQueueS, c.reliable_timeout_s = s.reliableTimeoutS, c.retry_s = s.retryS;
    c.max_payload_bytes = s.maxPayloadBytes, c.header_bytes = s.headerBytes;
}
} // namespace

FSIM_API void fsim_net_spec_init(fsim_net_spec* spec) {
    if (!spec) return;
    std::memset(spec, 0, sizeof *spec);
    spec->struct_size = sizeof *spec;
    fsim::comm::NetInfo d;
    netSpecToC(d, *spec);
    spec->name = nullptr;
}

FSIM_API void fsim_c2_node_spec_init(fsim_c2_node_spec* spec) {
    if (!spec) return;
    std::memset(spec, 0, sizeof *spec);
    spec->struct_size = sizeof *spec;
}

FSIM_API int fsim_world_create_c2_node(fsim_world* world, const fsim_c2_node_spec* spec, uint32_t* node) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !spec || !node || spec->struct_size < sizeof(fsim_c2_node_spec)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::C2NodeSpec s;
    if (spec->name) s.name = spec->name;
    s.latitudeRad = spec->latitude_rad, s.longitudeRad = spec->longitude_rad, s.altitudeMslM = spec->altitude_msl_m;
    *node = world->world.createC2Node(s);
    return *node ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "fsim_world_create_c2_node: not a position");
}

FSIM_API int fsim_world_remove_c2_node(fsim_world* world, uint32_t node) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    return world && world->world.removeC2Node(node) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_world_create_net(fsim_world* world, const fsim_net_spec* spec, uint32_t* net) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !net || (spec && spec->struct_size < sizeof(fsim_net_spec))) return FSIM_INVALID_ARGUMENT;
    *net = world->world.createNet(spec ? netSpecOf(*spec) : fsim::comm::NetSpec{});
    return FSIM_OK;
}

FSIM_API int fsim_world_join_net(fsim_world* world, uint32_t entity, uint32_t net) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    return world && world->world.joinNet(entity, net) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_world_leave_net(fsim_world* world, uint32_t entity, uint32_t net) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    return world && world->world.leaveNet(entity, net) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_world_assign_system(fsim_world* world, uint32_t entity, uint32_t system, const char* tail_number, uint32_t* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    const uint32_t id = world->world.assignSystem(entity, system, tail_number ? tail_number : "");
    if (out) *out = id;
    return id ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "fsim_world_assign_system: another's SystemID, or not a vehicle or C2 node");
}

FSIM_API uint32_t fsim_dms_system(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    return world ? world->world.dms().systemOf(entity) : 0;
}

FSIM_API uint32_t fsim_dms_net_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    return world ? static_cast<uint32_t>(world->world.dms().netsOf(entity).size()) : 0;
}

FSIM_API int fsim_dms_net(const fsim_world* world, uint32_t entity, uint32_t index, uint32_t* net, fsim_net_spec* spec) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    const auto nets = world->world.dms().netsOf(entity);
    if (index >= nets.size() || (spec && spec->struct_size < sizeof(fsim_net_spec))) return FSIM_INVALID_ARGUMENT;
    if (net) *net = nets[index].id;
    if (spec) {
        // (the name from the world's own copy: valid until the next net is made)
        if (const auto* n = world->world.dms().netInfo(nets[index].id)) netSpecToC(*n, *spec);
    }
    return FSIM_OK;
}

FSIM_API int fsim_dms_send(fsim_world* world, uint32_t entity, uint32_t topic, const void* bytes, size_t length,
                           const uint32_t* destinations, uint32_t destination_count, uint64_t* message, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED); // (ADR-31: as its own system alone)
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || (length && !bytes) || (destination_count && !destinations)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DmsMessage m;
    m.topic = topic;
    if (length) m.bytes.assign(static_cast<const uint8_t*>(bytes), static_cast<const uint8_t*>(bytes) + length);
    m.destinations.assign(destinations, destinations + destination_count);
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.dms().send(entity, m, world->world.simTime(), why);
    if (message) *message = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_dms_send: refused (fsim_dms_reason " + std::to_string(static_cast<int>(why)) + ")");
}

FSIM_API int fsim_dms_subscribe(fsim_world* world, uint32_t entity, uint32_t topic, int32_t on) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_INGRESS(world, FSIM_NOT_ALLOWED);
    return world && world->world.dms().subscribe(entity, topic, on != 0) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API int fsim_dms_subscribe_all(fsim_world* world, uint32_t entity, int32_t on) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_INGRESS(world, FSIM_NOT_ALLOWED);
    return world && world->world.dms().subscribeAll(entity, on != 0) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API uint32_t fsim_dms_subscriptions(const fsim_world* world, uint32_t entity, uint32_t* topics, uint32_t max, int32_t* all) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    if (!world) return 0;
    const auto list = world->world.dms().subscriptions(entity);
    for (uint32_t i = 0; topics && i < max && i < list.size(); ++i) topics[i] = list[i];
    if (all) *all = world->world.dms().subscribedAll(entity) ? 1 : 0;
    return static_cast<uint32_t>(list.size());
}

FSIM_API uint32_t fsim_dms_inbox_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0); // (its own system's inbox alone)
    FSIM_MA_INGRESS(world, 0);
    const auto* in = world ? world->world.dms().inbox(entity) : nullptr;
    return in ? static_cast<uint32_t>(in->size()) : 0;
}

FSIM_API int fsim_dms_inbox_get(const fsim_world* world, uint32_t entity, uint32_t index, fsim_dms_received* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_INGRESS(world, FSIM_NOT_ALLOWED);
    const auto* in = world && out ? world->world.dms().inbox(entity) : nullptr;
    if (!in || index >= in->size()) return FSIM_INVALID_ARGUMENT;
    const auto& r = (*in)[index];
    out->message = r.message, out->source = r.source, out->topic = r.topic, out->net = r.net;
    out->time_sent = r.timeSent, out->time_delivered = r.timeDelivered;
    out->bytes = r.bytes.data(), out->length = r.bytes.size();
    return FSIM_OK;
}

FSIM_API int fsim_dms_inbox_take(fsim_world* world, uint32_t entity, uint32_t count) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_INGRESS(world, FSIM_NOT_ALLOWED);
    if (!world || !world->world.dms().inbox(entity)) return FSIM_INVALID_ARGUMENT;
    world->world.dms().take(entity, count);
    return FSIM_OK;
}

FSIM_API int fsim_dms_transmission_status(const fsim_world* world, uint32_t entity, uint64_t message, fsim_dms_transmission* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto t = world && out ? world->world.dms().transmission(entity, message) : std::nullopt;
    if (!t) return FSIM_INVALID_ARGUMENT;
    out->message = t->message, out->topic = t->topic, out->time_sent = t->timeSent;
    out->published = t->published ? 1 : 0, out->queue_dropped = t->queueDropped ? 1 : 0;
    out->reached = t->reached, out->destination_count = static_cast<uint32_t>(t->destinations.size());
    return FSIM_OK;
}

FSIM_API int fsim_dms_transmission_destination(const fsim_world* world, uint32_t entity, uint64_t message, uint32_t index,
                                               uint32_t* system, int32_t* delivery) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto t = world ? world->world.dms().transmission(entity, message) : std::nullopt;
    if (!t || index >= t->destinations.size()) return FSIM_INVALID_ARGUMENT;
    if (system) *system = t->destinations[index].first;
    if (delivery) *delivery = static_cast<int32_t>(t->destinations[index].second);
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_system_entity(const fsim_world* world, uint32_t system) {
    FSIM_NOT_AGENT(world, 0);
    return world ? world->world.dms().entityOf(system) : 0;
}

FSIM_API uint32_t fsim_world_entity_system(const fsim_world* world, uint32_t entity) {
    FSIM_NOT_AGENT(world, 0);
    return world ? world->world.dms().systemOf(entity) : 0;
}

FSIM_API uint32_t fsim_world_net_count(const fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world ? static_cast<uint32_t>(world->world.dms().netCount()) : 0;
}

FSIM_API int fsim_world_net_stats(const fsim_world* world, uint32_t net, fsim_net_stats* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out || net == 0 || net > world->world.dms().netCount()) return FSIM_INVALID_ARGUMENT;
    const auto s = world->world.dms().stats(net);
    out->transmissions = s.transmissions, out->delivered = s.delivered, out->duplicates = s.duplicates;
    out->out_of_range = s.outOfRange, out->masked = s.masked, out->lost = s.lost, out->queue_dropped = s.queueDropped;
    out->failed = s.failed, out->overflowed = s.overflowed;
    return FSIM_OK;
}

// --- A-GRA's native messages and the picture (ABI 1.62; AB-4b) -------------------------------------------------------

FSIM_API void fsim_dissemination_plan_init(fsim_dissemination_plan* plan) {
    if (!plan) return;
    std::memset(plan, 0, sizeof *plan);
    plan->struct_size = sizeof *plan;
    const fsim::comm::DisseminationPlan d;
    plan->peer_timeout_s = d.peerTimeoutS, plan->track_timeout_s = d.trackTimeoutS;
}

FSIM_API void fsim_peer_report_init(fsim_peer_report* report) {
    if (!report) return;
    std::memset(report, 0, sizeof *report);
    report->struct_size = sizeof *report;
    report->tail_number = "";
}

FSIM_API void fsim_link_track_init(fsim_link_track* track) {
    if (!track) return;
    std::memset(track, 0, sizeof *track);
    track->struct_size = sizeof *track;
}

FSIM_API int fsim_world_set_dissemination(fsim_world* world, uint32_t entity, const fsim_dissemination_plan* plan) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !plan || plan->struct_size < sizeof(fsim_dissemination_plan)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DisseminationPlan p;
    p.positionPeriodS = plan->position_period_s, p.statusPeriodS = plan->status_period_s, p.tracksPeriodS = plan->tracks_period_s;
    p.peerTimeoutS = plan->peer_timeout_s, p.trackTimeoutS = plan->track_timeout_s;
    return world->world.setDissemination(entity, p) ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "fsim_world_set_dissemination: on no net");
}

FSIM_API int fsim_dms_publish(fsim_world* world, uint32_t entity, uint32_t topic, uint32_t payload, const uint32_t* destinations,
                              uint32_t destination_count, uint64_t* first, uint32_t* sent, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED); // (as its own system; the MA core builds them from its reports)
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || (destination_count && !destinations)) return FSIM_INVALID_ARGUMENT;
    const std::vector<fsim::comm::SystemId> to(destinations, destinations + destination_count);
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const auto ids = world->world.publishNative(entity, topic, static_cast<uint16_t>(std::min<uint32_t>(payload, 0xFFFFu)), to, why);
    if (first) *first = ids.empty() ? 0 : ids.front();
    if (sent) *sent = static_cast<uint32_t>(ids.size());
    if (reason) *reason = static_cast<int32_t>(why);
    // (no tracks to send is not a refusal)
    return ids.empty() && why != fsim::comm::DmsReason::None ? fail(FSIM_ERROR, "fsim_dms_publish: refused") : FSIM_OK;
}

FSIM_API uint32_t fsim_dms_peer_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    const auto* p = world ? world->world.dms().peersOf(entity) : nullptr;
    return p ? static_cast<uint32_t>(p->size()) : 0;
}

FSIM_API int fsim_dms_peer_get(const fsim_world* world, uint32_t entity, uint32_t index, fsim_peer_report* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* list = world && out ? world->world.dms().peersOf(entity) : nullptr;
    if (!list || index >= list->size() || out->struct_size < sizeof(fsim_peer_report)) return FSIM_INVALID_ARGUMENT;
    const auto& p = (*list)[index];
    out->system = p.system, out->net = p.net, out->received_s = p.receivedS, out->tail_number = p.tailNumber.c_str();
    out->has_position = p.hasPosition ? 1 : 0;
    const auto& q = p.position;
    out->time_s = q.timeS, out->latitude_rad = q.latitudeRad, out->longitude_rad = q.longitudeRad, out->altitude_msl_m = q.altitudeMslM;
    for (int i = 0; i < 3; ++i) out->velocity_ned_ms[i] = q.velocityNedMs[i];
    out->heading_rad = q.headingRad, out->airspeed_true_ms = q.airspeedTrueMs, out->airspeed_calibrated_ms = q.airspeedCalibratedMs;
    out->mach = q.mach;
    return FSIM_OK;
}

FSIM_API uint32_t fsim_dms_link_track_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    const auto* t = world ? world->world.dms().linkTracksOf(entity) : nullptr;
    return t ? static_cast<uint32_t>(t->size()) : 0;
}

FSIM_API int fsim_dms_link_track_get(const fsim_world* world, uint32_t entity, uint32_t index, fsim_link_track* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* list = world && out ? world->world.dms().linkTracksOf(entity) : nullptr;
    if (!list || index >= list->size() || out->struct_size < sizeof(fsim_link_track)) return FSIM_INVALID_ARGUMENT;
    const auto& l = (*list)[index];
    const auto& r = l.report;
    out->source = l.source, out->net = l.net, out->received_s = l.receivedS;
    out->track = r.track, out->payload = r.payload, out->updates = r.updates, out->time_s = r.timeS;
    out->latitude_rad = r.latitudeRad, out->longitude_rad = r.longitudeRad, out->height_m = r.heightM;
    for (int i = 0; i < 3; ++i) out->velocity_ned_ms[i] = r.velocityNedMs[i];
    out->position_sigma_m = r.positionSigmaM, out->velocity_sigma_ms = r.velocitySigmaMs;
    return FSIM_OK;
}

FSIM_API int fsim_vehicle_designate_link(fsim_world* world, uint32_t id, uint32_t source, uint32_t payload, uint64_t track, uint32_t* target) {
    FSIM_AGENT_VEHICLE(world, id, FSIM_NOT_ALLOWED);
    if (!world || !target || payload > 0xFFFFu) return FSIM_INVALID_ARGUMENT;
    *target = world->world.designateLink(id, source, static_cast<uint16_t>(payload), track);
    return FSIM_OK;
}

FSIM_API uint32_t fsim_vehicle_designation_source(const fsim_world* world, uint32_t id, uint32_t target) {
    FSIM_AGENT_VEHICLE(world, id, 0);
    const auto* d = world ? world->world.designation(target) : nullptr;
    return d && d->owner == id ? d->source : 0;
}

FSIM_API uint32_t fsim_world_link_track_truth(const fsim_world* world, uint32_t source, uint32_t payload, uint64_t track) {
    FSIM_NOT_AGENT(world, 0);
    if (!world || payload > 0xFFFFu) return 0;
    const uint32_t entity = world->world.dms().entityOf(source);
    for (const auto& t : world->world.amtiTracks(entity, static_cast<uint16_t>(payload)))
        if (t.id == track) return t.truth;
    return 0;
}

// --- C2 authority (ABI 1.63; AB-5a) ----------------------------------------------------------------------------------

FSIM_API void fsim_c2_authority_init(fsim_c2_authority* a) {
    if (!a) return;
    std::memset(a, 0, sizeof *a);
    a->struct_size = sizeof *a;
}

FSIM_API void fsim_c2_status_init(fsim_c2_status* s) {
    if (!s) return;
    std::memset(s, 0, sizeof *s);
    s->struct_size = sizeof *s;
}

FSIM_API int fsim_world_set_rbac(fsim_world* world, uint32_t entity, const fsim_rbac_role* roles, uint32_t count) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || (count && !roles)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::RbacConfig c;
    for (uint32_t i = 0; i < count; ++i)
        c.roles.push_back({roles[i].id, roles[i].name ? roles[i].name : "", roles[i].permissions, roles[i].approves_strikes != 0});
    world->world.maCore().setRbac(entity, c);
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_rbac_role_count(fsim_world* world, uint32_t entity) {
    FSIM_NOT_AGENT(world, 0);
    return world ? static_cast<uint32_t>(world->world.maCore().rbacOf(entity).roles.size()) : 0;
}

FSIM_API int fsim_world_rbac_role(fsim_world* world, uint32_t entity, uint32_t index, fsim_rbac_role* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto& roles = world->world.maCore().rbacOf(entity).roles;
    if (index >= roles.size()) return FSIM_INVALID_ARGUMENT;
    out->id = roles[index].id, out->name = roles[index].name.c_str(), out->permissions = roles[index].permissions;
    out->approves_strikes = roles[index].approvesStrikes ? 1 : 0;
    return FSIM_OK;
}

FSIM_API int fsim_agent_authority(const fsim_world* world, uint32_t entity, fsim_c2_authority* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    if (!world || !out || out->struct_size < sizeof(fsim_c2_authority)) return FSIM_INVALID_ARGUMENT;
    const auto* a = world->world.maCore().authority(entity);
    out->designated = a && a->designated ? 1 : 0;
    out->designated_modes = a ? a->designatedModes : 0, out->designated_by = a ? a->designatedBy : 0;
    out->designated_s = a ? a->designatedS : 0.0;
    out->operator_count = a ? static_cast<uint32_t>(a->operators.size()) : 0;
    out->command_count = a ? static_cast<uint32_t>(a->c2.size()) : 0;
    out->lost_comms_set = a && a->lostCommsSet ? 1 : 0, out->lost = a && a->lost ? 1 : 0;
    out->lost_comms_threshold_s = a ? a->lostComms.thresholdS : 0.0, out->lost_since_s = a ? a->lostSinceS : 0.0;
    out->lost_comms_plan = a && a->lostComms.activatePlan ? a->lostComms.planId : 0, out->alerts = a ? a->alerts : 0;
    out->package_id = a ? a->packageId : 0, out->package_leader = a ? a->leader : 0;
    out->member_count = a ? static_cast<uint32_t>(a->members.size()) : 0;
    return FSIM_OK;
}

FSIM_API int fsim_agent_authority_operator(const fsim_world* world, uint32_t entity, uint32_t index, uint32_t* system, uint32_t* role_id) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* a = world ? world->world.maCore().authority(entity) : nullptr;
    if (!a || index >= a->operators.size()) return FSIM_INVALID_ARGUMENT;
    if (system) *system = a->operators[index].system;
    if (role_id) *role_id = a->operators[index].roleId;
    return FSIM_OK;
}

FSIM_API const char* fsim_agent_operator_callsign(fsim_world* world, uint32_t entity, uint32_t index) {
    FSIM_AGENT_VEHICLE(world, entity, "");
    const auto* a = world ? world->world.maCore().authority(entity) : nullptr;
    return a && index < a->operators.size() ? a->operators[index].callsign.c_str() : "";
}

FSIM_API const char* fsim_agent_role_name(fsim_world* world, uint32_t entity, uint32_t role_id) {
    FSIM_AGENT_VEHICLE(world, entity, "");
    if (!world) return "";
    for (const auto& r : world->world.maCore().rbacOf(entity).roles)
        if (r.id == role_id) return r.name.c_str();
    return "";
}

FSIM_API int fsim_agent_authority_command(const fsim_world* world, uint32_t entity, uint32_t index, uint64_t* activity, uint32_t* system,
                                          uint64_t* command_id) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* a = world ? world->world.maCore().authority(entity) : nullptr;
    if (!a || index >= a->c2.size()) return FSIM_INVALID_ARGUMENT;
    if (activity) *activity = a->c2[index].activity;
    if (system) *system = a->c2[index].system;
    if (command_id) *command_id = a->c2[index].commandId;
    return FSIM_OK;
}

FSIM_API int fsim_c2_declare_role(fsim_world* world, uint32_t entity, uint32_t role_id, const char* callsign, const uint32_t* destinations,
                                  uint32_t destination_count, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED); // (as its own system)
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || (destination_count && !destinations)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const bool sent = world->world.maCore().declareRole(world->world, entity, role_id, callsign ? callsign : "",
                                                       std::vector<uint32_t>(destinations, destinations + destination_count), why);
    if (reason) *reason = static_cast<int32_t>(why);
    return sent ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_declare_role: not sent");
}

FSIM_API int fsim_c2_designate_control(fsim_world* world, uint32_t entity, uint32_t ma, uint32_t modes, uint64_t* command_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().designateControl(world->world, entity, ma, modes, why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_designate_control: not sent");
}

FSIM_API int fsim_c2_command_hsa(fsim_world* world, uint32_t entity, uint32_t ma, const double* hsa, uint64_t* command_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || !hsa) return FSIM_INVALID_ARGUMENT;
    fsim::comm::FlightCommand c;
    c.kind = fsim::comm::FlightCommand::Hsa;
    c.headingRad = hsa[0], c.courseRad = hsa[1], c.speed = hsa[2], c.speedReference = hsa[3], c.altitudeM = hsa[4], c.altitudeReference = hsa[5];
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().flightCommand(world->world, entity, ma, c, why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_command_hsa: not sent");
}

FSIM_API int fsim_c2_command_route(fsim_world* world, uint32_t entity, uint32_t ma, const double* points, uint32_t count, uint64_t* command_id,
                                   int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || !points || count == 0 || count > 16) return FSIM_INVALID_ARGUMENT;
    fsim::comm::FlightCommand c;
    c.kind = fsim::comm::FlightCommand::Waypoints, c.pointCount = count;
    for (uint32_t i = 0; i < count; ++i)
        for (int k = 0; k < 4; ++k) c.points[i][k] = points[i * 4 + static_cast<uint32_t>(k)];
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().flightCommand(world->world, entity, ma, c, why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_command_route: not sent");
}

FSIM_API int fsim_c2_cancel_command(fsim_world* world, uint32_t entity, uint32_t ma, uint64_t command_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim::comm::FlightCommand c;
    c.kind = fsim::comm::FlightCommand::Cancel, c.commandId = command_id;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().flightCommand(world->world, entity, ma, c, why);
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_cancel_command: not sent");
}

FSIM_API uint32_t fsim_c2_command_status_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    const auto* s = world ? world->world.dms().commandStatuses(entity) : nullptr;
    return s ? static_cast<uint32_t>(s->size()) : 0;
}

FSIM_API int fsim_c2_command_status_get(const fsim_world* world, uint32_t entity, uint32_t index, fsim_c2_status* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* list = world && out ? world->world.dms().commandStatuses(entity) : nullptr;
    if (!list || index >= list->size() || out->struct_size < sizeof(fsim_c2_status)) return FSIM_INVALID_ARGUMENT;
    const auto& e = (*list)[index];
    out->from = e.from, out->received_s = e.receivedS, out->command_id = e.status.commandId, out->topic = e.status.topic;
    out->state = e.status.state, out->reason = e.status.reason, out->activity = e.status.activity, out->time_s = e.status.timeS;
    out->percent = e.status.percent;
    return FSIM_OK;
}

// --- Tasks, overrides, release consent (ABI 1.64; AB-5b) ---------------------------------------------------------------

namespace {
void taskToC(const fsim::comm::TaskOrder& t, fsim_c2_task& c) {
    c.task_id = t.taskId, c.type = t.type, c.priority = t.priority;
    c.latitude_rad = t.latitudeRad, c.longitude_rad = t.longitudeRad, c.altitude_m = t.altitudeM, c.radius_m = t.radiusM;
    c.start_s = t.startS, c.end_s = t.endS;
    c.target_source = t.targetSource, c.target_payload = t.targetPayload, c.target_track = t.targetTrack;
    std::memcpy(c.text, t.text, sizeof c.text);
}
} // namespace

FSIM_API void fsim_c2_task_init(fsim_c2_task* task) {
    if (!task) return;
    std::memset(task, 0, sizeof *task);
    task->struct_size = sizeof *task;
    taskToC(fsim::comm::TaskOrder{}, *task);
    task->percent = std::numeric_limits<double>::quiet_NaN();
}

FSIM_API void fsim_consent_request_init(fsim_consent_request* request) {
    if (!request) return;
    std::memset(request, 0, sizeof *request);
    request->struct_size = sizeof *request;
}

FSIM_API int fsim_world_set_strike_policy(fsim_world* world, uint32_t entity, int policy) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || policy < 0 || policy > 2) return FSIM_INVALID_ARGUMENT;
    fsim::comm::RbacConfig c = world->world.maCore().rbac(entity);
    c.strikePolicy = static_cast<fsim::comm::StrikePolicy>(policy);
    world->world.maCore().setRbac(entity, c);
    return FSIM_OK;
}

FSIM_API int fsim_world_strike_policy(fsim_world* world, uint32_t entity) {
    FSIM_NOT_AGENT(world, -1);
    return world ? static_cast<int>(world->world.maCore().rbac(entity).strikePolicy) : -1;
}

FSIM_API uint32_t fsim_agent_c2_task_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    const auto* t = world ? world->world.maCore().tasks(entity) : nullptr;
    return t ? static_cast<uint32_t>(t->size()) : 0;
}

FSIM_API int fsim_agent_c2_task_get(const fsim_world* world, uint32_t entity, uint32_t index, fsim_c2_task* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* list = world && out ? world->world.maCore().tasks(entity) : nullptr;
    if (!list || index >= list->size() || out->struct_size < sizeof(fsim_c2_task)) return FSIM_INVALID_ARGUMENT;
    const auto& r = (*list)[index];
    out->from = r.from, out->received_s = r.receivedS, out->state = static_cast<int32_t>(r.state), out->percent = r.percent;
    taskToC(r.order, *out);
    return FSIM_OK;
}

FSIM_API int fsim_agent_answer_task(fsim_world* world, uint32_t entity, uint32_t from, uint64_t task_id, int state, uint32_t code, double percent) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, static_cast<int32_t*>(nullptr), FSIM_NOT_ALLOWED);
    if (!world || state < 1 || state > 4) return FSIM_INVALID_ARGUMENT;
    const bool ok = world->world.maCore().answerTask(world->world, entity, from, task_id, static_cast<fsim::comm::ReceivedTask::State>(state),
                                                    code, percent);
    return ok ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "fsim_agent_answer_task: not a task it holds, or not in that state");
}

FSIM_API uint32_t fsim_agent_override_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    const auto* o = world ? world->world.maCore().overrides(entity) : nullptr;
    return o ? static_cast<uint32_t>(o->size()) : 0;
}

FSIM_API int fsim_agent_override_get(const fsim_world* world, uint32_t entity, uint32_t index, uint32_t* from, int32_t* kind, double* value,
                                     double* expiry_s) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* list = world ? world->world.maCore().overrides(entity) : nullptr;
    if (!list || index >= list->size()) return FSIM_INVALID_ARGUMENT;
    const auto& o = (*list)[index];
    if (from) *from = o.from;
    if (kind) *kind = static_cast<int32_t>(o.kind);
    if (value) *value = o.value;
    if (expiry_s) *expiry_s = o.expiryS;
    return FSIM_OK;
}

FSIM_API int fsim_c2_send_task(fsim_world* world, uint32_t entity, uint32_t ma, const fsim_c2_task* task, uint64_t* task_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || !task || task->struct_size < sizeof(fsim_c2_task)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::TaskOrder t;
    t.type = task->type, t.priority = task->priority;
    t.latitudeRad = task->latitude_rad, t.longitudeRad = task->longitude_rad, t.altitudeM = task->altitude_m, t.radiusM = task->radius_m;
    t.startS = task->start_s, t.endS = task->end_s;
    t.targetSource = task->target_source, t.targetPayload = task->target_payload, t.targetTrack = task->target_track;
    std::memcpy(t.text, task->text, sizeof t.text);
    t.text[sizeof t.text - 1] = 0;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().sendTask(world->world, entity, ma, t, why);
    if (task_id) *task_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_send_task: not sent");
}

FSIM_API int fsim_c2_set_override(fsim_world* world, uint32_t entity, uint32_t ma, int kind, int32_t cancel, double value, double expiry_s,
                                  uint64_t* command_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || kind < 0 || kind > 1) return FSIM_INVALID_ARGUMENT;
    fsim::comm::OverrideCommand c;
    c.kind = static_cast<uint32_t>(kind), c.cancel = cancel ? 1u : 0u, c.value = value, c.expiryS = expiry_s;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().sendOverride(world->world, entity, ma, c, why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_set_override: not sent");
}

FSIM_API uint32_t fsim_c2_consent_count(const fsim_world* world, uint32_t entity) {
    FSIM_AGENT_VEHICLE(world, entity, 0);
    const auto* c = world ? const_cast<fsim_world*>(world)->world.dms().consentsOf(entity) : nullptr;
    return c ? static_cast<uint32_t>(c->size()) : 0;
}

FSIM_API int fsim_c2_consent_get(const fsim_world* world, uint32_t entity, uint32_t index, fsim_consent_request* out) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* list = world && out ? const_cast<fsim_world*>(world)->world.dms().consentsOf(entity) : nullptr;
    if (!list || index >= list->size() || out->struct_size < sizeof(fsim_consent_request)) return FSIM_INVALID_ARGUMENT;
    const auto& p = (*list)[index];
    const auto& q = p.request;
    out->from = p.from, out->received_s = p.receivedS, out->answered = p.answered ? 1 : 0;
    out->request_id = q.requestId, out->strike = q.strike;
    std::memcpy(out->store, q.store, sizeof out->store);
    out->quantity = q.quantity, out->target_source = q.targetSource, out->target_payload = q.targetPayload, out->target_track = q.targetTrack;
    out->latitude_rad = q.latitudeRad, out->longitude_rad = q.longitudeRad, out->altitude_m = q.altitudeM, out->time_s = q.timeS;
    return FSIM_OK;
}

FSIM_API int fsim_c2_answer_consent(fsim_world* world, uint32_t entity, uint32_t ma, uint64_t request_id, int32_t granted, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const bool sent = world->world.maCore().answerConsent(world->world, entity, ma, request_id, granted != 0, why);
    if (reason) *reason = static_cast<int32_t>(why);
    return sent ? FSIM_OK : fail(FSIM_INVALID_ARGUMENT, "fsim_c2_answer_consent: not a request it holds unanswered");
}

// --- Lost communication and teams (ABI 1.65; AB-5c) --------------------------------------------------------------------

FSIM_API int fsim_agent_package_member(const fsim_world* world, uint32_t entity, uint32_t index, uint32_t* system) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    const auto* a = world ? world->world.maCore().authority(entity) : nullptr;
    if (!a || index >= a->members.size()) return FSIM_INVALID_ARGUMENT;
    if (system) *system = a->members[index];
    return FSIM_OK;
}

FSIM_API int fsim_c2_set_lost_comms(fsim_world* world, uint32_t entity, uint32_t ma, double threshold_s, uint64_t plan_id, uint64_t* command_id,
                                    int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world) return FSIM_INVALID_ARGUMENT;
    fsim::comm::LostCommsSettings l;
    l.thresholdS = threshold_s, l.activatePlan = plan_id ? 1u : 0u, l.planId = plan_id;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().sendLostComms(world->world, entity, ma, l, why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_set_lost_comms: not sent");
}

FSIM_API int fsim_c2_form_package(fsim_world* world, uint32_t entity, uint32_t package_id, uint32_t leader, const uint32_t* members, uint32_t count,
                                  uint64_t* command_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || (count && !members)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().formPackage(world->world, entity, package_id, leader, std::vector<uint32_t>(members, members + count), why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_form_package: not sent");
}

FSIM_API int fsim_c2_dissolve_package(fsim_world* world, uint32_t entity, uint32_t package_id, const uint32_t* systems, uint32_t count,
                                      uint64_t* command_id, int32_t* reason) {
    FSIM_AGENT_VEHICLE(world, entity, FSIM_NOT_ALLOWED);
    FSIM_MA_EGRESS(world, reason, FSIM_NOT_ALLOWED);
    if (!world || (count && !systems)) return FSIM_INVALID_ARGUMENT;
    fsim::comm::DmsReason why = fsim::comm::DmsReason::None;
    const uint64_t id = world->world.maCore().dissolvePackage(world->world, entity, package_id, std::vector<uint32_t>(systems, systems + count), why);
    if (command_id) *command_id = id;
    if (reason) *reason = static_cast<int32_t>(why);
    return id ? FSIM_OK : fail(FSIM_ERROR, "fsim_c2_dissolve_package: not sent");
}

// --- MA services (ABI 1.72; docs/sdk/mission-training.md, section 9) ----------------------------------------------------

FSIM_API void fsim_ma_service_binding_init(fsim_ma_service_binding* binding) {
    if (!binding) return;
    std::memset(binding, 0, sizeof *binding);
    binding->struct_size = sizeof *binding;
}

FSIM_API int fsim_world_bind_ma_service(fsim_world* world, const fsim_ma_service_binding* binding) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !binding || binding->struct_size < sizeof(fsim_ma_service_binding) ||
        (binding->command_capability_count && !binding->command_capabilities))
        return FSIM_INVALID_ARGUMENT;
    fsim::session::MaServiceBinding b;
    b.serviceId = binding->service_id, b.platform = binding->platform, b.controller = binding->controller;
    for (uint32_t i = 0; i < binding->command_capability_count; ++i) {
        if (!binding->command_capabilities[i]) return FSIM_INVALID_ARGUMENT;
        b.capabilities.emplace_back(binding->command_capabilities[i]);
    }
    b.ownsDmsIngress = binding->owns_dms_ingress != 0, b.ownsDmsEgress = binding->owns_dms_egress != 0;
    std::string why;
    if (!world->world.bindMaService(b, why)) return fail(FSIM_INVALID_ARGUMENT, "fsim_world_bind_ma_service: " + why);
    fsim::sdk::lastError().clear();
    return FSIM_OK;
}

FSIM_API uint32_t fsim_world_ma_service_count(fsim_world* world) {
    FSIM_NOT_AGENT(world, 0);
    return world ? static_cast<uint32_t>(world->world.maServices().all().size()) : 0;
}

FSIM_API int fsim_world_ma_service(fsim_world* world, uint32_t index, fsim_ma_service_binding* out) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !out) return FSIM_INVALID_ARGUMENT;
    const auto& all = world->world.maServices().all();
    if (index >= all.size()) return FSIM_INVALID_ARGUMENT;
    const fsim::session::MaServiceBinding& b = all[index];
    world->serviceCapabilities.clear();
    for (const std::string& c : b.capabilities) world->serviceCapabilities.push_back(world->intern(c));
    fsim_ma_service_binding local;
    fsim_ma_service_binding_init(&local);
    local.platform = b.platform, local.service_id = b.serviceId, local.controller = b.controller;
    local.command_capability_count = static_cast<uint32_t>(world->serviceCapabilities.size());
    local.command_capabilities = world->serviceCapabilities.empty() ? nullptr : world->serviceCapabilities.data();
    local.owns_dms_ingress = b.ownsDmsIngress ? 1 : 0, local.owns_dms_egress = b.ownsDmsEgress ? 1 : 0;
    return copyOut(local, out) ? FSIM_OK : FSIM_INVALID_ARGUMENT;
}

FSIM_API uint32_t fsim_world_ma_capability_owner(fsim_world* world, uint32_t platform, const char* capability) {
    FSIM_NOT_AGENT(world, 0);
    return world && capability ? world->world.maServices().owner(platform, capability) : 0;
}

} // extern "C"
