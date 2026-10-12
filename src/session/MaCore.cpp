#include "session/MaCore.h"

#include "session/World.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <variant>

namespace fsim::session {

namespace {
bool hold(double v) noexcept { return std::isnan(v); }
} // namespace

comm::RbacConfig MaCore::rbac(std::uint32_t entity) const {
    const auto it = states_.find(entity);
    return it == states_.end() ? comm::RbacConfig::agraDefaults() : it->second.rbac;
}

const MaCore::Authority* MaCore::authority(std::uint32_t entity) const noexcept {
    const auto it = states_.find(entity);
    return it == states_.end() ? nullptr : &it->second.authority;
}

comm::SystemId MaCore::c2Of(control::ActivityId activity) const noexcept {
    const auto it = states_.find(control::activityVehicle(activity));
    if (it != states_.end())
        for (const C2Activity& a : it->second.authority.c2)
            if (a.activity == activity) return a.system;
    return comm::kNoSystem;
}

control::ControllerId MaCore::controllerFor(comm::SystemId system) {
    const auto it = controllers_.find(system);
    if (it != controllers_.end()) return it->second;
    const control::ControllerId c = kC2Controllers + static_cast<control::ControllerId>(controllers_.size()) + 1;
    controllers_[system] = c;
    return c;
}

const comm::Role& MaCore::leaderRole() {
    static const comm::Role leader{0, "team leader", comm::permission::kAll & ~comm::permission::kRoles, false};
    return leader;
}

const comm::Role* MaCore::roleOf(const State& s, comm::SystemId sender) const noexcept {
    // its team's leader: a member accepts what it sends as C2's (Peer 1.1) - the role it declared, if any, besides
    const comm::Role* declared = nullptr;
    for (const Operator& o : s.authority.operators)
        if (o.system == sender)
            for (const comm::Role& r : s.rbac.roles)
                if (r.id == o.roleId) declared = &r;
    if (declared) return declared;
    if (s.authority.leader != comm::kNoSystem && sender == s.authority.leader) return &leaderRole();
    return nullptr;
}

void MaCore::reply(World& world, std::uint32_t entity, comm::SystemId to, const comm::CommandStatus& status) {
    comm::DmsMessage m;
    m.topic = comm::topic::kCommandStatus;
    m.bytes = comm::encode(status);
    m.destinations = {to};
    comm::DmsReason why;
    world.dms().send(entity, m, world.simTime(), why, true); // (unreachable now: the sender learns nothing - as over the air)
}

void MaCore::retireEntity(std::uint32_t entity) noexcept {
    const auto it = states_.find(entity);
    if (it == states_.end()) return;
    it->second.retired = true;
    it->second.consents.release();
}

void MaCore::step(World& world) {
    for (const std::uint32_t entity : world.dms().coreWaiting())
        for (const comm::DmsReceived& m : world.dms().takeCore(entity)) {
            if (!world.vehicleState(entity)) continue; // (a C2 node has no MA core: what it is sent goes nowhere)
            handle(world, entity, m);
        }
    // its strikes' release consent (AB-5b): every platform on a net (one on none: the range answers, as before)
    for (const std::uint32_t entity : world.dms().entities())
        if (world.vehicleState(entity) && world.missionSystems(entity)) consents(world, entity, state(entity));
    for (auto& [entity, s] : states_)
        if (needsMaintenance(s) && s.authority.lostCommsSet) watchComms(world, entity, s);
    // C2's activities that ended: how, to the C2 that commanded them; overrides that expired
    for (auto& [entity, s] : states_) {
        if (!needsMaintenance(s)) continue;
        s.overrides.erase(std::remove_if(s.overrides.begin(), s.overrides.end(),
                                         [&](const comm::ActiveOverride& o) { return o.expiryS <= world.simTime(); }),
                          s.overrides.end());
        auto& list = s.authority.c2;
        for (std::size_t i = 0; i < list.size();) {
            const control::ActivityRecord* r = world.activity(list[i].activity);
            if (r && r->live()) {
                ++i;
                continue;
            }
            comm::CommandStatus st;
            st.commandId = list[i].commandId, st.topic = comm::topic::kFlightCommand, st.activity = list[i].activity;
            st.timeS = world.simTime();
            st.state = !r                                              ? comm::CommandStatus::Failed
                       : r->state == control::ActivityState::Completed ? comm::CommandStatus::Completed
                       : r->state == control::ActivityState::Failed    ? comm::CommandStatus::Failed
                                                                       : comm::CommandStatus::Canceled;
            st.reason = r ? static_cast<std::uint32_t>(r->reason) : 0u;
            reply(world, entity, list[i].system, st);
            list.erase(list.begin() + static_cast<std::ptrdiff_t>(i));
        }
    }
}

void MaCore::handle(World& world, std::uint32_t entity, const comm::DmsReceived& m) {
    State& s = state(entity);
    comm::CommandStatus st;
    st.topic = m.topic, st.timeS = world.simTime();
    if (m.topic == comm::topic::kOperator) { // a C2 system declares its role (1.2.9.4)
        comm::OperatorDeclaration d;
        if (!comm::decode(m.bytes, d)) return;
        const bool known = std::any_of(s.rbac.roles.begin(), s.rbac.roles.end(), [&](const comm::Role& r) { return r.id == d.roleId; });
        if (!known) {
            st.state = comm::CommandStatus::Rejected, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
            reply(world, entity, m.source, st);
            return;
        }
        auto& ops = s.authority.operators;
        ops.erase(std::remove_if(ops.begin(), ops.end(), [&](const Operator& o) { return o.system == m.source; }), ops.end());
        ops.push_back({m.source, d.roleId, std::string(d.callsign, strnlen(d.callsign, sizeof d.callsign))});
        // the role's systems now, broadcast (1.2.9.6)
        comm::OperatorRoleReport report;
        report.roleId = d.roleId;
        for (const Operator& o : ops)
            if (o.roleId == d.roleId && report.count < 16) report.systems[report.count++] = o.system;
        for (const comm::Role& r : s.rbac.roles)
            if (r.id == d.roleId) std::memcpy(report.name, r.name.data(), std::min(r.name.size(), sizeof report.name - 1));
        comm::DmsMessage b;
        b.topic = comm::topic::kOperatorRole;
        b.bytes = comm::encode(report);
        comm::DmsReason why;
        world.dms().send(entity, b, world.simTime(), why, true);
        reply(world, entity, m.source, st); // (accepted)
        return;
    }
    // everything else: its sender's role must authorize it (1.2.9; unauthorized: answered, not acted on - 1.2.9.5)
    std::uint32_t needs = 0;
    if (m.topic == comm::topic::kControlDesignation) needs = comm::permission::kControl;
    else if (m.topic == comm::topic::kFlightCommand) needs = comm::permission::kFlight;
    else if (m.topic == comm::topic::kTask) needs = comm::permission::kTask;
    else if (m.topic == comm::topic::kOverride) needs = comm::permission::kOverride;
    else if (m.topic == comm::topic::kConsentAnswer) needs = comm::permission::kWeapons;
    else if (m.topic == comm::topic::kLostComms) needs = comm::permission::kSettings;
    else if (m.topic == comm::topic::kPackage) needs = comm::permission::kPackage;
    else return;
    if (m.topic == comm::topic::kLostComms || m.topic == comm::topic::kPackage) {
        comm::LostCommsSettings l;
        comm::PackageCommand p;
        const bool lost = m.topic == comm::topic::kLostComms;
        if (lost ? !comm::decode(m.bytes, l) : !comm::decode(m.bytes, p)) return;
        st.commandId = lost ? l.commandId : p.commandId;
        const comm::Role* role = roleOf(s, m.source);
        if (!role || !(role->permissions & needs) || (!lost && role == &leaderRole())) { // (a leader does not remake its team)
            st.state = comm::CommandStatus::Unauthorized, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
            reply(world, entity, m.source, st);
            return;
        }
        Authority& a = s.authority;
        if (lost) {
            a.lostCommsSet = l.thresholdS > 0.0, a.lostComms = l, a.lostCommsBy = m.source, a.lostCommsSetS = world.simTime();
            a.lost = false;
        } else if (p.dissolve) {
            if (p.packageId == a.packageId) a.packageId = 0, a.leader = comm::kNoSystem, a.members.clear();
        } else {
            a.packageId = p.packageId, a.leader = p.leader;
            a.members.assign(p.members, p.members + std::min<std::uint32_t>(p.count, 16));
        }
        reply(world, entity, m.source, st);
        return;
    }
    if (m.topic == comm::topic::kTask) {
        comm::TaskOrder t;
        if (!comm::decode(m.bytes, t)) return;
        st.commandId = t.taskId;
        if (t.type == comm::TaskOrder::Strike) needs |= comm::permission::kWeapons;
        const comm::Role* role = roleOf(s, m.source);
        if (!role || (role->permissions & needs) != needs) {
            st.state = comm::CommandStatus::Unauthorized, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
            reply(world, entity, m.source, st);
            return;
        }
        // for its agent to decide on (its answer goes back when it gives it); a task sent again replaces it
        auto& tasks = s.tasks;
        tasks.erase(std::remove_if(tasks.begin(), tasks.end(), [&](const comm::ReceivedTask& r) { return r.from == m.source && r.order.taskId == t.taskId; }),
                    tasks.end());
        comm::ReceivedTask r;
        r.from = m.source, r.receivedS = world.simTime(), r.order = t;
        tasks.push_back(r);
        if (tasks.size() > 256) tasks.erase(tasks.begin());
        return;
    }
    if (m.topic == comm::topic::kOverride) {
        comm::OverrideCommand o;
        if (!comm::decode(m.bytes, o)) return;
        st.commandId = o.commandId;
        const comm::Role* role = roleOf(s, m.source);
        if (!role || !(role->permissions & needs)) {
            st.state = comm::CommandStatus::Unauthorized, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
            reply(world, entity, m.source, st);
            return;
        }
        auto& list = s.overrides;
        list.erase(std::remove_if(list.begin(), list.end(), [&](const comm::ActiveOverride& a) { return a.kind == o.kind; }), list.end());
        if (!o.cancel) list.push_back({m.source, o.kind, o.value, o.expiryS});
        overrideLive(world, entity, s);
        reply(world, entity, m.source, st);
        return;
    }
    if (m.topic == comm::topic::kConsentAnswer) {
        comm::ConsentAnswer a;
        if (!comm::decode(m.bytes, a)) return;
        st.commandId = a.requestId;
        const comm::Role* role = roleOf(s, m.source);
        const auto* request = s.consents.findRequest(a.requestId);
        // the approving roles alone, and the systems it asked
        const bool asked = request && std::find(request->asked.begin(), request->asked.end(), m.source) != request->asked.end();
        if (!role || !role->approvesStrikes || !(role->permissions & needs) || !asked) {
            st.state = comm::CommandStatus::Unauthorized, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
            reply(world, entity, m.source, st);
            return;
        }
        const ms::Reason r = world.strikeConsent(request->strike, a.granted != 0);
        st.state = r == ms::Reason::None ? comm::CommandStatus::Accepted : comm::CommandStatus::Rejected;
        st.reason = static_cast<std::uint32_t>(r), st.activity = request->strike;
        s.consents.complete(a.requestId); // (answered: later approving senders are unauthorized)
        reply(world, entity, m.source, st);
        return;
    }
    if (m.topic == comm::topic::kControlDesignation) {
        comm::ControlDesignation d;
        if (!comm::decode(m.bytes, d)) return;
        st.commandId = d.commandId;
    } else {
        comm::FlightCommand c;
        if (!comm::decode(m.bytes, c)) return;
        st.commandId = c.commandId;
    }
    const comm::Role* role = roleOf(s, m.source);
    if (!role || !(role->permissions & needs)) {
        st.state = comm::CommandStatus::Unauthorized, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
        reply(world, entity, m.source, st);
        return;
    }
    if (m.topic == comm::topic::kControlDesignation) {
        comm::ControlDesignation d;
        comm::decode(m.bytes, d);
        designate(world, entity, s, m.source, d.modes);
        reply(world, entity, m.source, st);
        return;
    }
    comm::FlightCommand c;
    comm::decode(m.bytes, c);
    fly(world, entity, s, m.source, c);
}

void MaCore::designate(World& world, std::uint32_t entity, State& s, comm::SystemId by, std::uint32_t modes) {
    // forwarded to FA (1.2.7.23; VI 1.2.2.9): the flight modes MA may command allowed, the rest revoked; control of the
    // allowed ones - and of the platform's own capabilities, which C2 does not designate - granted to MA's controller
    control::ControllerId ma = 0;
    for (const auto& a : world.agents())
        if (a.vehicle == entity) ma = a.controller;
    world.setControlMode(entity, control::ControlMode::Granted);
    for (const control::CapabilityDescriptor& d : world.capabilities(entity)) {
        if (!(d.interactions & control::kCommand) || d.kind == control::CapabilityKind::Status) continue;
        // (an A-GRA flight mode: as designated; the platform's levels, behaviours and support effectors: MA's still)
        const bool allowed = d.mode == control::FlightMode::None || (modes & (1u << static_cast<unsigned>(d.mode)));
        world.setAllowed(entity, d.id, allowed);
        if (allowed) world.requestControl(entity, d.id, ma);
    }
    s.authority.designated = true, s.authority.designatedModes = modes, s.authority.designatedBy = by;
    s.authority.designatedS = world.simTime();
}

void MaCore::fly(World& world, std::uint32_t entity, State& s, comm::SystemId by, const comm::FlightCommand& c) {
    comm::CommandStatus st;
    st.commandId = c.commandId, st.topic = comm::topic::kFlightCommand, st.timeS = world.simTime();
    const control::ControllerId controller = controllerFor(by);
    auto& list = s.authority.c2;
    if (c.kind == comm::FlightCommand::Cancel) {
        const auto it = std::find_if(list.begin(), list.end(), [&](const C2Activity& a) { return a.system == by && a.commandId == c.commandId; });
        if (it == list.end()) {
            st.state = comm::CommandStatus::Rejected, st.reason = static_cast<std::uint32_t>(control::Reason::UnknownActivity);
            reply(world, entity, by, st);
            return;
        }
        const control::CommandResult r = world.cancel(control::Caller(control::Source::Autopilot, controller), it->activity);
        st.state = r.accepted() ? comm::CommandStatus::Accepted : comm::CommandStatus::Rejected;
        st.reason = static_cast<std::uint32_t>(r.reason), st.activity = it->activity;
        reply(world, entity, by, st); // (its end follows when the step reports it)
        return;
    }
    // a mode C2 revoked from MA is not flown for C2 either
    const control::FlightMode mode = c.kind == comm::FlightCommand::Hsa ? control::FlightMode::HsaCsa : control::FlightMode::WaypointFollowing;
    if (s.authority.designated && !(s.authority.designatedModes & (1u << static_cast<unsigned>(mode)))) {
        st.state = comm::CommandStatus::Rejected, st.reason = static_cast<std::uint32_t>(control::Reason::NotAllowed);
        reply(world, entity, by, st);
        return;
    }
    // under MA's secondary control, as C2's: above the agent's own (a policy's) - authority_held on its axes for them
    control::CommandOptions o;
    o.source = control::Source::Autopilot;
    o.controller = controller;
    o.commandId = c.commandId;
    control::CommandResult r;
    if (c.kind == comm::FlightCommand::Hsa) {
        control::HsaCommand h;
        h.headingRad = c.headingRad, h.courseRad = c.courseRad, h.speed = c.speed, h.speedReference = c.speedReference;
        h.altitudeM = c.altitudeM, h.altitudeReference = c.altitudeReference;
        r = world.submit(entity, control::Command(h), o);
    } else {
        std::vector<control::Waypoint> points;
        for (std::uint32_t i = 0; i < c.pointCount && i < 16; ++i) {
            control::Waypoint w;
            w.latitudeRad = c.points[i][0], w.longitudeRad = c.points[i][1];
            w.altitudeM = c.points[i][2];
            if (!hold(c.points[i][3])) w.speed = c.points[i][3];
            points.push_back(w);
        }
        r = world.submit(entity, control::RouteCommand{}, Span<const control::Waypoint>(points.data(), points.size()), o);
    }
    st.state = r.accepted() ? comm::CommandStatus::Accepted : comm::CommandStatus::Rejected;
    st.reason = static_cast<std::uint32_t>(r.reason), st.activity = r.activity;
    if (r.accepted()) list.push_back({r.activity, by, c.commandId});
    reply(world, entity, by, st);
}

// --- a C2 system's sends ---------------------------------------------------------------------------------------------

std::uint64_t MaCore::declareRole(World& world, std::uint32_t entity, std::uint32_t roleId, const std::string& callsign,
                                  const std::vector<comm::SystemId>& destinations, comm::DmsReason& why) {
    comm::OperatorDeclaration d;
    d.roleId = roleId;
    std::memcpy(d.callsign, callsign.data(), std::min(callsign.size(), sizeof d.callsign - 1));
    comm::DmsMessage m;
    m.topic = comm::topic::kOperator, m.bytes = comm::encode(d), m.destinations = destinations;
    return world.dms().send(entity, m, world.simTime(), why, true) ? 1u : 0u;
}

std::uint64_t MaCore::designateControl(World& world, std::uint32_t entity, comm::SystemId ma, std::uint32_t modes, comm::DmsReason& why) {
    State& s = state(entity);
    comm::ControlDesignation d;
    d.commandId = s.nextCommand, d.modes = modes;
    comm::DmsMessage m;
    m.topic = comm::topic::kControlDesignation, m.bytes = comm::encode(d), m.destinations = {ma};
    if (!world.dms().send(entity, m, world.simTime(), why, true)) return 0;
    return s.nextCommand++;
}

std::uint64_t MaCore::flightCommand(World& world, std::uint32_t entity, comm::SystemId ma, comm::FlightCommand command, comm::DmsReason& why) {
    State& s = state(entity);
    if (command.kind != comm::FlightCommand::Cancel) command.commandId = s.nextCommand;
    comm::DmsMessage m;
    m.topic = comm::topic::kFlightCommand, m.bytes = comm::encode(command), m.destinations = {ma};
    if (!world.dms().send(entity, m, world.simTime(), why, true)) return 0;
    return command.kind == comm::FlightCommand::Cancel ? command.commandId : s.nextCommand++;
}

// --- AB-5b: tasks, overrides, release consent --------------------------------------------------------------------------

const std::vector<comm::ReceivedTask>* MaCore::tasks(std::uint32_t entity) const noexcept {
    const auto it = states_.find(entity);
    return it == states_.end() ? nullptr : &it->second.tasks;
}

bool MaCore::answerTask(World& world, std::uint32_t entity, comm::SystemId from, std::uint64_t taskId, comm::ReceivedTask::State state,
                        std::uint32_t code, double percent) {
    const auto it = states_.find(entity);
    if (it == states_.end()) return false;
    for (comm::ReceivedTask& t : it->second.tasks) {
        if (t.from != from || t.order.taskId != taskId) continue;
        const bool ended = t.state == comm::ReceivedTask::Rejected || t.state == comm::ReceivedTask::Completed ||
                           t.state == comm::ReceivedTask::Failed;
        // received: accepted or rejected; accepted: progress, completed or failed
        const bool fits = !ended && (t.state == comm::ReceivedTask::Received ? state == comm::ReceivedTask::Accepted || state == comm::ReceivedTask::Rejected
                                                                             : state != comm::ReceivedTask::Received && state != comm::ReceivedTask::Rejected);
        if (!fits) return false;
        t.state = state, t.percent = percent;
        comm::CommandStatus st;
        st.commandId = taskId, st.topic = comm::topic::kTask, st.reason = code, st.percent = percent, st.timeS = world.simTime();
        st.state = state == comm::ReceivedTask::Accepted    ? comm::CommandStatus::Accepted
                   : state == comm::ReceivedTask::Rejected  ? comm::CommandStatus::Rejected
                   : state == comm::ReceivedTask::Completed ? comm::CommandStatus::Completed
                                                            : comm::CommandStatus::Failed;
        reply(world, entity, from, st);
        return true;
    }
    return false;
}

const std::vector<comm::ActiveOverride>* MaCore::overrides(std::uint32_t entity) const noexcept {
    const auto it = states_.find(entity);
    return it == states_.end() ? nullptr : &it->second.overrides;
}

bool MaCore::hasOverrides(std::uint32_t entity) const noexcept {
    const auto it = states_.find(entity);
    return it != states_.end() && !it->second.overrides.empty();
}

void MaCore::applyOverrides(std::uint32_t entity, control::HsaCommand& hsa) const noexcept {
    if (const auto* list = overrides(entity))
        for (const comm::ActiveOverride& o : *list) {
            if (o.kind == comm::OverrideCommand::Speed) hsa.speed = o.value, hsa.speedReference = control::kHold; // (true airspeed)
            else hsa.altitudeM = o.value, hsa.altitudeReference = control::kHold;                             // (mean sea level)
        }
}

void MaCore::applyOverrides(std::uint32_t entity, std::vector<control::Waypoint>& points) const noexcept {
    if (const auto* list = overrides(entity))
        for (const comm::ActiveOverride& o : *list)
            for (control::Waypoint& w : points) {
                if (o.kind == comm::OverrideCommand::Speed) w.speed = o.value, w.speedReference = control::kHold;
                else w.altitudeM = o.value, w.altitudeReference = control::kHold, w.altitudeMinM = w.altitudeMaxM = control::kHold;
            }
}

void MaCore::overrideLive(World& world, std::uint32_t entity, const State& s) {
    // the agent's live HSA flies the overrides now (an UPDATE as its own: the fields left out kept)
    control::ControllerId ma = 0;
    for (const auto& a : world.agents())
        if (a.vehicle == entity) ma = a.controller;
    if (ma == 0) return;
    for (const control::ActivityRecord& r : world.activities(entity)) {
        if (!r.live() || r.controller != ma || r.source != control::Source::Policy) continue;
        control::Setpoint setpoint;
        if (!world.activitySetpoint(r.id, setpoint)) continue;
        const auto* command = std::get_if<control::Command>(&setpoint.command);
        if (!command || !std::holds_alternative<control::HsaCommand>(*command)) continue;
        control::HsaCommand h;
        for (const comm::ActiveOverride& o : s.overrides) {
            if (o.kind == comm::OverrideCommand::Speed) h.speed = o.value;
            else h.altitudeM = o.value;
        }
        world.update(control::Caller(control::Source::Policy, ma), r.id, control::Command(h));
    }
}

void MaCore::consents(World& world, std::uint32_t entity, State& s) {
    const ms::Host* host = world.missionSystems(entity);
    if (!host) return;
    std::vector<control::ActivityId> waiting; // (none, nearly always: nothing to do)
    host->awaitingConsent(waiting);
    for (const control::ActivityId id : waiting) {
        const auto strike = world.strikeStatus(id);
        if (!strike) continue;
        if (s.consents.findStrike(id)) continue; // (asked: waiting for an answer)
        if (s.rbac.strikePolicy != comm::StrikePolicy::RequiresApproval) { // (MA answers itself, as its policy says)
            world.strikeConsent(id, s.rbac.strikePolicy == comm::StrikePolicy::AutoApprove);
            continue;
        }
        // the systems whose declared role approves strikes - none yet: asked again when one declares
        std::vector<comm::SystemId> approvers;
        for (const Operator& o : s.authority.operators)
            for (const comm::Role& r : s.rbac.roles)
                if (r.id == o.roleId && r.approvesStrikes && (r.permissions & comm::permission::kWeapons)) approvers.push_back(o.system);
        if (approvers.empty()) continue;
        comm::ConsentRequest q;
        q.requestId = s.nextRequest++, q.strike = id, q.timeS = world.simTime();
        q.quantity = static_cast<std::uint32_t>(strike->stores.size());
        if (!strike->stores.empty())
            for (const ms::StoreInfo& st : world.stores(entity))
                if (st.index == strike->stores.front()) std::memcpy(q.store, st.store, std::min(std::strlen(st.store), sizeof q.store - 1));
        if (const auto* d = world.designation(strike->target)) { // (its target by track: its own radar's, or a network track's)
            q.targetSource = d->source != comm::kNoSystem ? d->source : world.dms().systemOf(entity);
            q.targetPayload = d->payload, q.targetTrack = d->track;
        }
        if (const effects::SensedState* sensed = world.sensedState(entity)) {
            q.latitudeRad = sensed->state.latitudeRad, q.longitudeRad = sensed->state.longitudeRad, q.altitudeM = sensed->state.altitudeMslM;
        }
        comm::DmsMessage m;
        m.topic = comm::topic::kConsentRequest, m.bytes = comm::encode(q), m.destinations = approvers;
        comm::DmsReason why;
        if (!world.dms().send(entity, m, world.simTime(), why, true)) continue;
        s.consents.record({id, q.requestId, approvers});
    }
}

std::uint64_t MaCore::sendTask(World& world, std::uint32_t entity, comm::SystemId ma, comm::TaskOrder task, comm::DmsReason& why) {
    State& s = state(entity);
    task.taskId = s.nextCommand;
    comm::DmsMessage m;
    m.topic = comm::topic::kTask, m.bytes = comm::encode(task), m.destinations = {ma};
    if (!world.dms().send(entity, m, world.simTime(), why, true)) return 0;
    return s.nextCommand++;
}

std::uint64_t MaCore::sendOverride(World& world, std::uint32_t entity, comm::SystemId ma, comm::OverrideCommand command, comm::DmsReason& why) {
    State& s = state(entity);
    command.commandId = s.nextCommand;
    comm::DmsMessage m;
    m.topic = comm::topic::kOverride, m.bytes = comm::encode(command), m.destinations = {ma};
    if (!world.dms().send(entity, m, world.simTime(), why, true)) return 0;
    return s.nextCommand++;
}

bool MaCore::answerConsent(World& world, std::uint32_t entity, comm::SystemId ma, std::uint64_t requestId, bool granted, comm::DmsReason& why) {
    auto* list = world.dms().consentsOf(entity);
    if (!list) return false;
    for (comm::PendingConsent& p : *list) {
        if (p.from != ma || p.request.requestId != requestId || p.answered) continue;
        comm::ConsentAnswer a;
        a.requestId = requestId, a.granted = granted ? 1u : 0u;
        comm::DmsMessage m;
        m.topic = comm::topic::kConsentAnswer, m.bytes = comm::encode(a), m.destinations = {ma};
        if (!world.dms().send(entity, m, world.simTime(), why, true)) return false;
        p.answered = true;
        return true;
    }
    return false;
}

// --- AB-5c: lost communication, teams --------------------------------------------------------------------------------------

void MaCore::watchComms(World& world, std::uint32_t entity, State& s) {
    // what it last heard from C2: any message from a system that declared a role to it (its heartbeats among them)
    Authority& a = s.authority;
    double heard = a.lostCommsSetS;
    for (const Operator& o : a.operators) {
        const double at = world.dms().lastHeard(entity, o.system);
        if (at == at && at > heard) heard = at;
    }
    const double now = world.simTime();
    if (now - heard <= a.lostComms.thresholdS) {
        a.lost = false; // (heard again: armed for the next loss)
        return;
    }
    if (a.lost) return;
    // the contingency (MissionContingencyAlertMT, raised though C2 cannot hear it), and its response
    a.lost = true, a.lostSinceS = heard, ++a.alerts;
    if (!a.lostComms.activatePlan) return;
    control::ControllerId ma = 0;
    for (const auto& ag : world.agents())
        if (ag.vehicle == entity) ma = ag.controller;
    control::CommandOptions o;
    o.controller = ma; // (MA's own response: its plan, as its agent would fly it)
    world.planCommand(entity, a.lostComms.planId, control::PlanCommand::PrepareForActivation, o);
    world.planCommand(entity, a.lostComms.planId, control::PlanCommand::Activate, o);
}

std::uint64_t MaCore::sendLostComms(World& world, std::uint32_t entity, comm::SystemId ma, comm::LostCommsSettings settings, comm::DmsReason& why) {
    State& s = state(entity);
    settings.commandId = s.nextCommand;
    comm::DmsMessage m;
    m.topic = comm::topic::kLostComms, m.bytes = comm::encode(settings), m.destinations = {ma};
    if (!world.dms().send(entity, m, world.simTime(), why, true)) return 0;
    return s.nextCommand++;
}

std::uint64_t MaCore::sendPackage(World& world, std::uint32_t entity, comm::PackageCommand command, const std::vector<comm::SystemId>& to,
                                  comm::DmsReason& why) {
    State& s = state(entity);
    command.commandId = s.nextCommand;
    comm::DmsMessage m;
    m.topic = comm::topic::kPackage, m.bytes = comm::encode(command), m.destinations = to;
    if (!world.dms().send(entity, m, world.simTime(), why, true)) return 0;
    return s.nextCommand++;
}

std::uint64_t MaCore::formPackage(World& world, std::uint32_t entity, std::uint32_t packageId, comm::SystemId leader,
                                  const std::vector<comm::SystemId>& members, comm::DmsReason& why) {
    comm::PackageCommand p;
    p.packageId = packageId, p.leader = leader;
    p.count = static_cast<std::uint32_t>(std::min<std::size_t>(members.size(), 16));
    for (std::uint32_t i = 0; i < p.count; ++i) p.members[i] = members[i];
    std::vector<comm::SystemId> to = members;
    if (std::find(to.begin(), to.end(), leader) == to.end()) to.push_back(leader);
    const comm::SystemId self = world.dms().systemOf(entity);
    to.erase(std::remove(to.begin(), to.end(), self), to.end()); // (not to itself)
    return sendPackage(world, entity, p, to, why);
}

std::uint64_t MaCore::dissolvePackage(World& world, std::uint32_t entity, std::uint32_t packageId, const std::vector<comm::SystemId>& systems,
                                      comm::DmsReason& why) {
    comm::PackageCommand p;
    p.packageId = packageId, p.dissolve = 1;
    std::vector<comm::SystemId> to = systems;
    const comm::SystemId self = world.dms().systemOf(entity);
    to.erase(std::remove(to.begin(), to.end(), self), to.end());
    return sendPackage(world, entity, p, to, why);
}

} // namespace fsim::session
