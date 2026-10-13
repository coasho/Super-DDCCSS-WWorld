// ADR-31: the rules an agent's handle is held to (AgentRules.h).

#include "session/AgentRules.h"

#include "session/World.h"

#include <mutex>
#include <variant>

namespace fsim::session {

namespace {

bool ownController(const AgentIdentity& agent, control::ControllerId& controller) noexcept {
    if (controller != 0 && controller != agent.controller) return false;
    controller = agent.controller;
    return true;
}

/// An id naming another aircraft than its own - not its own vehicle, and not a target its own vehicle designated.
bool namesOther(const AgentIdentity& agent, double id, const World* world) noexcept {
    if (control::isHold(id) || id == 0.0 || id == static_cast<double>(agent.vehicle)) return false;
    if (world && id >= static_cast<double>(World::kDesignations) && id < 4294967296.0) {
        const World::Designation* d = world->designation(static_cast<std::uint32_t>(id));
        if (d && d->owner == agent.vehicle) return false;
    }
    return true;
}

} // namespace

control::Reason agentOptions(const AgentIdentity& agent, control::CommandOptions& options) noexcept {
    if (options.source != control::Source::Policy || !ownController(agent, options.controller)) return control::Reason::NotAllowed;
    return control::Reason::None;
}

ms::Reason agentOptions(const AgentIdentity& agent, ms::CommandOptions& options) noexcept {
    if (options.source != control::Source::Policy || !ownController(agent, options.controller)) return ms::Reason::NotAllowed;
    return ms::Reason::None;
}

control::Reason agentCaller(const AgentIdentity& agent, control::Caller& caller) noexcept {
    if (caller.source != control::Source::Policy || !ownController(agent, caller.controller)) return control::Reason::NotAllowed;
    return control::Reason::None;
}

bool agentCaller(const AgentIdentity& agent, int& source, std::uint32_t& controller) noexcept {
    if (source != static_cast<int>(control::Source::Policy)) return false;
    control::ControllerId c = controller;
    if (!ownController(agent, c)) return false;
    controller = c;
    return true;
}

bool agentActivity(const AgentIdentity& agent, std::uint64_t activity, const World* world) noexcept {
    // its own vehicle's - but not a direct C2 command's, which its MA core flies for C2 (AB-5: section 9, item 5)
    return control::activityVehicle(activity) == agent.vehicle && !(world && world->maCore().c2Of(activity) != comm::kNoSystem);
}

control::Reason agentMustFly(const AgentIdentity& agent, const control::MustFlyCommand& mustFly, const World* world) noexcept {
    const bool entity = !control::isHold(mustFly.location) &&
                        static_cast<int>(mustFly.location) == static_cast<int>(control::MustFlyLocation::Entity);
    return entity && namesOther(agent, mustFly.target, world) ? control::Reason::NotAllowed : control::Reason::None;
}

control::Reason agentCommand(const AgentIdentity& agent, const control::Command& command, const World* world) noexcept {
    if (const auto* b = std::get_if<control::BehaviorCommand>(&command))
        return namesOther(agent, static_cast<double>(b->target), world) ? control::Reason::NotAllowed : control::Reason::None;
    if (const auto* m = std::get_if<control::MustFlyCommand>(&command)) return agentMustFly(agent, *m, world);
    return control::Reason::None;
}

ms::Reason agentStrike(const AgentIdentity& agent, const ms::StrikeCommand& strike, const World* world) noexcept {
    if (strike.consent) return ms::Reason::NotAllowed;
    return namesOther(agent, static_cast<double>(strike.target), world) ? ms::Reason::NotAllowed : ms::Reason::None;
}

bool agentDmsIngress(const AgentIdentity& agent, const World& world) noexcept {
    const MaServiceBindings& services = world.maServices();
    return !services.bound(agent.vehicle) || services.ingressOwner(agent.vehicle) == agent.controller;
}
bool agentDmsEgress(const AgentIdentity& agent, const World& world) noexcept {
    const MaServiceBindings& services = world.maServices();
    return !services.bound(agent.vehicle) || services.egressOwner(agent.vehicle) == agent.controller;
}

void agentReport(ms::AmtiDetection& detection) noexcept { detection.truth = 0; }
void agentReport(ms::AmtiTrack& track) noexcept { track.truth = 0; }
void agentReport(ms::Contact& contact) noexcept { contact.truth = 0; }

OwnShip ownShipOf(const effects::SensedState& sensed) noexcept {
    const sim::VehicleState& s = sensed.state;
    OwnShip o;
    o.timeS = s.simTime;
    o.latitudeRad = s.latitudeRad, o.longitudeRad = s.longitudeRad, o.altitudeMslM = s.altitudeMslM, o.altitudeAglM = s.altitudeAglM;
    for (int i = 0; i < 3; ++i) {
        o.velocityNedMs[i] = s.velocityNedMs[i], o.eulerRad[i] = s.eulerRad[i];
        o.angularRateBodyRadS[i] = s.angularRateBodyRadS[i], o.accelerationBodyMs2[i] = s.accelerationBodyMs2[i];
    }
    o.airspeedTrueMs = s.airspeedTrueMs, o.airspeedCalibratedMs = s.airspeedCalibratedMs, o.mach = s.mach;
    o.alphaRad = s.alphaRad, o.betaRad = s.betaRad, o.loadFactor = s.loadFactor;
    o.fuelKg = s.fuelKg, o.onGround = s.onGround;
    o.gnssValid = sensed.gnssValid, o.airDataValid = sensed.airDataValid;
    o.throttle = s.engineCount > 0 ? s.throttlePosition[0] : 0.0;
    return o;
}

namespace {

std::mutex& auditMutex() {
    static std::mutex m;
    return m;
}

std::vector<std::string>& auditList() {
    static std::vector<std::string> names;
    return names;
}

} // namespace

std::uint16_t auditCall(const char* name) {
    const std::lock_guard<std::mutex> lock(auditMutex());
    auto& names = auditList();
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i] == name) return static_cast<std::uint16_t>(i);
    names.emplace_back(name);
    return static_cast<std::uint16_t>(names.size() - 1);
}

std::vector<std::string> auditNames() {
    const std::lock_guard<std::mutex> lock(auditMutex());
    return auditList();
}

std::string notAgents(const char* call) {
    return std::string(call) + ": not an agent's call - an agent's handle answers its own vehicle's calls alone, as itself "
                               "(ADR-31; docs/sdk/boundary.md)";
}

} // namespace fsim::session
