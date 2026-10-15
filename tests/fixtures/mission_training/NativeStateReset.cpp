#include "NativeStateReset.h"

#include "fsim/MAService.h"
#include "session/World.h"
#include "ms/damage/State.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace fsim::test::native_reset {
namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr comm::Topic kMarkerTopic = comm::topic::kFreeForm + 73;
const std::vector<std::uint8_t> kMarker{0x4d, 0x34, 0, 0xff, 0x52, 0x45, 0x53, 0x45, 0x54};

void require(bool condition, const std::string& message) {
    if (!condition) throw Error("native reset fixture: " + message);
}

template <typename Predicate>
void until(World& world, double seconds, Predicate predicate, const char* message) {
    const double deadline = world.time() + seconds;
    while (!predicate() && world.time() < deadline) world.step();
    require(predicate(), message);
}

std::uint16_t payload(const Vehicle& vehicle, ms::Family family) {
    for (const auto& info : vehicle.payloads()) if (info.family == family) return info.index;
    throw Error("native reset fixture: required stock payload missing");
}

void operate(Agent& agent, std::uint16_t index) {
    ms::StateCommand command;
    command.payload = index;
    command.state = ms::PayloadState::Operate;
    require(agent.submit(command).accepted(), "payload activation refused");
}

bool activeFire(World& world, std::uint32_t id) {
    const auto* damage = world.impl().damage(id);
    auto* model = world.impl().model(id);
    const auto* state = world.impl().vehicleState(id);
    if (!damage || !model || !state || !std::isfinite(state->loadFactor)) return false;
    double pressure = std::numeric_limits<double>::quiet_NaN();
    for (const auto& value : model->properties("aero"))
        if (value.first == "qbar-psf") pressure = value.second * 47.880258888889;
    if (!std::isfinite(pressure) || pressure < 0.0) return false;
    for (const auto& entry : damage->tanks()) {
        const auto& tank = entry.second;
        if (entry.first < 0 || !tank.burning || tank.holeM2 <= 0.0) continue;
        double fuel = 0.0;
        for (const auto& value : model->properties("propulsion/tank[" + std::to_string(entry.first) + "]"))
            if (value.first == "contents-lbs") fuel = value.second * 0.45359237;
        const double leak = damage->leakKgS(entry.first, fuel, state->loadFactor, pressure);
        if (std::isfinite(fuel) && fuel > 0.0 && std::isfinite(leak) && leak > 0.0) return true;
    }
    return false;
}

std::vector<ipc::EngagementEvent> externalBurst(World& world, std::uint32_t victim, const std::array<double, 3>& offset) {
    // External Range setup uses the existing card/fragment/native damage path. Never writes a damage flag.
    const auto state = world.vehicle(victim).state(); // own copy: debris can append/reallocate vehicle state
    session::Detonation burst;
    burst.store = "aim-120c";
    for (int row = 0; row < 3; ++row) {
        burst.positionEcef[row] = state.positionEcef[row];
        for (int column = 0; column < 3; ++column) {
            const double rotation = state.rotationBodyToEcef[3 * row + column];
            burst.positionEcef[row] += rotation * offset[static_cast<std::size_t>(column)];
            burst.velocityEcef[row] += rotation * (state.velocityBodyMs[column] + (column == 0 ? 600.0 : 0.0));
        }
        burst.axisEcef[row] = burst.velocityEcef[row];
    }
    auto events = world.impl().detonate(burst);
    require(!events.empty(), "external native burst produced no events");
    return events;
}

} // namespace

WorldOptions options(const std::string& name, unsigned workers) {
    WorldOptions result;
    result.name = name;
    result.publish = false;
    result.pinWorkers = false;
    result.workers = workers;
    result.seed = 17;
    result.dt = 1.0 / 120.0;
    result.frameSkip = 4;
    return result;
}

Scenario configure(World& world) {
    require(world.time() == 0.0 && world.vehicleCount() == 0 && world.c2Nodes().empty(), "recipe needs an empty fresh World");
    EnvironmentState environment;
    environment.epochUtcSeconds = 1767225600.0; // 2026-01-01 00:00:00 UTC; never wall-clock dependent
    world.environment().set(environment);
    auto aircraft = [&](const char* name, double latitude) {
        VehicleSpec spec;
        spec.name = name;
        spec.type = "jsbsim:f16c";
        spec.initial.latitudeDeg = latitude;
        spec.initial.longitudeDeg = 0.0;
        spec.initial.altitudeMslM = 6000.0;
        spec.initial.airspeedTrueMs = 230.0;
        spec.initial.headingDeg = 0.0;
        return world.createVehicle(spec);
    };
    Scenario scenario;
    auto launcher = aircraft("fixture-launcher", 40.0);
    scenario.launcher = launcher.id();
    scenario.target = aircraft("fixture-target", 40.2).id();
    scenario.victim = aircraft("fixture-damage-victim", 40.5).id();
    scenario.removed = aircraft("fixture-removed", 40.8).id();
    scenario.stations = payload(launcher, ms::Family::StoreStation);
    scenario.countermeasures = payload(launcher, ms::Family::Countermeasures);
    require(launcher.loadStore(scenario.stations, 1, "aim-120c") == ms::Reason::None, "first missile load refused");
    require(launcher.loadStore(scenario.stations, 2, "aim-120c") == ms::Reason::None, "second missile load refused");
    comm::C2NodeSpec c2;
    c2.name = "fixture-c2";
    c2.latitudeRad = 39.9 * kDeg;
    c2.longitudeRad = 0.0;
    c2.altitudeMslM = 200.0;
    scenario.c2 = world.createC2Node(c2);
    comm::NetSpec net;
    net.name = "fixture-control";
    net.latencyS = 0.05;
    net.jitterS = 0.0;
    net.lossProbability = 0.0;
    scenario.net = world.createNet(net);
    require(world.joinNet(scenario.launcher, scenario.net) && world.joinNet(scenario.c2, scenario.net), "net join refused");
    comm::DisseminationPlan plan;
    plan.positionPeriodS = plan.statusPeriodS = plan.tracksPeriodS = 0.0;
    require(world.setDissemination(scenario.launcher, plan) && world.setDissemination(scenario.c2, plan), "dissemination setup refused");
    return scenario;
}

DirtyState contaminate(World& world, const Scenario& scenario) {
    DirtyState dirty;
    auto launcher = world.createAgent(scenario.launcher, "fixture-launcher-service");
    auto c2 = world.createAgent(scenario.c2, "fixture-c2-service");
    require(bindMAService(world, launcher, 4101, {"*"}, true, true), "launcher binding refused");
    require(bindMAService(world, c2, 4102, {}, true, true), "C2 binding refused");
    dirty.launcherSystem = launcher.system();
    dirty.c2System = c2.system();
    require(c2.declareRole(2, "FIXTURE-C2", {launcher.system()}), "role declaration refused");
    until(world, 2.0, [&] { return !launcher.authority().operators.empty(); }, "role delivery timed out");
    operate(launcher, scenario.stations);
    operate(launcher, scenario.countermeasures);
    until(world, 20.0, [&] {
        unsigned ready = 0;
        for (const auto& item : launcher.payloads())
            if ((item.index == scenario.stations || item.index == scenario.countermeasures) && item.state == ms::PayloadState::Operate) ++ready;
        return ready == 2;
    }, "payload startup timed out");

    ms::StrikeCommand strike;
    strike.payload = scenario.stations;
    strike.store = "aim-120c";
    strike.quantity = 1;
    const auto released = launcher.submit(strike);
    require(released.accepted(), "first actor strike refused");
    dirty.releasedActivity = released.activity;
    until(world, 20.0, [&] { return !c2.reportConsents().items.empty(); }, "first consent delivery timed out");
    const auto approval = c2.reportConsents().items.front().value;
    require(approval.request.strike == released.activity && !approval.answered, "first consent correlation mismatch");
    require(c2.answerConsent(approval.from, approval.request.requestId, true), "C2 consent answer refused");
    until(world, 5.0, [&] {
        const auto status = launcher.strikeStatus(released.activity);
        return status && status->phase == ms::StrikePhase::Done && status->weapons.size() == 1;
    }, "approved native release timed out");
    const auto releasedStatus = *launcher.strikeStatus(released.activity);
    dirty.liveWeapon = releasedStatus.weapons.front();
    dirty.consumedStore = releasedStatus.stores.front();

    const auto waiting = launcher.submit(strike);
    require(waiting.accepted(), "second actor strike refused");
    dirty.waitingActivity = waiting.activity;
    until(world, 20.0, [&] {
        for (const auto& item : c2.reportConsents().items)
            if (item.value.request.strike == waiting.activity && !item.value.answered) return true;
        return false;
    }, "second unanswered consent delivery timed out");
    for (const auto& item : c2.reportConsents().items)
        if (item.value.request.strike == waiting.activity) dirty.pendingConsent = item.value.request.requestId;
    comm::TaskOrder task;
    task.type = comm::TaskOrder::Patrol;
    task.priority = 7;
    task.latitudeRad = 40.1 * kDeg;
    task.longitudeRad = 0.0;
    task.altitudeM = 6000.0;
    task.radiusM = 2000.0;
    std::strcpy(task.text, "native reset fixture pending task");
    dirty.pendingTask = c2.sendTask(launcher.system(), task);
    require(dirty.pendingTask != 0, "typed task send refused");
    until(world, 3.0, [&] { return !launcher.c2Tasks().empty(); }, "typed task delivery timed out");

    // Fixed external burst recipe; native outcomes are asserted, never synthesized.
    for (const auto& offset : {std::array<double, 3>{-1.0, -4.0, 0.5}, std::array<double, 3>{3.7, -1.0, -0.55}})
        for (const auto& event : externalBurst(world, scenario.victim, offset)) {
            require(event.slot + 1 == scenario.victim || event.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Detonation),
                    "damage fixture burst reached an unrelated entity");
            if (event.kind == static_cast<std::uint8_t>(ipc::EngagementKind::CrewLost)) ++dirty.crewLossEvents;
            if (event.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Fire)) ++dirty.fireEvents;
        }
    world.step(3); // let native crew-control and fuel leak/fire consequences actually advance
    const auto* damage = world.impl().damage(scenario.victim);
    dirty.crewLost = damage && damage->crewLost();
    dirty.activeFire = activeFire(world, scenario.victim);

    require(world.vehicle(scenario.removed).remove(), "entity removal refused");
    dirty.removedEntity = scenario.removed;
    require(launcher.dispense(false) == ms::Reason::None, "native chaff release refused");
    require(world.decoys().size() == 1, "expected one native decoy");
    dirty.liveDecoy = world.decoys().front().id;
    comm::DmsMessage marker;
    marker.topic = kMarkerTopic;
    marker.bytes = kMarker;
    marker.destinations = {launcher.system()};
    dirty.queuedMessage = c2.send(marker);
    require(dirty.queuedMessage != 0, "queued marker send refused");
    // No step after enqueue: the live decoy and pending marker coexist with the other dirty state.
    requireDirty(world, scenario, dirty);
    return dirty;
}

void requireDirty(World& world, const Scenario& scenario, const DirtyState& dirty) {
    const auto* damage = world.impl().damage(scenario.victim);
    require(dirty.crewLost && dirty.crewLossEvents > 0 && damage && damage->crewLost(), "crew loss not present");
    require(dirty.activeFire && dirty.fireEvents > 0 && activeFire(world, scenario.victim), "active fuel fire not present");
    const auto weapons = world.weapons();
    const auto weapon = std::find_if(weapons.begin(), weapons.end(), [&](const auto& item) { return item.id == dirty.liveWeapon; });
    require(weapon != weapons.end() && weapon->end == ms::WeaponEnd::InFlight && weapon->launcher == scenario.launcher &&
            weapon->activity == dirty.releasedActivity && weapon->store == "aim-120c" && !weapon->jettisoned &&
            weapon->seeker == "active_radar" && weapon->phase != ms::WeaponPhase::Unguided,
            "guided in-flight release correlation missing");
    const auto stores = world.vehicle(scenario.launcher).stores();
    require(dirty.consumedStore < stores.size() && stores[dirty.consumedStore].status == ms::StoreStatus::Released &&
            stores[dirty.consumedStore].weapon == dirty.liveWeapon, "consumed store correlation missing");
    const auto decoys = world.decoys();
    require(decoys.size() == 1 && decoys[0].id == dirty.liveDecoy && decoys[0].owner == scenario.launcher &&
            !decoys[0].flare && decoys[0].ageS == 0.0 && world.countermeasures(scenario.launcher).chaff == 29,
            "live native decoy/inventory missing");
    const auto* tasks = world.impl().maCore().tasks(scenario.launcher);
    require(tasks && tasks->size() == 1 && tasks->front().order.taskId == dirty.pendingTask &&
            tasks->front().from == dirty.c2System && tasks->front().state == comm::ReceivedTask::Received &&
            tasks->front().order.priority == 7 && std::string(tasks->front().order.text) == "native reset fixture pending task",
            "delivered pending typed task missing");
    const auto waiting = world.vehicle(scenario.launcher).strikeStatus(dirty.waitingActivity);
    require(waiting && waiting->phase == ms::StrikePhase::AwaitingConsent && waiting->weapons.empty(), "pending native strike missing");
    bool pending = false;
    if (const auto* consents = world.impl().dms().consentsOf(scenario.c2))
        for (const auto& item : *consents)
            pending = pending || (!item.answered && item.from == dirty.launcherSystem &&
                      item.request.requestId == dirty.pendingConsent && item.request.strike == dirty.waitingActivity);
    require(pending, "delivered unanswered consent missing");
    const auto transmission = world.impl().dms().transmission(scenario.c2, dirty.queuedMessage);
    require(transmission && transmission->topic == kMarkerTopic && transmission->destinations.size() == 1 &&
            transmission->destinations[0].first == dirty.launcherSystem && transmission->destinations[0].second == comm::Delivery::Pending,
            "reliable marker is not queued");
    if (const auto* inbox = world.impl().dms().inbox(scenario.launcher))
        for (const auto& item : *inbox)
            require(item.source != dirty.c2System || item.message != dirty.queuedMessage, "queued marker arrived before stepping");
    require(!world.vehicle(scenario.removed).valid() && !world.vehicle("fixture-removed").valid(), "removed entity survived");
}

void requireDeliveredMarker(World& world, const Scenario& scenario, const DirtyState& dirty) {
    const auto transmission = world.impl().dms().transmission(scenario.c2, dirty.queuedMessage);
    require(transmission && transmission->destinations.size() == 1 &&
            transmission->destinations[0] == std::make_pair(dirty.launcherSystem, comm::Delivery::Delivered),
            "marker destination has not completed native delivery");
    unsigned matches = 0;
    if (const auto* inbox = world.impl().dms().inbox(scenario.launcher))
        for (const auto& item : *inbox) if (item.source == dirty.c2System && item.message == dirty.queuedMessage) {
            require(item.topic == kMarkerTopic && item.bytes == kMarker && item.net == scenario.net &&
                    item.timeDelivered > item.timeSent, "delivered marker content/provenance mismatch");
            ++matches;
        }
    require(matches == 1, "queued marker must arrive exactly once");
}

} // namespace fsim::test::native_reset
