// End-to-end native engagement acceptance. Every weapon flies through session::World and JSBSim.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT "D:/Projects/Cpp/Super-DDCCSS-WWorld/third_party/jsbsim"
#endif

using namespace fsim;
namespace {
constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180.0;
void require(bool value, const std::string& message) { if (!value) throw std::runtime_error(message); }

session::WorldOptions options(const std::string& name, unsigned workers = 1) {
    session::WorldOptions o;
    o.name = name; o.workers = workers; o.pinWorkers = false; o.publish = false; o.seed = 17;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    return o;
}

void advance(session::World& w, double seconds) {
    w.step(static_cast<unsigned>(std::lround(seconds / (w.dt() * w.frameSkip()))));
}

std::uint32_t aircraft(session::World& w, const std::string& name, double eastM, double headingDeg) {
    session::VehicleSpec s;
    s.name = name; s.type = "jsbsim:f16c";
    s.initial.altitudeMslM = 6000; s.initial.airspeedTrueMs = 250; s.initial.headingDeg = headingDeg;
    s.initial.longitudeDeg += eastM / (6371000.0 * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
    const auto id = w.createVehicle(s);
    require(id != 0, "F-16C creation failed");
    control::VelocityCommand hold;
    hold.airspeedMs = 250; hold.verticalSpeedMs = 0; hold.headingRad = headingDeg * kDeg;
    require(w.submit(id, hold).accepted(), "stable flight hold rejected");
    return id;
}

std::uint16_t payload(const session::World& w, std::uint32_t id, ms::Family family) {
    const auto* host = w.missionSystems(id);
    require(host != nullptr, "mission systems missing");
    for (const auto& p : host->payloads()) if (p.family == family) return p.index;
    throw std::runtime_error("required payload missing");
}

void operate(session::World& w, std::uint32_t id, std::uint16_t index) {
    const auto result = w.submit(id, ms::StateCommand{index, ms::PayloadState::Operate});
    require(result.accepted() || result.reason == ms::Reason::AlreadyInState, "payload operation rejected");
    for (unsigned i = 0; i < 1200 && w.missionSystems(id)->payloads()[index].state != ms::PayloadState::Operate; ++i) w.step();
    require(w.missionSystems(id)->payloads()[index].state == ms::PayloadState::Operate, "payload warmup did not finish");
}

ms::WeaponRecord weapon(const session::World& w, std::uint32_t id) {
    for (const auto& record : w.weapons()) if (record.id == id) return record;
    throw std::runtime_error("weapon record missing");
}

bool terminal(const ms::EngagementEvent& event) {
    return event.kind == "detonation" || event.kind == "expiry" || event.kind == "impact" || event.kind == "removal";
}

bool sameEvent(const ms::EngagementEvent& a, const ms::EngagementEvent& b) {
    return a.serial == b.serial && a.timeS == b.timeS && a.kind == b.kind && a.reason == b.reason &&
           a.entity == b.entity && a.launcher == b.launcher && a.target == b.target && a.destroyed == b.destroyed &&
           std::equal(a.positionEcef, a.positionEcef + 3, b.positionEcef) &&
           std::equal(a.velocityEcef, a.velocityEcef + 3, b.velocityEcef);
}

void terminalPoseMatchesBurst(const ms::WeaponRecord& record, const ms::EngagementEvent& event) {
    // Independently reconstruct WGS-84 ECEF from the record's public terminal coordinates.
    // Comparing metre coordinates catches recording the step endpoint instead of the swept burst pose.
    constexpr double equatorialRadiusM = 6378137.0;
    constexpr double eccentricitySquared = 6.6943799901413165e-3;
    const double latitude = record.endLatitudeRad, longitude = record.endLongitudeRad;
    require(std::isfinite(latitude) && std::isfinite(longitude) && std::isfinite(record.endHeightM),
            "detonated record must expose finite terminal coordinates");
    const double sinLatitude = std::sin(latitude), cosLatitude = std::cos(latitude);
    const double primeVertical = equatorialRadiusM / std::sqrt(1 - eccentricitySquared * sinLatitude * sinLatitude);
    const double x = (primeVertical + record.endHeightM) * cosLatitude * std::cos(longitude);
    const double y = (primeVertical + record.endHeightM) * cosLatitude * std::sin(longitude);
    const double z = (primeVertical * (1 - eccentricitySquared) + record.endHeightM) * sinLatitude;
    const double errorM = std::hypot(x - event.positionEcef[0], y - event.positionEcef[1], z - event.positionEcef[2]);
    require(errorM < 0.001, "terminal record must match interpolated detonation event pose within one millimetre; error_m=" +
                          std::to_string(errorM));
    require(record.endTimeS == event.timeS, "terminal record and detonation event must share simulation timestamp");
}

void replayableEvents(session::World& w, std::uint32_t missile) {
    const auto events = w.engagementEvents();
    const auto again = w.engagementEvents();
    require(events.size() == again.size(), "event reads must not consume events");
    unsigned terminals = 0;
    for (std::size_t i = 0; i < events.size(); ++i) {
        require(sameEvent(events[i], again[i]), "event replay changed content");
        require(i == 0 || events[i].serial > events[i - 1].serial, "event serial must increase monotonically");
        if (events[i].entity == missile && terminal(events[i])) ++terminals;
    }
    require(terminals == 1, "armed weapon must produce exactly one terminal event");
    const auto cursor = events.back().serial;
    require(w.engagementEvents(cursor).empty(), "cursor must exclude the event at its serial");
    const auto tail = w.engagementEvents(events.front().serial);
    require(tail.size() + 1 == events.size(), "cursor must retain every strictly newer event");
    for (std::size_t i = 0; i < tail.size(); ++i)
        require(sameEvent(tail[i], events[i + 1]), "cursor replay must preserve exact event content");
    advance(w, 1);
    unsigned after = 0;
    for (const auto& event : w.engagementEvents()) if (event.entity == missile && terminal(event)) ++after;
    require(after == 1, "later steps must not duplicate weapon terminal event");
}

struct Outcome { ms::WeaponRecord record; std::vector<ms::EngagementEvent> events; bool burned = false, coasted = false; };

Outcome engagement(double rangeM, unsigned workers, bool requireHit) {
    session::World w(options("native-engagement-" + std::to_string(workers) + "-" + std::to_string(rangeM), workers));
    const auto own = aircraft(w, "launcher", 0, 90);
    const auto stations = payload(w, own, ms::Family::StoreStation);
    const auto radar = payload(w, own, ms::Family::AirRadar);
    require(w.loadStore(own, stations, 1, "aim-120c") == ms::Reason::None, "missile load rejected");
    operate(w, own, stations); operate(w, own, radar);
    advance(w, 10);
    const auto* ownState = w.vehicleState(own);
    require(ownState && !ownState->crashed && !ownState->diverged && std::fabs(ownState->altitudeMslM - 6000) < 100,
            "launcher failed stable 6000-metre setup");
    session::VehicleSpec targetSpec;
    targetSpec.name = "opponent"; targetSpec.type = "jsbsim:f16c";
    targetSpec.initial.latitudeDeg = ownState->latitudeRad / kDeg;
    targetSpec.initial.longitudeDeg = ownState->longitudeRad / kDeg + rangeM / (6371000 * std::cos(ownState->latitudeRad)) / kDeg;
    targetSpec.initial.altitudeMslM = 6000; targetSpec.initial.airspeedTrueMs = 250; targetSpec.initial.headingDeg = 270;
    const auto target = w.createVehicle(targetSpec);
    require(target != 0, "opponent creation failed");
    control::VelocityCommand hold;
    hold.airspeedMs = 250; hold.verticalSpeedMs = 0; hold.headingRad = 270 * kDeg;
    require(w.submit(target, hold).accepted(), "opponent flight hold rejected");
    ms::AmtiCommand scan;
    scan.payload = radar; scan.azimuthHalfWidthRad = 5 * kDeg; scan.bars = 1; scan.rangeScaleM = 60000;
    require(w.submit(own, scan).accepted(), "native AMTI search rejected");
    std::uint64_t track = 0;
    for (unsigned i = 0; i < 150 && !track; ++i) {
        w.step();
        for (const auto& observation : w.amtiTracks(own, radar))
            if (observation.truth == target && observation.updates >= 3) { track = observation.id; break; }
    }
    require(track != 0, "head-on opponent did not generate a confirmed native radar track");
    // The truth field above validates this controlled fixture. Only the measured track is passed to guidance.
    const auto designation = w.designate(own, radar, track);
    require(designation >= session::World::kDesignations, "own radar track designation rejected");
    ms::AmtiCommand stt;
    stt.payload = radar; stt.task = ms::AmtiTask::Track; stt.mode = ms::AmtiMode::SingleTargetTrack;
    stt.track = track; stt.rangeScaleM = 60000;
    require(w.submit(own, stt).accepted(), "native STT rejected");
    ms::StrikeCommand strike;
    strike.payload = stations; strike.store = "aim-120c"; strike.target = designation;
    const auto accepted = w.submit(own, strike);
    require(accepted.accepted(), "armed native strike rejected");
    require(w.weapons().empty(), "acceptance must not release before warmup and consent");
    for (unsigned i = 0; i < 150; ++i) {
        const auto status = w.strikeStatus(accepted.activity);
        require(status.has_value(), "strike activity status missing");
        if (status->phase == ms::StrikePhase::AwaitingConsent) break;
        w.step();
    }
    require(w.strikeStatus(accepted.activity)->phase == ms::StrikePhase::AwaitingConsent, "missile warmup must reach consent request");
    require(w.weapons().empty(), "awaiting consent must preserve weapon inventory");
    require(w.strikeConsent(accepted.activity, true) == ms::Reason::None, "release consent rejected");
    for (unsigned i = 0; i < 30 && w.weapons().empty(); ++i) w.step();
    require(w.weapons().size() == 1, "consent must release exactly one native weapon");
    const auto missile = w.weapons().front().id;
    const auto motorThrust = w.model(missile)->property("propulsion/engine/vacuum-thrust_lbs");
    require(motorThrust.valid(), "native rocket thrust property missing");
    Outcome out;
    bool acquired = false, activated = false, supported = false, nativeCoast = false, midcourseSupported = false;
    double minRange = std::numeric_limits<double>::infinity(), maxSpeed = 0;
    for (unsigned i = 0; i < 3600 && w.alive(missile); ++i) {
        w.step();
        const auto record = weapon(w, missile);
        out.burned = out.burned || record.burning;
        out.coasted = out.coasted || (out.burned && !record.burning && w.alive(missile));
        if (w.alive(missile)) {
            const double thrust = motorThrust.get();
            nativeCoast = nativeCoast || (out.burned && thrust == 0);
            if (rangeM > 10000 && i % 300 == 0)
                std::cout << "MOTOR: age_s=" << w.simTime() - record.launchTimeS << " thrust_lbs=" << thrust
                          << " fuel_flow_kg_s=" << w.model(missile)->engineFuelFlowKgS(0)
                          << " fuel_kg=" << w.vehicleState(missile)->fuelKg << " burning=" << record.burning << '\n';
        }
        supported = supported || record.supported;
        midcourseSupported = midcourseSupported || (record.phase == ms::WeaponPhase::Midcourse && record.supported && !record.tracking);
        acquired = acquired || record.tracking;
        if (const auto* s = w.vehicleState(missile)) {
            maxSpeed = std::max(maxSpeed, s->airspeedTrueMs);
            if (const auto* t = w.vehicleState(target))
                minRange = std::min(minRange, std::hypot(s->positionEcef[0] - t->positionEcef[0],
                                                      s->positionEcef[1] - t->positionEcef[1], s->positionEcef[2] - t->positionEcef[2]));
        }
    }
    out.record = weapon(w, missile);
    for (const auto& event : w.engagementEvents()) if (event.entity == missile && event.kind == "activation") activated = true;
    std::cout << "DIAGNOSTIC: range=" << rangeM << " workers=" << workers << " end=" << ms::weaponEndName(out.record.end)
              << " phase=" << ms::weaponPhaseName(out.record.phase) << " min_range_m=" << minRange << " miss_m=" << out.record.missM
              << " maximum_speed_ms=" << maxSpeed << " burned=" << out.burned << " coasted=" << out.coasted
              << " native_coast=" << nativeCoast << " supported=" << supported << " acquired=" << acquired << " activated=" << activated << '\n';
    require(out.record.seeker == "active_radar" && !out.record.jettisoned, "native strike must create armed active radar missile");
    require(out.burned && maxSpeed > 350, "JSBSim motor must burn and accelerate native missile");
    require(supported, "own radar designation must supply native launcher support");
    if (rangeM > 20000) require(midcourseSupported, "pre-activation flight must use own designation without a seeker truth hold");
    require(activated && acquired, "native active seeker must activate and acquire");
    require(out.record.end != ms::WeaponEnd::InFlight, "weapon must reach a terminal outcome within its lifetime");
    if (requireHit) require(out.record.end == ms::WeaponEnd::Detonated && out.record.holding == target,
                            "five-kilometre head-on armed missile must detonate at observed opponent");
    if (out.record.end == ms::WeaponEnd::Detonated) {
        bool damageEvent = false, burstEvent = false;
        for (const auto& event : w.engagementEvents()) if (event.entity == missile && event.kind == "detonation") {
            burstEvent = true;
            terminalPoseMatchesBurst(out.record, event);
        }
        require(burstEvent, "detonated record must have an authoritative burst event");
        for (const auto& event : w.engagementEvents()) if (event.entity == missile && event.kind == "damage" && event.target == target) {
            damageEvent = true;
            require(w.damage(target) != nullptr && event.destroyed == w.damage(target)->crewLost(),
                    "damage event destruction must reflect native crew outcome");
        }
        require(damageEvent, "observed target detonation must connect to native damage events");
    }
    replayableEvents(w, missile);
    out.events = w.engagementEvents();
    return out;
}

void headOnAndWorkers() {
    const auto single = engagement(5000, 1, true);
    const auto parallel = engagement(5000, 2, true);
    require(single.record.endTimeS == parallel.record.endTimeS && single.record.missM == parallel.record.missM &&
            single.events.size() == parallel.events.size(), "worker counts must preserve terminal trajectory and event count");
    for (std::size_t i = 0; i < single.events.size(); ++i)
        require(sameEvent(single.events[i], parallel.events[i]), "worker counts must preserve exact engagement event content");
}

void poweredThenCoasting() {
    const auto outcome = engagement(30000, 1, false);
    require(outcome.burned && outcome.coasted, "native rocket must transition from powered flight to coast");
}

void truthTargetCannotSupplySupport() {
    session::World w(options("native-no-truth-midcourse"));
    const auto own = aircraft(w, "launcher", 0, 90);
    const auto target = aircraft(w, "opponent", 40000, 270);
    const auto stations = payload(w, own, ms::Family::StoreStation);
    require(w.loadStore(own, stations, 1, "aim-120c") == ms::Reason::None, "missile load rejected");
    operate(w, own, stations);
    ms::StrikeCommand strike;
    strike.payload = stations; strike.store = "aim-120c"; strike.target = target; strike.consent = true;
    require(w.submit(own, strike).accepted(), "platform strike rejected");
    for (unsigned i = 0; i < 150 && w.weapons().empty(); ++i) w.step();
    require(w.weapons().size() == 1, "platform strike failed to release");
    const auto missile = w.weapons().front().id;
    for (unsigned i = 0; i < 30; ++i) {
        w.step();
        const auto r = weapon(w, missile);
        require(!r.supported && !r.tracking && r.holding == 0 && r.timeToGoS == 0,
                "direct target entity must not synthesize launcher updates or seeker observations outside acquisition range");
    }
}

void detonationAssessesBothAircraftWhileCreatingDebris() {
    // Reuse test_damage.cpp's close MiG-29 wing-burst geometry. The deterministic geometry search
    // selects a case that actually separates the first victim before the second is assessed.
    unsigned fixture = 0;
    for (const double gap : {1.0, 0.7, 1.5})
        for (const double span : {3.0, 2.2, 4.0})
            for (const double aft : {1.5, 0.5, 2.5}) {
                session::World w(options("native-two-victim-debris-" + std::to_string(fixture++)));
                session::VehicleSpec spec;
                spec.name = "first-victim"; spec.type = "jsbsim:mig29a";
                spec.initial.altitudeMslM = 6000; spec.initial.airspeedTrueMs = 220; spec.initial.headingDeg = 90;
                const auto first = w.createVehicle(spec);
                spec.name = "second-victim";
                const auto second = w.createVehicle(spec);
                require(first != 0 && second != 0, "colocated MiG-29A creation failed");
                advance(w, 3);
                // Copy the state before detonation: debris creation may grow the world's state storage.
                const sim::VehicleState state = *w.vehicleState(first);
                const double* rotation = state.rotationBodyToEcef;
                const double offset[3] = {-aft, -span, gap};
                const double closing[3] = {600, 0, 0};
                session::Detonation burst;
                burst.store = "aim-120c";
                for (int row = 0; row < 3; ++row) {
                    burst.positionEcef[row] = state.positionEcef[row];
                    for (int column = 0; column < 3; ++column) {
                        burst.positionEcef[row] += rotation[3 * row + column] * offset[column];
                        burst.velocityEcef[row] += rotation[3 * row + column] * (state.velocityBodyMs[column] + closing[column]);
                    }
                    burst.axisEcef[row] = burst.velocityEcef[row];
                }
                const auto events = w.detonate(burst);
                require(!events.empty(), "close wing burst must produce native damage events");
                require(w.damage(first) != nullptr && w.damage(second) != nullptr,
                        "burst must assess both colocated aircraft while debris appends new slots");
                std::vector<std::uint32_t> firstDebris;
                for (const auto id : w.vehicleIds()) {
                    const auto* info = w.info(id);
                    if (info && info->role == ms::Role::Debris && info->launcher == first) firstDebris.push_back(id);
                }
                if (firstDebris.empty()) continue;
                require(w.alive(first) && w.alive(second), "detonation must preserve both native victim entries");
                advance(w, 1);
                for (const auto id : w.vehicleIds()) {
                    const auto* after = w.vehicleState(id);
                    require(after != nullptr && !after->diverged && std::isfinite(after->altitudeMslM) &&
                            std::isfinite(after->airspeedTrueMs) && std::isfinite(after->positionEcef[0]) &&
                            std::isfinite(after->positionEcef[1]) && std::isfinite(after->positionEcef[2]),
                            "both damaged aircraft and appended debris must remain valid during later native steps");
                }
                std::cout << "DEBRIS: geometry_aft_m=" << aft << " span_m=" << span << " gap_m=" << gap
                          << " first_victim_pieces=" << firstDebris.size() << " victim_count=2\n";
                return;
            }
    throw std::runtime_error("close MiG-29A wing-burst fixture must separate the first victim and exercise debris slot growth");
}

void infraredReleaseKeepsOnRailContact() {
    session::World w(options("native-infrared-launch-cue"));
    const auto own=aircraft(w,"launcher",0,90);
    const auto stations=payload(w,own,ms::Family::StoreStation);
    require(w.loadStore(own,stations,1,"aim-9x")==ms::Reason::None,"infrared missile load rejected");
    operate(w,own,stations);
    const auto ownState=*w.vehicleState(own);
    auto targetAt=[&](const std::string& name,double east,double north) {
        session::VehicleSpec s; s.name=name; s.type="jsbsim:f16c";
        s.initial.latitudeDeg=ownState.latitudeRad/kDeg+north/6371000.0/kDeg;
        s.initial.longitudeDeg=ownState.longitudeRad/kDeg+east/(6371000.0*std::cos(ownState.latitudeRad))/kDeg;
        s.initial.altitudeMslM=6000; s.initial.airspeedTrueMs=250; s.initial.headingDeg=90;
        const auto id=w.createVehicle(s);
        control::VelocityCommand hold; hold.airspeedMs=250; hold.verticalSpeedMs=0; hold.headingRad=90*kDeg;
        require(id && w.submit(id,hold).accepted(),"infrared fixture aircraft creation failed");
        return id;
    };
    const auto intended=targetAt("on-axis-contact",5000,0);
    targetAt("closer-off-axis-contact",940,342);
    require(w.seekerTarget(own,"aim-9x")==intended,"on-rail infrared observation must select on-axis contact");
    ms::StrikeCommand strike; strike.payload=stations; strike.store="aim-9x"; strike.target=intended; strike.consent=true;
    require(w.submit(own,strike).accepted(),"observed infrared release rejected");
    for(unsigned i=0;i<150 && w.weapons().empty();++i) w.step();
    require(w.weapons().size()==1,"infrared strike must release one weapon");
    const auto missile=w.weapons().front().id;
    w.step();
    const auto record=weapon(w,missile);
    require(record.tracking && record.holding==intended,"released infrared seeker must acquire its on-rail contact rather than a nearer off-axis contact");
}

void sweptDamageUsesEncounterPositions() {
    auto settings=options("native-swept-damage"); settings.frameSkip=240;
    session::World w(settings);
    const auto victim=aircraft(w,"fast-victim",0,90);
    const auto before=*w.vehicleState(victim);
    w.step();
    const auto after=*w.vehicleState(victim);
    session::Detonation burst; burst.store="aim-120c"; burst.stepFraction=.5;
    double endpointDistanceSquared=0;
    for(int k=0;k<3;++k) {
        burst.positionEcef[k]=(before.positionEcef[k]+after.positionEcef[k])*.5-4*after.rotationBodyToEcef[k*3+1];
        burst.velocityEcef[k]=600*after.rotationBodyToEcef[k*3];
        burst.axisEcef[k]=after.rotationBodyToEcef[k*3];
        const double d=burst.positionEcef[k]-after.positionEcef[k]; endpointDistanceSquared+=d*d;
    }
    require(std::sqrt(endpointDistanceSquared)>150,"swept fixture endpoint must lie beyond damage reach");
    const auto events=w.detonate(burst);
    require(w.damage(victim)!=nullptr && !events.empty() && events.front().target!=0,
            "swept damage must assess the encounter position even when the step endpoint is out of reach");
}

void inertJettison() {
    session::World w(options("native-inert-jettison"));
    const auto own = aircraft(w, "launcher", 0, 90);
    const auto opponent = aircraft(w, "opponent", 5000, 270);
    const auto stations = payload(w, own, ms::Family::StoreStation);
    require(w.loadStore(own, stations, 1, "aim-120c") == ms::Reason::None, "missile load rejected");
    ms::JettisonCommand command; command.payload = stations; command.emergency = true;
    require(w.submit(own, command).accepted(), "emergency jettison rejected");
    advance(w, 0.1);
    require(w.weapons().size() == 1, "jettison must create one native inert body");
    const auto missile = w.weapons().front().id;
    for (unsigned i = 0; i < 1800 && w.alive(missile); ++i) {
        w.step();
        const auto r = weapon(w, missile);
        require(r.jettisoned && r.phase == ms::WeaponPhase::Unguided && !r.tracking && !r.supported && !r.burning,
                "jettisoned missile must remain unguided, unpowered and unsupported");
        if (w.alive(missile)) require(w.model(missile)->engineFuelFlowKgS(0) == 0, "inert missile motor must remain off");
    }
    for (const auto& event : w.engagementEvents())
        require(event.entity != missile || (event.kind != "detonation" && event.kind != "damage"),
                "inert jettison must never detonate or damage opponents");
    require(w.damage(opponent) == nullptr, "inert jettison must not create opponent damage state");
}
} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {{"head-on armed engagement and worker determinism", headOnAndWorkers},
                          {"JSBSim motor powered flight then coast", poweredThenCoasting},
                          {"truth target cannot supply midcourse support", truthTargetCannotSupplySupport},
                          {"two-victim detonation safely appends native debris", detonationAssessesBothAircraftWhileCreatingDebris},
                          {"infrared release keeps on-rail contact", infraredReleaseKeepsOnRailContact},
                          {"swept damage uses encounter positions", sweptDamageUsesEncounterPositions},
                          {"inert jettison", inertJettison}};
    unsigned failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n'; }
    }
    return failures ? 1 : 0;
}
