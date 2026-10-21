// End-to-end native engagement acceptance. Every weapon flies through session::World and JSBSim.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"
#include "simulation/mission_systems/Model.h"
#include "foundation/io/AssetResolver.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT "D:/Projects/Cpp/Super-DDCCSS-WWorld/third_party/jsbsim"
#endif

using namespace fsim;
namespace {
constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180.0;
void require(bool value, const std::string& message) { if (!value) throw std::runtime_error(message); }
bool visualRun() {const auto* value=std::getenv("FSIM_ENGAGEMENT_VISUAL");return value && std::string(value)=="1";}
void step(session::World& world,unsigned count=1) {
    if (!visualRun()) {world.step(count);return;}
    for (unsigned i=0;i<count;++i) {
        world.step();
        std::this_thread::sleep_for(std::chrono::duration<double>(world.dt()*world.frameSkip()));
    }
}

session::WorldOptions options(const std::string& name, unsigned workers = 1) {
    session::WorldOptions o;
    o.name = name; o.workers = workers; o.pinWorkers = false; o.publish = false; o.seed = 17;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    if (visualRun()) o.publish=true;
    if(const char* prefix=std::getenv("FSIM_ENGAGEMENT_RECORD")) o.recordPath=std::string(prefix)+"-"+name+".fsrec";
    return o;
}

void advance(session::World& w, double seconds) {
    step(w,static_cast<unsigned>(std::lround(seconds / (w.dt() * w.frameSkip()))));
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
    for (unsigned i = 0; i < 1200 && w.missionSystems(id)->payloads()[index].state != ms::PayloadState::Operate; ++i) step(w);
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
    require(std::abs(record.endSpeedMs-std::hypot(event.velocityEcef[0],event.velocityEcef[1],event.velocityEcef[2]))<1e-6,
            "terminal speed and detonation velocity must describe the same encounter sample");
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

void functionalDamageAndContinuedFlight(session::World& w,std::uint32_t target,std::uint32_t missile,
                                       double beforeMassKg,const sim::VehicleState& beforeEncounter) {
    const auto functional=[&]() {
        const auto* damage=w.damage(target);
        if (!damage) return false;
        if (damage->crewLost() || !damage->lostChannels().empty()) return true;
        for (const auto& section : damage->sections()) if (section.separated) return true;
        const auto* state=w.vehicleState(target);
        for (int engine=0;state && engine<state->engineCount;++engine) if (damage->engineOut(engine)) return true;
        return false;
    };
    const double damageDeadline=w.simTime()+5;
    while (!functional() && w.simTime()<damageDeadline) step(w);
    require(functional(),"fixed-scene functional acceptance requires native structural separation or engine/control/crew loss; a hit alone is insufficient");
    bool linked=false;
    for (const auto& event : w.engagementEvents())
        if (event.entity==missile && event.target==target && event.kind=="damage") linked=true;
    require(linked,"functional target loss must have an authoritative damage event from this released weapon");
    const auto first=*w.vehicleState(target);
    const auto held=*w.inputs(target);
    const double followStart=w.simTime();
    const auto* damage=w.damage(target);
    const bool crewAtFirst=damage->crewLost();
    double separatedKg=0;
    for (std::size_t k=0;k<damage->sections().size();++k)
        if (damage->sections()[k].separated) separatedKg+=damage->airframe().sections[k].massKg;
    bool nativeConsequence=separatedKg>0 && beforeMassKg-w.model(target)->energy().massKg>=separatedKg-1e-6;
    unsigned engineLosses=0,channelLosses=0;
    for (int engine=0;engine<first.engineCount;++engine) if (damage->engineOut(engine)) {
        const auto seized=w.model(target)->property("propulsion/engine["+std::to_string(engine)+"]/seized");
        require(seized.valid() && seized.get()==1,"native engine loss must seize the actual JSBSim engine");
        nativeConsequence=true; ++engineLosses;
    }
    for (const auto& channel : damage->lostChannels()) {
        bool stuck=false;
        for (const auto& [name,value] : w.model(target)->properties("fcs"))
            if (name.find(channel)!=std::string::npos && name.find("/malfunction/fail_stuck")!=std::string::npos && value==1) stuck=true;
        require(stuck,"native control loss must set a real JSBSim actuator failure for "+channel);
        nativeConsequence=true; ++channelLosses;
    }
    advance(w,5);
    const auto last=*w.vehicleState(target);
    require(w.simTime()>=followStart+5-w.dt()*w.frameSkip(),"functional acceptance must continue native simulation for five seconds after loss");
    require(!last.diverged,"damaged native aircraft must retain a finite integrated outcome, including an authoritative crash");
    for (int k=0;k<3;++k)
        require(std::isfinite(last.positionEcef[k]) && std::isfinite(last.velocityNedMs[k]) &&
                std::isfinite(last.angularRateBodyRadS[k]) && std::isfinite(last.eulerRad[k]),
                "continued damaged flight contains nonfinite native motion");
    if (crewAtFirst) {
        const auto& after=*w.inputs(target);
        require(after.aileron==held.aileron && after.elevator==held.elevator && after.rudder==held.rudder &&
                std::equal(after.throttle,after.throttle+sim::ControlInputs::kMaxEngines,held.throttle),
                "crew loss must stop subsequent native crew control updates");
        nativeConsequence=true;
    }
    const double moved=std::hypot(last.positionEcef[0]-first.positionEcef[0],last.positionEcef[1]-first.positionEcef[1],
                                  last.positionEcef[2]-first.positionEcef[2]);
    require(nativeConsequence && (moved>0 || last.crashed),
            "functional loss must alter native mass/engine/control ownership and retain actual later motion or a native crash");
    std::cout << "FUNCTIONAL: target=" << target << " weapon=" << missile << " separated_dry_kg=" << separatedKg
              << " engine_losses=" << engineLosses << " control_losses=" << channelLosses << " crew_lost=" << damage->crewLost()
              << " follow_s=" << w.simTime()-followStart << " moved_m=" << moved << " crashed=" << last.crashed
              << " altitude_before_m=" << beforeEncounter.altitudeMslM << " altitude_after_m=" << last.altitudeMslM
              << " speed_before_ms=" << beforeEncounter.airspeedTrueMs << " speed_after_ms=" << last.airspeedTrueMs << '\n';
}

Outcome engagement(double rangeM, unsigned workers, bool requireHit, bool requireFunctional=false) {
    session::World w(options("native-engagement-" + std::to_string(workers) + "-" + std::to_string(rangeM), workers));
    const auto own = aircraft(w, "launcher", 0, 90);
    if (visualRun()) w.focusViewer(own);
    const auto stations = payload(w, own, ms::Family::StoreStation);
    const auto radar = payload(w, own, ms::Family::AirRadar);
    require(w.loadStore(own, stations, 1, "aim-120c") == ms::Reason::None, "missile load rejected");
    operate(w, own, stations); operate(w, own, radar);
    advance(w, 10);
    bool configuredAsset=false;
    for (const auto& store : w.missionSystems(own)->stores()) if (store.store && std::string(store.store)=="aim-120c") {
        const auto* card=w.missionSystems(own)->storeCard(store.index);
        if (card) std::cout << "MODEL-ASSET: card=" << card->id << " execution=" << card->value("weapon_execution_version",0)
            << " fuze=" << card->value("fuze_model_version",0) << " warhead=" << card->value("warhead_model_version",0) << '\n';
        configuredAsset=card && card->value("weapon_execution_version",0)==1 &&
            card->value("fuze_model_version",0)==1 && card->value("warhead_model_version",0)==1;
    }
    const auto library=io::AssetResolver{}.find("payloads");
    require(configuredAsset,"native acceptance must consume the current versioned model card, not a stale staged asset; payloads="+
        (library ? library->string() : std::string("unavailable")));
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
        step(w);
        for (const auto& observation : w.amtiTracks(own, radar))
            if (observation.truth == target && observation.updates >= 3) { track = observation.id; break; }
    }
    require(track != 0, "head-on opponent did not generate a confirmed native radar track");
    // The truth field above validates this controlled fixture. Only the measured track is passed to guidance.
    const auto designation = w.designate(own, radar, track);
    require(designation >= session::World::kDesignations, "own radar track designation rejected");
    const auto* measured = w.vehicleState(designation);
    require(measured != nullptr, "designation estimate missing");
    double measuredEcef[3]; ms::geom::velocityEcef(*measured, measuredEcef);
    for (const auto& observation : w.amtiTracks(own, radar)) if (observation.id == track)
        for (int k=0;k<3;++k)
            require(std::abs(measuredEcef[k]-observation.stateEcef[2*k+1]) < .001,
                    "designation must retain measured ECEF track velocity");
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
        step(w);
    }
    require(w.strikeStatus(accepted.activity)->phase == ms::StrikePhase::AwaitingConsent, "missile warmup must reach consent request");
    require(w.weapons().empty(), "awaiting consent must preserve weapon inventory");
    require(w.strikeConsent(accepted.activity, true) == ms::Reason::None, "release consent rejected");
    for (unsigned i = 0; i < 30 && w.weapons().empty(); ++i) step(w);
    require(w.weapons().size() == 1, "consent must release exactly one native weapon");
    const auto missile = w.weapons().front().id;
    if (visualRun()) w.focusViewer(missile);
    const auto motorThrust = w.model(missile)->property("propulsion/engine/vacuum-thrust_lbs");
    require(motorThrust.valid(), "native rocket thrust property missing");
    // Opt-in referee diagnostics only. All inputs/observations are read after
    // execution; this trace never changes guidance, control or model values.
    std::ofstream trace;
    std::vector<sim::PropertyHandle> traceProperties;
    const std::vector<std::string> tracePaths={"aero/qbar-psf","forces/fbx-aero-lbs","forces/fby-aero-lbs","forces/fbz-aero-lbs",
        "moments/l-aero-lbsft","moments/m-aero-lbsft","moments/n-aero-lbsft",
        "fcs/elevator-response-rad","fcs/rudder-response-rad","fcs/aileron-response-rad",
        "fcs/fin-0-delta-rad","fcs/fin-1-delta-rad","fcs/fin-2-delta-rad","fcs/fin-3-delta-rad",
        "inertia/ixx-slugs_ft2","inertia/iyy-slugs_ft2","inertia/izz-slugs_ft2",
        "metrics/Sw-sqft","metrics/bw-ft","fsim/control/cz-alpha","fsim/control/cy-beta","fsim/control/cm-alpha",
        "fsim/control/cn-beta","fsim/control/cm-q","fsim/control/cn-r","fsim/control/cl-p","fsim/control/cz-elevator",
        "fsim/control/cy-rudder","fsim/control/cm-elevator","fsim/control/cn-rudder","fsim/control/cl-aileron"};
    if (const char* prefix=std::getenv("FSIM_ENGAGEMENT_TRACE")) {
        trace.open(std::string(prefix)+"-r"+std::to_string(static_cast<int>(rangeM))+"-w"+std::to_string(workers)+".csv");
        require(trace.is_open(),"diagnostic trace cannot open output");
        trace << std::setprecision(17) << "age,phase,tracking,supported,holding,tgo,miss,burning,range_before,range_after,alt,speed,alpha,beta,p,q,r,roll,pitch,yaw,mass,fuel,thrust,elevator_cmd,rudder_cmd,aileron_cmd,elevator_pos,rudder_pos,aileron_pos";
        for (const char* group : {"own_p","own_v","target_p","target_v","estimate_p","estimate_v"})
            for (const char* axis : {"x","y","z"}) trace << ',' << group << '_' << axis;
        for (int k=0;k<9;++k) trace << ",own_rotation_" << k;
        trace << ",own_lat,own_lon,cg_x,cg_y,cg_z,own_alpha,own_beta,own_p,own_q,own_r,own_speed,mach";
        for (const auto& path : tracePaths) {trace << ',' << path;traceProperties.push_back(w.model(missile)->property(path));}
        trace << '\n';
    }
    Outcome out;
    bool acquired = false, activated = false, supported = false, nativeCoast = false, midcourseSupported = false;
    double minRange = std::numeric_limits<double>::infinity(), maxSpeed = 0;
    double targetMassBeforeEncounter=w.model(target)->energy().massKg;
    sim::VehicleState targetBeforeEncounter=*w.vehicleState(target);
    for (unsigned i = 0; i < 3600 && w.alive(missile); ++i) {
        const auto beforeWeapon=*w.vehicleState(missile),beforeTarget=*w.vehicleState(target);
        if (requireFunctional) {targetMassBeforeEncounter=w.model(target)->energy().massKg;targetBeforeEncounter=beforeTarget;}
        step(w);
        const auto record = weapon(w, missile);
        out.burned = out.burned || record.burning;
        out.coasted = out.coasted || (out.burned && !record.burning && w.alive(missile));
        if (w.alive(missile)) {
            require(record.burning == (w.vehicleState(missile)->thrustN[0] > .1),
                    "published burning must describe current native thrust, including ignition and burnout");
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
            if (trace.is_open()) {
                const auto distance=[](const auto& a,const auto& b) {return std::hypot(a.positionEcef[0]-b.positionEcef[0],a.positionEcef[1]-b.positionEcef[1],a.positionEcef[2]-b.positionEcef[2]);};
                const auto& t=*w.vehicleState(target); const auto* estimate=w.vehicleState(designation);
                const auto* command=w.inputs(missile); double cg[3]; w.model(missile)->cgFromOriginBodyM(cg);
                trace << w.simTime()-record.launchTimeS << ',' << ms::weaponPhaseName(record.phase) << ',' << record.tracking << ',' << record.supported << ',' << record.holding << ',' << record.timeToGoS << ',' << record.missM << ',' << record.burning
                      << ',' << distance(beforeWeapon,beforeTarget) << ',' << distance(*s,t) << ',' << s->altitudeMslM << ',' << s->airspeedTrueMs << ',' << s->alphaRad << ',' << s->betaRad;
                for (double v : s->angularRateBodyRadS) trace << ',' << v;
                for (double v : s->eulerRad) trace << ',' << v;
                trace << ',' << w.model(missile)->energy().massKg << ',' << s->fuelKg << ',' << s->thrustN[0]
                      << ',' << command->elevator << ',' << command->rudder << ',' << command->aileron
                      << ',' << s->elevatorRad << ',' << s->rudderRad << ',' << s->aileronRad;
                for (const auto* state : {&beforeWeapon,&beforeTarget,estimate}) {
                    if (!state) {for (int k=0;k<6;++k) trace << ",nan";continue;}
                    for (double v : state->positionEcef) trace << ',' << v;
                    double velocity[3]; ms::geom::velocityEcef(*state,velocity);
                    for (double v : velocity) trace << ',' << v;
                }
                for (double v : beforeWeapon.rotationBodyToEcef) trace << ',' << v;
                trace << ',' << beforeWeapon.latitudeRad << ',' << beforeWeapon.longitudeRad;
                for (double v : cg) trace << ',' << v;
                trace << ',' << beforeWeapon.alphaRad << ',' << beforeWeapon.betaRad;
                for (double v : beforeWeapon.angularRateBodyRadS) trace << ',' << v;
                trace << ',' << beforeWeapon.airspeedTrueMs << ',' << s->mach;
                for (const auto& property : traceProperties) trace << ',' << (property.valid()?property.get():std::numeric_limits<double>::quiet_NaN());
                trace << '\n';
            }
        }
    }
    out.record = weapon(w, missile);
    if (visualRun()) w.focusViewer(target);
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
                            "head-on armed missile regression must detonate at its observed opponent");
    if (out.record.end == ms::WeaponEnd::Detonated) {
        bool damageEvent = false, burstEvent = false;
        for (const auto& event : w.engagementEvents()) if (event.entity == missile && event.kind == "detonation") {
            burstEvent = true;
            require(event.target == target, "burst event must retain the seeker's observed target");
            terminalPoseMatchesBurst(out.record, event);
        }
        require(burstEvent, "detonated record must have an authoritative burst event");
        require(w.damage(target) != nullptr, "observed target detonation must run native damage assessment");
        for (const auto& event : w.engagementEvents()) if (event.entity == missile && event.kind == "damage" && event.target == target) {
            damageEvent = true;
            require(w.damage(target) != nullptr && event.destroyed == w.damage(target)->crewLost(),
                    "damage event destruction must reflect native crew outcome");
        }
        // Detonation alone does not imply damage. These fixed-scene hit
        // acceptance cases separately require actual native hit evidence.
        if (requireHit) require(damageEvent, "fixed-scene hit acceptance requires actual target damage; detonation alone is insufficient");
    }
    if (requireFunctional) functionalDamageAndContinuedFlight(w,target,missile,targetMassBeforeEncounter,targetBeforeEncounter);
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
void fiveKilometreFunctionalAcceptance() { engagement(5000,1,true,true); }

void poweredThenCoasting() {
    const auto outcome = engagement(30000, 1, false);
    require(outcome.burned && outcome.coasted, "native rocket must transition from powered flight to coast");
}
void thirtyKilometreNativeControlRegression() {
    // Exact software regression for the existing fixture, not a calibrated
    // launch envelope. Geometry, model, seed and fuze bounds are unchanged.
    const auto single=engagement(30000,1,true);
    const auto parallel=engagement(30000,2,true);
    require(single.record.endTimeS==parallel.record.endTimeS && single.record.missM==parallel.record.missM &&
            single.events.size()==parallel.events.size(),"30 km native control regression must be worker-independent");
    for (std::size_t i=0;i<single.events.size();++i)
        require(sameEvent(single.events[i],parallel.events[i]),"30 km native event stream must preserve exact worker determinism");
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
    for (unsigned i = 0; i < 150 && w.weapons().empty(); ++i) step(w);
    require(w.weapons().size() == 1, "platform strike failed to release");
    const auto missile = w.weapons().front().id;
    for (unsigned i = 0; i < 30; ++i) {
        step(w);
        const auto r = weapon(w, missile);
        require(!r.supported && !r.tracking && r.holding == 0 && std::isnan(r.timeToGoS),
                "direct target entity must not synthesize launcher updates or seeker observations outside acquisition range");
    }
    advance(w,6);
    require(w.alive(missile),"unsupported missile must remain available for low-energy fixture");
    // A diagnostic reset supplies a slow but physically valid airborne state;
    // it retains the weapon lifecycle age and guidance ownership by contract.
    sim::InitialConditions slow;
    slow.altitudeMslM=12000; slow.airspeedTrueMs=100; slow.headingDeg=90;
    slow.enginesRunning=false;
    require(w.resetVehicle(missile,&slow),"low-energy missile reset failed");
    auto contents=w.model(missile)->property("propulsion/tank[0]/contents-lbs");
    require(contents.valid(),"missile native propellant property missing"); contents.set(0);
    step(w);
    require(w.alive(missile) && w.vehicleState(missile)->airspeedTrueMs<180 &&
            !weapon(w,missile).burning && weapon(w,missile).end==ms::WeaponEnd::InFlight,
            "A live slow coasting weapon must continue native flight until contact or configured life expiry");
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
    for(unsigned i=0;i<150 && w.weapons().empty();++i) step(w);
    require(w.weapons().size()==1,"infrared strike must release one weapon");
    const auto missile=w.weapons().front().id;
    step(w);
    const auto record=weapon(w,missile);
    require(record.tracking && record.holding==intended,"released infrared seeker must acquire its on-rail contact rather than a nearer off-axis contact");
}

void sweptDamageUsesEncounterPositions() {
    auto settings=options("native-swept-damage"); settings.frameSkip=240;
    session::World w(settings);
    const auto victim=aircraft(w,"fast-victim",0,90);
    const auto before=*w.vehicleState(victim);
    step(w);
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

void sweptDamageRetainsEncounterCg() {
    const auto assess=[](bool changeLaterMass) {
        auto settings=options("native-swept-damage-cg"); settings.frameSkip=4;
        session::World w(settings);
        const auto victim=aircraft(w,"launcher-victim",0,90);
        const auto stations=payload(w,victim,ms::Family::StoreStation);
        require(w.loadStore(victim,stations,1,"aim-9x")==ms::Reason::None,"CG fixture missile load rejected");
        operate(w,victim,stations);
        const auto target=aircraft(w,"remote-contact",5000,90);
        ms::StrikeCommand strike; strike.payload=stations; strike.store="aim-9x";
        strike.target=target; strike.consent=true;
        require(w.submit(victim,strike).accepted(),"CG fixture release rejected");
        for(unsigned i=0;i<150 && w.weapons().empty();++i) step(w);
        require(w.weapons().size()==1,"CG fixture needs a guided release to capture native motion");
        const auto encounter=*w.vehicleState(victim);
        auto* model=w.model(victim);
        double cgBefore[3]; require(model->cgFromOriginBodyM(cgBefore),"CG fixture offset unavailable");
        step(w);
        if(changeLaterMass) {
            auto pilot=model->property("inertia/pointmass-weight-lbs");
            require(pilot.valid(),"CG fixture pilot pointmass missing");
            require(model->beginMassChange(),"CG fixture mass transaction rejected");
            pilot.set(0);
            require(model->endMassChange(nullptr,nullptr),"CG fixture mass transaction failed");
            double cgAfter[3]; require(model->cgFromOriginBodyM(cgAfter),"changed CG unavailable");
            require(std::abs(cgAfter[0]-cgBefore[0])>1e-3,"CG fixture must actually change the later offset");
        }
        session::Detonation burst; burst.store="aim-120c"; burst.stepFraction=0;
        const auto* R=encounter.rotationBodyToEcef;
        for(int r=0;r<3;++r) {
            burst.positionEcef[r]=encounter.positionEcef[r]-R[3*r]*cgBefore[0]-R[3*r+1]*cgBefore[1]-R[3*r+2]*cgBefore[2]+4*R[3*r+1];
            burst.velocityEcef[r]=600*R[3*r]; burst.axisEcef[r]=R[3*r];
        }
        w.detonate(burst);
        const auto* damage=w.damage(victim); require(damage!=nullptr,"CG fixture burst was not assessed");
        std::vector<double> result;
        bool loaded=false;
        for(const auto& section:damage->sections()) {
            result.insert(result.end(),{section.strength,section.kick,section.holeM2,section.tornM2});
            loaded=loaded || section.kick>0 || section.holeM2>0 || section.tornM2>0;
        }
        require(loaded,"CG fixture must produce real section loading");
        return result;
    };
    const auto unchanged=assess(false), changedLater=assess(true);
    require(unchanged==changedLater,
            "A later mass/CG change must not move encounter-time geometry or rewrite its damage assessment");
}

void noWeaponSweptDamageRetainsEncounterCg() {
    const auto assess=[](bool changeLaterMass) {
        auto settings=options("native-boundary-damage-cg"); settings.frameSkip=4;
        session::World w(settings);
        const auto victim=aircraft(w,"boundary-victim",0,90);
        const auto encounter=*w.vehicleState(victim);
        auto* model=w.model(victim);
        double cgBefore[3]; require(model->cgFromOriginBodyM(cgBefore),"Boundary CG fixture offset unavailable");
        step(w);
        require(w.weapons().empty(),"Boundary history must be available without any released weapon");
        if(changeLaterMass) {
            auto pilot=model->property("inertia/pointmass-weight-lbs");
            require(pilot.valid() && model->beginMassChange(),"Boundary CG fixture mass transaction unavailable");
            pilot.set(0);
            require(model->endMassChange(nullptr,nullptr),"Boundary CG fixture mass transaction failed");
            double cgAfter[3]; require(model->cgFromOriginBodyM(cgAfter),"Changed boundary CG unavailable");
            require(std::abs(cgAfter[0]-cgBefore[0])>1e-3,"Boundary fixture must actually change the later CG");
        }
        session::Detonation burst; burst.store="aim-120c"; burst.stepFraction=0;
        const auto* r=encounter.rotationBodyToEcef;
        for(int k=0;k<3;++k) {
            burst.positionEcef[k]=encounter.positionEcef[k]-r[3*k]*cgBefore[0]-r[3*k+1]*cgBefore[1]-r[3*k+2]*cgBefore[2]+4*r[3*k+1];
            burst.velocityEcef[k]=600*r[3*k]; burst.axisEcef[k]=r[3*k];
        }
        const auto events=w.detonate(burst);
        const auto* damage=w.damage(victim);
        require(damage && !events.empty() && events.front().target!=0,"A no-weapon captured encounter must be assessed");
        std::vector<double> result;
        bool loaded=false;
        for(const auto& section:damage->sections()) {
            result.insert(result.end(),{section.strength,section.kick,section.holeM2,section.tornM2});
            loaded=loaded || section.kick>0 || section.holeM2>0 || section.tornM2>0;
        }
        require(loaded,"The no-weapon captured encounter must produce real section loading");
        return result;
    };
    require(assess(false)==assess(true),
            "Later mass removal cannot rewrite a no-weapon boundary encounter's CG or damage assessment");
}

void batchedNativeContactPrecedesArming() {
    auto settings=options("native-contact-before-arming"); settings.frameSkip=240;
    session::World w(settings);
    const auto own=aircraft(w,"launcher",0,90);
    const auto stations=payload(w,own,ms::Family::StoreStation);
    require(w.loadStore(own,stations,1,"aim-9x")==ms::Reason::None,"IR fixture load rejected");
    operate(w,own,stations);
    const auto target=aircraft(w,"contact-target",5000,90);
    ms::StrikeCommand strike; strike.payload=stations; strike.store="aim-9x";
    strike.target=target; strike.consent=true;
    require(w.submit(own,strike).accepted(),"IR contact fixture strike rejected");
    for (unsigned i=0;i<10 && w.weapons().empty();++i) step(w);
    require(w.weapons().size()==1,"IR contact fixture release failed");
    const auto missile=w.weapons().front().id;
    const auto initial=*w.vehicleState(missile);
    sim::InitialConditions contact;
    contact.latitudeDeg=initial.latitudeRad/kDeg; contact.longitudeDeg=initial.longitudeRad/kDeg;
    contact.altitudeMslM=.1; contact.airspeedTrueMs=250; contact.headingDeg=90; contact.enginesRunning=false;
    contact.velocityNedMs[0]=0; contact.velocityNedMs[1]=250; contact.velocityNedMs[2]=100;
    require(w.resetVehicle(missile,&contact),"contact weapon reset failed");
    auto propellant=w.model(missile)->property("propulsion/tank[0]/contents-lbs");
    require(propellant.valid(),"contact fixture propellant missing"); propellant.set(0);
    auto crossing=contact;
    crossing.longitudeDeg+=400/(6371000*std::cos(initial.latitudeRad))/kDeg;
    crossing.altitudeMslM=6; crossing.headingDeg=270; crossing.enginesRunning=true;
    crossing.velocityNedMs[1]=-250; crossing.velocityNedMs[2]=0;
    require(w.resetVehicle(target,&crossing),"crossing target reset failed");
    const auto controls=w.weaponControlsV1(missile);
    require(controls.has_value(),"contact fixture live weapon diagnostic missing");
    const auto weaponSlot=controls->slot;
    step(w);
    const auto record=weapon(w,missile);
    require(record.holding==target,"contact fixture must acquire the crossing target before native stepping");
    require(!w.alive(missile) && record.end==ms::WeaponEnd::Impact,
            "Native contact before one-second arming must remain Impact despite a later batch encounter");
    require(std::isfinite(record.endSpeedMs) && record.endSpeedMs>0,
            "Batched impact must retain a finite incoming speed");
    bool nativeImpact=false;
    for(const auto& event:w.takeEngagementEvents())
        if(event.slot==weaponSlot && static_cast<ipc::EngagementKind>(event.kind)==ipc::EngagementKind::Impact) {
            nativeImpact=true;
            const auto& v=event.velocityEcef;
            require(std::abs(record.endSpeedMs-std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]))<1e-9,
                    "Batched terminal speed must match the native incoming-contact event");
        }
    require(nativeImpact,"Batched terrain impact must retain its native contact event");
    for (const auto& event : w.engagementEvents())
        require(event.entity!=missile || event.kind!="detonation",
                "A frozen crash pose must not detonate after native contact");
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
        step(w);
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

int main(int argc,char** argv) {
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {{"head-on armed engagement and worker determinism", headOnAndWorkers},
                          {"five-kilometre native functional damage and continued flight", fiveKilometreFunctionalAcceptance},
                          {"JSBSim motor powered flight then coast", poweredThenCoasting},
                          {"thirty-kilometre native control regression", thirtyKilometreNativeControlRegression},
                          {"truth target cannot supply midcourse support", truthTargetCannotSupplySupport},
                          {"two-victim detonation safely appends native debris", detonationAssessesBothAircraftWhileCreatingDebris},
                          {"infrared release keeps on-rail contact", infraredReleaseKeepsOnRailContact},
                          {"swept damage uses encounter positions", sweptDamageUsesEncounterPositions},
                          {"no-weapon swept damage retains encounter CG", noWeaponSweptDamageRetainsEncounterCg},
                          {"swept damage retains encounter CG", sweptDamageRetainsEncounterCg},
                          {"batched native contact precedes arming", batchedNativeContactPrecedesArming},
                          {"inert jettison", inertJettison}};
    unsigned failures = 0;
    unsigned executed = 0;
    for (const auto& test : tests) {
        if (argc>1 && std::string(test.name).find(argv[1])==std::string::npos) continue;
        ++executed;
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n'; }
    }
    if (!executed) {std::cerr << "No native engagement cases matched\n";return 2;}
    return failures ? 1 : 0;
}
