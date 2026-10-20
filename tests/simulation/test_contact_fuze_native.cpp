// Named mechanism fixtures: real Host releases, generated stores and native
// flight. Repositioning constructs a finite wing crossing; no original range
// scenario, fuze value, force or aerodynamic coefficient is changed.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef FSIM_TEST_JSBSIM_ROOT
#define FSIM_TEST_JSBSIM_ROOT "D:/Projects/Cpp/Super-DDCCSS-WWorld/third_party/jsbsim"
#endif
#ifndef FSIM_TEST_AIRCRAFT_DIR
#define FSIM_TEST_AIRCRAFT_DIR "D:/Projects/Cpp/Super-DDCCSS-WWorld/aircraft"
#endif

using namespace fsim;
namespace {
namespace damage = ms::damage;
constexpr double kPi = 3.14159265358979323846, kDeg = kPi / 180;
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
session::WorldOptions options(int frameSkip, unsigned workers) {
    session::WorldOptions o;
    o.name = "native-contact-mechanism"; o.frameSkip = frameSkip; o.workers = workers;
    o.pinWorkers = false; o.publish = false; o.seed = 17; o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    return o;
}
ms::WeaponRecord record(const session::World& world, std::uint32_t id) {
    for (const auto& value : world.weapons()) if (value.id == id) return value;
    throw std::runtime_error("released weapon record missing");
}
std::uint32_t aircraft(session::World& world, const std::string& name, sim::InitialConditions initial) {
    session::VehicleSpec spec;
    spec.name = name; spec.type = "jsbsim:f16c"; spec.initial = initial;
    const auto id = world.createVehicle(spec);
    require(id != 0, "native F-16C creation failed");
    return id;
}
damage::V3 rotate(const sim::VehicleState& state, damage::V3 v) {
    const auto* r = state.rotationBodyToEcef;
    return {r[0]*v.x+r[1]*v.y+r[2]*v.z, r[3]*v.x+r[4]*v.y+r[5]*v.z, r[6]*v.x+r[7]*v.y+r[8]*v.z};
}
damage::V3 position(const sim::VehicleState& state) {
    return {state.positionEcef[0], state.positionEcef[1], state.positionEcef[2]};
}
sim::InitialConditions atEcef(damage::V3 p) {
    // JSBSim FGInertial defaults, converted from its authored feet. Its ASL
    // initial condition is radial height, not ellipsoidal normal height
    // (FGPropagate::GetAltitudeASL and FGInitialCondition::SetAltitudeASLFtIC).
    constexpr double a = 20925646.32546*.3048, b = 20855486.5951*.3048;
    constexpr double e2 = 1-b*b/(a*a);
    const double horizontal = std::hypot(p.x, p.y);
    double latitude = std::atan2(p.z, horizontal*(1-e2)), radius = a;
    for (int i = 0; i < 12; ++i) {
        radius = a/std::sqrt(1-e2*std::sin(latitude)*std::sin(latitude));
        latitude = std::atan2(p.z+e2*radius*std::sin(latitude), horizontal);
    }
    sim::InitialConditions initial;
    initial.latitudeDeg = latitude/kDeg; initial.longitudeDeg = std::atan2(p.y,p.x)/kDeg;
    const double geocentricLatitude=std::atan2(p.z,horizontal);
    const double cosGeocentric=std::cos(geocentricLatitude);
    const double seaLevelRadius=a*std::sqrt(1-e2)/std::sqrt(1-e2*cosGeocentric*cosGeocentric);
    initial.altitudeMslM = std::hypot(horizontal,p.z)-seaLevelRadius;
    initial.headingDeg = 90; initial.airspeedTrueMs = 250;
    initial.velocityNedMs[0] = 0; initial.velocityNedMs[1] = 250; initial.velocityNedMs[2] = 40;
    return initial;
}

struct Fixture {
    session::World world;
    std::uint32_t missile = 0, victim = 0, formerTarget = 0;
    damage::Airframe geometry;
    damage::V3 wingPoint;
    int wingSection = -1;
    bool jettisoned = false;

    Fixture(int frameSkip = 1, unsigned workers = 1, bool inert = false)
        : world(options(frameSkip, workers)), jettisoned(inert) {
        std::string error;
        const auto frame = damage::readAirframe(std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"f16c/f16c.damage.json", error);
        require(frame.has_value(), "actual F-16C attached geometry unavailable: "+error);
        geometry = *frame;
        wingSection = geometry.section("right wing");
        require(wingSection >= 0, "authored right wing missing");
        bool selected = false;
        for (const auto& panel : geometry.panels) if (panel.surface && panel.section == wingSection) {
            damage::V3 centre;
            bool level = true;
            for (const auto corner : panel.corners) { centre += corner*.25; level = level && corner.z == 0; }
            if (level && centre.y > 3.5 && (!selected || centre.x > wingPoint.x)) {
                wingPoint = centre; selected = true;
            }
        }
        require(selected, "fixture requires an actual finite outboard wing face");
        sim::InitialConditions initial;
        initial.altitudeMslM = 6000; initial.airspeedTrueMs = 250; initial.headingDeg = 90;
        const auto launcher = aircraft(world, "launcher", initial);
        std::uint16_t stations = 0;
        bool found = false;
        for (const auto& p : world.missionSystems(launcher)->payloads()) if (p.family == ms::Family::StoreStation) {
            stations = p.index; found = true;
        }
        require(found, "native stations missing");
        require(world.loadStore(launcher,stations,1,"aim-9x") == ms::Reason::None, "AIM-9X load rejected");
        if (inert) {
            ms::JettisonCommand command; command.payload = stations; command.emergency = true;
            for (const auto& store : world.missionSystems(launcher)->stores())
                if (std::string(store.store) == "aim-9x") command.stores.push_back(store.index);
            require(command.stores.size() == 1, "fixture must select exactly its loaded missile");
            require(world.submit(launcher,command).accepted(), "native jettison rejected");
        } else {
            const auto power = world.submit(launcher,ms::StateCommand{stations,ms::PayloadState::Operate});
            require(power.accepted() || power.reason == ms::Reason::AlreadyInState, "stations power rejected");
            for (unsigned i=0; i<2000 && world.missionSystems(launcher)->payloads()[stations].state!=ms::PayloadState::Operate; ++i)
                world.step();
            require(world.missionSystems(launcher)->payloads()[stations].state == ms::PayloadState::Operate,
                    "station warmup timed out");
            const auto own = *world.vehicleState(launcher);
            initial.latitudeDeg = own.latitudeRad/kDeg;
            initial.longitudeDeg = own.longitudeRad/kDeg+5000/(6371000*std::cos(own.latitudeRad))/kDeg;
            initial.altitudeMslM = own.altitudeMslM;
            formerTarget = aircraft(world,"observed-release-target",initial);
            ms::StrikeCommand command;
            command.payload=stations; command.store="aim-9x"; command.target=formerTarget; command.consent=true;
            require(world.submit(launcher,command).accepted(), "observed infrared Host release rejected");
        }
        for (unsigned i=0; i<2000 && world.weapons().empty(); ++i) world.step();
        require(world.weapons().size()==1, "Host must release exactly one native missile");
        missile = world.weapons().front().id;
        require(record(world,missile).jettisoned == inert, "native release arming provenance changed");
        if (!inert) {
            world.step();
            require(record(world,missile).tracking, "real released seeker must first acquire its on-rail target");
            require(world.removeVehicle(formerTarget), "remove observed target failed");
        }
        require(world.removeVehicle(launcher), "remove distant launcher failed");
    }

    void ageUntilArmed() {
        for (unsigned i=0; i<1000 && world.simTime()-record(world,missile).launchTimeS<1.25; ++i) world.step();
        require(world.alive(missile), "native weapon ended before mechanism setup");
        require(world.simTime()-record(world,missile).launchTimeS>=1.25, "fixture did not pass unchanged one-second arming");
        require(!record(world,missile).tracking, "removed observation must leave the seeker without tracking");
    }

    damage::V3 noseFromOrigin() {
        auto& model = *world.model(missile);
        const auto version = model.property("fsim/model/nose-version");
        require(version.valid() && version.get()==1, "generated authored nose metadata required");
        double nose[3], origin[3];
        const char* axes[] = {"x","y","z"};
        for (int k=0; k<3; ++k) {
            const auto n=model.property(std::string("fsim/model/nose-")+axes[k]+"-in");
            const auto o=model.property(std::string("fsim/model/origin-")+axes[k]+"-in");
            require(n.valid() && o.valid() && std::isfinite(n.get()) && std::isfinite(o.get()), "authored nose frame unavailable");
            nose[k]=n.get(); origin[k]=o.get();
        }
        return {(origin[0]-nose[0])*.0254,(nose[1]-origin[1])*.0254,(origin[2]-nose[2])*.0254};
    }

    damage::V3 relativeNose() {
        const auto missileState=*world.vehicleState(missile), targetState=*world.vehicleState(victim);
        double ownCg[3], targetCg[3];
        require(world.model(missile)->cgFromOriginBodyM(ownCg) && world.model(victim)->cgFromOriginBodyM(targetCg),
                "native CG unavailable");
        const auto nose=position(missileState)+rotate(missileState,noseFromOrigin()-damage::V3{ownCg[0],ownCg[1],ownCg[2]});
        const auto origin=position(targetState)-rotate(targetState,{targetCg[0],targetCg[1],targetCg[2]});
        const auto delta=nose-origin;
        const auto* r=targetState.rotationBodyToEcef;
        return {r[0]*delta.x+r[3]*delta.y+r[6]*delta.z,r[1]*delta.x+r[4]*delta.y+r[7]*delta.z,
                r[2]*delta.x+r[5]*delta.y+r[8]*delta.z};
    }

    void placeCrossing(bool nearMiss=false, bool detached=false, bool requireShift=false) {
        sim::InitialConditions targetInitial;
        targetInitial.altitudeMslM=6000; targetInitial.airspeedTrueMs=250; targetInitial.headingDeg=90;
        if (!victim) victim=aircraft(world,"physical-contact-victim",targetInitial);
        if (detached) {
            require(world.separateSection(victim,"right wing"), "actual wing separation failed");
            bool piece=false;
            for (const auto id : world.vehicleIds()) if (world.info(id)->role==ms::Role::Debris) piece=true;
            require(piece,"separation must create a real detached body for the excluded-role control");
        }
        const auto energy=world.model(missile)->energy();
        require(std::isfinite(energy.fuelKg) && energy.fuelCapacityKg>0,"native missile fuel state unavailable");
        // Preserve the actually reached propellant fraction while repositioning;
        // resetting to a full motor would erase the natural nonzero CG shift.
        auto initial=targetInitial;
        initial.fuelFraction=energy.fuelKg/energy.fuelCapacityKg;
        initial.enginesRunning=!jettisoned;
        initial.velocityNedMs[0]=0; initial.velocityNedMs[1]=250; initial.velocityNedMs[2]=40;
        require(world.resetVehicle(missile,&initial),"native mechanism reset failed");
        auto contactPoint=wingPoint;
        if (nearMiss) {
            double maximumY=0;
            for (const auto& panel : geometry.panels) {
                if (panel.surface) for(const auto corner:panel.corners) maximumY=std::max(maximumY,corner.y);
                else maximumY=std::max(maximumY,panel.centre.y+panel.half.y);
            }
            for (const auto& component : geometry.components) maximumY=std::max(maximumY,component.centre.y+component.half.y);
            contactPoint.y=maximumY+1;
        }
        contactPoint.z=-.1;
        const auto target=*world.vehicleState(victim), own=*world.vehicleState(missile);
        double ownCg[3], targetCg[3];
        require(world.model(missile)->cgFromOriginBodyM(ownCg) && world.model(victim)->cgFromOriginBodyM(targetCg),
                "native mechanism CG unavailable");
        if (requireShift) require(std::hypot(ownCg[0],ownCg[1],ownCg[2])>.001,
                                  "armed contact fixture must exercise the naturally burnt nonzero missile CG");
        const auto targetOrigin=position(target)-rotate(target,{targetCg[0],targetCg[1],targetCg[2]});
        const auto desiredNose=targetOrigin+rotate(target,contactPoint);
        const auto desiredCg=desiredNose-rotate(own,noseFromOrigin()-damage::V3{ownCg[0],ownCg[1],ownCg[2]});
        auto exact=atEcef(desiredCg);
        exact.fuelFraction=initial.fuelFraction; exact.enginesRunning=initial.enginesRunning;
        require(world.resetVehicle(missile,&exact),"exact authored nose placement failed");
        const auto placed=relativeNose();
        require(std::abs(placed.x-contactPoint.x)<.001 && std::abs(placed.y-contactPoint.y)<.001 && std::abs(placed.z+.1)<.001,
                "independent nose reconstruction must reach the selected authored face within one millimetre; actual="+
                std::to_string(placed.x)+","+std::to_string(placed.y)+","+std::to_string(placed.z)+" wanted="+
                std::to_string(contactPoint.x)+","+std::to_string(contactPoint.y)+",-0.1 CG_error_m="+
                std::to_string(damage::length(position(*world.vehicleState(missile))-desiredCg)));
        const auto& state=*world.vehicleState(missile);
        const auto toTarget=position(target)-position(state);
        const auto forward=rotate(state,{1,0,0});
        require(damage::dot(toTarget,forward)/damage::length(toTarget)<std::cos(45*kDeg),
                "physical victim must be outside unchanged infrared seeker gimbal");
    }

    void assertNoDetonation() {
        require(world.alive(missile) && record(world,missile).end==ms::WeaponEnd::InFlight,
                "negative contact control must remain in flight");
        for (const auto& event : world.engagementEvents()) if (event.entity==missile)
            require(event.kind!="detonation","negative contact control emitted detonation");
    }
};

std::array<double,3> armed(int skip, unsigned workers) {
    Fixture fixture(skip,workers);
    fixture.ageUntilArmed();
    fixture.placeCrossing(false,false,true);
    fixture.world.step();
    const auto final=record(fixture.world,fixture.missile);
    require(!final.tracking,"physical contact must not require reacquiring the seeker target");
    require(final.end==ms::WeaponEnd::Detonated && !fixture.world.alive(fixture.missile),
            "armed lost-seeker finite-face crossing must detonate through native World");
    unsigned count=0;
    std::array<double,3> at{};
    for (const auto& event : fixture.world.engagementEvents()) if (event.entity==fixture.missile && event.kind=="detonation") {
        ++count;
        require(event.target==fixture.victim && event.reason=="contact", "contact detonation must retain actual physical victim provenance");
        std::copy_n(event.positionEcef,3,at.begin());
    }
    require(count==1,"one contact crossing must emit exactly one detonation");
    fixture.world.step(2);
    unsigned later=0;
    for (const auto& event : fixture.world.engagementEvents()) if (event.entity==fixture.missile && event.kind=="detonation") ++later;
    require(later==1,"later steps must not duplicate contact initiation");
    return at;
}
void armedAndWorkers() {
    const auto first=armed(1,1), second=armed(1,2);
    require(first==second,"worker count must preserve deterministic contact burst coordinates");
}
void batched() { armed(8,2); }
void unarmed() {
    Fixture fixture;
    fixture.placeCrossing();
    fixture.world.step();
    fixture.assertNoDetonation();
    require(fixture.relativeNose().z>.05,"unarmed point must actually cross the finite wing plane");
    fixture.ageUntilArmed();
    fixture.placeCrossing(); // Same released weapon; a reset is not a new release.
    fixture.world.step();
    fixture.assertNoDetonation();
    require(fixture.relativeNose().z>.05,"second crossing must be real after arming");
}
void inert() {
    Fixture fixture(1,1,true);
    fixture.ageUntilArmed(); fixture.placeCrossing(); fixture.world.step(); fixture.assertNoDetonation();
    require(record(fixture.world,fixture.missile).jettisoned,"contact must preserve inert jettison provenance");
    require(fixture.relativeNose().z>.05,"inert point must actually cross the finite wing plane");
    require(fixture.world.damage(fixture.victim)==nullptr,"inert contact probe must not create damage lifecycle state");
}
void nearMiss() {
    Fixture fixture;
    fixture.ageUntilArmed(); fixture.placeCrossing(true); fixture.world.step(); fixture.assertNoDetonation();
    require(fixture.world.damage(fixture.victim)==nullptr,"near miss must not lazily create damage or RNG state");
}
void detached() {
    Fixture fixture;
    fixture.ageUntilArmed(); fixture.placeCrossing(false,true); fixture.world.step(); fixture.assertNoDetonation();
    require(fixture.relativeNose().z>.05,"point must cross the absent authored wing plane");
}
} // namespace

int main(int argc,char** argv) {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]={{"armed lost seeker and worker determinism",armedAndWorkers},
                        {"batched native point contact",batched},{"unarmed first contact stays consumed",unarmed},
                        {"inert jettison physical crossing",inert},{"finite geometry near miss",nearMiss},
                        {"detached geometry excluded",detached}};
    unsigned executed=0, failures=0;
    for (const auto& test:tests) {
        if (argc>1 && std::string(test.name).find(argv[1])==std::string::npos) continue;
        ++executed;
        try {test.run();std::cout<<"PASS: "<<test.name<<'\n';}
        catch(const std::exception& error) {++failures;std::cerr<<"FAIL: "<<test.name<<": "<<error.what()<<'\n';}
    }
    if (!executed) {std::cerr<<"No native contact cases matched\n";return 2;}
    return failures ? 1 : 0;
}
