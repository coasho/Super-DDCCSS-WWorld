// Named mechanism fixtures: real Host releases, generated stores and native
// flight. Repositioning constructs a finite wing crossing; no original range
// scenario, fuze value, force or aerodynamic coefficient is changed.
#include "simulation/runtime/world/World.h"
#include "simulation/flight/FlightModel.h"
#include "simulation/flight/JsbsimModel.h"
#include <FGFDMExec.h>
#include <models/FGMassBalance.h>
#include <models/FGPropulsion.h>
#include "foundation/core/Json.h"
#include "simulation/mission_systems/Cards.h"
#include "foundation/io/AssetResolver.h"
#include "simulation/mission_systems/damage/Impact.h"
#include "simulation/mission_systems/damage/State.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <set>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
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
void unarmedPhysicalImpulse() {
    Fixture fixture;
    fixture.placeCrossing();
    fixture.world.step();
    fixture.assertNoDetonation();
    const auto* damage = fixture.world.damage(fixture.victim);
    require(damage != nullptr, "unarmed physical contact must assess mechanical loading independently of a warhead");
    require(std::any_of(damage->sections().begin(), damage->sections().end(), [](const auto& section) {
        return section.kick > 0 || section.separated;
    }), "a real closing body contact must reach native structural loading without fragment Hit fabrication");
}
void unarmed() {
    Fixture fixture;
    fixture.placeCrossing();
    fixture.world.step();
    fixture.assertNoDetonation();
    require(fixture.relativeNose().z>.05,"unarmed point must actually cross the finite wing plane");
    fixture.ageUntilArmed();
    require(fixture.world.resetVehicle(fixture.victim),"restore physical victim geometry before testing consumed channel");
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
    const auto* damage=fixture.world.damage(fixture.victim);
    require(damage!=nullptr,"inert body still transfers kinetic impulse without a warhead");
    require(std::any_of(damage->sections().begin(),damage->sections().end(),[](const auto& section) {
        return section.kick>0 || section.separated;
    }),"inert kinetic contact must load actual structure rather than create a fragment Hit");
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

// This observation reads values already emitted by the actual native World.
// It never reconstructs a hit, supplies component work, or changes a source.
struct MaterialObservation {
    std::string previous, prefix;
    bool hadPrevious=false;
    MaterialObservation() {
        if(const auto* value=std::getenv("FSIM_ENGAGEMENT_TRACE")) {previous=value;hadPrevious=true;}
        const auto serial=std::chrono::steady_clock::now().time_since_epoch().count();
        const auto directory=std::filesystem::temp_directory_path()/"fsim-material-coverage";
        std::filesystem::create_directories(directory);
        prefix=(directory/("native-"+std::to_string(serial))).generic_string();
        set(prefix);
    }
    static void set(const std::string& value) {
#ifdef _WIN32
        require(_putenv_s("FSIM_ENGAGEMENT_TRACE",value.c_str())==0,"cannot bind material observer");
#else
        require(setenv("FSIM_ENGAGEMENT_TRACE",value.c_str(),1)==0,"cannot bind material observer");
#endif
    }
    ~MaterialObservation() {
#ifdef _WIN32
        _putenv_s("FSIM_ENGAGEMENT_TRACE",hadPrevious?previous.c_str():"");
#else
        if(hadPrevious)setenv("FSIM_ENGAGEMENT_TRACE",previous.c_str(),1);else unsetenv("FSIM_ENGAGEMENT_TRACE");
#endif
    }
    void check(std::uint32_t sourceId,const std::string& store,std::uint32_t targetId,
               double boundary,bool contactBirth) const {
        using J=core::Json;
        const auto path=prefix+"-effects-"+store+"-"+std::to_string(sourceId)+".jsonl";
        require(std::filesystem::is_regular_file(path),"native effects observation missing: "+path);
        require(std::filesystem::file_size(path)<=64u*1024u*1024u,"native effects observation exceeds byte bound");
        std::ifstream stream(path,std::ios::binary);
        J source,first;bool sourceSeen=false,firstSeen=false;std::string line;
        while(std::getline(stream,line)) {
            require(line.size()<=8u*1024u*1024u,"native observer row exceeds byte bound");
            const auto row=J::parse(line,path);
            require(row.string("schema","")=="fsim.sim-m03.effects-observation.v1","wrong native observer schema");
            if(row.string("kind","")=="source") {
                require(!sourceSeen,"one physical source must not be emitted twice");source=row;sourceSeen=true;
            } else if(row.string("kind","")=="assessment"&&!firstSeen) {
                require(sourceSeen,"assessment precedes actual source");first=row;firstSeen=true;
            }
        }
        require(!stream.bad()&&sourceSeen&&firstSeen,"source and first native assessment required");
        require(source.boolean("field_valid",false)&&first.boolean("valid",false),"actual field/assessment refused");
        require(source.string("weapon","")==std::to_string(sourceId)&&first.string("weapon","")==std::to_string(sourceId),"foreign source observation");
        require(source.string("generation","")==first.string("generation",""),"source generation changed");
        require(first.number("world_time_s",-1)==boundary,"assessment must run at actual contact boundary");
        require(first.number("field_age_before_s",-1)==0&&first.number("duration_s",0)>0,"first native interval missing");
        require(first.string("input_projectiles","")==source.string("field_emitted_count",""),"source population changed before first interval");
        if(contactBirth)require(source.number("burst_time_s",-1)<boundary&&source.number("world_time_s",-1)==boundary,
                               "actual contact must precede the outer-step boundary");
        else require(source.number("world_time_s",-1)<boundary,"older field must already be pending before contact");
        const auto findTarget=[&](const J& document)->const J& {
            require(document.child("targets").isArray(),"native target list unavailable");
            const J* found=nullptr;
            for(const auto& row:document.child("targets").asArray())if(row.string("identity","")==std::to_string(targetId)) {
                require(!found,"duplicate material owner");found=&row;
            }
            require(found!=nullptr,"original material owner absent from first native interval");return *found;
        };
        const auto& before=findTarget(source);const auto& assessed=findTarget(first);
        require(before.string("generation","")==assessed.string("generation",""),"recipient generation changed");
        const auto ids=[](const J& geometry,const char* key) {
            require(geometry.child(key).isArray(),"native attached material list unavailable");std::set<std::string> result;
            for(const auto& id:geometry.child(key).asArray()) {
                require(id.isString()&&result.insert(id.asString()).second,"duplicate or invalid material index");
            }
            return result;
        };
        for(const auto* key:{"attached_panels","attached_components"}) {
            const auto expected=ids(before.child("geometry"),key),actual=ids(assessed.child("geometry"),key);
            require(!expected.empty(),"fixture requires actual original authored material");
            for(const auto& id:expected)require(actual.count(id)==1,
                std::string("unassessed pre-boundary material source=")+std::to_string(sourceId)+" target="+std::to_string(targetId)+" "+key+"="+id+" observation="+path);
        }
        std::cout<<"MATERIAL_COVERAGE source="<<sourceId<<" target="<<targetId<<" first_interval_s="<<first.number("duration_s",0)<<" path="<<path<<'\n';
    }
};

// Generic high-speed native collision fixture, separate from the unchanged
// engagement scenes. It uses the actual authored top face of nose body44.
// No material, strength, force, source or fuze parameter is changed.
void verifyOpeningNoseRootDrive(Fixture& fixture,damage::V3 facePointBody) {
    sim::VehicleState source,target;
    fixture.world.model(fixture.missile)->state(source);fixture.world.model(fixture.victim)->state(target);
    double targetCg[3];require(fixture.world.model(fixture.victim)->cgFromOriginBodyM(targetCg),"actual opening target CG unavailable");
    const auto point=position(target)+rotate(target,facePointBody-damage::V3{targetCg[0],targetCg[1],targetCg[2]});
    const auto body=[&](std::uint32_t id,const sim::VehicleState& state) {
        const auto* provider=dynamic_cast<const sim::RigidMassSourceV1*>(fixture.world.model(id));sim::RigidMassPropertiesV1 mass;
        require(provider&&provider->rigidMassPropertiesV1(mass),"actual opening full native inertia unavailable");
        damage::ContactBody result;result.massKg=mass.massKg;
        result.velocity=rotate(state,{state.velocityBodyMs[0],state.velocityBodyMs[1],state.velocityBodyMs[2]});
        result.angularVelocity=rotate(state,{state.angularRateBodyRadS[0],state.angularRateBodyRadS[1],state.angularRateBodyRadS[2]});
        result.arm=point-position(state);const auto* r=state.rotationBodyToEcef;
        for(int row=0;row<3;++row)for(int col=0;col<3;++col)for(int i=0;i<3;++i)for(int j=0;j<3;++j)
            result.inertia[3*row+col]+=r[3*row+i]*mass.inertiaBodyKgM2[3*i+j]*r[3*col+j];
        return result;
    };
    const auto a=body(fixture.missile,source),b=body(fixture.victim,target);
    const auto predicted=damage::inelasticBodyContact(a,b,rotate(target,{0,0,-1}));
    require(predicted.valid&&predicted.approaching,"opening native contact prediction must be valid and closing");
    const auto* r=target.rotationBodyToEcef;const auto& j=predicted.impulseB;
    const damage::V3 impulseBody{r[0]*j.x+r[3]*j.y+r[6]*j.z,r[1]*j.x+r[4]*j.y+r[7]*j.z,r[2]*j.x+r[5]*j.y+r[8]*j.z};
    const int nose=fixture.geometry.section("nose");const auto& section=fixture.geometry.sections[static_cast<std::size_t>(nose)];
    double reach=1;
    for(const auto& panel:fixture.geometry.panels)if(panel.section==nose)reach=std::max(reach,damage::length(panel.middle()-section.root));
    const double frequency=std::sqrt(fixture.geometry.nUltimate[1]*9.80665/(damage::kTipDeflection*reach));
    require(std::isfinite(frequency)&&frequency>0,"authored nose bending frequency unavailable");
    const auto moment=damage::cross(facePointBody-section.root,impulseBody);
    double ratio=0,requiredImpulse=std::numeric_limits<double>::infinity();
    for(std::size_t axis=0;axis<section.axes.size()&&axis<section.strengthNm.size();++axis) {
        require(section.strengthNm[axis]>0,"authored nose strength must stay positive");
        ratio=std::max(ratio,std::abs(damage::dot(moment,section.axes[axis]))*frequency/section.strengthNm[axis]);
        const double lever=std::abs(damage::dot(damage::cross(facePointBody-section.root,{0,0,1}),section.axes[axis]));
        if(lever>0)requiredImpulse=std::min(requiredImpulse,section.strengthNm[axis]/(lever*frequency));
    }
    // This is an OPENING-POSE prediction from the actual native tensor, not a
    // substitute for the later C contact solve or the actual separation check.
    require(ratio>1,"chosen actual opening pose does not overcome unchanged authored nose root strength");
    std::cout<<"NOSE_OPENING_SETUP source_mass_kg="<<a.massKg<<" target_mass_kg="<<b.massKg
             <<" effective_mass_kg="<<predicted.effectiveMassKg<<" closing_ms="<<predicted.closingMs
             <<" predicted_impulse_ns="<<damage::length(predicted.impulseB)<<" required_normal_impulse_ns="<<requiredImpulse
             <<" root_drive_ratio="<<ratio<<" source_pitch_rad="<<source.eulerRad[1]<<'\n';
}

void placeNoseMaterialCrossing(Fixture& fixture) {
    require(fixture.geometry.panels.size()>44,"authored nose body44 unavailable");
    const auto& panel=fixture.geometry.panels[44];
    const int nose=fixture.geometry.section("nose");
    require(nose>=0&&panel.section==nose&&!panel.surface&&panel.half.z>0,
            "fixture requires actual F-16C finite nose body44");
    const auto& section=fixture.geometry.sections[static_cast<std::size_t>(nose)];
    require(section.hasRoot&&!section.strengthNm.empty()&&section.strengthNm[0]>0,
            "fixture must retain authored finite nose root strength");
    sim::InitialConditions targetInitial;
    targetInitial.altitudeMslM=6000;targetInitial.airspeedTrueMs=250;targetInitial.headingDeg=90;
    if(!fixture.victim)fixture.victim=aircraft(fixture.world,"nose-material-contact-victim",targetInitial);
    require(fixture.victim!=fixture.formerTarget,"physical collision body must differ from former observed release target");
    const auto energy=fixture.world.model(fixture.missile)->energy();
    require(std::isfinite(energy.fuelKg)&&energy.fuelCapacityKg>0,"actual released missile propellant unavailable");
    auto initial=targetInitial;
    initial.fuelFraction=energy.fuelKg/energy.fuelCapacityKg;initial.enginesRunning=!fixture.jettisoned;
    initial.pitchDeg=-80;
    initial.velocityNedMs[0]=0;initial.velocityNedMs[1]=250;initial.velocityNedMs[2]=400;
    require(fixture.world.resetVehicle(fixture.missile,&initial),"native nose fixture reset refused");
    auto contactPoint=panel.centre;contactPoint.z-=panel.half.z+.1;
    const auto target=*fixture.world.vehicleState(fixture.victim),own=*fixture.world.vehicleState(fixture.missile);
    double ownCg[3],targetCg[3];
    require(fixture.world.model(fixture.missile)->cgFromOriginBodyM(ownCg)&&
            fixture.world.model(fixture.victim)->cgFromOriginBodyM(targetCg),"actual crossing CG unavailable");
    const auto targetOrigin=position(target)-rotate(target,{targetCg[0],targetCg[1],targetCg[2]});
    const auto desiredNose=targetOrigin+rotate(target,contactPoint);
    const auto desiredCg=desiredNose-rotate(own,fixture.noseFromOrigin()-damage::V3{ownCg[0],ownCg[1],ownCg[2]});
    auto exact=atEcef(desiredCg);exact.fuelFraction=initial.fuelFraction;exact.enginesRunning=initial.enginesRunning;
    exact.pitchDeg=initial.pitchDeg;
    for(int k=0;k<3;++k)exact.velocityNedMs[k]=initial.velocityNedMs[k];
    require(fixture.world.resetVehicle(fixture.missile,&exact),"actual finite nose-face placement refused");
    const auto placed=fixture.relativeNose();
    require(damage::length(placed-contactPoint)<.001,"native reconstructed nose must start .1m above the actual finite face");
    const auto& actual=*fixture.world.vehicleState(fixture.missile);
    require(std::abs(actual.velocityNedMs[2]-400)<1e-8,"native fixture did not adopt its actual crossing velocity");
    const auto toTarget=position(target)-position(actual),forward=rotate(actual,{1,0,0});
    require(damage::dot(toTarget,forward)/damage::length(toTarget)<std::cos(45*kDeg),
            "physical body must remain outside the unchanged seeker gimbal");
    require(!record(fixture.world,fixture.missile).tracking,"physical nose contact must not use a synthesized seeker observation");
    require(std::abs(actual.eulerRad[1]+80*kDeg)<1e-9,"native source must retain the declared new-fixture opening pitch");
    auto facePoint=panel.centre;facePoint.z-=panel.half.z;
    verifyOpeningNoseRootDrive(fixture,facePoint);
}

void requireActualNoseDeparture(Fixture& fixture) {
    const int nose=fixture.geometry.section("nose");
    const auto* state=fixture.world.damage(fixture.victim);
    require(state&&nose>=0&&static_cast<std::size_t>(nose)<state->sections().size()&&
            state->sections()[static_cast<std::size_t>(nose)].separated,
            "actual authored nose must separate; no strength or outcome condition may be weakened");
    bool nosePiece=false;
    for(const auto id:fixture.world.vehicleIds()) {
        const auto* info=fixture.world.info(id);
        if(info->role==ms::Role::Debris&&info->launcher==fixture.victim&&
           info->type.find(":nose:")!=std::string::npos) {
            nosePiece=true;
            require(info->launchTimeS==fixture.world.simTime(),"actual nose child birth must remain boundary C");
        }
    }
    require(nosePiece,"real generated nose child must exist, not just a separated-state flag");
}

void inertNoseContactMaterialControl() {
    Fixture fixture(1,1,true);fixture.ageUntilArmed();placeNoseMaterialCrossing(fixture);
    fixture.world.step();fixture.assertNoDetonation();requireActualNoseDeparture(fixture);
    bool mechanical=false;
    for(const auto& event:fixture.world.engagementEvents())if(event.entity==fixture.missile&&event.target==fixture.victim&&
        event.kind=="damage"&&event.reason=="body_contact")mechanical=true;
    require(mechanical,"inert control must prove actual mechanical contact without a warhead source");
}

void noseContactBoundaryMaterialCoverage() {
    Fixture fixture;fixture.ageUntilArmed();placeNoseMaterialCrossing(fixture);
    MaterialObservation observation;fixture.world.step();
    require(record(fixture.world,fixture.missile).end==ms::WeaponEnd::Detonated,
            "actual armed finite nose crossing must detonate through native contact");
    bool actualContact=false;
    for(const auto& event:fixture.world.engagementEvents())if(event.entity==fixture.missile&&event.target==fixture.victim&&
        event.kind=="detonation")actualContact=event.reason=="contact";
    require(actualContact,"native contact provenance required; a proximity result cannot substitute");
    requireActualNoseDeparture(fixture);
    observation.check(fixture.missile,"aim-9x",fixture.victim,fixture.world.simTime(),true);
}

void olderFieldNoseBoundaryMaterialCoverage() {
    Fixture fixture;fixture.ageUntilArmed();
    fixture.placeCrossing(true); // Safely park beyond all material during actual source-history capture.
    ms::WeaponLaunch release;release.type="jsbsim:aim120c";release.offsetBodyM[1]=30;
    const auto older=fixture.world.launchWeapon(fixture.victim,release);
    require(older&&older!=fixture.missile,"older field requires a distinct actual native source");
    fixture.world.step(2);fixture.assertNoDetonation();
    placeNoseMaterialCrossing(fixture);
    MaterialObservation observation;
    const auto source=*fixture.world.vehicleState(older);
    session::Detonation burst;burst.weapon=older;burst.store="aim-120c";burst.stepFraction=1;
    const auto axis=rotate(source,{1,0,0}),velocity=rotate(source,{source.velocityBodyMs[0],source.velocityBodyMs[1],source.velocityBodyMs[2]});
    for(int k=0;k<3;++k) {burst.positionEcef[k]=source.positionEcef[k];burst.velocityEcef[k]=(&velocity.x)[k];burst.axisEcef[k]=(&axis.x)[k];}
    require(!fixture.world.detonateCapturedV1(burst).empty(),"older actual captured field refused");
    fixture.world.step();requireActualNoseDeparture(fixture);
    require(record(fixture.world,fixture.missile).end==ms::WeaponEnd::Detonated,"actual armed nose contact must detonate");
    bool contact=false;
    for(const auto& event:fixture.world.engagementEvents())if(event.entity==fixture.missile&&event.target==fixture.victim&&event.kind=="detonation")
        contact=event.reason=="contact";
    require(contact,"actual nose contact provenance cannot be replaced by proximity");
    observation.check(fixture.missile,"aim-9x",fixture.victim,fixture.world.simTime(),true);
    observation.check(older,"aim-120c",fixture.victim,fixture.world.simTime(),false);
}

// A real Host rail release at the SAME C as a mechanical contact. The witness
// records actual native application state; it does not simulate a replacement
// contact or label arbitrary before-step flight energy as contact work.
void sameBoundaryContactRailRelease() {
    using J=core::Json;
    Fixture fixture(1,1,true);fixture.ageUntilArmed();
    sim::InitialConditions targetInitial;
    targetInitial.altitudeMslM=6000;targetInitial.airspeedTrueMs=250;targetInitial.headingDeg=90;
    fixture.victim=aircraft(fixture.world,"same-C-release-recipient",targetInitial);
    const auto& panel=fixture.geometry.panels.at(51);
    require(!panel.surface&&panel.section==fixture.geometry.section("centre")&&
            !fixture.geometry.sections[static_cast<std::size_t>(panel.section)].hasRoot,
            "release fixture requires actual finite rootless centre body51");
    std::uint16_t payload=0;bool stationPayload=false;
    for(const auto& value:fixture.world.missionSystems(fixture.victim)->payloads())if(value.family==ms::Family::StoreStation) {
        payload=value.index;stationPayload=true;
    }
    require(stationPayload,"actual recipient station payload unavailable");
    require(fixture.world.loadStore(fixture.victim,payload,1,"aim-9x")==ms::Reason::None,"actual rail store load refused");
    std::uint16_t storeIndex=0;bool selected=false;double carriedStoreKg=0;
    for(const auto& store:fixture.world.missionSystems(fixture.victim)->stores())if(store.station==1&&std::string(store.store)=="aim-9x") {
        require(!selected,"fixture must select one actual rail store");storeIndex=store.index;selected=true;
        require(std::string(store.carriage)=="rail"&&store.massKg>0,"actual zero-ejection rail carriage required");
        carriedStoreKg=store.massKg;
    }
    require(selected,"actual loaded rail store unavailable");
    const auto library=io::AssetResolver{}.find("payloads");require(library.has_value(),"actual payload library unavailable");
    std::vector<std::string> errors;
    const auto fit=ms::readFit(std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"f16c/payloads.json",*library,errors);
    require(fit!=nullptr,"actual recipient fit unavailable");
    const ms::Card* stationCard=nullptr;
    for(const auto& entry:fit->payloads)if(entry.name=="stations") {
        const auto found=fit->cards.find(entry.card);if(found!=fit->cards.end())stationCard=found->second.get();
    }
    require(stationCard&&stationCard->family==ms::Family::StoreStation,"authored station card binding unavailable");
    const auto* station=stationCard->station(1);
    require(station&&station->carriage.find("rail")!=std::string::npos&&station->extractionMs==0,
            "fixture requires actual authored rail with no extraction impulse");

    const auto fuel=fixture.world.model(fixture.missile)->energy();
    require(fuel.fuelCapacityKg>0&&std::isfinite(fuel.fuelKg),"actual inert source fuel state unavailable");
    auto initial=targetInitial;initial.enginesRunning=false;initial.fuelFraction=fuel.fuelKg/fuel.fuelCapacityKg;
    initial.velocityNedMs[0]=0;initial.velocityNedMs[1]=250;initial.velocityNedMs[2]=40;
    require(fixture.world.resetVehicle(fixture.missile,&initial),"native release-contact setup refused");
    auto point=panel.centre;point.z-=panel.half.z+.1;
    const auto target=*fixture.world.vehicleState(fixture.victim),own=*fixture.world.vehicleState(fixture.missile);
    double ownCg[3],targetCg[3];
    require(fixture.world.model(fixture.missile)->cgFromOriginBodyM(ownCg)&&fixture.world.model(fixture.victim)->cgFromOriginBodyM(targetCg),
            "actual release-contact CG unavailable");
    const auto origin=position(target)-rotate(target,{targetCg[0],targetCg[1],targetCg[2]});
    const auto desiredNose=origin+rotate(target,point);
    const auto desiredCg=desiredNose-rotate(own,fixture.noseFromOrigin()-damage::V3{ownCg[0],ownCg[1],ownCg[2]});
    auto exact=atEcef(desiredCg);exact.fuelFraction=initial.fuelFraction;exact.enginesRunning=false;
    for(int k=0;k<3;++k)exact.velocityNedMs[k]=initial.velocityNedMs[k];
    require(fixture.world.resetVehicle(fixture.missile,&exact),"native finite centre-face placement refused");
    require(damage::length(fixture.relativeNose()-point)<.001,"actual nose must start above selected finite centre face");
    ms::JettisonCommand command;command.payload=payload;command.emergency=true;command.stores={storeIndex};
    require(fixture.world.submit(fixture.victim,command).accepted(),"actual same-C Host jettison refused");
    auto* parentNative=dynamic_cast<sim::JsbsimModel*>(fixture.world.model(fixture.victim));
    require(parentNative,"fixture requires its actual JSBSim parent");
    const auto massBalance=parentNative->exec().GetMassBalance();
    const auto propulsion=parentNative->exec().GetPropulsion();
    require(parentNative->exec().GetFDMCount()==0&&massBalance->in.GasMass==0,
            "explicit material sum requires no mated child or gas mass");
    constexpr double kgPerLb=.45359237;
    const double openingFuelKg=propulsion->GetTanksWeight()*kgPerLb;
    const double openingMaterialKg=(massBalance->GetEmptyWeight()+massBalance->GetTotalPointMassWeight())*kgPerLb+openingFuelKg;
    require(fixture.world.missionSystems(fixture.victim)->stationMassKg(payload,1)==carriedStoreKg,
            "queued release must retain its actual material on the station");
    MaterialObservation observation;fixture.world.step();fixture.assertNoDetonation();
    bool mechanical=false;unsigned nativeBursts=0;
    for(const auto& event:fixture.world.engagementEvents()) {
        if(event.entity==fixture.missile&&event.target==fixture.victim&&event.kind=="damage"&&event.reason=="body_contact")mechanical=true;
        if(event.kind=="detonation")++nativeBursts;
    }
    require(mechanical&&nativeBursts==0,"release fixture requires actual mechanical contact without a warhead source");
    const auto* material=fixture.world.damage(fixture.victim);require(material!=nullptr,"real contact response unavailable");
    require(std::none_of(material->sections().begin(),material->sections().end(),[](const auto& section){return section.separated;}),
            "rootless release fixture must retain its actual whole recipient");
    std::uint32_t released=0;
    for(const auto& value:fixture.world.weapons())if(value.launcher==fixture.victim&&value.store=="aim-9x"&&value.jettisoned&&
        value.launchTimeS==fixture.world.simTime()) {require(!released,"one Host store must release exactly once");released=value.id;}
    require(released!=0,"actual Host rail release must occur at the contact boundary");
    std::uint64_t releaseSerial=0;
    for(const auto& event:fixture.world.engagementEvents())if(event.entity==released&&event.kind=="launch") {
        require(!releaseSerial,"actual launch event must not repeat");releaseSerial=event.serial;
    }
    require(releaseSerial!=0,"actual rail launch event unavailable");
    const auto path=observation.prefix+"-effects-aim-9x-"+std::to_string(fixture.missile)+".jsonl";
    require(std::filesystem::is_regular_file(path)&&std::filesystem::file_size(path)<=64u*1024u*1024u,"bounded native response observation missing");
    std::ifstream stream(path,std::ios::binary);std::string line;J application;unsigned applications=0;
    while(std::getline(stream,line)) {
        require(line.size()<=8u*1024u*1024u,"native response row exceeds byte bound");
        const auto row=J::parse(line,path);
        require(row.string("schema","")=="fsim.sim-m03.effects-observation.v1","wrong native response schema");
        if(row.string("kind","")!="response_application")continue;
        if(row.string("target","")!=std::to_string(fixture.victim))continue;
        require(row.string("weapon","")==std::to_string(fixture.missile)&&
                row.string("generation","")==std::to_string(fixture.world.info(fixture.missile)->generation)&&
                row.string("target_generation","")==std::to_string(fixture.world.info(fixture.victim)->generation),
                "native response lifetime identity mismatch");
        require(row.number("world_time_s",-1)==fixture.world.simTime()&&row.string("phase","")=="native_post_apply",
                "native application and release must share the actual boundary epoch");
        application=row;++applications;
    }
    require(!stream.bad()&&applications==1,"one actual native recipient application required");
    require(application.boolean("pose_available",false)&&application.boolean("cg_available",false)&&
            application.boolean("mass_available",false),"native application pose/CG/mass unavailable");
    const auto serial=application.string("next_combat_serial","");
    require(!serial.empty()&&serial.size()<=20&&std::all_of(serial.begin(),serial.end(),[](char c){return c>='0'&&c<='9';}),
            "native serial must be an unsigned decimal identity");
    const auto nextSerial=std::stoull(serial);
    // A snapshot is not called pre-fly merely because it exists. The ACTUAL
    // subsequent launch serial must prove that ordering independently.
    require(releaseSerial>=nextSerial,"native contact response happened after same-C release inheritance");
    const auto vector=[](const J& doc,const char* key) {
        const auto& a=doc.child(key);require(a.isArray()&&a.asArray().size()==3,std::string("native vector unavailable: ")+key);
        damage::V3 out;
        for(std::size_t k=0;k<3;++k){require(a.asArray()[k].isNumber()&&std::isfinite(a.asArray()[k].asNumber()),"nonfinite native vector");(&out.x)[k]=a.asArray()[k].asNumber();}
        return out;
    };
    const auto toNed=[](damage::V3 euler,damage::V3 body) {
        // Independent successive rigid rotations Rz(yaw) Ry(pitch) Rx(roll).
        const double cr=std::cos(euler.x),sr=std::sin(euler.x),cp=std::cos(euler.y),sp=std::sin(euler.y),cy=std::cos(euler.z),sy=std::sin(euler.z);
        const damage::V3 roll{body.x,cr*body.y-sr*body.z,sr*body.y+cr*body.z};
        const damage::V3 pitch{cp*roll.x+sp*roll.z,roll.y,-sp*roll.x+cp*roll.z};
        return damage::V3{cy*pitch.x-sy*pitch.y,sy*pitch.x+cy*pitch.y,pitch.z};
    };
    const auto cg=vector(application,"application_cg_from_origin_body_m");
    const damage::V3 offset{station->positionM[0]-cg.x,station->positionM[1]-cg.y,station->positionM[2]-cg.z};
    const auto preOmega=vector(application,"pre_impulse_angular_rate_body_rad_s"),postOmega=vector(application,"angular_rate_body_rad_s");
    const auto before=vector(application,"pre_impulse_velocity_ned_ms")+
        toNed(vector(application,"pre_impulse_euler_rad"),damage::cross(preOmega,offset));
    const auto expected=vector(application,"velocity_ned_ms")+
        toNed(vector(application,"euler_rad"),damage::cross(postOmega,offset));
    // The raw native NED getter can currently lag SetUVW. Do not silently
    // replace it and then claim the old source law was physically complete.
    // Record the independent body-to-NED coherence defect separately; this
    // regression closes the unchanged native rail law and ordering only.
    const auto derivedNed=toNed(vector(application,"euler_rad"),vector(application,"velocity_body_ms"));
    const double sdkNedDisagreement=damage::length(derivedNed-vector(application,"velocity_ned_ms"));
    const auto rawNedDisagreement=application.boolean("raw_native_cache_ned_available",false)?
        std::to_string(damage::length(derivedNed-vector(application,"raw_native_cache_ned_ms"))):std::string("unavailable");
    sim::VehicleState actual;fixture.world.model(released)->state(actual);
    constexpr double velocityTolerance=2e-10;
    const damage::V3 measured{actual.velocityNedMs[0],actual.velocityNedMs[1],actual.velocityNedMs[2]};
    for(int k=0;k<3;++k)require(std::abs((&measured.x)[k]-(&expected.x)[k])<=velocityTolerance,
        "actual released store failed unchanged native NED rail-inheritance law");
    for(int k=0;k<3;++k)require(std::abs(actual.angularRateBodyRadS[k]-(&postOmega.x)[k])<=velocityTolerance,
        "actual released store failed native angular-rate inheritance");
    const auto childMass=fixture.world.model(released)->energy().massKg;
    const auto parentMass=fixture.world.model(fixture.victim)->energy().massKg;
    const auto applicationMass=application.number("application_mass_kg",-1);
    require(childMass>0&&parentMass>0&&applicationMass>0,"actual release masses unavailable");
    const double epsilon=std::numeric_limits<double>::epsilon();
    const auto& fuelValue=application.child("current_native_tank_fuel_kg");
    require(fuelValue.isNumber()&&std::isfinite(fuelValue.asNumber()),"actual native tank contents unavailable");
    const double consumedFuelKg=openingFuelKg-fuelValue.asNumber();
    require(consumedFuelKg>=0,"this powered parent cannot gain fuel in the fixture");
    std::cout<<std::setprecision(17)<<"SAME_C_EPOCH opening_material_kg="<<openingMaterialKg
             <<" application_cached_kg="<<applicationMass<<" opening_fuel_kg="<<openingFuelKg
             <<" application_current_fuel_kg="<<fuelValue.asNumber()<<'\n';
    // MassBalance precedes propulsion in EACH native substep (F-16C has nine).
    // Its last cached mass lies inside the actual measured fuel-consumption
    // interval; it is not the opening outer-tick mass. The release reconciles
    // current contents. This reference never reruns or flushes MassBalance.
    const double boundaryMaterialKg=openingMaterialKg-consumedFuelKg;
    const double massRoundoff=512*epsilon*std::max(1.0,openingMaterialKg);
    require(applicationMass>=boundaryMaterialKg-massRoundoff&&applicationMass<=openingMaterialKg+massRoundoff,
            "cached native mass must lie within the measured consumption interval");
    std::cout<<std::setprecision(17)<<"SAME_C_MASS application_kg="<<applicationMass
             <<" parent_kg="<<parentMass<<" child_kg="<<childMass
             <<" native_consumed_fuel_kg="<<consumedFuelKg
             <<" residual_kg="<<(boundaryMaterialKg-parentMass-childMass)<<'\n';
    require(std::abs(boundaryMaterialKg-parentMass-childMass)<=512*epsilon*std::max(1.0,boundaryMaterialKg),
            "same-C release must preserve actual pre-release mass ownership");
    const auto* rigid=dynamic_cast<const sim::RigidMassSourceV1*>(fixture.world.model(released));sim::RigidMassPropertiesV1 properties;
    require(rigid&&rigid->rigidMassPropertiesV1(properties),"actual released full inertia unavailable");
    const auto rotational=[&](damage::V3 omega) {
        double value=0;
        for(int i=0;i<3;++i)for(int j=0;j<3;++j) {
            require(std::isfinite(properties.inertiaBodyKgM2[3*i+j]),"released inertia must be finite");
            value+=(&omega.x)[i]*properties.inertiaBodyKgM2[3*i+j]*(&omega.x)[j];
        }
        return .5*value;
    };
    const damage::V3 actualOmega{actual.angularRateBodyRadS[0],actual.angularRateBodyRadS[1],actual.angularRateBodyRadS[2]};
    const auto kinetic=[&](damage::V3 velocity,damage::V3 omega){return .5*childMass*damage::dot(velocity,velocity)+rotational(omega);};
    const double beforeK=kinetic(before,preOmega),expectedDelta=kinetic(expected,postOmega)-beforeK;
    const double actualDelta=kinetic(measured,actualOmega)-beforeK;
    const double energyTolerance=childMass*(damage::length(expected)+damage::length(measured))*velocityTolerance+
        512*epsilon*std::max({1.0,std::abs(beforeK),std::abs(kinetic(expected,postOmega))});
    require(std::abs(expectedDelta)>4*energyTolerance,"actual contact must produce a numerically resolved inherited kinetic-energy change");
    require(std::abs(actualDelta-expectedDelta)<=energyTolerance,"same-C release omitted kinetic increment required by native NED/PQR law");
    std::cout<<"SAME_C_RELEASE contact_source="<<fixture.missile<<" recipient="<<fixture.victim<<" released="<<released
             <<" delta_native_law_inherited_kinetic_j="<<actualDelta<<" sdk_ned_body_disagreement_ms="<<sdkNedDisagreement<<" raw_ned_body_disagreement_ms="<<rawNedDisagreement
             <<" response="<<path<<'\n';
}

// Actual World queue test: every oracle input is observed at the contact's
// discrete C solve, excluding unrelated pre-C gravity/aerodynamic integration.
struct MultiContactBody {
    double mass=0;
    std::array<double,9> inertia{};
    damage::V3 position,velocity,omega,arm;
};
damage::V3 multiContactVector(const core::Json& row,const char* key) {
    const auto& a=row.child(key);
    require(a.isArray()&&a.asArray().size()==3,std::string("missing contact vector: ")+key);
    double v[3];
    for(std::size_t k=0;k<3;++k) {
        require(a.asArray()[k].isNumber()&&std::isfinite(a.asArray()[k].asNumber()),"nonfinite contact coordinate");
        v[k]=a.asArray()[k].asNumber();
    }
    return {v[0],v[1],v[2]};
}
damage::V3 multiContactProduct(const std::array<double,9>& a,damage::V3 v) {
    return {a[0]*v.x+a[1]*v.y+a[2]*v.z,a[3]*v.x+a[4]*v.y+a[5]*v.z,a[6]*v.x+a[7]*v.y+a[8]*v.z};
}
// Independent partial-pivot Gaussian elimination; production uses Cholesky.
damage::V3 multiContactSolve(std::array<double,9> a,damage::V3 value) {
    double b[3]={value.x,value.y,value.z};
    for(std::size_t col=0;col<3;++col) {
        std::size_t pivot=col;
        for(std::size_t row=col+1;row<3;++row)if(std::abs(a[3*row+col])>std::abs(a[3*pivot+col]))pivot=row;
        require(std::isfinite(a[3*pivot+col])&&a[3*pivot+col]!=0,"independent contact tensor is singular");
        if(pivot!=col) {for(std::size_t k=0;k<3;++k)std::swap(a[3*pivot+k],a[3*col+k]);std::swap(b[pivot],b[col]);}
        for(std::size_t row=col+1;row<3;++row) {
            const double factor=a[3*row+col]/a[3*col+col];
            for(std::size_t k=col;k<3;++k)a[3*row+k]-=factor*a[3*col+k];
            b[row]-=factor*b[col];
        }
    }
    double x[3]{};
    for(std::size_t reverse=3;reverse>0;--reverse) {
        const auto row=reverse-1;double valueAtRow=b[row];
        for(std::size_t k=row+1;k<3;++k)valueAtRow-=a[3*row+k]*x[k];
        x[row]=valueAtRow/a[3*row+row];require(std::isfinite(x[row]),"nonfinite independent tensor solution");
    }
    return {x[0],x[1],x[2]};
}
MultiContactBody multiContactInput(const core::Json& row,damage::V3 position) {
    MultiContactBody out;out.position=position;out.mass=row.number("mass_kg",-1);
    require(std::isfinite(out.mass)&&out.mass>0,"actual contact mass unavailable");
    const auto& a=row.child("inertia_ecef_kg_m2");require(a.isArray()&&a.asArray().size()==9,"actual full contact tensor unavailable");
    for(std::size_t k=0;k<9;++k) {
        require(a.asArray()[k].isNumber()&&std::isfinite(a.asArray()[k].asNumber()),"nonfinite actual tensor");
        out.inertia[k]=a.asArray()[k].asNumber();
    }
    const auto& i=out.inertia;
    const double determinant=i[0]*(i[4]*i[8]-i[5]*i[7])-i[1]*(i[3]*i[8]-i[5]*i[6])+i[2]*(i[3]*i[7]-i[4]*i[6]);
    require(i[0]>0&&i[0]*i[4]-i[1]*i[3]>0&&determinant>0,"actual tensor must be positive definite");
    out.velocity=multiContactVector(row,"velocity_ecef_ms");out.omega=multiContactVector(row,"angular_velocity_ecef_rad_s");
    out.arm=multiContactVector(row,"arm_ecef_m");return out;
}
MultiContactBody multiContactNative(session::World& world,std::uint32_t id) {
    sim::VehicleState state;world.model(id)->state(state);
    const auto* provider=dynamic_cast<const sim::RigidMassSourceV1*>(world.model(id));sim::RigidMassPropertiesV1 mass;
    require(provider&&provider->rigidMassPropertiesV1(mass),"actual final native full tensor unavailable");
    MultiContactBody out;out.mass=mass.massKg;out.position=position(state);
    out.velocity=rotate(state,{state.velocityBodyMs[0],state.velocityBodyMs[1],state.velocityBodyMs[2]});
    out.omega=rotate(state,{state.angularRateBodyRadS[0],state.angularRateBodyRadS[1],state.angularRateBodyRadS[2]});
    const auto* r=state.rotationBodyToEcef;
    for(std::size_t row=0;row<3;++row)for(std::size_t col=0;col<3;++col)
        for(std::size_t a=0;a<3;++a)for(std::size_t b=0;b<3;++b)
            out.inertia[3*row+col]+=r[3*row+a]*mass.inertiaBodyKgM2[3*a+b]*r[3*col+b];
    return out;
}
double multiContactKinetic(const MultiContactBody& body) {
    return .5*(body.mass*damage::dot(body.velocity,body.velocity)+damage::dot(body.omega,multiContactProduct(body.inertia,body.omega)));
}
// Bounds cover <=4096 elementary double operations, an a-posteriori full-tensor
// condition bound, and the actual finite
// precision SI<->JSBSim conversion constants used by the native impulse setter.
// They are numerical reference bounds, never collision/strength tolerances.
double multiContactConditionBound(const std::array<double,9>& a) {
    std::array<double,9> inverse{};
    for(std::size_t col=0;col<3;++col) {
        const auto x=multiContactSolve(a,{col==0?1.:0.,col==1?1.:0.,col==2?1.:0.});
        inverse[col]=x.x;inverse[3+col]=x.y;inverse[6+col]=x.z;
    }
    double normA=0,normInverse=0,residual=0;
    for(std::size_t row=0;row<3;++row) {
        double sumA=0,sumInverse=0,sumResidual=0;
        for(std::size_t col=0;col<3;++col) {
            sumA+=std::abs(a[3*row+col]);sumInverse+=std::abs(inverse[3*row+col]);
            double product=0;for(std::size_t k=0;k<3;++k)product+=a[3*row+k]*inverse[3*k+col];
            sumResidual+=std::abs((row==col?1.:0.)-product);
        }
        normA=std::max(normA,sumA);normInverse=std::max(normInverse,sumInverse);residual=std::max(residual,sumResidual);
    }
    const double epsilon=std::numeric_limits<double>::epsilon();
    const double normProduct=normA*normInverse*(1+32*epsilon);
    // Banach inverse residual bound, with outward arithmetic allowance.
    const double residualBound=residual+32*epsilon*(1+normProduct);
    require(std::isfinite(normProduct)&&residualBound<1,"independent full-tensor inverse cannot be numerically certified");
    return normProduct/(1-residualBound);
}
double multiContactRelativeError(double condition=1) {
    const double gamma=4096*std::numeric_limits<double>::epsilon()*(1+condition);
    require(std::isfinite(gamma)&&gamma<1,"independent contact reference condition bound unavailable");
    return gamma/(1-gamma)+
        8*std::abs(.224808943099711*14.5939029372064*.3048-1)+
        8*std::abs(.737562149277*1.3558179483314-1);
}
void multiContactClose(damage::V3 actual,damage::V3 expected,const char* label,double relativeError) {
    const double bound=relativeError*std::max({1.,damage::length(actual),damage::length(expected)});
    require(damage::length(actual-expected)<=bound,std::string(label)+" error="+std::to_string(damage::length(actual-expected))+" bound="+std::to_string(bound));
}

std::vector<std::pair<std::uint32_t,std::uint64_t>> runWorldContactSequence(unsigned contactCount) {
    session::World world(options(1,1));
    sim::InitialConditions initial;initial.altitudeMslM=6000;initial.airspeedTrueMs=250;
    initial.headingDeg=123;initial.pitchDeg=7;initial.rollDeg=-18;
    const auto launcher=aircraft(world,"three-source-real-Host",initial);
    std::uint16_t payload=0;bool found=false;
    for(const auto& row:world.missionSystems(launcher)->payloads())if(row.family==ms::Family::StoreStation){payload=row.index;found=true;}
    require(found,"actual Host station payload missing");
    for(const int station:{1,2,8})require(world.loadStore(launcher,payload,station,"aim-9x")==ms::Reason::None,"actual three-rail load refused");
    ms::JettisonCommand command;command.payload=payload;command.emergency=true;
    for(const auto& store:world.missionSystems(launcher)->stores())if(std::string(store.store)=="aim-9x") {
        require(std::string(store.carriage)=="rail","three sources must retain actual rail carriage");command.stores.push_back(store.index);
    }
    require(command.stores.size()==3&&world.submit(launcher,command).accepted(),"actual three-source Host jettison refused");
    for(unsigned step=0;step<2000&&world.weapons().size()<3;++step)world.step();
    require(world.weapons().size()==3,"Host must create all three actual sources");
    std::vector<std::uint32_t> sources;
    for(const auto& value:world.weapons()) {
        require(value.jettisoned&&value.store=="aim-9x","source identity/inert provenance changed");sources.push_back(value.id);
    }
    require(world.removeVehicle(launcher),"actual launcher removal failed");
    const auto victim=aircraft(world,"rotated-full-SPD-recipient",initial);
    std::string error;const auto geometry=damage::readAirframe(std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR)/"f16c/f16c.damage.json",error);
    require(geometry.has_value(),"actual finite target material missing: "+error);
    const auto& panel=geometry->panels.at(51);
    require(!panel.surface&&panel.section==geometry->section("centre")&&!geometry->sections[static_cast<std::size_t>(panel.section)].hasRoot,
            "three-contact control requires actual rootless centre body51");
    const auto target=*world.vehicleState(victim);double targetCg[3];require(world.model(victim)->cgFromOriginBodyM(targetCg),"native target CG unavailable");
    const auto targetOrigin=position(target)-rotate(target,{targetCg[0],targetCg[1],targetCg[2]});
    const auto relative=rotate(target,{0,0,40});
    const double cr=std::cos(target.latitudeRad),sr=std::sin(target.latitudeRad),co=std::cos(target.longitudeRad),so=std::sin(target.longitudeRad);
    const damage::V3 relativeNed{-sr*co*relative.x-sr*so*relative.y+cr*relative.z,-so*relative.x+co*relative.y,
                                  -cr*co*relative.x-cr*so*relative.y-sr*relative.z};
    for(std::size_t index=0;index<sources.size();++index) {
        const auto id=sources[index];const auto fuel=world.model(id)->energy();
        require(fuel.fuelCapacityKg>0&&std::isfinite(fuel.fuelKg),"actual source fuel state unavailable");
        auto reset=initial;reset.enginesRunning=false;reset.fuelFraction=fuel.fuelKg/fuel.fuelCapacityKg;
        reset.velocityNedMs[0]=target.velocityNedMs[0]+relativeNed.x;
        reset.velocityNedMs[1]=target.velocityNedMs[1]+relativeNed.y;
        reset.velocityNedMs[2]=target.velocityNedMs[2]+relativeNed.z;
        require(world.resetVehicle(id,&reset),"actual source attitude/velocity reset refused");
        auto* model=world.model(id);double cg[3];require(model->cgFromOriginBodyM(cg),"actual source CG unavailable");
        double nose[3];const char* axes[]={"x","y","z"};
        const auto version=model->property("fsim/model/nose-version");require(version.valid()&&version.get()==1,"authored source nose metadata missing");
        for(std::size_t axis=0;axis<3;++axis) {
            const auto n=model->property(std::string("fsim/model/nose-")+axes[axis]+"-in");
            const auto o=model->property(std::string("fsim/model/origin-")+axes[axis]+"-in");
            require(n.valid()&&o.valid(),"authored native nose coordinate unavailable");nose[axis]=(axis==1?n.get()-o.get():o.get()-n.get())*.0254;
        }
        auto point=panel.centre;point.x+=(static_cast<double>(index)-1)*.25;point.y+=(static_cast<double>(index)-1)*.3;point.z-=panel.half.z+.1;
        require(std::abs(point.x-panel.centre.x)<panel.half.x&&std::abs(point.y-panel.centre.y)<panel.half.y,"declared source must start over finite face interior");
        if(index>=contactCount)point.y+=1000; // Disclosed one-contact control; same real sources remain in flight.
        const auto own=*world.vehicleState(id);
        const auto desired=targetOrigin+rotate(target,point)-rotate(own,{nose[0]-cg[0],nose[1]-cg[1],nose[2]-cg[2]});
        auto exact=atEcef(desired);exact.headingDeg=initial.headingDeg;exact.pitchDeg=initial.pitchDeg;exact.rollDeg=initial.rollDeg;
        exact.enginesRunning=false;exact.fuelFraction=reset.fuelFraction;
        for(std::size_t axis=0;axis<3;++axis)exact.velocityNedMs[axis]=reset.velocityNedMs[axis];
        require(world.resetVehicle(id,&exact),"finite three-source placement refused");
        const auto placed=*world.vehicleState(id);double placedCg[3];require(model->cgFromOriginBodyM(placedCg),"placed native CG missing");
        const auto actualNose=position(placed)+rotate(placed,{nose[0]-placedCg[0],nose[1]-placedCg[1],nose[2]-placedCg[2]});
        require(damage::length(actualNose-(targetOrigin+rotate(target,point)))<.001,"actual source nose placement differs from declared face");
    }
    MaterialObservation observation;world.step();
    std::vector<core::Json> solves;std::vector<std::pair<std::uint32_t,std::uint64_t>> identities;
    unsigned mechanicalEvents=0;
    for(const auto& event:world.engagementEvents()) {
        require(event.kind!="detonation","inert multi-contact must not introduce a warhead");
        if(event.target==victim&&event.kind=="damage"&&event.reason=="body_contact")++mechanicalEvents;
    }
    require(mechanicalEvents>0,"actual native mechanical response event missing");
    for(const auto id:sources) {
        require(world.alive(id)&&record(world,id).jettisoned&&record(world,id).end==ms::WeaponEnd::InFlight,"actual inert source lifecycle changed");
        const auto generation=world.info(id)->generation;identities.push_back({id,generation});
        const auto path=observation.prefix+"-effects-aim-9x-"+std::to_string(id)+".jsonl";
        if(!std::filesystem::is_regular_file(path))continue;
        require(std::filesystem::file_size(path)<=64u*1024u*1024u,"actual contact observer exceeds existing file bound");
        std::ifstream stream(path);std::string line;unsigned count=0;
        while(std::getline(stream,line)) {
            require(line.size()<=8u*1024u*1024u,"actual contact row exceeds existing bound");
            const auto row=core::Json::parse(line,path);if(row.string("kind","")!="mechanical_contact_solve")continue;
            require(++count==1&&row.string("schema","")=="fsim.sim-m03.effects-observation.v1","duplicate/foreign actual solve");
            require(row.string("weapon","")==std::to_string(id)&&row.string("generation","")==std::to_string(generation)&&
                    row.string("target","")==std::to_string(victim)&&row.string("target_generation","")==std::to_string(world.info(victim)->generation),"actual solve lifetime identity mismatch");
            require(row.number("world_time_s",-1)==world.simTime()&&row.number("application_epoch_s",-1)==world.simTime(),"solve must share actual C epoch");
            require(row.number("contact_epoch_s",-1)>world.simTime()-world.dt()&&row.number("contact_epoch_s",-1)<=world.simTime(),"real contact must occur inside captured interval");
            require(row.number("source_model_time_s",-1)>=0&&row.number("recipient_model_time_s",-1)>=0&&!row.string("next_combat_serial","").empty(),"actual native clocks/serial missing");
            solves.push_back(row);
        }
        require(!stream.bad(),"contact observation read failed");
    }
    require(solves.size()==contactCount,"all declared real mechanical contacts must be observed exactly once");
    std::sort(solves.begin(),solves.end(),[](const auto& a,const auto& b){return std::stoull(a.string("source_slot",""))<std::stoull(b.string("source_slot",""));});
    const auto* material=world.damage(victim);require(material,"actual recipient material response missing");
    require(std::none_of(material->sections().begin(),material->sections().end(),[](const auto& value){return value.separated;}),"rootless control must not change physical mass/topology");
    double conditionBound=1;
    for(const auto& row:solves)for(const char* key:{"source_c_input","recipient_pre_virtual_c_input","recipient_post_virtual_c_input"}) {
        const auto body=multiContactInput(row.child(key),{});
        conditionBound=std::max(conditionBound,multiContactConditionBound(body.inertia));
    }
    const double relativeError=multiContactRelativeError(conditionBound);
    const auto origin=multiContactVector(solves.front(),"recipient_cg_ecef_m");
    auto recipient=multiContactInput(solves.front().child("recipient_pre_virtual_c_input"),origin);
    const double off=std::abs(recipient.inertia[1])+std::abs(recipient.inertia[2])+std::abs(recipient.inertia[5]);
    require(off>4096*std::numeric_limits<double>::epsilon()*std::max({recipient.inertia[0],recipient.inertia[4],recipient.inertia[8]}),"actual rotated tensor must have resolved off-diagonal entries");
    const auto initialRecipient=recipient;
    auto beforeP=recipient.velocity*recipient.mass;
    double momentumScale=damage::length(beforeP);
    auto beforeL=multiContactProduct(recipient.inertia,recipient.omega);
    double angularScale=damage::length(beforeL);
    double beforeK=multiContactKinetic(recipient),loss=0;
    std::vector<MultiContactBody> actualSources;
    for(const auto& row:solves) {
        const auto source=multiContactInput(row.child("source_c_input"),multiContactVector(row,"source_cg_ecef_m"));
        const auto used=multiContactInput(row.child("recipient_post_virtual_c_input"),multiContactVector(row,"recipient_cg_ecef_m"));
        multiContactClose(used.velocity,recipient.velocity,"actual virtual-C linear state",relativeError);multiContactClose(used.omega,recipient.omega,"actual virtual-C angular state",relativeError);
        require(std::abs(used.mass-initialRecipient.mass)<=relativeError*initialRecipient.mass,"recipient C mass changed inside contact sequence");
        for(std::size_t k=0;k<9;++k)require(std::abs(used.inertia[k]-initialRecipient.inertia[k])<=relativeError*std::max(1.,std::abs(initialRecipient.inertia[k])),"recipient C tensor changed inside contact sequence");
        beforeP+=source.velocity*source.mass;
        momentumScale+=damage::length(source.velocity*source.mass);
        angularScale+=damage::length(multiContactProduct(source.inertia,source.omega))+damage::length(damage::cross(source.position-origin,source.velocity*source.mass));
        beforeL+=multiContactProduct(source.inertia,source.omega)+damage::cross(source.position-origin,source.velocity*source.mass);
        beforeK+=multiContactKinetic(source);
        const auto point=multiContactVector(row,"point_ecef_m");
        multiContactClose(source.arm,point-source.position,"actual source arm",relativeError);multiContactClose(used.arm,point-used.position,"actual recipient arm",relativeError);
        auto normal=multiContactVector(row,"normal_ecef");const auto normalLength=damage::length(normal);require(normalLength>0,"actual contact normal unavailable");normal=normal*(1/normalLength);
        const auto ca=damage::cross(source.arm,normal),cb=damage::cross(used.arm,normal);
        const double mobility=1/source.mass+1/used.mass+damage::dot(ca,multiContactSolve(source.inertia,ca))+damage::dot(cb,multiContactSolve(used.inertia,cb));
        const auto relativeContactVelocity=source.velocity+damage::cross(source.omega,source.arm)-recipient.velocity-damage::cross(recipient.omega,used.arm);
        const double closing=-damage::dot(relativeContactVelocity,normal);require(closing>0&&mobility>0,"actual three-contact sequence must be closing and finite");
        const auto ja=normal*(closing/mobility),jb=ja*(-1);
        const auto la=damage::cross(source.arm,ja),lb=damage::cross(used.arm,jb);
        multiContactClose(multiContactVector(row,"impulse_a_ns"),ja,"independent actual source J",relativeError);multiContactClose(multiContactVector(row,"impulse_b_ns"),jb,"independent actual recipient J",relativeError);
        multiContactClose(multiContactVector(row,"angular_impulse_a_nms"),la,"independent source torque",relativeError);multiContactClose(multiContactVector(row,"angular_impulse_b_nms"),lb,"independent recipient torque",relativeError);
        const double dissipation=.5*closing*damage::length(ja),recorded=row.number("dissipated_j",-1);
        require(std::abs(recorded-dissipation)<=relativeError*std::max(1.,dissipation),"actual native dissipation differs from independent scalar contact identity");loss+=dissipation;
        recipient.velocity+=jb*(1/recipient.mass);recipient.omega+=multiContactSolve(recipient.inertia,lb);
        const auto id=static_cast<std::uint32_t>(std::stoul(row.string("weapon","")));const auto actual=multiContactNative(world,id);
        multiContactClose(actual.velocity,source.velocity+ja*(1/source.mass),"actual native source reaction velocity",relativeError);
        multiContactClose(actual.omega,source.omega+multiContactSolve(source.inertia,la),"actual native source reaction angular velocity",relativeError);
        require(std::abs(actual.mass-source.mass)<=relativeError*source.mass,"source physical mass changed during contact");
        actualSources.push_back(actual);
    }
    const auto actualRecipient=multiContactNative(world,victim);
    multiContactClose(actualRecipient.velocity,recipient.velocity,"actual World accumulated recipient velocity",relativeError);
    multiContactClose(actualRecipient.omega,recipient.omega,"actual World accumulated recipient angular velocity",relativeError);
    require(std::abs(actualRecipient.mass-initialRecipient.mass)<=relativeError*initialRecipient.mass,"recipient mass ownership changed during rootless contact");
    for(std::size_t k=0;k<9;++k)require(std::abs(actualRecipient.inertia[k]-initialRecipient.inertia[k])<=relativeError*std::max(1.,std::abs(initialRecipient.inertia[k])),"recipient final C tensor changed during rootless contact");
    auto afterP=actualRecipient.velocity*actualRecipient.mass;
    auto afterL=multiContactProduct(actualRecipient.inertia,actualRecipient.omega)+damage::cross(actualRecipient.position-origin,afterP);
    momentumScale+=damage::length(afterP);
    angularScale+=damage::length(multiContactProduct(actualRecipient.inertia,actualRecipient.omega))+damage::length(damage::cross(actualRecipient.position-origin,afterP));
    double afterK=multiContactKinetic(actualRecipient);
    for(const auto& source:actualSources) {
        afterP+=source.velocity*source.mass;
        momentumScale+=damage::length(source.velocity*source.mass);
        angularScale+=damage::length(multiContactProduct(source.inertia,source.omega))+damage::length(damage::cross(source.position-origin,source.velocity*source.mass));
        afterL+=multiContactProduct(source.inertia,source.omega)+damage::cross(source.position-origin,source.velocity*source.mass);
        afterK+=multiContactKinetic(source);
    }
    require(damage::length(afterP-beforeP)<=relativeError*std::max(1.,momentumScale),"actual whole-cohort linear momentum changed");
    require(damage::length(afterL-beforeL)<=relativeError*std::max(1.,angularScale),"actual common-origin angular momentum changed");
    const double energyBound=relativeError*std::max({1.,beforeK,afterK});
    require(loss>4*energyBound,"actual dissipation must be numerically resolved");
    require(afterK<=beforeK+energyBound&&std::abs(beforeK-afterK-loss)<=energyBound,"actual three-contact cohort violates native kinetic ledger");
    std::cout<<"WORLD_CONTACT_SEQUENCE contacts="<<contactCount<<" actual_before_k_j="<<beforeK<<" actual_after_k_j="<<afterK
             <<" independent_loss_j="<<loss<<" numerical_bound_j="<<energyBound<<" condition_bound="<<conditionBound<<" observation="<<observation.prefix<<'\n';
    return identities;
}
void threeWorldContactsConserveRigidState() {
    const auto one=runWorldContactSequence(1),three=runWorldContactSequence(3);
    require(one==three,"single/three-contact controls must retain identical real source identities/generations");
}

} // namespace

int main(int argc,char** argv) {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]={{"armed lost seeker and worker determinism",armedAndWorkers},
                        {"unarmed body contact carries kinetic impulse",unarmedPhysicalImpulse},
                        {"batched native point contact",batched},{"unarmed first contact stays consumed",unarmed},
                        {"inert jettison physical crossing",inert},{"finite geometry near miss",nearMiss},
                        {"detached geometry excluded",detached},
                        {"inert nose contact material control",inertNoseContactMaterialControl},
                        {"nose contact boundary material coverage",noseContactBoundaryMaterialCoverage},
                        {"older field nose contact boundary material coverage",olderFieldNoseBoundaryMaterialCoverage},
                        {"same-C contact rail release inheritance",sameBoundaryContactRailRelease},
                        {"three World contacts conserve rigid state",threeWorldContactsConserveRigidState}};
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
