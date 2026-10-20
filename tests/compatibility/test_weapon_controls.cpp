#include "fsim/World.h"
#include "fsim/WeaponControls.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <algorithm>
#include <set>

TEST_CASE("public weapon controls retain actual individual fins and reject stale entities", "[sdk][weapon_controls]") {
    fsim::WorldOptions options;
    options.name="weapon-controls-public"; options.publish=false; options.terrain=false;
    options.workers=1; options.pinWorkers=false; options.jsbsimRoot=FSIM_TEST_JSBSIM_ROOT;
    fsim::World world(options);
    fsim::VehicleSpec spec;
    spec.name="carrier"; spec.type="jsbsim:f16c";
    spec.initial.latitudeDeg=40; spec.initial.longitudeDeg=0; spec.initial.altitudeMslM=6000;
    spec.initial.airspeedTrueMs=250; spec.initial.headingDeg=90;
    auto carrier=world.createVehicle(spec);
    fsim::control::VelocityCommand hold;
    constexpr double deg=3.14159265358979323846/180;
    hold.airspeedMs=250; hold.verticalSpeedMs=0; hold.headingRad=90*deg;
    REQUIRE(carrier.submit(hold).accepted());
    CHECK_FALSE(world.weaponControlsV1(carrier.id()));
    CHECK_FALSE(world.weaponControlsV1(0));
    REQUIRE(carrier.loadLoadout("air superiority")==fsim::ms::Reason::None);
    std::uint16_t stations=0xffff;
    for(const auto& p:carrier.payloads()) if(p.family==fsim::ms::Family::StoreStation) stations=p.index;
    REQUIRE(stations!=0xffff);
    REQUIRE(carrier.submit(fsim::ms::StateCommand{stations,fsim::ms::PayloadState::Operate}).accepted());
    world.step(180);
    // Create the opponent after setup, relative to the stable launcher's
    // actual position. A retained default longitude placed it across the Earth.
    spec.name="target"; spec.initial.latitudeDeg=carrier.state().latitudeRad/deg+.01;
    spec.initial.longitudeDeg=carrier.state().longitudeRad/deg+.15; spec.initial.headingDeg=270;
    auto target=world.createVehicle(spec);
    hold.headingRad=270*deg; REQUIRE(target.submit(hold).accepted());
    world.step();
    fsim::ms::StrikeCommand strike;
    strike.payload=stations; strike.store="aim-120c"; strike.target=target.id(); strike.consent=true;
    const auto activity=carrier.submit(strike);
    const auto& ownState=carrier.state(); const auto& targetState=target.state();
    double ray[3],body[3]{};
    for(unsigned k=0;k<3;++k) ray[k]=targetState.positionEcef[k]-ownState.positionEcef[k];
    for(unsigned k=0;k<3;++k) for(unsigned j=0;j<3;++j) body[k]+=ownState.rotationBodyToEcef[j*3+k]*ray[j];
    const double range=std::sqrt(ray[0]*ray[0]+ray[1]*ray[1]+ray[2]*ray[2]);
    const double offAxis=std::acos(std::clamp(body[0]/std::max(range,1.),-1.,1.))*180/3.14159265358979323846;
    CAPTURE(static_cast<int>(activity.status),static_cast<int>(activity.reason),fsim::ms::reasonName(activity.reason),
            world.time(),range,offAxis,ownState.crashed,targetState.crashed,ownState.airspeedTrueMs,
            ownState.eulerRad[0],ownState.eulerRad[1],ownState.eulerRad[2]);
    REQUIRE(activity.accepted());
    for(unsigned n=0;n<360 && world.weapons().empty();++n) world.step();
    REQUIRE(world.weapons().size()==1);
    const auto id=world.weapons().front().id;
    auto weapon=world.vehicle(id);
    world.step(12);
    const auto sample=world.weaponControlsV1(id); REQUIRE(sample);
    REQUIRE(sample->available); REQUIRE(sample->controllerAvailable);
    REQUIRE(sample->fins.size()==4);
    CHECK(sample->sampleTimeS==world.time());
    CHECK(sample->controllerTimeS<=sample->sampleTimeS);
    std::set<std::uint32_t> ids;
    bool moved=false;
    for(const auto& fin:sample->fins) {
        CHECK(fin.valid); CHECK(fin.id!=0); CHECK(ids.insert(fin.id).second);
        CHECK(fin.actualRad>=fin.minimumRad); CHECK(fin.actualRad<=fin.maximumRad);
        moved=moved || std::abs(fin.actualRad)>1e-8;
    }
    CHECK(moved);
    const auto index=sample->fins.front().nativeIndex;
    const auto path="fcs/fin-"+std::to_string(index)+"-actuator/malfunction/fail_stuck";
    weapon.property(path).set(1);
    const double jammed=sample->fins.front().actualRad;
    world.step(12);
    const auto failed=world.weaponControlsV1(id); REQUIRE(failed);
    CHECK(failed->fins.front().failedStuck);
    CHECK(failed->fins.front().actualRad==jammed);
    CHECK(weapon.property("external_reactions/fsim/magnitude").get()==0);
    CHECK(weapon.property("external_reactions/fsim/magnitude-lbsft").get()==0);
    REQUIRE(weapon.remove()); CHECK_FALSE(world.weaponControlsV1(id));
}
