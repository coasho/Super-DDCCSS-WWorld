#include "simulation/mission_systems/weapon/NavigationCaptureV2.h"
#include "simulation/mission_systems/components/support/GuidanceOwnFrameV2.h"
#include "simulation/mission_systems/components/implementations/guidance/MeasuredPnGuidanceV2.h"
#include <utility>
#include "mode_flights.h"
#include "foundation/transport/WeaponDiagnosticsV1.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/NativeStepObservationV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <algorithm>
#include <chrono>

namespace {
using namespace fsim::ms::weapon;
NavigationStrapdownInputV2 rotatingCapture(double time) {
    NavigationStrapdownInputV2 in;in.timeS=time;in.position={6378137,0,0};
    const double angle=.8*time,c=std::cos(angle),s=std::sin(angle);
    in.bodyToEcef={V3{c,s,0},V3{-s,c,0},V3{0,0,1}};
    in.angularRateBodyRelativeEcefRadS={0,0,.8};return in;
}
struct Replay {
    NavigationStrapdownV2 navigation;
    NavigationCaptureV2 captures;
    std::size_t sensorEpochs=0,controlEpochs=0;
    explicit Replay(std::uint64_t seed):navigation(NavigationStrapdownConfigV2{},seed) {
        const auto release=rotatingCapture(0);REQUIRE(navigation.initialize(release));captures.reset(release);
    }
    void run(unsigned frameSkip) {
        // Same 106 native FDM captures per 120 Hz OUTER control step. Only
        // grouping into World batches differs; sensor and control clocks do not.
        constexpr unsigned outerSteps=120,internalSteps=106;
        for(unsigned batch=0;batch<outerSteps;batch+=frameSkip) {
            for(unsigned outer=batch;outer<std::min(batch+frameSkip,outerSteps);++outer) {
                const double read=static_cast<double>(outer)/120.;
                REQUIRE(captures.observe(navigation,rotatingCapture(read),true));
                CHECK(navigation.estimate().timeS==read);++controlEpochs;
                CHECK(navigation.imuSample().timeS<=read);
                CHECK(read-navigation.imuSample().timeS<.010000000001);
                for(unsigned inner=1;inner<=internalSteps;++inner) {
                    const double time=(static_cast<double>(outer)+static_cast<double>(inner)/internalSteps)/120.;
                    const auto before=navigation.estimate().timeS;
                    REQUIRE(captures.observe(navigation,rotatingCapture(time),false));
                    if(navigation.estimate().timeS>before)++sensorEpochs;
                }
            }
            const double boundary=static_cast<double>(std::min(batch+frameSkip,outerSteps))/120.;
            REQUIRE(captures.observe(navigation,rotatingCapture(boundary),true));
        }
    }
};
}

TEST_CASE("Captured strapdown has identical sensor RNG and control epochs across World batching", "[weapon-navigation-integration]") {
    Replay single(128),batched(128);single.run(1);batched.run(8);
    CHECK(single.controlEpochs==120);CHECK(batched.controlEpochs==120);
    CHECK(single.sensorEpochs==100);CHECK(batched.sensorEpochs==100);
    CHECK(single.navigation.attitudeBodyToEcef()==batched.navigation.attitudeBodyToEcef());
    CHECK(single.navigation.errorCovariance()==batched.navigation.errorCovariance());
    const auto a=single.navigation.estimate(),b=batched.navigation.estimate();
    CHECK(a.position.x==b.position.x);CHECK(a.position.y==b.position.y);CHECK(a.position.z==b.position.z);
    CHECK(a.velocity.x==b.velocity.x);CHECK(a.velocity.y==b.velocity.y);CHECK(a.velocity.z==b.velocity.z);
    CHECK(single.navigation.imuSample().gyroRadS.z==batched.navigation.imuSample().gyroRadS.z);
    CHECK(a.lastImuSampleS==Catch::Approx(1.));
}

TEST_CASE("Dense own motion does not propagate strapdown covariance between actual events", "[weapon-navigation-integration]") {
    Replay replay(129);const auto initial=replay.navigation.errorCovariance();
    for(unsigned i=1;i<100;++i)REQUIRE(replay.captures.observe(replay.navigation,rotatingCapture(static_cast<double>(i)*.00009),false));
    CHECK(replay.navigation.estimate().timeS==0);CHECK(replay.navigation.errorCovariance()==initial);
    REQUIRE(replay.captures.observe(replay.navigation,rotatingCapture(.01),false));
    CHECK(replay.navigation.estimate().timeS==.01);CHECK(replay.navigation.imuSample().timeS==.01);
    const auto after=replay.navigation.errorCovariance();
    REQUIRE(replay.captures.observe(replay.navigation,rotatingCapture(.01001),false));
    CHECK(replay.navigation.estimate().timeS==.01);CHECK(replay.navigation.errorCovariance()==after);
}

TEST_CASE("Control gravity vertical follows estimated WGS84 position without native latitude", "[weapon-navigation-integration]") {
    // Independent forward geodetic construction at a non-equatorial latitude.
    constexpr double latitude=.7,longitude=-1.2,altitude=6000;
    constexpr double a=6378137.,e2=6.6943799901413165e-3;
    const double sl=std::sin(latitude),cl=std::cos(latitude),n=a/std::sqrt(1-e2*sl*sl);
    const V3 estimated{(n+altitude)*cl*std::cos(longitude),(n+altitude)*cl*std::sin(longitude),(n*(1-e2)+altitude)*sl};
    const auto gravity=navigationGravityVerticalV2(estimated);
    CHECK(gravity.x==Catch::Approx(-9.80665*cl*std::cos(longitude)).margin(1e-9));
    CHECK(gravity.y==Catch::Approx(-9.80665*cl*std::sin(longitude)).margin(1e-9));
    CHECK(gravity.z==Catch::Approx(-9.80665*sl).margin(1e-9));
    CHECK_FALSE(std::isfinite(navigationGravityVerticalV2({}).z));
}

TEST_CASE("Native pool fanout preserves an existing synchronous FDM observer", "[weapon-navigation-integration][world]") {
    using namespace fsim;
    sim::VehiclePool pool(1,false);auto model=std::make_unique<sim::JsbsimModel>(1./120.,std::make_shared<sim::FlatGround>(0.));
    sim::InitialConditions initial;initial.altitudeMslM=6000;initial.airspeedTrueMs=250;
    const auto authored=std::filesystem::path(FSIM_TEST_DATA_DIR).parent_path().parent_path().parent_path()/"aircraft";
    REQUIRE(model->load({"f16c",FSIM_TEST_JSBSIM_ROOT,authored},initial));
    auto* native=model.get();pool.add(std::move(model));unsigned primary=0,prior=0;
    const auto observe=[](void* context,unsigned,unsigned) noexcept {++*static_cast<unsigned*>(context);};
    sim::NativeStepObserverV1 old{native,&prior,observe};sim::ScopedNativeStepObserverV1 existing(old);
    pool.setNativeObserverFactoryV1([&](std::size_t,int,sim::FlightModel&){return sim::NativeStepObserverV1{native,&primary,observe};});
    sim::ControlInputs input;pool.step(Span<const sim::ControlInputs>(&input,1),4);
    CHECK(primary>0);CHECK(primary==prior);
    CHECK(sim::nativeStepObserverV1.model==old.model);CHECK(sim::nativeStepObserverV1.context==old.context);
    CHECK(sim::nativeStepObserverV1.observe==old.observe);
}

TEST_CASE("World strapdown and gyro reach each native outer control and final boundary", "[weapon-navigation-integration][world]") {
    using namespace fsim;
    for(const int frameSkip:{1,4}) {
        CAPTURE(frameSkip);
        const auto name="nav-native-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        auto options=modes::options(name.c_str());options.publish=true;options.frameSkip=frameSkip;
        session::World world(options);const auto own=modes::wing(world,"f16c",6000,250);
        session::VehicleSpec target;target.type="jsbsim:f16c";target.name="native-target";
        target.initial=world.info(own)->initial;target.initial.longitudeDeg+=.06;target.initial.headingDeg=270;
        const auto targetId=world.createVehicle(target);REQUIRE(targetId);
        std::uint16_t stations=0xffff;
        for(const auto& payload:world.missionSystems(own)->payloads())if(payload.family==ms::Family::StoreStation)stations=payload.index;
        REQUIRE(stations!=0xffff);REQUIRE(world.loadStore(own,stations,1,"aim-9x")==ms::Reason::None);
        REQUIRE(world.submit(own,ms::StateCommand{stations,ms::PayloadState::Operate}).accepted());
        for(unsigned i=0;i<1500&&world.missionSystems(own)->payloads()[stations].state!=ms::PayloadState::Operate;++i){world.step();}
        REQUIRE(world.missionSystems(own)->payloads()[stations].state==ms::PayloadState::Operate);
        // Explicit referee native integration fixture, not actor consent evidence.
        ms::StrikeCommand strike;strike.payload=stations;strike.store="aim-9x";strike.target=targetId;strike.consent=true;
        REQUIRE(world.submit(own,strike).accepted());
        for(unsigned i=0;i<600&&world.weapons().empty();++i){world.step();}
        REQUIRE(world.weapons().size()==1);
        const auto weapon=world.weapons().front().id;
        auto pose=world.info(weapon)->initial;pose.bodyRatesRadS[0]=.4;pose.bodyRatesRadS[1]=.1;pose.bodyRatesRadS[2]=-.1;
        REQUIRE(world.resetVehicle(weapon,&pose));const auto generation=world.info(weapon)->generation;
        ipc::WeaponDiagnosticsV1 reader;std::uint64_t pid=0,frame=0;std::uint32_t incarnation=0;
        world.publishNow();REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,frame));REQUIRE(reader.open(name,pid,incarnation));
        std::uint64_t selectionRequest=0;
        REQUIRE(reader.requestSelection(weapon,generation,frame,&selectionRequest));
        auto diagnostic=std::make_unique<ipc::WeaponDiagnosticFrameV1>();
        for(unsigned i=0;i<3;++i) {
            world.step();
            // Presentation publication is wall-clock throttled. Explicitly
            // snapshot this control boundary; do not read a prior valid frame.
            const double boundary=world.simTime();
            world.publishNow();
            CHECK(world.simTime()==boundary);
            REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,frame));REQUIRE(reader.read(frame,*diagnostic));
            INFO("World time " << world.simTime() << ", published time " << diagnostic->simTimeS
                 << ", frame " << frame << ", selected entity " << diagnostic->entity
                 << ", generation " << diagnostic->generation << ", selection status "
                 << static_cast<unsigned>(diagnostic->selectionStatus) << ", navigation cause "
                 << diagnostic->navigation.stamp.cause);
            REQUIRE(diagnostic->entity==weapon);REQUIRE(diagnostic->generation==generation);
            REQUIRE(diagnostic->selectionRequest==selectionRequest);
            REQUIRE(diagnostic->simTimeS==boundary);
            REQUIRE(diagnostic->navigation.stamp.status==ipc::WeaponDiagnosticStatusV1::Available);
            CHECK(diagnostic->navigation.stamp.sampleTimeS==world.simTime());
            CHECK(diagnostic->navigation.lastImuTimeS<=world.simTime());
            CHECK(world.simTime()-diagnostic->navigation.lastImuTimeS<.01000000001);
            const auto controls=world.weaponControlsV1(weapon);REQUIRE(controls);
            REQUIRE(controls->controllerAvailable);
            CHECK(controls->controllerTimeS==Catch::Approx(world.simTime()-options.dt).margin(1e-12));
            const auto epochs=world.weaponNavigationControlEpochsV2(weapon);REQUIRE(epochs);
            CHECK(epochs->first==controls->controllerTimeS);
            CHECK(epochs->second==epochs->first);
            const auto observed=diagnostic->navigation;
            world.publishNow();
            REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,frame));REQUIRE(reader.read(frame,*diagnostic));
            CHECK(world.simTime()==boundary);
            CHECK(diagnostic->navigation.stamp.sampleTimeS==observed.stamp.sampleTimeS);
            CHECK(diagnostic->navigation.lastImuTimeS==observed.lastImuTimeS);
            for(unsigned k=0;k<21*21;++k)CHECK(diagnostic->navigation.errorCovariance[k]==observed.errorCovariance[k]);
        }
        // A copied finite-plant snapshot remains historical after the source
        // disappears. It must never authorize dereferencing a removed FDM.
        REQUIRE(diagnostic->control.stamp.status==ipc::WeaponDiagnosticStatusV1::Available);
        const auto retainedControl=diagnostic->control;
        REQUIRE(world.removeVehicle(weapon));CHECK_FALSE(world.alive(weapon));
        world.publishNow();
        REQUIRE(ipc::weaponDiagnosticWorldIdentityV1(name,pid,incarnation,frame));
        REQUIRE(reader.read(frame,*diagnostic));
        REQUIRE(diagnostic->entity==weapon);REQUIRE(diagnostic->generation==generation);
        CHECK(diagnostic->selectionStatus==ipc::WeaponDiagnosticStatusV1::Removed);
        CHECK(diagnostic->control.stamp.status==ipc::WeaponDiagnosticStatusV1::Removed);
        CHECK(diagnostic->control.stamp.sampleTimeS==retainedControl.stamp.sampleTimeS);
        CHECK(diagnostic->control.finCount==retainedControl.finCount);
        for(std::uint32_t k=0;k<retainedControl.finCount;++k) {
            CHECK(diagnostic->control.fins[k].id==retainedControl.fins[k].id);
            CHECK(diagnostic->control.fins[k].actualRad==retainedControl.fins[k].actualRad);
        }
    }
}

TEST_CASE("Selected strapdown guidance vertical depends on its own pose rather than physical scene up",
          "[weapon-navigation-integration][guidance-own-frame]") {
    using namespace fsim::ms;
    using namespace components;
    SeekerFrameV2 physical;physical.timeS=physical.ageS=1;physical.velocity={250,0,0};
    physical.position={0,6378137,0};physical.up={0,1,0};physical.boresight={1,0,0};
    physical.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
    SeekerNavigationPoseV1 navigation;navigation.valid=true;navigation.timeS=physical.timeS;
    navigation.position={6378137,0,0};navigation.velocity=physical.velocity;
    navigation.bodyToEcef=*physical.bodyAxes;
    auto otherPhysical=physical;otherPhysical.up={0,0,1};
    const auto first=measuredGuidanceOwnFrameV2(physical,navigation,true,true);
    const auto second=measuredGuidanceOwnFrameV2(otherPhysical,navigation,true,true);
    weapon::Spec spec;spec.seeker="infrared";spec.seekerModel.engineering.version=3;
    MeasuredPnGuidanceV2 a(spec),b(spec);SeekerResultV2 result;result.valid=true;
    const auto one=a.updateMeasured(first,result),two=b.updateMeasured(second,result);
    // Independent equatorial WGS84 vertical; this is a generic no-target
    // gravity-compensation dependency fixture, not an encounter or tuning case.
    CHECK(one.acceleration.x==Catch::Approx(9.80665).margin(1e-12));
    CHECK(one.acceleration.y==Catch::Approx(0).margin(1e-12));
    CHECK(one.acceleration.z==Catch::Approx(0).margin(1e-12));
    CHECK(two.acceleration.x==one.acceleration.x);
    CHECK(two.acceleration.y==one.acceleration.y);CHECK(two.acceleration.z==one.acceleration.z);
    CHECK(physical.up.x==0);CHECK(physical.up.y==1);CHECK(physical.up.z==0);
    CHECK(otherPhysical.up.x==0);CHECK(otherPhysical.up.y==0);CHECK(otherPhysical.up.z==1);
}

TEST_CASE("Guidance own-frame selection retains legacy up and refuses unavailable selected navigation",
          "[weapon-navigation-integration][guidance-own-frame]") {
    using namespace fsim::ms;
    using namespace components;
    SeekerFrameV2 physical;physical.timeS=physical.ageS=1;physical.up={0,1,0};
    physical.velocity={250,0,0};physical.boresight={1,0,0};
    SeekerNavigationPoseV1 navigation;navigation.valid=true;navigation.timeS=physical.timeS;
    navigation.position={6378137,0,0};navigation.velocity=physical.velocity;
    navigation.bodyToEcef={{{1,0,0},{0,1,0},{0,0,1}}};
    for(const auto selection:{std::pair<bool,bool>{false,false},{false,true},{true,false}}) {
        const auto legacy=measuredGuidanceOwnFrameV2(physical,navigation,selection.first,selection.second);
        CHECK(legacy.up.x==physical.up.x);CHECK(legacy.up.y==physical.up.y);CHECK(legacy.up.z==physical.up.z);
    }
    weapon::Spec spec;spec.seeker="infrared";spec.seekerModel.engineering.version=3;
    SeekerResultV2 result;result.valid=true;
    for(unsigned invalid=0;invalid<3;++invalid) {
        auto bad=navigation;
        if(invalid==0)bad.valid=false;
        if(invalid==1)bad.timeS=physical.timeS-1;
        if(invalid==2)bad.position={};
        const auto frame=measuredGuidanceOwnFrameV2(physical,bad,true,true);
        CHECK_FALSE(std::isfinite(frame.up.x));CHECK_FALSE(std::isfinite(frame.up.y));CHECK_FALSE(std::isfinite(frame.up.z));
        MeasuredPnGuidanceV2 guidance(spec);const auto refused=guidance.updateMeasured(frame,result);
        CHECK(refused.phase==WeaponPhase::Unguided);
    }
}

TEST_CASE("Estimated guidance vertical and body demand retain rigid longitude rotation equivariance",
          "[weapon-navigation-integration][guidance-own-frame]") {
    using namespace fsim::ms;
    using namespace components;
    const double c=std::cos(.01),s=std::sin(.01);
    SeekerFrameV2 physical;physical.timeS=physical.ageS=1;physical.up={0,0,1};
    physical.velocity={250,0,0};physical.boresight={1,0,0};
    SeekerNavigationPoseV1 original;original.valid=true;original.timeS=physical.timeS;
    original.position={6378137,0,0};original.velocity=physical.velocity;
    original.bodyToEcef={{{1,0,0},{0,1,0},{0,0,1}}};
    auto rotated=original;rotated.position={6378137*c,6378137*s,0};
    rotated.velocity={250*c,250*s,0};rotated.bodyToEcef={{{c,s,0},{-s,c,0},{0,0,1}}};
    weapon::Spec spec;spec.seeker="infrared";spec.seekerModel.engineering.version=3;
    SeekerResultV2 result;result.valid=true;MeasuredPnGuidanceV2 a(spec),b(spec);
    const auto one=a.updateMeasured(measuredGuidanceOwnFrameV2(physical,original,true,true),result);
    const auto two=b.updateMeasured(measuredGuidanceOwnFrameV2(physical,rotated,true,true),result);
    CHECK(two.acceleration.x==Catch::Approx(c*one.acceleration.x).margin(1e-12));
    CHECK(two.acceleration.y==Catch::Approx(s*one.acceleration.x).margin(1e-12));
    CHECK(two.acceleration.z==Catch::Approx(one.acceleration.z).margin(1e-12));
    const double bodyX=c*two.acceleration.x+s*two.acceleration.y;
    const double bodyY=-s*two.acceleration.x+c*two.acceleration.y;
    CHECK(bodyX==Catch::Approx(one.acceleration.x).margin(1e-12));
    CHECK(bodyY==Catch::Approx(one.acceleration.y).margin(1e-12));
    CHECK(physical.up.x==0);CHECK(physical.up.y==0);CHECK(physical.up.z==1);
}
