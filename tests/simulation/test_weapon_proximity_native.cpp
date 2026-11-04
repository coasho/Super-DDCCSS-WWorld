#include "mode_flights.h"
#include "simulation/mission_systems/components/registration/BuiltInProvidersV1.h"
#include "simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h"
#include "simulation/mission_systems/components/interfaces/SeekerSceneClockV1.h"
#include "simulation/mission_systems/components/interfaces/MeasuredGuidanceV2.h"
#include <catch2/catch_approx.hpp>
#include <memory>

namespace {
using namespace fsim;
using namespace fsim::ms::components;
constexpr double degrees=180./3.14159265358979323846;

// An explicit test-only failed detector supplies no observation or attribution.
// Native release, flight, finite controls, fuze and damage remain production code.
class NoDetection final:public SeekerProviderV1,public SeekerExecutionV2,
    public RadioSeekerExecutionV1,public SeekerSceneClockV1 {
public:
    NoDetection(std::unique_ptr<SeekerProviderV1> inner,std::shared_ptr<unsigned> samples)
        :inner_(std::move(inner)),samples_(std::move(samples)){}
    const ms::weapon::SeekerConfigV1& config() const noexcept override{return inner_->config();}
    double infraredRange(double a,double b,bool c) const noexcept override{return inner_->infraredRange(a,b,c);}
    double radarRange(double a,double b,bool c) const noexcept override{return inner_->radarRange(a,b,c);}
    bool notched(double a,double b) const noexcept override{return inner_->notched(a,b);}
    double decoySignal(double a,bool b) const noexcept override{return inner_->decoySignal(a,b);}
    bool illuminated(const double a[3],double b,double c,double d,double e) const noexcept override{return inner_->illuminated(a,b,c,d,e);}
    bool sceneCaptureDueV1(double) const noexcept override{return false;}
    void seedTrack(const SeekerTrackV2&) override{}
    void resetTracking() noexcept override{}
    SeekerResultV2 sample(const SeekerFrameV2&) override{++*samples_;SeekerResultV2 r;r.valid=true;r.active=true;return r;}
    bool configureRadioV1(const RadioReceiverConfigV1&,std::string& error) override{error="test detector has failed";return false;}
    SeekerResultV2 sampleRadioV1(const SeekerFrameV2& frame,const RadioSceneFrameV1&) override{return sample(frame);}
    const RadioReceiverResultV1* radioSpectrumV1() const noexcept override{return nullptr;}
    const RadioReceiverConfigV1* radioPolicyV1() const noexcept override{return nullptr;}
private:
    std::unique_ptr<SeekerProviderV1> inner_;
    std::shared_ptr<unsigned> samples_;
};

struct Fixture {
    std::shared_ptr<unsigned> observations=std::make_shared<unsigned>(0);
    std::shared_ptr<GuidanceProviderV1*> guidance=std::make_shared<GuidanceProviderV1*>(nullptr);
    session::World world;
    std::uint32_t launcher=0,weapon=0;
    explicit Fixture(const char* name,bool inert=false,const char* store="aim-120c"):world(modes::options(name)) {
        auto registry=builtInProvidersV1();const auto factories=registry.seeker;std::string error;
        const std::string equipment=std::string(store)=="aim-120d"?"equipment.aim-120d":"equipment.aim-120c-7";
        REQUIRE(registry.seeker.registerFactory(equipment+".seeker","fsim.seeker.active_radar.v3",
            [factories,samples=observations](const auto& descriptor,const auto& context,auto& message)->std::unique_ptr<SeekerProviderV1>{
                auto inner=factories.create(descriptor,context,message);if(!inner)return {};
                return std::make_unique<NoDetection>(std::move(inner),samples);
            },true,error));
        const auto guidanceFactories=registry.guidance;
        REQUIRE(registry.guidance.registerFactory(equipment+".guidance","fsim.guidance.measured_pn.v2",
            [guidanceFactories,capture=guidance](const auto& descriptor,const auto& context,auto& message) {
                auto provider=guidanceFactories.create(descriptor,context,message);
                *capture=provider.get();return provider;
            },true,error));
        REQUIRE(world.setPayloadProvidersV1(registry,error));
        launcher=modes::wing(world,"f16c",6000,250);
        std::uint16_t station=0xffff;
        for(const auto& payload:world.missionSystems(launcher)->payloads())
            if(payload.family==ms::Family::StoreStation)station=payload.index;
        REQUIRE(station!=0xffff);REQUIRE(world.loadStore(launcher,station,1,store)==ms::Reason::None);
        REQUIRE(world.submit(launcher,ms::StateCommand{station,ms::PayloadState::Operate}).accepted());
        for(unsigned i=0;i<1500&&world.missionSystems(launcher)->payloads()[station].state!=ms::PayloadState::Operate;++i)world.step();
        REQUIRE(world.missionSystems(launcher)->payloads()[station].state==ms::PayloadState::Operate);
        if(inert) {
            ms::JettisonCommand command;command.payload=station;command.emergency=true;
            REQUIRE(world.submit(launcher,command).accepted());
        } else {
            // This referee-only fixture requests a coordinate-independent native
            // release. It is not public actor consent or sensor acceptance.
            session::VehicleSpec distant;distant.name="release-reference";distant.type="jsbsim:f16c";
            distant.initial=world.info(launcher)->initial;distant.initial.longitudeDeg+=.2;
            const auto reference=world.createVehicle(distant);REQUIRE(reference);
            ms::StrikeCommand command;command.payload=station;command.store=store;command.target=reference;command.consent=true;
            REQUIRE(world.submit(launcher,command).accepted());
        }
        for(unsigned i=0;i<600&&world.weapons().empty();++i)world.step();
        REQUIRE(world.weapons().size()==1);weapon=world.weapons().front().id;
    }
    void placeWeapon(double age) {
        while(world.simTime()-world.info(weapon)->launchTimeS<age)world.step();
        auto pose=world.info(weapon)->initial;
        pose.latitudeDeg=36.25;pose.longitudeDeg=-115.03;pose.altitudeMslM=6000;
        pose.headingDeg=90;pose.pitchDeg=pose.rollDeg=0;pose.airspeedTrueMs=500;
        REQUIRE(world.resetVehicle(weapon,&pose));
    }
    std::uint32_t target(const char* name,double east,double north) {
        const auto* state=world.vehicleState(weapon);REQUIRE(state);
        session::VehicleSpec spec;spec.name=name;spec.type="jsbsim:f16c";
        spec.initial.latitudeDeg=state->latitudeRad*degrees+north/6371000.*degrees;
        spec.initial.longitudeDeg=state->longitudeRad*degrees+east/(6371000.*std::cos(state->latitudeRad))*degrees;
        spec.initial.altitudeMslM=6000;spec.initial.airspeedTrueMs=250;spec.initial.headingDeg=270;
        const auto id=world.createVehicle(spec);REQUIRE(id);return id;
    }
    unsigned run(double duration,std::uint32_t* victim=nullptr) {
        const auto stop=world.simTime()+duration;
        while(world.simTime()<stop&&world.weapons().front().end==ms::WeaponEnd::InFlight) {
            world.step();CHECK_FALSE(world.weapons().front().tracking);
        }
        unsigned detonations=0;
        for(const auto& event:world.engagementEvents())if(event.entity==weapon&&event.kind=="detonation") {
            ++detonations;if(victim)*victim=event.target;
        }
        return detonations;
    }
};
}

TEST_CASE("Time armed native proximity fuze sees an untracked physical aircraft", "[weapon-proximity-native]") {
    Fixture fixture("physical-proximity-untracked");fixture.placeWeapon(1.1);
    const auto target=fixture.target("untracked",200,8);std::uint32_t victim=0;
    CHECK(fixture.run(.6,&victim)==1);CHECK(victim==target);CHECK(*fixture.observations>0);
    CHECK(fixture.world.weapons().front().end==ms::WeaponEnd::Detonated);
    CHECK(fixture.world.weapons().front().missM<11.47213595499958);
}

TEST_CASE("Native proximity retains its authored radius arming and inert jettison controls", "[weapon-proximity-native]") {
    SECTION("outside unchanged sphere") {
        Fixture fixture("physical-proximity-outside");fixture.placeWeapon(1.1);fixture.target("outside",200,40);
        CHECK(fixture.run(.6)==0);CHECK(fixture.world.weapons().front().end==ms::WeaponEnd::InFlight);
    }
    SECTION("before unchanged arming time") {
        Fixture fixture("physical-proximity-unarmed");fixture.placeWeapon(0);fixture.target("early",200,8);
        CHECK(fixture.run(.6)==0);CHECK(fixture.world.weapons().front().end==ms::WeaponEnd::InFlight);
    }
    SECTION("inert emergency release") {
        Fixture fixture("physical-proximity-jettison",true);fixture.placeWeapon(1.1);fixture.target("inert",200,8);
        CHECK(fixture.run(.6)==0);CHECK(fixture.world.weapons().front().jettisoned);
    }
}

TEST_CASE("Native physical proximity selects the earliest candidate independent of insertion order", "[weapon-proximity-native]") {
    for(const bool reverse:{false,true}) {
        CAPTURE(reverse);Fixture fixture(reverse?"physical-proximity-order-b":"physical-proximity-order-a");
        fixture.placeWeapon(1.1);std::uint32_t near=0;
        if(reverse){fixture.target("later",205,8);near=fixture.target("earlier",195,8);}
        else {near=fixture.target("earlier",195,8);fixture.target("later",205,8);}
        std::uint32_t victim=0;CHECK(fixture.run(.6,&victim)==1);CHECK(victim==near);
    }
}

TEST_CASE("Native weapon pose reset clears optional terminal guidance state", "[weapon-guidance-reset-native]") {
    Fixture fixture("terminal-guidance-reset");REQUIRE(*fixture.guidance);
    auto* guide=dynamic_cast<MeasuredGuidanceV2*>(*fixture.guidance);REQUIRE(guide);
    SeekerFrameV2 frame;frame.timeS=frame.ageS=0;frame.velocity={250,0,0};frame.up={0,0,1};
    SeekerResultV2 contact;contact.valid=true;contact.active=true;contact.tracking=true;
    SeekerTrackV2 track;track.localId=1;track.direction={1,0,0};contact.track=track;
    REQUIRE(guide->updateMeasured(frame,contact).phase==ms::WeaponPhase::Homing);
    auto pose=fixture.world.info(fixture.weapon)->initial;
    REQUIRE(fixture.world.resetVehicle(fixture.weapon,&pose));
    SeekerResultV2 preterminal;preterminal.valid=true;
    preterminal.midcourseEstimate=LauncherEstimateV2{{10000,0,0},{},0};
    const auto fresh=guide->updateMeasured(frame,preterminal);
    CHECK(fresh.phase==ms::WeaponPhase::Midcourse);CHECK(fresh.acceleration.z>9.80665);
}

TEST_CASE("Explicit V0 scene policy detects an untracked physical target without replacing its sweep",
          "[weapon-proximity-native][weapon-proximity-scene-v0]") {
    // Original AIM120D full-chain RED retained separately: actual armed CPA
    // 1.1723276214934657 m inside11.47213595499958 m with no main track/fuze.
    // This orthogonal detector-failure fixture leaves actual release/flight,
    // existing V0 geometry, arming, source and damage entirely native.
    Fixture fixture("v0-scene-untracked",false,"aim-120d");fixture.placeWeapon(1.1);
    const auto target=fixture.target("untracked",200,8);std::uint32_t victim=0;
    CHECK(fixture.run(.6,&victim)==1);CHECK(victim==target);CHECK(*fixture.observations>0);
    CHECK(fixture.world.weapons().front().end==ms::WeaponEnd::Detonated);
    CHECK(fixture.world.weapons().front().missM<11.47213595499958);
}

TEST_CASE("Explicit V0 scene policy retains radius arming inertness and earliest candidate semantics",
          "[weapon-proximity-native][weapon-proximity-scene-v0]") {
    SECTION("outside original sphere") {
        Fixture fixture("v0-scene-outside",false,"aim-120d");fixture.placeWeapon(1.1);
        fixture.target("outside",200,40);
        CHECK(fixture.run(.6)==0);CHECK(fixture.world.weapons().front().end==ms::WeaponEnd::InFlight);
    }
    SECTION("before original arming") {
        Fixture fixture("v0-scene-unarmed",false,"aim-120d");fixture.placeWeapon(0);
        fixture.target("early",200,8);
        CHECK(fixture.run(.6)==0);CHECK(fixture.world.weapons().front().end==ms::WeaponEnd::InFlight);
    }
    SECTION("actual inert emergency release") {
        Fixture fixture("v0-scene-jettison",true,"aim-120d");fixture.placeWeapon(1.1);
        fixture.target("inert",200,8);
        CHECK(fixture.run(.6)==0);CHECK(fixture.world.weapons().front().jettisoned);
    }
    SECTION("candidate order") {
        for(const bool reverse:{false,true}) {
            CAPTURE(reverse);Fixture fixture(reverse?"v0-scene-order-b":"v0-scene-order-a",false,"aim-120d");
            fixture.placeWeapon(1.1);std::uint32_t near=0;
            if(reverse){fixture.target("later",205,8);near=fixture.target("earlier",195,8);}
            else {near=fixture.target("earlier",195,8);fixture.target("later",205,8);}
            std::uint32_t victim=0;CHECK(fixture.run(.6,&victim)==1);CHECK(victim==near);
        }
    }
    SECTION("equal fractions use stable native identity") {
        Fixture fixture("v0-scene-equal",false,"aim-120d");fixture.placeWeapon(1.1);
        const auto first=fixture.target("first",200,8),second=fixture.target("second",200,8);
        std::uint32_t victim=0;CHECK(fixture.run(.6,&victim)==1);
        CHECK(victim==std::min(first,second));
    }
}
