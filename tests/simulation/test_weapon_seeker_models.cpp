#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/components/interfaces/MeasuredGuidanceV2.h"
#include "simulation/mission_systems/components/implementations/seeker/NonImagingIrSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ImagingIrSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/SemiActiveRadarSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ActiveRadarSeekerV2.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

namespace {
using namespace fsim::ms;
using namespace fsim::ms::components;
struct Runtime {
    std::unique_ptr<GuidanceProviderV1> guidance;
    std::unique_ptr<SeekerProviderV1> sensor;
    explicit Runtime(const char* id) {
        std::vector<std::string> errors;
        const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
        const auto cards=readStoreLibrary(library,errors);
        REQUIRE(errors.empty());
        const auto card=cards.at(id);
        ProfileV1 profile;std::string error;
        REQUIRE(readProfileV1(library/(std::string(id)+".json"),*card,profile,error));
        weapon::Spec spec;spec.seeker=card->seeker;
        REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
        ProviderContextV1 context{*card,profile,spec,std::nullopt};
        guidance=builtInProvidersV1().guidance.create(*profile.find("guidance"),context,error);
        INFO(error);REQUIRE(guidance);
        sensor=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);
        INFO(error);REQUIRE(sensor);
    }
    weapon::Output step(const weapon::Input& input) {
        auto* seeker=dynamic_cast<SeekerExecutionV2*>(sensor.get());
        auto* controller=dynamic_cast<MeasuredGuidanceV2*>(guidance.get());
        REQUIRE(seeker);REQUIRE(controller);
        SeekerFrameV2 f;f.timeS=input.ageS;f.ageS=input.ageS;f.dtS=input.dtS;
        f.position=input.position;f.velocity=input.velocity;f.up=input.up;f.boresight=*input.boresight;
        for(const auto& observation:input.visible) {
            SeekerSampleV2 s;s.direction=measured::unit(measured::subtract(observation.position,input.position));
            s.signal=observation.signal;f.samples.push_back(s);
        }
        const auto result=seeker->sample(f);auto out=controller->updateMeasured(f,result);
        if(result.attributionIndex)out.holding=input.visible[*result.attributionIndex].id;
        return out;
    }
};
weapon::Input frame(double time) {
    weapon::Input in;in.position={0,0,0};in.velocity={250,0,0};in.up={0,0,1};
    in.boresight=weapon::V3{1,0,0};in.ageS=time;in.dtS=.1;
    return in;
}
}

TEST_CASE("Representative IR guidance depends on measured bearing history rather than hidden velocity", "[representative-weapons]") {
    for(const auto* id:{"aim-9m","aim-9x"}) {
        INFO(id);Runtime first(id),second(id);
        auto a=frame(0);a.visible={{17,{1000,0,0},{-100,40,0},false,1}};
        auto b=a;b.visible.front().position.x=2000;b.visible.front().velocity={100,-30,0};
        first.step(a);second.step(b);
        a.ageS=b.ageS=.1;a.visible.front().position.y=10;b.visible.front().position.y=20;
        const auto x=first.step(a),y=second.step(b);
        REQUIRE(x.tracking);REQUIRE(y.tracking);
        CHECK(x.acceleration.x==Catch::Approx(y.acceleration.x).margin(1e-10));
        CHECK(x.acceleration.y==Catch::Approx(y.acceleration.y).margin(1e-10));
        CHECK(std::isnan(x.timeToGoS));CHECK(std::isnan(y.timeToGoS));
    }
}

TEST_CASE("Every authored AAM selects independent measured terminal sensing and guidance", "[representative-weapons][fleet]") {
    std::vector<std::string> errors;
    const auto cards=readStoreLibrary(std::filesystem::path(FSIM_TEST_STORE_DIR),errors);
    REQUIRE(errors.empty());
    unsigned count=0;
    for(const auto& [id,card]:cards) if(card->storeClass=="aam") {
        INFO(id);Runtime runtime(id.c_str());
        CHECK(dynamic_cast<SeekerExecutionV2*>(runtime.sensor.get())!=nullptr);
        CHECK(dynamic_cast<MeasuredGuidanceV2*>(runtime.guidance.get())!=nullptr);
        ++count;
    }
    REQUIRE(count==24);
}

TEST_CASE("Every authored IR terminal profile is invariant to hidden target range and velocity", "[representative-weapons][fleet]") {
    std::vector<std::string> errors;
    const auto cards=readStoreLibrary(std::filesystem::path(FSIM_TEST_STORE_DIR),errors);
    REQUIRE(errors.empty());
    unsigned count=0;
    for(const auto& [id,card]:cards) if(card->storeClass=="aam" &&
            (card->seeker=="infrared" || card->seeker=="imaging_infrared")) {
        INFO(id);Runtime first(id.c_str()),second(id.c_str());
        auto a=frame(0);a.visible={{17,{1000,0,0},{-100,40,0},false,1}};
        auto b=a;b.visible.front().position.x=2000;b.visible.front().velocity={100,-30,0};
        first.step(a);second.step(b);
        a.ageS=b.ageS=.1;a.visible.front().position.y=10;b.visible.front().position.y=20;
        const auto x=first.step(a),y=second.step(b);
        REQUIRE(x.tracking);REQUIRE(y.tracking);
        CHECK(x.acceleration.x==Catch::Approx(y.acceleration.x).margin(1e-10));
        CHECK(x.acceleration.y==Catch::Approx(y.acceleration.y).margin(1e-10));
        CHECK(std::isnan(x.timeToGoS));CHECK(std::isnan(y.timeToGoS));
        ++count;
    }
    REQUIRE(count==12);
}

TEST_CASE("Representative IR discrimination cannot consult semantic decoy attribution", "[representative-weapons]") {
    for(const auto* id:{"aim-9m","aim-9x"}) {
        INFO(id);Runtime first(id),second(id);
        auto a=frame(0);a.visible={{17,{1000,0,0},{0,0,0},false,1}};
        first.step(a);second.step(a);
        a.ageS=.1;a.visible.push_back({18,{1000,1,0},{0,0,0},false,8});
        auto b=a;b.visible.back().decoy=true;
        const auto x=first.step(a),y=second.step(b);
        REQUIRE(x.tracking);REQUIRE(y.tracking);
        CHECK(x.acceleration.x==Catch::Approx(y.acceleration.x).margin(1e-10));
        CHECK(x.acceleration.y==Catch::Approx(y.acceleration.y).margin(1e-10));
        CHECK(x.holding==y.holding);
    }
}

namespace {
weapon::Spec syntheticSpec(weapon::SeekerModeV1 mode) {
    weapon::Spec s;s.seeker=weapon::seekerModeNameV1(mode);s.seekerModel.version=1;s.seekerModel.mode=mode;
    s.seekerModel.acquireM=s.acquireM=100;s.gimbalRad=s.seekerModel.gimbalRad=1;
    s.seekerModel.associationCosine=.99;s.executionVersion=1;s.capabilityVersion=1;
    return s;
}
SeekerFrameV2 sensorFrame(double time) {
    SeekerFrameV2 f;f.timeS=f.ageS=time;f.dtS=.1;f.boresight={1,0,0};f.velocity={250,0,0};f.up={0,0,1};return f;
}
SeekerSampleV2 point(double y,double strength=1) {
    SeekerSampleV2 s;s.direction=measured::unit({1,y,0});s.signal=strength;return s;
}
}

TEST_CASE("Unresolved IR centroids and spatial IR tracks execute different measurement mechanisms", "[representative-weapons]") {
    NonImagingIrSeekerV2 unresolved(syntheticSpec(weapon::SeekerModeV1::Infrared));
    ImagingIrSeekerV2 imaging(syntheticSpec(weapon::SeekerModeV1::ImagingInfrared));
    auto f=sensorFrame(0);f.samples={point(0)};
    REQUIRE(unresolved.sample(f).tracking);REQUIRE(imaging.sample(f).tracking);
    f=sensorFrame(.1);f.samples={point(0),point(.02,8)};
    const auto a=unresolved.sample(f),b=imaging.sample(f);
    REQUIRE(a.tracking);REQUIRE(b.tracking);
    CHECK(a.track->direction.y>.01);CHECK(b.track->direction.y==Catch::Approx(0).margin(1e-12));
    CHECK_FALSE(a.track->rangeM);CHECK_FALSE(b.track->rangeM);
    CHECK_FALSE(b.spatialFeaturesAvailable);CHECK_FALSE(b.covarianceAvailable);
}

TEST_CASE("Spatial association uses admitted geometry features without semantic classification", "[representative-weapons]") {
    ImagingIrSeekerV2 imaging(syntheticSpec(weapon::SeekerModeV1::ImagingInfrared));
    auto f=sensorFrame(0);auto first=point(0);first.imageExtent=ImageExtentV2{.04,.01};f.samples={first};
    REQUIRE(imaging.sample(f).tracking);
    f=sensorFrame(.1);auto distractor=point(0,100);distractor.imageExtent=ImageExtentV2{.01,.01};
    first.direction=measured::unit({1,.01,0});f.samples={distractor,first};
    const auto result=imaging.sample(f);REQUIRE(result.tracking);CHECK(result.spatialFeaturesAvailable);
    REQUIRE(result.attributionIndex);CHECK(*result.attributionIndex==1);
    CHECK(result.track->direction.y>.009);
}

TEST_CASE("Semi-active receiver requires its associated source and both propagation legs", "[representative-weapons]") {
    SemiActiveRadarSeekerV2 receiver(syntheticSpec(weapon::SeekerModeV1::SemiActiveRadar));
    auto f=sensorFrame(0);f.illuminated=true;f.configuredIlluminator=77;
    auto s=point(0);s.illuminator=77;s.transmitterPathClear=s.receiverPathClear=true;f.samples={s};
    REQUIRE(receiver.sample(f).tracking);
    for(int defect=0;defect<4;++defect) {
        auto bad=f;bad.timeS=bad.ageS=(defect+1)*.1;
        if(defect==0)bad.samples[0].illuminator=78;
        if(defect==1)bad.samples[0].transmitterPathClear=false;
        if(defect==2)bad.samples[0].receiverPathClear=false;
        if(defect==3)bad.illuminated=false;
        const auto result=receiver.sample(bad);CHECK(result.valid);CHECK_FALSE(result.tracking);
    }
    f.timeS=f.ageS=.5;const auto restored=receiver.sample(f);CHECK(restored.tracking);
    CHECK_FALSE(restored.track->rangeM);
}

TEST_CASE("Active radar separates permitted midcourse estimates from autonomous sensing", "[representative-weapons]") {
    auto spec=syntheticSpec(weapon::SeekerModeV1::ActiveRadar);spec.inertialMidcourse=spec.launcherUpdates=true;
    ActiveRadarSeekerV2 radar(spec,weapon::Observation{9,{1000,0,0},{0,0,0},false});
    auto f=sensorFrame(0);auto s=point(0);s.rangeM=100;s.rangeRateMs=-250;f.samples={s};
    const auto before=radar.sample(f);CHECK_FALSE(before.active);CHECK_FALSE(before.tracking);REQUIRE(before.midcourseEstimate);
    f.timeS=f.ageS=1;f.position={900,0,0};
    const auto acquired=radar.sample(f);CHECK(acquired.active);CHECK(acquired.tracking);
    f.timeS=f.ageS=1.1;f.supported=false;
    const auto autonomous=radar.sample(f);CHECK(autonomous.active);CHECK(autonomous.tracking);
    f.timeS=f.ageS=1.2;f.samples.clear();
    const auto lost=radar.sample(f);CHECK(lost.active);CHECK_FALSE(lost.tracking);REQUIRE(lost.track);
    f.timeS=f.ageS=1.3;f.samples={s};CHECK(radar.sample(f).tracking);
}

TEST_CASE("Sensor sampling is non-consuming per epoch and invalid epochs do not poison history", "[representative-weapons]") {
    ImagingIrSeekerV2 first(syntheticSpec(weapon::SeekerModeV1::ImagingInfrared));
    ImagingIrSeekerV2 independent(syntheticSpec(weapon::SeekerModeV1::ImagingInfrared));
    auto f=sensorFrame(1);f.samples={point(0)};const auto frozen=first.sample(f);REQUIRE(frozen.tracking);
    f.samples.clear();const auto repeat=first.sample(f);CHECK(repeat.tracking);CHECK(repeat.track->localId==frozen.track->localId);
    CHECK_FALSE(independent.sample(f).tracking);
    f.timeS=0;CHECK_FALSE(first.sample(f).valid);
    f.timeS=1.1;f.samples={point(.01)};const auto next=first.sample(f);REQUIRE(next.tracking);CHECK(next.track->rateAvailable);
    first.resetTracking();f.samples.clear();CHECK_FALSE(first.sample(f).tracking);
}

TEST_CASE("IR and SARH navigation propagates only admitted dated estimates and resets independently", "[representative-weapons]") {
    for(const auto mode:{weapon::SeekerModeV1::ImagingInfrared,weapon::SeekerModeV1::SemiActiveRadar}) {
        auto spec=syntheticSpec(mode);spec.inertialMidcourse=true;spec.launcherUpdates=true;
        std::unique_ptr<MeasuredSeekerV2> sensor;
        if(mode==weapon::SeekerModeV1::ImagingInfrared)sensor=std::make_unique<ImagingIrSeekerV2>(spec);
        else sensor=std::make_unique<SemiActiveRadarSeekerV2>(spec);
        sensor->seedNavigationEstimate({17,{10000,0,0},{-100,0,0},false,1});
        auto f=sensorFrame(100);f.ageS=0;f.supported=false;
        auto r=sensor->sample(f);REQUIRE(r.valid);REQUIRE(r.midcourseEstimate);
        CHECK_FALSE(r.tracking);CHECK_FALSE(r.track);
        CHECK(r.midcourseEstimate->position.x==10000);
        f.timeS=101;f.ageS=1;
        // An undelivered update is not a navigation observation.
        f.launcherEstimate=LauncherEstimateV2{{50000,0,0},{0,0,0},101};
        r=sensor->sample(f);REQUIRE(r.midcourseEstimate);
        CHECK(r.midcourseEstimate->position.x==9900);
        f.timeS=102;f.ageS=2;f.supported=true;
        f.launcherEstimate=LauncherEstimateV2{{9000,0,0},{-50,0,0},101.5};
        r=sensor->sample(f);REQUIRE(r.valid);REQUIRE(r.midcourseEstimate);
        CHECK(r.midcourseEstimate->position.x==8975);
        f.timeS=103;f.ageS=3;f.launcherEstimate->sampleTimeS=104;
        CHECK_FALSE(sensor->sample(f).valid);
        // Rejected future data cannot replace the retained last navigation fix.
        f.supported=false;f.launcherEstimate.reset();
        r=sensor->sample(f);REQUIRE(r.valid);REQUIRE(r.midcourseEstimate);
        CHECK(r.midcourseEstimate->position.x==8925);
        sensor->resetTracking();f.timeS=104;f.ageS=4;
        r=sensor->sample(f);REQUIRE(r.valid);CHECK_FALSE(r.midcourseEstimate);
    }
}

TEST_CASE("Disabled IR navigation cannot ingest launcher estimates or infer target range", "[representative-weapons]") {
    auto spec=syntheticSpec(weapon::SeekerModeV1::ImagingInfrared);
    spec.inertialMidcourse=false;spec.launcherUpdates=false;
    ImagingIrSeekerV2 sensor(spec);
    sensor.seedNavigationEstimate({17,{10000,0,0},{-100,0,0},false,1});
    auto f=sensorFrame(0);f.supported=true;f.launcherEstimate=LauncherEstimateV2{{10000,0,0},{-100,0,0},0};
    const auto r=sensor.sample(f);REQUIRE(r.valid);CHECK_FALSE(r.midcourseEstimate);CHECK_FALSE(r.track);
}

TEST_CASE("A rejected future navigation update preserves the last accepted IR and SARH snapshot", "[representative-weapons]") {
    for(const auto mode:{weapon::SeekerModeV1::ImagingInfrared,weapon::SeekerModeV1::SemiActiveRadar}) {
        auto spec=syntheticSpec(mode);spec.inertialMidcourse=true;spec.launcherUpdates=true;
        std::unique_ptr<MeasuredSeekerV2> sensor;
        if(mode==weapon::SeekerModeV1::ImagingInfrared)sensor=std::make_unique<ImagingIrSeekerV2>(spec);
        else sensor=std::make_unique<SemiActiveRadarSeekerV2>(spec);
        auto f=sensorFrame(0);f.supported=f.illuminated=true;f.configuredIlluminator=7;
        auto s=point(0);s.illuminator=7;s.transmitterPathClear=s.receiverPathClear=true;f.samples={s};
        const auto accepted=sensor->sample(f);REQUIRE(accepted.valid);REQUIRE(accepted.tracking);REQUIRE(accepted.track);
        f.timeS=f.ageS=1;f.launcherEstimate=LauncherEstimateV2{{10000,0,0},{-100,0,0},2};
        CHECK_FALSE(sensor->sample(f).valid);
        f.timeS=f.ageS=0;f.samples.clear();f.launcherEstimate.reset();
        const auto retained=sensor->sample(f);
        REQUIRE(retained.valid);REQUIRE(retained.tracking);REQUIRE(retained.track);
        CHECK(retained.track->localId==accepted.track->localId);
        CHECK(retained.track->sampleTimeS==accepted.track->sampleTimeS);
    }
}

TEST_CASE("Measured composition rejects a different admitted variant in the same family", "[representative-weapons]") {
    std::vector<std::string> errors;const auto root=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(root,errors);REQUIRE(errors.empty());
    const auto card=cards.at("aim-120c");ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(root/"aim-120c.json",*card,profile,error));
    profile.components.at("seeker").bindings["variant"]="Meteor";
    weapon::Spec spec;spec.seeker=card->seeker;spec.acquireM=123;
    CHECK_FALSE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
    CHECK(spec.acquireM==123);CHECK(error.find("variant")!=std::string::npos);
}

TEST_CASE("Measured IR preserves unsupported launcher-update and legacy-call semantics", "[representative-weapons]") {
    Runtime runtime("aim-9m");auto* controller=dynamic_cast<MeasuredGuidanceV2*>(runtime.guidance.get());REQUIRE(controller);
    auto f=sensorFrame(0);f.supported=true;SeekerResultV2 r;r.valid=true;r.supported=true;
    const auto output=controller->updateMeasured(f,r);CHECK_FALSE(output.supported);
    CHECK_FALSE(runtime.guidance->update(frame(0)).tracking);
    CHECK(std::isnan(output.timeToGoS));CHECK(std::isnan(output.timeToActiveS));
}

TEST_CASE("Active radar reset clears activation and prior navigation without affecting another receiver", "[representative-weapons]") {
    auto spec=syntheticSpec(weapon::SeekerModeV1::ActiveRadar);
    spec.launcherUpdates=true;spec.inertialMidcourse=true;
    ActiveRadarSeekerV2 reset(spec,std::nullopt),independent(spec,std::nullopt);
    auto f=sensorFrame(0);f.supported=true;
    f.launcherEstimate=LauncherEstimateV2{{50,0,0},{0,0,0},0};
    REQUIRE(reset.sample(f).active);REQUIRE(independent.sample(f).active);
    reset.resetTracking();f.timeS=f.ageS=1;
    f.launcherEstimate=LauncherEstimateV2{{10000,0,0},{-100,0,0},1};
    const auto fresh=reset.sample(f),retained=independent.sample(f);
    REQUIRE(fresh.valid);CHECK_FALSE(fresh.active);CHECK(retained.active);
    REQUIRE(fresh.midcourseEstimate);CHECK(fresh.midcourseEstimate->position.x==10000);
}

TEST_CASE("Measured component schemas reject false variants and mismatched observables atomically", "[representative-weapons]") {
    std::vector<std::string> errors;const auto root=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(root,errors);REQUIRE(errors.empty());
    const auto card=cards.at("aim-9x");ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(root/"aim-9x.json",*card,profile,error));
    weapon::SeekerConfigV1 unchanged;unchanged.acquireM=123;
    for(const auto* field:{"variant","measurement_model","image_model","servo_model","mode"}) {
        auto bad=*profile.find("seeker");bad.bindings[field]="unsupported";
        CHECK_FALSE(weapon::configureSeeker(bad,unchanged,error));CHECK(unchanged.acquireM==123);
    }
    weapon::Spec spec;REQUIRE(weapon::configureSeeker(*profile.find("seeker"),spec,error));
    auto bad=*profile.find("guidance");bad.bindings["measurement_family"]="radar";
    CHECK_FALSE(weapon::configureGuidance(bad,spec,error));
}

TEST_CASE("Spatial tracker association is invariant to input ordering including coincident samples", "[representative-weapons]") {
    ImagingIrSeekerV2 a(syntheticSpec(weapon::SeekerModeV1::ImagingInfrared));
    ImagingIrSeekerV2 b(syntheticSpec(weapon::SeekerModeV1::ImagingInfrared));
    auto f=sensorFrame(0);auto first=point(0),second=point(0);
    first.imageExtent=ImageExtentV2{.04,.01};second.imageExtent=ImageExtentV2{.01,.01};
    f.samples={first,second};const auto x=a.sample(f);std::reverse(f.samples.begin(),f.samples.end());const auto y=b.sample(f);
    REQUIRE(x.track);REQUIRE(y.track);CHECK(x.track->imageExtent->widthRad==y.track->imageExtent->widthRad);
    f.timeS=f.ageS=.1;f.samples[0].direction=measured::unit({1,.01,0});
    const auto nextA=a.sample(f),nextB=b.sample(f);REQUIRE(nextA.track);REQUIRE(nextB.track);
    CHECK(nextA.track->direction.y==Catch::Approx(nextB.track->direction.y).margin(1e-12));
}
