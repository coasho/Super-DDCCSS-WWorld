#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/components/interfaces/SeekerExecutionV2.h"
#include "simulation/mission_systems/components/interfaces/SeekerLaunchNavigationV1.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#include "simulation/mission_systems/components/implementations/guidance/MeasuredPnGuidanceV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ActiveRadarSeekerV2.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <map>

namespace {
using namespace fsim::ms;
using namespace fsim::ms::components;
using namespace fsim::ms::components::measured;
struct Receiver {
    weapon::Spec spec;
    std::unique_ptr<SeekerProviderV1> provider;
    SeekerExecutionV2* sensor=nullptr;
    explicit Receiver(const char* id="aim-120c",bool scalarMode=true,bool quadrantCompatibility=false) {
        std::vector<std::string> errors;std::string error;
        const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
        const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
        const auto card=cards.at(id);ProfileV1 profile;
        REQUIRE(readProfileV1(library/(std::string(id)+".json"),*card,profile,error));
        // These retained scalar V3 method fixtures explicitly select that
        // binding. Current physical receiver dispatch has separate cases.
        if(quadrantCompatibility) {
            auto& d=profile.components.at("seeker");
            REQUIRE(d.bindings.at("receiver_model")=="thermal_conscan_reticle.v1");
            d.bindings["receiver_model"]="thermal_quadrant.v1";
            d.bindings.erase("reticle_pattern");d.bindings.erase("reticle_design");
            for(const auto* role:{"receiver_reticle_periods","receiver_reticle_samples_per_scan","receiver_reticle_spatial_samples",
                "receiver_reticle_scan_radius_rad","receiver_reticle_stop_radius_rad","receiver_reticle_frequency_hz"}) {
                d.parameters.erase(role);d.parameterRefs.erase(role);
            }
        }
        DescriptorV1 scalar;ReceiverConfigurationV3 receiver;
        REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,receiver,error));
        if(scalarMode)profile.components.at("seeker")=std::move(scalar);
        REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
        ProviderContextV1 context{*card,profile,spec,std::nullopt};
        provider=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);
        INFO(error);REQUIRE(provider);sensor=dynamic_cast<SeekerExecutionV2*>(provider.get());REQUIRE(sensor);
    }
};
SeekerFrameV2 scene(double time=0,double bearing=0) {
    SeekerFrameV2 f;f.timeS=time;f.ageS=time;f.dtS=.01;f.position={0,0,0};f.velocity={250,0,0};
    f.up={0,0,1};f.boresight={1,0,0};f.receiverSeed=19;
    f.bodyAxes=std::array<weapon::V3,3>{weapon::V3{1,0,0},weapon::V3{0,1,0},weapon::V3{0,0,1}};
    SeekerSampleV2 s;s.direction={std::cos(bearing),std::sin(bearing),0};s.signal=1;
    s.rangeM=1000;s.rangeRateMs=-500;s.scene=SeekerSceneV3{1000,5};f.samples.push_back(s);return f;
}
}

TEST_CASE("Finite engineering seeker selection is admitted without changing legacy V2", "[weapon-sensor-engineering]") {
    using namespace fsim::ms;
    using namespace fsim::ms::components;
    std::vector<std::string> errors;
    const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);
    REQUIRE(errors.empty());
    const auto card=cards.at("aim-120c");
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"aim-120c.json",*card,profile,error));
    auto descriptor=*profile.find("seeker");
    descriptor.implementationId="fsim.seeker.active_radar.v3";
    descriptor.bindings["measurement_model"]="radiometric_engineering_v1";
    descriptor.bindings["servo_model"]="stabilized_finite_v1";
    weapon::SeekerConfigV1 config;
    INFO(error);
    REQUIRE(weapon::configureSeeker(descriptor,config,error));
    REQUIRE(config.engineering.version==3);
    const auto original=config;
    descriptor.parameters.erase("gimbal_rate_rad_s");
    CHECK_FALSE(weapon::configureSeeker(descriptor,config,error));
    CHECK(config.engineering.gimbalRateRadS==original.engineering.gimbalRateRadS);
}

TEST_CASE("Current physical receiver descriptors normalize atomically and require complete policies", "[weapon-sensor-engineering]") {
    std::vector<std::string> errors;std::string error;
    const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    unsigned count=0;
    for(const auto& [id,card]:cards)if(card->storeClass=="aam") {
        INFO(id);ProfileV1 profile;REQUIRE(readProfileV1(library/(id+".json"),*card,profile,error));
        const auto original=*profile.find("seeker");DescriptorV1 normalized;ReceiverConfigurationV3 physical;
        REQUIRE(receiverConfigurationV3(original,normalized,physical,error));
        CHECK((physical.reticle||physical.inherited.single.kind!=PhysicalReceiverKindV1::Scalar));CHECK(normalized.bindings.count("receiver_model")==0);
        CHECK(physical.inherited.dual.has_value()==(id=="mica-ir"));CHECK(physical.inherited.secondBandScene.has_value()==physical.inherited.dual.has_value());
        weapon::SeekerConfigV1 retained,current;REQUIRE(weapon::configureSeeker(normalized,retained,error));
        REQUIRE(weapon::configureSeeker(original,current,error));CHECK(current.acquireM==retained.acquireM);
        CHECK(current.engineering.samplePeriodS==retained.engineering.samplePeriodS);
        const auto beforeKind=physical.inherited.single.kind;const auto beforeId=normalized.implementationId;
        auto broken=original;const auto missing=physical.inherited.single.kind==PhysicalReceiverKindV1::Radio?"receiver_rf_gate_m":"receiver_psf_sigma_rad";
        broken.parameters.erase(missing);CHECK_FALSE(receiverConfigurationV3(broken,normalized,physical,error));
        CHECK(physical.inherited.single.kind==beforeKind);CHECK(normalized.implementationId==beforeId);
        broken=original;broken.parameters.at(missing).uncertainty=0;
        CHECK_FALSE(receiverConfigurationV3(broken,normalized,physical,error));
        ++count;
    }
    REQUIRE(count==24);
}

TEST_CASE("Actual MICA dual-band detector retains dynamic range on a settled 300 K background", "[weapon-sensor-engineering][thermal-dynamic-range]") {
    std::vector<std::string> errors;std::string error;
    const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    const auto card=cards.at("mica-ir");ProfileV1 profile;
    REQUIRE(readProfileV1(library/"mica-ir.json",*card,profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV2 physical;
    REQUIRE(receiverConfigurationV2(*profile.find("seeker"),scalar,physical,error));REQUIRE(physical.dual);REQUIRE(physical.secondBandScene);
    ThermalDualBandReceiverV1 receiver;REQUIRE(receiver.configure(*physical.dual,error));
    ThermalDualBandSceneFrameV1 scene;
    for(unsigned band=0;band<2;++band) {
        auto& channel=scene.channels[band];const auto& policy=band?*physical.secondBandScene:physical.single.scene;
        channel.up={0,0,1};channel.groundHorizonCosine=1;
        channel.skyRadianceWm2Sr=channel.groundRadianceWm2Sr=ThermalImageReceiverV1::bandRadiance(300,1,policy.lowUm,policy.highUm);
    }
    ThermalDualBandResultV1 result;
    for(unsigned tick=0;tick<3;++tick) {
        for(auto& channel:scene.channels)channel.timeS=tick*.02;
        result=receiver.expose(scene,{1,0,0},715);REQUIRE(result.valid);
    }
    for(unsigned band=0;band<2;++band) {
        CAPTURE(band);const auto& image=result.channels[band];REQUIRE(image.valid);REQUIRE_FALSE(image.expectedElectrons.empty());
        const double expected=*std::max_element(image.expectedElectrons.begin(),image.expectedElectrons.end());
        CAPTURE(expected,image.config.fullWellElectrons);
        CHECK(expected>0);CHECK(expected<image.config.fullWellElectrons*.5);
        CHECK(std::all_of(image.measuredElectrons.begin(),image.measuredElectrons.end(),[&](double charge){return charge<image.config.fullWellElectrons;}));
    }
}

TEST_CASE("RF receiver matches independent monostatic and bistatic energy equations", "[weapon-sensor-engineering]") {
    Receiver r;auto c=r.spec.seekerModel;SeekerSceneV3 s{2000,3};
    const double pi=std::acos(-1.),lambda=299792458./c.engineering.frequencyHz;
    const double gain=std::pow(10.,c.emitterPeakGainDbi/10.);
    const double expected=c.emitterPowerW*gain*gain*3*lambda*lambda*.02/
        (std::pow(4*pi,3)*std::pow(2000.,4)*1.380649e-23*c.engineering.temperatureK*
         std::pow(10.,(c.engineering.noiseFigureDb+c.engineering.systemLossDb)/10.));
    CHECK(EngineeringSeekerV3::rfSnr(c,s,false,.02)==Catch::Approx(expected).epsilon(1e-12));
    const double near=EngineeringSeekerV3::rfSnr(c,s,false,.02);s.rangeM*=2;
    CHECK(EngineeringSeekerV3::rfSnr(c,s,false,.02)==Catch::Approx(near/16));
    s.transmitterRangeM=4000;s.transmitterPowerW=1000;s.transmitterGainLinear=1000;s.transmitterFrequencyHz=10e9;
    const double bistatic=EngineeringSeekerV3::rfSnr(c,s,true,.02);s.transmitterRangeM*=2;
    CHECK(EngineeringSeekerV3::rfSnr(c,s,true,.02)==Catch::Approx(bistatic/4));
    s.receiverExcessLossDb=10;
    CHECK(EngineeringSeekerV3::rfSnr(c,s,true,.02)==Catch::Approx(bistatic/40));
    s.transmitterPowerW=0;CHECK(EngineeringSeekerV3::rfSnr(c,s,true,.02)==0);
}

TEST_CASE("IR band radiometry has independent inverse square extinction and temperature response", "[weapon-sensor-engineering]") {
    Receiver r("aim-9x");auto c=r.spec.seekerModel;SeekerSceneV3 s{1000,0};
    const auto first=EngineeringSeekerV3::irSnr(c,s,1);s.rangeM=2000;
    CHECK(EngineeringSeekerV3::irSnr(c,s,1)==Catch::Approx(first*.25*std::exp(-c.engineering.irExtinctionPerM*1000)));
    CHECK(EngineeringSeekerV3::bandFraction(900,3,5)>0);
    // Independent dimensionless x=c2/(lambda*T) midpoint100000 integral.
    CHECK(EngineeringSeekerV3::bandFraction(900,3,5)==Catch::Approx(.35894628827477898).margin(2e-8));
    CHECK(EngineeringSeekerV3::bandFraction(900,.1,100)==Catch::Approx(1).margin(.01));
    CHECK(EngineeringSeekerV3::bandFraction(900,5,3)==0);
    const auto old=EngineeringSeekerV3::irSnr(c,s,1);c.engineering.backgroundWm2*=100;
    CHECK(EngineeringSeekerV3::irSnr(c,s,1)<old);
    s.rangeM=0;CHECK(EngineeringSeekerV3::irSnr(c,s,1)==0);
}

TEST_CASE("Thermal source aspect follows body orientation independently of speed and distance", "[weapon-sensor-engineering]") {
    const weapon::V3 forward{.5,std::sqrt(3.)/2,0};
    CHECK(EngineeringSeekerV3::sourceAspectCosineV3({1,0,0},forward)==Catch::Approx(.5));
    CHECK(EngineeringSeekerV3::sourceAspectCosineV3({100000,0,0},scale(forward,77))==Catch::Approx(.5));
    CHECK(EngineeringSeekerV3::sourceAspectCosineV3({-100000,0,0},forward)==Catch::Approx(-.5));
    CHECK(std::isnan(EngineeringSeekerV3::sourceAspectCosineV3({},forward)));
}

TEST_CASE("Sensor observable ordering and aligned cadence preserve seeded measurements", "[weapon-sensor-engineering]") {
    Receiver a,b,c;SeekerResultV2 x,y,z;
    for(unsigned i=0;i<=20;++i) {
        auto f=scene(i*.01);auto other=f.samples[0];other.direction=unit({1,.03,0});other.signal=2;
        f.samples.push_back(other);x=a.sensor->sample(f);
        std::reverse(f.samples.begin(),f.samples.end());y=b.sensor->sample(f);
        if(i) {auto extra=scene(i*.01-.005);extra.samples.push_back(other);c.sensor->sample(extra);}
        std::reverse(f.samples.begin(),f.samples.end());z=c.sensor->sample(f);
        REQUIRE(x.tracking==y.tracking);REQUIRE(x.tracking==z.tracking);
        if(x.tracking) {REQUIRE(x.track);REQUIRE(y.track);REQUIRE(z.track);
            CHECK(x.track->direction.y==y.track->direction.y);
            CHECK(x.track->direction.y==z.track->direction.y);
            CHECK(x.track->sampleTimeS==z.track->sampleTimeS);
        }
    }
}

TEST_CASE("Physical scene clock is read only and follows accepted exposures rather than delivery", "[weapon-scene-clock]") {
    Receiver receiver;
    auto* clock=dynamic_cast<SeekerSceneClockV1*>(receiver.provider.get());REQUIRE(clock);
    auto f=scene();CHECK(clock->sceneCaptureDueV1(0));CHECK(clock->sceneCaptureDueV1(0));
    receiver.sensor->sample(f);
    CHECK_FALSE(clock->sceneCaptureDueV1(0));CHECK_FALSE(clock->sceneCaptureDueV1(.001));
    CHECK_FALSE(clock->sceneCaptureDueV1(.019));CHECK(clock->sceneCaptureDueV1(.02));
    f.timeS=f.ageS=.01;const auto delivered=receiver.sensor->sample(f);REQUIRE(delivered.tracking);
    CHECK_FALSE(clock->sceneCaptureDueV1(.019));CHECK(clock->sceneCaptureDueV1(.02));
    f.timeS=-1;CHECK_FALSE(receiver.sensor->sample(f).valid);
    CHECK(clock->sceneCaptureDueV1(.02));
    f.timeS=f.ageS=.025;receiver.sensor->sample(f);
    CHECK_FALSE(clock->sceneCaptureDueV1(.044));CHECK(clock->sceneCaptureDueV1(.045));
    CHECK_FALSE(clock->sceneCaptureDueV1(std::numeric_limits<double>::quiet_NaN()));
    CHECK_FALSE(clock->sceneCaptureDueV1(std::numeric_limits<double>::infinity()));
    receiver.sensor->resetTracking();CHECK(clock->sceneCaptureDueV1(0));
}

TEST_CASE("Legacy quadrant compatibility retains a resolved source at attainable angular rate", "[weapon-quadrant-servo][thermal-quadrant-compatibility]") {
    Receiver receiver("aim-9m",false,true);
    auto* sensor=dynamic_cast<ThermalSeekerExecutionV1*>(receiver.provider.get());REQUIRE(sensor);
    const auto* optical=sensor->thermalPolicyV1();REQUIRE(optical);
    const double halfField=.5*std::min(optical->width,optical->height)*optical->pixelPitchRad;
    unsigned observed=0,tracked=0;double maximumError=0;
    for(unsigned i=0;i<81;++i) {
        const double now=i*.01,angle=.5*std::max(0.,now-.2);
        auto f=scene(now);f.samples.clear();f.position={6378137.,0,0};
        ThermalSceneFrameV1 image;image.timeS=now;image.up={0,0,1};
        image.skyRadianceWm2Sr=image.groundRadianceWm2Sr=1;
        SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=now;nav.position=f.position;nav.velocity=f.velocity;nav.bodyToEcef=*f.bodyAxes;
        image.navigation=nav;
        ThermalRadiancePatchV1 patch;patch.direction={std::cos(angle),std::sin(angle),0};
        patch.directionRate={-.5*std::sin(angle),.5*std::cos(angle),0};
        patch.rangeM=1000;patch.widthRad=patch.heightRad=.004;patch.radianceWm2Sr=100;
        patch.transmission=1;patch.pathRadianceWm2Sr=1;
        image.patches.push_back(patch);image.refereeAttributionIndices.push_back(std::nullopt);
        const auto result=sensor->sampleThermalV1(f,image);
        if(now>=.3) {
            ++observed;tracked+=result.tracking;
            const double error=std::acos(std::clamp(dot(unit(patch.direction),unit(result.pointingDirection)),-1.,1.));
            maximumError=std::max(maximumError,error);
        }
    }
    INFO("max pointing error="<<maximumError<<" physical half field="<<halfField<<" tracking="<<tracked<<'/'<<observed);
    REQUIRE(observed>0);CHECK(tracked==observed);CHECK(maximumError<halfField);
}

TEST_CASE("Legacy quadrant compatibility centering survives rejected precise associations", "[weapon-quadrant-acquisition][thermal-quadrant-compatibility]") {
    for(unsigned axis=0;axis<2;++axis)for(const double sign:{-1.,1.}) {
        Receiver receiver("aim-9m",false,true);
        auto* sensor=dynamic_cast<ThermalSeekerExecutionV1*>(receiver.provider.get());REQUIRE(sensor);
        const auto* optical=sensor->thermalPolicyV1();REQUIRE(optical);
        const double halfField=.5*std::min(optical->width,optical->height)*optical->pixelPitchRad;
        double maximumError=0;unsigned measured=0;
        for(unsigned i=0;i<31;++i) {
            const double now=i/30.;auto f=scene(now);f.dtS=1./30;f.samples.clear();f.position={6378137.,0,0};
            ThermalSceneFrameV1 image;image.timeS=now;image.up={0,0,1};
            image.skyRadianceWm2Sr=image.groundRadianceWm2Sr=1;
            SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=now;nav.position=f.position;nav.velocity=f.velocity;nav.bodyToEcef=*f.bodyAxes;
            image.navigation=nav;
            ThermalRadiancePatchV1 patch;patch.direction={std::cos(.03),axis?0:sign*std::sin(.03),axis?sign*std::sin(.03):0};
            patch.rangeM=1000;patch.widthRad=patch.heightRad=.004;patch.radianceWm2Sr=100;
            patch.transmission=1;patch.pathRadianceWm2Sr=1;image.patches.push_back(patch);image.refereeAttributionIndices.push_back(std::nullopt);
            const auto result=sensor->sampleThermalV1(f,image);if(i)REQUIRE(result.valid);
            if(now>=.2) {
                ++measured;maximumError=std::max(maximumError,std::acos(std::clamp(dot(unit(patch.direction),unit(result.pointingDirection)),-1.,1.)));
            }
        }
        INFO("axis="<<axis<<" sign="<<sign<<" max optical error="<<maximumError);
        REQUIRE(measured>0);CHECK(maximumError<halfField);
    }
}

TEST_CASE("Legacy stationary quadrant acquisition settles within detector sampling resolution", "[weapon-quadrant-settling][thermal-quadrant-compatibility]") {
    Receiver receiver("aim-9m",false,true);
    auto* sensor=dynamic_cast<ThermalSeekerExecutionV1*>(receiver.provider.get());REQUIRE(sensor);
    const auto* optical=sensor->thermalPolicyV1();REQUIRE(optical);
    const auto direction=unit(weapon::V3{1,.027,-.019});
    std::ofstream trace;
    if(const auto* path=std::getenv("FSIM_QUADRANT_TRACE")) {
        trace.open(path);REQUIRE(trace.good());trace.precision(17);
        trace<<"time_s,error_rad,point_x,point_y,point_z,tracking,image_s,features,snr,q0,q1,q2,q3\n";
    }
    double maximumError=0;unsigned observed=0,tracked=0;
    for(unsigned i=0;i<121;++i) {
        const double now=i/30.;auto f=scene(now);f.dtS=1./30;f.samples.clear();f.position={6378137.,0,0};
        ThermalSceneFrameV1 image;image.timeS=now;image.up={0,0,1};
        image.skyRadianceWm2Sr=image.groundRadianceWm2Sr=1;
        SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=now;nav.position=f.position;nav.velocity=f.velocity;nav.bodyToEcef=*f.bodyAxes;image.navigation=nav;
        ThermalRadiancePatchV1 patch;patch.direction=direction;patch.rangeM=1000;
        patch.widthRad=patch.heightRad=.004;patch.radianceWm2Sr=100;patch.transmission=1;patch.pathRadianceWm2Sr=1;
        image.patches.push_back(patch);image.refereeAttributionIndices.push_back(std::nullopt);
        const auto result=sensor->sampleThermalV1(f,image);
        if(trace.is_open()) {
            trace<<now<<','<<std::acos(std::clamp(dot(direction,unit(result.pointingDirection)),-1.,1.))
                <<','<<result.pointingDirection.x<<','<<result.pointingDirection.y<<','<<result.pointingDirection.z<<','<<result.tracking;
            if(const auto* detector=sensor->thermalImageV1()) {
                trace<<','<<detector->timeS<<','<<detector->features.size()<<',';
                if(!detector->features.empty())trace<<detector->features.front().signalToNoise;
                for(const double charge:detector->quadrantElectrons)trace<<','<<charge;
            } else trace<<",,,,,,,";
            trace<<'\n';
        }
        if(now>=2) {
            REQUIRE(result.valid);++observed;tracked+=result.tracking;
            maximumError=std::max(maximumError,std::acos(std::clamp(dot(direction,unit(result.pointingDirection)),-1.,1.)));
        }
    }
    INFO("stationary optical error="<<maximumError<<" tracking="<<tracked<<'/'<<observed);
    CHECK(maximumError<2*optical->pixelPitchRad);CHECK(tracked==observed);
}

TEST_CASE("Legacy noninvertible quadrant exposures retain centering without inventing a resolved LOS rate", "[weapon-quadrant-rate-observability][thermal-quadrant-compatibility]") {
    Receiver receiver("aim-9m",false,true);
    auto* sensor=dynamic_cast<ThermalSeekerExecutionV1*>(receiver.provider.get());REQUIRE(sensor);
    const auto direction=unit(weapon::V3{1,.027,-.019});
    std::map<double,bool> unresolved;
    unsigned coarseTracks=0,resolvedRates=0;
    for(unsigned i=0;i<121;++i) {
        const double now=i/30.;auto f=scene(now);f.dtS=1./30;f.samples.clear();f.position={6378137.,0,0};
        ThermalSceneFrameV1 image;image.timeS=now;image.up={0,0,1};
        image.skyRadianceWm2Sr=image.groundRadianceWm2Sr=1;
        SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=now;nav.position=f.position;nav.velocity=f.velocity;nav.bodyToEcef=*f.bodyAxes;image.navigation=nav;
        ThermalRadiancePatchV1 patch;patch.direction=direction;patch.rangeM=1000;
        patch.widthRad=patch.heightRad=.004;patch.radianceWm2Sr=100;patch.transmission=1;patch.pathRadianceWm2Sr=1;
        image.patches.push_back(patch);image.refereeAttributionIndices.push_back(std::nullopt);
        const auto result=sensor->sampleThermalV1(f,image);
        const auto* detector=sensor->thermalImageV1();
        if(detector&&detector->features.size()==1) {
            const auto& q=detector->quadrantElectrons;const double total=q[0]+q[1]+q[2]+q[3];
            const double noise=detector->config.detectionSigma*2/detector->features.front().signalToNoise;
            const double dx=(q[1]+q[3]-q[0]-q[2])/total,dy=(q[0]+q[1]-q[2]-q[3])/total;
            const auto clipped=[&](double ratio){return std::abs(ratio)>noise&&std::abs(ratio)+noise>=1;};
            unresolved[detector->timeS]=clipped(dx)||clipped(dy);
        }
        if(result.tracking&&result.track&&result.measurementTimeS) {
            const auto capture=unresolved.find(*result.measurementTimeS);REQUIRE(capture!=unresolved.end());
            if(capture->second) {++coarseTracks;CHECK_FALSE(result.track->rateAvailable);}
            else if(result.track->rateAvailable)++resolvedRates;
        }
    }
    REQUIRE(coarseTracks>0);REQUIRE(resolvedRates>0);
}

TEST_CASE("Unavailable coarse quadrant seed cannot supply a resolved position for the first fresh rate",
          "[weapon-quadrant-rate-observability][weapon-quadrant-seed-observability]") {
    Receiver first("aim-9m",false,true),second("aim-9m",false,true);
    auto* opticalA=dynamic_cast<ThermalSeekerExecutionV1*>(first.provider.get());REQUIRE(opticalA);
    auto* opticalB=dynamic_cast<ThermalSeekerExecutionV1*>(second.provider.get());REQUIRE(opticalB);
    const auto frame=[](double now) {
        auto f=scene(now);f.dtS=1./30;f.samples.clear();f.position={6378137.,0,0};return f;
    };
    const auto opticalScene=[](const SeekerFrameV2& f,weapon::V3 direction) {
        ThermalSceneFrameV1 image;image.timeS=f.timeS;image.up={0,0,1};
        image.skyRadianceWm2Sr=image.groundRadianceWm2Sr=1;
        SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=f.timeS;
        nav.position=f.position;nav.velocity=f.velocity;nav.bodyToEcef=*f.bodyAxes;image.navigation=nav;
        ThermalRadiancePatchV1 patch;patch.direction=direction;patch.rangeM=1000;
        patch.widthRad=patch.heightRad=.004;patch.radianceWm2Sr=100;
        patch.transmission=1;patch.pathRadianceWm2Sr=1;
        image.patches.push_back(patch);image.refereeAttributionIndices.push_back(std::nullopt);return image;
    };
    const auto unresolved=[](const ThermalImageResultV1& image) {
        REQUIRE(image.features.size()==1);
        const auto& q=image.quadrantElectrons;const double total=q[0]+q[1]+q[2]+q[3];
        const double noise=image.config.detectionSigma*2/image.features.front().signalToNoise;
        const double dx=(q[1]+q[3]-q[0]-q[2])/total,dy=(q[0]+q[1]-q[2]-q[3])/total;
        const auto clipped=[&](double ratio){return std::abs(ratio)>noise&&std::abs(ratio)+noise>=1;};
        return clipped(dx)||clipped(dy);
    };
    std::map<double,bool> capturedA;
    std::optional<SeekerTrackV2> coarse;
    // Obtain the seed from an actual noninvertible physical exposure, rather
    // than manufacturing an unavailable flag or a detector/target identity.
    for(unsigned i=0;i<121&&!coarse;++i) {
        const auto f=frame(i/30.);
        const auto result=opticalA->sampleThermalV1(f,opticalScene(f,unit(weapon::V3{1,.027,-.019})));
        if(const auto* image=opticalA->thermalImageV1();image&&image->features.size()==1)
            capturedA[image->timeS]=unresolved(*image);
        if(result.tracking&&result.track&&result.measurementTimeS) {
            const auto measured=capturedA.find(*result.measurementTimeS);REQUIRE(measured!=capturedA.end());
            if(measured->second)coarse=*result.track;
        }
    }
    REQUIRE(coarse);REQUIRE_FALSE(coarse->rateAvailable);
    second.sensor->seedTrack(*coarse);
    std::map<double,bool> capturedB;unsigned freshResolved=0;
    double lastPublished=-1;
    // Centre a separate stationary numerical source in B's admitted seed
    // direction. Only B's measured resolved exposures can establish its rate.
    // No physical policy, source intensity/noise or sampler convention changes.
    for(unsigned i=1;i<=30&&freshResolved<2;++i) {
        auto f=frame(coarse->sampleTimeS+i/30.);
        // Hold B's actual body cone aligned with its seed direction, so servo
        // return-to-boresight cannot make the second exposure unresolved.
        const auto forward=unit(coarse->direction);
        const auto right=unit(cross(weapon::V3{0,0,1},forward));
        f.boresight=forward;f.bodyAxes=std::array<weapon::V3,3>{forward,right,cross(forward,right)};
        const auto result=opticalB->sampleThermalV1(f,opticalScene(f,coarse->direction));
        if(const auto* image=opticalB->thermalImageV1();image&&image->features.size()==1)
            capturedB[image->timeS]=unresolved(*image);
        if(result.tracking&&result.track&&result.measurementTimeS&&*result.measurementTimeS>lastPublished) {
            lastPublished=*result.measurementTimeS;
            const auto measured=capturedB.find(lastPublished);REQUIRE(measured!=capturedB.end());
            REQUIRE_FALSE(measured->second);++freshResolved;
            if(freshResolved==1)CHECK_FALSE(result.track->rateAvailable);
            else CHECK(result.track->rateAvailable);
        }
    }
    REQUIRE(freshResolved==2);
}

TEST_CASE("Launch navigation initialization preserves policy and rejects malformed estimates atomically", "[weapon-launch-navigation]") {
    for(const auto* id:{"asraam","pl-10","mica-ir","aim-9m","aim-9x"}) {
        Receiver receiver(id);INFO(id);
        auto* initializer=dynamic_cast<SeekerLaunchNavigationV1*>(receiver.provider.get());REQUIRE(initializer);
        const auto* clock=dynamic_cast<SeekerSceneClockV1*>(receiver.provider.get());REQUIRE(clock);
        const bool enabled=receiver.spec.inertialMidcourse||receiver.spec.launcherUpdates;
        weapon::Observation estimate{41,{1000,200,0},{0,20,0},false};
        CHECK(initializer->seedLaunchNavigationV1(estimate)==enabled);
        CHECK(clock->sceneCaptureDueV1(10));
        auto malformed=estimate;malformed.position.x=std::numeric_limits<double>::quiet_NaN();
        CHECK_FALSE(initializer->seedLaunchNavigationV1(malformed));
        auto f=scene(10);f.ageS=0;f.samples.clear();receiver.sensor->sample(f);
        f.timeS=10.01;f.ageS=.01;const auto result=receiver.sensor->sample(f);REQUIRE(result.valid);
        CHECK(result.midcourseEstimate.has_value()==receiver.spec.inertialMidcourse);
        if(result.midcourseEstimate) {
            CHECK(result.midcourseEstimate->position.x==1000);
            CHECK(result.midcourseEstimate->position.y==200);
            CHECK(result.midcourseEstimate->sampleTimeS==10);
        }
    }
}

TEST_CASE("Physical MICA factory retains its authorized launch navigation through dual-band configuration", "[weapon-launch-navigation][weapon-sensor-engineering]") {
    std::vector<std::string> errors;std::string error;
    const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    const auto card=cards.at("mica-ir");ProfileV1 profile;
    REQUIRE(readProfileV1(library/"mica-ir.json",*card,profile,error));
    weapon::Spec spec;REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
    REQUIRE(spec.inertialMidcourse);
    const auto& timing=spec.seekerModel.engineering;
    REQUIRE(timing.processingDelayS>0);REQUIRE(timing.processingDelayS<timing.samplePeriodS);
    constexpr double launchS=10;
    const weapon::Observation initial{41,{1000,200,0},{0,20,0},false};
    for(const bool seeded:{true,false}) {
        INFO("authorized initial estimate="<<seeded);
        ProviderContextV1 context{*card,profile,spec,seeded?std::optional<weapon::Observation>{initial}:std::nullopt};
        auto provider=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);
        INFO(error);REQUIRE(provider);
        auto* physical=dynamic_cast<ThermalDualBandSeekerExecutionV1*>(provider.get());REQUIRE(physical);
        REQUIRE(physical->thermalDualBandPolicyV1());
        const auto sample=[&](double now) {
            auto frame=scene(now);frame.ageS=now-launchS;frame.samples.clear();
            // No target scene, launcher update or post-factory seed can restore
            // a navigation estimate lost during physical receiver setup.
            frame.supported=false;frame.launcherEstimate.reset();
            ThermalDualBandSceneFrameV1 bands;
            for(auto& channel:bands.channels) {
                channel.timeS=now;channel.up={0,0,1};channel.groundHorizonCosine=1;
                SeekerNavigationPoseV1 own;own.valid=true;own.timeS=now;
                own.position=frame.position;own.velocity=frame.velocity;own.bodyToEcef=*frame.bodyAxes;
                channel.navigation=own;
            }
            return physical->sampleThermalDualBandV1(frame,bands);
        };
        CHECK_FALSE(sample(launchS).valid);
        const auto first=sample(launchS+timing.processingDelayS);REQUIRE(first.valid);
        CHECK(first.measurementTimeS==launchS);
        CHECK(first.deliveredTimeS==launchS+timing.processingDelayS);
        CHECK(first.midcourseEstimate.has_value()==seeded);
        if(seeded) {
            REQUIRE(first.midcourseEstimate);
            CHECK(first.midcourseEstimate->position.x==initial.position.x);
            CHECK(first.midcourseEstimate->position.y==initial.position.y);
            CHECK(first.midcourseEstimate->sampleTimeS==launchS);
        }
        const auto nextExposure=launchS+timing.samplePeriodS;
        sample(nextExposure);
        const auto second=sample(nextExposure+timing.processingDelayS);REQUIRE(second.valid);
        CHECK(second.measurementTimeS==nextExposure);
        CHECK(second.deliveredTimeS==nextExposure+timing.processingDelayS);
        CHECK(second.midcourseEstimate.has_value()==seeded);
        if(seeded) {
            REQUIRE(second.midcourseEstimate);
            CHECK(second.midcourseEstimate->position.x==initial.position.x);
            CHECK(second.midcourseEstimate->position.y==Catch::Approx(initial.position.y+initial.velocity.y*timing.samplePeriodS));
            CHECK(second.midcourseEstimate->sampleTimeS==nextExposure);
        }
    }
}

TEST_CASE("Scene clock queries preserve all sampled detector outputs and random history", "[weapon-scene-clock]") {
    Receiver baseline,queried;
    const auto* clock=dynamic_cast<const SeekerSceneClockV1*>(queried.provider.get());REQUIRE(clock);
    for(unsigned i=0;i<400;++i) {
        auto f=scene(i/120.);if(i>100&&i<180)f.samples.clear();
        (void)clock->sceneCaptureDueV1(f.timeS);(void)clock->sceneCaptureDueV1(f.timeS+100);
        const auto a=baseline.sensor->sample(f),b=queried.sensor->sample(f);
        CHECK(a.valid==b.valid);CHECK(a.tracking==b.tracking);CHECK(a.attributionIndex==b.attributionIndex);
        CHECK(a.measurementTimeS==b.measurementTimeS);CHECK(a.deliveredTimeS==b.deliveredTimeS);
        CHECK(a.pointingDirection.x==b.pointingDirection.x);CHECK(a.pointingDirection.y==b.pointingDirection.y);
        REQUIRE(bool(a.track)==bool(b.track));
        if(a.track){CHECK(a.track->direction.y==b.track->direction.y);CHECK(a.track->sampleTimeS==b.track->sampleTimeS);}
    }
}

TEST_CASE("Physical ARH terminal coast retains measured geometry and its correlated own navigation origin", "[weapon-terminal-coast]") {
    std::array<LauncherEstimateV2,2> coasts;
    for(unsigned run=0;run<2;++run) {
        Receiver receiver("aim-120c",false);
        auto* radio=dynamic_cast<RadioSeekerExecutionV1*>(receiver.provider.get());REQUIRE(radio);
        const auto* policy=radio->radioPolicyV1();REQUIRE(policy);
        const weapon::V3 bias=run?weapon::V3{100,20,-30}:weapon::V3{};
        const auto observe=[&](double now,bool visible) {
            auto frame=scene(now);frame.samples.clear();frame.position={250*now,0,0};
            RadioSceneFrameV1 physical;physical.timeS=now;physical.frequencyHz=policy->frequencyHz;physical.up=frame.up;
            SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=now;
            nav.position=add(frame.position,bias);nav.velocity=frame.velocity;nav.bodyToEcef=*frame.bodyAxes;
            physical.navigation=nav;
            if(visible) {
                const weapon::V3 relative{1500-500*now,60*now+(now>=.8?150:0),0};
                RadioEchoSceneV1 echo;echo.direction=unit(relative);
                echo.receiverRangeM=echo.delayRangeM=norm(relative);echo.rcsM2=1e6;
                echo.dopplerHz=-2*dot(weapon::V3{-500,60,0},echo.direction)/(299792458./policy->frequencyHz);
                physical.echoes.push_back(echo);physical.refereeAttributionIndices.push_back(std::nullopt);
            }
            return radio->sampleRadioV1(frame,physical);
        };
        std::optional<LauncherEstimateV2> last;
        SeekerResultV2 coast;
        for(unsigned tick=0;tick<=70;++tick) {
            const double now=tick*.01;const auto result=observe(now,tick<15);
            if(result.tracking&&result.track&&result.track->rateAvailable) {
                REQUIRE(result.midcourseEstimate);
                const auto& track=*result.track;REQUIRE(track.rangeM);REQUIRE(track.rangeRateMs);
                const auto captureOwn=add(weapon::V3{250*track.sampleTimeS,0,0},bias);
                const auto expectedPosition=add(captureOwn,scale(track.direction,*track.rangeM));
                const auto expectedVelocity=add(weapon::V3{250,0,0},add(scale(track.direction,*track.rangeRateMs),scale(track.directionRate,*track.rangeM)));
                CHECK(norm(subtract(result.midcourseEstimate->position,expectedPosition))<1e-9);
                CHECK(norm(subtract(result.midcourseEstimate->velocity,expectedVelocity))<1e-9);
                CHECK(result.midcourseEstimate->sampleTimeS==track.sampleTimeS);
                REQUIRE(result.deliveredTimeS);CHECK(*result.deliveredTimeS>track.sampleTimeS);
                last=result.midcourseEstimate;
            }
            if(tick==50)coast=result;
        }
        REQUIRE(last);REQUIRE(coast.valid);CHECK_FALSE(coast.tracking);CHECK_FALSE(coast.attributionIndex);
        REQUIRE(coast.midcourseEstimate);CHECK(coast.midcourseEstimate->sampleTimeS==last->sampleTimeS);
        CHECK(norm(subtract(coast.midcourseEstimate->position,last->position))<1e-12);
        CHECK(norm(subtract(coast.midcourseEstimate->velocity,last->velocity))<1e-12);
        CHECK_FALSE(coast.track); // Expired association is not a manufactured terminal contact.
        coasts[run]=*coast.midcourseEstimate;
        CHECK(std::abs(last->velocity.y)>1); // Genuine transverse motion, not detector noise alone.
        observe(.8,true);const auto replacement=observe(.81,true);
        REQUIRE(replacement.tracking);REQUIRE(replacement.track);CHECK_FALSE(replacement.track->rateAvailable);
        CHECK_FALSE(replacement.midcourseEstimate);
        observe(.82,true);const auto replacementState=observe(.83,true);
        REQUIRE(replacementState.tracking);REQUIRE(replacementState.midcourseEstimate);
        CHECK(replacementState.midcourseEstimate->sampleTimeS>=.8);
        receiver.sensor->resetTracking();SeekerResultV2 reset;
        for(unsigned tick=100;tick<=105;++tick)reset=observe(tick*.01,false);
        REQUIRE(reset.valid);CHECK_FALSE(reset.midcourseEstimate);
    }
    CHECK(norm(subtract(subtract(coasts[1].position,coasts[0].position),weapon::V3{100,20,-30}))<1e-9);
    CHECK(norm(subtract(coasts[1].velocity,coasts[0].velocity))<1e-9);
    CHECK(coasts[0].sampleTimeS==coasts[1].sampleTimeS);
}

TEST_CASE("Physical ARH coast combines admitted support transverse velocity with its own observed Doppler", "[weapon-supported-coast]") {
    // The repeated prior has its original state epoch. No source identity,
    // truth velocity or receiver attribution is supplied to the sensor.
    for(unsigned mode=0;mode<7;++mode)for(const bool biased:{false,true}) {
        CAPTURE(mode,biased);
        Receiver receiver("aim-120c",false);
        auto* radio=dynamic_cast<RadioSeekerExecutionV1*>(receiver.provider.get());REQUIRE(radio);
        const auto* policy=radio->radioPolicyV1();REQUIRE(policy);
        const weapon::V3 bias=biased?weapon::V3{100,20,-30}:weapon::V3{};
        const weapon::V3 targetVelocity{-250,60,0},ownVelocity{250,0,0};
        const auto observe=[&](double now,bool visible) {
            auto frame=scene(now);frame.samples.clear();frame.position={250*now,0,0};
            frame.supported=mode!=2;
            LauncherEstimateV2 prior{{1500,0,0},targetVelocity,0};
            if(mode==3&&now>=.08)prior.position.y=10000; // Observable cue inconsistency.
            if(mode==4)prior.velocity.y=160; // Unobserved wrong transverse prior remains an error.
            if(mode==5&&now<.06){prior.sampleTimeS=.01;prior.position=add(prior.position,scale(prior.velocity,.01));}
            if(mode==6&&now>=.06)frame.supported=false;
            if(mode!=1&&now>=.01)frame.launcherEstimate=prior;
            RadioSceneFrameV1 physical;physical.timeS=now;physical.frequencyHz=policy->frequencyHz;physical.up=frame.up;
            SeekerNavigationPoseV1 nav;nav.valid=true;nav.timeS=now;nav.position=add(frame.position,bias);
            nav.velocity=ownVelocity;nav.bodyToEcef=*frame.bodyAxes;physical.navigation=nav;
            if(visible) {
                const weapon::V3 relative{1500-500*now,60*now,0};
                RadioEchoSceneV1 echo;echo.direction=unit(relative);echo.receiverRangeM=echo.delayRangeM=norm(relative);
                echo.rcsM2=1e6;echo.dopplerHz=-2*dot(subtract(targetVelocity,ownVelocity),echo.direction)/(299792458./policy->frequencyHz);
                physical.echoes.push_back(echo);physical.refereeAttributionIndices.push_back(std::nullopt);
            }
            return radio->sampleRadioV1(frame,physical);
        };
        unsigned supportedSamples=0,fallbackSamples=0;std::optional<LauncherEstimateV2> last;
        for(unsigned tick=0;tick<=15;++tick) {
            const auto result=observe(tick*.01,true);
            if(!result.tracking||!result.track||!result.track->rateAvailable)continue;
            const auto& track=*result.track;REQUIRE(track.rangeM);REQUIRE(track.rangeRateMs);REQUIRE(result.midcourseEstimate);
            const auto own=add(weapon::V3{250*track.sampleTimeS,0,0},bias);
            CHECK(norm(subtract(result.midcourseEstimate->position,add(own,scale(track.direction,*track.rangeM))))<1e-9);
            const bool support=mode!=1&&mode!=2&&!(mode==3&&track.sampleTimeS>=.08)&&
                !(mode==5&&track.sampleTimeS>=.06)&&!(mode==6&&track.sampleTimeS>=.06);
            weapon::V3 expected;
            if(support) {
                const auto prior=mode==4?weapon::V3{-250,160,0}:targetVelocity;
                expected=add(prior,scale(track.direction,*track.rangeRateMs+dot(ownVelocity,track.direction)-dot(prior,track.direction)));
                // Doppler cancels from transverse rate; its independent
                // expectation uses the admitted prior, not the output coast.
                const auto relativeVelocity=subtract(prior,ownVelocity);
                const auto angular=scale(subtract(relativeVelocity,scale(track.direction,dot(relativeVelocity,track.direction))),1 / *track.rangeM);
                CHECK(norm(subtract(track.directionRate,angular))<1e-12);
                ++supportedSamples;
                if(mode==4)CHECK(norm(subtract(result.midcourseEstimate->velocity,targetVelocity))>90);
            } else {
                expected=add(ownVelocity,add(scale(track.direction,*track.rangeRateMs),scale(track.directionRate,*track.rangeM)));
                ++fallbackSamples;
            }
            CHECK(norm(subtract(result.midcourseEstimate->velocity,expected))<1e-9);
            CHECK(result.midcourseEstimate->sampleTimeS==track.sampleTimeS);
            REQUIRE(result.deliveredTimeS);CHECK(*result.deliveredTimeS>track.sampleTimeS);
            last=result.midcourseEstimate;
        }
        REQUIRE(last);
        if(mode==1||mode==2)CHECK(fallbackSamples>0);else CHECK(supportedSamples>0);
        if(mode==3||mode==5||mode==6)CHECK(fallbackSamples>0);
        SeekerResultV2 coast;
        for(unsigned tick=16;tick<=50;++tick)coast=observe(tick*.01,false);
        REQUIRE(coast.valid);CHECK_FALSE(coast.tracking);CHECK_FALSE(coast.attributionIndex);CHECK_FALSE(coast.track);
        REQUIRE(coast.midcourseEstimate);
        CHECK(coast.midcourseEstimate->sampleTimeS==last->sampleTimeS);
        CHECK(norm(subtract(coast.midcourseEstimate->position,last->position))<1e-12);
        CHECK(norm(subtract(coast.midcourseEstimate->velocity,last->velocity))<1e-12);
        receiver.sensor->resetTracking();
        const auto reset=observe(.6,false);(void)reset;
        const auto delivered=observe(.61,false);REQUIRE(delivered.valid);
        // The ordinary delivered launcher navigation may remain available after
        // reset; no previous onboard RF capture may survive that reset.
        if(delivered.midcourseEstimate)CHECK(delivered.midcourseEstimate->sampleTimeS>=.6);
    }
}

TEST_CASE("V3 terminal seeker loss never restarts launch loft after association expiry", "[weapon-terminal-coast]") {
    Receiver receiver;auto spec=receiver.spec;REQUIRE(spec.inertialMidcourse);REQUIRE(spec.loftAccelerationG>0);
    for(const bool finite:{true,false}) {
        auto selected=spec;if(!finite)selected.seekerModel.engineering.version=0;
        MeasuredPnGuidanceV2 guidance(selected);auto frame=scene();
        SeekerResultV2 measured;measured.valid=true;measured.active=true;measured.tracking=true;
        SeekerTrackV2 track;track.localId=1;track.direction={1,0,0};track.sampleTimeS=0;
        measured.track=track;
        CHECK(guidance.updateMeasured(frame,measured).phase==WeaponPhase::Homing);
        frame.timeS=frame.ageS=1;measured.tracking=false;measured.track.reset();
        measured.midcourseEstimate=LauncherEstimateV2{{10000,0,0},{},0};
        const auto coast=guidance.updateMeasured(frame,measured);
        if(finite) {
            CHECK(coast.phase==WeaponPhase::Lost);
            CHECK(coast.acceleration.z==Catch::Approx(9.80665).margin(1e-12));
        } else {
            CHECK(coast.phase==WeaponPhase::Midcourse);
            CHECK(coast.acceleration.z>9.80665);
        }
    }
}

TEST_CASE("Rejected V3 terminal packets cannot latch a terminal guidance phase", "[weapon-terminal-coast]") {
    Receiver receiver;
    for(unsigned defect=0;defect<4;++defect) {
        MeasuredPnGuidanceV2 guidance(receiver.spec);auto frame=scene();
        SeekerResultV2 invalid;invalid.valid=true;invalid.active=true;invalid.tracking=true;
        SeekerTrackV2 track;track.localId=1;track.direction={1,0,0};
        if(defect==1)track.direction.x=std::numeric_limits<double>::quiet_NaN();
        if(defect==2){track.sampleTimeS=1;invalid.measurementTimeS=1;}
        if(defect==3)track.rangeM=std::numeric_limits<double>::quiet_NaN();
        if(defect)invalid.track=track;
        guidance.updateMeasured(frame,invalid);
        frame.timeS=frame.ageS=.2;
        SeekerResultV2 midcourse;midcourse.valid=true;midcourse.midcourseEstimate=LauncherEstimateV2{{10000,0,0},{},0};
        const auto actual=guidance.updateMeasured(frame,midcourse);
        INFO(defect);CHECK(actual.phase==WeaponPhase::Midcourse);CHECK(actual.acceleration.z>9.80665);
    }
}

TEST_CASE("Delayed midcourse estimates extrapolate their original state epoch exactly once", "[weapon-sensor-epoch-red]") {
    weapon::Spec spec;spec.inertialMidcourse=true;spec.seeker="active_radar";
    MeasuredPnGuidanceV2 delayed(spec),current(spec);auto f=scene(1);
    SeekerResultV2 old;old.valid=true;old.midcourseEstimate=LauncherEstimateV2{{1000,1000,0},{0,100,0},0};
    auto now=old;now.midcourseEstimate=LauncherEstimateV2{{1000,1100,0},{0,100,0},1};
    const auto a=delayed.updateMeasured(f,old),b=current.updateMeasured(f,now);
    CHECK(a.acceleration.y==Catch::Approx(b.acceleration.y).margin(1e-12));
    CHECK(a.timeToGoS==Catch::Approx(b.timeToGoS).margin(1e-12));
}

TEST_CASE("An empty scene can produce unattributed receiver noise contacts", "[weapon-sensor-background-red]") {
    Receiver admitted;auto spec=admitted.spec;spec.seekerModel.engineering.detectionSnr=2;
    EngineeringSeekerV3 receiver(spec,std::make_unique<ActiveRadarSeekerV2>(spec,std::nullopt));
    unsigned contacts=0;
    for(unsigned i=0;i<1000;++i) {
        auto f=scene(i*.02);f.samples.clear();const auto r=receiver.sample(f);
        if(r.tracking) {++contacts;CHECK_FALSE(r.attributionIndex);}
    }
    CHECK(contacts>50);CHECK(contacts<220);
}

TEST_CASE("On rail finite search advances with elapsed sensor time before launch age exists", "[weapon-sensor-onrail-scan-red]") {
    Receiver r("aim-9x");bool acquired=false;
    for(unsigned i=0;i<400;++i) {
        auto f=scene(100+i*.01,.3);f.ageS=0;
        acquired=acquired||r.sensor->sample(f).tracking;
    }
    CHECK(acquired);
}

TEST_CASE("Quantized receiver delivery reports actual first availability without renewing measurement time", "[weapon-sensor-cadence-red]") {
    Receiver r;auto f=scene();r.sensor->sample(f);
    f.timeS=f.ageS=4./120;const auto first=r.sensor->sample(f);
    REQUIRE(first.tracking);REQUIRE(first.deliveredTimeS);REQUIRE(first.measurementTimeS);
    CHECK(*first.deliveredTimeS==f.timeS);CHECK(*first.measurementTimeS==0);
    const auto repeat=r.sensor->sample(f);CHECK(repeat.deliveredTimeS==first.deliveredTimeS);
}

TEST_CASE("Finite raster reacquires a late body-center contact within its geometric revisit budget", "[weapon-sensor-engineering]") {
    for(const double appearance:{.4,2.3,8.7,17.9}) {
        Receiver r("aim-9x");const auto& p=r.spec.seekerModel.engineering;
        const double bound=4*r.spec.gimbalRad/std::min(p.searchRateRadS,p.gimbalRateRadS)+
            r.spec.gimbalRad/p.gimbalRateRadS+5*p.gimbalLagS+p.samplePeriodS+p.processingDelayS+.02;
        bool acquired=false;
        for(unsigned i=0;i<unsigned(std::ceil((appearance+bound)/.01));++i) {
            auto f=scene(i*.01);f.ageS=0;if(f.timeS<appearance)f.samples.clear();
            const auto result=r.sensor->sample(f);if(f.timeS>=appearance)acquired=acquired||result.tracking;
        }
        INFO(appearance);CHECK(acquired);
    }
}

TEST_CASE("Seeded detector likelihood reuses native RF threshold and Gaussian IR theory", "[weapon-sensor-engineering]") {
    CHECK(EngineeringSeekerV3::detectionProbability(0,10,false)==Catch::Approx(std::exp(-10)).epsilon(1e-8));
    CHECK(EngineeringSeekerV3::detectionProbability(10,10,true)==Catch::Approx(.5));
    CHECK(EngineeringSeekerV3::detectionProbability(9,10,true)==Catch::Approx(.158655253931457));
    CHECK(EngineeringSeekerV3::detectionProbability(100,10,false)>.999);
    CHECK(EngineeringSeekerV3::detectionProbability(1,10,false)<EngineeringSeekerV3::detectionProbability(10,10,false));
}

TEST_CASE("Finite sensing delays retain actual measurement epoch and repeated reads are non consuming", "[weapon-sensor-engineering]") {
    Receiver r;auto f=scene();auto a=r.sensor->sample(f);CHECK_FALSE(a.tracking);
    f.timeS=f.ageS=.005;CHECK_FALSE(r.sensor->sample(f).tracking);
    f.timeS=f.ageS=.01;a=r.sensor->sample(f);REQUIRE(a.tracking);REQUIRE(a.track);
    REQUIRE(a.measurementTimeS);CHECK(*a.measurementTimeS==0);CHECK(a.track->sampleTimeS==0);
    REQUIRE(a.attributionIndex);CHECK(*a.attributionIndex==0);CHECK(a.covarianceAvailable);
    const auto repeat=r.sensor->sample(f);CHECK(repeat.track->direction.y==a.track->direction.y);
    f.timeS=f.ageS=.019;auto held=r.sensor->sample(f);CHECK(held.track->sampleTimeS==0);
    f.timeS=-1;CHECK_FALSE(r.sensor->sample(f).valid);
    f.timeS=f.ageS=.02;CHECK(r.sensor->sample(f).track->sampleTimeS==0);
    f.timeS=f.ageS=.03;CHECK(r.sensor->sample(f).track->sampleTimeS==.02);
}

TEST_CASE("Finite receiver memory keeps measured track identity across a missed exposure without inventing detection", "[weapon-measurement-memory]") {
    Receiver receiver;
    const auto sample=[&](double now,bool visible) {
        auto frame=scene(now);
        frame.supported=true;frame.launcherEstimate=LauncherEstimateV2{{1000,0,0},{},0};
        if(!visible)frame.samples.clear();
        return receiver.sensor->sample(frame);
    };
    sample(0,true);
    auto first=sample(.01,true);REQUIRE(first.track);REQUIRE(first.tracking);
    sample(.02,true);
    auto established=sample(.03,true);REQUIRE(established.track);
    const auto identity=established.track->localId;
    sample(.04,false);
    const auto missed=sample(.05,false);REQUIRE(missed.valid);
    CHECK_FALSE(missed.tracking);CHECK_FALSE(missed.attributionIndex);
    REQUIRE(missed.track);CHECK(missed.track->sampleTimeS==established.track->sampleTimeS);
    sample(.06,true);
    const auto reacquired=sample(.07,true);REQUIRE(reacquired.tracking);REQUIRE(reacquired.track);
    CHECK(reacquired.track->localId==identity);
    CHECK(reacquired.track->rateAvailable);
    CHECK(reacquired.track->sampleTimeS==.06);
    CHECK(reacquired.measurementTimeS==.06);
    sample(.08,false);sample(.09,false);
    sample(.3,false);const auto expired=sample(.31,false);
    CHECK_FALSE(expired.tracking);CHECK_FALSE(expired.attributionIndex);
    sample(.32,true);const auto fresh=sample(.33,true);
    REQUIRE(fresh.tracking);REQUIRE(fresh.track);
    CHECK(fresh.track->localId!=identity);
    CHECK_FALSE(fresh.track->rateAvailable);
}

TEST_CASE("Finite pointing responds to actual body rotation and mechanical field limits", "[weapon-sensor-engineering]") {
    Receiver r;auto f=scene();r.sensor->sample(f);f.timeS=f.ageS=.01;r.sensor->sample(f);
    f.timeS=f.ageS=.02;const double angle=.2;
    f.boresight={std::cos(angle),std::sin(angle),0};
    f.bodyAxes=std::array<weapon::V3,3>{f.boresight,weapon::V3{-std::sin(angle),std::cos(angle),0},weapon::V3{0,0,1}};
    auto result=r.sensor->sample(f);
    const double moved=std::atan2(result.pointingDirection.y,result.pointingDirection.x);
    CHECK(moved>angle-r.spec.seekerModel.engineering.gimbalRateRadS*.01-1e-9);
    CHECK(moved<angle);
    CHECK(dot(result.pointingDirection,unit(f.boresight))>=std::cos(r.spec.gimbalRad)-1e-12);
}

TEST_CASE("Engineering receiver reset repeats its own noise stream and other receivers stay independent", "[weapon-sensor-engineering]") {
    Receiver a,b;auto f=scene();a.sensor->sample(f);b.sensor->sample(f);
    f.timeS=f.ageS=.01;auto x=a.sensor->sample(f),y=b.sensor->sample(f);REQUIRE(x.track);REQUIRE(y.track);
    CHECK(x.track->direction.y==y.track->direction.y);
    a.sensor->resetTracking();f=scene();a.sensor->sample(f);f.timeS=f.ageS=.01;auto reset=a.sensor->sample(f);
    REQUIRE(reset.track);CHECK(reset.track->direction.y==x.track->direction.y);
    Receiver c;f=scene();f.receiverSeed=20;c.sensor->sample(f);f.timeS=f.ageS=.01;auto independent=c.sensor->sample(f);
    REQUIRE(independent.track);CHECK(independent.track->direction.y!=x.track->direction.y);
}

TEST_CASE("Every current AAM retains its delayed scalar engineering method", "[weapon-sensor-engineering][fleet][scalar-compatibility]") {
    for(const auto* id:{"aim-120c","aim-120d","aim-7m","aim-9m","aim-9x","asraam","iris-t","magic-2","meteor",
        "mica-em","mica-ir","pl-10","pl-12","pl-15","pl-8","r-27er","r-27et","r-27r","r-27t","r-37m","r-60m","r-73","r-77m","super-530d"}) {
        INFO(id);Receiver r(id);auto f=scene();
        f.illuminated=true;f.configuredIlluminator=7;
        f.samples[0].illuminator=7;f.samples[0].transmitterPathClear=f.samples[0].receiverPathClear=true;
        auto& p=*f.samples[0].scene;p.transmitterRangeM=2000;p.transmitterPowerW=1000;p.transmitterGainLinear=1000;p.transmitterFrequencyHz=10e9;
        CHECK_FALSE(r.sensor->sample(f).tracking);f.timeS=f.ageS=.01;
        const auto observed=r.sensor->sample(f);REQUIRE(observed.tracking);REQUIRE(observed.track);
        CHECK(observed.measurementTimeS==0);CHECK(observed.deliveredTimeS==.01);
        if(r.spec.seeker!="active_radar") {CHECK_FALSE(observed.track->rangeM);CHECK_FALSE(observed.track->rangeRateMs);}
        CHECK(observed.angularVarianceRad2.value_or(0)>0);
    }
}


TEST_CASE("Engineering seed admission is atomic and unavailable angular rates are canonical", "[weapon-sensor-seed-red]") {
    Receiver admitted;const auto spec=admitted.spec;
    const auto make=[&]{return std::make_unique<EngineeringSeekerV3>(spec,std::make_unique<ActiveRadarSeekerV2>(spec,std::nullopt));};
    SeekerTrackV2 valid;valid.localId=9;valid.sampleTimeS=0;valid.direction={1,0,0};
    const auto observe=[&](EngineeringSeekerV3& sensor) {
        auto f=scene(.02);sensor.sample(f);f.timeS=f.ageS=.03;return sensor.sample(f);
    };
    auto reference=make();reference->seedTrack(valid);const auto expected=observe(*reference);
    REQUIRE(expected.tracking);REQUIRE(expected.track);
    for(unsigned defect=0;defect<6;++defect) {
        auto sensor=make();sensor->seedTrack(valid);auto malformed=valid;
        if(defect==0){malformed.rateAvailable=true;malformed.directionRate.x=std::numeric_limits<double>::quiet_NaN();}
        if(defect==1)malformed.localId=0;
        if(defect==2)malformed.rangeM=std::numeric_limits<double>::infinity();
        if(defect==3)malformed.rangeRateMs=std::numeric_limits<double>::quiet_NaN();
        if(defect==4)malformed.imageExtent=ImageExtentV2{0,.01};
        if(defect==5)malformed.direction={std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),0};
        sensor->seedTrack(malformed);const auto actual=observe(*sensor);INFO(defect);
        REQUIRE(actual.track);CHECK(actual.tracking);CHECK(actual.track->localId==expected.track->localId);
        CHECK(actual.track->direction.x==expected.track->direction.x);CHECK(actual.track->direction.y==expected.track->direction.y);
        CHECK(actual.track->directionRate.x==expected.track->directionRate.x);CHECK(actual.track->directionRate.y==expected.track->directionRate.y);
    }
    auto unavailable=make();auto noRate=valid;noRate.rateAvailable=false;
    noRate.directionRate={std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity(),0};
    unavailable->seedTrack(noRate);const auto actual=observe(*unavailable);REQUIRE(actual.track);
    CHECK(actual.track->localId==expected.track->localId);CHECK(actual.track->direction.y==expected.track->direction.y);
    CHECK(actual.track->directionRate.x==expected.track->directionRate.x);CHECK(actual.track->directionRate.y==expected.track->directionRate.y);
}
