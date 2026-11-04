#include "simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#include "simulation/mission_systems/components/implementations/seeker/ImagingIrSeekerV2.h"
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <numeric>
#include <algorithm>
#if __has_include("simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h")
#include "simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h"
#define FSIM_TEST_DUAL_THERMAL 1
#endif

namespace {
using namespace fsim::ms::components;
ThermalImageConfigV1 policy() {
    ThermalImageConfigV1 c;
    c.version=1;c.width=32;c.height=32;c.adcBits=16;c.exposureSamples=4;
    c.pixelPitchRad=.002;c.psfSigmaRad=.002;c.apertureAreaM2=2e-7;
    c.opticalTransmission=.7;c.quantumEfficiency=.6;c.effectiveWavelengthM=4e-6;
    c.integrationTimeS=.01;c.pixelResponseTimeS=.001;c.readNoiseElectrons=2;
    c.darkCurrentElectronsS=1;c.fullWellElectrons=1e7;c.detectionSigma=5;
    c.backgroundRadiusPixels=5;c.minimumFeaturePixels=1;c.associationGateRad=.03;
    return c;
}
ThermalSceneFrameV1 scene(double time=0,double contrast=2) {
    ThermalSceneFrameV1 f;f.timeS=time;f.skyRadianceWm2Sr=1;f.groundRadianceWm2Sr=1;f.up={0,0,1};
    ThermalRadiancePatchV1 p;p.direction={1,0,0};p.widthRad=.004;p.heightRad=.004;p.rangeM=1000;
    p.radianceWm2Sr=1+contrast;p.transmission=1;p.pathRadianceWm2Sr=1;
    f.patches.push_back(p);f.refereeAttributionIndices.push_back(9);return f;
}
SeekerNavigationPoseV1 pose(double time,double yaw=0) {
    SeekerNavigationPoseV1 p;p.valid=true;p.timeS=time;p.velocity={250,0,0};
    p.bodyToEcef={{{std::cos(yaw),std::sin(yaw),0},{-std::sin(yaw),std::cos(yaw),0},{0,0,1}}};return p;
}
}

TEST_CASE("Captured optical pointing reproduces exact detector charge and random history", "[weapon-thermal-capture-replay]") {
    ThermalImageReceiverV1 physical;std::string error;REQUIRE(physical.configure(policy(),error));
    for(unsigned i=0;i<32;++i) {
        auto f=scene(i*.02);
        const fsim::ms::weapon::V3 pointing{1.013+.0001*i,.0123+.00003*i,.00567};
        auto before=physical;
        const auto actual=physical.expose(f,pointing,123,true);REQUIRE(actual.valid);
        const auto replay=before.expose(f,actual.pointing,123,true);REQUIRE(replay.valid);
        CHECK(actual.expectedElectrons==replay.expectedElectrons);
        CHECK(actual.measuredElectrons==replay.measuredElectrons);
    }
}

TEST_CASE("Thermal image uses radiance solid angle and physical photon energy", "[weapon-thermal-image]") {
    ThermalImageReceiverV1 sensor;auto c=policy();std::string error;REQUIRE(sensor.configure(c,error));
    auto f=scene();f.patches.clear();f.refereeAttributionIndices.clear();
    auto r=sensor.expose(f,{1,0,0},123,true);REQUIRE(r.valid);REQUIRE(r.expectedElectrons.size()==1024);
    // Exact rectangular-pyramid solid angle at the central pixel, independent
    // of the production renderer. The first finite pixel response is included.
    const double x0=-c.pixelPitchRad,x1=0,y0=-c.pixelPitchRad,y1=0;
    const auto primitive=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    const double omega=primitive(std::tan(x1),std::tan(y1))-primitive(std::tan(x0),std::tan(y1))-
        primitive(std::tan(x1),std::tan(y0))+primitive(std::tan(x0),std::tan(y0));
    const double expected=omega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*c.quantumEfficiency*
        c.effectiveWavelengthM/(6.62607015e-34*299792458.)*(-std::expm1(-c.integrationTimeS/c.pixelResponseTimeS))+
        c.darkCurrentElectronsS*c.integrationTimeS;
    CHECK(r.expectedElectrons[15*32+15]==Catch::Approx(expected).epsilon(1e-10));
    CHECK(ThermalImageReceiverV1::bandRadiance(900,1,3,5)==Catch::Approx(4250.708940699046).epsilon(1e-7));
}

TEST_CASE("Image measurements require contrast and never use referee identity", "[weapon-thermal-image]") {
    auto c=policy();ThermalImageReceiverV1 a,b;std::string error;REQUIRE(a.configure(c,error));REQUIRE(b.configure(c,error));
    auto f=scene();auto other=f;REQUIRE(other.refereeAttributionIndices.size()==1);other.refereeAttributionIndices.at(0)=9999;
    auto x=a.expose(f,{1,0,0},55,true),y=b.expose(other,{1,0,0},55,true);
    REQUIRE(x.valid);REQUIRE(y.valid);CHECK(x.measuredElectrons==y.measuredElectrons);
    REQUIRE_FALSE(x.features.empty());REQUIRE(x.features.size()==y.features.size());
    CHECK(x.features[0].direction.y==y.features[0].direction.y);
    CHECK(x.features[0].extent.widthRad==y.features[0].extent.widthRad);
    CHECK(x.features[0].refereeAttributionIndex==9);CHECK(y.features[0].refereeAttributionIndex==9999);
    a.reset();auto equal=scene(0,0);auto blank=a.expose(equal,{1,0,0},55,true);
    // A high threshold suppresses this frozen noise-only negative without
    // disabling physical photon/read noise or relying on truth classification.
    c.detectionSigma=20;REQUIRE(a.configure(c,error));blank=a.expose(equal,{1,0,0},55,true);
    CHECK(blank.features.empty());
}

TEST_CASE("Signed quadrant error follows detector flux and supports cold contrast", "[weapon-thermal-image]") {
    auto c=policy();ThermalImageReceiverV1 sensor;std::string error;REQUIRE(sensor.configure(c,error));
    auto f=scene();f.patches[0].direction={1,.001,0};auto r=sensor.expose(f,{1,0,0},14,false);
    REQUIRE(r.valid);REQUIRE(r.features.size()==1);CHECK(r.features[0].direction.y>0);
    sensor.reset();f.patches[0].radianceWm2Sr=0;auto cold=sensor.expose(f,{1,0,0},14,false);
    REQUIRE(cold.features.size()==1);CHECK(cold.features[0].signedContrastElectrons<0);
    CHECK(cold.features[0].direction.y>0);
}

TEST_CASE("Unresolved quadrant magnitude does not claim a precise bearing variance", "[weapon-quadrant-uncertainty]") {
    const auto c=policy();ThermalImageReceiverV1 sensor;std::string error;REQUIRE(sensor.configure(c,error));
    for(unsigned seed=0;seed<32;++seed) {
        sensor.reset();auto f=scene(0,20);const double angle=seed%2?.02:-.02;
        f.patches[0].direction={std::cos(angle),std::sin(angle),0};
        const auto r=sensor.expose(f,{1,0,0},seed,false);REQUIRE(r.valid);REQUIRE(r.features.size()==1);
        const auto& feature=r.features.front();const auto& source=f.patches.front().direction;
        const double errorRad=std::acos(std::clamp(feature.direction.x*source.x+feature.direction.y*source.y+feature.direction.z*source.z,-1.,1.));
        INFO("seed="<<seed<<" error="<<errorRad<<" admitted variance="<<feature.angularVarianceRad2);
        CHECK(feature.angularVarianceRad2>=errorRad*errorRad);
    }
}

TEST_CASE("Quadrant Gaussian inversion refuses a measured nonuniform background", "[weapon-quadrant-background]") {
    const auto c=policy();ThermalImageReceiverV1 sensor;std::string error;REQUIRE(sensor.configure(c,error));
    auto f=scene();f.patches.clear();f.refereeAttributionIndices.clear();
    f.skyRadianceWm2Sr=1;f.groundRadianceWm2Sr=10;f.groundHorizonCosine=-.02;
    for(unsigned seed=0;seed<8;++seed) {
        sensor.reset();const auto result=sensor.expose(f,{1,0,0},seed,false);REQUIRE(result.valid);
        REQUIRE_FALSE(result.measuredElectrons.empty());CHECK(result.features.empty());
    }
}

TEST_CASE("Quadrant endpoint admission follows measured noise confidence", "[weapon-quadrant-endpoint]") {
    auto c=policy();c.readNoiseElectrons=100;
    ThermalImageReceiverV1 sensor;std::string error;REQUIRE(sensor.configure(c,error));
    unsigned candidates=0,outsideLegacy=0;
    for(unsigned seed=0;seed<64;++seed) {
        sensor.reset();auto f=scene(0,1);const double angle=seed%2?.02:-.02;
        f.patches[0].direction={std::cos(angle),std::sin(angle),0};
        const auto r=sensor.expose(f,{1,0,0},seed,false);REQUIRE(r.valid);
        const auto& q=r.quadrantElectrons;const double total=q[0]+q[1]+q[2]+q[3];
        const double sigma=std::sqrt(r.quadrantVarianceElectrons2),snr=std::abs(total)/sigma;
        if(snr<c.detectionSigma)continue;
        const double differenceX=q[1]+q[3]-q[0]-q[2],differenceY=q[0]+q[1]-q[2]-q[3];
        // A physical difference lies in [-|total|,|total|]. Both measured
        // terms have at most this configured noise bound; neither a fixed
        // 1.05 ratio cutoff nor target identity supplies its confidence.
        if(std::abs(differenceX)>std::abs(total)+2*c.detectionSigma*sigma||
           std::abs(differenceY)>std::abs(total)+2*c.detectionSigma*sigma)continue;
        ++candidates;outsideLegacy+=std::abs(differenceX/total)>1.05||std::abs(differenceY/total)>1.05;
        INFO("seed="<<seed<<" measured SNR="<<snr);CHECK(r.features.size()==1);
    }
    CHECK(candidates>16);CHECK(outsideLegacy>0);
}

TEST_CASE("Thermal response saturation reset and time admission are physical", "[weapon-thermal-image]") {
    auto c=policy();ThermalImageReceiverV1 sensor;std::string error;REQUIRE(sensor.configure(c,error));
    auto f=scene();auto first=sensor.expose(f,{1,0,0},19,true);auto repeat=sensor.expose(f,{1,0,0},900,true);
    CHECK(first.measuredElectrons==repeat.measuredElectrons);sensor.reset();
    CHECK(sensor.expose(f,{1,0,0},19,true).measuredElectrons==first.measuredElectrons);
    f.timeS=-1;CHECK_FALSE(sensor.expose(f,{1,0,0},19,true).valid);
    c.fullWellElectrons=100;REQUIRE(sensor.configure(c,error));f=scene();
    auto saturated=sensor.expose(f,{1,0,0},19,true);REQUIRE(saturated.valid);
    for(double value:saturated.measuredElectrons)CHECK((value>=0&&value<=100));
    auto invalid=c;invalid.quantumEfficiency=0;CHECK_FALSE(sensor.configure(invalid,error));
    invalid=c;invalid.width=129;CHECK_FALSE(sensor.configure(invalid,error));
}

TEST_CASE("Optical spreading preserves energy and motion increases measured image width", "[weapon-thermal-image]") {
    auto c=policy();ThermalImageReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto f=scene();auto stationary=receiver.expose(f,{1,0,0},7,true);
    receiver.reset();f.patches[0].directionRate={0,2,0};auto moving=receiver.expose(f,{1,0,0},7,true);
    REQUIRE(stationary.valid);REQUIRE(moving.valid);
    receiver.reset();f=scene();f.patches.clear();f.refereeAttributionIndices.clear();
    auto background=receiver.expose(f,{1,0,0},7,true);
    const auto moments=[&](const ThermalImageResultV1& image) {
        double total=0,first=0,second=0;
        for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
            // Correct the known slowly varying pixel solid angle using a
            // separate blank exposure below, rather than a fitted baseline.
            const double weight=std::max(0.,image.expectedElectrons[y*c.width+x]-background.expectedElectrons[y*c.width+x]);
            const double angle=(static_cast<double>(x)+.5-.5*c.width)*c.pixelPitchRad;
            total+=weight;first+=weight*angle;second+=weight*angle*angle;
        }
        return std::array<double,2>{total,second/total-(first/total)*(first/total)};
    };
    const auto a=moments(stationary),b=moments(moving);
    CHECK(b[0]==Catch::Approx(a[0]).epsilon(.01));
    CHECK(b[1]>a[1]+1e-5);
    double signal=0;for(std::size_t i=0;i<background.expectedElectrons.size();++i)
        signal+=stationary.expectedElectrons[i]-background.expectedElectrons[i];
    const double geometricOmega=.004*.004;
    const double oracle=2*geometricOmega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*
        c.quantumEfficiency*c.effectiveWavelengthM/(6.62607015e-34*299792458.)*
        (-std::expm1(-c.integrationTimeS/c.pixelResponseTimeS));
    CHECK(signal==Catch::Approx(oracle).epsilon(.001));
}

TEST_CASE("Image detection resolves sources from actual pixel components", "[weapon-thermal-image]") {
    auto c=policy();ThermalImageReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto f=scene();f.patches[0].direction={std::cos(.012),std::sin(.012),0};
    auto second=f.patches[0];second.direction={std::cos(.012),-std::sin(.012),0};
    f.patches.push_back(second);f.refereeAttributionIndices.push_back(10);
    auto resolved=receiver.expose(f,{1,0,0},81,true);REQUIRE(resolved.valid);
    REQUIRE(resolved.features.size()==2);
    REQUIRE(resolved.segmentationAvailable);REQUIRE(resolved.segmentation.size()==c.width*c.height);
    CHECK(std::count(resolved.segmentation.begin(),resolved.segmentation.end(),1)>0);
    CHECK(std::count(resolved.segmentation.begin(),resolved.segmentation.end(),2)>0);
    CHECK(*std::max_element(resolved.segmentation.begin(),resolved.segmentation.end())==2);
    CHECK(resolved.features[0].direction.y*resolved.features[1].direction.y<0);
    receiver.reset();f.patches[0].direction={std::cos(.0001),std::sin(.0001),0};
    f.patches[1].direction={std::cos(.0001),-std::sin(.0001),0};
    auto unresolved=receiver.expose(f,{1,0,0},81,true);REQUIRE(unresolved.valid);
    CHECK(unresolved.features.size()==1);
    receiver.reset();f.patches[0].direction=f.patches[1].direction={1,0,0};
    f.patches[0].rangeM=1000;f.patches[1].rangeM=500;f.patches[1].radianceWm2Sr=1;
    c.detectionSigma=20;REQUIRE(receiver.configure(c,error));
    CHECK(receiver.expose(f,{1,0,0},81,true).features.empty());
}

TEST_CASE("Photon/read statistics and rejected scene retry preserve detector state", "[weapon-thermal-image]") {
    auto c=policy();c.width=8;c.height=8;c.detectionSigma=30;
    ThermalImageReceiverV1 a,b;std::string error;REQUIRE(a.configure(c,error));REQUIRE(b.configure(c,error));
    auto f=scene();f.patches.clear();f.refereeAttributionIndices.clear();
    auto first=a.expose(f,{1,0,0},100,true);REQUIRE(first.valid);
    b.expose(f,{1,0,0},100,true);auto invalid=f;invalid.timeS=.02;
    invalid.skyRadianceWm2Sr=std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(a.expose(invalid,{1,0,0},100,true).valid);
    f.timeS=.02;CHECK(a.expose(f,{1,0,0},100,true).measuredElectrons==b.expose(f,{1,0,0},100,true).measuredElectrons);
    double sum=0,squares=0,expected=0;constexpr unsigned samples=512;
    for(unsigned i=0;i<samples;++i) {
        f.timeS=.04+.02*i;auto result=a.expose(f,{1,0,0},100,true);REQUIRE(result.valid);
        const double value=result.measuredElectrons[27];sum+=value;squares+=value*value;expected+=result.expectedElectrons[27];
    }
    const double mean=sum/samples,reference=expected/samples;
    const double adcStep=c.fullWellElectrons/(std::ldexp(1.,static_cast<int>(c.adcBits))-1);
    const double variance=reference+c.readNoiseElectrons*c.readNoiseElectrons+adcStep*adcStep/12;
    CHECK(std::abs(mean-reference)<5*std::sqrt(variance/samples));
    CHECK((squares-sum*sum/samples)/(samples-1)==Catch::Approx(variance).epsilon(.25));
}

TEST_CASE("Explicit image execution traverses finite capture and delivery without scalar bearing injection", "[weapon-thermal-image]") {
    using namespace fsim::ms;
    weapon::Spec spec;spec.seeker="imaging_infrared";spec.executionVersion=2;
    spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::ImagingInfrared;
    spec.seekerModel.engineering.version=3;
    EngineeringSeekerV3 sensor(spec,std::make_unique<ImagingIrSeekerV2>(spec));
    auto c=policy();std::string error;REQUIRE(sensor.configureThermalV1(c,error));
    SeekerFrameV2 frame;frame.timeS=0;frame.ageS=0;frame.dtS=.01;
    frame.position={0,0,0};frame.velocity={250,0,0};frame.up={0,0,1};frame.boresight={1,0,0};frame.receiverSeed=73;
    frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
    // A wholly different scalar bearing cannot supply or change image error.
    SeekerSampleV2 fake;fake.direction={0,1,0};fake.signal=1e100;frame.samples.push_back(fake);
    auto thermal=scene();thermal.navigation=pose(0);CHECK_FALSE(sensor.sampleThermalV1(frame,thermal).tracking);
    frame.timeS=frame.ageS=.01;thermal.timeS=.01;thermal.navigation=pose(.01);const auto delivered=sensor.sampleThermalV1(frame,thermal);
    REQUIRE(delivered.valid);REQUIRE(delivered.tracking);REQUIRE(delivered.track);
    CHECK(delivered.measurementTimeS==0);CHECK(delivered.deliveredTimeS==.01);
    CHECK(std::abs(delivered.track->direction.y)<.003);
    CHECK_FALSE(delivered.track->rangeM);CHECK(delivered.covarianceAvailable);
    thermal.timeS=.02;CHECK_FALSE(sensor.sampleThermalV1(frame,thermal).valid);
    sensor.resetTracking();frame.timeS=frame.ageS=0;thermal.timeS=0;thermal.navigation=pose(0);
    sensor.sampleThermalV1(frame,thermal);frame.timeS=frame.ageS=.01;thermal.timeS=.01;thermal.navigation=pose(.01);
    const auto replay=sensor.sampleThermalV1(frame,thermal);REQUIRE(replay.track);
    CHECK(replay.track->direction.x==delivered.track->direction.x);
    CHECK(replay.track->direction.y==delivered.track->direction.y);
    CHECK(replay.track->direction.z==delivered.track->direction.z);
}

TEST_CASE("Navigation orientation changes homing reconstruction and cue without changing raw first image", "[weapon-thermal-image]") {
    using namespace fsim::ms;
    weapon::Spec spec;spec.seeker="imaging_infrared";spec.executionVersion=2;spec.launcherUpdates=true;
    spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::ImagingInfrared;spec.seekerModel.engineering.version=3;
    EngineeringSeekerV3 aligned(spec,std::make_unique<ImagingIrSeekerV2>(spec));
    EngineeringSeekerV3 rotated(spec,std::make_unique<ImagingIrSeekerV2>(spec));
    std::string error;REQUIRE(aligned.configureThermalV1(policy(),error));REQUIRE(rotated.configureThermalV1(policy(),error));
    SeekerFrameV2 frame;frame.dtS=.01;frame.up={0,0,1};frame.boresight={1,0,0};frame.velocity={250,0,0};frame.receiverSeed=72;
    frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
    frame.supported=true;frame.launcherEstimate=LauncherEstimateV2{{10000,0,0},{0,0,0},0};
    auto a=scene(),b=scene();a.navigation=pose(0);b.navigation=pose(0,.2);
    aligned.sampleThermalV1(frame,a);rotated.sampleThermalV1(frame,b);
    const auto* imageA=aligned.thermalImageV1();const auto* imageB=rotated.thermalImageV1();
    REQUIRE(imageA);REQUIRE(imageB);if(!imageA||!imageB)return;
    CHECK(imageA->measuredElectrons==imageB->measuredElectrons);
    frame.timeS=frame.ageS=.01;a.timeS=b.timeS=.01;a.navigation=pose(.01);b.navigation=pose(.01,.2);
    const auto ra=aligned.sampleThermalV1(frame,a),rb=rotated.sampleThermalV1(frame,b);
    REQUIRE(ra.track);REQUIRE(rb.track);
    CHECK(std::atan2(rb.track->direction.y,rb.track->direction.x)-std::atan2(ra.track->direction.y,ra.track->direction.x)==Catch::Approx(.2).margin(1e-10));
    CHECK(rb.pointingDirection.y<ra.pointingDirection.y-.01);
    b.navigation.reset();CHECK_FALSE(rotated.sampleThermalV1(frame,b).valid);
    b.navigation=pose(0,.2);CHECK_FALSE(rotated.sampleThermalV1(frame,b).valid);
}

TEST_CASE("Bounded image population retains a late bright component and canonical moving patches", "[weapon-thermal-image]") {
    auto c=policy();c.width=128;c.height=128;c.psfSigmaRad=.00025;c.detectionSigma=20;
    ThermalImageReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto f=scene();f.patches.clear();f.refereeAttributionIndices.clear();
    for(unsigned i=0;i<40;++i) {
        auto p=scene().patches[0];
        const double x=(16.5+12*(i%8)-64)*c.pixelPitchRad,y=(20.5+16*(i/8)-64)*c.pixelPitchRad;
        const double length=std::sqrt(1+std::tan(x)*std::tan(x)+std::tan(y)*std::tan(y));
        p.direction={1/length,-std::tan(x)/length,std::tan(y)/length};
        p.widthRad=p.heightRad=.001;p.radianceWm2Sr=i==39?21:3;
        f.patches.push_back(p);f.refereeAttributionIndices.push_back(i);
    }
    auto result=receiver.expose(f,{1,0,0},99,true);REQUIRE(result.valid);REQUIRE(result.features.size()==32);
    CHECK(std::any_of(result.features.begin(),result.features.end(),[](const auto& feature){return feature.refereeAttributionIndex==39;}));
    c=policy();ThermalImageReceiverV1 other;REQUIRE(receiver.configure(c,error));REQUIRE(other.configure(c,error));
    f=scene();f.patches.push_back(f.patches[0]);f.patches.push_back(f.patches[0]);
    f.refereeAttributionIndices={1,2,3};f.patches[0].directionRate={0,.13,0};
    f.patches[1].directionRate={0,-.21,0};f.patches[2].directionRate={0,.07,0};
    auto reversed=f;std::reverse(reversed.patches.begin(),reversed.patches.end());
    std::reverse(reversed.refereeAttributionIndices.begin(),reversed.refereeAttributionIndices.end());
    auto a=receiver.expose(f,{1,0,0},11,true),b=other.expose(reversed,{1,0,0},11,true);
    CHECK(a.expectedElectrons==b.expectedElectrons);CHECK(a.measuredElectrons==b.measuredElectrons);
}

namespace {
double gaussianRectangleAxis(double sourceLow,double sourceHigh,double pixelLow,double pixelHigh,double sigma) {
    // Independent numerical source-plane integration of Gaussian probability
    // in the detector interval. This is deliberately not the renderer's closed
    // form rectangle primitive or a pixel-centred discrete convolution.
    constexpr unsigned intervals=512;
    const double step=(sourceHigh-sourceLow)/intervals;
    double sum=0;
    for(unsigned i=0;i<=intervals;++i) {
        const double x=sourceLow+i*step;
        const double probability=.5*(std::erf((pixelHigh-x)/(std::sqrt(2.)*sigma))-
            std::erf((pixelLow-x)/(std::sqrt(2.)*sigma)));
        sum+=(i==0||i==intervals?1:i%2?4:2)*probability;
    }
    return sum*step/3;
}
fsim::ms::weapon::V3 thermalRay(double x,double y) {
    const double length=std::sqrt(1+std::tan(x)*std::tan(x)+std::tan(y)*std::tan(y));
    return {1/length,-std::tan(x)/length,std::tan(y)/length};
}
double independentPixelOmega(double x0,double x1,double y0,double y1) {
    const auto primitive=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    return primitive(std::tan(x1),std::tan(y1))-primitive(std::tan(x0),std::tan(y1))-
        primitive(std::tan(x1),std::tan(y0))+primitive(std::tan(x0),std::tan(y0));
}
}

TEST_CASE("Continuous displaced subpixel rectangles match independent Gaussian detector integrals", "[weapon-thermal-image][thermal-completion]") {
    auto c=policy();c.psfSigmaRad=.25*c.pixelPitchRad;
    std::string error;
    for(const double offset:{.22,.74,15.8,16.3}) {
        ThermalImageReceiverV1 receiver;REQUIRE(receiver.configure(c,error));
        auto f=scene();f.skyRadianceWm2Sr=f.groundRadianceWm2Sr=0;
        auto& p=f.patches[0];const double cx=offset*c.pixelPitchRad,cy=.37*c.pixelPitchRad;
        p.direction=thermalRay(cx,cy);p.widthRad=.12*c.pixelPitchRad;p.heightRad=.19*c.pixelPitchRad;p.radianceWm2Sr=2;
        auto image=receiver.expose(f,{1,0,0},321,true);REQUIRE(image.valid);
        double measuredFlux=0,oracleFlux=0;
        for(unsigned y=0;y<c.height;++y)for(unsigned x=0;x<c.width;++x) {
            const double x0=(x-.5*c.width)*c.pixelPitchRad,y0=(y-.5*c.height)*c.pixelPitchRad;
            const double probability=gaussianRectangleAxis(cx-.5*p.widthRad,cx+.5*p.widthRad,x0,x0+c.pixelPitchRad,c.psfSigmaRad)*
                gaussianRectangleAxis(cy-.5*p.heightRad,cy+.5*p.heightRad,y0,y0+c.pixelPitchRad,c.psfSigmaRad);
            const double omega=independentPixelOmega(x0,x0+c.pixelPitchRad,y0,y0+c.pixelPitchRad);
            const double gain=omega*c.apertureAreaM2*c.opticalTransmission*c.integrationTimeS*c.quantumEfficiency*
                c.effectiveWavelengthM/(6.62607015e-34*299792458.)*(-std::expm1(-c.integrationTimeS/c.pixelResponseTimeS));
            const auto i=y*c.width+x;
            const double rendered=(image.expectedElectrons[i]-c.darkCurrentElectronsS*c.integrationTimeS)/gain;
            const double oracle=p.radianceWm2Sr*probability/(c.pixelPitchRad*c.pixelPitchRad);
            CHECK(rendered==Catch::Approx(oracle).margin(2e-8));
            measuredFlux+=rendered*c.pixelPitchRad*c.pixelPitchRad;
            oracleFlux+=p.radianceWm2Sr*probability;
        }
        CHECK(measuredFlux==Catch::Approx(oracleFlux).epsilon(1e-6));
        if(offset<1)CHECK(oracleFlux==Catch::Approx(p.radianceWm2Sr*p.widthRad*p.heightRad).epsilon(1e-6));
        else CHECK(oracleFlux<p.radianceWm2Sr*p.widthRad*p.heightRad);
    }
}

TEST_CASE("Disjoint opaque subpixel patches do not attenuate one another", "[weapon-thermal-image][thermal-completion]") {
    auto c=policy();c.psfSigmaRad=.25*c.pixelPitchRad;std::string error;
    auto f=scene();f.skyRadianceWm2Sr=f.groundRadianceWm2Sr=0;
    auto p=f.patches[0];p.widthRad=.2*c.pixelPitchRad;p.heightRad=.3*c.pixelPitchRad;
    p.direction=thermalRay(.2*c.pixelPitchRad,.4*c.pixelPitchRad);p.radianceWm2Sr=2;p.rangeM=1000;
    auto q=p;q.direction=thermalRay(.7*c.pixelPitchRad,.4*c.pixelPitchRad);q.radianceWm2Sr=3;q.rangeM=500;
    const auto expose=[&](std::vector<ThermalRadiancePatchV1> patches) {
        ThermalImageReceiverV1 receiver;REQUIRE(receiver.configure(c,error));
        auto frame=f;frame.patches=std::move(patches);frame.refereeAttributionIndices.resize(frame.patches.size());
        return receiver.expose(frame,{1,0,0},987,true);
    };
    const auto a=expose({p}),b=expose({q}),together=expose({p,q});REQUIRE(together.valid);
    for(std::size_t i=0;i<a.expectedElectrons.size();++i)
        CHECK(together.expectedElectrons[i]==Catch::Approx(a.expectedElectrons[i]+b.expectedElectrons[i]-c.darkCurrentElectronsS*c.integrationTimeS).margin(1e-7));
}

TEST_CASE("Occluded bright source cannot own image or quadrant detector attribution", "[weapon-thermal-image][thermal-completion]") {
    auto c=policy();std::string error;
    auto f=scene();f.patches[0].radianceWm2Sr=1e5;f.patches[0].rangeM=2000;
    auto near=f.patches[0];near.rangeM=500;near.radianceWm2Sr=4;
    f.patches.push_back(near);f.refereeAttributionIndices={90,12};
    for(const bool imaging:{false,true}) {
        ThermalImageReceiverV1 receiver,reference;REQUIRE(receiver.configure(c,error));REQUIRE(reference.configure(c,error));
        auto alone=f;alone.patches={near};alone.refereeAttributionIndices={12};
        const auto image=receiver.expose(f,{1,0,0},714,imaging),visible=reference.expose(alone,{1,0,0},714,imaging);
        REQUIRE(image.valid);REQUIRE_FALSE(image.features.empty());
        CHECK(image.expectedElectrons==visible.expectedElectrons);
        CHECK(image.measuredElectrons==visible.measuredElectrons);
        for(const auto& feature:image.features)CHECK(feature.refereeAttributionIndex==12);
    }
}

TEST_CASE("Partly overlapping opaque rectangles preserve the exact visible support", "[weapon-thermal-image][thermal-completion]") {
    auto c=policy();std::string error;
    auto f=scene();f.skyRadianceWm2Sr=f.groundRadianceWm2Sr=0;
    auto far=f.patches[0];far.widthRad=.8*c.pixelPitchRad;far.heightRad=.5*c.pixelPitchRad;
    far.direction=thermalRay(.4*c.pixelPitchRad,0);far.radianceWm2Sr=3;far.rangeM=1000;
    auto near=far;near.direction=thermalRay(.8*c.pixelPitchRad,0);near.radianceWm2Sr=5;near.rangeM=500;
    auto clipped=far;clipped.widthRad=.4*c.pixelPitchRad;clipped.direction=thermalRay(.2*c.pixelPitchRad,0);
    const auto expose=[&](std::vector<ThermalRadiancePatchV1> patches) {
        ThermalImageReceiverV1 receiver;REQUIRE(receiver.configure(c,error));auto frame=f;
        frame.patches=std::move(patches);frame.refereeAttributionIndices.resize(frame.patches.size());
        return receiver.expose(frame,{1,0,0},135,true);
    };
    const auto actual=expose({far,near}),visibleFar=expose({clipped}),visibleNear=expose({near});
    REQUIRE(actual.valid);
    for(std::size_t i=0;i<actual.expectedElectrons.size();++i)
        CHECK(actual.expectedElectrons[i]==Catch::Approx(visibleFar.expectedElectrons[i]+visibleNear.expectedElectrons[i]-c.darkCurrentElectronsS*c.integrationTimeS).margin(1e-7));
}

TEST_CASE("Dual waveband receiver executes independent images and measured spectral continuity", "[weapon-thermal-image][thermal-completion]") {
#if FSIM_TEST_DUAL_THERMAL
    ThermalDualBandConfigV1 c;c.version=1;c.channels={policy(),policy()};
    c.channels[1].effectiveWavelengthM=10e-6;c.angularPairGateRad=.004;c.logRatioGate=.35;c.spectralMemoryS=.2;
    ThermalDualBandReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    ThermalDualBandSceneFrameV1 frame;frame.channels={scene(),scene()};
    frame.channels[1].patches[0].radianceWm2Sr=8;
    auto first=receiver.expose(frame,{1,0,0},912);REQUIRE(first.valid);REQUIRE(first.features.size()==1);
    CHECK(first.channels[0].expectedElectrons!=first.channels[1].expectedElectrons);
    REQUIRE(first.features[0].logSpectralRatio);CHECK(std::isfinite(*first.features[0].logSpectralRatio));
    REQUIRE(receiver.commitSelectedMeasurementV1(first.timeS,0));
    frame.channels[0].timeS=frame.channels[1].timeS=.02;
    // Retained target moves slightly. A brighter nearby source has the wrong
    // measured two-channel colour; both carry deliberately useless identities.
    for(unsigned b=0;b<2;++b) {
        auto& channel=frame.channels[b];channel.patches[0].direction=thermalRay(.001,0);
        auto flare=channel.patches[0];flare.direction=thermalRay(.014,0);
        flare.radianceWm2Sr=b?6:30;channel.patches.push_back(flare);channel.refereeAttributionIndices={999,999};
    }
    auto second=receiver.expose(frame,{1,0,0},912);REQUIRE(second.valid);
    std::string spectralDiagnostic="initial measured colour="+std::to_string(*first.features[0].logSpectralRatio);
    for(unsigned band=0;band<2;++band)for(const auto& feature:second.channels[band].features)
        spectralDiagnostic+="; band="+std::to_string(band)+" angle="+std::to_string(std::atan2(-feature.direction.y,feature.direction.x))+
            " flux="+std::to_string(feature.signedContrastElectrons)+" snr="+std::to_string(feature.signalToNoise);
    INFO(spectralDiagnostic);REQUIRE_FALSE(second.features.empty());
    CHECK(std::abs(second.features[0].direction.y)<.004);
    CHECK(second.spectralAssociationAvailable);
    auto changed=frame;changed.channels[0].timeS=changed.channels[1].timeS=.04;
    changed.channels[1].patches.clear();changed.channels[1].refereeAttributionIndices.clear();
    CHECK(receiver.expose(changed,{1,0,0},912).features.empty());
    receiver.reset();CHECK(receiver.expose(frame,{1,0,0},912).valid);
#else
    FAIL("MICA requires an executing additive dual-waveband receiver, not a single image plus a declaration");
#endif
}

TEST_CASE("Dual thermal finite wrapper rejects missing channels and replays measured image streams", "[weapon-thermal-image][thermal-completion]") {
#if FSIM_TEST_DUAL_THERMAL
    using namespace fsim::ms;
    weapon::Spec spec;spec.seeker="imaging_infrared";spec.executionVersion=2;
    spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::ImagingInfrared;spec.seekerModel.engineering.version=3;
    EngineeringSeekerV3 sensor(spec,std::make_unique<ImagingIrSeekerV2>(spec));
    ThermalDualBandConfigV1 c;c.version=1;c.channels={policy(),policy()};c.channels[1].effectiveWavelengthM=10e-6;
    c.angularPairGateRad=.004;c.logRatioGate=.35;c.spectralMemoryS=.2;
    std::string error;REQUIRE(sensor.configureThermalDualBandV1(c,error));
    SeekerFrameV2 frame;frame.dtS=.01;frame.up={0,0,1};frame.boresight={1,0,0};frame.velocity={250,0,0};frame.receiverSeed=812;
    frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
    ThermalDualBandSceneFrameV1 bands;bands.channels={scene(),scene()};bands.channels[1].patches[0].radianceWm2Sr=8;
    for(auto& channel:bands.channels)channel.navigation=pose(0);
    CHECK_FALSE(sensor.sample(frame).valid);CHECK_FALSE(sensor.sampleThermalV1(frame,bands.channels[0]).valid);
    auto invalid=bands;invalid.channels[1].timeS=.01;CHECK_FALSE(sensor.sampleThermalDualBandV1(frame,invalid).valid);
    CHECK_FALSE(sensor.sampleThermalDualBandV1(frame,bands).tracking);
    const auto* firstPointer=sensor.thermalDualBandV1();REQUIRE(firstPointer);if(!firstPointer)return;
    const auto first=*firstPointer;REQUIRE(first.valid);
    REQUIRE(first.channels[0].valid);REQUIRE(first.channels[1].valid);
    frame.timeS=frame.ageS=.01;
    for(auto& channel:bands.channels){channel.timeS=.01;channel.navigation=pose(.01);}
    const auto delivered=sensor.sampleThermalDualBandV1(frame,bands);REQUIRE(delivered.valid);REQUIRE(delivered.tracking);
    CHECK(delivered.measurementTimeS==0);CHECK(delivered.processingReadyTimeS==.01);CHECK(delivered.deliveredTimeS==.01);
    sensor.resetTracking();frame.timeS=frame.ageS=0;
    for(auto& channel:bands.channels){channel.timeS=0;channel.navigation=pose(0);channel.refereeAttributionIndices={4242};}
    sensor.sampleThermalDualBandV1(frame,bands);
    const auto* replayPointer=sensor.thermalDualBandV1();REQUIRE(replayPointer);if(!replayPointer)return;
    const auto replay=*replayPointer;REQUIRE(replay.valid);REQUIRE(replay.features.size()==first.features.size());
    for(unsigned b=0;b<2;++b)CHECK(replay.channels[b].measuredElectrons==first.channels[b].measuredElectrons);
    REQUIRE_FALSE(replay.features.empty());CHECK(replay.features[0].direction.y==first.features[0].direction.y);
    CHECK(replay.features[0].logSpectralRatio==first.features[0].logSpectralRatio);
    CHECK(replay.features[0].refereeAttributionIndex==4242);
#else
    FAIL("Additive dual-waveband execution is required for finite native seeker delivery");
#endif
}

TEST_CASE("Frame-local attribution indices cannot identify retained detector charge after scene replacement", "[weapon-thermal-image][thermal-attribution-history]") {
    auto c=policy();c.pixelResponseTimeS=.2;
    ThermalImageReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto first=scene();first.patches[0].radianceWm2Sr=21;first.refereeAttributionIndices={9};
    auto image=receiver.expose(first,{1,0,0},901,true);REQUIRE(image.valid);REQUIRE_FALSE(image.features.empty());
    CHECK(image.features[0].refereeAttributionIndex==9);
    auto replaced=scene(.02);replaced.patches[0].direction=thermalRay(.018,0);
    // Current frame slot9 now denotes a different, displaced physical return.
    // Prior detector charge remains real but has no stable provenance token.
    replaced.refereeAttributionIndices={9};
    auto retained=receiver.expose(replaced,{1,0,0},901,true);REQUIRE(retained.valid);
    const auto ghost=std::find_if(retained.features.begin(),retained.features.end(),[](const auto& feature){
        return std::abs(std::atan2(feature.direction.y,feature.direction.x))<.004;
    });
    REQUIRE(ghost!=retained.features.end());CHECK_FALSE(ghost->refereeAttributionIndex.has_value());
}

TEST_CASE("Dual spectral history follows the family's selected measured source from first acquisition", "[weapon-thermal-image][thermal-fix-round1]") {
    using namespace fsim::ms;
    std::optional<SeekerResultV2> reference;
    std::array<std::vector<double>,2> referenceImages;
    for(unsigned truthVariant=0;truthVariant<3;++truthVariant) {
        weapon::Spec spec;spec.seeker="imaging_infrared";spec.executionVersion=2;
        spec.seekerModel.version=1;spec.seekerModel.mode=weapon::SeekerModeV1::ImagingInfrared;spec.seekerModel.engineering.version=3;
        EngineeringSeekerV3 sensor(spec,std::make_unique<ImagingIrSeekerV2>(spec));
        ThermalDualBandConfigV1 c;c.version=1;c.channels={policy(),policy()};c.channels[1].effectiveWavelengthM=10e-6;
        c.angularPairGateRad=.004;c.logRatioGate=.35;c.spectralMemoryS=.2;
        std::string error;REQUIRE(sensor.configureThermalDualBandV1(c,error));
        ThermalDualBandSceneFrameV1 bands;bands.channels={scene(),scene()};
        bands.channels[1].patches[0].radianceWm2Sr=8;
        // Both sources exist from the first exposure. Native imaging association
        // selects the dim central measured peak; SNR alone favours the competitor.
        for(unsigned band=0;band<2;++band) {
            auto competitor=bands.channels[band].patches[0];competitor.direction=thermalRay(.014,0);
            competitor.radianceWm2Sr=band?6:30;bands.channels[band].patches.push_back(competitor);
            if(truthVariant==0)bands.channels[band].refereeAttributionIndices={7,99};
            if(truthVariant==1)bands.channels[band].refereeAttributionIndices={99,7};
            if(truthVariant==2)bands.channels[band].refereeAttributionIndices={std::nullopt,std::nullopt};
        }
        SeekerFrameV2 frame;frame.dtS=.01;frame.up={0,0,1};frame.boresight={1,0,0};frame.velocity={250,0,0};frame.receiverSeed=812;
        frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
        SeekerResultV2 firstDelivered,secondDelivered;
        for(unsigned step=0;step<4;++step) {
            const double time=.01*step;frame.timeS=frame.ageS=time;
            for(auto& channel:bands.channels){channel.timeS=time;channel.navigation=pose(time);}
            const auto result=sensor.sampleThermalDualBandV1(frame,bands);
            if(step==1)firstDelivered=result;
            if(step==3)secondDelivered=result;
        }
        INFO(truthVariant);
        REQUIRE(firstDelivered.valid);REQUIRE(firstDelivered.tracking);REQUIRE(firstDelivered.track);
        CHECK(firstDelivered.measurementTimeS==0);CHECK(std::abs(firstDelivered.track->direction.y)<.004);
        REQUIRE(secondDelivered.valid);REQUIRE(secondDelivered.tracking);REQUIRE(secondDelivered.track);
        CHECK(secondDelivered.measurementTimeS==.02);CHECK(std::abs(secondDelivered.track->direction.y)<.004);
        CHECK(secondDelivered.track->localId==firstDelivered.track->localId);
        const auto* image=sensor.thermalDualBandV1();REQUIRE(image);if(!image)return;
        CHECK(image->timeS==.02);
        CHECK(std::any_of(image->features.begin(),image->features.end(),[](const auto& feature){
            return std::abs(feature.direction.y)<.004;
        }));
        if(!reference) {
            reference=secondDelivered;
            for(unsigned band=0;band<2;++band)referenceImages[band]=image->channels[band].measuredElectrons;
        } else {
            REQUIRE(reference->track);
            CHECK(secondDelivered.track->direction.x==reference->track->direction.x);
            CHECK(secondDelivered.track->direction.y==reference->track->direction.y);
            CHECK(secondDelivered.track->direction.z==reference->track->direction.z);
            for(unsigned band=0;band<2;++band)CHECK(image->channels[band].measuredElectrons==referenceImages[band]);
        }
    }
}

TEST_CASE("Each dual channel enforces its independent significant feature support policy", "[weapon-thermal-image][thermal-fix-round1]") {
    ThermalDualBandConfigV1 c;c.version=1;c.channels={policy(),policy()};c.channels[1].effectiveWavelengthM=10e-6;
    c.angularPairGateRad=.004;c.logRatioGate=.35;c.spectralMemoryS=.2;
    ThermalDualBandSceneFrameV1 bands;bands.channels={scene(),scene()};bands.channels[1].patches[0].radianceWm2Sr=8;
    ThermalDualBandReceiverV1 control;std::string error;REQUIRE(control.configure(c,error));
    REQUIRE_FALSE(control.expose(bands,{1,0,0},812).features.empty());
    const double fullSupport=static_cast<double>(c.channels[0].width)*c.channels[0].height;
    for(const auto minimum:std::array<std::array<double,2>,5>{{{{fullSupport,fullSupport}},{{1,fullSupport}},{{fullSupport,1}},{{1,64}},{{64,1}}}}) {
        auto required=c;required.channels[0].minimumFeaturePixels=minimum[0];required.channels[1].minimumFeaturePixels=minimum[1];
        ThermalDualBandReceiverV1 sensor;REQUIRE(sensor.configure(required,error));
        const auto image=sensor.expose(bands,{1,0,0},812);REQUIRE(image.valid);
        INFO("channel minima="<<minimum[0]<<","<<minimum[1]);
        for(unsigned band=0;band<2;++band)if(minimum[band]==fullSupport)CHECK(image.channels[band].features.empty());
        CHECK(image.features.empty());
    }
}

namespace {
template<class Receiver>
auto commitMeasuredFeature(Receiver& receiver,double epoch,std::size_t feature,int)
    ->decltype(receiver.commitSelectedMeasurementV1(epoch,feature)) {
    return receiver.commitSelectedMeasurementV1(epoch,feature);
}
template<class Receiver>
bool commitMeasuredFeature(Receiver&,double,std::size_t,long) {return false;}
}

TEST_CASE("Dual measured-association commits validate epoch and index atomically with stable retries", "[weapon-thermal-image][thermal-fix-round1]") {
    ThermalDualBandConfigV1 c;c.version=1;c.channels={policy(),policy()};c.channels[1].effectiveWavelengthM=10e-6;
    c.angularPairGateRad=.004;c.logRatioGate=.35;c.spectralMemoryS=.2;
    ThermalDualBandReceiverV1 a,b;std::string error;REQUIRE(a.configure(c,error));REQUIRE(b.configure(c,error));
    ThermalDualBandSceneFrameV1 bands;bands.channels={scene(),scene()};bands.channels[1].patches[0].radianceWm2Sr=8;
    for(unsigned band=0;band<2;++band) {
        auto competitor=bands.channels[band].patches[0];competitor.direction=thermalRay(.014,0);
        competitor.radianceWm2Sr=band?6:30;bands.channels[band].patches.push_back(competitor);
        bands.channels[band].refereeAttributionIndices={std::nullopt,std::nullopt};
    }
    const auto first=a.expose(bands,{1,0,0},812),reference=b.expose(bands,{1,0,0},812);
    REQUIRE(first.valid);REQUIRE(reference.valid);REQUIRE(first.features.size()>=2);
    // The missing additive method is an explicit RED assertion, so this test
    // still compiles against the reviewed production headers.
    REQUIRE(commitMeasuredFeature(a,0,0,0));REQUIRE(commitMeasuredFeature(b,0,0,0));
    CHECK(commitMeasuredFeature(a,0,0,0));
    CHECK_FALSE(commitMeasuredFeature(a,0,1,0));
    CHECK_FALSE(commitMeasuredFeature(a,0,first.features.size(),0));
    CHECK_FALSE(commitMeasuredFeature(a,.01,0,0));
    CHECK_FALSE(commitMeasuredFeature(a,std::numeric_limits<double>::quiet_NaN(),0,0));
    for(auto& channel:bands.channels)channel.timeS=.02;
    const auto next=a.expose(bands,{1,0,0},812),expected=b.expose(bands,{1,0,0},812);
    REQUIRE(next.valid);REQUIRE(next.features.size()==expected.features.size());
    for(std::size_t i=0;i<next.features.size();++i) {
        CHECK(next.features[i].direction.x==expected.features[i].direction.x);
        CHECK(next.features[i].direction.y==expected.features[i].direction.y);
        CHECK(next.features[i].logSpectralRatio==expected.features[i].logSpectralRatio);
    }
    for(unsigned band=0;band<2;++band)CHECK(next.channels[band].measuredElectrons==expected.channels[band].measuredElectrons);
    CHECK_FALSE(commitMeasuredFeature(a,0,0,0));
}
