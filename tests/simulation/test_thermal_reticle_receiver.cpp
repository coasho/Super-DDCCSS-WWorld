#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>
#include <limits>
#include <numeric>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <chrono>
#if __has_include("simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h")
#include "simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h"
#include "../reference/frozen_thermal_image_receiver_v1.h"
#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#define FSIM_HAS_RETICLE 1
#endif

#ifndef FSIM_HAS_RETICLE
TEST_CASE("A real photon-domain conical-scan receiver is required", "[thermal-reticle]") { FAIL("Reticle receiver is not implemented"); }
#else
namespace {
using namespace fsim::ms::components;
constexpr double pi=3.14159265358979323846;
fsim::ms::weapon::V3 unitRay(double x,double y,double z) {
    const double length=std::hypot(x,std::hypot(y,z));return {x/length,y/length,z/length};
}
ThermalReticleConfigV1 reticlePolicy() {
    ThermalReticleConfigV1 c;c.version=1;c.periods=12;c.samplesPerScan=128;c.spatialSamples=4;
    c.scanRadiusRad=.024;c.stopRadiusRad=.048;c.frequencyHz=100;
    auto& p=c.detector;p.version=1;p.width=64;p.height=64;p.adcBits=16;p.exposureSamples=4;
    p.pixelPitchRad=.0015;p.psfSigmaRad=.00075;p.apertureAreaM2=.0007917304360898403;
    p.opticalTransmission=.7;p.quantumEfficiency=.6;p.effectiveWavelengthM=4e-6;
    p.integrationTimeS=1e-5;p.pixelResponseTimeS=1e-4;p.readNoiseElectrons=2;
    p.darkCurrentElectronsS=1;p.fullWellElectrons=1e6;p.detectionSigma=5;
    p.backgroundRadiusPixels=5;p.minimumFeaturePixels=1;p.associationGateRad=.03;
    return c;
}
ThermalSceneFrameV1 reticleScene(double time,double contrast=0) {
    ThermalSceneFrameV1 f;f.timeS=time;f.up={0,0,1};f.skyRadianceWm2Sr=f.groundRadianceWm2Sr=1;
    if(contrast!=0) {
        ThermalRadiancePatchV1 p;p.direction=unitRay(1,.003,-.0015);p.rangeM=1000;
        p.widthRad=p.heightRad=.003;p.radianceWm2Sr=1+contrast;p.transmission=1;p.pathRadianceWm2Sr=1;
        f.patches.push_back(p);f.refereeAttributionIndices.push_back(9);
    }
    return f;
}
void retainReticleWaveform(const char* name,const ThermalReticleResultV1& result) {
    const auto amplitude=[&](const std::vector<double>& values,unsigned harmonic) {
        if(values.size()!=result.phases.size())return 0.;
        double real=0,imaginary=0;
        for(std::size_t j=0;j<values.size();++j){real+=values[j]*std::cos(harmonic*result.phases[j]);imaginary+=values[j]*std::sin(harmonic*result.phases[j]);}
        return values.empty()?0.:2*std::hypot(real,imaginary)/static_cast<double>(values.size());
    };
    UNSCOPED_INFO("native scalar "<<name<<" valid="<<result.valid<<" complete="<<result.complete<<" saturation="<<result.saturated
        <<" LSB="<<result.adcStepElectrons<<" fitSnr="<<result.fitSnr<<" residual="<<result.fitResidual
        <<" incidentH3="<<amplitude(result.incidentElectrons,3)<<" incidentH5="<<amplitude(result.incidentElectrons,5)
        <<" incidentH12="<<amplitude(result.incidentElectrons,12)<<" measuredH12="<<amplitude(result.measuredElectrons,12));
    if(const auto* directory=std::getenv("FSIM_THERMAL_RETICLE_REFERENCE_DIR")) {
        const auto root=std::filesystem::path(directory);
        for(const auto& part:root){auto text=part.generic_string();for(auto& ch:text)if(ch>='A'&&ch<='Z')ch=static_cast<char>(ch-'A'+'a');REQUIRE(text!=".codex");}
        std::filesystem::create_directories(root);
        std::ofstream out(root/(std::string(name)+".csv"));REQUIRE(out.good());out<<std::setprecision(17);
        out<<"phase_rad,incident_electrons,expected_electrons,measured_electrons,adc_step_electrons,saturated,fit_snr,fit_residual,variance_bound_electrons2\n";
        REQUIRE(result.incidentElectrons.size()==result.phases.size());REQUIRE(result.expectedElectrons.size()==result.phases.size());REQUIRE(result.measuredElectrons.size()==result.phases.size());
        for(std::size_t j=0;j<result.phases.size();++j)out<<result.phases[j]<<','<<result.incidentElectrons[j]<<','<<result.expectedElectrons[j]<<','<<result.measuredElectrons[j]<<','<<result.adcStepElectrons<<','<<result.saturated<<','<<result.fitSnr<<','<<result.fitResidual<<','<<result.varianceBoundElectrons2<<'\n';
        out.flush();REQUIRE(out.good());
    }
}
}

TEST_CASE("Reticle reference carrier and off-axis phase derive from real nutation", "[thermal-reticle][thermal-reticle-reference]") {
    const auto c=reticlePolicy();
    // Published SA-14 model reference only:100Hz optics,12 radial periods.
    CHECK(c.frequencyHz*c.periods==1200);
    for(const double phase:{.3,1.1,2.4,4.2}) {
        CHECK(ThermalReticleReceiverV1::angularSpeedV1(0,0,c.scanRadiusRad,phase,c.frequencyHz)==Catch::Approx(200*pi).epsilon(1e-12));
        const double x=.002,y=-.003,h=1e-8;
        const auto angle=[&](double p){return std::atan2(y+c.scanRadiusRad*std::sin(p),x+c.scanRadiusRad*std::cos(p));};
        const double numerical=(angle(phase+h)-angle(phase-h))/(2*h)*200*pi;
        CHECK(ThermalReticleReceiverV1::angularSpeedV1(x,y,c.scanRadiusRad,phase,c.frequencyHz)==Catch::Approx(numerical).margin(2e-5));
    }
    CHECK(ThermalReticleReceiverV1::maskV1(c,1,1,0)==0);
    unsigned transitions=0;double previous=ThermalReticleReceiverV1::maskV1(c,0,0,2*pi*.25/4096);
    for(unsigned j=1;j<=4096;++j) {
        const double current=ThermalReticleReceiverV1::maskV1(c,0,0,2*pi*(j+.25)/4096);
        if(current!=previous)++transitions;
        previous=current;
    }
    CHECK(transitions==24);CHECK(.5*transitions*c.frequencyHz==1200);
}

TEST_CASE("Actual reticle photons reproduce the independent primary carrier and charge response", "[thermal-reticle][thermal-reticle-reference]") {
    const auto c=reticlePolicy();ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto f=reticleScene(0,100);f.patches[0].direction={1,0,0};receiver.expose(f,{1,0,0},17);
    f.timeS=.02;const auto result=receiver.expose(f,{1,0,0},17);REQUIRE(result.valid);REQUIRE(result.complete);
    REQUIRE(result.incidentElectrons.size()==c.samplesPerScan);
    CHECK(result.fullWellElectrons==c.detector.fullWellElectrons*c.detector.width*c.detector.height);
    CHECK(result.readNoiseElectrons==Catch::Approx(c.detector.readNoiseElectrons*std::sqrt(c.detector.width*c.detector.height)));
    CHECK(result.adcStepElectrons==Catch::Approx(result.fullWellElectrons/65535));
    const auto harmonic=[&](unsigned k) {
        double real=0,imaginary=0;
        for(unsigned j=0;j<c.samplesPerScan;++j) {
            real+=result.incidentElectrons[j]*std::cos(k*result.phases[j]);
            imaginary+=result.incidentElectrons[j]*std::sin(k*result.phases[j]);
        }
        return std::hypot(real,imaginary);
    };
    CHECK(harmonic(12)>harmonic(11));CHECK(harmonic(12)>harmonic(13));
    double previous=0,previousTime=0;
    for(unsigned j=0;j<c.samplesPerScan;++j) {
        const double time=result.exposureStartTimeS+(j+.5)/(c.frequencyHz*c.samplesPerScan);
        const double alpha=-std::expm1(-(time-previousTime)/c.detector.pixelResponseTimeS);
        previous+=alpha*(result.incidentElectrons[j]-previous);
        CHECK(result.expectedElectrons[j]==Catch::Approx(previous).epsilon(1e-12));previousTime=time;
        CHECK(result.measuredElectrons[j]>=0);CHECK(result.measuredElectrons[j]<=result.fullWellElectrons);
        CHECK(result.measuredElectrons[j]/result.adcStepElectrons==Catch::Approx(std::round(result.measuredElectrons[j]/result.adcStepElectrons)).margin(1e-9));
    }
    if(const auto* directory=std::getenv("FSIM_THERMAL_RETICLE_REFERENCE_DIR")) {
        const auto root=std::filesystem::path(directory);
        for(const auto& part:root) {
            auto text=part.generic_string();for(auto& ch:text)if(ch>='A'&&ch<='Z')ch=static_cast<char>(ch-'A'+'a');
            REQUIRE(text!=".codex");
        }
        std::filesystem::create_directories(root);
        std::ofstream out(root/"reticle-primary-waveform.csv");REQUIRE(out.good());out<<std::setprecision(17);
        out<<"phase_rad,incident_electrons,expected_electrons,measured_electrons\n";
        for(unsigned j=0;j<c.samplesPerScan;++j)out<<result.phases[j]<<','<<result.incidentElectrons[j]<<','<<result.expectedElectrons[j]<<','<<result.measuredElectrons[j]<<'\n';
        out.flush();REQUIRE(out.good());
    }
}

TEST_CASE("Bright short optical input is not clipped before a slow scalar response", "[thermal-reticle][thermal-reticle-shot-order]") {
    auto c=reticlePolicy();const double firstElapsed=.5/(c.frequencyHz*c.samplesPerScan),alpha=.05;
    c.detector.pixelResponseTimeS=-firstElapsed/std::log1p(-alpha);
    const auto& p=c.detector;const double fullWell=p.fullWellElectrons*p.width*p.height;
    // Independent geometric quadrature of the declared stationary mask gives
    // the UNIFORM RADIANCE producing exactly2C incident charge at the first
    // detector sample. No stochastic output or terminal outcome sets input.
    const auto primitive=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    double throughput=0;
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const double x=(col+.5-.5*p.width)*p.pixelPitchRad,y=(row+.5-.5*p.height)*p.pixelPitchRad;
        const double x0=std::tan(x-p.pixelPitchRad/2),x1=std::tan(x+p.pixelPitchRad/2);
        const double y0=std::tan(y-p.pixelPitchRad/2),y1=std::tan(y+p.pixelPitchRad/2);
        const double omega=primitive(x1,y1)-primitive(x0,y1)-primitive(x1,y0)+primitive(x0,y0);
        for(unsigned k=0;k<p.exposureSamples;++k)for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx) {
            const double phase=2*pi*(.5/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
            const double qx=x+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad+c.scanRadiusRad*std::cos(phase);
            const double qy=y+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad+c.scanRadiusRad*std::sin(phase);
            if(qx*qx+qy*qy<=c.stopRadiusRad*c.stopRadiusRad&&std::cos(c.periods*std::atan2(qy,qx))>=0)
                throughput+=omega/(static_cast<double>(p.exposureSamples)*c.spatialSamples*c.spatialSamples);
        }
    }
    const double gain=p.apertureAreaM2*p.opticalTransmission*p.integrationTimeS*p.quantumEfficiency*p.effectiveWavelengthM/(6.62607015e-34*299792458.);
    const double dark=p.width*p.height*p.darkCurrentElectronsS*p.integrationTimeS;
    const double radiance=(2*fullWell-dark)/(throughput*gain);
    ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    auto f=reticleScene(0);f.skyRadianceWm2Sr=f.groundRadianceWm2Sr=radiance;receiver.expose(f,{1,0,0},17);
    f.timeS=1/c.frequencyHz;const auto actual=receiver.expose(f,{1,0,0},17);REQUIRE(actual.valid);REQUIRE(actual.complete);
    REQUIRE_FALSE(actual.incidentElectrons.empty());
    CHECK(actual.incidentElectrons.front()==Catch::Approx(2*fullWell).epsilon(1e-9));
    const double expected=alpha*2*fullWell;
    CHECK(actual.expectedElectrons.front()==Catch::Approx(expected).epsilon(1e-9));
    // Five shot/read standard deviations plus the exact half-LSB bound.
    const double bound=5*(alpha*std::sqrt(2*fullWell)+actual.readNoiseElectrons)+actual.adcStepElectrons/2;
    REQUIRE(bound<.01*fullWell);CHECK(actual.measuredElectrons.front()>0);CHECK(actual.measuredElectrons.front()<fullWell);
    CHECK(std::abs(actual.measuredElectrons.front()-expected)<=bound);
}

TEST_CASE("Reticle scalar detector is causal repeatable and independent of referee attribution", "[thermal-reticle]") {
    const auto c=reticlePolicy();ThermalReticleReceiverV1 a,b;std::string error;
    REQUIRE(a.configure(c,error));REQUIRE(b.configure(c,error));
    auto f=reticleScene(0,100);REQUIRE(f.patches.size()==1);REQUIRE(f.refereeAttributionIndices.size()==1);
    auto g=f;g.refereeAttributionIndices.clear();g.refereeAttributionIndices.emplace_back(999);
    const auto start=a.expose(f,{1,0,0},17);const auto other=b.expose(g,{1,0,0},17);
    REQUIRE(start.valid);REQUIRE(other.valid);CHECK_FALSE(start.complete);CHECK(start.features.empty());
    f.timeS=g.timeS=.02;
    const auto x=a.expose(f,{1,0,0},17),y=b.expose(g,{1,0,0},17);
    REQUIRE(x.valid);REQUIRE(x.complete);CHECK(x.expectedElectrons.size()==c.samplesPerScan);
    REQUIRE(x.features.size()==1);CHECK(x.bearingResolved);CHECK_FALSE(x.saturated);
    CHECK(std::atan2(x.features[0].direction.y,x.features[0].direction.x)==Catch::Approx(.003).margin(2*c.detector.pixelPitchRad));
    CHECK(std::atan2(x.features[0].direction.z,x.features[0].direction.x)==Catch::Approx(-.0015).margin(2*c.detector.pixelPitchRad));
    CHECK(x.measuredElectrons==y.measuredElectrons);CHECK(x.expectedElectrons==y.expectedElectrons);
    CHECK(x.measurementTimeS==Catch::Approx(.015));CHECK(x.exposureStartTimeS>=0);
    CHECK(a.expose(f,{1,0,0},17).measuredElectrons==x.measuredElectrons);
    a.reset();a.expose(reticleScene(0,100),{1,0,0},17);
    CHECK(a.expose(f,{1,0,0},17).measuredElectrons==x.measuredElectrons);
}

TEST_CASE("Reticle uniform background has no deterministic carrier and never supplies a target identity", "[thermal-reticle]") {
    auto c=reticlePolicy();ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    receiver.expose(reticleScene(0),{1,0,0},17);
    const auto r=receiver.expose(reticleScene(.02),{1,0,0},17);REQUIRE(r.valid);REQUIRE(r.complete);
    CHECK(r.features.empty());
    // Finite optical quadrature is symmetric; exact DC is tested separately
    // from random photon/read/ADC output, which remains enabled here.
    REQUIRE(r.expectedElectrons.size()==c.samplesPerScan);
    for(const double value:r.expectedElectrons)CHECK(std::isfinite(value));
    CHECK_FALSE(r.bearingResolved);
}

TEST_CASE("Reticle rejects extended DC clutter and retains an actual compact signal across a structured horizon", "[thermal-reticle][thermal-reticle-clutter]") {
    const auto c=reticlePolicy();std::string error;
    for(const bool source:{false,true}) {
        ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(c,error));
        auto f=reticleScene(0,source?100:0);f.skyRadianceWm2Sr=1;f.groundRadianceWm2Sr=3;f.groundHorizonCosine=-.001;
        receiver.expose(f,{1,0,0},17);f.timeS=.02;const auto result=receiver.expose(f,{1,0,0},17);
        retainReticleWaveform(source?"reticle-horizon-source":"reticle-horizon-empty",result);
        REQUIRE(result.valid);REQUIRE(result.complete);CHECK_FALSE(result.saturated);
        if(source){REQUIRE(result.features.size()==1);CHECK(result.bearingResolved);}
        else {CHECK(result.features.empty());CHECK_FALSE(result.bearingResolved);}
    }
    ThermalReticleReceiverV1 cloud;REQUIRE(cloud.configure(c,error));
    auto f=reticleScene(0,2);f.patches[0].direction={1,0,0};f.patches[0].widthRad=f.patches[0].heightRad=.4;
    cloud.expose(f,{1,0,0},17);f.timeS=.02;const auto result=cloud.expose(f,{1,0,0},17);
    REQUIRE(result.valid);CHECK_FALSE(result.saturated);CHECK(result.features.empty());
}

TEST_CASE("Reticle out-of-field light and source order cannot manufacture a measurement", "[thermal-reticle]") {
    const auto c=reticlePolicy();std::string error;ThermalReticleReceiverV1 absent,outside;
    REQUIRE(absent.configure(c,error));REQUIRE(outside.configure(c,error));
    auto a=reticleScene(0),b=reticleScene(0,100);b.patches[0].direction=unitRay(1,.3,.2);
    absent.expose(a,{1,0,0},17);outside.expose(b,{1,0,0},17);a.timeS=b.timeS=.02;
    const auto x=absent.expose(a,{1,0,0},17),y=outside.expose(b,{1,0,0},17);
    REQUIRE(x.valid);REQUIRE(y.valid);CHECK(x.incidentElectrons==y.incidentElectrons);
    CHECK(x.measuredElectrons==y.measuredElectrons);CHECK(y.features.empty());
    ThermalReticleReceiverV1 first,reordered;REQUIRE(first.configure(c,error));REQUIRE(reordered.configure(c,error));
    a=reticleScene(0,100);auto flare=a.patches[0];flare.direction=unitRay(1,-.004,.002);flare.radianceWm2Sr=72;flare.rangeM=1500;
    a.patches.push_back(flare);a.refereeAttributionIndices.push_back(17);b=a;
    std::reverse(b.patches.begin(),b.patches.end());b.refereeAttributionIndices={999,888};
    first.expose(a,{1,0,0},17);reordered.expose(b,{1,0,0},17);a.timeS=b.timeS=.02;
    const auto p=first.expose(a,{1,0,0},17),q=reordered.expose(b,{1,0,0},17);
    REQUIRE(p.valid);REQUIRE(q.valid);CHECK(p.incidentElectrons==q.incidentElectrons);CHECK(p.measuredElectrons==q.measuredElectrons);
    REQUIRE(p.features.size()==q.features.size());
    if(!p.features.empty()){CHECK(p.features[0].direction.x==q.features[0].direction.x);CHECK(p.features[0].direction.y==q.features[0].direction.y);CHECK_FALSE(p.features[0].refereeAttributionIndex);}
}

TEST_CASE("Reticle rejects invalid aliased or noncausal policies without changing its last sample", "[thermal-reticle]") {
    auto c=reticlePolicy();ThermalReticleReceiverV1 receiver;std::string error;
    REQUIRE(receiver.configure(c,error));receiver.expose(reticleScene(0),{1,0,0},17);
    const auto before=receiver.expose(reticleScene(.02),{1,0,0},17);
    for(unsigned i=0;i<4;++i) {
        auto invalid=c;
        if(i==0)invalid.frequencyHz=0;
        if(i==1)invalid.samplesPerScan=invalid.periods;
        if(i==2)invalid.scanRadiusRad=std::numeric_limits<double>::quiet_NaN();
        if(i==3)invalid.detector.integrationTimeS=1;
        CHECK_FALSE(receiver.configure(invalid,error));
        CHECK(receiver.expose(reticleScene(.02),{1,0,0},17).measuredElectrons==before.measuredElectrons);
    }
    CHECK_FALSE(receiver.expose(reticleScene(.01),{1,0,0},17).valid);
}

TEST_CASE("Unsupported scalar shot domain refuses without consuming the receiver history", "[thermal-reticle][thermal-reticle-shot-order]") {
    const auto c=reticlePolicy();ThermalReticleReceiverV1 actual,control;std::string error;
    REQUIRE(actual.configure(c,error));REQUIRE(control.configure(c,error));
    for(const double time:{0.,.02}) {actual.expose(reticleScene(time),{1,0,0},17);control.expose(reticleScene(time),{1,0,0},17);}
    auto unsupported=reticleScene(.03);unsupported.skyRadianceWm2Sr=unsupported.groundRadianceWm2Sr=1e9;
    REQUIRE(ThermalImageReceiverV1::validScene(unsupported,c.detector));
    CHECK_FALSE(actual.expose(unsupported,{1,0,0},17).valid);
    const auto a=actual.expose(reticleScene(.04),{1,0,0},17),b=control.expose(reticleScene(.04),{1,0,0},17);
    REQUIRE(a.valid);REQUIRE(b.valid);CHECK(a.expectedElectrons==b.expectedElectrons);CHECK(a.measuredElectrons==b.measuredElectrons);
}

TEST_CASE("Optical extraction preserves the independent frozen single-band detector bit for bit", "[thermal-reticle-optical-oracle]") {
    static_assert(sizeof(ThermalImageReceiverV1)==sizeof(FrozenThermalImageReceiverV1));
    const auto c=reticlePolicy().detector;std::string error;
    for(const bool imaging:{false,true}) {
        ThermalImageReceiverV1 current;FrozenThermalImageReceiverV1 frozen;
        REQUIRE(current.configure(c,error));REQUIRE(frozen.configure(c,error));
        for(unsigned i=0;i<12;++i) {
            auto f=reticleScene(i*.02,100);
            f.patches[0].direction=unitRay(1,.0003*static_cast<double>(i)-.002,-.001);
            f.patches[0].directionRate={0,.01,0};
            if(i>=4){f.skyRadianceWm2Sr=1;f.groundRadianceWm2Sr=3;f.groundHorizonCosine=-.001;}
            if(i>=8) {
                auto nearer=f.patches[0];nearer.direction=unitRay(1,.001,.001);nearer.rangeM=500;
                nearer.radianceWm2Sr=4;f.patches.push_back(nearer);f.refereeAttributionIndices.push_back(27);
                if(i%2){std::reverse(f.patches.begin(),f.patches.end());std::reverse(f.refereeAttributionIndices.begin(),f.refereeAttributionIndices.end());}
            }
            const auto a=current.expose(f,{1.013,.00013*static_cast<double>(i),.00071},19,imaging);
            const auto b=frozen.expose(f,{1.013,.00013*static_cast<double>(i),.00071},19,imaging);
            REQUIRE(a.valid);REQUIRE(b.valid);CHECK(a.expectedElectrons==b.expectedElectrons);
            CHECK(a.measuredElectrons==b.measuredElectrons);CHECK(a.backgroundElectrons==b.backgroundElectrons);
            CHECK(a.contrastElectrons==b.contrastElectrons);CHECK(a.noiseSigmaElectrons==b.noiseSigmaElectrons);
            CHECK(a.detectionThresholdElectrons==b.detectionThresholdElectrons);CHECK(a.segmentation==b.segmentation);
            CHECK(a.quadrantElectrons==b.quadrantElectrons);CHECK(a.quadrantBackgroundElectrons==b.quadrantBackgroundElectrons);
            CHECK(a.quadrantVarianceElectrons2==b.quadrantVarianceElectrons2);REQUIRE(a.features.size()==b.features.size());
            for(std::size_t n=0;n<a.features.size();++n) {
                const auto& x=a.features[n];const auto& y=b.features[n];
                CHECK(x.direction.x==y.direction.x);CHECK(x.direction.y==y.direction.y);CHECK(x.direction.z==y.direction.z);
                CHECK(x.signalToNoise==y.signalToNoise);CHECK(x.angularVarianceRad2==y.angularVarianceRad2);
                CHECK(x.signedContrastElectrons==y.signedContrastElectrons);CHECK(x.centroidColumn==y.centroidColumn);CHECK(x.centroidRow==y.centroidRow);
                CHECK(x.extent.widthRad==y.extent.widthRad);CHECK(x.extent.heightRad==y.extent.heightRad);
                CHECK(x.refereeAttributionIndex==y.refereeAttributionIndex);
            }
        }
    }
}

TEST_CASE("All seven actual nonimaging cards select and execute the scalar reticle mechanism", "[thermal-reticle][thermal-reticle-population]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    for(const auto* id:{"aim-9m","magic-2","r-60m","r-73","r-27t","r-27et","pl-8"}) {
        INFO(id);std::string error;const auto card=cards.at(id);ProfileV1 profile;
        REQUIRE(readProfileV1(library/(std::string(id)+".json"),*card,profile,error));
        const auto descriptor=*profile.find("seeker");REQUIRE(descriptor.bindings.at("receiver_model")=="thermal_conscan_reticle.v1");
        DescriptorV1 scalar;ReceiverConfigurationV3 receiver;
        REQUIRE(receiverConfigurationV3(descriptor,scalar,receiver,error));REQUIRE(receiver.reticle);
        CHECK_FALSE(receiver.inherited.dual);CHECK(receiver.inherited.single.kind==PhysicalReceiverKindV1::Scalar);
        CHECK(receiver.reticle->policy.periods==12);CHECK(receiver.reticle->policy.frequencyHz==Catch::Approx(132.62911924324612));
        auto malformed=descriptor;malformed.bindings["reticle_pattern"]="unknown";
        CHECK_FALSE(receiverConfigurationV3(malformed,scalar,receiver,error));
        malformed=descriptor;malformed.parameters.erase("receiver_reticle_frequency_hz");
        CHECK_FALSE(receiverConfigurationV3(malformed,scalar,receiver,error));
        weapon::Spec spec;REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
        ProviderContextV1 context{*card,profile,spec,std::nullopt};
        auto provider=builtInProvidersV1().seeker.create(descriptor,context,error);INFO(error);REQUIRE(provider);
        auto* native=dynamic_cast<ThermalReticleSeekerExecutionV1*>(provider.get());REQUIRE(native);
        auto* old=dynamic_cast<ThermalSeekerExecutionV1*>(provider.get());REQUIRE(old);
        bool measured=false,tracked=false;
        for(unsigned i=0;i<30;++i) {
            SeekerFrameV2 frame;frame.timeS=frame.ageS=i*.01;frame.dtS=.01;frame.velocity={250,0,0};
            frame.boresight={1,0,0};frame.up={0,0,1};frame.receiverSeed=17;
            frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
            auto optical=reticleScene(frame.timeS,100);SeekerNavigationPoseV1 navigation;
            navigation.valid=true;navigation.timeS=frame.timeS;navigation.velocity=frame.velocity;navigation.bodyToEcef=*frame.bodyAxes;
            optical.navigation=navigation;
            // Native source-field input only. There is no scalar target sample
            // and the legacy quadrant execution interface must refuse this profile.
            CHECK_FALSE(old->sampleThermalV1(frame,optical).valid);
            const auto actual=native->sampleThermalReticleV1(frame,optical);REQUIRE(actual.valid);
            if(!actual.measurementTimeS) {
                // A valid startup frame is not a completed detector exposure.
                // Preserve the initial view until the first real scan and its
                // processing delay have supplied a delivered measurement.
                const auto& pointing=actual.pointingDirection;
                REQUIRE(std::isfinite(pointing.x));REQUIRE(std::isfinite(pointing.y));REQUIRE(std::isfinite(pointing.z));
                CHECK(std::hypot(pointing.x,std::hypot(pointing.y,pointing.z))==Catch::Approx(1).margin(16*std::numeric_limits<double>::epsilon()));
                CHECK(pointing.x>=1-16*std::numeric_limits<double>::epsilon());
            }
            if(const auto* raw=native->thermalReticleV1();raw&&raw->complete) {
                measured=true;CHECK(raw->measurementTimeS<=frame.timeS);CHECK(raw->phases.size()==128);
                if(!raw->features.empty())CHECK_FALSE(raw->features[0].refereeAttributionIndex);
            }
            if(actual.tracking&&actual.track) {
                tracked=true;REQUIRE(actual.measurementTimeS);REQUIRE(actual.processingReadyTimeS);REQUIRE(actual.deliveredTimeS);
                CHECK(actual.track->sampleTimeS==*actual.measurementTimeS);
                CHECK(*actual.measurementTimeS<*actual.processingReadyTimeS);CHECK(*actual.processingReadyTimeS<=*actual.deliveredTimeS+1e-12);
                CHECK_FALSE(actual.track->rangeM);CHECK_FALSE(actual.track->rangeRateMs);
            }
        }
        CHECK(measured);CHECK(tracked);
    }
}

TEST_CASE("Reticle midpoint navigation removes own rotation from measured inertial LOS rate", "[thermal-reticle][thermal-reticle-navigation]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;std::string error;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());const auto card=cards.at("aim-9m");
    ProfileV1 profile;REQUIRE(readProfileV1(library/"aim-9m.json",*card,profile,error));
    weapon::Spec spec;REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
    ProviderContextV1 context{*card,profile,spec,std::nullopt};
    auto first=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);
    auto second=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);
    REQUIRE(first);REQUIRE(second);
    auto* a=dynamic_cast<ThermalReticleSeekerExecutionV1*>(first.get());auto* b=dynamic_cast<ThermalReticleSeekerExecutionV1*>(second.get());REQUIRE(a);REQUIRE(b);
    constexpr double rotationRate=.4;
    struct Sensitivity {double position,rate,sigma;};std::vector<Sensitivity> coefficients;
    double priorEpoch=-1;unsigned compared=0;
    for(unsigned i=0;i<90;++i) {
        std::array<SeekerResultV2,2> results;
        for(unsigned index=0;index<2;++index) {
            const double time=i/120.,yaw=index?rotationRate*time:0;
            SeekerFrameV2 frame;frame.timeS=frame.ageS=time;frame.dtS=1/120.;frame.receiverSeed=17;frame.velocity={250,0,0};frame.up={0,0,1};
            frame.bodyAxes=std::array<weapon::V3,3>{{{std::cos(yaw),std::sin(yaw),0},{-std::sin(yaw),std::cos(yaw),0},{0,0,1}}};frame.boresight=(*frame.bodyAxes)[0];
            auto optical=reticleScene(time,100);SeekerNavigationPoseV1 navigation;
            navigation.valid=true;navigation.timeS=time;navigation.velocity=frame.velocity;navigation.bodyToEcef=*frame.bodyAxes;optical.navigation=navigation;
            results[index]=(index?b:a)->sampleThermalReticleV1(frame,optical);REQUIRE(results[index].valid);
        }
        const auto& x=results[0];const auto& y=results[1];
        if(!x.tracking||!y.tracking||!x.track||!y.track)continue;
        REQUIRE(x.track->sampleTimeS==y.track->sampleTimeS);
        const double epoch=x.track->sampleTimeS;if(epoch==priorEpoch)continue;
        REQUIRE(x.angularVarianceRad2);REQUIRE(y.angularVarianceRad2);
        const double sigma=5*std::sqrt(2*(*x.angularVarianceRad2+*y.angularVarianceRad2));
        REQUIRE(std::isfinite(sigma));REQUIRE(sigma<.1);
        if(priorEpoch<0)coefficients.push_back({1,0,sigma});
        else {
            const double elapsed=epoch-priorEpoch;REQUIRE(elapsed>0);
            const double alpha=-std::expm1(-elapsed/first->config().engineering.trackLagS),beta=alpha*alpha;
            // Analytic l1 impulse bound of the declared alpha-beta filter.
            // It is derived from input covariance and cadence before observing
            // the rate predicate; it is not an outcome-fitted tolerance.
            for(auto& coefficient:coefficients) {
                const double prediction=coefficient.position+elapsed*coefficient.rate;
                coefficient.position=(1-alpha)*prediction;coefficient.rate-=beta*prediction/elapsed;
            }
            coefficients.push_back({alpha,beta/elapsed,sigma});
            if(x.track->rateAvailable&&y.track->rateAvailable) {
                double bound=0;for(const auto& coefficient:coefficients)bound+=std::abs(coefficient.rate)*coefficient.sigma;
                // Unit-ray normalization has Lipschitz factor<=2 inside the
                // admitted0.1rad measured uncertainty region above.
                bound*=2;REQUIRE(bound<rotationRate);
                const auto& u=x.track->directionRate;const auto& v=y.track->directionRate;
                CHECK(std::hypot(u.x-v.x,std::hypot(u.y-v.y,u.z-v.z))<=bound);++compared;
            }
        }
        priorEpoch=epoch;
    }
    REQUIRE(compared>=4);
}

TEST_CASE("Actual thermal association predicts at the measured capture epoch", "[thermal-reticle][thermal-reticle-association]") {
    using namespace fsim::ms;using namespace fsim::ms::components::measured;
    std::vector<std::string> errors;std::string error;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());const auto card=cards.at("aim-9m");
    ProfileV1 profile;REQUIRE(readProfileV1(library/"aim-9m.json",*card,profile,error));
    weapon::Spec spec;REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 receiver;REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,receiver,error));REQUIRE(receiver.reticle);
    ProviderContextV1 context{*card,profile,spec,std::nullopt};auto provider=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);REQUIRE(provider);
    auto* sensor=dynamic_cast<ThermalReticleSeekerExecutionV1*>(provider.get());REQUIRE(sensor);
    std::optional<SeekerTrackV2> captured;
    for(unsigned i=0;i<30&&!captured;++i) {
        SeekerFrameV2 frame;frame.timeS=frame.ageS=i*.01;frame.dtS=.01;frame.receiverSeed=17;frame.up={0,0,1};frame.boresight={1,0,0};
        frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
        auto optical=reticleScene(frame.timeS,100);SeekerNavigationPoseV1 navigation;
        navigation.valid=true;navigation.timeS=frame.timeS;navigation.bodyToEcef=*frame.bodyAxes;optical.navigation=navigation;
        const auto result=sensor->sampleThermalReticleV1(frame,optical);REQUIRE(result.valid);
        if(result.tracking&&result.track)captured=result.track;
    }
    REQUIRE(captured);
    // Direct production-method counterexample: retain a REAL measured bearing
    // and sample epoch, prescribe a finite tangent rate, and compare exact
    // capture-vs-read prediction. This does not claim a full engagement at the
    // prescribed rate; the separate own-rotation case exercises coupled optics.
    auto track=*captured;const double gate=receiver.reticle->policy.detector.associationGateRad,latency=.02;
    const auto tangent=unit(cross(track.direction,V{0,0,1}));
    track.directionRate=scale(tangent,2*std::tan(gate)/latency);track.rateAvailable=true;
    const double capture=track.sampleTimeS,read=capture+latency;
    CHECK(EngineeringSeekerV3::measuredAssociationV1(track.direction,track,capture,read,gate));
    CHECK_FALSE(EngineeringSeekerV3::measuredAssociationV1(track.direction,track,read,read,gate));
    CHECK_FALSE(EngineeringSeekerV3::measuredAssociationV1(track.direction,track,read,capture,gate));
    for(const double epoch:{capture,read}) {
        const auto legacy=unit(add(track.direction,scale(track.directionRate,std::max(0.,epoch-track.sampleTimeS))));
        CHECK(EngineeringSeekerV3::measuredAssociationV1(legacy,track,epoch,epoch,gate));
    }
    track.rateAvailable=false;track.directionRate={};
    CHECK(EngineeringSeekerV3::measuredAssociationV1(track.direction,track,capture,read,gate));
    CHECK(EngineeringSeekerV3::measuredAssociationV1(track.direction,track,read,read,gate));
}
TEST_CASE("Fast reticle unit-cone calibration preserves the original continuous optical integral", "[thermal-reticle][thermal-reticle-calibration]") {
    auto p=reticlePolicy().detector;p.exposureSamples=1;
    using Ray=fsim::ms::weapon::V3;
    const auto earthUp=unitRay(.4,.6,.6928),tangent=unitRay(-earthUp.y,earthUp.x,0);
    const auto earthForward=unitRay(tangent.x+.013*earthUp.x,tangent.y+.013*earthUp.y,tangent.z+.013*earthUp.z);
    const std::array<std::pair<Ray,Ray>,3> poses{{{{1,0,0},{0,0,1}},
        {unitRay(1,.05,.2),{0,0,1}},{earthForward,earthUp}}};
    const std::array<std::optional<std::array<double,4>>,3> regions{{std::nullopt,
        std::array<double,4>{-.004,.006,-.003,.002},std::array<double,4>{-.00225,.00075,-.00002,.00003}}};
    std::size_t fastCount=0,cases=0;double maximumError=0,referenceSeconds=0,fastSeconds=0;
    for(std::size_t pose=0;pose<poses.size();++pose)for(const double offset:{-.027,0.,.019})for(const auto& region:regions) {
        const auto f=poses[pose].first,u=poses[pose].second;
        const double cutoff=std::clamp(f.x*u.x+f.y*u.y+f.z*u.z+offset,-1.,1.);
        ThermalSceneFrameV1 hypothesis;hypothesis.up=u;hypothesis.groundRadianceWm2Sr=1;hypothesis.groundHorizonCosine=cutoff;
        const auto referenceStart=std::chrono::steady_clock::now();
        const auto ground=ThermalImageReceiverV1::incidentFieldV1(hypothesis,f,p);REQUIRE(ground);
        auto expected=ground->radiance;
        if(region) {
            const auto right=unitRay(f.y*u.z-f.z*u.y,f.z*u.x-f.x*u.z,f.x*u.y-f.y*u.x);
            const Ray vertical{right.y*f.z-right.z*f.y,right.z*f.x-right.x*f.z,right.x*f.y-right.y*f.x};
            const double x=.5*((*region)[0]+(*region)[1]),y=.5*((*region)[2]+(*region)[3]);
            ThermalRadiancePatchV1 black;
            black.direction=unitRay(f.x+right.x*std::tan(x)+vertical.x*std::tan(y),
                f.y+right.y*std::tan(x)+vertical.y*std::tan(y),f.z+right.z*std::tan(x)+vertical.z*std::tan(y));
            black.widthRad=(*region)[1]-(*region)[0];black.heightRad=(*region)[3]-(*region)[2];black.rangeM=1;black.transmission=1;
            hypothesis.patches.push_back(black);hypothesis.refereeAttributionIndices.emplace_back(std::nullopt);
            const auto outside=ThermalImageReceiverV1::incidentFieldV1(hypothesis,f,p);REQUIRE(outside);
            for(std::size_t i=0;i<expected.size();++i)expected[i]-=outside->radiance[i];
        }
        referenceSeconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-referenceStart).count();
        bool accelerated=false;const auto fastStart=std::chrono::steady_clock::now();
        const auto actual=ThermalReticleReceiverV1::unitGroundFieldV1(p,f,u,cutoff,region,&accelerated);
        fastSeconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-fastStart).count();
        REQUIRE(actual);REQUIRE(actual->size()==expected.size());++cases;if(accelerated)++fastCount;
        INFO("pose="<<pose<<" cutoff="<<cutoff<<" region="<<region.has_value()<<" accelerated="<<accelerated);
        // The retained optical integrator's declared absolute pixel-radiance
        // accuracy is1e-9. No detector/scene acceptance tolerance changes.
        for(std::size_t i=0;i<expected.size();++i) {
            CHECK(std::isfinite((*actual)[i]));
            maximumError=std::max(maximumError,std::abs((*actual)[i]-expected[i]));
            CHECK(std::abs((*actual)[i]-expected[i])<=1e-9);
        }
        if(pose==0)CHECK(accelerated);
    }
    CHECK(cases==27);CHECK(fastCount>0);
    if(const auto* directory=std::getenv("FSIM_THERMAL_RETICLE_REFERENCE_DIR")) {
        const auto root=std::filesystem::path(directory);
        for(const auto& part:root){auto text=part.generic_string();for(auto& ch:text)if(ch>='A'&&ch<='Z')ch=static_cast<char>(ch-'A'+'a');REQUIRE(text!=".codex");}
        std::filesystem::create_directories(root);std::ofstream out(root/"reticle-calibration-comparison.csv");REQUIRE(out.good());
        out<<std::setprecision(17)<<"cases,accelerated_cases,maximum_pixel_error,reference_seconds,accelerated_seconds\n"
            <<cases<<','<<fastCount<<','<<maximumError<<','<<referenceSeconds<<','<<fastSeconds<<'\n';
        out.flush();REQUIRE(out.good());
    }
    CHECK_FALSE(ThermalReticleReceiverV1::unitGroundFieldV1(p,{1,0,0},{0,0,1},std::numeric_limits<double>::quiet_NaN()));
    CHECK_FALSE(ThermalReticleReceiverV1::unitGroundFieldV1(p,{1,0,0},{0,0,1},0,std::array<double,4>{1,0,0,1}));
}

TEST_CASE("Mixed conic endpoint support cannot disappear between interior quadrature nodes", "[thermal-reticle][thermal-reticle-calibration-endpoint]") {
    auto p=reticlePolicy().detector;p.exposureSamples=1;
    const auto forward=unitRay(1,0,.2);
    const fsim::ms::weapon::V3 up{0,0,1},right{0,-1,0},vertical{-forward.z,0,forward.x};
    const double cutoff=forward.z,rightEdge=.5*p.width*p.pixelPitchRad,inside=rightEdge-.00015;
    // Independent native-ray bisection places the rectangle's lower edge on
    // the cone just before the detector's right boundary. The resulting
    // positive ground cap occupies <the last GL8 node gap for EVERY pixel
    // whose source support reaches that boundary. No measured output sets it.
    double lo=-.01,hi=.01;
    for(unsigned iteration=0;iteration<80;++iteration) {
        const double y=.5*(lo+hi);
        const auto ray=unitRay(forward.x+right.x*std::tan(inside)+vertical.x*std::tan(y),
            forward.y+right.y*std::tan(inside)+vertical.y*std::tan(y),
            forward.z+right.z*std::tan(inside)+vertical.z*std::tan(y));
        if(ray.z<cutoff)lo=y;else hi=y;
    }
    const double bottom=.5*(lo+hi),top=bottom+.003;
    const std::array<double,4> support{0,rightEdge,bottom,top};
    ThermalSceneFrameV1 hypothesis;hypothesis.up=up;hypothesis.groundRadianceWm2Sr=1;hypothesis.groundHorizonCosine=cutoff;
    const auto ground=ThermalImageReceiverV1::incidentFieldV1(hypothesis,forward,p);REQUIRE(ground);
    ThermalRadiancePatchV1 black;const double x=.5*rightEdge,y=.5*(bottom+top);
    black.direction=unitRay(forward.x+right.x*std::tan(x)+vertical.x*std::tan(y),
        forward.y+right.y*std::tan(x)+vertical.y*std::tan(y),forward.z+right.z*std::tan(x)+vertical.z*std::tan(y));
    black.widthRad=rightEdge;black.heightRad=top-bottom;black.rangeM=1;black.transmission=1;
    hypothesis.patches.push_back(black);hypothesis.refereeAttributionIndices.emplace_back(std::nullopt);
    const auto outside=ThermalImageReceiverV1::incidentFieldV1(hypothesis,forward,p);REQUIRE(outside);
    const auto index=static_cast<std::size_t>(p.height/2)*p.width+p.width-1;
    const double expected=ground->radiance[index]-outside->radiance[index];
    REQUIRE(expected>1e-9);
    bool accelerated=false;
    const auto actual=ThermalReticleReceiverV1::unitGroundFieldV1(p,forward,up,cutoff,support,&accelerated);REQUIRE(actual);
    INFO("positive endpoint pixel="<<expected<<" actual="<<(*actual)[index]<<" accelerated="<<accelerated);
    CHECK(std::abs((*actual)[index]-expected)<=1e-9);
    CHECK_FALSE(accelerated);
}
#endif
