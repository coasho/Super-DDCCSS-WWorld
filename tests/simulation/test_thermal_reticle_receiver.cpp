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
#include <cstring>
#include <cfenv>
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif
#if __has_include("simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h")
#include "simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h"
#include "../reference/frozen_thermal_image_receiver_v1.h"
#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h"
#include "simulation/mission_systems/components/implementations/seeker/NonImagingIrSeekerV2.h"
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

TEST_CASE("Reticle reports a coarse but finite bearing for a detected source near the stop", "[thermal-reticle][thermal-reticle-edge]") {
    const auto c=reticlePolicy();
    for(const double y:{.0,.01,.02,.03,.04,.045})for(const double width:{.001,.003,.006,.012}) {
        INFO("y="<<y<<" width="<<width);
        ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
        auto f=reticleScene(0,100);REQUIRE(f.patches.size()==1);
        f.patches[0].direction=unitRay(1,y,0);
        f.patches[0].widthRad=f.patches[0].heightRad=width;
        const auto start=receiver.expose(f,{1,0,0},17);REQUIRE(start.valid);
        f.timeS=.02;const auto result=receiver.expose(f,{1,0,0},17);
        REQUIRE(result.valid);REQUIRE(result.complete);
        // The detection gate may legitimately reject a sub-threshold source.
        // A source that clears it must still supply a coarse measured bearing.
        INFO("fitSnr="<<result.fitSnr<<" residual="<<result.fitResidual<<" features="<<result.features.size()
            <<" centre="<<result.fittedCentreXRad<<','<<result.fittedCentreYRad
            <<" support="<<result.fittedWidthRad<<','<<result.fittedHeightRad);
        const auto& inference=result.horizonDiagnostic;
        INFO("horizon refusal="<<inference.refusal<<" nullResidual="<<inference.nullResidual<<" nullCutoff="<<inference.nullCutoff
            <<" sourceAvailable="<<inference.fitAvailable<<" sourceResidual="<<inference.sourceResidual
            <<" sourceShape="<<inference.shape[0]<<','<<inference.shape[1]<<','<<inference.shape[2]<<','<<inference.shape[3]<<','<<inference.shape[4]
            <<" budgetAvailable="<<inference.budgetAvailable<<" skyGroundSource="<<inference.rawBudget[0]<<','<<inference.rawBudget[1]<<','<<inference.rawBudget[2]
            <<" informationAvailable="<<inference.informationAvailable<<" determinant="<<inference.informationDeterminant<<" trace="<<inference.informationTrace
            <<" localVariance="<<inference.localVariance<<" alternativeAvailable="<<inference.alternativeAvailable
            <<" alternativeDelta="<<inference.alternativeDelta<<" alternativeSeparation="<<inference.alternativeSeparation);
        if(result.fitSnr>12) {
            REQUIRE(result.features.size()==1);CHECK(result.bearingResolved);
            // A coarse bearing must remain inside the optical stop; an
            // extended edge source is not required to reproduce its centroid.
            CHECK(std::abs(std::atan2(result.features[0].direction.y,result.features[0].direction.x))<=c.stopRadiusRad);
            CHECK(std::isfinite(result.features[0].angularVarianceRad2));
            CHECK(result.features[0].angularVarianceRad2>0);
        }
    }
}

TEST_CASE("Actual Sidewinder reticle cannot acquire an empty curved Earth horizon", "[thermal-reticle][thermal-reticle-empty-earth]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"aim-9m.json",*cards.at("aim-9m"),profile,error));
    const auto* descriptor=profile.find("seeker");REQUIRE(descriptor);
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*descriptor,scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    const auto& background=configuration.reticle->scene;
    // Independently derive the same physical 6 km Earth limb used by World.
    // There are no target, flare, attribution or precomputed detector samples.
    const double ratio=6371000./6377000.;
    const double horizon=-std::sqrt(1-ratio*ratio);
    for(const double offset:{-.006,0.,.006}) {
        INFO("offset="<<offset);
        ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
        auto scene=reticleScene(0);scene.groundHorizonCosine=horizon;
        scene.skyRadianceWm2Sr=ThermalImageReceiverV1::bandRadiance(background.skyTemperatureK,1,background.lowUm,background.highUm);
        scene.groundRadianceWm2Sr=ThermalImageReceiverV1::bandRadiance(background.groundTemperatureK,1,background.lowUm,background.highUm);
        REQUIRE(scene.patches.empty());REQUIRE(scene.refereeAttributionIndices.empty());
        const double elevation=std::asin(horizon)+offset;
        const auto pointing=unitRay(std::cos(elevation),0,std::sin(elevation));
        REQUIRE(receiver.expose(scene,pointing,17).valid);
        scene.timeS=.02;const auto observed=receiver.expose(scene,pointing,17);
        retainReticleWaveform(offset<0?"reticle-empty-earth-low":offset>0?"reticle-empty-earth-high":"reticle-empty-earth-centre",observed);
        REQUIRE(observed.valid);REQUIRE(observed.complete);CHECK_FALSE(observed.saturated);
        CHECK(observed.features.empty());CHECK_FALSE(observed.bearingResolved);
    }
}

TEST_CASE("A finite stop-edge radiator must explain its scalar waveform before a bearing is emitted", "[thermal-reticle][thermal-reticle-support-fit]") {
    const auto policy=reticlePolicy();ThermalReticleReceiverV1 receiver;std::string error;
    REQUIRE(receiver.configure(policy,error));
    auto scene=reticleScene(0,100);scene.patches[0].direction=unitRay(1,.045,0);
    scene.patches[0].widthRad=scene.patches[0].heightRad=.012;
    REQUIRE(receiver.expose(scene,{1,0,0},17).valid);
    scene.timeS=.02;const auto result=receiver.expose(scene,{1,0,0},17);
    retainReticleWaveform("reticle-wide-edge-support-fit",result);
    REQUIRE(result.valid);REQUIRE(result.complete);CHECK_FALSE(result.saturated);
    REQUIRE(result.features.size()==1);CHECK(result.bearingResolved);
    // An opaque rectangular radiator is inside this inverse's declared model.
    // Use all samples as an upper bound on residual degrees of freedom, so
    // this necessary goodness-of-fit bound does not copy nuisance internals.
    const double n=static_cast<double>(result.measuredElectrons.size());
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    CHECK(result.fitResidual<=n+2*std::sqrt(n*tail)+2*tail);
}

TEST_CASE("Coarse one-edge optical support cannot manufacture angular rate and precise sources retain it", "[thermal-reticle][thermal-reticle-coarse-rate]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;std::string error;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());const auto card=cards.at("aim-9m");
    ProfileV1 profile;REQUIRE(readProfileV1(library/"aim-9m.json",*card,profile,error));
    weapon::Spec spec;REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
    // A diagnostic slow servo keeps the physical one-edge source unresolved
    // while the detector/tracker run. No authored card or acceptance scene is
    // changed; both coarse and precise controls use this identical finite servo.
    spec.seekerModel.engineering.gimbalRateRadS=1e-9;
    const auto policy=reticlePolicy();
    for(const bool coarse:{true,false}) {
        INFO("coarse="<<coarse);
        EngineeringSeekerV3 receiver(spec,std::make_unique<NonImagingIrSeekerV2>(spec));
        REQUIRE(receiver.configureThermalReticleV1(policy,error));
        unsigned tracked=0,rates=0;
        for(unsigned i=0;i<25;++i) {
            SeekerFrameV2 frame;frame.timeS=frame.ageS=i*.01;frame.dtS=.01;frame.receiverSeed=17;
            frame.velocity={250,0,0};frame.boresight={1,0,0};frame.up={0,0,1};
            frame.bodyAxes=std::array<weapon::V3,3>{{{1,0,0},{0,1,0},{0,0,1}}};
            auto optical=reticleScene(frame.timeS,100);
            if(coarse) {
                optical.patches[0].direction=unitRay(1,.045,0);
                optical.patches[0].widthRad=.04;optical.patches[0].heightRad=.003;
            }
            SeekerNavigationPoseV1 navigation;navigation.valid=true;navigation.timeS=frame.timeS;
            navigation.velocity=frame.velocity;navigation.bodyToEcef=*frame.bodyAxes;optical.navigation=navigation;
            const auto observed=receiver.sampleThermalReticleV1(frame,optical);REQUIRE(observed.valid);
            if(const auto* raw=receiver.thermalReticleV1();raw&&raw->complete) {
                UNSCOPED_INFO("coarse="<<coarse<<" read="<<frame.timeS<<" rawFeatures="<<raw->features.size()
                    <<" precise="<<raw->preciseBearing<<" SNR="<<raw->fitSnr<<" residual="<<raw->fitResidual
                    <<" compactSNR="<<raw->compactFitSnr<<" compactResidual="<<raw->compactFitResidual
                    <<" centre="<<raw->fittedCentreXRad<<','<<raw->fittedCentreYRad
                    <<" support="<<raw->fittedWidthRad<<','<<raw->fittedHeightRad);
                if(i==10)retainReticleWaveform(coarse?"reticle-coarse-rate-middle":"reticle-precise-rate-middle",*raw);
            }
            if(!observed.tracking||!observed.track)continue;
            ++tracked;if(observed.track->rateAvailable)++rates;
            if(coarse) {
                CHECK_FALSE(observed.track->rateAvailable);
                CHECK(observed.track->directionRate.x==0);CHECK(observed.track->directionRate.y==0);CHECK(observed.track->directionRate.z==0);
                REQUIRE(observed.angularVarianceRad2);
                CHECK(*observed.angularVarianceRad2>=4*policy.stopRadiusRad*policy.stopRadiusRad);
            }
        }
        REQUIRE(tracked>=3);
        if(!coarse)CHECK(rates>=3);
    }
}

TEST_CASE("Wide signed radiators retain finite support and genuine measured bearing", "[thermal-reticle][thermal-reticle-wide-support]") {
    const auto policy=reticlePolicy();
    for(const double contrast:{-2.5,2.5})for(const double width:{.075,.09}) {
      DYNAMIC_SECTION("contrast="<<contrast<<" width="<<width) {
        INFO("contrast="<<contrast<<" width="<<width);ThermalReticleReceiverV1 receiver;std::string error;
        REQUIRE(receiver.configure(policy,error));
        auto scene=reticleScene(0,1);scene.skyRadianceWm2Sr=scene.groundRadianceWm2Sr=3;
        scene.patches[0].radianceWm2Sr=3+contrast;
        scene.patches[0].widthRad=width;scene.patches[0].heightRad=.003;
        REQUIRE(receiver.expose(scene,{1,0,0},17).valid);
        scene.timeS=.02;const auto result=receiver.expose(scene,{1,0,0},17);
        INFO("SNR="<<result.fitSnr<<" residual="<<result.fitResidual
            <<" compactSNR="<<result.compactFitSnr<<" compactResidual="<<result.compactFitResidual
            <<" centre="<<result.fittedCentreXRad<<','<<result.fittedCentreYRad
            <<" support="<<result.fittedWidthRad<<','<<result.fittedHeightRad);
        retainReticleWaveform(contrast<0?(width<.08?"reticle-wide-negative-075":"reticle-wide-negative-090"):
            (width<.08?"reticle-wide-positive-075":"reticle-wide-positive-090"),result);
        REQUIRE(result.valid);REQUIRE(result.complete);CHECK_FALSE(result.saturated);
        REQUIRE(result.features.size()==1);CHECK(result.bearingResolved);
        const double n=static_cast<double>(result.measuredElectrons.size());
        const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
        CHECK(result.fitResidual<=n+2*std::sqrt(n*tail)+2*tail);
        CHECK(result.features[0].signedContrastElectrons*contrast>0);
        const auto expected=scene.patches[0].direction,actual=result.features[0].direction;
        const double dot=expected.x*actual.x+expected.y*actual.y+expected.z*actual.z;
        const double errorRad=std::acos(std::clamp(dot,-1.,1.));
        CHECK(errorRad<=policy.detector.detectionSigma*std::sqrt(result.features[0].angularVarianceRad2));
      }
    }
}

TEST_CASE("A small physical optical stop cannot certify a hidden source edge from rectangular containment", "[thermal-reticle][thermal-reticle-mask-profile]") {
    auto policy=reticlePolicy();policy.stopRadiusRad=.012;policy.scanRadiusRad=.006;
    ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(policy,error));
    auto scene=reticleScene(0,100);scene.patches[0].direction=unitRay(1,-std::tan(.016),0);
    scene.patches[0].widthRad=.020;scene.patches[0].heightRad=.003;
    // Both edges are inside the detector rectangle, but the outer edge lies
    // beyond stop+scan+the original nine-sigma optical support.
    REQUIRE(.016+.5*.020+9*policy.detector.psfSigmaRad<.5*policy.detector.width*policy.detector.pixelPitchRad);
    REQUIRE(.016+.5*.020>policy.stopRadiusRad+policy.scanRadiusRad+9*policy.detector.psfSigmaRad);
    REQUIRE(receiver.expose(scene,{1,0,0},17).valid);
    scene.timeS=.02;const auto result=receiver.expose(scene,{1,0,0},17);
    INFO("compactResidual="<<result.compactFitResidual<<" centre="<<result.fittedCentreXRad<<','<<result.fittedCentreYRad
        <<" support="<<result.fittedWidthRad<<','<<result.fittedHeightRad);
    REQUIRE(result.valid);REQUIRE(result.complete);CHECK_FALSE(result.saturated);
    REQUIRE(result.features.size()==1);CHECK(result.bearingResolved);
    CHECK_FALSE(result.preciseBearing);
    CHECK(result.features[0].angularVarianceRad2>=4*policy.stopRadiusRad*policy.stopRadiusRad);
}

TEST_CASE("A projected background null cannot require negative raw sky or ground light", "[thermal-reticle][thermal-reticle-physical-null]") {
    // The independent physical equations are sky=s0-k*b and ground=sky+b.
    // These cases include old scalar least-squares optima that violate each
    // boundary, two valid signed contrasts and an empty physical domain.
    struct Case {double skyAtZero,projection,unconstrained,expected;};
    for(const auto c:std::array<Case,6>{{{1,.25,-8,-4./3},{1,.25,8,4},{1,.25,1,1},
                                      {1,.25,-1,-1},{1,1.25,8,.8},{1,-.25,-8,-.8}}}) {
        INFO("s0="<<c.skyAtZero<<" k="<<c.projection<<" unconstrained="<<c.unconstrained);
        const auto admitted=ThermalReticleReceiverV1::physicalBackgroundAmplitudeV1(c.skyAtZero,c.projection,c.unconstrained);
        REQUIRE(admitted);CHECK(*admitted==Catch::Approx(c.expected).margin(1e-12));
        const double sky=c.skyAtZero-c.projection*(*admitted),ground=sky+*admitted;
        CHECK(std::isfinite(sky));CHECK(std::isfinite(ground));CHECK(sky>=0);CHECK(ground>=0);
    }
    CHECK_FALSE(ThermalReticleReceiverV1::physicalBackgroundAmplitudeV1(-1,.25,0));
    CHECK_FALSE(ThermalReticleReceiverV1::physicalBackgroundAmplitudeV1(1,std::numeric_limits<double>::quiet_NaN(),0));
}

TEST_CASE("Finite mirror uncertainty covers local error and mode displacement without authorizing disjoint centres", "[thermal-reticle][thermal-reticle-mode-envelope]") {
    const auto policy=reticlePolicy();
    using Mode=std::array<double,5>;
    const std::vector<Mode> overlapping{{{0,-8e-6,.003,.003,1e-10}},{{0,8e-6,.003,.003,4e-10}}};
    const auto bound=ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(policy,overlapping);
    REQUIRE(bound);
    // A mode can carry local error AND a displaced centre. Taking max(V,D^2)
    // loses their combined error; Minkowski needs (sqrt(V)+D)^2.
    CHECK(*bound>=std::pow(2e-5+16e-6,2));
    CHECK(*bound==Catch::Approx(std::pow(2e-5+16e-6,2)).margin(1e-16));
    CHECK_FALSE(ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(policy,
        {{{-.02,0,.003,.003,1e-10}},{{.02,0,.003,.003,1e-10}}}));
    CHECK_FALSE(ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(policy,
        {{{0,0,.003,.003,0}}}));
    CHECK_FALSE(ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(policy,
        {{{0,0,.003,.003,.01}}}));
    const auto selected=ThermalReticleReceiverV1::finiteMirrorEnvelopeV1(policy,
        {{{0,0,.003,.003,4e-10}}});
    REQUIRE(selected);CHECK(*selected>=4e-10);
}

TEST_CASE("A physical background hypothesis preserves bright sources over a cold nonnegative field", "[thermal-reticle][thermal-reticle-physical-source]") {
    const auto policy=reticlePolicy();
    for(const double background:{.001,.01,1.})for(const double contrast:{1.,10.})for(const double width:{.003,.012}) {
        INFO("background="<<background<<" contrast="<<contrast<<" width="<<width);
        ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(policy,error));
        auto scene=reticleScene(0,1);scene.skyRadianceWm2Sr=scene.groundRadianceWm2Sr=background;
        scene.patches[0].radianceWm2Sr=background+contrast;scene.patches[0].widthRad=width;
        REQUIRE(receiver.expose(scene,{1,0,0},17).valid);
        scene.timeS=.02;const auto result=receiver.expose(scene,{1,0,0},17);
        REQUIRE(result.valid);REQUIRE(result.complete);CHECK_FALSE(result.saturated);
        const auto& inference=result.horizonDiagnostic;
        INFO("horizon refusal="<<inference.refusal<<" nullResidual="<<inference.nullResidual<<" nullCutoff="<<inference.nullCutoff
            <<" sourceAvailable="<<inference.fitAvailable<<" sourceResidual="<<inference.sourceResidual
            <<" sourceShape="<<inference.shape[0]<<','<<inference.shape[1]<<','<<inference.shape[2]<<','<<inference.shape[3]<<','<<inference.shape[4]
            <<" budgetAvailable="<<inference.budgetAvailable<<" skyGroundSource="<<inference.rawBudget[0]<<','<<inference.rawBudget[1]<<','<<inference.rawBudget[2]
            <<" informationAvailable="<<inference.informationAvailable<<" determinant="<<inference.informationDeterminant<<" trace="<<inference.informationTrace
            <<" localVariance="<<inference.localVariance<<" alternativeAvailable="<<inference.alternativeAvailable
            <<" alternativeDelta="<<inference.alternativeDelta<<" alternativeSeparation="<<inference.alternativeSeparation);
        REQUIRE(result.features.size()==1);CHECK(result.bearingResolved);
        const auto expected=scene.patches[0].direction,actual=result.features[0].direction;
        const double dot=expected.x*actual.x+expected.y*actual.y+expected.z*actual.z;
        CHECK(std::acos(std::clamp(dot,-1.,1.))<=policy.detector.detectionSigma*std::sqrt(result.features[0].angularVarianceRad2));
    }
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
        REQUIRE(std::isfinite(sigma));
        if(!(sigma<.1)) {
            const auto* rawA=a->thermalReticleV1();const auto* rawB=b->thermalReticleV1();
            if(rawA)retainReticleWaveform("reticle-navigation-static-failure",*rawA);
            if(rawB)retainReticleWaveform("reticle-navigation-rotating-failure",*rawB);
            const auto diagnostic=[&](const char* name,const ThermalReticleResultV1* raw) {
                if(!raw)return;
                UNSCOPED_INFO(name<<" time="<<static_cast<double>(i)/120<<" capture="<<raw->measurementTimeS<<" precise="<<raw->preciseBearing
                    <<" compactResidual="<<raw->compactFitResidual<<" centre="<<raw->fittedCentreXRad<<','<<raw->fittedCentreYRad
                    <<" support="<<raw->fittedWidthRad<<','<<raw->fittedHeightRad<<" alternativeKind="<<raw->ambiguousAlternativeKind
                    <<" alternate="<<raw->alternativeShape[0]<<','<<raw->alternativeShape[1]<<','<<raw->alternativeShape[2]<<','<<raw->alternativeShape[3]
                    <<" amplitude="<<raw->alternativeAmplitude<<" deltaCost="<<raw->alternativeCostDifference
                    <<" rawBudget="<<raw->alternativeRadianceAvailable<<" sky="<<raw->alternativeSky<<" source="<<raw->alternativeSource);
            };
            diagnostic("static",rawA);diagnostic("rotating",rawB);
        }
        REQUIRE(sigma<.1);
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

TEST_CASE("Captured centered Sidewinder evidence retains an honest bearing across a finite mirror cluster", "[thermal-reticle][thermal-reticle-captured-horizon]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"aim-9m.json",*cards.at("aim-9m"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(configuration.reticle->policy,error));
    // Exact read14.700/capture14.691343 actual ADC charges from the frozen
    // current AIM-9M chain. No expected photons, source geometry or identity
    // is supplied to the shared production inverse.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=14.699999999999958;measured.measurementTimeS=14.691343885247306;
    measured.exposureStartTimeS=14.687573974063;measured.exposureEndTimeS=14.695113796431615;
    measured.varianceBoundElectrons2=5310739571.946104;
    measured.measuredElectrons={
        25751571.75120552,26251602.2706464,27251663.30952817,28001709.088689495,29001770.127571262,30001831.16645303,30751876.945614357,31251907.46505524,
        31751937.984496124,32251968.503937006,32251968.503937006,32001953.244216565,31751937.984496124,31501922.72477568,31001892.205334798,31001892.205334798,
        31001892.205334798,31501922.72477568,32251968.503937006,33502044.80253922,34752121.10114142,36252212.659464076,37502288.958066285,38752365.25666849,
        39752426.295550264,40752487.334432036,41502533.113593355,42252578.89275468,43002624.67191601,43752670.451077335,44252700.97051822,44752731.489959106,
        44752731.489959106,44752731.489959106,44752731.489959106,44252700.97051822,44002685.71079778,43252639.93163645,42752609.41219557,42002563.633034244,
        41252517.85387292,40252456.81499115,39252395.77610938,38002319.477507174,36502227.91918452,35252151.62058231,33752060.06225966,32501983.76365745,
        31751937.984496124,31251907.46505524,31251907.46505524,31251907.46505524,31501922.72477568,32001953.244216565,32251968.503937006,32251968.503937006,
        32251968.503937006,32001953.244216565,31501922.72477568,31001892.205334798,30251846.42617347,29251785.387291703,28251724.348409936,27251663.30952817,
        25501556.491485074,23751449.67344198,21501312.335958004,19501190.25819447,17751083.440151375,16250991.881828725,15250930.842946956,14500885.063785631,
        14000854.544344747,14000854.544344747,14000854.544344747,13750839.284624305,13500824.024903864,13250808.76518342,13000793.50546298,12750778.245742537,
        12500762.986022096,12500762.986022096,12750778.245742537,13000793.50546298,13250808.76518342,13500824.024903864,13500824.024903864,13500824.024903864,
        13500824.024903864,13250808.76518342,13000793.50546298,13000793.50546298,13250808.76518342,13500824.024903864,13750839.284624305,14000854.544344747,
        14250869.804065188,14250869.804065188,14000854.544344747,13750839.284624305,13500824.024903864,13250808.76518342,13000793.50546298,13250808.76518342,
        13250808.76518342,13500824.024903864,13500824.024903864,13750839.284624305,13500824.024903864,13250808.76518342,13000793.50546298,12750778.245742537,
        12500762.986022096,12500762.986022096,12500762.986022096,13000793.50546298,13250808.76518342,13500824.024903864,13750839.284624305,13750839.284624305,
        13500824.024903864,13750839.284624305,14000854.544344747,14750900.323506072,15750961.36238784,17251052.920710493,19001159.738753587,21001281.816517122};
    const fsim::ms::weapon::V3 pointing{-0.0025904725074475735,0.9999887078718503,-0.0039841662836093555},up{0.7660437159805902,0.000685870338001339,0.6427881103353799};
    measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("horizon refusal="<<measured.horizonDiagnostic.refusal<<" null="<<measured.horizonDiagnostic.nullResidual
        <<" residual="<<measured.fitResidual<<" localVariance="<<measured.horizonDiagnostic.localVariance
        <<" aliasDelta="<<measured.horizonDiagnostic.alternativeDelta<<" aliasSeparation="<<measured.horizonDiagnostic.alternativeSeparation);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Exact native reproduction was proved before this repair. A better
    // joint minimum may improve that residual; retain the same upper1e-5
    // tolerance while preserving the unchanged compact proposal checks.
    CHECK(measured.fitResidual<=114.32529754469537+1e-5);
    CHECK(measured.compactFitResidual==Catch::Approx(2742.596821987238).margin(1e-5));
    CHECK(measured.fittedCentreXRad==Catch::Approx(2.4902343750000002e-05).margin(1e-10));
    CHECK(measured.fittedCentreYRad==Catch::Approx(-0.048).margin(1e-10));
    CHECK(measured.fittedWidthRad==Catch::Approx(0.09599999999999999).margin(1e-10));
    CHECK(measured.fittedHeightRad==Catch::Approx(0.016984628677368167).margin(1e-10));
    REQUIRE_FALSE(measured.features.empty());
    CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precision is deliberately not required: a genuine ambiguous return may
    // remain coarse while preserving its measured existence and uncertainty.
}


TEST_CASE("Captured source-free Magic horizon cannot become a false strip bearing", "[thermal-reticle][thermal-reticle-captured-background]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"magic-2.json",*cards.at("magic-2"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(configuration.reticle->policy,error));
    // Frozen actual raw capture46.924084511: the only opponent is145deg
    // outside the optical field; launcher is excluded by the production
    // candidate filter and no decoys exist. No truth enters this inverse.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=46.93333333333323;measured.measurementTimeS=46.92408451107858;
    measured.exposureStartTimeS=46.920314599894276;measured.exposureEndTimeS=46.92785442226289;
    measured.varianceBoundElectrons2=5955641984.164849;
    measured.measuredElectrons={
        273516694.13416344,274516755.1730452,276266861.99108833,278516999.3285723,280767136.6660563,282767258.74381983,284767380.82158333,286517487.63962644,
        288017579.1979491,289517670.7562717,291267777.57431483,292767869.1326375,294767991.210401,296518098.0284441,298268204.8464872,300018311.6645303,
        301768418.4825734,303768540.56033695,305518647.37838,307518769.45614356,309768906.79362756,312269059.39083195,314519196.72831595,317269364.5852408,
        319769517.1824452,322019654.5199292,324019776.5976927,326019898.6754563,327519990.2337789,328770066.5323811,329770127.5712629,331020203.8698651,
        331770249.6490264,331520234.38930595,331020203.8698651,330270158.0907038,329270097.051822,328270036.0129402,326769944.45461756,325019837.6365745,
        323019715.55881095,320519562.96160656,318019410.3644021,315769273.0269181,313269120.4297137,310768967.8325093,308768845.7547458,306768723.6769822,
        304768601.59921867,302768479.52145517,301018372.70341206,299268265.88536894,297268143.80760545,295518036.98956233,293767930.1715192,292017823.35347617,
        290517731.7951535,289017640.23683083,287517548.6785082,285517426.60074466,283517304.5229811,281267167.18549716,279017029.84801316,276516877.2508088,
        273516694.13416344,270266495.7577977,266516266.86199108,263016053.22590488,259265824.33009827,255515595.43429163,252015381.79820544,248515168.16211927,
        245014954.52603307,241764756.14966732,238764573.03302202,235764389.91637674,233014222.05945188,230514069.46224743,228263932.12476346,226513825.30672038,
        225263749.00811815,224263687.9692364,224013672.70951596,224013672.70951596,224263687.9692364,224513703.22895685,225013733.74839774,225513764.2678386,
        226013794.7872795,226263810.04699993,226513825.30672038,226263810.04699993,226263810.04699993,226013794.7872795,226013794.7872795,226263810.04699993,
        226263810.04699993,226263810.04699993,226013794.7872795,226013794.7872795,226263810.04699993,226263810.04699993,226513825.30672038,226513825.30672038,
        226263810.04699993,226013794.7872795,225513764.2678386,225263749.00811815,224513703.22895685,224013672.70951596,223763657.4497955,223763657.4497955,
        224013672.70951596,224763718.4886773,225763779.52755904,227513886.34560215,229764023.68308613,232014161.0205701,234514313.61777452,237264481.47469938,
        240264664.59134468,243514862.96771044,246765061.34407616,250265274.98016235,253765488.61624855,257515717.51205516,261265946.4078618,265266190.56338888};
    const fsim::ms::weapon::V3 pointing{-0.4136531586021842,0.8266783909544756,0.3814366294777622},up{0.76604767460494,0.003855967431650216,0.6427721927304649};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("horizon refusal="<<measured.horizonDiagnostic.refusal<<" null="<<measured.horizonDiagnostic.nullResidual
        <<" nullCutoff="<<measured.horizonDiagnostic.nullCutoff<<" compact="<<measured.compactFitResidual);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    CHECK(measured.compactFitResidual==Catch::Approx(122.52975954025439).margin(1e-5));
    REQUIRE(measured.features.empty());
    CHECK_FALSE(measured.bearingResolved);CHECK_FALSE(measured.preciseBearing);
}


TEST_CASE("Captured Magic source and unknown horizon must meet the original joint residual gate", "[thermal-reticle][thermal-reticle-captured-joint-fit]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"magic-2.json",*cards.at("magic-2"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Original current native residual4691.930328 at capture14.857219 is
    // diagnostic evidence, not the acceptance standard. Independent bounded
    // fits from measured null minima reach112.512684; no target pose is used.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=14.866666666666625;measured.measurementTimeS=14.857219977356849;
    measured.exposureStartTimeS=14.853450066172542;measured.exposureEndTimeS=14.860989888541157;
    measured.varianceBoundElectrons2=5511303122.949204;
    measured.measuredElectrons={
        90505524.01879998,91255569.7979613,92255630.83684307,93255691.87572484,94755783.43404749,96255874.99237014,98255997.07013367,100506134.40761764,
        102756271.74510163,105006409.0825856,106756515.9006287,108006592.19923091,108506622.71867178,108506622.71867178,108506622.71867178,108256607.45895135,
        108506622.71867178,109256668.49783312,110506744.79643533,112006836.35475798,114006958.4325215,115757065.2505646,117257156.80888726,118757248.36720991,
        120007324.66581212,121507416.22413477,123257523.04217787,125007629.86022095,127007751.9379845,129257889.27546847,131258011.35323201,133008118.1712751,
        134008179.21015687,134008179.21015687,133008118.1712751,131508026.61295244,129757919.79490936,127757797.71714582,125757675.63938229,124007568.82133919,
        122257462.00329609,121007385.70469388,119507294.14637123,118007202.58804858,116257095.7700055,114506988.9519624,112506866.87419887,111006775.31587622,
        109756699.017274,109006653.23811267,108756637.97839223,109006653.23811267,109256668.49783312,109256668.49783312,108506622.71867178,107256546.42006958,
        105506439.60202649,103256302.2645425,101006164.92705853,99006042.849295,97005920.77153146,95505829.21320881,94005737.65488616,92505646.0965635,
        90005493.49935909,86505279.8632729,82255020.44802539,77504730.51333699,72754440.5786486,68504181.16340108,65003967.5273149,62003784.410669595,
        59503631.81346518,57003479.21626075,54003296.099615455,50753097.72324971,46752853.56772264,42502594.152475126,38252334.73722761,34502105.841420986,
        31501922.72477568,29751815.90673259,28501739.608130377,28001709.088689495,27751693.828969054,27251663.30952817,26501617.53036684,25501556.491485074,
        24251480.192882866,23251419.154001098,22501373.37483977,22251358.11511933,22251358.11511933,22751388.634560212,23251419.154001098,24001464.933162425,
        24251480.192882866,24251480.192882866,23751449.67344198,23001403.894280657,22501373.37483977,22001342.85539889,22001342.85539889,22501373.37483977,
        23251419.154001098,24251480.192882866,25251541.231764633,26251602.2706464,26501617.53036684,26751632.790087286,27001648.049807727,27501678.56924861,
        28501739.608130377,30501861.685893912,33502044.80253922,37252273.69834585,41252517.85387292,45502777.269120425,49253006.16492706,52253189.28157236,
        55003357.13849722,57503509.73570164,60003662.33290606,63003845.44955136,66754074.34535799,71254349.02032594,76004638.95501435,80754928.88970274};
    const fsim::ms::weapon::V3 pointing{-0.01352449169656142,0.9998141198279883,-0.013740957642558808},up{0.76604367199441,0.0008084043506342989,0.6427880203299736};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("originalNativeResidual=4691.930328421221 replay="<<measured.fitResidual
        <<" horizon refusal="<<measured.horizonDiagnostic.refusal<<" null="<<measured.horizonDiagnostic.nullResidual
        <<" nullCutoff="<<measured.horizonDiagnostic.nullCutoff);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    CHECK(measured.compactFitResidual==Catch::Approx(22029.907771119426).margin(1e-5));
    // Same128phase/7nuisance/7jointparameter Laurent-Massart expression
    // as production. No new source admission threshold is introduced.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-7;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double originalBound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(measured.fitResidual<=originalBound);
    REQUIRE_FALSE(measured.features.empty());
    CHECK(measured.bearingResolved);CHECK(std::isfinite(measured.features.front().angularVarianceRad2));
    CHECK(measured.features.front().angularVarianceRad2>0);
}

TEST_CASE("Captured PL-8 second centered loss retains measured source evidence", "[thermal-reticle][thermal-reticle-captured-current-fit]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"pl-8.json",*cards.at("pl-8"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent measured-only bounded fit reaches 196.267753254445;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.699999999999955;measured.measurementTimeS=15.69414026027317;
    measured.exposureStartTimeS=15.690370349088862;measured.exposureEndTimeS=15.697910171457478;
    measured.varianceBoundElectrons2=5693873793.850993;
    measured.measuredElectrons={
        157759628.88359883,157759628.88359883,157509613.62387842,156759567.8447171,156009522.06555575,155509491.54611486,156259537.3252762,158259659.40303972,
        162009888.29884636,166760178.23353475,171760483.4279436,176010742.84319112,179010925.95983642,181011048.03759995,182011109.0764817,182761154.85564303,
        183761215.8945248,185011292.193127,186761399.01117012,188761521.08893365,191261673.68613806,193761826.28334248,196261978.8805469,198762131.4777513,
        201512299.33467618,204262467.19160104,206512604.529085,208762741.866569,210762863.94433254,212012940.24293473,213263016.54153696,214513092.84013915,
        215263138.61930048,215013123.35958004,214263077.5804187,213763047.06097785,212762986.02209607,211262894.46377343,209512787.64573032,207512665.5679668,
        205262528.23048282,202512360.37355796,200012207.77635354,197262039.91942868,194761887.32222426,192011719.4652994,189761582.12781543,187511444.79033142,
        186011353.23200879,184761276.93340656,184011231.15424526,183511200.63480437,182511139.5959226,180261002.25843862,177010803.88207287,172260513.94738448,
        167260208.75297564,162759934.0780077,159259720.4419215,157509613.62387842,157009583.10443753,157259598.36415797,157759628.88359883,157759628.88359883,
        156009522.06555575,152009277.9100287,146508942.19617897,140008545.44344747,133758163.95043643,128007812.97686626,123257523.04217787,120007324.66581212,
        117507172.0686077,115007019.47140329,112256851.61447842,109006653.23811267,104756393.82286516,100256119.14789721,95755844.47292925,91505585.05768174,
        88255386.681316,85255203.5646707,83005066.22718671,81004944.14942318,79004822.07165964,77004699.99389611,75254593.17585301,73754501.61753036,
        72254410.05920771,70754318.50088505,69504242.20228285,68254165.90368064,67004089.60507844,65754013.30647622,64753952.26759446,64003906.48843313,
        63753891.228712685,63503875.96899225,64003906.48843313,64753952.26759446,65503998.04675578,66504059.08563755,67754135.38423976,68754196.42312153,
        70004272.72172374,71504364.28004639,73004455.83836904,74504547.3966917,76254654.21473478,77754745.77305743,79504852.59110053,81504974.66886407,
        83755112.00634804,86505279.8632729,90005493.49935909,94505768.17432705,99006042.849295,103506317.52426295,107256546.42006958,110506744.79643533,
        113006897.39363974,115007019.47140329,117507172.0686077,121007385.70469388,126007690.89910272,131758041.87267289,138508453.88512483,145258865.89757675};
    const fsim::ms::weapon::V3 pointing{-0.023391991768011017,0.9994488824987582,-0.02355304636578861},up{0.7660436378547169,0.0008497081209197034,0.6427880077431601};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Original joint Gaussian residual admission bound, without tolerance.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-7;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(measured.horizonDiagnostic.fitAvailable);
    CHECK(measured.horizonDiagnostic.sourceResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


TEST_CASE("Captured PL-8 third centered loss retains measured source evidence", "[thermal-reticle][thermal-reticle-captured-current-fit]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"pl-8.json",*cards.at("pl-8"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent measured-only bounded fit reaches 184.263303165205;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.733333333333288;measured.measurementTimeS=15.72429954974763;
    measured.exposureStartTimeS=15.720529638563324;measured.exposureEndTimeS=15.72806946093194;
    measured.varianceBoundElectrons2=5706232162.288309;
    measured.measuredElectrons={
        163009949.33772814,163259964.59744856,162759934.0780077,161759873.03912592,160759812.00024414,160259781.48080328,161009827.2599646,163759995.11688945,
        168260269.79185742,174010620.76542756,179260941.21955684,183261185.37508392,186011353.23200879,187261429.530611,187761460.05005187,188011475.3097723,
        188511505.8292132,189511566.86809498,191011658.42641762,193011780.50418115,195261917.84166515,197762070.43886957,200512238.29579443,203762436.67216015,
        206762619.78880545,209762802.90545076,212762986.02209607,215013123.35958004,217013245.43734357,218263321.7359458,219263382.77482757,220263443.81370932,
        220763474.3331502,220513459.07342976,220013428.55398887,219513398.034548,219013367.51510713,217763291.2165049,216013184.39846182,213763047.06097785,
        211012879.20405298,208012696.08740768,204762497.71104193,201762314.59439662,199012146.73747176,196261978.8805469,193761826.28334248,191761704.20557895,
        190261612.64725628,189261551.60837454,189011536.3486541,189011536.3486541,188761521.08893365,187261429.530611,184511261.67368615,179760971.73899773,
        174260636.025148,168760300.31129828,164760056.15577123,162509918.81828725,161759873.03912592,162259903.5585668,162759934.0780077,162759934.0780077,
        160759812.00024414,156509552.58499664,151009216.87114692,144758835.37813586,138508453.88512483,133008118.1712751,129007874.01574802,126007690.89910272,
        123507538.3018983,121007385.70469388,117757187.32832815,114256973.69224195,109756699.017274,105256424.34230605,100506134.40761764,96505890.25209057,
        93005676.61600439,90005493.49935909,87505340.90215467,85255203.5646707,83255081.48690715,81504974.66886407,79754867.85082097,78504791.55221877,
        77254715.25361656,76004638.95501435,74754562.65641214,73504486.35780992,72254410.05920771,70754318.50088505,69504242.20228285,68754196.42312153,
        68254165.90368064,68254165.90368064,68754196.42312153,69754257.46200329,70754318.50088505,72004394.79948728,73004455.83836904,74004516.8772508,
        75254593.17585301,76254654.21473478,77504730.51333699,79004822.07165964,80504913.6299823,82255020.44802539,84005127.26606849,86005249.34383202,
        88505401.94103643,91255569.7979613,95005798.69376792,99256058.10901543,104006348.04370384,108506622.71867178,112756882.1339193,116007080.51028505,
        118757248.36720991,121007385.70469388,123257523.04217787,126507721.4185436,131007996.09351157,136508331.80736127,143008728.56009278,149759140.5725447};
    const fsim::ms::weapon::V3 pointing{-0.024160821563294577,0.9994126993820153,-0.024300432410560322},up{0.7660436394934843,0.0008533667696848084,0.6427880009433382};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Original joint Gaussian residual admission bound, without tolerance.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-7;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(measured.horizonDiagnostic.fitAvailable);
    CHECK(measured.horizonDiagnostic.sourceResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


TEST_CASE("Captured R-60M first centered loss retains measured source evidence", "[thermal-reticle][thermal-reticle-captured-current-fit]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"r-60m.json",*cards.at("r-60m"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent measured-only bounded fit reaches 119.982800940352;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.699999999999955;measured.measurementTimeS=15.69414026027317;
    measured.exposureStartTimeS=15.690370349088862;measured.exposureEndTimeS=15.697910171457478;
    measured.varianceBoundElectrons2=5466361113.782667;
    measured.measuredElectrons={
        73754501.61753036,74504547.3966917,76004638.95501435,77754745.77305743,79504852.59110053,80754928.88970274,82005005.18830495,82505035.70774584,
        82755050.96746628,82505035.70774584,82255020.44802539,82255020.44802539,82255020.44802539,82755050.96746628,84005127.26606849,85505218.82439114,
        87755356.16187511,90255508.75907953,93005676.61600439,95755844.47292925,98756027.58957456,101256180.18677898,103256302.2645425,104756393.82286516,
        105506439.60202649,105506439.60202649,105256424.34230605,105506439.60202649,106006470.12146737,107506561.67979002,109256668.49783312,111506805.83531709,
        113256912.65336019,114006958.4325215,113256912.65336019,112006836.35475798,110006714.27699444,108006592.19923091,106006470.12146737,104756393.82286516,
        104256363.30342428,104006348.04370384,104006348.04370384,103256302.2645425,102256241.22566074,100256119.14789721,97755966.55069278,95005798.69376792,
        91755600.31740218,89005432.46047732,86505279.8632729,84505157.78550936,83005066.22718671,82255020.44802539,82255020.44802539,82255020.44802539,
        82755050.96746628,83005066.22718671,83005066.22718671,82505035.70774584,81504974.66886407,80004883.11054142,78004761.03277788,75754623.6952939,
        73504486.35780992,71254349.02032594,69004211.68284197,66754074.34535799,64753952.26759446,62753830.18983092,60503692.85234694,58253555.51486297,
        56003418.17737899,53503265.58017457,51253128.24269059,49253006.16492706,47252884.08716352,45252762.00939999,43502655.19135689,41752548.3733138,
        40002441.55527071,38752365.25666849,37502288.958066285,36502227.91918452,35502166.88030275,34252090.58170054,32501983.76365745,30501861.685893912,
        28501739.608130377,26501617.53036684,25501556.491485074,25251541.231764633,25751571.75120552,26501617.53036684,27501678.56924861,28001709.088689495,
        27751693.828969054,26751632.790087286,25501556.491485074,24501495.452603307,23751449.67344198,24001464.933162425,25001525.972044192,26751632.790087286,
        28501739.608130377,30001831.16645303,31501922.72477568,32501983.76365745,33252029.542818774,33752060.06225966,34752121.10114142,36002197.39974364,
        37502288.958066285,39252395.77610938,41002502.59415247,43002624.67191601,44752731.489959106,46752853.56772264,48502960.38576573,50753097.72324971,
        52753219.801013246,54753341.87877678,57003479.21626075,59253616.55374473,61253738.63150827,63503875.96899225,65754013.30647622,68254165.90368064};
    const fsim::ms::weapon::V3 pointing{-0.019980189173994516,0.9996176529576662,-0.019113867635251972},up{0.7660437387575071,0.0008554378369320834,0.6427878798923695};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Original joint Gaussian residual admission bound, without tolerance.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-7;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(measured.horizonDiagnostic.fitAvailable);
    CHECK(measured.horizonDiagnostic.sourceResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


TEST_CASE("Captured R-60M second centered loss retains measured source evidence", "[thermal-reticle][thermal-reticle-captured-current-fit]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"r-60m.json",*cards.at("r-60m"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent measured-only bounded fit reaches 88.919160023470;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.733333333333288;measured.measurementTimeS=15.72429954974763;
    measured.exposureStartTimeS=15.720529638563324;measured.exposureEndTimeS=15.72806946093194;
    measured.varianceBoundElectrons2=5471417131.832117;
    measured.measuredElectrons={
        76754684.73417567,77504730.51333699,79004822.07165964,80754928.88970274,82505035.70774584,84005127.26606849,84755173.04522981,85255203.5646707,
        85255203.5646707,85005188.30495025,85005188.30495025,85005188.30495025,85755234.08411157,87005310.38271378,88755417.20075688,90755539.27852042,
        93005676.61600439,95255813.95348836,97505951.29097235,100006103.88817677,102256241.22566074,104506378.56314471,106256485.38118781,107506561.67979002,
        108006592.19923091,108256607.45895135,108006592.19923091,108006592.19923091,108506622.71867178,109506683.75755356,111256790.57559665,113506927.91308063,
        115507049.99084416,116257095.7700055,116007080.51028505,114757004.21168284,112756882.1339193,110756760.05615577,108756637.97839223,107256546.42006958,
        106506500.64090826,106256485.38118781,106256485.38118781,106006470.12146737,105256424.34230605,103756332.7839834,102006225.9659403,99756088.62845632,
        97255936.0312519,94505768.17432705,91755600.31740218,89255447.72019777,87005310.38271378,85755234.08411157,85005188.30495025,84755173.04522981,
        85255203.5646707,85755234.08411157,86005249.34383202,85755234.08411157,84755173.04522981,83005066.22718671,81004944.14942318,79004822.07165964,
        76754684.73417567,74254532.13697125,72004394.79948728,70004272.72172374,68254165.90368064,66254043.82591711,64003906.48843313,61503753.89122871,
        59003601.29402429,56503448.69681987,54003296.099615455,51753158.762131475,49753036.68436794,48002929.86632485,46252823.04828175,44752731.489959106,
        43252639.93163645,42002563.633034244,40752487.334432036,39752426.295550264,38752365.25666849,37252273.69834585,35502166.88030275,33252029.542818774,
        31251907.46505524,29501800.647012144,28751754.86785082,28501739.608130377,29001770.127571262,29751815.90673259,30501861.685893912,30751876.945614357,
        30251846.42617347,29251785.387291703,28001709.088689495,27001648.049807727,26501617.53036684,27001648.049807727,28251724.348409936,29751815.90673259,
        31751937.984496124,33252029.542818774,34502105.841420986,35502166.88030275,36252212.659464076,37002258.4386254,37752304.21778673,39002380.51638894,
        40502472.07471159,42002563.633034244,43752670.451077335,45502777.269120425,47502899.34688396,49253006.16492706,51253128.24269059,53503265.58017457,
        55753402.917658545,58003540.255142525,60253677.592626505,62503814.93011048,64503937.00787401,66754074.34535799,69004211.68284197,71254349.02032594};
    const fsim::ms::weapon::V3 pointing{-0.020972890911853555,0.9995844115574198,-0.019777310691953338},up{0.7660437361496044,0.0008591687630669203,0.6427878780243073};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Original joint Gaussian residual admission bound, without tolerance.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-7;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(measured.horizonDiagnostic.fitAvailable);
    CHECK(measured.horizonDiagnostic.sourceResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


TEST_CASE("Captured PL-8 first moving exposure requires time-resolved source inference", "[thermal-reticle][thermal-reticle-captured-moving-v2]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"pl-8.json",*cards.at("pl-8"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent time-resolved measured-only bounded fit reaches 87.401989949429;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.766666666666621;measured.measurementTimeS=15.76199866159071;
    measured.exposureStartTimeS=15.758228750406401;measured.exposureEndTimeS=15.765768572775018;
    measured.varianceBoundElectrons2=5727016633.087801;
    measured.measuredElectrons={
        169510346.09045961,169510346.09045961,169260330.83073917,168260269.79185742,167760239.27241653,168010254.53213698,169760361.35018006,173760605.50570714,
        179010925.95983642,184511261.67368615,189261551.60837454,192261734.72501984,194011841.54306293,194511872.0625038,194511872.0625038,194511872.0625038,
        195011902.5819447,196011963.62082645,197512055.17914912,199512177.25691265,202012329.85411707,204762497.71104193,207762680.82768723,211512909.72349387,
        215263138.61930048,218763352.25538668,222013550.63175243,224763718.4886773,226763840.56644082,228013916.865043,228763962.64420435,229514008.42336568,
        230014038.94280657,229764023.68308613,229263993.16364524,229013977.9039248,228513947.3844839,227513886.34560215,225763779.52755904,223263626.93035462,
        220013428.55398887,216513214.9179027,212762986.02209607,209262772.38600987,206012574.00964415,203012390.89299884,200512238.29579443,198262100.95831043,
        196762009.3999878,195761948.361106,195511933.10138556,195511933.10138556,195761948.361106,195261917.84166515,193511811.02362204,190011597.38753584,
        185011292.193127,179260941.21955684,174510651.28486845,171260452.9085027,169760361.35018006,169260330.83073917,169510346.09045961,169510346.09045961,
        167260208.75297564,163009949.33772814,157509613.62387842,151509247.3905878,145758896.41701764,140758591.2226088,137008362.32680216,134258194.4698773,
        131758041.87267289,129257889.27546847,125757675.63938229,122007446.74357565,117507172.0686077,112756882.1339193,108006592.19923091,104006348.04370384,
        100256119.14789721,97005920.77153146,94255752.9146066,91755600.31740218,89505462.97991821,87755356.16187511,86255264.60355246,85005188.30495025,
        84255142.52578892,83255081.48690715,82255020.44802539,81004944.14942318,79504852.59110053,77754745.77305743,76504669.47445522,75504608.43557346,
        75004577.91613257,75004577.91613257,75504608.43557346,76754684.73417567,78004761.03277788,79254837.33138008,80504913.6299823,81504974.66886407,
        82255020.44802539,83255081.48690715,84005127.26606849,85255203.5646707,86755295.12299335,88505401.94103643,90505524.01879998,92755661.35628395,
        95505829.21320881,98506012.32985412,102256241.22566074,106506500.64090826,111256790.57559665,116007080.51028505,120507355.18525301,124257584.08105963,
        127257767.19770494,129507904.53518891,131508026.61295244,134508209.72959775,138508453.88512483,143758774.3392541,149759140.5725447,156259537.3252762};
    const fsim::ms::weapon::V3 pointing{-0.025356503886003866,0.9993557752009503,-0.02539847009539433},up{0.7660436417408655,0.0008579363646490414,0.642787992182171};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Versioned moving-family Gaussian residual bound:7 nuisance and9 source parameters.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-9;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(std::isfinite(measured.fitResidual));
    CHECK(measured.fitResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


TEST_CASE("Captured PL-8 second moving exposure requires time-resolved source inference", "[thermal-reticle][thermal-reticle-captured-moving-v2]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"pl-8.json",*cards.at("pl-8"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent time-resolved measured-only bounded fit reaches 115.919984668449;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.799999999999955;measured.measurementTimeS=15.792157951065171;
    measured.exposureStartTimeS=15.788388039880864;measured.exposureEndTimeS=15.79592786224948;
    measured.varianceBoundElectrons2=5752856690.326936;
    measured.measuredElectrons={
        175510712.32375023,175760727.58347067,175510712.32375023,175510712.32375023,176260758.10291156,178510895.44039553,181761093.81676129,186261368.49172923,
        191261673.68613806,195511933.10138556,198512116.21803087,200262223.03607398,201262284.07495573,201512299.33467618,201512299.33467618,201762314.59439662,
        202762375.6332784,204012451.9318806,205762558.7499237,208012696.08740768,210512848.6846121,213513031.8012574,217263260.697064,221513520.11231154,
        225763779.52755904,230014038.94280657,233514252.57889274,236514435.69553804,238514557.77330157,239514618.81218335,240264664.59134468,241014710.37050602,
        241514740.88994688,241264725.63022643,241014710.37050602,240764695.11078557,240264664.59134468,239514618.81218335,237514496.73441982,234764328.87749496,
        231264115.24140877,227263871.0858817,223013611.67063418,218763352.25538668,215013123.35958004,212012940.24293473,209012757.12628943,206762619.78880545,
        204762497.71104193,203512421.4124397,202512360.37355796,202512360.37355796,202512360.37355796,202262345.1138375,201512299.33467618,199512177.25691265,
        196011963.62082645,191761704.20557895,187011414.27089056,183011170.11536348,179760971.73899773,178010864.92095464,177260819.1417933,176260758.10291156,
        174260636.025148,170510407.1293414,165510101.93493256,159759750.9613624,154009399.98779222,149009094.7933834,145258865.89757675,142758713.30037233,
        140508575.96288836,138008423.36568394,134758224.9893182,131007996.09351157,126507721.4185436,122007446.74357565,117257156.80888726,113006897.39363974,
        109006653.23811267,105256424.34230605,102006225.9659403,99006042.849295,96755905.51181102,94755783.43404749,93255691.87572484,92505646.0965635,
        91755600.31740218,91005554.53824085,90255508.75907953,89005432.46047732,87505340.90215467,85505218.82439114,84005127.26606849,82755050.96746628,
        82005005.18830495,82005005.18830495,82755050.96746628,84255142.52578892,85755234.08411157,87255325.64243422,88505401.94103643,89505462.97991821,
        90255508.75907953,90755539.27852042,91255569.7979613,92255630.83684307,93755722.39516571,95755844.47292925,98005981.81041323,100756149.66733809,
        103756332.7839834,107256546.42006958,111256790.57559665,115757065.2505646,120507355.18525301,125257645.1199414,129507904.53518891,133258133.43099554,
        136008301.28792042,138008423.36568394,140258560.70316792,143008728.56009278,147258987.97534028,152259293.16974914,158009644.14331928,164010010.3766099};
    const fsim::ms::weapon::V3 pointing{-0.02659755179205417,0.9992892836297561,-0.02671138074674657},up{0.7660436437192948,0.0008615886592105429,0.6427879849392399};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Versioned moving-family Gaussian residual bound:7 nuisance and9 source parameters.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-9;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(std::isfinite(measured.fitResidual));
    CHECK(measured.fitResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


TEST_CASE("Captured PL-8 moving exposure preserves time-varying ground occultation", "[thermal-reticle][thermal-reticle-captured-moving-v2]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"pl-8.json",*cards.at("pl-8"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    // Actual current native ADC capture; pose is the native inference pose.
    // Independent time-resolved measured-only bounded fit reaches 102.363182061227;
    // expected photons, target geometry and referee identity are excluded.
    ThermalReticleResultV1 measured;measured.valid=true;measured.complete=true;
    measured.timeS=15.833333333333288;measured.measurementTimeS=15.822317240539633;
    measured.exposureStartTimeS=15.818547329355326;measured.exposureEndTimeS=15.826087151723941;
    measured.varianceBoundElectrons2=5778696648.539973;
    measured.measuredElectrons={
        181761093.81676129,182511139.5959226,183511200.63480437,185261307.45284745,187761460.05005187,190761643.16669717,194011841.54306293,197012024.65970823,
        200262223.03607398,203262406.1527193,205262528.23048282,206762619.78880545,207762680.82768723,208512726.60684857,209012757.12628943,210012818.1651712,
        211262894.46377343,213013001.2818165,214763108.0998596,217013245.43734357,219763413.29426843,223013611.67063418,227263871.0858817,232014161.0205701,
        236764450.9552585,241514740.88994688,245514985.04547396,248515168.16211927,250515290.2398828,251515351.27876458,252265397.05792588,252765427.57736677,
        253015442.8370872,253015442.8370872,252765427.57736677,252765427.57736677,252515412.31764632,251765366.53848502,249765244.46072146,246765061.34407616,
        242764817.1885491,238264542.51358116,233264237.3191723,228763962.64420435,224763718.4886773,221263504.8525911,218263321.7359458,215763169.13874137,
        213763047.06097785,212012940.24293473,210762863.94433254,210012818.1651712,209262772.38600987,208512726.60684857,207762680.82768723,206012574.00964415,
        203762436.67216015,201012268.8152353,198262100.95831043,195011902.5819447,191761704.20557895,188761521.08893365,186511383.75144967,184261246.4139657,
        181261063.2973204,177260819.1417933,172260513.94738448,166760178.23353475,161509857.77940547,156759567.8447171,153259354.2086309,150509186.35170603,
        148509064.2739425,146258926.93645853,143508759.07953367,140008545.44344747,135758286.02819997,131258011.35323201,126507721.4185436,122007446.74357565,
        117757187.32832815,113756943.17280108,110006714.27699444,106506500.64090826,103756332.7839834,101756210.70621985,100506134.40761764,100006103.88817677,
        99506073.36873588,99006042.849295,98255997.07013367,96755905.51181102,95005798.69376792,93005676.61600439,91005554.53824085,89505462.97991821,
        88505401.94103643,88755417.20075688,89505462.97991821,91005554.53824085,93005676.61600439,94755783.43404749,96255874.99237014,97505951.29097235,
        98005981.81041323,98506012.32985412,98756027.58957456,99506073.36873588,100756149.66733809,102756271.74510163,105256424.34230605,108506622.71867178,
        112006836.35475798,115757065.2505646,120007324.66581212,124757614.60050051,129507904.53518891,134008179.21015687,138258438.6254044,141758652.26149055,
        144258804.85869497,146258926.93645853,148009033.7545016,151009216.87114692,154759445.76695356,159509735.70164195,164760056.15577123,170260391.86962095};
    const fsim::ms::weapon::V3 pointing{-0.027745243155399297,0.9992249983220609,-0.027921393420192547},up{0.7660436458447704,0.0008652381603665867,0.6427879775040741};measured.pointing=pointing;
    const auto original=measured.measuredElectrons;
    receiver.inferMeasuredV1(measured,pointing,up);
    INFO("capture="<<measured.measurementTimeS<<" refusal="<<measured.horizonDiagnostic.refusal
        <<" null="<<measured.horizonDiagnostic.nullResidual<<" source="<<measured.horizonDiagnostic.sourceResidual
        <<" compact="<<measured.compactFitResidual<<" fit="<<measured.horizonDiagnostic.fitAvailable
        <<" budget="<<measured.horizonDiagnostic.budgetAvailable<<" local="<<measured.horizonDiagnostic.localVariance);
    CHECK(measured.measuredElectrons==original);CHECK(receiver.latest()==nullptr);
    // Versioned moving-family Gaussian residual bound:7 nuisance and9 source parameters.
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-9;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(std::isfinite(measured.fitResidual));
    CHECK(measured.fitResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
    const auto& feature=measured.features.front();
    CHECK(std::isfinite(feature.angularVarianceRad2));CHECK(feature.angularVarianceRad2>0);
    CHECK(std::isfinite(feature.direction.x));CHECK(std::isfinite(feature.direction.y));CHECK(std::isfinite(feature.direction.z));
    // Precise bearing/rate is deliberately not required by this existence test.
}


#include "simulation/mission_systems/components/implementations/seeker/MovingRadiatorInverseV2.h"

TEST_CASE("Moving endpoint reconstruction preserves canonical rays and renderer rate domains", "[thermal-reticle][thermal-reticle-moving-v2-domain]") {
    using namespace fsim::ms::components::moving_radiator_v2;
    const std::array<std::array<double,2>,4> intervals{{{{-.99,-.001}},{{-.8,-.2}},{{-.7,.3}},{{.2,.8}}}};
    for(const double scale:{.001,.03,1.})for(const auto offsets:intervals)
    for(const double angle:{0.,.02,.4})for(const double speed:{0.,1e-12,1e-6,.01,.49,.5}) {
        const double a=offsets[0]*scale,b=offsets[1]*scale;
        const auto ray=[&](double t){return unitRay(std::cos(angle)-std::sin(angle)*speed*t/scale,std::sin(angle)+std::cos(angle)*speed*t/scale,0);};
        const auto first=ray(a),last=ray(b);bool recovered=false;
        for(unsigned branch=0;branch<2;++branch)if(const auto path=endpointPath(first,last,a,b,scale,branch)) {
            const auto x=directionAt(*path,a),y=directionAt(*path,b);
            CHECK(std::hypot(x.x-first.x,std::hypot(x.y-first.y,x.z-first.z))<1e-10);
            CHECK(std::hypot(y.x-last.x,std::hypot(y.y-last.y,y.z-last.z))<1e-10);
            const auto mirror=endpointPath({first.x,-first.y,first.z},{last.x,-last.y,last.z},a,b,scale,branch);REQUIRE(mirror);
            CHECK(std::hypot(mirror->readDirection.x-path->readDirection.x,mirror->readDirection.y+path->readDirection.y)<1e-10);
            CHECK(std::hypot(mirror->readDerivative.x-path->readDerivative.x,mirror->readDerivative.y+path->readDerivative.y)*scale<1e-10);
            const double directionError=std::hypot(path->readDirection.x-std::cos(angle),path->readDirection.y-std::sin(angle));
            const double rateError=std::hypot(path->readDerivative.x*scale+std::sin(angle)*speed,path->readDerivative.y*scale-std::cos(angle)*speed);
            if(directionError<1e-10&&rateError<1e-10)recovered=true;
        }
        INFO("scale="<<scale<<" offsets="<<offsets[0]<<','<<offsets[1]<<" angle="<<angle<<" rateT="<<speed);
        CHECK(recovered);
    }
    const auto same=unitRay(1,.02,-.01);
    REQUIRE(endpointPath(same,same,-.01,-.002,.01,0));
    CHECK(endpointPath(same,same,-.01,-.002,.01,0)->stationary);
    CHECK_FALSE(endpointPath(same,same,-.01,-.002,.01,1));
    CHECK_FALSE(endpointPath(same,same,0,0,.01,0));
    CHECK_FALSE(endpointPath({0,0,0},same,-.01,-.002,.01,0));
    CHECK_FALSE(endpointPath(same,same,-.01,-.002,0,0));
    CHECK_FALSE(endpointPath(same,same,std::numeric_limits<double>::quiet_NaN(),-.002,.01,0));
    const auto over=[&](double t){return unitRay(1,(.5+1e-7)*t,0);};
    CHECK_FALSE(endpointPath(over(-.8),over(-.2),-.8,-.2,1,0));
    CHECK_FALSE(endpointPath(over(-.8),over(-.2),-.8,-.2,1,1));
    const auto singular=endpointPath(unitRay(1,-2,0),unitRay(1,-.5,0),-.8,-.2,.01,0);
    if(singular)CHECK(singular->domainBoundary);
}

TEST_CASE("Moving-family Gaussian trial budget preserves the original aggregate nominal bound", "[thermal-reticle][thermal-reticle-moving-v2-budget]") {
    // Independent Mills/chi-square2 union calculation for all supported
    // dimension extremes and sigma approaching its strictly-positive infimum.
    for(const unsigned width:{8u,64u,128u})for(const unsigned height:{8u,64u,128u})for(const double sigma:{1e-12,5.,30.}) {
        const double nx=width*std::ldexp(1.,18)+1,ny=height*std::ldexp(1.,18)+1;
        const double z=std::sqrt(sigma*sigma+2*(std::log(2.)+2*std::log(nx)+2*std::log(ny)));
        const double aggregate=.5+.25+1/(std::sqrt(2*pi)*z);
        CHECK(aggregate<1);
        CHECK(aggregate<=.786751313678);
    }
}

TEST_CASE("Zero-motion V2 basis keeps original pole chronology without hidden electronic state", "[thermal-reticle][thermal-reticle-moving-v2-basis]") {
    auto c=reticlePolicy();c.detector.width=c.detector.height=8;c.periods=3;c.samplesPerScan=24;c.spatialSamples=2;
    c.scanRadiusRad=.003;c.stopRadiusRad=.006;
    ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(c,error));
    receiver.expose(reticleScene(0,100),{1,0,0},77);
    const auto actual=receiver.expose(reticleScene(.05,100),{1,0,0},77);REQUIRE(actual.complete);REQUIRE(actual.valid);
    // Forward(1,0,0) and up(0,0,1) put the optical right axis along-Y.
    const double x=-std::atan(.003),y=std::atan(-.0015);
    const std::array<double,7> shape{{x,y,x,y,.003,.003,0}};
    const auto basis=receiver.movingBasisForCalibrationV2(actual,{1,0,0},{0,0,1},shape);REQUIRE(basis);
    auto alternate=actual;alternate.expectedElectrons.assign(c.samplesPerScan,1e90);alternate.incidentElectrons.clear();
    alternate.measuredElectrons.assign(c.samplesPerScan,12345.);alternate.varianceBoundElectrons2=7;
    const auto repeated=receiver.movingBasisForCalibrationV2(alternate,{1,0,0},{0,0,1},shape);REQUIRE(repeated);
    CHECK((*basis)[0]==(*repeated)[0]);CHECK((*basis)[1]==(*repeated)[1]);
    const auto* latest=receiver.latest();REQUIRE(latest);
    if(latest)CHECK(latest->measuredElectrons==actual.measuredElectrons);
    const auto& p=c.detector;const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));
    std::vector<double> flat(c.samplesPerScan,0);
    const auto primitive=[](double a,double b){return std::atan(a*b/std::sqrt(1+a*a+b*b));};
    for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
        const double cx=(col+.5-.5*p.width)*p.pixelPitchRad,cy=(row+.5-.5*p.height)*p.pixelPitchRad;
        const double xl=std::tan(cx-.5*p.pixelPitchRad),xr=std::tan(cx+.5*p.pixelPitchRad),yl=std::tan(cy-.5*p.pixelPitchRad),yr=std::tan(cy+.5*p.pixelPitchRad);
        const double omega=primitive(xr,yr)-primitive(xl,yr)-primitive(xr,yl)+primitive(xl,yl);
        for(unsigned j=0;j<c.samplesPerScan;++j) {
            double weight=0;
            for(unsigned k=0;k<p.exposureSamples;++k) {
                const double phase=2*pi*((j+.5)/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
                for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx)
                    weight+=ThermalReticleReceiverV1::maskV1(c,cx+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad,cy+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad,phase);
            }
            weight/=static_cast<double>(p.exposureSamples)*c.spatialSamples*c.spatialSamples;flat[j]+=omega*weight;
        }
    }
    const double gain=p.apertureAreaM2*p.opticalTransmission*p.integrationTimeS*p.quantumEfficiency*p.effectiveWavelengthM/(6.62607015e-34*299792458.);
    const double source=100*.003*.003*gain,dark=static_cast<double>(p.width)*p.height*p.darkCurrentElectronsS*p.integrationTimeS;
    double skyState=0,darkState=0;std::vector<double> residual(c.samplesPerScan);
    for(unsigned j=0;j<c.samplesPerScan;++j) {
        skyState+=alpha*(gain*flat[j]-skyState);darkState+=alpha*(dark-darkState);
        residual[j]=actual.expectedElectrons[j]-skyState-darkState-source*(*basis)[1][j];
    }
    // First-gap response and unknown prior charge supply ONE transient, not
    // hidden forward state. This check reads expected photons only as oracle.
    const double initial=residual.at(0);double maximum=0;
    for(unsigned j=0;j<c.samplesPerScan;++j)maximum=std::max(maximum,std::abs(residual[j]-initial*std::pow(1-alpha,j)));
    CHECK(maximum<1e-5);
}

#include "simulation/mission_systems/components/support/MeasuredSeekerV2.h"
TEST_CASE("Moving regional native-background memo preserves original optical bytes", "[thermal-reticle-moving-v2-memo]") {
    using fsim::ms::components::ThermalReticleReceiverV1;
    using fsim::ms::components::ThermalImageReceiverV1;
    using fsim::ms::components::ThermalSceneFrameV1;

    auto c=reticlePolicy();
    const auto& p=c.detector;
    const auto pointing=unitRay(1,.01,.001),up=unitRay(0,0,1);
    const auto nativePointing=fsim::ms::components::measured::unit(pointing),nativeUp=fsim::ms::components::measured::unit(up);
    const double halfX=.5*p.width*p.pixelPitchRad,halfY=.5*p.height*p.pixelPitchRad;
    for(const double cutoff:{-.03,0.,.03}) {
        auto optical=p;optical.exposureSamples=1;
        ThermalSceneFrameV1 scene;scene.up=nativeUp;scene.groundHorizonCosine=cutoff;scene.groundRadianceWm2Sr=1;
        const auto native=ThermalImageReceiverV1::incidentFieldV1(scene,nativePointing,optical);
        REQUIRE(native.has_value());
        unsigned requests=0;
        const auto provider=[&]()->const std::vector<double>* {++requests;return &native->radiance;};
        for(const auto& region:std::vector<std::array<double,4>>{
            {{-.01,.01,-.006,.006}},
            {{-halfX-.006,-halfX+.006,-halfY,halfY}},
            {{-halfX,halfX,-halfY-.006,-halfY+.006}},
            {{-halfX,halfX,-halfY,halfY}}}) {
            bool oldAccelerated=false,newAccelerated=false;
            const auto original=ThermalReticleReceiverV1::unitGroundFieldV1(p,pointing,up,cutoff,region,&oldAccelerated);
            const auto memo=ThermalReticleReceiverV1::unitGroundFieldMemoV2(p,pointing,up,cutoff,region,provider,&newAccelerated);
            REQUIRE(original.has_value());REQUIRE(memo.has_value());
            CHECK(oldAccelerated==newAccelerated);
            REQUIRE(original->size()==memo->size());
            // Includes sensitive field-edge fallbacks and all detector cells;
            // no tolerance can hide altered native integral/subtraction bits.
            CHECK(*original==*memo);
        }
        INFO("native memo requests="<<requests); CHECK(requests>0);
    }
}

#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
#include <xmmintrin.h>
#endif
TEST_CASE("Certified phase-subset refusal preserves uncertain and admissible candidates", "[thermal-reticle][thermal-reticle-moving-v2-subset]") {
    using namespace fsim::ms::components::moving_radiator_v2;
    const auto length=[](const std::vector<double>& x){double result=0;for(const auto v:x)result=std::hypot(result,v);return result;};
    for(const unsigned rows:{16u,32u,64u})for(const double perturbation:{0.,1e-12,1e-6})
    for(const double amplitude:{-1e6,-2.,0.,3.,1e6}) {
        std::array<std::vector<double>,2> exact,approximate;
        exact[0].resize(rows);exact[1].resize(rows);
        std::vector<double> y(rows),actualResidual(rows),targetError(rows);
        std::array<std::vector<double>,2> errors;
        for(unsigned j=0;j<rows;++j) {
            const double phase=2*pi*(j+.5)/rows;
            exact[0][j]=std::sin(phase);exact[1][j]=std::cos(phase);
            actualResidual[j]=.01*std::sin(3*phase);
            y[j]=amplitude*exact[0][j]-2*exact[1][j]+actualResidual[j];
            targetError[j]=perturbation*std::cos(4*phase);
            y[j]+=targetError[j];
        }
        approximate=exact;
        for(unsigned column=0;column<2;++column) {
            errors[column].resize(rows);
            for(unsigned j=0;j<rows;++j) {
                errors[column][j]=perturbation*std::sin((column+5)*2*pi*(j+.5)/rows);
                approximate[column][j]+=errors[column][j];
            }
        }
        // Explicitly bounded data/column perturbations; a genuinely allowed
        // full residual must never be discarded by the numerical certificate.
        const double rounding=64*std::numeric_limits<double>::epsilon()*std::max(1.,length(y));
        const double columnGuard=64*std::numeric_limits<double>::epsilon()*std::max({1.,length(exact[0]),length(exact[1])});
        const double allowed=length(actualResidual)+rounding;
        INFO("rows="<<rows<<" perturbation="<<perturbation<<" amplitude="<<amplitude);
        CHECK_FALSE(certifiedSubsetRefusalV2(y,approximate,length(targetError)+rounding,
            {{length(errors[0])+columnGuard,length(errors[1])+columnGuard}},allowed));
    }
    const unsigned rows=32;
    std::array<std::vector<double>,2> columns;
    columns[0].resize(rows);columns[1].resize(rows);std::vector<double> y(rows);
    for(unsigned j=0;j<rows;++j) {
        const double phase=2*pi*(j+.5)/rows;
        columns[0][j]=std::sin(phase);columns[1][j]=std::cos(phase);y[j]=10*std::sin(3*phase);
    }
    const bool extended=subsetCertificateArithmeticV2();
    if(extended)REQUIRE(certifiedSubsetRefusalV2(y,columns,0,{{0,0}},1));
    else CHECK_FALSE(certifiedSubsetRefusalV2(y,columns,0,{{0,0}},1));
    CHECK_FALSE(certifiedSubsetRefusalV2(y,columns,-1,{{0,0}},1));
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
    bool flushedDecision=true,flushedAvailable=true;
    {
        struct RestoreMxcsr {unsigned saved;~RestoreMxcsr(){_mm_setcsr(saved);}} restore{_mm_getcsr()};
        _mm_setcsr(restore.saved|0x8040u);
        flushedAvailable=subsetCertificateArithmeticV2();
        flushedDecision=certifiedSubsetRefusalV2(y,columns,0,{{0,0}},1);
    }
    CHECK_FALSE(flushedAvailable);CHECK_FALSE(flushedDecision);
#endif
    columns[1]=columns[0];
    CHECK_FALSE(certifiedSubsetRefusalV2(y,columns,0,{{0,0}},1));
    for(unsigned j=0;j<rows;++j)columns[1][j]=std::cos(2*pi*(j+.5)/rows);
    for(auto& value:y)value*=1e-201;
    // Native SSE may underflow. The caller converts its zero cost using the
    // explicit absolute square/sum allowance; a relative-only bound is unsafe.
    const double zeroCostNorm=std::sqrt(rows*std::numeric_limits<double>::denorm_min());
    CHECK_FALSE(certifiedSubsetRefusalV2(y,columns,0,{{0,0}},zeroCostNorm));
}

TEST_CASE("Shared moving regional phase kernel preserves full native basis bytes", "[thermal-reticle][thermal-reticle-moving-v2-phase]") {
    using namespace fsim::ms::components;
    using namespace fsim::ms::components::moving_radiator_v2;
    namespace m=fsim::ms::components::measured;
    auto c=reticlePolicy();c.detector.width=c.detector.height=8;c.periods=3;c.samplesPerScan=24;c.spatialSamples=2;
    c.scanRadiusRad=.003;c.stopRadiusRad=.006;
    const auto& p=c.detector;ThermalReticleReceiverV1 receiver;std::string error;
    REQUIRE(receiver.configure(c,error));
    ThermalReticleResultV1 clock;clock.valid=true;clock.complete=true;clock.timeS=.05;
    clock.exposureStartTimeS=.04;clock.exposureEndTimeS=.05;clock.measurementTimeS=.045;
    clock.varianceBoundElectrons2=1;clock.measuredElectrons.assign(c.samplesPerScan,0);
    const fsim::ms::weapon::V3 pointing{1,0,0},up{0,0,1};
    const Shape shape{{-.0015,-.0008,.0015,.0008,.003,.003,0}};
    const auto basis=receiver.movingBasisForCalibrationV2(clock,pointing,up,shape);REQUIRE(basis);
    const double first=clock.exposureStartTimeS+.5/(c.frequencyHz*c.samplesPerScan)+p.integrationTimeS*(1./p.exposureSamples-.5);
    const double last=clock.exposureStartTimeS+(c.samplesPerScan-.5)/(c.frequencyHz*c.samplesPerScan)+.5*p.integrationTimeS;
    const auto route=endpointPath(m::unit({1,std::tan(shape[0]),std::tan(shape[1])}),
        m::unit({1,std::tan(shape[2]),std::tan(shape[3])}),first-clock.timeS,last-clock.timeS,p.integrationTimeS,0);
    REQUIRE(route);const auto full=ThermalReticleReceiverV1::unitGroundFieldV1(p,pointing,up,shape[6]);REQUIRE(full);
    std::vector<double> reference(c.samplesPerScan,0);
    const auto f=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    for(unsigned j=0;j<c.samplesPerScan;++j)for(unsigned k=0;k<p.exposureSamples;++k) {
        const double state=clock.exposureStartTimeS+(j+.5)/(c.frequencyHz*c.samplesPerScan)+
            p.integrationTimeS*((k+.5)/p.exposureSamples-.5)+.5*p.integrationTimeS/p.exposureSamples;
        const auto photon=m::unit(m::subtract(directionAt(*route,state-clock.timeS),m::scale(route->readDerivative,.5*p.integrationTimeS/p.exposureSamples)));
        const double x=std::atan2(photon.y,photon.x),y=std::atan2(photon.z,photon.x);
        const std::array<double,4> region{{x-.5*shape[4],x+.5*shape[4],y-.5*shape[5],y+.5*shape[5]}};
        const auto occulted=ThermalReticleReceiverV1::unitGroundFieldV1(p,pointing,up,shape[6],region);REQUIRE(occulted);
        // Frozen pre-optimization pixel/subexposure accumulation. These fields
        // cross the physical horizon at every time, including all four inner
        // integrations; no temporal or spatial quadrature is approximated.
        for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
            const auto i=static_cast<std::size_t>(row)*p.width+col;
            const double cx=(col+.5-.5*p.width)*p.pixelPitchRad,cy=(row+.5-.5*p.height)*p.pixelPitchRad;
            const double xl=std::tan(cx-p.pixelPitchRad/2),xr=std::tan(cx+p.pixelPitchRad/2);
            const double yl=std::tan(cy-p.pixelPitchRad/2),yr=std::tan(cy+p.pixelPitchRad/2);
            const double solid=f(xr,yr)-f(xl,yr)-f(xr,yl)+f(xl,yl);
            const double phase=2*pi*((j+.5)/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
            double weight=0;
            for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx)
                weight+=ThermalReticleReceiverV1::maskV1(c,cx+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad,
                    cy+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad,phase);
            weight/=static_cast<double>(c.spatialSamples)*c.spatialSamples;
            const double field=(*full)[i]-(*occulted)[i];
            reference[j]+=field*solid*weight/(p.pixelPitchRad*p.pixelPitchRad)/p.exposureSamples;
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));double previous=0;
    for(auto& value:reference){previous+=alpha*(value-previous);value=previous;}
    CHECK(reference==(*basis)[0]);
    CHECK(receiver.latest()==nullptr);
}

TEST_CASE("Bounded-unit optical zero shortcut matches immutable native detector bytes", "[thermal-reticle][thermal-reticle-unit-zero]") {
    auto c=reticlePolicy().detector;c.exposureSamples=1;
    // This is the actual 64x64 single-exposure unit-field policy used by the
    // inverse's regional renderer, including broad field-edge opaque supports.
    const std::array<std::array<double,3>,4> scenes{{{{0,1,0}},{{1,0,-.001}},{{.25,.75,.002}},{{1,3,-.0005}}}};
    for(std::size_t i=0;i<scenes.size();++i) {
        INFO("scene="<<i<<" sky="<<scenes[i][0]<<" ground="<<scenes[i][1]);
        ThermalImageReceiverV1 current;FrozenThermalImageReceiverV1 frozen;std::string error;
        REQUIRE(current.configure(c,error));REQUIRE(frozen.configure(c,error));
        auto frame=reticleScene(.1,0);frame.skyRadianceWm2Sr=scenes[i][0];frame.groundRadianceWm2Sr=scenes[i][1];
        frame.groundHorizonCosine=scenes[i][2];
        ThermalRadiancePatchV1 opaque;opaque.rangeM=1;opaque.transmission=1;opaque.radianceWm2Sr=0;
        if(i==2) {
            opaque.direction=unitRay(1,.002,-.001);
            opaque.widthRad=1.5*c.width*c.pixelPitchRad;opaque.heightRad=.12*c.height*c.pixelPitchRad;
        } else {
            opaque.direction=unitRay(1,i%2?.022:-.022,.004);
            opaque.widthRad=.85*c.width*c.pixelPitchRad;opaque.heightRad=1.3*c.height*c.pixelPitchRad;
        }
        frame.patches.push_back(opaque);frame.refereeAttributionIndices.push_back(17);
        const fsim::ms::weapon::V3 pointing{1.013,.0013*static_cast<double>(i)-.001,.00071};
        const auto actual=current.expose(frame,pointing,8127,false);
        const auto reference=frozen.expose(frame,pointing,8127,false);
        REQUIRE(actual.valid);REQUIRE(reference.valid);REQUIRE(actual.expectedElectrons.size()==4096);
        CHECK(actual.expectedElectrons==reference.expectedElectrons);
        CHECK(actual.measuredElectrons==reference.measuredElectrons);
        CHECK(actual.backgroundElectrons==reference.backgroundElectrons);
        CHECK(actual.contrastElectrons==reference.contrastElectrons);
        CHECK(actual.noiseSigmaElectrons==reference.noiseSigmaElectrons);
        CHECK(actual.detectionThresholdElectrons==reference.detectionThresholdElectrons);
        CHECK(actual.quadrantElectrons==reference.quadrantElectrons);
        CHECK(actual.quadrantBackgroundElectrons==reference.quadrantBackgroundElectrons);
        CHECK(actual.quadrantVarianceElectrons2==reference.quadrantVarianceElectrons2);
        CHECK(std::any_of(actual.expectedElectrons.begin(),actual.expectedElectrons.end(),[](double value){return value>1;}));
    }
}

TEST_CASE("Unit optical shortcut preserves signed bytes and non-nearest fallback", "[thermal-reticle][thermal-reticle-unit-byte-controls]") {
    struct RestoreFloatingEnvironment {
        std::fenv_t saved{};
        bool available=std::fegetenv(&saved)==0;
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        unsigned csr=_mm_getcsr();
#endif
        ~RestoreFloatingEnvironment() noexcept {
            if(available)std::fesetenv(&saved);
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
            _mm_setcsr(csr);
#endif
        }
    } restore;
    REQUIRE(restore.available);
    const auto sameScalar=[](const double& a,const double& b){return std::memcmp(&a,&b,sizeof(double))==0;};
    const auto sameArray=[&](const auto& a,const auto& b){
        return a.size()==b.size()&&std::equal(a.begin(),a.end(),b.begin(),sameScalar);
    };
    // Prove the comparator itself distinguishes the sign of exact zero.
    CHECK_FALSE(sameScalar(0.,-0.));
    CHECK_FALSE(sameArray(std::array<double,1>{{0.}},std::array<double,1>{{-0.}}));
    auto c=reticlePolicy().detector;c.exposureSamples=1;
    // Two signed-zero unit scenes under nearest exercise the shortcut domain;
    // the identical bounded scene under either directed rounding must retain
    // the unchanged native integral path. Four captures, no timing assertion.
    struct Control {int rounding;double sky,ground;};
    const std::array<Control,4> controls{{{FE_TONEAREST,-0.,1.},{FE_TONEAREST,1.,-0.},
        {FE_DOWNWARD,.25,.75},{FE_UPWARD,.25,.75}}};
    for(const auto& control:controls) {
        INFO("rounding="<<control.rounding<<" sky sign="<<std::signbit(control.sky)<<" ground sign="<<std::signbit(control.ground));
        REQUIRE(std::fesetround(control.rounding)==0);
        REQUIRE(std::fegetround()==control.rounding);
#if defined(__SSE__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP>=1)
        const unsigned expected=control.rounding==FE_DOWNWARD?0x2000u:control.rounding==FE_UPWARD?0x4000u:0u;
        REQUIRE((_mm_getcsr()&0x6000u)==expected);
#endif
        ThermalImageReceiverV1 current;FrozenThermalImageReceiverV1 frozen;std::string error;
        REQUIRE(current.configure(c,error));REQUIRE(frozen.configure(c,error));
        auto frame=reticleScene(.1,0);frame.skyRadianceWm2Sr=control.sky;frame.groundRadianceWm2Sr=control.ground;
        frame.groundHorizonCosine=.002;
        ThermalRadiancePatchV1 opaque;opaque.rangeM=1;opaque.transmission=1;opaque.radianceWm2Sr=0;
        opaque.direction=unitRay(1,.022,.004);opaque.widthRad=.85*c.width*c.pixelPitchRad;
        opaque.heightRad=1.3*c.height*c.pixelPitchRad;frame.patches.push_back(opaque);frame.refereeAttributionIndices.push_back(17);
        const fsim::ms::weapon::V3 pointing{1.013,-.001,.00071};
        const auto actual=current.expose(frame,pointing,8127,false);
        const auto reference=frozen.expose(frame,pointing,8127,false);
        REQUIRE(actual.valid);REQUIRE(reference.valid);REQUIRE(actual.expectedElectrons.size()==4096);
        CHECK(sameArray(actual.expectedElectrons,reference.expectedElectrons));
        CHECK(sameArray(actual.measuredElectrons,reference.measuredElectrons));
        CHECK(sameArray(actual.backgroundElectrons,reference.backgroundElectrons));
        CHECK(sameArray(actual.contrastElectrons,reference.contrastElectrons));
        CHECK(sameArray(actual.noiseSigmaElectrons,reference.noiseSigmaElectrons));
        CHECK(sameArray(actual.detectionThresholdElectrons,reference.detectionThresholdElectrons));
        CHECK(sameArray(actual.quadrantElectrons,reference.quadrantElectrons));
        CHECK(sameScalar(actual.quadrantBackgroundElectrons,reference.quadrantBackgroundElectrons));
        CHECK(sameScalar(actual.quadrantVarianceElectrons2,reference.quadrantVarianceElectrons2));
    }
}


namespace {
ThermalReticleResultV1 r60MissedStaticMeasuredPacket() {
    ThermalReticleResultV1 m;m.valid=true;m.complete=true;
    m.timeS=15.866666666666621;
    m.measurementTimeS=15.860016352382711;
    m.exposureStartTimeS=15.856246441198403;
    m.exposureEndTimeS=15.863786263567018;
    m.varianceBoundElectrons2=5511303122.949204;
    m.measuredElectrons={97005920.77153146,97505951.29097235,98005981.81041323,98506012.32985412,98756027.58957456,99256058.10901543,100256119.14789721,102006225.9659403,104006348.04370384,106756515.9006287,109006653.23811267,111256790.57559665,112756882.1339193,113506927.91308063,114006958.4325215,114256973.69224195,114506988.9519624,114757004.21168284,115757065.2505646,116757126.28944637,117757187.32832815,119007263.62693036,120507355.18525301,122007446.74357565,123757553.56161875,125757675.63938229,128007812.97686626,130257950.31435023,132008057.13239333,133258133.43099554,133758163.95043643,134008179.21015687,133508148.69071598,132508087.65183422,131508026.61295244,130757980.83379112,130507965.57407068,130257950.31435023,130007935.0546298,129507904.53518891,128507843.49630715,126757736.67826405,124757614.60050051,122257462.00329609,120007324.66581212,118007202.58804858,116257095.7700055,115507049.99084416,114757004.21168284,114506988.9519624,114256973.69224195,114006958.4325215,113756943.17280108,113006897.39363974,111756821.09503753,109756699.017274,107506561.67979002,105256424.34230605,103506317.52426295,102256241.22566074,101006164.92705853,99756088.62845632,98255997.07013367,97005920.77153146,95505829.21320881,93755722.39516571,92255630.83684307,90255508.75907953,88005371.42159556,85505218.82439114,82755050.96746628,80004883.11054142,77504730.51333699,75504608.43557346,73504486.35780992,71504364.28004639,69504242.20228285,67504120.12451932,65253982.78703534,63003845.44955136,60753708.11206739,58503570.77458341,56753463.95654032,55253372.39821766,54253311.35933589,53753280.83989501,53503265.58017457,53253250.32045413,52753219.801013246,52003174.02185192,50753097.72324971,49002990.90520661,47502899.34688396,46502838.308002196,45752792.52884087,45502777.269120425,46002807.788561314,47002868.82744308,48002929.86632485,48752975.645486176,49253006.16492706,49503021.4246475,49753036.68436794,49503021.4246475,49503021.4246475,50003051.944088385,50503082.46352927,51503143.50241104,52753219.801013246,54003296.099615455,55253372.39821766,56503448.69681987,58253555.51486297,60503692.85234694,62753830.18983092,65003967.5273149,67004089.60507844,69004211.68284197,70504303.24116462,72004394.79948728,73754501.61753036,75754623.6952939,78254776.29249832,80754928.88970274,83505096.7466276,86505279.8632729,89255447.72019777,92005615.57712263};
    m.phases={0.02454369260617026,0.07363107781851078,0.1227184630308513,0.1718058482431918,0.22089323345553233,0.2699806186678728,0.3190680038802134,0.36815538909255385,0.4172427743048944,0.4663301595172349,0.5154175447295755,0.5645049299419159,0.6135923151542565,0.662679700366597,0.7117670855789375,0.760854470791278,0.8099418560036186,0.859029241215959,0.9081166264282996,0.9572040116406401,1.0062913968529805,1.055378782065321,1.1044661672776617,1.1535535524900022,1.2026409377023426,1.2517283229146832,1.3008157081270237,1.3499030933393643,1.3989904785517047,1.4480778637640452,1.4971652489763858,1.5462526341887264,1.5953400194010667,1.6444274046134073,1.6935147898257479,1.7426021750380885,1.7916895602504288,1.8407769454627694,1.88986433067511,1.9389517158874505,1.9880391010997909,2.0371264863121317,2.086213871524472,2.1353012567368124,2.184388641949153,2.2334760271614935,2.282563412373834,2.3316507975861747,2.380738182798515,2.429825568010856,2.478912953223196,2.5280003384355365,2.5770877236478773,2.6261751088602177,2.675262494072558,2.724349879284899,2.773437264497239,2.82252464970958,2.8716120349219203,2.9206994201342606,2.9697868053466014,3.018874190558942,3.067961575771282,3.117048960983623,3.1661363461959633,3.215223731408304,3.2643111166206444,3.313398501832985,3.3624858870453256,3.411573272257666,3.4606606574700063,3.509748042682347,3.5588354278946874,3.6079228131070282,3.6570101983193686,3.706097583531709,3.7551849687440497,3.80427235395639,3.8533597391687304,3.902447124381071,3.9515345095934116,4.000621894805752,4.049709280018092,4.0987966652304335,4.147884050442774,4.196971435655114,4.246058820867455,4.295146206079795,4.344233591292136,4.3933209765044765,4.442408361716817,4.491495746929157,4.540583132141498,4.589670517353839,4.638757902566179,4.6878452877785195,4.73693267299086,4.7860200582032,4.8351074434155406,4.884194828627882,4.933282213840222,4.9823695990525625,5.031456984264903,5.080544369477243,5.129631754689584,5.178719139901925,5.227806525114265,5.2768939103266055,5.325981295538946,5.375068680751287,5.424156065963627,5.473243451175968,5.522330836388308,5.5714182216006485,5.620505606812989,5.66959299202533,5.71868037723767,5.767767762450011,5.816855147662351,5.8659425328746915,5.915029918087033,5.964117303299373,6.013204688511713,6.062292073724054,6.111379458936394,6.160466844148735,6.209554229361076,6.258641614573416};
    m.pointing={-0.02554972686418913,0.9993799073582142,-0.024228335185322723};return m;
}
}

TEST_CASE("Archived R60 measured packet retains its original static source-evidence predicate", "[thermal-reticle][thermal-reticle-measured-prior]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"r-60m.json",*cards.at("r-60m"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    auto measured=r60MissedStaticMeasuredPacket();
    // Two archived precise scalar feature rays, not truth or current track state.
    ThermalReticleMeasuredPriorV2 prior{15.822317240539633,15.833333333333288,{-0.02276252594694555,0.9994438737487444,-0.024368230108451506},{-0.03380246239806965,-0.0018099342900733271,-0.042657932318783155},0.01946714401245117,0.009427242279052735,-0.04326684001176012};
    receiver.inferStaticHorizonForCalibrationV2(measured,measured.pointing,{0.7660437073695859,0.0008759250945638627,0.6427878897075526},&prior);
    INFO("refusal="<<measured.horizonDiagnostic.refusal<<" source="<<measured.horizonDiagnostic.sourceResidual);
    CHECK(measured.measuredElectrons==r60MissedStaticMeasuredPacket().measuredElectrons);
    const double degrees=static_cast<double>(policy.samplesPerScan)-7-7;
    const double tail=.5*policy.detector.detectionSigma*policy.detector.detectionSigma;
    const double bound=degrees+2*std::sqrt(degrees*tail)+2*tail;
    REQUIRE(measured.horizonDiagnostic.fitAvailable);
    CHECK(measured.horizonDiagnostic.sourceResidual<=bound);
    REQUIRE_FALSE(measured.features.empty());CHECK(measured.bearingResolved);
}

TEST_CASE("A measured prior cannot create evidence in a source-free scalar packet", "[thermal-reticle][thermal-reticle-measured-prior-controls]") {
    auto policy=reticlePolicy();ThermalReticleReceiverV1 receiver;std::string error;REQUIRE(receiver.configure(policy,error));
    ThermalReticleResultV1 blank;blank.valid=blank.complete=true;blank.timeS=.03;blank.measurementTimeS=.025;
    blank.exposureStartTimeS=.02;blank.exposureEndTimeS=.03;blank.varianceBoundElectrons2=1;
    blank.measuredElectrons.assign(policy.samplesPerScan,0.);
    ThermalReticleMeasuredPriorV2 prior{.01,.015,{1,0,0},{0,.01,0},.003,.003,0};
    const auto check=[&](const ThermalReticleMeasuredPriorV2* proposal) {
        auto measured=blank;receiver.inferStaticHorizonForCalibrationV2(measured,{1,0,0},{0,0,1},proposal);
        CHECK_FALSE(measured.bearingResolved);CHECK_FALSE(measured.preciseBearing);CHECK(measured.features.empty());
        CHECK(measured.measuredElectrons==blank.measuredElectrons);CHECK(receiver.latest()==nullptr);
    };
    check(nullptr);check(&prior);
    prior.captureTimeS=blank.measurementTimeS;check(&prior);
    prior.captureTimeS=.01;prior.availableTimeS=blank.exposureStartTimeS+.001;check(&prior);
    prior.availableTimeS=.015;prior.direction.x=std::numeric_limits<double>::quiet_NaN();check(&prior);
}

TEST_CASE("Invalid measured prior retains the complete captured no-prior result", "[thermal-reticle][thermal-reticle-measured-prior-invalid]") {
    using namespace fsim::ms;
    std::vector<std::string> errors;const auto library=std::filesystem::path(FSIM_TEST_STORE_DIR);
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());ProfileV1 profile;std::string error;
    REQUIRE(readProfileV1(library/"r-60m.json",*cards.at("r-60m"),profile,error));
    DescriptorV1 scalar;ReceiverConfigurationV3 configuration;
    REQUIRE(receiverConfigurationV3(*profile.find("seeker"),scalar,configuration,error));REQUIRE(configuration.reticle);
    const auto& policy=configuration.reticle->policy;
    ThermalReticleReceiverV1 receiver;REQUIRE(receiver.configure(policy,error));
    auto measured=r60MissedStaticMeasuredPacket();
    auto reference=measured;
    receiver.inferStaticHorizonForCalibrationV2(reference,reference.pointing,{0.7660437073695859,0.0008759250945638627,0.6427878897075526});
    ThermalReticleMeasuredPriorV2 invalid;invalid.captureTimeS=measured.measurementTimeS;
    invalid.availableTimeS=measured.timeS;invalid.direction={1,0,0};invalid.widthRad=invalid.heightRad=.003;
    // An equal/future capture cannot initialize itself, even if all other
    // fields happen to be finite. This must run the FULL unchanged cold path.
    receiver.inferStaticHorizonForCalibrationV2(measured,measured.pointing,{0.7660437073695859,0.0008759250945638627,0.6427878897075526},&invalid);
    CHECK(measured.bearingResolved==reference.bearingResolved);CHECK(measured.preciseBearing==reference.preciseBearing);
    CHECK(measured.horizonDiagnostic.refusal==reference.horizonDiagnostic.refusal);
    CHECK(measured.horizonDiagnostic.shape==reference.horizonDiagnostic.shape);
    CHECK(measured.horizonDiagnostic.sourceResidual==reference.horizonDiagnostic.sourceResidual);
    CHECK(measured.horizonDiagnostic.nullResidual==reference.horizonDiagnostic.nullResidual);
    CHECK(measured.horizonDiagnostic.rawBudget==reference.horizonDiagnostic.rawBudget);
    CHECK(measured.measuredElectrons==reference.measuredElectrons);REQUIRE(measured.features.size()==reference.features.size());
    for(std::size_t i=0;i<measured.features.size();++i) {
        CHECK(measured.features[i].direction.x==reference.features[i].direction.x);
        CHECK(measured.features[i].direction.y==reference.features[i].direction.y);
        CHECK(measured.features[i].direction.z==reference.features[i].direction.z);
        CHECK(measured.features[i].angularVarianceRad2==reference.features[i].angularVarianceRad2);
    }
}

#endif

#if defined(FSIM_HAS_RETICLE)
#include "simulation/mission_systems/components/support/MovingConicMomentsV4.h"
TEST_CASE("Private conic V4 retains field budget signed zero and unsupported cone fallback", "[thermal-reticle][thermal-conic-v4-wrapper]") {
    namespace v4=fsim::ms::components::conic_moments_v4;
    const auto policy=reticlePolicy();const auto& p=policy.detector;
    REQUIRE(v4::arithmeticAvailable());
    const long double tiny=std::numeric_limits<long double>::denorm_min();
    for(const long double zero:{0.L,-0.L}){const auto endpoints=v4::outwardForCalibration(zero);
        CHECK(endpoints[0]==-tiny);CHECK(endpoints[1]==tiny);CHECK(endpoints[0]<endpoints[1]);}
    const auto moments=v4::momentsForCalibration(.0232499999999999998335,.0299999999999999988898,
        .0299999999999999988898,.031500000000000000222,.00075,.0307499999999999995559);
    REQUIRE(moments);CHECK(moments->at(2).low>0);
    v4::Request request;request.width=p.width;request.height=p.height;
    request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;
    request.pointing={{1,0,0}};request.up={{0,0,1}};request.region={{-.01,.03,-.012,.006}};
    auto bank=v4::Bank::prepare(request);REQUIRE(bank);
    const auto rendered=bank->evaluate(request.region);REQUIRE(rendered);
    const auto original=ThermalReticleReceiverV1::unitGroundFieldV1(p,{1,0,0},{0,0,1},0,request.region);
    REQUIRE(original);REQUIRE(rendered->field.size()==original->size());
    for(std::size_t i=0;i<original->size();++i){CHECK(rendered->absoluteError[i]<=v4::fieldBudget);
        CHECK(std::abs(rendered->field[i]-original->at(i))<=v4::fieldBudget);}
    request.pointing={{0,0,1}};request.cutoff=std::cos(.012);CHECK_FALSE(v4::Bank::prepare(request));
}
#endif

#if defined(FSIM_HAS_RETICLE)
TEST_CASE("Version4 moving basis matches its explicitly materialized conic operator", "[thermal-reticle][thermal-reticle-moving-v4-phase]") {
    using namespace fsim::ms::components;
    using namespace fsim::ms::components::moving_radiator_v2;
    namespace m=fsim::ms::components::measured;
    auto c=reticlePolicy();c.detector.width=c.detector.height=8;c.periods=3;c.samplesPerScan=24;c.spatialSamples=2;
    c.scanRadiusRad=.003;c.stopRadiusRad=.006;
    const auto& p=c.detector;ThermalReticleReceiverV1 receiver;std::string error;
    REQUIRE(receiver.configure(c,error));
    ThermalReticleResultV1 clock;clock.valid=true;clock.complete=true;clock.timeS=.05;
    clock.exposureStartTimeS=.04;clock.exposureEndTimeS=.05;clock.measurementTimeS=.045;
    clock.varianceBoundElectrons2=1;clock.measuredElectrons.assign(c.samplesPerScan,0);
    const fsim::ms::weapon::V3 pointing{1,0,0},up{0,0,1};
    const Shape shape{{-.0015,-.0008,.0015,.0008,.003,.003,0}};
    const auto basis=receiver.movingBasisForCalibrationV4(clock,pointing,up,shape);REQUIRE(basis);
    const double first=clock.exposureStartTimeS+.5/(c.frequencyHz*c.samplesPerScan)+p.integrationTimeS*(1./p.exposureSamples-.5);
    const double last=clock.exposureStartTimeS+(c.samplesPerScan-.5)/(c.frequencyHz*c.samplesPerScan)+.5*p.integrationTimeS;
    const auto route=endpointPath(m::unit({1,std::tan(shape[0]),std::tan(shape[1])}),
        m::unit({1,std::tan(shape[2]),std::tan(shape[3])}),first-clock.timeS,last-clock.timeS,p.integrationTimeS,0);
    REQUIRE(route);const auto full=ThermalReticleReceiverV1::unitGroundFieldV1(p,pointing,up,shape[6]);REQUIRE(full);
    namespace v4=fsim::ms::components::conic_moments_v4;
    v4::Request request;request.width=p.width;request.height=p.height;
    request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;
    request.pointing={{pointing.x,pointing.y,pointing.z}};request.up={{up.x,up.y,up.z}};request.cutoff=shape[6];
    auto bank=v4::Bank::prepare(request);REQUIRE(bank);
    const auto legacy=receiver.movingBasisForCalibrationV2(clock,pointing,up,shape);REQUIRE(legacy);
    CHECK((*legacy)[1]==(*basis)[1]); // The source radiator operator is unchanged.
    std::vector<double> reference(c.samplesPerScan,0);
    const auto f=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    for(unsigned j=0;j<c.samplesPerScan;++j)for(unsigned k=0;k<p.exposureSamples;++k) {
        const double state=clock.exposureStartTimeS+(j+.5)/(c.frequencyHz*c.samplesPerScan)+
            p.integrationTimeS*((k+.5)/p.exposureSamples-.5)+.5*p.integrationTimeS/p.exposureSamples;
        const auto photon=m::unit(m::subtract(directionAt(*route,state-clock.timeS),m::scale(route->readDerivative,.5*p.integrationTimeS/p.exposureSamples)));
        const double x=std::atan2(photon.y,photon.x),y=std::atan2(photon.z,photon.x);
        const std::array<double,4> region{{x-.5*shape[4],x+.5*shape[4],y-.5*shape[5],y+.5*shape[5]}};
        const auto occulted=bank->evaluate(region);REQUIRE(occulted);
        // Independently materialized V4 regional fields with the frozen
        // pixel/subexposure accumulation verify the explicit operator boundary.
        for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
            const auto i=static_cast<std::size_t>(row)*p.width+col;
            const double cx=(col+.5-.5*p.width)*p.pixelPitchRad,cy=(row+.5-.5*p.height)*p.pixelPitchRad;
            const double xl=std::tan(cx-p.pixelPitchRad/2),xr=std::tan(cx+p.pixelPitchRad/2);
            const double yl=std::tan(cy-p.pixelPitchRad/2),yr=std::tan(cy+p.pixelPitchRad/2);
            const double solid=f(xr,yr)-f(xl,yr)-f(xr,yl)+f(xl,yl);
            const double phase=2*pi*((j+.5)/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
            double weight=0;
            for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx)
                weight+=ThermalReticleReceiverV1::maskV1(c,cx+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad,
                    cy+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad,phase);
            weight/=static_cast<double>(c.spatialSamples)*c.spatialSamples;
            const double field=(*full)[i]-occulted->field[i];
            reference[j]+=field*solid*weight/(p.pixelPitchRad*p.pixelPitchRad)/p.exposureSamples;
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));double previous=0;
    for(auto& value:reference){previous+=alpha*(value-previous);value=previous;}
    CHECK(reference==(*basis)[0]);
    CHECK(receiver.latest()==nullptr);
}

#endif

#if defined(FSIM_HAS_RETICLE)
TEST_CASE("Version5 moving basis matches its explicitly materialized conic operator", "[thermal-reticle][thermal-reticle-moving-v5-phase]") {
    using namespace fsim::ms::components;
    using namespace fsim::ms::components::moving_radiator_v2;
    namespace m=fsim::ms::components::measured;
    auto c=reticlePolicy();c.detector.width=c.detector.height=8;c.periods=3;c.samplesPerScan=24;c.spatialSamples=2;
    c.scanRadiusRad=.003;c.stopRadiusRad=.006;
    const auto& p=c.detector;ThermalReticleReceiverV1 receiver;std::string error;
    REQUIRE(receiver.configure(c,error));
    ThermalReticleResultV1 clock;clock.valid=true;clock.complete=true;clock.timeS=.05;
    clock.exposureStartTimeS=.04;clock.exposureEndTimeS=.05;clock.measurementTimeS=.045;
    clock.varianceBoundElectrons2=1;clock.measuredElectrons.assign(c.samplesPerScan,0);
    const fsim::ms::weapon::V3 pointing{1,0,0},up{0,0,1};
    const Shape shape{{-.0015,-.0008,.0015,.0008,.003,.003,0}};
    const auto basis=receiver.movingBasisForCalibrationV5(clock,pointing,up,shape);REQUIRE(basis);
    const double first=clock.exposureStartTimeS+.5/(c.frequencyHz*c.samplesPerScan)+p.integrationTimeS*(1./p.exposureSamples-.5);
    const double last=clock.exposureStartTimeS+(c.samplesPerScan-.5)/(c.frequencyHz*c.samplesPerScan)+.5*p.integrationTimeS;
    const auto route=endpointPath(m::unit({1,std::tan(shape[0]),std::tan(shape[1])}),
        m::unit({1,std::tan(shape[2]),std::tan(shape[3])}),first-clock.timeS,last-clock.timeS,p.integrationTimeS,0);
    REQUIRE(route);const auto full=ThermalReticleReceiverV1::unitGroundFieldV1(p,pointing,up,shape[6]);REQUIRE(full);
    namespace v4=fsim::ms::components::conic_moments_v4;
    v4::Request request;request.width=p.width;request.height=p.height;
    request.pixelPitchRad=p.pixelPitchRad;request.psfSigmaRad=p.psfSigmaRad;
    request.pointing={{pointing.x,pointing.y,pointing.z}};request.up={{up.x,up.y,up.z}};request.cutoff=shape[6];
    auto bank=v4::Bank::prepare(request);REQUIRE(bank);
    const auto legacy=receiver.movingBasisForCalibrationV2(clock,pointing,up,shape);REQUIRE(legacy);
    CHECK((*legacy)[1]==(*basis)[1]); // The source radiator operator is unchanged.
    std::vector<double> reference(c.samplesPerScan,0);
    const auto f=[](double x,double y){return std::atan(x*y/std::sqrt(1+x*x+y*y));};
    for(unsigned j=0;j<c.samplesPerScan;++j)for(unsigned k=0;k<p.exposureSamples;++k) {
        const double state=clock.exposureStartTimeS+(j+.5)/(c.frequencyHz*c.samplesPerScan)+
            p.integrationTimeS*((k+.5)/p.exposureSamples-.5)+.5*p.integrationTimeS/p.exposureSamples;
        const auto photon=m::unit(m::subtract(directionAt(*route,state-clock.timeS),m::scale(route->readDerivative,.5*p.integrationTimeS/p.exposureSamples)));
        const double x=std::atan2(photon.y,photon.x),y=std::atan2(photon.z,photon.x);
        const std::array<double,4> region{{x-.5*shape[4],x+.5*shape[4],y-.5*shape[5],y+.5*shape[5]}};
        const auto occulted=bank->evaluateFastMoments(region);REQUIRE(occulted);
        // Independently materialized V5 regional fields with the frozen
        // pixel/subexposure accumulation verify the explicit operator boundary.
        for(unsigned row=0;row<p.height;++row)for(unsigned col=0;col<p.width;++col) {
            const auto i=static_cast<std::size_t>(row)*p.width+col;
            REQUIRE(occulted->absoluteError[i]<=v4::fieldBudget);
            const double cx=(col+.5-.5*p.width)*p.pixelPitchRad,cy=(row+.5-.5*p.height)*p.pixelPitchRad;
            const double xl=std::tan(cx-p.pixelPitchRad/2),xr=std::tan(cx+p.pixelPitchRad/2);
            const double yl=std::tan(cy-p.pixelPitchRad/2),yr=std::tan(cy+p.pixelPitchRad/2);
            const double solid=f(xr,yr)-f(xl,yr)-f(xr,yl)+f(xl,yl);
            const double phase=2*pi*((j+.5)/c.samplesPerScan+c.frequencyHz*p.integrationTimeS*((k+.5)/p.exposureSamples-.5));
            double weight=0;
            for(unsigned sy=0;sy<c.spatialSamples;++sy)for(unsigned sx=0;sx<c.spatialSamples;++sx)
                weight+=ThermalReticleReceiverV1::maskV1(c,cx+((sx+.5)/c.spatialSamples-.5)*p.pixelPitchRad,
                    cy+((sy+.5)/c.spatialSamples-.5)*p.pixelPitchRad,phase);
            weight/=static_cast<double>(c.spatialSamples)*c.spatialSamples;
            const double field=(*full)[i]-occulted->field[i];
            reference[j]+=field*solid*weight/(p.pixelPitchRad*p.pixelPitchRad)/p.exposureSamples;
        }
    }
    const double alpha=-std::expm1(-1/(c.frequencyHz*c.samplesPerScan*p.pixelResponseTimeS));double previous=0;
    for(auto& value:reference){previous+=alpha*(value-previous);value=previous;}
    CHECK(reference==(*basis)[0]);
    CHECK(receiver.latest()==nullptr);
}

#endif
