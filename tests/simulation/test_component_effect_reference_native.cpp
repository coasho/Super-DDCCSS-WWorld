#include "simulation/mission_systems/components/Providers.h"
#include "simulation/mission_systems/weapon/Composition.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h"
#include "foundation/core/Json.h"
#include "ComponentReferenceRuntimeV1.h"
#include <picosha2.h>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

// These are referee method observations of the actual selected native
// providers, not replacement providers or an equipment calibration fixture.
// Independent primary-equation comparisons are made by the companion Python
// tool. The original composition and every card parameter remain untouched.
namespace {
using namespace fsim::ms;
using namespace fsim::ms::components;
using J=fsim::core::Json;
std::string hashFile(const std::filesystem::path& path) {
    REQUIRE(std::filesystem::file_size(path)<=512ULL*1024*1024);
    std::ifstream file(path,std::ios::binary);REQUIRE(file.good());
    picosha2::hash256_one_by_one hasher;
    char buffer[65536];
    while(file.read(buffer,sizeof(buffer))||file.gcount()>0)
        hasher.process(buffer,buffer+file.gcount());
    REQUIRE(file.eof());hasher.finish();
    return picosha2::get_hash_hex_string(hasher);
}
std::filesystem::path runningExecutable() {
#ifdef _WIN32
    std::vector<wchar_t> buffer(32768);
    const auto length=GetModuleFileNameW(nullptr,buffer.data(),static_cast<DWORD>(buffer.size()));
    REQUIRE(length>0);REQUIRE(length<buffer.size());
    return std::filesystem::path(std::wstring(buffer.data(),length));
#else
    return std::filesystem::canonical("/proc/self/exe");
#endif
}
J proximity(weapon::FuzeAdapter& provider,double scale) {
    const auto& s=provider.spec();
    const double age=s.armingS+.25,duration=1.,miss=scale*s.fuzeM;
    REQUIRE(age+duration<s.lifeS);
    provider.resetRelease();
    const std::vector<weapon::MotionSample> own{{0,{100,miss,0}},{duration,{-100,miss,0}}};
    const std::vector<weapon::MotionSample> target{{0,{0,0,0}},{duration,{0,0,0}}};
    const auto r=provider.evaluateProximity(own,target,age,duration,duration,false,7);
    REQUIRE(r.valid);REQUIRE(std::isfinite(r.fraction));REQUIRE(std::isfinite(r.missM));
    J out;out.set("radius_m",s.fuzeM).set("arming_s",s.armingS).set("life_s",s.lifeS)
        .set("delay_s",s.fuzeDelayS).set("age_s",age).set("duration_s",duration)
        .set("scale",scale).set("x0_m",100.).set("x1_m",-100.).set("y_m",miss)
        .set("fraction",r.fraction).set("miss_m",r.missM).set("triggered",r.triggered);
    return out;
}
J endpointApproach(weapon::FuzeAdapter& provider) {
    const auto& s=provider.spec();const double age=s.armingS+.25;
    provider.resetRelease();
    const std::vector<weapon::MotionSample> own{{0,{100,0,0}},{1,{.5*s.fuzeM,0,0}}};
    const std::vector<weapon::MotionSample> target{{0,{0,0,0}},{1,{0,0,0}}};
    const auto r=provider.evaluateProximity(own,target,age,1,1,false,7);
    REQUIRE(r.valid);REQUIRE(std::isfinite(r.missM));
    J out;out.set("model_version",static_cast<double>(s.fuzeModelVersion))
        .set("radius_m",s.fuzeM).set("miss_m",r.missM).set("fraction",r.fraction)
        .set("triggered",r.triggered);return out;
}
J emission(const damage::WarheadAdapter& adapter) {
    const auto* scene=dynamic_cast<const damage::WarheadSceneExecutionV2*>(&adapter);
    REQUIRE(scene);fsim::Rng rng(1943);
    damage::Burst burst;burst.axis={1,0,0};
    const auto field=scene->emitScene(burst,rng);REQUIRE(field.valid);REQUIRE_FALSE(field.projectiles.empty());
    long double mass=0,energy=0;damage::V3 momentum;
    double minRadius=std::numeric_limits<double>::infinity(),maxRadius=0,maxAxial=0;
    double minimumSpeed=std::numeric_limits<double>::infinity(),maximumSpeed=0,ringGap=0;
    double minMaterialLength=std::numeric_limits<double>::infinity(),maxMaterialLength=0;
    int segments=0,expanding=0;double expansion=0,breakup=0;
    const auto radius=[](damage::V3 p){return std::hypot(p.y,p.z);};
    for(std::size_t i=0;i<field.projectiles.size();++i) {
        const auto& p=field.projectiles[i];mass+=p.massKg;
        minRadius=std::min(minRadius,radius(p.at));maxRadius=std::max(maxRadius,radius(p.at));
        maxAxial=std::max(maxAxial,std::abs(p.at.x));
        minimumSpeed=std::min(minimumSpeed,damage::length(p.velocity));
        maximumSpeed=std::max(maximumSpeed,damage::length(p.velocity));
        if(p.segment) {
            ++segments;energy+=static_cast<long double>(p.massKg)/6*(damage::dot(p.velocity,p.velocity)+
                damage::dot(p.velocity,p.endVelocity)+damage::dot(p.endVelocity,p.endVelocity));
            momentum+=(p.velocity+p.endVelocity)*(p.massKg/2);
        } else {energy+=static_cast<long double>(p.massKg)/2*damage::dot(p.velocity,p.velocity);momentum+=p.velocity*p.massKg;}
        if(p.expansionPerS>0) {
            ++expanding;expansion=p.expansionPerS;breakup=p.breakupAgeS;
            minMaterialLength=std::min(minMaterialLength,p.materialLengthM);
            maxMaterialLength=std::max(maxMaterialLength,p.materialLengthM);
            ringGap=std::max(ringGap,damage::length(p.end-field.projectiles[(i+1)%field.projectiles.size()].at));
        }
    }
    const auto& w=adapter.model();J out;
    out.set("count",static_cast<double>(field.projectiles.size())).set("mass_kg",static_cast<double>(mass))
        .set("energy_j",static_cast<double>(energy)).set("momentum_norm_ns",damage::length(momentum))
        .set("segments",segments).set("expanding",expanding).set("ring_gap_m",ringGap)
        .set("minimum_speed_ms",minimumSpeed).set("maximum_speed_ms",maximumSpeed)
        .set("minimum_radius_m",minRadius).set("maximum_radius_m",maxRadius).set("maximum_axial_m",maxAxial)
        .set("outer_radius_m",w.diameterM/2).set("inner_radius_m",w.caseInnerRadiusM>0?w.caseInnerRadiusM:
            std::max(0.,w.diameterM/2-w.caseThicknessM)).set("length_m",w.lengthM)
        .set("expansion_per_s",expansion).set("breakup_s",breakup)
        .set("minimum_material_length_m",expanding?minMaterialLength:0.)
        .set("maximum_material_length_m",maxMaterialLength);
    return out;
}
}

TEST_CASE("Actual selected AAM effects methods expose primary-reference observations",
          "[component-effects-reference]") {
    const char* directory=std::getenv("FSIM_COMPONENT_EFFECT_REFERENCE_DIR");
    REQUIRE(directory);const std::filesystem::path output(directory);
    REQUIRE(std::filesystem::is_directory(output));
    const char* epochFile=std::getenv("FSIM_COMPONENT_EFFECT_REFERENCE_EPOCH");REQUIRE(epochFile);
    const std::filesystem::path epoch(epochFile);
    REQUIRE(std::filesystem::is_regular_file(epoch));REQUIRE(std::filesystem::file_size(epoch)<=1024*1024);
    const auto epochHash=hashFile(epoch);
    const auto executable=runningExecutable();
    const auto runtime=fsim::testing::componentReferenceRuntimeV1(hashFile);
    J executor;executor.set("path",executable.generic_string()).set("sha256",hashFile(executable));
    const std::filesystem::path library(FSIM_TEST_STORE_DIR);
    std::vector<std::string> errors;std::string error;
    const auto cards=readStoreLibrary(library,errors);REQUIRE(errors.empty());
    J::Array rows;
    for(const auto& [id,card]:cards)if(card->storeClass=="aam") {
        INFO(id);ProfileV1 profile;
        REQUIRE(readProfileV1(library/(id+".json"),*card,profile,error));
        weapon::Spec spec;REQUIRE(weapon::configureCompositionV1(*card,profile,spec,spec,error));
        const ProviderContextV1 context{*card,profile,spec,std::nullopt};
        auto seeker=builtInProvidersV1().seeker.create(*profile.find("seeker"),context,error);
        auto fuze=builtInProvidersV1().fuze.create(*profile.find("fuze"),context,error);
        auto warhead=builtInProvidersV1().warhead.create(*profile.find("warhead"),context,error);
        INFO(error);REQUIRE(seeker);REQUIRE(fuze);REQUIRE(warhead);
        DescriptorV1 normalized;ReceiverConfigurationV2 receiver;
        REQUIRE(receiverConfigurationV2(*profile.find("seeker"),normalized,receiver,error));
        REQUIRE(receiver.single.kind!=PhysicalReceiverKindV1::Scalar);
        J row;row.set("model",id).set("card_sha256",hashFile(library/(id+".json")))
            .set("seeker_implementation",profile.find("seeker")->implementationId)
            .set("fuze_implementation",std::string(fuze->implementationId()))
            .set("warhead_implementation",std::string(warhead->implementationId()));
        J::Array measurements;
        if(receiver.single.kind==PhysicalReceiverKindV1::Radio) {
            const auto& c=receiver.single.radio;
            const rf::Pattern pattern(c.aperture,c.frequencyHz);
            const double direction[3]{1,0,0};const double gain=pattern.gain(direction);
            const double dwell=c.observationWindowS;
            REQUIRE(gain>0);REQUIRE(dwell>0);
            // Separate transmitter input is a declared numerical test source,
            // not a claim about any OEM launcher or illumination fit.
            for(const bool semi:{false,true}) {
                RadioEchoSceneV1 echo;echo.receiverRangeM=5000;echo.transmitterRangeM=10000;
                echo.rcsM2=1;echo.receiverExcessLossDb=.3;echo.transmitterExcessLossDb=.5;
                echo.transmitterPowerW=1000;echo.transmitterGainLinear=100;
                const double actual=RadioReceiverV1::energySnr(c,echo,gain,c.frequencyHz,dwell,semi);
                REQUIRE(std::isfinite(actual));REQUIRE(actual>0);
                J m;m.set("law","radar_energy").set("semi_active",semi)
                    .set("frequency_hz",c.frequencyHz).set("temperature_k",c.temperatureK)
                    .set("noise_figure_db",c.noiseFigureDb).set("system_loss_db",c.systemLossDb)
                    .set("receiver_gain",gain).set("dwell_s",dwell)
                    .set("power_w",semi?echo.transmitterPowerW:c.transmitAveragePowerW)
                    .set("transmitter_gain",semi?echo.transmitterGainLinear:gain)
                    .set("transmitter_range_m",semi?echo.transmitterRangeM:echo.receiverRangeM)
                    .set("receiver_range_m",echo.receiverRangeM).set("rcs_m2",echo.rcsM2)
                    .set("receiver_excess_loss_db",echo.receiverExcessLossDb)
                    .set("transmitter_excess_loss_db",semi?echo.transmitterExcessLossDb:echo.receiverExcessLossDb)
                    .set("actual",actual);measurements.emplace_back(std::move(m));
            }
        } else {
            std::vector<ReceiverScenePolicyV1> bands{receiver.single.scene};
            if(receiver.secondBandScene)bands.push_back(*receiver.secondBandScene);
            for(const auto& band:bands)for(const double temperature:{300.,600.,1000.,2000.}) {
                const double actual=ThermalImageReceiverV1::bandRadiance(temperature,.8,band.lowUm,band.highUm);
                REQUIRE(std::isfinite(actual));REQUIRE(actual>0);
                J m;m.set("law","planck_band_radiance").set("temperature_k",temperature)
                    .set("emissivity",.8).set("low_um",band.lowUm).set("high_um",band.highUm)
                    .set("actual",actual);measurements.emplace_back(std::move(m));
            }
            // Constant radiance has an independent photon-budget/solid-angle
            // solution for either the quadrant or imaging detector. Observe
            // expected charge before random shot/read noise and ADC clipping.
            const auto& c=receiver.single.thermal;
            ThermalImageReceiverV1 optical;REQUIRE(optical.configure(c,error));
            ThermalSceneFrameV1 scene;scene.timeS=1;scene.up={0,0,1};
            scene.skyRadianceWm2Sr=1;scene.groundRadianceWm2Sr=1;
            const bool imaging=receiver.single.kind==PhysicalReceiverKindV1::Image;
            const auto image=optical.expose(scene,{1,0,0},2005,imaging);REQUIRE(image.valid);
            const unsigned x=c.width/2,y=c.height/2;
            const auto index=static_cast<std::size_t>(y)*c.width+x;
            REQUIRE(index<image.expectedElectrons.size());
            J m;m.set("law","uniform_detector_charge").set("width",static_cast<double>(c.width))
                .set("height",static_cast<double>(c.height)).set("column",static_cast<double>(x))
                .set("row",static_cast<double>(y)).set("imaging",imaging)
                .set("pixel_pitch_rad",c.pixelPitchRad).set("aperture_m2",c.apertureAreaM2)
                .set("transmission",c.opticalTransmission).set("integration_s",c.integrationTimeS)
                .set("response_s",c.pixelResponseTimeS).set("quantum_efficiency",c.quantumEfficiency)
                .set("wavelength_m",c.effectiveWavelengthM).set("dark_electrons_s",c.darkCurrentElectronsS)
                .set("radiance_w_m2_sr",1.).set("actual",image.expectedElectrons[index]);
            measurements.emplace_back(std::move(m));
        }
        row.set("seeker",J(std::move(measurements)));
        row.set("fuze",J::Array{proximity(*fuze,.5),proximity(*fuze,1.5)});
        row.set("fuze_endpoint",endpointApproach(*fuze));
        const auto& w=warhead->model();
        const double cylinder=damage::gurneyCylinder(w.gurneyMs,w.caseKg,w.explosiveKg);
        REQUIRE(std::isfinite(cylinder));REQUIRE(cylinder>0);REQUIRE(w.speedMs>0);
        J wm;wm.set("gurney_ms",w.gurneyMs).set("case_kg",w.caseKg).set("charge_kg",w.explosiveKg)
            .set("actual",cylinder).set("source_speed_ms",w.speedMs)
            .set("model_version",static_cast<double>(w.modelVersion))
            .set("fragment_source",static_cast<int>(w.fragmentSource))
            .set("rod",w.kind==damage::Warhead::Kind::Rod)
            .set("fragment_count",w.fragments).set("fragment_kg",w.fragmentKg)
            .set("fragment_area_m2",w.fragmentAreaM2).set("tnt_kg",w.tntKg);
        wm.set("emission",emission(*warhead));
        row.set("warhead",std::move(wm));rows.emplace_back(std::move(row));
    }
    REQUIRE(rows.size()==24);
    J document;document.set("schema","fsim.sim-m03.component-effects-native.v1")
        .set("complete",true).set("models",J(std::move(rows)))
        .set("input_epoch_sha256",epochHash).set("executable",std::move(executor)).set("runtime_modules",runtime)
        .set("scope","Native selected-provider method observations; not equipment calibration or public/visual acceptance.");
    std::ofstream file(output/"component-effects-native.json",std::ios::binary|std::ios::trunc);
    REQUIRE(file.good());file<<document.dump()<<'\n';file.flush();REQUIRE(file.good());
}
