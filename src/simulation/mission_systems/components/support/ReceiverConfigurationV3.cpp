#include "ReceiverConfigurationV3.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h"
#include "contracts/equipment/EquipmentModels.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace fsim::ms::components {
bool receiverConfigurationV3(const DescriptorV1& original,DescriptorV1& scalar,ReceiverConfigurationV3& output,std::string& error) {
    const auto selection=original.bindings.find("receiver_model");
    if(selection==original.bindings.end()||selection->second!="thermal_conscan_reticle.v1") {
        ReceiverConfigurationV3 next;
        if(!receiverConfigurationV2(original,scalar,next.inherited,error))return false;
        output=std::move(next);return true;
    }
    try {
        auto d=original;
        if(const auto* model=equipment::component(d.implementationId)) {
            if(model->kind!=d.kind)throw std::runtime_error("reticle equipment role mismatch");
            d.implementationId=std::string(model->mechanismId);
        }
        if(d.kind!="seeker"||d.implementationId!="fsim.seeker.nonimaging_ir.v3"||d.configurationVersion!=1||d.availability!=AvailabilityV1::Available)
            throw std::runtime_error("reticle requires an available finite nonimaging IR seeker");
        const auto binding=[&](const char* key,const char* value,bool erase=true) {
            const auto at=d.bindings.find(key);
            if(at==d.bindings.end()||at->second!=value)throw std::runtime_error(std::string("reticle requires ")+key+"="+value);
            if(erase)d.bindings.erase(at);
        };
        binding("mode","infrared",false);binding("navigation_frame","strapdown_body_ecef.v1");
        binding("reticle_pattern","stationary_radial_spokes.v1");binding("reticle_design","blur_response_limited.v1");
        d.bindings.erase("receiver_model");
        ReceiverConfigurationV3 next;next.reticle.emplace();auto& policy=next.reticle->policy;auto& c=policy.detector;auto& s=next.reticle->scene;
        policy.version=1;c.version=1;
        const auto number=[&](const char* role,const char* unit,bool erase=true) {
            double value=0;
            if(!parameterV1(original,role,unit,value,error))throw std::runtime_error(error);
            const auto& p=original.parameters.at(role);
            if(!std::isfinite(value)||!std::isfinite(p.uncertainty)||p.uncertainty<=0)
                throw std::runtime_error(std::string("reticle requires finite uncertainty: ")+role);
            if(erase){d.parameters.erase(role);d.parameterRefs.erase(role);}return value;
        };
        const auto integer=[&](const char* role) {
            const double value=number(role,"1");
            if(value<1||value>100000||value!=std::floor(value))throw std::runtime_error("reticle requires bounded integer design");
            return static_cast<unsigned>(value);
        };
        c.width=integer("receiver_image_width");c.height=integer("receiver_image_height");c.adcBits=integer("receiver_adc_bits");c.exposureSamples=integer("receiver_exposure_samples");
        const struct {const char* role;const char* unit;double ThermalImageConfigV1::*member;} fields[]={
            {"receiver_pixel_pitch_rad","rad",&ThermalImageConfigV1::pixelPitchRad},{"receiver_psf_sigma_rad","rad",&ThermalImageConfigV1::psfSigmaRad},
            {"receiver_aperture_area_m2","m^2",&ThermalImageConfigV1::apertureAreaM2},{"receiver_optical_transmission","1",&ThermalImageConfigV1::opticalTransmission},
            {"receiver_quantum_efficiency","1",&ThermalImageConfigV1::quantumEfficiency},{"receiver_effective_wavelength_m","m",&ThermalImageConfigV1::effectiveWavelengthM},
            {"receiver_integration_s","s",&ThermalImageConfigV1::integrationTimeS},{"receiver_pixel_response_s","s",&ThermalImageConfigV1::pixelResponseTimeS},
            {"receiver_read_noise_electrons","electron",&ThermalImageConfigV1::readNoiseElectrons},{"receiver_dark_current_electrons_s","electron/s",&ThermalImageConfigV1::darkCurrentElectronsS},
            {"receiver_full_well_electrons","electron",&ThermalImageConfigV1::fullWellElectrons},{"receiver_detection_sigma","1",&ThermalImageConfigV1::detectionSigma},
            {"receiver_background_radius_pixels","1",&ThermalImageConfigV1::backgroundRadiusPixels},{"receiver_minimum_feature_pixels","1",&ThermalImageConfigV1::minimumFeaturePixels},
            {"receiver_association_gate_rad","rad",&ThermalImageConfigV1::associationGateRad}};
        for(const auto& field:fields)c.*field.member=number(field.role,field.unit);
        s.lowUm=number("ir_band_low_um","um",false);s.highUm=number("ir_band_high_um","um",false);s.extinctionPerM=number("ir_extinction_m_1","1/m",false);
        const struct {const char* role;const char* unit;double ReceiverScenePolicyV1::*member;} scene[]={
            {"scene_sky_temperature_k","K",&ReceiverScenePolicyV1::skyTemperatureK},{"scene_ground_temperature_k","K",&ReceiverScenePolicyV1::groundTemperatureK},
            {"scene_body_emissivity","1",&ReceiverScenePolicyV1::bodyEmissivity},{"scene_skin_recovery_factor","1",&ReceiverScenePolicyV1::skinRecoveryFactor},
            {"scene_engine_heat_fraction","1",&ReceiverScenePolicyV1::engineHeatFraction},{"scene_exhaust_emissivity","1",&ReceiverScenePolicyV1::exhaustEmissivity},
            {"scene_exhaust_area_scale","1",&ReceiverScenePolicyV1::exhaustAreaScale},{"scene_flare_temperature_k","K",&ReceiverScenePolicyV1::flareTemperatureK},
            {"scene_flare_emissivity","1",&ReceiverScenePolicyV1::flareEmissivity},{"scene_flare_initial_radius_m","m",&ReceiverScenePolicyV1::flareInitialRadiusM},
            {"scene_flare_expansion_m_s","m/s",&ReceiverScenePolicyV1::flareExpansionMs},{"scene_flare_cooling_s","s",&ReceiverScenePolicyV1::flareCoolingS}};
        for(const auto& field:scene)s.*field.member=number(field.role,field.unit);
        policy.periods=integer("receiver_reticle_periods");policy.samplesPerScan=integer("receiver_reticle_samples_per_scan");
        policy.spatialSamples=integer("receiver_reticle_spatial_samples");
        policy.scanRadiusRad=number("receiver_reticle_scan_radius_rad","rad");policy.stopRadiusRad=number("receiver_reticle_stop_radius_rad","rad");
        policy.frequencyHz=number("receiver_reticle_frequency_hz","Hz");
        constexpr double pi=3.14159265358979323846;
        const double stop=.5*std::min(c.width,c.height)*c.pixelPitchRad,radius=.5*stop;
        const double periods=std::max(1.,std::floor(pi*radius/(8*c.psfSigmaRad)));
        if(!ThermalReticleReceiverV1::validConfig(policy)||policy.stopRadiusRad!=stop||policy.scanRadiusRad!=radius||policy.periods!=periods||
            policy.frequencyHz!=1/(2*pi*policy.periods*c.pixelResponseTimeS)||
            s.lowUm<=0||s.highUm<=s.lowUm||s.extinctionPerM<0||s.skyTemperatureK<=0||s.groundTemperatureK<=0||s.flareTemperatureK<=0||
            s.skyTemperatureK>5000||s.groundTemperatureK>5000||s.flareTemperatureK>5000||s.bodyEmissivity<=0||s.bodyEmissivity>1||
            s.exhaustEmissivity<=0||s.exhaustEmissivity>1||s.flareEmissivity<=0||s.flareEmissivity>1||s.skinRecoveryFactor<=0||s.skinRecoveryFactor>1||
            s.engineHeatFraction<=0||s.engineHeatFraction>1||s.exhaustAreaScale<=0||s.exhaustAreaScale>1||s.flareInitialRadiusM<=0||s.flareExpansionMs<=0||s.flareCoolingS<=0)
            throw std::runtime_error("reticle physical/design policy outside declared domain");
        DescriptorV1 admitted;
        if(!receiverConfigurationV2(d,admitted,next.inherited,error))return false;
        scalar=std::move(admitted);output=std::move(next);return true;
    } catch(const std::exception& failure){error=failure.what();return false;}
}
}
