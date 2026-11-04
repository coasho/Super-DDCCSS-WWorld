#include "ReceiverConfigurationV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h"
#include "contracts/equipment/EquipmentModels.h"
#include <cmath>
#include <stdexcept>

namespace fsim::ms::components {
bool receiverConfigurationV1(const DescriptorV1& descriptor,DescriptorV1& scalar,ReceiverConfigurationV1& output,std::string& error) {
    if(const auto* model=equipment::component(descriptor.implementationId)) {
        if(model->kind!=descriptor.kind){error="named receiver has the wrong component role";return false;}
        auto normalized=descriptor;normalized.implementationId=std::string(model->mechanismId);
        return receiverConfigurationV1(normalized,scalar,output,error);
    }
    try {
        const auto selection=descriptor.bindings.find("receiver_model");
        if(selection==descriptor.bindings.end()){scalar=descriptor;output={};return true;}
        auto normalized=descriptor;ReceiverConfigurationV1 next;
        const auto binding=[&](const char* name,const char* expected){
            const auto at=descriptor.bindings.find(name);
            if(at==descriptor.bindings.end()||at->second!=expected)throw std::runtime_error(std::string("physical receiver needs ")+name+"="+expected);
            normalized.bindings.erase(name);
        };
        binding("navigation_frame","strapdown_body_ecef.v1");normalized.bindings.erase("receiver_model");
        const auto mode=descriptor.bindings.find("mode");
        if(descriptor.configurationVersion!=1||descriptor.availability!=AvailabilityV1::Available||mode==descriptor.bindings.end()||
            descriptor.implementationId.size()<3||descriptor.implementationId.substr(descriptor.implementationId.size()-3)!=".v3")
            throw std::runtime_error("physical receiver requires an available finite V3 descriptor");
        const auto number=[&](const char* role,const char* unit,bool extension=true){
            double value=0;
            if(!parameterV1(descriptor,role,unit,value,error))throw std::runtime_error(error);
            const auto& p=descriptor.parameters.at(role);
            if(!std::isfinite(p.uncertainty)||p.uncertainty<=0)throw std::runtime_error(std::string("physical receiver needs explicit uncertainty: ")+role);
            if(extension){normalized.parameters.erase(role);normalized.parameterRefs.erase(role);}
            return value;
        };
        const auto integer=[&](const char* role){
            const double value=number(role,"1");
            if(value<0||value>100000||value!=std::floor(value))throw std::runtime_error(std::string("physical receiver requires bounded integer ")+role);
            return static_cast<unsigned>(value);
        };
        if(selection->second=="thermal_image.v1"||selection->second=="thermal_quadrant.v1") {
            const bool image=selection->second=="thermal_image.v1";
            if(mode->second!=(image?"imaging_infrared":"infrared"))throw std::runtime_error("thermal receiver family mismatch");
            next.kind=image?PhysicalReceiverKindV1::Image:PhysicalReceiverKindV1::Quadrant;auto& c=next.thermal;c.version=1;
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
            auto& s=next.scene;s.lowUm=number("ir_band_low_um","um",false);s.highUm=number("ir_band_high_um","um",false);s.extinctionPerM=number("ir_extinction_m_1","1/m",false);
            const struct {const char* role;const char* unit;double ReceiverScenePolicyV1::*member;} sceneFields[]={
                {"scene_sky_temperature_k","K",&ReceiverScenePolicyV1::skyTemperatureK},{"scene_ground_temperature_k","K",&ReceiverScenePolicyV1::groundTemperatureK},
                {"scene_body_emissivity","1",&ReceiverScenePolicyV1::bodyEmissivity},{"scene_skin_recovery_factor","1",&ReceiverScenePolicyV1::skinRecoveryFactor},
                {"scene_engine_heat_fraction","1",&ReceiverScenePolicyV1::engineHeatFraction},{"scene_exhaust_emissivity","1",&ReceiverScenePolicyV1::exhaustEmissivity},
                {"scene_exhaust_area_scale","1",&ReceiverScenePolicyV1::exhaustAreaScale},{"scene_flare_temperature_k","K",&ReceiverScenePolicyV1::flareTemperatureK},
                {"scene_flare_emissivity","1",&ReceiverScenePolicyV1::flareEmissivity},{"scene_flare_initial_radius_m","m",&ReceiverScenePolicyV1::flareInitialRadiusM},
                {"scene_flare_expansion_m_s","m/s",&ReceiverScenePolicyV1::flareExpansionMs},{"scene_flare_cooling_s","s",&ReceiverScenePolicyV1::flareCoolingS}};
            for(const auto& field:sceneFields)s.*field.member=number(field.role,field.unit);
            if(!ThermalImageReceiverV1::validConfig(c)||s.skyTemperatureK<=0||s.groundTemperatureK<=0||s.flareTemperatureK<=0||
                s.skyTemperatureK>5000||s.groundTemperatureK>5000||s.flareTemperatureK>5000||s.bodyEmissivity<=0||s.bodyEmissivity>1||
                s.exhaustEmissivity<=0||s.exhaustEmissivity>1||s.flareEmissivity<=0||s.flareEmissivity>1||
                s.skinRecoveryFactor<=0||s.skinRecoveryFactor>1||s.engineHeatFraction<=0||s.engineHeatFraction>1||
                s.exhaustAreaScale<=0||s.exhaustAreaScale>1||s.flareInitialRadiusM<=0||s.flareExpansionMs<=0||s.flareCoolingS<=0)
                throw std::runtime_error("physical thermal/scene policy outside admitted domain");
        } else if(selection->second=="rf_signal.v1") {
            if(mode->second!="active_radar"&&mode->second!="semi_active_radar")throw std::runtime_error("radio receiver family mismatch");
            next.kind=PhysicalReceiverKindV1::Radio;auto& c=next.radio;c.version=1;
            binding("rf_aperture","circular_uniform.v1");binding("rf_waveform","medium_prf.v1");
            c.frequencyHz=number("frequency_hz","Hz",false);c.temperatureK=number("temperature_k","K",false);
            c.noiseFigureDb=number("noise_figure_db","dB",false);c.systemLossDb=number("system_loss_db","dB",false);
            c.transmitAveragePowerW=number("emitter_power_w","W",false);
            c.temporalMemoryS=number("track_memory_s","s",false);
            c.aperture.shape=rf::ApertureShape::Circular;c.aperture.taper=rf::Taper::Uniform;c.aperture.electronic=false;
            c.aperture.widthM=number("receiver_rf_aperture_width_m","m");c.aperture.heightM=number("receiver_rf_aperture_height_m","m");
            c.aperture.efficiency=number("receiver_rf_aperture_efficiency","1");c.aperture.pedestal=number("receiver_rf_aperture_pedestal","1");
            c.aperture.peakGainDbi=number("receiver_rf_aperture_peak_gain_dbi","dBi");c.aperture.floorDbi=number("receiver_rf_aperture_floor_dbi","dBi");
            c.aperture.backDbi=number("receiver_rf_aperture_back_dbi","dBi");c.aperture.taylorSidelobeDb=number("receiver_rf_taylor_sidelobe_db","dB");
            c.aperture.taylorNbar=static_cast<int>(integer("receiver_rf_taylor_nbar"));c.aperture.scanExponent=number("receiver_rf_scan_exponent","1");
            const double dwell=number("receiver_rf_dwell_s","s"),gate=number("receiver_rf_gate_m","m"),duty=number("receiver_rf_duty","1");
            c.observationWindowS=dwell;
            const unsigned prfs=integer("receiver_rf_prfs"),m=integer("receiver_rf_coincidence_m");
            if(dwell<=0||dwell>.25||gate<=0||duty<=0||duty>=1||prfs<1||prfs>5||m<1||m>prfs)throw std::runtime_error("invalid explicit RF waveform design");
            c.waveform=radar::signal::mediumPrf(c.frequencyHz,dwell,static_cast<int>(prfs),static_cast<int>(m),gate,duty);
            c.bandwidthHz=299792458./(2*gate);
            c.processing.pfaInterval=number("receiver_rf_pfa_interval","1");c.processing.cfarCells=static_cast<int>(integer("receiver_rf_cfar_cells"));
            c.processing.cfarGuard=static_cast<int>(integer("receiver_rf_cfar_guard"));c.processing.coincidence=static_cast<int>(integer("receiver_rf_coincidence_cells"));
            c.processing.rangeScaleM=number("receiver_rf_range_m","m");c.processing.minRateMs=number("receiver_rf_min_rate_m_s","m/s");
            c.processing.maxRateMs=number("receiver_rf_max_rate_m_s","m/s");c.processing.notchMs=number("receiver_rf_notch_m_s","m/s");
            c.processing.knownNoiseThreshold=false;c.processing.reportAmbiguities=true;c.processing.velocityOnly=false;
            c.monopulseSlope=number("receiver_rf_monopulse_slope","1");c.clutterRateSigmaMs=number("receiver_rf_clutter_sigma_m_s","m/s");
            c.associationGateRad=number("receiver_rf_association_gate_rad","rad");
            auto& s=next.scene;s.groundSigma0Land=number("scene_ground_sigma0_land","1");s.groundSigma0Sea=number("scene_ground_sigma0_sea","1");
            s.groundGrazingExponent=number("scene_ground_grazing_exponent","1");s.groundRangeM=number("scene_ground_range_m","m");
            s.groundRangeBands=integer("scene_ground_range_bands");s.groundAzimuthSamples=integer("scene_ground_azimuth_samples");
            if(!RadioReceiverV1::validConfig(c)||s.groundSigma0Land<=0||s.groundSigma0Sea<=0||s.groundGrazingExponent<=0||s.groundRangeM<=0||
                !s.groundRangeBands||!s.groundAzimuthSamples||s.groundRangeBands>512||s.groundAzimuthSamples>512||
                s.groundRangeBands*s.groundAzimuthSamples>512)throw std::runtime_error("physical RF/scene policy outside admitted domain");
        } else throw std::runtime_error("unknown explicit physical receiver model");
        scalar=std::move(normalized);output=std::move(next);return true;
    } catch(const std::exception& failure){error=failure.what();return false;}
}

bool receiverConfigurationV2(const DescriptorV1& descriptor,DescriptorV1& scalar,ReceiverConfigurationV2& output,std::string& error) {
    const auto selection=descriptor.bindings.find("receiver_model");
    if(selection==descriptor.bindings.end()||selection->second!="thermal_dual_image.v1") {
        ReceiverConfigurationV2 next;
        if(!receiverConfigurationV1(descriptor,scalar,next.single,error))return false;
        output=std::move(next);return true;
    }
    try {
        auto first=descriptor;first.bindings["receiver_model"]="thermal_image.v1";
        ThermalDualBandConfigV1 dual;dual.version=1;auto& c=dual.channels[1];c.version=1;
        const auto number=[&](const std::string& role,const char* unit) {
            double value=0;
            if(!parameterV1(descriptor,role.c_str(),unit,value,error))throw std::runtime_error(error);
            const auto& p=descriptor.parameters.at(role);
            if(!std::isfinite(value)||!std::isfinite(p.uncertainty)||p.uncertainty<=0)
                throw std::runtime_error("dual receiver needs explicit finite value and uncertainty: "+role);
            first.parameters.erase(role);first.parameterRefs.erase(role);return value;
        };
        const auto integer=[&](const char* role) {
            const double value=number(role,"1");
            if(value<0||value>100000||value!=std::floor(value))throw std::runtime_error("dual receiver requires bounded integer");
            return static_cast<unsigned>(value);
        };
        c.width=integer("receiver_band2_image_width");c.height=integer("receiver_band2_image_height");
        c.adcBits=integer("receiver_band2_adc_bits");c.exposureSamples=integer("receiver_band2_exposure_samples");
        const struct {const char* role;const char* unit;double ThermalImageConfigV1::*member;} fields[]={
            {"receiver_band2_pixel_pitch_rad","rad",&ThermalImageConfigV1::pixelPitchRad},
            {"receiver_band2_psf_sigma_rad","rad",&ThermalImageConfigV1::psfSigmaRad},
            {"receiver_band2_aperture_area_m2","m^2",&ThermalImageConfigV1::apertureAreaM2},
            {"receiver_band2_optical_transmission","1",&ThermalImageConfigV1::opticalTransmission},
            {"receiver_band2_quantum_efficiency","1",&ThermalImageConfigV1::quantumEfficiency},
            {"receiver_band2_effective_wavelength_m","m",&ThermalImageConfigV1::effectiveWavelengthM},
            {"receiver_band2_integration_s","s",&ThermalImageConfigV1::integrationTimeS},
            {"receiver_band2_pixel_response_s","s",&ThermalImageConfigV1::pixelResponseTimeS},
            {"receiver_band2_read_noise_electrons","electron",&ThermalImageConfigV1::readNoiseElectrons},
            {"receiver_band2_dark_current_electrons_s","electron/s",&ThermalImageConfigV1::darkCurrentElectronsS},
            {"receiver_band2_full_well_electrons","electron",&ThermalImageConfigV1::fullWellElectrons},
            {"receiver_band2_detection_sigma","1",&ThermalImageConfigV1::detectionSigma},
            {"receiver_band2_background_radius_pixels","1",&ThermalImageConfigV1::backgroundRadiusPixels},
            {"receiver_band2_minimum_feature_pixels","1",&ThermalImageConfigV1::minimumFeaturePixels},
            {"receiver_band2_association_gate_rad","rad",&ThermalImageConfigV1::associationGateRad}};
        for(const auto& field:fields)c.*field.member=number(field.role,field.unit);
        const double low=number("ir_band2_low_um","um"),high=number("ir_band2_high_um","um"),extinction=number("ir_band2_extinction_m_1","1/m");
        dual.angularPairGateRad=number("receiver_dual_pair_gate_rad","rad");
        dual.logRatioGate=number("receiver_dual_log_ratio_gate","1");
        dual.spectralMemoryS=number("receiver_dual_spectral_memory_s","s");
        ReceiverConfigurationV2 next;
        DescriptorV1 normalized;
        if(!receiverConfigurationV1(first,normalized,next.single,error))return false;
        dual.channels[0]=next.single.thermal;
        if(!ThermalDualBandReceiverV1::validConfig(dual)||low<=0||high<=low||high>1000||extinction<0||extinction>1||
            next.single.scene.lowUm<=0||next.single.scene.highUm<=next.single.scene.lowUm||
            (low==next.single.scene.lowUm&&high==next.single.scene.highUm)||
            c.effectiveWavelengthM<low*1e-6||c.effectiveWavelengthM>high*1e-6||
            dual.channels[0].effectiveWavelengthM<next.single.scene.lowUm*1e-6||
            dual.channels[0].effectiveWavelengthM>next.single.scene.highUm*1e-6)
            throw std::runtime_error("dual thermal band/receiver policy outside admitted domain");
        next.dual=dual;next.secondBandScene=next.single.scene;
        next.secondBandScene->lowUm=low;next.secondBandScene->highUm=high;next.secondBandScene->extinctionPerM=extinction;
        scalar=std::move(normalized);output=std::move(next);return true;
    } catch(const std::exception& failure){error=failure.what();return false;}
}
} // namespace fsim::ms::components
