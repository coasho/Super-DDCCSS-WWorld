#include "Seeker.h"
#include "simulation/mission_systems/components/support/EquipmentSelectionV1.h"
#include "simulation/mission_systems/components/Profile.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV1.h"
#include "simulation/mission_systems/components/support/ReceiverConfigurationV3.h"
#include <array>
#include <iterator>

namespace fsim::ms::weapon {
namespace {
constexpr double pi=3.14159265358979323846;
struct Variant { const char* card; const char* name; const char* mode; };
constexpr Variant variants[]={
    {"aim-120c","AIM-120C-7","active_radar"},
    {"aim-120d","AIM-120D","active_radar"},
    {"aim-7m","AIM-7M","semi_active_radar"},
    {"aim-9m","AIM-9M","infrared"},
    {"aim-9x","AIM-9X Block I","imaging_infrared"},
    {"asraam","AIM-132 ASRAAM","imaging_infrared"},
    {"iris-t","IRIS-T","imaging_infrared"},
    {"magic-2","R.550 Magic 2","infrared"},
    {"meteor","Meteor","active_radar"},
    {"mica-em","MICA EM","active_radar"},
    {"mica-ir","MICA IR","imaging_infrared"},
    {"pl-10","PL-10","imaging_infrared"},
    {"pl-12","PL-12","active_radar"},
    {"pl-15","PL-15","active_radar"},
    {"pl-8","PL-8","infrared"},
    {"r-27er","R-27ER","semi_active_radar"},
    {"r-27et","R-27ET","infrared"},
    {"r-27r","R-27R","semi_active_radar"},
    {"r-27t","R-27T","infrared"},
    {"r-37m","R-37M","active_radar"},
    {"r-60m","R-60M","infrared"},
    {"r-73","R-73","infrared"},
    {"r-77m","R-77M","active_radar"},
    {"super-530d","Super 530D","semi_active_radar"}
};
}
bool seekerVariantMatchesV2(const components::DescriptorV1& descriptor,const std::string& storeId) noexcept {
    const auto variant=descriptor.bindings.find("variant"),mode=descriptor.bindings.find("mode");
    if(variant==descriptor.bindings.end() || mode==descriptor.bindings.end())return false;
    return std::any_of(std::begin(variants),std::end(variants),[&](const auto& v) {
        return storeId==v.card && variant->second==v.name && mode->second==v.mode;
    });
}
bool configureSeeker(const components::DescriptorV1& descriptor, SeekerConfigV1& config, std::string& error) {
    if(descriptor.implementationId.rfind("equipment.",0)==0) {
        components::DescriptorV1 normalized;
        if(!components::equipmentSelectionV1(descriptor,"seeker",nullptr,normalized,error))return false;
        return configureSeeker(normalized,config,error);
    }
    const auto& id=descriptor.implementationId;
    const bool engineering=id=="fsim.seeker.nonimaging_ir.v3"||id=="fsim.seeker.imaging_ir.v3"||
        id=="fsim.seeker.semi_active_radar.v3"||id=="fsim.seeker.active_radar.v3";
    const char* expectedMode=nullptr;
    if(id=="fsim.seeker.nonimaging_ir.v2"||id=="fsim.seeker.nonimaging_ir.v3") expectedMode="infrared";
    if(id=="fsim.seeker.imaging_ir.v2"||id=="fsim.seeker.imaging_ir.v3") expectedMode="imaging_infrared";
    if(id=="fsim.seeker.semi_active_radar.v2"||id=="fsim.seeker.semi_active_radar.v3") expectedMode="semi_active_radar";
    if(id=="fsim.seeker.active_radar.v2"||id=="fsim.seeker.active_radar.v3") expectedMode="active_radar";
    if(descriptor.kind!="seeker" || (id!="fsim.seeker.encounter.v1"&&!expectedMode) ||
            descriptor.configurationVersion!=1 || descriptor.availability!=components::AvailabilityV1::Available) {
        error="unsupported or unavailable seeker component configuration"; return false;
    }
    components::DescriptorV1 legacySlots;
    components::ReceiverConfigurationV3 receiver;
    if(!components::receiverConfigurationV3(descriptor,legacySlots,receiver,error))return false;
    const struct {const char* role;const char* unit;double SensorEngineeringV3::*member;} dynamics[]={
        {"sample_period_s","s",&SensorEngineeringV3::samplePeriodS},{"processing_delay_s","s",&SensorEngineeringV3::processingDelayS},
        {"gimbal_rate_rad_s","rad/s",&SensorEngineeringV3::gimbalRateRadS},{"gimbal_lag_s","s",&SensorEngineeringV3::gimbalLagS},
        {"track_lag_s","s",&SensorEngineeringV3::trackLagS},{"field_half_angle_rad","rad",&SensorEngineeringV3::fieldHalfAngleRad},
        {"angle_noise_rad","rad",&SensorEngineeringV3::angleNoiseRad},{"range_noise_m","m",&SensorEngineeringV3::rangeNoiseM},
        {"range_rate_noise_m_s","m/s",&SensorEngineeringV3::rangeRateNoiseMs},{"frequency_hz","Hz",&SensorEngineeringV3::frequencyHz},
        {"noise_figure_db","dB",&SensorEngineeringV3::noiseFigureDb},{"system_loss_db","dB",&SensorEngineeringV3::systemLossDb},
        {"bandwidth_hz","Hz",&SensorEngineeringV3::bandwidthHz},{"detection_snr","1",&SensorEngineeringV3::detectionSnr},
        {"ir_intensity_w_sr","W/sr",&SensorEngineeringV3::irIntensityWSr},{"ir_extinction_m_1","1/m",&SensorEngineeringV3::irExtinctionPerM},
        {"ir_noise_w_m2","W/m^2",&SensorEngineeringV3::irNoiseWm2},{"background_w_m2","W/m^2",&SensorEngineeringV3::backgroundWm2},
        {"track_memory_s","s",&SensorEngineeringV3::memoryS},{"temperature_k","K",&SensorEngineeringV3::temperatureK},
        {"ir_band_low_um","um",&SensorEngineeringV3::irBandLowUm},{"ir_band_high_um","um",&SensorEngineeringV3::irBandHighUm},
        {"ir_source_temperature_k","K",&SensorEngineeringV3::irSourceTemperatureK},
        {"angle_resolution_rad","rad",&SensorEngineeringV3::angleResolutionRad},
        {"search_rate_rad_s","rad/s",&SensorEngineeringV3::searchRateRadS},
        {"lookdown_clutter_db","dB",&SensorEngineeringV3::lookdownClutterDb}};
    SensorEngineeringV3 sensor;
    if(engineering) {
        sensor.version=3;
        for(const auto& slot:dynamics) {
            double value;
            if(!components::parameterV1(descriptor,slot.role,slot.unit,value,error)||!std::isfinite(value)||value<=0) {
                error="finite engineering sensor requires positive admitted "+std::string(slot.role);return false;
            }
            if(!std::isfinite(descriptor.parameters.at(slot.role).uncertainty)||descriptor.parameters.at(slot.role).uncertainty<=0) {
                error="finite engineering sensor requires nonzero screening uncertainty: "+std::string(slot.role);return false;
            }
            sensor.*slot.member=value;legacySlots.parameters.erase(slot.role);legacySlots.parameterRefs.erase(slot.role);
        }
        if(sensor.fieldHalfAngleRad>=pi/2||sensor.processingDelayS>2||sensor.memoryS>5||sensor.samplePeriodS>1||sensor.samplePeriodS<.001||
           sensor.irBandLowUm<.1||sensor.irBandHighUm>100||sensor.irBandHighUm<=sensor.irBandLowUm||sensor.irSourceTemperatureK>5000||
           sensor.frequencyHz<1e6||sensor.frequencyHz>1e12||sensor.temperatureK>2000||sensor.noiseFigureDb>100||sensor.systemLossDb>100||
           sensor.angleNoiseRad>=.1||sensor.gimbalRateRadS>1000||sensor.gimbalLagS>5||sensor.trackLagS>5) {
            error="engineering sensor values outside finite supported domain";return false;
        }
    }
    if(!components::slotsV1(legacySlots,{"acquire_m","gimbal_deg","launch_fov_deg","minimum_range_m",
            "aspect_speed_floor_m_s","afterburner_threshold","ir_aspect_min","ir_aspect_span","ir_afterburner_multiplier",
            "radar_rcs_floor_m2","radar_rcs_reference_m2","radar_rcs_exponent","radar_range_scale_min","radar_range_scale_max",
            "radar_lookdown_cosine","radar_notch_m_s","illumination_beam_multiplier","aircraft_signal","ir_decoy_peak",
            "ir_decoy_decay_s","radar_decoy_peak","radar_decoy_rise_s","radar_decoy_decay_s","decoy_resistance",
            "association_cosine","emitter_power_w","emitter_peak_gain_dbi","emitter_sidelobe_dbi","emitter_half_width_rad"},
            {"mode","variant","measurement_model","image_model","servo_model"},error)) return false;
    const auto& bindings=legacySlots.bindings;
    const auto mode=bindings.find("mode");
    if(mode==bindings.end()) {error="seeker component needs its exact mode binding";return false;}
    if(expectedMode) {
        const auto binding=[&](const char* name,const char* expected) {
            const auto it=bindings.find(name);
            return it!=bindings.end()&&it->second==expected;
        };
        const bool supportedVariant=std::any_of(std::begin(variants),std::end(variants),[&](const auto& v) {
            return mode->second==v.mode && binding("variant",v.name);
        });
        if(bindings.size()!=5||mode->second!=expectedMode||!supportedVariant||
           !binding("measurement_model",engineering?"radiometric_engineering_v1":"ideal_geometric")||
           !binding("servo_model",engineering?"stabilized_finite_v1":"unavailable")||
           !binding("image_model",expectedMode==std::string("imaging_infrared")?"geometric_extent":"not_applicable")) {
            error="measured seeker needs its exact variant, observables and explicit fidelity bindings";return false;
        }
    } else if(bindings.size()!=1) {error="legacy seeker accepts only its mode binding";return false;}
    SeekerConfigV1 candidate;
    if(mode->second=="active_radar") candidate.mode=SeekerModeV1::ActiveRadar;
    else if(mode->second=="semi_active_radar") candidate.mode=SeekerModeV1::SemiActiveRadar;
    else if(mode->second=="infrared") candidate.mode=SeekerModeV1::Infrared;
    else if(mode->second=="imaging_infrared") candidate.mode=SeekerModeV1::ImagingInfrared;
    else {error="unsupported seeker component mode";return false;}
    const struct {const char* role;const char* unit;double SeekerConfigV1::* member;double scale=1;} slots[]={
        {"acquire_m","m",&SeekerConfigV1::acquireM},{"gimbal_deg","deg",&SeekerConfigV1::gimbalRad,pi/180},
        {"launch_fov_deg","deg",&SeekerConfigV1::launchFovRad,pi/180},{"minimum_range_m","m",&SeekerConfigV1::minimumRangeM},
        {"aspect_speed_floor_m_s","m/s",&SeekerConfigV1::aspectSpeedFloorMs},{"afterburner_threshold","1",&SeekerConfigV1::afterburnerThreshold},
        {"ir_aspect_min","1",&SeekerConfigV1::irAspectMin},{"ir_aspect_span","1",&SeekerConfigV1::irAspectSpan},
        {"ir_afterburner_multiplier","1",&SeekerConfigV1::irAfterburnerMultiplier},{"radar_rcs_floor_m2","m^2",&SeekerConfigV1::radarRcsFloorM2},
        {"radar_rcs_reference_m2","m^2",&SeekerConfigV1::radarRcsReferenceM2},{"radar_rcs_exponent","1",&SeekerConfigV1::radarRcsExponent},
        {"radar_range_scale_min","1",&SeekerConfigV1::radarRangeScaleMin},{"radar_range_scale_max","1",&SeekerConfigV1::radarRangeScaleMax},
        {"radar_lookdown_cosine","1",&SeekerConfigV1::radarLookdownCosine},{"radar_notch_m_s","m/s",&SeekerConfigV1::radarNotchMs},
        {"illumination_beam_multiplier","1",&SeekerConfigV1::illuminationBeamMultiplier},{"aircraft_signal","1",&SeekerConfigV1::aircraftSignal},
        {"ir_decoy_peak","1",&SeekerConfigV1::irDecoyPeak},{"ir_decoy_decay_s","s",&SeekerConfigV1::irDecoyDecayS},
        {"radar_decoy_peak","1",&SeekerConfigV1::radarDecoyPeak},{"radar_decoy_rise_s","s",&SeekerConfigV1::radarDecoyRiseS},
        {"radar_decoy_decay_s","s",&SeekerConfigV1::radarDecoyDecayS},{"decoy_resistance","1",&SeekerConfigV1::decoyResistance},
        {"association_cosine","1",&SeekerConfigV1::associationCosine},{"emitter_power_w","W",&SeekerConfigV1::emitterPowerW},
        {"emitter_peak_gain_dbi","dBi",&SeekerConfigV1::emitterPeakGainDbi},{"emitter_sidelobe_dbi","dBi",&SeekerConfigV1::emitterSidelobeDbi},
        {"emitter_half_width_rad","rad",&SeekerConfigV1::emitterHalfWidthRad}};
    for(const auto& slot:slots) {
        double value=0;
        if(!components::parameterV1(descriptor,slot.role,slot.unit,value,error)) return false;
        candidate.*slot.member=value*slot.scale;
    }
    candidate.version=1;
    candidate.engineering=sensor;
    if(!validSeekerConfigV1(candidate)) {error="seeker component configuration is outside its supported model domain";return false;}
    config=candidate;error.clear();return true;
}
bool validSeekerConfigV1(const SeekerConfigV1& c) noexcept {
    if(c.version==0) return c.engineering.version==0;
    if(c.version!=1 || !*seekerModeNameV1(c.mode)) return false;
    const auto& p=c.engineering;
    if(p.version!=0) {
        if(p.version!=3)return false;
        for(const double value:{p.samplePeriodS,p.processingDelayS,p.gimbalRateRadS,p.gimbalLagS,p.trackLagS,
            p.fieldHalfAngleRad,p.angleNoiseRad,p.angleResolutionRad,p.rangeNoiseM,p.rangeRateNoiseMs,p.frequencyHz,
            p.noiseFigureDb,p.systemLossDb,p.bandwidthHz,p.detectionSnr,p.irIntensityWSr,p.irExtinctionPerM,
            p.irNoiseWm2,p.backgroundWm2,p.memoryS,p.temperatureK,p.irBandLowUm,p.irBandHighUm,p.irSourceTemperatureK,
            p.searchRateRadS,p.lookdownClutterDb})if(!std::isfinite(value)||value<=0)return false;
        if(p.samplePeriodS<.001||p.samplePeriodS>1||p.processingDelayS>2||p.memoryS>5||
           p.fieldHalfAngleRad>=pi/2||p.angleNoiseRad>=.1||p.angleResolutionRad>=pi/2||
           p.frequencyHz<1e6||p.frequencyHz>1e12||p.temperatureK>2000||p.noiseFigureDb>100||p.systemLossDb>100||
           p.irBandLowUm<.1||p.irBandHighUm>100||p.irBandHighUm<=p.irBandLowUm||p.irSourceTemperatureK>5000||
           p.gimbalRateRadS>1000||p.gimbalLagS>5||p.trackLagS>5||p.lookdownClutterDb>100||p.detectionSnr>30)return false;
    }
    const std::array<double,29> values={c.acquireM,c.gimbalRad,c.launchFovRad,c.minimumRangeM,c.aspectSpeedFloorMs,
        c.afterburnerThreshold,c.irAspectMin,c.irAspectSpan,c.irAfterburnerMultiplier,c.radarRcsFloorM2,c.radarRcsReferenceM2,
        c.radarRcsExponent,c.radarRangeScaleMin,c.radarRangeScaleMax,c.radarLookdownCosine,c.radarNotchMs,
        c.illuminationBeamMultiplier,c.aircraftSignal,c.irDecoyPeak,c.irDecoyDecayS,c.radarDecoyPeak,c.radarDecoyRiseS,
        c.radarDecoyDecayS,c.decoyResistance,c.associationCosine,c.emitterPowerW,c.emitterPeakGainDbi,c.emitterSidelobeDbi,c.emitterHalfWidthRad};
    if(!std::all_of(values.begin(),values.end(),[](double value){return std::isfinite(value);})) return false;
    return c.acquireM>=0 && c.gimbalRad>=0 && c.gimbalRad<=pi && c.launchFovRad>=0 && c.launchFovRad<=pi &&
        c.minimumRangeM>0 && c.aspectSpeedFloorMs>0 && c.afterburnerThreshold>=0 && c.afterburnerThreshold<=1 &&
        c.irAspectMin>=0 && c.irAspectSpan>=0 && c.irAfterburnerMultiplier>0 && c.radarRcsFloorM2>0 && c.radarRcsReferenceM2>0 &&
        c.radarRcsExponent>=0 && c.radarRangeScaleMin>=0 && c.radarRangeScaleMax>=c.radarRangeScaleMin &&
        c.radarLookdownCosine>=-1 && c.radarLookdownCosine<=1 && c.radarNotchMs>=0 && c.illuminationBeamMultiplier>0 &&
        c.aircraftSignal>0 && c.irDecoyPeak>=0 && c.irDecoyDecayS>0 && c.radarDecoyPeak>=0 && c.radarDecoyRiseS>0 &&
        c.radarDecoyDecayS>0 && c.decoyResistance>0 && c.associationCosine>=-1 && c.associationCosine<=1 &&
        c.emitterPowerW>=0 && c.emitterHalfWidthRad>0 && c.emitterHalfWidthRad<=pi;
}
const char* seekerModeNameV1(SeekerModeV1 mode) noexcept {
    switch(mode) {
    case SeekerModeV1::ActiveRadar:return "active_radar";
    case SeekerModeV1::SemiActiveRadar:return "semi_active_radar";
    case SeekerModeV1::Infrared:return "infrared";
    case SeekerModeV1::ImagingInfrared:return "imaging_infrared";
    default:return "";
    }
}
double infraredAcquisitionRangeM(double baseM,double aspectCosine,bool afterburning,const SeekerConfigV1& config) noexcept {
    if(config.version==0) return infraredAcquisitionRangeM(baseM,aspectCosine,afterburning);
    if(!validSeekerConfigV1(config) || !std::isfinite(baseM) || baseM<0 || !std::isfinite(aspectCosine)) return 0;
    const double aspect=.5*(1+std::clamp(aspectCosine,-1.0,1.0));
    const double reach=baseM*(config.irAspectMin+config.irAspectSpan*aspect)*(afterburning?config.irAfterburnerMultiplier:1.0);
    return std::isfinite(reach)?reach:0;
}
double radarAcquisitionRangeM(double baseM,double sigmaM2,bool modelled,const SeekerConfigV1& config) noexcept {
    if(config.version==0) return modelled ? baseM*std::clamp(std::pow(std::max(sigmaM2,.001)/5.0,.25),.4,1.0) : baseM;
    if(!validSeekerConfigV1(config)) return 0;
    return modelled ? baseM*std::clamp(std::pow(std::max(sigmaM2,config.radarRcsFloorM2)/config.radarRcsReferenceM2,
        config.radarRcsExponent),config.radarRangeScaleMin,config.radarRangeScaleMax) : baseM;
}
bool radarNotched(double vertical,double radial,const SeekerConfigV1& config) noexcept {
    if(config.version==0) return vertical<-.03 && std::fabs(radial)<28;
    return validSeekerConfigV1(config) && vertical<config.radarLookdownCosine && std::fabs(radial)<config.radarNotchMs;
}
double decoySignal(double age,bool heat,const SeekerConfigV1& config) noexcept {
    if(config.version==0) return heat?6*std::exp(-age/2.0):3*std::min(1.0,age/.3)*std::exp(-age/6.0);
    if(!validSeekerConfigV1(config) || !std::isfinite(age) || age<0) return 0;
    return heat?config.irDecoyPeak*std::exp(-age/config.irDecoyDecayS):
        config.radarDecoyPeak*std::min(1.0,age/config.radarDecoyRiseS)*std::exp(-age/config.radarDecoyDecayS);
}
bool withinIlluminationBeam(const double direction[3],double azimuth,double elevation,double azimuthWidth,
                           double elevationWidth,const SeekerConfigV1& config) noexcept {
    if(config.version==0) return withinIlluminationBeam(direction,azimuth,elevation,azimuthWidth,elevationWidth);
    if(!validSeekerConfigV1(config) || !direction || !std::isfinite(direction[0]) || !std::isfinite(direction[1]) ||
        !std::isfinite(direction[2]) || !std::isfinite(azimuth) || !std::isfinite(elevation) || !std::isfinite(azimuthWidth) ||
        !std::isfinite(elevationWidth) || azimuthWidth<=0 || elevationWidth<=0) return false;
    const double norm=std::hypot(direction[0],direction[1],direction[2]);
    if(!(norm>0) || !std::isfinite(norm)) return false;
    const double beam[3]={std::cos(elevation)*std::cos(azimuth),std::cos(elevation)*std::sin(azimuth),-std::sin(elevation)};
    const double cosine=direction[0]/norm*beam[0]+direction[1]/norm*beam[1]+direction[2]/norm*beam[2];
    const double width=std::max(azimuthWidth,elevationWidth);
    const double limit=width>=pi/config.illuminationBeamMultiplier?pi:config.illuminationBeamMultiplier*width;
    return std::acos(std::clamp(cosine,-1.0,1.0))<=limit;
}
} // namespace fsim::ms::weapon
