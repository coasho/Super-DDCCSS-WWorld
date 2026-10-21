#include "Seeker.h"
#include "simulation/mission_systems/components/Profile.h"
#include <array>

namespace fsim::ms::weapon {
namespace {
constexpr double pi=3.14159265358979323846;
}
bool configureSeeker(const components::DescriptorV1& descriptor, SeekerConfigV1& config, std::string& error) {
    const auto& id=descriptor.implementationId;
    const char* expectedMode=nullptr;const char* variant=nullptr;
    if(id=="fsim.seeker.nonimaging_ir.v2") {expectedMode="infrared";variant="AIM-9M";}
    if(id=="fsim.seeker.imaging_ir.v2") {expectedMode="imaging_infrared";variant="AIM-9X Block I";}
    if(id=="fsim.seeker.semi_active_radar.v2") {expectedMode="semi_active_radar";variant="AIM-7M";}
    if(id=="fsim.seeker.active_radar.v2") {expectedMode="active_radar";variant="AIM-120C-7";}
    if(descriptor.kind!="seeker" || (id!="fsim.seeker.encounter.v1"&&!expectedMode) ||
            descriptor.configurationVersion!=1 || descriptor.availability!=components::AvailabilityV1::Available) {
        error="unsupported or unavailable seeker component configuration"; return false;
    }
    if(!components::slotsV1(descriptor,{"acquire_m","gimbal_deg","launch_fov_deg","minimum_range_m",
            "aspect_speed_floor_m_s","afterburner_threshold","ir_aspect_min","ir_aspect_span","ir_afterburner_multiplier",
            "radar_rcs_floor_m2","radar_rcs_reference_m2","radar_rcs_exponent","radar_range_scale_min","radar_range_scale_max",
            "radar_lookdown_cosine","radar_notch_m_s","illumination_beam_multiplier","aircraft_signal","ir_decoy_peak",
            "ir_decoy_decay_s","radar_decoy_peak","radar_decoy_rise_s","radar_decoy_decay_s","decoy_resistance",
            "association_cosine","emitter_power_w","emitter_peak_gain_dbi","emitter_sidelobe_dbi","emitter_half_width_rad"},
            {"mode","variant","measurement_model","image_model","servo_model"},error)) return false;
    const auto mode=descriptor.bindings.find("mode");
    if(mode==descriptor.bindings.end()) {error="seeker component needs its exact mode binding";return false;}
    if(expectedMode) {
        const auto binding=[&](const char* name,const char* expected) {
            const auto it=descriptor.bindings.find(name);
            return it!=descriptor.bindings.end()&&it->second==expected;
        };
        if(descriptor.bindings.size()!=5||mode->second!=expectedMode||!binding("variant",variant)||
           !binding("measurement_model","ideal_geometric")||!binding("servo_model","unavailable")||
           !binding("image_model",id=="fsim.seeker.imaging_ir.v2"?"geometric_extent":"not_applicable")) {
            error="measured seeker needs its exact variant, observables and explicit fidelity bindings";return false;
        }
    } else if(descriptor.bindings.size()!=1) {error="legacy seeker accepts only its mode binding";return false;}
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
    if(!validSeekerConfigV1(candidate)) {error="seeker component configuration is outside its supported model domain";return false;}
    config=candidate;error.clear();return true;
}
bool validSeekerConfigV1(const SeekerConfigV1& c) noexcept {
    if(c.version==0) return true;
    if(c.version!=1 || !*seekerModeNameV1(c.mode)) return false;
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
