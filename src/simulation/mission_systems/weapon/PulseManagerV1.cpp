#include "PulseManagerV1.h"
#include "contracts/equipment/EquipmentModels.h"
#include <cmath>

namespace fsim::ms::weapon {
namespace {
bool positive(double value) {return std::isfinite(value)&&value>0;}
bool valid(const PulsePolicyV1& p) {
    return p.version==1&&positive(p.secondStartS)&&positive(p.holdLimitS)&&p.holdLimitS<=120&&
        positive(p.reserveSpeedMs)&&p.reserveSpeedMs<=5000&&positive(p.terminalLeadS)&&p.terminalLeadS<=120;
}
}
bool configurePulseV1(const components::DescriptorV1& d,PulsePolicyV1& output,std::string& error) {
    if(const auto* model=equipment::component(d.implementationId)) {
        if(model->kind!=d.kind){error="named pulse model has the wrong component role";return false;}
        auto normalized=d;normalized.implementationId=std::string(model->mechanismId);
        return configurePulseV1(normalized,output,error);
    }
    if(d.kind!="propulsion"||d.configurationVersion!=1||d.availability!=components::AvailabilityV1::Available||
       d.implementationId!="fsim.propulsion.jsbsim_commanded_dual_pulse.v1") {error="unsupported commanded pulse profile";return false;}
    const auto scalar=[&](const char* role,const char* unit,double& value) {return components::parameterV1(d,role,unit,value,error);};
    double count,first,coast,second,buildup,t0,t1,t2;
    PulsePolicyV1 p;p.version=1;
    if(!scalar("phase_count","1",count)||count!=3||!scalar("phase_0_duration_s","s",first)||
       !scalar("phase_1_duration_s","s",coast)||!scalar("phase_2_duration_s","s",second)||
       !scalar("phase_0_thrust_kn","kN",t0)||!scalar("phase_1_thrust_kn","kN",t1)||!scalar("phase_2_thrust_kn","kN",t2)||
       !scalar("buildup_s","s",buildup)||!scalar("pulse_hold_limit_s","s",p.holdLimitS)||
       !scalar("pulse_reserve_speed_m_s","m/s",p.reserveSpeedMs)||!scalar("pulse_terminal_lead_s","s",p.terminalLeadS))return false;
    if(!positive(first)||!positive(coast)||!positive(second)||!positive(t0)||t1!=0||!positive(t2)||
       !positive(buildup)||buildup>first||p.terminalLeadS<second) {error="commanded pulse needs two positive phases around an authored coast";return false;}
    for(const auto* role:{"pulse_hold_limit_s","pulse_reserve_speed_m_s","pulse_terminal_lead_s"}) {
        const auto it=d.parameters.find(role);
        if(it==d.parameters.end()||!positive(it->second.uncertainty)) {error="pulse policy needs nonzero admitted uncertainty";return false;}
    }
    p.secondStartS=first+coast+buildup*(1-2/std::acos(-1.));
    if(!valid(p)) {error="pulse policy outside finite supported domain";return false;}
    output=p;error.clear();return true;
}
PulseDecisionV1 PulseManagerV1::update(const PulseInputV1& in) noexcept {
    if(!valid(policy_)||!std::isfinite(in.timeS)||in.timeS<0||in.timeS<lastTimeS_||
       !std::isfinite(in.nativeAgeS)||in.nativeAgeS<0||in.nativeAgeS<lastNativeAgeS_||
       !std::isfinite(in.speedMs)||in.speedMs<0||!std::isfinite(in.fuelKg)||in.fuelKg<0||
       (in.timeToGoS&&(!std::isfinite(*in.timeToGoS)||*in.timeToGoS<0)))return {};
    if(in.timeS==lastTimeS_)return cached_;
    PulseDecisionV1 result;result.valid=true;
    if(requested_) {result.ignite=true;result.state=PulseStateV1::Requested;}
    else if(in.fuelKg==0)result.state=PulseStateV1::NoFuel;
    else if(expired_||in.nativeAgeS>policy_.secondStartS+policy_.holdLimitS) {expired_=true;result.state=PulseStateV1::Expired;}
    else if(in.nativeAgeS<policy_.secondStartS)result.state=PulseStateV1::FirstOrCoast;
    else if(in.speedMs<=policy_.reserveSpeedMs||(in.timeToGoS&&*in.timeToGoS<=policy_.terminalLeadS)) {
        requested_=true;result.ignite=true;result.state=PulseStateV1::Requested;
    } else result.state=PulseStateV1::Holding;
    lastTimeS_=in.timeS;lastNativeAgeS_=in.nativeAgeS;cached_=result;return result;
}
}
