#pragma once
#include "simulation/mission_systems/components/Profile.h"
#include <optional>
#include <string>

namespace fsim::ms::weapon {
struct PulsePolicyV1 {
    unsigned version=0;
    double secondStartS=0,holdLimitS=0,reserveSpeedMs=0,terminalLeadS=0;
};
struct PulseInputV1 {
    double timeS=0,nativeAgeS=0,speedMs=0,fuelKg=0;
    std::optional<double> timeToGoS;
};
enum class PulseStateV1 { Invalid, FirstOrCoast, Holding, Requested, Expired, NoFuel };
struct PulseDecisionV1 {bool valid=false,ignite=false;PulseStateV1 state=PulseStateV1::Invalid;};
bool configurePulseV1(const components::DescriptorV1&,PulsePolicyV1&,std::string& error);
class PulseManagerV1 {
public:
    explicit PulseManagerV1(PulsePolicyV1 policy):policy_(policy){}
    PulseDecisionV1 update(const PulseInputV1&) noexcept;
    PulseDecisionV1 cachedDecisionV1() const noexcept {return cached_;}
    void reset() noexcept {lastTimeS_=-1;lastNativeAgeS_=-1;requested_=false;expired_=false;cached_={};}
private:
    PulsePolicyV1 policy_;
    double lastTimeS_=-1,lastNativeAgeS_=-1;
    bool requested_=false,expired_=false;
    PulseDecisionV1 cached_;
};
}
