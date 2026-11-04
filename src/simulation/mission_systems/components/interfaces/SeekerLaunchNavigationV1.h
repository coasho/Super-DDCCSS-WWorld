#pragma once
#include "simulation/mission_systems/weapon/Guidance.h"

namespace fsim::ms::components {
// Optional private seam for the same authorized prelaunch estimate that a
// fresh provider factory receives. It must not reset physical receiver state.
class SeekerLaunchNavigationV1 {
public:
    virtual ~SeekerLaunchNavigationV1()=default;
    virtual bool seedLaunchNavigationV1(const weapon::Observation&)=0;
};
}
