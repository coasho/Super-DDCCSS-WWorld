#pragma once

#include "simulation/mission_systems/components/interfaces/SeekerNavigationPoseV1.h"
#include "simulation/mission_systems/weapon/NavigationCaptureV2.h"
#include <limits>

namespace fsim::ms::components {

// Private World adapter: sensor/scene physics keep their original frame.
// Only this guidance-specific value receives the selected own-navigation pose.
inline SeekerFrameV2 measuredGuidanceOwnFrameV2(const SeekerFrameV2& physical,
    const SeekerNavigationPoseV1& navigation,bool physicalReceiver,bool strapdownSelected) {
    auto frame=physical;
    if(!physicalReceiver)return frame;
    // Preserve the existing physical-receiver guidance routing.
    frame.position=navigation.position;frame.velocity=navigation.velocity;
    frame.boresight=navigation.bodyToEcef[0];frame.bodyAxes=navigation.bodyToEcef;
    if(strapdownSelected) {
        const double unavailable=std::numeric_limits<double>::quiet_NaN();
        frame.up={unavailable,unavailable,unavailable};
        if(navigation.valid&&navigation.timeS==physical.timeS) {
            const auto gravity=weapon::navigationGravityVerticalV2(navigation.position);
            frame.up={-gravity.x/9.80665,-gravity.y/9.80665,-gravity.z/9.80665};
        }
    }
    return frame;
}

} // namespace fsim::ms::components
