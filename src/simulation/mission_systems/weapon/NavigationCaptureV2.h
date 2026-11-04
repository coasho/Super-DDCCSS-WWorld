#pragma once
#include "NavigationStrapdownV2.h"

namespace fsim::ms::weapon {
/// Retained control-law gravity magnitude, along the WGS84 geodetic vertical
/// computed solely from an admitted estimated ECEF own position.
V3 navigationGravityVerticalV2(V3 estimatedPosition) noexcept;
/// Dense own-motion capture decimator. Covariance is advanced only at actual
/// scheduled sensor events and explicitly requested native control epochs.
/// Sensor physics interpolates the adjacent physical captures, not stale
/// estimator states. No sensor clock or RNG is reset by observation/batching.
class NavigationCaptureV2 {
public:
    void reset(const NavigationStrapdownInputV2& release) noexcept {previous_=release;initialized_=true;}
    bool observe(NavigationStrapdownV2&,const NavigationStrapdownInputV2&,bool controlRead);
private:
    NavigationStrapdownInputV2 previous_;
    bool initialized_=false;
};
}
