#pragma once

namespace fsim::ms::components {
/// Optional private observation-planning capability. It never advances a
/// sensor, consumes a random draw or replaces per-step servo/illumination work.
/// Providers without this sibling still receive the complete scene each tick.
class SeekerSceneClockV1 {
public:
    virtual ~SeekerSceneClockV1()=default;
    virtual bool sceneCaptureDueV1(double timeS) const noexcept=0;
};
}
