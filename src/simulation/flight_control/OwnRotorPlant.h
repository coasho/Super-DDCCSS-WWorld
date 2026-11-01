#pragma once

#include <cstdint>

namespace fsim::control {
struct Performance;
class RotorPosition;
// Native implementation seam, deliberately absent from public SDK headers.
// A trusted control callback receives only its own same-epoch plant bias.
struct OwnRotorPlantV1 {
    double angularAcceleration[3] = {};
    double specificForceBody[3] = {}; ///< shared airframe + declared passive rotor force / native mass, m/s^2
    bool passiveForceIncluded = false;
    bool available = false;
    const Performance* coursePerformance = nullptr;
    double courseBandwidthRadS = 0;
    const RotorPosition* positionController = nullptr;
};
class OwnRotorPlantViewV1 {
public:
    virtual ~OwnRotorPlantViewV1() = default;
    virtual bool ownRotorPlantV1(std::uint32_t vehicle,OwnRotorPlantV1&) const noexcept = 0;
};
struct OwnRotorPlantContextV1 {
    const void* owner = nullptr;
    std::uint32_t vehicle = 0;
    OwnRotorPlantV1 snapshot;
};
inline thread_local OwnRotorPlantContextV1 ownRotorPlantContextV1;
class OwnRotorPlantScopeV1 {
public:
    OwnRotorPlantScopeV1(const void* owner,std::uint32_t vehicle,const OwnRotorPlantV1& snapshot)
        : previous_(ownRotorPlantContextV1) { ownRotorPlantContextV1={owner,vehicle,snapshot}; }
    ~OwnRotorPlantScopeV1() { ownRotorPlantContextV1=previous_; }
    OwnRotorPlantScopeV1(const OwnRotorPlantScopeV1&)=delete;
    OwnRotorPlantScopeV1& operator=(const OwnRotorPlantScopeV1&)=delete;
private:
    OwnRotorPlantContextV1 previous_;
};
}
