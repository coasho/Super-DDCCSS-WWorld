#pragma once

#include "SeekerExecutionV2.h"

namespace fsim::ms::components {
/// Current admitted own-navigation solution, separate from actual scene and
/// mechanical gimbal axes. Columns are the ESTIMATED body-to-ECEF rotation.
/// New physical receiver selections require valid=true and exact frame epoch;
/// missing/stale navigation never falls back to referee orientation/position.
struct SeekerNavigationPoseV1 {
    bool valid=false;
    double timeS=0;
    weapon::V3 position,velocity;
    std::array<weapon::V3,3> bodyToEcef{};
};
} // namespace fsim::ms::components
