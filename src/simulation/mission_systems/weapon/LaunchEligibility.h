#pragma once

#include "fsim/MissionSystems.h"
#include "fsim/VehicleState.h"

#include <cstdint>

namespace fsim::ms {
struct StoreCard;
}

namespace fsim::ms::weapon {

/// Target/FOV rejection, or the next facts an eligible selected AAM needs.
struct LaunchEligibility {
    Reason reason = Reason::None;
    bool illuminationRequired = false;
};

/// The caller's designation and radar lookups, collected only after target/FOV acceptance.
struct IlluminationFacts {
    bool designated = false;
    std::uint32_t owner = 0;
    std::uint64_t track = 0;
    bool illuminating = false;
    std::uint64_t radarTrack = 0;
};

/// Pure rules for the first selected AAM; inventory selection and host admission stay with the caller.
LaunchEligibility checkLaunchEligibility(const StoreCard& card, const sim::VehicleState& own,
                                        const sim::VehicleState* target);
Reason checkLaunchIllumination(std::uint32_t launcher, const IlluminationFacts& facts) noexcept;

} // namespace fsim::ms::weapon
