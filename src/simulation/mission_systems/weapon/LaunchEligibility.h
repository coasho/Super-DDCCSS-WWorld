#pragma once

#include "fsim/MissionSystems.h"
#include "fsim/VehicleState.h"

#include <cstdint>

namespace fsim::ms {
struct StoreCard;
}

namespace fsim::ms::weapon {
struct SeekerConfigV1;

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
    /// The designation belongs to this launcher's radar, rather than a network
    /// source whose independently allocated track number may happen to match.
    bool localSource = true;
};

/// Pure rules for each selected AAM; inventory selection and host admission stay with the caller.
LaunchEligibility checkLaunchEligibility(const StoreCard& card, const sim::VehicleState& own,
                                        const sim::VehicleState* target);
LaunchEligibility checkLaunchEligibility(const StoreCard& card, const sim::VehicleState& own,
                                        const sim::VehicleState* target, const SeekerConfigV1&);
Reason checkLaunchIllumination(std::uint32_t launcher, const IlluminationFacts& facts) noexcept;

} // namespace fsim::ms::weapon
