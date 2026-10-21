#pragma once

#include "simulation/mission_systems/damage/Damage.h"

#include <memory>
#include <string_view>

namespace fsim::ms::damage {

/// Internal versioned provider sidecar; StoreCard and public SDK layouts stay
/// unchanged. An adapter owns one validated native warhead snapshot, while the
/// caller retains ownership of the per-release assessment RNG.
class WarheadAdapter {
public:
    virtual ~WarheadAdapter() = default;
    virtual std::string_view implementationId() const noexcept = 0;
    virtual const Warhead& model() const noexcept = 0;
    virtual Impact assess(const Burst&, const AttachedGeometryView&,
                          const Target&, Rng&) const = 0;
};

} // namespace fsim::ms::damage
