#pragma once

#include "contracts/world_stream/WorldLayout.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string_view>

namespace fsim::showcase {

// Native stream semantics: Hit.slot is the victim, Hit.target is the weapon
// slot + 1, and Hit.index counts actual intersections. This does not synthesize
// an outcome from a detonation, contact, blast or general damage notification.
inline bool matchingFragmentHit(const ipc::EngagementEvent& event,std::uint32_t weaponSlot,
    std::uint32_t victimSlot,std::string_view store,double terminalTime) noexcept {
    const auto end=std::find(std::begin(event.name),std::end(event.name),'\0');
    return weaponSlot!=std::numeric_limits<std::uint32_t>::max() && std::isfinite(terminalTime) &&
        event.kind==static_cast<std::uint8_t>(ipc::EngagementKind::Hit) && event.slot==victimSlot &&
        event.target==weaponSlot+1 && event.index>0 && event.simTime==terminalTime &&
        end!=std::end(event.name) && std::string_view(event.name,static_cast<std::size_t>(end-event.name))==store;
}

} // namespace fsim::showcase
