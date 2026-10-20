#pragma once

#include <algorithm>
#include <cmath>

namespace fsim::ms::weapon {

/// Existing infrared encounter approximation, shared by on-rail and released
/// observations. aspectCosine is the target's velocity direction dotted with
/// the observer-to-target ray. The .45/.55 aspect factors and 1.35 afterburner
/// multiplier are assumptions, not calibrated signatures or transmission.
inline double infraredAcquisitionRangeM(double baseM, double aspectCosine, bool afterburning) noexcept {
    if (!std::isfinite(baseM) || baseM < 0 || !std::isfinite(aspectCosine)) return 0;
    const double aspect = .5 * (1 + std::clamp(aspectCosine, -1.0, 1.0));
    const double reach = baseM * (.45 + .55 * aspect) * (afterburning ? 1.35 : 1.0);
    return std::isfinite(reach) ? reach : 0;
}

} // namespace fsim::ms::weapon
