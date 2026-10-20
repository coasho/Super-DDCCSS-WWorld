#pragma once

#include <algorithm>
#include <cmath>
#include "fsim/MissionSystems.h"

namespace fsim::ms::weapon {

/// The existing on-rail query remains available before preparation, and also
/// while a prepared missile waits for consent. Gone/warming stores are absent.
inline bool onRailObservationAvailable(StoreStatus status) noexcept {
    return status == StoreStatus::Loaded || status == StoreStatus::Ready;
}

/// The native radar's existing geometric admission convention, applied to
/// each SARH candidate from the actual transmitter. This is not received-SNR
/// calibration. directionBody points from transmitter to candidate, x forward,
/// y right, z down; beam elevation is positive up.
inline bool withinIlluminationBeam(const double directionBody[3], double azimuthRad,
                                  double elevationRad, double azimuthBeamwidthRad,
                                  double elevationBeamwidthRad) noexcept {
    if (!directionBody || !std::isfinite(directionBody[0]) || !std::isfinite(directionBody[1])
        || !std::isfinite(directionBody[2]) || !std::isfinite(azimuthRad) || !std::isfinite(elevationRad)
        || !std::isfinite(azimuthBeamwidthRad) || !std::isfinite(elevationBeamwidthRad)
        || azimuthBeamwidthRad <= 0 || elevationBeamwidthRad <= 0) return false;
    const double norm = std::hypot(directionBody[0], directionBody[1], directionBody[2]);
    if (!(norm > 0) || !std::isfinite(norm)) return false;
    const double beam[3] = {std::cos(elevationRad) * std::cos(azimuthRad),
        std::cos(elevationRad) * std::sin(azimuthRad), -std::sin(elevationRad)};
    const double cosine = directionBody[0] / norm * beam[0] + directionBody[1] / norm * beam[1]
        + directionBody[2] / norm * beam[2];
    const double width = std::max(azimuthBeamwidthRad, elevationBeamwidthRad);
    // Avoid multiplying an unbounded caller value; pi already admits every ray.
    constexpr double pi = 3.14159265358979323846;
    const double limit = width >= pi / 3 ? pi : 3 * width;
    return std::acos(std::clamp(cosine, -1.0, 1.0)) <= limit;
}

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
