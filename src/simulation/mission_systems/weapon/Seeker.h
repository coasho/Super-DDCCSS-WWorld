#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include "fsim/MissionSystems.h"

namespace fsim::ms::components { struct DescriptorV1; }
namespace fsim::ms::weapon {

enum class SeekerModeV1 { Unknown, ActiveRadar, SemiActiveRadar, Infrared, ImagingInfrared };

/// Private resolved assumptions for the existing encounter seeker. Version
/// zero retains legacy callers; explicit descriptors must resolve version one.
/// Instances contain values only, never references to a card or global state.
struct SeekerConfigV1 {
    unsigned version = 0;
    SeekerModeV1 mode = SeekerModeV1::Unknown;
    double acquireM = 17000, gimbalRad = 1.047, launchFovRad = .4363323129985824;
    double minimumRangeM = 1, aspectSpeedFloorMs = 1, afterburnerThreshold = .5;
    double irAspectMin = .45, irAspectSpan = .55, irAfterburnerMultiplier = 1.35;
    double radarRcsFloorM2 = .001, radarRcsReferenceM2 = 5, radarRcsExponent = .25;
    double radarRangeScaleMin = .4, radarRangeScaleMax = 1;
    double radarLookdownCosine = -.03, radarNotchMs = 28, illuminationBeamMultiplier = 3;
    double aircraftSignal = 1, irDecoyPeak = 6, irDecoyDecayS = 2;
    double radarDecoyPeak = 3, radarDecoyRiseS = .3, radarDecoyDecayS = 6;
    double decoyResistance = 1.6, associationCosine = .999390827019096;
    double emitterPowerW = 20, emitterPeakGainDbi = 25, emitterSidelobeDbi = -15, emitterHalfWidthRad = .35;
};

bool configureSeeker(const components::DescriptorV1& descriptor, SeekerConfigV1& config, std::string& error);
bool validSeekerConfigV1(const SeekerConfigV1& config) noexcept;
const char* seekerModeNameV1(SeekerModeV1 mode) noexcept;

double infraredAcquisitionRangeM(double baseM, double aspectCosine, bool afterburning,
                                 const SeekerConfigV1& config) noexcept;
double radarAcquisitionRangeM(double baseM, double sigmaM2, bool modelled,
                              const SeekerConfigV1& config) noexcept;
bool radarNotched(double verticalCosine, double targetRadialSpeedMs, const SeekerConfigV1& config) noexcept;
double decoySignal(double ageS, bool infrared, const SeekerConfigV1& config) noexcept;
bool withinIlluminationBeam(const double directionBody[3], double azimuthRad,
                           double elevationRad, double azimuthBeamwidthRad,
                           double elevationBeamwidthRad, const SeekerConfigV1& config) noexcept;

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
