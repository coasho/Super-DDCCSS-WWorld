#pragma once

// SIM-M07 (MS-11): Positioning, Navigation & Timing (PNT) Payload Models:
// - Pure Inertial Navigation System (`an-asn-109`): Groves Schuler oscillation + drift, RF-jamming immune.
// - Military SAASM GPS (`magr-2k`): L1 (1575.42 MHz) / L2 (1227.60 MHz) effective C/N0 under RF jamming
//   (Groves Eq. 8.35), dead-reckoning fallback when jammed, and clean re-acquisition when jamming ceases.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::pnt {

/// Schuler period T_s = 2 * pi * sqrt(R_e / g) (~5063.5 s = 84.4 min).
inline constexpr double kSchulerPeriodS = 5063.5;

/// Effective GNSS carrier-to-noise-plus-interference density ratio (C/N0)_eff (dB-Hz)
/// following Groves Eq. 8.35: (C/N0)_eff = C / (N0 + J0 / Q_j).
double effectiveGnssCn0DbHz(double carrierPowerDbw, double noiseFigureDb,
                            double jammerDensityWHz, double antiJamMarginDb) noexcept;

/// Pure INS horizontal radial drift envelope (m) after elapsed time `elapsedS`
/// combining Schuler oscillation and navigation-grade drift rate `driftRateMs`.
double insHorizontalErrorM(double elapsedS, double driftRateMs, double initialErrorM = 15.0) noexcept;

/// Construct a PNT model (`an-asn-109`, `magr-2k`).
std::unique_ptr<Model> makePntReceiver(const Card& card, std::uint16_t count);

} // namespace fsim::ms::pnt
