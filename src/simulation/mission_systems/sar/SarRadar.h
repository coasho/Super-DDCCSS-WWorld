#pragma once

// SIM-M07 (MS-11): Synthetic Aperture Radar (SAR) Spotlight/Strip Collection.
// Aligned with Curlander & McDonough (1991) synthetic-aperture integration time
// and Noise Equivalent Sigma Zero (NESZ) radiometry, including in-band RF jamming.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::sar {

/// Synthetic aperture coherent integration time (s):
/// T_int = lambda * R / (2 * v * rho_a * sin(squint)).
double integrationTimeS(double wavelengthM, double slantRangeM, double speedMs,
                        double azimuthResolutionM, double squintRad) noexcept;

/// Curlander & McDonough Noise Equivalent Sigma Zero (dB):
/// NESZ = 4 * (4*pi)^3 * R^3 * k_B * T_0 * F * L_sys * v * sin(incidence) / (P_avg * G^2 * lambda^3 * rho_r).
double neszDb(double slantRangeM, double speedMs, double incidenceRad,
              double avgPowerW, double antennaGainDbi, double frequencyHz,
              double rangeResolutionM, double noiseFigureDb, double systemLossDb,
              double extraJamDegradationDb = 0.0) noexcept;

/// Construct a SAR payload model (`asars-2`, `eiss-sar`).
std::unique_ptr<Model> makeSarRadar(const Card& card, std::uint16_t count);

} // namespace fsim::ms::sar
