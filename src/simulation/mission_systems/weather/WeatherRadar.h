#pragma once

// SIM-M07 (MS-11): Airborne Weather Radar (Marshall-Palmer Z-R reflectivity,
// Probert-Jones beam-filled return, ITU-R P.838 X-band two-way rain attenuation).

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::weather {

/// Marshall-Palmer (1948) equivalent reflectivity factor Z = 200 * R^1.6 expressed in dBZ
/// (returns -infinity when rainRateMmH <= 0).
double marshallPalmerDbz(double rainRateMmH) noexcept;

/// ITU-R P.838-3 X-band (~9.375 GHz) one-way specific rain attenuation k_r = a * R^b (dB/km).
double rainSpecificAttenuationDbKm(double rainRateMmH, double frequencyHz = 9.375e9) noexcept;

/// Construct an airborne weather radar model from its payload card (`an-apn-241`, `an-aps-133`, `kc135r-radar`).
std::unique_ptr<Model> makeWeatherRadar(const Card& card, std::uint16_t count);

} // namespace fsim::ms::weather
