#pragma once

// Electronic attack (ADR-30, MS-9b / SIM-M05): RF noise jamming models for the
// payload family `electronic_attack`. A jammer executes `TaskKind::Jam` tasks
// either preemptively (sector or omnidirectional at a commanded frequency and
// bandwidth) or reactively by following an own-platform ESM contact's bearing
// and frequency. Optical/IR countermeasures (e.g. DIRCM) are explicitly
// distinguished from RF jammers and refuse RF noise-jamming tasks rather than
// fabricating RF emissions.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::ea {

/// Compute linear spectral overlap fraction in [0, 1] between a jammer centred
/// at `jamFreqHz` with bandwidth `jamBwHz` and a victim receiver centred at
/// `rxFreqHz` with bandwidth `rxBwHz`.
double spectralOverlap(double jamFreqHz, double jamBwHz, double rxFreqHz, double rxBwHz) noexcept;

/// Compute received jammer power spectral density J0 (W/Hz) at a victim
/// receiver given jammer average power (W), transmit gain towards victim (dBi),
/// victim receive gain towards jammer (dBi), one-way path loss (dB), jammer
/// bandwidth (Hz), and spectral overlap fraction in [0, 1].
double receivedJammerDensityWHz(double powerW, double txGainDbi, double rxGainDbi,
                                double pathLossDb, double jamBwHz, double overlap) noexcept;

/// Construct an electronic-attack model for `card` (`count` pods/units).
/// Returns null if the card is not an RF electronic-attack payload (e.g. DIRCM).
std::unique_ptr<Model> makeJammer(const Card& card, std::uint16_t count);

} // namespace fsim::ms::ea
