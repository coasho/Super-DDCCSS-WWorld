#pragma once

// SIM-M10 (B-37): Physical-Optics Loaded-Store Radar Cross Section Model (Knott / Ruck).
//
// Computes the radar cross section contribution (m^2) of an externally carried store
// (external fuel tank, jamming/targeting pod, bomb, or missile) from its StoreCard's
// published geometry (length_m, diameter_m) and storeClass at a given body-frame look
// direction and radar frequency. Used as a fallback whenever a precomputed per-store
// rcs.bin table is not present; stores inside closed internal weapon bays contribute 0.

#include "simulation/mission_systems/Cards.h"

namespace fsim::ms::rcs {

/// Estimate monostatic RCS (m^2) of one externally carried store at body-frame look
/// direction `directionBody` (unit vector from target toward radar) and `frequencyHz`.
double externalStoreSigmaM2(const StoreCard& card,
                            const double directionBody[3],
                            double frequencyHz) noexcept;

} // namespace fsim::ms::rcs
