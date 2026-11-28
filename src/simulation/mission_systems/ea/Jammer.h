#pragma once

// Electronic attack (ADR-30, MS-9b / SIM-M05): RF noise jamming and coherent
// DRFM deception models (RGPO, VGPO, combined RGPO+VGPO, multi-false-target)
// for the payload family `electronic_attack`. A jammer executes `TaskKind::Jam`
// noise tasks or `EaDeceptionCommandV1` DRFM tasks either preemptively or
// reactively by following an own-platform ESM contact. Optical/IR countermeasures
// (e.g. AN/AAQ-24 DIRCM) are explicitly distinguished from RF jammers in
// `makeJammer` (returning null) and handled by `ea::makeDircm`.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::ea {

/// One coherent DRFM deceptive echo produced by a repeater jammer against a
/// victim radar or missile RF seeker along the jammer's true line of sight.
struct DrfmGhost {
    int slot = 0;                ///< stable ghost slot index (0..7)
    double rangeOffsetM = 0.0;   ///< causal range delay offset added to true range (m, >= 0)
    double rangeRateOffsetMs = 0.0; ///< radial range-rate shift added to true range rate (m/s, positive opening)
    double equivalentRcsM2 = 0.0;   ///< physical ERP-capped equivalent repeater RCS (m^2)
};

/// Stable synthetic ID in [kDrfmFalseTargetIds, kDesignations) for a DRFM false echo.
constexpr std::uint32_t drfmGhostId(std::uint32_t jammerEntity, std::uint16_t payload, int slot) noexcept {
    return kDrfmFalseTargetIds |
           ((jammerEntity & 0xFFFFu) << 8) |
           ((static_cast<std::uint32_t>(payload) & 0xFu) << 4) |
           (static_cast<std::uint32_t>(slot) & 0xFu);
}

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

/// Evaluate coherent DRFM repeater echoes for `em` as seen by a victim radar or
/// RF seeker at slant range `rangeM` with average transmit power `victimPowerW`,
/// antenna linear gain toward jammer `victimGainLinear`, linear one-way excess
/// propagation loss `victimPathExcessLinear`, jammer transmit gain `txGainDbi`,
/// and spectral overlap `overlap`. Appends physical ghosts to `out`.
void evaluateDrfmGhosts(const Emission& em, double rangeM, double victimPowerW,
                        double victimGainLinear, double victimPathExcessLinear,
                        double txGainDbi, double overlap,
                        std::vector<DrfmGhost>& out) noexcept;

/// Construct an RF electronic-attack model for `card` (`count` pods/units).
/// Returns null if the card is not an RF electronic-attack payload (e.g. DIRCM).
std::unique_ptr<Model> makeJammer(const Card& card, std::uint16_t count);

} // namespace fsim::ms::ea
