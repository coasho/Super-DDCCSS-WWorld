#include "simulation/mission_systems/damage/Destruction.h"

#include <algorithm>
#include <cmath>

namespace fsim::ms::damage {

double hydrodynamicRamSpraySpeedMs(double fragmentEnergyJ, double fastestFragmentMs, double fuelKg) noexcept {
    if (!std::isfinite(fragmentEnergyJ) || !(fragmentEnergyJ > 0.0) || !std::isfinite(fastestFragmentMs) ||
        !(fastestFragmentMs > 0.0) || !std::isfinite(fuelKg) || !(fuelKg > 1e-3))
        return 0.0;
    // Hydrodynamic ram shock/cavity overpressure (Ball survivability / Disimile et al.):
    // Effective fluid mass accelerated near the exit wall is bounded by the local
    // cavity slug (~1-8 kg, clamped to actual fuel present), with ~15% of impact
    // kinetic energy coupled into exit-jet kinetic work.
    constexpr double kCouplingEfficiency = 0.15;
    const double slugMassKg = std::clamp(0.04 * fuelKg, 0.5, std::min(8.0, fuelKg));
    const double jetEnergyJ = kCouplingEfficiency * fragmentEnergyJ;
    // Fluid shock pressure bound: P_ram <= 0.5 * rho * (0.22 * v_frag)^2
    const double vEnergy = std::sqrt(2.0 * jetEnergyJ / slugMassKg);
    const double vShockCap = std::min(120.0, 0.22 * fastestFragmentMs);
    return std::clamp(std::min(vEnergy, vShockCap), 0.0, 120.0);
}

double ullageFlammability(double ullageFraction) noexcept {
    if (!std::isfinite(ullageFraction) || ullageFraction <= kUllageMinFraction || ullageFraction >= kUllageMaxFraction)
        return 0.0;
    // Smooth bell peaking near 55% ullage headspace (optimal kerosene vapour-air mixture).
    constexpr double kPeak = 0.55;
    if (ullageFraction < kPeak) {
        const double t = (ullageFraction - kUllageMinFraction) / (kPeak - kUllageMinFraction);
        return t * t * (3.0 - 2.0 * t);
    }
    const double t = (kUllageMaxFraction - ullageFraction) / (kUllageMaxFraction - kPeak);
    return t * t * (3.0 - 2.0 * t);
}

DestructionOutcome evaluateDestruction(VehicleDestructionState& state, const DestructionStepInput& input,
                                       bool allowCatastrophic, Rng& rng) noexcept {
    DestructionOutcome out;
    const double pressureScale = std::sqrt(std::clamp(input.airPressurePa / 101325.0, 0.05, 1.0));

    int separatedSections = 0;
    bool anyDamage = false;
    bool anyFire = false;
    bool centreCollapsed = false;
    double maxSectionDmg = 0.0;
    double sumSectionDmg = 0.0;

    if (input.damage) {
        const auto& frame = input.damage->airframe();
        const auto& sections = input.damage->sections();
        const std::size_t count = std::min<std::size_t>(sections.size(), ipc::kMaxSectionScorchV1);
        state.sectionCount = static_cast<std::uint8_t>(count);
        for (std::size_t s = 0; s < count; ++s) {
            const auto& sec = sections[s];
            double dmg = 0.0;
            if (sec.separated) {
                ++separatedSections;
                dmg = 1.0;
            } else {
                const double loss = 1.0 - std::clamp(sec.strength, 0.0, 1.0);
                const double skin = frame.sections[s].skinAreaM2 > 1e-6
                                        ? std::clamp((sec.tornM2 + 2.0 * sec.holeM2) / frame.sections[s].skinAreaM2, 0.0, 1.0)
                                        : 0.0;
                dmg = std::clamp(std::max(loss, skin), 0.0, 1.0);
                // Non-root (centre fuselage) structural collapse check.
                if (!frame.sections[s].hasRoot && (sec.removed >= kFuselageCollapseRemoved ||
                                                   input.maxBlastOverpressurePa >= kFuselageCollapseBlastPa)) {
                    centreCollapsed = true;
                }
            }
            if (dmg > 0.01) anyDamage = true;
            state.sectionDamage[s] = std::max(state.sectionDamage[s], dmg);
            maxSectionDmg = std::max(maxSectionDmg, state.sectionDamage[s]);
            sumSectionDmg += state.sectionDamage[s];
        }
        for (const auto& [tankIdx, tankState] : input.damage->tanks()) {
            (void)tankIdx;
            if (tankState.holeM2 > 0.0) anyDamage = true;
            if (tankState.burning) {
                anyFire = true;
                anyDamage = true;
                if (tankState.component >= 0 && static_cast<std::size_t>(tankState.component) < frame.components.size()) {
                    const int sec = frame.components[static_cast<std::size_t>(tankState.component)].section;
                    if (sec >= 0 && static_cast<std::size_t>(sec) < count)
                        state.sectionDamage[static_cast<std::size_t>(sec)] =
                            std::max(state.sectionDamage[static_cast<std::size_t>(sec)], 0.55);
                }
            }
        }
        if (input.damage->crewLost() || !input.damage->lostChannels().empty()) anyDamage = true;
        if (count > 0) state.overallDamage = std::clamp(std::max(state.overallDamage, sumSectionDmg / static_cast<double>(count)), 0.0, 1.0);
        const double targetScorch = std::clamp(maxSectionDmg * 0.85 + (anyFire ? 0.35 : 0.0), 0.0, 1.0);
        state.scorchIntensity = std::max(state.scorchIntensity, targetScorch);
    }

    // Hydrodynamic ram spray speed across impacted tanks (B-58).
    for (const auto& t : input.tanks) {
        if (t.hitEnergyJ > 0.0 && t.fuelKg > 0.0) {
            out.hydrodynamicRamSprayMs =
                std::max(out.hydrodynamicRamSprayMs, hydrodynamicRamSpraySpeedMs(t.hitEnergyJ, t.fastestFragmentMs, t.fuelKg));
        }
    }

    // Promote non-catastrophic tiers monotonically (never downgrade GroundWreck / WaterImpact / CatastrophicKill).
    if (state.tier < DestructionTier::CatastrophicKill) {
        if (separatedSections > 0)
            state.tier = DestructionTier::StructuralBreakup;
        else if (anyDamage && state.tier == DestructionTier::Intact)
            state.tier = DestructionTier::Damaged;
    }

    if (!allowCatastrophic || state.tier >= DestructionTier::CatastrophicKill) return out;

    // --- 1. Fuel-tank ullage overpressure ignition (B-58) ---
    double totalUllageTntKg = 0.0;
    double peakUllagePa = 0.0;
    bool ullageTriggered = false;
    for (const auto& t : input.tanks) {
        if (!(t.capacityKg > 1.0) || !(t.fuelKg > 0.5)) continue;
        const double ullageFrac = std::clamp(1.0 - t.fuelKg / t.capacityKg, 0.0, 1.0);
        const double flam = ullageFlammability(ullageFrac);
        if (flam <= 0.0) continue;
        const bool highEnergyHit = t.hitEnergyJ >= kUllageFragmentEnergyJ && t.fastestFragmentMs >= 700.0;
        const bool blastHit = t.blastOverpressurePa >= kUllageBlastPa && (t.burning || t.hitEnergyJ > 2000.0 || t.blastOverpressurePa >= 320000.0);
        if (!highEnergyHit && !blastHit) continue;
        const double overpressurePa = kUllagePeakDeflagrationPa * pressureScale * flam;
        peakUllagePa = std::max(peakUllagePa, overpressurePa);
        if (overpressurePa >= kUllageCollapsePa) {
            const double pIgnite = std::clamp(0.45 + 0.55 * flam * std::min(1.0, (t.hitEnergyJ / 35000.0) + (t.blastOverpressurePa / 350000.0)), 0.0, 0.98);
            if (rng.uniform() < pIgnite) {
                ullageTriggered = true;
                const double tankVolM3 = t.capacityKg / kFuelDensity;
                const double ullageVolM3 = tankVolM3 * ullageFrac;
                totalUllageTntKg += (ullageVolM3 * kUllageEnergyDensityJM3 * pressureScale) / kTntEnergyJKg;
            }
        }
    }

    // --- 2. Sympathetic on-board weapon / ammo cook-off (B-58) ---
    double cookedExplosiveKg = 0.0;
    bool anyFireOnOrdnance = false;
    for (const auto& ord : input.ordnance) {
        if (!(ord.explosiveKg > 0.01)) continue;
        if (ord.onFireSection) anyFireOnOrdnance = true;
        const bool directCookoff = ord.hitEnergyJ >= kCookoffHitEnergyJ || ord.blastOverpressurePa >= kCookoffBlastPa;
        if (directCookoff) cookedExplosiveKg += ord.explosiveKg;
    }
    if (anyFireOnOrdnance && input.dtS > 0.0) {
        state.cookoffExposureS += input.dtS;
        if (state.cookoffExposureS >= kCookoffThermalS) {
            for (const auto& ord : input.ordnance)
                if (ord.onFireSection && ord.explosiveKg > 0.01) cookedExplosiveKg += ord.explosiveKg;
        }
    }
    const bool cookoffTriggered = cookedExplosiveKg > 0.05;

    // --- 3. Catastrophic structural collapse ---
    const bool structuralCollapse = input.newlySeparatedCount >= 2 || separatedSections >= 3 || centreCollapsed;

    if (ullageTriggered || cookoffTriggered || structuralCollapse) {
        out.catastrophicTriggered = true;
        if (cookoffTriggered)
            out.cause = CatastrophicCause::SympatheticCookoff;
        else if (ullageTriggered)
            out.cause = CatastrophicCause::UllageOverpressure;
        else
            out.cause = CatastrophicCause::StructuralCollapse;

        out.ullageOverpressurePa = peakUllagePa;
        // Total TNT-equivalent blast is strictly bounded by real ullage vapour + real carried HE mass.
        double secondaryOrdKg = cookedExplosiveKg;
        if (!cookoffTriggered) {
            // Secondary sympathy from catastrophic disintegration: 50% of onboard armed HE cooks off.
            for (const auto& ord : input.ordnance)
                if (ord.explosiveKg > 0.01) secondaryOrdKg += 0.5 * ord.explosiveKg;
        }
        out.explosionTntKg = std::max(0.0, totalUllageTntKg + secondaryOrdKg);
        // Atomize 45% of actual remaining fuel into a mid-air fireball (never fabricated mass).
        out.fireballFuelKg = std::max(0.0, input.totalRemainingFuelKg) * 0.45;

        state.tier = DestructionTier::CatastrophicKill;
        state.cause = out.cause;
        state.hullRetired = true;
        state.overallDamage = 1.0;
        state.scorchIntensity = 1.0;
        for (std::uint8_t s = 0; s < state.sectionCount; ++s) state.sectionDamage[s] = 1.0;
    }
    return out;
}

void applyGroundOrWaterDestruction(VehicleDestructionState& state, const StrikeOutcome& outcome, double remainingFuelKg,
                                   bool enableSlide) noexcept {
    state.surface = outcome.surface;
    state.tier = outcome.surface == SurfaceKind::Water ? DestructionTier::WaterImpact : DestructionTier::GroundWreck;
    state.overallDamage = std::max(state.overallDamage, std::clamp(outcome.severityMs / 45.0, 0.35, 1.0));
    state.scorchIntensity = std::max(state.scorchIntensity, outcome.fireballKg > 0.0 || outcome.poolKg > 0.0 ? 0.95 : 0.45);
    const double postSpeed = length(outcome.after);
    state.groundSliding = enableSlide && postSpeed > 2.5 && outcome.angleRad < 0.78539816339; // < 45 deg slides along surface
    state.slideSpeedMs = state.groundSliding ? postSpeed : 0.0;
    state.wreckFuelKg = std::max(0.0, std::min(remainingFuelKg, outcome.poolKg));
    state.wreckBurnRateKgS = outcome.poolKgS > 0.0 ? std::min(outcome.poolKgS, kBurnMostKgS) : 0.0;
    // Extreme steep high-energy dive (> 45 deg and > 120 m/s) disintegrates hull on impact too.
    if (enableSlide && outcome.angleRad >= 0.78539816339 && outcome.normalMs > 100.0) {
        state.hullRetired = true;
        state.overallDamage = 1.0;
        for (std::uint8_t s = 0; s < state.sectionCount; ++s) state.sectionDamage[s] = 1.0;
    }
}

ipc::VehicleDestructionWireV1 toWire(const VehicleDestructionState& state, std::uint32_t slot, std::uint32_t vehicleId,
                                     std::uint64_t generation, double liveFuelKg) noexcept {
    ipc::VehicleDestructionWireV1 w;
    w.generation = generation;
    w.slot = slot;
    w.vehicleId = vehicleId;
    w.overallDamage = static_cast<float>(std::clamp(state.overallDamage, 0.0, 1.0));
    w.scorchIntensity = static_cast<float>(std::clamp(state.scorchIntensity, 0.0, 1.0));
    w.fuelRemainingKg = static_cast<float>(std::max(0.0, std::isfinite(liveFuelKg) ? liveFuelKg : state.wreckFuelKg));
    w.sectionCount = std::min<std::uint8_t>(state.sectionCount, static_cast<std::uint8_t>(ipc::kMaxSectionScorchV1));
    for (std::uint8_t i = 0; i < w.sectionCount; ++i)
        w.sectionDamage[i] = static_cast<float>(std::clamp(state.sectionDamage[i], 0.0, 1.0));
    w.tier = static_cast<std::uint8_t>(state.tier);
    w.hullRetired = state.hullRetired ? 1u : 0u;
    w.surfaceKind = static_cast<std::uint8_t>(state.surface);
    return w;
}

} // namespace fsim::ms::damage
