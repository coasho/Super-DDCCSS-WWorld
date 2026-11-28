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

HitAspect classifyHitAspect(const V3& fromBody) noexcept {
    const double ax = std::abs(fromBody.x);
    const double ay = std::abs(fromBody.y);
    const double az = std::abs(fromBody.z);
    if (!(ax + ay + az > 1e-6)) return HitAspect::Unknown;
    if (ax >= ay && ax >= az) return fromBody.x >= 0.0 ? HitAspect::Front : HitAspect::Rear;
    if (ay >= az) return fromBody.y >= 0.0 ? HitAspect::Right : HitAspect::Left;
    return fromBody.z < 0.0 ? HitAspect::Dorsal : HitAspect::Ventral;
}

DestructionOutcome evaluateDestruction(VehicleDestructionState& state, const DestructionStepInput& input,
                                       bool allowCatastrophic, Rng& rng) noexcept {
    DestructionOutcome out;
    const double pressureScale = std::sqrt(std::clamp(input.airPressurePa / 101325.0, 0.05, 1.0));

    if (input.fragmentsStriking > 0) {
        state.hitFragmentsTotal += input.fragmentsStriking;
    }
    if (input.hasHitDirection) {
        const HitAspect aspect = classifyHitAspect(input.hitFromBody);
        if (aspect != HitAspect::Unknown) state.hitAspect = aspect;
    }

    int separatedSections = 0;
    bool anyDamage = (input.fragmentsStriking > 0);
    bool anyFire = input.engineFireActive || state.engineFireActive;
    bool anyPowerOrControlLoss = input.engineFireActive || state.engineFireActive;
    bool centreCollapsed = false;
    double maxSectionDmg = 0.0;
    double sumSectionDmg = 0.0;
    std::array<bool, ipc::kMaxSectionScorchV1> burningSec{};
    bool centreBayFire = false;

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
                // Non-root (centre fuselage) structural collapse check: strictly uses the centre fuselage's
                // OWN blast overpressure & skin loss — never a tailcone or wingtip panel's peak blast!
                const double secBlastPa = input.hasSectionBlast ? input.sectionBlastPa[s] : input.maxBlastOverpressurePa;
                if (!frame.sections[s].hasRoot && (std::max(sec.removed, skin) >= kFuselageCollapseRemoved ||
                                                   secBlastPa >= kFuselageCollapseBlastPa)) {
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
                    if (sec >= 0 && static_cast<std::size_t>(sec) < count) {
                        burningSec[static_cast<std::size_t>(sec)] = true;
                        if (!frame.sections[static_cast<std::size_t>(sec)].hasRoot) centreBayFire = true;
                        state.sectionDamage[static_cast<std::size_t>(sec)] =
                            std::max(state.sectionDamage[static_cast<std::size_t>(sec)], 0.55);
                    }
                }
            }
        }
        if ((input.engineFireActive || state.engineFireActive) && input.engineFireSection >= 0 &&
            static_cast<std::size_t>(input.engineFireSection) < count) {
            burningSec[static_cast<std::size_t>(input.engineFireSection)] = true;
            state.sectionDamage[static_cast<std::size_t>(input.engineFireSection)] =
                std::max(state.sectionDamage[static_cast<std::size_t>(input.engineFireSection)], 0.58);
        }
        if (input.damage->anyEngineOut() || !input.damage->lostChannels().empty() || input.damage->crewLost()) {
            anyDamage = true;
            anyPowerOrControlLoss = true;
        }
        if (count > 0) state.overallDamage = std::clamp(std::max(state.overallDamage, sumSectionDmg / static_cast<double>(count)), 0.0, 1.0);
        const double targetScorch = std::clamp(maxSectionDmg * 0.85 + (anyFire ? 0.35 : 0.0), 0.0, 1.0);
        state.scorchIntensity = std::max(state.scorchIntensity, targetScorch);
    }

    // Hydrodynamic ram spray speed & deposited energy accounting across impacted tanks (B-58).
    for (const auto& t : input.tanks) {
        if (t.hitEnergyJ > 0.0) {
            state.hitEnergyTotalJ += t.hitEnergyJ;
            if (t.fuelKg > 0.0) {
                out.hydrodynamicRamSprayMs =
                    std::max(out.hydrodynamicRamSprayMs, hydrodynamicRamSpraySpeedMs(t.hitEnergyJ, t.fastestFragmentMs, t.fuelKg));
            }
        }
    }

    // Progressive in-flight thermal weakening of burning load-bearing roots (aluminium 2024-T3 strength loss under fire)
    // and centre-fuselage thermal runaway soak (delayed secondary mid-air ullage explosion).
    bool thermalRunawayTriggered = false;
    double thermalRunawayTntKg = 0.0;
    if (allowCatastrophic && input.dtS > 0.0 && state.tier < DestructionTier::CatastrophicKill && input.damage) {
        const auto& frame = input.damage->airframe();
        const auto& sections = input.damage->sections();
        const std::size_t count = std::min<std::size_t>(sections.size(), ipc::kMaxSectionScorchV1);
        for (std::size_t s = 0; s < count; ++s) {
            if (!burningSec[s] || sections[s].separated) continue;
            state.sectionThermalS[s] += input.dtS;
            state.sectionDamage[s] = std::max(state.sectionDamage[s], std::clamp(0.50 + 0.07 * state.sectionThermalS[s], 0.0, 0.96));
            if (frame.sections[s].hasRoot && state.sectionThermalS[s] > kThermalWeakenOnsetS) {
                out.thermalStrengthDelta[s] = kThermalWeakenRatePerS * input.dtS;
            }
        }
        // Only sustained intense centre-fuselage fuel fire (>= 2.4 kg/s) with flammable centre headspace
        // escalates to delayed secondary ullage deflagration; moderate engine/tail fires stay in Grade 2 FireAndStall.
        if (centreBayFire && input.activeFuelFireRateKgS >= 2.4) {
            double bestFlam = 0.0;
            double ullageTnt = 0.0;
            for (const auto& t : input.tanks) {
                if (!t.isCentreSection || !(t.capacityKg > 1.0) || !(t.fuelKg > 5.0)) continue;
                const double u = std::clamp(1.0 - t.fuelKg / t.capacityKg, 0.0, 1.0);
                const double flam = ullageFlammability(u);
                if (flam > 0.0) {
                    bestFlam = std::max(bestFlam, flam);
                    ullageTnt += ((t.capacityKg / kFuelDensity) * u * kUllageEnergyDensityJM3 * pressureScale) / kTntEnergyJKg;
                }
            }
            if (bestFlam > 0.15) {
                const double rateWeight = std::clamp(input.activeFuelFireRateKgS / 4.2, 0.55, 1.45);
                state.centreThermalRunawayS += input.dtS * rateWeight * (0.65 + 0.55 * bestFlam);
                if (state.centreThermalRunawayS >= kCentreThermalRunawayS) {
                    thermalRunawayTriggered = true;
                    thermalRunawayTntKg = std::max(ullageTnt, 0.5);
                }
            }
        }
    }

    // Promote non-catastrophic tiers & physical damage grades monotonically.
    if (state.tier < DestructionTier::CatastrophicKill) {
        if (separatedSections > 0) {
            state.tier = DestructionTier::StructuralBreakup;
            state.grade = std::max(state.grade, DamageGrade::StructuralBreakup);
        } else if (anyFire || anyPowerOrControlLoss) {
            if (state.tier == DestructionTier::Intact) state.tier = DestructionTier::Damaged;
            state.grade = std::max(state.grade, DamageGrade::FireAndStall);
        } else if (anyDamage) {
            if (state.tier == DestructionTier::Intact) state.tier = DestructionTier::Damaged;
            state.grade = std::max(state.grade, DamageGrade::LightPerforation);
        }
    }

    if (!allowCatastrophic || state.tier >= DestructionTier::CatastrophicKill) return out;

    // --- 1. Fuel-tank ullage overpressure ignition (B-58 / VIS-M11b location-aware) ---
    // Outboard wing tanks blow off THAT wing (Grade 3 StructuralBreakup); centre fuselage main tanks
    // require concentrated multi-fragment core saturation or high shock overpressure for instant Grade 4 CatastrophicKill.
    double totalUllageTntKg = thermalRunawayTriggered ? thermalRunawayTntKg : 0.0;
    double peakUllagePa = 0.0;
    bool ullageTriggered = thermalRunawayTriggered;
    for (const auto& t : input.tanks) {
        if (!(t.capacityKg > 1.0) || !(t.fuelKg > 0.5)) continue;
        const double ullageFrac = std::clamp(1.0 - t.fuelKg / t.capacityKg, 0.0, 1.0);
        const double flam = ullageFlammability(ullageFrac);
        if (flam <= 0.0) continue;

        if (!t.isCentreSection) {
            // Outboard wing fuel cell: localized overpressure shears the wing rather than vaporizing the fuselage.
            const bool wingCoreHit =
                (t.hitCount >= kUllageWingMinHits && t.hitEnergyJ >= kUllageWingEnergyJ && t.fastestFragmentMs >= 700.0) ||
                (t.hitCount == 0 && t.hitEnergyJ >= kUllageWingEnergyJ && t.fastestFragmentMs >= 700.0);
            const bool wingBlastHit = t.blastOverpressurePa >= kUllageWingBlastPa && (t.hitCount >= 2 || t.burning || t.blastOverpressurePa >= 260000.0);
            if ((wingCoreHit || wingBlastHit) && t.section >= 0 && out.rupturedWingSection < 0) {
                const double overpressurePa = kUllagePeakDeflagrationPa * pressureScale * flam;
                peakUllagePa = std::max(peakUllagePa, overpressurePa);
                if (overpressurePa >= kUllageCollapsePa * 0.72) {
                    const double pBlow = std::clamp(0.55 + 0.42 * flam, 0.0, 0.96);
                    if (rng.uniform() < pBlow) {
                        out.rupturedWingSection = t.section;
                        out.rupturedWingSpillKg = std::clamp(t.fuelKg * 0.28, 8.0, 38.0);
                        state.tier = DestructionTier::StructuralBreakup;
                        state.grade = std::max(state.grade, DamageGrade::StructuralBreakup);
                    }
                }
            }
            continue;
        }

        // Centre fuselage main tank: concentrated fragment saturation or high shock overpressure.
        const bool highEnergyHit =
            t.hitCount > 0
                ? ((t.hitCount >= kUllageCoreMinHits && t.hitEnergyJ >= kUllageCoreEnergyJ && t.fastestFragmentMs >= 750.0) ||
                   (t.hitEnergyJ >= kUllageCoreSingleEnergyJ && t.fastestFragmentMs >= 700.0))
                : (t.hitEnergyJ >= kUllageFragmentEnergyJ && t.fastestFragmentMs >= 700.0);
        const bool blastHit =
            t.blastOverpressurePa >= kUllageBlastPa &&
            (t.burning || t.hitCount >= 3 || (t.hitCount == 0 && t.hitEnergyJ > 2000.0) || t.blastOverpressurePa >= 320000.0);
        if (!highEnergyHit && !blastHit) continue;
        const double overpressurePa = kUllagePeakDeflagrationPa * pressureScale * flam;
        peakUllagePa = std::max(peakUllagePa, overpressurePa);
        if (overpressurePa >= kUllageCollapsePa) {
            const double pIgnite =
                std::clamp(0.45 + 0.55 * flam * std::min(1.0, (t.hitEnergyJ / 55000.0) + (t.blastOverpressurePa / 350000.0)), 0.0, 0.98);
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
    const int effectiveSeparated = separatedSections + (out.rupturedWingSection >= 0 ? 1 : 0);
    const bool structuralCollapse = input.newlySeparatedCount >= 2 || effectiveSeparated >= 3 || centreCollapsed;

    if (ullageTriggered || cookoffTriggered || structuralCollapse) {
        out.catastrophicTriggered = true;
        out.rupturedWingSection = -1; // subsumed into full catastrophic breakup
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
        state.grade = DamageGrade::CatastrophicKill;
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
    state.grade = DamageGrade::SurfaceImpact;
    state.overallDamage = std::max(state.overallDamage, std::clamp(outcome.severityMs / 45.0, 0.35, 1.0));
    state.scorchIntensity = std::max(state.scorchIntensity, outcome.fireballKg > 0.0 || outcome.poolKg > 0.0 ? 0.95 : 0.45);
    const double postSpeed = length(outcome.after);
    // High-speed water impact (stagnation pressure 0.5 * rho_w * v^2 >= ~4.6 MPa at >= 95 m/s, or steep normal entry >= 20 m/s)
    // violently shatters the airframe on entry into hydrodynamic wreckage & geyser plumes rather than skimming intact.
    const bool highSpeedWaterShatter =
        enableSlide && outcome.surface == SurfaceKind::Water && (outcome.normalMs >= 20.0 || outcome.speedMs >= 95.0);
    // Steep or high-vertical-energy land impact (>= 45 deg at >= 32 m/s normal, or >= 55 m/s normal at any angle)
    // disintegrates the airframe into tumbling JoltPhysics sections/shards & crater fireball rather than freezing intact.
    const bool steepLandDisintegrate =
        enableSlide && outcome.surface != SurfaceKind::Water &&
        ((outcome.angleRad >= 0.78539816339 && outcome.normalMs >= 32.0) || outcome.normalMs >= 55.0);
    if (highSpeedWaterShatter || steepLandDisintegrate) {
        state.hullRetired = true;
        state.groundSliding = false;
        state.slideSpeedMs = 0.0;
        state.overallDamage = 1.0;
        for (std::uint8_t s = 0; s < state.sectionCount; ++s) state.sectionDamage[s] = 1.0;
    } else {
        state.groundSliding = enableSlide && postSpeed > 2.5 && outcome.angleRad < 0.78539816339; // < 45 deg slides along surface
        state.slideSpeedMs = state.groundSliding ? postSpeed : 0.0;
    }
    state.wreckFuelKg = std::max(0.0, std::min(remainingFuelKg, outcome.poolKg));
    state.wreckBurnRateKgS = outcome.poolKgS > 0.0 ? std::min(outcome.poolKgS, kBurnMostKgS) : 0.0;
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
