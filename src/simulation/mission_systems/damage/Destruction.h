#pragma once

// Multi-tier native aircraft destruction state machine (VIS-M11, closing B-58).
// Models physical escalation strictly from native damage/impact inputs:
//   Tier 0 (Intact) -> Tier 1 (Damaged) -> Tier 2 (StructuralBreakup)
//   -> Tier 3 (CatastrophicKill: fuel-tank ullage overpressure ignition,
//      sympathetic store/ammo cook-off, or centre-fuselage collapse)
//   -> Tier 4/5 (GroundWreck / WaterImpact: belly slide & wreck fuel burn-down).
// Aligned with MIL-HDBK-2069 / Ball aircraft combat survivability and
// hydrodynamic-ram exit-spray physics (never scripted or arbitrary).

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "simulation/mission_systems/damage/Impact.h"
#include "simulation/mission_systems/damage/State.h"

#include <array>
#include <cstdint>
#include <vector>

namespace fsim::ms::damage {

enum class DestructionTier : std::uint8_t {
    Intact = 0,
    Damaged = 1,
    StructuralBreakup = 2,
    CatastrophicKill = 3,
    GroundWreck = 4,
    WaterImpact = 5,
};

enum class CatastrophicCause : std::uint8_t {
    None = 0,
    UllageOverpressure = 1,
    SympatheticCookoff = 2,
    StructuralCollapse = 3,
};

// Physical thresholds (MIL-HDBK-2069 / Ball combat survivability; estimates noted):
inline constexpr double kUllageMinFraction = 0.12;        ///< minimum vapour-air headspace fraction for ignition
inline constexpr double kUllageMaxFraction = 0.96;        ///< above this, tank is nearly dry of fuel vapour
inline constexpr double kUllageFragmentEnergyJ = 18000.0; ///< high-energy incendiary/fragment ullage ignition threshold
inline constexpr double kUllageBlastPa = 240000.0;        ///< blast overpressure capable of rupturing/igniting ullage (~35 psi)
inline constexpr double kUllageCollapsePa = 350000.0;     ///< confined bay overpressure that ruptures fuselage structure (~50 psi)
inline constexpr double kUllagePeakDeflagrationPa = 720000.0; ///< stoichiometric kerosene vapour deflagration peak (~7.2 bar)
inline constexpr double kUllageEnergyDensityJM3 = 2.8e6;  ///< stoichiometric JP-8 vapour combustion energy per m^3 headspace
inline constexpr double kTntEnergyJKg = 4.184e6;          ///< 1 kg TNT equivalent
inline constexpr double kCookoffHitEnergyJ = 12000.0;     ///< direct fragment strike energy inducing sympathetic store detonation
inline constexpr double kCookoffBlastPa = 250000.0;       ///< sympathetic detonation shock pressure threshold
inline constexpr double kCookoffThermalS = 6.0;           ///< sustained section fuel fire soak time to ordnance cook-off
inline constexpr double kFuselageCollapseRemoved = 0.65;  ///< centre-fuselage skin-removal collapse threshold
inline constexpr double kFuselageCollapseBlastPa = 450000.0; ///< catastrophic fuselage blast crush (~65 psi)

struct TankUllageInput {
    int tankIndex = -1;
    int section = -1;
    double fuelKg = 0.0;
    double capacityKg = 0.0;
    double hitEnergyJ = 0.0;
    double fastestFragmentMs = 0.0;
    double blastOverpressurePa = 0.0;
    bool burning = false;
};

struct CarriedOrdnanceInput {
    int section = -1;
    double explosiveKg = 0.0;
    double hitEnergyJ = 0.0;
    double blastOverpressurePa = 0.0;
    bool onFireSection = false;
};

struct DestructionStepInput {
    const VehicleDamage* damage = nullptr;
    double airPressurePa = 101325.0;
    double maxBlastOverpressurePa = 0.0;
    double totalRemainingFuelKg = 0.0;
    int newlySeparatedCount = 0;
    double dtS = 0.0;
    std::vector<TankUllageInput> tanks;
    std::vector<CarriedOrdnanceInput> ordnance;
};

struct DestructionOutcome {
    bool catastrophicTriggered = false;
    CatastrophicCause cause = CatastrophicCause::None;
    double explosionTntKg = 0.0;          ///< TNT-equivalent blast from real ullage vapour + cooked-off HE mass
    double fireballFuelKg = 0.0;          ///< actual remaining fuel dispersed into mid-air fireball
    double ullageOverpressurePa = 0.0;    ///< peak confined tank ullage overpressure (Pa)
    double hydrodynamicRamSprayMs = 0.0;  ///< physically derived hydrodynamic-ram spray exit speed (m/s, B-58)
};

struct VehicleDestructionState {
    DestructionTier tier = DestructionTier::Intact;
    CatastrophicCause cause = CatastrophicCause::None;
    SurfaceKind surface = SurfaceKind::Soil;
    bool hullRetired = false;
    bool groundSliding = false;
    double overallDamage = 0.0;
    double scorchIntensity = 0.0;
    std::array<double, ipc::kMaxSectionScorchV1> sectionDamage{};
    std::uint8_t sectionCount = 0;
    double cookoffExposureS = 0.0;
    double wreckFuelKg = 0.0;
    double wreckBurnRateKgS = 0.0;
    double totalFuelBurnedKg = 0.0;
    double slideSpeedMs = 0.0;
};

/// Hydrodynamic-ram fuel spray exit speed (m/s, B-58) derived from fragment
/// kinetic energy deposited in kerosene (rho = 800 kg/m^3) rather than an
/// arbitrary fraction of fragment velocity.
double hydrodynamicRamSpraySpeedMs(double fragmentEnergyJ, double fastestFragmentMs, double fuelKg) noexcept;

/// Evaluate fuel-tank ullage flammability weight (0..1) from ullage volume fraction.
double ullageFlammability(double ullageFraction) noexcept;

/// Update per-section damage/scorch metrics and evaluate Tier 0..3 transitions
/// (Damaged -> StructuralBreakup -> CatastrophicKill via ullage/cook-off/collapse).
DestructionOutcome evaluateDestruction(VehicleDestructionState& state, const DestructionStepInput& input,
                                       bool allowCatastrophic, Rng& rng) noexcept;

/// Record a ground or water impact transition (Tier 4 GroundWreck or Tier 5 WaterImpact).
void applyGroundOrWaterDestruction(VehicleDestructionState& state, const StrikeOutcome& outcome, double remainingFuelKg,
                                   bool enableSlide) noexcept;

/// Encode per-vehicle destruction state to the additive VIS-M11 wire record.
ipc::VehicleDestructionWireV1 toWire(const VehicleDestructionState& state, std::uint32_t slot, std::uint32_t vehicleId,
                                     std::uint64_t generation, double liveFuelKg) noexcept;

} // namespace fsim::ms::damage
