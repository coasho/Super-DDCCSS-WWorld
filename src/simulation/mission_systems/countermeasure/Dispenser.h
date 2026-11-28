#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace fsim::ms::countermeasure {

enum class Kind { Chaff, Flare };

struct Point { double x = 0.0, y = 0.0, z = 0.0; };

/// Physically calibrated countermeasure dispenser & cartridge parameters.
/// Defaults model a standard NATO AN/ALE-47 fit carrying MJU-7/B MTV IR flares
/// (1x2x8 in, ~0.38 kg, ~4.2 s burn, ~6500 W/sr MWIR peak) and RR-170A/AL
/// Al-coated glass dipole chaff cartridges (1x1x8 in, ~18 m^2 X-band peak RCS,
/// ~28 s wind-drifting dipole cloud persistence).
struct Spec {
    double minIntervalS = 0.35;          ///< Minimum sequencer interval between releases (s)
    // IR pyrotechnic flare cartridge (MJU-7/B MTV baseline)
    double flareLifeS = 4.2;             ///< Total active burn duration (s)
    double flareRiseS = 0.22;            ///< Pyrotechnic ignition rise time constant (s)
    double flareMassKg = 0.38;           ///< Initial cartridge pellet mass at ejection (kg)
    double flareBurnedMassKg = 0.09;     ///< Residual slag/casing mass at burnout (kg)
    double flareDragAreaM2 = 0.0058;     ///< Mean tumbling Cd*A with combustion base-bleed (m^2)
    double flarePeakIntensityWSr = 6500.0; ///< Peak 3-5 um MWIR radiant intensity (W/sr)
    double flarePeakTemperatureK = 2150.0; ///< Peak MTV combustion flame temperature (K)
    double flareCoreRadiusM = 0.18;      ///< Physical peak MTV flame core radius (~6500 W/sr at 2150 K, m)
    double flareEjectSpeedMs = 24.0;     ///< Squib cartridge ejection speed relative to airframe (m/s)
    // Radar chaff dipole cartridge (RR-170A/AL baseline)
    double chaffLifeS = 28.0;            ///< Effective radar-scattering cloud persistence (s)
    double chaffBloomTimeS = 1.6;        ///< Dipole de-clumping & wake vortex bloom constant (s)
    double chaffBloomRadiusM = 12.0;     ///< Nominal bloomed dipole cloud radius (m)
    double chaffPeakRcsM2 = 18.0;        ///< Peak resonant band monostatic RCS (m^2)
    double chaffCenterFreqHz = 10.0e9;   ///< Dipole half-wave resonance center frequency (Hz)
    double chaffDragCoeffM2Kg = 0.095;   ///< High-speed burst deceleration drag factor (m^2/kg at sea level)
    double chaffTerminalSinkMs = 1.2;    ///< Dipole cloud sea-level gravitational settling speed (m/s)
    double chaffEjectSpeedMs = 18.5;     ///< Squib cartridge ejection speed relative to airframe (m/s)
};

struct Decoy {
    std::uint32_t id = 0, owner = 0;
    Kind kind = Kind::Chaff;
    double launchTimeS = 0.0, lifeS = 0.0; ///< Simulation launch timestamp and physical lifetime, seconds.
    Point position, velocity;
    Spec spec; ///< Cartridge physical parameters active for this decoy
};

/// Piecewise constant local atmospheric environment in the decoy's Cartesian frame.
/// `down` is normalized on acceptance; `airVelocity` is local wind velocity (m/s);
/// `airDensityKgM3` is local air density (kg/m^3, ISA sea-level default 1.225).
struct Motion {
    Point down;
    Point airVelocity;
    double airDensityKgM3 = 1.225;
};

struct Sample {
    Decoy state;
    double ageS = std::numeric_limits<double>::quiet_NaN();
};

/// Owned motion history for one decoy live at the beginning of a step. Expired
/// decoys retain their last partial trajectory after leaving the live list.
struct Trajectory {
    Sample before, activeEnd;
    double stepDurationS = 0.0, liveDurationS = 0.0;
    Motion motion;
    bool valid = false, expires = false;
};

/// Standard ISA troposphere/stratosphere air density (kg/m^3) from altitude MSL and sea-level reference.
double isaAirDensityKgM3(double altitudeMslM, double seaLevelTempK = 288.15, double seaLevelPressurePa = 101325.0) noexcept;

/// 3-phase MTV pyrotechnic flare burn factor in [0, 1]: ignition rise -> combustion plateau -> burnout tail.
double flareBurnFactor(const Spec& spec, double ageS) noexcept;

/// Instantaneous 3-5 um MWIR radiant intensity (W/sr) of a flare decoy at `ageS`.
double flareIntensityWSr(const Decoy& decoy, double ageS) noexcept;

/// Instantaneous effective MTV combustion temperature (K) at `ageS`.
double flareTemperatureK(const Decoy& decoy, double ageS, double ambientK = 250.0) noexcept;

/// Instantaneous physical flare combustion envelope radius (m) at `ageS`.
double flareCoreRadiusM(const Decoy& decoy, double ageS) noexcept;

/// Instantaneous chaff dipole cloud physical radius (m) under wake bloom + turbulent wind-shear diffusion.
double chaffBloomRadiusM(const Decoy& decoy, double ageS) noexcept;

/// Instantaneous chaff dipole cloud monostatic RCS (m^2) at radar frequency `frequencyHz`.
double chaffRcsM2(const Decoy& decoy, double ageS, double frequencyHz = 10.0e9) noexcept;

/// Non-consuming physical RK4 sample along an owned decoy trajectory.
/// The mathematical endpoint at liveDurationS is allowed for geometry even when expires is true:
/// visibility and fuze callers MUST treat that expiry endpoint as exclusive.
/// Invalid trajectories, times and arithmetic leave `out` unchanged; no extrapolation is performed.
bool sampleAt(const Trajectory& trajectory, double offsetS, Sample& out) noexcept;

class Dispenser {
public:
    Dispenser(std::uint32_t owner, unsigned chaff, unsigned flare, Spec spec = {});
    /// Consume one cartridge on acceptance. IDs are allocated by the world, never by this module.
    /// All kinds share `spec.minIntervalS`; rejection has no side effects.
    bool dispense(Kind kind, double now, Point position, Point velocity, std::uint32_t id);
    /// Advance age and motion by simulation seconds using sea-level ISA density and zero wind.
    /// Invalid steps leave the entire state unchanged. Invalid propagation removes its pose.
    void step(double dt, Point down);
    /// Advance each decoy with its own supplied local atmospheric environment (gravity direction,
    /// wind velocity, air density) and return its owned before/active-end trajectory, including
    /// partial final lifetimes. Invalid dt, missing callbacks or any invalid environment reject
    /// the whole step atomically without mutation.
    std::vector<Trajectory> advance(double dt, std::function<Motion(const Decoy&)> environment);
    unsigned remaining(Kind kind) const;
    const Spec& spec() const noexcept { return spec_; }
    const std::vector<Decoy>& decoys() const;

private:
    std::uint32_t owner_;
    unsigned chaff_, flare_;
    Spec spec_;
    double lastReleaseS_ = -1.0;
    std::vector<Decoy> decoys_;
    struct Age { double elapsedS = 0.0, correctionS = 0.0; };
    std::vector<Age> ages_;
};

} // namespace fsim::ms::countermeasure
