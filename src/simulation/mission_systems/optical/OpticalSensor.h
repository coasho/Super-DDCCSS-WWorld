#pragma once

// SIM-M06 (MS-10): Passive Optical (IRST / EOTS / FLIR / DAS / MAWS / ISR EO-IR)
// and Active Optical Laser Rangefinder / Designator payload models.
// Aligned with Hudson (Infrared System Engineering), Mahulikar et al. (2007),
// LOWTRAN/MODTRAN altitude-scaled Beer-Lambert transmission, Kruse visibility
// extinction, and the two-way Lambertian laser rangefinder link equation.

#include "simulation/mission_systems/Model.h"

namespace fsim::ms::optical {

/// Exact dimensionless Planck blackbody in-band radiation fraction over [lowUm, highUm]
/// at temperature `temperatureK` (SI constants, Simpson quadrature).
double planckBandFraction(double temperatureK, double lowUm, double highUm) noexcept;

/// ICAO / US Standard Atmosphere 1976 static air temperature (K) at altitude `altitudeM`
/// shifted by sea-level reference `seaLevelK`.
double isaTemperatureK(double altitudeM, double seaLevelK = 288.15) noexcept;

/// Aerodynamic kinetic-heating skin recovery temperature:
/// T_skin = T_amb * (1 + r * 0.2 * M^2), default turbulent recovery factor r = 0.85.
double kineticSkinTemperatureK(double ambientK, double mach, double recoveryFactor = 0.85) noexcept;

/// Altitude-stratified Beer-Lambert + Kruse visibility + cloud/precipitation optical
/// transmittance in [0, 1] over passband [lowUm, highUm] along slant path `rangeM`.
double atmosphericTransmittance(double rangeM, double hSensorM, double hTargetM,
                                double lowUm, double highUm,
                                const sim::EnvironmentState& env) noexcept;

/// Physical contrast radiant intensity (W/sr) of an entity in passband [lowUm, highUm]
/// toward `sensorPosEcef`, combining aerodynamic kinetic-heating skin contrast, aspect-
/// dependent turbine nozzle/plume emission, and burning solid-rocket plumes.
double targetRadiantIntensityWSr(const sim::VehicleState& target, Role role, bool weaponBurning,
                                 const double sensorPosEcef[3], double lowUm, double highUm,
                                 double extentRmsM, double ambientSeaLevelK = 288.15) noexcept;

/// Two-way Lambertian laser rangefinder optical return SNR:
/// P_r = (P_t * A_r * rho * eta_sys * tau_atm^2) / (pi * R^2), SNR = P_r / nepW.
double laserReturnSnr(double rangeM, double hSensorM, double hTargetM,
                      double peakPowerW, double apertureDiameterM, double wavelengthUm,
                      double reflectance, double nepW,
                      const sim::EnvironmentState& env) noexcept;

/// Construct a passive/active optical sensor model from its payload card.
std::unique_ptr<Model> makeOpticalSensor(const Card& card, std::uint16_t count);

} // namespace fsim::ms::optical
