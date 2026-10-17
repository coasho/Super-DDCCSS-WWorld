#pragma once

#include <functional>

namespace fsim::world::cocip {

/// The contrail cirrus prediction model CoCiP (Schumann 2012) as pycontrails has it (contrailcirrus/pycontrails,
/// Apache-2.0, (c) Contrails.org and the Breakthrough Energy Foundation: pycontrails/models/sac.py, models/cocip/wake_vortex.py,
/// models/cocip/contrail_properties.py, models/cocip/wind_shear.py, models/cocip/cocip.py, physics/thermo.py,
/// physics/constants.py, core/fuel.py), ported function for function with its default parameters (CocipParams,
/// SACParams, EmissionsParams, JetA): whether a contrail forms (the Schmidt-Appleman criterion), where the wake
/// vortex carries it and how much of its ice survives, and then, step by step, how its Gaussian plume spreads, its ice
/// mixes with the air and falls out, how deep it is optically and whether it persists.

/// The air at a point (K, Pa, specific humidity kg/kg).
struct Air {
    double temperature = 0.0, pressure = 0.0, specificHumidity = 0.0;
};
/// The air round a point, by height (m): what CoCiP reads from its weather data.
using AirAt = std::function<Air(double altitudeM)>;

/// The wind's shear there: its magnitude and its part across the flight track (1/s).
struct Shear {
    double total = 0.0, normal = 0.0;
};

/// The aircraft making it.
struct Aircraft {
    double wingspan = 10.0;   ///< m
    double mass = 10000.0;    ///< kg
    double trueAirspeed = 0.0;///< m/s
    double fuelFlow = 0.0;    ///< kg/s
};

// physics/thermo.py, constants.py
double eSatIce(double T);
double eSatLiquid(double T);
double eSatLiquidPrime(double T);
double qSatIce(double T, double p);
double rhoDry(double T, double p);
double rhi(double q, double T, double p);
double rh(double q, double T, double p);
double potentialTemperature(double T, double p);
double bruntVaisala(double p, double T, double dThetaDz);

/// The Schmidt-Appleman criterion (models/sac.py): the mixing line's slope G, the threshold temperature T_LM, the
/// critical relative humidity, whether it holds, and T_critical_sac.
struct Formation {
    bool forms = false;
    double G = 0.0, tLM = 0.0, rhCritical = 0.0, tCritical = 0.0;
};
Formation formation(const Air& air, double engineEfficiency = 0.3);
/// The same for any exhaust (a rocket's): its water and the heat it brings the plume, each per kg of what it burns
/// (the mixing line's slope G = EI_H2O c_p p / (epsilon Q (1 - eta)), with Q (1 - eta) that heat).
Formation formationOf(const Air& air, double waterPerKg, double plumeHeatPerKg);
/// The share of the exhaust's particles that freeze into ice crystals (cocip.py's activation of the soot near the
/// threshold: all of them 5 K or more below T_critical_sac, a third at it).
double iceActivation(double temperature, double tCritical);

/// A plume's state (cocip.py's per-waypoint variables, after calc_contrail_properties).
struct Plume {
    double altitude = 0.0;     ///< m: its centre
    double width = 0.0, depth = 0.0, sigmaYz = 0.0;
    double iwc = 0.0;          ///< ice water content, kg/kg
    double nIcePerM = 0.0;     ///< ice particles per metre of trail
    // calc_contrail_properties
    double dsDz = 0.0, dsnDz = 0.0; ///< the shear, enhanced (wind_shear_enhancement_factor)
    double areaEff = 0.0, plumeMassPerM = 0.0, rIceVol = 0.0, fallSpeed = 0.0, diffuseH = 0.0, diffuseV = 0.0;
    double nIcePerVol = 0.0, tau = 0.0, dnDtAgg = 0.0, dnDtTurb = 0.0;
    double age = 0.0;          ///< s
    bool alive = false;
};

/// The wake vortex phase and the first persistent contrail (cocip.py _simulate_wake_vortex_downwash,
/// _find_initial_persistent_contrails): the plume just behind the aircraft (state 0: the jets mixed, before the
/// descent) and after its vortex pair has carried it down (state 1, the CoCiP contrail's start); `persistent`
/// false when its ice does not survive the descent (a short trail, gone in the vortex phase).
struct Downwash {
    Formation sac;
    double dzMax = 0.0;       ///< m, the vortex pair's descent
    double vortexSpeed = 0.0; ///< m/s, its initial descent speed
    double separation = 0.0;  ///< m, the pair's spacing b0 (wake_vortex_separation)
    double epsilonStar = 0.0; ///< the air's turbulence normalised by the pair (normalized_dissipation_rate)
    Plume young;              ///< state 0 (the width and depth CoCiP takes for its initial ice)
    Plume plume;              ///< state 1
    double fSurv = 0.0;
    bool persistent = false;
};
Downwash downwash(const Aircraft& aircraft, double altitudeM, const AirAt& air, const Shear& shear);

/// The vortex pair's lifespan to linking (s): Crow and Bate's (1976) law in the normalised dissipation rate
/// eps* = (eps b0)^(1/3) / w0 - eps* = T* exp(-0.70 T*) for eps* <= 0.2535, T* = 0.41 / eps* above - with
/// T* = t w0 / b0.
double crowBateLinkTime(double epsilonStar, double separation, double descentSpeed);

/// calc_contrail_properties: what follows from the plume's width, depth, ice and the air there.
void properties(Plume& plume, const Air& air, double dThetaDz, const Shear& shear);

/// One time step (calc_timestep_contrail_evolution, the original ice budget, no stretching, the wind's vertical
/// velocity zero): the plume spreads, sediments, mixes and loses particles; then its properties, and whether it
/// persists (contrail_persistent with CocipParams' limits). The plume's height changes; `air` gives the air there.
/// altitudeLimits false: without CocipParams' altitude bounds (6 - 13 km), its weather data's domain rather than the
/// physics - for a trail laid above or below it (a missile's).
void step(Plume& plume, const AirAt& air, const Shear& shear, double dt, bool altitudeLimits = true);

/// A plume of something that neither falls nor sublimates - a rocket's smoke - spread over a time step as step()
/// spreads the ice's: by the air's turbulence and the shear (plume_temporal_evolution, its diffusivities as
/// calc_contrail_properties has them). Its width, depth and sigma_yz change; nothing else.
void disperse(Plume& plume, const AirAt& air, const Shear& shear, double dt);

/// CoCiP's initial ice water content (contrail_properties.initial_iwc) for a plume of this width and depth: the
/// exhaust's water spread over it, with the air's, less ice saturation (kg/kg).
double initialIwc(const Air& air, double fuelDist, double width, double depth);
/// The same for the exhaust's water itself (kg per metre of trail).
double initialIwcOfWater(const Air& air, double waterDist, double width, double depth);

/// The potential temperature's gradient at a height (K/m), as cocip.py takes it from the air 200 m below.
double potentialTemperatureGradient(const AirAt& air, double altitudeM);

/// The extinction per metre of trail (m): its optical depth across its width, times the width.
inline double extinctionPerMetre(const Plume& p) { return p.tau * p.width; }

} // namespace fsim::world::cocip
