#include "world/Cocip.h"

#include <algorithm>
#include <cmath>
#include <limits>

// Ported from pycontrails (contrailcirrus/pycontrails, Apache-2.0, (c) Contrails.org and the Breakthrough Energy Foundation); each function names the
// one it follows. Their defaults are kept; where the viewer's world gives what their weather data would, Airflow
// says so.

namespace fsim::world::cocip {

namespace {

constexpr double kPi = 3.14159265358979323846;
// physics/constants.py
constexpr double kG = 9.80665, kRd = 287.05, kRv = 461.51, kEpsilon = kRd / kRv, kCpd = 1004.0, kCpv = 1870.0;
constexpr double kRhoIce = 917.0, kGamma = 1.4, kPSurface = 101325.0, kMuIce = 1.31, kLambdaLight = 550e-9, kCr = 0.9;
constexpr double kAbsoluteZero = -273.15;
// core/fuel.py JetA
constexpr double kQFuel = 43.13e6, kEiH2O = 1.23;
// models/emissions EmissionsParams.default_nvpm_ei_n; CocipParams
constexpr double kNvpmEiN = 1e15, kMinAei = 1e13;
constexpr double kDzM = 200.0, kEffectiveVerticalResolution = 2000.0, kWindShearEnhancementExponent = 0.5;
constexpr double kInitialWakeVortexDepth = 0.5, kTurbulentVerticalVelocityScale = 0.1, kSedimentationImpactFactor = 0.5;
constexpr double kMaxDepth = 1500.0, kMinAltitude = 6000.0, kMaxAltitude = 13000.0, kMaxAge = 20.0 * 3600.0;
constexpr double kMinTau = 1e-6, kMaxTau = 1e10, kMinNIce = 1e3, kMaxNIce = 1e20;

// wind_shear.wind_shear_enhancement_factor
double windShearEnhancementFactor(double depth) {
    return 0.5 * (1.0 + std::pow(kEffectiveVerticalResolution / std::max(depth, 1e-6), kWindShearEnhancementExponent));
}

// thermo.c_pm
double cPm(double q) { return kCpd * (1.0 - q) + kCpv * q; }

// contrail_properties.scattering_extinction_efficiency / light_wave_phase_delay
double extinctionEfficiency(double r) {
    const double phase = std::min(4.0 * kPi * (kMuIce - 1.0) / kLambdaLight * r, 100.0);
    return 2.0 - (4.0 / phase) * (std::sin(phase) - (1.0 - std::cos(phase)) / phase);
}

// contrail_properties.ice_particle_terminal_fall_speed (and ice_particle_mass)
double terminalFallSpeed(double p, double T, double r) {
    const double ipm = (4.0 / 3.0) * kPi * r * r * r * kRhoIce;
    double alpha;
    if (ipm >= 4.264e-8) alpha = 8.80 * std::pow(ipm, 0.096);
    else if (ipm >= 2.166e-9) alpha = 329.8 * std::pow(ipm, 0.31);
    else if (ipm >= 2.146e-13) alpha = 63292.4 * std::pow(ipm, 0.57);
    else alpha = 735.4 * std::pow(ipm, 0.42);
    return alpha * std::pow(30000.0 / p, 0.178) * std::pow(233.0 / T, 0.394);
}

} // namespace

// thermo.e_sat_ice (Sonntag)
double eSatIce(double T) {
    return 100.0 * std::exp(-6024.5282 / T + 24.7219 + 0.010613868 * T - 1.3198825e-5 * T * T - 0.49382577 * std::log(T));
}

// thermo.mk05_e_sat_liquid (Murphy and Koop 2005)
double eSatLiquid(double T) {
    return std::exp(54.842763 - 6763.22 / T - 4.21 * std::log(T) + 0.000367 * T +
                    std::tanh(0.0415 * (T - 218.8)) * (53.878 - 1331.22 / T - 9.44523 * std::log(T) + 0.014025 * T));
}

// thermo.mk05_e_sat_liquid_prime
double eSatLiquidPrime(double T) {
    const double tanhTerm = std::tanh(0.0415 * (T - 218.8));
    return eSatLiquid(T) * (6763.22 / (T * T) - 4.21 / T + 0.000367 +
                            0.0415 * (1.0 - tanhTerm * tanhTerm) * (53.878 - 1331.22 / T - 9.44523 * std::log(T) + 0.014025 * T) +
                            tanhTerm * (1331.22 / (T * T) - 9.44523 / T + 0.014025));
}

double qSatIce(double T, double p) { return kEpsilon * eSatIce(T) / p; }
double rhoDry(double T, double p) { return p / (kRd * T); }
double rhi(double q, double T, double p) { return q * p / (kEpsilon * eSatIce(T)); }
double rh(double q, double T, double p) { return q * p / (kEpsilon * eSatLiquid(T)); }
double potentialTemperature(double T, double p) { return T * std::pow(kPSurface / p, kRd / kCpd); }

// thermo.brunt_vaisala_frequency
double bruntVaisala(double p, double T, double dThetaDz) {
    return std::sqrt(std::max(dThetaDz, 1e-6) * kG / potentialTemperature(T, p));
}

// thermo.T_potential_gradient over cocip.py's dz_m below the point
double potentialTemperatureGradient(const AirAt& air, double altitudeM) {
    const Air top = air(altitudeM), btm = air(altitudeM - kDzM);
    return (potentialTemperature(top.temperature, top.pressure) - potentialTemperature(btm.temperature, btm.pressure)) / kDzM;
}

// contrail_properties.initial_iwc and q_exhaust
double initialIwc(const Air& air, double fuelDist, double width, double depth) {
    return initialIwcOfWater(air, kEiH2O * fuelDist, width, depth);
}

double initialIwcOfWater(const Air& air, double waterDist, double width, double depth) {
    const double rho = rhoDry(air.temperature, air.pressure);
    const double qExhaust = waterDist / ((kPi / 4.0) * width * depth * rho);
    return std::max(qExhaust + air.specificHumidity - qSatIce(air.temperature, air.pressure), 0.0);
}

namespace {
Formation formationWith(const Air& air, double G);
}

// sac.slope_mixing_line, T_sat_liquid, rh_critical_sac, sac, T_critical_sac
Formation formation(const Air& air, double engineEfficiency) {
    const double p = air.pressure, q = air.specificHumidity;
    return formationWith(air, kEiH2O * cPm(q) * p / (kEpsilon * kQFuel * (1.0 - engineEfficiency)));
}

Formation formationOf(const Air& air, double waterPerKg, double plumeHeatPerKg) {
    return formationWith(air, waterPerKg * cPm(air.specificHumidity) * air.pressure / (kEpsilon * plumeHeatPerKg));
}

// cocip.py's activation (in _find_initial_persistent_contrails)
double iceActivation(double temperature, double tCritical) {
    const double dTemp = std::min(temperature - tCritical, 0.0);
    return dTemp < -5.0 ? 1.0 : -0.661 * std::exp(dTemp) + 1.0;
}

namespace {
Formation formationWith(const Air& air, double G) {
    Formation f;
    const double T = air.temperature, p = air.pressure, q = air.specificHumidity;
    f.G = G;
    if (!(f.G > 0.053)) return f;
    const double logG = std::log(f.G - 0.053);
    f.tLM = -46.46 - kAbsoluteZero + 9.43 * logG + 0.72 * logG * logG;
    double rhCrit = (f.G * (T - f.tLM) + eSatLiquid(f.tLM)) / eSatLiquid(T);
    rhCrit = std::clamp(rhCrit, 0.0, 1.0);
    if (T > f.tLM) rhCrit = std::numeric_limits<double>::infinity();
    f.rhCritical = rhCrit;
    const double u = rh(q, T, p);
    f.forms = u > rhCrit;
    // T_critical_sac: Newton's method from T_LM - 1 (maxiter 10)
    f.tCritical = f.tLM;
    if (u < 0.999 && std::isfinite(f.tLM)) {
        const double eL = eSatLiquid(f.tLM);
        double t = f.tLM - 1.0;
        for (int i = 0; i < 10; ++i) {
            const double func = t - f.tLM + (eL - u * eSatLiquid(t)) / f.G;
            const double prime = 1.0 - u * eSatLiquidPrime(t) / f.G;
            if (prime == 0.0) break;
            const double next = t - func / prime;
            if (!std::isfinite(next)) break;
            const bool done = std::abs(next - t) < 1.48e-8;
            t = next;
            if (done) break;
        }
        f.tCritical = t;
    }
    return f;
}
} // namespace

double crowBateLinkTime(double epsilonStar, double separation, double descentSpeed) {
    const double e = std::max(epsilonStar, 1e-6);
    double T;
    if (e > 0.2535) {
        T = 0.41 / e;
    } else {
        // eps* = T exp(-0.7 T) on its falling branch (T > 1 / 0.7), by Newton's method
        T = std::max(-std::log(e) / 0.7 * 1.2, 1.0 / 0.7 + 0.5);
        for (int i = 0; i < 30; ++i) {
            const double f = T * std::exp(-0.7 * T) - e, df = std::exp(-0.7 * T) * (1.0 - 0.7 * T);
            if (df == 0.0) break;
            const double next = std::max(T - f / df, 1.0 / 0.7 + 1e-6);
            if (std::abs(next - T) < 1e-9) {
                T = next;
                break;
            }
            T = next;
        }
    }
    return T * separation / std::max(descentSpeed, 1e-3);
}

void properties(Plume& c, const Air& air, double dThetaDz, const Shear& shear) {
    const double T = air.temperature, p = air.pressure;
    const double rho = rhoDry(T, p);
    // shear enhancements
    const double enhancement = windShearEnhancementFactor(c.depth);
    c.dsDz = shear.total * enhancement;
    c.dsnDz = shear.normal * enhancement;
    // effective area (plume_effective_cross_sectional_area, plume_effective_depth)
    const double syy = 0.125 * c.width * c.width, szz = 0.125 * c.depth * c.depth;
    c.areaEff = 2.0 * kPi * std::sqrt(std::max(syy * szz - c.sigmaYz * c.sigmaYz, 0.0));
    const double area = std::max(c.areaEff, 1e-9);
    const double depthEff = area / std::max(c.width, 1e-9);
    // ice particles
    c.nIcePerVol = c.nIcePerM / area;
    const double nIcePerKg = c.nIcePerVol / rho;
    c.plumeMassPerM = area * rho;
    // ice_particle_volume_mean_radius
    double r = c.iwc;
    if (c.iwc > 0.0 && nIcePerKg > 0.0) r = std::cbrt((3.0 / (4.0 * kPi)) * (c.iwc / kRhoIce / nIcePerKg));
    c.rIceVol = std::max(r, 1e-10);
    // contrail_optical_depth
    c.tau = c.rIceVol <= 1e-9 ? 0.0 : std::max(kCr * kPi * c.rIceVol * c.rIceVol * (c.nIcePerM / c.width) * extinctionEfficiency(c.rIceVol), 0.0);
    c.fallSpeed = terminalFallSpeed(p, T, c.rIceVol);
    // horizontal_diffusivity, vertical_diffusivity (no radiative heating)
    c.diffuseH = 0.1 * c.dsDz * c.depth * c.depth;
    const double nBv = std::max(bruntVaisala(p, T, dThetaDz), 0.001);
    c.diffuseV = kTurbulentVerticalVelocityScale * kTurbulentVerticalVelocityScale / nBv + kSedimentationImpactFactor * c.fallSpeed * depthEff;
    // particle_losses_aggregation, particle_losses_turbulence
    c.dnDtAgg = 8.0 * kPi * c.rIceVol * c.rIceVol * c.fallSpeed / area;
    const double wd = std::max(c.width, c.depth);
    c.dnDtTurb = 0.1 * std::abs(c.diffuseH / (wd * wd) + c.diffuseV / (depthEff * depthEff));
}

Downwash downwash(const Aircraft& ac, double altitudeM, const AirAt& airAt, const Shear& shear) {
    Downwash d;
    const Air air = airAt(altitudeM);
    const double T = air.temperature, p = air.pressure;
    d.sac = formation(air);
    if (!d.sac.forms || ac.trueAirspeed <= 1.0 || ac.fuelFlow <= 0.0) return d;
    const double b = ac.wingspan, m = ac.mass, V = ac.trueAirspeed;
    const double dThetaDz = potentialTemperatureGradient(airAt, altitudeM);

    // wake_vortex.max_downward_displacement
    const double rho = rhoDry(T, p);
    const double nBv = bruntVaisala(p, T, dThetaDz);
    const double t0 = std::pow(kPi, 4) / 32.0 * b * b * b * rho * V / (m * kG);
    const double dzStrong = (1.49 * 16.0) / (2.0 * std::pow(kPi, 3)) * m * kG / (b * b * rho * V * nBv);
    double dzMax = dzStrong;
    double epsnSt;
    {
        const double dz = std::max(dzStrong, 10.0);
        const double enhancement = windShearEnhancementFactor(dz);
        const double epsn = 0.5 * kTurbulentVerticalVelocityScale * kTurbulentVerticalVelocityScale * (shear.total * enhancement * enhancement);
        const double c = std::cbrt(kPi / 4.0) * std::pow(kPi, 3) / 8.0;
        epsnSt = std::min(c * std::cbrt(epsn * b) * b * b * rho * V / (kG * m), 0.36);
    }
    if (nBv * t0 < 0.8) {
        const double b0 = kPi * b / 4.0;
        dzMax = b0 * (7.68 * (1.0 - 4.07 * epsnSt + 5.67 * epsnSt * epsnSt) * (0.79 - nBv * t0) + 1.88);
    }
    d.dzMax = dzMax;
    d.epsilonStar = epsnSt;
    d.separation = kPi * b / 4.0;
    // the vortex pair's initial circulation (unterstrasser_wake_vortex._initial_wake_vortex_circulation) and its
    // descent speed, Gamma0 / (2 pi b0)
    {
        const double b0 = kPi * b / 4.0;
        const double gamma0 = kG * m / (rho * b0 * V);
        d.vortexSpeed = gamma0 / (2.0 * kPi * b0);
    }
    const double width = kPi / 4.0 * b, depth = dzMax * kInitialWakeVortexDepth;

    // _find_initial_persistent_contrails
    const double fuelDist = ac.fuelFlow / V;
    const double iwc = initialIwc(air, fuelDist, width, depth);
    const double altitude1 = altitudeM - 0.5 * depth;
    const Air air1 = airAt(altitude1);
    double iwcAd;
    {
        const double T1 = T * std::pow(air1.pressure / p, (kGamma - 1.0) / kGamma);
        iwcAd = std::max(kRd / kRv * (eSatIce(T1) / air1.pressure - eSatIce(T) / p), 0.0);
    }
    const double iwc1 = std::max(iwc - iwcAd, 0.0);
    const double fActivation = iceActivation(T, d.sac.tCritical);
    const double aei = std::max(kNvpmEiN * fActivation, kMinAei);
    const double n0 = fuelDist * aei;
    d.fSurv = (iwc > 0.0 && iwc1 > 0.0) ? std::min(iwc1 / iwc, 1.0) : 0.5;
    const double rhi1 = rhi(air1.specificHumidity, air1.temperature, air1.pressure);
    d.persistent = iwc1 > 1e-12 && iwc1 < 1e10 && rhi1 > 0.0 && rhi1 < 1e10;

    d.young.altitude = altitudeM;
    d.young.width = width;
    d.young.depth = depth;
    d.young.iwc = iwc;
    d.young.nIcePerM = n0;
    d.young.alive = iwc > 0.0;
    properties(d.young, air, dThetaDz, shear);

    d.plume.altitude = altitude1;
    d.plume.width = width;
    d.plume.depth = depth;
    d.plume.iwc = iwc1;
    d.plume.nIcePerM = n0 * d.fSurv;
    d.plume.alive = d.persistent;
    properties(d.plume, air1, potentialTemperatureGradient(airAt, altitude1), shear);
    return d;
}

void step(Plume& c, const AirAt& airAt, const Shear& shear, double dt, bool altitudeLimits) {
    if (!c.alive || dt <= 0.0) return;
    const Air air1 = airAt(c.altitude);
    // plume_temporal_evolution (seg_ratio 1)
    const double syy = 0.125 * c.width * c.width, szz = 0.125 * c.depth * c.depth;
    double diffuseV = c.diffuseV;
    {
        const double maxSzz = 0.125 * kMaxDepth * kMaxDepth;
        diffuseV = std::min(diffuseV, (maxSzz - szz) / (2.0 * dt));
    }
    const double s2 = c.dsnDz * c.dsnDz, dt2 = dt * dt, dt3 = dt * dt2;
    const double syy2 = (2.0 / 3.0) * s2 * diffuseV * dt3 + s2 * szz * dt2 + 2.0 * (c.diffuseH + c.dsnDz * c.sigmaYz) * dt + syy;
    const double szz2 = 2.0 * diffuseV * dt + szz;
    const double syz2 = c.dsnDz * diffuseV * dt2 + c.dsnDz * szz * dt + c.sigmaYz;
    // new_contrail_dimensions
    const double width2 = std::sqrt(std::max(8.0 * syy2, 0.0)), depth2 = std::sqrt(std::max(8.0 * szz2, 0.0));
    // geo.advect_level: the ice falls (no vertical wind)
    const double altitude2 = c.altitude - c.fallSpeed * dt;
    const Air air2 = airAt(altitude2);
    const double rho2 = rhoDry(air2.temperature, air2.pressure);
    const double qSat1 = qSatIce(air1.temperature, air1.pressure), qSat2 = qSatIce(air2.temperature, air2.pressure);
    // new_effective_area_from_sigma, plume_mass_per_distance
    const double area2 = 2.0 * kPi * std::sqrt(std::max(syy2 * szz2 - syz2 * syz2, 0.0));
    const double mass2 = area2 * rho2;
    // new_ice_water_content (the original budget)
    double iwc2;
    {
        const double qMean = 0.5 * (air1.specificHumidity + air2.specificHumidity);
        const double h2o1 = c.plumeMassPerM * (c.iwc + qSat1);
        const double h2o2 = h2o1 + (mass2 - c.plumeMassPerM) * qMean;
        iwc2 = mass2 > 0.0 ? std::max(h2o2 / mass2 - qSat2, 0.0) : 0.0;
    }
    // new_ice_particle_number
    double n2;
    {
        const double n1 = std::max(c.nIcePerM, 0.0);
        const double expTerm = c.dnDtTurb * dt < 80.0 ? std::exp(-c.dnDtTurb * dt) : 0.0;
        if (c.dnDtTurb * dt < 1e-5) {
            n2 = n1 / (1.0 + c.dnDtAgg * dt * n1);
        } else {
            const double numerator = c.dnDtTurb * n1 * expTerm;
            const double denominator = c.dnDtTurb + c.dnDtAgg * n1 * (1.0 - expTerm);
            n2 = denominator > 0.0 ? numerator / denominator : 0.0;
        }
        n2 = std::max(n2, 0.0);
    }
    c.width = width2;
    c.depth = depth2;
    c.sigmaYz = syz2;
    c.altitude = altitude2;
    c.iwc = iwc2;
    c.nIcePerM = n2;
    c.age += dt;
    properties(c, air2, potentialTemperatureGradient(airAt, altitude2), shear);
    // contrail_persistent
    c.alive = c.age < kMaxAge && c.tau >= kMinTau && c.tau <= kMaxTau && c.nIcePerVol >= kMinNIce && c.nIcePerVol <= kMaxNIce &&
              (!altitudeLimits || (c.altitude >= kMinAltitude && c.altitude <= kMaxAltitude)) && std::isfinite(c.width);
}


void disperse(Plume& c, const AirAt& airAt, const Shear& shear, double dt) {
    if (dt <= 0.0) return;
    // the diffusivities and the enhanced shear as calc_contrail_properties has them (no ice: nothing falls), then
    // plume_temporal_evolution as step() takes it
    properties(c, airAt(c.altitude), potentialTemperatureGradient(airAt, c.altitude), shear);
    const double syy = 0.125 * c.width * c.width, szz = 0.125 * c.depth * c.depth;
    const double maxSzz = 0.125 * kMaxDepth * kMaxDepth;
    const double diffuseV = std::min(c.diffuseV, (maxSzz - szz) / (2.0 * dt));
    const double s2 = c.dsnDz * c.dsnDz, dt2 = dt * dt, dt3 = dt * dt2;
    const double syy2 = (2.0 / 3.0) * s2 * diffuseV * dt3 + s2 * szz * dt2 + 2.0 * (c.diffuseH + c.dsnDz * c.sigmaYz) * dt + syy;
    const double szz2 = 2.0 * diffuseV * dt + szz;
    const double syz2 = c.dsnDz * diffuseV * dt2 + c.dsnDz * szz * dt + c.sigmaYz;
    c.width = std::sqrt(std::max(8.0 * syy2, 0.0));
    c.depth = std::sqrt(std::max(8.0 * szz2, 0.0));
    c.sigmaYz = syz2;
    c.age += dt;
}

} // namespace fsim::world::cocip
