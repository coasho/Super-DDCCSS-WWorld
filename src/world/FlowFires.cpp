#include "world/FlowFires.h"

#include <algorithm>
#include <cmath>

namespace fsim::world::flowfires {

namespace {

constexpr double kPi = 3.141592653589793;
constexpr double kStepS = 1.0 / 60.0;
constexpr double kRate = 60.0;
constexpr double kLeakTubeM = 8.5;  // a leak's luminous length with no burning time: its tube and the products' spread        // a source's cells to its values within a step (Flow's couple rate, 1/s)
// the smoke Flow's advection adds as what holds it expands (measured: 1.29 - 1.36 times the soot put in, after the
// fireball): the soot put in that much less
constexpr double kSmokeKept = 0.75;

double thinner(double pressurePa) { return std::cbrt(101325.0 / std::max(pressurePa, 100.0)); }
double sphereVolume(double r) { return 4.0 / 3.0 * kPi * r * r * r; }
/// soot (the solver's smoke: 1/m) of so many kg of it spread through so many m^3
double soot(double sootKg, double volumeM3) { return kSmokeKept * kSootExtinction * sootKg / std::max(volumeM3, 1e-6); }

/// Random velocity (Gaussian, `rms` m/s each way) given to a sphere's cells: an eddy of the turbulence the source
/// stands in, from a seed (the same seed the same eddy).
void turbulence(FlowSolver::Sphere& s, double rms, std::uint64_t seed) {
    std::uint64_t x = seed * 6364136223846793005ull + 1442695040888963407ull;
    const auto uniform = [&]() {
        x ^= x >> 33, x *= 0xff51afd7ed558ccdull, x ^= x >> 33, x *= 0xc4ceb9fe1a85ec53ull, x ^= x >> 33;
        return (static_cast<double>(x >> 11) + 0.5) / 9007199254740992.0;
    };
    for (int a = 0; a < 3; ++a) {
        // (Box-Muller)
        const double n = std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * kPi * uniform());
        s.velocity[a] += rms * n;
    }
    s.velocityRate = kRate;
}

/// A moving source over the call: spheres along where it went (`moved`, m, back from where it is), at most 3/4 of its
/// radius apart - the solver's own trace follows a source's velocity, and a source's cells are its fuel's, not its
/// gas's - each its own luid. How much of the swept tube they cover: its volume (m^3), for what is spread through it.
double along(const FlowSolver::Sphere& s, const double moved[3], std::uint64_t luid, std::vector<FlowSolver::Sphere>& out) {
    const double length = std::sqrt(moved[0] * moved[0] + moved[1] * moved[1] + moved[2] * moved[2]);
    const int n = std::clamp(static_cast<int>(std::ceil(length / (0.75 * s.radius))), 1, 32);
    for (int j = 0; j < n; ++j) {
        FlowSolver::Sphere k = s;
        const double back = n > 1 ? static_cast<double>(j) / static_cast<double>(n - 1) : 0.0;
        for (int a = 0; a < 3; ++a) k.at[a] = s.at[a] - moved[a] * back;
        k.luid = luid * 64u + static_cast<std::uint64_t>(j);
        k.traceSamples = 0;
        out.push_back(k);
    }
    return kPi * s.radius * s.radius * length + sphereVolume(s.radius);
}

} // namespace

double pressureAt(const Air& air, double altitudeM) {
    // the standard atmosphere's troposphere and lower stratosphere, from the day's sea level
    const double h = std::max(altitudeM, -500.0);
    const double T0 = air.seaLevelK, lapse = 0.0065, g = 9.80665, R = 287.053;
    const double tropopause = 11000.0;
    if (h <= tropopause) return air.seaLevelPa * std::pow(1.0 - lapse * h / T0, g / (R * lapse));
    const double p11 = air.seaLevelPa * std::pow(1.0 - lapse * tropopause / T0, g / (R * lapse));
    return p11 * std::exp(-g * (h - tropopause) / (R * (T0 - lapse * tropopause)));
}

double ccpsDiameter(double fuelKg, double pressurePa) { return 5.8 * std::cbrt(std::max(fuelKg, 0.0)) * thinner(pressurePa); }
double ccpsDuration(double fuelKg) { return 0.45 * std::cbrt(std::max(fuelKg, 0.0)); }
double detonationDiameter(double tntKg, double pressurePa) {
    // (5.89 W^0.31 m fired at the ground - a hemisphere - and 2^(-1/3) of it in free air: 4.67 W^0.31 m)
    return 5.89 / std::cbrt(2.0) * std::pow(std::max(tntKg, 0.0), 0.31) * thinner(pressurePa);
}
double detonationDuration(double tntKg) { return 0.06 * std::cbrt(std::max(tntKg, 0.0)); }
double jetFlameLength(double rateKgS) { return 0.00326 * std::pow(std::max(rateKgS, 0.0) * 43.0e6, 0.478); }

FlowSolver::Combustion combustion() {
    FlowSolver::Combustion k;
    k.coolingRate = kCooling;
    k.smokePerBurn = 0.0;
    k.divergencePerBurn = 0.0;
    // hot gas rises as its density deficit, (T - T0) / T0 of the air's: per unit of the solver's temperature
    k.buoyancyPerTemp = (kFlameK - kAmbientK) / kAmbientK;
    // its smoke thinning as a plume's does (Flow's advection keeps what it carries: no mixing with the air), so the
    // solver's blocks free once it is gone
    k.smokeFade = 0.0, k.smokeDamping = kSmokeDilution;
    // (and the gas's motion dying away - its expansion's, its rise's - so the blocks free where nothing moves)
    k.velocityDamping = kVelocityDamping;
    // (a block with no smoke left in it freed once its gas is slower than a wake's last stirring)
    k.keepSpeed = kKeepSpeed;
    return k;
}

bool detonation(double tntKg, const double at[3], const double drift[3], double pressurePa, double age, double dt, std::uint64_t luid,
                std::vector<FlowSolver::Sphere>& out) {
    const double burn = kDetonationBurnS * std::cbrt(tntKg);
    if (!(tntKg > 0.0) || age > burn || age < -dt) return age <= burn;
    const double R = 0.5 * detonationDiameter(tntKg, pressurePa);
    FlowSolver::Sphere s;
    s.radius = std::max(kDetonationCore * std::pow(thinner(pressurePa), kDetonationCoreByAir) * R, 1.0);
    for (int a = 0; a < 3; ++a) s.at[a] = at[a] + drift[a] * std::max(age, 0.0), s.velocity[a] = drift[a];
    s.velocityRate = kRate;
    s.temperature = 1.0, s.temperatureRate = kRate;
    s.smoke = soot(kTntSoot * tntKg, sphereVolume(R)), s.smokeRate = kRate;
    if (age < kDetonationGrowSteps * kStepS) {
        s.divergence = kDetonationVolume * std::pow(tntKg / 8.0, kDetonationVolumeByMass) * std::pow(thinner(pressurePa), kDetonationVolumeByAir) *
                       sphereVolume(R) / (kDetonationGrowSteps * kStepS) / sphereVolume(s.radius);
        s.divergenceRate = kRate;
    }
    s.luid = luid;
    out.push_back(s);
    return true;
}

bool spill(double fuelKg, const double at[3], const double path[3], double pressurePa, double age, double dt, std::uint64_t luid,
           std::vector<FlowSolver::Sphere>& out) {
    const double grow = ccpsDuration(fuelKg) / 3.0;
    if (!(fuelKg > 0.0) || age > std::max(grow, kSpillReleaseS) || age < -dt) return age <= std::max(grow, kSpillReleaseS);
    const double cloud = std::cbrt(fuelKg) * thinner(pressurePa);
    const double fireball = 0.5 * ccpsDiameter(fuelKg, pressurePa);
    const double speed = std::sqrt(path[0] * path[0] + path[1] * path[1] + path[2] * path[2]);
    if (age < kSpillReleaseS) {
        // the fuel, lit, laid along the hole's path
        FlowSolver::Sphere s;
        s.radius = cloud;
        for (int a = 0; a < 3; ++a) s.at[a] = at[a] + path[a] * std::max(age, 0.0);
        s.fuel = kSpillFuel * std::cbrt(fuelKg), s.fuelRate = kRate;
        s.temperature = 1.0, s.temperatureRate = kRate;
        s.smoke = soot(kSootYield * fuelKg, sphereVolume(fireball)), s.smokeRate = kRate;
        // the spray leaving with some of the hole's speed: its shear with the still air the fireball's turbulence
        for (int a = 0; a < 3; ++a) s.velocity[a] = kSpillCarry * path[a];
        s.velocityRate = kRate;
        const double since = std::min(dt, std::max(age, 0.0));
        const double moved[3] = {path[0] * since, path[1] * since, path[2] * since};
        (void)speed;
        along(s, moved, luid, out);
    }
    if (age < grow) {
        // its products' volume, made where the fuel is: what the cloud pushes out is the burning fuel
        FlowSolver::Sphere g;
        g.radius = cloud;
        for (int a = 0; a < 3; ++a) g.at[a] = at[a] + path[a] * 0.5 * kSpillReleaseS;
        g.divergence = kSpillVolume * std::pow(100.0 / fuelKg, 0.2) * sphereVolume(fireball) / grow / sphereVolume(cloud);
        g.divergenceRate = kRate;
        g.luid = (luid + 1) * 64u;
        out.push_back(g);
    }
    return true;
}

double wakeCirculation(double massKg, double spanM, double speedMs, double densityKgM3) {
    const double b0 = kPi / 4.0 * std::max(spanM, 1.0);
    return std::max(massKg, 0.0) * 9.80665 / (std::max(densityKgM3, 0.05) * std::max(speedMs, 10.0) * b0);
}

void wake(double massKg, double spanM, const double at[3], const double velocity[3], const double right[3], const double up[3],
          double pressurePa, double airK, double dt, std::uint64_t luid, std::vector<FlowSolver::Sphere>& out) {
    const double speed = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
    if (!(massKg > 0.0) || !(spanM > 0.0) || speed < 30.0) return;
    const double rho = pressurePa / (287.05 * std::max(airK, 150.0));
    const double gamma = wakeCirculation(massKg, spanM, speed, rho);
    const double b0 = kPi / 4.0 * spanM;
    // the pair's flow across the wake (Biot-Savart of two Lamb-Oseen vortices, its core some 5 % of the span, two cells
    // at least), given at a grid of points across it - kWakeGrid a side over 1.5 times the spacing - each a sphere a cell
    // of it round
    const double core = std::max(kWakeCore * spanM, 1.0);
    const double extent = 1.5 * b0, cell = extent / kWakeGrid;
    const double moved[3] = {velocity[0] * dt, velocity[1] * dt, velocity[2] * dt};
    std::uint64_t k = 0;
    for (int iy = 0; iy < kWakeGrid; ++iy)
        for (int iz = 0; iz < kWakeGrid; ++iz) {
            const double y = -0.5 * extent + (iy + 0.5) * cell, z = -0.5 * extent + (iz + 0.5) * cell;
            double vRight = 0.0, vUp = 0.0;
            for (int side = -1; side <= 1; side += 2) {
                // a right vortex (side +1, at +b0/2) takes the air down inboard of it, up outboard
                const double dy = y - side * 0.5 * b0, dz = z, r2 = dy * dy + dz * dz;
                const double f = side * gamma / (2.0 * kPi * std::max(r2, 1e-6)) * (1.0 - std::exp(-r2 / (core * core)));
                vUp += f * dy, vRight += -f * dz;
            }
            FlowSolver::Sphere s;
            s.radius = 0.6 * cell;
            for (int a = 0; a < 3; ++a) {
                s.at[a] = at[a] + right[a] * y + up[a] * z;
                s.velocity[a] = right[a] * vRight + up[a] * vUp;
            }
            s.velocityRate = kRate;
            along(s, moved, luid * 64u + k++, out);
        }
}

void leak(double rateKgS, const double at[3], const double velocity[3], double pressurePa, double dt, std::uint64_t luid,
          std::vector<FlowSolver::Sphere>& out) {
    if (!(rateKgS > 0.0)) return;
    const double speed = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1] + velocity[2] * velocity[2]);
    // its fuel burning as long as the jet flame is long, at the hole's speed through the air (measured: the luminous
    // length kLeakTubeM, the swept tube's own, then 0.04 s of flight plus 0.95 s per unit of fuel)
    const double burn = std::max(jetFlameLength(rateKgS) - kLeakTubeM, 0.0) / std::max(speed, 10.0);
    const double products = kProductsM3PerKg * std::pow(thinner(pressurePa), 3.0) * rateKgS; // m^3/s
    FlowSolver::Sphere s;
    s.radius = kLeakRadiusM;
    for (int a = 0; a < 3; ++a) s.at[a] = at[a];
    s.fuel = std::max(0.03, (burn - 0.04) / 0.95), s.fuelRate = kRate;
    s.temperature = 1.0, s.temperatureRate = kRate;
    s.smoke = soot(kSootYield * rateKgS, products), s.smokeRate = kRate;
    // laid along the hole's path since the last call, its products' volume spread through what that covers, each sphere
    // given the near wake's turbulence (below)
    const double moved[3] = {velocity[0] * dt, velocity[1] * dt, velocity[2] * dt};
    const std::size_t first = out.size();
    const double covered = along(s, moved, luid, out);
    for (std::size_t k = first; k < out.size(); ++k) {
        out[k].divergence = products / covered, out[k].divergenceRate = kRate;
        // the turbulence close behind an aircraft (synthetic inflow: random eddies the sphere's size, kWakeTurbulence of
        // its airspeed rms - an estimate of a near wake's intensity), different each call
        turbulence(out[k], kWakeTurbulence * speed, luid * 7919u + static_cast<std::uint64_t>(out[k].at[0] * 4.0) * 104729u +
                                                        static_cast<std::uint64_t>(out[k].at[1] * 4.0) * 1299709u + k);
    }
}

} // namespace fsim::world::flowfires
