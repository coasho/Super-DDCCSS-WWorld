#include "simulation/mission_systems/damage/Debris.h"

namespace fsim::ms::damage {

void geodetic(const V3& p, double& lat, double& lon, double& h) noexcept {
    constexpr double a = 6378137.0, f = 1.0 / 298.257223563, b = a * (1.0 - f), e2 = f * (2.0 - f), ep2 = e2 / (1.0 - e2);
    const double r = std::hypot(p.x, p.y);
    lon = std::atan2(p.y, p.x);
    const double th = std::atan2(p.z * a, r * b);
    lat = std::atan2(p.z + ep2 * b * std::pow(std::sin(th), 3), r - e2 * a * std::pow(std::cos(th), 3));
    const double sl = std::sin(lat), N = a / std::sqrt(1.0 - e2 * sl * sl);
    h = std::abs(std::cos(lat)) > 1e-9 ? r / std::cos(lat) - N : std::abs(p.z) - b;
}

double standardDensity(double h) noexcept {
    // ISA: the troposphere's lapse to 11 km, isothermal (216.65 K) above
    constexpr double T0 = 288.15, p0 = 101325.0, L = 0.0065, R = 287.05287, g = 9.80665;
    if (h < 11000.0) {
        const double T = T0 - L * std::max(h, -500.0);
        return p0 * std::pow(T / T0, g / (L * R)) / (R * T);
    }
    const double p11 = p0 * std::pow(216.65 / T0, g / (L * R));
    return p11 * std::exp(-g * (h - 11000.0) / (R * 216.65)) / (R * 216.65);
}

} // namespace fsim::ms::damage
