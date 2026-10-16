#include "simulation/mission_systems/damage/Impact.h"

#include <algorithm>
#include <cmath>

namespace fsim::ms::damage {

namespace {
double smooth(double lo, double hi, double x) noexcept {
    const double t = std::clamp((x - lo) / (hi - lo), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
} // namespace

StrikeOutcome groundStrike(const GroundStrike& in) noexcept {
    StrikeOutcome o;
    const V3 n = normalized(in.normal);
    const double m = std::max(in.massKg, 0.0);
    o.speedMs = length(in.velocity);
    o.normalMs = -dot(in.velocity, n); // (into the ground: positive)
    const V3 along = in.velocity + n * o.normalMs;
    o.tangentMs = length(along);
    o.energyJ = 0.5 * m * o.speedMs * o.speedMs;
    if (!(o.normalMs > 0.0) || !(m > 0.0)) return o;
    o.angleRad = std::atan2(o.normalMs, o.tangentMs);
    o.normalEnergyJ = 0.5 * m * o.normalMs * o.normalMs;

    // stopped across the surface within what it has to crush
    o.crushM = kCrushShare * std::cbrt(m / kBulkDensityKgM3) + kGroundGiveM;
    o.decelMs2 = o.normalMs * o.normalMs / (2.0 * o.crushM);
    o.stopS = 2.0 * o.crushM / o.normalMs;
    // the ground's impulse: up its normal m (1 + e) v_n, against the motion along it mu times that - no more than
    // stops it
    const V3 t = o.tangentMs > 1e-9 ? along * (1.0 / o.tangentMs) : V3{};
    const double drag = std::min(o.tangentMs, kGroundFriction * (1.0 + kRestitution) * o.normalMs);
    o.after = t * (o.tangentMs - drag) + n * (kRestitution * o.normalMs);
    // through the crush an accelerometer reads the ground's push: its share along the surface as its drag's is of the
    // impulse's normal part
    const double share = drag / ((1.0 + kRestitution) * o.normalMs);
    o.specificForce = n * o.decelMs2 - t * (share * o.decelMs2);

    // its fuel
    o.severityMs = std::hypot(o.normalMs, kSlideShare * o.tangentMs);
    o.spiltShare = smooth(kTankSafeMs, kTankBurstMs, o.severityMs);
    const double spilt = std::max(in.fuelKg, 0.0) * o.spiltShare;
    o.fireballKg = spilt * kMistMost * smooth(kTankSafeMs, kMistFullMs, o.severityMs);
    o.poolKg = spilt - o.fireballKg;
    o.poolKgS = o.poolKg / kPoolBurnS;
    return o;
}

} // namespace fsim::ms::damage
