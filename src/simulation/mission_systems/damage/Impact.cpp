#include "simulation/mission_systems/damage/Impact.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fsim::ms::damage {

namespace {
bool finiteVector(V3 a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
// Cholesky solve preserves the full inertia tensor, including products of
// inertia. Invalid/non-positive tensors are unavailable, never replaced by a
// guessed diagonal or a body-size estimate.
bool inertiaSolve(const double* m, V3 rhs, V3& out) {
    double scale = 0;
    for (int i=0;i<9;++i) { if (!std::isfinite(m[i])) return false; scale=std::max(scale,std::abs(m[i])); }
    const double roundoff=64*std::numeric_limits<double>::epsilon()*scale;
    if (std::abs(m[1]-m[3])>roundoff || std::abs(m[2]-m[6])>roundoff || std::abs(m[5]-m[7])>roundoff || !(m[0]>0)) return false;
    const double l00=std::sqrt(m[0]), l10=m[3]/l00, l20=m[6]/l00;
    const double d11=m[4]-l10*l10;
    if (!(d11>0)) return false;
    const double l11=std::sqrt(d11), l21=(m[7]-l20*l10)/l11;
    const double d22=m[8]-l20*l20-l21*l21;
    if (!(d22>0)) return false;
    const double l22=std::sqrt(d22);
    const double y0=rhs.x/l00, y1=(rhs.y-l10*y0)/l11, y2=(rhs.z-l20*y0-l21*y1)/l22;
    out.z=y2/l22; out.y=(y1-l21*out.z)/l11; out.x=(y0-l10*out.y-l20*out.z)/l00;
    return finiteVector(out);
}
}

BodyContactOutcome inelasticBodyContact(const ContactBody& a, const ContactBody& b, V3 normal) noexcept {
    BodyContactOutcome out;
    if (!(a.massKg>0) || !(b.massKg>0) || !std::isfinite(a.massKg) || !std::isfinite(b.massKg) ||
        !finiteVector(a.velocity) || !finiteVector(b.velocity) || !finiteVector(a.angularVelocity) ||
        !finiteVector(b.angularVelocity) || !finiteVector(a.arm) || !finiteVector(b.arm) || !finiteVector(normal)) return out;
    const double norm=std::hypot(normal.x,normal.y,normal.z);
    if (!(norm>0) || !std::isfinite(norm)) return out;
    const V3 n=normal*(1/norm), ca=cross(a.arm,n), cb=cross(b.arm,n);
    V3 ia,ib;
    if (!inertiaSolve(a.inertia,ca,ia) || !inertiaSolve(b.inertia,cb,ib)) return out;
    const double inverseMass=1/a.massKg+1/b.massKg+dot(ca,ia)+dot(cb,ib);
    const V3 relative=a.velocity+cross(a.angularVelocity,a.arm)-b.velocity-cross(b.angularVelocity,b.arm);
    if (!(inverseMass>0) || !std::isfinite(inverseMass) || !finiteVector(relative)) return out;
    out.closingMs=-dot(relative,n);
    if (!std::isfinite(out.closingMs)) return {};
    out.valid=true;
    out.effectiveMassKg=1/inverseMass;
    if (!(out.closingMs>0)) return out;
    const double magnitude=out.closingMs*out.effectiveMassKg;
    out.dissipatedJ=.5*out.closingMs*magnitude;
    if (!std::isfinite(magnitude) || !std::isfinite(out.dissipatedJ)) return {};
    out.approaching=true;
    out.impulseA=n*magnitude; out.impulseB=out.impulseA*(-1);
    out.angularImpulseA=cross(a.arm,out.impulseA); out.angularImpulseB=cross(b.arm,out.impulseB);
    if (!finiteVector(out.angularImpulseA) || !finiteVector(out.angularImpulseB)) return {};
    return out;
}

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
