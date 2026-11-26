#include "simulation/runtime/world/DebrisField.h"

#include "foundation/core/Units.h"
#include "simulation/mission_systems/damage/Debris.h"

#include <algorithm>
#include <cmath>

namespace fsim::session {

namespace {

using ms::damage::V3;
using ms::damage::cross;
using ms::damage::dot;
using ms::damage::length;
using ms::damage::normalized;

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

V3 v3(const double a[3]) noexcept { return {a[0], a[1], a[2]}; }
void store(double dst[3], const V3& v) noexcept { dst[0] = v.x; dst[1] = v.y; dst[2] = v.z; }

// Convert row-major 3x3 body-to-ECEF rotation matrix to unit quaternion [x, y, z, w].
void mat3ToQuat(const double R[9], double q[4]) noexcept {
    const double trace = R[0] + R[4] + R[8];
    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        w = 0.25 * s;
        x = (R[7] - R[5]) / s;
        y = (R[2] - R[6]) / s;
        z = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        const double s = 2.0 * std::sqrt(std::max(1.0 + R[0] - R[4] - R[8], 1e-12));
        w = (R[7] - R[5]) / s;
        x = 0.25 * s;
        y = (R[1] + R[3]) / s;
        z = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        const double s = 2.0 * std::sqrt(std::max(1.0 + R[4] - R[0] - R[8], 1e-12));
        w = (R[2] - R[6]) / s;
        x = (R[1] + R[3]) / s;
        y = 0.25 * s;
        z = (R[5] + R[7]) / s;
    } else {
        const double s = 2.0 * std::sqrt(std::max(1.0 + R[8] - R[0] - R[4], 1e-12));
        w = (R[3] - R[1]) / s;
        x = (R[2] + R[6]) / s;
        y = (R[5] + R[7]) / s;
        z = 0.25 * s;
    }
    const double len = std::sqrt(x * x + y * y + z * z + w * w);
    if (len > 1e-12) {
        const double inv = 1.0 / len;
        q[0] = x * inv; q[1] = y * inv; q[2] = z * inv; q[3] = w * inv;
    } else {
        q[0] = q[1] = q[2] = 0.0; q[3] = 1.0;
    }
}

V3 quatRotate(const double q[4], const V3& v) noexcept {
    const V3 u{q[0], q[1], q[2]};
    const double w = q[3];
    const V3 t = cross(u, v) * 2.0;
    return v + t * w + cross(u, t);
}

V3 quatRotateInverse(const double q[4], const V3& v) noexcept {
    const double qi[4] = {-q[0], -q[1], -q[2], q[3]};
    return quatRotate(qi, v);
}

void integrateQuat(double q[4], const V3& omegaBody, double dt) noexcept {
    // dq/dt = 0.5 * q * (omega_x, omega_y, omega_z, 0)
    const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
    const double ox = omegaBody.x, oy = omegaBody.y, oz = omegaBody.z;
    const double dx = 0.5 * ( qw * ox + qy * oz - qz * oy);
    const double dy = 0.5 * ( qw * oy - qx * oz + qz * ox);
    const double dz = 0.5 * ( qw * oz + qx * oy - qy * ox);
    const double dw = 0.5 * (-qx * ox - qy * oy - qz * oz);
    q[0] += dx * dt;
    q[1] += dy * dt;
    q[2] += dz * dt;
    q[3] += dw * dt;
    const double len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (len > 1e-12) {
        const double inv = 1.0 / len;
        for (int k = 0; k < 4; ++k) q[k] *= inv;
    } else {
        q[0] = q[1] = q[2] = 0.0; q[3] = 1.0;
    }
}

V3 randomUnitSphere(Rng& rng) noexcept {
    const double z = 2.0 * rng.uniform() - 1.0;
    const double phi = 2.0 * kPi * rng.uniform();
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    return {r * std::cos(phi), r * std::sin(phi), z};
}

void randomUnitQuat(Rng& rng, double q[4]) noexcept {
    const double u1 = rng.uniform();
    const double u2 = 2.0 * kPi * rng.uniform();
    const double u3 = 2.0 * kPi * rng.uniform();
    const double a = std::sqrt(std::max(0.0, 1.0 - u1));
    const double b = std::sqrt(std::max(0.0, u1));
    q[0] = a * std::sin(u2);
    q[1] = a * std::cos(u2);
    q[2] = b * std::sin(u3);
    q[3] = b * std::cos(u3);
}

void orthonormalBasis(const V3& n, V3& t1, V3& t2) noexcept {
    const V3 ref = std::abs(n.z) < 0.9 ? V3{0.0, 0.0, 1.0} : V3{1.0, 0.0, 0.0};
    t1 = normalized(cross(ref, n));
    t2 = normalized(cross(n, t1));
}

int archetypePriority(ipc::DebrisArchetypeV1 a) noexcept {
    switch (a) {
    case ipc::DebrisArchetypeV1::AirframeHull: return 5;
    case ipc::DebrisArchetypeV1::WingFragment: return 4;
    case ipc::DebrisArchetypeV1::BurningChunk: return 3;
    case ipc::DebrisArchetypeV1::PanelShard: return 2;
    case ipc::DebrisArchetypeV1::SplashPlume: return 1;
    case ipc::DebrisArchetypeV1::SparkEmber: return 0;
    }
    return 0;
}

} // namespace

DebrisField::DebrisField() { clear(); }

void DebrisField::clear() noexcept {
    for (auto& b : bodies_) b = DebrisBody{};
    activeCount_ = 0;
    nextId_ = 1;
}

void DebrisField::retire(DebrisBody& b) noexcept {
    if (!b.alive) return;
    b.alive = false;
    if (activeCount_ > 0) --activeCount_;
}

DebrisBody& DebrisField::allocate(ipc::DebrisArchetypeV1 archetype) noexcept {
    // Prefer an inactive slot first.
    for (auto& b : bodies_) {
        if (!b.alive) {
            b = DebrisBody{};
            b.id = nextId_++;
            b.archetype = archetype;
            b.alive = true;
            ++activeCount_;
            return b;
        }
    }
    // Pool full: deterministically evict lowest-priority oldest chunk so major wreckage stays intact.
    std::size_t victim = 0;
    int bestPrio = 99;
    double bestAge = -1.0;
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        const int p = archetypePriority(bodies_[i].archetype);
        if (p < bestPrio || (p == bestPrio && bodies_[i].ageS > bestAge)) {
            bestPrio = p;
            bestAge = bodies_[i].ageS;
            victim = i;
        }
    }
    DebrisBody& b = bodies_[victim];
    b = DebrisBody{};
    b.id = nextId_++;
    b.archetype = archetype;
    b.alive = true;
    return b;
}

void DebrisField::spawnSectionBreakup(const SectionBreakupSpawn& in, Rng& rng) {
    const V3 secPos = v3(in.sectionPositionEcef);
    const V3 rootPos = v3(in.rootPositionEcef);
    const V3 secVel = v3(in.sectionVelocityEcef);

    // Reserve ~12% of structural mass for fracture shards when replacing the primary piece,
    // conserving linear momentum strictly across primary chunk + fracture spray.
    constexpr int kShardCount = 14;
    constexpr int kSparkCount = 8;
    const double totalMass = std::max(in.sectionMassKg, 20.0);
    const double shardPoolMass = in.spawnPrimaryChunk ? totalMass * 0.10 : std::min(totalMass * 0.08, 35.0);
    const double primaryMass = std::max(totalMass - shardPoolMass, 1.0);

    // Generate balanced zero-mean shard velocity perturbations so total momentum is strictly conserved.
    std::array<V3, kShardCount> shardDv{};
    V3 sumP{};
    double sumM = 0.0;
    std::array<double, kShardCount> shardM{};
    for (int i = 0; i < kShardCount; ++i) {
        const double m = (shardPoolMass / kShardCount) * (0.5 + rng.uniform());
        shardM[static_cast<std::size_t>(i)] = m;
        const V3 dir = randomUnitSphere(rng);
        const double spd = 6.0 + 26.0 * rng.uniform();
        shardDv[static_cast<std::size_t>(i)] = dir * spd;
        sumP += shardDv[static_cast<std::size_t>(i)] * m;
        sumM += m;
    }
    const V3 meanDv = sumM > 1e-6 ? sumP * (1.0 / sumM) : V3{};

    if (in.spawnPrimaryChunk) {
        auto& body = allocate(in.isWing ? ipc::DebrisArchetypeV1::WingFragment : ipc::DebrisArchetypeV1::AirframeHull);
        body.ownerVehicleId = in.ownerVehicleId;
        store(body.positionEcef, secPos);
        store(body.velocityEcef, secVel);
        mat3ToQuat(in.rotationBodyToEcef, body.quatBodyToEcef);
        // Realistic structural tumble kick from asymmetric root shear.
        body.omegaBodyRadS[0] = in.angularRateBodyRadS[0] + (rng.uniform() - 0.5) * 3.5;
        body.omegaBodyRadS[1] = in.angularRateBodyRadS[1] + (rng.uniform() - 0.5) * 3.0;
        body.omegaBodyRadS[2] = in.angularRateBodyRadS[2] + (rng.uniform() - 0.5) * 3.5;
        body.massKg = primaryMass;
        const double span = std::clamp(in.spanM, 1.2, 12.0);
        if (in.isWing) {
            body.halfExtentsM[0] = 0.45 * span;
            body.halfExtentsM[1] = 0.55 * span;
            body.halfExtentsM[2] = 0.14;
            body.areaM2 = std::max(0.6, 0.8 * span * span);
        } else {
            body.halfExtentsM[0] = 0.65 * span;
            body.halfExtentsM[1] = 0.38 * span;
            body.halfExtentsM[2] = 0.34 * span;
            body.areaM2 = std::max(0.8, 0.55 * span * span);
        }
        body.fuelRemainingKg = std::max(0.0, in.fuelKg);
        body.burning = in.burning || (in.fuelKg > 15.0 && rng.uniform() < 0.65);
        body.burnRateKgS = body.burning ? std::clamp(body.fuelRemainingKg / 25.0, 0.25, 2.5) : 0.0;
        body.temperatureK = body.burning ? 1450.0 : 520.0;
        body.smokeIntensity = body.burning ? 0.95 : 0.35;
        body.maxAgeS = 90.0;
    }

    for (int i = 0; i < kShardCount; ++i) {
        const bool hotChunk = in.burning && (i < 4);
        auto& s = allocate(hotChunk ? ipc::DebrisArchetypeV1::BurningChunk : ipc::DebrisArchetypeV1::PanelShard);
        s.ownerVehicleId = in.ownerVehicleId;
        const V3 jitter = randomUnitSphere(rng) * (0.4 + 1.2 * rng.uniform());
        store(s.positionEcef, rootPos + jitter);
        store(s.velocityEcef, secVel + (shardDv[static_cast<std::size_t>(i)] - meanDv));
        randomUnitQuat(rng, s.quatBodyToEcef);
        s.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 14.0;
        s.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 14.0;
        s.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 14.0;
        s.massKg = shardM[static_cast<std::size_t>(i)];
        const double l = 0.30 + 0.65 * rng.uniform();
        s.halfExtentsM[0] = l;
        s.halfExtentsM[1] = l * (0.4 + 0.45 * rng.uniform());
        s.halfExtentsM[2] = 0.04 + 0.06 * rng.uniform();
        s.areaM2 = 2.2 * s.halfExtentsM[0] * s.halfExtentsM[1];
        s.burning = hotChunk;
        s.fuelRemainingKg = hotChunk ? 4.0 + 8.0 * rng.uniform() : 0.0;
        s.burnRateKgS = hotChunk ? 0.35 : 0.0;
        s.temperatureK = hotChunk ? 1650.0 : 650.0 + 350.0 * rng.uniform();
        s.smokeIntensity = hotChunk ? 0.85 : 0.20;
        s.maxAgeS = 45.0;
    }

    for (int i = 0; i < kSparkCount; ++i) {
        auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
        sp.ownerVehicleId = in.ownerVehicleId;
        store(sp.positionEcef, rootPos + randomUnitSphere(rng) * 0.6);
        store(sp.velocityEcef, secVel + randomUnitSphere(rng) * (18.0 + 45.0 * rng.uniform()));
        randomUnitQuat(rng, sp.quatBodyToEcef);
        sp.massKg = 0.05;
        sp.areaM2 = 0.008;
        sp.halfExtentsM[0] = 0.25; sp.halfExtentsM[1] = 0.06; sp.halfExtentsM[2] = 0.06;
        sp.burning = true;
        sp.temperatureK = 2200.0;
        sp.smokeIntensity = 0.15;
        sp.maxAgeS = 2.0 + 2.5 * rng.uniform();
    }
}

void DebrisField::spawnCatastrophicKill(const CatastrophicSpawn& in, Rng& rng) {
    const V3 origin = v3(in.positionEcef);
    const V3 parentVel = v3(in.velocityEcef);
    const double totalMass = std::max(in.airframeMassKg, 500.0);
    // Physical radial blast velocity scale from Gurney-style energy partition (bounded for fuselage chunks).
    const double blastScale = std::clamp(std::sqrt(std::max(in.explosionTntKg, 2.0) * 120.0), 18.0, 95.0);
    const double radius = std::clamp(in.radiusM, 3.0, 18.0);

    constexpr int kHullChunks = 4;
    constexpr int kWingChunks = 4;
    constexpr int kBurningChunks = 14;
    constexpr int kPanelShards = 18;
    constexpr int kSparks = 20;
    constexpr int kStructuralTotal = kHullChunks + kWingChunks + kBurningChunks + kPanelShards;

    struct Spec {
        ipc::DebrisArchetypeV1 arch;
        double massWeight;
        double speedFactor;
        bool burning;
    };
    std::array<Spec, kStructuralTotal> specs{};
    int idx = 0;
    for (int i = 0; i < kHullChunks; ++i) specs[static_cast<std::size_t>(idx++)] = {ipc::DebrisArchetypeV1::AirframeHull, 14.0, 0.35, true};
    for (int i = 0; i < kWingChunks; ++i) specs[static_cast<std::size_t>(idx++)] = {ipc::DebrisArchetypeV1::WingFragment, 8.0, 0.55, i < 2};
    for (int i = 0; i < kBurningChunks; ++i) specs[static_cast<std::size_t>(idx++)] = {ipc::DebrisArchetypeV1::BurningChunk, 2.2, 0.85, true};
    for (int i = 0; i < kPanelShards; ++i) specs[static_cast<std::size_t>(idx++)] = {ipc::DebrisArchetypeV1::PanelShard, 0.7, 1.25, false};

    double sumWeights = 0.0;
    std::array<double, kStructuralTotal> weights{};
    std::array<V3, kStructuralTotal> dvs{};
    V3 sumP{};
    for (int i = 0; i < kStructuralTotal; ++i) {
        const double w = specs[static_cast<std::size_t>(i)].massWeight * (0.65 + 0.7 * rng.uniform());
        weights[static_cast<std::size_t>(i)] = w;
        sumWeights += w;
    }
    for (int i = 0; i < kStructuralTotal; ++i) {
        const double m = totalMass * (weights[static_cast<std::size_t>(i)] / sumWeights);
        const V3 dir = randomUnitSphere(rng);
        const double spd = blastScale * specs[static_cast<std::size_t>(i)].speedFactor * (0.55 + 0.9 * rng.uniform());
        dvs[static_cast<std::size_t>(i)] = dir * spd;
        sumP += dvs[static_cast<std::size_t>(i)] * m;
    }
    // Subtract weighted mean velocity delta so exact linear momentum sum(m_i * v_i) == M_total * v_parent!
    const V3 dvCorrection = sumP * (1.0 / totalMass);

    const int burningCount = kHullChunks + 2 + kBurningChunks;
    const double fuelPerBurner = std::max(0.0, in.remainingFuelKg) * 0.55 / static_cast<double>(std::max(burningCount, 1));

    for (int i = 0; i < kStructuralTotal; ++i) {
        const auto& sp = specs[static_cast<std::size_t>(i)];
        const double m = totalMass * (weights[static_cast<std::size_t>(i)] / sumWeights);
        auto& b = allocate(sp.arch);
        b.ownerVehicleId = in.ownerVehicleId;
        const V3 offset = randomUnitSphere(rng) * (radius * (0.15 + 0.55 * rng.uniform()));
        store(b.positionEcef, origin + offset);
        store(b.velocityEcef, parentVel + dvs[static_cast<std::size_t>(i)] - dvCorrection);
        randomUnitQuat(rng, b.quatBodyToEcef);
        const double spin = sp.arch == ipc::DebrisArchetypeV1::AirframeHull ? 4.0 : 12.0;
        b.omegaBodyRadS[0] = (rng.uniform() - 0.5) * spin;
        b.omegaBodyRadS[1] = (rng.uniform() - 0.5) * spin;
        b.omegaBodyRadS[2] = (rng.uniform() - 0.5) * spin;
        b.massKg = m;
        if (sp.arch == ipc::DebrisArchetypeV1::AirframeHull) {
            b.halfExtentsM[0] = 0.95 + 0.65 * rng.uniform();
            b.halfExtentsM[1] = 0.55 + 0.35 * rng.uniform();
            b.halfExtentsM[2] = 0.35 + 0.25 * rng.uniform();
            b.areaM2 = 3.2 * b.halfExtentsM[0] * b.halfExtentsM[1];
            b.maxAgeS = 110.0;
        } else if (sp.arch == ipc::DebrisArchetypeV1::WingFragment) {
            b.halfExtentsM[0] = 0.85 + 0.70 * rng.uniform();
            b.halfExtentsM[1] = 0.65 + 0.65 * rng.uniform();
            b.halfExtentsM[2] = 0.09;
            b.areaM2 = 2.8 * b.halfExtentsM[0] * b.halfExtentsM[1];
            b.maxAgeS = 95.0;
        } else if (sp.arch == ipc::DebrisArchetypeV1::BurningChunk) {
            b.halfExtentsM[0] = 0.28 + 0.32 * rng.uniform();
            b.halfExtentsM[1] = 0.20 + 0.24 * rng.uniform();
            b.halfExtentsM[2] = 0.14 + 0.16 * rng.uniform();
            b.areaM2 = 2.4 * b.halfExtentsM[0] * b.halfExtentsM[1];
            b.maxAgeS = 75.0;
        } else {
            b.halfExtentsM[0] = 0.25 + 0.45 * rng.uniform();
            b.halfExtentsM[1] = 0.15 + 0.28 * rng.uniform();
            b.halfExtentsM[2] = 0.035;
            b.areaM2 = 2.2 * b.halfExtentsM[0] * b.halfExtentsM[1];
            b.maxAgeS = 60.0;
        }
        b.burning = sp.burning;
        b.fuelRemainingKg = sp.burning ? std::max(3.0, fuelPerBurner * (0.6 + 0.8 * rng.uniform())) : 0.0;
        b.burnRateKgS = sp.burning ? std::clamp(b.fuelRemainingKg / 22.0, 0.25, 3.0) : 0.0;
        b.temperatureK = sp.burning ? 1650.0 + 450.0 * rng.uniform() : 750.0 + 400.0 * rng.uniform();
        b.smokeIntensity = sp.burning ? 1.0 : 0.30;
    }

    // Incandescent high-velocity spark/ember cloud (mass-negligible visual tracer sparks).
    for (int i = 0; i < kSparks; ++i) {
        auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
        sp.ownerVehicleId = in.ownerVehicleId;
        store(sp.positionEcef, origin + randomUnitSphere(rng) * (0.3 * radius));
        store(sp.velocityEcef, parentVel + randomUnitSphere(rng) * (blastScale * (1.2 + 1.4 * rng.uniform())));
        randomUnitQuat(rng, sp.quatBodyToEcef);
        sp.massKg = 0.08;
        sp.areaM2 = 0.012;
        sp.halfExtentsM[0] = 0.40; sp.halfExtentsM[1] = 0.08; sp.halfExtentsM[2] = 0.08;
        sp.burning = true;
        sp.temperatureK = 2350.0;
        sp.smokeIntensity = 0.20;
        sp.maxAgeS = 3.0 + 3.5 * rng.uniform();
    }
}

void DebrisField::spawnImpactCloud(const ImpactCloudSpawn& in, Rng& rng) {
    const V3 pos = v3(in.positionEcef);
    const V3 normal = normalized(v3(in.normalEcef));
    const V3 after = v3(in.afterVelocityEcef);
    const double afterSpeed = length(after);
    const double inSpeed = length(v3(in.incomingVelocityEcef));
    V3 t1, t2;
    orthonormalBasis(normal, t1, t2);
    const bool water = (in.surface == ms::damage::SurfaceKind::Water);

    const int chunkCount = in.hullDisintegrated ? 34 : 20;
    const int sparkOrSplashCount = water ? 22 : 16;

    for (int i = 0; i < chunkCount; ++i) {
        ipc::DebrisArchetypeV1 arch = ipc::DebrisArchetypeV1::PanelShard;
        if (i < 3) arch = ipc::DebrisArchetypeV1::AirframeHull;
        else if (i < 7) arch = ipc::DebrisArchetypeV1::WingFragment;
        else if (!water && i < 15) arch = ipc::DebrisArchetypeV1::BurningChunk;

        auto& b = allocate(arch);
        b.ownerVehicleId = in.ownerVehicleId;
        const double ang = 2.0 * kPi * rng.uniform();
        const double rad = 1.0 + 4.5 * rng.uniform();
        const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
        store(b.positionEcef, pos + radial * rad + normal * (1.2 + 2.5 * rng.uniform()));

        // Eject upward & forward along post-impact track (steep dive fans radially; shallow strike sprays forward!).
        const double upSpd = (12.0 + 0.22 * inSpeed) * (0.4 + 0.9 * rng.uniform());
        const double outSpd = (8.0 + 0.16 * inSpeed) * (0.4 + 0.8 * rng.uniform());
        const V3 vel = after * (0.65 + 0.55 * rng.uniform()) + normal * upSpd + radial * outSpd;
        store(b.velocityEcef, vel);
        randomUnitQuat(rng, b.quatBodyToEcef);
        b.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 10.0;
        b.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 10.0;
        b.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 10.0;
        b.massKg = arch == ipc::DebrisArchetypeV1::AirframeHull ? 280.0 : (arch == ipc::DebrisArchetypeV1::WingFragment ? 140.0 : 18.0);
        if (arch == ipc::DebrisArchetypeV1::AirframeHull) {
            b.halfExtentsM[0] = 0.85 + 0.55 * rng.uniform();
            b.halfExtentsM[1] = 0.50 + 0.30 * rng.uniform();
            b.halfExtentsM[2] = 0.32 + 0.22 * rng.uniform();
        } else if (arch == ipc::DebrisArchetypeV1::WingFragment) {
            b.halfExtentsM[0] = 0.80 + 0.60 * rng.uniform();
            b.halfExtentsM[1] = 0.60 + 0.55 * rng.uniform();
            b.halfExtentsM[2] = 0.09;
        } else {
            b.halfExtentsM[0] = 0.26 + 0.34 * rng.uniform();
            b.halfExtentsM[1] = 0.18 + 0.24 * rng.uniform();
            b.halfExtentsM[2] = 0.05 + 0.09 * rng.uniform();
        }
        b.areaM2 = std::max(0.25, 2.4 * b.halfExtentsM[0] * b.halfExtentsM[1]);
        b.burning = (!water || in.fireballKg > 50.0) && (arch == ipc::DebrisArchetypeV1::BurningChunk || i < 5);
        b.fuelRemainingKg = b.burning ? std::max(2.0, std::min(in.poolKg * 0.06, 25.0)) : 0.0;
        b.burnRateKgS = b.burning ? 0.35 : 0.0;
        b.temperatureK = b.burning ? 1600.0 : (water ? 340.0 : 650.0);
        b.smokeIntensity = b.burning ? 0.90 : 0.25;
        b.inWater = water;
        b.maxAgeS = water ? 22.0 : 65.0;
    }

    for (int i = 0; i < sparkOrSplashCount; ++i) {
        auto& p = allocate(water ? ipc::DebrisArchetypeV1::SplashPlume : ipc::DebrisArchetypeV1::SparkEmber);
        p.ownerVehicleId = in.ownerVehicleId;
        const double ang = 2.0 * kPi * rng.uniform();
        const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
        store(p.positionEcef, pos + radial * (2.0 + 6.0 * rng.uniform()) + normal * (1.0 + 2.0 * rng.uniform()));
        if (water) {
            const double plumeUp = (18.0 + 0.28 * inSpeed) * (0.45 + 0.85 * rng.uniform());
            const double plumeOut = (10.0 + 0.15 * afterSpeed) * (0.4 + 0.9 * rng.uniform());
            store(p.velocityEcef, after * 0.35 + normal * plumeUp + radial * plumeOut);
            randomUnitQuat(rng, p.quatBodyToEcef);
            p.massKg = 45.0;
            p.areaM2 = 4.0;
            const double r = 1.6 + 2.8 * rng.uniform();
            p.halfExtentsM[0] = r; p.halfExtentsM[1] = r; p.halfExtentsM[2] = r * 1.3;
            p.burning = false;
            p.inWater = true;
            p.temperatureK = 290.0;
            p.smokeIntensity = 0.85;
            p.maxAgeS = 5.5 + 3.5 * rng.uniform();
        } else {
            const double upSpd = (14.0 + 0.25 * inSpeed) * (0.5 + rng.uniform());
            const double outSpd = (18.0 + 0.30 * inSpeed) * (0.5 + rng.uniform());
            store(p.velocityEcef, after * 0.85 + normal * upSpd + radial * outSpd);
            randomUnitQuat(rng, p.quatBodyToEcef);
            p.massKg = 0.08;
            p.areaM2 = 0.01;
            p.halfExtentsM[0] = 0.38; p.halfExtentsM[1] = 0.07; p.halfExtentsM[2] = 0.07;
            p.burning = true;
            p.temperatureK = 2300.0;
            p.smokeIntensity = 0.20;
            p.maxAgeS = 2.5 + 2.5 * rng.uniform();
        }
    }
}

void DebrisField::spawnSlideSparks(const SlideSparkSpawn& in, Rng& rng) {
    const V3 pos = v3(in.positionEcef);
    const V3 vel = v3(in.velocityEcef);
    const V3 normal = normalized(v3(in.normalEcef));
    const double speed = length(vel);
    if (!(speed > 6.0)) return;
    V3 t1, t2;
    orthonormalBasis(normal, t1, t2);
    const bool water = (in.surface == ms::damage::SurfaceKind::Water);
    const int count = 4;
    for (int i = 0; i < count; ++i) {
        auto& p = allocate(water ? ipc::DebrisArchetypeV1::SplashPlume : ipc::DebrisArchetypeV1::SparkEmber);
        p.ownerVehicleId = in.ownerVehicleId;
        const double ang = 2.0 * kPi * rng.uniform();
        const V3 side = t1 * std::cos(ang) + t2 * std::sin(ang);
        store(p.positionEcef, pos + side * (1.8 * rng.uniform()) + normal * (water ? 1.2 : 0.4));
        const V3 spray = vel * (water ? (0.20 + 0.30 * rng.uniform()) : (0.45 + 0.35 * rng.uniform())) +
                         normal * (water ? (7.0 + 0.22 * speed * rng.uniform()) : (3.0 + 0.10 * speed * rng.uniform())) +
                         side * (3.0 + 0.14 * speed * rng.uniform());
        store(p.velocityEcef, spray);
        randomUnitQuat(rng, p.quatBodyToEcef);
        p.massKg = water ? 18.0 : 0.04;
        p.areaM2 = water ? 2.2 : 0.008;
        if (water) {
            const double r = 1.8 + 2.2 * rng.uniform();
            p.halfExtentsM[0] = r; p.halfExtentsM[1] = r; p.halfExtentsM[2] = r;
            p.temperatureK = 290.0;
            p.smokeIntensity = 0.78;
            p.inWater = true;
            p.maxAgeS = 2.6 + 1.4 * rng.uniform();
        } else {
            p.halfExtentsM[0] = 0.28; p.halfExtentsM[1] = 0.06; p.halfExtentsM[2] = 0.06;
            p.burning = true;
            p.temperatureK = 2150.0;
            p.smokeIntensity = 0.15;
            p.maxAgeS = 0.8 + 1.0 * rng.uniform();
        }
    }
}

void DebrisField::step(double dt, const sim::GroundProvider* ground, const sim::EnvironmentState& env, double simTime) {
    (void)simTime;
    if (activeCount_ == 0 || !(dt > 0.0)) return;
    const double windDirRad = units::degreesToRadians(std::isfinite(env.windDirectionDeg) ? env.windDirectionDeg : 0.0);
    const double windSpd = std::isfinite(env.windSpeedMs) ? std::max(env.windSpeedMs, 0.0) : 0.0;
    const double windNorth = -windSpd * std::cos(windDirRad);
    const double windEast = -windSpd * std::sin(windDirRad);

    for (auto& b : bodies_) {
        if (!b.alive) continue;
        b.ageS += dt;
        if (b.ageS >= b.maxAgeS) {
            retire(b);
            continue;
        }

        V3 pos = v3(b.positionEcef);
        V3 vel = v3(b.velocityEcef);
        V3 omega = v3(b.omegaBodyRadS);

        double lat = 0.0, lon = 0.0, h = 0.0;
        ms::damage::geodetic(pos, lat, lon, h);
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        const V3 up{cl * co, cl * so, sl};
        const V3 north{-sl * co, -sl * so, cl};
        const V3 east{-so, co, 0.0};
        const V3 windEcef = north * windNorth + east * windEast;

        if (!b.grounded) {
            // Aerodynamic drag with tumbling flat-plate incidence modulation + gravity.
            const double rho = ms::damage::standardDensity(h);
            const V3 vRel = vel - windEcef;
            const double speed = length(vRel);
            V3 acc = up * (-kG);
            if (speed > 1e-4 && b.massKg > 1e-4) {
                const V3 vHat = vRel * (1.0 / speed);
                const V3 plateNormal = quatRotate(b.quatBodyToEcef, {0.0, 0.0, 1.0});
                const double cosInc = std::abs(dot(vHat, plateNormal));
                // Bluff-body / tumbling plate Cd in [0.35, 1.30].
                const double cd = b.archetype == ipc::DebrisArchetypeV1::SplashPlume ? 0.95 : (0.35 + 0.93 * cosInc);
                const double qDyn = 0.5 * rho * speed * speed;
                const double dragAcc = std::min(qDyn * cd * b.areaM2 / b.massKg, speed / dt * 0.92);
                acc += vHat * (-dragAcc);

                // Aerodynamic tumbling torque & spin damping.
                if (b.archetype != ipc::DebrisArchetypeV1::SparkEmber && b.archetype != ipc::DebrisArchetypeV1::SplashPlume) {
                    const V3 torqueEcef = cross(plateNormal, vHat) * (0.14 * qDyn * b.areaM2);
                    const V3 torqueBody = quatRotateInverse(b.quatBodyToEcef, torqueEcef);
                    const double inertiaProxy = std::max(0.05, b.massKg * (b.halfExtentsM[0] * b.halfExtentsM[0] + b.halfExtentsM[1] * b.halfExtentsM[1]) * 0.25);
                    omega += torqueBody * (dt / inertiaProxy);
                    omega = omega * std::exp(-0.22 * dt);
                }
            }

            vel += acc * dt;
            pos += vel * dt;
            integrateQuat(b.quatBodyToEcef, omega, dt);

            // Re-evaluate altitude for ground/water contact.
            ms::damage::geodetic(pos, lat, lon, h);
            const double newGroundH = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
            const double clearance = std::min(b.halfExtentsM[2], 0.35);
            if (h <= newGroundH + clearance) {
                const double lift = (newGroundH + clearance) - h;
                pos += up * lift;
                const double vNorm = dot(vel, up);
                const V3 vTang = vel - up * vNorm;
                const double tSpd = length(vTang);
                if (b.archetype == ipc::DebrisArchetypeV1::SplashPlume || b.archetype == ipc::DebrisArchetypeV1::SparkEmber) {
                    retire(b);
                    continue;
                }
                const double restitution = b.inWater ? 0.02 : 0.14;
                const double friction = b.inWater ? 0.55 : 0.65;
                if (-vNorm > 3.5 && !b.inWater) {
                    const double dragT = std::min(tSpd, friction * (1.0 + restitution) * (-vNorm));
                    const V3 tDir = tSpd > 1e-6 ? vTang * (1.0 / tSpd) : V3{};
                    vel = tDir * (tSpd - dragT) + up * (restitution * (-vNorm));
                    omega = omega * 0.55;
                } else {
                    vel = vTang * (b.inWater ? 0.25 : 0.65);
                    omega = omega * 0.25;
                    b.grounded = true;
                }
            }
        } else {
            // Resting/sliding on terrain or water surface.
            const double tSpd = length(vel);
            if (tSpd > 0.1) {
                const double mu = b.inWater ? 1.8 : 0.75;
                const double nextSpd = std::max(0.0, tSpd - mu * kG * dt);
                vel = vel * (nextSpd / tSpd);
                pos += vel * dt;
                ms::damage::geodetic(pos, lat, lon, h);
                const double newGroundH = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
                const double clearance = std::min(b.halfExtentsM[2], 0.35);
                pos += up * ((newGroundH + clearance) - h);
            } else {
                vel = {};
            }
            omega = omega * std::exp(-4.0 * dt);
            if (b.inWater && b.burning) {
                // Water quickly quenches burning chunks on immersion.
                b.fuelRemainingKg = std::max(0.0, b.fuelRemainingKg - 2.5 * dt);
            }
        }

        // Fuel burn-down and Stefan-Boltzmann + convective cooling.
        if (b.burning && b.fuelRemainingKg > 0.0 && b.burnRateKgS > 0.0) {
            b.fuelRemainingKg = std::max(0.0, b.fuelRemainingKg - b.burnRateKgS * dt);
            if (b.fuelRemainingKg <= 0.0) {
                b.burning = false;
                b.burnRateKgS = 0.0;
            }
        }
        const double ambientK = std::isfinite(env.temperatureSeaLevelK) ? std::max(216.0, env.temperatureSeaLevelK - 0.0065 * std::max(h, 0.0)) : 288.15;
        if (b.burning) {
            const double targetK = b.archetype == ipc::DebrisArchetypeV1::SparkEmber ? 1900.0 : 1550.0;
            b.temperatureK += (targetK - b.temperatureK) * std::min(1.0, 0.4 * dt);
        } else {
            const double coolRate = b.archetype == ipc::DebrisArchetypeV1::SparkEmber ? 1.8 : (b.inWater && b.grounded ? 2.5 : 0.18);
            b.temperatureK = ambientK + (b.temperatureK - ambientK) * std::exp(-coolRate * dt);
            b.smokeIntensity *= std::exp(-0.12 * dt);
        }

        store(b.positionEcef, pos);
        store(b.velocityEcef, vel);
        store(b.omegaBodyRadS, omega);
    }
}

void DebrisField::fillWire(std::vector<ipc::DebrisBodyWireV1>& out) const {
    out.clear();
    if (activeCount_ == 0) return;
    out.reserve(activeCount_);
    for (const auto& b : bodies_) {
        if (!b.alive) continue;
        ipc::DebrisBodyWireV1 w;
        for (int k = 0; k < 3; ++k) {
            w.positionEcef[k] = b.positionEcef[k];
            w.velocityEcef[k] = b.velocityEcef[k];
            w.halfExtentsM[k] = static_cast<float>(b.halfExtentsM[k]);
        }
        w.id = b.id;
        w.ownerVehicleId = b.ownerVehicleId;
        w.temperatureK = static_cast<float>(b.temperatureK);
        w.smokeIntensity = static_cast<float>(std::clamp(b.smokeIntensity, 0.0, 1.0));
        w.ageS = static_cast<float>(b.ageS);
        w.maxAgeS = static_cast<float>(b.maxAgeS);
        for (int k = 0; k < 4; ++k) w.quatBodyToEcef[k] = static_cast<float>(b.quatBodyToEcef[k]);
        w.archetype = static_cast<std::uint8_t>(b.archetype);
        std::uint8_t flags = 0;
        if (b.burning) flags |= ipc::kDebrisFlagBurning;
        if (b.grounded) flags |= ipc::kDebrisFlagGrounded;
        if (b.inWater) flags |= ipc::kDebrisFlagWater;
        if (b.smokeIntensity > 0.05) flags |= ipc::kDebrisFlagSmoking;
        w.flags = flags;
        out.push_back(w);
    }
}

double DebrisField::totalMassKg(std::uint32_t ownerVehicleId) const noexcept {
    double sum = 0.0;
    for (const auto& b : bodies_) {
        if (!b.alive) continue;
        if (ownerVehicleId && b.ownerVehicleId != ownerVehicleId) continue;
        if (b.archetype == ipc::DebrisArchetypeV1::SparkEmber || b.archetype == ipc::DebrisArchetypeV1::SplashPlume) continue;
        sum += b.massKg;
    }
    return sum;
}

ms::damage::V3 DebrisField::totalMomentumEcef(std::uint32_t ownerVehicleId) const noexcept {
    V3 sum{};
    for (const auto& b : bodies_) {
        if (!b.alive) continue;
        if (ownerVehicleId && b.ownerVehicleId != ownerVehicleId) continue;
        if (b.archetype == ipc::DebrisArchetypeV1::SparkEmber || b.archetype == ipc::DebrisArchetypeV1::SplashPlume) continue;
        sum += v3(b.velocityEcef) * b.massKg;
    }
    return sum;
}

} // namespace fsim::session
