#include "simulation/runtime/world/DebrisField.h"

#include "foundation/core/Units.h"
#include "simulation/mission_systems/damage/Debris.h"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <cmath>
#include <mutex>
#include <unordered_map>

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

// Convert row-major 3x3 rotation matrix to unit quaternion [x, y, z, w].
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

void quatToMat3(const double q[4], double R[9]) noexcept {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;
    R[0] = 1.0 - 2.0 * (yy + zz); R[1] = 2.0 * (xy - wz);       R[2] = 2.0 * (xz + wy);
    R[3] = 2.0 * (xy + wz);       R[4] = 1.0 - 2.0 * (xx + zz); R[5] = 2.0 * (yz - wx);
    R[6] = 2.0 * (xz - wy);       R[7] = 2.0 * (yz + wx);       R[8] = 1.0 - 2.0 * (xx + yy);
}

V3 mulMat3(const double R[9], const V3& v) noexcept {
    return {R[0] * v.x + R[1] * v.y + R[2] * v.z,
            R[3] * v.x + R[4] * v.y + R[5] * v.z,
            R[6] * v.x + R[7] * v.y + R[8] * v.z};
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

void quatMul(const double a[4], const double b[4], double out[4]) noexcept {
    const double ax = a[0], ay = a[1], az = a[2], aw = a[3];
    const double bx = b[0], by = b[1], bz = b[2], bw = b[3];
    out[0] = aw * bx + ax * bw + ay * bz - az * by;
    out[1] = aw * by - ax * bz + ay * bw + az * bx;
    out[2] = aw * bz + ax * by - ay * bx + az * bw;
    out[3] = aw * bw - ax * bx - ay * by - az * bz;
    const double len = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    if (len > 1e-12) {
        const double inv = 1.0 / len;
        for (int k = 0; k < 4; ++k) out[k] *= inv;
    } else {
        out[0] = out[1] = out[2] = 0.0; out[3] = 1.0;
    }
}

void integrateQuat(double q[4], const V3& omegaBody, double dt) noexcept {
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

bool isSolidRigidBody(ipc::DebrisArchetypeV1 a) noexcept {
    return a == ipc::DebrisArchetypeV1::AirframeHull || a == ipc::DebrisArchetypeV1::WingFragment ||
           a == ipc::DebrisArchetypeV1::PanelShard || a == ipc::DebrisArchetypeV1::BurningChunk;
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

// --- Jolt Physics 5.3 deterministic layers & global one-time type registration ---
namespace jolt_layers {
constexpr JPH::ObjectLayer kTerrain = 0;
constexpr JPH::ObjectLayer kDebris = 1;
constexpr JPH::ObjectLayer kNumLayers = 2;

namespace bp {
constexpr JPH::BroadPhaseLayer kTerrain(0);
constexpr JPH::BroadPhaseLayer kDebris(1);
constexpr JPH::uint kNumLayers = 2;
} // namespace bp

class BroadPhaseLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return bp::kNumLayers; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        return inLayer == kTerrain ? bp::kTerrain : bp::kDebris;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        return inLayer == bp::kTerrain ? "TERRAIN" : "DEBRIS";
    }
#endif
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        if (inLayer1 == kTerrain) return inLayer2 == bp::kDebris;
        return inLayer2 == bp::kTerrain;
    }
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    // Collide every dynamic wreckage rigid body & fuselage against terrain HeightField,
    // without spawning artificial overlap impulses between freshly-shattered adjacent airframe sections.
    bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        return inObject1 != inObject2;
    }
};

void ensureJoltInitialized() {
    static std::once_flag once;
    std::call_once(once, []() {
        JPH::RegisterDefaultAllocator();
        if (!JPH::Factory::sInstance) JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    });
}

} // namespace jolt_layers

} // namespace

// Local EUN (+X East, +Y Up, +Z South -> right-handed, native Jolt Y-up!) tangent physics world.
struct DebrisField::JoltWorld {
    // Must construct BEFORE tempAlloc / jobSystem / physics so JPH::RegisterDefaultAllocator()
    // and JPH::Factory / JPH::RegisterTypes() are active before any Jolt allocation occurs.
    struct InitGuard {
        InitGuard() { jolt_layers::ensureJoltInitialized(); }
    } initGuard;
    JPH::TempAllocatorImpl tempAlloc{4 * 1024 * 1024};
    JPH::JobSystemSingleThreaded jobSystem{JPH::cMaxPhysicsJobs};
    jolt_layers::BroadPhaseLayerInterfaceImpl bpLayerInterface;
    jolt_layers::ObjectVsBroadPhaseLayerFilterImpl objVsBpFilter;
    jolt_layers::ObjectLayerPairFilterImpl objPairFilter;
    JPH::PhysicsSystem physics;

    bool initialized = false;
    bool hasTerrain = false;
    JPH::BodyID terrainBodyId;
    double terrainCenterX = 0.0;
    double terrainCenterZ = 0.0;

    V3 originEcef{};
    V3 east{1, 0, 0}, up{0, 1, 0}, south{0, 0, 1};
    double originGroundH = 0.0;
    double eunToEcefQuat[4] = {0, 0, 0, 1};
    double ecefToEunQuat[4] = {0, 0, 0, 1};

    struct WreckMeta {
        JPH::BodyID bodyId;
        ms::damage::SurfaceKind surface = ms::damage::SurfaceKind::Soil;
        double halfExtentsM[3] = {5.5, 1.35, 1.35};
        bool alive = false;
    };
    std::unordered_map<std::uint32_t, WreckMeta> wrecks;
    std::uint32_t nextWreckHandle = 1;

    JoltWorld() {
        jolt_layers::ensureJoltInitialized();
        physics.Init(1024, 0, 2048, 2048, bpLayerInterface, objVsBpFilter, objPairFilter);
        physics.SetGravity(JPH::Vec3(0.0f, static_cast<float>(-kG), 0.0f));
        JPH::PhysicsSettings ps = physics.GetPhysicsSettings();
        ps.mNumVelocitySteps = 8;
        ps.mNumPositionSteps = 3;
        ps.mPenetrationSlop = 0.015f;
        physics.SetPhysicsSettings(ps);
    }

    ~JoltWorld() {
        auto& bi = physics.GetBodyInterfaceNoLock();
        for (const auto& [h, w] : wrecks) {
            (void)h;
            if (w.alive && !w.bodyId.IsInvalid()) {
                bi.RemoveBody(w.bodyId);
                bi.DestroyBody(w.bodyId);
            }
        }
        if (hasTerrain && !terrainBodyId.IsInvalid()) {
            bi.RemoveBody(terrainBodyId);
            bi.DestroyBody(terrainBodyId);
        }
    }

    void setupFrame(const double refEcef[3], const sim::GroundProvider* ground) {
        double lat = 0.0, lon = 0.0, h = 0.0;
        ms::damage::geodetic(v3(refEcef), lat, lon, h);
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        east = {-so, co, 0.0};
        up = {cl * co, cl * so, sl};
        const V3 north{-sl * co, -sl * so, cl};
        south = north * (-1.0);
        originGroundH = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
        // Place local origin on the ground surface directly underneath the event so terrain Y ~ 0.
        originEcef = v3(refEcef) + up * (originGroundH - h);

        // Column basis [east, up, south] transforms EUN -> ECEF.
        const double M[9] = {east.x, up.x, south.x, east.y, up.y, south.y, east.z, up.z, south.z};
        mat3ToQuat(M, eunToEcefQuat);
        ecefToEunQuat[0] = -eunToEcefQuat[0];
        ecefToEunQuat[1] = -eunToEcefQuat[1];
        ecefToEunQuat[2] = -eunToEcefQuat[2];
        ecefToEunQuat[3] = eunToEcefQuat[3];
        initialized = true;
        rebuildTerrain(0.0, 0.0, ground);
    }

    JPH::Vec3 posToEun(const V3& ecef) const noexcept {
        const V3 d = ecef - originEcef;
        return JPH::Vec3(static_cast<float>(dot(d, east)), static_cast<float>(dot(d, up)), static_cast<float>(dot(d, south)));
    }

    V3 posToEcef(const JPH::Vec3& p) const noexcept {
        return originEcef + east * static_cast<double>(p.GetX()) + up * static_cast<double>(p.GetY()) + south * static_cast<double>(p.GetZ());
    }

    JPH::Vec3 vecToEun(const V3& v) const noexcept {
        return JPH::Vec3(static_cast<float>(dot(v, east)), static_cast<float>(dot(v, up)), static_cast<float>(dot(v, south)));
    }

    V3 vecToEcef(const JPH::Vec3& v) const noexcept {
        return east * static_cast<double>(v.GetX()) + up * static_cast<double>(v.GetY()) + south * static_cast<double>(v.GetZ());
    }

    JPH::Quat quatBodyToEun(const double qBodyToEcef[4]) const noexcept {
        double qEun[4];
        quatMul(ecefToEunQuat, qBodyToEcef, qEun);
        return JPH::Quat(static_cast<float>(qEun[0]), static_cast<float>(qEun[1]), static_cast<float>(qEun[2]), static_cast<float>(qEun[3])).Normalized();
    }

    void quatEunToBodyEcef(const JPH::Quat& qBodyToEun, double qBodyToEcef[4]) const noexcept {
        const double qEun[4] = {qBodyToEun.GetX(), qBodyToEun.GetY(), qBodyToEun.GetZ(), qBodyToEun.GetW()};
        quatMul(eunToEcefQuat, qEun, qBodyToEcef);
    }

    void rebuildTerrain(double centerX, double centerZ, const sim::GroundProvider* ground) {
        auto& bi = physics.GetBodyInterfaceNoLock();
        if (hasTerrain && !terrainBodyId.IsInvalid()) {
            bi.RemoveBody(terrainBodyId);
            bi.DestroyBody(terrainBodyId);
            hasTerrain = false;
        }
        // 64x64 Jolt HeightFieldShape covering 2.52 km x 2.52 km at 40 m spacing around active wreckage.
        constexpr JPH::uint32 kN = 64;
        constexpr float kStepM = 40.0f;
        constexpr float kHalfM = 0.5f * static_cast<float>(kN - 1) * kStepM;
        std::vector<float> samples(static_cast<std::size_t>(kN) * kN, 0.0f);
        if (ground) {
            for (JPH::uint32 iz = 0; iz < kN; ++iz) {
                const double z = centerZ - static_cast<double>(kHalfM) + static_cast<double>(iz) * static_cast<double>(kStepM);
                for (JPH::uint32 ix = 0; ix < kN; ++ix) {
                    const double x = centerX - static_cast<double>(kHalfM) + static_cast<double>(ix) * static_cast<double>(kStepM);
                    const V3 pEcef = originEcef + east * x + south * z;
                    double lat = 0.0, lon = 0.0, h = 0.0;
                    ms::damage::geodetic(pEcef, lat, lon, h);
                    const double gh = ground->heightAboveEllipsoidM(lat, lon);
                    // Account for Earth curvature drop relative to tangent plane so ECEF altitude matches exact surface!
                    const double ySample = (std::isfinite(gh) ? gh : originGroundH) - h;
                    samples[static_cast<std::size_t>(iz) * kN + ix] = static_cast<float>(ySample);
                }
            }
        }
        JPH::HeightFieldShapeSettings hf(samples.data(),
                                         JPH::Vec3(static_cast<float>(centerX) - kHalfM, 0.0f, static_cast<float>(centerZ) - kHalfM),
                                         JPH::Vec3(kStepM, 1.0f, kStepM), kN);
        auto shapeRes = hf.Create();
        if (shapeRes.HasError()) {
            // Fallback static ground slab at y = 0.
            JPH::BodyCreationSettings bcs(new JPH::BoxShape(JPH::Vec3(4000.0f, 10.0f, 4000.0f)),
                                          JPH::RVec3(static_cast<float>(centerX), -10.0f, static_cast<float>(centerZ)),
                                          JPH::Quat::sIdentity(), JPH::EMotionType::Static, jolt_layers::kTerrain);
            bcs.mFriction = 0.72f;
            bcs.mRestitution = 0.14f;
            terrainBodyId = bi.CreateAndAddBody(bcs, JPH::EActivation::DontActivate);
        } else {
            JPH::BodyCreationSettings bcs(shapeRes.Get(), JPH::RVec3::sZero(), JPH::Quat::sIdentity(), JPH::EMotionType::Static,
                                          jolt_layers::kTerrain);
            bcs.mFriction = 0.72f;
            bcs.mRestitution = 0.14f;
            terrainBodyId = bi.CreateAndAddBody(bcs, JPH::EActivation::DontActivate);
        }
        hasTerrain = !terrainBodyId.IsInvalid();
        terrainCenterX = centerX;
        terrainCenterZ = centerZ;
    }
};

DebrisField::DebrisField() { clear(); }
DebrisField::~DebrisField() = default;

void DebrisField::clear() noexcept {
    jolt_.reset();
    for (auto& b : bodies_) b = DebrisBody{};
    activeCount_ = 0;
    nextId_ = 1;
}

void DebrisField::ensureJolt(const double originEcef[3], const sim::GroundProvider* ground) {
    if (!jolt_) jolt_ = std::make_unique<JoltWorld>();
    if (!jolt_->initialized) jolt_->setupFrame(originEcef, ground);
}

void DebrisField::unregisterJoltBody(DebrisBody& b) noexcept {
    if (jolt_ && b.joltBodyId != 0xFFFFFFFFu) {
        auto& bi = jolt_->physics.GetBodyInterfaceNoLock();
        const JPH::BodyID bid(b.joltBodyId);
        if (bi.IsAdded(bid)) bi.RemoveBody(bid);
        bi.DestroyBody(bid);
        b.joltBodyId = 0xFFFFFFFFu;
    }
}

void DebrisField::registerJoltBody(DebrisBody& b) {
    if (!isSolidRigidBody(b.archetype) || !jolt_ || !jolt_->initialized) return;
    unregisterJoltBody(b);
    auto& bi = jolt_->physics.GetBodyInterfaceNoLock();
    const float hx = static_cast<float>(std::clamp(b.halfExtentsM[0], 0.12, 12.0));
    const float hy = static_cast<float>(std::clamp(b.halfExtentsM[1], 0.12, 12.0));
    const float hz = static_cast<float>(std::clamp(b.halfExtentsM[2], 0.08, 6.0));
    auto shape = new JPH::BoxShape(JPH::Vec3(hx, hy, hz), std::min({hx, hy, hz}) * 0.25f);
    JPH::BodyCreationSettings bcs(shape, jolt_->posToEun(v3(b.positionEcef)), jolt_->quatBodyToEun(b.quatBodyToEcef),
                                  JPH::EMotionType::Dynamic, jolt_layers::kDebris);
    bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = static_cast<float>(std::clamp(b.massKg, 0.5, 40000.0));
    bcs.mFriction = b.inWater ? 0.38f : 0.68f;
    bcs.mRestitution = b.inWater ? 0.22f : 0.20f;
    bcs.mLinearDamping = 0.0f;
    bcs.mAngularDamping = 0.06f;
    bcs.mMaxLinearVelocity = 1500.0f;
    bcs.mMaxAngularVelocity = 40.0f;
    bcs.mLinearVelocity = jolt_->vecToEun(v3(b.velocityEcef));
    // Convert body angular velocity to EUN world angular velocity.
    const V3 omegaEcef = quatRotate(b.quatBodyToEcef, v3(b.omegaBodyRadS));
    bcs.mAngularVelocity = jolt_->vecToEun(omegaEcef);
    const JPH::BodyID bid = bi.CreateAndAddBody(bcs, JPH::EActivation::Activate);
    if (!bid.IsInvalid()) b.joltBodyId = bid.GetIndexAndSequenceNumber();
}

void DebrisField::retire(DebrisBody& b) noexcept {
    if (!b.alive) return;
    unregisterJoltBody(b);
    b.alive = false;
    if (activeCount_ > 0) --activeCount_;
}

DebrisBody& DebrisField::allocate(ipc::DebrisArchetypeV1 archetype) noexcept {
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
    unregisterJoltBody(b);
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

    constexpr int kShardCount = 8;
    constexpr int kSparkCount = 8;
    const double totalMass = std::max(in.sectionMassKg, 20.0);
    const double shardPoolMass = in.spawnPrimaryChunk ? totalMass * 0.10 : std::min(totalMass * 0.08, 35.0);
    const double primaryMass = std::max(totalMass - shardPoolMass, 1.0);

    std::array<V3, kShardCount> shardDv{};
    V3 sumP{};
    double sumM = 0.0;
    std::array<double, kShardCount> shardM{};
    for (int i = 0; i < kShardCount; ++i) {
        const double m = (shardPoolMass / kShardCount) * (0.5 + rng.uniform());
        shardM[static_cast<std::size_t>(i)] = m;
        const V3 dir = randomUnitSphere(rng);
        const double spd = 6.0 + 24.0 * rng.uniform();
        shardDv[static_cast<std::size_t>(i)] = dir * spd;
        sumP += shardDv[static_cast<std::size_t>(i)] * m;
        sumM += m;
    }
    const V3 meanDv = sumM > 1e-6 ? sumP * (1.0 / sumM) : V3{};

    if (in.spawnPrimaryChunk) {
        auto& body = allocate(in.isWing ? ipc::DebrisArchetypeV1::WingFragment : ipc::DebrisArchetypeV1::AirframeHull);
        body.ownerVehicleId = in.ownerVehicleId;
        body.partTag = in.partTag;
        store(body.positionEcef, secPos);
        store(body.velocityEcef, secVel);
        mat3ToQuat(in.rotationBodyToEcef, body.quatBodyToEcef);
        body.omegaBodyRadS[0] = in.angularRateBodyRadS[0] + (rng.uniform() - 0.5) * 3.8;
        body.omegaBodyRadS[1] = in.angularRateBodyRadS[1] + (rng.uniform() - 0.5) * 3.2;
        body.omegaBodyRadS[2] = in.angularRateBodyRadS[2] + (rng.uniform() - 0.5) * 3.8;
        body.massKg = primaryMass;
        const double span = std::clamp(in.spanM, 1.4, 12.0);
        if (in.isWing) {
            body.halfExtentsM[0] = 0.52 * span;
            body.halfExtentsM[1] = 0.58 * span;
            body.halfExtentsM[2] = 0.18;
            // Time-averaged projected tumbling drag area of a sheared wing plate (~0.24 * span^2, not 0.85 * span^2).
            body.areaM2 = std::clamp(0.24 * span * span, 0.9, 3.8);
        } else {
            body.halfExtentsM[0] = 0.72 * span;
            body.halfExtentsM[1] = 0.44 * span;
            body.halfExtentsM[2] = 0.38 * span;
            body.areaM2 = std::clamp(0.32 * span * span, 1.0, 4.5);
        }
        body.fuelRemainingKg = std::max(0.0, in.fuelKg);
        body.burning = in.burning || (in.fuelKg > 15.0 && rng.uniform() < 0.65);
        body.burnRateKgS = body.burning ? std::clamp(body.fuelRemainingKg / 25.0, 0.25, 2.5) : 0.0;
        body.temperatureK = body.burning ? 1450.0 : 520.0;
        body.smokeIntensity = body.burning ? 0.95 : 0.35;
        body.maxAgeS = 90.0;
        if (jolt_ && jolt_->initialized) registerJoltBody(body);
    }

    for (int i = 0; i < kShardCount; ++i) {
        const bool hotChunk = in.burning && (i < 4);
        auto& s = allocate(hotChunk ? ipc::DebrisArchetypeV1::BurningChunk : ipc::DebrisArchetypeV1::PanelShard);
        s.ownerVehicleId = in.ownerVehicleId;
        s.partTag = static_cast<ipc::DebrisPartTagV1>(static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard0) + (i % 6));
        const V3 jitter = randomUnitSphere(rng) * (0.4 + 1.2 * rng.uniform());
        store(s.positionEcef, rootPos + jitter);
        store(s.velocityEcef, secVel + (shardDv[static_cast<std::size_t>(i)] - meanDv));
        randomUnitQuat(rng, s.quatBodyToEcef);
        s.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 12.0;
        s.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 12.0;
        s.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 12.0;
        s.massKg = shardM[static_cast<std::size_t>(i)];
        const double l = 0.15 + 0.19 * rng.uniform();
        s.halfExtentsM[0] = l;
        s.halfExtentsM[1] = l * (0.45 + 0.42 * rng.uniform());
        s.halfExtentsM[2] = 0.04 + 0.03 * rng.uniform();
        s.areaM2 = 2.2 * s.halfExtentsM[0] * s.halfExtentsM[1];
        s.burning = hotChunk;
        s.fuelRemainingKg = hotChunk ? 3.0 + 5.0 * rng.uniform() : 0.0;
        s.burnRateKgS = hotChunk ? 0.30 : 0.0;
        s.temperatureK = hotChunk ? 1650.0 : 620.0 + 280.0 * rng.uniform();
        s.smokeIntensity = hotChunk ? 0.75 : 0.10;
        s.maxAgeS = 30.0;
        if (jolt_ && jolt_->initialized) registerJoltBody(s);
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
        sp.temperatureK = 2250.0;
        sp.smokeIntensity = 0.0;
        sp.maxAgeS = 1.6 + 2.0 * rng.uniform();
    }
}

void DebrisField::spawnCatastrophicKill(const CatastrophicSpawn& in, Rng& rng) {
    const V3 origin = v3(in.positionEcef);
    const V3 parentVel = v3(in.velocityEcef);
    const double totalMass = std::max(in.airframeMassKg, 500.0);
    // Physical radial blast velocity scale for real tearing separation without unnatural hyper-scatter.
    const double blastScale = std::clamp(std::sqrt(std::max(in.explosionTntKg, 2.0) * 65.0), 12.0, 52.0);

    // Real 5 aircraft structural sections (rendered from actual aircraft GLB fsim:section:* subgraphs!)
    // spawned right at their true body offsets with exact parent orientation -> ZERO teleport, ZERO pop!
    static constexpr double kDefaultOffsets[5][3] = {
        { 3.84,  0.00, -0.03}, // SectionNose
        { 0.00,  0.00,  0.00}, // SectionCentre (ruptured core)
        { 0.27, -2.13, -0.24}, // SectionLeftWing
        { 0.27,  2.13, -0.24}, // SectionRightWing
        {-4.30,  0.00,  0.34}, // SectionTail
    };
    static constexpr double kSectionMassShare[5] = {0.16, 0.34, 0.14, 0.14, 0.14};
    static constexpr ipc::DebrisArchetypeV1 kSectionArch[5] = {
        ipc::DebrisArchetypeV1::AirframeHull,
        ipc::DebrisArchetypeV1::AirframeHull,
        ipc::DebrisArchetypeV1::WingFragment,
        ipc::DebrisArchetypeV1::WingFragment,
        ipc::DebrisArchetypeV1::AirframeHull,
    };
    static constexpr ipc::DebrisPartTagV1 kSectionTag[5] = {
        ipc::DebrisPartTagV1::SectionNose,
        ipc::DebrisPartTagV1::SectionCentre,
        ipc::DebrisPartTagV1::SectionLeftWing,
        ipc::DebrisPartTagV1::SectionRightWing,
        ipc::DebrisPartTagV1::SectionTail,
    };
    static constexpr double kSectionHalf[5][3] = {
        {2.2, 0.72, 0.72}, // nose
        {2.6, 0.95, 0.85}, // centre
        {1.8, 2.15, 0.18}, // left wing
        {1.8, 2.15, 0.18}, // right wing
        {2.1, 1.55, 1.25}, // tail + vertical fin
    };

    constexpr int kSections = 5;
    constexpr int kVoronoiShards = 12;
    constexpr int kBurningChunks = 6;
    constexpr int kSparks = 24;
    constexpr int kStructuralTotal = kSections + kVoronoiShards + kBurningChunks;

    std::array<double, kStructuralTotal> masses{};
    std::array<V3, kStructuralTotal> dvs{};
    std::array<V3, kStructuralTotal> offsets{};
    V3 sumP{};

    // Reserve 8% of airframe mass for Voronoi skin shards & burning internal chunks, 92% in remaining intact sections.
    // Skip any section already detached earlier in Grade 3 StructuralBreakup so delayed Grade 3 -> Grade 4 never duplicates a wing/tail!
    double sumSec = 0.0;
    for (int i = 0; i < kSections; ++i) {
        if ((in.alreadySeparatedMask & (1u << i)) != 0u) continue;
        sumSec += kSectionMassShare[i];
    }
    const double secMassTotal = sumSec > 1e-6 ? totalMass * 0.92 : 0.0;
    for (int i = 0; i < kSections; ++i) {
        if ((in.alreadySeparatedMask & (1u << i)) != 0u || !(sumSec > 1e-6)) {
            masses[static_cast<std::size_t>(i)] = 0.0;
            continue;
        }
        masses[static_cast<std::size_t>(i)] = secMassTotal * (kSectionMassShare[i] / sumSec);
        const double ox = in.hasSectionOffsets ? in.sectionOffsetsBodyM[i][0] : kDefaultOffsets[i][0];
        const double oy = in.hasSectionOffsets ? in.sectionOffsetsBodyM[i][1] : kDefaultOffsets[i][1];
        const double oz = in.hasSectionOffsets ? in.sectionOffsetsBodyM[i][2] : kDefaultOffsets[i][2];
        const V3 bodyOff{ox, oy, oz};
        offsets[static_cast<std::size_t>(i)] = mulMat3(in.rotationBodyToEcef, bodyOff);
        // Natural outward radial separation velocity away from ruptured centre fuselage spine!
        V3 sepBody = (i == 1) ? V3{0.2, 0.15 * (rng.uniform() - 0.5), 0.35}
                              : normalized(bodyOff + V3{(rng.uniform() - 0.5) * 0.6, (rng.uniform() - 0.5) * 0.6, (rng.uniform() - 0.5) * 0.6});
        const double sepSpd = (i == 1) ? (3.0 + 3.0 * rng.uniform()) : (blastScale * (0.28 + 0.25 * rng.uniform()));
        dvs[static_cast<std::size_t>(i)] = mulMat3(in.rotationBodyToEcef, sepBody * sepSpd);
        sumP += dvs[static_cast<std::size_t>(i)] * masses[static_cast<std::size_t>(i)];
    }

    double shardWeightSum = 0.0;
    std::array<double, kVoronoiShards + kBurningChunks> sw{};
    for (int j = 0; j < kVoronoiShards + kBurningChunks; ++j) {
        sw[static_cast<std::size_t>(j)] = (j < kBurningChunks ? 2.0 : 0.8) * (0.6 + 0.8 * rng.uniform());
        shardWeightSum += sw[static_cast<std::size_t>(j)];
    }
    const double shardPoolTotal = totalMass - secMassTotal;
    for (int j = 0; j < kVoronoiShards + kBurningChunks; ++j) {
        const std::size_t idx = static_cast<std::size_t>(kSections + j);
        masses[idx] = shardPoolTotal * (sw[static_cast<std::size_t>(j)] / shardWeightSum);
        const V3 dir = randomUnitSphere(rng);
        offsets[idx] = dir * (0.6 + 2.4 * rng.uniform());
        const double spd = blastScale * (j < kBurningChunks ? (0.50 + 0.55 * rng.uniform()) : (0.75 + 0.85 * rng.uniform()));
        dvs[idx] = dir * spd;
        sumP += dvs[idx] * masses[idx];
    }
    // Exact linear momentum conservation: sum(m_i * v_i) == M_total * v_parent!
    const V3 dvCorrection = sumP * (1.0 / totalMass);

    const double fuelPerSection = std::max(0.0, in.remainingFuelKg) * 0.65 / 4.0;
    for (int i = 0; i < kStructuralTotal; ++i) {
        const bool isSection = i < kSections;
        if (isSection && !(masses[static_cast<std::size_t>(i)] > 0.0)) continue;
        const bool isBurnChunk = !isSection && i < (kSections + kBurningChunks);
        const ipc::DebrisArchetypeV1 arch =
            isSection ? kSectionArch[i] : (isBurnChunk ? ipc::DebrisArchetypeV1::BurningChunk : ipc::DebrisArchetypeV1::PanelShard);
        auto& b = allocate(arch);
        b.ownerVehicleId = in.ownerVehicleId;
        b.partTag = isSection
                        ? kSectionTag[i]
                        : static_cast<ipc::DebrisPartTagV1>(static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard0) + (i % 6));
        store(b.positionEcef, origin + offsets[static_cast<std::size_t>(i)]);
        store(b.velocityEcef, parentVel + dvs[static_cast<std::size_t>(i)] - dvCorrection);
        if (isSection) {
            // Exact aircraft orientation at break instant (seamless continuity from intact hull!).
            mat3ToQuat(in.rotationBodyToEcef, b.quatBodyToEcef);
            const double tumble = (i == 1) ? 1.8 : 4.2;
            b.omegaBodyRadS[0] = in.angularRateBodyRadS[0] + (rng.uniform() - 0.5) * tumble;
            b.omegaBodyRadS[1] = in.angularRateBodyRadS[1] + (rng.uniform() - 0.5) * tumble;
            b.omegaBodyRadS[2] = in.angularRateBodyRadS[2] + (rng.uniform() - 0.5) * tumble;
            b.halfExtentsM[0] = kSectionHalf[i][0];
            b.halfExtentsM[1] = kSectionHalf[i][1];
            b.halfExtentsM[2] = kSectionHalf[i][2];
            b.areaM2 = 2.6 * b.halfExtentsM[0] * b.halfExtentsM[1];
            // Centre fuselage + one wing + tail carry active aviation fuel fires -> drive real NVIDIA Flow flames & thick black smoke!
            const bool burns = (i == 1 || i == 2 || i == 4);
            b.burning = burns;
            b.fuelRemainingKg = burns ? std::max(18.0, fuelPerSection * (i == 1 ? 1.8 : 0.7)) : 0.0;
            b.burnRateKgS = burns ? std::clamp(b.fuelRemainingKg / 18.0, 1.5, 9.0) : 0.0;
            b.temperatureK = burns ? (i == 1 ? 1850.0 : 1550.0) : 780.0;
            b.smokeIntensity = burns ? 1.0 : 0.45;
            b.maxAgeS = 120.0;
        } else {
            randomUnitQuat(rng, b.quatBodyToEcef);
            b.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 14.0;
            b.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 14.0;
            b.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 14.0;
            b.halfExtentsM[0] = isBurnChunk ? (0.18 + 0.18 * rng.uniform()) : (0.14 + 0.20 * rng.uniform());
            b.halfExtentsM[1] = b.halfExtentsM[0] * (0.45 + 0.42 * rng.uniform());
            b.halfExtentsM[2] = isBurnChunk ? 0.09 : 0.04;
            b.areaM2 = 2.2 * b.halfExtentsM[0] * b.halfExtentsM[1];
            b.burning = isBurnChunk;
            b.fuelRemainingKg = isBurnChunk ? (3.5 + 6.0 * rng.uniform()) : 0.0;
            b.burnRateKgS = isBurnChunk ? 0.45 : 0.0;
            b.temperatureK = isBurnChunk ? (1600.0 + 400.0 * rng.uniform()) : (680.0 + 300.0 * rng.uniform());
            b.smokeIntensity = isBurnChunk ? 0.85 : 0.08;
            b.maxAgeS = 65.0;
        }
        b.massKg = masses[static_cast<std::size_t>(i)];
        if (jolt_ && jolt_->initialized) registerJoltBody(b);
    }

    // Incandescent high-velocity metallic sparks spraying cleanly out of the explosion core.
    for (int i = 0; i < kSparks; ++i) {
        auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
        sp.ownerVehicleId = in.ownerVehicleId;
        store(sp.positionEcef, origin + randomUnitSphere(rng) * (0.5 + 1.8 * rng.uniform()));
        store(sp.velocityEcef, parentVel + randomUnitSphere(rng) * (blastScale * (1.2 + 1.8 * rng.uniform())));
        randomUnitQuat(rng, sp.quatBodyToEcef);
        sp.massKg = 0.06;
        sp.areaM2 = 0.01;
        sp.halfExtentsM[0] = 0.36; sp.halfExtentsM[1] = 0.07; sp.halfExtentsM[2] = 0.07;
        sp.burning = true;
        sp.temperatureK = 2400.0;
        sp.smokeIntensity = 0.0;
        sp.maxAgeS = 1.6 + 2.4 * rng.uniform();
    }
}

void DebrisField::spawnImpactCloud(const ImpactCloudSpawn& in, Rng& rng) {
    const V3 pos = v3(in.positionEcef);
    const V3 normal = normalized(v3(in.normalEcef));
    const V3 after = v3(in.afterVelocityEcef);
    const V3 incoming = v3(in.incomingVelocityEcef);
    const double afterSpeed = length(after);
    const double inSpeed = length(incoming);
    V3 t1, t2;
    orthonormalBasis(normal, t1, t2);
    const bool water = (in.surface == ms::damage::SurfaceKind::Water);
    const double normalIn = -dot(incoming, normal);
    const bool steepLandDive = !water && in.hullDisintegrated && (normalIn > 0.65 * std::max(inSpeed, 1.0));
    const V3 fwdHoriz = length(after - normal * dot(after, normal)) > 1.0
                            ? normalized(after - normal * dot(after, normal))
                            : (length(incoming - normal * dot(incoming, normal)) > 1.0
                                   ? normalized(incoming - normal * dot(incoming, normal))
                                   : t1);
    const V3 sideHoriz = normalized(cross(normal, fwdHoriz));

    if (in.hullDisintegrated) {
        // Real 5 aircraft GLB sections hurled violently out of steep land dive or high-speed water shatter!
        static constexpr ipc::DebrisPartTagV1 kTags[5] = {
            ipc::DebrisPartTagV1::SectionTail,
            ipc::DebrisPartTagV1::SectionLeftWing,
            ipc::DebrisPartTagV1::SectionRightWing,
            ipc::DebrisPartTagV1::SectionNose,
            ipc::DebrisPartTagV1::SectionCentre,
        };
        static constexpr ipc::DebrisArchetypeV1 kArchs[5] = {
            ipc::DebrisArchetypeV1::AirframeHull,
            ipc::DebrisArchetypeV1::WingFragment,
            ipc::DebrisArchetypeV1::WingFragment,
            ipc::DebrisArchetypeV1::AirframeHull,
            ipc::DebrisArchetypeV1::AirframeHull,
        };
        static constexpr std::uint8_t kSecBits[5] = {
            1u << 4, // tail
            1u << 2, // left wing
            1u << 3, // right wing
            1u << 0, // nose
            1u << 1, // centre
        };
        static constexpr double kHalf[5][3] = {
            {2.0, 1.5, 1.2}, {1.8, 2.1, 0.18}, {1.8, 2.1, 0.18}, {2.0, 0.72, 0.72}, {2.4, 0.92, 0.82},
        };
        for (int i = 0; i < 5; ++i) {
            if ((in.alreadySeparatedMask & kSecBits[i]) != 0u) continue; // already spawned via separated(); never duplicate!
            auto& b = allocate(kArchs[i]);
            b.ownerVehicleId = in.ownerVehicleId;
            b.partTag = kTags[i];
            if (steepLandDive) {
                // Near-vertical ground explosion: blast broken F-16C wings/tail/fuselage in a 360-deg upward crater cone
                // so burning structural wreckage arcs high out of the fireball and bounces across real terrain!
                const double az = (static_cast<double>(i) / 5.0) * 2.0 * kPi + (rng.uniform() - 0.5) * 0.65;
                const V3 radDir = fwdHoriz * std::cos(az) + sideHoriz * std::sin(az);
                store(b.positionEcef, pos + radDir * (1.8 + 2.6 * rng.uniform()) + normal * (2.6 + 2.8 * rng.uniform()));
                const double upSpd = 18.0 + 18.0 * rng.uniform();
                const double outSpd = 14.0 + 18.0 * rng.uniform();
                store(b.velocityEcef, radDir * outSpd + fwdHoriz * (4.0 + 6.0 * rng.uniform()) + normal * upSpd);
            } else {
                const double sideSign = (i == 1) ? -1.0 : ((i == 2) ? 1.0 : (rng.uniform() - 0.5) * 1.4);
                store(b.positionEcef, pos + fwdHoriz * (1.0 + 1.8 * i) + sideHoriz * (sideSign * (1.5 + 2.0 * rng.uniform())) +
                                          normal * (water ? (1.0 + 1.0 * rng.uniform()) : (2.0 + 2.0 * rng.uniform())));
                // Real ocean impact behaves almost like concrete on entry: heavy airframe sections lose most forward speed
                // immediately into hydrodynamic drag & settle within ~20-45 m rather than skiing 150 m like speedboats!
                const double upSpd = water ? (1.8 + 0.022 * inSpeed * (0.25 + 0.75 * rng.uniform()))
                                           : (12.0 + 0.14 * inSpeed * (0.45 + 0.85 * rng.uniform()));
                const double fwdSpd = (water ? 0.17 : 0.42) * inSpeed * (0.50 + 0.55 * rng.uniform());
                const double latSpd = (water ? (3.5 + 0.045 * inSpeed) : (5.0 + 0.09 * inSpeed)) * sideSign * (0.5 + 0.7 * rng.uniform());
                store(b.velocityEcef, fwdHoriz * fwdSpd + sideHoriz * latSpd + normal * upSpd);
            }
            mat3ToQuat(in.rotationBodyToEcef, b.quatBodyToEcef);
            b.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 7.5;
            b.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 7.0;
            b.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 7.5;
            b.massKg = std::max(180.0, in.massKg * 0.16);
            b.halfExtentsM[0] = kHalf[i][0];
            b.halfExtentsM[1] = kHalf[i][1];
            b.halfExtentsM[2] = kHalf[i][2];
            b.areaM2 = 2.5 * b.halfExtentsM[0] * b.halfExtentsM[1];
            b.burning = steepLandDive ? (i != 3) : (!water && (i == 0 || i == 4));
            b.fuelRemainingKg = b.burning ? std::min(in.poolKg * (steepLandDive ? 0.18 : 0.15), 45.0) : 0.0;
            b.burnRateKgS = b.burning ? (steepLandDive ? 1.8 : 1.2) : 0.0;
            b.temperatureK = b.burning ? (steepLandDive ? 1750.0 : 1600.0) : (water ? 360.0 : 620.0);
            b.smokeIntensity = b.burning ? 1.0 : (water ? 0.12 : 0.30);
            b.inWater = water;
            b.maxAgeS = water ? 18.0 : 75.0;
            if (jolt_ && jolt_->initialized) registerJoltBody(b);
        }
    }

    // Real manifold Voronoi painted aircraft shards bouncing/skipping away from impact!
    const int shardCount = steepLandDive ? 24 : (in.hullDisintegrated ? 16 : 14);
    for (int i = 0; i < shardCount; ++i) {
        const bool hotChunk = !water && (steepLandDive ? (i < 8) : (i < 4));
        auto& b = allocate(hotChunk ? ipc::DebrisArchetypeV1::BurningChunk : ipc::DebrisArchetypeV1::PanelShard);
        b.ownerVehicleId = in.ownerVehicleId;
        b.partTag = static_cast<ipc::DebrisPartTagV1>(static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard0) + (i % 6));
        const double ang = 2.0 * kPi * rng.uniform();
        const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
        store(b.positionEcef, pos + radial * (1.2 + 3.8 * rng.uniform()) + normal * (1.4 + 2.4 * rng.uniform()));
        if (steepLandDive) {
            const double upSpd = 16.0 + 28.0 * rng.uniform();
            const double outSpd = 12.0 + 26.0 * rng.uniform();
            store(b.velocityEcef, normal * upSpd + radial * outSpd + fwdHoriz * (3.0 + 6.0 * rng.uniform()));
        } else {
            const double upSpd = (8.0 + 0.15 * inSpeed) * (0.4 + 0.9 * rng.uniform());
            const double outSpd = (7.0 + 0.14 * inSpeed) * (0.4 + 0.8 * rng.uniform());
            store(b.velocityEcef, after * (0.42 + 0.38 * rng.uniform()) + normal * upSpd + radial * outSpd);
        }
        randomUnitQuat(rng, b.quatBodyToEcef);
        b.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 12.0;
        b.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 12.0;
        b.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 12.0;
        b.massKg = 14.0 + 22.0 * rng.uniform();
        b.halfExtentsM[0] = 0.35 + 0.45 * rng.uniform();
        b.halfExtentsM[1] = 0.22 + 0.30 * rng.uniform();
        b.halfExtentsM[2] = 0.08 + 0.06 * rng.uniform();
        b.areaM2 = 2.2 * b.halfExtentsM[0] * b.halfExtentsM[1];
        b.burning = hotChunk;
        b.fuelRemainingKg = hotChunk ? (steepLandDive ? 7.5 : 4.0) : 0.0;
        b.burnRateKgS = hotChunk ? 0.55 : 0.0;
        b.temperatureK = hotChunk ? 1750.0 : (water ? 320.0 : 600.0);
        b.smokeIntensity = hotChunk ? 0.92 : 0.05;
        b.inWater = water;
        b.maxAgeS = water ? 16.0 : 55.0;
        if (jolt_ && jolt_->initialized) registerJoltBody(b);
    }

    if (water) {
        // Quench & hydrodynamically decelerate ALL already-separated sections (nose/tail broken off earlier in this
        // impact step) so they lose entry momentum into the incompressible ocean rather than skiing 150 m in a straight arrow!
        for (auto& existing : bodies_) {
            if (!existing.alive || existing.ownerVehicleId != in.ownerVehicleId || !isSolidRigidBody(existing.archetype)) continue;
            if (!existing.inWater) {
                for (int k = 0; k < 3; ++k) existing.velocityEcef[k] *= 0.40;
                if (jolt_ && jolt_->initialized && existing.joltBodyId != 0xFFFFFFFFu) {
                    jolt_->physics.GetBodyInterfaceNoLock().SetLinearVelocity(
                        JPH::BodyID(existing.joltBodyId), jolt_->vecToEun(v3(existing.velocityEcef)));
                }
            }
            existing.inWater = true;
            existing.burning = false;
            existing.fuelRemainingKg = 0.0;
            existing.burnRateKgS = 0.0;
            existing.smokeIntensity = 0.0;
            existing.temperatureK = 320.0;
            existing.maxAgeS = std::min(existing.maxAgeS, 18.0);
        }
        // 1) Towering hollow-centered white seawater spire/geyser erupting straight skyward from entry cavity (24 curtains)
        for (int i = 0; i < 24; ++i) {
            auto& p = allocate(ipc::DebrisArchetypeV1::SplashPlume);
            p.ownerVehicleId = in.ownerVehicleId;
            p.partTag = ipc::DebrisPartTagV1::Generic; // Grounded vertical hydrodynamic water spire
            const double ang = (static_cast<double>(i) / 24.0) * 2.0 * kPi + (rng.uniform() - 0.5) * 0.35;
            const bool outerCrown = (i >= 14);
            const double rad = outerCrown ? (5.2 + 5.5 * rng.uniform()) : (1.4 + 4.4 * rng.uniform());
            const double fwdOff = 2.6 + std::cos(ang) * rad * 1.10;
            const double sideOff = std::sin(ang) * rad;
            store(p.positionEcef, pos + fwdHoriz * fwdOff + sideHoriz * sideOff + normal * 1.0);
            const double outDrift = outerCrown ? (4.8 + 7.5 * rng.uniform()) : (1.8 + 3.8 * rng.uniform());
            store(p.velocityEcef, fwdHoriz * (std::cos(ang) * outDrift + 4.2) + sideHoriz * (std::sin(ang) * outDrift));
            randomUnitQuat(rng, p.quatBodyToEcef);
            p.massKg = 45.0;
            p.areaM2 = 5.5;
            p.halfExtentsM[0] = outerCrown ? (4.6 + 3.0 * rng.uniform()) : (4.2 + 3.0 * rng.uniform());
            p.halfExtentsM[1] = outerCrown ? (11.0 + 9.0 * rng.uniform()) : (24.0 + 16.0 * (0.60 + 0.45 * rng.uniform()));
            p.halfExtentsM[2] = 1.8;
            p.burning = false;
            p.inWater = true;
            p.grounded = true;
            p.temperatureK = 288.0;
            p.smokeIntensity = 0.85;
            p.maxAgeS = outerCrown ? (2.1 + 1.0 * rng.uniform()) : (2.8 + 1.2 * rng.uniform());
        }
        // 2) Subtle circular radial base-surge mist & lacy seafoam ring expanding smoothly around the entry cavity
        //    (delicate low-opacity aerated rings with dark ocean water clearly visible — NEVER a solid painted white arrow!)
        for (int i = 0; i < 8; ++i) {
            auto& fw = allocate(ipc::DebrisArchetypeV1::SplashPlume);
            fw.ownerVehicleId = in.ownerVehicleId;
            fw.partTag = ipc::DebrisPartTagV1::FoamWake;
            const double ang = (static_cast<double>(i) / 8.0) * 2.0 * kPi + (rng.uniform() - 0.5) * 0.45;
            const double rad = 1.5 + 3.8 * rng.uniform();
            const V3 radial = fwdHoriz * std::cos(ang) + sideHoriz * std::sin(ang);
            store(fw.positionEcef, pos + fwdHoriz * 1.2 + radial * rad + normal * 0.35);
            store(fw.velocityEcef, radial * (4.5 + 6.5 * rng.uniform()) + fwdHoriz * (2.0 + 2.5 * rng.uniform()));
            randomUnitQuat(rng, fw.quatBodyToEcef);
            fw.massKg = 100.0;
            fw.areaM2 = 12.0;
            const double r = 4.2 + 2.4 * rng.uniform();
            fw.halfExtentsM[0] = r; fw.halfExtentsM[1] = r; fw.halfExtentsM[2] = 0.6;
            fw.inWater = true;
            fw.grounded = true;
            fw.temperatureK = 288.0;
            fw.smokeIntensity = (i < 3) ? 0.54 : (0.32 + 0.16 * rng.uniform());
            fw.maxAgeS = 8.0 + 4.0 * rng.uniform();
        }
    } else if (steepLandDive) {
        // Near-vertical steep-dive ground explosion:
        // 1) Wide irregular charred blast crater floor beneath the erupting fireball
        for (int i = 0; i < 8; ++i) {
            auto& sc = allocate(ipc::DebrisArchetypeV1::SplashPlume);
            sc.ownerVehicleId = in.ownerVehicleId;
            sc.partTag = ipc::DebrisPartTagV1::ScorchMark;
            const double ang = (static_cast<double>(i) / 8.0) * 2.0 * kPi + (rng.uniform() - 0.5) * 0.4;
            const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
            const double off = (i == 0) ? 0.0 : (3.0 + 5.5 * rng.uniform());
            store(sc.positionEcef, pos + radial * off);
            store(sc.velocityEcef, radial); // orient scorch lobe radially outward around crater
            randomUnitQuat(rng, sc.quatBodyToEcef);
            sc.massKg = 100.0;
            sc.areaM2 = 18.0;
            const double r = (i == 0) ? 9.5 : (5.5 + 3.5 * rng.uniform());
            sc.halfExtentsM[0] = r * 1.25; sc.halfExtentsM[1] = r; sc.halfExtentsM[2] = 0.25;
            sc.grounded = true;
            sc.temperatureK = 350.0;
            sc.smokeIntensity = 0.90;
            sc.maxAgeS = 35.0;
        }
        // 2) Continuous overlapping low-hugging shockwave dust skirt around the crater base (NEVER separated dots!).
        for (int i = 0; i < 14; ++i) {
            auto& d = allocate(ipc::DebrisArchetypeV1::SplashPlume);
            d.ownerVehicleId = in.ownerVehicleId;
            d.partTag = ipc::DebrisPartTagV1::DustPlume;
            const double ang = (static_cast<double>(i) / 14.0) * 2.0 * kPi + (rng.uniform() - 0.5) * 0.22;
            const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
            store(d.positionEcef, pos + radial * (3.2 + 3.2 * rng.uniform()) + normal * (1.2 + 1.2 * rng.uniform()));
            store(d.velocityEcef, radial * (6.5 + 7.5 * rng.uniform()) + normal * (1.5 + 2.8 * rng.uniform()));
            randomUnitQuat(rng, d.quatBodyToEcef);
            d.massKg = 28.0;
            d.areaM2 = 5.0;
            const double r = 4.4 + 2.0 * rng.uniform();
            d.halfExtentsM[0] = r; d.halfExtentsM[1] = r; d.halfExtentsM[2] = r;
            d.temperatureK = 320.0;
            d.smokeIntensity = 0.65;
            d.maxAgeS = 3.8 + 1.6 * rng.uniform();
        }
        // 3) 44 brilliant incandescent high-arcing crater sparks showering out of the fireball!
        for (int i = 0; i < 44; ++i) {
            auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
            sp.ownerVehicleId = in.ownerVehicleId;
            const double ang = 2.0 * kPi * rng.uniform();
            const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
            store(sp.positionEcef, pos + radial * (0.8 + 3.2 * rng.uniform()) + normal * (1.2 + 2.2 * rng.uniform()));
            store(sp.velocityEcef, normal * (18.0 + 30.0 * rng.uniform()) + radial * (12.0 + 28.0 * rng.uniform()));
            randomUnitQuat(rng, sp.quatBodyToEcef);
            sp.massKg = 0.07;
            sp.areaM2 = 0.01;
            sp.halfExtentsM[0] = 0.44; sp.halfExtentsM[1] = 0.075; sp.halfExtentsM[2] = 0.075;
            sp.burning = true;
            sp.temperatureK = 2500.0;
            sp.smokeIntensity = 0.0;
            sp.maxAgeS = 1.6 + 2.2 * rng.uniform();
        }
    } else {
        // Shallow ground belly-slide crash: billowing warm brown-tan soil/dust ejecta plume + bright scraping sparks!
        for (int i = 0; i < 16; ++i) {
            auto& d = allocate(ipc::DebrisArchetypeV1::SplashPlume);
            d.ownerVehicleId = in.ownerVehicleId;
            d.partTag = ipc::DebrisPartTagV1::DustPlume;
            const double ang = 2.0 * kPi * rng.uniform();
            const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
            store(d.positionEcef, pos + radial * (2.0 + 6.0 * rng.uniform()) + normal * (1.2 + 2.0 * rng.uniform()));
            store(d.velocityEcef, after * (0.20 + 0.25 * rng.uniform()) + normal * (5.0 + 11.0 * rng.uniform()) + radial * (4.0 + 10.0 * rng.uniform()));
            randomUnitQuat(rng, d.quatBodyToEcef);
            d.massKg = 25.0;
            d.areaM2 = 5.0;
            const double r = 2.4 + 2.8 * rng.uniform();
            d.halfExtentsM[0] = r; d.halfExtentsM[1] = r; d.halfExtentsM[2] = r;
            d.temperatureK = 300.0;
            d.smokeIntensity = 0.78;
            d.maxAgeS = 4.5 + 3.0 * rng.uniform();
        }
        for (int i = 0; i < 24; ++i) {
            auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
            sp.ownerVehicleId = in.ownerVehicleId;
            const double ang = 2.0 * kPi * rng.uniform();
            const V3 radial = t1 * std::cos(ang) + t2 * std::sin(ang);
            store(sp.positionEcef, pos + radial * (1.0 + 3.0 * rng.uniform()) + normal * 0.8);
            store(sp.velocityEcef, after * (0.45 + 0.40 * rng.uniform()) + normal * (6.0 + 18.0 * rng.uniform()) + radial * (8.0 + 22.0 * rng.uniform()));
            randomUnitQuat(rng, sp.quatBodyToEcef);
            sp.massKg = 0.06;
            sp.areaM2 = 0.01;
            sp.halfExtentsM[0] = 0.38; sp.halfExtentsM[1] = 0.06; sp.halfExtentsM[2] = 0.06;
            sp.burning = true;
            sp.temperatureK = 2400.0;
            sp.smokeIntensity = 0.0;
            sp.maxAgeS = 1.2 + 1.8 * rng.uniform();
        }
    }
    (void)afterSpeed;
}

void DebrisField::spawnSlideSparks(const SlideSparkSpawn& in, Rng& rng) {
    const V3 pos = v3(in.positionEcef);
    const V3 vel = v3(in.velocityEcef);
    const V3 normal = normalized(v3(in.normalEcef));
    const double speed = length(vel);
    if (!(speed > 4.0)) return;
    V3 t1, t2;
    orthonormalBasis(normal, t1, t2);
    const V3 fwd = normalized(vel - normal * dot(vel, normal));
    const V3 sideDir = normalized(cross(normal, fwd));
    const bool water = (in.surface == ms::damage::SurfaceKind::Water);

    if (water) {
        for (int i = 0; i < 4; ++i) {
            auto& p = allocate(ipc::DebrisArchetypeV1::SplashPlume);
            p.ownerVehicleId = in.ownerVehicleId;
            p.partTag = (i == 0) ? ipc::DebrisPartTagV1::FoamWake : ipc::DebrisPartTagV1::Generic;
            const double side = (rng.uniform() - 0.5) * 4.5;
            store(p.positionEcef, pos + sideDir * side + normal * (i == 0 ? 0.4 : 1.2));
            store(p.velocityEcef, vel * 0.25 + normal * (i == 0 ? 0.0 : (7.0 + 0.18 * speed * rng.uniform())) + sideDir * (side * 2.5));
            randomUnitQuat(rng, p.quatBodyToEcef);
            p.massKg = 25.0;
            p.areaM2 = 3.0;
            const double r = (i == 0) ? 3.5 : (1.8 + 1.6 * rng.uniform());
            p.halfExtentsM[0] = r; p.halfExtentsM[1] = r; p.halfExtentsM[2] = r;
            p.inWater = true;
            p.grounded = (i == 0);
            p.temperatureK = 288.0;
            p.smokeIntensity = 0.80;
            p.maxAgeS = (i == 0) ? 7.0 : (2.2 + 1.4 * rng.uniform());
        }
        return;
    }

    // Ground belly-slide:
    // 1) Dense showering incandescent belly-friction sparks spraying aft & sideways!
    for (int i = 0; i < 6; ++i) {
        auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
        sp.ownerVehicleId = in.ownerVehicleId;
        const double side = (rng.uniform() - 0.5) * 2.2;
        store(sp.positionEcef, pos + fwd * (-1.0 + 2.5 * rng.uniform()) + sideDir * side - normal * 0.65);
        store(sp.velocityEcef, vel * (0.28 + 0.35 * rng.uniform()) + normal * (2.5 + 0.09 * speed * rng.uniform()) +
                                   sideDir * ((rng.uniform() - 0.5) * (6.0 + 0.14 * speed)));
        randomUnitQuat(rng, sp.quatBodyToEcef);
        sp.massKg = 0.04;
        sp.areaM2 = 0.008;
        sp.halfExtentsM[0] = 0.34; sp.halfExtentsM[1] = 0.055; sp.halfExtentsM[2] = 0.055;
        sp.burning = true;
        sp.temperatureK = 2400.0;
        sp.smokeIntensity = 0.0;
        sp.maxAgeS = 0.55 + 0.85 * rng.uniform();
    }
    // 2) Billowing warm brown-tan dirt/dust cloud kicked up on both sides of the scraping fuselage!
    for (int i = 0; i < 2; ++i) {
        auto& d = allocate(ipc::DebrisArchetypeV1::SplashPlume);
        d.ownerVehicleId = in.ownerVehicleId;
        d.partTag = ipc::DebrisPartTagV1::DustPlume;
        const double sideSign = (i == 0) ? -1.0 : 1.0;
        store(d.positionEcef, pos - fwd * (1.8 + 2.0 * rng.uniform()) + sideDir * (sideSign * (1.5 + 1.8 * rng.uniform())) + normal * 0.6);
        store(d.velocityEcef, vel * 0.16 + normal * (3.5 + 5.5 * rng.uniform()) + sideDir * (sideSign * (4.0 + 6.0 * rng.uniform())));
        randomUnitQuat(rng, d.quatBodyToEcef);
        d.massKg = 20.0;
        d.areaM2 = 4.5;
        const double r = 2.0 + 1.8 * rng.uniform();
        d.halfExtentsM[0] = r; d.halfExtentsM[1] = r; d.halfExtentsM[2] = r;
        d.temperatureK = 300.0;
        d.smokeIntensity = 0.72;
        d.maxAgeS = 3.8 + 2.2 * rng.uniform();
    }
    // 3) Dark scorched skid furrow left on the terrain surface behind the fuselage!
    {
        auto& sc = allocate(ipc::DebrisArchetypeV1::SplashPlume);
        sc.ownerVehicleId = in.ownerVehicleId;
        sc.partTag = ipc::DebrisPartTagV1::ScorchMark;
        store(sc.positionEcef, pos - fwd * 2.5 - normal * 0.85);
        store(sc.velocityEcef, fwd * 1.0); // encodes skid alignment direction for ground quad!
        randomUnitQuat(rng, sc.quatBodyToEcef);
        sc.massKg = 100.0;
        sc.areaM2 = 8.0;
        sc.halfExtentsM[0] = 4.2; sc.halfExtentsM[1] = 2.2; sc.halfExtentsM[2] = 0.25;
        sc.grounded = true;
        sc.temperatureK = 320.0;
        sc.smokeIntensity = 0.82;
        sc.maxAgeS = 25.0;
    }
}

void DebrisField::spawnHitBurst(const HitBurstSpawn& in, Rng& rng) {
    if (in.fragmentsStriking <= 0) return;
    const V3 pos = v3(in.positionEcef);
    const V3 vel = v3(in.velocityEcef);
    V3 exitDir = normalized(v3(in.exitDirEcef));
    if (length(exitDir) < 0.5) exitDir = {0.0, 0.0, 1.0};

    // Strict grade- and source-aware debris scaling:
    // - 20/30 mm gun bullets already emit crisp GPU ricochet sparks & HEI puffs via BulletAirframeHit;
    //   never flood the air with half-meter 3-D metal chunks on every bullet perforation!
    //   Only emit at most 1 tiny torn skin fleck when an actual panel rips under severe cannon fire.
    // - Missile warhead fragments scale cleanly from Grade 1 (subtle sparks + at most 1 tiny fleck)
    //   to Grade 2 (1-2 small flecks) to Grade 3 (2-4 ragged fragments).
    int sparkCount = 0;
    int flakeCount = 0;
    double flakeMinM = 0.07;
    double flakeSpanM = 0.05;
    double flakeLifeS = 5.5;
    bool allowHotFlake = false;

    if (in.isGunHit) {
        sparkCount = 0;
        if (in.panelsTorn > 0 && in.grade >= ms::damage::DamageGrade::FireAndStall) {
            flakeCount = 1;
            flakeMinM = 0.07;
            flakeSpanM = 0.05;
            flakeLifeS = 4.5;
        }
    } else if (in.grade <= ms::damage::DamageGrade::LightPerforation) {
        sparkCount = std::clamp(3 + in.fragmentsStriking / 24, 3, 5);
        flakeCount = (in.panelsTorn > 0 || in.fragmentsStriking >= 12) ? 1 : 0;
        flakeMinM = 0.07;
        flakeSpanM = 0.05;
        flakeLifeS = 5.5;
    } else if (in.grade == ms::damage::DamageGrade::FireAndStall) {
        sparkCount = std::clamp(4 + in.fragmentsStriking / 20, 4, 8);
        flakeCount = std::clamp((in.panelsTorn > 0 ? 1 : 0) + in.fragmentsStriking / 40, 1, 2);
        flakeMinM = 0.09;
        flakeSpanM = 0.07;
        flakeLifeS = 7.5;
        allowHotFlake = in.fiery && in.panelsTorn > 0;
    } else {
        sparkCount = std::clamp(6 + in.fragmentsStriking / 18, 6, 10);
        flakeCount = std::clamp(2 + in.fragmentsStriking / 35, 2, 4);
        flakeMinM = 0.12;
        flakeSpanM = 0.10;
        flakeLifeS = 11.0;
        allowHotFlake = in.fiery;
    }

    for (int i = 0; i < sparkCount; ++i) {
        auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
        sp.ownerVehicleId = in.ownerVehicleId;
        const V3 randDir = normalized(exitDir * 0.55 + randomUnitSphere(rng) * 0.75);
        store(sp.positionEcef, pos + randomUnitSphere(rng) * (0.25 + 0.60 * rng.uniform()));
        store(sp.velocityEcef, vel * (0.82 + 0.14 * rng.uniform()) + randDir * (12.0 + 30.0 * rng.uniform()));
        randomUnitQuat(rng, sp.quatBodyToEcef);
        sp.massKg = 0.03;
        sp.areaM2 = 0.006;
        sp.halfExtentsM[0] = 0.18; sp.halfExtentsM[1] = 0.040; sp.halfExtentsM[2] = 0.040;
        sp.burning = true;
        sp.temperatureK = in.fiery ? 2350.0 : 2100.0;
        sp.smokeIntensity = 0.0;
        sp.maxAgeS = 0.40 + 0.55 * rng.uniform();
    }
    for (int i = 0; i < flakeCount; ++i) {
        const bool hot = allowHotFlake && (i == 0);
        auto& fl = allocate(hot ? ipc::DebrisArchetypeV1::BurningChunk : ipc::DebrisArchetypeV1::PanelShard);
        fl.ownerVehicleId = in.ownerVehicleId;
        fl.partTag = static_cast<ipc::DebrisPartTagV1>(static_cast<std::uint8_t>(ipc::DebrisPartTagV1::VoronoiShard0) + (i % 6));
        const V3 randDir = normalized(exitDir * 0.60 + randomUnitSphere(rng) * 0.65);
        store(fl.positionEcef, pos + randomUnitSphere(rng) * (0.3 + 0.6 * rng.uniform()));
        store(fl.velocityEcef, vel * (0.86 + 0.10 * rng.uniform()) + randDir * (7.0 + 18.0 * rng.uniform()));
        randomUnitQuat(rng, fl.quatBodyToEcef);
        fl.omegaBodyRadS[0] = (rng.uniform() - 0.5) * 18.0;
        fl.omegaBodyRadS[1] = (rng.uniform() - 0.5) * 18.0;
        fl.omegaBodyRadS[2] = (rng.uniform() - 0.5) * 18.0;
        fl.massKg = 0.4 + 1.0 * rng.uniform();
        fl.halfExtentsM[0] = flakeMinM + flakeSpanM * rng.uniform();
        fl.halfExtentsM[1] = fl.halfExtentsM[0] * (0.55 + 0.35 * rng.uniform());
        fl.halfExtentsM[2] = 0.02;
        fl.areaM2 = 2.0 * fl.halfExtentsM[0] * fl.halfExtentsM[1];
        fl.burning = hot;
        fl.fuelRemainingKg = hot ? 0.6 : 0.0;
        fl.burnRateKgS = hot ? 0.20 : 0.0;
        fl.temperatureK = hot ? 1550.0 : 480.0;
        fl.smokeIntensity = hot ? 0.35 : 0.0;
        fl.maxAgeS = flakeLifeS + 3.0 * rng.uniform();
        if (jolt_ && jolt_->initialized) registerJoltBody(fl);
    }
}

void DebrisField::spawnDamageEmbers(const DamageEmberSpawn& in, Rng& rng) {
    const V3 pos = v3(in.positionEcef);
    const V3 vel = v3(in.velocityEcef);
    const double inten = std::clamp(in.intensity, 0.2, 1.0);
    const int count = inten > 0.65 ? 2 : 1;
    for (int i = 0; i < count; ++i) {
        auto& sp = allocate(ipc::DebrisArchetypeV1::SparkEmber);
        sp.ownerVehicleId = in.ownerVehicleId;
        store(sp.positionEcef, pos + randomUnitSphere(rng) * (0.35 + 0.55 * rng.uniform()));
        store(sp.velocityEcef, vel * (0.76 + 0.14 * rng.uniform()) + randomUnitSphere(rng) * (4.0 + 10.0 * inten));
        randomUnitQuat(rng, sp.quatBodyToEcef);
        sp.massKg = 0.025;
        sp.areaM2 = 0.008;
        sp.halfExtentsM[0] = 0.22; sp.halfExtentsM[1] = 0.055; sp.halfExtentsM[2] = 0.055;
        sp.burning = true;
        sp.temperatureK = 2100.0 + 350.0 * rng.uniform();
        sp.smokeIntensity = 0.0;
        sp.maxAgeS = 0.65 + 0.90 * rng.uniform();
    }
}

std::uint32_t DebrisField::spawnGroundWreck(const GroundWreckSpawn& in, const sim::GroundProvider* ground, Rng& rng) {
    ensureJolt(in.positionEcef, ground);
    auto& bi = jolt_->physics.GetBodyInterfaceNoLock();

    const float hx = static_cast<float>(std::clamp(in.halfExtentsM[0], 1.5, 25.0));
    const float hy = static_cast<float>(std::clamp(in.halfExtentsM[1], 0.8, 12.0));
    const float hz = static_cast<float>(std::clamp(in.halfExtentsM[2], 0.8, 6.0));
    // Place fuselage cleanly ON TOP OF the actual terrain heightfield (belly clearance >= hz + 0.35 m -> NEVER buried!).
    double lat = 0.0, lon = 0.0, h = 0.0;
    ms::damage::geodetic(v3(in.positionEcef), lat, lon, h);
    const double groundH = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const V3 up{cl * co, cl * so, sl};
    const double minClear = static_cast<double>(hz) + 0.35;
    V3 startPos = v3(in.positionEcef);
    if (h < groundH + minClear) startPos += up * (groundH + minClear - h);

    // Give the fuselage a natural bank & slight yaw skid on impact so it looks violently battered rather than laser-level!
    double qBodyToEcef[4];
    mat3ToQuat(in.rotationBodyToEcef, qBodyToEcef);
    const double bankRad = units::degreesToRadians(in.bankKickDeg * (rng.uniform() < 0.5 ? 1.0 : -1.0));
    const double pitchRad = units::degreesToRadians(3.5); // nose slightly up on belly impact
    const double yawKickRad = units::degreesToRadians((rng.uniform() - 0.5) * 14.0);
    // Small body-axis perturbation quaternion (roll X, pitch Y, yaw Z in FRD body frame).
    const double cr = std::cos(0.5 * bankRad), sr = std::sin(0.5 * bankRad);
    const double cp = std::cos(0.5 * pitchRad), sp = std::sin(0.5 * pitchRad);
    const double cy = std::cos(0.5 * yawKickRad), sy = std::sin(0.5 * yawKickRad);
    const double qKick[4] = {
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    };
    double qInit[4];
    quatMul(qBodyToEcef, qKick, qInit);

    auto shape = new JPH::BoxShape(JPH::Vec3(hx, hy, hz), 0.15f);
    JPH::BodyCreationSettings bcs(shape, jolt_->posToEun(startPos), jolt_->quatBodyToEun(qInit), JPH::EMotionType::Dynamic,
                                  jolt_layers::kDebris);
    bcs.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    bcs.mMassPropertiesOverride.mMass = static_cast<float>(std::clamp(in.massKg, 500.0, 200000.0));
    bcs.mFriction = in.surface == ms::damage::SurfaceKind::Rock ? 0.72f : 0.58f;
    bcs.mRestitution = 0.08f;
    bcs.mLinearDamping = 0.04f;
    bcs.mAngularDamping = 0.65f;
    bcs.mMaxLinearVelocity = 500.0f;
    bcs.mMaxAngularVelocity = 12.0f;
    // Strip downward dive velocity into horizontal skid + slight bounce!
    V3 vel = v3(in.velocityEcef);
    const double vn = dot(vel, up);
    if (vn < 0.0) vel = vel - up * vn;
    vel += up * 1.4; // slight crisp ground-contact bounce
    bcs.mLinearVelocity = jolt_->vecToEun(vel);
    // Gentle crash yaw/roll skid rate so fuselage visibly twists & leans while skidding!
    const V3 omegaEcef = quatRotate(qInit, V3{(bankRad > 0.0 ? 0.35 : -0.35), 0.12, (yawKickRad >= 0.0 ? 0.28 : -0.28)});
    bcs.mAngularVelocity = jolt_->vecToEun(omegaEcef);

    const JPH::BodyID bid = bi.CreateAndAddBody(bcs, JPH::EActivation::Activate);
    if (bid.IsInvalid()) return 0;
    const std::uint32_t handle = jolt_->nextWreckHandle++;
    JoltWorld::WreckMeta meta;
    meta.bodyId = bid;
    meta.surface = in.surface;
    meta.halfExtentsM[0] = hx; meta.halfExtentsM[1] = hy; meta.halfExtentsM[2] = hz;
    meta.alive = true;
    jolt_->wrecks[handle] = meta;
    return handle;
}

bool DebrisField::queryGroundWreck(std::uint32_t handle, double posEcef[3], double velEcef[3], double rotBodyToEcef[9],
                                   double eulerRad[3]) const noexcept {
    if (!jolt_ || !jolt_->initialized) return false;
    const auto it = jolt_->wrecks.find(handle);
    if (it == jolt_->wrecks.end() || !it->second.alive) return false;
    const auto& bi = jolt_->physics.GetBodyInterfaceNoLock();
    const JPH::BodyID bid = it->second.bodyId;
    const JPH::RVec3 pEun = bi.GetCenterOfMassPosition(bid);
    const JPH::Vec3 vEun = bi.GetLinearVelocity(bid);
    const JPH::Quat qEun = bi.GetRotation(bid);

    const V3 p = jolt_->posToEcef(JPH::Vec3(pEun));
    const V3 v = jolt_->vecToEcef(vEun);
    double qBodyToEcef[4];
    jolt_->quatEunToBodyEcef(qEun, qBodyToEcef);
    quatToMat3(qBodyToEcef, rotBodyToEcef);
    store(posEcef, p);
    store(velEcef, v);

    // Derive NED roll, pitch, yaw from rotBodyToEcef for HUD/telemetry readouts.
    double lat = 0.0, lon = 0.0, h = 0.0;
    ms::damage::geodetic(p, lat, lon, h);
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const V3 north{-sl * co, -sl * so, cl};
    const V3 east{-so, co, 0.0};
    const V3 down{-cl * co, -cl * so, -sl};
    const V3 bx{rotBodyToEcef[0], rotBodyToEcef[3], rotBodyToEcef[6]};
    const V3 by{rotBodyToEcef[1], rotBodyToEcef[4], rotBodyToEcef[7]};
    const V3 bz{rotBodyToEcef[2], rotBodyToEcef[5], rotBodyToEcef[8]};
    const double bxN = dot(bx, north), bxE = dot(bx, east), bxD = dot(bx, down);
    const double byD = dot(by, down), bzD = dot(bz, down);
    eulerRad[0] = std::atan2(byD, bzD);
    eulerRad[1] = std::asin(std::clamp(-bxD, -1.0, 1.0));
    eulerRad[2] = std::atan2(bxE, bxN);
    return true;
}

void DebrisField::step(double dt, const sim::GroundProvider* ground, const sim::EnvironmentState& env, double simTime) {
    (void)simTime;
    const bool hasWrecks = jolt_ && !jolt_->wrecks.empty();
    if ((activeCount_ == 0 && !hasWrecks) || !(dt > 0.0)) return;

    // Ensure Jolt world is initialized around the first active solid body!
    if (!jolt_ || !jolt_->initialized) {
        for (const auto& b : bodies_) {
            if (b.alive && isSolidRigidBody(b.archetype)) {
                ensureJolt(b.positionEcef, ground);
                break;
            }
        }
    }
    if (jolt_ && jolt_->initialized) {
        // Register any solid debris bodies spawned prior to first step (when ground_ wasn't available yet).
        for (auto& b : bodies_) {
            if (b.alive && isSolidRigidBody(b.archetype) && b.joltBodyId == 0xFFFFFFFFu) registerJoltBody(b);
        }
        // Keep terrain heightfield centered near active low-altitude wreckage if it slides > 450 m.
        for (const auto& [h, w] : jolt_->wrecks) {
            (void)h;
            if (!w.alive) continue;
            const JPH::RVec3 pw = jolt_->physics.GetBodyInterfaceNoLock().GetCenterOfMassPosition(w.bodyId);
            const double dx = static_cast<double>(pw.GetX()) - jolt_->terrainCenterX;
            const double dz = static_cast<double>(pw.GetZ()) - jolt_->terrainCenterZ;
            if (dx * dx + dz * dz > 450.0 * 450.0) {
                jolt_->rebuildTerrain(static_cast<double>(pw.GetX()), static_cast<double>(pw.GetZ()), ground);
                break;
            }
        }
    }

    const double windDirRad = units::degreesToRadians(std::isfinite(env.windDirectionDeg) ? env.windDirectionDeg : 0.0);
    const double windSpd = std::isfinite(env.windSpeedMs) ? std::max(env.windSpeedMs, 0.0) : 0.0;
    const double windNorth = -windSpd * std::cos(windDirRad);
    const double windEast = -windSpd * std::sin(windDirRad);

    // 1. Apply aerodynamic drag, pitch-tumbling torques & water-surface drag to Jolt rigid bodies BEFORE Jolt step.
    if (jolt_ && jolt_->initialized) {
        auto& bi = jolt_->physics.GetBodyInterfaceNoLock();
        for (auto& b : bodies_) {
            if (!b.alive || !isSolidRigidBody(b.archetype) || b.joltBodyId == 0xFFFFFFFFu) continue;
            const JPH::BodyID bid(b.joltBodyId);
            const JPH::RVec3 pEun = bi.GetCenterOfMassPosition(bid);
            const JPH::Vec3 vEun = bi.GetLinearVelocity(bid);
            const V3 pos = jolt_->posToEcef(JPH::Vec3(pEun));
            const V3 vel = jolt_->vecToEcef(vEun);

            double lat = 0.0, lon = 0.0, h = 0.0;
            ms::damage::geodetic(pos, lat, lon, h);
            const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
            const V3 north{-sl * co, -sl * so, cl};
            const V3 east{-so, co, 0.0};
            const V3 windEcef = north * windNorth + east * windEast;
            const V3 vRel = vel - windEcef;
            const double speed = length(vRel);
            const double rho = ms::damage::standardDensity(h);
            if (speed > 0.2 && b.massKg > 0.1) {
                const V3 vHat = vRel * (1.0 / speed);
                const V3 plateNormal = quatRotate(b.quatBodyToEcef, {0.0, 0.0, 1.0});
                const double cosInc = std::abs(dot(vHat, plateNormal));
                const double cd = 0.38 + 0.92 * cosInc;
                const double qDyn = 0.5 * rho * speed * speed;
                const double dragF = std::min(qDyn * cd * b.areaM2, b.massKg * speed / std::max(dt, 1e-3) * 0.75);
                bi.AddForce(bid, jolt_->vecToEun(vHat * (-dragF)));
                const V3 torqueEcef = cross(plateNormal, vHat) * (0.14 * qDyn * b.areaM2);
                bi.AddTorque(bid, jolt_->vecToEun(torqueEcef));
            }
            // Hydrodynamic drag & splash damping when water-impact wreckage skips/sinks on ocean surface.
            if (b.inWater && pEun.GetY() <= 0.6f) {
                bi.AddForce(bid, -vEun * static_cast<float>(b.massKg * 1.45));
                if (b.burning) b.fuelRemainingKg = std::max(0.0, b.fuelRemainingKg - 3.0 * dt);
            }
        }
        // Keep ground-wreck fuselage skidding smoothly with gentle aerodynamic drag & roll-cap stabilization.
        for (const auto& [h, w] : jolt_->wrecks) {
            (void)h;
            if (!w.alive) continue;
            const JPH::Vec3 v = bi.GetLinearVelocity(w.bodyId);
            const float spd = v.Length();
            if (spd > 1.0f) {
                // Aerodynamic drag on sliding fuselage (~0.5 * 1.225 * CdA * v^2)
                const float aeroF = 0.5f * 1.225f * 5.2f * spd * spd;
                bi.AddForce(w.bodyId, -v * (aeroF / spd));
            }
        }
        // Deterministic single-threaded Jolt Physics step!
        jolt_->physics.Update(static_cast<float>(dt), 1, &jolt_->tempAlloc, &jolt_->jobSystem);
    }

    // 2. Sync Jolt rigid bodies back to ECEF + advance lightweight particle plumes (sparks, geysers, dust, foam, scorch).
    struct WaveWakeEvent {
        V3 pos;
        V3 vel;
        double speed = 0.0;
        std::uint32_t ownerVehicleId = 0;
        std::uint32_t seed = 0;
        bool isMajor = false;
        bool isSlap = false;
    };
    std::vector<WaveWakeEvent> waveSkipSplashes;
    std::vector<WaveWakeEvent> waveFoamWakes;
    for (auto& b : bodies_) {
        if (!b.alive) continue;
        b.ageS += dt;
        if (b.ageS >= b.maxAgeS) {
            retire(b);
            continue;
        }

        if (isSolidRigidBody(b.archetype) && jolt_ && b.joltBodyId != 0xFFFFFFFFu) {
            auto& bi = jolt_->physics.GetBodyInterfaceNoLock();
            const JPH::BodyID bid(b.joltBodyId);
            const JPH::RVec3 pEun = bi.GetCenterOfMassPosition(bid);
            const JPH::Vec3 vEun = bi.GetLinearVelocity(bid);
            const JPH::Quat qEun = bi.GetRotation(bid);
            const JPH::Vec3 wEun = bi.GetAngularVelocity(bid);

            const V3 pos = jolt_->posToEcef(JPH::Vec3(pEun));
            const V3 vel = jolt_->vecToEcef(vEun);
            double qEcef[4];
            jolt_->quatEunToBodyEcef(qEun, qEcef);
            const V3 wBody = quatRotateInverse(qEcef, jolt_->vecToEcef(wEun));

            double lat = 0.0, lon = 0.0, h = 0.0;
            ms::damage::geodetic(pos, lat, lon, h);
            const double gh = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
            const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
            const V3 up{cl * co, cl * so, sl};
            const double agl = h - gh;
            const bool wasGrounded = b.grounded;
            const bool isMajor = (b.archetype == ipc::DebrisArchetypeV1::AirframeHull ||
                                  b.archetype == ipc::DebrisArchetypeV1::WingFragment);
            b.grounded = agl <= std::max(b.halfExtentsM[2] + 0.55, 0.9);
            if (b.inWater && agl <= std::max(b.halfExtentsM[2] + 0.9, 1.4)) {
                // Realistic heavy hydrodynamic ocean deceleration: incompressible water rapidly arrests wreckage within ~20-45 m.
                const float damp = static_cast<float>(std::exp(-5.4 * dt));
                bi.SetLinearVelocity(bid, JPH::Vec3(vEun.GetX() * damp, vEun.GetY() * 0.62f, vEun.GetZ() * damp));
                bi.SetAngularVelocity(bid, wEun * damp);
            }
            if (b.inWater) {
                const V3 horizVel = vel - up * dot(vel, up);
                const double hSpd = length(horizVel);
                const std::uint32_t detSeed = b.id * 2654435761u ^ static_cast<std::uint32_t>(b.ageS * 120.0);
                // Sparse localized splash rings ONLY on genuine wave slaps (plus at most a brief short entry disturbance < 0.55 s)
                // -> eliminates cartoon solid-white highway/arrow stripes painted across the ocean!
                if (b.grounded && !wasGrounded && length(vel) > 8.0) {
                    if (waveSkipSplashes.size() < 6)
                        waveSkipSplashes.push_back({pos, horizVel, length(vel), b.ownerVehicleId, detSeed, isMajor, true});
                    if (waveFoamWakes.size() < 4)
                        waveFoamWakes.push_back({pos, horizVel, std::max(hSpd, 10.0), b.ownerVehicleId, detSeed ^ 0x9e3779b9u, isMajor, true});
                } else if (isMajor && b.ageS < 0.55 && agl <= std::max(b.halfExtentsM[2] + 1.8, 2.2) && hSpd > 14.0) {
                    b.waveWakeTimerS += dt;
                    if (b.waveWakeTimerS >= 0.22 && waveFoamWakes.size() < 3) {
                        b.waveWakeTimerS = 0.0;
                        waveFoamWakes.push_back({pos, horizVel, hSpd, b.ownerVehicleId, detSeed, isMajor, false});
                    }
                }
            }
            store(b.positionEcef, pos);
            store(b.velocityEcef, vel);
            for (int k = 0; k < 4; ++k) b.quatBodyToEcef[k] = qEcef[k];
            store(b.omegaBodyRadS, wBody);
        } else {
            // Lightweight ballistic/buoyant VFX particles (SparkEmber, SplashPlume, DustPlume, FoamWake, ScorchMark).
            V3 pos = v3(b.positionEcef);
            V3 vel = v3(b.velocityEcef);
            double lat = 0.0, lon = 0.0, h = 0.0;
            ms::damage::geodetic(pos, lat, lon, h);
            const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
            const V3 up{cl * co, cl * so, sl};
            const V3 north{-sl * co, -sl * so, cl};
            const V3 east{-so, co, 0.0};
            const V3 windEcef = north * windNorth + east * windEast;
            const double gh = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;

            if (b.partTag == ipc::DebrisPartTagV1::ScorchMark) {
                // Static dark scorched furrow locked cleanly on top of terrain surface!
                pos += up * (gh + 0.12 - h);
                store(b.positionEcef, pos);
                continue;
            }
            if (b.partTag == ipc::DebrisPartTagV1::FoamWake) {
                // Spreading white foam patch drifting & damping cleanly above ocean wave troughs.
                vel = vel * std::exp(-1.1 * dt);
                pos += vel * dt;
                ms::damage::geodetic(pos, lat, lon, h);
                const double gh2 = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
                pos += up * (gh2 + 0.85 - h);
                store(b.positionEcef, pos);
                store(b.velocityEcef, vel);
                continue;
            }
            if (b.partTag == ipc::DebrisPartTagV1::DustPlume) {
                // Warm soil dust cloud: rapid air drag, slight upward thermal/turbulent billow, stays above ground.
                const V3 vRel = vel - windEcef;
                vel = vel - vRel * std::min(0.95, 1.35 * dt);
                vel += up * (0.45 * dt);
                pos += vel * dt;
                ms::damage::geodetic(pos, lat, lon, h);
                const double gh2 = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
                if (h < gh2 + 0.8) pos += up * (gh2 + 0.8 - h);
                store(b.positionEcef, pos);
                store(b.velocityEcef, vel);
                continue;
            }

            if (b.archetype == ipc::DebrisArchetypeV1::SplashPlume && b.partTag == ipc::DebrisPartTagV1::Generic) {
                // Grounded hydrodynamic water curtain: anchored at sea level, rises sharply to peakH in first 28%
                // of lifetime, then collapses smoothly straight downward under gravity into the waves!
                vel = vel * std::exp(-1.4 * dt);
                pos += vel * dt;
                ms::damage::geodetic(pos, lat, lon, h);
                const double gh2 = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
                const double u = std::clamp(b.ageS / std::max(b.maxAgeS, 0.5), 0.0, 1.0);
                // Asymmetric ballistic fountain profile: fast sharp shoot-up, smooth gravity rain-down collapse!
                const double rise = u < 0.28 ? std::sin((u / 0.28) * (0.5 * kPi))
                                             : std::cos(((u - 0.28) / 0.72) * (0.5 * kPi));
                const double peakH = std::max(b.halfExtentsM[1], 4.0);
                const double curH = std::max(1.2, peakH * std::pow(std::max(rise, 0.0), 0.78));
                b.halfExtentsM[2] = 0.5 * curH;
                // Center of vertical curtain sits at seaLevel + 0.45 * curH so base stays rooted below wave crests!
                pos += up * (gh2 + 0.45 * curH - h);
                store(b.positionEcef, pos);
                store(b.velocityEcef, vel);
                continue;
            }

            // SparkEmber ballistic arc.
            const double rho = ms::damage::standardDensity(h);
            const V3 vRel = vel - windEcef;
            const double speed = length(vRel);
            V3 acc = up * (-kG);
            if (speed > 1e-4 && b.massKg > 1e-4) {
                const V3 vHat = vRel * (1.0 / speed);
                const double cd = b.archetype == ipc::DebrisArchetypeV1::SplashPlume ? 0.62 : 0.85;
                const double dragAcc = std::min(0.5 * rho * speed * speed * cd * b.areaM2 / b.massKg, speed / dt * 0.85);
                acc += vHat * (-dragAcc);
            }
            vel += acc * dt;
            pos += vel * dt;
            integrateQuat(b.quatBodyToEcef, v3(b.omegaBodyRadS), dt);
            ms::damage::geodetic(pos, lat, lon, h);
            const double gh2 = ground ? ground->heightAboveEllipsoidM(lat, lon) : 0.0;
            if (h <= gh2 + 0.15) {
                retire(b);
                continue;
            }
            store(b.positionEcef, pos);
            store(b.velocityEcef, vel);
        }

        // Fuel burn-down and physical cooling on solid chunks.
        double lat = 0.0, lon = 0.0, h = 0.0;
        ms::damage::geodetic(v3(b.positionEcef), lat, lon, h);
        if (b.burning && b.fuelRemainingKg > 0.0 && b.burnRateKgS > 0.0) {
            b.fuelRemainingKg = std::max(0.0, b.fuelRemainingKg - b.burnRateKgS * dt);
            if (b.fuelRemainingKg <= 0.0) {
                b.burning = false;
                b.burnRateKgS = 0.0;
            }
        }
        const double ambientK = std::isfinite(env.temperatureSeaLevelK) ? std::max(216.0, env.temperatureSeaLevelK - 0.0065 * std::max(h, 0.0)) : 288.15;
        if (b.burning) {
            const double targetK = b.archetype == ipc::DebrisArchetypeV1::SparkEmber ? 2100.0 : 1600.0;
            b.temperatureK += (targetK - b.temperatureK) * std::min(1.0, 0.5 * dt);
        } else {
            const double coolRate = b.archetype == ipc::DebrisArchetypeV1::SparkEmber ? 2.2 : (b.inWater && b.grounded ? 3.0 : 0.18);
            b.temperatureK = ambientK + (b.temperatureK - ambientK) * std::exp(-coolRate * dt);
            b.smokeIntensity *= std::exp(-0.10 * dt);
        }
    }

    // Deterministic hash [0, 1) for causal wave-skip splashes & trailing seafoam wake (preserves bit-exact determinism!).
    const auto detHash01 = [](std::uint32_t x) noexcept -> double {
        x ^= x >> 16;
        x *= 0x7feb352du;
        x ^= x >> 15;
        x *= 0x846ca68bu;
        x ^= x >> 16;
        return static_cast<double>(x & 0x00FFFFFFu) / 16777216.0;
    };

    // Spawn secondary vertical ocean splash curtains where skipping aircraft wreckage slaps the waves!
    for (const WaveWakeEvent& ev : waveSkipSplashes) {
        double lat = 0.0, lon = 0.0, h = 0.0;
        ms::damage::geodetic(ev.pos, lat, lon, h);
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        const V3 up{cl * co, cl * so, sl};
        const double u = detHash01(ev.seed);
        auto& p = allocate(ipc::DebrisArchetypeV1::SplashPlume);
        p.ownerVehicleId = ev.ownerVehicleId;
        p.partTag = ipc::DebrisPartTagV1::Generic;
        store(p.positionEcef, ev.pos + up * 1.0);
        store(p.velocityEcef, ev.vel * 0.14);
        p.massKg = 35.0;
        p.areaM2 = 3.0;
        p.halfExtentsM[0] = ev.isMajor ? (2.2 + 1.4 * u) : (1.5 + 0.9 * u);
        p.halfExtentsM[1] = std::clamp(ev.speed * (ev.isMajor ? 0.28 : 0.20), 5.5, 16.5);
        p.halfExtentsM[2] = 1.2;
        p.inWater = true;
        p.grounded = true;
        p.temperatureK = 288.0;
        p.smokeIntensity = 0.80;
        p.maxAgeS = 1.9 + 0.9 * u;
    }

    // Causally spawn localized blooming ocean foam wakes behind skipping/ploughing wreckage!
    for (const WaveWakeEvent& ev : waveFoamWakes) {
        double lat = 0.0, lon = 0.0, h = 0.0;
        ms::damage::geodetic(ev.pos, lat, lon, h);
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        const V3 up{cl * co, cl * so, sl};
        const double u0 = detHash01(ev.seed ^ 0x517cc1b7u);
        const double u1 = detHash01(ev.seed ^ 0x27220a95u);
        const V3 fwd = length(ev.vel) > 0.5 ? normalized(ev.vel) : V3{1.0, 0.0, 0.0};
        const V3 side = normalized(cross(up, fwd));
        auto& fw = allocate(ipc::DebrisArchetypeV1::SplashPlume);
        fw.ownerVehicleId = ev.ownerVehicleId;
        fw.partTag = ipc::DebrisPartTagV1::FoamWake;
        store(fw.positionEcef, ev.pos + side * ((u0 - 0.5) * (ev.isMajor ? 3.5 : 2.0)) + up * 0.35);
        store(fw.velocityEcef, ev.vel * 0.12 + side * ((u0 - 0.5) * 2.4));
        fw.massKg = 80.0;
        fw.areaM2 = 10.0;
        const double r = ev.isMajor ? (ev.isSlap ? (3.8 + 1.8 * u1) : (3.0 + 1.6 * u1)) : (2.0 + 1.2 * u1);
        fw.halfExtentsM[0] = r;
        fw.halfExtentsM[1] = r;
        fw.halfExtentsM[2] = 0.6;
        fw.inWater = true;
        fw.grounded = true;
        fw.temperatureK = 288.0;
        // Subtle, lacy broken ocean foam rings (dark ocean water shines clearly through gaps; never solid white paint!).
        fw.smokeIntensity = ev.isSlap ? (ev.isMajor ? (0.42 + 0.14 * u0) : (0.28 + 0.10 * u0))
                                      : (0.26 + 0.14 * u0);
        fw.maxAgeS = 6.5 + 3.5 * u1;
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
        std::uint8_t flags = ipc::packDebrisPartTagV1(b.partTag);
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
        if (!isSolidRigidBody(b.archetype)) continue;
        sum += b.massKg;
    }
    return sum;
}

ms::damage::V3 DebrisField::totalMomentumEcef(std::uint32_t ownerVehicleId) const noexcept {
    V3 sum{};
    for (const auto& b : bodies_) {
        if (!b.alive) continue;
        if (ownerVehicleId && b.ownerVehicleId != ownerVehicleId) continue;
        if (!isSolidRigidBody(b.archetype)) continue;
        sum += v3(b.velocityEcef) * b.massKg;
    }
    return sum;
}

} // namespace fsim::session
