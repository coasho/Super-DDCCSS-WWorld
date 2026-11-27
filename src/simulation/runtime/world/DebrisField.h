#pragma once

// Lightweight deterministic 6-DOF tumbling rigid-body debris & crash physics pool (VIS-M11).
// Owned by session::World; uses Jolt Physics (MIT, cross-platform deterministic) for real
// 6-DOF terrain-contact rigid-body collision, tumbling, bouncing, rolling, and sliding of
// aircraft structural sections, Voronoi hull shards, and ground-wreck fuselages without
// spawning heavyweight JSBSim child vehicles.

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Rng.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/mission_systems/damage/Damage.h"
#include "simulation/mission_systems/damage/Impact.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace fsim::session {

struct DebrisBody {
    std::uint32_t id = 0;
    std::uint32_t ownerVehicleId = 0;
    ipc::DebrisArchetypeV1 archetype = ipc::DebrisArchetypeV1::PanelShard;
    ipc::DebrisPartTagV1 partTag = ipc::DebrisPartTagV1::Generic;
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    double quatBodyToEcef[4] = {0.0, 0.0, 0.0, 1.0}; ///< unit quaternion [x, y, z, w]
    double omegaBodyRadS[3] = {};
    double massKg = 5.0;
    double areaM2 = 0.25;
    double halfExtentsM[3] = {0.5, 0.4, 0.06};
    double temperatureK = 300.0;
    double fuelRemainingKg = 0.0;
    double burnRateKgS = 0.0;
    double smokeIntensity = 0.0;
    double ageS = 0.0;
    double maxAgeS = 35.0;
    double waveWakeTimerS = 0.0;
    std::uint32_t joltBodyId = 0xFFFFFFFFu;
    bool burning = false;
    bool grounded = false;
    bool inWater = false;
    bool alive = false;
};

struct SectionBreakupSpawn {
    std::uint32_t ownerVehicleId = 0;
    ipc::DebrisPartTagV1 partTag = ipc::DebrisPartTagV1::Generic;
    double rootPositionEcef[3] = {};
    double sectionPositionEcef[3] = {};
    double sectionVelocityEcef[3] = {};
    double rotationBodyToEcef[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double angularRateBodyRadS[3] = {};
    double sectionMassKg = 200.0;
    double fuelKg = 0.0;
    double spanM = 3.0;
    bool isWing = false;
    bool burning = false;
    bool spawnPrimaryChunk = true; ///< true in LightweightDebris mode (replaces jsbsim:<piece>); false spawns fracture shards only
};

struct CatastrophicSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    double rotationBodyToEcef[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double angularRateBodyRadS[3] = {};
    double airframeMassKg = 8000.0;
    double remainingFuelKg = 1000.0;
    double explosionTntKg = 15.0;
    double radiusM = 7.0;
    /// Actual aircraft section offsets in body axes [nose, centre, left-wing, right-wing, tail] (m).
    /// Zero-initialized falls back to representative fighter proportions.
    double sectionOffsetsBodyM[5][3] = {};
    bool hasSectionOffsets = false;
    /// Bitmask of sections already detached earlier in Grade 3 StructuralBreakup (never duplicated!):
    /// bit 0=nose, 1=centre, 2=left wing, 3=right wing, 4=tail.
    std::uint8_t alreadySeparatedMask = 0;
};

struct HitBurstSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    double exitDirEcef[3] = {0.0, 0.0, 1.0};
    int fragmentsStriking = 1;
    bool fiery = false;
};

struct DamageEmberSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    double intensity = 1.0; ///< 0..1 scaled by active fire rate
};

struct ImpactCloudSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double afterVelocityEcef[3] = {};
    double incomingVelocityEcef[3] = {};
    double normalEcef[3] = {0.0, 0.0, 1.0};
    double rotationBodyToEcef[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double sectionOffsetsBodyM[5][3] = {};
    bool hasSectionOffsets = false;
    double massKg = 8000.0;
    double fireballKg = 0.0;
    double poolKg = 0.0;
    ms::damage::SurfaceKind surface = ms::damage::SurfaceKind::Soil;
    bool hullDisintegrated = false;
    /// Bitmask of sections already spawned via separated(): bit 0=nose, 1=centre, 2=left-wing, 3=right-wing, 4=tail.
    std::uint8_t alreadySeparatedMask = 0;
};

struct GroundWreckSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    double rotationBodyToEcef[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double halfExtentsM[3] = {5.5, 1.35, 1.35}; ///< fuselage collider box resting ON TOP OF terrain
    double massKg = 8500.0;
    double bankKickDeg = 18.0; ///< initial asymmetric crash roll/yaw tumble kick
    ms::damage::SurfaceKind surface = ms::damage::SurfaceKind::Soil;
};

struct SlideSparkSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    double normalEcef[3] = {0.0, 0.0, 1.0};
    ms::damage::SurfaceKind surface = ms::damage::SurfaceKind::Soil;
};

class DebrisField {
public:
    static constexpr std::size_t kMaxBodies = ipc::kMaxDebrisV1;

    DebrisField();
    ~DebrisField();
    DebrisField(const DebrisField&) = delete;
    DebrisField& operator=(const DebrisField&) = delete;

    void clear() noexcept;
    std::size_t activeCount() const noexcept { return activeCount_; }
    const std::array<DebrisBody, kMaxBodies>& slots() const noexcept { return bodies_; }

    /// Spawn a departing structural assembly + ragged fracture shards at a section break.
    void spawnSectionBreakup(const SectionBreakupSpawn& in, Rng& rng);

    /// Spawn a catastrophic mid-air disintegration cloud conserving total linear momentum.
    void spawnCatastrophicKill(const CatastrophicSpawn& in, Rng& rng);

    /// Spawn ground-impact debris/dust/sparks or water-impact hydrodynamic geyser, foam wake & skipping wreckage.
    void spawnImpactCloud(const ImpactCloudSpawn& in, Rng& rng);

    /// Spawn continuous scrape sparks, billowing dust clouds, and charred ground skid marks during belly-slide.
    void spawnSlideSparks(const SlideSparkSpawn& in, Rng& rng);

    /// Spawn a localized burst of metallic skin-perforation sparks & tiny aluminium flakes on non-catastrophic hits (Grade 1-3).
    void spawnHitBurst(const HitBurstSpawn& in, Rng& rng);

    /// Spawn periodic glowing embers trailing behind an in-flight burning engine or fuel bay (Grade 2-3).
    void spawnDamageEmbers(const DamageEmberSpawn& in, Rng& rng);

    /// Register a real JoltPhysics 6-DOF dynamic rigid body for a surviving ground-wreck fuselage resting on terrain.
    /// Returns an opaque handle (0 on failure) queried each step via queryGroundWreck().
    std::uint32_t spawnGroundWreck(const GroundWreckSpawn& in, const sim::GroundProvider* ground, Rng& rng);

    /// Query the live JoltPhysics pose & velocity of a registered ground-wreck fuselage in ECEF/body axes.
    bool queryGroundWreck(std::uint32_t handle, double posEcef[3], double velEcef[3], double rotBodyToEcef[9],
                          double eulerRad[3]) const noexcept;

    /// Advance all active rigid-body chunks and ground wrecks deterministically by `dt` seconds via JoltPhysics.
    void step(double dt, const sim::GroundProvider* ground, const sim::EnvironmentState& env, double simTime);

    /// Populate wire output vector for live IPC sidecar and companion .debris recording.
    void fillWire(std::vector<ipc::DebrisBodyWireV1>& out) const;

    /// Total mass and momentum of live bodies belonging to `ownerVehicleId` (conservation verification).
    double totalMassKg(std::uint32_t ownerVehicleId = 0) const noexcept;
    ms::damage::V3 totalMomentumEcef(std::uint32_t ownerVehicleId = 0) const noexcept;

private:
    struct JoltWorld;

    DebrisBody& allocate(ipc::DebrisArchetypeV1 archetype) noexcept;
    void retire(DebrisBody& b) noexcept;
    void ensureJolt(const double originEcef[3], const sim::GroundProvider* ground);
    void registerJoltBody(DebrisBody& b);
    void unregisterJoltBody(DebrisBody& b) noexcept;

    std::array<DebrisBody, kMaxBodies> bodies_{};
    std::size_t activeCount_ = 0;
    std::uint32_t nextId_ = 1;
    std::unique_ptr<JoltWorld> jolt_;
};

} // namespace fsim::session
