#pragma once

// Lightweight deterministic 6-DOF tumbling rigid-body debris pool (VIS-M11).
// Owned by session::World; replaces heavyweight JSBSim child vehicles during
// structural breakups, catastrophic mid-air disintegrations (B-58), and ground
// or water impact clouds (B-54, B-55) with fixed-capacity (< 0.05 ms/step)
// deterministic physics and bit-identical replay.

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "fsim/EnvironmentState.h"
#include "fsim/Rng.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/mission_systems/damage/Damage.h"
#include "simulation/mission_systems/damage/Impact.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fsim::session {

struct DebrisBody {
    std::uint32_t id = 0;
    std::uint32_t ownerVehicleId = 0;
    ipc::DebrisArchetypeV1 archetype = ipc::DebrisArchetypeV1::PanelShard;
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
    bool burning = false;
    bool grounded = false;
    bool inWater = false;
    bool alive = false;
};

struct SectionBreakupSpawn {
    std::uint32_t ownerVehicleId = 0;
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
};

struct ImpactCloudSpawn {
    std::uint32_t ownerVehicleId = 0;
    double positionEcef[3] = {};
    double afterVelocityEcef[3] = {};
    double incomingVelocityEcef[3] = {};
    double normalEcef[3] = {0.0, 0.0, 1.0};
    double massKg = 8000.0;
    double fireballKg = 0.0;
    double poolKg = 0.0;
    ms::damage::SurfaceKind surface = ms::damage::SurfaceKind::Soil;
    bool hullDisintegrated = false;
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

    void clear() noexcept;
    std::size_t activeCount() const noexcept { return activeCount_; }
    const std::array<DebrisBody, kMaxBodies>& slots() const noexcept { return bodies_; }

    /// Spawn a departing structural assembly + ragged fracture shards at a section break.
    void spawnSectionBreakup(const SectionBreakupSpawn& in, Rng& rng);

    /// Spawn a catastrophic mid-air disintegration cloud conserving total linear momentum.
    void spawnCatastrophicKill(const CatastrophicSpawn& in, Rng& rng);

    /// Spawn ground-impact debris/sparks or water-impact splash plumes & sinking chunks.
    void spawnImpactCloud(const ImpactCloudSpawn& in, Rng& rng);

    /// Spawn continuous scrape sparks or water spray during a post-impact surface slide.
    void spawnSlideSparks(const SlideSparkSpawn& in, Rng& rng);

    /// Advance all active rigid-body chunks deterministically by `dt` seconds.
    void step(double dt, const sim::GroundProvider* ground, const sim::EnvironmentState& env, double simTime);

    /// Populate wire output vector for live IPC sidecar and companion .debris recording.
    void fillWire(std::vector<ipc::DebrisBodyWireV1>& out) const;

    /// Total mass and momentum of live bodies belonging to `ownerVehicleId` (conservation verification).
    double totalMassKg(std::uint32_t ownerVehicleId = 0) const noexcept;
    ms::damage::V3 totalMomentumEcef(std::uint32_t ownerVehicleId = 0) const noexcept;

private:
    DebrisBody& allocate(ipc::DebrisArchetypeV1 archetype) noexcept;
    void retire(DebrisBody& b) noexcept;

    std::array<DebrisBody, kMaxBodies> bodies_{};
    std::size_t activeCount_ = 0;
    std::uint32_t nextId_ = 1;
};

} // namespace fsim::session
