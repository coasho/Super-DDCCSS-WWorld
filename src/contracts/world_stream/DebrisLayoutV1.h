#pragma once

// Additive debris and per-vehicle destruction visual wire contract (VIS-M11).
// WorldLayout v9, VehicleState, VehicleSample, and fsrec v8 remain byte-for-byte
// frozen; live Worlds publish this via a separate triple-buffered seqlock
// shared-memory segment and optional companion .debris recording sidecar.

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace fsim::ipc {

inline constexpr std::uint32_t kDebrisMagicV1 = 0x44454231u; // "DEB1"
inline constexpr std::uint32_t kDebrisVersionV1 = 1u;
inline constexpr char kDebrisRecordMagicV1[6] = {'F', 'S', 'D', 'E', 'B', '1'};
inline constexpr std::uint32_t kMaxDebrisV1 = 512u;
inline constexpr std::uint32_t kMaxVehicleDestructionV1 = 256u;
inline constexpr std::uint32_t kMaxSectionScorchV1 = 8u;

inline constexpr std::uint8_t kDebrisFlagBurning = 1u << 0;
inline constexpr std::uint8_t kDebrisFlagGrounded = 1u << 1;
inline constexpr std::uint8_t kDebrisFlagWater = 1u << 2;
inline constexpr std::uint8_t kDebrisFlagSmoking = 1u << 3;

enum class DestructionTierV1 : std::uint8_t {
    Intact = 0,
    Damaged = 1,
    StructuralBreakup = 2,
    CatastrophicKill = 3,
    GroundWreck = 4,
    WaterImpact = 5,
};

enum class DebrisArchetypeV1 : std::uint8_t {
    AirframeHull = 0,
    WingFragment = 1,
    PanelShard = 2,
    BurningChunk = 3,
    SparkEmber = 4,
    SplashPlume = 5,
};

enum class SurfaceKindV1 : std::uint8_t {
    Soil = 0,
    Water = 1,
    Rock = 2,
};

/// Plain fixed-stride 6-DOF tumbling debris chunk state (zero implicit padding).
struct DebrisBodyWireV1 {
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    std::uint32_t id = 0;
    std::uint32_t ownerVehicleId = 0;
    float halfExtentsM[3] = {0.5f, 0.5f, 0.1f};
    float temperatureK = 300.0f;
    float smokeIntensity = 0.0f;
    float ageS = 0.0f;
    float maxAgeS = 30.0f;
    float quatBodyToEcef[4] = {0.0f, 0.0f, 0.0f, 1.0f}; ///< unit quaternion [x, y, z, w]
    std::uint8_t archetype = 0;                         ///< DebrisArchetypeV1
    std::uint8_t flags = 0;                             ///< kDebrisFlag*
    std::uint8_t reserved[2] = {};
};

/// Per-vehicle destruction & procedural scorch state (zero implicit padding).
struct VehicleDestructionWireV1 {
    std::uint64_t generation = 0;
    std::uint32_t slot = 0;
    std::uint32_t vehicleId = 0;
    float overallDamage = 0.0f;
    float scorchIntensity = 0.0f;
    float fuelRemainingKg = 0.0f;
    float sectionDamage[kMaxSectionScorchV1] = {};
    std::uint8_t tier = 0;        ///< DestructionTierV1
    std::uint8_t hullRetired = 0; ///< 1 when catastrophic kill retired intact airframe mesh
    std::uint8_t surfaceKind = 0; ///< SurfaceKindV1
    std::uint8_t sectionCount = 0;
};

struct alignas(64) DebrisHeaderV1 {
    std::uint32_t magic = 0;
    std::uint32_t version = kDebrisVersionV1;
    std::uint32_t debrisCapacity = 0;
    std::uint32_t vehicleCapacity = 0;
    std::uint32_t incarnation = 0;
    std::uint32_t reserved = 0;
    std::uint64_t publisherPid = 0;
    std::atomic<std::uint32_t> alive{0};
};

struct alignas(64) DebrisSlotV1 {
    std::atomic<std::uint32_t> seq{0};
    std::uint32_t debrisCount = 0;
    std::uint32_t vehicleCount = 0;
    std::uint32_t reserved = 0;
    std::uint64_t sequence = 0;
    double simTime = 0.0;
};

static_assert(std::is_trivially_copyable_v<DebrisBodyWireV1>, "debris body wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<VehicleDestructionWireV1>, "vehicle destruction wire must be trivially copyable");
static_assert(sizeof(DebrisBodyWireV1) == 104 && alignof(DebrisBodyWireV1) == 8, "debris body V1 stride is frozen");
static_assert(sizeof(VehicleDestructionWireV1) == 64 && alignof(VehicleDestructionWireV1) == 8, "vehicle destruction V1 stride is frozen");

inline bool validDebrisBodyV1(const DebrisBodyWireV1& b) noexcept {
    if (!b.id || b.archetype > static_cast<std::uint8_t>(DebrisArchetypeV1::SplashPlume) || b.reserved[0] || b.reserved[1])
        return false;
    for (double v : b.positionEcef) if (!std::isfinite(v)) return false;
    for (double v : b.velocityEcef) if (!std::isfinite(v)) return false;
    for (float v : b.halfExtentsM) if (!std::isfinite(v) || !(v > 0.0f)) return false;
    if (!std::isfinite(b.temperatureK) || b.temperatureK < 0.0f || !std::isfinite(b.smokeIntensity) ||
        b.smokeIntensity < 0.0f || !std::isfinite(b.ageS) || b.ageS < 0.0f || !std::isfinite(b.maxAgeS) ||
        !(b.maxAgeS > 0.0f))
        return false;
    double q2 = 0.0;
    for (float q : b.quatBodyToEcef) {
        if (!std::isfinite(q)) return false;
        const double dq = static_cast<double>(q);
        q2 += dq * dq;
    }
    return q2 > 0.5 && q2 < 1.5;
}

inline bool validVehicleDestructionV1(const VehicleDestructionWireV1& v, std::uint32_t capacity) noexcept {
    if (!v.vehicleId || v.slot >= capacity || v.tier > static_cast<std::uint8_t>(DestructionTierV1::WaterImpact) ||
        v.hullRetired > 1u || v.surfaceKind > static_cast<std::uint8_t>(SurfaceKindV1::Rock) ||
        v.sectionCount > kMaxSectionScorchV1)
        return false;
    if (!std::isfinite(v.overallDamage) || v.overallDamage < 0.0f || !std::isfinite(v.scorchIntensity) ||
        v.scorchIntensity < 0.0f || !std::isfinite(v.fuelRemainingKg) || v.fuelRemainingKg < 0.0f)
        return false;
    for (std::uint8_t i = 0; i < v.sectionCount; ++i)
        if (!std::isfinite(v.sectionDamage[i]) || v.sectionDamage[i] < 0.0f) return false;
    return true;
}

} // namespace fsim::ipc
