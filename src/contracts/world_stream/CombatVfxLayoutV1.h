#pragma once

// Additive combat VFX wire contract (VIS-M13): live ballistic gun projectiles,
// active countermeasure decoys (flares & chaff), and discrete authoritative
// combat VFX events (muzzle flash, bullet airframe/surface impact, detonation
// shockwave, countermeasure ejection).
// WorldLayout v9, VehicleState, VehicleSample, fsrec v8, and DebrisLayoutV1
// remain byte-for-byte frozen; live Worlds publish this via a separate
// triple-buffered seqlock shared-memory segment and lazy companion .vfx
// recording sidecar.

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace fsim::ipc {

inline constexpr std::uint32_t kCombatVfxMagicV1 = 0x56465831u; // "VFX1"
inline constexpr std::uint32_t kCombatVfxVersionV1 = 1u;
inline constexpr char kCombatVfxRecordMagicV1[6] = {'F', 'S', 'V', 'F', 'X', '1'};

inline constexpr std::uint32_t kMaxProjectilesV1 = 512u;
inline constexpr std::uint32_t kMaxDecoysV1 = 256u;
inline constexpr std::uint32_t kMaxCombatVfxEventsV1 = 128u;

inline constexpr std::uint8_t kProjectileFlagTracer = 1u << 0;
inline constexpr std::uint8_t kProjectileFlagHe = 1u << 1;

enum class DecoyKindV1 : std::uint8_t {
    Chaff = 0,
    Flare = 1,
};

enum class CombatVfxEventKindV1 : std::uint8_t {
    None = 0,
    MuzzleFlash = 1,         ///< Gun burst muzzle flash: dirEcef = barrel axis, magnitude = calibre m, param = rounds
    BulletAirframeHit = 2,   ///< Ballistic hit on aircraft skin: dirEcef = incoming axis, magnitude = energy J, param = explosiveKg
    BulletSurfaceImpact = 3, ///< Ballistic ground/water hit: dirEcef = surface normal, magnitude = energy J, flags = SurfaceKindV1
    DetonationShockwave = 4, ///< Warhead / catastrophic blast wave: magnitude = TNT kg, param = blast radius m
    CountermeasureEject = 5, ///< Dispenser ejection pop: dirEcef = ejection axis, flags = DecoyKindV1
};

/// Live ballistic gun projectile state (80 bytes, align 8, zero implicit padding).
struct ProjectileWireV1 {
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    std::uint32_t id = 0;
    std::uint32_t shooterVehicleId = 0;
    float calibreM = 0.02f;
    float ageS = 0.0f;
    float maxAgeS = 8.0f;
    float explosiveKg = 0.0f;
    std::uint8_t flags = 0; ///< kProjectileFlag*
    std::uint8_t reserved[7] = {};
};

/// Live countermeasure flare or chaff cloud state (80 bytes, align 8, zero implicit padding).
struct DecoyWireV1 {
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    std::uint32_t id = 0;
    std::uint32_t ownerVehicleId = 0;
    float ageS = 0.0f;
    float lifeS = 4.0f;
    float intensityOrSigma = 0.0f; ///< Flare radiant intensity (W/sr) or Chaff RCS (m^2)
    float bloomRadiusM = 0.5f;     ///< Flare core scale or physical chaff dipole cloud radius (m)
    std::uint8_t kind = 0;         ///< DecoyKindV1
    std::uint8_t reserved[7] = {};
};

/// Authoritative discrete combat VFX event (80 bytes, align 8, zero implicit padding).
struct CombatVfxEventWireV1 {
    double positionEcef[3] = {};
    double velocityEcef[3] = {};
    float dirEcef[3] = {};
    float magnitude = 0.0f;
    float param = 0.0f;
    std::uint32_t serial = 0;
    std::uint32_t sourceVehicleId = 0;
    std::uint8_t kind = 0;  ///< CombatVfxEventKindV1
    std::uint8_t flags = 0; ///< SurfaceKindV1 / DecoyKindV1
    std::uint8_t reserved[2] = {};
};

struct alignas(64) CombatVfxHeaderV1 {
    std::uint32_t magic = 0;
    std::uint32_t version = kCombatVfxVersionV1;
    std::uint32_t projectileCapacity = 0;
    std::uint32_t decoyCapacity = 0;
    std::uint32_t eventCapacity = 0;
    std::uint32_t incarnation = 0;
    std::uint64_t publisherPid = 0;
    std::atomic<std::uint32_t> alive{0};
};

struct alignas(64) CombatVfxSlotV1 {
    std::atomic<std::uint32_t> seq{0};
    std::uint32_t projectileCount = 0;
    std::uint32_t decoyCount = 0;
    std::uint32_t eventCount = 0;
    std::uint64_t sequence = 0;
    double simTime = 0.0;
};

static_assert(std::is_trivially_copyable_v<ProjectileWireV1>, "projectile wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<DecoyWireV1>, "decoy wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<CombatVfxEventWireV1>, "combat VFX event wire must be trivially copyable");
static_assert(sizeof(ProjectileWireV1) == 80 && alignof(ProjectileWireV1) == 8, "ProjectileWireV1 stride is frozen");
static_assert(sizeof(DecoyWireV1) == 80 && alignof(DecoyWireV1) == 8, "DecoyWireV1 stride is frozen");
static_assert(sizeof(CombatVfxEventWireV1) == 80 && alignof(CombatVfxEventWireV1) == 8, "CombatVfxEventWireV1 stride is frozen");

inline bool validProjectileWireV1(const ProjectileWireV1& p) noexcept {
    if (!p.id || (p.flags & ~static_cast<std::uint8_t>(kProjectileFlagTracer | kProjectileFlagHe)) != 0u) return false;
    for (std::uint8_t r : p.reserved) if (r != 0u) return false;
    for (double v : p.positionEcef) if (!std::isfinite(v)) return false;
    for (double v : p.velocityEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(p.calibreM) && p.calibreM > 0.0f && std::isfinite(p.ageS) && p.ageS >= 0.0f &&
           std::isfinite(p.maxAgeS) && p.maxAgeS > 0.0f && std::isfinite(p.explosiveKg) && p.explosiveKg >= 0.0f;
}

inline bool validDecoyWireV1(const DecoyWireV1& d) noexcept {
    if (!d.id || d.kind > static_cast<std::uint8_t>(DecoyKindV1::Flare)) return false;
    for (std::uint8_t r : d.reserved) if (r != 0u) return false;
    for (double v : d.positionEcef) if (!std::isfinite(v)) return false;
    for (double v : d.velocityEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(d.ageS) && d.ageS >= 0.0f && std::isfinite(d.lifeS) && d.lifeS > 0.0f &&
           std::isfinite(d.intensityOrSigma) && d.intensityOrSigma >= 0.0f &&
           std::isfinite(d.bloomRadiusM) && d.bloomRadiusM > 0.0f;
}

inline bool validCombatVfxEventV1(const CombatVfxEventWireV1& e) noexcept {
    if (!e.serial || e.kind == 0u || e.kind > static_cast<std::uint8_t>(CombatVfxEventKindV1::CountermeasureEject) ||
        e.reserved[0] != 0u || e.reserved[1] != 0u)
        return false;
    for (double v : e.positionEcef) if (!std::isfinite(v)) return false;
    for (double v : e.velocityEcef) if (!std::isfinite(v)) return false;
    for (float v : e.dirEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(e.magnitude) && e.magnitude >= 0.0f && std::isfinite(e.param) && e.param >= 0.0f;
}

} // namespace fsim::ipc
