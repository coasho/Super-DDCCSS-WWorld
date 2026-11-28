#pragma once

// Additive Tactical, EMSO & Extended Propulsion wire contract (VIS-M14):
// strictly read-only native SDK presentation snapshots for 3-D radar scan/STT
// volumes, EA jamming lobes & radar burn-through pockets, ESM DF rays & 3-D
// CRLB geolocation ellipsoids, Link 16 / C2 / Mode 5 IFF / PNT drift pulses,
// flyout-derived 5-D launchZoneV2 DLZ/WEZ + gun LCOS + Mk 82 CCIP cues, SAR /
// weather radar ground swaths, and B-57/B-59 8-engine & rotorcraft telemetry.
// WorldLayout v9, VehicleState, VehicleSample, fsrec v8, DebrisLayoutV1, and
// CombatVfxLayoutV1 remain byte-for-byte frozen; published over a separate
// wait-free triple-buffered seqlock shared-memory segment (`fsim.tact.v1.<world>`)
// and lazy companion `<recordPath>.tact` replay sidecar.

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace fsim::ipc {

inline constexpr std::uint32_t kTacticalMagicV1 = 0x54414354u; // "TACT"
inline constexpr std::uint32_t kTacticalVersionV1 = 1u;
inline constexpr char kTacticalRecordMagicV1[6] = {'F', 'S', 'T', 'A', 'C', '1'};

inline constexpr std::uint32_t kMaxTacticalRadarsV1 = 64u;
inline constexpr std::uint32_t kMaxTacticalJammersV1 = 32u;
inline constexpr std::uint32_t kMaxTacticalEsmRaysV1 = 96u;
inline constexpr std::uint32_t kMaxTacticalLinksV1 = 64u;
inline constexpr std::uint32_t kMaxTacticalWeaponsV1 = 32u;
inline constexpr std::uint32_t kMaxTacticalSwathsV1 = 32u;
inline constexpr std::uint32_t kMaxTacticalPropulsionV1 = 32u;
inline constexpr std::uint32_t kTacticalMaxEnginesV1 = 8u;

// TacticalRadarVolumeWireV1::flags
inline constexpr std::uint8_t kTactRadarSearching = 1u << 0;
inline constexpr std::uint8_t kTactRadarRotodome = 1u << 1;
inline constexpr std::uint8_t kTactRadarElectronic = 1u << 2;
inline constexpr std::uint8_t kTactRadarSttLock = 1u << 3;
inline constexpr std::uint8_t kTactRadarIlluminating = 1u << 4;
inline constexpr std::uint8_t kTactRadarJammed = 1u << 5;
inline constexpr std::uint8_t kTactRadarHasTarget = 1u << 6;

// TacticalJammerLobeWireV1::flags
inline constexpr std::uint8_t kTactJamAllRound = 1u << 0;
inline constexpr std::uint8_t kTactJamComms = 1u << 1;

// TacticalEsmRayWireV1::flags
inline constexpr std::uint8_t kTactEsmWarning = 1u << 0;
inline constexpr std::uint8_t kTactEsmHasGeo = 1u << 1;
inline constexpr std::uint8_t kTactEsmComint = 1u << 2;

// TacticalLinkPulseWireV1::kind
enum class TacticalLinkKindV1 : std::uint8_t {
    None = 0,
    Link16Dms = 1,           ///< Modeled DMS / Link 16 datalink transmission pulse
    C2Consent = 2,           ///< Modeled C2 ROLE_QB consent / task transmission
    Mode5IffInterrogate = 3, ///< 1030 MHz Mode 5 IFF interrogation ring
    Mode5IffReply = 4,       ///< 1090 MHz Mode 5 IFF reply pulse
    PntGnssDrift = 5,        ///< PNT horizontal uncertainty ring (param = pos_error_h m; flag bit 0 = jammed)
};
inline constexpr std::uint8_t kTactLinkJammed = 1u << 0;
inline constexpr std::uint8_t kTactLinkGnssLocked = 1u << 1;

// TacticalWeaponCueWireV1::flags
inline constexpr std::uint8_t kTactWeaponHasDlz = 1u << 0;
inline constexpr std::uint8_t kTactWeaponHasLcos = 1u << 1;
inline constexpr std::uint8_t kTactWeaponHasCcip = 1u << 2;
inline constexpr std::uint8_t kTactWeaponInRange = 1u << 3;
inline constexpr std::uint8_t kTactWeaponInNez = 1u << 4;

// TacticalSwathWireV1::kind
enum class TacticalSwathKindV1 : std::uint8_t {
    None = 0,
    SarSpotlight = 1, ///< ASARS-2 / EISS-SAR spotlight collection box (metricDb = NESZ dB)
    WeatherCell = 2,  ///< AN/APN-241 / AN/APS-133 precipitation swath (metricDb = dBZ)
    GroundMap = 3,    ///< AN/APQ-166 / H-6K ground-map terrain clearance swath
};
inline constexpr std::uint8_t kTactSwathJammed = 1u << 0;

// TacticalPropulsionExtWireV1::flags
inline constexpr std::uint8_t kTactPropRotorcraft = 1u << 0;
inline constexpr std::uint8_t kTactPropPropeller = 1u << 1;
inline constexpr std::uint8_t kTactPropOverWater = 1u << 2;

/// Native radar scan volume, active beam, RCS-aware detection envelope & burn-through range (96 B, align 8).
struct TacticalRadarVolumeWireV1 {
    double targetEcef[3] = {};          ///< Designated/STT target position (when kTactRadarHasTarget)
    std::uint32_t slot = 0;             ///< Carrier vehicle slot
    std::uint32_t vehicleId = 0;        ///< Carrier vehicle id
    float azimuthCentreRad = 0.0f;      ///< Scan centre azimuth (body axes, rad)
    float azimuthHalfWidthRad = 0.0f;   ///< Scan half-width (rad; pi for rotodome)
    float elevationCentreRad = 0.0f;    ///< Scan elevation centre (rad)
    float elevationHalfWidthRad = 0.0f; ///< Scan half-height (rad)
    float beamAzimuthRad = 0.0f;        ///< Instantaneous beam azimuth (rad)
    float beamElevationRad = 0.0f;      ///< Instantaneous beam elevation (rad)
    float rangeScaleM = 0.0f;           ///< Active range scale (m)
    float detectionRangeM = 0.0f;       ///< Unjammed nominal detection range for current/reference RCS (m)
    float burnThroughRangeM = 0.0f;     ///< Live jammed burn-through range under (1 + J0/N0)^(-1/4) (m)
    float jammerRatioDb = 0.0f;         ///< Live 10*log10(1 + J0/N0) degradation (dB)
    float targetRangeM = 0.0f;          ///< Slant range to designated target (m)
    std::uint16_t payloadIndex = 0;
    std::uint8_t mode = 0;              ///< ms::AmtiMode
    std::uint8_t flags = 0;             ///< kTactRadar*
    std::uint32_t targetId = 0;         ///< Track/target id (0 none)
    std::uint8_t reserved[12] = {};
};

/// Native EA directional / barrage RF jamming lobe (96 B, align 8).
struct TacticalJammerLobeWireV1 {
    double originEcef[3] = {};    ///< Jammer platform ECEF (m)
    double boresightEcef[3] = {}; ///< Unit boresight direction in ECEF
    std::uint32_t slot = 0;
    std::uint32_t vehicleId = 0;
    float halfAngleRad = 0.0f;    ///< Mainlobe half-angle (rad)
    float rangeM = 0.0f;          ///< Effective jamming cone reach (m)
    float erpDbw = 0.0f;          ///< Effective radiated power (dBW)
    float frequencyGhz = 0.0f;    ///< Center frequency (GHz)
    float bandwidthMhz = 0.0f;    ///< Jamming bandwidth (MHz)
    std::uint8_t flags = 0;       ///< kTactJam*
    std::uint8_t reserved[19] = {};
};

/// Native ESM / RWR bearing-only DF ray & Gauss-Newton Fisher-Information CRLB geolocation (96 B, align 8).
struct TacticalEsmRayWireV1 {
    double originEcef[3] = {}; ///< Observer platform ECEF (m)
    double geoPosEcef[3] = {}; ///< Triangulated 3-D emitter position when kTactEsmHasGeo
    float rayDirEcef[3] = {};  ///< Unit bearing ray direction in ECEF
    float rayLengthM = 0.0f;   ///< Visual ray length (m)
    float crlbCep50M = 0.0f;   ///< Horizontal 50% CEP from FIM CRLB (m, 0 if bearing-only)
    float crlbVertM = 0.0f;    ///< Vertical 1-sigma uncertainty (m)
    float receivedDbm = 0.0f;  ///< Received emitter power (dBm)
    std::uint32_t observerSlot = 0;
    std::uint32_t observerVehicleId = 0;
    std::uint8_t emitterMode = 0; ///< ms::EmitterMode
    std::uint8_t flags = 0;       ///< kTactEsm*
    std::uint8_t reserved[10] = {};
};

/// Native Link 16 / C2 / Mode 5 IFF / PNT drift visualization pulse (80 B, align 8).
struct TacticalLinkPulseWireV1 {
    double fromEcef[3] = {};
    double toEcef[3] = {};
    std::uint32_t fromSlot = 0;
    std::uint32_t toSlot = 0;
    std::uint32_t fromVehicleId = 0;
    std::uint32_t toVehicleId = 0;
    float ageS = 0.0f;      ///< Pulse age for animated packet travel (s)
    float durationS = 1.0f; ///< Total display duration (s)
    float param = 0.0f;     ///< PntGnssDrift: horiz error m; Mode5/Link16: link margin or radius m
    std::uint8_t kind = 0;  ///< TacticalLinkKindV1
    std::uint8_t flags = 0; ///< kTactLink*
    std::uint8_t reserved[2] = {};
};

/// Native 5-D flyout DLZ/WEZ + McCoy gun LCOS lead pipper + wind-compensated Mk 82 CCIP cue (128 B, align 8).
struct TacticalWeaponCueWireV1 {
    double targetEcef[3] = {};     ///< Designated air target position (ECEF, m)
    double lcosPipperEcef[3] = {}; ///< Native ballistic gun lead aimpoint at target slant range (ECEF, m)
    double ccipImpactEcef[3] = {}; ///< Native wind-compensated bomb impact point on terrain (ECEF, m)
    std::uint32_t slot = 0;
    std::uint32_t vehicleId = 0;
    float rAeroM = 0.0f;           ///< Aerodynamic max range (m)
    float rMaxM = 0.0f;            ///< Flyout R_max (m)
    float rNeM = 0.0f;             ///< Flyout R_ne no-escape range (m)
    float rMinM = 0.0f;            ///< Flyout R_min (m)
    float gunMuzzleSpeedMs = 0.0f; ///< Active gun muzzle speed (m/s)
    float gunDragK = 0.0f;         ///< Sea-level ballistic retardation k (1/m)
    char storeName[16] = {};       ///< Active missile/bomb store mnemonic
    std::uint8_t flags = 0;        ///< kTactWeapon*
    std::uint8_t reserved[7] = {};
};

/// Native SAR spotlight footprint or weather radar reflectivity cell projected onto ground (96 B, align 8).
struct TacticalSwathWireV1 {
    double centerEcef[3] = {};    ///< Ground-projected center (ECEF, m)
    float axisEastEcef[3] = {};   ///< Unit swath width axis (ECEF)
    float axisNorthEcef[3] = {};  ///< Unit swath length axis (ECEF)
    float halfWidthM = 0.0f;      ///< Half-width (m)
    float halfLengthM = 0.0f;     ///< Half-length (m)
    float metricDb = 0.0f;        ///< SAR NESZ (dB) or Weather reflectivity (dBZ)
    float resolutionM = 0.0f;     ///< SAR resolution (m) or cell altitude (m)
    std::uint32_t slot = 0;
    std::uint32_t vehicleId = 0;
    std::uint8_t kind = 0;        ///< TacticalSwathKindV1
    std::uint8_t flags = 0;       ///< kTactSwath*
    std::uint8_t reserved[22] = {};
};

/// Extended 8-engine & rotorcraft propulsion telemetry (B-57 & B-59, zero break to VehicleState, 192 B, align 8).
struct alignas(8) TacticalPropulsionExtWireV1 {
    std::uint32_t slot = 0;
    std::uint32_t vehicleId = 0;
    float throttle[kTacticalMaxEnginesV1] = {};
    float thrustN[kTacticalMaxEnginesV1] = {};
    float engineN2[kTacticalMaxEnginesV1] = {};
    float afterburner[kTacticalMaxEnginesV1] = {};
    float fuelFlowKgS[kTacticalMaxEnginesV1] = {};
    float rotorRpm = 0.0f;         ///< Main rotor / propeller RPM
    float rotorRadiusM = 0.0f;     ///< Main rotor / propeller radius (m)
    float diskLoadingPa = 0.0f;    ///< Rotor disk loading / thrust density (Pa)
    std::uint8_t engineCount = 0;  ///< Actual native engine count (up to 8)
    std::uint8_t flags = 0;        ///< kTactProp*
    std::uint8_t reserved[10] = {};
};

struct alignas(64) TacticalOverlayHeaderV1 {
    std::uint32_t magic = 0;
    std::uint32_t version = kTacticalVersionV1;
    std::uint32_t radarCapacity = 0;
    std::uint32_t jammerCapacity = 0;
    std::uint32_t esmCapacity = 0;
    std::uint32_t linkCapacity = 0;
    std::uint32_t weaponCapacity = 0;
    std::uint32_t swathCapacity = 0;
    std::uint32_t propulsionCapacity = 0;
    std::uint32_t incarnation = 0;
    std::uint64_t publisherPid = 0;
    std::atomic<std::uint32_t> alive{0};
};

struct alignas(64) TacticalOverlaySlotV1 {
    std::atomic<std::uint32_t> seq{0};
    std::uint32_t radarCount = 0;
    std::uint32_t jammerCount = 0;
    std::uint32_t esmCount = 0;
    std::uint32_t linkCount = 0;
    std::uint32_t weaponCount = 0;
    std::uint32_t swathCount = 0;
    std::uint32_t propulsionCount = 0;
    std::uint64_t sequence = 0;
    double simTime = 0.0;
};

static_assert(std::is_trivially_copyable_v<TacticalRadarVolumeWireV1>, "radar wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<TacticalJammerLobeWireV1>, "jammer wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<TacticalEsmRayWireV1>, "esm wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<TacticalLinkPulseWireV1>, "link wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<TacticalWeaponCueWireV1>, "weapon cue wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<TacticalSwathWireV1>, "swath wire must be trivially copyable");
static_assert(std::is_trivially_copyable_v<TacticalPropulsionExtWireV1>, "propulsion ext wire must be trivially copyable");

static_assert(sizeof(TacticalRadarVolumeWireV1) == 96 && alignof(TacticalRadarVolumeWireV1) == 8, "TacticalRadarVolumeWireV1 stride frozen");
static_assert(sizeof(TacticalJammerLobeWireV1) == 96 && alignof(TacticalJammerLobeWireV1) == 8, "TacticalJammerLobeWireV1 stride frozen");
static_assert(sizeof(TacticalEsmRayWireV1) == 96 && alignof(TacticalEsmRayWireV1) == 8, "TacticalEsmRayWireV1 stride frozen");
static_assert(sizeof(TacticalLinkPulseWireV1) == 80 && alignof(TacticalLinkPulseWireV1) == 8, "TacticalLinkPulseWireV1 stride frozen");
static_assert(sizeof(TacticalWeaponCueWireV1) == 128 && alignof(TacticalWeaponCueWireV1) == 8, "TacticalWeaponCueWireV1 stride frozen");
static_assert(sizeof(TacticalSwathWireV1) == 96 && alignof(TacticalSwathWireV1) == 8, "TacticalSwathWireV1 stride frozen");
static_assert(sizeof(TacticalPropulsionExtWireV1) == 192 && alignof(TacticalPropulsionExtWireV1) == 8, "TacticalPropulsionExtWireV1 stride frozen");

inline bool validTacticalRadarV1(const TacticalRadarVolumeWireV1& r) noexcept {
    if (!r.vehicleId) return false;
    for (std::uint8_t b : r.reserved) if (b != 0u) return false;
    for (double v : r.targetEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(r.azimuthCentreRad) && std::isfinite(r.azimuthHalfWidthRad) && r.azimuthHalfWidthRad >= 0.0f &&
           std::isfinite(r.elevationCentreRad) && std::isfinite(r.elevationHalfWidthRad) && r.elevationHalfWidthRad >= 0.0f &&
           std::isfinite(r.beamAzimuthRad) && std::isfinite(r.beamElevationRad) &&
           std::isfinite(r.rangeScaleM) && r.rangeScaleM >= 0.0f &&
           std::isfinite(r.detectionRangeM) && r.detectionRangeM >= 0.0f &&
           std::isfinite(r.burnThroughRangeM) && r.burnThroughRangeM >= 0.0f &&
           std::isfinite(r.jammerRatioDb) && std::isfinite(r.targetRangeM) && r.targetRangeM >= 0.0f;
}

inline bool validTacticalJammerV1(const TacticalJammerLobeWireV1& j) noexcept {
    if (!j.vehicleId) return false;
    for (std::uint8_t b : j.reserved) if (b != 0u) return false;
    for (double v : j.originEcef) if (!std::isfinite(v)) return false;
    for (double v : j.boresightEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(j.halfAngleRad) && j.halfAngleRad >= 0.0f &&
           std::isfinite(j.rangeM) && j.rangeM > 0.0f &&
           std::isfinite(j.erpDbw) && std::isfinite(j.frequencyGhz) && std::isfinite(j.bandwidthMhz);
}

inline bool validTacticalEsmRayV1(const TacticalEsmRayWireV1& e) noexcept {
    if (!e.observerVehicleId) return false;
    for (std::uint8_t b : e.reserved) if (b != 0u) return false;
    for (double v : e.originEcef) if (!std::isfinite(v)) return false;
    for (double v : e.geoPosEcef) if (!std::isfinite(v)) return false;
    for (float v : e.rayDirEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(e.rayLengthM) && e.rayLengthM > 0.0f &&
           std::isfinite(e.crlbCep50M) && e.crlbCep50M >= 0.0f &&
           std::isfinite(e.crlbVertM) && e.crlbVertM >= 0.0f && std::isfinite(e.receivedDbm);
}

inline bool validTacticalLinkPulseV1(const TacticalLinkPulseWireV1& l) noexcept {
    if (!l.fromVehicleId || l.kind == 0u || l.kind > static_cast<std::uint8_t>(TacticalLinkKindV1::PntGnssDrift)) return false;
    for (std::uint8_t b : l.reserved) if (b != 0u) return false;
    for (double v : l.fromEcef) if (!std::isfinite(v)) return false;
    for (double v : l.toEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(l.ageS) && l.ageS >= 0.0f && std::isfinite(l.durationS) && l.durationS > 0.0f && std::isfinite(l.param);
}

inline bool validTacticalWeaponCueV1(const TacticalWeaponCueWireV1& w) noexcept {
    if (!w.vehicleId || w.flags == 0u) return false;
    for (std::uint8_t b : w.reserved) if (b != 0u) return false;
    for (double v : w.targetEcef) if (!std::isfinite(v)) return false;
    for (double v : w.lcosPipperEcef) if (!std::isfinite(v)) return false;
    for (double v : w.ccipImpactEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(w.rAeroM) && w.rAeroM >= 0.0f && std::isfinite(w.rMaxM) && w.rMaxM >= 0.0f &&
           std::isfinite(w.rNeM) && w.rNeM >= 0.0f && std::isfinite(w.rMinM) && w.rMinM >= 0.0f &&
           std::isfinite(w.gunMuzzleSpeedMs) && w.gunMuzzleSpeedMs >= 0.0f &&
           std::isfinite(w.gunDragK) && w.gunDragK >= 0.0f;
}

inline bool validTacticalSwathV1(const TacticalSwathWireV1& s) noexcept {
    if (!s.vehicleId || s.kind == 0u || s.kind > static_cast<std::uint8_t>(TacticalSwathKindV1::GroundMap)) return false;
    for (std::uint8_t b : s.reserved) if (b != 0u) return false;
    for (double v : s.centerEcef) if (!std::isfinite(v)) return false;
    for (float v : s.axisEastEcef) if (!std::isfinite(v)) return false;
    for (float v : s.axisNorthEcef) if (!std::isfinite(v)) return false;
    return std::isfinite(s.halfWidthM) && s.halfWidthM > 0.0f &&
           std::isfinite(s.halfLengthM) && s.halfLengthM > 0.0f &&
           std::isfinite(s.metricDb) && std::isfinite(s.resolutionM) && s.resolutionM >= 0.0f;
}

inline bool validTacticalPropulsionExtV1(const TacticalPropulsionExtWireV1& p) noexcept {
    if (!p.vehicleId || p.engineCount > kTacticalMaxEnginesV1) return false;
    for (std::uint8_t b : p.reserved) if (b != 0u) return false;
    for (std::uint32_t i = 0; i < kTacticalMaxEnginesV1; ++i) {
        if (!std::isfinite(p.throttle[i]) || !std::isfinite(p.thrustN[i]) || !std::isfinite(p.engineN2[i]) ||
            !std::isfinite(p.afterburner[i]) || !std::isfinite(p.fuelFlowKgS[i]))
            return false;
    }
    return std::isfinite(p.rotorRpm) && p.rotorRpm >= 0.0f &&
           std::isfinite(p.rotorRadiusM) && p.rotorRadiusM >= 0.0f &&
           std::isfinite(p.diskLoadingPa) && p.diskLoadingPa >= 0.0f;
}

} // namespace fsim::ipc
