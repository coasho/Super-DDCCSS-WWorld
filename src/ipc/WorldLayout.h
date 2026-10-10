#pragma once

// Shared-memory layout of a published world (design 9.7, ADR-19). Plain
// data with atomics only where the seqlock/sequence protocol needs them; the
// same header is compiled into fsim.dll (publisher) and the viewer (mirror).
// The layout only grows within kLayoutVersion.

#include "sim/ControlInputs.h"
#include "sim/EnvironmentState.h"
#include "sim/VehicleState.h"

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace fsim::ipc {

constexpr std::uint32_t kMagic = 0x4D495346u; // "FSIM"
constexpr std::uint32_t kLayoutVersion = 7; // 2: VehicleState grew (engines' and moving parts' state); 3: Mission Systems records; 4: their tracks;
                                            // 5: a radar's range-Doppler map; 6: the stores; 7: each store's card (the viewer draws it)
constexpr std::uint32_t kNameLength = 64;
constexpr std::uint32_t kTypeLength = 64;
constexpr std::uint32_t kPathLength = 256;
constexpr std::uint32_t kSlots = 3;

constexpr const char* kWorldPrefix = "fsim.world.";
constexpr const char* kRegistryName = "fsim.registry";

static_assert(std::atomic<std::uint64_t>::is_always_lock_free, "shared-memory sequence counters must be lock-free");
static_assert(std::atomic<std::uint32_t>::is_always_lock_free, "shared-memory seqlocks must be lock-free");

/// Seqlock protocol: the writer increments `seq` to odd, writes, increments to
/// even; a reader copies while `seq` is even and unchanged across the copy.
using SeqLock = std::atomic<std::uint32_t>;

/// Vehicle table row: identity and configuration; changes rarely.
struct VehicleRecord {
    SeqLock seq{0};
    std::uint32_t id = 0;                ///< world-unique id (never reused within a world)
    std::uint64_t generation = 0;        ///< bumped on create / reset / remove
    std::uint8_t alive = 0;
    std::uint8_t controlLevel = 0;       ///< control::Level of the active command
    std::uint8_t reserved[6] = {0, 0, 0, 0, 0, 0};
    char name[kNameLength] = {};
    char type[kTypeLength] = {};         ///< e.g. "jsbsim:c172x"
    char model[kPathLength] = {};        ///< optional visual override (glTF path)
    double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
};

/// Snapshot of every vehicle at one instant; triple buffered.
struct StateSlotHeader {
    SeqLock seq{0};
    std::uint32_t reserved = 0;
    std::uint64_t sequence = 0;          ///< publish counter (monotonic across slots)
    double simTime = 0.0;
    std::int64_t wallNs = 0;             ///< steady clock at publish, system-wide on Windows (QPC)
};

struct VehicleSample {
    sim::VehicleState state;
    sim::ControlInputs inputs;
    double derived[8] = {0, 0, 0, 0, 0, 0, 0, 0}; ///< cascade summary (attitude/velocity targets), level-dependent
};

struct alignas(64) WorldHeader {
    std::uint32_t magic = 0;
    std::uint32_t layoutVersion = 0;
    std::uint32_t headerSize = 0;
    std::uint32_t capacity = 0;          ///< vehicle slots
    std::uint64_t publisherPid = 0;
    char name[kNameLength] = {};
    double dt = 0.0;
    std::int32_t frameSkip = 1;
    /// A request that viewers follow a vehicle (World::focusViewer): bumped by each one, focusVehicle set first. In what was
    /// reserved space: no field moves, and a viewer from before it never looks.
    std::atomic<std::uint32_t> focusSequence{0};
    std::uint64_t tableOffset = 0;       ///< VehicleRecord[capacity]
    std::uint64_t slotOffset = 0;        ///< StateSlotHeader + VehicleSample[capacity], x kSlots
    std::uint64_t slotStride = 0;
    std::uint64_t totalSize = 0;

    std::atomic<std::uint32_t> alive{0};             ///< 1 while the publisher exists
    std::atomic<std::uint32_t> readers{0};           ///< attached mirrors (informational)
    std::atomic<std::uint64_t> tableGeneration{0};   ///< bumped whenever any row changes
    std::atomic<std::uint32_t> latestSlot{0};        ///< index of the newest complete slot
    std::atomic<std::uint32_t> focusVehicle{0};      ///< the vehicle id the last request named
    std::atomic<std::uint64_t> publishSequence{0};
    std::atomic<std::int64_t> lastPublishWallNs{0};
    std::atomic<std::uint64_t> vehicleSteps{0};      ///< total FDM vehicle-steps so far
    std::atomic<std::uint64_t> worldSteps{0};        ///< world.step() calls so far

    SeqLock environmentSeq{0};
    /// Which world this is: set anew each time a publisher (re)initialises the segment, never 0 once it has. A world of
    /// the same name made after another closed reuses its segment in place (a viewer still holds it open), with every
    /// counter back at 0: a mirror that sees this change re-attaches instead of waiting for counters it has passed.
    /// In what was reserved space: no field moves.
    std::atomic<std::uint32_t> incarnation{0};
    sim::EnvironmentState environment;
    std::uint64_t missionOffset = 0;     ///< MissionRecord[capacity], missionStride apart (layout 3)
    std::uint64_t missionStride = 0;
};

// --- Mission Systems (ADR-30, MS-4) ---------------------------------------------
// Each vehicle slot's payloads and radar picture, as its Mission Systems host
// has them (the SDK's instances: the viewer draws them, never simulates its
// own). Written by the publisher when they change; seqlocked per slot.

constexpr std::uint32_t kMissionPayloads = 16;
constexpr std::uint32_t kMissionRadars = 2;
constexpr std::uint32_t kMissionDetections = 96; ///< a radar's newest detections
constexpr std::uint32_t kMissionTracks = 48;      ///< a radar's tracks shown (its first, by number)

struct MissionPayload {
    char name[24] = {};          ///< in the fit: "radar"
    char designation[40] = {};   ///< "AN/APG-68(V)9"
    char capabilities[64] = {};  ///< its family's capabilities, those not built yet with their stage: "amti", "esm (MS-9)"
    std::uint8_t family = 0;     ///< ms::Family
    std::uint8_t state = 0;      ///< ms::PayloadState
    std::uint16_t count = 0;
    float percent = 0.0f;        ///< a warm-up's or a BIT's progress (NaN: none)
};

struct MissionDetection {
    double timeS = 0.0;
    float rangeM = 0.0f, azimuthRad = 0.0f, elevationRad = 0.0f, rangeRateMs = 0.0f, snrDb = 0.0f;
    std::uint32_t truth = 0;
};

/// A radar's track (MS-5) as the display draws it: from the carrier now, in its body axes.
struct MissionTrack {
    std::uint32_t id = 0, updates = 0;
    float rangeM = 0.0f, azimuthRad = 0.0f, elevationRad = 0.0f, rangeRateMs = 0.0f;
    float headingRad = 0.0f;     ///< its course relative to the carrier's nose (right positive)
    float speedMs = 0.0f, sigmaM = 0.0f, sinceUpdateS = 0.0f;
    std::uint32_t truth = 0;
    std::uint32_t reserved = 0;
};

struct MissionRadar {
    std::uint16_t payload = 0;
    std::uint8_t searching = 0, mode = 0, rotodome = 0, electronic = 0, bars = 0;
    std::uint8_t signalLevel = 0; ///< it searches at the signal level (MS-6)
    float beamAzimuthRad = 0.0f, beamElevationRad = 0.0f, framePeriodS = 0.0f;
    float azimuthCentreRad = 0.0f, azimuthHalfWidthRad = 0.0f, elevationCentreRad = 0.0f, rangeScaleM = 0.0f;
    float azimuthLimitRad = 0.0f, elevationLimitRad = 0.0f, instrumentedRangeM = 0.0f;
    float beamwidthAzimuthRad = 0.0f, beamwidthElevationRad = 0.0f;
    double frequencyHz = 0.0;
    std::uint64_t dwells = 0, detections = 0; ///< ever
    std::uint32_t recentCount = 0;            ///< of `recent`, oldest first
    std::uint32_t reserved2 = 0;
    MissionDetection recent[kMissionDetections];
    std::uint64_t falseAlarms = 0;
    std::uint32_t trackCount = 0, tentativeTracks = 0; ///< of `tracks`, and those not confirmed yet
    MissionTrack tracks[kMissionTracks];
};

constexpr std::uint32_t kRangeDopplerRows = 128, kRangeDopplerColumns = 64, kRangeDopplerPeaks = 32;

/// A radar's range-Doppler map at the signal level (MS-6c): its last dwell's first interval (its lowest PRF) as the
/// processor had it - range and Doppler folded by that PRF, each shown cell the largest of the gates and filters it
/// holds - and the cells the processor took as peaks.
struct MissionRangeDoppler {
    std::uint8_t radar = 0xFF;               ///< of the record's `radars` (0xFF: none at the signal level)
    std::uint8_t reserved[3] = {};
    std::uint16_t rows = 0, columns = 0;     ///< shown: range up from the first receive gate; Doppler across, closing to the right, zero in the middle
    std::uint16_t rowBin = 1, columnBin = 1; ///< the gates and filters a shown cell holds
    std::uint32_t peakCount = 0;
    float prfHz = 0.0f;
    float firstRangeM = 0.0f, rowM = 0.0f;   ///< the first row's folded range, and a row's height
    float ambiguousRangeM = 0.0f, ambiguousRateMs = 0.0f;
    float mainBeamRateMs = 0.0f, notchMs = 0.0f, altitudeM = 0.0f; ///< the notches (NaN: none)
    std::uint32_t reserved2 = 0;
    double timeS = 0.0;
    std::uint8_t db[kRangeDopplerRows * kRangeDopplerColumns] = {}; ///< [row * columns + column], over the noise, 0.25 dB a step
    std::uint16_t peakRow[kRangeDopplerPeaks] = {}, peakColumn[kRangeDopplerPeaks] = {};
};

constexpr std::uint32_t kMissionStores = 32;

/// An installed store (MS-7): its station, type and state, as the store management has it.
struct MissionStore {
    std::uint8_t station = 0;
    std::uint8_t status = 0;      ///< ms::StoreStatus
    std::uint8_t bayDoors = 0;    ///< its station's weapon-bay doors, 0 shut .. 255 open (MS-7e)
    std::uint8_t reserved = 0;
    char mnemonic[20] = {};       ///< "Mk 82"
    char store[16] = {};          ///< its card's id, "aim-120c": the viewer draws it on its station (aircraft/aim120c's model)
};

/// What happens in an engagement that the viewer draws (MS-7h): the state MS-7d's fuze, MS-7g's kill and MS-9a's
/// chaff write as they happen - recorded ('V' records) and, later, mirrored. Plain data, one layout for every kind.
enum class EngagementKind : std::uint8_t {
    None = 0,
    Detonation = 1, ///< a warhead went off: `slot` the weapon's, `target` its target's slot + 1 (0 none), at
                    ///< `positionEcef` moving at `velocityEcef` (the weapon's, as it went off), `amountKg` its explosive's
                    ///< mass (TNT equivalent), `massKg` the whole warhead's, `name` its store card's id
    Fire = 2,       ///< a vehicle's component caught fire (MS-7g's kill: its fuel holed, its engine failed): `slot` the
                    ///< vehicle's, `name` the component's (its model manifest's `component` lines), `amountKg` the fuel
                    ///< released at once (its fireball), `rateKgS` what leaks and burns on (0: it is out), `massKg`
                    ///< what leaves it each second (kg/s: what does not burn its mist); written again as they change
    // MS-7g's damage (session/WorldDamage.cpp), each `slot` the vehicle's, at its place and velocity:
    EngineOut = 3,   ///< `name` the engine's component, `index` its engine
    ControlLost = 4, ///< `name` the control channel lost
    CrewLost = 5,    ///< `name` the crew's component
    PanelTorn = 6,   ///< the blast tore a skin panel off: `name` its section, `index` the panel (its damage data's)
    Separated = 7,   ///< a breakup section's root failed and it left: `name` the section
    Hit = 8,         ///< the fragments struck it: `target` the weapon's slot + 1 (0: none), `index` how many struck,
                     ///< `name` the weapon's store card's id
};

struct EngagementEvent {
    std::uint8_t kind = 0;        ///< EngagementKind
    std::uint8_t reserved[3] = {};
    std::uint32_t slot = 0;
    std::uint32_t target = 0;
    std::uint32_t index = 0;      ///< a kind's own count or index (EngineOut, PanelTorn, Hit)
    double simTime = 0.0;         ///< when it happened (s)
    double positionEcef[3] = {};  ///< where (m)
    double velocityEcef[3] = {};  ///< m/s
    double amountKg = 0.0;
    double massKg = 0.0;
    double rateKgS = 0.0;
    char name[32] = {};
};

struct MissionRecord {
    SeqLock seq{0};
    std::uint32_t vehicleId = 0;  ///< 0: no vehicle in the slot (or none with payloads)
    std::uint64_t revision = 0;   ///< bumped by each write
    double simTime = 0.0;         ///< when it was written
    std::uint32_t payloadCount = 0, radarCount = 0;
    MissionPayload payloads[kMissionPayloads];
    MissionRadar radars[kMissionRadars];
    MissionRangeDoppler rangeDoppler; ///< the first radar at the signal level's (MS-6c)
    std::uint32_t storeCount = 0;     ///< (MS-7) of `stores`, station by station
    std::uint32_t reserved3 = 0;
    MissionStore stores[kMissionStores];
};

/// Byte layout helpers shared by publisher and mirror.
struct Layout {
    static constexpr std::size_t align(std::size_t n) noexcept { return (n + 63) & ~static_cast<std::size_t>(63); }
    static std::size_t tableOffset() noexcept { return align(sizeof(WorldHeader)); }
    static std::size_t tableSize(std::uint32_t capacity) noexcept { return align(sizeof(VehicleRecord) * capacity); }
    static std::size_t slotStride(std::uint32_t capacity) noexcept {
        return align(sizeof(StateSlotHeader)) + align(sizeof(VehicleSample) * capacity);
    }
    static std::size_t slotOffset(std::uint32_t capacity) noexcept { return tableOffset() + tableSize(capacity); }
    static std::size_t missionOffset(std::uint32_t capacity) noexcept { return slotOffset(capacity) + slotStride(capacity) * kSlots; }
    static std::size_t missionStride() noexcept { return align(sizeof(MissionRecord)); }
    static std::size_t totalSize(std::uint32_t capacity) noexcept { return missionOffset(capacity) + missionStride() * capacity; }
};

/// Registry of live worlds for discovery (a tiny separate mapping).
constexpr std::uint32_t kRegistryEntries = 32;
struct RegistryEntry {
    SeqLock seq{0};
    std::uint32_t reserved = 0;
    std::uint64_t pid = 0;
    std::int64_t createdWallNs = 0;
    char name[kNameLength] = {};
};
struct alignas(64) RegistryHeader {
    std::uint32_t magic = 0;
    std::uint32_t layoutVersion = 0;
    std::atomic<std::uint32_t> lock{0};  ///< spin lock for registration (rare)
    std::uint32_t reserved = 0;
    RegistryEntry entries[kRegistryEntries];
};

} // namespace fsim::ipc
