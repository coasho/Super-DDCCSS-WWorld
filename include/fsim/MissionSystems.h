#pragma once

// Mission Systems (ADR-30, docs/mission-systems.md): an aircraft's payloads - its
// radars, receivers, sensors, store stations, jammers, datalinks - and the
// capabilities they give, commanded through the SDK as the flight capabilities
// are (A-GRA ASK 6.0a's Mission Systems interface). Each payload is a real one,
// from its public specification: its card (assets/payloads/<card>.json) and
// the aircraft's fit (aircraft/<type>/payloads.json).

#include "fsim/Capability.h"
#include "fsim/Export.h"

#include <cstdint>
#include <limits>
#include <string>

namespace fsim::ms {

/// What a payload is (its card's family).
enum class Family : std::uint8_t {
    AirRadar,         ///< a fire-control or surveillance radar (A-GRA's AMTI)
    WeatherRadar,
    Sar,              ///< synthetic aperture radar imaging
    Smti,             ///< surface moving-target indication
    Esm,              ///< electronic support: emitter detection and location (and radar warning)
    PassiveOptical,   ///< an electro-optical or infrared sensor: a targeting pod, an IRST, a FLIR
    ElectronicAttack, ///< a jammer
    StoreStation,     ///< a weapon or store station and what it carries
    Gun,
    Countermeasures,  ///< chaff and flare dispensers, towed decoys
    CommTerminal,     ///< a radio or a datalink terminal
    Pnt,              ///< navigation: GNSS, inertial
    GroundMapRadar,   ///< a real-beam ground-mapping or terrain-avoidance radar: A-GRA has no capability for it (its status only)
    Count
};
/// "air_radar", "weather_radar", "sar", "smti", "esm", "passive_optical", "electronic_attack",
/// "store_station", "gun", "countermeasures", "comm_terminal", "pnt", "ground_map_radar".
FSIM_API const char* familyName(Family family) noexcept;

/// A payload's power and operating state (A-GRA's SubsystemStateEnum, as a payload's).
enum class PayloadState : std::uint8_t {
    Off,
    Starting, ///< powered, warming up (its card's warm-up time); Standby when done
    Standby,  ///< ready, not operating (a radar that does not emit)
    Operate,  ///< operating: its capabilities can be commanded
    Bit,      ///< running its built-in test; back to the state before it when done
    Failed,   ///< failed (a BIT that failed, or a failure injected)
    Count
};
/// "off", "starting", "standby", "operate", "bit", "failed".
FSIM_API const char* payloadStateName(PayloadState state) noexcept;

/// Why a Mission Systems command was refused, an activity ended, or a
/// capability is unavailable. Names match control::Reason's where they mean
/// the same thing ("not_granted", "not_implemented", ...).
enum class Reason : std::uint16_t {
    None = 0,
    UnknownVehicle,
    UnknownPayload,
    UnknownCapability,
    UnknownActivity,
    InvalidParameter,
    NotSupported,     ///< the aircraft does not carry a payload that gives it
    NotImplemented,   ///< carried, not modelled yet: its support names the stage that builds it
    NotGranted,       ///< ControlMode::Granted, and the caller holds no grant for it
    NotAllowed,
    AuthorityHeld,    ///< another controller holds its grant
    PayloadOff,       ///< unavailable: its payload is off
    PayloadStarting,  ///< unavailable: its payload warms up
    PayloadStandby,   ///< unavailable: its payload is in standby, not operating
    BitInProgress,    ///< unavailable: its payload runs its built-in test
    PayloadFailed,    ///< unavailable: its payload failed
    Restricted,       ///< unavailable: the platform restricts it
    AlreadyInState,   ///< a state command for the state it is in
    ActivityEnded,    ///< an UPDATE or CANCEL of an activity that ended
    // an activity's end
    GoalReached,
    Requested,        ///< canceled on request
    Superseded,       ///< canceled: a newer command of its payload took over
    Revoked,
    Released,
    BitFailed,        ///< a BIT that found a fault
    // stores (MS-7)
    NoStore,            ///< no store of the type asked for is free on the payload (or not as many)
    StoreBusy,          ///< a store asked for is in another strike or jettison
    StoreNotFlyable,    ///< the store's flight model is not built yet
    StationCannotTake,  ///< a load the station cannot take: its classes, capacity or limit
    NotAwaitingConsent, ///< release consent for a strike that is not asking for it
    ConsentDenied,      ///< a strike ended: its release consent refused
    OverMaxWeight,      ///< a load that would take the aircraft past its maximum take-off weight
    Count
};
/// "unknown_payload", "payload_standby", "goal_reached", ...
FSIM_API const char* reasonName(Reason reason) noexcept;
/// In words: "the payload is in standby, not operating", ...
FSIM_API const char* reasonDescription(Reason reason) noexcept;

/// A Mission Systems activity's id: its vehicle in the high 32 bits, a serial
/// with kActivityBit set in the low ones (apart from flight activities' ids).
using ActivityId = std::uint64_t;
inline constexpr std::uint32_t kActivityBit = 0x80000000u;
constexpr bool isMissionActivity(std::uint64_t id) noexcept { return (static_cast<std::uint32_t>(id) & kActivityBit) != 0; }

/// One payload on a vehicle. Its strings live as long as the vehicle.
struct PayloadInfo {
    std::uint16_t index = 0;    ///< the vehicle's payloads, in its fit's order
    const char* name = "";      ///< in the fit: "radar", "station 3", "rwr"
    const char* card = "";      ///< its card's id: "an-apg-68v9"
    const char* designation = ""; ///< "AN/APG-68(V)9"
    const char* summary = "";   ///< one line: what it is
    Family family = Family::AirRadar;
    PayloadState state = PayloadState::Off;
    std::uint16_t count = 1;    ///< how many (a station's stores, a dispenser's cartridges): its fit's
    const char* source = "";    ///< the fit's source for the aircraft carrying it
};

/// What a Mission Systems capability is (A-GRA's capability types, those the
/// MS volume names).
enum class CapabilityType : std::uint8_t {
    Status,           ///< the payload's state, power and built-in test (A-GRA's SubsystemStatus, SubsystemStateCommand, SubsystemBIT)
    Amti,             ///< air moving-target indication: search and track (AMTI_Capability)
    WeatherRadar,
    Sar,
    Smti,
    Esm,
    PassiveOptical,   ///< PO_Capability
    ElectronicAttack, ///< EA_Capability
    Strike,           ///< weapon employment: release, fire, abort, jettison (StrikeCapability, MA_StrikeCapability)
    StoreManagement,  ///< inventory, release consent, launch zones (StoreManagement*, DLZ, WEZ)
    CommTerminal,
    Pnt,
    Count
};
/// The capability's id: "fsim.ms.status", "fsim.ms.amti", "fsim.ms.weather_radar", "fsim.ms.sar",
/// "fsim.ms.smti", "fsim.ms.esm", "fsim.ms.po", "fsim.ms.ea", "fsim.ms.strike", "fsim.ms.store_management",
/// "fsim.ms.comm_terminal", "fsim.ms.pnt".
FSIM_API const char* capabilityTypeId(CapabilityType type) noexcept;

/// One capability of one payload. Its instance id - "<type's id>:<payload name>",
/// "fsim.ms.amti:radar" - addresses it in the control calls (World::requestControl ...).
struct CapabilityInfo {
    std::uint16_t index = 0;     ///< the vehicle's Mission Systems capabilities, in order
    std::uint16_t payload = 0;   ///< the payload that gives it
    CapabilityType type = CapabilityType::Status;
    const char* id = "";         ///< its instance id: "fsim.ms.amti:radar"
    control::Support support = control::Support::NotImplemented; ///< whether it is modelled in this build
    std::uint8_t stage = 0;      ///< NotImplemented, Partial: the ADR-30 stage that builds it (MS-n: n)
    const char* missing = "";    ///< Partial: what is not built yet
};

/// A capability's availability now, as a policy is answered.
struct CapabilityStatus {
    control::Availability availability = control::Availability::Available;
    Reason reason = Reason::None;
    double nextAvailableS = std::numeric_limits<double>::quiet_NaN(); ///< simulation time it is expected back; NaN: not known
};

// --- Commands ----------------------------------------------------------------

/// Power a payload on or off, to standby or to operate (the status
/// capability's; A-GRA's SubsystemStateCommand). Off to Standby or Operate
/// passes through Starting for the card's warm-up time. Completes when the
/// payload is in the state asked for.
struct StateCommand {
    std::uint16_t payload = 0;
    PayloadState state = PayloadState::Standby; ///< Off, Standby or Operate
};

/// Run a payload's built-in test (A-GRA's SubsystemBIT_Command): for its card's
/// BIT time, its capabilities unavailable meanwhile; then back to the state it
/// was in. Completes GoalReached, or Failed BitFailed with the payload Failed.
struct BitCommand {
    std::uint16_t payload = 0;
};

struct CommandOptions {
    control::Source source = control::Source::Policy;
    control::ControllerId controller = 0;
    std::uint64_t commandId = 0; ///< the caller's id (A-GRA's CommandID): echoed, kept with the activity
};

struct CommandResult {
    control::CommandStatus status = control::CommandStatus::Rejected;
    Reason reason = Reason::None;
    ActivityId activity = 0;
    std::uint64_t commandId = 0;
    bool accepted() const noexcept { return status == control::CommandStatus::Accepted; }
};

// --- AMTI: an air search radar (MS-4; src/ms/radar) -----------------------------

/// How the radar searches (its waveform, and what its processing rejects).
enum class AmtiMode : std::uint8_t {
    RangeWhileSearch, ///< medium PRF: every aspect, range and velocity measured; a slow target meets the side lobes' clutter
    VelocitySearch,   ///< high PRF: closing targets clear of the side lobes' clutter band only, at the longest range
    TrackWhileScan,   ///< as range-while-search over a smaller volume (its tracks: MS-5)
    Surveillance,     ///< an airborne early-warning radar's 360 deg search (a rotodome's only mode)
};
/// "range_while_search", "velocity_search", "track_while_scan", "surveillance".
FSIM_API const char* amtiModeName(AmtiMode mode) noexcept;

/// Search a volume (A-GRA's AMTI_Command): the radar payload must be operating.
/// A new one for the same radar supersedes the one it runs; the activity runs
/// until canceled, superseded or the payload stops operating.
struct AmtiCommand {
    std::uint16_t payload = 0;
    AmtiMode mode = AmtiMode::RangeWhileSearch;      ///< Surveillance for a rotodome, one of the others for a fighter's radar
    double azimuthCentreRad = 0.0;                   ///< from the nose, right positive (clamped to the radar's field of regard)
    double azimuthHalfWidthRad = 1.0471975511965976; ///< +-60 deg (a rotodome: always 360 deg)
    double elevationCentreRad = 0.0;                 ///< above the body's x-y plane
    int bars = 2;                                    ///< elevation bars, 1-8
    double rangeScaleM = 150e3;                      ///< targets beyond it are not reported (and the main beam's clutter beyond it not met)
    /// MS-6: the search at the signal level - each dwell through the radar's samples and processing, the ground's
    /// clutter among them - instead of the detection level's closed forms. Medium-PRF modes (not velocity search).
    bool signalLevel = false;
};

/// One detection: a look that crossed the radar's threshold, measured with
/// errors that shrink as its signal-to-noise ratio grows.
struct AmtiDetection {
    std::uint64_t serial = 0;    ///< the radar's count of detections before it (0, 1, 2, ...)
    std::uint16_t payload = 0;
    double timeS = 0.0;          ///< simulation time of the dwell
    std::uint32_t truth = 0;     ///< the entity that made it: the simulation's knowledge, not the radar's (for scoring)
    double rangeM = 0.0;
    double azimuthRad = 0.0;     ///< from the nose, right positive (body axes)
    double elevationRad = 0.0;   ///< above the body's x-y plane
    double rangeRateMs = 0.0;    ///< positive opening
    double snrDb = 0.0;          ///< the dwell's (what the measurement errors scale with)
    double latitudeRad = 0.0, longitudeRad = 0.0, heightM = 0.0; ///< where the measurement puts it
};

/// One of a radar's tracks (MS-5): a target its track processing holds - Stone Soup's GNN tracker, an unscented Kalman
/// filter on a nearly-constant-velocity model, fed a dwell at a time - predicted to the world's time.
struct AmtiTrack {
    std::uint64_t id = 0;          ///< the radar's own numbering
    std::uint16_t payload = 0;
    double timeS = 0.0;            ///< the world's time it is predicted to
    double lastUpdateS = 0.0;      ///< when a detection last updated it
    std::uint32_t updates = 0;     ///< the detections that updated it, ever
    double latitudeRad = 0.0, longitudeRad = 0.0, heightM = 0.0;
    double velocityNedMs[3] = {0, 0, 0};
    double positionSigmaM = 0.0;   ///< the position's uncertainty: the root mean square of its three axes' standard deviations
    double velocitySigmaMs = 0.0;
    double rangeM = 0.0, azimuthRad = 0.0, elevationRad = 0.0, rangeRateMs = 0.0; ///< from the carrier, in its body axes
    std::uint32_t truth = 0;       ///< the entity whose detection last updated it (0: a false alarm) - for scoring, not the radar's knowledge
    double stateEcef[6] = {0, 0, 0, 0, 0, 0}; ///< x, vx, y, vy, z, vz (m, m/s, ECEF)
    double covarianceEcef[36] = {};           ///< its covariance, row-major
};

/// A radar payload's picture: its scan now, its volume and limits.
struct RadarStatus {
    bool searching = false;
    ActivityId activity = 0;     ///< the AMTI activity it runs (0: none)
    AmtiMode mode = AmtiMode::RangeWhileSearch;
    double beamAzimuthRad = 0.0, beamElevationRad = 0.0; ///< where the beam points now (body axes)
    double framePeriodS = 0.0;   ///< a whole scan of its volume
    double azimuthCentreRad = 0.0, azimuthHalfWidthRad = 0.0, elevationCentreRad = 0.0; ///< the volume it searches
    int bars = 0;
    double rangeScaleM = 0.0;
    double azimuthLimitRad = 0.0, elevationLimitRad = 0.0; ///< its field of regard (pi: a rotodome's 360 deg)
    double instrumentedRangeM = 0.0;
    double frequencyHz = 0.0;
    double beamwidthAzimuthRad = 0.0, beamwidthElevationRad = 0.0;
    bool rotodome = false;
    bool electronic = false;     ///< an electronically scanned array
    std::uint64_t dwells = 0;    ///< beam positions looked at, ever
    std::uint64_t detections = 0; ///< detections made, ever (the next one's serial)
    std::uint64_t falseAlarms = 0; ///< of them, false alarms
    std::uint32_t tracks = 0;    ///< confirmed tracks (MS-5)
    std::uint32_t tentativeTracks = 0; ///< tracks not yet confirmed
    bool signalLevel = false;    ///< it searches at the signal level (MS-6)
};

// --- Activities ----------------------------------------------------------------

struct ActivityRecord {
    ActivityId id = 0;
    std::uint16_t capability = 0; ///< index into the vehicle's Mission Systems capabilities
    std::uint16_t payload = 0;
    control::Source source = control::Source::Policy;
    control::ControllerId controller = 0;
    std::uint64_t commandId = 0;
    control::ActivityState state = control::ActivityState::Pending;
    Reason reason = Reason::None;
    double startTime = 0.0;
    double endTime = std::numeric_limits<double>::quiet_NaN();
    double percent = std::numeric_limits<double>::quiet_NaN(); ///< 0..100: a warm-up's, a BIT's
    bool live() const noexcept {
        return state == control::ActivityState::Pending || state == control::ActivityState::Active;
    }
};

// --- Entities: weapons in flight (MS-2) ------------------------------------------

/// What an entity in the world is: an aircraft the SDK commands, or a weapon in
/// flight - released by an aircraft, a player the sensors see and the viewer
/// draws, flown by its own model (its seeker and guidance: MS-7), never by the
/// flight capabilities (those report it `restricted`). No ground entities (D7).
enum class Role : std::uint8_t { Aircraft, Weapon, Debris };
/// "aircraft", "weapon", "debris" (a piece that left a hit aircraft: MS-7g).
FSIM_API const char* roleName(Role role) noexcept;

/// A store released by an aircraft: the platform's release (a scenario's, as
/// failPayload is its contingency), placed at the launcher's station, flying
/// off with the launcher's velocity, attitude and the ejection's push.
struct WeaponLaunch {
    std::string type = "jsbsim:mk82";     ///< its flight model
    std::string name;                     ///< empty: "<launcher>-<model>-<id>"
    double offsetBodyM[3] = {0, 0, 0};    ///< its CG from the launcher's, in the launcher's body axes (x forward, y right, z down)
    double ejectionBodyMs[3] = {0, 0, 0}; ///< added to the launcher's velocity, in its body axes (an ejector rack: z down)
    /// It leaves unarmed and unlit: a motor it has is never started (a jettison; a store that is not employable).
    bool inert = false;
};

/// How a weapon's flight ended.
enum class WeaponEnd : std::uint8_t {
    InFlight, ///< not yet
    Impact,   ///< it struck the terrain (or the sea)
    Removed,  ///< taken out of the world
};
/// "in_flight", "impact", "removed".
FSIM_API const char* weaponEndName(WeaponEnd end) noexcept;

struct WeaponRecord {
    std::uint32_t id = 0;
    std::uint32_t launcher = 0;
    std::string name, type;
    double launchTimeS = 0.0;
    // (MS-7) the store it was, where an aircraft's store management released it; empty where the platform launched it
    std::string store;            ///< its store card's id
    int station = 0;
    ActivityId activity = 0;      ///< the strike or jettison that released it
    std::uint32_t target = 0;     ///< the strike's target (0: none)
    bool jettisoned = false;      ///< released unarmed (a jettison)
    double endTimeS = std::numeric_limits<double>::quiet_NaN();
    WeaponEnd end = WeaponEnd::InFlight;
    double endLatitudeRad = std::numeric_limits<double>::quiet_NaN(); ///< where it ended (in flight: NaN)
    double endLongitudeRad = std::numeric_limits<double>::quiet_NaN();
    double endHeightM = std::numeric_limits<double>::quiet_NaN();
    double endSpeedMs = std::numeric_limits<double>::quiet_NaN();
};

// --- Stores and weapon employment (MS-7) ---------------------------------------
// A-GRA's Store Management and Weapon Employment sequences as SDK calls: the
// inventory (MA_StrikeCapability: each installed store, its station, type and
// carriage), strike (a weapon type, a quantity, a target; release consent;
// releases a ripple interval apart), abort (the strike canceled), jettison.
// A store station payload's capabilities: fsim.ms.store_management (the
// inventory, consent) and fsim.ms.strike (strike, jettison). Its payload
// operating is the master arm: a strike needs it; a standard jettison needs it
// powered (standby or operate); an emergency jettison goes in any state.

/// A store's state (A-GRA's StoreStatus, as the store management reports it).
enum class StoreStatus : std::uint8_t {
    Loaded,     ///< on its station, unpowered (or not powerable)
    PoweringUp, ///< a powerable store warming up for a strike (its card's power_up_s)
    Ready,      ///< powered, ready to release (or an unpowered store chosen for a strike)
    Releasing,  ///< leaving its station this step
    Released,   ///< gone: a weapon in flight (its entity in `weapon`)
    Jettisoned, ///< gone, unarmed
    PoweringDown, ///< a standard jettison: powered down first
    Lost,       ///< gone with the part of its aircraft it was on (MS-7g: a breakup section that left)
};
/// "loaded", "powering_up", "ready", "releasing", "released", "jettisoned", "powering_down".
FSIM_API const char* storeStatusName(StoreStatus status) noexcept;

/// One installed store (A-GRA's MA_StrikeCapability: its type, station, carriage).
struct StoreInfo {
    std::uint16_t index = 0;       ///< the vehicle's stores, station by station (a strike or jettison names them so)
    std::uint16_t payload = 0;     ///< the station payload
    int station = 0;               ///< the aircraft's numbering
    const char* stationName = "";  ///< "left mid"
    const char* store = "";        ///< its card's id: "mk82"
    const char* mnemonic = "";     ///< "Mk 82"
    const char* storeClass = "";   ///< "aam", "bomb", "tank", "pod"
    const char* carriage = "";     ///< how it leaves: "rail" (under its own power) or "ejector" (pushed clear)
    bool powerable = false, jettisonable = true;
    bool flyable = false;          ///< its flight model is built (a store without one cannot be released yet)
    StoreStatus status = StoreStatus::Loaded;
    double massKg = 0.0;           ///< its mass now (a tank's with its fuel left)
    double fuelKg = 0.0;           ///< a tank's fuel left (kg; 0: not a tank)
    std::uint32_t weapon = 0;      ///< its entity once released (0: none)
    /// Its station's weapon-bay doors, 0 shut .. 1 open (0 for a station outside a bay): a bay's store leaves once they
    /// are open, and they shut after it (MS-7e)
    double bayDoors = 0.0;
};

/// Strike (A-GRA's MA_Task Strike, then its TaskCommand): release `quantity` stores of a type at a target. The
/// station payload must be operating (master arm). Powerable stores power up first; then the activity waits for
/// release consent unless the command gives it (`consent`: MA's weapons release authority answering itself); then the
/// stores leave `rippleS` apart, each a weapon entity. Canceling it before its last release aborts the rest.
struct StrikeCommand {
    std::uint16_t payload = 0;     ///< the station payload
    std::string store;             ///< the store type (its card's id); empty: any it carries that can be released
    int quantity = 1;
    std::vector<std::uint16_t> stores; ///< exactly these stores (StoreInfo::index); overrides store and quantity
    std::uint32_t target = 0;      ///< the entity it is for (a guided weapon's, MS-7d); 0: none
    bool consent = false;          ///< release consent given with the command
    double rippleS = 0.25;         ///< between releases
};

/// Where a strike is (A-GRA's MA_StrikeActivity, with its consent request).
enum class StrikePhase : std::uint8_t {
    PoweringUp,      ///< its stores warming up
    AwaitingConsent, ///< asking for release consent (StrikeConsentRequest)
    Releasing,       ///< releasing, a ripple interval apart
    Done,            ///< every store released (or the activity ended)
};
/// "powering_up", "awaiting_consent", "releasing", "done".
FSIM_API const char* strikePhaseName(StrikePhase phase) noexcept;

struct StrikeStatus {
    ActivityId activity = 0;
    StrikePhase phase = StrikePhase::Done;
    bool jettison = false;           ///< a jettison's (it never waits for consent)
    std::uint32_t target = 0;
    std::vector<std::uint16_t> stores; ///< the stores it releases, in order
    std::vector<std::uint32_t> weapons; ///< those released so far, their entities
};

/// Jettison (A-GRA's MA_Task Jettison): release stores unarmed - all that can be (`stores` empty) or those named.
/// Standard: a powered store is powered down first (its card's power_down_s), the station payload powered; emergency:
/// at once, whatever the payload's state.
struct JettisonCommand {
    std::uint16_t payload = 0;
    std::vector<std::uint16_t> stores; ///< StoreInfo::index; empty: every jettisonable store on the payload
    bool emergency = false;
};

// --- The radio path (MS-2; src/ms/rf) ----------------------------------------------

/// The radio path between two entities, as the payloads see it: ITU-R P.528
/// (NTIA's implementation) over the smooth earth, P.526's delta-Bullington for
/// the terrain under it, P.838 for rain; or the ITM over the terrain, asked for.
struct RadioPathOptions {
    double frequencyHz = 10e9;
    int polarization = 0;         ///< 0 horizontal, 1 vertical
    double timePercent = 50.0;    ///< the loss not exceeded this percentage of the time (median: 50)
    double rainRateMmH = 0.0;     ///< rain along the path below the rain height
    double rainHeightM = 3360.0;  ///< P.839's: the 0 deg C isotherm plus 360 m
    bool itm = false;             ///< the ITM over the terrain where it applies (both ends 0.5-3000 m above it, 20 MHz-20 GHz)
};

struct RadioPath {
    int code = 0;                 ///< the model's: 0 success; P.528 11 or the ITM 1, with warnings; else an invalid input
    bool viaItm = false;
    int mode = 1;                 ///< 1 line of sight, 2 diffraction, 3 troposcatter (the smooth earth's, or the ITM's)
    double groundRangeM = 0.0, slantRangeM = 0.0;
    double elevationRad[2] = {0, 0}; ///< at each end towards the other (local level)
    double azimuthRad[2] = {0, 0};   ///< clockwise from north
    double clearanceM = std::numeric_limits<double>::infinity(); ///< the straight ray's least height above the terrain (4/3 earth)
    bool terrainMasked = false;
    double freeSpaceDb = 0.0, basicDb = 0.0, gasDb = 0.0, terrainDb = 0.0, rainDb = 0.0, totalDb = 0.0;
};

// --- Radar cross section (MS-3; tools/rcs, src/ms/rcs) -----------------------------

/// A radar looking at an entity: its frequency and its linear polarisations, as angles from the vertical (the plane
/// through the line of sight and the local vertical at the target) towards the horizontal: 0 vertical, pi/2 horizontal.
struct RcsQuery {
    double frequencyHz = 9.5e9;
    double transmitPolarizationRad = 0.0;
    double receivePolarizationRad = std::numeric_limits<double>::quiet_NaN(); ///< NaN: the transmit polarisation's
};

/// What the radar sees: the target type's table (shooting and bouncing rays over its model, MS-3) at the aspect it is
/// seen from - the mean over that aspect bin, and how much the RCS fluctuates in it (E[sigma^2]/E[sigma]^2: 1 steady,
/// 2 Rayleigh - Swerling 1/2).
struct RadarCrossSection {
    bool modelled = false; ///< false: no table for the target's type (sigma NaN)
    double sigmaM2 = std::numeric_limits<double>::quiet_NaN();
    double fluctuation = std::numeric_limits<double>::quiet_NaN();
    double aspectAzimuthRad = 0.0;   ///< the radar's direction in the target's body axes: from its nose towards its right wing
    double aspectElevationRad = 0.0; ///< above its body's x-y plane
};

} // namespace fsim::ms
