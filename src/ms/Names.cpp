#include "fsim/MissionSystems.h"

#include <cstddef>

namespace fsim::ms {

const char* familyName(Family family) noexcept {
    static constexpr const char* kNames[] = {"air_radar",         "weather_radar", "sar", "smti",          "esm",           "passive_optical",
                                             "electronic_attack", "store_station", "gun", "countermeasures", "comm_terminal", "pnt",
                                             "ground_map_radar"};
    static_assert(sizeof(kNames) / sizeof(kNames[0]) == static_cast<std::size_t>(Family::Count));
    const auto i = static_cast<std::size_t>(family);
    return i < static_cast<std::size_t>(Family::Count) ? kNames[i] : "";
}

const char* payloadStateName(PayloadState state) noexcept {
    static constexpr const char* kNames[] = {"off", "starting", "standby", "operate", "bit", "failed"};
    static_assert(sizeof(kNames) / sizeof(kNames[0]) == static_cast<std::size_t>(PayloadState::Count));
    const auto i = static_cast<std::size_t>(state);
    return i < static_cast<std::size_t>(PayloadState::Count) ? kNames[i] : "";
}

namespace {

struct ReasonText {
    const char* name;
    const char* description;
};
constexpr ReasonText kReasons[] = {
    {"none", ""},
    {"unknown_vehicle", "no vehicle has that id"},
    {"unknown_payload", "the vehicle has no payload by that index"},
    {"unknown_capability", "no Mission Systems capability by that id"},
    {"unknown_activity", "no activity by that id"},
    {"invalid_parameter", "a parameter is not one the command takes"},
    {"not_supported", "the aircraft carries no payload that gives it"},
    {"not_implemented", "carried, but not modelled in this build: its support names the stage that builds it"},
    {"not_granted", "the vehicle requires a grant for it, and the caller holds none"},
    {"not_allowed", "the policy may not have it"},
    {"authority_held", "another controller holds it"},
    {"payload_off", "the payload is off"},
    {"payload_starting", "the payload is warming up"},
    {"payload_standby", "the payload is in standby, not operating"},
    {"bit_in_progress", "the payload runs its built-in test"},
    {"payload_failed", "the payload has failed"},
    {"restricted", "the platform restricts it"},
    {"already_in_state", "the payload is already in that state"},
    {"activity_ended", "the activity has ended"},
    {"goal_reached", "done"},
    {"requested", "canceled on request"},
    {"superseded", "a newer command of its payload took over"},
    {"revoked", "the platform revoked the grant"},
    {"released", "the controller released its grant"},
    {"bit_failed", "the built-in test found a fault"},
    {"no_store", "no store of that type is free on the payload, or not as many"},
    {"store_busy", "a store asked for is in another strike or jettison"},
    {"store_not_flyable", "the store's flight model is not built yet"},
    {"station_cannot_take", "the station cannot take that load: its store classes, capacity or limit"},
    {"not_awaiting_consent", "the strike is not asking for release consent"},
    {"consent_denied", "release consent was refused"},
    {"over_max_weight", "the load would take the aircraft past its maximum take-off weight"},
    {"unknown_track", "the radar holds no track of that number"},
    {"track_lost", "the radar lost the target it tracked"},
};
static_assert(sizeof(kReasons) / sizeof(kReasons[0]) == static_cast<std::size_t>(Reason::Count));

} // namespace

const char* reasonName(Reason reason) noexcept {
    const auto i = static_cast<std::size_t>(reason);
    return i < static_cast<std::size_t>(Reason::Count) ? kReasons[i].name : "";
}

const char* reasonDescription(Reason reason) noexcept {
    const auto i = static_cast<std::size_t>(reason);
    return i < static_cast<std::size_t>(Reason::Count) ? kReasons[i].description : "";
}

const char* amtiModeName(AmtiMode m) noexcept {
    switch (m) {
    case AmtiMode::RangeWhileSearch: return "range_while_search";
    case AmtiMode::VelocitySearch: return "velocity_search";
    case AmtiMode::TrackWhileScan: return "track_while_scan";
    case AmtiMode::Surveillance: return "surveillance";
    case AmtiMode::SingleTargetTrack: return "single_target_track";
    }
    return "";
}

const char* amtiTaskName(AmtiTask t) noexcept {
    switch (t) {
    case AmtiTask::Volume: return "volume";
    case AmtiTask::Cued: return "cued";
    case AmtiTask::Track: return "track";
    }
    return "";
}

AmtiCommand acmCommand(std::uint16_t payload, AcmPattern pattern) noexcept {
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    AmtiCommand c;
    c.payload = payload;
    c.task = AmtiTask::Cued;
    c.mode = AmtiMode::RangeWhileSearch;
    c.rangeScaleM = 18520.0; // (10 nmi)
    c.bars = 1;
    if (pattern == AcmPattern::VerticalScan) {
        c.azimuthHalfWidthRad = 5.0 * kDeg;
        c.elevationCentreRad = 20.0 * kDeg, c.elevationHalfWidthRad = 30.0 * kDeg; // (-10 to +50 deg)
    } else {
        c.azimuthHalfWidthRad = 0.1 * kDeg; // (one beam position, on the nose)
    }
    return c;
}

const char* storeStatusName(StoreStatus status) noexcept {
    switch (status) {
    case StoreStatus::Loaded: return "loaded";
    case StoreStatus::PoweringUp: return "powering_up";
    case StoreStatus::Ready: return "ready";
    case StoreStatus::Releasing: return "releasing";
    case StoreStatus::Released: return "released";
    case StoreStatus::Jettisoned: return "jettisoned";
    case StoreStatus::PoweringDown: return "powering_down";
    case StoreStatus::Lost: return "lost";
    }
    return "";
}

const char* strikePhaseName(StrikePhase phase) noexcept {
    switch (phase) {
    case StrikePhase::PoweringUp: return "powering_up";
    case StrikePhase::AwaitingConsent: return "awaiting_consent";
    case StrikePhase::Releasing: return "releasing";
    case StrikePhase::Done: return "done";
    }
    return "";
}

const char* roleName(Role role) noexcept { return role == Role::Weapon ? "weapon" : role == Role::Debris ? "debris" : "aircraft"; }

const char* weaponEndName(WeaponEnd end) noexcept {
    switch (end) {
    case WeaponEnd::InFlight: return "in_flight";
    case WeaponEnd::Impact: return "impact";
    case WeaponEnd::Removed: return "removed";
    }
    return "";
}

const char* capabilityTypeId(CapabilityType type) noexcept {
    static constexpr const char* kIds[] = {"fsim.ms.status", "fsim.ms.amti", "fsim.ms.weather_radar", "fsim.ms.sar",
                                           "fsim.ms.smti",   "fsim.ms.esm",  "fsim.ms.po",            "fsim.ms.ea",
                                           "fsim.ms.strike", "fsim.ms.store_management", "fsim.ms.comm_terminal", "fsim.ms.pnt"};
    static_assert(sizeof(kIds) / sizeof(kIds[0]) == static_cast<std::size_t>(CapabilityType::Count));
    const auto i = static_cast<std::size_t>(type);
    return i < static_cast<std::size_t>(CapabilityType::Count) ? kIds[i] : "";
}

} // namespace fsim::ms
