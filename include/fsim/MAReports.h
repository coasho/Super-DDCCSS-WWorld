#pragma once

// fsim SDK: what a legal report is worth - where it came from, when, and how old it is (docs/sdk/mission-training.md,
// section 10; ADR-31).
//
// An Agent already reads its platform's reports: its own ship, its radar's tracks, its passive contacts, the tracks and
// peers its links delivered, C2's tasks, the authority over it. This header adds nothing to what it may know. It gives
// each of those reports its provenance and times, in one form for every language, so that an unknown stays unknown, a
// stale report says so, and nothing is ever filled in from truth:
//
//   Agent::reportTracks() ...     the same values as Agent::amtiTracks(), each with its ReportMeta
//   fsim_agent_report(...)        the same metadata through the C ABI, beside the existing C reads
//   fsim.Agent.report_tracks()    and through Python
//
// Every value comes from the Agent's own legal call; the metadata is computed from those same reports - never from a
// referee record, another platform's state, or a message that has not been delivered.

#include "fsim/Comm.h"
#include "fsim/Export.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace fsim::ma {

/// Whether a report can be given at all.
enum class Availability : std::uint8_t {
    Available = 0,
    Unavailable,   ///< the SDK cannot give it now: its platform is gone
    NotApplicable, ///< its platform has no such thing: a C2 node's navigation, its sensors
    NotOwner,      ///< another configured MA service of the platform holds it (fsim/MAService.h)
};
/// What a held report is worth.
enum class Validity : std::uint8_t {
    Valid = 0, ///< held now, and its source reports it good
    Stale,     ///< held now, older than the caller's ReportOptions::staleAfterS
    Invalid,   ///< its own source flags it: navigation with a sensor invalid
    Unknown,   ///< it holds nothing: never received, or not applicable
};
/// Where a report came from.
enum class Source : std::uint8_t {
    OwnNavigation = 0, ///< the platform's navigation and air data
    OwnSensor,         ///< one of its payloads: a radar's track, a receiver's contact
    OwnPlatform,       ///< the platform itself
    MaCore,            ///< its MA core's state: the authority over it
    Dms,               ///< its DMS
    Link,              ///< delivered over a link by another system
};
/// "available", "unavailable", "not_applicable", "not_owner"; "valid", "stale", "invalid", "unknown";
/// "own_navigation", "own_sensor", "own_platform", "ma_core", "dms", "link" - the contract's names.
FSIM_API const char* availabilityName(Availability availability) noexcept;
FSIM_API const char* validityName(Validity validity) noexcept;
FSIM_API const char* sourceName(Source source) noexcept;

/// A time or an age the source does not state.
inline constexpr double kUnknown = std::numeric_limits<double>::quiet_NaN();
/// No payload: the report is not a payload's.
inline constexpr std::uint16_t kNoPayload = 0xFFFF;

/// A report's provenance and age, in simulation seconds.
struct ReportMeta {
    Source source = Source::OwnPlatform;
    Validity validity = Validity::Unknown;
    comm::SystemId system = comm::kNoSystem; ///< the system it came from, where a link delivered it (0: none)
    comm::NetId net = 0;                     ///< the net it came by (0: none)
    std::uint16_t payload = kNoPayload;      ///< the payload whose report it is
    double sampleTimeS = kUnknown;  ///< when its source measured or stated it; unknown where the source states none
    double receiveTimeS = kUnknown; ///< when it reached this platform: a link's delivery time; an own report's, the time read
    double ageS = kUnknown;         ///< the time read less sampleTimeS; unknown exactly when the sample time is
};

/// How a caller wants its reports judged.
struct ReportOptions {
    /// A held report older than this (its ageS) is Stale. Unknown - the default - marks none: the native picture drops
    /// what its dissemination plan times out, and until then a report is simply held.
    double staleAfterS = kUnknown;
};

/// One item of a collection, with its provenance.
template <typename T>
struct Reported {
    ReportMeta meta;
    T value{};
};
/// A collection as the Agent holds it now. Not Available, it lists nothing.
template <typename T>
struct Collection {
    Availability availability = Availability::Unavailable;
    std::vector<Reported<T>> items;
};
/// A single report. Not Available, `value` is its type's default and means nothing.
template <typename T>
struct Report {
    Availability availability = Availability::Unavailable;
    ReportMeta meta;
    T value{};
};

} // namespace fsim::ma
