#pragma once

// The metadata of an agent's legal reports (fsim/MAReports.h; docs/sdk/mission-training.md, section 10): for one
// agent and one kind of report, whether it can be given and - item by item, in the order the agent's own read gives
// them - where each came from, when, and how old it is.
//
// It reads what the agent's own calls read and nothing else: the platform's sensed state, its payloads' tracks and
// contacts, its system's picture, its MA core's tasks and authority. No truth record, no other platform's state.
// One implementation serves the C++ Agent, the C ABI and Python, and the mission gateway after them.

#include "fsim/MAReports.h"
#include "session/AgentRules.h"

#include <cstdint>
#include <vector>

namespace fsim::session {

class World;

/// The kinds, numbered as the C ABI's fsim_report_kind.
enum class MaReportKind : std::uint8_t { Navigation = 0, Tracks, Contacts, LinkTracks, Peers, Tasks, Authority, Count };

struct MaReportMetas {
    ma::Availability availability = ma::Availability::Unavailable;
    /// Navigation, Authority: one where available. Tracks: Agent::amtiTracks of each air radar, in payload order.
    /// Contacts: World::contacts of each payload, in payload order. LinkTracks, Peers: the DMS picture's order. Tasks:
    /// the MA core's order.
    std::vector<ma::ReportMeta> metas;
};

MaReportMetas reportMetas(const World& world, const AgentIdentity& agent, MaReportKind kind, const ma::ReportOptions& options);

/// The payloads whose tracks and contacts a platform's reports list, in payload order (none for an entity that is not
/// a live vehicle).
std::vector<std::uint16_t> radarPayloads(const World& world, std::uint32_t vehicle);
std::vector<std::uint16_t> payloadIndices(const World& world, std::uint32_t vehicle);

} // namespace fsim::session
