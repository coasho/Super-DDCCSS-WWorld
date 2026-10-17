#pragma once

// The metadata of an agent's legal reports (fsim/MAReports.h; docs/sdk/mission-training.md, section 10): for one
// agent and one kind of report, whether it can be given and - item by item, in the order the agent's own read gives
// them - where each came from, when, and how old it is.
//
// It reads what the agent's own calls read and nothing else: the platform's sensed state, its payloads' tracks and
// contacts, its system's picture, its MA core's tasks and authority. No truth record, no other platform's state.
// One implementation serves the C++ Agent, the C ABI and Python, and the mission gateway after them.

#include "fsim/MAReports.h"
#include "simulation/runtime/services/AgentRules.h"

#include <cstdint>
#include <vector>

namespace fsim::session {

class World;

/// The kinds, numbered as the C ABI's fsim_report_kind.
enum class MaReportKind : std::uint8_t {
    Navigation = 0, Tracks, Contacts, LinkTracks, Peers, Tasks, Authority, PeerHeartbeats,
    Stores, Strikes, Countermeasures, Consents, Count
};

struct MaReportMetas {
    ma::Availability availability = ma::Availability::Unavailable;
    /// Navigation, Authority: one where available. Tracks: Agent::amtiTracks of each air radar, in payload order.
    /// Contacts: World::contacts of each payload, in payload order. LinkTracks, Peers: the DMS picture's order. Tasks:
    /// the MA core's order. PeerHeartbeats: beside Peers, index for index. A peer's two reports - its position and its
    /// heartbeat - arrive as two messages: Peers gives the delivery of the position it holds, PeerHeartbeats that of its
    /// last heartbeat, and neither borrows the other's net or time.
    /// Stores: native inventory order. Strikes: equipmentStrikes order. Countermeasures: one inventory where
    /// applicable. Consents: equipmentConsents order, from the configured ingress owner's delivered inbox.
    std::vector<ma::ReportMeta> metas;
};

MaReportMetas reportMetas(const World& world, const AgentIdentity& agent, MaReportKind kind, const ma::ReportOptions& options);

/// Its platform's native strike/jettison statuses, in mission activity order, subject to the existing Agent activity
/// read boundary. A target is its own designation or zero; weapon identities grant no weapon telemetry.
std::vector<ms::StrikeStatus> equipmentStrikes(const World& world, const AgentIdentity& agent);
/// Its delivered native consent requests, copied without consuming or marking them answered. No values for a dead
/// platform or another configured DMS ingress service; reportMetas distinguishes Unavailable and NotOwner.
std::vector<comm::PendingConsent> equipmentConsents(const World& world, const AgentIdentity& agent);

/// The payloads whose tracks and contacts a platform's reports list, in payload order (none for an entity that is not
/// a live vehicle).
std::vector<std::uint16_t> radarPayloads(const World& world, std::uint32_t vehicle);
std::vector<std::uint16_t> payloadIndices(const World& world, std::uint32_t vehicle);

} // namespace fsim::session
