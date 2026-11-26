#pragma once

// SIM-M08 (MS-12): External OMS / A-GRA / UCI Service Adapter strictly above fsim::Agent.
//
// OmsServiceAdapter binds exclusively to a single platform's fsim::Agent and an external
// IOmsTransport. It never accesses World or Referee, never injects release consent, and
// keeps external OMS service messages strictly distinct from modeled RF DMS traffic.

#include "adapters/oms/OmsSchema.h"
#include "adapters/oms/OmsTransport.h"
#include "fsim/Agent.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsim::oms {

struct OmsAdapterConfig {
    std::uint64_t serviceId = 1001;
    std::string serviceName = "fsim.oms.agent";
    double heartbeatPeriodS = 1.0;
    double peerHeartbeatTimeoutS = 3.0;
    FrameMode frameMode = FrameMode::Ndjson;
    std::size_t maxFrameBytes = kDefaultMaxFrameBytes;
    std::size_t maxCommandHistory = 1024;
};

struct OmsAdapterStats {
    std::uint64_t framesSent = 0;
    std::uint64_t framesReceived = 0;
    std::uint64_t heartbeatsSent = 0;
    std::uint64_t peerHeartbeatsReceived = 0;
    std::uint64_t commandsProcessed = 0;
    std::uint64_t duplicateCommands = 0;
    std::uint64_t commandConflicts = 0;
    std::uint64_t protocolErrors = 0;
};

class OmsServiceAdapter {
public:
    OmsServiceAdapter(Agent& agent, IOmsTransport& transport, OmsAdapterConfig config = {});

    /// Publish ServiceRegistrationMT followed by initial ServiceStatusMT, SubsystemStatusMT,
    /// and CapabilityStatusMT snapshots.
    bool registerService(double nowS);

    /// Drain inbound frames from the transport, validate and dispatch commands (with bounded
    /// idempotent deduplication), update peer heartbeat watchdog health, and emit periodic
    /// ServiceHeartbeatMT envelopes. Returns the number of inbound frames processed.
    std::size_t poll(double nowS);

    /// Publish current ServiceStatusMT, SubsystemStatusMT, CapabilityStatusMT, and ActivityStatusMT.
    bool publishStatus(double nowS);

    /// Publish current Agent-scoped OwnShipMDT, AMTI_TrackMT, ContactMT, PayloadReportMT,
    /// and StoreInventoryMT snapshots (strictly without referee truth).
    bool publishSensorReports(double nowS);

    ServiceHealth ownHealth() const noexcept { return ownHealth_; }
    ServiceHealth peerHealth() const noexcept { return peerHealth_; }
    std::uint64_t peerServiceId() const noexcept { return peerServiceId_; }
    const OmsAdapterConfig& config() const noexcept { return config_; }
    const OmsAdapterStats& stats() const noexcept { return stats_; }

private:
    struct CommandRecord {
        MessageType messageType = MessageType::SubsystemStateCommandMT;
        std::string canonicalBody;
        control::CommandStatus status = control::CommandStatus::Rejected;
        std::string reason;
        std::string reasonDescription;
        std::uint64_t activityId = 0;
    };

    bool sendEnvelope(MessageType type, std::uint64_t correlationId, double nowS, std::string bodyJson);
    void handleInboundFrame(std::string_view frameJson, double nowS);
    void handleCommandEnvelope(const OmsEnvelope& env, double nowS);
    void updateWatchdog(double nowS);
    void rememberCommand(std::uint64_t commandId, CommandRecord rec);

    Agent& agent_;
    IOmsTransport& transport_;
    OmsAdapterConfig config_;
    FrameDecoder decoder_;

    std::uint64_t nextMessageId_ = 1;
    std::uint64_t heartbeatSeq_ = 0;
    double lastHeartbeatSentS_ = std::numeric_limits<double>::quiet_NaN();
    bool registered_ = false;

    ServiceHealth ownHealth_ = ServiceHealth::Operational;
    ServiceHealth peerHealth_ = ServiceHealth::Offline;
    std::uint64_t peerServiceId_ = 0;
    double lastPeerHeartbeatS_ = std::numeric_limits<double>::quiet_NaN();

    std::unordered_map<std::uint64_t, CommandRecord> commandCache_;
    std::deque<std::uint64_t> commandOrder_;
    OmsAdapterStats stats_{};
};

/// Lightweight external OMS client helper for protocol fixtures and external consumers.
class OmsClientPeer {
public:
    OmsClientPeer(IOmsTransport& transport,
                  std::uint64_t serviceId,
                  FrameMode frameMode = FrameMode::Ndjson,
                  std::size_t maxFrameBytes = kDefaultMaxFrameBytes);

    bool sendMessage(MessageType type, double nowS, std::string bodyJson, std::uint64_t correlationId = 0);
    bool sendHeartbeat(double nowS, ServiceHealth health = ServiceHealth::Operational);
    bool sendRawBytes(std::string_view rawBytes);

    /// Drain all currently buffered complete inbound envelopes from the transport.
    std::vector<OmsEnvelope> receiveAll(std::vector<std::string>* parseErrors = nullptr);

private:
    IOmsTransport& transport_;
    std::uint64_t serviceId_ = 0;
    FrameMode frameMode_ = FrameMode::Ndjson;
    std::size_t maxFrameBytes_ = kDefaultMaxFrameBytes;
    FrameDecoder decoder_;
    std::uint64_t nextMessageId_ = 1;
    std::uint64_t heartbeatSeq_ = 0;
};

} // namespace fsim::oms
