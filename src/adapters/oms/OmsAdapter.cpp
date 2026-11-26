#include "adapters/oms/OmsAdapter.h"

#include <cmath>
#include <utility>

namespace fsim::oms {

OmsServiceAdapter::OmsServiceAdapter(Agent& agent, IOmsTransport& transport, OmsAdapterConfig config)
    : agent_(agent),
      transport_(transport),
      config_(std::move(config)),
      decoder_(config_.frameMode, config_.maxFrameBytes) {
    ownHealth_ = (agent_.valid() && transport_.connected())
                     ? ServiceHealth::Operational
                     : ServiceHealth::Offline;
}

bool OmsServiceAdapter::sendEnvelope(MessageType type,
                                     std::uint64_t correlationId,
                                     double nowS,
                                     std::string bodyJson) {
    if (!transport_.connected()) {
        ownHealth_ = ServiceHealth::Offline;
        return false;
    }
    OmsEnvelope env;
    env.messageType = type;
    env.messageId = nextMessageId_++;
    env.correlationId = correlationId;
    env.sourceServiceId = config_.serviceId;
    env.timestampS = nowS;
    env.bodyJson = std::move(bodyJson);

    std::string jsonText;
    if (!serializeEnvelope(env, jsonText)) {
        ++stats_.protocolErrors;
        return false;
    }
    std::string wire;
    FrameError ferr = FrameError::None;
    if (!encodeFrame(jsonText, config_.frameMode, wire, &ferr, config_.maxFrameBytes)) {
        ++stats_.protocolErrors;
        return false;
    }
    if (!transport_.sendBytes(wire)) {
        ownHealth_ = ServiceHealth::Offline;
        return false;
    }
    ++stats_.framesSent;
    return true;
}

bool OmsServiceAdapter::registerService(double nowS) {
    if (!transport_.connected() || !agent_.valid()) {
        ownHealth_ = ServiceHealth::Offline;
        return false;
    }
    ownHealth_ = ServiceHealth::Operational;
    const bool okReg = sendEnvelope(
        MessageType::ServiceRegistrationMT,
        0,
        nowS,
        makeServiceRegistrationBody(config_.serviceId,
                                    config_.serviceName,
                                    agent_.vehicle(),
                                    agent_.type(),
                                    config_.heartbeatPeriodS,
                                    config_.peerHeartbeatTimeoutS));
    registered_ = okReg;
    if (!okReg) return false;
    lastHeartbeatSentS_ = nowS;
    ++heartbeatSeq_;
    ++stats_.heartbeatsSent;
    sendEnvelope(MessageType::ServiceHeartbeatMT,
                 0,
                 nowS,
                 makeServiceHeartbeatBody(config_.serviceId, heartbeatSeq_, ownHealth_));
    return publishStatus(nowS);
}

void OmsServiceAdapter::rememberCommand(std::uint64_t commandId, CommandRecord rec) {
    if (config_.maxCommandHistory == 0) return;
    if (commandCache_.find(commandId) == commandCache_.end()) {
        if (commandOrder_.size() >= config_.maxCommandHistory) {
            const std::uint64_t oldest = commandOrder_.front();
            commandOrder_.pop_front();
            commandCache_.erase(oldest);
        }
        commandOrder_.push_back(commandId);
    }
    commandCache_[commandId] = std::move(rec);
}

void OmsServiceAdapter::handleCommandEnvelope(const OmsEnvelope& env, double nowS) {
    ParsedCommand parsed;
    std::string err;
    if (!parseCommandBody(env.messageType, env.bodyJson, parsed, &err)) {
        ++stats_.protocolErrors;
        sendEnvelope(MessageType::ProtocolErrorMT,
                     env.messageId,
                     nowS,
                     makeProtocolErrorBody("invalid_command_body", err));
        return;
    }

    // Bounded idempotent deduplication by command_id.
    if (auto it = commandCache_.find(parsed.commandId); it != commandCache_.end()) {
        const CommandRecord& prev = it->second;
        if (prev.messageType == parsed.messageType && prev.canonicalBody == parsed.canonicalBody) {
            ++stats_.duplicateCommands;
            sendEnvelope(MessageType::CommandResponseMT,
                         env.messageId,
                         nowS,
                         makeCommandResponseBody(parsed.commandId,
                                                 prev.status,
                                                 prev.reason,
                                                 prev.reasonDescription,
                                                 prev.activityId,
                                                 true));
            return;
        }
        ++stats_.commandConflicts;
        sendEnvelope(MessageType::CommandResponseMT,
                     env.messageId,
                     nowS,
                     makeCommandResponseBody(parsed.commandId,
                                             control::CommandStatus::Rejected,
                                             "invalid_parameter",
                                             "command_id_conflict",
                                             0,
                                             false));
        return;
    }

    ms::CommandOptions opts;
    opts.commandId = parsed.commandId;

    CommandRecord rec;
    rec.messageType = parsed.messageType;
    rec.canonicalBody = parsed.canonicalBody;

    auto recordMsResult = [&](const ms::CommandResult& res) {
        rec.status = res.status;
        rec.reason = ms::reasonName(res.reason);
        rec.reasonDescription = ms::reasonDescription(res.reason);
        rec.activityId = res.activity;
    };

    if (const auto* stateCmd = std::get_if<ms::StateCommand>(&parsed.payload)) {
        recordMsResult(agent_.submit(*stateCmd, opts));
    } else if (const auto* bitCmd = std::get_if<ms::BitCommand>(&parsed.payload)) {
        recordMsResult(agent_.submit(*bitCmd, opts));
    } else if (const auto* amtiCmd = std::get_if<ms::AmtiCommand>(&parsed.payload)) {
        recordMsResult(agent_.submit(*amtiCmd, opts));
    } else if (const auto* taskCmd = std::get_if<ms::PayloadTask>(&parsed.payload)) {
        recordMsResult(agent_.submit(*taskCmd, opts));
    } else if (const auto* strikeCmd = std::get_if<ms::StrikeCommand>(&parsed.payload)) {
        recordMsResult(agent_.submit(*strikeCmd, opts));
    } else if (const auto* jettisonCmd = std::get_if<ms::JettisonCommand>(&parsed.payload)) {
        recordMsResult(agent_.submit(*jettisonCmd, opts));
    } else if (const auto* cancelCmd = std::get_if<ActivityCancelPayload>(&parsed.payload)) {
        recordMsResult(agent_.cancelMission(cancelCmd->activityId));
    } else if (const auto* grantCmd = std::get_if<ControlGrantPayload>(&parsed.payload)) {
        const control::Reason reason = grantCmd->request
                                           ? agent_.requestControl(grantCmd->capability)
                                           : agent_.releaseControl(grantCmd->capability);
        rec.status = (reason == control::Reason::None)
                         ? control::CommandStatus::Accepted
                         : control::CommandStatus::Rejected;
        rec.reason = control::reasonName(reason);
        rec.reasonDescription = control::reasonDescription(reason);
        rec.activityId = 0;
    }

    ++stats_.commandsProcessed;
    sendEnvelope(MessageType::CommandResponseMT,
                 env.messageId,
                 nowS,
                 makeCommandResponseBody(parsed.commandId,
                                         rec.status,
                                         rec.reason,
                                         rec.reasonDescription,
                                         rec.activityId,
                                         false));
    rememberCommand(parsed.commandId, std::move(rec));
}

void OmsServiceAdapter::handleInboundFrame(std::string_view frameJson, double nowS) {
    ++stats_.framesReceived;
    OmsEnvelope env;
    std::string err;
    if (!parseEnvelope(frameJson, env, &err)) {
        ++stats_.protocolErrors;
        sendEnvelope(MessageType::ProtocolErrorMT,
                     0,
                     nowS,
                     makeProtocolErrorBody("invalid_envelope", err));
        return;
    }

    if (env.messageType == MessageType::ServiceHeartbeatMT) {
        ServiceHeartbeatPayload hb;
        if (!parseServiceHeartbeatBody(env.bodyJson, hb, &err)) {
            ++stats_.protocolErrors;
            sendEnvelope(MessageType::ProtocolErrorMT,
                         env.messageId,
                         nowS,
                         makeProtocolErrorBody("invalid_heartbeat", err));
            return;
        }
        ++stats_.peerHeartbeatsReceived;
        peerServiceId_ = hb.serviceId;
        lastPeerHeartbeatS_ = nowS;
        const ServiceHealth prevPeer = peerHealth_;
        peerHealth_ = hb.health;
        if (peerHealth_ != prevPeer) {
            sendEnvelope(MessageType::ServiceStatusMT,
                         env.messageId,
                         nowS,
                         makeServiceStatusBody(config_.serviceId,
                                               ownHealth_,
                                               peerHealth_,
                                               peerServiceId_,
                                               lastPeerHeartbeatS_,
                                               stats_.commandsProcessed,
                                               stats_.duplicateCommands,
                                               stats_.protocolErrors));
        }
        return;
    }

    switch (env.messageType) {
    case MessageType::SubsystemStateCommandMT:
    case MessageType::SubsystemBITCommandMT:
    case MessageType::AMTI_CommandMT:
    case MessageType::PayloadTaskCommandMT:
    case MessageType::StrikeCommandMT:
    case MessageType::JettisonCommandMT:
    case MessageType::ActivityCancelCommandMT:
    case MessageType::ControlGrantCommandMT:
        handleCommandEnvelope(env, nowS);
        return;
    default:
        ++stats_.protocolErrors;
        sendEnvelope(MessageType::ProtocolErrorMT,
                     env.messageId,
                     nowS,
                     makeProtocolErrorBody("unsupported_inbound_message", messageTypeName(env.messageType)));
        return;
    }
}

void OmsServiceAdapter::updateWatchdog(double nowS) {
    if (!transport_.connected() || !agent_.valid()) {
        ownHealth_ = ServiceHealth::Offline;
        peerHealth_ = ServiceHealth::Offline;
        return;
    }
    ownHealth_ = ServiceHealth::Operational;

    if (std::isfinite(lastPeerHeartbeatS_) && config_.peerHeartbeatTimeoutS > 0.0) {
        const double ageS = nowS - lastPeerHeartbeatS_;
        ServiceHealth nextPeer = peerHealth_;
        if (ageS > 2.0 * config_.peerHeartbeatTimeoutS) {
            nextPeer = ServiceHealth::Faulted;
        } else if (ageS > config_.peerHeartbeatTimeoutS) {
            nextPeer = ServiceHealth::Degraded;
        }
        if (nextPeer != peerHealth_) {
            peerHealth_ = nextPeer;
            sendEnvelope(MessageType::ServiceStatusMT,
                         0,
                         nowS,
                         makeServiceStatusBody(config_.serviceId,
                                               ownHealth_,
                                               peerHealth_,
                                               peerServiceId_,
                                               lastPeerHeartbeatS_,
                                               stats_.commandsProcessed,
                                               stats_.duplicateCommands,
                                               stats_.protocolErrors));
        }
    }

    if (registered_ && config_.heartbeatPeriodS > 0.0) {
        if (!std::isfinite(lastHeartbeatSentS_) || (nowS - lastHeartbeatSentS_) + 1e-9 >= config_.heartbeatPeriodS) {
            lastHeartbeatSentS_ = nowS;
            ++heartbeatSeq_;
            ++stats_.heartbeatsSent;
            sendEnvelope(MessageType::ServiceHeartbeatMT,
                         0,
                         nowS,
                         makeServiceHeartbeatBody(config_.serviceId, heartbeatSeq_, ownHealth_));
        }
    }
}

std::size_t OmsServiceAdapter::poll(double nowS) {
    std::size_t processed = 0;
    if (!transport_.connected()) {
        updateWatchdog(nowS);
        return 0;
    }

    for (;;) {
        const std::string chunk = transport_.receiveBytes(config_.maxFrameBytes);
        if (chunk.empty()) break;
        std::vector<std::string> frames;
        FrameError ferr = FrameError::None;
        if (!decoder_.feed(chunk, frames, &ferr)) {
            ++stats_.protocolErrors;
            sendEnvelope(MessageType::ProtocolErrorMT,
                         0,
                         nowS,
                         makeProtocolErrorBody("framing_error", frameErrorName(ferr)));
            break;
        }
        for (const std::string& frame : frames) {
            handleInboundFrame(frame, nowS);
            ++processed;
        }
    }

    updateWatchdog(nowS);
    return processed;
}

bool OmsServiceAdapter::publishStatus(double nowS) {
    if (!transport_.connected() || !agent_.valid()) {
        ownHealth_ = ServiceHealth::Offline;
        return false;
    }
    bool ok = true;
    ok &= sendEnvelope(MessageType::ServiceStatusMT,
                       0,
                       nowS,
                       makeServiceStatusBody(config_.serviceId,
                                             ownHealth_,
                                             peerHealth_,
                                             peerServiceId_,
                                             lastPeerHeartbeatS_,
                                             stats_.commandsProcessed,
                                             stats_.duplicateCommands,
                                             stats_.protocolErrors));
    ok &= sendEnvelope(MessageType::SubsystemStatusMT,
                       0,
                       nowS,
                       makeSubsystemStatusBody(agent_.payloads()));
    ok &= sendEnvelope(MessageType::CapabilityStatusMT,
                       0,
                       nowS,
                       makeCapabilityStatusBody(agent_));
    ok &= sendEnvelope(MessageType::ActivityStatusMT,
                       0,
                       nowS,
                       makeActivityStatusBody(agent_.missionActivities()));
    return ok;
}

bool OmsServiceAdapter::publishSensorReports(double nowS) {
    if (!transport_.connected() || !agent_.valid()) {
        ownHealth_ = ServiceHealth::Offline;
        return false;
    }
    bool ok = true;
    ok &= sendEnvelope(MessageType::OwnShipMDT, 0, nowS, makeOwnShipBody(agent_.ownship()));

    const auto payloads = agent_.payloads();
    for (const auto& p : payloads) {
        if (p.family == ms::Family::AirRadar) {
            const auto tracks = agent_.amtiTracks(p.index);
            if (!tracks.empty()) {
                ok &= sendEnvelope(MessageType::AMTI_TrackMT, 0, nowS, makeAmtiTracksBody(p.index, tracks));
            }
        }
        const auto contacts = agent_.contacts(p.index);
        if (!contacts.empty()) {
            ok &= sendEnvelope(MessageType::ContactMT, 0, nowS, makeContactsBody(p.index, contacts));
        }
        const auto report = agent_.payloadReport(p.index);
        if (!report.empty()) {
            ok &= sendEnvelope(MessageType::PayloadReportMT, 0, nowS, makePayloadReportBody(p.index, report));
        }
    }

    const auto stores = agent_.stores();
    if (!stores.empty()) {
        ok &= sendEnvelope(MessageType::StoreInventoryMT, 0, nowS, makeStoreInventoryBody(stores));
    }
    return ok;
}

// --- OmsClientPeer ---

OmsClientPeer::OmsClientPeer(IOmsTransport& transport,
                             std::uint64_t serviceId,
                             FrameMode frameMode,
                             std::size_t maxFrameBytes)
    : transport_(transport),
      serviceId_(serviceId),
      frameMode_(frameMode),
      maxFrameBytes_(maxFrameBytes),
      decoder_(frameMode, maxFrameBytes) {}

bool OmsClientPeer::sendMessage(MessageType type, double nowS, std::string bodyJson, std::uint64_t correlationId) {
    if (!transport_.connected()) return false;
    OmsEnvelope env;
    env.messageType = type;
    env.messageId = nextMessageId_++;
    env.correlationId = correlationId;
    env.sourceServiceId = serviceId_;
    env.timestampS = nowS;
    env.bodyJson = std::move(bodyJson);

    std::string jsonText;
    if (!serializeEnvelope(env, jsonText)) return false;
    std::string wire;
    if (!encodeFrame(jsonText, frameMode_, wire, nullptr, maxFrameBytes_)) return false;
    return transport_.sendBytes(wire);
}

bool OmsClientPeer::sendHeartbeat(double nowS, ServiceHealth health) {
    ++heartbeatSeq_;
    return sendMessage(MessageType::ServiceHeartbeatMT,
                       nowS,
                       makeServiceHeartbeatBody(serviceId_, heartbeatSeq_, health));
}

bool OmsClientPeer::sendRawBytes(std::string_view rawBytes) {
    return transport_.sendBytes(rawBytes);
}

std::vector<OmsEnvelope> OmsClientPeer::receiveAll(std::vector<std::string>* parseErrors) {
    std::vector<OmsEnvelope> out;
    for (;;) {
        const std::string chunk = transport_.receiveBytes(maxFrameBytes_);
        if (chunk.empty()) break;
        std::vector<std::string> frames;
        FrameError ferr = FrameError::None;
        if (!decoder_.feed(chunk, frames, &ferr)) {
            if (parseErrors) parseErrors->push_back(std::string("framing_error: ") + frameErrorName(ferr));
            break;
        }
        for (const std::string& frame : frames) {
            OmsEnvelope env;
            std::string err;
            if (parseEnvelope(frame, env, &err)) {
                out.push_back(std::move(env));
            } else if (parseErrors) {
                parseErrors->push_back(std::move(err));
            }
        }
    }
    return out;
}

} // namespace fsim::oms
