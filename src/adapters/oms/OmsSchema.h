#pragma once

// SIM-M08 (MS-12): Versioned external OMS / A-GRA / UCI JSON schema codec (fsim.oms.v1).
//
// Rules enforced by this codec:
// 1. All 64-bit identifiers are encoded strictly as unsigned base-10 decimal strings.
// 2. Non-finite floating-point values (NaN, +/-Inf) are encoded as JSON null, never 0 or NaN.
// 3. Top-level envelope keys and command body keys are closed sets (unknown fields rejected).
// 4. Every envelope explicitly marks transport_domain = "external_oms" and
//    authority_scope = "agent_only" to separate external OMS traffic from modeled RF DMS.
// 5. Sensor reports (AMTI_TrackMT, ContactMT) never include referee entity truth.

#include "fsim/Agent.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace fsim::oms {

inline constexpr std::string_view kSchemaVersion = "fsim.oms.v1";
inline constexpr std::string_view kTransportDomain = "external_oms";
inline constexpr std::string_view kAuthorityScope = "agent_only";

enum class ServiceHealth : std::uint8_t {
    Operational,
    Degraded,
    Faulted,
    Offline,
};

const char* serviceHealthName(ServiceHealth health) noexcept;
bool parseServiceHealth(std::string_view name, ServiceHealth& out) noexcept;

enum class MessageType : std::uint8_t {
    ServiceRegistrationMT,
    ServiceHeartbeatMT,
    ServiceStatusMT,
    SubsystemStatusMT,
    CapabilityStatusMT,
    ActivityStatusMT,
    SubsystemStateCommandMT,
    SubsystemBITCommandMT,
    AMTI_CommandMT,
    PayloadTaskCommandMT,
    StrikeCommandMT,
    JettisonCommandMT,
    ActivityCancelCommandMT,
    ControlGrantCommandMT,
    CommandResponseMT,
    OwnShipMDT,
    AMTI_TrackMT,
    ContactMT,
    PayloadReportMT,
    StoreInventoryMT,
    ProtocolErrorMT,
};

const char* messageTypeName(MessageType type) noexcept;
bool parseMessageType(std::string_view name, MessageType& out) noexcept;

/// Format an unsigned 64-bit identifier as a base-10 decimal string.
std::string formatUint64(std::uint64_t value);

/// Parse an unsigned 64-bit decimal string strictly (digits only, no sign/whitespace/overflow).
bool parseUint64Strict(std::string_view text, std::uint64_t& out) noexcept;

/// Versioned wire envelope for all fsim.oms.v1 messages.
struct OmsEnvelope {
    std::string schema = std::string(kSchemaVersion);
    std::string transportDomain = std::string(kTransportDomain);
    std::string authorityScope = std::string(kAuthorityScope);
    MessageType messageType = MessageType::ServiceStatusMT;
    std::uint64_t messageId = 0;
    std::uint64_t correlationId = 0;
    std::uint64_t sourceServiceId = 0;
    double timestampS = 0.0;
    std::string bodyJson = "{}";
};

bool serializeEnvelope(const OmsEnvelope& env, std::string& outJson, std::string* error = nullptr);
bool parseEnvelope(std::string_view jsonText, OmsEnvelope& outEnv, std::string* error = nullptr);

// --- Decoded inbound service & command payloads ---

struct ServiceHeartbeatPayload {
    std::uint64_t serviceId = 0;
    std::uint64_t sequence = 0;
    ServiceHealth health = ServiceHealth::Operational;
};

struct ActivityCancelPayload {
    std::uint64_t activityId = 0;
};

struct ControlGrantPayload {
    std::string capability;
    bool request = true; ///< true = requestControl, false = releaseControl
};

using CommandVariant = std::variant<
    ms::StateCommand,
    ms::BitCommand,
    ms::AmtiCommand,
    ms::PayloadTask,
    ms::StrikeCommand,
    ms::JettisonCommand,
    ActivityCancelPayload,
    ControlGrantPayload>;

struct ParsedCommand {
    std::uint64_t commandId = 0;
    MessageType messageType = MessageType::SubsystemStateCommandMT;
    std::string canonicalBody; ///< canonical JSON representation for idempotent conflict checks
    CommandVariant payload;
};

bool parseServiceHeartbeatBody(std::string_view bodyJson, ServiceHeartbeatPayload& out, std::string* error = nullptr);
bool parseCommandBody(MessageType type, std::string_view bodyJson, ParsedCommand& out, std::string* error = nullptr);

// --- Outbound status, response, and Agent-scoped report body builders ---

std::string makeServiceRegistrationBody(std::uint64_t serviceId,
                                        std::string_view serviceName,
                                        std::uint32_t vehicleId,
                                        std::string_view platformType,
                                        double heartbeatPeriodS,
                                        double peerHeartbeatTimeoutS);

std::string makeServiceHeartbeatBody(std::uint64_t serviceId,
                                     std::uint64_t sequence,
                                     ServiceHealth health);

std::string makeServiceStatusBody(std::uint64_t serviceId,
                                  ServiceHealth ownHealth,
                                  ServiceHealth peerHealth,
                                  std::uint64_t peerServiceId,
                                  double lastPeerHeartbeatS,
                                  std::uint64_t commandsProcessed,
                                  std::uint64_t duplicateCommands,
                                  std::uint64_t protocolErrors);

std::string makeSubsystemStatusBody(const std::vector<ms::PayloadInfo>& payloads);
std::string makeCapabilityStatusBody(const Agent& agent);
std::string makeActivityStatusBody(const std::vector<ms::ActivityRecord>& activities);

std::string makeCommandResponseBody(std::uint64_t commandId,
                                    control::CommandStatus status,
                                    std::string_view reason,
                                    std::string_view reasonDescription,
                                    std::uint64_t activityId,
                                    bool duplicate);

std::string makeOwnShipBody(const OwnShip& ownShip);
std::string makeAmtiTracksBody(std::uint16_t payload, const std::vector<ms::AmtiTrack>& tracks);
std::string makeContactsBody(std::uint16_t payload, const std::vector<ms::Contact>& contacts);
std::string makePayloadReportBody(std::uint16_t payload, const std::vector<ms::PayloadValue>& values);
std::string makeStoreInventoryBody(const std::vector<ms::StoreInfo>& stores);
std::string makeProtocolErrorBody(std::string_view code, std::string_view detail);

} // namespace fsim::oms
