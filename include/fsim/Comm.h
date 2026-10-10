#pragma once

// Abstract communication interfaces (design 9.6, ADR-22): nodes exchange
// messages through a medium that decides delivery; codecs give payloads a
// format; protocols add application-level behaviour. Every piece is an
// interface with a small built-in implementation.

#include "fsim/Control.h" // WorldView
#include "fsim/Export.h"
#include "fsim/Rng.h"
#include "fsim/Span.h"

#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace fsim::comm {

using Address = std::uint32_t;
inline constexpr Address kBroadcast = 0xFFFFFFFFu;
inline constexpr Address kNoAddress = 0;

/// Payload format ids. Users register their own above kUserFormat.
enum Format : std::uint32_t { kRaw = 1, kJson = 2, kUserFormat = 1000 };

struct Payload {
    std::uint32_t format = kRaw;
    std::vector<std::uint8_t> bytes;
};

struct Message {
    Address from = kNoAddress;
    Address to = kBroadcast;
    std::uint32_t channel = 0;
    double timeSent = 0.0;
    double timeDelivered = 0.0;
    Payload payload;
};

/// Raw codec: trivially copyable structs as bytes (same build both ends).
template <typename T>
Payload encodeRaw(const T& value) {
    static_assert(std::is_trivially_copyable_v<T>, "raw payloads must be trivially copyable");
    Payload p;
    p.format = kRaw;
    p.bytes.resize(sizeof(T));
    std::memcpy(p.bytes.data(), &value, sizeof(T));
    return p;
}
template <typename T>
bool decodeRaw(const Payload& p, T& out) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "raw payloads must be trivially copyable");
    if (p.format != kRaw || p.bytes.size() != sizeof(T)) return false;
    std::memcpy(&out, p.bytes.data(), sizeof(T));
    return true;
}

/// Named numeric fields; the JSON codec is the language-neutral format.
using Fields = std::vector<std::pair<std::string, double>>;
struct FSIM_API JsonCodec {
    static Payload encode(const Fields& fields);
    static bool decode(const Payload& payload, Fields& out);
};

/// A codec plugin for other formats (msgpack, protobuf, ...).
class Codec {
public:
    virtual ~Codec() = default;
    virtual const char* id() const noexcept = 0;
    virtual std::uint32_t format() const noexcept = 0;
    virtual Payload encode(const Fields& fields) const = 0;
    virtual bool decode(const Payload& payload, Fields& out) const = 0;
};

class Network;

/// An endpoint: a vehicle (address = vehicle id) or an external node.
class Node {
public:
    Node(Address address, std::uint32_t vehicleId) : address_(address), vehicleId_(vehicleId) {}
    Address address() const noexcept { return address_; }
    std::uint32_t vehicleId() const noexcept { return vehicleId_; } ///< 0 for external nodes

    /// Messages delivered during the last network step.
    Span<const Message> inbox() const noexcept { return Span<const Message>(inbox_); }
    std::uint64_t sent() const noexcept { return sent_; }
    std::uint64_t received() const noexcept { return received_; }

private:
    friend class Network;
    Address address_;
    std::uint32_t vehicleId_;
    std::vector<Message> inbox_;
    std::uint64_t sent_ = 0, received_ = 0;
};

/// Delivery model: decides if and when a message reaches a node.
class Medium {
public:
    virtual ~Medium() = default;
    virtual const char* id() const noexcept = 0;
    /// Return false to drop; else set `deliverAt` (>= now).
    virtual bool route(const Message& message, const Node& from, const Node& to, const control::WorldView* world, double now,
                       Rng& rng, double& deliverAt) = 0;
};

/// Instant, lossless.
class IdealMedium final : public Medium {
public:
    const char* id() const noexcept override { return "ideal"; }
    bool route(const Message&, const Node&, const Node&, const control::WorldView*, double now, Rng&, double& deliverAt) override {
        deliverAt = now;
        return true;
    }
};

/// Range-limited, delayed, jittered, lossy link between vehicle nodes.
class FSIM_API LinkModel final : public Medium {
public:
    const char* id() const noexcept override { return "link"; }
    bool route(const Message& message, const Node& from, const Node& to, const control::WorldView* world, double now, Rng& rng,
               double& deliverAt) override;

    double rangeM = 20000.0;
    double latencyS = 0.05;
    double jitterS = 0.01;
    double lossProbability = 0.0;
};

/// Application-level behaviour attached to a node (beacons, request/response, ...).
class Protocol {
public:
    virtual ~Protocol() = default;
    virtual const char* id() const noexcept = 0;
    virtual void onStep(Node& node, Network& network, const control::WorldView* world, double simTime, double dt) = 0;
    virtual void onReceive(Node& node, const Message& message) { (void)node; (void)message; }
};

/// Periodic JSON state report on `channel` (id, lat, lon, alt, heading, speed).
class FSIM_API BeaconProtocol final : public Protocol {
public:
    explicit BeaconProtocol(double periodS = 1.0, std::uint32_t channel = 1) : periodS_(periodS), channel_(channel) {}
    const char* id() const noexcept override { return "beacon"; }
    void onStep(Node& node, Network& network, const control::WorldView* world, double simTime, double dt) override;

private:
    double periodS_, nextAt_ = 0.0;
    std::uint32_t channel_;
};

/// Byte transport under a BridgeProtocol: one datagram out, one datagram in.
/// Implement it for your own link (serial, shared memory, a message queue);
/// createUdpTransport is built in. receive() must not block.
class Transport {
public:
    virtual ~Transport() = default;
    virtual const char* id() const noexcept = 0;
    virtual bool send(const std::uint8_t* bytes, std::size_t length) = 0;
    /// Next waiting datagram; false when none.
    virtual bool receive(std::vector<std::uint8_t>& out) = 0;
};

/// Non-blocking UDP: bound to `localPort` (0 = any free port), sending to
/// `remoteHost:remotePort`. Null (with `error`) when the port is taken.
FSIM_API std::unique_ptr<Transport> createUdpTransport(std::uint16_t localPort, const std::string& remoteHost, std::uint16_t remotePort,
                                                       std::string* error = nullptr);

/// Wire format of a bridged message (little-endian, 36-byte header):
///   "FSMG" | u8 version = 1 | u8 reserved[3] | u32 from | u32 to | u32 channel | u32 format | f64 timeSent | u32 length | bytes
/// Any process that speaks this over UDP takes part in the world's network.
FSIM_API std::vector<std::uint8_t> encodeWire(const Message& message);
FSIM_API bool decodeWire(const std::uint8_t* bytes, std::size_t length, Message& out) noexcept;

/// Carries a node's traffic over a Transport, so an external process (or
/// hardware) is a node of the world: every message delivered to the node goes
/// out as one datagram; every datagram received comes in as a message from
/// the node, with the wire's `to`, `channel` and payload.
class FSIM_API BridgeProtocol final : public Protocol {
public:
    explicit BridgeProtocol(std::unique_ptr<Transport> transport) : transport_(std::move(transport)) {}
    const char* id() const noexcept override { return "bridge"; }
    void onStep(Node& node, Network& network, const control::WorldView* world, double simTime, double dt) override;
    void onReceive(Node& node, const Message& message) override;
    std::uint64_t datagramsOut() const noexcept { return out_; }
    std::uint64_t datagramsIn() const noexcept { return in_; }
    std::uint64_t rejected() const noexcept { return rejected_; } ///< datagrams that were not wire messages
    Transport& transport() noexcept { return *transport_; }

private:
    std::unique_ptr<Transport> transport_;
    std::vector<std::uint8_t> buffer_;
    std::uint64_t out_ = 0, in_ = 0, rejected_ = 0;
};

// --- Systems, nets and the DMS (ADR-31, AB-4; A-GRA's Decentralized Messaging Service, MS 1.2.5) ------------------
//
// Every platform and C2 node that communicates is a system, named by the SystemID the range assigned it (never an
// entity id). Systems talk over nets the range makes and puts them on; each system's DMS sends - publishes on a topic,
// or sends reliably to given systems - and receives what reached it, stamped with its sender's SystemID.

using SystemId = std::uint32_t;
using NetId = std::uint32_t;
using Topic = std::uint32_t;
inline constexpr SystemId kNoSystem = 0;

/// Topics: A-GRA's native message types below kFreeForm (sent by the MA core from its platform's own reports: AB-4b),
/// an agent's own free-form topics from it.
namespace topic {
inline constexpr Topic kSystemStatus = 1;  ///< SystemStatusMT: a heartbeat, its tail number (Peer 1.2.6.3)
inline constexpr Topic kPositionReport = 2; ///< MA_PositionReportMT: its own ship as its navigation reports it (Peer 1.2.6.2)
inline constexpr Topic kEntity = 3;         ///< EntityMT: one of its tracks (Peer 1.2.5.4)
inline constexpr Topic kFreeForm = 1000;    ///< the first of an agent's own topics
} // namespace topic

/// A net: a radio network the range makes (AB-4: the link model's terms, the radio path's line of sight, a shared data
/// rate; MS-11's terminals replace them with link budgets, slots and jamming). The defaults stand for a Link 16 net:
/// its published normal-mode range (300 NM) and band (960-1215 MHz) and its packed-2 single-pulse rate (57.6 kbit/s);
/// the latency, jitter, queue, retry and size limits are estimates.
struct NetSpec {
    std::string name;
    double rangeM = 555600.0;        ///< the longest slant range a message closes
    double frequencyHz = 1.0e9;      ///< for the radio path
    bool lineOfSight = true;         ///< the earth's curvature and the terrain mask it (MS-2's radio path)
    double latencyS = 0.1;           ///< (estimate) from the end of its transmission to its delivery
    double jitterS = 0.05;           ///< (estimate) uniform, +-
    double lossProbability = 0.0;    ///< each attempt to each receiver
    double dataRateBps = 57600.0;    ///< the net's: shared evenly by its members (each transmits in its share)
    double maxQueueS = 2.0;          ///< (estimate) a message that would wait longer for its sender's share is dropped
    double reliableTimeoutS = 10.0;  ///< (estimate) a reliable send fails if not delivered by then
    double retryS = 0.5;             ///< (estimate) a reliable send's next attempt after a failed one
    std::uint32_t maxPayloadBytes = 1024; ///< (estimate) the largest payload
    std::uint32_t headerBytes = 64;  ///< (estimate) the wrapper and header each message carries over the air
};

/// What a sender gives its DMS (MA_TxDataPayloadCommandMT): a topic, its payload, and optionally destinations - then
/// sent reliably to each (else published to whoever subscribed).
struct DmsMessage {
    Topic topic = topic::kFreeForm;
    std::vector<std::uint8_t> bytes;
    std::vector<SystemId> destinations;
};

/// Why a send was refused.
enum class DmsReason : std::uint8_t {
    None = 0,
    NoSystem,      ///< the sender is on no net
    ReservedTopic, ///< a native type's topic: the MA core's to send (AB-4b)
    TooLarge,      ///< larger than a net it would travel on allows
    UnknownSystem, ///< a destination no net it is on reaches
};

/// What arrived (MA_RxDataPayloadMT): its sender as the DMS stamped it, once however many paths it came by.
struct DmsReceived {
    std::uint64_t message = 0; ///< the sender's numbering
    SystemId source = kNoSystem;
    Topic topic = 0;
    NetId net = 0;             ///< the net it came by
    double timeSent = 0.0, timeDelivered = 0.0;
    std::vector<std::uint8_t> bytes;
};

enum class Delivery : std::uint8_t { Pending = 0, Delivered, Failed };

/// A send's status (MA_TxDataPayloadCommandStatusMT): each destination's, for a reliable one; how many it reached and
/// whether its sender's share could take it, for a published one.
struct DmsTransmission {
    std::uint64_t message = 0;
    Topic topic = 0;
    double timeSent = 0.0;
    bool published = false;
    std::uint32_t reached = 0;   ///< receivers it was delivered to
    bool queueDropped = false;   ///< (published) no net it was on could take it in time
    std::vector<std::pair<SystemId, Delivery>> destinations;
};

/// A net as its members know it.
struct NetInfo {
    NetId id = 0;
    std::string name;
    NetSpec spec;
};

/// A net's traffic (the referee's).
struct NetStats {
    std::uint64_t transmissions = 0;  ///< a sender's transmissions on it, retries among them
    std::uint64_t delivered = 0;      ///< messages delivered by it (first arrivals)
    std::uint64_t duplicates = 0;     ///< arrivals another net delivered first
    std::uint64_t outOfRange = 0, masked = 0, lost = 0; ///< attempts to a receiver that failed, by why
    std::uint64_t queueDropped = 0;   ///< transmissions its sender's share could not take in time
    std::uint64_t failed = 0;         ///< reliable sends to a destination that timed out
    std::uint64_t overflowed = 0;     ///< messages it delivered that a full inbox dropped (its oldest waiting)
};

/// (estimate) A system's inbox: what arrived waits until its MA takes it, up to this many - the oldest dropped first.
inline constexpr std::size_t kInboxCapacity = 1024;

// --- A-GRA's native messages (AB-4b): sent by the MA core from its platform's own reports, never by an agent's bytes --
//
// On the air each is its fixed little-endian layout below (a size standing for its encoded UCI message: an estimate);
// a receiver's DMS builds its picture from them, and its inbox holds them too (their bytes, decodable with decode()).

/// SystemStatusMT (Peer 1.2.6.3): a heartbeat - its tail number.
struct SystemStatusReport {
    double timeS = 0.0;
    char tailNumber[24] = {};
};

/// MA_PositionReportMT (Peer 1.2.6.2): its own ship as its navigation and air data report it.
struct PositionReport {
    double timeS = 0.0;
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeMslM = 0.0;
    double velocityNedMs[3] = {0, 0, 0}; ///< ground velocity north, east and down
    double headingRad = 0.0;
    double airspeedTrueMs = 0.0, airspeedCalibratedMs = 0.0, mach = 0.0; ///< its domain velocity's size, CAS, Mach
};

/// EntityMT (Peer 1.2.5.4): one of its radar's tracks, un-fused - its own numbering, never the entity behind it.
struct EntityReport {
    std::uint64_t track = 0;
    std::uint32_t payload = 0;           ///< its radar
    std::uint32_t updates = 0;
    double timeS = 0.0;                  ///< the time its estimate is for
    double latitudeRad = 0.0, longitudeRad = 0.0, heightM = 0.0;
    double velocityNedMs[3] = {0, 0, 0};
    double positionSigmaM = 0.0, velocitySigmaMs = 0.0;
};

template <typename T>
std::vector<std::uint8_t> encode(const T& report) {
    static_assert(std::is_trivially_copyable_v<T>, "a native report is trivially copyable");
    std::vector<std::uint8_t> b(sizeof(T));
    std::memcpy(b.data(), &report, sizeof(T));
    return b;
}
template <typename T>
bool decode(const std::vector<std::uint8_t>& bytes, T& out) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "a native report is trivially copyable");
    if (bytes.size() != sizeof(T)) return false;
    std::memcpy(&out, bytes.data(), sizeof(T));
    return true;
}

/// A system's Data Dissemination Plan (the range's, from mission planning: MS 1.2.5): what its MA core publishes on
/// its own, and how long its picture keeps what it heard. A period of 0 publishes nothing.
struct DisseminationPlan {
    double positionPeriodS = 0.0; ///< its position report
    double statusPeriodS = 0.0;   ///< its heartbeat
    double tracksPeriodS = 0.0;   ///< its radars' tracks, each its own entity report
    double peerTimeoutS = 30.0;   ///< (estimate) a peer's position report is dropped from its picture after this long
    double trackTimeoutS = 12.0;  ///< (estimate) a received track is dropped after this long without a report
};

/// What a system's picture holds of a peer: its last position report, as it arrived.
struct PeerReport {
    SystemId system = kNoSystem;
    NetId net = 0;
    double receivedS = 0.0;
    std::string tailNumber; ///< from its last heartbeat (empty: none heard)
    bool hasPosition = false;
    PositionReport position;
};

/// A track received over a link (a network track): its source system's report, as it arrived.
struct LinkTrack {
    SystemId source = kNoSystem;
    NetId net = 0;
    double receivedS = 0.0;
    EntityReport report;
};

// --- C2 authority (AB-5; A-GRA's C2 volume 1.2.7, 1.2.9): RBAC, control designations, direct flight commands ---------
//
// Topics 10 to 49 travel to a system's MA core, which authorizes each by its sender's declared role before anything
// happens; 50 to 99 are the MA core's answers, delivered to the sender's inbox (and its picture of command statuses).

namespace topic {
inline constexpr Topic kOperator = 10;           ///< OperatorMT: a C2 system declares its role (C2 1.2.9.4)
inline constexpr Topic kControlDesignation = 11; ///< MA_FlightCapabilityMT + ControlRequestMT: the flight modes MA may command (1.2.7.23)
inline constexpr Topic kFlightCommand = 12;      ///< MA_FlightCommandMT: a direct HSA/CSA or waypoint command, or its cancel (1.2.7.8-9)
inline constexpr Topic kTask = 13;              ///< MA_TaskMT and its command: a task for MA's agent to decide on (AB-5b)
inline constexpr Topic kOverride = 14;           ///< MA_PlatformOverrideCommandMT: a speed or altitude override, or its cancel (1.2.12.1)
inline constexpr Topic kConsentAnswer = 15;      ///< StrikeConsentRequestStatusMT: release consent granted or refused (1.2.15.6)
inline constexpr Topic kLostComms = 16;          ///< MA_PlanningFunctionSettingsCommandMT's CommsLost trigger and its response (1.2.4.3)
inline constexpr Topic kPackage = 17;            ///< a team: its leader and members, or its dissolution (Peer 1.2.11)
inline constexpr Topic kToCoreLast = 49;
inline constexpr Topic kOperatorRole = 50;       ///< MA_OperatorRoleMT: a role's C2 systems, broadcast on a change (1.2.9.6)
inline constexpr Topic kCommandStatus = 51;      ///< a C2 command's status: accepted, rejected, unauthorized; then how it ended
inline constexpr Topic kConsentRequest = 52;     ///< StrikeConsentRequestMT: a strike asks the approving roles for release consent
} // namespace topic

/// What MA may do with a strike it decided on (MA_ApprovalPolicyMT, a strike task's): release without asking, never, or
/// only with the consent of a system whose role approves strikes (A-GRA's default).
enum class StrikePolicy : std::uint8_t { AutoApprove = 0, AutoReject = 1, RequiresApproval = 2 };

/// What a role may send an MA (A-GRA's MA_AuthorizationMT, by kind of message).
namespace permission {
inline constexpr std::uint32_t kFlight = 1u << 0;   ///< direct flight commands (ByRequirementExecution: FLIGHT capabilities)
inline constexpr std::uint32_t kControl = 1u << 1;  ///< control designations
inline constexpr std::uint32_t kTask = 1u << 2;     ///< tasks (AB-5b)
inline constexpr std::uint32_t kWeapons = 1u << 3;  ///< weapon capabilities: strike tasks, release consent, approvals of strikes (AB-5b)
inline constexpr std::uint32_t kOverride = 1u << 4; ///< platform overrides (AB-5b)
inline constexpr std::uint32_t kSettings = 1u << 5; ///< settings: lost communication's (AB-5c)
inline constexpr std::uint32_t kPackage = 1u << 6;  ///< teams (AB-5c)
inline constexpr std::uint32_t kRoles = 1u << 7;    ///< role configuration in flight (Admin's alone)
inline constexpr std::uint32_t kAll = 0xFFu;
} // namespace permission

/// A role (MA_OperatorRoleMT) and what it may send (MA_AuthorizationMT).
struct Role {
    std::uint32_t id = 0;
    std::string name;
    std::uint32_t permissions = 0; ///< permission bits
    bool approvesStrikes = false;  ///< its systems are asked for strikes' release consent (MA_ApprovalAuthorityMT; AB-5b)
};

/// A system's RBAC configuration (mission planning's: the range loads it). By default A-GRA's five roles: Admin (1)
/// everything, QB (2) everything but role configuration, approving strikes; AVC (3) flight, control, tasks, overrides,
/// settings, teams - not weapons; LRE (4) flight commands; Observer (5) nothing. A-GRA describes the roles and leaves
/// their exact permission table to its model: these sets are estimates from its descriptions.
struct RbacConfig {
    std::vector<Role> roles;
    StrikePolicy strikePolicy = StrikePolicy::RequiresApproval; ///< (AB-5b) its strikes' release consent
    static RbacConfig agraDefaults() {
        RbacConfig c;
        c.roles = {{1, "Admin", permission::kAll, false},
                   {2, "QB", permission::kAll & ~permission::kRoles, true},
                   {3, "AVC", permission::kFlight | permission::kControl | permission::kTask | permission::kOverride | permission::kSettings |
                                  permission::kPackage, false},
                   {4, "LRE", permission::kFlight, false},
                   {5, "Observer", 0u, false}};
        return c;
    }
};

/// OperatorMT: the role its sender takes, and its callsign.
struct OperatorDeclaration {
    std::uint32_t roleId = 0;
    char callsign[28] = {};
};

/// MA_OperatorRoleMT: a role's C2 systems, as an MA knows them.
struct OperatorRoleReport {
    std::uint32_t roleId = 0;
    std::uint32_t count = 0;
    SystemId systems[16] = {};
    char name[24] = {};
};

/// A control designation (MA_FlightCapabilityMT and ControlRequestMT, MA the controller, FA the controllee): the flight
/// modes MA may command, a bit per control::FlightMode (1 << mode).
struct ControlDesignation {
    std::uint64_t commandId = 0;
    std::uint32_t modes = 0;
    std::uint32_t reserved = 0;
};

/// MA_FlightCommandMT: a direct command - HSA/CSA, or waypoints - or the cancel of one C2 sent.
struct FlightCommand {
    enum Kind : std::uint32_t { Hsa = 0, Waypoints = 1, Cancel = 2 };
    std::uint64_t commandId = 0;           ///< the sender's; a cancel names the command it cancels
    std::uint32_t kind = Hsa;
    std::uint32_t pointCount = 0;
    double headingRad = 0, courseRad = 0, speed = 0, speedReference = 0, altitudeM = 0, altitudeReference = 0; ///< control::kHold leaves one out
    double points[16][4] = {};             ///< latitude, longitude (rad), altitude (m), speed (m/s; kHold: left out)
};

/// The MA core's answer to a C2 command (A-GRA's Command-2 status messages): first accepted, rejected or unauthorized;
/// a flight command's activity later, how it ended.
struct CommandStatus {
    enum State : std::uint32_t { Accepted = 0, Rejected = 1, Unauthorized = 2, Completed = 3, Failed = 4, Canceled = 5 };
    std::uint64_t commandId = 0;
    std::uint32_t topic = 0;    ///< the command's topic
    std::uint32_t state = Accepted;
    std::uint32_t reason = 0;   ///< control::Reason; a task's: the agent's own code
    std::uint32_t reserved = 0;
    std::uint64_t activity = 0; ///< a flight command's activity
    double timeS = 0.0;
    double percent = std::numeric_limits<double>::quiet_NaN(); ///< a task's progress, as its agent reports it (AB-5b)
};

/// MA_TaskMT and its command (AB-5b; C2 1.2.13): a task for the MA's agent - which accepts or rejects it with a reason,
/// plans its execution and reports its progress. Its target, if any, is a network track (its source system, radar and
/// number), never an entity.
struct TaskOrder {
    enum Type : std::uint32_t { Patrol = 0, Strike = 1, Reconnaissance = 2, Escort = 3, Transit = 4, Other = 5 };
    std::uint64_t taskId = 0;    ///< the sender's
    std::uint32_t type = Patrol; ///< a Strike needs the weapons permission too
    std::uint32_t priority = 0;
    double latitudeRad = std::numeric_limits<double>::quiet_NaN(), longitudeRad = std::numeric_limits<double>::quiet_NaN();
    double altitudeM = std::numeric_limits<double>::quiet_NaN(), radiusM = std::numeric_limits<double>::quiet_NaN();
    double startS = std::numeric_limits<double>::quiet_NaN(), endS = std::numeric_limits<double>::quiet_NaN(); ///< its window
    SystemId targetSource = kNoSystem;
    std::uint32_t targetPayload = 0;
    std::uint64_t targetTrack = 0;
    char text[64] = {};          ///< free text: its remarks
};

/// A task as the agent holds it.
struct ReceivedTask {
    enum State : std::uint8_t { Received = 0, Accepted, Rejected, Completed, Failed };
    SystemId from = kNoSystem;
    double receivedS = 0.0;
    State state = Received;
    double percent = std::numeric_limits<double>::quiet_NaN();
    TaskOrder order;
};

/// MA_PlatformOverrideCommandMT (C2 1.2.12.1): an overriding speed or altitude for MA's flight, until canceled or its
/// expiry; MA's own commands fly it in place of theirs.
struct OverrideCommand {
    enum Kind : std::uint32_t { Speed = 0, Altitude = 1 };
    std::uint64_t commandId = 0;
    std::uint32_t kind = Speed;
    std::uint32_t cancel = 0;    ///< 1: clear the override of this kind
    double value = 0.0;          ///< m/s, or m above mean sea level
    double expiryS = std::numeric_limits<double>::quiet_NaN(); ///< NaN: until canceled
};

/// An override in force, as the agent reads it.
struct ActiveOverride {
    SystemId from = kNoSystem;
    std::uint32_t kind = OverrideCommand::Speed;
    double value = 0.0;
    double expiryS = std::numeric_limits<double>::quiet_NaN();
};

/// StrikeConsentRequestMT (C2 1.2.15.6): a strike asking for release consent - what, how many, at what (a track: its
/// source system, radar and number), from where.
struct ConsentRequest {
    std::uint64_t requestId = 0;
    std::uint64_t strike = 0;      ///< the strike's activity
    char store[24] = {};           ///< its store type
    std::uint32_t quantity = 0;
    std::uint32_t targetPayload = 0;
    SystemId targetSource = kNoSystem; ///< the system whose track it is (the MA's own, or a network track's source)
    std::uint32_t reserved = 0;
    std::uint64_t targetTrack = 0;
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeM = 0.0; ///< the launcher, as its navigation reports it
    double timeS = 0.0;
};

/// StrikeConsentRequestStatusMT: the answer.
struct ConsentAnswer {
    std::uint64_t requestId = 0;
    std::uint32_t granted = 0;
    std::uint32_t reserved = 0;
};

/// The lost-communication contingency C2 configures (C2 1.2.4.3; AB-5c): hearing nothing from C2 for `thresholdS`, MA
/// raises a contingency alert - and, `activatePlan`, activates the route plan it holds by that id.
struct LostCommsSettings {
    std::uint64_t commandId = 0;
    double thresholdS = 60.0;
    std::uint32_t activatePlan = 0; ///< 1: activate `planId` too (else the alert alone)
    std::uint32_t reserved = 0;
    std::uint64_t planId = 0;
};

/// A team (Peer 1.2.11; AB-5c): its leader and members. A member's MA core accepts what its leader sends as C2's (A-GRA:
/// members accept any message from their team's leader - C2's commands relayed), and from authorized C2 alone besides.
struct PackageCommand {
    std::uint64_t commandId = 0;
    std::uint32_t packageId = 0;
    std::uint32_t dissolve = 0;
    SystemId leader = kNoSystem;
    std::uint32_t count = 0;
    SystemId members[16] = {};
};

/// A consent request as an approving C2 system holds it.
struct PendingConsent {
    SystemId from = kNoSystem; ///< the MA asking
    double receivedS = 0.0;
    bool answered = false;
    ConsentRequest request;
};

/// A command's last status, as its sender's picture keeps it.
struct CommandStatusEntry {
    SystemId from = kNoSystem; ///< the MA that answered
    double receivedS = 0.0;
    CommandStatus status;
};

/// A C2 node (ADR-31, section 9 item 2): a compute and communications node with a position - no flight model, no VI or
/// MS. The range makes it, puts it on nets, and may make an agent for it.
struct C2NodeSpec {
    std::string name;
    double latitudeRad = 0.0, longitudeRad = 0.0, altitudeMslM = 0.0;
};

/// The world's message fabric, stepped with the simulation.
class FSIM_API Network {
public:
    Network();
    ~Network();

    Node& createNode(Address address, std::uint32_t vehicleId = 0);
    void removeNode(Address address);
    Node* node(Address address) noexcept;
    const Node* node(Address address) const noexcept;

    void setMedium(std::unique_ptr<Medium> medium) { medium_ = std::move(medium); }
    Medium& medium() noexcept { return *medium_; }
    void attach(Address address, std::unique_ptr<Protocol> protocol);

    /// Queue a message from a node. `timeSent` is stamped here.
    void send(Message message);

    /// Run protocols, route queued messages, deliver due ones into inboxes.
    void step(double simTime, double dt, const control::WorldView* world, Rng& rng);

    std::uint64_t routed() const noexcept { return routed_; }
    std::uint64_t dropped() const noexcept { return dropped_; }

private:
    struct InFlight {
        Message message;
        Address to;
        double deliverAt;
    };
    struct Entry {
        std::unique_ptr<Node> node;
        std::vector<std::unique_ptr<Protocol>> protocols;
    };
    std::vector<Entry> nodes_;
    std::vector<Message> outbox_;
    std::vector<InFlight> inflight_;
    std::unique_ptr<Medium> medium_;
    double now_ = 0.0;
    std::uint64_t routed_ = 0, dropped_ = 0;
};

} // namespace fsim::comm
