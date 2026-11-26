// SIM-M08 (MS-12): External OMS / A-GRA / UCI Protocol Adapter fixtures and acceptance tests.

#include "adapters/oms/OmsAdapter.h"
#include "adapters/oms/OmsSchema.h"
#include "adapters/oms/OmsTransport.h"
#include "foundation/core/Json.h"
#include "fsim/Agent.h"
#include "fsim/World.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using fsim::core::Json;
using namespace fsim::oms;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

fsim::WorldOptions makeWorldOptions() {
    fsim::WorldOptions opts;
    opts.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    opts.publish = false;
    opts.workers = 1;
    opts.pinWorkers = false;
    opts.seed = 42;
    return opts;
}

fsim::VehicleSpec makeVehicle(const char* type,
                              const char* name,
                              double latDeg,
                              double lonDeg,
                              double altM,
                              double headingDeg,
                              double speedMs) {
    fsim::VehicleSpec spec;
    spec.type = std::string("jsbsim:") + type;
    spec.name = name;
    spec.initial.latitudeDeg = latDeg;
    spec.initial.longitudeDeg = lonDeg;
    spec.initial.altitudeMslM = altM;
    spec.initial.headingDeg = headingDeg;
    spec.initial.airspeedTrueMs = speedMs;
    return spec;
}

std::optional<std::uint16_t> findPayload(const fsim::Agent& agent, fsim::ms::Family family) {
    for (const auto& p : agent.payloads()) {
        if (p.family == family) return p.index;
    }
    return std::nullopt;
}

const OmsEnvelope* findLastByType(const std::vector<OmsEnvelope>& msgs, MessageType type) {
    for (auto it = msgs.rbegin(); it != msgs.rend(); ++it) {
        if (it->messageType == type) return &(*it);
    }
    return nullptr;
}

} // namespace

TEST_CASE("SIM-M08 framing and schema validation rejects malformed, oversized, unknown-field, and non-decimal 64-bit envelopes",
          "[sim-m08][oms][schema]") {
    SECTION("NDJSON and LengthPrefixed codecs survive arbitrary chunk fragmentation") {
        const std::string p1 = R"({"a":1,"id":"9007199254740993"})";
        const std::string p2 = R"({"b":2,"id":"18446744073709551615"})";

        for (FrameMode mode : {FrameMode::Ndjson, FrameMode::LengthPrefixed}) {
            std::string f1, f2;
            FrameError err = FrameError::None;
            REQUIRE(encodeFrame(p1, mode, f1, &err));
            REQUIRE(err == FrameError::None);
            REQUIRE(encodeFrame(p2, mode, f2, &err));

            const std::string stream = f1 + f2;
            FrameDecoder dec(mode);
            std::vector<std::string> frames;
            for (std::size_t pos = 0; pos < stream.size(); pos += 3) {
                const std::size_t n = std::min<std::size_t>(3, stream.size() - pos);
                REQUIRE(dec.feed(std::string_view(stream.data() + pos, n), frames, &err));
                REQUIRE(err == FrameError::None);
            }
            REQUIRE(dec.bufferedBytes() == 0);
            REQUIRE(frames.size() == 2);
            REQUIRE(frames[0] == p1);
            REQUIRE(frames[1] == p2);
        }
    }

    SECTION("Framing rejects empty, newline-corrupted, and oversized payloads") {
        std::string out;
        FrameError err = FrameError::None;
        REQUIRE_FALSE(encodeFrame("", FrameMode::Ndjson, out, &err));
        REQUIRE(err == FrameError::EmptyPayload);

        REQUIRE_FALSE(encodeFrame("{\"a\":1}\n", FrameMode::Ndjson, out, &err));
        REQUIRE(err == FrameError::MalformedFrame);

        REQUIRE_FALSE(encodeFrame(std::string(128, 'x'), FrameMode::Ndjson, out, &err, 64));
        REQUIRE(err == FrameError::OversizedFrame);

        FrameDecoder dec(FrameMode::LengthPrefixed, 32);
        std::string badLen(4, '\0');
        badLen[3] = 64; // exceeds maxFrameBytes = 32
        std::vector<std::string> frames;
        REQUIRE_FALSE(dec.feed(badLen, frames, &err));
        REQUIRE(err == FrameError::OversizedFrame);
    }

    SECTION("Strict 64-bit unsigned decimal string parser rejects invalid forms and preserves >2^53") {
        std::uint64_t val = 0;
        REQUIRE(parseUint64Strict("0", val));
        REQUIRE(val == 0u);
        REQUIRE(parseUint64Strict("9007199254740993", val));
        REQUIRE(val == 9007199254740993ull);
        REQUIRE(parseUint64Strict("18446744073709551615", val));
        REQUIRE(val == UINT64_MAX);

        REQUIRE_FALSE(parseUint64Strict("", val));
        REQUIRE_FALSE(parseUint64Strict("-1", val));
        REQUIRE_FALSE(parseUint64Strict("0x10", val));
        REQUIRE_FALSE(parseUint64Strict(" 42", val));
        REQUIRE_FALSE(parseUint64Strict("18446744073709551616", val)); // overflow
    }

    SECTION("Envelope parser enforces version, external_oms domain, agent_only scope, and closed keys") {
        OmsEnvelope validEnv;
        validEnv.messageType = MessageType::ServiceHeartbeatMT;
        validEnv.messageId = 9007199254740993ull;
        validEnv.correlationId = 18446744073709551615ull;
        validEnv.sourceServiceId = 4242ull;
        validEnv.timestampS = 12.5;
        validEnv.bodyJson = makeServiceHeartbeatBody(4242ull, 7ull, ServiceHealth::Operational);

        std::string encoded, err;
        REQUIRE(serializeEnvelope(validEnv, encoded, &err));

        OmsEnvelope decoded;
        REQUIRE(parseEnvelope(encoded, decoded, &err));
        REQUIRE(decoded.messageId == 9007199254740993ull);
        REQUIRE(decoded.correlationId == 18446744073709551615ull);
        REQUIRE(decoded.sourceServiceId == 4242ull);
        REQUIRE(decoded.transportDomain == "external_oms");
        REQUIRE(decoded.authorityScope == "agent_only");

        // Wrong schema version
        REQUIRE_FALSE(parseEnvelope(
            R"({"schema":"fsim.oms.v99","transport_domain":"external_oms","authority_scope":"agent_only","message_type":"ServiceHeartbeatMT","message_id":"1","correlation_id":"0","source_service_id":"1","timestamp_s":0.0,"body":{}})",
            decoded, &err));

        // Wrong transport_domain (attempting to spoof modeled RF DMS)
        REQUIRE_FALSE(parseEnvelope(
            R"({"schema":"fsim.oms.v1","transport_domain":"modeled_rf_dms","authority_scope":"agent_only","message_type":"ServiceHeartbeatMT","message_id":"1","correlation_id":"0","source_service_id":"1","timestamp_s":0.0,"body":{}})",
            decoded, &err));

        // Wrong authority_scope (attempting to claim referee scope)
        REQUIRE_FALSE(parseEnvelope(
            R"({"schema":"fsim.oms.v1","transport_domain":"external_oms","authority_scope":"referee","message_type":"ServiceHeartbeatMT","message_id":"1","correlation_id":"0","source_service_id":"1","timestamp_s":0.0,"body":{}})",
            decoded, &err));

        // Unknown top-level field rejected
        REQUIRE_FALSE(parseEnvelope(
            R"({"schema":"fsim.oms.v1","transport_domain":"external_oms","authority_scope":"agent_only","message_type":"ServiceHeartbeatMT","message_id":"1","correlation_id":"0","source_service_id":"1","timestamp_s":0.0,"body":{},"extra":123})",
            decoded, &err));

        // Command body with command_id = "0" rejected
        ParsedCommand cmd;
        REQUIRE_FALSE(parseCommandBody(MessageType::SubsystemBITCommandMT,
                                       R"({"command_id":"0","payload":0})", cmd, &err));
        // Command body with unknown field rejected
        REQUIRE_FALSE(parseCommandBody(MessageType::SubsystemBITCommandMT,
                                       R"({"command_id":"1","payload":0,"inject_consent":true})", cmd, &err));
    }
}

TEST_CASE("SIM-M08 service registration, periodic heartbeat, peer watchdog degradation/recovery, and disconnect",
          "[sim-m08][oms][heartbeat]") {
    fsim::World world(makeWorldOptions());
    auto f35 = world.createVehicle(makeVehicle("f35a", "oms_f35", 35.0, -118.0, 6000.0, 0.0, 220.0));
    auto agent = world.createAgent(f35, "oms_agent");
    REQUIRE(agent.valid());

    auto [adapterPort, clientPort] = LoopbackOmsTransport::createPair();
    OmsAdapterConfig cfg;
    cfg.serviceId = 2001;
    cfg.serviceName = "f35a.oms.adapter";
    cfg.heartbeatPeriodS = 1.0;
    cfg.peerHeartbeatTimeoutS = 3.0;

    OmsServiceAdapter adapter(agent, *adapterPort, cfg);
    OmsClientPeer client(*clientPort, 9001);

    REQUIRE(adapter.registerService(0.0));
    std::vector<std::string> parseErrs;
    auto msgs = client.receiveAll(&parseErrs);
    REQUIRE(parseErrs.empty());
    REQUIRE(msgs.size() >= 5);
    REQUIRE(msgs[0].messageType == MessageType::ServiceRegistrationMT);
    REQUIRE(msgs[0].transportDomain == "external_oms");
    REQUIRE(msgs[0].authorityScope == "agent_only");
    REQUIRE(findLastByType(msgs, MessageType::ServiceHeartbeatMT) != nullptr);
    REQUIRE(findLastByType(msgs, MessageType::ServiceStatusMT) != nullptr);
    REQUIRE(findLastByType(msgs, MessageType::SubsystemStatusMT) != nullptr);
    REQUIRE(findLastByType(msgs, MessageType::CapabilityStatusMT) != nullptr);

    // Peer sends heartbeat at t = 0.5 -> peer health becomes OPERATIONAL.
    REQUIRE(client.sendHeartbeat(0.5, ServiceHealth::Operational));
    REQUIRE(adapter.poll(0.5) == 1);
    REQUIRE(adapter.peerHealth() == ServiceHealth::Operational);
    REQUIRE(adapter.peerServiceId() == 9001);

    // Periodic own heartbeat fires at t = 1.0 and t = 2.0.
    adapter.poll(1.0);
    adapter.poll(2.0);
    REQUIRE(adapter.stats().heartbeatsSent == 3); // initial + t=1.0 + t=2.0

    // Starve peer past timeout (0.5 + 3.0 = 3.5) -> DEGRADED at t = 4.0.
    adapter.poll(4.0);
    REQUIRE(adapter.peerHealth() == ServiceHealth::Degraded);
    auto statusMsgs = client.receiveAll(&parseErrs);
    REQUIRE(parseErrs.empty());
    const auto* degradedStatus = findLastByType(statusMsgs, MessageType::ServiceStatusMT);
    REQUIRE(degradedStatus != nullptr);
    REQUIRE(Json::parse(degradedStatus->bodyJson).string("peer_health", "") == "DEGRADED");

    // Starve past 2 * timeout (0.5 + 6.0 = 6.5) -> FAULTED at t = 7.0.
    adapter.poll(7.0);
    REQUIRE(adapter.peerHealth() == ServiceHealth::Faulted);

    // Peer recovers with fresh heartbeat at t = 7.5 -> OPERATIONAL.
    REQUIRE(client.sendHeartbeat(7.5, ServiceHealth::Operational));
    REQUIRE(adapter.poll(7.5) == 1);
    REQUIRE(adapter.peerHealth() == ServiceHealth::Operational);

    // Transport disconnect transitions adapter offline.
    clientPort->close();
    adapter.poll(8.0);
    REQUIRE(adapter.ownHealth() == ServiceHealth::Offline);
    REQUIRE(adapter.peerHealth() == ServiceHealth::Offline);
}

TEST_CASE("SIM-M08 F-35A optical/radar command-response, BIT, idempotency, conflict rejection, and truth redaction",
          "[sim-m08][oms][f35a]") {
    fsim::World world(makeWorldOptions());
    auto f35 = world.createVehicle(makeVehicle("f35a", "blue_f35", 35.0, -118.0, 6000.0, 0.0, 230.0));
    // Tail-chase geometry: F-16C ~8 km ahead of F-35A on same heading so it remains inside forward FOR
    // throughout payload warm-up, BIT, passive IR search, radar tracking, and LRF ranging.
    auto f16 = world.createVehicle(makeVehicle("f16c", "red_f16", 35.072, -118.0, 6000.0, 0.0, 230.0));

    fsim::control::HsaCommand levelNorth;
    levelNorth.headingRad = 0.0;
    levelNorth.altitudeM = 6000.0;
    levelNorth.speed = 230.0;
    REQUIRE(f35.submit(levelNorth).accepted());
    REQUIRE(f16.submit(levelNorth).accepted());

    auto agent = world.createAgent(f35, "blue_ma");
    auto referee = world.referee();
    REQUIRE(agent.valid());

    const auto optIdx = findPayload(agent, fsim::ms::Family::PassiveOptical);
    const auto radarIdx = findPayload(agent, fsim::ms::Family::AirRadar);
    REQUIRE(optIdx.has_value());
    REQUIRE(radarIdx.has_value());

    auto [adapterPort, clientPort] = LoopbackOmsTransport::createPair();
    OmsServiceAdapter adapter(agent, *adapterPort);
    OmsClientPeer client(*clientPort, 7001);
    REQUIRE(adapter.registerService(world.time()));
    (void)client.receiveAll();

    // 1. Native refusal propagation: PO_SEARCH while EOTS is Off -> payload_off.
    REQUIRE(client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                               R"({"command_id":"100","payload":)" + std::to_string(*optIdx) +
                                   R"(,"kind":"PO_SEARCH"})"));
    REQUIRE(adapter.poll(world.time()) == 1);
    {
        auto resps = client.receiveAll();
        REQUIRE(resps.size() == 1);
        REQUIRE(resps[0].messageType == MessageType::CommandResponseMT);
        const Json body = Json::parse(resps[0].bodyJson);
        REQUIRE(body.string("command_id", "") == "100");
        REQUIRE(body.string("status", "") == "REJECTED");
        REQUIRE(body.string("reason", "") == "payload_off");
        REQUIRE_FALSE(body.boolean("duplicate", true));
    }

    // 2. Power on EOTS and Radar using 64-bit command_id > 2^53.
    const std::string bigCmdId = "9007199254740993";
    const std::string powerEotsBody =
        R"({"command_id":")" + bigCmdId + R"(","payload":)" + std::to_string(*optIdx) + R"(,"state":"OPERATE"})";
    REQUIRE(client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(), powerEotsBody));
    REQUIRE(client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(),
                               R"({"command_id":"9007199254740994","payload":)" + std::to_string(*radarIdx) +
                                   R"(,"state":"OPERATE"})"));
    REQUIRE(adapter.poll(world.time()) == 2);
    std::string eotsPowerActId;
    {
        auto resps = client.receiveAll();
        REQUIRE(resps.size() == 2);
        const Json b0 = Json::parse(resps[0].bodyJson);
        REQUIRE(b0.string("command_id", "") == bigCmdId);
        REQUIRE(b0.string("status", "") == "ACCEPTED");
        REQUIRE_FALSE(b0.boolean("duplicate", true));
        eotsPowerActId = b0.string("activity_id", "0");
        REQUIRE(eotsPowerActId != "0");
    }

    // 3. Idempotent replay of bigCmdId returns cached response with duplicate=true.
    REQUIRE(client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(), powerEotsBody));
    REQUIRE(adapter.poll(world.time()) == 1);
    {
        auto resps = client.receiveAll();
        REQUIRE(resps.size() == 1);
        const Json b = Json::parse(resps[0].bodyJson);
        REQUIRE(b.string("command_id", "") == bigCmdId);
        REQUIRE(b.string("status", "") == "ACCEPTED");
        REQUIRE(b.string("activity_id", "") == eotsPowerActId);
        REQUIRE(b.boolean("duplicate", false));
        REQUIRE(adapter.stats().duplicateCommands == 1);
    }

    // 4. Reusing bigCmdId with a different body is rejected as command_id_conflict.
    REQUIRE(client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(),
                               R"({"command_id":")" + bigCmdId + R"(","payload":)" + std::to_string(*optIdx) +
                                   R"(,"state":"OFF"})"));
    REQUIRE(adapter.poll(world.time()) == 1);
    {
        auto resps = client.receiveAll();
        REQUIRE(resps.size() == 1);
        const Json b = Json::parse(resps[0].bodyJson);
        REQUIRE(b.string("status", "") == "REJECTED");
        REQUIRE(b.string("reason", "") == "invalid_parameter");
        REQUIRE(b.string("reason_description", "") == "command_id_conflict");
        REQUIRE(adapter.stats().commandConflicts == 1);
    }

    // Warm up payloads (EOTS & Radar) to Operate (~35 s at 30 Hz world step).
    world.step(1050u);

    // 5. Run BIT on EOTS, verify acceptance and completion.
    REQUIRE(client.sendMessage(MessageType::SubsystemBITCommandMT, world.time(),
                               R"({"command_id":"200","payload":)" + std::to_string(*optIdx) + "}"));
    REQUIRE(adapter.poll(world.time()) == 1);
    {
        auto resps = client.receiveAll();
        const auto* resp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(resp != nullptr);
        REQUIRE(Json::parse(resp->bodyJson).string("status", "") == "ACCEPTED");
    }
    world.step(360u);

    // 6. Start AMTI volume search + Passive Optical search through external OMS commands.
    REQUIRE(client.sendMessage(MessageType::AMTI_CommandMT, world.time(),
                               R"({"command_id":"300","payload":)" + std::to_string(*radarIdx) +
                                   R"(,"mode":"RANGE_WHILE_SEARCH","task":"VOLUME","range_scale_m":80000.0})"));
    REQUIRE(client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                               R"({"command_id":"301","payload":)" + std::to_string(*optIdx) +
                                   R"(,"kind":"PO_SEARCH"})"));
    REQUIRE(adapter.poll(world.time()) == 2);
    std::string radarActId;
    {
        auto resps = client.receiveAll();
        REQUIRE(resps.size() >= 2);
        for (const auto& r : resps) {
            if (r.messageType != MessageType::CommandResponseMT) continue;
            const Json b = Json::parse(r.bodyJson);
            if (b.string("command_id", "") == "300") {
                REQUIRE(b.string("status", "") == "ACCEPTED");
                radarActId = b.string("activity_id", "0");
            } else if (b.string("command_id", "") == "301") {
                REQUIRE(b.string("status", "") == "ACCEPTED");
            }
        }
    }
    REQUIRE(radarActId != "0");

    // Step world to acquire passive IR contact and AMTI radar track.
    world.step(120u);
    REQUIRE(adapter.publishSensorReports(world.time()));
    auto reports = client.receiveAll();

    const auto* ownShipMsg = findLastByType(reports, MessageType::OwnShipMDT);
    REQUIRE(ownShipMsg != nullptr);
    REQUIRE(Json::parse(ownShipMsg->bodyJson).number("fuel_kg", 0.0) > 0.0);

    std::string irContactId;
    bool sawOpticalContact = false;
    bool sawRadarTrack = false;
    for (const auto& env : reports) {
        // Strict truth redaction: wire JSON must never carry a "truth" key!
        REQUIRE(env.bodyJson.find("\"truth\"") == std::string::npos);

        if (env.messageType == MessageType::ContactMT) {
            const Json body = Json::parse(env.bodyJson);
            if (static_cast<std::uint16_t>(body.number("payload", -1)) == *optIdx) {
                const auto& arr = body.child("contacts").asArray();
                REQUIRE_FALSE(arr.empty());
                sawOpticalContact = true;
                irContactId = arr[0].string("contact_id", "");
                REQUIRE_FALSE(irContactId.empty());
                // Passive IR search does NOT fabricate range -> must serialize as JSON null!
                const Json* rng = arr[0].find("range_m");
                REQUIRE(rng != nullptr);
                REQUIRE(rng->isNull());
            }
        } else if (env.messageType == MessageType::AMTI_TrackMT) {
            const Json body = Json::parse(env.bodyJson);
            if (static_cast<std::uint16_t>(body.number("payload", -1)) == *radarIdx) {
                sawRadarTrack = !body.child("tracks").asArray().empty();
            }
        }
    }
    REQUIRE(sawOpticalContact);
    REQUIRE(sawRadarTrack);
    REQUIRE(referee.vehicles().size() == 2);

    // 7. Issue PO_TRACK on the passive IR contact -> triggers LRF ranging on F-35A EOTS!
    REQUIRE(client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                               R"({"command_id":"302","payload":)" + std::to_string(*optIdx) +
                                   R"(,"kind":"PO_TRACK","contact_id":")" + irContactId + R"("})"));
    REQUIRE(adapter.poll(world.time()) == 1);
    (void)client.receiveAll();
    world.step(40u);
    REQUIRE(adapter.publishSensorReports(world.time()));

    bool sawLaserRange = false;
    for (const auto& env : client.receiveAll()) {
        if (env.messageType == MessageType::ContactMT) {
            const Json body = Json::parse(env.bodyJson);
            if (static_cast<std::uint16_t>(body.number("payload", -1)) == *optIdx) {
                for (const Json& c : body.child("contacts").asArray()) {
                    const Json* rng = c.find("range_m");
                    if (rng && rng->isNumber() && rng->asNumber() > 500.0) {
                        sawLaserRange = true;
                    }
                }
            }
        }
    }
    REQUIRE(sawLaserRange);

    // 8. Cancel the AMTI radar activity through ActivityCancelCommandMT and verify ActivityStatusMT.
    REQUIRE(client.sendMessage(MessageType::ActivityCancelCommandMT, world.time(),
                               R"({"command_id":"303","activity_id":")" + radarActId + R"("})"));
    REQUIRE(adapter.poll(world.time()) == 1);
    REQUIRE(adapter.publishStatus(world.time()));
    {
        auto resps = client.receiveAll();
        const auto* resp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(resp != nullptr);
        const Json b = Json::parse(resp->bodyJson);
        REQUIRE(b.string("status", "") == "CANCELED");
        REQUIRE(b.string("reason", "") == "none");

        const auto* actStatus = findLastByType(resps, MessageType::ActivityStatusMT);
        REQUIRE(actStatus != nullptr);
        bool foundCanceledRadar = false;
        const Json actBody = Json::parse(actStatus->bodyJson);
        for (const Json& a : actBody.child("activities").asArray()) {
            if (a.string("activity_id", "") == radarActId) {
                REQUIRE(a.string("state", "") == "canceled");
                REQUIRE(a.string("reason", "") == "requested");
                foundCanceledRadar = true;
            }
        }
        REQUIRE(foundCanceledRadar);
    }
}

TEST_CASE("SIM-M08 Granted control mode, LengthPrefixed Weather/SAR commands, and strict DMS separation",
          "[sim-m08][oms][ms11][dms]") {
    fsim::World world(makeWorldOptions());

    // Configure rain for C-130J weather radar and clear cross-track geometry for U-2S SAR.
    auto env = world.environment().state();
    env.precipitation = 0.8;
    world.environment().set(env);

    auto c130 = world.createVehicle(makeVehicle("c130j", "wx_c130", 35.0, -118.0, 4500.0, 0.0, 140.0));
    auto u2 = world.createVehicle(makeVehicle("u2s", "sar_u2", 35.0, -117.5, 18000.0, 0.0, 210.0));

    // Hold steady north heading on both aircraft so SAR dead-ahead vs cross-track geometry stays exact.
    fsim::control::HsaCommand c130Hold;
    c130Hold.headingRad = 0.0;
    c130Hold.altitudeM = 4500.0;
    c130Hold.speed = 140.0;
    REQUIRE(c130.submit(c130Hold).accepted());

    fsim::control::HsaCommand u2Hold;
    u2Hold.headingRad = 0.0;
    u2Hold.altitudeM = 18000.0;
    u2Hold.speed = 210.0;
    REQUIRE(u2.submit(u2Hold).accepted());

    // Put C-130J into Granted control mode and attach a modeled RF DMS net to prove strict separation.
    REQUIRE(c130.setControlMode(fsim::control::ControlMode::Granted) == fsim::control::Reason::None);
    fsim::comm::NetSpec netSpec;
    netSpec.name = "rf_tactical_net";
    const auto netId = world.createNet(netSpec);
    REQUIRE(netId != 0);
    REQUIRE(world.joinNet(c130.id(), netId));

    auto c130Agent = world.createAgent(c130, "c130_ma");
    auto u2Agent = world.createAgent(u2, "u2_ma");
    auto referee = world.referee();

    const auto wxIdx = findPayload(c130Agent, fsim::ms::Family::WeatherRadar);
    const auto sarIdx = findPayload(u2Agent, fsim::ms::Family::Sar);
    REQUIRE(wxIdx.has_value());
    REQUIRE(sarIdx.has_value());

    std::string wxStatusCap, wxCap;
    for (const auto& c : c130Agent.missionCapabilities()) {
        if (c.payload == *wxIdx && c.type == fsim::ms::CapabilityType::Status) wxStatusCap = c.id;
        if (c.payload == *wxIdx && c.type == fsim::ms::CapabilityType::WeatherRadar) wxCap = c.id;
    }
    REQUIRE_FALSE(wxStatusCap.empty());
    REQUIRE_FALSE(wxCap.empty());

    // Use LengthPrefixed framing for both adapters.
    auto [c130AdpPort, c130CliPort] = LoopbackOmsTransport::createPair();
    OmsAdapterConfig c130Cfg;
    c130Cfg.serviceId = 3001;
    c130Cfg.frameMode = FrameMode::LengthPrefixed;
    OmsServiceAdapter c130Adapter(c130Agent, *c130AdpPort, c130Cfg);
    OmsClientPeer c130Client(*c130CliPort, 8001, FrameMode::LengthPrefixed);
    REQUIRE(c130Adapter.registerService(world.time()));
    (void)c130Client.receiveAll();

    // Without control grant, SubsystemStateCommandMT is rejected with not_granted.
    REQUIRE(c130Client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(),
                                   R"({"command_id":"10","payload":)" + std::to_string(*wxIdx) +
                                       R"(,"state":"OPERATE"})"));
    REQUIRE(c130Adapter.poll(world.time()) == 1);
    {
        auto resps = c130Client.receiveAll();
        const auto* resp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(resp != nullptr);
        const Json b = Json::parse(resp->bodyJson);
        REQUIRE(b.string("status", "") == "REJECTED");
        REQUIRE(b.string("reason", "") == "not_granted");
    }

    // Request grants via ControlGrantCommandMT -> accepted -> weather radar powers on and searches.
    REQUIRE(c130Client.sendMessage(MessageType::ControlGrantCommandMT, world.time(),
                                   R"({"command_id":"11","capability":")" + wxStatusCap + R"(","action":"REQUEST"})"));
    REQUIRE(c130Client.sendMessage(MessageType::ControlGrantCommandMT, world.time(),
                                   R"({"command_id":"12","capability":")" + wxCap + R"(","action":"REQUEST"})"));
    REQUIRE(c130Client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(),
                                   R"({"command_id":"13","payload":)" + std::to_string(*wxIdx) +
                                       R"(,"state":"OPERATE"})"));
    REQUIRE(c130Adapter.poll(world.time()) == 3);
    for (const auto& envMsg : c130Client.receiveAll()) {
        if (envMsg.messageType == MessageType::CommandResponseMT) {
            REQUIRE(Json::parse(envMsg.bodyJson).string("status", "") == "ACCEPTED");
        }
    }

    // Power on U-2S SAR in parallel over LengthPrefixed OMS transport.
    auto [u2AdpPort, u2CliPort] = LoopbackOmsTransport::createPair();
    OmsAdapterConfig u2Cfg;
    u2Cfg.serviceId = 3002;
    u2Cfg.frameMode = FrameMode::LengthPrefixed;
    OmsServiceAdapter u2Adapter(u2Agent, *u2AdpPort, u2Cfg);
    OmsClientPeer u2Client(*u2CliPort, 8002, FrameMode::LengthPrefixed);
    REQUIRE(u2Adapter.registerService(world.time()));
    (void)u2Client.receiveAll();

    REQUIRE(u2Client.sendMessage(MessageType::SubsystemStateCommandMT, world.time(),
                                 R"({"command_id":"20","payload":)" + std::to_string(*sarIdx) +
                                     R"(,"state":"OPERATE"})"));
    REQUIRE(u2Adapter.poll(world.time()) == 1);
    (void)u2Client.receiveAll();

    world.step(1050u);

    // C-130J WEATHER_SEARCH succeeds and yields Weather contacts.
    REQUIRE(c130Client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                                   R"({"command_id":"14","payload":)" + std::to_string(*wxIdx) +
                                       R"(,"kind":"WEATHER_SEARCH","range_scale_m":60000.0})"));
    REQUIRE(c130Adapter.poll(world.time()) == 1);
    {
        auto resps = c130Client.receiveAll();
        const auto* resp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(resp != nullptr);
        REQUIRE(Json::parse(resp->bodyJson).string("status", "") == "ACCEPTED");
    }

    // U-2S SAR_IMAGE dead-ahead -> fails on step with native out_of_reach in ActivityStatusMT.
    const double u2Lat = u2Agent.ownship().latitudeRad;
    const double u2Lon = u2Agent.ownship().longitudeRad;
    REQUIRE(u2Client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                                 R"({"command_id":"21","payload":)" + std::to_string(*sarIdx) +
                                     R"(,"kind":"SAR_IMAGE","latitude_rad":)" + std::to_string(u2Lat + 0.4 * kDeg) +
                                     R"(,"longitude_rad":)" + std::to_string(u2Lon) + R"(,"resolution_m":1.0})"));
    REQUIRE(u2Adapter.poll(world.time()) == 1);
    std::string deadAheadActId;
    {
        auto resps = u2Client.receiveAll();
        const auto* resp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(resp != nullptr);
        const Json b = Json::parse(resp->bodyJson);
        REQUIRE(b.string("status", "") == "ACCEPTED");
        deadAheadActId = b.string("activity_id", "0");
    }
    world.step(4u);
    REQUIRE(u2Adapter.publishStatus(world.time()));
    {
        auto msgs = u2Client.receiveAll();
        const auto* actStatus = findLastByType(msgs, MessageType::ActivityStatusMT);
        REQUIRE(actStatus != nullptr);
        bool sawOutOfReach = false;
        const Json actBody = Json::parse(actStatus->bodyJson);
        for (const Json& a : actBody.child("activities").asArray()) {
            if (a.string("activity_id", "") == deadAheadActId) {
                REQUIRE(a.string("state", "") == "failed");
                REQUIRE(a.string("reason", "") == "out_of_reach");
                sawOutOfReach = true;
            }
        }
        REQUIRE(sawOutOfReach);
    }

    // Cross-track east SAR_IMAGE -> accepted & produces Image contact!
    REQUIRE(u2Client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                                 R"({"command_id":"22","payload":)" + std::to_string(*sarIdx) +
                                     R"(,"kind":"SAR_IMAGE","latitude_rad":)" + std::to_string(u2Lat) +
                                     R"(,"longitude_rad":)" + std::to_string(u2Lon + 0.4 * kDeg) +
                                     R"(,"resolution_m":1.0})"));
    REQUIRE(u2Adapter.poll(world.time()) == 1);
    {
        auto resps = u2Client.receiveAll();
        const auto* resp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(resp != nullptr);
        REQUIRE(Json::parse(resp->bodyJson).string("status", "") == "ACCEPTED");
    }

    world.step(120u);
    REQUIRE(c130Adapter.publishSensorReports(world.time()));
    REQUIRE(u2Adapter.publishSensorReports(world.time()));

    bool sawWeatherContact = false;
    for (const auto& r : c130Client.receiveAll()) {
        if (r.messageType == MessageType::ContactMT) {
            const Json body = Json::parse(r.bodyJson);
            for (const Json& c : body.child("contacts").asArray()) {
                if (c.string("kind", "") == "weather") sawWeatherContact = true;
            }
        }
    }
    REQUIRE(sawWeatherContact);

    bool sawSarImage = false;
    for (const auto& r : u2Client.receiveAll()) {
        if (r.messageType == MessageType::ContactMT) {
            const Json body = Json::parse(r.bodyJson);
            for (const Json& c : body.child("contacts").asArray()) {
                if (c.string("kind", "") == "image") sawSarImage = true;
            }
        }
    }
    REQUIRE(sawSarImage);

    // Release grant on C-130J -> subsequent WEATHER_SEARCH is refused not_granted.
    REQUIRE(c130Client.sendMessage(MessageType::ControlGrantCommandMT, world.time(),
                                   R"({"command_id":"15","capability":")" + wxCap + R"(","action":"RELEASE"})"));
    REQUIRE(c130Client.sendMessage(MessageType::PayloadTaskCommandMT, world.time(),
                                   R"({"command_id":"16","payload":)" + std::to_string(*wxIdx) +
                                       R"(,"kind":"WEATHER_SEARCH"})"));
    REQUIRE(c130Adapter.poll(world.time()) == 2);
    {
        auto resps = c130Client.receiveAll();
        const auto* lastResp = findLastByType(resps, MessageType::CommandResponseMT);
        REQUIRE(lastResp != nullptr);
        const Json b = Json::parse(lastResp->bodyJson);
        REQUIRE(b.string("command_id", "") == "16");
        REQUIRE(b.string("status", "") == "REJECTED");
        REQUIRE(b.string("reason", "") == "not_granted");
    }

    // Strict separation check: all external OMS exchanges produced ZERO modeled RF DMS packets!
    REQUIRE(c130Adapter.stats().framesSent > 0);
    REQUIRE(c130Adapter.stats().framesReceived > 0);
    REQUIRE(referee.netStats(netId).transmissions == 0);
    REQUIRE(c130Agent.inbox().empty());
}
