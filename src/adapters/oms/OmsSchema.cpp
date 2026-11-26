#include "adapters/oms/OmsSchema.h"

#include "foundation/core/Json.h"

#include <cmath>
#include <limits>
#include <unordered_set>

namespace fsim::oms {

using fsim::core::Json;

namespace {

Json numOrNull(double v) {
    return std::isfinite(v) ? Json(v) : Json(nullptr);
}

Json vec3Json(const double v[3]) {
    Json arr(Json::Array{});
    arr.push(numOrNull(v[0]));
    arr.push(numOrNull(v[1]));
    arr.push(numOrNull(v[2]));
    return arr;
}

const char* commandStatusUpper(control::CommandStatus status) noexcept {
    switch (status) {
    case control::CommandStatus::Accepted: return "ACCEPTED";
    case control::CommandStatus::Rejected: return "REJECTED";
    case control::CommandStatus::Canceled: return "CANCELED";
    case control::CommandStatus::Valid: return "VALID";
    }
    return "REJECTED";
}

bool checkAllowedKeys(const Json& obj,
                      const std::unordered_set<std::string_view>& allowed,
                      std::string* error) {
    if (!obj.isObject()) {
        if (error) *error = "expected JSON object";
        return false;
    }
    for (const auto& [k, _] : obj.asObject()) {
        if (allowed.find(k) == allowed.end()) {
            if (error) *error = "unknown field '" + k + "'";
            return false;
        }
    }
    return true;
}

bool reqString(const Json& obj, std::string_view key, std::string& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || !v->isString()) {
        if (error) *error = "missing or non-string field '" + std::string(key) + "'";
        return false;
    }
    out = v->asString();
    return true;
}

bool reqUint64Str(const Json& obj, std::string_view key, std::uint64_t& out, std::string* error) {
    std::string text;
    if (!reqString(obj, key, text, error)) return false;
    if (!parseUint64Strict(text, out)) {
        if (error) *error = "invalid 64-bit unsigned decimal string for field '" + std::string(key) + "'";
        return false;
    }
    return true;
}

bool optUint64Str(const Json& obj, std::string_view key, std::uint64_t fallback, std::uint64_t& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || v->isNull()) {
        out = fallback;
        return true;
    }
    if (!v->isString() || !parseUint64Strict(v->asString(), out)) {
        if (error) *error = "invalid 64-bit unsigned decimal string for field '" + std::string(key) + "'";
        return false;
    }
    return true;
}

bool reqFiniteDouble(const Json& obj, std::string_view key, double& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || !v->isNumber() || !std::isfinite(v->asNumber())) {
        if (error) *error = "missing or non-finite numeric field '" + std::string(key) + "'";
        return false;
    }
    out = v->asNumber();
    return true;
}

bool optFiniteDouble(const Json& obj, std::string_view key, double fallback, double& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || v->isNull()) {
        out = fallback;
        return true;
    }
    if (!v->isNumber() || !std::isfinite(v->asNumber())) {
        if (error) *error = "non-finite or non-numeric field '" + std::string(key) + "'";
        return false;
    }
    out = v->asNumber();
    return true;
}

bool reqNonNegInt(const Json& obj, std::string_view key, std::uint64_t maxVal, std::uint64_t& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || !v->isNumber() || !std::isfinite(v->asNumber())) {
        if (error) *error = "missing or non-numeric integer field '" + std::string(key) + "'";
        return false;
    }
    const double d = v->asNumber();
    if (d < 0.0 || std::floor(d) != d || d > static_cast<double>(maxVal)) {
        if (error) *error = "out-of-range integer field '" + std::string(key) + "'";
        return false;
    }
    out = static_cast<std::uint64_t>(d);
    return true;
}

bool optNonNegInt(const Json& obj, std::string_view key, std::uint64_t maxVal, std::uint64_t fallback, std::uint64_t& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || v->isNull()) {
        out = fallback;
        return true;
    }
    return reqNonNegInt(obj, key, maxVal, out, error);
}

bool optBool(const Json& obj, std::string_view key, bool fallback, bool& out, std::string* error) {
    const Json* v = obj.find(key);
    if (!v || v->isNull()) {
        out = fallback;
        return true;
    }
    if (!v->isBool()) {
        if (error) *error = "non-boolean field '" + std::string(key) + "'";
        return false;
    }
    out = v->asBool();
    return true;
}

bool parsePayloadStateStr(std::string_view s, ms::PayloadState& out) noexcept {
    if (s == "off" || s == "OFF") { out = ms::PayloadState::Off; return true; }
    if (s == "standby" || s == "STANDBY") { out = ms::PayloadState::Standby; return true; }
    if (s == "operate" || s == "OPERATE") { out = ms::PayloadState::Operate; return true; }
    return false;
}

bool parseAmtiModeStr(std::string_view s, ms::AmtiMode& out) noexcept {
    if (s == "range_while_search" || s == "RANGE_WHILE_SEARCH") { out = ms::AmtiMode::RangeWhileSearch; return true; }
    if (s == "velocity_search" || s == "VELOCITY_SEARCH") { out = ms::AmtiMode::VelocitySearch; return true; }
    if (s == "track_while_scan" || s == "TRACK_WHILE_SCAN") { out = ms::AmtiMode::TrackWhileScan; return true; }
    if (s == "surveillance" || s == "SURVEILLANCE") { out = ms::AmtiMode::Surveillance; return true; }
    if (s == "single_target_track" || s == "SINGLE_TARGET_TRACK") { out = ms::AmtiMode::SingleTargetTrack; return true; }
    return false;
}

bool parseAmtiTaskStr(std::string_view s, ms::AmtiTask& out) noexcept {
    if (s == "volume" || s == "VOLUME") { out = ms::AmtiTask::Volume; return true; }
    if (s == "cued" || s == "CUED") { out = ms::AmtiTask::Cued; return true; }
    if (s == "track" || s == "TRACK") { out = ms::AmtiTask::Track; return true; }
    return false;
}

bool parseTaskKindStr(std::string_view s, ms::TaskKind& out) noexcept {
    if (s == "dispense" || s == "CM_DISPENSE") { out = ms::TaskKind::Dispense; return true; }
    if (s == "fire" || s == "GUN_FIRE") { out = ms::TaskKind::Fire; return true; }
    if (s == "jam" || s == "EA_JAM") { out = ms::TaskKind::Jam; return true; }
    if (s == "search" || s == "PO_SEARCH" || s == "WEATHER_SEARCH" || s == "SMTI_SEARCH") { out = ms::TaskKind::Search; return true; }
    if (s == "track" || s == "PO_TRACK") { out = ms::TaskKind::Track; return true; }
    if (s == "image" || s == "SAR_IMAGE") { out = ms::TaskKind::Image; return true; }
    return false;
}

} // namespace

const char* serviceHealthName(ServiceHealth health) noexcept {
    switch (health) {
    case ServiceHealth::Operational: return "OPERATIONAL";
    case ServiceHealth::Degraded: return "DEGRADED";
    case ServiceHealth::Faulted: return "FAULTED";
    case ServiceHealth::Offline: return "OFFLINE";
    }
    return "OFFLINE";
}

bool parseServiceHealth(std::string_view name, ServiceHealth& out) noexcept {
    if (name == "OPERATIONAL") { out = ServiceHealth::Operational; return true; }
    if (name == "DEGRADED") { out = ServiceHealth::Degraded; return true; }
    if (name == "FAULTED") { out = ServiceHealth::Faulted; return true; }
    if (name == "OFFLINE") { out = ServiceHealth::Offline; return true; }
    return false;
}

const char* messageTypeName(MessageType type) noexcept {
    switch (type) {
    case MessageType::ServiceRegistrationMT: return "ServiceRegistrationMT";
    case MessageType::ServiceHeartbeatMT: return "ServiceHeartbeatMT";
    case MessageType::ServiceStatusMT: return "ServiceStatusMT";
    case MessageType::SubsystemStatusMT: return "SubsystemStatusMT";
    case MessageType::CapabilityStatusMT: return "CapabilityStatusMT";
    case MessageType::ActivityStatusMT: return "ActivityStatusMT";
    case MessageType::SubsystemStateCommandMT: return "SubsystemStateCommandMT";
    case MessageType::SubsystemBITCommandMT: return "SubsystemBITCommandMT";
    case MessageType::AMTI_CommandMT: return "AMTI_CommandMT";
    case MessageType::PayloadTaskCommandMT: return "PayloadTaskCommandMT";
    case MessageType::StrikeCommandMT: return "StrikeCommandMT";
    case MessageType::JettisonCommandMT: return "JettisonCommandMT";
    case MessageType::ActivityCancelCommandMT: return "ActivityCancelCommandMT";
    case MessageType::ControlGrantCommandMT: return "ControlGrantCommandMT";
    case MessageType::CommandResponseMT: return "CommandResponseMT";
    case MessageType::OwnShipMDT: return "OwnShipMDT";
    case MessageType::AMTI_TrackMT: return "AMTI_TrackMT";
    case MessageType::ContactMT: return "ContactMT";
    case MessageType::PayloadReportMT: return "PayloadReportMT";
    case MessageType::StoreInventoryMT: return "StoreInventoryMT";
    case MessageType::ProtocolErrorMT: return "ProtocolErrorMT";
    }
    return "ProtocolErrorMT";
}

bool parseMessageType(std::string_view name, MessageType& out) noexcept {
    static constexpr MessageType kAll[] = {
        MessageType::ServiceRegistrationMT,
        MessageType::ServiceHeartbeatMT,
        MessageType::ServiceStatusMT,
        MessageType::SubsystemStatusMT,
        MessageType::CapabilityStatusMT,
        MessageType::ActivityStatusMT,
        MessageType::SubsystemStateCommandMT,
        MessageType::SubsystemBITCommandMT,
        MessageType::AMTI_CommandMT,
        MessageType::PayloadTaskCommandMT,
        MessageType::StrikeCommandMT,
        MessageType::JettisonCommandMT,
        MessageType::ActivityCancelCommandMT,
        MessageType::ControlGrantCommandMT,
        MessageType::CommandResponseMT,
        MessageType::OwnShipMDT,
        MessageType::AMTI_TrackMT,
        MessageType::ContactMT,
        MessageType::PayloadReportMT,
        MessageType::StoreInventoryMT,
        MessageType::ProtocolErrorMT,
    };
    for (MessageType t : kAll) {
        if (name == messageTypeName(t)) {
            out = t;
            return true;
        }
    }
    return false;
}

std::string formatUint64(std::uint64_t value) {
    return std::to_string(value);
}

bool parseUint64Strict(std::string_view text, std::uint64_t& out) noexcept {
    if (text.empty()) return false;
    std::uint64_t acc = 0;
    constexpr std::uint64_t kMax = std::numeric_limits<std::uint64_t>::max();
    for (char c : text) {
        if (c < '0' || c > '9') return false;
        const auto digit = static_cast<std::uint64_t>(c - '0');
        if (acc > (kMax - digit) / 10u) return false;
        acc = acc * 10u + digit;
    }
    out = acc;
    return true;
}

bool serializeEnvelope(const OmsEnvelope& env, std::string& outJson, std::string* error) {
    if (env.schema != kSchemaVersion) {
        if (error) *error = "unsupported schema version";
        return false;
    }
    if (env.transportDomain != kTransportDomain) {
        if (error) *error = "transport_domain must be external_oms";
        return false;
    }
    if (env.authorityScope != kAuthorityScope) {
        if (error) *error = "authority_scope must be agent_only";
        return false;
    }
    if (!std::isfinite(env.timestampS)) {
        if (error) *error = "timestamp_s must be finite";
        return false;
    }
    Json body;
    try {
        body = Json::parse(env.bodyJson, "oms.body");
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
    if (!body.isObject()) {
        if (error) *error = "body must be a JSON object";
        return false;
    }

    Json doc(Json::Object{});
    doc.set("schema", env.schema);
    doc.set("transport_domain", env.transportDomain);
    doc.set("authority_scope", env.authorityScope);
    doc.set("message_type", messageTypeName(env.messageType));
    doc.set("message_id", formatUint64(env.messageId));
    doc.set("correlation_id", formatUint64(env.correlationId));
    doc.set("source_service_id", formatUint64(env.sourceServiceId));
    doc.set("timestamp_s", env.timestampS);
    doc.set("body", std::move(body));
    outJson = doc.dump();
    return true;
}

bool parseEnvelope(std::string_view jsonText, OmsEnvelope& outEnv, std::string* error) {
    Json doc;
    try {
        doc = Json::parse(jsonText, "oms.envelope");
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }

    static const std::unordered_set<std::string_view> kTopKeys = {
        "schema", "transport_domain", "authority_scope", "message_type",
        "message_id", "correlation_id", "source_service_id", "timestamp_s", "body",
    };
    if (!checkAllowedKeys(doc, kTopKeys, error)) return false;

    std::string schema, domain, scope, typeStr;
    if (!reqString(doc, "schema", schema, error)) return false;
    if (schema != kSchemaVersion) {
        if (error) *error = "unsupported schema version '" + schema + "'";
        return false;
    }
    if (!reqString(doc, "transport_domain", domain, error)) return false;
    if (domain != kTransportDomain) {
        if (error) *error = "invalid transport_domain '" + domain + "'";
        return false;
    }
    if (!reqString(doc, "authority_scope", scope, error)) return false;
    if (scope != kAuthorityScope) {
        if (error) *error = "invalid authority_scope '" + scope + "'";
        return false;
    }
    if (!reqString(doc, "message_type", typeStr, error)) return false;
    MessageType mt = MessageType::ProtocolErrorMT;
    if (!parseMessageType(typeStr, mt)) {
        if (error) *error = "unknown message_type '" + typeStr + "'";
        return false;
    }

    std::uint64_t msgId = 0, corrId = 0, svcId = 0;
    double ts = 0.0;
    if (!reqUint64Str(doc, "message_id", msgId, error)) return false;
    if (!optUint64Str(doc, "correlation_id", 0, corrId, error)) return false;
    if (!reqUint64Str(doc, "source_service_id", svcId, error)) return false;
    if (!reqFiniteDouble(doc, "timestamp_s", ts, error)) return false;

    const Json* body = doc.find("body");
    if (!body || !body->isObject()) {
        if (error) *error = "missing or non-object field 'body'";
        return false;
    }

    outEnv.schema = std::move(schema);
    outEnv.transportDomain = std::move(domain);
    outEnv.authorityScope = std::move(scope);
    outEnv.messageType = mt;
    outEnv.messageId = msgId;
    outEnv.correlationId = corrId;
    outEnv.sourceServiceId = svcId;
    outEnv.timestampS = ts;
    outEnv.bodyJson = body->dump();
    return true;
}

bool parseServiceHeartbeatBody(std::string_view bodyJson, ServiceHeartbeatPayload& out, std::string* error) {
    Json obj;
    try {
        obj = Json::parse(bodyJson, "oms.heartbeat");
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
    static const std::unordered_set<std::string_view> kKeys = {"service_id", "sequence", "health"};
    if (!checkAllowedKeys(obj, kKeys, error)) return false;
    if (!reqUint64Str(obj, "service_id", out.serviceId, error)) return false;
    if (!reqUint64Str(obj, "sequence", out.sequence, error)) return false;
    std::string healthStr;
    if (!reqString(obj, "health", healthStr, error)) return false;
    if (!parseServiceHealth(healthStr, out.health)) {
        if (error) *error = "invalid service health '" + healthStr + "'";
        return false;
    }
    return true;
}

bool parseCommandBody(MessageType type, std::string_view bodyJson, ParsedCommand& out, std::string* error) {
    Json obj;
    try {
        obj = Json::parse(bodyJson, "oms.command");
    } catch (const std::exception& ex) {
        if (error) *error = ex.what();
        return false;
    }
    if (!obj.isObject()) {
        if (error) *error = "command body must be an object";
        return false;
    }

    std::uint64_t cmdId = 0;
    if (!reqUint64Str(obj, "command_id", cmdId, error)) return false;
    if (cmdId == 0) {
        if (error) *error = "command_id must be non-zero";
        return false;
    }

    out.commandId = cmdId;
    out.messageType = type;

    if (type == MessageType::SubsystemStateCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {"command_id", "payload", "state"};
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        std::uint64_t payload = 0;
        if (!reqNonNegInt(obj, "payload", 0xFFFFu, payload, error)) return false;
        std::string stateStr;
        if (!reqString(obj, "state", stateStr, error)) return false;
        ms::StateCommand cmd{};
        cmd.payload = static_cast<std::uint16_t>(payload);
        if (!parsePayloadStateStr(stateStr, cmd.state)) {
            if (error) *error = "invalid payload state '" + stateStr + "'";
            return false;
        }
        Json canon(Json::Object{});
        canon.set("payload", static_cast<int>(cmd.payload));
        canon.set("state", ms::payloadStateName(cmd.state));
        out.canonicalBody = canon.dump();
        out.payload = cmd;
        return true;
    }

    if (type == MessageType::SubsystemBITCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {"command_id", "payload"};
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        std::uint64_t payload = 0;
        if (!reqNonNegInt(obj, "payload", 0xFFFFu, payload, error)) return false;
        ms::BitCommand cmd{};
        cmd.payload = static_cast<std::uint16_t>(payload);
        Json canon(Json::Object{});
        canon.set("payload", static_cast<int>(cmd.payload));
        out.canonicalBody = canon.dump();
        out.payload = cmd;
        return true;
    }

    if (type == MessageType::AMTI_CommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {
            "command_id", "payload", "mode", "task", "track_id",
            "azimuth_centre_rad", "azimuth_half_width_rad",
            "elevation_centre_rad", "elevation_half_width_rad",
            "bars", "range_scale_m", "signal_level", "illuminate",
        };
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        std::uint64_t payload = 0;
        if (!reqNonNegInt(obj, "payload", 0xFFFFu, payload, error)) return false;
        ms::AmtiCommand cmd{};
        cmd.payload = static_cast<std::uint16_t>(payload);
        if (const Json* m = obj.find("mode"); m && !m->isNull()) {
            if (!m->isString() || !parseAmtiModeStr(m->asString(), cmd.mode)) {
                if (error) *error = "invalid AMTI mode";
                return false;
            }
        }
        if (const Json* t = obj.find("task"); t && !t->isNull()) {
            if (!t->isString() || !parseAmtiTaskStr(t->asString(), cmd.task)) {
                if (error) *error = "invalid AMTI task";
                return false;
            }
        }
        if (!optUint64Str(obj, "track_id", 0, cmd.track, error)) return false;
        if (!optFiniteDouble(obj, "azimuth_centre_rad", cmd.azimuthCentreRad, cmd.azimuthCentreRad, error)) return false;
        if (!optFiniteDouble(obj, "azimuth_half_width_rad", cmd.azimuthHalfWidthRad, cmd.azimuthHalfWidthRad, error)) return false;
        if (!optFiniteDouble(obj, "elevation_centre_rad", cmd.elevationCentreRad, cmd.elevationCentreRad, error)) return false;
        if (!optFiniteDouble(obj, "elevation_half_width_rad", cmd.elevationHalfWidthRad, cmd.elevationHalfWidthRad, error)) return false;
        std::uint64_t bars = static_cast<std::uint64_t>(cmd.bars);
        if (!optNonNegInt(obj, "bars", 64u, bars, bars, error)) return false;
        cmd.bars = static_cast<int>(bars);
        if (!optFiniteDouble(obj, "range_scale_m", cmd.rangeScaleM, cmd.rangeScaleM, error)) return false;
        if (!optBool(obj, "signal_level", false, cmd.signalLevel, error)) return false;
        if (!optBool(obj, "illuminate", false, cmd.illuminate, error)) return false;

        Json canon(Json::Object{});
        canon.set("payload", static_cast<int>(cmd.payload));
        canon.set("mode", ms::amtiModeName(cmd.mode));
        canon.set("task", ms::amtiTaskName(cmd.task));
        canon.set("track_id", formatUint64(cmd.track));
        canon.set("azimuth_centre_rad", cmd.azimuthCentreRad);
        canon.set("azimuth_half_width_rad", cmd.azimuthHalfWidthRad);
        canon.set("elevation_centre_rad", cmd.elevationCentreRad);
        canon.set("elevation_half_width_rad", numOrNull(cmd.elevationHalfWidthRad));
        canon.set("bars", cmd.bars);
        canon.set("range_scale_m", cmd.rangeScaleM);
        canon.set("signal_level", cmd.signalLevel);
        canon.set("illuminate", cmd.illuminate);
        out.canonicalBody = canon.dump();
        out.payload = cmd;
        return true;
    }

    if (type == MessageType::PayloadTaskCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {
            "command_id", "payload", "kind", "contact_id",
            "chaff", "flares", "salvos", "interval_s", "rounds",
            "azimuth_centre_rad", "elevation_centre_rad",
            "azimuth_half_width_rad", "elevation_half_width_rad",
            "range_scale_m", "frequency_hz", "bandwidth_hz",
            "latitude_rad", "longitude_rad", "resolution_m", "duration_s",
        };
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        std::uint64_t payload = 0;
        if (!reqNonNegInt(obj, "payload", 0xFFFFu, payload, error)) return false;
        std::string kindStr;
        if (!reqString(obj, "kind", kindStr, error)) return false;
        ms::PayloadTask task{};
        task.payload = static_cast<std::uint16_t>(payload);
        if (!parseTaskKindStr(kindStr, task.kind)) {
            if (error) *error = "invalid payload task kind '" + kindStr + "'";
            return false;
        }
        if (!optUint64Str(obj, "contact_id", 0, task.contact, error)) return false;
        std::uint64_t chaff = 0, flares = 0, salvos = 1, rounds = 0;
        if (!optNonNegInt(obj, "chaff", 10000u, 0u, chaff, error)) return false;
        if (!optNonNegInt(obj, "flares", 10000u, 0u, flares, error)) return false;
        if (!optNonNegInt(obj, "salvos", 10000u, 1u, salvos, error)) return false;
        if (!optNonNegInt(obj, "rounds", 100000u, 0u, rounds, error)) return false;
        task.chaff = static_cast<int>(chaff);
        task.flares = static_cast<int>(flares);
        task.salvos = static_cast<int>(salvos);
        task.rounds = static_cast<int>(rounds);
        if (!optFiniteDouble(obj, "interval_s", task.intervalS, task.intervalS, error)) return false;
        if (!optFiniteDouble(obj, "azimuth_centre_rad", task.azimuthCentreRad, task.azimuthCentreRad, error)) return false;
        if (!optFiniteDouble(obj, "elevation_centre_rad", task.elevationCentreRad, task.elevationCentreRad, error)) return false;
        if (!optFiniteDouble(obj, "azimuth_half_width_rad", task.azimuthHalfWidthRad, task.azimuthHalfWidthRad, error)) return false;
        if (!optFiniteDouble(obj, "elevation_half_width_rad", task.elevationHalfWidthRad, task.elevationHalfWidthRad, error)) return false;
        if (!optFiniteDouble(obj, "range_scale_m", task.rangeScaleM, task.rangeScaleM, error)) return false;
        if (!optFiniteDouble(obj, "frequency_hz", task.frequencyHz, task.frequencyHz, error)) return false;
        if (!optFiniteDouble(obj, "bandwidth_hz", task.bandwidthHz, task.bandwidthHz, error)) return false;
        if (!optFiniteDouble(obj, "latitude_rad", task.latitudeRad, task.latitudeRad, error)) return false;
        if (!optFiniteDouble(obj, "longitude_rad", task.longitudeRad, task.longitudeRad, error)) return false;
        if (!optFiniteDouble(obj, "resolution_m", task.resolutionM, task.resolutionM, error)) return false;
        if (!optFiniteDouble(obj, "duration_s", task.durationS, task.durationS, error)) return false;

        Json canon(Json::Object{});
        canon.set("payload", static_cast<int>(task.payload));
        canon.set("kind", ms::taskKindName(task.kind));
        canon.set("contact_id", formatUint64(task.contact));
        canon.set("chaff", task.chaff);
        canon.set("flares", task.flares);
        canon.set("salvos", task.salvos);
        canon.set("interval_s", numOrNull(task.intervalS));
        canon.set("rounds", task.rounds);
        canon.set("azimuth_centre_rad", numOrNull(task.azimuthCentreRad));
        canon.set("elevation_centre_rad", numOrNull(task.elevationCentreRad));
        canon.set("azimuth_half_width_rad", numOrNull(task.azimuthHalfWidthRad));
        canon.set("elevation_half_width_rad", numOrNull(task.elevationHalfWidthRad));
        canon.set("range_scale_m", numOrNull(task.rangeScaleM));
        canon.set("frequency_hz", numOrNull(task.frequencyHz));
        canon.set("bandwidth_hz", numOrNull(task.bandwidthHz));
        canon.set("latitude_rad", numOrNull(task.latitudeRad));
        canon.set("longitude_rad", numOrNull(task.longitudeRad));
        canon.set("resolution_m", numOrNull(task.resolutionM));
        canon.set("duration_s", numOrNull(task.durationS));
        out.canonicalBody = canon.dump();
        out.payload = task;
        return true;
    }

    if (type == MessageType::StrikeCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {
            "command_id", "payload", "store", "quantity", "stores", "target", "consent", "ripple_s",
        };
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        std::uint64_t payload = 0;
        if (!reqNonNegInt(obj, "payload", 0xFFFFu, payload, error)) return false;
        ms::StrikeCommand cmd{};
        cmd.payload = static_cast<std::uint16_t>(payload);
        if (const Json* s = obj.find("store"); s && !s->isNull()) {
            if (!s->isString()) { if (error) *error = "non-string store"; return false; }
            cmd.store = s->asString();
        }
        std::uint64_t qty = 1, target = 0;
        if (!optNonNegInt(obj, "quantity", 64u, 1u, qty, error)) return false;
        if (!optNonNegInt(obj, "target", 0xFFFFFFFFu, 0u, target, error)) return false;
        cmd.quantity = static_cast<int>(qty);
        cmd.target = static_cast<std::uint32_t>(target);
        if (!optBool(obj, "consent", false, cmd.consent, error)) return false;
        if (!optFiniteDouble(obj, "ripple_s", cmd.rippleS, cmd.rippleS, error)) return false;
        if (const Json* arr = obj.find("stores"); arr && !arr->isNull()) {
            if (!arr->isArray()) { if (error) *error = "stores must be an array"; return false; }
            for (const Json& el : arr->asArray()) {
                if (!el.isNumber() || !std::isfinite(el.asNumber()) || el.asNumber() < 0.0 ||
                    std::floor(el.asNumber()) != el.asNumber() || el.asNumber() > 65535.0) {
                    if (error) *error = "invalid store index";
                    return false;
                }
                cmd.stores.push_back(static_cast<std::uint16_t>(el.asNumber()));
            }
        }
        Json canon(Json::Object{});
        canon.set("payload", static_cast<int>(cmd.payload));
        canon.set("store", cmd.store);
        canon.set("quantity", cmd.quantity);
        canon.set("target", static_cast<double>(cmd.target));
        canon.set("consent", cmd.consent);
        canon.set("ripple_s", cmd.rippleS);
        Json storesJson(Json::Array{});
        for (std::uint16_t st : cmd.stores) storesJson.push(static_cast<int>(st));
        canon.set("stores", std::move(storesJson));
        out.canonicalBody = canon.dump();
        out.payload = std::move(cmd);
        return true;
    }

    if (type == MessageType::JettisonCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {
            "command_id", "payload", "stores", "emergency",
        };
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        std::uint64_t payload = 0;
        if (!reqNonNegInt(obj, "payload", 0xFFFFu, payload, error)) return false;
        ms::JettisonCommand cmd{};
        cmd.payload = static_cast<std::uint16_t>(payload);
        if (!optBool(obj, "emergency", false, cmd.emergency, error)) return false;
        if (const Json* arr = obj.find("stores"); arr && !arr->isNull()) {
            if (!arr->isArray()) { if (error) *error = "stores must be an array"; return false; }
            for (const Json& el : arr->asArray()) {
                if (!el.isNumber() || !std::isfinite(el.asNumber()) || el.asNumber() < 0.0 ||
                    std::floor(el.asNumber()) != el.asNumber() || el.asNumber() > 65535.0) {
                    if (error) *error = "invalid store index";
                    return false;
                }
                cmd.stores.push_back(static_cast<std::uint16_t>(el.asNumber()));
            }
        }
        Json canon(Json::Object{});
        canon.set("payload", static_cast<int>(cmd.payload));
        canon.set("emergency", cmd.emergency);
        Json storesJson(Json::Array{});
        for (std::uint16_t st : cmd.stores) storesJson.push(static_cast<int>(st));
        canon.set("stores", std::move(storesJson));
        out.canonicalBody = canon.dump();
        out.payload = std::move(cmd);
        return true;
    }

    if (type == MessageType::ActivityCancelCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {"command_id", "activity_id"};
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        ActivityCancelPayload cmd{};
        if (!reqUint64Str(obj, "activity_id", cmd.activityId, error)) return false;
        Json canon(Json::Object{});
        canon.set("activity_id", formatUint64(cmd.activityId));
        out.canonicalBody = canon.dump();
        out.payload = cmd;
        return true;
    }

    if (type == MessageType::ControlGrantCommandMT) {
        static const std::unordered_set<std::string_view> kKeys = {"command_id", "capability", "action"};
        if (!checkAllowedKeys(obj, kKeys, error)) return false;
        ControlGrantPayload cmd{};
        if (!reqString(obj, "capability", cmd.capability, error)) return false;
        std::string action;
        if (!reqString(obj, "action", action, error)) return false;
        if (action == "request" || action == "REQUEST") {
            cmd.request = true;
        } else if (action == "release" || action == "RELEASE") {
            cmd.request = false;
        } else {
            if (error) *error = "invalid control grant action '" + action + "'";
            return false;
        }
        Json canon(Json::Object{});
        canon.set("capability", cmd.capability);
        canon.set("action", cmd.request ? "REQUEST" : "RELEASE");
        out.canonicalBody = canon.dump();
        out.payload = std::move(cmd);
        return true;
    }

    if (error) *error = "message_type is not an executable command";
    return false;
}

std::string makeServiceRegistrationBody(std::uint64_t serviceId,
                                        std::string_view serviceName,
                                        std::uint32_t vehicleId,
                                        std::string_view platformType,
                                        double heartbeatPeriodS,
                                        double peerHeartbeatTimeoutS) {
    Json obj(Json::Object{});
    obj.set("service_id", formatUint64(serviceId));
    obj.set("service_name", std::string(serviceName));
    obj.set("vehicle_id", static_cast<double>(vehicleId));
    obj.set("platform_type", std::string(platformType));
    obj.set("heartbeat_period_s", numOrNull(heartbeatPeriodS));
    obj.set("peer_heartbeat_timeout_s", numOrNull(peerHeartbeatTimeoutS));
    return obj.dump();
}

std::string makeServiceHeartbeatBody(std::uint64_t serviceId,
                                     std::uint64_t sequence,
                                     ServiceHealth health) {
    Json obj(Json::Object{});
    obj.set("service_id", formatUint64(serviceId));
    obj.set("sequence", formatUint64(sequence));
    obj.set("health", serviceHealthName(health));
    return obj.dump();
}

std::string makeServiceStatusBody(std::uint64_t serviceId,
                                  ServiceHealth ownHealth,
                                  ServiceHealth peerHealth,
                                  std::uint64_t peerServiceId,
                                  double lastPeerHeartbeatS,
                                  std::uint64_t commandsProcessed,
                                  std::uint64_t duplicateCommands,
                                  std::uint64_t protocolErrors) {
    Json obj(Json::Object{});
    obj.set("service_id", formatUint64(serviceId));
    obj.set("own_health", serviceHealthName(ownHealth));
    obj.set("peer_health", serviceHealthName(peerHealth));
    obj.set("peer_service_id", formatUint64(peerServiceId));
    obj.set("last_peer_heartbeat_s", numOrNull(lastPeerHeartbeatS));
    obj.set("commands_processed", formatUint64(commandsProcessed));
    obj.set("duplicate_commands", formatUint64(duplicateCommands));
    obj.set("protocol_errors", formatUint64(protocolErrors));
    return obj.dump();
}

std::string makeSubsystemStatusBody(const std::vector<ms::PayloadInfo>& payloads) {
    Json arr(Json::Array{});
    for (const auto& p : payloads) {
        Json item(Json::Object{});
        item.set("payload", static_cast<int>(p.index));
        item.set("name", p.name ? p.name : "");
        item.set("card", p.card ? p.card : "");
        item.set("designation", p.designation ? p.designation : "");
        item.set("family", ms::familyName(p.family));
        item.set("state", ms::payloadStateName(p.state));
        item.set("count", static_cast<int>(p.count));
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("subsystems", std::move(arr));
    return obj.dump();
}

std::string makeCapabilityStatusBody(const Agent& agent) {
    const auto caps = agent.missionCapabilities();
    Json arr(Json::Array{});
    for (const auto& c : caps) {
        const auto st = agent.missionCapabilityStatus(c.id ? c.id : "");
        Json item(Json::Object{});
        item.set("index", static_cast<int>(c.index));
        item.set("payload", static_cast<int>(c.payload));
        item.set("type", ms::capabilityTypeId(c.type));
        item.set("id", c.id ? c.id : "");
        item.set("support", control::supportName(c.support));
        item.set("availability", control::availabilityName(st.availability));
        item.set("reason", ms::reasonName(st.reason));
        item.set("next_available_s", numOrNull(st.nextAvailableS));
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("capabilities", std::move(arr));
    return obj.dump();
}

std::string makeActivityStatusBody(const std::vector<ms::ActivityRecord>& activities) {
    Json arr(Json::Array{});
    for (const auto& a : activities) {
        Json item(Json::Object{});
        item.set("activity_id", formatUint64(a.id));
        item.set("command_id", formatUint64(a.commandId));
        item.set("payload", static_cast<int>(a.payload));
        item.set("capability_index", static_cast<int>(a.capability));
        item.set("state", control::activityStateName(a.state));
        item.set("reason", ms::reasonName(a.reason));
        item.set("start_time_s", numOrNull(a.startTime));
        item.set("end_time_s", numOrNull(a.endTime));
        item.set("percent", numOrNull(a.percent));
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("activities", std::move(arr));
    return obj.dump();
}

std::string makeCommandResponseBody(std::uint64_t commandId,
                                    control::CommandStatus status,
                                    std::string_view reason,
                                    std::string_view reasonDescription,
                                    std::uint64_t activityId,
                                    bool duplicate) {
    Json obj(Json::Object{});
    obj.set("command_id", formatUint64(commandId));
    obj.set("status", commandStatusUpper(status));
    obj.set("reason", std::string(reason));
    obj.set("reason_description", std::string(reasonDescription));
    obj.set("activity_id", formatUint64(activityId));
    obj.set("duplicate", duplicate);
    return obj.dump();
}

std::string makeOwnShipBody(const OwnShip& ownShip) {
    Json obj(Json::Object{});
    obj.set("time_s", numOrNull(ownShip.timeS));
    obj.set("latitude_rad", numOrNull(ownShip.latitudeRad));
    obj.set("longitude_rad", numOrNull(ownShip.longitudeRad));
    obj.set("altitude_msl_m", numOrNull(ownShip.altitudeMslM));
    obj.set("altitude_agl_m", numOrNull(ownShip.altitudeAglM));
    obj.set("velocity_ned_ms", vec3Json(ownShip.velocityNedMs));
    obj.set("euler_rad", vec3Json(ownShip.eulerRad));
    obj.set("angular_rate_body_rad_s", vec3Json(ownShip.angularRateBodyRadS));
    obj.set("acceleration_body_ms2", vec3Json(ownShip.accelerationBodyMs2));
    obj.set("airspeed_true_ms", numOrNull(ownShip.airspeedTrueMs));
    obj.set("airspeed_calibrated_ms", numOrNull(ownShip.airspeedCalibratedMs));
    obj.set("mach", numOrNull(ownShip.mach));
    obj.set("alpha_rad", numOrNull(ownShip.alphaRad));
    obj.set("beta_rad", numOrNull(ownShip.betaRad));
    obj.set("load_factor", numOrNull(ownShip.loadFactor));
    obj.set("fuel_kg", numOrNull(ownShip.fuelKg));
    obj.set("on_ground", ownShip.onGround);
    obj.set("gnss_valid", ownShip.gnssValid);
    obj.set("air_data_valid", ownShip.airDataValid);
    obj.set("throttle", numOrNull(ownShip.throttle));
    return obj.dump();
}

std::string makeAmtiTracksBody(std::uint16_t payload, const std::vector<ms::AmtiTrack>& tracks) {
    Json arr(Json::Array{});
    for (const auto& t : tracks) {
        // Referee truth is intentionally omitted from external OMS actor reports.
        Json item(Json::Object{});
        item.set("track_id", formatUint64(t.id));
        item.set("payload", static_cast<int>(t.payload));
        item.set("time_s", numOrNull(t.timeS));
        item.set("last_update_s", numOrNull(t.lastUpdateS));
        item.set("updates", static_cast<double>(t.updates));
        item.set("latitude_rad", numOrNull(t.latitudeRad));
        item.set("longitude_rad", numOrNull(t.longitudeRad));
        item.set("height_m", numOrNull(t.heightM));
        item.set("velocity_ned_ms", vec3Json(t.velocityNedMs));
        item.set("position_sigma_m", numOrNull(t.positionSigmaM));
        item.set("velocity_sigma_ms", numOrNull(t.velocitySigmaMs));
        item.set("range_m", numOrNull(t.rangeM));
        item.set("azimuth_rad", numOrNull(t.azimuthRad));
        item.set("elevation_rad", numOrNull(t.elevationRad));
        item.set("range_rate_ms", numOrNull(t.rangeRateMs));
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("payload", static_cast<int>(payload));
    obj.set("tracks", std::move(arr));
    return obj.dump();
}

std::string makeContactsBody(std::uint16_t payload, const std::vector<ms::Contact>& contacts) {
    Json arr(Json::Array{});
    for (const auto& c : contacts) {
        // Referee truth is intentionally omitted from external OMS actor reports.
        Json item(Json::Object{});
        item.set("contact_id", formatUint64(c.id));
        item.set("payload", static_cast<int>(c.payload));
        item.set("kind", ms::contactKindName(c.kind));
        item.set("mode", ms::emitterModeName(c.mode));
        item.set("warning", c.warning);
        item.set("first_time_s", numOrNull(c.firstTimeS));
        item.set("time_s", numOrNull(c.timeS));
        item.set("azimuth_rad", numOrNull(c.azimuthRad));
        item.set("elevation_rad", numOrNull(c.elevationRad));
        item.set("bearing_sigma_rad", numOrNull(c.bearingSigmaRad));
        item.set("range_m", numOrNull(c.rangeM));
        item.set("range_rate_ms", numOrNull(c.rangeRateMs));
        item.set("latitude_rad", numOrNull(c.latitudeRad));
        item.set("longitude_rad", numOrNull(c.longitudeRad));
        item.set("height_m", numOrNull(c.heightM));
        item.set("strength_db", numOrNull(c.strengthDb));
        item.set("frequency_hz", numOrNull(c.frequencyHz));
        item.set("size_m", numOrNull(c.sizeM));
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("payload", static_cast<int>(payload));
    obj.set("contacts", std::move(arr));
    return obj.dump();
}

std::string makePayloadReportBody(std::uint16_t payload, const std::vector<ms::PayloadValue>& values) {
    Json arr(Json::Array{});
    for (const auto& v : values) {
        Json item(Json::Object{});
        item.set("name", v.name ? v.name : "");
        item.set("value", numOrNull(v.value));
        item.set("unit", v.unit ? v.unit : "");
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("payload", static_cast<int>(payload));
    obj.set("values", std::move(arr));
    return obj.dump();
}

std::string makeStoreInventoryBody(const std::vector<ms::StoreInfo>& stores) {
    Json arr(Json::Array{});
    for (const auto& s : stores) {
        Json item(Json::Object{});
        item.set("index", static_cast<int>(s.index));
        item.set("payload", static_cast<int>(s.payload));
        item.set("station", s.station);
        item.set("station_name", s.stationName ? s.stationName : "");
        item.set("store", s.store ? s.store : "");
        item.set("mnemonic", s.mnemonic ? s.mnemonic : "");
        item.set("store_class", s.storeClass ? s.storeClass : "");
        item.set("carriage", s.carriage ? s.carriage : "");
        item.set("powerable", s.powerable);
        item.set("jettisonable", s.jettisonable);
        item.set("flyable", s.flyable);
        item.set("status", ms::storeStatusName(s.status));
        item.set("mass_kg", numOrNull(s.massKg));
        item.set("fuel_kg", numOrNull(s.fuelKg));
        item.set("weapon_id", static_cast<double>(s.weapon));
        item.set("bay_doors", numOrNull(s.bayDoors));
        arr.push(std::move(item));
    }
    Json obj(Json::Object{});
    obj.set("stores", std::move(arr));
    return obj.dump();
}

std::string makeProtocolErrorBody(std::string_view code, std::string_view detail) {
    Json obj(Json::Object{});
    obj.set("code", std::string(code));
    obj.set("detail", std::string(detail));
    return obj.dump();
}

} // namespace fsim::oms
