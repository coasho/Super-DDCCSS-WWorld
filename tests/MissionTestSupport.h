#pragma once

// Shared inputs of the mission-episode gateway tests (docs/sdk/mission-training.md; M5 core): the frozen contract's
// two-ship configuration with this build's JSBSim data, its fixtures, and a reader for the documents the public API
// returns. The tests reach the episode only through fsim/MissionEpisode.h; core::Json only reads the text back.

#include "foundation/core/Json.h"
#include "fsim/MissionEpisode.h"

#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace mission_test {

inline std::string fixturePath(const std::string& name) {
    return std::string(FSIM_TEST_SOURCE_DIR) + "/tests/fixtures/mission_training/contract/" + name;
}

inline std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot read " + path);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

inline std::string fixture(const std::string& name) { return readFile(fixturePath(name)); }

/// Replace the one occurrence of `from` (it must be there exactly once).
inline std::string replaced(std::string text, std::string_view from, std::string_view to) {
    const std::size_t at = text.find(from);
    if (at == std::string::npos || text.find(from, at + 1) != std::string::npos)
        throw std::runtime_error("test configuration edit '" + std::string(from) + "' is not unique");
    return text.replace(at, from.size(), to);
}

inline std::string jsonString(std::string_view s) {
    std::string out = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

/// The frozen two-ship configuration, with this build's JSBSim data.
inline std::string config() {
    return replaced(fixture("config.two-ship-c2.json"), "\"terrain\": false",
                    "\"terrain\": false, \"jsbsim_root\": " + jsonString(FSIM_TEST_JSBSIM_ROOT));
}

/// The same, with `extra` light aircraft more: a reset that takes long enough for another thread to meet it.
inline std::string loadedConfig(int extra) {
    std::string vehicles;
    for (int i = 0; i < extra; ++i)
        vehicles += "{\"name\": \"load-" + std::to_string(i) + "\", \"type\": \"jsbsim:c172x\", \"initial\": {\"lat_deg\": 36.1, "
                    "\"lon_deg\": -115.1, \"alt_msl_m\": 3000, \"heading_deg\": 0, \"airspeed_ms\": 50}},\n";
    return replaced(replaced(config(), "\"capacity\": 8", "\"capacity\": " + std::to_string(extra + 8)), "\"vehicles\": [",
                    "\"vehicles\": [\n" + vehicles);
}

inline fsim::core::Json parse(const std::string& text) { return fsim::core::Json::parse(text, "document"); }
inline const fsim::core::Json& at(const fsim::core::Json& j, std::string_view key) {
    const fsim::core::Json* m = j.find(key);
    if (!m) throw std::runtime_error("document has no member '" + std::string(key) + "'");
    return *m;
}
inline std::string str(const fsim::core::Json& j, std::string_view key) { return at(j, key).asString(); }

/// An `ma.actions.v1` document answering an episode key (reset's text) as `service` at `tick`.
inline std::string actions(const std::string& episodeKey, std::uint64_t service, std::uint64_t tick, const std::string& requests) {
    return "{\"schema\":\"ma.actions.v1\",\"episode\":" + episodeKey + ",\"service_id\":\"" + std::to_string(service) + "\",\"tick\":\"" +
           std::to_string(tick) + "\",\"requests\":[" + requests + "]}";
}
inline std::string request(std::uint64_t id, const std::string& kind, const std::string& operation, const std::string& parameters) {
    return "{\"request_id\":\"" + std::to_string(id) + "\",\"kind\":\"" + kind + "\",\"operation\":\"" + operation +
           "\",\"parameters\":" + parameters + "}";
}

/// A receipt of an `ma.receipts.v1` or `ma.observation.v1` document by request id (null: none).
inline const fsim::core::Json* receipt(const fsim::core::Json& document, std::uint64_t requestId) {
    for (const fsim::core::Json& r : at(document, "receipts").asArray())
        if (str(r, "request_id") == std::to_string(requestId)) return &r;
    return nullptr;
}
/// The submission's status of one request ("" when it has none).
inline std::string submitted(const fsim::core::Json& receipts, std::uint64_t requestId, std::string* reason = nullptr) {
    const fsim::core::Json& s = at(receipts, "submission");
    for (const fsim::core::Json& r : at(s, "requests").asArray())
        if (str(r, "request_id") == std::to_string(requestId)) {
            if (reason) *reason = at(r, "reason").isNull() ? std::string() : str(r, "reason");
            return str(r, "status");
        }
    return {};
}

// Service ids of the frozen configuration.
inline constexpr std::uint64_t kLeadFlight = 1, kLeadSensors = 2, kCrc = 4, kWing = 9007199254740995ull;

} // namespace mission_test
