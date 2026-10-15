#pragma once

// `ma.config.v1` (docs/sdk/mission-training.md, section 7): read strictly, held to every rule of the contract, and
// mapped onto the M4 episode's native recipe. Internal to the SDK.

#include "env/MissionEpisode.h"
#include "fsim/Comm.h"

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::mission {

/// Section 14's limits, the configuration's or their defaults.
struct Limits {
    std::uint32_t maxDocumentBytes = 1048576;
    std::uint32_t maxRequestsPerDocument = 64;
    std::uint32_t maxInFlightRequests = 256;
    std::uint32_t completedRequestRetentionTicks = 64;
    std::uint32_t maxMessageBytes = 1024;
};

/// One `services` entry (section 9).
struct ServiceConfig {
    std::uint64_t serviceId = 0;
    std::string name, platform;
    std::vector<std::string> capabilities;
    bool ownsDmsIngress = false, ownsDmsEgress = false;
};

/// One noncombat task criterion (section 13).
struct Criterion {
    enum class Kind { TickLimit, RegionReached, C2TaskState } kind = Kind::TickLimit;
    enum class Outcome { Success, Failure, Truncated } outcome = Outcome::Truncated;
    std::string id;
    std::uint64_t maxTicks = 0;                 ///< tick_limit
    std::string platform, fromPlatform;         ///< region_reached, c2_task_state
    double latitudeRad = 0.0, longitudeRad = 0.0, radiusM = 0.0;
    double altitudeMinM = -std::numeric_limits<double>::infinity(), altitudeMaxM = std::numeric_limits<double>::infinity();
    double dwellS = 0.0;
    comm::ReceivedTask::State state = comm::ReceivedTask::Completed; ///< c2_task_state
};

struct Config {
    std::uint64_t envId = 0, baseSeed = 0;
    /// The M4 recipe: timing, scenario, C2 nodes, systems, nets, dissemination, RBAC, execution. Its services, native
    /// step and judge are the gateway's to add.
    env::EpisodeConfig episode;
    std::vector<ServiceConfig> services;          ///< ascending service id
    double staleAfterS = std::numeric_limits<double>::quiet_NaN(); ///< NaN: nothing marked stale
    Limits limits;
    std::vector<Criterion> criteria;              ///< in the configuration's order
    std::vector<std::string> platforms;           ///< every platform name: scenario vehicles (with counts), then C2 nodes
    bool isolated() const noexcept { return !episode.diagnostic && !episode.publish && episode.recordDir.empty(); }
    const ServiceConfig* service(std::uint64_t id) const noexcept;
};

/// Read and check a configuration; Malformed (std::invalid_argument) for anything the contract refuses.
std::shared_ptr<const Config> parseConfig(std::string_view text);

} // namespace fsim::mission
