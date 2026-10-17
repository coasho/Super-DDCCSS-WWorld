#pragma once

// The mission gateway of one generation (docs/sdk/mission-training.md, sections 10 to 13; M5 core). Internal to the
// SDK: the public MissionEpisode makes one at every reset, as the M4 episode's judge, so that it lives and dies with
// that generation's World and is reached only under the episode's boundary.
//
// It holds the generation's public World facade and one Agent per configured service - each request is made as that
// service's own Agent, through the SDK's existing permission paths and nothing else - and the transport's records:
// receipts, deduplication, the snapshot frozen at each tick, the referee's documents. Native semantics, refusals and
// authority stay the SDK's; the gateway adds only the contract's own checks and its records.

#include "training/native/episodes/runtime/MissionEpisode.h"
#include "fsim/Agent.h"
#include "fsim/World.h"
#include "training/native/mission/MissionConfig.h"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace fsim::mission {

/// A call that found the episode's boundary held: a std::runtime_error to C++ callers, FSIM_ERROR through the C ABI.
class Busy : public std::runtime_error {
public:
    explicit Busy(const std::string& what) : std::runtime_error(what) {}
};

/// What a reset made for the gateway: the generation's public facade over its new World, and one Agent per service
/// (ascending service id), each bound as its configuration says.
struct Minted {
    std::uint64_t generation = 0, localEpisodeIndex = 0;
    std::unique_ptr<World> facade;
    std::vector<Agent> agents;
};

class Gateway final : public env::EpisodeJudge {
public:
    Gateway(std::shared_ptr<const Config> config, Minted minted);
    ~Gateway() override;
    Gateway(const Gateway&) = delete;
    Gateway& operator=(const Gateway&) = delete;

    /// Dispatch the closing tick's queued requests: ascending service id, then request id.
    void beforeStep(session::World& world, std::uint64_t tick) override;
    /// Read the native statuses of open receipts, forget what retention lets go, take each ingress owner's inbox once,
    /// judge the criteria and freeze the new tick's snapshots and referee documents.
    void afterStep(const session::World& world, std::uint64_t tick) override;

    bool serves(std::uint64_t serviceId) const noexcept;
    /// Validate and queue; the acknowledgment (`ma.receipts.v1`). Malformed (std::invalid_argument) for a malformed
    /// document, nothing changed.
    std::string submit(std::uint64_t serviceId, std::string_view actions);
    std::string receipts(std::uint64_t serviceId) const;
    const std::string& observation(std::uint64_t serviceId) const;
    const std::string& outcome() const noexcept { return outcome_; }
    const std::string& truth() const noexcept { return truth_; }
    /// `ma.costs.v1` of the current tick (M7 core): the referee's.
    const std::string& costs() const noexcept { return costs_; }
    /// The generation's episode key, as the schema's `episode_key` object.
    const std::string& episodeKey() const noexcept { return key_; }

    struct Native;
    struct Record;
    struct Service;

private:
    Service& service(std::uint64_t serviceId);
    const Service& service(std::uint64_t serviceId) const;
    void dispatch(Service& s, Record& r);
    void refresh(Service& s, Record& r);
    bool owns(const Service& s, const std::string& capability) const;
    std::optional<std::uint64_t> ownerService(std::uint32_t platform, const std::string& capability) const;
    void judge();
    void freeze();
    std::string observe(Service& s);
    std::string receiptsOf(const Service& s) const;

    std::shared_ptr<const Config> config_;
    std::unique_ptr<World> facade_;
    std::uint64_t generation_ = 0, localEpisodeIndex_ = 0, tick_ = 0;
    std::string key_;
    std::vector<std::unique_ptr<Service>> services_;            ///< ascending service id
    std::map<std::uint64_t, std::uint64_t> activityOwners_;     ///< an activity a service's request made, by its id
    struct Platform {
        std::string name;
        std::uint32_t entity = 0;
        bool c2Node = false;
    };
    std::vector<Platform> platforms_;                           ///< the recipe's, as this generation built them
    struct Judged {
        bool satisfied = false;
        std::uint64_t tick = 0;
        double insideSinceS = 0.0;
        bool inside = false;
    };
    std::vector<Judged> criteria_;
    std::string outcome_, truth_, costs_;
};

} // namespace fsim::mission
