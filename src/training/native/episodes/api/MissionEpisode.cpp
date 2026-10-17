// The public mission episode family (fsim/MissionEpisode.h; docs/sdk/mission-training.md, sections 3 to 5; M5 core).
//
// MissionEpisode wraps the M4 owner (env::MissionEpisode): its recipe is the configuration's, its native step mints one
// Agent per service through the public World facade and binds it (fsim/MAService.h), and its judge is the generation's
// mission gateway. Endpoints and referees are generation-bound children (env::EpisodeContext): each call takes the
// episode's one boundary, answers only in its own generation, and reaches the gateway alone - never the World.

#include "fsim/MissionEpisode.h"

#include "training/native/episodes/runtime/MissionEpisode.h"
#include "fsim/MAService.h"
#include "fsim/World.h"
#include "training/native/mission/MissionConfig.h"
#include "training/native/episodes/api/MissionEpisodeImpl.h"
#include "training/native/mission/MissionGateway.h"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

namespace fsim {

using mission::Minting;

namespace {

[[noreturn]] void lifetime(env::EpisodeStatus status, const char* call) {
    switch (status) {
    case env::EpisodeStatus::Busy: throw mission::Busy(std::string("mission episode: ") + call + ": busy - another call holds the episode");
    case env::EpisodeStatus::NotReset: throw std::runtime_error(std::string("mission episode: ") + call + ": the episode has no World (not reset)");
    case env::EpisodeStatus::Stale: throw std::runtime_error(std::string("mission episode: ") + call + ": a handle of an earlier generation");
    default: throw std::runtime_error(std::string("mission episode: ") + call + ": the episode is closed");
    }
}

/// The platform of a service by its name: a vehicle's or a C2 node's.
std::uint32_t platformNamed(World& world, const std::string& name) {
    for (const std::uint32_t node : world.c2Nodes())
        if (const auto spec = world.referee().c2Node(node); spec && spec->name == name) return node;
    return world.vehicle(name).id();
}

/// The gateway of a child's generation, or the call's failure.
template <typename F>
void withGateway(env::EpisodeContext& context, const char* call, F&& f) {
    const env::EpisodeStatus status = context.use([&](env::EpisodeJudge* judge, session::World&) {
        auto* gateway = dynamic_cast<mission::Gateway*>(judge);
        if (!gateway) throw std::runtime_error(std::string("mission episode: ") + call + ": the generation has no gateway");
        f(*gateway);
    });
    if (status != env::EpisodeStatus::Ok) lifetime(status, call);
}

} // namespace

struct MissionEndpoint::Impl {
    env::EpisodeContext context;
    std::uint64_t serviceId = 0;
};

struct MissionReferee::Impl {
    env::EpisodeContext context;
};

env::ResetHooks mission::resetHooks(const std::shared_ptr<Minting>& minting, std::uint64_t localEpisodeIndex, std::string& key) {
    env::ResetHooks hooks;
    hooks.prepare = [minting, localEpisodeIndex](std::uint64_t generation) {
        minting->generation = generation;
        minting->localEpisodeIndex = localEpisodeIndex;
        minting->minted = {};
    };
    hooks.complete = [&key](env::EpisodeJudge* judge) {
        auto* gateway = dynamic_cast<mission::Gateway*>(judge);
        if (!gateway) throw std::runtime_error("mission episode: reset: the generation has no gateway");
        key = gateway->episodeKey();
    };
    return hooks;
}

// --- MissionEndpoint --------------------------------------------------------------------------------------------------

MissionEndpoint::MissionEndpoint(Impl* impl) noexcept : impl_(impl) {}
MissionEndpoint::~MissionEndpoint() { delete impl_; }
MissionEndpoint::MissionEndpoint(MissionEndpoint&& other) noexcept : impl_(other.impl_) { other.impl_ = nullptr; }
MissionEndpoint& MissionEndpoint::operator=(MissionEndpoint&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = other.impl_;
        other.impl_ = nullptr;
    }
    return *this;
}

std::string MissionEndpoint::observationJsonV1() const {
    if (!impl_) throw std::runtime_error("mission endpoint: moved from");
    std::string out;
    withGateway(impl_->context, "observation", [&](mission::Gateway& g) { out = g.observation(impl_->serviceId); });
    return out;
}

std::string MissionEndpoint::submitJsonV1(std::string_view actionsJsonV1) {
    if (!impl_) throw std::runtime_error("mission endpoint: moved from");
    std::string out;
    withGateway(impl_->context, "submit", [&](mission::Gateway& g) { out = g.submit(impl_->serviceId, actionsJsonV1); });
    return out;
}

std::string MissionEndpoint::receiptsJsonV1() const {
    if (!impl_) throw std::runtime_error("mission endpoint: moved from");
    std::string out;
    withGateway(impl_->context, "receipts", [&](mission::Gateway& g) { out = g.receipts(impl_->serviceId); });
    return out;
}

// --- MissionReferee ---------------------------------------------------------------------------------------------------

MissionReferee::MissionReferee(Impl* impl) noexcept : impl_(impl) {}
MissionReferee::~MissionReferee() { delete impl_; }
MissionReferee::MissionReferee(MissionReferee&& other) noexcept : impl_(other.impl_) { other.impl_ = nullptr; }
MissionReferee& MissionReferee::operator=(MissionReferee&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = other.impl_;
        other.impl_ = nullptr;
    }
    return *this;
}

std::string MissionReferee::outcomeJsonV1() const {
    if (!impl_) throw std::runtime_error("mission referee: moved from");
    std::string out;
    withGateway(impl_->context, "outcome", [&](mission::Gateway& g) { out = g.outcome(); });
    return out;
}

std::string MissionReferee::costsJsonV1() const {
    if (!impl_) throw std::runtime_error("mission referee: moved from");
    std::string out;
    withGateway(impl_->context, "costs", [&](mission::Gateway& g) { out = g.costs(); });
    return out;
}

std::string MissionReferee::truthJsonV1() const {
    if (!impl_) throw std::runtime_error("mission referee: moved from");
    std::string out;
    withGateway(impl_->context, "truth", [&](mission::Gateway& g) { out = g.truth(); });
    return out;
}

// --- MissionEpisode ---------------------------------------------------------------------------------------------------

MissionEpisode::MissionEpisode(std::string_view configJsonV1) {
    std::shared_ptr<const mission::Config> config = mission::parseConfig(configJsonV1);
    auto minting = std::make_shared<Minting>();
    env::EpisodeConfig recipe = config->episode;
    // The native step: after the recipe's sections, one Agent per service in ascending service id, each minted and
    // bound through the public SDK exactly as a range would (fsim/MAService.h). The facade borrows the new World and
    // goes with the gateway, before the World does.
    recipe.native = [minting, config](session::World& world) {
        mission::Minted made;
        made.generation = minting->generation, made.localEpisodeIndex = minting->localEpisodeIndex;
        made.facade.reset(World::borrowed(world));
        for (const mission::ServiceConfig& s : config->services) {
            Agent agent = made.facade->createAgent(platformNamed(*made.facade, s.platform), "episode-service-" + std::to_string(s.serviceId));
            std::string why;
            if (!bindMAService(*made.facade, agent, s.serviceId, s.capabilities, s.ownsDmsIngress, s.ownsDmsEgress, &why))
                throw std::runtime_error("service " + std::to_string(s.serviceId) + " (" + s.name + "): " + why);
            made.agents.push_back(agent);
        }
        minting->minted = std::move(made);
    };
    recipe.judge = [minting, config](const session::World&) -> std::unique_ptr<env::EpisodeJudge> {
        return std::make_unique<mission::Gateway>(config, std::move(minting->minted));
    };
    try {
        impl_ = new Impl(config, minting, std::move(recipe));
    } catch (const env::EpisodeError& e) {
        throw std::invalid_argument(std::string("ma.config.v1: ") + e.what());
    }
}

MissionEpisode::~MissionEpisode() { delete impl_; }
MissionEpisode::MissionEpisode(MissionEpisode&& other) noexcept : impl_(other.impl_) { other.impl_ = nullptr; }
MissionEpisode& MissionEpisode::operator=(MissionEpisode&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = other.impl_;
        other.impl_ = nullptr;
    }
    return *this;
}

std::string MissionEpisode::reset(std::uint64_t localEpisodeIndex) {
    if (!impl_) throw std::runtime_error("mission episode: moved from");
    // One operation under the reset's own boundary: the generation's minting inputs, the native rebuild and the key it
    // returns. A reset refused busy runs neither hook and changes nothing; no call after the rebuild can change or fail
    // the key. Locals only: the episode may end while its reset runs (env::MissionEpisode::reset).
    const std::shared_ptr<Minting> minting = impl_->minting;
    std::string key;
    const env::ResetHooks hooks = mission::resetHooks(minting, localEpisodeIndex, key);
    try {
        impl_->episode.reset(localEpisodeIndex, hooks);
    } catch (const env::EpisodeError& e) {
        if (e.status() == env::EpisodeStatus::Busy) throw mission::Busy("mission episode: reset: busy - another call holds the episode");
        throw std::runtime_error(e.what());
    }
    return key;
}

void MissionEpisode::step() {
    if (!impl_) throw std::runtime_error("mission episode: moved from");
    const env::EpisodeStatus status = impl_->episode.step();
    if (status != env::EpisodeStatus::Ok) lifetime(status, "step");
}

MissionEndpoint MissionEpisode::endpoint(std::uint64_t serviceId) {
    if (!impl_) throw std::runtime_error("mission episode: moved from");
    if (!impl_->config->service(serviceId)) throw std::invalid_argument("mission episode: no configured service " + std::to_string(serviceId));
    env::EpisodeContext context = impl_->episode.context();
    if (const env::EpisodeStatus status = context.status(); status != env::EpisodeStatus::Ok) lifetime(status, "endpoint");
    return MissionEndpoint(new MissionEndpoint::Impl{std::move(context), serviceId});
}

bool MissionEpisode::removePlatform(std::string_view platform) {
    if (!impl_) throw std::runtime_error("mission episode: moved from");
    const std::string name(platform);
    const auto& names = impl_->config->platforms;
    if (std::find(names.begin(), names.end(), name) == names.end())
        throw std::invalid_argument("mission episode: no platform '" + name + "' in the configuration");
    bool removed = false;
    env::EpisodeContext context = impl_->episode.context();
    const env::EpisodeStatus status = context.use([&](env::EpisodeJudge*, session::World& world) {
        // the World's own removal (a vehicle's or a C2 node's): its bindings end, its services' roles at the next tick
        for (const std::uint32_t node : world.c2Nodes())
            if (const comm::C2NodeSpec* spec = world.c2Node(node); spec && spec->name == name) {
                removed = world.removeC2Node(node);
                return;
            }
        if (const std::uint32_t vehicle = world.find(name)) removed = world.removeVehicle(vehicle);
    });
    if (status != env::EpisodeStatus::Ok) lifetime(status, "removePlatform");
    return removed;
}

MissionReferee MissionEpisode::referee() {
    if (!impl_) throw std::runtime_error("mission episode: moved from");
    env::EpisodeContext context = impl_->episode.context();
    if (const env::EpisodeStatus status = context.status(); status != env::EpisodeStatus::Ok) lifetime(status, "referee");
    return MissionReferee(new MissionReferee::Impl{std::move(context)});
}

} // namespace fsim
