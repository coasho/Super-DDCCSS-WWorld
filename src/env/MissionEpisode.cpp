// Independent mission episodes (MissionEpisode.h; docs/sdk/mission-training.md, sections 4 and 8).

#include "env/MissionEpisode.h"

#include "fsim/Rng.h"
#include "session/Scenario.h"

#include <algorithm>
#include <filesystem>
#include <thread>

namespace fsim::env {

// --- seed streams ---------------------------------------------------------------------------------------------------

namespace episode_seeds {
namespace {
std::uint64_t next(std::uint64_t x) noexcept { return Rng(x).next(); }
} // namespace

std::uint64_t stream(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex, std::uint64_t streamClass) noexcept {
    return next(next(next(next(baseSeed) ^ envId) ^ localEpisodeIndex) ^ streamClass);
}
std::uint64_t world(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex) noexcept {
    return stream(baseSeed, envId, localEpisodeIndex, 0);
}
std::uint64_t service(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex, std::uint64_t serviceId) noexcept {
    return next(stream(baseSeed, envId, localEpisodeIndex, 1) ^ serviceId);
}
std::uint64_t component(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex, std::uint64_t serviceId,
                        std::uint64_t componentId) noexcept {
    return next(service(baseSeed, envId, localEpisodeIndex, serviceId) ^ componentId);
}
} // namespace episode_seeds

const char* episodeStatusName(EpisodeStatus status) noexcept {
    switch (status) {
    case EpisodeStatus::Ok: return "ok";
    case EpisodeStatus::Closed: return "closed";
    case EpisodeStatus::Stale: return "stale";
    case EpisodeStatus::Busy: return "busy";
    case EpisodeStatus::NotReset: return "not_reset";
    case EpisodeStatus::UnknownService: return "unknown_service";
    case EpisodeStatus::NoJudge: return "no_judge";
    }
    return "?";
}

EpisodeJudge::~EpisodeJudge() = default;
void EpisodeJudge::afterStep(const session::World&, std::uint64_t) {}

// --- the owner's state ----------------------------------------------------------------------------------------------

struct Episode {
    EpisodeConfig config;
    std::uint64_t localIndex = 0, tick = 0;
    std::string worldName, recordPath;
    std::unique_ptr<EpisodeJudge> judge;
    std::vector<std::pair<std::uint64_t, session::AgentIdentity>> agents; ///< by ascending service id
    std::unique_ptr<session::World> world;

    /// The generation's own state, in dependency order: its task/judge and Agents, then the World - whose Mission
    /// Systems, guidance, damage, DMS and MA core go with it, its publication and recording ending and closing.
    void teardown() noexcept {
        judge.reset();
        agents.clear();
        world.reset();
        worldName.clear(), recordPath.clear();
        tick = 0;
    }
};

session::World* episodeWorld(Episode& episode) noexcept { return episode.world.get(); }

const session::AgentIdentity* episodeService(Episode& episode, std::uint64_t serviceId) noexcept {
    for (const auto& [id, agent] : episode.agents)
        if (id == serviceId) return &agent;
    return nullptr;
}

namespace {

[[noreturn]] void refuse(EpisodeStatus status, const std::string& why) { throw EpisodeError(status, "mission episode: " + why); }

/// A platform of the recipe by its name: a vehicle's or a C2 node's, unique across both.
std::uint32_t platformOf(const session::World& world, const std::string& name) {
    std::uint32_t found = world.find(name);
    for (const std::uint32_t node : world.c2Nodes())
        if (const comm::C2NodeSpec* spec = world.c2Node(node); spec && spec->name == name) {
            if (found) refuse(EpisodeStatus::NotReset, "platform name '" + name + "' names more than one platform");
            found = node;
        }
    if (!found) refuse(EpisodeStatus::NotReset, "no platform named '" + name + "' in the recipe");
    return found;
}

/// Section 7's recipe on an empty World, section by section, each array in its own order.
void applyRecipe(session::World& world, const EpisodeConfig& c) {
    if (c.scenario) session::applyScenario(world, *c.scenario);
    for (const comm::C2NodeSpec& node : c.c2Nodes)
        if (!world.createC2Node(node)) refuse(EpisodeStatus::NotReset, "C2 node '" + node.name + "' refused");
    for (const EpisodeSystem& s : c.systems)
        if (!world.assignSystem(platformOf(world, s.platform), s.system, s.tailNumber))
            refuse(EpisodeStatus::NotReset, "system of '" + s.platform + "' refused");
    for (const EpisodeNet& n : c.nets) {
        const comm::NetId net = world.createNet(n.spec);
        if (!net) refuse(EpisodeStatus::NotReset, "net '" + n.spec.name + "' refused");
        for (const std::string& member : n.members)
            if (!world.joinNet(platformOf(world, member), net)) refuse(EpisodeStatus::NotReset, "'" + member + "' cannot join net '" + n.spec.name + "'");
    }
    for (const EpisodeDissemination& d : c.dissemination)
        if (!world.setDissemination(platformOf(world, d.platform), d.plan))
            refuse(EpisodeStatus::NotReset, "dissemination plan of '" + d.platform + "' refused");
    for (const EpisodeRbac& r : c.rbac) world.maCore().setRbac(platformOf(world, r.platform), r.config);
    if (c.native) c.native(world);
}

} // namespace

// --- the boundary and the children ----------------------------------------------------------------------------------

EpisodeBoundary::EpisodeBoundary(EpisodeControl& control) noexcept : control_(control) {
    bool expected = false;
    held_ = control_.busy.compare_exchange_strong(expected, true, std::memory_order_acq_rel);
}

EpisodeBoundary::~EpisodeBoundary() {
    if (held_) control_.busy.store(false, std::memory_order_release);
}

namespace {
EpisodeStatus childStatus(const EpisodeControl* control, std::uint64_t generation, EpisodeStatus opened) noexcept {
    if (!control || control->closed.load(std::memory_order_acquire)) return EpisodeStatus::Closed;
    if (control->generation.load(std::memory_order_acquire) != generation) return EpisodeStatus::Stale;
    return opened;
}
} // namespace

EpisodeStatus EpisodeService::status() const noexcept { return childStatus(control_.get(), generation_, opened_); }
EpisodeStatus EpisodeService::checked() const noexcept { return childStatus(control_.get(), generation_, opened_); }
EpisodeStatus EpisodeReferee::status() const noexcept { return childStatus(control_.get(), generation_, opened_); }
EpisodeStatus EpisodeReferee::checked() const noexcept { return childStatus(control_.get(), generation_, opened_); }

// --- the owner --------------------------------------------------------------------------------------------------------

MissionEpisode::MissionEpisode(EpisodeConfig config) : control_(std::make_shared<EpisodeControl>()), state_(std::make_unique<Episode>()) {
    const EpisodeConfig& c = config;
    if (c.name.empty()) refuse(EpisodeStatus::Ok, "a World name is required");
    if (!(c.dtS > 0.0) || c.frameSkip < 1 || c.decisionSteps < 1 || c.workers < 1)
        refuse(EpisodeStatus::Ok, "dt must be positive, and frame skip, decision steps and workers at least 1");
    if ((c.publish || !c.recordDir.empty()) && !c.diagnostic)
        refuse(EpisodeStatus::Ok, "publication and recording are diagnostic only (a non-isolated episode)");
    std::vector<std::uint64_t> ids;
    for (const ServiceSpec& s : c.services) {
        if (s.serviceId == 0) refuse(EpisodeStatus::Ok, "a service id is not 0");
        if (std::find(ids.begin(), ids.end(), s.serviceId) != ids.end()) refuse(EpisodeStatus::Ok, "service " + std::to_string(s.serviceId) + " given twice");
        ids.push_back(s.serviceId);
    }
    state_->config = std::move(config);
    // (services bound in ascending id whatever the order given: the order decides nothing, section 7)
    std::sort(state_->config.services.begin(), state_->config.services.end(),
              [](const ServiceSpec& a, const ServiceSpec& b) { return a.serviceId < b.serviceId; });
    control_->episode = state_.get();
}

MissionEpisode::~MissionEpisode() { close(); }

MissionEpisode::MissionEpisode(MissionEpisode&& other) noexcept : control_(std::move(other.control_)), state_(std::move(other.state_)) {}

MissionEpisode& MissionEpisode::operator=(MissionEpisode&& other) noexcept {
    if (this != &other) {
        close();
        control_ = std::move(other.control_);
        state_ = std::move(other.state_);
    }
    return *this;
}

void MissionEpisode::close() noexcept {
    if (!control_) return;
    // Closed first, under the boundary: a child still held - or a call in progress elsewhere, waited out here - never
    // reaches the World after this. Then the World goes. (Destroying an episode from inside its own call deadlocks.)
    for (;;) {
        EpisodeBoundary boundary(*control_);
        if (boundary.held()) {
            control_->closed.store(true, std::memory_order_release);
            control_->episode = nullptr;
            break;
        }
        std::this_thread::yield();
    }
    if (state_) state_->teardown();
    state_.reset();
    control_.reset();
}

EpisodeStatus MissionEpisode::enter(const EpisodeBoundary& boundary, session::World*& world) const noexcept {
    if (!boundary.held()) return EpisodeStatus::Busy;
    if (control_->closed.load(std::memory_order_acquire) || !state_) return EpisodeStatus::Closed;
    world = state_->world.get();
    return world ? EpisodeStatus::Ok : EpisodeStatus::NotReset;
}

EpisodeJudge* MissionEpisode::currentJudge() noexcept { return state_ ? state_->judge.get() : nullptr; }

std::uint64_t MissionEpisode::reset(std::uint64_t localEpisodeIndex) {
    if (!control_) refuse(EpisodeStatus::Closed, "the episode was moved from");
    EpisodeBoundary boundary(*control_);
    if (!boundary.held()) refuse(EpisodeStatus::Busy, "reset while another call holds the episode");
    if (control_->closed.load(std::memory_order_acquire) || !state_) refuse(EpisodeStatus::Closed, "the episode is closed");
    Episode& e = *state_;
    // Every earlier child is stale from here on; then the old World and all its generation's state go.
    const std::uint64_t generation = control_->generation.load(std::memory_order_relaxed) + 1;
    control_->generation.store(generation, std::memory_order_release);
    e.teardown();
    e.localIndex = localEpisodeIndex;

    const EpisodeConfig& c = e.config;
    session::WorldOptions o;
    const std::string name = c.diagnostic ? c.name + "-e" + std::to_string(c.envId) + "-g" + std::to_string(generation) : c.name;
    o.name = name;
    o.dt = c.dtS, o.frameSkip = c.frameSkip;
    o.workers = c.workers, o.pinWorkers = false;
    o.seed = episode_seeds::world(c.baseSeed, c.envId, localEpisodeIndex);
    o.capacity = c.capacity;
    o.jsbsimRoot = c.jsbsimRoot;
    o.publish = c.diagnostic && c.publish;
    const std::string record = c.diagnostic && !c.recordDir.empty() ? (std::filesystem::path(c.recordDir) / (name + ".fsrec")).string() : std::string();
    o.recordPath = record;
    try {
        auto world = std::make_unique<session::World>(o);
        applyRecipe(*world, c);
        std::vector<std::pair<std::uint64_t, session::AgentIdentity>> agents;
        for (const ServiceSpec& s : c.services) { // (ascending service id)
            const std::uint32_t platform = platformOf(*world, s.platform);
            const std::string agentName = "episode-service-" + std::to_string(s.serviceId);
            const control::ControllerId controller = world->mintAgent(platform, agentName);
            if (!controller) refuse(EpisodeStatus::NotReset, "service " + std::to_string(s.serviceId) + ": '" + s.platform + "' cannot have an agent");
            session::MaServiceBinding binding;
            binding.serviceId = s.serviceId, binding.platform = platform, binding.controller = controller;
            binding.capabilities = s.commandCapabilities;
            binding.ownsDmsIngress = s.ownsDmsIngress, binding.ownsDmsEgress = s.ownsDmsEgress;
            std::string why;
            if (!world->bindMaService(binding, why)) refuse(EpisodeStatus::NotReset, why);
            agents.emplace_back(s.serviceId, session::AgentIdentity{platform, controller, agentName, world->agents().size() - 1});
        }
        std::unique_ptr<EpisodeJudge> judge = c.judge ? c.judge(*world) : nullptr;
        e.world = std::move(world);
        e.agents = std::move(agents);
        e.judge = std::move(judge);
        e.worldName = name;
        e.recordPath = record;
    } catch (const EpisodeError&) {
        e.teardown();
        throw;
    } catch (const std::exception& error) {
        e.teardown();
        refuse(EpisodeStatus::NotReset, std::string("the recipe failed: ") + error.what());
    }
    return generation;
}

EpisodeStatus MissionEpisode::step() {
    if (!control_) return EpisodeStatus::Closed;
    session::World* world = nullptr;
    EpisodeBoundary boundary(*control_);
    const EpisodeStatus s = enter(boundary, world);
    if (s != EpisodeStatus::Ok) return s;
    Episode& e = *state_;
    world->step(e.config.decisionSteps);
    ++e.tick;
    if (e.judge) e.judge->afterStep(*world, e.tick);
    return EpisodeStatus::Ok;
}

std::uint64_t MissionEpisode::generation() const noexcept { return control_ ? control_->generation.load(std::memory_order_acquire) : 0; }
std::uint64_t MissionEpisode::localEpisodeIndex() const noexcept { return state_ ? state_->localIndex : 0; }
std::uint64_t MissionEpisode::tick() const noexcept { return state_ ? state_->tick : 0; }
bool MissionEpisode::open() const noexcept { return state_ && state_->world; }
std::string MissionEpisode::worldName() const { return state_ ? state_->worldName : std::string(); }
std::string MissionEpisode::recordPath() const { return state_ ? state_->recordPath : std::string(); }

std::uint64_t MissionEpisode::worldSeed() const noexcept {
    return state_ ? episode_seeds::world(state_->config.baseSeed, state_->config.envId, state_->localIndex) : 0;
}
std::uint64_t MissionEpisode::serviceSeed(std::uint64_t serviceId) const noexcept {
    return state_ ? episode_seeds::service(state_->config.baseSeed, state_->config.envId, state_->localIndex, serviceId) : 0;
}
std::uint64_t MissionEpisode::componentSeed(std::uint64_t serviceId, std::uint64_t componentId) const noexcept {
    return state_ ? episode_seeds::component(state_->config.baseSeed, state_->config.envId, state_->localIndex, serviceId, componentId) : 0;
}

EpisodeService MissionEpisode::service(std::uint64_t serviceId) {
    if (!control_ || !state_) return EpisodeService(nullptr, 0, serviceId, EpisodeStatus::Closed);
    const std::uint64_t generation = control_->generation.load(std::memory_order_acquire);
    const auto& services = state_->config.services;
    const bool known = std::any_of(services.begin(), services.end(), [&](const ServiceSpec& s) { return s.serviceId == serviceId; });
    const EpisodeStatus opened = !known ? EpisodeStatus::UnknownService : state_->world ? EpisodeStatus::Ok : EpisodeStatus::NotReset;
    return EpisodeService(control_, generation, serviceId, opened);
}

EpisodeReferee MissionEpisode::referee() {
    if (!control_ || !state_) return EpisodeReferee(nullptr, 0, EpisodeStatus::Closed);
    return EpisodeReferee(control_, control_->generation.load(std::memory_order_acquire), state_->world ? EpisodeStatus::Ok : EpisodeStatus::NotReset);
}

} // namespace fsim::env
