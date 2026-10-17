// Independent mission episodes (MissionEpisode.h; docs/sdk/mission-training.md, sections 4 and 8).

#include "env/MissionEpisode.h"

#include "fsim/Rng.h"
#include "simulation/runtime/world/Scenario.h"
#include "simulation/api/WorldOptions.h"

#include <algorithm>
#include <filesystem>

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
void EpisodeJudge::beforeStep(session::World&, std::uint64_t) {}
void EpisodeJudge::afterStep(const session::World&, std::uint64_t) {}

// --- the owner's state ----------------------------------------------------------------------------------------------

struct Episode {
    std::shared_ptr<const EpisodeConfig> config;
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
    ~Episode() { teardown(); }
};

EpisodeControl::EpisodeControl() : info(std::make_shared<const EpisodeInfo>()) {}
EpisodeControl::~EpisodeControl() = default;

session::World* episodeWorld(const EpisodeControl& control) noexcept { return control.episode ? control.episode->world.get() : nullptr; }

const session::AgentIdentity* episodeService(const EpisodeControl& control, std::uint64_t serviceId) noexcept {
    if (!control.episode) return nullptr;
    for (const auto& [id, agent] : control.episode->agents)
        if (id == serviceId) return &agent;
    return nullptr;
}

namespace {

[[noreturn]] void refuse(EpisodeStatus status, const std::string& why) { throw EpisodeError(status, "mission episode: " + why); }

/// Publish a snapshot already made, unless the episode has ended: the Closed snapshot is the last one, whatever a call
/// still finishing does. Cannot fail: an exceptional exit publishes its failed state with it.
void publishReady(EpisodeControl& control, std::shared_ptr<const EpisodeInfo> info) noexcept {
    try {
        const std::lock_guard<std::mutex> lock(control.handoff);
        if (!control.closed.load()) std::atomic_store(&control.info, std::move(info));
    } catch (...) {
        // (a mutex that cannot be locked: the metadata keeps its last snapshot)
    }
}
void publish(EpisodeControl& control, EpisodeInfo info) { publishReady(control, std::make_shared<const EpisodeInfo>(std::move(info))); }
std::shared_ptr<const EpisodeInfo> published(const EpisodeControl& control) { return std::atomic_load(&control.info); }

/// The metadata of an open (or failed) generation, as its state now is.
EpisodeInfo infoOf(const Episode& e, std::uint64_t generation) {
    EpisodeInfo i;
    i.status = e.world ? EpisodeStatus::Ok : EpisodeStatus::NotReset;
    i.generation = generation, i.localEpisodeIndex = e.localIndex, i.tick = e.tick;
    i.worldSeed = generation ? episode_seeds::world(e.config->baseSeed, e.config->envId, e.localIndex) : 0;
    i.worldName = e.worldName, i.recordPath = e.recordPath;
    return i;
}

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

/// A scenario's world section, held to what the recipe admits: its construction inputs taken (EpisodeWorld), and
/// anything else it would set refused rather than dropped.
void admitScenarioWorld(EpisodeConfig& c) {
    if (!c.scenario) return;
    if (c.scenario->vecenv.present) refuse(EpisodeStatus::Ok, "a scenario's vecenv section does not configure a mission episode");
    // Exact arity makes an added public option a required admission-policy review, as well as a native-mapping
    // review in session::makeWorldOptions. Every field is admitted, episode-owned or name-checked below.
    const auto& [name, dt, frameSkip, workers, pinWorkers, seed, capacity, publish, publishIntervalSeconds, jsbsimRoot,
                 terrain, terrainUrl, terrainZoom, terrainOffline, storeDrag, ground, recordPath, recordIntervalSeconds] = c.scenario->world;
    const WorldOptions d;
    if (dt != d.dt || frameSkip != d.frameSkip || workers != d.workers || pinWorkers != d.pinWorkers || seed != d.seed ||
        publish != d.publish || publishIntervalSeconds != d.publishIntervalSeconds || storeDrag != d.storeDrag || ground ||
        !recordPath.empty() || recordIntervalSeconds != d.recordIntervalSeconds)
        refuse(EpisodeStatus::Ok, "the scenario's world section may set only its name, capacity, JSBSim root and terrain: timing, workers, "
                                  "seed, publication, recording and ground are the episode's");
    if (name != d.name && name != c.name) refuse(EpisodeStatus::Ok, "the scenario names its World '" + name + "', the episode '" + c.name + "'");
    if (c.world != EpisodeWorld{}) refuse(EpisodeStatus::Ok, "with a scenario, its world section gives the construction inputs: leave the episode's own at their defaults");
    c.world.capacity = capacity;
    c.world.jsbsimRoot = jsbsimRoot;
    c.world.terrain = terrain;
    c.world.terrainUrl = terrainUrl;
    c.world.terrainZoom = terrainZoom;
    c.world.terrainOffline = terrainOffline;
}

} // namespace

// --- the boundary and the children ----------------------------------------------------------------------------------

EpisodeBoundary::EpisodeBoundary(std::shared_ptr<EpisodeControl> control) noexcept : control_(std::move(control)) {
    bool expected = false;
    held_ = control_->busy.compare_exchange_strong(expected, true);
}

EpisodeBoundary::~EpisodeBoundary() {
    if (!held_) return;
    std::unique_ptr<Episode> retired;
    {
        // (under the hand-off lock: an ending episode either finds the boundary still held, and hands its state to
        // this release, or finds it free and ends the state itself)
        const std::lock_guard<std::mutex> lock(control_->handoff);
        retired = std::move(control_->retired);
        if (retired) control_->episode = nullptr; // (the state it pointed to ends below)
        control_->busy.store(false);
    }
    // The episode ended during this call: its World goes now that the call no longer uses it.
}

namespace {
/// From the published state, without the boundary.
EpisodeStatus childStatus(const EpisodeControl* control, std::uint64_t generation, bool known) noexcept {
    if (!control || control->closed.load()) return EpisodeStatus::Closed;
    if (control->generation.load() != generation) return EpisodeStatus::Stale;
    if (!known) return EpisodeStatus::UnknownService;
    const auto info = published(*control);
    return info->generation == generation && info->status == EpisodeStatus::Ok ? EpisodeStatus::Ok : EpisodeStatus::NotReset;
}
/// Under the boundary: from the state itself.
EpisodeStatus childChecked(const EpisodeControl* control, std::uint64_t generation, bool known) noexcept {
    if (!control || control->closed.load() || !control->episode) return EpisodeStatus::Closed;
    if (control->generation.load() != generation) return EpisodeStatus::Stale;
    if (!known) return EpisodeStatus::UnknownService;
    return control->episode->world ? EpisodeStatus::Ok : EpisodeStatus::NotReset;
}
} // namespace

EpisodeStatus EpisodeService::status() const noexcept { return childStatus(control_.get(), generation_, known_); }
EpisodeStatus EpisodeService::checked() const noexcept { return childChecked(control_.get(), generation_, known_); }
EpisodeStatus EpisodeReferee::status() const noexcept { return childStatus(control_.get(), generation_, true); }
EpisodeStatus EpisodeReferee::checked() const noexcept { return childChecked(control_.get(), generation_, true); }
EpisodeStatus EpisodeContext::status() const noexcept { return childStatus(control_.get(), generation_, true); }
EpisodeStatus EpisodeContext::checked() const noexcept { return childChecked(control_.get(), generation_, true); }

EpisodeJudge* episodeJudge(const EpisodeControl& control) noexcept { return control.episode ? control.episode->judge.get() : nullptr; }

// --- the owner --------------------------------------------------------------------------------------------------------

MissionEpisode::MissionEpisode(EpisodeConfig config) : control_(std::make_shared<EpisodeControl>()), state_(std::make_unique<Episode>()) {
    EpisodeConfig& c = config;
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
    admitScenarioWorld(c);
    // (services bound in ascending id whatever the order given: the order decides nothing, section 7)
    std::sort(c.services.begin(), c.services.end(), [](const ServiceSpec& a, const ServiceSpec& b) { return a.serviceId < b.serviceId; });
    config_ = std::make_shared<const EpisodeConfig>(std::move(config));
    state_->config = config_;
    control_->episode = state_.get();
}

MissionEpisode::~MissionEpisode() { close(); }

MissionEpisode::MissionEpisode(MissionEpisode&& other) noexcept
    : control_(std::move(other.control_)), config_(std::move(other.config_)), state_(std::move(other.state_)) {}

MissionEpisode& MissionEpisode::operator=(MissionEpisode&& other) noexcept {
    if (this != &other) {
        close();
        control_ = std::move(other.control_);
        config_ = std::move(other.config_);
        state_ = std::move(other.state_);
    }
    return *this;
}

void MissionEpisode::close() noexcept {
    if (!control_) return;
    std::shared_ptr<EpisodeControl> control = std::move(control_);
    std::unique_ptr<Episode> state = std::move(state_);
    config_.reset();
    {
        const std::lock_guard<std::mutex> lock(control->handoff);
        // Closed first: no call that takes the boundary from here on reaches the state.
        control->closed.store(true);
        // A call in progress - its callback perhaps this very disposal - still holds the World: it ends it on release.
        if (control->busy.load()) control->retired = std::move(state);
        else control->episode = nullptr;
        EpisodeInfo closed;
        closed.status = EpisodeStatus::Closed;
        std::atomic_store(&control->info, std::make_shared<const EpisodeInfo>(std::move(closed)));
    }
    // (with the boundary free, the state ends here; `episode` is cleared above under the lock, and a call taking the
    // boundary after that finds `closed` and touches nothing)
    state.reset();
}

EpisodeStatus MissionEpisode::enter(const EpisodeBoundary& boundary, session::World*& world) const noexcept {
    if (!boundary.held()) return EpisodeStatus::Busy;
    if (control_->closed.load() || !control_->episode) return EpisodeStatus::Closed;
    world = control_->episode->world.get();
    return world ? EpisodeStatus::Ok : EpisodeStatus::NotReset;
}

EpisodeJudge* MissionEpisode::currentJudge() noexcept { return control_ && control_->episode ? control_->episode->judge.get() : nullptr; }

std::uint64_t MissionEpisode::reset(std::uint64_t localEpisodeIndex) { return reset(localEpisodeIndex, ResetHooks{}); }

std::uint64_t MissionEpisode::reset(std::uint64_t localEpisodeIndex, const ResetHooks& hooks) {
    if (!control_) refuse(EpisodeStatus::Closed, "the episode was moved from");
    // Locals only from here: the recipe or the judge factory may end this episode, and with it this object.
    const std::shared_ptr<EpisodeControl> control = control_;
    EpisodeBoundary boundary(control);
    if (!boundary.held()) refuse(EpisodeStatus::Busy, "reset while another call holds the episode");
    return resetHeld(control, localEpisodeIndex, hooks);
}

std::uint64_t MissionEpisode::resetHeld(const std::shared_ptr<EpisodeControl>& control, std::uint64_t localEpisodeIndex, const ResetHooks& hooks) {
    if (control->closed.load() || !control->episode) refuse(EpisodeStatus::Closed, "the episode is closed");
    Episode& e = *control->episode;
    const EpisodeConfig& c = *e.config;
    const std::uint64_t generation = control->generation.load() + 1;
    if (hooks.prepare) hooks.prepare(generation); // (under the boundary, before anything of the episode changes)
    // The snapshots this reset may have to publish on its way out are made before anything changes, so that every exit
    // - however it is thrown - leaves coherent metadata: the generation's, with no World and not building.
    EpisodeInfo failedInfo;
    failedInfo.status = EpisodeStatus::NotReset;
    failedInfo.generation = generation, failedInfo.localEpisodeIndex = localEpisodeIndex;
    failedInfo.worldSeed = episode_seeds::world(c.baseSeed, c.envId, localEpisodeIndex);
    EpisodeInfo buildingInfo = failedInfo;
    buildingInfo.building = true;
    std::shared_ptr<const EpisodeInfo> failed = std::make_shared<const EpisodeInfo>(failedInfo);
    std::shared_ptr<const EpisodeInfo> building = std::make_shared<const EpisodeInfo>(buildingInfo);

    // Every earlier child is stale from here on; then the old World and all its generation's state go.
    control->generation.store(generation);
    e.teardown();
    e.localIndex = localEpisodeIndex;
    publishReady(*control, std::move(building));
    try {
        WorldOptions o;
        const std::string name = c.diagnostic ? c.name + "-e" + std::to_string(c.envId) + "-g" + std::to_string(generation) : c.name;
        o.name = name;
        o.dt = c.dtS, o.frameSkip = c.frameSkip;
        o.workers = c.workers, o.pinWorkers = false;
        o.seed = failedInfo.worldSeed;
        // The admitted subset is exhaustive too: adding an episode construction field needs an explicit mapping.
        const auto& [capacity, jsbsimRoot, terrain, terrainUrl, terrainZoom, terrainOffline] = c.world;
        o.capacity = capacity;
        o.jsbsimRoot = jsbsimRoot;
        o.terrain = terrain;
        o.terrainUrl = terrainUrl;
        o.terrainZoom = terrainZoom;
        o.terrainOffline = terrainOffline;
        o.publish = c.diagnostic && c.publish;
        const std::string record = c.diagnostic && !c.recordDir.empty() ? (std::filesystem::path(c.recordDir) / (name + ".fsrec")).string() : std::string();
        o.recordPath = record;
        auto world = std::make_unique<session::World>(session::makeWorldOptions(o));
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
        if (hooks.complete) hooks.complete(e.judge.get()); // (the caller's result, read before the boundary goes)
        publishReady(*control, std::make_shared<const EpisodeInfo>(infoOf(e, generation)));
        return generation;
    } catch (...) {
        // Whatever was thrown, by the recipe, the native setup, the judge factory or an allocation: no World, the failed
        // generation's metadata, the boundary released on the way out.
        e.teardown();
        publishReady(*control, std::move(failed));
        try {
            throw;
        } catch (const EpisodeError&) {
            throw; // (the episode's own refusal, or a callback's, as it is)
        } catch (const std::exception& error) {
            refuse(EpisodeStatus::NotReset, std::string("the recipe failed: ") + error.what());
        }
        // (anything else - a callback's non-standard exception - leaves here as it was thrown)
    }
}

EpisodeStatus MissionEpisode::step() {
    if (!control_) return EpisodeStatus::Closed;
    const std::shared_ptr<EpisodeControl> control = control_; // (the judge may end this episode)
    EpisodeBoundary boundary(control);
    if (!boundary.held()) return EpisodeStatus::Busy;
    return stepHeld(control);
}

EpisodeStatus MissionEpisode::stepHeld(const std::shared_ptr<EpisodeControl>& control) {
    if (control->closed.load() || !control->episode) return EpisodeStatus::Closed;
    session::World* world = control->episode->world.get();
    if (!world) return EpisodeStatus::NotReset;
    Episode& e = *control->episode;
    if (e.judge) e.judge->beforeStep(*world, e.tick);
    world->step(e.config->decisionSteps);
    ++e.tick;
    publish(*control, infoOf(e, control->generation.load()));
    if (e.judge) e.judge->afterStep(*world, e.tick);
    return EpisodeStatus::Ok;
}

// --- the range's compound operation -----------------------------------------------------------------------------------

EpisodeHold::EpisodeHold(MissionEpisode& episode) noexcept : control_(episode.control_) {
    if (!control_) return; // (a moved-from episode: nothing to hold)
    boundary_.emplace(control_);
    held_ = boundary_->held();
}

EpisodeStatus EpisodeHold::step() {
    if (!control_) return EpisodeStatus::Closed;
    if (!held_) return EpisodeStatus::Busy;
    return MissionEpisode::stepHeld(control_);
}

std::uint64_t EpisodeHold::reset(std::uint64_t localEpisodeIndex, const ResetHooks& hooks) {
    if (!control_) refuse(EpisodeStatus::Closed, "the episode was moved from");
    if (!held_) refuse(EpisodeStatus::Busy, "reset while another call holds the episode");
    return MissionEpisode::resetHeld(control_, localEpisodeIndex, hooks);
}

EpisodeJudge* EpisodeHold::judge() noexcept {
    if (!held_ || control_->closed.load() || !control_->episode) return nullptr;
    return control_->episode->judge.get();
}

std::shared_ptr<const EpisodeInfo> MissionEpisode::info() const {
    if (!control_) {
        EpisodeInfo closed;
        closed.status = EpisodeStatus::Closed;
        return std::make_shared<const EpisodeInfo>(closed);
    }
    return published(*control_);
}

std::uint64_t MissionEpisode::generation() const { return info()->generation; }
std::uint64_t MissionEpisode::localEpisodeIndex() const { return info()->localEpisodeIndex; }
std::uint64_t MissionEpisode::tick() const { return info()->tick; }
bool MissionEpisode::open() const { return info()->status == EpisodeStatus::Ok; }
std::string MissionEpisode::worldName() const { return info()->worldName; }
std::string MissionEpisode::recordPath() const { return info()->recordPath; }
std::uint64_t MissionEpisode::worldSeed() const { return info()->worldSeed; }

std::uint64_t MissionEpisode::serviceSeed(std::uint64_t serviceId) const {
    const auto i = info();
    return config_ && i->generation ? episode_seeds::service(config_->baseSeed, config_->envId, i->localEpisodeIndex, serviceId) : 0;
}
std::uint64_t MissionEpisode::componentSeed(std::uint64_t serviceId, std::uint64_t componentId) const {
    const auto i = info();
    return config_ && i->generation ? episode_seeds::component(config_->baseSeed, config_->envId, i->localEpisodeIndex, serviceId, componentId) : 0;
}

EpisodeService MissionEpisode::service(std::uint64_t serviceId) {
    if (!control_ || !config_) return EpisodeService(nullptr, 0, serviceId, false);
    const auto& services = config_->services; // (immutable: no boundary needed)
    const bool known = std::any_of(services.begin(), services.end(), [&](const ServiceSpec& s) { return s.serviceId == serviceId; });
    return EpisodeService(control_, control_->generation.load(), serviceId, known);
}

EpisodeReferee MissionEpisode::referee() {
    if (!control_) return EpisodeReferee(nullptr, 0);
    return EpisodeReferee(control_, control_->generation.load());
}

EpisodeContext MissionEpisode::context() {
    if (!control_) return EpisodeContext(nullptr, 0);
    return EpisodeContext(control_, control_->generation.load());
}

} // namespace fsim::env
