#pragma once

// Independent mission episodes (M4; docs/sdk/mission-training.md, sections 4 and 8). The internal owner the public
// MissionEpisode facade (M5) is built on.
//
// An episode owns exactly one World at a time, and with it everything mutable of the episode: the World's Mission
// Systems, guidance, damage, DMS and MA core state, the episode's decision tick, its task/judge state and its random
// streams. A reset destroys that World and builds a new one from the episode's deterministic recipe; it never stands
// in for, or changes, the legacy Vehicle reset. Children - a service's context, the referee's - are bound to the
// generation they were opened in, through a small control block that outlives the World: a reset makes them stale, the
// parent's destruction closes them before the World goes, and moving the parent keeps them.
//
// Every call that touches the World - reset, step, a child's call, the range's own access - holds the episode's one
// boundary for its duration. A call that finds it held fails as Busy at once; none waits. Ending the episode never
// waits either: a World a call is using outlives the episode until that call returns. Metadata is published whole and
// readable at any time (EpisodeInfo).

#include "fsim/Comm.h"
#include "fsim/Scenario.h"
#include "session/AgentRules.h"
#include "session/World.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fsim::env {

/// Section 8's seed streams: functions of their arguments alone - not of a generation, another episode or a schedule.
/// With next(x) = fsim::Rng(x).next() (SplitMix64), arithmetic modulo 2^64:
///   stream(class) = next(next(next(next(base) ^ env) ^ local) ^ class); world = stream(0);
///   service(id) = next(stream(1) ^ id); component(id, c) = next(service(id) ^ c).
namespace episode_seeds {
std::uint64_t stream(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex, std::uint64_t streamClass) noexcept;
std::uint64_t world(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex) noexcept;
std::uint64_t service(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex, std::uint64_t serviceId) noexcept;
std::uint64_t component(std::uint64_t baseSeed, std::uint64_t envId, std::uint64_t localEpisodeIndex, std::uint64_t serviceId,
                        std::uint64_t componentId) noexcept;
} // namespace episode_seeds

/// How a call on an episode or one of its children went.
enum class EpisodeStatus : std::uint8_t {
    Ok,
    Closed,         ///< the episode was destroyed or moved from
    Stale,          ///< the child belongs to an earlier generation
    Busy,           ///< another call holds the episode's boundary
    NotReset,       ///< no World: never reset, the last reset failed, or a reset is building it
    UnknownService, ///< no service of that id in this episode's configuration
    NoJudge,        ///< the configuration makes no task/judge
};
const char* episodeStatusName(EpisodeStatus status) noexcept;

/// A refused construction or reset, with its status and why: Ok for an invalid configuration; Busy or Closed for a
/// reset refused with nothing changed; NotReset for a reset whose recipe failed, leaving the episode with no World.
class EpisodeError : public std::runtime_error {
public:
    EpisodeError(EpisodeStatus status, const std::string& what) : std::runtime_error(what), status_(status) {}
    EpisodeStatus status() const noexcept { return status_; }

private:
    EpisodeStatus status_;
};

// The configuration's recipe, section 7's in native form: applied to an empty World on every reset, section by section
// in this order, each array in its own order (array order is native creation order).
struct EpisodeSystem {
    std::string platform;                    ///< a scenario vehicle's or C2 node's name
    comm::SystemId system = comm::kNoSystem; ///< kNoSystem: the next free one
    std::string tailNumber;
};
struct EpisodeNet {
    comm::NetSpec spec;
    std::vector<std::string> members; ///< joined in this order
};
struct EpisodeDissemination {
    std::string platform;
    comm::DisseminationPlan plan;
};
struct EpisodeRbac {
    std::string platform;
    comm::RbacConfig config;
};
/// One MA service: its Agent is minted at every reset, in ascending service id, and bound (section 9).
struct ServiceSpec {
    std::uint64_t serviceId = 0;
    std::string platform;
    std::vector<std::string> commandCapabilities;
    bool ownsDmsIngress = false, ownsDmsEgress = false;
};

/// An episode's own task/judge state: made afresh by the configuration's factory at every reset, told of every
/// decision tick, and never shared with another episode or generation.
class EpisodeJudge {
public:
    virtual ~EpisodeJudge();
    /// After the tick's World steps: the World, read-only, and the tick just completed (1 for the first).
    virtual void afterStep(const session::World& world, std::uint64_t tick);
};

/// The World construction inputs the recipe admits (section 7: a scenario's world section may hold its name, capacity,
/// JSBSim root and terrain keys). With a scenario they are that scenario's `world` section's, and the episode's own
/// `world` must be left at its defaults; without one, they are these. They apply to every reset's World.
struct EpisodeWorld {
    std::uint32_t capacity = 256;     ///< published vehicle slots
    std::string jsbsimRoot;           ///< empty: auto-detect
    bool terrain = false;             ///< physics ground from elevation tiles
    std::string terrainUrl;           ///< tile template; empty: the native default
    unsigned terrainZoom = 15;
    bool terrainOffline = true;       ///< the local tiles alone, never fetched
    bool operator==(const EpisodeWorld& o) const {
        return capacity == o.capacity && jsbsimRoot == o.jsbsimRoot && terrain == o.terrain && terrainUrl == o.terrainUrl &&
               terrainZoom == o.terrainZoom && terrainOffline == o.terrainOffline;
    }
    bool operator!=(const EpisodeWorld& o) const { return !(*this == o); }
};

struct EpisodeConfig {
    std::string name = "episode";                  ///< the World's name; a diagnostic one's is "<name>-e<env>-g<generation>"
    std::uint64_t envId = 0, baseSeed = 0;         ///< seed inputs (section 8)
    double dtS = 1.0 / 120.0;                      ///< WorldOptions::dt
    int frameSkip = 4;                             ///< WorldOptions::frameSkip
    unsigned decisionSteps = 1;                    ///< World steps per decision tick (step())
    unsigned workers = 1;                          ///< World worker threads (unpinned)
    EpisodeWorld world;                            ///< construction inputs without a scenario (with one: its own)
    bool diagnostic = false;                       ///< non-isolated: required for publish and recordDir
    bool publish = false;                          ///< publish the World (diagnostic only)
    std::string recordDir;                         ///< an existing directory: record to "<recordDir>/<world name>.fsrec" (diagnostic only)
    /// Its environment, effects and vehicles (session::applyScenario), and its world section's admitted construction
    /// inputs (EpisodeWorld). A world section that sets anything else - timing, workers, seed, publication, recording,
    /// a ground provider - or names the World otherwise than `name` is refused: the episode decides those.
    std::optional<Scenario> scenario;
    std::vector<comm::C2NodeSpec> c2Nodes;
    std::vector<EpisodeSystem> systems;
    std::vector<EpisodeNet> nets;
    std::vector<EpisodeDissemination> dissemination;
    std::vector<EpisodeRbac> rbac;
    /// The range's own deterministic native setup, after the sections above and before the services: whatever else
    /// the scenario needs from existing native calls. It receives the new World and keeps nothing of it.
    std::function<void(session::World&)> native;
    std::vector<ServiceSpec> services;             ///< any order: bound in ascending service id
    /// Makes the episode's task/judge for each new World; empty: none.
    std::function<std::unique_ptr<EpisodeJudge>(const session::World&)> judge;
};

/// The episode's metadata, published whole at each change - construction, a reset's start and end, each step, the
/// episode's end - and read whole: never half of one state and half of another, and never through the boundary, so it
/// is readable at any time, from any thread, even while a call holds the boundary.
struct EpisodeInfo {
    /// Ok: the generation has its World. NotReset: it has none - before the first reset, after a failed one, or while
    /// `building`. Closed: the episode was destroyed or moved from.
    EpisodeStatus status = EpisodeStatus::NotReset;
    bool building = false;                 ///< a reset is building this generation's World now
    std::uint64_t generation = 0, localEpisodeIndex = 0, tick = 0;
    std::uint64_t worldSeed = 0;           ///< the generation's (0 before the first reset)
    std::string worldName, recordPath;     ///< the generation's World's ("" without one; recordPath "" when not recording)
};

struct Episode; // the owner's state (MissionEpisode.cpp)

/// What parent and children share. It outlives the World and is freed with its last holder.
struct EpisodeControl {
    EpisodeControl();
    ~EpisodeControl();
    std::atomic<bool> busy{false};            ///< the boundary
    std::atomic<bool> closed{false};          ///< set once, when the episode ends; never cleared
    std::atomic<std::uint64_t> generation{0}; ///< advanced by each reset, under the boundary
    Episode* episode = nullptr;               ///< the owner's state while open: read and written under the boundary
    std::shared_ptr<const EpisodeInfo> info;  ///< published metadata (std::atomic_load / std::atomic_store)
    /// Orders an episode's end against the boundary's release. An episode ended while a call holds the boundary - from
    /// inside that call's callback, or from another thread - hands its state here, World and all; the call releasing
    /// the boundary ends it, after its callback returned. Nothing waits.
    std::mutex handoff;
    std::unique_ptr<Episode> retired;
};

/// The boundary, held for one call: acquired at once or not at all. It keeps the control block alive for the call -
/// the parent may end inside it - and on release ends a state handed to it.
class EpisodeBoundary {
public:
    explicit EpisodeBoundary(std::shared_ptr<EpisodeControl> control) noexcept;
    ~EpisodeBoundary();
    EpisodeBoundary(const EpisodeBoundary&) = delete;
    EpisodeBoundary& operator=(const EpisodeBoundary&) = delete;
    bool held() const noexcept { return held_; }

private:
    std::shared_ptr<EpisodeControl> control_;
    bool held_ = false;
};

/// A child's view of the owner's state for one call, under the boundary.
session::World* episodeWorld(const EpisodeControl& control) noexcept;
const session::AgentIdentity* episodeService(const EpisodeControl& control, std::uint64_t serviceId) noexcept;

/// One service's context in one generation: what M5's endpoint is built on. It answers Ok only while its generation is
/// current and has its World - including a generation whose reset was still building the World when it was opened.
class EpisodeService {
public:
    EpisodeService() = default;
    EpisodeService(std::shared_ptr<EpisodeControl> control, std::uint64_t generation, std::uint64_t serviceId, bool known)
        : control_(std::move(control)), generation_(generation), serviceId_(serviceId), known_(known) {}
    std::uint64_t serviceId() const noexcept { return serviceId_; }
    std::uint64_t generation() const noexcept { return generation_; }
    /// Whether it would answer now, from the published state (Busy is not reported: the boundary is not taken).
    EpisodeStatus status() const noexcept;
    /// f(World&, its Agent's identity) under the boundary, in its generation only; otherwise the status, f not called.
    /// The World and identity are valid until f returns - even if the episode ends inside f.
    template <typename F>
    EpisodeStatus use(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        EpisodeBoundary boundary(control_);
        if (!boundary.held()) return EpisodeStatus::Busy;
        const EpisodeStatus s = checked();
        if (s != EpisodeStatus::Ok) return s;
        session::World* world = episodeWorld(*control_);
        const session::AgentIdentity* agent = episodeService(*control_, serviceId_);
        if (!world) return EpisodeStatus::NotReset;
        if (!agent) return EpisodeStatus::UnknownService;
        f(*world, *agent);
        return EpisodeStatus::Ok;
    }

private:
    EpisodeStatus checked() const noexcept; ///< under the boundary
    std::shared_ptr<EpisodeControl> control_;
    std::uint64_t generation_ = 0, serviceId_ = 0;
    bool known_ = false;
};

/// The referee's read of one generation's World: what M5's referee documents are built on.
class EpisodeReferee {
public:
    EpisodeReferee() = default;
    EpisodeReferee(std::shared_ptr<EpisodeControl> control, std::uint64_t generation) : control_(std::move(control)), generation_(generation) {}
    std::uint64_t generation() const noexcept { return generation_; }
    EpisodeStatus status() const noexcept;
    /// f(const World&) under the boundary, in its generation only; valid until f returns, as for EpisodeService::use.
    template <typename F>
    EpisodeStatus inspect(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        EpisodeBoundary boundary(control_);
        if (!boundary.held()) return EpisodeStatus::Busy;
        const EpisodeStatus s = checked();
        if (s != EpisodeStatus::Ok) return s;
        const session::World* world = episodeWorld(*control_);
        if (!world) return EpisodeStatus::NotReset;
        f(*world);
        return EpisodeStatus::Ok;
    }

private:
    EpisodeStatus checked() const noexcept;
    std::shared_ptr<EpisodeControl> control_;
    std::uint64_t generation_ = 0;
};

class MissionEpisode {
public:
    /// Validates the configuration and builds no World: generation 0. EpisodeError for: a name empty, dt not
    /// positive, frameSkip, decisionSteps or workers 0, publication or recording without diagnostic, a service id 0
    /// or given twice, and a scenario world section the recipe does not admit (EpisodeConfig::scenario).
    explicit MissionEpisode(EpisodeConfig config);
    /// Ends the episode: its children closed at once. Its World goes now - or, when a call holds the boundary (from
    /// inside that call, or another thread's), when that call returns, so what it holds stays valid. Never waits.
    ~MissionEpisode();
    MissionEpisode(MissionEpisode&& other) noexcept;
    /// Ends this episode as the destructor does, then takes the other's: its identity, children and World.
    MissionEpisode& operator=(MissionEpisode&& other) noexcept;
    MissionEpisode(const MissionEpisode&) = delete;
    MissionEpisode& operator=(const MissionEpisode&) = delete;

    /// Advance the generation - every earlier child stale from here on - destroy the owned World, if any, and build a
    /// fresh one for local episode `localEpisodeIndex`: world seed, recipe, then one Agent per service in ascending id
    /// and its binding, then the judge. The new generation (the previous plus 1). EpisodeError: Busy or Closed, nothing
    /// changed. A reset that fails leaves the episode with no World until a later reset succeeds, its metadata the
    /// failed generation's (NotReset, not building) and the boundary free - whatever was thrown: the episode's own
    /// EpisodeError, a std::exception from a callback as EpisodeError (NotReset), anything else as it was thrown.
    std::uint64_t reset(std::uint64_t localEpisodeIndex);
    /// One decision tick: decisionSteps World steps, then the judge. NotReset, Busy or Closed otherwise.
    EpisodeStatus step();

    /// The published metadata (EpisodeInfo); the getters below read one snapshot each.
    std::shared_ptr<const EpisodeInfo> info() const;
    std::uint64_t generation() const;
    std::uint64_t localEpisodeIndex() const;
    std::uint64_t tick() const;            ///< decision ticks since the last reset
    bool open() const;                     ///< the current generation has its World
    std::string worldName() const;
    std::string recordPath() const;
    std::uint64_t worldSeed() const;
    std::uint64_t serviceSeed(std::uint64_t serviceId) const;
    std::uint64_t componentSeed(std::uint64_t serviceId, std::uint64_t componentId) const;

    /// The current generation's children. Opening takes no boundary and never fails busy: a child binds to the
    /// generation current when it is opened (during a reset, the one being built) and its calls take the boundary.
    /// A service the configuration does not name answers UnknownService.
    EpisodeService service(std::uint64_t serviceId);
    EpisodeReferee referee();

    /// The range's own access to the current World, under the boundary: scenario setup and inspection. The World is
    /// valid until f returns, even if the episode ends inside f.
    template <typename F>
    EpisodeStatus range(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        EpisodeBoundary boundary(control_);
        session::World* world = nullptr;
        const EpisodeStatus s = enter(boundary, world);
        if (s != EpisodeStatus::Ok) return s;
        f(*world);
        return EpisodeStatus::Ok; // (touches nothing of this object: it may have ended inside f)
    }
    /// f(EpisodeJudge&) under the boundary; NoJudge when the configuration makes none.
    template <typename F>
    EpisodeStatus withJudge(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        EpisodeBoundary boundary(control_);
        session::World* world = nullptr;
        const EpisodeStatus s = enter(boundary, world);
        if (s != EpisodeStatus::Ok) return s;
        EpisodeJudge* judge = currentJudge();
        if (!judge) return EpisodeStatus::NoJudge;
        f(*judge);
        return EpisodeStatus::Ok;
    }

private:
    EpisodeStatus enter(const EpisodeBoundary& boundary, session::World*& world) const noexcept;
    EpisodeJudge* currentJudge() noexcept;
    void close() noexcept;

    std::shared_ptr<EpisodeControl> control_;   ///< null once moved from
    std::shared_ptr<const EpisodeConfig> config_; ///< immutable after construction
    std::unique_ptr<Episode> state_;
};

} // namespace fsim::env
