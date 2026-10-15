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
// boundary for its duration. A call that finds it held fails as Busy at once; none waits.

#include "fsim/Comm.h"
#include "fsim/Scenario.h"
#include "session/AgentRules.h"
#include "session/World.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
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
    NotReset,       ///< no World: never reset, or the last reset failed
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

struct EpisodeConfig {
    std::string name = "episode";                  ///< the World's name; a diagnostic one's is "<name>-e<env>-g<generation>"
    std::uint64_t envId = 0, baseSeed = 0;         ///< seed inputs (section 8)
    double dtS = 1.0 / 120.0;                      ///< WorldOptions::dt
    int frameSkip = 4;                             ///< WorldOptions::frameSkip
    unsigned decisionSteps = 1;                    ///< World steps per decision tick (step())
    unsigned workers = 1;                          ///< World worker threads (unpinned)
    std::uint32_t capacity = 256;                  ///< WorldOptions::capacity
    std::string jsbsimRoot;                        ///< empty: auto-detect
    bool diagnostic = false;                       ///< non-isolated: required for publish and recordDir
    bool publish = false;                          ///< publish the World (diagnostic only)
    std::string recordDir;                         ///< an existing directory: record to "<recordDir>/<world name>.fsrec" (diagnostic only)
    std::optional<Scenario> scenario;              ///< its environment, effects and vehicles (session::applyScenario); its
                                                   ///< `world` options are not used - the fields above set the World
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

struct Episode; // the owner's state (MissionEpisode.cpp)

/// What parent and children share: the boundary, the closed flag and the current generation. It outlives the World
/// and is freed with its last holder.
struct EpisodeControl {
    std::atomic<bool> busy{false};
    std::atomic<bool> closed{false};            ///< set under the boundary, never cleared
    std::atomic<std::uint64_t> generation{0};   ///< advanced under the boundary by each reset
    Episode* episode = nullptr;     ///< the owner's state while open (stable across moves of the owner)
};

/// The boundary, held for one call: acquired at once or not at all.
class EpisodeBoundary {
public:
    explicit EpisodeBoundary(EpisodeControl& control) noexcept;
    ~EpisodeBoundary();
    EpisodeBoundary(const EpisodeBoundary&) = delete;
    EpisodeBoundary& operator=(const EpisodeBoundary&) = delete;
    bool held() const noexcept { return held_; }

private:
    EpisodeControl& control_;
    bool held_ = false;
};

/// A child's view of the owner's state for one call.
session::World* episodeWorld(Episode& episode) noexcept;
const session::AgentIdentity* episodeService(Episode& episode, std::uint64_t serviceId) noexcept;

/// One service's context in one generation: what M5's endpoint is built on.
class EpisodeService {
public:
    EpisodeService() = default;
    EpisodeService(std::shared_ptr<EpisodeControl> control, std::uint64_t generation, std::uint64_t serviceId, EpisodeStatus opened)
        : control_(std::move(control)), generation_(generation), serviceId_(serviceId), opened_(opened) {}
    std::uint64_t serviceId() const noexcept { return serviceId_; }
    std::uint64_t generation() const noexcept { return generation_; }
    /// Whether it would answer now (Busy is not reported: the boundary is not taken).
    EpisodeStatus status() const noexcept;
    /// f(World&, its Agent's identity) under the boundary, in its generation only; otherwise the status, f not called.
    template <typename F>
    EpisodeStatus use(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        EpisodeBoundary boundary(*control_);
        if (!boundary.held()) return EpisodeStatus::Busy;
        const EpisodeStatus s = checked();
        if (s != EpisodeStatus::Ok) return s;
        session::World* world = episodeWorld(*control_->episode);
        const session::AgentIdentity* agent = episodeService(*control_->episode, serviceId_);
        if (!world || !agent) return EpisodeStatus::UnknownService;
        f(*world, *agent);
        return EpisodeStatus::Ok;
    }

private:
    EpisodeStatus checked() const noexcept; ///< under the boundary
    std::shared_ptr<EpisodeControl> control_;
    std::uint64_t generation_ = 0, serviceId_ = 0;
    EpisodeStatus opened_ = EpisodeStatus::Closed;
};

/// The referee's read of one generation's World: what M5's referee documents are built on.
class EpisodeReferee {
public:
    EpisodeReferee() = default;
    EpisodeReferee(std::shared_ptr<EpisodeControl> control, std::uint64_t generation, EpisodeStatus opened)
        : control_(std::move(control)), generation_(generation), opened_(opened) {}
    std::uint64_t generation() const noexcept { return generation_; }
    EpisodeStatus status() const noexcept;
    /// f(const World&) under the boundary, in its generation only.
    template <typename F>
    EpisodeStatus inspect(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        EpisodeBoundary boundary(*control_);
        if (!boundary.held()) return EpisodeStatus::Busy;
        const EpisodeStatus s = checked();
        if (s != EpisodeStatus::Ok) return s;
        const session::World* world = episodeWorld(*control_->episode);
        if (!world) return EpisodeStatus::NotReset;
        f(*world);
        return EpisodeStatus::Ok;
    }

private:
    EpisodeStatus checked() const noexcept;
    std::shared_ptr<EpisodeControl> control_;
    std::uint64_t generation_ = 0;
    EpisodeStatus opened_ = EpisodeStatus::Closed;
};

class MissionEpisode {
public:
    /// Validates the configuration and builds no World: generation 0. EpisodeError for: a name empty, dt not
    /// positive, frameSkip, decisionSteps or workers 0, publication or recording without diagnostic, a service id 0
    /// or given twice.
    explicit MissionEpisode(EpisodeConfig config);
    ~MissionEpisode();
    MissionEpisode(MissionEpisode&& other) noexcept;
    MissionEpisode& operator=(MissionEpisode&& other) noexcept;
    MissionEpisode(const MissionEpisode&) = delete;
    MissionEpisode& operator=(const MissionEpisode&) = delete;

    /// Destroy the owned World, if any - its children stale from here on - and build a fresh one for local episode
    /// `localEpisodeIndex`: world seed, recipe, then one Agent per service in ascending id and its binding, then the
    /// judge. The new generation (the previous plus 1). EpisodeError: Busy or Closed, nothing changed; a recipe that
    /// failed leaves the episode with no World (NotReset) until a later reset succeeds.
    std::uint64_t reset(std::uint64_t localEpisodeIndex);
    /// One decision tick: decisionSteps World steps, then the judge. NotReset, Busy or Closed otherwise.
    EpisodeStatus step();

    std::uint64_t generation() const noexcept;
    std::uint64_t localEpisodeIndex() const noexcept;
    std::uint64_t tick() const noexcept; ///< decision ticks since the last reset
    bool open() const noexcept;          ///< it has a World
    std::string worldName() const;       ///< the current World's name ("" before the first reset)
    std::string recordPath() const;      ///< its recording ("" when not recording)
    std::uint64_t worldSeed() const noexcept;
    std::uint64_t serviceSeed(std::uint64_t serviceId) const noexcept;
    std::uint64_t componentSeed(std::uint64_t serviceId, std::uint64_t componentId) const noexcept;

    /// The current generation's children. A service the configuration does not name answers UnknownService.
    EpisodeService service(std::uint64_t serviceId);
    EpisodeReferee referee();

    /// The range's own access to the current World, under the boundary: scenario setup and inspection.
    template <typename F>
    EpisodeStatus range(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        session::World* world = nullptr;
        EpisodeBoundary boundary(*control_);
        const EpisodeStatus s = enter(boundary, world);
        if (s != EpisodeStatus::Ok) return s;
        f(*world);
        return EpisodeStatus::Ok;
    }
    /// f(EpisodeJudge&) under the boundary; NoJudge when the configuration makes none.
    template <typename F>
    EpisodeStatus withJudge(F&& f) {
        if (!control_) return EpisodeStatus::Closed;
        session::World* world = nullptr;
        EpisodeBoundary boundary(*control_);
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

    std::shared_ptr<EpisodeControl> control_; ///< null once moved from
    std::unique_ptr<Episode> state_;
};

} // namespace fsim::env
