// Independent mission episodes (M4; docs/sdk/mission-training.md, sections 4 and 8): one World per episode, rebuilt
// whole on reset; children bound to a generation; seed streams that depend on nothing but their inputs. The native
// states a reset must clear come from the supplied fixture (docs/sdk/native-state-reset-fixture.md), applied here to
// the episode's own reset - not to a hand-built replacement World.
#include "env/MissionEpisode.h"
#include "fixtures/mission_training/NativeStateReset.h"

#include "foundation/core/Json.h"
#include "fsim/MAService.h"
#include "fsim/Rng.h"
#include "fsim/World.h"
#include "simulation/runtime/world/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#endif

using namespace fsim;
namespace fixture = fsim::test::native_reset;
using env::EpisodeStatus;
using env::MissionEpisode;

namespace {

void equivalent(const fixture::Snapshot& first, const fixture::Snapshot& second) {
    std::ostringstream differences;
    unsigned count = 0;
    for (const auto& item : first) {
        const auto other = second.find(item.first);
        if (other == second.end() || other->second != item.second) {
            if (count < 8) differences << item.first << ": " << item.second << " vs " << (other == second.end() ? "<absent>" : other->second) << '\n';
            ++count;
        }
    }
    for (const auto& item : second)
        if (first.find(item.first) == first.end()) {
            if (count < 8) differences << "extra: " << item.first << '\n';
            ++count;
        }
    INFO(differences.str());
    REQUIRE(count == 0);
}

/// The fixture's scenario as an episode recipe: the same timing as fixture::options, the recipe applied through the
/// borrowed facade only for the call (docs/sdk/native-state-reset-fixture.md), and the ids it made kept for the test.
struct FixtureEpisode {
    std::shared_ptr<fixture::Scenario> scenario = std::make_shared<fixture::Scenario>();
    env::EpisodeConfig config;
    FixtureEpisode(std::uint64_t envId, std::uint64_t baseSeed, const char* name = "fixture-episode") {
        config.name = name;
        config.envId = envId, config.baseSeed = baseSeed;
        config.dtS = 1.0 / 120.0, config.frameSkip = 4, config.decisionSteps = 1, config.workers = 1;
        auto out = scenario;
        config.native = [out](session::World& world) {
            const std::unique_ptr<World> facade(World::borrowed(world));
            *out = fixture::configure(*facade);
        };
    }
};

/// The episode's World through the borrowed facade, for one call, under the episode's boundary.
template <typename F>
void onWorld(MissionEpisode& episode, F&& f) {
    const EpisodeStatus status = episode.range([&](session::World& world) {
        const std::unique_ptr<World> facade(World::borrowed(world));
        f(*facade);
    });
    REQUIRE(status == EpisodeStatus::Ok);
}

fixture::Snapshot snapshot(MissionEpisode& episode) {
    fixture::Snapshot out;
    onWorld(episode, [&](World& world) { out = fixture::capture(world); });
    return out;
}

fixture::DirtyState contaminate(MissionEpisode& episode, const fixture::Scenario& scenario) {
    fixture::DirtyState dirty;
    onWorld(episode, [&](World& world) {
        dirty = fixture::contaminate(world, scenario);
        fixture::requireDirty(world, scenario, dirty);
    });
    return dirty;
}

/// An independent World built as the episode builds its own - the same options and effective seed - and the same
/// recipe: what "fresh" means, without any episode in it.
struct FreshWorld {
    session::World native;
    std::unique_ptr<World> facade;
    fixture::Scenario scenario;
    explicit FreshWorld(const session::WorldOptions& options) : native(options), facade(World::borrowed(native)) {
        scenario = fixture::configure(*facade);
    }
};

session::WorldOptions freshOptions(const MissionEpisode& episode, const char* name = "fixture-episode") {
    session::WorldOptions o;
    o.name = name;
    o.dt = 1.0 / 120.0, o.frameSkip = 4, o.workers = 1, o.pinWorkers = false, o.publish = false;
    o.seed = episode.worldSeed();
    return o;
}

std::uint64_t next(std::uint64_t x) { return Rng(x).next(); }

std::uint64_t parseU64(const core::Json& j) { return std::stoull(j.asString()); }

std::filesystem::path manifestPath() { return std::filesystem::path(FSIM_TEST_SOURCE_DIR) / "tests/fixtures/mission_training/contract/manifest.json"; }

double privateBytes() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof counters;
    if (GetProcessMemoryInfo(GetCurrentProcess(), reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters), sizeof counters))
        return static_cast<double>(counters.PrivateUsage);
#endif
    return 0.0;
}


/// One of this test executable's own cases, run in a process of its own and bounded: a case that may deadlock cannot
/// hang this process, and nothing of this process's stack is shared with it. On timeout the child is terminated and
/// waited for, so nothing is left running.
struct ChildRun {
    bool finished = false;   ///< it exited by itself within the bound
    unsigned long exitCode = 0;
    std::string output;      ///< what it printed (stdout and stderr)
};

ChildRun runChild(const std::string& testName, double seconds) {
    ChildRun run;
#ifdef _WIN32
    const std::filesystem::path dir = std::filesystem::path(FSIM_TEST_OUTPUT_DIR) / "children";
    std::filesystem::create_directories(dir);
    std::string file;
    for (const char c : testName) file += (std::isalnum(static_cast<unsigned char>(c)) ? c : '-');
    const std::filesystem::path log = dir / (file + ".log");
    SECURITY_ATTRIBUTES inherit{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
    const HANDLE out = CreateFileW(log.wstring().c_str(), GENERIC_WRITE, FILE_SHARE_READ, &inherit, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (out == INVALID_HANDLE_VALUE) throw std::runtime_error("runChild: cannot create " + log.string());
    wchar_t self[32768];
    const DWORD size = GetModuleFileNameW(nullptr, self, 32768);
    std::wstring command = L"\"" + std::wstring(self, size) + L"\" \"" + std::filesystem::path(testName).wstring() + L"\"";
    STARTUPINFOW startup{};
    startup.cb = sizeof startup;
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    startup.hStdOutput = startup.hStdError = out;
    PROCESS_INFORMATION process{};
    const BOOL started = CreateProcessW(nullptr, command.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process);
    CloseHandle(out);
    if (!started) throw std::runtime_error("runChild: cannot start the child");
    const DWORD bound = static_cast<DWORD>(seconds * 1000.0);
    run.finished = WaitForSingleObject(process.hProcess, bound) == WAIT_OBJECT_0;
    if (!run.finished) {
        TerminateProcess(process.hProcess, 124); // (a deadlocked child: ended, and waited for)
        WaitForSingleObject(process.hProcess, INFINITE);
    }
    DWORD code = 0;
    GetExitCodeProcess(process.hProcess, &code);
    run.exitCode = code;
    CloseHandle(process.hThread);
    CloseHandle(process.hProcess);
    std::ifstream in(log, std::ios::binary);
    run.output.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
#else
    (void)testName, (void)seconds;
    throw std::runtime_error("runChild: the bounded child runner needs Windows");
#endif
    return run;
}

/// A case run as a bounded child must pass there.
void requireChildPasses(const std::string& testName, double seconds = 120.0) {
    const ChildRun run = runChild(testName, seconds);
    INFO(testName << " printed:\n" << run.output);
    REQUIRE(run.finished);
    CHECK(run.exitCode == 0);
}

/// A thread's body with its own failure slot, read only after the thread is joined: two threads that fail at once
/// each keep their own failure.
template <typename F>
auto guarded(std::exception_ptr& slot, F body) {
    return [&slot, body]() mutable {
        try {
            body();
        } catch (...) {
            slot = std::current_exception();
        }
    };
}

/// After joining: the first failure, rethrown, with how many there were.
void rethrowFirst(const std::vector<std::exception_ptr>& slots) {
    for (const auto& slot : slots)
        if (slot) std::rethrow_exception(slot);
}

/// A judge whose destruction is observable: the episode's own state, torn down with its World.
struct Witness : env::EpisodeJudge {
    std::shared_ptr<std::atomic<bool>> destroyed;
    explicit Witness(std::shared_ptr<std::atomic<bool>> d) : destroyed(std::move(d)) {}
    ~Witness() override { destroyed->store(true); }
};

/// A C2-only episode with one service and a witness judge - quick to build, no flight model.
env::EpisodeConfig c2Episode(const char* name, std::shared_ptr<std::atomic<bool>> destroyed) {
    env::EpisodeConfig c;
    c.name = name;
    c.c2Nodes = {{"crc", 0.7, 0.0, 200.0}};
    c.services = {{1, "crc", {}, true, true}};
    c.judge = [destroyed](const session::World&) {
        destroyed->store(false);
        return std::make_unique<Witness>(destroyed);
    };
    return c;
}

/// Thread-safe (no assertions): the fixture scenario run on an episode, sampled after each listed step count.
std::vector<fixture::Snapshot> trace(MissionEpisode& episode, const fixture::Scenario& scenario, std::uint64_t local, const std::vector<unsigned>& steps) {
    std::vector<fixture::Snapshot> out;
    episode.reset(local);
    const auto sample = [&] {
        fixture::Snapshot shot;
        if (episode.range([&](session::World& world) {
                const std::unique_ptr<World> facade(World::borrowed(world));
                shot = fixture::capture(*facade);
            }) != EpisodeStatus::Ok)
            throw std::runtime_error("trace: the episode refused a capture");
        out.push_back(std::move(shot));
    };
    if (episode.range([&](session::World& world) {
            const std::unique_ptr<World> facade(World::borrowed(world));
            fixture::requireDirty(*facade, scenario, fixture::contaminate(*facade, scenario));
        }) != EpisodeStatus::Ok)
        throw std::runtime_error("trace: the episode refused contamination");
    sample();
    for (const unsigned n : steps) {
        for (unsigned k = 0; k < n; ++k)
            if (episode.step() != EpisodeStatus::Ok) throw std::runtime_error("trace: a step was refused");
        sample();
    }
    return out;
}

void sameTrace(const std::vector<fixture::Snapshot>& a, const std::vector<fixture::Snapshot>& b) {
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CAPTURE(i);
        equivalent(a[i], b[i]);
    }
}

} // namespace

TEST_CASE("mission episode: seed streams are section 8's, from their inputs alone", "[ma][episode][seed]") {
    // the contract's own formula, written out independently of the implementation
    const auto stream = [](std::uint64_t base, std::uint64_t envId, std::uint64_t local, std::uint64_t cls) {
        return next(next(next(next(base) ^ envId) ^ local) ^ cls);
    };
    CHECK(next(0) == 16294208416658607535ull);
    std::ifstream in(manifestPath(), std::ios::binary);
    REQUIRE(in);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const core::Json manifest = core::Json::parse(text, "manifest.json");
    const core::Json::Array& vectors = manifest.child("seed_streams").asArray();
    REQUIRE(vectors.size() >= 2);
    for (std::size_t i = 0; i < vectors.size(); ++i) {
        const core::Json& v = vectors[i];
        const auto u64 = [&](const char* key) { return parseU64(v.child(key)); };
        const std::uint64_t base = u64("base_seed"), envId = u64("env_id"), local = u64("local_episode_index");
        const std::uint64_t service = u64("service_id"), component = u64("component_id");
        CAPTURE(i);
        CHECK(env::episode_seeds::world(base, envId, local) == u64("world_seed"));
        CHECK(env::episode_seeds::service(base, envId, local, service) == u64("service_seed"));
        CHECK(env::episode_seeds::component(base, envId, local, service, component) == u64("component_seed"));
        CHECK(env::episode_seeds::world(base, envId, local) == stream(base, envId, local, 0));
        CHECK(env::episode_seeds::service(base, envId, local, service) == next(stream(base, envId, local, 1) ^ service));
    }
    // an episode's seeds are those of its inputs - not of its generation, nor of another episode
    FixtureEpisode a(7, 99), b(8, 99);
    MissionEpisode first(a.config), other(b.config);
    first.reset(3);
    other.reset(0);
    other.reset(1);
    const std::uint64_t seed = first.worldSeed();
    CHECK(seed == env::episode_seeds::world(99, 7, 3));
    CHECK(first.serviceSeed(41) == env::episode_seeds::service(99, 7, 3, 41));
    CHECK(first.componentSeed(41, 5) == env::episode_seeds::component(99, 7, 3, 41, 5));
    first.reset(4);
    first.reset(3); // (generation 3 now, the same local episode as generation 1)
    CHECK(first.generation() == 3);
    CHECK(first.worldSeed() == seed);
    CHECK(other.worldSeed() == env::episode_seeds::world(99, 8, 1));
}

TEST_CASE("mission episode: the fixture's required states hold for the effective seeds the episodes use", "[ma][episode][fixture]") {
    // Before relying on the fixture under the episodes' derived seeds, show its native outcomes occur there.
    for (const auto& [envId, base, local] : {std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{7, 99, 0}, {7, 99, 1}, {8, 99, 0}, {11, 5, 2}}) {
        CAPTURE(envId, base, local);
        FixtureEpisode e(envId, base);
        MissionEpisode episode(e.config);
        episode.reset(local);
        const auto dirty = contaminate(episode, *e.scenario);
        CHECK(dirty.crewLost);
        CHECK(dirty.activeFire);
        CHECK(dirty.liveWeapon != 0);
        CHECK(dirty.liveDecoy != 0);
        CHECK(dirty.pendingTask != 0);
        CHECK(dirty.pendingConsent != 0);
        CHECK(dirty.queuedMessage != 0);
        CHECK(dirty.removedEntity == e.scenario->removed);
    }
}

TEST_CASE("mission episode: construction validates and builds no World; nothing answers before the first reset", "[ma][episode][lifecycle]") {
    FixtureEpisode e(1, 2);
    MissionEpisode episode(e.config);
    CHECK(episode.generation() == 0);
    CHECK_FALSE(episode.open());
    CHECK(episode.step() == EpisodeStatus::NotReset);
    bool called = false;
    CHECK(episode.range([&](session::World&) { called = true; }) == EpisodeStatus::NotReset);
    CHECK(episode.referee().inspect([&](const session::World&) { called = true; }) == EpisodeStatus::NotReset);
    CHECK(episode.service(1).use([&](session::World&, const session::AgentIdentity&) { called = true; }) == EpisodeStatus::UnknownService);
    env::EpisodeConfig withService;
    withService.name = "episode-before-reset";
    withService.c2Nodes = {{"crc", 0.7, 0.0, 200.0}};
    withService.services = {{1, "crc", {}, true, true}};
    MissionEpisode unset(withService);
    CHECK(unset.service(1).status() == EpisodeStatus::NotReset);
    CHECK(unset.service(1).use([&](session::World&, const session::AgentIdentity&) { called = true; }) == EpisodeStatus::NotReset);
    CHECK_FALSE(called);
    // what the contract refuses at construction
    const auto refused = [](auto change) {
        FixtureEpisode bad(1, 2);
        change(bad.config);
        CHECK_THROWS_AS(MissionEpisode(bad.config), env::EpisodeError);
    };
    refused([](env::EpisodeConfig& c) { c.decisionSteps = 0; });
    refused([](env::EpisodeConfig& c) { c.frameSkip = 0; });
    refused([](env::EpisodeConfig& c) { c.dtS = 0.0; });
    refused([](env::EpisodeConfig& c) { c.workers = 0; });
    refused([](env::EpisodeConfig& c) { c.name.clear(); });
    refused([](env::EpisodeConfig& c) { c.publish = true; });            // (publication is diagnostic only)
    refused([](env::EpisodeConfig& c) { c.recordDir = "D:/ENV/tmp"; });  // (and so is recording)
    refused([](env::EpisodeConfig& c) { c.services = {{0, "fixture-launcher", {"*"}, true, true}}; });
    refused([](env::EpisodeConfig& c) { c.services = {{4, "a", {}, false, false}, {4, "b", {}, false, false}}; });
}

TEST_CASE("mission episode: a reset is a fresh World - after every fixture state, equal to a fresh episode and a fresh World", "[ma][episode][reset]") {
    FixtureEpisode e(7, 99);
    MissionEpisode episode(e.config);
    REQUIRE(episode.reset(0) == 1);
    const auto clean = snapshot(episode);
    const auto dirty = contaminate(episode, *e.scenario);
    CHECK(dirty.crewLost);
    CHECK(dirty.activeFire);
    equivalent(snapshot(episode), snapshot(episode)); // (reading consumes nothing)
    CHECK(snapshot(episode) != clean);

    // the same local episode again: generation 2, and the owned World built from nothing
    REQUIRE(episode.reset(0) == 2);
    CHECK(episode.tick() == 0);
    const auto reconstructed = *e.scenario;
    // equal to a brand-new episode of the same inputs at its first reset ...
    FixtureEpisode f(7, 99);
    MissionEpisode freshEpisode(f.config);
    REQUIRE(freshEpisode.reset(0) == 1);
    // ... and to an independent World with the same options, seed and recipe. Nothing is normalized: the episode
    // generation is not part of the World, so it does not appear in the captured native state at all.
    FreshWorld fresh(freshOptions(episode));
    CHECK(reconstructed.removed == fresh.scenario.removed);
    const auto afterReset = snapshot(episode);
    equivalent(afterReset, snapshot(freshEpisode));
    equivalent(afterReset, fixture::capture(*fresh.facade));
    equivalent(afterReset, clean);
    // nothing of the dirty generation is left: no weapon, decoy, damage, removed entity or retained event
    onWorld(episode, [&](World& world) {
        CHECK(world.weapons().empty());
        CHECK(world.decoys().empty());
        CHECK(world.impl().damage(reconstructed.victim) == nullptr);
        CHECK(world.vehicle(reconstructed.removed).valid());
        CHECK(world.impl().engagementEvents().empty());
        CHECK(world.impl().takeEngagementEvents().empty());
        CHECK(fsim::maServiceBindings(world).empty());
        CHECK(world.impl().agents().empty());
    });
    // idle replay: equal after one, four and thirty more World steps
    for (const unsigned steps : {1u, 4u, 30u}) {
        for (unsigned k = 0; k < steps; ++k) {
            REQUIRE(episode.step() == EpisodeStatus::Ok);
            REQUIRE(freshEpisode.step() == EpisodeStatus::Ok);
        }
        fresh.facade->step(steps);
        equivalent(snapshot(episode), snapshot(freshEpisode));
        equivalent(snapshot(episode), fixture::capture(*fresh.facade));
    }
    // active replay: contamination again gives the same native identities and values - no next-id, RNG, queue or
    // guidance state carried over from the dirty generation
    const auto replay = contaminate(episode, reconstructed);
    const auto freshReplay = contaminate(freshEpisode, *f.scenario);
    fixture::DirtyState independent;
    independent = fixture::contaminate(*fresh.facade, fresh.scenario);
    for (const fixture::DirtyState* other : {&freshReplay, static_cast<const fixture::DirtyState*>(&independent)}) {
        CHECK(replay.liveWeapon == other->liveWeapon);
        CHECK(replay.liveDecoy == other->liveDecoy);
        CHECK(replay.releasedActivity == other->releasedActivity);
        CHECK(replay.waitingActivity == other->waitingActivity);
        CHECK(replay.pendingTask == other->pendingTask);
        CHECK(replay.pendingConsent == other->pendingConsent);
        CHECK(replay.queuedMessage == other->queuedMessage);
        CHECK(replay.consumedStore == other->consumedStore);
    }
    // and the dirty generation's identities were the same native numbers: a fresh World restarts them
    CHECK(replay.liveWeapon == dirty.liveWeapon);
    CHECK(replay.pendingConsent == dirty.pendingConsent);
    equivalent(snapshot(episode), snapshot(freshEpisode));
    equivalent(snapshot(episode), fixture::capture(*fresh.facade));
    for (unsigned k = 0; k < 30; ++k) {
        REQUIRE(episode.step() == EpisodeStatus::Ok);
        REQUIRE(freshEpisode.step() == EpisodeStatus::Ok);
    }
    fresh.facade->step(30);
    onWorld(episode, [&](World& world) { fixture::requireDeliveredMarker(world, reconstructed, replay); });
    onWorld(freshEpisode, [&](World& world) { fixture::requireDeliveredMarker(world, *f.scenario, freshReplay); });
    fixture::requireDeliveredMarker(*fresh.facade, fresh.scenario, independent);
    equivalent(snapshot(episode), snapshot(freshEpisode));
    equivalent(snapshot(episode), fixture::capture(*fresh.facade));

    // another local episode: another effective seed, and again exactly a fresh one of those inputs
    REQUIRE(episode.reset(1) == 3);
    FixtureEpisode g(7, 99);
    MissionEpisode freshOne(g.config);
    freshOne.reset(1);
    CHECK(episode.worldSeed() == freshOne.worldSeed());
    CHECK(episode.worldSeed() != env::episode_seeds::world(99, 7, 0));
    contaminate(episode, *e.scenario);
    contaminate(freshOne, *g.scenario);
    equivalent(snapshot(episode), snapshot(freshOne));
}

TEST_CASE("mission episode: the legacy vehicle reset keeps its semantics; only the episode reset rebuilds", "[ma][episode][reset][legacy]") {
    FixtureEpisode e(7, 99);
    MissionEpisode episode(e.config);
    episode.reset(0);
    const auto dirty = contaminate(episode, *e.scenario);
    // inside the episode, Vehicle.reset is what it always was: the vehicle restarts, the World and its state stay
    onWorld(episode, [&](World& world) {
        const double before = world.time();
        REQUIRE(world.vehicle(e.scenario->launcher).reset());
        CHECK(world.time() == before);
        REQUIRE(world.weapons().size() == 1);
        CHECK(world.weapons().front().id == dirty.liveWeapon);
        CHECK(world.decoys().size() == 1);
        CHECK(fsim::maServiceBindings(world).size() == 2);
        const auto* tasks = world.impl().maCore().tasks(e.scenario->launcher);
        REQUIRE(tasks);
        CHECK(tasks->size() == 1);
        CHECK_FALSE(world.vehicle(e.scenario->removed).valid());
    });
    CHECK(episode.generation() == 1); // (a vehicle reset is not an episode reset)
    episode.reset(0);
    onWorld(episode, [&](World& world) {
        CHECK(world.weapons().empty());
        CHECK(world.decoys().empty());
        CHECK(fsim::maServiceBindings(world).empty());
        CHECK(world.vehicle(e.scenario->removed).valid());
    });
}

TEST_CASE("mission episode: children answer only in their generation; a reset makes them stale", "[ma][episode][handles]") {
    env::EpisodeConfig config;
    config.name = "episode-handles";
    config.envId = 3, config.baseSeed = 4;
    config.c2Nodes = {{"crc", 39.9 * 0.017453292519943295, 0.0, 200.0}};
    config.services = {{21, "crc", {}, true, true}};
    MissionEpisode episode(config);
    episode.reset(0);
    auto service = episode.service(21);
    auto referee = episode.referee();
    CHECK(service.generation() == 1);
    CHECK(service.status() == EpisodeStatus::Ok);
    CHECK(referee.status() == EpisodeStatus::Ok);
    session::AgentIdentity seen;
    bool onNode = false;
    const EpisodeStatus used = service.use([&](session::World& world, const session::AgentIdentity& agent) {
        seen = agent;
        onNode = world.c2Node(agent.vehicle) != nullptr;
    });
    CHECK(used == EpisodeStatus::Ok);
    CHECK(onNode);
    CHECK(seen.controller != 0);
    CHECK(episode.service(99).status() == EpisodeStatus::UnknownService);
    bool called = false;
    CHECK(episode.service(99).use([&](session::World&, const session::AgentIdentity&) { called = true; }) == EpisodeStatus::UnknownService);

    episode.reset(0);
    CHECK(service.status() == EpisodeStatus::Stale);
    CHECK(referee.status() == EpisodeStatus::Stale);
    CHECK(service.use([&](session::World&, const session::AgentIdentity&) { called = true; }) == EpisodeStatus::Stale);
    CHECK(referee.inspect([&](const session::World&) { called = true; }) == EpisodeStatus::Stale);
    CHECK_FALSE(called); // (a stale child touches nothing)
    // the range opens the new generation's children again
    auto renewed = episode.service(21);
    CHECK(renewed.generation() == 2);
    CHECK(renewed.status() == EpisodeStatus::Ok);
    control::ControllerId again = 0;
    CHECK(renewed.use([&](session::World&, const session::AgentIdentity& agent) { again = agent.controller; }) == EpisodeStatus::Ok);
    CHECK(again == seen.controller); // (a fresh World mints the same first controller: the number is no proof of generation)
    CHECK(service.status() == EpisodeStatus::Stale);
}

TEST_CASE("mission episode: one boundary - a call while another holds it fails busy and never waits", "[ma][episode][handles]") {
    env::EpisodeConfig config;
    config.name = "episode-busy";
    config.c2Nodes = {{"crc", 0.7, 0.0, 200.0}};
    config.services = {{1, "crc", {}, true, true}};
    MissionEpisode episode(config);
    episode.reset(0);
    auto service = episode.service(1);
    auto referee = episode.referee();
    std::vector<EpisodeStatus> inner;
    CHECK(service.use([&](session::World&, const session::AgentIdentity&) {
        inner.push_back(episode.step());
        inner.push_back(referee.inspect([](const session::World&) {}));
        inner.push_back(episode.range([](session::World&) {}));
        try {
            episode.reset(1);
            inner.push_back(EpisodeStatus::Ok);
        } catch (const env::EpisodeError& error) {
            inner.push_back(error.status());
        }
    }) == EpisodeStatus::Ok);
    CHECK(inner == std::vector<EpisodeStatus>(4, EpisodeStatus::Busy));
    CHECK(episode.generation() == 1);
    CHECK(episode.tick() == 0);
    CHECK(episode.step() == EpisodeStatus::Ok); // (released afterwards)
    CHECK(episode.tick() == 1);
}

TEST_CASE("mission episode: a hold is one operation - steps, judge reads and resets no other call enters", "[ma][episode][handles]") {
    // M7 core's batch closes a slot's tick through an EpisodeHold: whatever it does under it, a child's call and the
    // range's own fail busy meanwhile, and the episode is free again once the hold goes.
    struct Judge : env::EpisodeJudge {
        std::uint64_t after = 0;
        void afterStep(const session::World&, std::uint64_t tick) override { after = tick; }
    };
    env::EpisodeConfig config;
    config.name = "episode-hold";
    config.c2Nodes = {{"crc", 0.7, 0.0, 200.0}};
    config.services = {{1, "crc", {}, true, true}};
    config.judge = [](const session::World&) { return std::make_unique<Judge>(); };
    MissionEpisode episode(config);
    episode.reset(0);
    auto service = episode.service(1);
    auto referee = episode.referee();
    {
        env::EpisodeHold hold(episode);
        REQUIRE(hold.held());
        CHECK(hold.step() == EpisodeStatus::Ok);
        auto* judge = dynamic_cast<Judge*>(hold.judge());
        REQUIRE(judge);
        CHECK(judge->after == 1);
        // every other call meanwhile - a child's, the range's - fails busy, whatever the hold does between them
        CHECK(service.use([](session::World&, const session::AgentIdentity&) {}) == EpisodeStatus::Busy);
        CHECK(referee.inspect([](const session::World&) {}) == EpisodeStatus::Busy);
        CHECK(episode.step() == EpisodeStatus::Busy);
        CHECK(episode.range([](session::World&) {}) == EpisodeStatus::Busy);
        CHECK_FALSE(env::EpisodeHold(episode).held());
        CHECK(hold.step() == EpisodeStatus::Ok);
        CHECK(episode.tick() == 2);
        std::uint64_t prepared = 0;
        env::ResetHooks hooks;
        hooks.prepare = [&](std::uint64_t generation) { prepared = generation; };
        CHECK(hold.reset(5, hooks) == 2); // (a reset under the same hold: the old generation's children stale at once)
        CHECK(prepared == 2);
        CHECK(episode.generation() == 2);
        CHECK(episode.localEpisodeIndex() == 5);
        CHECK(service.use([](session::World&, const session::AgentIdentity&) {}) == EpisodeStatus::Busy); // (still held)
        CHECK(dynamic_cast<Judge*>(hold.judge())->after == 0); // (the new generation's own judge)
        CHECK(hold.step() == EpisodeStatus::Ok);
    }
    CHECK(episode.tick() == 1);
    CHECK(service.use([](session::World&, const session::AgentIdentity&) {}) == EpisodeStatus::Stale);
    CHECK(episode.step() == EpisodeStatus::Ok); // (free again)
    INFO("a hold that is not held does nothing: Busy");
    auto fresh = episode.service(1);
    bool innerHeld = true, resetBusy = false;
    EpisodeStatus innerStep = EpisodeStatus::Ok;
    env::EpisodeJudge* innerJudge = nullptr;
    const EpisodeStatus outer = fresh.use([&](session::World&, const session::AgentIdentity&) {
        env::EpisodeHold inner(episode);
        innerHeld = inner.held();
        innerStep = inner.step();
        innerJudge = inner.judge();
        try {
            inner.reset(9, {});
        } catch (const env::EpisodeError& error) {
            resetBusy = error.status() == EpisodeStatus::Busy;
        }
    });
    CHECK(outer == EpisodeStatus::Ok);
    CHECK_FALSE(innerHeld);
    CHECK(innerStep == EpisodeStatus::Busy);
    CHECK(innerJudge == nullptr);
    CHECK(resetBusy);
    CHECK(episode.generation() == 2);
    INFO("a moved-from episode: Closed");
    MissionEpisode moved(std::move(episode));
    env::EpisodeHold gone(episode);
    CHECK_FALSE(gone.held());
    CHECK(gone.step() == EpisodeStatus::Closed);
    CHECK(env::EpisodeHold(moved).held());
}

TEST_CASE("mission episode: destroying the parent closes its children first; moving it keeps them", "[ma][episode][handles]") {
    env::EpisodeConfig config;
    config.name = "episode-lifetime";
    config.c2Nodes = {{"crc", 0.7, 0.0, 200.0}};
    config.services = {{5, "crc", {}, true, true}};
    std::optional<env::EpisodeService> orphan;
    std::optional<env::EpisodeReferee> orphanReferee;
    {
        MissionEpisode episode(config);
        episode.reset(0);
        orphan = episode.service(5);
        orphanReferee = episode.referee();
        CHECK(orphan->status() == EpisodeStatus::Ok);
    }
    bool called = false;
    CHECK(orphan->status() == EpisodeStatus::Closed);
    CHECK(orphan->use([&](session::World&, const session::AgentIdentity&) { called = true; }) == EpisodeStatus::Closed);
    CHECK(orphanReferee->inspect([&](const session::World&) { called = true; }) == EpisodeStatus::Closed);
    CHECK_FALSE(called);

    // a move keeps the episode's identity: its children go on working, the moved-from object is closed
    MissionEpisode original(config);
    original.reset(0);
    auto child = original.service(5);
    MissionEpisode moved(std::move(original));
    CHECK(child.status() == EpisodeStatus::Ok);
    CHECK(child.use([&](session::World& world, const session::AgentIdentity& agent) { called = world.c2Node(agent.vehicle) != nullptr; }) ==
          EpisodeStatus::Ok);
    CHECK(called);
    CHECK(moved.generation() == 1);
    CHECK(moved.step() == EpisodeStatus::Ok);
    CHECK(original.step() == EpisodeStatus::Closed);       // NOLINT(bugprone-use-after-move): a moved-from episode is closed
    CHECK_FALSE(original.open());                          // NOLINT(bugprone-use-after-move)
    CHECK(original.service(5).status() == EpisodeStatus::Closed);
    CHECK_THROWS_AS(original.reset(0), env::EpisodeError);
    // move assignment: the target's own episode ends - its children closed - and it takes the source's
    MissionEpisode target(config);
    target.reset(0);
    auto targetChild = target.service(5);
    target = std::move(moved);
    CHECK(targetChild.status() == EpisodeStatus::Closed);
    CHECK(child.status() == EpisodeStatus::Ok);
    CHECK(target.step() == EpisodeStatus::Ok);
    CHECK(target.tick() == 2);
    target.reset(0);
    CHECK(child.status() == EpisodeStatus::Stale);
}

TEST_CASE("mission episode: episodes share no mutable state - contamination, steps and resets of one leave another as it was", "[ma][episode][independence]") {
    FixtureEpisode a(7, 99), b(8, 99);
    MissionEpisode first(a.config), second(b.config);
    first.reset(0);
    second.reset(0);
    const auto untouched = snapshot(second);
    const auto dirty = contaminate(first, *a.scenario);
    equivalent(snapshot(second), untouched);
    for (int k = 0; k < 10; ++k) REQUIRE(first.step() == EpisodeStatus::Ok);
    first.reset(5);
    contaminate(first, *a.scenario);
    equivalent(snapshot(second), untouched);
    CHECK(dirty.crewLost);

    // B's sequence, sampled through its native outcomes, is the same alone and interleaved with A's resets and steps
    const auto run = [&](bool interleave) {
        FixtureEpisode bb(8, 99), aa(7, 99);
        MissionEpisode b2(bb.config), a2(aa.config);
        std::vector<fixture::Snapshot> samples;
        b2.reset(2);
        if (interleave) {
            a2.reset(0);
            contaminate(a2, *aa.scenario);
        }
        contaminate(b2, *bb.scenario);
        samples.push_back(snapshot(b2));
        for (int k = 0; k < 6; ++k) {
            if (interleave) {
                a2.reset(static_cast<std::uint64_t>(k));
                for (int j = 0; j < k; ++j) a2.step();
            }
            REQUIRE(b2.step() == EpisodeStatus::Ok);
            samples.push_back(snapshot(b2));
        }
        b2.reset(3);
        if (interleave) {
            a2.reset(9);
        }
        contaminate(b2, *bb.scenario);
        samples.push_back(snapshot(b2));
        return samples;
    };
    const auto alone = run(false), interleaved = run(true);
    REQUIRE(alone.size() == interleaved.size());
    for (std::size_t i = 0; i < alone.size(); ++i) {
        CAPTURE(i);
        equivalent(alone[i], interleaved[i]);
    }
}

TEST_CASE("mission episode: the recipe is applied in the contract's order and services bind in ascending id", "[ma][episode][recipe]") {
    const auto build = [](bool reversed) {
        env::EpisodeConfig config;
        config.name = "episode-recipe";
        config.envId = 1, config.baseSeed = 1;
        Scenario scenario;
        scenario.source = "episode-recipe";
        for (const auto& [name, lat] : {std::pair<const char*, double>{"lead", 40.0}, {"wing", 40.02}}) {
            ScenarioVehicle v;
            v.spec.name = name;
            v.spec.type = "jsbsim:f16c";
            v.spec.initial.latitudeDeg = lat;
            v.spec.initial.altitudeMslM = 6000.0;
            v.spec.initial.airspeedTrueMs = 230.0;
            scenario.vehicles.push_back(v);
        }
        config.scenario = scenario;
        config.c2Nodes = {{"crc", 39.9 * 0.017453292519943295, 0.0, 200.0}};
        config.systems = {{"wing", 0x5301, "WING-1"}};
        comm::NetSpec net;
        net.name = "l16";
        config.nets = {{net, {"crc", "lead", "wing"}}};
        comm::DisseminationPlan plan;
        plan.positionPeriodS = 1.0;
        config.dissemination = {{"lead", plan}};
        comm::RbacConfig rbac = comm::RbacConfig::agraDefaults();
        rbac.strikePolicy = comm::StrikePolicy::AutoApprove;
        config.rbac = {{"lead", rbac}};
        config.services = {{30, "lead", {"fsim.guidance.hsa"}, false, true}, {10, "lead", {}, true, false}, {20, "crc", {}, true, true}};
        if (reversed) std::reverse(config.services.begin(), config.services.end());
        return config;
    };
    const auto read = [](MissionEpisode& episode) {
        std::vector<std::string> out;
        CHECK(episode.range([&](session::World& w) {
            const std::uint32_t lead = w.find("lead"), wing = w.find("wing");
            out.push_back("ids " + std::to_string(lead) + " " + std::to_string(wing));
            for (const std::uint32_t c2 : w.c2Nodes()) out.push_back("c2 " + std::to_string(c2) + " " + w.c2Node(c2)->name);
            out.push_back("systems " + std::to_string(w.dms().systemOf(w.c2Nodes().front())) + " " + std::to_string(w.dms().systemOf(lead)) +
                          " " + std::to_string(w.dms().systemOf(wing)));
            for (const auto& n : w.dms().nets()) out.push_back("net " + std::to_string(n.id) + " " + n.name);
            out.push_back("plan " + std::to_string(w.dms().plan(lead).positionPeriodS));
            out.push_back("policy " + std::to_string(static_cast<int>(w.maCore().rbac(lead).strikePolicy)));
            for (const auto& b : w.maServices().all())
                out.push_back("service " + std::to_string(b.serviceId) + " " + std::to_string(b.platform) + " " + std::to_string(b.controller) +
                              " " + std::to_string(b.ownsDmsIngress) + std::to_string(b.ownsDmsEgress));
            for (const auto& a : w.agents()) out.push_back("agent " + a.name + " " + std::to_string(a.controller));
            out.push_back("hsa " + std::to_string(w.maServices().owner(lead, "fsim.guidance.hsa")));
        }) == EpisodeStatus::Ok);
        return out;
    };
    MissionEpisode forward(build(false)), backward(build(true));
    forward.reset(0);
    backward.reset(0);
    const auto a = read(forward), b = read(backward);
    CHECK(a == b); // (the services' array order decides nothing)
    for (const auto& line : a) UNSCOPED_INFO(line);
    REQUIRE(a.size() >= 8);
    CHECK(a[0] == "ids 1 2");
    CHECK(a[2] == "systems 1392508929 1392508930 21249"); // (c2 and lead: the next free ids as they joined; wing: assigned)
    CHECK(a[3] == "net 1 l16");
    // the agents in ascending service id: 10, 20, 30 - each named by its service
    CHECK(a[a.size() - 4].rfind("agent episode-service-10 ", 0) == 0);
    CHECK(a[a.size() - 3].rfind("agent episode-service-20 ", 0) == 0);
    CHECK(a[a.size() - 2].rfind("agent episode-service-30 ", 0) == 0);
    // a platform the recipe does not make cannot be bound: the reset fails and the episode has no World
    env::EpisodeConfig bad = build(false);
    bad.services.push_back({40, "nowhere", {}, false, false});
    MissionEpisode broken(bad);
    CHECK_THROWS_AS(broken.reset(0), env::EpisodeError);
    CHECK_FALSE(broken.open());
    CHECK(broken.step() == EpisodeStatus::NotReset);
}

TEST_CASE("mission episode: the decision tick and the task state belong to the generation", "[ma][episode][cursors]") {
    struct Counting : env::EpisodeJudge {
        std::shared_ptr<std::vector<std::uint64_t>> seen;
        explicit Counting(std::shared_ptr<std::vector<std::uint64_t>> s) : seen(std::move(s)) {}
        std::uint64_t ticks = 0;
        void afterStep(const session::World&, std::uint64_t tick) override { ++ticks, seen->push_back(tick); }
    };
    auto seen = std::make_shared<std::vector<std::uint64_t>>();
    unsigned made = 0;
    FixtureEpisode e(7, 99);
    e.config.judge = [&](const session::World&) {
        ++made;
        return std::make_unique<Counting>(seen);
    };
    e.config.decisionSteps = 3;
    MissionEpisode episode(e.config);
    episode.reset(0);
    CHECK(made == 1);
    double t0 = 0.0;
    onWorld(episode, [&](World& world) { t0 = world.time(); });
    REQUIRE(episode.step() == EpisodeStatus::Ok);
    REQUIRE(episode.step() == EpisodeStatus::Ok);
    CHECK(episode.tick() == 2);
    onWorld(episode, [&](World& world) { CHECK(world.impl().worldSteps() == 6); }); // (three World steps a tick)
    std::uint64_t ticks = 0;
    CHECK(episode.withJudge([&](env::EpisodeJudge& judge) { ticks = dynamic_cast<Counting&>(judge).ticks; }) == EpisodeStatus::Ok);
    CHECK(ticks == 2);
    // the generation's native events are read by the referee without consuming them
    contaminate(episode, *e.scenario);
    REQUIRE(episode.step() == EpisodeStatus::Ok);
    std::size_t before = 0;
    CHECK(episode.referee().inspect([&](const session::World& w) { before = w.engagementEvents().size(); }) == EpisodeStatus::Ok);
    CHECK(before > 0);
    episode.reset(0);
    CHECK(made == 2); // (a new judge: nothing of the last one's state)
    CHECK(episode.tick() == 0);
    CHECK(episode.withJudge([&](env::EpisodeJudge& judge) { ticks = dynamic_cast<Counting&>(judge).ticks; }) == EpisodeStatus::Ok);
    CHECK(ticks == 0);
    std::size_t after = 1;
    CHECK(episode.referee().inspect([&](const session::World& w) { after = w.engagementEvents().size(); }) == EpisodeStatus::Ok);
    CHECK(after == 0);
    CHECK(*seen == std::vector<std::uint64_t>{1, 2, 3});
    (void)t0;
}

TEST_CASE("mission episode: diagnostic publication and recording are named by env and generation", "[ma][episode][diagnostic]") {
    const std::filesystem::path dir = std::filesystem::path(FSIM_TEST_OUTPUT_DIR) / "mission-episode";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const auto config = [&](std::uint64_t envId) {
        env::EpisodeConfig c;
        c.name = "m4-diag";
        c.envId = envId;
        c.diagnostic = true, c.publish = true;
        c.recordDir = dir.string();
        Scenario scenario;
        scenario.source = "m4-diag";
        ScenarioVehicle v;
        v.spec.name = "viper";
        v.spec.type = "jsbsim:f16c";
        v.spec.initial.altitudeMslM = 3000.0;
        v.spec.initial.airspeedTrueMs = 200.0;
        scenario.vehicles.push_back(v);
        c.scenario = scenario;
        return c;
    };
    MissionEpisode one(config(1)), two(config(2));
    one.reset(0);
    two.reset(0);
    CHECK(one.worldName() == "m4-diag-e1-g1");
    CHECK(two.worldName() == "m4-diag-e2-g1");
    CHECK(one.recordPath() == (dir / "m4-diag-e1-g1.fsrec").string());
    bool published = false, recording = false;
    for (MissionEpisode* e : {&one, &two}) {
        CHECK(e->referee().inspect([&](const session::World& w) { published = w.published(), recording = w.recording(); }) == EpisodeStatus::Ok);
        CHECK(published);
        CHECK(recording);
        REQUIRE(e->step() == EpisodeStatus::Ok);
    }
    one.reset(0);
    CHECK(one.worldName() == "m4-diag-e1-g2");
    CHECK(one.recordPath() == (dir / "m4-diag-e1-g2.fsrec").string());
    CHECK(one.referee().inspect([&](const session::World& w) { published = w.published(); }) == EpisodeStatus::Ok);
    CHECK(published);
    // the first generation's World is gone: its recording is closed and complete, its name free again
    CHECK(std::filesystem::exists(dir / "m4-diag-e1-g1.fsrec"));
    CHECK(std::filesystem::file_size(dir / "m4-diag-e1-g1.fsrec") > 0);
    {
        session::WorldOptions o;
        o.name = "m4-diag-e1-g1";
        o.workers = 1, o.pinWorkers = false;
        session::World reuse(o);
        CHECK(reuse.published());
    }
    // not diagnostic: neither published nor recorded, whatever the name
    env::EpisodeConfig quiet = config(3);
    quiet.diagnostic = false, quiet.publish = false, quiet.recordDir.clear();
    MissionEpisode silent(quiet);
    silent.reset(0);
    CHECK(silent.worldName() == "m4-diag");
    CHECK(silent.recordPath().empty());
    CHECK(silent.referee().inspect([&](const session::World& w) { published = w.published(), recording = w.recording(); }) == EpisodeStatus::Ok);
    CHECK_FALSE(published);
    CHECK_FALSE(recording);
}

TEST_CASE("mission episode: initialization time and memory, measured", "[ma][episode][measure]") {
    // Measured, not optimized (M4): one World per episode, rebuilt on every reset.
    using clock = std::chrono::steady_clock;
    const double before = privateBytes();
    const auto start = clock::now();
    FixtureEpisode e(7, 99);
    MissionEpisode episode(e.config);
    const auto constructed = clock::now();
    episode.reset(0);
    const auto firstReset = clock::now();
    const double withWorld = privateBytes();
    contaminate(episode, *e.scenario);
    const auto contaminated = clock::now();
    double simulated = 0.0;
    onWorld(episode, [&](World& world) { simulated = world.time(); });
    constexpr int kResets = 5;
    for (int k = 0; k < kResets; ++k) episode.reset(static_cast<std::uint64_t>(k));
    const auto resets = clock::now();
    const double afterResets = privateBytes();
    const auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b - a).count(); };
    std::printf("[measure] construct %.3f ms; first reset %.1f ms; fixture contamination %.1f ms (%.2f simulated s); reset mean %.1f ms "
                "over %d; private bytes +%.1f MiB with World, +%.1f MiB after %d resets\n",
                ms(start, constructed), ms(constructed, firstReset), ms(firstReset, contaminated), simulated, ms(contaminated, resets) / kResets,
                kResets,
                (withWorld - before) / 1048576.0, (afterResets - before) / 1048576.0, kResets);
    CHECK(episode.generation() == kResets + 1);
}

// --- the review of 8254d7e: disposal from a callback, opening during a reset, terrain, worker and thread schedules ----

namespace {

/// One disposal path, in the child process that runs it: the episode, its children and the callback all live there.
template <typename Body>
void disposalPath(const char* name, Body body) {
    auto destroyed = std::make_shared<std::atomic<bool>>(false);
    auto owner = std::make_unique<MissionEpisode>(c2Episode(name, destroyed));
    owner->reset(0);
    auto service = owner->service(1);
    auto referee = owner->referee();
    bool aliveAfterDisposal = false, worldUsable = false;
    EpisodeStatus result = EpisodeStatus::Closed;
    body(owner, service, referee, destroyed, aliveAfterDisposal, worldUsable, result);
    CHECK(result == EpisodeStatus::Ok);
    CHECK(aliveAfterDisposal); // (the episode's state outlived the parent until the callback returned)
    CHECK(worldUsable);        // (and so did the World the callback holds)
    CHECK(destroyed->load());  // (then it went)
    CHECK(service.status() == EpisodeStatus::Closed);
    CHECK(referee.status() == EpisodeStatus::Closed);
}

} // namespace

TEST_CASE("episode child dispose from referee", "[.][episode-child]") {
    disposalPath("dispose-from-referee", [](auto& owner, auto&, auto& referee, auto& destroyed, bool& alive, bool& usable, EpisodeStatus& result) {
        result = referee.inspect([&](const session::World& world) {
            owner.reset();
            alive = !destroyed->load();
            usable = world.c2Nodes().size() == 1 && world.simTime() == 0.0;
        });
    });
}

TEST_CASE("episode child dispose from service", "[.][episode-child]") {
    disposalPath("dispose-from-service", [](auto& owner, auto& service, auto&, auto& destroyed, bool& alive, bool& usable, EpisodeStatus& result) {
        result = service.use([&](session::World& world, const session::AgentIdentity& agent) {
            owner.reset();
            alive = !destroyed->load();
            world.step();
            usable = world.c2Node(agent.vehicle) != nullptr && world.worldSteps() == 1;
        });
    });
}

TEST_CASE("episode child dispose from range", "[.][episode-child]") {
    disposalPath("dispose-from-range", [](auto& owner, auto&, auto&, auto& destroyed, bool& alive, bool& usable, EpisodeStatus& result) {
        MissionEpisode* parent = owner.get();
        result = parent->range([&](session::World& world) {
            owner.reset();
            alive = !destroyed->load();
            world.step(2);
            usable = world.worldSteps() == 2;
        });
    });
}

TEST_CASE("episode child dispose from judge", "[.][episode-child]") {
    disposalPath("dispose-from-judge", [](auto& owner, auto&, auto&, auto& destroyed, bool& alive, bool& usable, EpisodeStatus& result) {
        MissionEpisode* parent = owner.get();
        result = parent->withJudge([&](env::EpisodeJudge& judge) {
            owner.reset();
            alive = !destroyed->load();
            usable = dynamic_cast<Witness*>(&judge) != nullptr;
        });
    });
}

TEST_CASE("episode child move-assign into active parent", "[.][episode-child]") {
    disposalPath("move-assign-into-active-parent", [](auto& owner, auto&, auto&, auto& destroyed, bool& alive, bool& usable, EpisodeStatus& result) {
        auto other = std::make_shared<std::atomic<bool>>(false);
        MissionEpisode replacement(c2Episode("move-assign-replacement", other));
        replacement.reset(0);
        auto kept = replacement.service(1);
        MissionEpisode* parent = owner.get();
        result = parent->range([&](session::World& world) {
            *parent = std::move(replacement); // (the active parent's own episode ends; its World must not, yet)
            alive = !destroyed->load();
            world.step();
            usable = world.worldSteps() == 1;
        });
        // the parent now is the replacement: its children work, its World is the replacement's
        if (kept.status() != EpisodeStatus::Ok || parent->step() != EpisodeStatus::Ok || other->load()) usable = false;
        owner.reset();
    });
}

TEST_CASE("episode child dispose while another thread is in a callback", "[.][episode-child]") {
    auto destroyed = std::make_shared<std::atomic<bool>>(false);
    auto owner = std::make_unique<MissionEpisode>(c2Episode("dispose-elsewhere", destroyed));
    owner->reset(0);
    auto service = owner->service(1);
    std::promise<void> entered, release;
    auto gate = release.get_future().share();
    std::atomic<bool> worldUsable{false}, aliveAfterDisposal{false};
    std::thread caller([&] {
        service.use([&](session::World& world, const session::AgentIdentity&) {
            entered.set_value();
            gate.wait(); // (the parent is disposed of meanwhile)
            aliveAfterDisposal = !destroyed->load();
            world.step();
            worldUsable = world.worldSteps() == 1;
        });
    });
    entered.get_future().wait();
    owner.reset(); // (must return while the other thread is still inside its callback: the bound is the parent's)
    release.set_value();
    caller.join();
    CHECK(aliveAfterDisposal.load());
    CHECK(worldUsable.load());
    CHECK(destroyed->load());
    CHECK(service.status() == EpisodeStatus::Closed);
}

TEST_CASE("mission episode: disposing of the parent inside a callback, or while another thread is in one, neither deadlocks nor frees the World it uses",
          "[ma][episode][handles][dispose]") {
    // Each path runs in a bounded child process: a deadlock fails its case here, and nothing is left running.
    for (const char* name : {"episode child dispose from referee", "episode child dispose from service", "episode child dispose from range",
                             "episode child dispose from judge", "episode child move-assign into active parent",
                             "episode child dispose while another thread is in a callback"})
        requireChildPasses(name);
}

// The bounded runner's own failure paths: a child that hangs is ended within the bound; one that fails is reported.
TEST_CASE("episode child hangs", "[.][episode-child]") {
    std::this_thread::sleep_for(std::chrono::hours(1));
}
TEST_CASE("episode child fails", "[.][episode-child]") {
    FAIL("a deliberate failure");
}
TEST_CASE("episode child passes", "[.][episode-child]") {
    SUCCEED("a deliberate pass");
}

TEST_CASE("mission episode: the bounded child runner ends a hung child and reports a failed one", "[ma][episode][harness]") {
    const auto start = std::chrono::steady_clock::now();
    const ChildRun hung = runChild("episode child hangs", 3.0);
    const double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    CHECK_FALSE(hung.finished);
    CHECK(hung.exitCode == 124); // (terminated by the runner, and waited for)
    CHECK(waited < 30.0);
    const ChildRun failed = runChild("episode child fails", 60.0);
    CHECK(failed.finished);
    CHECK(failed.exitCode != 0);
    CHECK(failed.output.find("a deliberate failure") != std::string::npos);
    const ChildRun passed = runChild("episode child passes", 60.0);
    CHECK(passed.finished);
    CHECK(passed.exitCode == 0);
}

TEST_CASE("mission episode: threads that fail at once each keep their own failure", "[ma][episode][harness]") {
    std::exception_ptr first, second;
    std::promise<void> go;
    auto start = go.get_future().share();
    std::thread a(guarded(first, [start] {
        start.wait();
        throw std::runtime_error("first worker");
    }));
    std::thread b(guarded(second, [start] {
        start.wait();
        throw std::runtime_error("second worker");
    }));
    go.set_value(); // (both fail together)
    a.join();
    b.join();
    const auto message = [](const std::exception_ptr& slot) {
        try {
            std::rethrow_exception(slot);
        } catch (const std::exception& error) {
            return std::string(error.what());
        }
    };
    REQUIRE(first);
    REQUIRE(second);
    CHECK(message(first) == "first worker");
    CHECK(message(second) == "second worker");
    std::string rethrown;
    try {
        rethrowFirst({nullptr, second, first});
    } catch (const std::exception& error) {
        rethrown = error.what();
    }
    CHECK(rethrown == "second worker");
    CHECK_NOTHROW(rethrowFirst({nullptr, nullptr}));
}

TEST_CASE("mission episode: a child opened while a reset is building its World answers once the reset is done", "[ma][episode][handles][concurrency]") {
    for (const bool fails : {false, true}) {
        CAPTURE(fails);
        auto destroyed = std::make_shared<std::atomic<bool>>(false);
        env::EpisodeConfig config = c2Episode("open-during-reset", destroyed);
        std::promise<void> entered, release;
        auto gate = release.get_future().share();
        auto first = std::make_shared<std::atomic<bool>>(true);
        config.native = [&, first](session::World&) {
            if (!first->exchange(false)) return; // (only the first reset pauses)
            entered.set_value();
            gate.wait();
            if (fails) throw std::runtime_error("the recipe failed");
        };
        MissionEpisode owner(config);
        std::thread resetter([&] {
            try {
                owner.reset(0);
            } catch (const env::EpisodeError&) {
            }
        });
        entered.get_future().wait();
        auto service = owner.service(1);
        auto referee = owner.referee();
        const EpisodeStatus during = owner.step();
        release.set_value();
        resetter.join();
        CHECK(during == EpisodeStatus::Busy);
        bool called = false;
        const EpisodeStatus used = service.use([&](session::World&, const session::AgentIdentity&) { called = true; });
        const EpisodeStatus inspected = referee.inspect([&](const session::World&) {});
        if (!fails) {
            CHECK(used == EpisodeStatus::Ok); // (it belongs to the generation being built, and that generation now has its World)
            CHECK(called);
            CHECK(inspected == EpisodeStatus::Ok);
        } else {
            CHECK(used == EpisodeStatus::NotReset); // (the generation has no World)
            CHECK_FALSE(called);
            CHECK(inspected == EpisodeStatus::NotReset);
            owner.reset(1);
            CHECK(service.status() == EpisodeStatus::Stale);
        }
    }
}

TEST_CASE("mission episode: the scenario's terrain is the World's on every reset, offline", "[ma][episode][recipe][terrain]") {
    env::EpisodeConfig config;
    config.name = "episode-terrain";
    Scenario scenario;
    scenario.source = "episode-terrain";
    scenario.world.terrain = true;
    scenario.world.terrainOffline = true; // (local tiles only: nothing is fetched)
    scenario.world.terrainZoom = 12;
    scenario.world.terrainUrl = "file:///D:/ENV/tmp/m4-no-terrain-tiles/{z}/{x}/{y}.png";
    config.scenario = scenario;
    MissionEpisode episode(config);
    for (const std::uint64_t local : {0u, 1u}) {
        episode.reset(local);
        bool terrain = false;
        unsigned zoom = 0;
        double height = std::numeric_limits<double>::quiet_NaN();
        REQUIRE(episode.range([&](session::World& world) {
            terrain = world.terrain() != nullptr;
            if (terrain) zoom = world.terrain()->zoom(), height = world.terrain()->heightAboveEllipsoidM(0.7, 0.1);
        }) == EpisodeStatus::Ok);
        CHECK(terrain);
        CHECK(zoom == 12);
        CHECK(std::isfinite(height)); // (no tile on disk: the provider still answers, offline)
    }
}

TEST_CASE("mission episode: the World's worker count does not change an episode's native sequence", "[ma][episode][seed][workers]") {
    const std::vector<unsigned> steps{1, 4, 30};
    std::vector<fixture::Snapshot> reference;
    for (const unsigned workers : {1u, 2u, 4u}) {
        CAPTURE(workers);
        FixtureEpisode e(7, 99);
        e.config.workers = workers;
        MissionEpisode episode(e.config);
        const auto samples = trace(episode, *e.scenario, 2, steps);
        if (reference.empty()) reference = samples;
        else sameTrace(samples, reference);
    }
}

TEST_CASE("mission episode: episodes on their own threads, resetting at their own times, keep their sequences", "[ma][episode][seed][concurrency]") {
    const std::vector<unsigned> steps{1, 4, 30};
    // serial references, one episode at a time
    const auto alone = [&](std::uint64_t envId, std::uint64_t local) {
        FixtureEpisode e(envId, 99);
        MissionEpisode episode(e.config);
        return trace(episode, *e.scenario, local, steps);
    };
    const auto referenceA = alone(7, 2), referenceB = alone(8, 3);
    // the same episodes on threads of their own, while a third resets and steps without pause
    std::vector<fixture::Snapshot> tracedA, tracedB;
    std::exception_ptr failA, failB, failChurn; // (one slot per thread, read after joining)
    std::atomic<bool> running{true};
    std::atomic<unsigned> churned{0};
    std::thread a(guarded(failA, [&] {
        FixtureEpisode e(7, 99);
        MissionEpisode episode(e.config);
        tracedA = trace(episode, *e.scenario, 2, steps);
    }));
    std::thread b(guarded(failB, [&] {
        FixtureEpisode e(8, 99);
        MissionEpisode episode(e.config);
        episode.reset(0); // (resets of its own first, at other local episodes)
        episode.reset(9);
        tracedB = trace(episode, *e.scenario, 3, steps);
    }));
    std::thread churn(guarded(failChurn, [&] {
        FixtureEpisode e(9, 5);
        MissionEpisode episode(e.config);
        for (std::uint64_t k = 0; running.load(); ++k) {
            episode.reset(k);
            episode.range([&](session::World& world) {
                const std::unique_ptr<World> facade(World::borrowed(world));
                fixture::contaminate(*facade, *e.scenario);
            });
            for (int j = 0; j < 3; ++j) episode.step();
            ++churned;
        }
    }));
    a.join();
    b.join();
    running = false;
    churn.join();
    rethrowFirst({failA, failB, failChurn});
    CHECK(churned.load() > 0);
    sameTrace(tracedA, referenceA);
    sameTrace(tracedB, referenceB);
}

TEST_CASE("mission episode: metadata is published whole - read at any time, from any thread, during resets and steps", "[ma][episode][handles][concurrency]") {
    auto destroyed = std::make_shared<std::atomic<bool>>(false);
    env::EpisodeConfig config = c2Episode("snapshot", destroyed);
    config.diagnostic = true; // (the World's name carries the generation: name and generation must agree)
    config.envId = 4;
    MissionEpisode episode(config);
    std::atomic<bool> running{true};
    std::atomic<unsigned> reads{0}, incoherent{0};
    std::thread reader([&] {
        while (running.load()) {
            const auto i = episode.info();
            const bool open = i->status == EpisodeStatus::Ok;
            const std::string expected = "snapshot-e4-g" + std::to_string(i->generation);
            if (open && (i->building || i->worldName != expected || i->worldSeed != env::episode_seeds::world(0, 4, i->localEpisodeIndex))) ++incoherent;
            if (!open && (!i->worldName.empty() || i->tick != 0)) ++incoherent;
            if (i->building && i->status != EpisodeStatus::NotReset) ++incoherent;
            ++reads;
            // the parent's own getters read one snapshot each; a child opened meanwhile binds to a generation
            (void)episode.generation(), (void)episode.tick(), (void)episode.worldName(), (void)episode.open();
            const auto child = episode.service(1);
            if (child.status() == EpisodeStatus::Ok && child.generation() == 0) ++incoherent;
        }
    });
    for (std::uint64_t k = 0; k < 12; ++k) {
        episode.reset(k);
        for (int j = 0; j < 5; ++j) REQUIRE(episode.step() == EpisodeStatus::Ok);
    }
    running = false;
    reader.join();
    CHECK(reads.load() > 0);
    CHECK(incoherent.load() == 0);
    const auto last = episode.info();
    CHECK(last->status == EpisodeStatus::Ok);
    CHECK(last->generation == 12);
    CHECK(last->localEpisodeIndex == 11);
    CHECK(last->tick == 5);
    CHECK(last->worldName == "snapshot-e4-g12");
    CHECK_FALSE(last->building);
    // a child opened inside a callback binds to the current generation and answers once that call is over
    std::optional<env::EpisodeService> inside;
    CHECK(episode.range([&](session::World&) { inside = episode.service(1); }) == EpisodeStatus::Ok);
    REQUIRE(inside);
    CHECK(inside->generation() == 12);
    CHECK(inside->use([](session::World&, const session::AgentIdentity&) {}) == EpisodeStatus::Ok);
    // ended: Closed, from the parent's own metadata as from its children
    auto referee = episode.referee();
    MissionEpisode taken(std::move(episode));
    CHECK(episode.info()->status == EpisodeStatus::Closed); // NOLINT(bugprone-use-after-move)
    CHECK(taken.info()->generation == 12);
    CHECK(referee.status() == EpisodeStatus::Ok);
}

TEST_CASE("mission episode: World construction inputs - the scenario's admitted keys or the episode's own, never dropped", "[ma][episode][recipe][terrain]") {
    // without a scenario: the episode's own construction inputs, terrain among them, on every reset
    env::EpisodeConfig own;
    own.name = "episode-terrain-own";
    own.world.terrain = true;
    own.world.terrainOffline = true;
    own.world.terrainZoom = 11;
    own.world.terrainUrl = "file:///D:/ENV/tmp/m4-no-terrain-tiles/{z}/{x}/{y}.png";
    own.world.capacity = 64;
    MissionEpisode episode(own);
    for (const std::uint64_t local : {0u, 3u}) {
        episode.reset(local);
        unsigned zoom = 0;
        REQUIRE(episode.range([&](session::World& world) { zoom = world.terrain() ? world.terrain()->zoom() : 0; }) == EpisodeStatus::Ok);
        CHECK(zoom == 11);
    }
    // a scenario world section that sets what the recipe does not admit is refused, not ignored
    const auto refused = [](auto change) {
        env::EpisodeConfig c;
        c.name = "episode-world-refused";
        Scenario scenario;
        scenario.source = "episode-world-refused";
        change(scenario.world, c);
        c.scenario = scenario;
        CHECK_THROWS_AS(MissionEpisode(c), env::EpisodeError);
    };
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.dt = 1.0 / 60.0; });
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.frameSkip = 2; });
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.workers = 3; });
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.seed = 5; });
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.publish = false; });
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.recordPath = "D:/ENV/tmp/x.fsrec"; });
    refused([](WorldOptions& w, env::EpisodeConfig&) { w.name = "another"; });
    refused([](WorldOptions& w, env::EpisodeConfig& c) { w.terrain = true, c.world.terrainZoom = 12; }); // (two sources)
    // the admitted keys, and the World's name when it is the episode's own, are accepted
    env::EpisodeConfig fine;
    fine.name = "episode-world-fine";
    Scenario scenario;
    scenario.source = "episode-world-fine";
    scenario.world.name = "episode-world-fine";
    scenario.world.capacity = 32;
    scenario.world.terrain = true;
    scenario.world.terrainZoom = 10;
    scenario.world.terrainUrl = "file:///D:/ENV/tmp/m4-no-terrain-tiles/{z}/{x}/{y}.png";
    fine.scenario = scenario;
    MissionEpisode accepted(fine);
    accepted.reset(0);
    unsigned zoom = 0;
    REQUIRE(accepted.range([&](session::World& world) { zoom = world.terrain() ? world.terrain()->zoom() : 0; }) == EpisodeStatus::Ok);
    CHECK(zoom == 10);
}

TEST_CASE("mission episode: every exceptional exit from a reset leaves coherent failed-reset metadata", "[ma][episode][reset][exceptions]") {
    // Whatever a range callback throws - standard or not - the reset unwinds to: no World, not building, the boundary
    // free, and a later reset that succeeds. The callback's own exception reaches the caller unchanged unless it is a
    // std::exception (then an EpisodeError, NotReset, as before).
    const auto failing = [](bool inJudge, auto thrower) {
        auto fail = std::make_shared<bool>(true);
        env::EpisodeConfig config;
        config.name = "reset-exception";
        config.c2Nodes = {{"crc", 0.7, 0.0, 200.0}};
        config.services = {{1, "crc", {}, true, true}};
        if (inJudge)
            config.judge = [fail, thrower](const session::World&) -> std::unique_ptr<env::EpisodeJudge> {
                if (*fail) thrower();
                return nullptr;
            };
        else
            config.native = [fail, thrower](session::World&) {
                if (*fail) thrower();
            };
        return std::make_pair(config, fail);
    };
    const auto coherentAfterFailure = [](MissionEpisode& episode, std::uint64_t generation) {
        const auto info = episode.info();
        CHECK(info->status == EpisodeStatus::NotReset);
        CHECK_FALSE(info->building);
        CHECK(info->generation == generation);
        CHECK(info->worldName.empty());
        CHECK(info->tick == 0);
        CHECK_FALSE(episode.open());
        CHECK(episode.step() == EpisodeStatus::NotReset);                     // (the boundary is free, and no World)
        CHECK(episode.range([](session::World&) {}) == EpisodeStatus::NotReset);
        CHECK(episode.service(1).status() == EpisodeStatus::NotReset);
        CHECK(episode.referee().status() == EpisodeStatus::NotReset);
    };
    for (const bool inJudge : {false, true}) {
        CAPTURE(inJudge);
        { // a non-standard exception: rethrown as it is
            auto [config, fail] = failing(inJudge, [] { throw 42; });
            MissionEpisode episode(config);
            int caught = 0;
            try {
                episode.reset(0);
            } catch (int value) {
                caught = value;
            }
            CHECK(caught == 42);
            coherentAfterFailure(episode, 1);
            auto earlier = episode.service(1);
            *fail = false;
            CHECK(episode.reset(1) == 2); // (a retry succeeds)
            CHECK(episode.open());
            CHECK_FALSE(episode.info()->building);
            CHECK(earlier.status() == EpisodeStatus::Stale);
            CHECK(episode.service(1).use([](session::World&, const session::AgentIdentity&) {}) == EpisodeStatus::Ok);
        }
        { // a standard exception: an EpisodeError, NotReset
            auto [config, fail] = failing(inJudge, [] { throw std::runtime_error("refused"); });
            MissionEpisode episode(config);
            try {
                episode.reset(0);
                FAIL("the reset did not fail");
            } catch (const env::EpisodeError& error) {
                CHECK(error.status() == EpisodeStatus::NotReset);
            }
            coherentAfterFailure(episode, 1);
            *fail = false;
            CHECK(episode.reset(0) == 2);
            CHECK(episode.open());
        }
        { // an EpisodeError of the callback's own: as it is
            auto [config, fail] = failing(inJudge, [] { throw env::EpisodeError(EpisodeStatus::UnknownService, "own"); });
            MissionEpisode episode(config);
            try {
                episode.reset(0);
                FAIL("the reset did not fail");
            } catch (const env::EpisodeError& error) {
                CHECK(error.status() == EpisodeStatus::UnknownService);
            }
            coherentAfterFailure(episode, 1);
            *fail = false;
            CHECK(episode.reset(0) == 2);
        }
    }
}
