// Independent mission episodes (M4; docs/sdk/mission-training.md, sections 4 and 8): one World per episode, rebuilt
// whole on reset; children bound to a generation; seed streams that depend on nothing but their inputs. The native
// states a reset must clear come from the supplied fixture (docs/sdk/native-state-reset-fixture.md), applied here to
// the episode's own reset - not to a hand-built replacement World.
#include "env/MissionEpisode.h"
#include "fixtures/mission_training/NativeStateReset.h"

#include "core/Json.h"
#include "fsim/MAService.h"
#include "fsim/Rng.h"
#include "fsim/World.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
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
