// Construction options remain independent of episode control and Viewer presentation. Every public field maps to
// native construction, while episode-owned scenario inputs fail explicitly instead of being silently dropped.

#include "training/native/episodes/runtime/MissionEpisode.h"
#include "fsim/World.h"
#include "simulation/runtime/world/World.h"
#include "simulation/api/WorldOptions.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>

namespace {

void requireMapping(const fsim::WorldOptions& input) {
    const auto mapped = fsim::session::makeWorldOptions(input);
    CHECK(mapped.name == input.name);
    CHECK(mapped.dt == input.dt);
    CHECK(mapped.frameSkip == input.frameSkip);
    CHECK(mapped.workers == input.workers);
    CHECK(mapped.pinWorkers == input.pinWorkers);
    CHECK(mapped.seed == input.seed);
    CHECK(mapped.capacity == input.capacity);
    CHECK(mapped.publish == input.publish);
    CHECK(mapped.publishIntervalSeconds == input.publishIntervalSeconds);
    CHECK(mapped.jsbsimRoot == std::filesystem::path(input.jsbsimRoot));
    CHECK(mapped.terrain == input.terrain);
    CHECK(mapped.terrainUrl == input.terrainUrl);
    CHECK(mapped.terrainZoom == input.terrainZoom);
    CHECK(mapped.terrainOffline == input.terrainOffline);
    CHECK(mapped.storeDrag == input.storeDrag);
    CHECK(mapped.ground == input.ground);
    CHECK(mapped.recordPath == std::filesystem::path(input.recordPath));
    CHECK(mapped.recordIntervalSeconds == input.recordIntervalSeconds);
    CHECK(mapped.terrainPrefetchRadiusM == fsim::session::WorldOptions{}.terrainPrefetchRadiusM);
}

template <class Change>
void refuseScenario(Change&& change) {
    fsim::env::EpisodeConfig config;
    config.name = "episode-option-refusal";
    fsim::Scenario scenario;
    change(scenario);
    config.scenario = std::move(scenario);
    CHECK_THROWS_AS(fsim::env::MissionEpisode(config), fsim::env::EpisodeError);
}

} // namespace

TEST_CASE("World construction: all public C++ options have one complete native mapping", "[episode][recipe][construction]") {
    requireMapping(fsim::WorldOptions{});
    for (const bool enabled : {false, true}) {
        fsim::WorldOptions input;
        input.name = enabled ? "mapped-world-a" : "mapped-world-b";
        input.dt = enabled ? 0.0125 : 0.0075;
        input.frameSkip = enabled ? 7 : 3;
        input.workers = enabled ? 4u : 2u;
        input.pinWorkers = enabled;
        input.seed = enabled ? 0xfedcba9876543210ULL : 0x123456789abcdef0ULL;
        input.capacity = enabled ? 509u : 127u;
        input.publish = enabled;
        input.publishIntervalSeconds = enabled ? 0.125 : 0.25;
        input.jsbsimRoot = enabled ? "D:/ENV/tmp/mapping/root-a" : "D:/ENV/tmp/mapping/root-b";
        input.terrain = enabled;
        input.terrainUrl = enabled ? "file:///tiles-a/{z}/{x}/{y}.png" : "file:///tiles-b/{z}/{x}/{y}.png";
        input.terrainZoom = enabled ? 12u : 9u;
        input.terrainOffline = enabled;
        input.storeDrag = enabled;
        input.ground = enabled ? std::make_shared<fsim::FlatGround>(432.25) : nullptr;
        input.recordPath = enabled ? "D:/ENV/tmp/mapping/a.fsrec" : "D:/ENV/tmp/mapping/b.fsrec";
        input.recordIntervalSeconds = enabled ? 0.375 : 0.625;
        requireMapping(input);
    }
}

TEST_CASE("World construction: public and native defaults retain their distinct terrain policy", "[episode][recipe][construction]") {
    const fsim::WorldOptions publicDefaults;
    const fsim::session::WorldOptions nativeDefaults;
    const auto mapped = fsim::session::makeWorldOptions(publicDefaults);
    CHECK(publicDefaults.terrainOffline);
    CHECK_FALSE(nativeDefaults.terrainOffline);
    CHECK(mapped.terrainOffline);
    CHECK(mapped.terrainPrefetchRadiusM == nativeDefaults.terrainPrefetchRadiusM);
}

TEST_CASE("World construction: independent boolean options cannot be interchanged", "[boundary][recipe][construction]") {
    for (const auto member : {&fsim::WorldOptions::pinWorkers, &fsim::WorldOptions::publish,
                              &fsim::WorldOptions::terrain, &fsim::WorldOptions::terrainOffline,
                              &fsim::WorldOptions::storeDrag}) {
        fsim::WorldOptions input;
        input.*member = !(input.*member);
        requireMapping(input);
    }
}

TEST_CASE("mission episode: every remaining episode-owned scenario option is explicitly refused", "[episode][recipe][construction]") {
    SECTION("worker affinity") {
        refuseScenario([](fsim::Scenario& scenario) { scenario.world.pinWorkers = false; });
    }
    SECTION("publication interval") {
        refuseScenario([](fsim::Scenario& scenario) { scenario.world.publishIntervalSeconds = 0.25; });
    }
    SECTION("store drag policy") {
        refuseScenario([](fsim::Scenario& scenario) { scenario.world.storeDrag = false; });
    }
    SECTION("custom ground provider") {
        refuseScenario([](fsim::Scenario& scenario) { scenario.world.ground = std::make_shared<fsim::FlatGround>(12.5); });
    }
    SECTION("recording interval") {
        refuseScenario([](fsim::Scenario& scenario) { scenario.world.recordIntervalSeconds = 0.5; });
    }
}

TEST_CASE("mission episode: a scenario's flight VecEnv configuration is refused instead of discarded", "[boundary][recipe][vecenv]") {
    refuseScenario([](fsim::Scenario& scenario) { scenario.vecenv.present = true; });
}
