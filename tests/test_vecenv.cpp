// Environment layer: layout, determinism, auto-reset semantics.
#include "env/VecEnv.h"
#include "fsim/BuiltinEffects.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace fsim;

namespace {

env::Scenario testScenario() {
    env::Scenario s;
    s.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    s.maxEpisodeSteps = 20;
    return s;
}

env::VecEnv::Options options(unsigned numEnvs, std::uint64_t seed, unsigned workers) {
    env::VecEnv::Options o;
    o.numEnvs = numEnvs;
    o.seed = seed;
    o.workers = workers;
    return o;
}

std::vector<float> zeroActions(const env::VecEnv& e) { return std::vector<float>(e.numVehicles() * e.actionSize(), 0.0f); }

} // namespace

TEST_CASE("VecEnv sizes and layout", "[env]") {
    auto s = testScenario();
    s.vehiclesPerEnv = 2;
    env::VecEnv e(s, options(3, 0, 2));

    REQUIRE(e.numEnvs() == 3);
    REQUIRE(e.vehiclesPerEnv() == 2);
    REQUIRE(e.numVehicles() == 6);
    REQUIRE(e.observationSize() == 20);
    REQUIRE(e.actionSize() == 4);
    REQUIRE(e.observationNames().size() == e.observationSize());
    REQUIRE(e.actionNames().size() == e.actionSize());
    REQUIRE_THAT(e.agentStepSeconds(), Catch::Matchers::WithinRel(4.0 / 120.0, 1e-9));

    auto r = e.reset(7);
    REQUIRE(r.observations.size() == 6 * 20);
    REQUIRE(r.rewards.size() == 6);
    REQUIRE(r.terminated.size() == 6);
    REQUIRE(r.truncated.size() == 6);
    REQUIRE(r.episodeSteps.size() == 3);
    for (auto v : r.observations) REQUIRE(std::isfinite(v));
    for (auto t : r.terminated) REQUIRE(t == 0);

    // Different envs sample different targets.
    REQUIRE(e.taskState(0).targetAltitudeM != e.taskState(2).targetAltitudeM);

    r = e.step(zeroActions(e));
    REQUIRE(r.episodeSteps[0] == 1);
    REQUIRE(e.vehicleSteps() == 6 * 4);
    REQUIRE_THROWS_AS(e.step(Span<const float>(r.rewards.data(), 1)), std::invalid_argument);
}

TEST_CASE("VecEnv trajectories are deterministic for a seed and worker count", "[env]") {
    auto run = [](unsigned workers) {
        env::VecEnv e(testScenario(), options(4, 99, workers));
        e.reset();
        std::vector<float> a(e.numVehicles() * e.actionSize());
        std::vector<float> trace;
        for (int k = 0; k < 10; ++k) {
            for (std::size_t i = 0; i < a.size(); ++i) a[i] = 0.1f * static_cast<float>((i + static_cast<std::size_t>(k)) % 5) - 0.2f;
            auto r = e.step(a);
            trace.insert(trace.end(), r.observations.begin(), r.observations.end());
            trace.insert(trace.end(), r.rewards.begin(), r.rewards.end());
        }
        return trace;
    };
    const auto one = run(1), four = run(4);
    REQUIRE(one.size() == four.size());
    REQUIRE(one == four);

    // A different seed gives different initial conditions.
    env::VecEnv a(testScenario(), options(4, 99, 1));
    env::VecEnv b(testScenario(), options(4, 100, 1));
    const auto ra = a.reset(), rb = b.reset();
    REQUIRE(!std::equal(ra.observations.begin(), ra.observations.end(), rb.observations.begin()));
}

TEST_CASE("VecEnv truncates at maxEpisodeSteps and auto-resets next step", "[env]") {
    auto s = testScenario();
    s.maxEpisodeSteps = 5;
    env::VecEnv e(s, options(2, 0, 1));
    const auto first = e.reset(3);

    env::VecEnv::StepResult r{};
    for (int k = 0; k < 5; ++k) r = e.step(zeroActions(e));
    REQUIRE(r.episodeSteps[0] == 5);
    REQUIRE(r.truncated[0] == 1);
    REQUIRE(r.terminated[0] == 0);
    REQUIRE(r.rewards[0] > 0.0f);
    const std::vector<float> terminal(r.observations.begin(), r.observations.end());
    REQUIRE(std::equal(terminal.begin(), terminal.end(), e.finalObservations().begin()));

    // Next step: new episode, flags cleared, reward 0, step counter reset,
    // observation is a fresh initial state (not a continuation).
    r = e.step(zeroActions(e));
    REQUIRE(r.episodeSteps[0] == 0);
    REQUIRE(r.truncated[0] == 0);
    REQUIRE(r.terminated[0] == 0);
    REQUIRE(r.rewards[0] == 0.0f);
    REQUIRE(!std::equal(terminal.begin(), terminal.end(), r.observations.begin()));
    // ...and it is an initial state: no time has elapsed, so the reset's
    // throttle command has not reached the engine yet (0 as after reset()).
    const auto& names = e.observationNames();
    const auto it = std::find(names.begin(), names.end(), "throttle");
    REQUIRE(it != names.end());
    const auto throttle = static_cast<std::size_t>(it - names.begin());
    REQUIRE(r.observations[throttle] == first.observations[throttle]);

    r = e.step(zeroActions(e));
    REQUIRE(r.episodeSteps[0] == 1);
}

TEST_CASE("VecEnv terminates on crash", "[env]") {
    auto s = testScenario();
    s.maxEpisodeSteps = 4000;
    s.initial.centre.altitudeMslM = 60.0; // just above the 30 m AGL floor
    s.initial.altitudeJitterM = 0.0;
    env::VecEnv e(s, options(1, 0, 1));
    e.reset(1);
    REQUIRE(e.actionSize() == 4);
    std::vector<float> dive{0.0f, -1.0f, 0.0f, -1.0f}; // full nose-down elevator, idle throttle
    bool terminated = false;
    for (int k = 0; k < 300 && !terminated; ++k) {
        auto r = e.step(dive);
        terminated = r.terminated[0] != 0;
        if (terminated) REQUIRE(r.rewards[0] < -9.0f); // -10 crash penalty plus the (< 1) shaping term
    }
    REQUIRE(terminated);
}

TEST_CASE("VecEnv: each vehicle's agent observes its own ship as reported and acts as its own controller", "[env][agents]") {
    // (ADR-31, AB-3) sensor noise on one vehicle moves its observation alone; the rewards - the referee's, from the true
    // state - stay the same to the bit (actuator actions: no loop flies on the noisy report)
    auto s = testScenario();
    s.vehiclesPerEnv = 2;
    env::VecEnv clean(s, options(2, 5, 1)), noisy(s, options(2, 5, 1));
    const auto& agents = noisy.world().agents();
    REQUIRE(agents.size() == 4);
    for (std::size_t i = 0; i < agents.size(); ++i) {
        REQUIRE(agents[i].controller > session::World::kAgentControllers);
        for (std::size_t j = 0; j < i; ++j) REQUIRE(agents[j].controller != agents[i].controller);
    }
    auto noise = std::make_unique<effects::GaussianSensorNoise>();
    noise->positionSigmaM = noise->velocitySigmaMs = noise->attitudeSigmaRad = noise->rateSigmaRadS = noise->airspeedSigmaMs = 0.0;
    noise->altitudeSigmaM = 30.0;
    REQUIRE(noisy.world().addEffect(agents[0].vehicle, std::move(noise)));

    clean.reset(5);
    noisy.reset(5);
    const std::size_t o = noisy.observationSize();
    bool moved = false;
    for (int k = 0; k < 10; ++k) {
        const auto rc = clean.step(zeroActions(clean));
        const auto rn = noisy.step(zeroActions(noisy));
        for (std::size_t i = 0; i < 4; ++i) {
            REQUIRE(rc.rewards[i] == rn.rewards[i]);
            REQUIRE(rc.terminated[i] == rn.terminated[i]);
            for (std::size_t j = 0; j < o; ++j)
                if (i != 0) REQUIRE(rc.observations[i * o + j] == rn.observations[i * o + j]);
                else if (rc.observations[j] != rn.observations[j]) moved = true;
            // its observation: its own ship's report
            REQUIRE(rn.observations[i * o] == static_cast<float>(std::clamp(noisy.view(i).ownship().altitudeMslM / 1000.0, -5.0, 5.0)));
        }
    }
    REQUIRE(moved);
    // its actions: its agent's activity
    for (std::size_t i = 0; i < 4; ++i) {
        bool mine = false;
        for (const auto& a : noisy.world().activities(agents[i].vehicle))
            if (a.live()) mine = a.controller == agents[i].controller;
        REQUIRE(mine);
    }
}
