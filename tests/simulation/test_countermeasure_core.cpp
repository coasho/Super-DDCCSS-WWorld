#include "simulation/mission_systems/countermeasure/Dispenser.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace cm = fsim::ms::countermeasure;

namespace {
void require(bool result, const char* message) {
    if (!result) throw std::runtime_error(message);
}

bool samePoint(cm::Point a, cm::Point b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool finite(cm::Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool nearPoint(cm::Point a, cm::Point b, double tolerance = 1e-11) {
    return std::fabs(a.x - b.x) <= tolerance && std::fabs(a.y - b.y) <= tolerance && std::fabs(a.z - b.z) <= tolerance;
}

cm::Decoy analytic(cm::Decoy initial, cm::Point wind, cm::Point down, double seconds, double drag) {
    const double norm = std::hypot(down.x, down.y, down.z);
    const double attenuation = std::exp(-drag * seconds), integral = (1 - attenuation) / drag;
    const auto axis = [&](double position, double velocity, double air, double direction, double& p, double& v) {
        const double acceleration = 9.80665 * direction / norm;
        p = position + air * seconds + (velocity - air) * integral + acceleration / drag * (seconds - integral);
        v = air + (velocity - air) * attenuation + acceleration * integral;
    };
    cm::Decoy out = initial;
    axis(initial.position.x, initial.velocity.x, wind.x, down.x, out.position.x, out.velocity.x);
    axis(initial.position.y, initial.velocity.y, wind.y, down.y, out.position.y, out.velocity.y);
    axis(initial.position.z, initial.velocity.z, wind.z, down.z, out.position.z, out.velocity.z);
    return out;
}

void inventoryAndSharedCooldown() {
    cm::Dispenser dispenser(7, 2, 1);
    require(dispenser.dispense(cm::Kind::Chaff, 100.0, {1, 2, 3}, {100, 0, 0}, 11), "first chaff must release");
    require(dispenser.remaining(cm::Kind::Chaff) == 1 && dispenser.remaining(cm::Kind::Flare) == 1,
            "chaff must consume only chaff inventory");
    require(!dispenser.dispense(cm::Kind::Flare, 100.34, {}, {}, 12), "alternating kinds must share cooldown");
    require(dispenser.dispense(cm::Kind::Flare, 100.35, {}, {}, 12), "cooldown boundary must allow release");
    require(dispenser.remaining(cm::Kind::Flare) == 0, "flare inventory must be exhausted");
    require(!dispenser.dispense(cm::Kind::Flare, 100.70, {}, {}, 13), "empty flare inventory must reject release");
    require(dispenser.dispense(cm::Kind::Chaff, 100.70, {}, {}, 13), "rejection must not advance cooldown");
    require(!dispenser.dispense(cm::Kind::Chaff, 101.05, {}, {}, 14), "empty chaff inventory must reject release");
    require(dispenser.decoys().size() == 3, "rejections must not create entities");
    const auto& first = dispenser.decoys().front();
    require(first.id == 11 && first.owner == 7 && first.kind == cm::Kind::Chaff && first.launchTimeS == 100.0,
            "decoy must preserve identity, owner, kind and simulation launch time");
    require(samePoint(first.position, {1, 2, 3}) && samePoint(first.velocity, {100, 0, 0}),
            "release must preserve initial kinematics");
}

void invalidReleaseDoesNotConsume() {
    cm::Dispenser dispenser(7, 1, 1);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    require(!dispenser.dispense(cm::Kind::Chaff, nan, {}, {}, 1), "NaN time must reject");
    require(!dispenser.dispense(cm::Kind::Chaff, inf, {}, {}, 1), "infinite time must reject");
    require(!dispenser.dispense(cm::Kind::Chaff, -0.1, {}, {}, 1), "negative time must reject");
    require(!dispenser.dispense(cm::Kind::Chaff, 0, {nan, 0, 0}, {}, 1), "nonfinite position must reject");
    require(!dispenser.dispense(cm::Kind::Chaff, 0, {}, {0, inf, 0}, 1), "nonfinite velocity must reject");
    require(!dispenser.dispense(cm::Kind::Chaff, 0, {}, {}, 0), "zero ID must reject");
    require(!dispenser.dispense(static_cast<cm::Kind>(9), 0, {}, {}, 1), "unknown kind must reject");
    require(dispenser.remaining(cm::Kind::Chaff) == 1 && dispenser.remaining(cm::Kind::Flare) == 1,
            "invalid release must not consume inventory");
    require(dispenser.dispense(cm::Kind::Chaff, 0, {}, {}, 1), "invalid attempts must not advance cooldown");
}

void duplicateAndBackwardReleaseDoNotConsume() {
    cm::Dispenser dispenser(7, 3, 0);
    require(dispenser.dispense(cm::Kind::Chaff, 5, {}, {}, 1), "initial release must succeed");
    require(!dispenser.dispense(cm::Kind::Chaff, 6, {}, {}, 1), "duplicate live ID must reject");
    require(!dispenser.dispense(cm::Kind::Chaff, 4, {}, {}, 2), "backward simulation time must reject");
    require(dispenser.remaining(cm::Kind::Chaff) == 2 && dispenser.decoys().size() == 1,
            "duplicate and backward releases must preserve inventory and entities");
    require(dispenser.dispense(cm::Kind::Chaff, 5.35, {}, {}, 2), "invalid attempts must preserve cooldown");
}

void invalidStepDoesNotAdvanceOrPoison() {
    cm::Dispenser dispenser(7, 1, 0);
    require(dispenser.dispense(cm::Kind::Chaff, 50000, {1, 2, 3}, {100, 20, 0}, 1), "release must succeed");
    const auto before = dispenser.decoys().front();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    dispenser.step(0, {0, 0, 1});
    dispenser.step(-1, {0, 0, 1});
    dispenser.step(nan, {0, 0, 1});
    dispenser.step(inf, {0, 0, 1});
    dispenser.step(1, {nan, 0, 1});
    dispenser.step(1, {});
    const auto& after = dispenser.decoys().front();
    require(samePoint(before.position, after.position) && samePoint(before.velocity, after.velocity),
            "invalid steps must leave kinematics unchanged");
    dispenser.step(5.99, {0, 0, 1});
    require(dispenser.decoys().size() == 1, "invalid steps must not age decoys");
    dispenser.step(0.01, {0, 0, 1});
    require(dispenser.decoys().empty(), "expiry must use elapsed simulation steps");
}

void distinctLifetimeAndLargeStepExpiry() {
    cm::Dispenser dispenser(7, 1, 1);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {}, {}, 1), "chaff release must succeed");
    require(dispenser.dispense(cm::Kind::Flare, 0.35, {}, {}, 2), "flare release must succeed");
    dispenser.step(3.99, {0, 0, 1});
    require(dispenser.decoys().size() == 2, "both kinds must survive before flare expiry");
    dispenser.step(0.01, {0, 0, 1});
    require(dispenser.decoys().size() == 1 && dispenser.decoys().front().kind == cm::Kind::Chaff,
            "flare must expire independently before chaff");
    dispenser.step(std::numeric_limits<double>::max(), {0, 0, 1});
    require(dispenser.decoys().empty(), "large finite step must safely expire surviving decoys");
    require(dispenser.remaining(cm::Kind::Chaff) == 0 && dispenser.remaining(cm::Kind::Flare) == 0,
            "expiry must not replenish inventory");
}

void fixedStepLifetimeBoundary() {
    cm::Dispenser chaff(7, 1, 0), flare(7, 0, 1);
    require(chaff.dispense(cm::Kind::Chaff, 0, {}, {}, 1), "chaff release must succeed");
    require(flare.dispense(cm::Kind::Flare, 0, {}, {}, 2), "flare release must succeed");
    for (int i = 0; i < 79; ++i) flare.step(0.05, {0, 0, 1});
    require(flare.decoys().size() == 1, "flare must survive the step before its lifetime boundary");
    flare.step(0.05, {0, 0, 1});
    require(flare.decoys().empty(), "flare must expire at four seconds of fixed simulation steps");
    for (int i = 0; i < 119; ++i) chaff.step(0.05, {0, 0, 1});
    require(chaff.decoys().size() == 1, "chaff must survive the step before its lifetime boundary");
    chaff.step(0.05, {0, 0, 1});
    require(chaff.decoys().empty(), "chaff must expire at six seconds of fixed simulation steps");
}

void motionAndDirection() {
    cm::Dispenser dispenser(7, 1, 1);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {}, {100, 0, 0}, 1), "chaff release must succeed");
    require(dispenser.dispense(cm::Kind::Flare, 0.35, {}, {100, 0, 0}, 2), "flare release must succeed");
    dispenser.step(1, {0, -2, 0});
    const auto& chaff = dispenser.decoys()[0];
    const auto& flare = dispenser.decoys()[1];
    require(chaff.velocity.x < 5 && flare.velocity.x > chaff.velocity.x && flare.velocity.x < 100,
            "chaff must settle quickly while flare retains more launch velocity");
    require(chaff.position.x > 0 && chaff.position.x < 100 && flare.position.x > chaff.position.x,
            "drag must reduce displacement without reversing launch motion");
    require(chaff.velocity.y < 0 && flare.velocity.y < 0 && chaff.position.y < 0 && flare.position.y < 0,
            "gravity must follow supplied down direction");
    require(chaff.position.z == 0 && flare.position.z == 0, "gravity must not assume a fixed world axis");
}

void deterministicRepeatAndStepPartition() {
    cm::Dispenser one(7, 1, 0), repeat(7, 1, 0), partition(7, 1, 0);
    for (auto* dispenser : {&one, &repeat, &partition})
        require(dispenser->dispense(cm::Kind::Chaff, 0, {1, 2, 3}, {100, 20, 0}, 1), "release must succeed");
    one.step(1, {0, 0, 1});
    repeat.step(1, {0, 0, 1});
    for (int i = 0; i < 10; ++i) partition.step(0.1, {0, 0, 1});
    const auto& a = one.decoys().front();
    const auto& b = repeat.decoys().front();
    const auto& c = partition.decoys().front();
    require(samePoint(a.position, b.position) && samePoint(a.velocity, b.velocity), "same inputs must repeat exactly");
    require(std::fabs(a.position.x - c.position.x) < 1e-10 && std::fabs(a.position.z - c.position.z) < 1e-10 &&
            std::fabs(a.velocity.x - c.velocity.x) < 1e-10 && std::fabs(a.velocity.z - c.velocity.z) < 1e-10,
            "motion must remain consistent across simulation step partitions");
}

void extremeFiniteStateRemainsFinite() {
    cm::Dispenser dispenser(7, 1, 0);
    const double largest = std::numeric_limits<double>::max();
    require(dispenser.dispense(cm::Kind::Chaff, 0, {largest, largest, largest}, {largest, largest, largest}, 1),
            "finite launch state must be accepted");
    dispenser.step(1, {largest, largest, largest});
    require(dispenser.decoys().empty(), "arithmetic failure must remove the invalid live pose rather than freeze it as fresh motion");
}

void trajectoriesPreserveBeforeAndAnalyticAfter() {
    cm::Dispenser dispenser(7, 1, 0);
    require(dispenser.dispense(cm::Kind::Chaff, 1000, {1, 2, 3}, {100, 20, -5}, 1), "release must succeed");
    const auto initial = dispenser.decoys().front();
    const auto paths = dispenser.advance(.5, [](const cm::Decoy&) { return cm::Motion{{0, 0, -2}, {10, -3, 0}}; });
    require(paths.size() == 1 && paths[0].valid && !paths[0].expires, "a live decoy must produce one valid owned trajectory");
    const auto& path = paths.front();
    require(path.before.ageS == 0 && samePoint(path.before.state.position, initial.position)
                && samePoint(path.before.state.velocity, initial.velocity),
            "trajectory must retain actual pre-step kinematics rather than reconstructing them from final velocity");
    require(path.stepDurationS == .5 && path.liveDurationS == .5 && path.activeEnd.ageS == .5,
            "trajectory must retain the requested step and actual active duration");
    const auto expected = analytic(initial, {10, -3, 0}, {0, 0, -2}, .5, 4);
    require(nearPoint(path.activeEnd.state.position, expected.position) && nearPoint(path.activeEnd.state.velocity, expected.velocity),
            "trajectory endpoint must follow the analytic wind-relative linear-drag law");
    require(samePoint(dispenser.decoys().front().position, path.activeEnd.state.position),
            "live storage must publish the actual trajectory endpoint");
    cm::Sample atStart, atEnd;
    require(cm::sampleAt(path, 0, atStart) && cm::sampleAt(path, .5, atEnd), "both geometric interval endpoints must be sampleable");
    require(samePoint(atStart.state.position, initial.position) && samePoint(atEnd.state.position, path.activeEnd.state.position),
            "endpoint samples must preserve stored exact endpoint states");
}

void partialExpiryRetainsFinalTrajectory() {
    cm::Dispenser dispenser(7, 0, 1);
    require(dispenser.dispense(cm::Kind::Flare, 50, {2, 3, 4}, {80, 0, 5}, 1), "flare release must succeed");
    const auto initial = dispenser.decoys().front();
    const auto environment = [](const cm::Decoy&) { return cm::Motion{{0, 0, -1}, {}}; };
    dispenser.advance(3.9, environment);
    const auto paths = dispenser.advance(.2, environment);
    require(paths.size() == 1 && paths[0].valid && paths[0].expires && dispenser.decoys().empty(),
            "an expiring decoy must retain its final partial trajectory after leaving live storage");
    const auto& path = paths.front();
    require(std::fabs(path.before.ageS - 3.9) < 1e-12 && path.activeEnd.ageS == 4
                && std::fabs(path.liveDurationS - .1) < 1e-12 && path.stepDurationS == .2,
            "expiry must cap motion at its actual lifetime rather than dropping the whole interval");
    const auto expected = analytic(initial, {}, {0, 0, -1}, 4, .4);
    require(nearPoint(path.activeEnd.state.position, expected.position) && nearPoint(path.activeEnd.state.velocity, expected.velocity),
            "expiry endpoint must be the actual analytic state at the lifetime boundary");
    cm::Sample boundary;
    require(cm::sampleAt(path, path.liveDurationS, boundary) && boundary.ageS == 4,
            "exclusive expiry must still expose its mathematical endpoint for clipped geometry");
    const auto unchanged = boundary;
    require(!cm::sampleAt(path, std::nextafter(path.liveDurationS, std::numeric_limits<double>::infinity()), boundary)
                && boundary.ageS == unchanged.ageS && samePoint(boundary.state.position, unchanged.state.position),
            "sampling beyond expiry must fail without extrapolating or changing the output");
}

void individualEnvironmentAndWindRelativeMotion() {
    cm::Dispenser together(7, 2, 0), alone(7, 1, 0);
    require(together.dispense(cm::Kind::Chaff, 0, {}, {100, 0, 0}, 1), "first release must succeed");
    require(together.dispense(cm::Kind::Chaff, .35, {5, 6, 7}, {0, 50, 0}, 2), "second release must succeed");
    require(alone.dispense(cm::Kind::Chaff, .35, {5, 6, 7}, {0, 50, 0}, 2), "independent release must succeed");
    const auto initialFirst = together.decoys()[0], initialSecond = together.decoys()[1];
    const auto environment = [](const cm::Decoy& decoy) {
        return decoy.id == 1 ? cm::Motion{{0, 0, 1}, {10, 0, 0}} : cm::Motion{{0, -1, 0}, {0, 20, 0}};
    };
    const auto paths = together.advance(.25, environment), independent = alone.advance(.25, environment);
    require(paths.size() == 2 && independent.size() == 1 && paths[0].valid && paths[1].valid,
            "each decoy must receive its own local environment");
    const auto a = analytic(initialFirst, {10, 0, 0}, {0, 0, 1}, .25, 4);
    const auto b = analytic(initialSecond, {0, 20, 0}, {0, -1, 0}, .25, 4);
    require(nearPoint(paths[0].activeEnd.state.position, a.position) && nearPoint(paths[1].activeEnd.state.position, b.position),
            "different local down and air velocity must affect only the corresponding decoy");
    require(samePoint(paths[1].activeEnd.state.position, independent[0].activeEnd.state.position)
                && samePoint(paths[1].activeEnd.state.velocity, independent[0].activeEnd.state.velocity),
            "another decoy's presence must not change a decoy's trajectory");
    cm::Dispenser still(7, 1, 0), movingAir(7, 1, 0);
    require(still.dispense(cm::Kind::Chaff, 0, {}, {10, 0, 0}, 1)
                && movingAir.dispense(cm::Kind::Chaff, 0, {}, {60, 0, 0}, 1), "frame-shift releases must succeed");
    const auto stillPath = still.advance(.5, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}}; });
    const auto airPath = movingAir.advance(.5, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {50, 0, 0}}; });
    require(std::fabs(airPath[0].activeEnd.state.position.x - stillPath[0].activeEnd.state.position.x - 25) < 1e-11
                && std::fabs(airPath[0].activeEnd.state.velocity.x - stillPath[0].activeEnd.state.velocity.x - 50) < 1e-11,
            "drag must depend on air-relative velocity, preserving a uniform translating-air frame");
}

void analyticSamplingIsNonConsuming() {
    cm::Dispenser dispenser(7, 1, 0);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {1, 2, 3}, {100, 20, 0}, 1), "release must succeed");
    const auto paths = dispenser.advance(1, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {5, 0, 0}}; });
    const auto stored = dispenser.decoys().front();
    cm::Sample first, again;
    require(cm::sampleAt(paths[0], .25, first) && cm::sampleAt(paths[0], .25, again), "interior analytic samples must be available");
    const auto expected = analytic(paths[0].before.state, {5, 0, 0}, {0, 0, 1}, .25, 4);
    require(first.ageS == .25 && samePoint(first.state.position, again.state.position)
                && nearPoint(first.state.position, expected.position) && nearPoint(first.state.velocity, expected.velocity),
            "repeated interior samples must use the analytic trajectory rather than an endpoint chord");
    require(samePoint(dispenser.decoys().front().position, stored.position)
                && samePoint(dispenser.decoys().front().velocity, stored.velocity),
            "sampling an owned trajectory must not consume or mutate live state");
    const auto saved = first;
    require(!cm::sampleAt(paths[0], -.01, first) && !cm::sampleAt(paths[0], std::numeric_limits<double>::quiet_NaN(), first)
                && !cm::sampleAt(paths[0], 1.01, first) && samePoint(first.state.position, saved.state.position),
            "invalid sample times must not extrapolate or mutate output");
}

void invalidEnvironmentIsAtomic() {
    cm::Dispenser dispenser(7, 2, 0);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {1, 2, 3}, {100, 20, 0}, 1)
                && dispenser.dispense(cm::Kind::Chaff, .35, {4, 5, 6}, {20, 10, 0}, 2), "releases must succeed");
    const auto a = dispenser.decoys()[0], b = dispenser.decoys()[1];
    const auto paths = dispenser.advance(.5, [](const cm::Decoy& decoy) {
        return decoy.id == 1 ? cm::Motion{{0, 0, 1}, {}} : cm::Motion{{}, {}};
    });
    require(paths.empty() && dispenser.decoys().size() == 2
                && samePoint(dispenser.decoys()[0].position, a.position) && samePoint(dispenser.decoys()[1].position, b.position),
            "one invalid environment must reject the whole step before any decoy advances");
    require(dispenser.advance(.5, {}).empty(), "a missing environment callback must reject atomically");
    dispenser.step(5.99, {0, 0, 1});
    require(dispenser.decoys().size() == 2, "rejected environment must not age either decoy");
    dispenser.step(.01, {0, 0, 1});
    require(dispenser.decoys().empty(), "valid legacy wrapper steps must still use compensated lifetime accounting");
}

void overflowIsExplicitAndDoesNotPublishOldMotion() {
    cm::Dispenser dispenser(7, 1, 0);
    const double largest = std::numeric_limits<double>::max();
    require(dispenser.dispense(cm::Kind::Chaff, 0, {largest, 0, 0}, {largest, 0, 0}, 1), "finite extreme release must succeed");
    const auto paths = dispenser.advance(1, [](const cm::Decoy&) { return cm::Motion{{1, 0, 0}, {}}; });
    require(paths.size() == 1 && !paths[0].valid && dispenser.decoys().empty(),
            "arithmetic propagation failure must retain an invalid trajectory record and remove its invalid live pose");
    require(finite(paths[0].before.state.position) && finite(paths[0].before.state.velocity),
            "invalid trajectory diagnostics must retain the actual finite input sample");
    cm::Sample output;
    output.ageS = 123;
    require(!cm::sampleAt(paths[0], 0, output) && output.ageS == 123,
            "invalid trajectories cannot be sampled as fresh observations even at a stored old pose");
}
} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[] = {
        {"inventory and shared cooldown", inventoryAndSharedCooldown},
        {"invalid release preserves inventory", invalidReleaseDoesNotConsume},
        {"duplicate and backward release", duplicateAndBackwardReleaseDoNotConsume},
        {"invalid step preserves state", invalidStepDoesNotAdvanceOrPoison},
        {"distinct lifetime and large step expiry", distinctLifetimeAndLargeStepExpiry},
        {"fixed step lifetime boundary", fixedStepLifetimeBoundary},
        {"motion and gravity direction", motionAndDirection},
        {"deterministic repeat and step partition", deterministicRepeatAndStepPartition},
        {"extreme finite motion fails closed", extremeFiniteStateRemainsFinite},
        {"owned before and analytic after", trajectoriesPreserveBeforeAndAnalyticAfter},
        {"partial expiry retains trajectory", partialExpiryRetainsFinalTrajectory},
        {"individual environment and wind-relative motion", individualEnvironmentAndWindRelativeMotion},
        {"non-consuming analytic samples", analyticSamplingIsNonConsuming},
        {"invalid environment is atomic", invalidEnvironmentIsAtomic},
        {"overflow invalidates live motion", overflowIsExplicitAndDoesNotPublishOldMotion},
    };
    unsigned failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n'; }
    }
    return failures ? 1 : 0;
}
