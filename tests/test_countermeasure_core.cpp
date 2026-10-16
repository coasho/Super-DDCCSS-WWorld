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
    require(dispenser.decoys().size() == 1, "finite large direction must normalize safely");
    require(finite(dispenser.decoys().front().position) && finite(dispenser.decoys().front().velocity),
            "finite inputs must not poison simulation state on arithmetic overflow");
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
        {"extreme finite state remains finite", extremeFiniteStateRemainsFinite},
    };
    unsigned failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n'; }
    }
    return failures ? 1 : 0;
}
