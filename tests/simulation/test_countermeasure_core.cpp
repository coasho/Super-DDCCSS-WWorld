#include "simulation/mission_systems/countermeasure/Dispenser.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace cm = fsim::ms::countermeasure;

namespace {
constexpr double kGravity = 9.80665;
constexpr double kSeaLevelRho = 1.225;

void require(bool result, const char* message) {
    if (!result) throw std::runtime_error(message);
}

bool samePoint(cm::Point a, cm::Point b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool finite(cm::Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool nearPoint(cm::Point a, cm::Point b, double tolerance = 1e-6) {
    return std::fabs(a.x - b.x) <= tolerance && std::fabs(a.y - b.y) <= tolerance && std::fabs(a.z - b.z) <= tolerance;
}

cm::Point unitDown(cm::Point d) {
    const double n = std::hypot(d.x, d.y, d.z);
    return {d.x / n, d.y / n, d.z / n};
}

// Independent fine-step RK4 reference solution of the physical countermeasure ODE
// (used to validate step integration, expiry capping, and interior sampling).
cm::Decoy referenceIntegrate(cm::Decoy initial, cm::Motion motion, double startAgeS, double seconds) {
    const cm::Point dn = unitDown(motion.down);
    const double rho = motion.airDensityKgM3;
    const auto acc = [&](double ageS, cm::Point vel) -> cm::Point {
        const double rx = vel.x - motion.airVelocity.x;
        const double ry = vel.y - motion.airVelocity.y;
        const double rz = vel.z - motion.airVelocity.z;
        if (initial.kind == cm::Kind::Flare) {
            const double frac = std::clamp(ageS / std::max(initial.lifeS, 1e-3), 0.0, 1.0);
            const double m = std::max(initial.spec.flareBurnedMassKg,
                                      initial.spec.flareMassKg + (initial.spec.flareBurnedMassKg - initial.spec.flareMassKg) * frac);
            const double spd = std::hypot(rx, ry, rz);
            const double k = 0.5 * rho * initial.spec.flareDragAreaM2 * spd / m;
            return {kGravity * dn.x - k * rx, kGravity * dn.y - k * ry, kGravity * dn.z - k * rz};
        }
        const double rhoRatio = rho / kSeaLevelRho;
        const double vSink = initial.spec.chaffTerminalSinkMs / std::sqrt(std::max(rhoRatio, 1e-3));
        const double ux = rx - vSink * dn.x;
        const double uy = ry - vSink * dn.y;
        const double uz = rz - vSink * dn.z;
        const double uMag = std::hypot(ux, uy, uz);
        const double rate = kGravity / std::max(vSink, 0.05) + rhoRatio * initial.spec.chaffDragCoeffM2Kg * uMag;
        return {-rate * ux, -rate * uy, -rate * uz};
    };

    cm::Point p = initial.position;
    cm::Point v = initial.velocity;
    const int steps = std::max(1, static_cast<int>(std::ceil(seconds / 2e-4)));
    const double h = seconds / static_cast<double>(steps);
    double t = startAgeS;
    for (int i = 0; i < steps; ++i) {
        const cm::Point a1 = acc(t, v);
        const cm::Point v2{v.x + 0.5 * h * a1.x, v.y + 0.5 * h * a1.y, v.z + 0.5 * h * a1.z};
        const cm::Point a2 = acc(t + 0.5 * h, v2);
        const cm::Point v3{v.x + 0.5 * h * a2.x, v.y + 0.5 * h * a2.y, v.z + 0.5 * h * a2.z};
        const cm::Point a3 = acc(t + 0.5 * h, v3);
        const cm::Point v4{v.x + h * a3.x, v.y + h * a3.y, v.z + h * a3.z};
        const cm::Point a4 = acc(t + h, v4);
        const double h6 = h / 6.0;
        p.x += h6 * (v.x + 2.0 * v2.x + 2.0 * v3.x + v4.x);
        p.y += h6 * (v.y + 2.0 * v2.y + 2.0 * v3.y + v4.y);
        p.z += h6 * (v.z + 2.0 * v2.z + 2.0 * v3.z + v4.z);
        v.x += h6 * (a1.x + 2.0 * a2.x + 2.0 * a3.x + a4.x);
        v.y += h6 * (a1.y + 2.0 * a2.y + 2.0 * a3.y + a4.y);
        v.z += h6 * (a1.z + 2.0 * a2.z + 2.0 * a3.z + a4.z);
        t += h;
    }
    cm::Decoy out = initial;
    out.position = p;
    out.velocity = v;
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
    require(std::fabs(first.lifeS - dispenser.spec().chaffLifeS) < 1e-12,
            "decoy must carry calibrated cartridge lifetime");
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
    require(dispenser.advance(1.0, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, 0.0}; }).empty(),
            "zero air density must reject");
    require(dispenser.advance(1.0, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, -1.0}; }).empty(),
            "negative air density must reject");
    require(dispenser.advance(1.0, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, 25.0}; }).empty(),
            "out-of-range air density must reject");
    require(dispenser.advance(1.0, [nan](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, nan}; }).empty(),
            "NaN air density must reject");
    const auto& after = dispenser.decoys().front();
    require(samePoint(before.position, after.position) && samePoint(before.velocity, after.velocity),
            "invalid steps must leave kinematics unchanged");
    dispenser.step(27.99, {0, 0, 1});
    require(dispenser.decoys().size() == 1, "invalid steps must not age decoys");
    dispenser.step(0.01, {0, 0, 1});
    require(dispenser.decoys().empty(), "expiry must use elapsed simulation steps");
}

void distinctLifetimeAndLargeStepExpiry() {
    cm::Dispenser dispenser(7, 1, 1);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {}, {}, 1), "chaff release must succeed");
    require(dispenser.dispense(cm::Kind::Flare, 0.35, {}, {}, 2), "flare release must succeed");
    dispenser.step(4.19, {0, 0, 1});
    require(dispenser.decoys().size() == 2, "both kinds must survive before MJU-7/B flare expiry");
    dispenser.step(0.01, {0, 0, 1});
    require(dispenser.decoys().size() == 1 && dispenser.decoys().front().kind == cm::Kind::Chaff,
            "flare must expire independently before chaff cloud");
    dispenser.step(std::numeric_limits<double>::max(), {0, 0, 1});
    require(dispenser.decoys().empty(), "large finite step must safely expire surviving decoys");
    require(dispenser.remaining(cm::Kind::Chaff) == 0 && dispenser.remaining(cm::Kind::Flare) == 0,
            "expiry must not replenish inventory");
}

void fixedStepLifetimeBoundary() {
    cm::Dispenser chaff(7, 1, 0), flare(7, 0, 1);
    require(chaff.dispense(cm::Kind::Chaff, 0, {}, {}, 1), "chaff release must succeed");
    require(flare.dispense(cm::Kind::Flare, 0, {}, {}, 2), "flare release must succeed");
    // MJU-7/B flare: 4.20 s == 84 steps of 0.05 s
    for (int i = 0; i < 83; ++i) flare.step(0.05, {0, 0, 1});
    require(flare.decoys().size() == 1, "flare must survive the step before its 4.2 s lifetime boundary");
    flare.step(0.05, {0, 0, 1});
    require(flare.decoys().empty(), "flare must expire at 4.2 s of fixed simulation steps");
    // RR-170A/AL chaff: 28.00 s == 560 steps of 0.05 s
    for (int i = 0; i < 559; ++i) chaff.step(0.05, {0, 0, 1});
    require(chaff.decoys().size() == 1, "chaff must survive the step before its 28.0 s lifetime boundary");
    chaff.step(0.05, {0, 0, 1});
    require(chaff.decoys().empty(), "chaff must expire at 28.0 s of fixed simulation steps");
}

void motionAndDirection() {
    cm::Dispenser dispenser(7, 1, 1);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {}, {240, 0, 0}, 1), "chaff release must succeed");
    require(dispenser.dispense(cm::Kind::Flare, 0.35, {}, {240, 0, 0}, 2), "flare release must succeed");
    dispenser.step(1.0, {0, -2, 0});
    const auto& chaff = dispenser.decoys()[0];
    const auto& flare = dispenser.decoys()[1];
    require(chaff.velocity.x < 1.0 && flare.velocity.x > 40.0 && flare.velocity.x < 150.0,
            "chaff dipoles must arrest to near-zero horizontal speed while flare retains realistic ballistic momentum");
    require(chaff.position.x > 0 && chaff.position.x < 30.0 && flare.position.x > chaff.position.x + 50.0,
            "quadratic drag must separate chaff quickly behind the aircraft while flare trails smoothly");
    require(chaff.velocity.y < 0 && std::fabs(chaff.velocity.y + 1.2) < 0.05,
            "chaff must settle near sea-level terminal sink speed (~1.2 m/s) along supplied down");
    require(flare.velocity.y < chaff.velocity.y && chaff.position.y < 0 && flare.position.y < chaff.position.y,
            "denser flare must fall faster than chaff dipole cloud along supplied down direction");
    require(chaff.position.z == 0 && flare.position.z == 0, "gravity must not assume a fixed world axis");
}

void deterministicRepeatAndStepPartition() {
    for (const cm::Kind kind : {cm::Kind::Chaff, cm::Kind::Flare}) {
        cm::Dispenser one(7, 1, 1), repeat(7, 1, 1), partition(7, 1, 1);
        for (auto* dispenser : {&one, &repeat, &partition})
            require(dispenser->dispense(kind, 0, {1, 2, 3}, {220, 20, -5}, 1), "release must succeed");
        one.step(1.0, {0, 0, 1});
        repeat.step(1.0, {0, 0, 1});
        for (int i = 0; i < 10; ++i) partition.step(0.1, {0, 0, 1});
        const auto& a = one.decoys().front();
        const auto& b = repeat.decoys().front();
        const auto& c = partition.decoys().front();
        require(samePoint(a.position, b.position) && samePoint(a.velocity, b.velocity), "same inputs must repeat exactly");
        require(nearPoint(a.position, c.position, 1e-6) && nearPoint(a.velocity, c.velocity, 1e-6),
                "RK4 sub-stepped physical motion must remain consistent across simulation step partitions");
    }
}

void extremeFiniteStateRemainsFinite() {
    cm::Dispenser dispenser(7, 1, 0);
    const double largest = std::numeric_limits<double>::max();
    require(dispenser.dispense(cm::Kind::Chaff, 0, {largest, largest, largest}, {largest, largest, largest}, 1),
            "finite launch state must be accepted");
    dispenser.step(1, {largest, largest, largest});
    require(dispenser.decoys().empty(), "arithmetic failure must remove the invalid live pose rather than freeze it as fresh motion");
}

void trajectoriesPreserveBeforeAndPhysicalAfter() {
    cm::Dispenser dispenser(7, 1, 1);
    require(dispenser.dispense(cm::Kind::Flare, 1000, {1, 2, 3}, {230, 20, -5}, 1), "flare release must succeed");
    const auto initial = dispenser.decoys().front();
    const cm::Motion env{{0, 0, -2}, {10, -3, 0}, 0.78};
    const auto paths = dispenser.advance(0.5, [env](const cm::Decoy&) { return env; });
    require(paths.size() == 1 && paths[0].valid && !paths[0].expires, "a live decoy must produce one valid owned trajectory");
    const auto& path = paths.front();
    require(path.before.ageS == 0 && samePoint(path.before.state.position, initial.position)
                && samePoint(path.before.state.velocity, initial.velocity),
            "trajectory must retain actual pre-step kinematics rather than reconstructing them from final velocity");
    require(path.stepDurationS == 0.5 && path.liveDurationS == 0.5 && path.activeEnd.ageS == 0.5,
            "trajectory must retain the requested step and actual active duration");
    const auto expected = referenceIntegrate(initial, env, 0.0, 0.5);
    require(nearPoint(path.activeEnd.state.position, expected.position, 1e-6)
                && nearPoint(path.activeEnd.state.velocity, expected.velocity, 1e-6),
            "trajectory endpoint must match physical quadratic-drag + mass-burn ODE solution");
    require(samePoint(dispenser.decoys().front().position, path.activeEnd.state.position),
            "live storage must publish the actual trajectory endpoint");
    cm::Sample atStart, atEnd;
    require(cm::sampleAt(path, 0, atStart) && cm::sampleAt(path, 0.5, atEnd), "both geometric interval endpoints must be sampleable");
    require(samePoint(atStart.state.position, initial.position) && samePoint(atEnd.state.position, path.activeEnd.state.position),
            "endpoint samples must preserve stored exact endpoint states");
}

void partialExpiryRetainsFinalTrajectory() {
    cm::Dispenser dispenser(7, 0, 1);
    require(dispenser.dispense(cm::Kind::Flare, 50, {2, 3, 4}, {180, 0, 5}, 1), "flare release must succeed");
    const auto initial = dispenser.decoys().front();
    const cm::Motion env{{0, 0, -1}, {}, 0.90};
    const auto environment = [env](const cm::Decoy&) { return env; };
    dispenser.advance(4.1, environment);
    const auto paths = dispenser.advance(0.2, environment);
    require(paths.size() == 1 && paths[0].valid && paths[0].expires && dispenser.decoys().empty(),
            "an expiring decoy must retain its final partial trajectory after leaving live storage");
    const auto& path = paths.front();
    require(std::fabs(path.before.ageS - 4.1) < 1e-12 && std::fabs(path.activeEnd.ageS - 4.2) < 1e-12
                && std::fabs(path.liveDurationS - 0.1) < 1e-12 && path.stepDurationS == 0.2,
            "expiry must cap motion at its calibrated 4.2 s lifetime rather than dropping the whole interval");
    const auto expected = referenceIntegrate(initial, env, 0.0, 4.2);
    require(nearPoint(path.activeEnd.state.position, expected.position, 1e-5)
                && nearPoint(path.activeEnd.state.velocity, expected.velocity, 1e-5),
            "expiry endpoint must match the physical ODE state at the lifetime boundary");
    cm::Sample boundary;
    require(cm::sampleAt(path, path.liveDurationS, boundary) && std::fabs(boundary.ageS - 4.2) < 1e-12,
            "exclusive expiry must still expose its mathematical endpoint for clipped geometry");
    const auto unchanged = boundary;
    require(!cm::sampleAt(path, std::nextafter(path.liveDurationS, std::numeric_limits<double>::infinity()), boundary)
                && boundary.ageS == unchanged.ageS && samePoint(boundary.state.position, unchanged.state.position),
            "sampling beyond expiry must fail without extrapolating or changing the output");
}

void individualEnvironmentAndWindRelativeMotion() {
    cm::Dispenser together(7, 2, 0), alone(7, 1, 0);
    require(together.dispense(cm::Kind::Chaff, 0, {}, {180, 0, 0}, 1), "first release must succeed");
    require(together.dispense(cm::Kind::Chaff, 0.35, {5, 6, 7}, {0, 120, 0}, 2), "second release must succeed");
    require(alone.dispense(cm::Kind::Chaff, 0.35, {5, 6, 7}, {0, 120, 0}, 2), "independent release must succeed");
    const auto initialFirst = together.decoys()[0], initialSecond = together.decoys()[1];
    const cm::Motion envA{{0, 0, 1}, {10, 0, 0}, 1.225};
    const cm::Motion envB{{0, -1, 0}, {0, 20, 0}, 0.55};
    const auto environment = [envA, envB](const cm::Decoy& decoy) {
        return decoy.id == 1 ? envA : envB;
    };
    const auto paths = together.advance(0.25, environment), independent = alone.advance(0.25, environment);
    require(paths.size() == 2 && independent.size() == 1 && paths[0].valid && paths[1].valid,
            "each decoy must receive its own local environment");
    const auto a = referenceIntegrate(initialFirst, envA, 0.0, 0.25);
    const auto b = referenceIntegrate(initialSecond, envB, 0.0, 0.25);
    require(nearPoint(paths[0].activeEnd.state.position, a.position, 1e-6)
                && nearPoint(paths[1].activeEnd.state.position, b.position, 1e-6),
            "different local down, air velocity and density must affect only the corresponding decoy");
    require(samePoint(paths[1].activeEnd.state.position, independent[0].activeEnd.state.position)
                && samePoint(paths[1].activeEnd.state.velocity, independent[0].activeEnd.state.velocity),
            "another decoy's presence must not change a decoy's trajectory");

    // Galilean invariance under uniform translating wind for both Chaff and Flare.
    for (const cm::Kind kind : {cm::Kind::Chaff, cm::Kind::Flare}) {
        cm::Dispenser still(7, 1, 1), movingAir(7, 1, 1);
        require(still.dispense(kind, 0, {}, {140, 0, 0}, 1)
                    && movingAir.dispense(kind, 0, {}, {190, 0, 0}, 1),
                "frame-shift releases must succeed");
        const auto stillPath = still.advance(0.5, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, 0.85}; });
        const auto airPath = movingAir.advance(0.5, [](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {50, 0, 0}, 0.85}; });
        require(std::fabs(airPath[0].activeEnd.state.position.x - stillPath[0].activeEnd.state.position.x - 25.0) < 1e-9
                    && std::fabs(airPath[0].activeEnd.state.velocity.x - stillPath[0].activeEnd.state.velocity.x - 50.0) < 1e-9,
                "aerodynamic drag must depend strictly on air-relative velocity, preserving Galilean wind invariance");
    }
}

void physicalSamplingIsNonConsuming() {
    cm::Dispenser dispenser(7, 0, 1);
    require(dispenser.dispense(cm::Kind::Flare, 0, {1, 2, 3}, {200, 20, 0}, 1), "release must succeed");
    const cm::Motion env{{0, 0, 1}, {5, 0, 0}, 0.92};
    const auto paths = dispenser.advance(1.0, [env](const cm::Decoy&) { return env; });
    const auto stored = dispenser.decoys().front();
    cm::Sample first, again;
    require(cm::sampleAt(paths[0], 0.25, first) && cm::sampleAt(paths[0], 0.25, again),
            "interior physical samples must be available");
    const auto expected = referenceIntegrate(paths[0].before.state, env, 0.0, 0.25);
    require(first.ageS == 0.25 && samePoint(first.state.position, again.state.position)
                && nearPoint(first.state.position, expected.position, 1e-6)
                && nearPoint(first.state.velocity, expected.velocity, 1e-6),
            "repeated interior samples must follow the physical trajectory rather than an endpoint chord");
    require(samePoint(dispenser.decoys().front().position, stored.position)
                && samePoint(dispenser.decoys().front().velocity, stored.velocity),
            "sampling an owned trajectory must not consume or mutate live state");
    const auto saved = first;
    require(!cm::sampleAt(paths[0], -0.01, first) && !cm::sampleAt(paths[0], std::numeric_limits<double>::quiet_NaN(), first)
                && !cm::sampleAt(paths[0], 1.01, first) && samePoint(first.state.position, saved.state.position),
            "invalid sample times must not extrapolate or mutate output");
}

void invalidEnvironmentIsAtomic() {
    cm::Dispenser dispenser(7, 2, 0);
    require(dispenser.dispense(cm::Kind::Chaff, 0, {1, 2, 3}, {100, 20, 0}, 1)
                && dispenser.dispense(cm::Kind::Chaff, 0.35, {4, 5, 6}, {20, 10, 0}, 2),
            "releases must succeed");
    const auto a = dispenser.decoys()[0], b = dispenser.decoys()[1];
    const auto paths = dispenser.advance(0.5, [](const cm::Decoy& decoy) {
        return decoy.id == 1 ? cm::Motion{{0, 0, 1}, {}} : cm::Motion{{}, {}};
    });
    require(paths.empty() && dispenser.decoys().size() == 2
                && samePoint(dispenser.decoys()[0].position, a.position)
                && samePoint(dispenser.decoys()[1].position, b.position),
            "one invalid environment must reject the whole step before any decoy advances");
    const auto badDensity = dispenser.advance(0.5, [](const cm::Decoy& decoy) {
        return decoy.id == 1 ? cm::Motion{{0, 0, 1}, {}, 1.0} : cm::Motion{{0, 0, 1}, {}, 0.0};
    });
    require(badDensity.empty() && samePoint(dispenser.decoys()[0].position, a.position),
            "one invalid air density must reject the whole step atomically");
    require(dispenser.advance(0.5, {}).empty(), "a missing environment callback must reject atomically");
    dispenser.step(27.99, {0, 0, 1});
    require(dispenser.decoys().size() == 2, "rejected environment must not age either decoy");
    dispenser.step(0.01, {0, 0, 1});
    require(dispenser.decoys().empty(), "valid wrapper steps must preserve compensated lifetime accounting");
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

void altitudeAirDensityAndMassBurnPhysics() {
    // Standard ISA atmosphere density profile checkpoints.
    const double rho0 = cm::isaAirDensityKgM3(0.0);
    const double rho11k = cm::isaAirDensityKgM3(11000.0);
    const double rho20k = cm::isaAirDensityKgM3(20000.0);
    require(std::fabs(rho0 - 1.225) < 0.002, "ISA sea-level air density must match ~1.225 kg/m^3");
    require(std::fabs(rho11k - 0.3639) < 0.003, "ISA tropopause (11 km) density must match ~0.364 kg/m^3");
    require(std::fabs(rho20k - 0.0880) < 0.003, "ISA stratosphere (20 km) density must match ~0.088 kg/m^3");

    // 1. High-altitude thin air reduces flare aerodynamic drag compared to sea level.
    cm::Dispenser seaLevel(1, 0, 1), highAlt(1, 0, 1);
    require(seaLevel.dispense(cm::Kind::Flare, 0.0, {}, {250.0, 0.0, 0.0}, 1), "sea-level flare must release");
    require(highAlt.dispense(cm::Kind::Flare, 0.0, {}, {250.0, 0.0, 0.0}, 2), "high-alt flare must release");
    seaLevel.advance(1.5, [rho0](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, rho0}; });
    highAlt.advance(1.5, [rho11k](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {}, rho11k}; });
    require(highAlt.decoys().front().velocity.x > seaLevel.decoys().front().velocity.x + 50.0,
            "flare at 11 km thin air must retain substantially more forward speed than at sea level");

    // 2. Pyrotechnic mass burn depletion increases aerodynamic deceleration toward burnout.
    cm::Spec constMassSpec;
    constMassSpec.flareBurnedMassKg = constMassSpec.flareMassKg;
    cm::Dispenser burning(2, 0, 1), constantMass(2, 0, 1, constMassSpec);
    require(burning.dispense(cm::Kind::Flare, 0.0, {}, {250.0, 0.0, 0.0}, 1), "burning flare must release");
    require(constantMass.dispense(cm::Kind::Flare, 0.0, {}, {250.0, 0.0, 0.0}, 2), "constant-mass flare must release");
    burning.step(3.5, {0, 0, 1});
    constantMass.step(3.5, {0, 0, 1});
    require(burning.decoys().front().velocity.x < constantMass.decoys().front().velocity.x - 5.0,
            "burning flare losing propellant mass must decelerate faster than constant-mass grain");

    // 3. Chaff dipole cloud captures ambient wind quickly and sinks faster in thinner upper-air.
    cm::Dispenser chaffLow(3, 1, 0), chaffHigh(3, 1, 0);
    require(chaffLow.dispense(cm::Kind::Chaff, 0.0, {}, {260.0, 0.0, 0.0}, 1), "low chaff must release");
    require(chaffHigh.dispense(cm::Kind::Chaff, 0.0, {}, {260.0, 0.0, 0.0}, 2), "high chaff must release");
    chaffLow.advance(1.0, [rho0](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {15.0, -5.0, 0.0}, rho0}; });
    chaffHigh.advance(1.0, [rho11k](const cm::Decoy&) { return cm::Motion{{0, 0, 1}, {15.0, -5.0, 0.0}, rho11k}; });
    const auto& cl = chaffLow.decoys().front();
    const auto& ch = chaffHigh.decoys().front();
    require(std::fabs(cl.velocity.x - 15.0) < 0.5 && std::fabs(cl.velocity.y - (-5.0)) < 0.5,
            "chaff cloud must converge to ambient horizontal wind within 1 s");
    require(ch.velocity.z > cl.velocity.z + 0.5,
            "chaff terminal settling speed must scale inversely with sqrt(air density) at high altitude");
}

void radiometryAndRcsProfiles() {
    cm::Dispenser dispenser(9, 1, 1);
    require(dispenser.dispense(cm::Kind::Flare, 0.0, {}, {}, 1), "flare release must succeed");
    require(dispenser.dispense(cm::Kind::Chaff, 0.35, {}, {}, 2), "chaff release must succeed");
    const auto& flare = dispenser.decoys()[0];
    const auto& chaff = dispenser.decoys()[1];

    // 3-phase MTV flare burn curve: ignition -> plateau -> burnout -> zero.
    const double i0 = cm::flareIntensityWSr(flare, 0.0);
    const double iRise = cm::flareIntensityWSr(flare, 0.15);
    const double iPeak = cm::flareIntensityWSr(flare, 0.60);
    const double iTail = cm::flareIntensityWSr(flare, 3.80);
    const double iOut = cm::flareIntensityWSr(flare, 4.20);
    require(i0 == 0.0 && iRise > 3000.0 && iPeak > 5500.0 && iPeak <= 6500.0 && iTail > 50.0 && iTail < 1500.0 && iOut == 0.0,
            "MJU-7/B flare intensity must follow 3-phase MTV ignition, plateau and burnout profile");

    const double tPeak = cm::flareTemperatureK(flare, 0.60, 260.0);
    const double tOut = cm::flareTemperatureK(flare, 4.20, 260.0);
    require(tPeak > 2050.0 && tPeak <= 2150.0 && tOut == 260.0,
            "flare temperature must peak near 2150 K and return to ambient at burnout");

    const double rPeak = cm::flareCoreRadiusM(flare, 0.60);
    require(rPeak > 0.12 && rPeak <= 0.19, "flare core flame envelope radius must stay physically bounded (~0.18 m)");

    // Chaff dipole cloud wake bloom & frequency-selective monostatic RCS.
    const double bloom0 = cm::chaffBloomRadiusM(chaff, 0.0);
    const double bloom5 = cm::chaffBloomRadiusM(chaff, 5.0);
    require(std::fabs(bloom0 - 0.6) < 1e-9 && bloom5 > 12.0 && bloom5 < 20.0,
            "chaff cloud radius must bloom from ~0.6 m canister burst to >12 m wake cloud");

    const double rcsEarly = cm::chaffRcsM2(chaff, 0.05, 10.0e9);
    const double rcsX = cm::chaffRcsM2(chaff, 2.0, 10.0e9);
    const double rcsVhf = cm::chaffRcsM2(chaff, 2.0, 150.0e6);
    const double rcsLate = cm::chaffRcsM2(chaff, 20.0, 10.0e9);
    const double rcsExpired = cm::chaffRcsM2(chaff, 28.0, 10.0e9);
    require(rcsEarly > 0.5 && rcsEarly < rcsX && rcsX > 15.0 && rcsX <= 18.0,
            "X-band chaff RCS must de-clump rapidly toward calibrated ~18 m^2 peak");
    require(rcsVhf < rcsX * 0.35, "X-band dipole pack must exhibit sharply reduced RCS off-band at VHF");
    require(rcsLate > 2.0 && rcsLate < rcsX && rcsExpired == 0.0,
            "chaff RCS must persist across wind-shear dispersion and reach zero at expiry");

    // Custom Spec (e.g., B-52H bulk chaff bundle) must govern emitted decoy properties.
    cm::Spec heavy;
    heavy.chaffLifeS = 36.0;
    heavy.chaffPeakRcsM2 = 45.0;
    heavy.flareLifeS = 5.0;
    heavy.flarePeakIntensityWSr = 9500.0;
    cm::Dispenser heavyDisp(10, 1, 1, heavy);
    require(heavyDisp.dispense(cm::Kind::Chaff, 0.0, {}, {}, 10), "custom chaff release must succeed");
    require(heavyDisp.dispense(cm::Kind::Flare, 0.35, {}, {}, 11), "custom flare release must succeed");
    require(cm::chaffRcsM2(heavyDisp.decoys()[0], 2.0, 10.0e9) > 38.0, "custom chaff spec must control peak RCS");
    require(cm::flareIntensityWSr(heavyDisp.decoys()[1], 0.8) > 8000.0, "custom flare spec must control peak intensity");
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
        {"owned before and physical RK4 after", trajectoriesPreserveBeforeAndPhysicalAfter},
        {"partial expiry retains trajectory", partialExpiryRetainsFinalTrajectory},
        {"individual environment and wind-relative motion", individualEnvironmentAndWindRelativeMotion},
        {"non-consuming physical samples", physicalSamplingIsNonConsuming},
        {"invalid environment is atomic", invalidEnvironmentIsAtomic},
        {"overflow invalidates live motion", overflowIsExplicitAndDoesNotPublishOldMotion},
        {"altitude air density and mass burn physics", altitudeAirDensityAndMassBurnPhysics},
        {"radiometry and RCS profiles", radiometryAndRcsProfiles},
    };
    unsigned failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n'; }
    }
    return failures ? 1 : 0;
}
