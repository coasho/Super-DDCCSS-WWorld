// Integration tests against the tracked vendored JSBSim data tree. They are
// the M1 exit criteria in miniature: vehicles load, fly, and produce
// bit-identical trajectories regardless of how many workers step them.

#include "foundation/core/Log.h"
#include "foundation/core/Rng.h"
#include "foundation/core/StableArray.h"
#include "simulation/flight/Attitude.h"
#include "simulation/flight/GroundProvider.h"
#include "simulation/flight/JsbsimModel.h"
#include "simulation/flight/VehiclePool.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <vector>

using namespace fsim;
using namespace fsim::sim;

namespace {

const std::filesystem::path kRoot = FSIM_TEST_JSBSIM_ROOT;
constexpr double kDt = 1.0 / 120.0;

InitialConditions icFor(unsigned i) {
    Rng rng = Rng::forVehicle(99, 0, i);
    InitialConditions ic;
    ic.latitudeDeg += rng.uniform(-0.1, 0.1);
    ic.longitudeDeg += rng.uniform(-0.1, 0.1);
    ic.altitudeMslM = 1200.0 + rng.uniform(-100.0, 100.0);
    ic.headingDeg = rng.uniform(0.0, 360.0);
    ic.airspeedTrueMs = 55.0;
    return ic;
}

std::vector<VehicleState> fly(unsigned vehicles, unsigned workers, unsigned agentSteps, int frameSkip) {
    auto ground = std::make_shared<FlatGround>(0.0);
    VehiclePool pool(workers);
    for (unsigned i = 0; i < vehicles; ++i) {
        auto m = std::make_unique<JsbsimModel>(kDt, ground);
        REQUIRE(m->load(AircraftSpec{"c172x", kRoot}, icFor(i)));
        pool.add(std::move(m));
    }
    std::vector<ControlInputs> inputs(vehicles);
    for (unsigned i = 0; i < vehicles; ++i) {
        inputs[i].setThrottleAll(0.6);
        inputs[i].gearDown = 0.0;
        inputs[i].aileron = (i % 2 ? 0.05 : -0.05);
    }
    for (unsigned s = 0; s < agentSteps; ++s) pool.step(Span<const ControlInputs>(inputs), frameSkip);
    const auto states = pool.states();
    return std::vector<VehicleState>(states.begin(), states.end());
}

bool bitIdentical(const VehicleState& a, const VehicleState& b) {
    return std::memcmp(&a, &b, sizeof(VehicleState)) == 0;
}

} // namespace

TEST_CASE("c172x loads and flies with sane state", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 1000.0;
    ic.airspeedTrueMs = 55.0;
    ic.headingDeg = 90.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    REQUIRE(model.loaded());
    CHECK(model.dt() == kDt);

    VehicleState s;
    model.state(s);
    CHECK(std::abs(s.altitudeMslM - 1000.0) < 1.0);
    CHECK(std::abs(s.airspeedTrueMs - 55.0) < 0.5);
    CHECK(s.engineCount == 1);
    CHECK_FALSE(s.diverged);
    CHECK_FALSE(s.onGround);
    // ECEF radius ~ Earth radius + altitude
    const double r = std::sqrt(s.positionEcef[0] * s.positionEcef[0] + s.positionEcef[1] * s.positionEcef[1] +
                               s.positionEcef[2] * s.positionEcef[2]);
    CHECK(r > 6.35e6);
    CHECK(r < 6.40e6);
    // unit quaternion
    double n = 0;
    for (double q : s.attitudeEcefToBody) n += q * q;
    CHECK(std::abs(n - 1.0) < 1e-9);

    ControlInputs in;
    in.setThrottleAll(0.7);
    in.gearDown = 0.0;
    for (int i = 0; i < 120; ++i) model.step(in); // 1 s
    model.state(s);
    CHECK(std::abs(s.simTime - 1.0) < 1e-9);
    CHECK(s.stepCount == 120);
    CHECK_FALSE(s.diverged);
    CHECK(s.airspeedTrueMs > 40.0);
    CHECK(s.altitudeMslM > 900.0);
    // the propeller turns at the engine's rpm; a piston has no turbine's state
    CHECK(s.engineRpm[0] > 1500.0);
    CHECK(s.engineRpm[0] < 3000.0);
    CHECK(s.engineN2[0] == 0.0);
    CHECK(s.afterburner[0] == 0.0);
    CHECK(s.leadingEdgeFlapRad == 0.0);
}

TEST_CASE("a fighter reports its engine and its leading-edge flaps", "[sim][jsbsim]") {
    // what user code reads and the viewer's models show: the core's speed, the
    // nozzle, the afterburner, and the flaps as the aircraft's own flight
    // controls move them (the stock F-16's fcs/lef-pos-deg)
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 3000.0;
    ic.airspeedTrueMs = 120.0;
    REQUIRE(model.load(AircraftSpec{"f16", kRoot}, ic));
    ControlInputs in;
    in.gearDown = 0.0;
    in.setThrottleAll(0.3);
    for (int i = 0; i < 5 * 120; ++i) model.step(in);
    VehicleState s;
    model.state(s);
    CHECK(s.engineCount == 1);
    CHECK(s.engineRpm[0] == 0.0); // a jet: no propeller
    CHECK(s.engineN2[0] > 60.0);
    CHECK(s.engineN2[0] < 95.0);
    CHECK(s.nozzlePosition[0] > 0.1); // part power: the nozzle part open
    CHECK(s.afterburner[0] == 0.0);
    const double cruiseLef = s.leadingEdgeFlapRad;
    // full throttle and a pull: the core spooling up with no reheat until it is at
    // its rating (cmake/JsbsimPatches.cmake), then the reheat lighting as its fuel
    // arrives, the nozzle opening with it; the flaps down with the angle of attack
    in.setThrottleAll(1.0);
    in.elevator = -0.6;
    double litBelowRating = 0.0;
    for (int i = 0; i < 5 * 120; ++i) {
        model.step(in);
        model.state(s);
        if (s.engineN2[0] < 97.0) litBelowRating = std::max(litBelowRating, s.afterburner[0]);
    }
    CHECK(litBelowRating == 0.0);
    CHECK(s.engineN2[0] > 95.0);
    const double litAt5 = s.afterburner[0];
    CHECK(litAt5 > 0.0);
    CHECK(litAt5 < 1.0); // (building: not all at once)
    for (int i = 0; i < 10 * 120; ++i) model.step(in);
    model.state(s);
    CHECK(s.afterburner[0] > litAt5);
    CHECK(s.afterburner[0] <= 1.0);
    CHECK(s.nozzlePosition[0] > 0.5);
    CHECK(s.alphaRad > 0.1);
    CHECK(s.leadingEdgeFlapRad > cruiseLef + 0.1);
}

TEST_CASE("property handles read live JSBSim values", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, InitialConditions{}));

    auto vc = model.property("velocities/vc-kts");
    REQUIRE(vc.valid());
    CHECK(vc.get() > 50.0);

    auto bogus = model.property("no/such/property");
    CHECK_FALSE(bogus.valid());
    CHECK(bogus.get() == 0.0);
}

TEST_CASE("reset returns the vehicle to initial conditions", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    VehicleState first;
    model.state(first);

    ControlInputs in;
    in.setThrottleAll(1.0);
    in.elevator = -0.3;
    for (int i = 0; i < 600; ++i) model.step(in);
    VehicleState flown;
    model.state(flown);
    REQUIRE(std::abs(flown.altitudeMslM - first.altitudeMslM) > 5.0);

    REQUIRE(model.reset(ic));
    VehicleState after;
    model.state(after);
    CHECK(after.stepCount == 0);
    CHECK(std::abs(after.altitudeMslM - first.altitudeMslM) < 0.5);
    CHECK(std::abs(after.airspeedTrueMs - first.airspeedTrueMs) < 0.5);
    CHECK(std::abs(after.simTime) < 1e-9);
}

TEST_CASE("rocket reset clears burn history and replays a fresh synthetic model", "[sim][jsbsim][rocket-reset]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    const AircraftSpec spec{"rocket_reset", kRoot, FSIM_TEST_DATA_DIR};
    InitialConditions ic;
    ic.altitudeMslM = 1000.0;
    ic.airspeedTrueMs = 30.0;
    ic.enginesRunning = false;
    JsbsimModel used(kDt, ground), fresh(kDt, ground);
    REQUIRE(used.load(spec, ic));
    REQUIRE(fresh.load(spec, ic));

    const auto value = [](JsbsimModel& model, const std::string& path) {
        auto property = model.property(path);
        REQUIRE(property.valid());
        return property.get();
    };
    const auto compare = [&] {
        for (unsigned engine = 0; engine < 2; ++engine) {
            for (const char* suffix : {"total-impulse", "total-vac-impulse", "vacuum-thrust_lbs", "thrust-lbs"}) {
                const std::string path = "propulsion/engine[" + std::to_string(engine) + "]/" + suffix;
                INFO(path);
                CHECK(value(used, path) == value(fresh, path));
            }
        }
        for (unsigned tank = 0; tank < 3; ++tank) {
            const std::string path = "propulsion/tank[" + std::to_string(tank) + "]/contents-lbs";
            INFO(path);
            CHECK(value(used, path) == value(fresh, path));
        }
        CHECK(used.engineFuelFlowKgS(0) == fresh.engineFuelFlowKgS(0));
        CHECK(used.engineFuelFlowKgS(1) == fresh.engineFuelFlowKgS(1));
        CHECK(value(used, "propulsion/engine[1]/isp") == value(fresh, "propulsion/engine[1]/isp"));
        CHECK(value(used, "propulsion/engine[1]/isp") > 100.0);
        CHECK(value(used, "propulsion/engine[1]/mixture-ratio") == 1.0);
        VehicleState a, b;
        used.state(a);
        fresh.state(b);
        CHECK(bitIdentical(a, b));
    };

    ControlInputs on, off;
    on.setThrottleAll(1.0);
    for (int step = 0; step < 24; ++step) used.step(on);
    for (unsigned engine = 0; engine < 2; ++engine)
        REQUIRE(value(used, "propulsion/engine[" + std::to_string(engine) + "]/total-vac-impulse") > 0.0);
    REQUIRE(value(used, "propulsion/tank/contents-lbs") < 1.0);
    REQUIRE(used.reset(ic));
    compare();
    // A solid motor that burned before reset must stay off until a new ignition.
    for (int step = 0; step < 12; ++step) {
        used.step(off);
        fresh.step(off);
        compare();
    }
    for (int step = 0; step < 24; ++step) {
        const double liquidBefore = value(used, "propulsion/tank[1]/contents-lbs") +
                                    value(used, "propulsion/tank[2]/contents-lbs");
        used.step(on);
        fresh.step(on);
        const double liquidAfter = value(used, "propulsion/tank[1]/contents-lbs") +
                                   value(used, "propulsion/tank[2]/contents-lbs");
        CHECK(std::abs(used.engineFuelFlowKgS(1) - (liquidBefore - liquidAfter) * 0.45359237 / kDt) < 1e-12);
        compare();
    }
    // Include a liquid engine's spent/starved history, not only an ongoing burn.
    for (int step = 0; step < 300; ++step) used.step(on);
    REQUIRE(value(used, "propulsion/tank[1]/contents-lbs") == 0.0);
    REQUIRE(value(used, "propulsion/tank[2]/contents-lbs") == 0.0);
    REQUIRE(value(used, "propulsion/engine[1]/vacuum-thrust_lbs") == 0.0);
    REQUIRE(used.reset(ic));
    REQUIRE(fresh.reset(ic));
    compare();
    for (int step = 0; step < 24; ++step) {
        used.step(on);
        fresh.step(on);
        compare();
    }
}

TEST_CASE("rocket pulse schedule follows ignition time across coast and reset", "[sim][jsbsim][rocket-pulse]") {
    const double dt = GENERATE(1.0 / 60.0, 1.0 / 120.0);
    auto ground = std::make_shared<FlatGround>(0.0);
    const AircraftSpec spec{"rocket_pulse", kRoot, FSIM_TEST_DATA_DIR};
    InitialConditions ic;
    ic.altitudeMslM = 1000.0;
    ic.airspeedTrueMs = 30.0;
    ic.enginesRunning = false;
    JsbsimModel used(dt, ground), fresh(dt, ground);
    REQUIRE(used.load(spec, ic));
    REQUIRE(fresh.load(spec, ic));
    ControlInputs on, off;
    on.setThrottleAll(1.0);
    const auto thrust = [](JsbsimModel& model, int engine) {
        auto property = model.property("propulsion/engine[" + std::to_string(engine) + "]/vacuum-thrust_lbs");
        REQUIRE(property.valid());
        return property.get();
    };
    // Wall/simulation time before ignition must not consume the schedule.
    for (int step = 0; step < 30; ++step) {
        used.step(off);
        CHECK(thrust(used, 0) == 0.0);
    }
    const auto replay = [&](JsbsimModel& model) {
        for (int step = 0; step < static_cast<int>(1.0 / dt); ++step) {
            // A solid motor stays lit through zero thrust even with throttle off.
            model.step(step == 0 ? on : off);
            const double time = step * dt;
            INFO("ignition-relative time " << time << ", dt " << dt);
            if (time < 0.08) CHECK(thrust(model, 0) == 10.0);
            if (time > 0.12 && time < 0.58) CHECK(thrust(model, 0) == 0.0);
            if (time > 0.62 && time < 0.68) CHECK(thrust(model, 0) == 6.0);
            if (time > 0.72) CHECK(thrust(model, 0) == 0.0);
            // The unmarked table keeps its consumed-mass meaning, not seconds.
            CHECK(thrust(model, 1) == 10.0);
        }
        const double impulse = model.property("propulsion/engine/total-vac-impulse").get();
        CHECK(std::abs(impulse - 1.6) <= 20.0 * dt);
        CHECK(model.property("propulsion/tank/contents-lbs").get() > 0.98);
    };
    replay(used);
    REQUIRE(used.reset(ic));
    CHECK(thrust(used, 0) == 0.0);
    CHECK(used.property("propulsion/engine/total-vac-impulse").get() == 0.0);
    for (int step = 0; step < 30; ++step) {
        used.step(off);
        fresh.step(off);
    }
    for (int step = 0; step < static_cast<int>(1.0 / dt); ++step) {
        used.step(step == 0 ? on : off);
        fresh.step(step == 0 ? on : off);
        CHECK(thrust(used, 0) == thrust(fresh, 0));
        CHECK(used.property("propulsion/tank/contents-lbs").get() == fresh.property("propulsion/tank/contents-lbs").get());
        VehicleState a, b;
        used.state(a);
        fresh.state(b);
        CHECK(bitIdentical(a, b));
    }
    // Marked tables retain the upstream fuel-availability gate. Fuel need is
    // evaluated after thrust, so allow its existing first-step latency.
    REQUIRE(used.reset(ic));
    used.property("propulsion/tank/contents-lbs").set(0.0);
    for (int step = 0; step < 20; ++step) {
        used.step(on);
        if (step > 1) CHECK(thrust(used, 0) == 0.0);
    }
    REQUIRE(used.reset(ic));
    used.step(on);
    CHECK(thrust(used, 0) == 10.0);
}

TEST_CASE("rocket startup leaves synthetic engines unburned until the first simulation step", "[sim][jsbsim][rocket-reset]") {
    auto ground = std::make_shared<FlatGround>(0.0);
    const AircraftSpec spec{"rocket_reset", kRoot, FSIM_TEST_DATA_DIR};
    InitialConditions ic;
    ic.altitudeMslM = 1000.0;
    ic.airspeedTrueMs = 30.0;
    ic.enginesRunning = true;
    JsbsimModel model(kDt, ground);
    REQUIRE(model.load(spec, ic));
    const auto unburned = [&] {
        VehicleState state;
        model.state(state);
        CHECK(state.simTime == 0.0);
        CHECK(state.stepCount == 0);
        CHECK(model.property("propulsion/engine[1]/isp").get() == 100.0 + model.property("velocities/mach").get());
        for (unsigned engine = 0; engine < 2; ++engine) {
            for (const char* suffix : {"total-impulse", "total-vac-impulse", "vacuum-thrust_lbs", "thrust-lbs"}) {
                const std::string path = "propulsion/engine[" + std::to_string(engine) + "]/" + suffix;
                INFO(path);
                REQUIRE(model.property(path).valid());
                CHECK(model.property(path).get() == 0.0);
            }
        }
    };
    unburned();
    ControlInputs on;
    on.setThrottleAll(1.0);
    for (int step = 0; step < 12; ++step) model.step(on);
    REQUIRE(model.property("propulsion/engine/total-vac-impulse").get() > 0.0);
    REQUIRE(model.property("propulsion/engine[1]/total-vac-impulse").get() > 0.0);
    REQUIRE(model.reset(ic));
    unburned();
}

TEST_CASE("the first step after a reset starts from the new state, however violent the last one was", "[sim][jsbsim]") {
    // The integrators remember past accelerations, and JSBSim's reset seeded
    // that memory from the last step before it: after a violent step - here a
    // 1.3 MN shove on hangar's 23 kg Skua, as a crash gives - the first step
    // after the reset replayed it (25 m/s became 1700 m/s).
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 3000.0;
    ic.airspeedTrueMs = 25.0;
    ic.headingDeg = 0.0;
    REQUIRE(model.load(AircraftSpec{"skua", kRoot, FSIM_TEST_AIRCRAFT_DIR}, ic));
    ControlInputs in;
    for (int i = 0; i < 20; ++i) model.step(in);
    const double shove[3] = {-1.3e6, 0.0, 0.0}; // newtons, body axes
    const double none[3] = {0.0, 0.0, 0.0};
    model.setExternalForceBody(shove, none);
    for (int i = 0; i < 6; ++i) model.step(in);
    model.setExternalForceBody(none, none);

    REQUIRE(model.reset(ic));
    VehicleState before;
    model.state(before);
    model.step(in);
    VehicleState after;
    model.state(after);
    CHECK_FALSE(after.diverged);
    CHECK(std::abs(after.airspeedTrueMs - before.airspeedTrueMs) < 1.0);
    CHECK(std::abs(after.velocityNedMs[2] - before.velocityNedMs[2]) < 1.0);
}

TEST_CASE("a wheel that lands on its side does not blow the aircraft up", "[sim][jsbsim]") {
    // JSBSim divides a wheel's force by the cosine of its strut's angle to the
    // ground normal, twice: a wheel meeting the ground sideways - a cartwheel -
    // made that force grow without bound and the stock c172x blew up (to
    // 5000 m/s). flightsim builds JSBSim with the projection bounded
    // (cmake/JsbsimPatches.cmake).
    log::setLevel(log::Level::Error);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 8.0;
    ic.airspeedTrueMs = 35.0;
    ic.headingDeg = 90.0;
    ic.pitchDeg = -30.0;
    ic.rollDeg = 100.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    ControlInputs in; // idle, hands off
    in.setThrottleAll(0.0);
    double fastest = 0.0;
    VehicleState s;
    for (int i = 0; i < 1200 && !s.diverged; ++i) { // 10 s
        model.step(in);
        model.state(s);
        fastest = std::max(fastest, std::sqrt(s.velocityNedMs[0] * s.velocityNedMs[0] + s.velocityNedMs[1] * s.velocityNedMs[1] +
                                              s.velocityNedMs[2] * s.velocityNedMs[2]));
    }
    CHECK_FALSE(s.diverged);
    CHECK(fastest < 1.3 * ic.airspeedTrueMs); // the crash takes energy out, never puts it in
    log::setLevel(log::Level::Warn);
}

TEST_CASE("the wheels report their struts, steering and roll", "[sim][jsbsim]") {
    // what the viewer's gear follows: parked, each strut is compressed by
    // the weight on it; taxiing, the wheels roll at the ground speed and the
    // nose wheel steers with the pedals (hangar's C172 links them)
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.onGround = true;
    ic.airspeedTrueMs = 0.0;
    REQUIRE(model.load(AircraftSpec{"c172", kRoot, FSIM_TEST_AIRCRAFT_DIR}, ic));
    ControlInputs in;
    in.setThrottleAll(0.0);
    in.brakeLeft = in.brakeRight = 1.0;
    for (int i = 0; i < 3 * 120; ++i) model.step(in);
    VehicleState s;
    model.state(s);
    REQUIRE(s.wheelCount == 3); // the nose wheel, then the left and right mains
    for (int k = 0; k < 3; ++k) {
        CHECK(s.wheelCompressionM[k] > 0.005);
        CHECK(s.wheelCompressionM[k] < 0.5);
        CHECK(std::abs(s.wheelSpeedMs[k]) < 0.05);
    }
    CHECK(s.wheelCompressionM[3] == 0.0);
    in.brakeLeft = in.brakeRight = 0.0;
    in.setThrottleAll(0.6);
    in.rudder = 0.5;
    for (int i = 0; i < 6 * 120; ++i) model.step(in);
    model.state(s);
    const double ground_speed = std::sqrt(s.velocityNedMs[0] * s.velocityNedMs[0] + s.velocityNedMs[1] * s.velocityNedMs[1]);
    CHECK(ground_speed > 1.0);
    CHECK(std::abs(s.wheelSpeedMs[1] - ground_speed) < 0.2 * ground_speed);
    CHECK(s.wheelSteerRad[0] < -0.05); // left pedal: the rudder and the nose wheel both turn left
    CHECK(s.wheelSteerRad[1] == 0.0); // the mains do not steer
}

TEST_CASE("ground provider drives AGL and gear contact", "[sim][jsbsim]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(500.0); // terrain at 500 m
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.altitudeMslM = 1500.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    VehicleState s;
    model.state(s);
    CHECK(std::abs(s.altitudeAglM - 1000.0) < 2.0);

    // Start on the ground: JSBSim should place the gear on the provider's terrain.
    InitialConditions onGround;
    onGround.onGround = true;
    onGround.airspeedTrueMs = 0.0;
    JsbsimModel parked(kDt, ground);
    REQUIRE(parked.load(AircraftSpec{"c172x", kRoot}, onGround));
    ControlInputs idle;
    idle.brakeLeft = idle.brakeRight = 1.0;
    for (int i = 0; i < 600; ++i) parked.step(idle); // settle 5 s on the gear springs
    bool touchedGround = false;
    for (int i = 0; i < 120; ++i) { // observe 1 s
        parked.step(idle);
        parked.state(s);
        touchedGround = touchedGround || s.onGround;
    }
    CAPTURE(s.altitudeMslM, s.altitudeAglM, s.airspeedTrueMs, s.velocityNedMs[2]);
    CHECK(touchedGround);
    CHECK(std::abs(s.altitudeMslM - 500.0) < 3.0); // CG sits ~1 m above the terrain on its gear
    CHECK(s.altitudeAglM < 3.0);
    CHECK(std::abs(s.velocityNedMs[2]) < 0.5);
}

TEST_CASE("trajectories are bit-identical across worker counts", "[sim][jsbsim][determinism]") {
    log::setLevel(log::Level::Warn);
    constexpr unsigned vehicles = 6, agentSteps = 60;
    constexpr int frameSkip = 4;

    const auto serial = fly(vehicles, 1, agentSteps, frameSkip);
    const auto two = fly(vehicles, 2, agentSteps, frameSkip);
    const auto four = fly(vehicles, 4, agentSteps, frameSkip);

    REQUIRE(serial.size() == vehicles);
    for (unsigned i = 0; i < vehicles; ++i) {
        CHECK_FALSE(serial[i].diverged);
        CHECK(serial[i].stepCount == agentSteps * frameSkip);
        CHECK(bitIdentical(serial[i], two[i]));
        CHECK(bitIdentical(serial[i], four[i]));
    }
    // Vehicles really are different from each other (different ICs / ailerons).
    CHECK_FALSE(bitIdentical(serial[0], serial[1]));
}

TEST_CASE("quaternion helper reproduces JSBSim's body-to-ECEF matrix", "[sim][jsbsim][attitude]") {
    log::setLevel(log::Level::Warn);
    auto ground = std::make_shared<FlatGround>(0.0);
    JsbsimModel model(kDt, ground);
    InitialConditions ic;
    ic.headingDeg = 123.0;
    ic.pitchDeg = 4.0;
    ic.rollDeg = -17.0;
    REQUIRE(model.load(AircraftSpec{"c172x", kRoot}, ic));
    ControlInputs in;
    in.setThrottleAll(0.7);
    in.aileron = 0.2;
    for (int i = 0; i < 90; ++i) model.step(in); // a rolling, turning state
    VehicleState s;
    model.state(s);

    double R[9];
    attitude::bodyToEcefFromQuaternion(s.attitudeEcefToBody, R);
    for (int i = 0; i < 9; ++i) CHECK(std::abs(R[i] - s.rotationBodyToEcef[i]) < 1e-9);

    // slerp end points are the inputs; the midpoint is a unit quaternion.
    double q[4];
    attitude::slerp(s.attitudeEcefToBody, s.attitudeEcefToBody, 0.5, q);
    for (int i = 0; i < 4; ++i) CHECK(std::abs(q[i] - s.attitudeEcefToBody[i]) < 1e-12);
}

TEST_CASE("state snapshots never move as the store grows", "[sim][stable]") {
    // The SDK hands out pointers to these and promises they hold while the
    // vehicle lives; a vector would have moved them at every doubling.
    core::StableArray<sim::VehicleState> states(3000);
    const sim::VehicleState* first = &states.emplace_back();
    for (int i = 1; i < 3000; ++i) states.emplace_back().stepCount = static_cast<std::uint32_t>(i);
    CHECK(&states[0] == first);
    CHECK(states.data() == first);
    CHECK(&states[2999] == first + 2999); // still one contiguous block
    CHECK(states[2999].stepCount == 2999u);
    CHECK(states.size() == 3000u);
    CHECK_THROWS_AS(states.emplace_back(), std::length_error);
}
