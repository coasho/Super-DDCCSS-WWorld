#include "simulation/flight_control/RotorAllocationV1.h"
#include "simulation/flight_control/Route.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>

TEST_CASE("Rotor allocation keeps integral authority below the actual tilt cone", "[rotorcraft][control]") {
    using namespace fsim::control;
    const auto allocation=allocateRotorThrustV1(1./23,1./71,9.81,24.*3.14159265358979323846/180,0,0);
    REQUIRE_FALSE(allocation.saturated);
    // This is the observed double-rounding residual of an unsaturated
    // atan2/tan round trip, not evidence that the actuator reached a stop.
    const auto delta=rotorIntegralIncrementV1(-.01,0,-6.938893903907228e-18,0,allocation.saturated);
    CHECK(delta[0]==-.01);CHECK(delta[1]==0);
}

TEST_CASE("Rotor allocation limits the complete thrust direction and permits integral unwinding", "[rotorcraft][control]") {
    using namespace fsim::control;
    for(const double trim:{-.03,0.,.03}) {
        const auto allocation=allocateRotorThrustV1(20,15,9.81,.4,trim,-trim);
        REQUIRE(allocation.saturated);
        const double tilt=std::acos(std::clamp(std::cos(allocation.pitch)*std::cos(allocation.roll),-1.,1.));
        CHECK(std::abs(tilt-.4)<1e-12);
    }
    const auto inward=rotorIntegralIncrementV1(-.01,.02,1,0,true);
    CHECK(inward[0]==-.01);CHECK(inward[1]==.02);
    const auto outward=rotorIntegralIncrementV1(.01,.02,1,0,true);
    CHECK(outward[0]==0);CHECK(outward[1]==.02);
}

TEST_CASE("Rotor course angle correction is invariant under a uniform change of time units", "[rotorcraft][control]") {
    using namespace fsim;
    using namespace fsim::control;
    constexpr double error = 0.1, dimensionlessStep = 0.01;
    for (double bandwidth : {0.2, 0.8, 2.0}) {
        sim::VehicleState state;
        state.velocityNedMs[0] = 4.0 * std::cos(error);
        state.velocityNedMs[1] = -4.0 * std::sin(error);
        Performance performance;
        performance.hovers = true;
        performance.velocityBandwidthRadS = bandwidth;
        performance.maxAccelerationMs2 = 3.0;
        ControlContext ctx{1, state, state, dimensionlessStep / bandwidth, nullptr, nullptr};
        route::Fix fix;
        fix.courseRad = 0.0;
        route::Steer steer;
        steer.speed = 4.0;
        steer.reference = SpeedReference::GroundSpeed;
        route::Trims trims;
        double course = 0.0, heading = 0.0;
        route::follow(ctx, state, performance, WindEstimate{}, true, fix, route::Ahead{}, steer, trims, course, heading);
        // The same fraction of a course-loop time constant accumulates the
        // same angle. A turn-rate integral would instead scale with bandwidth.
        CHECK(std::abs(trims.courseRadS - 0.25 * error * dimensionlessStep) < 1e-14);
    }
}
