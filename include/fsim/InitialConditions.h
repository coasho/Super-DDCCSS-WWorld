#pragma once

#include <limits>

namespace fsim::sim {

/// Where and how to start. Geodetic, SI. Sampled per vehicle by the
/// environment from the scenario's distributions.
struct InitialConditions {
    double latitudeDeg = 37.6188;   ///< default: KSFO area
    double longitudeDeg = -122.375;
    double altitudeMslM = 1500.0;
    double headingDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
    double airspeedTrueMs = 60.0;
    bool onGround = false;          ///< if true, altitude is taken from terrain
    /// Its velocity over the earth (north, east, down; m/s) when all three are given:
    /// a released store's, its launcher's and its ejection's. Else `airspeedTrueMs` along the nose.
    double velocityNedMs[3] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN()};
    /// Its body rates (p, q, r; rad/s) when all three are given: a piece leaving a rolling aircraft (MS-7g). Else none.
    double bodyRatesRadS[3] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::quiet_NaN()};
};

} // namespace fsim::sim

namespace fsim {
using sim::InitialConditions;
}
