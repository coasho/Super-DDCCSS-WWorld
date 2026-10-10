#pragma once
// The trackers' models (ADR-30, MS-5), as Stone Soup defines them (Dstl, MIT;
// models/transition/linear.py, models/measurement/nonlinear.py): the state
// x = [x, vx, y, vy, z, vz] in a Cartesian frame; nearly constant velocity
// (white-noise acceleration, a diffusion coefficient a axis); and a radar's
// measurement z = [elevation, bearing, range, range rate] from a sensor at a
// translation, rotated, moving.

#include "ms/track/Linalg.h"

namespace fsim::ms::track {

using State = Vec<6>;
using Covar = Mat<6, 6>;
using Meas = Vec<4>;
using MeasCovar = Mat<4, 4>;

/// Stone Soup's CombinedLinearGaussianTransitionModel of three ConstantVelocity (one an axis).
struct ConstantVelocity {
    double q[3] = {1.0, 1.0, 1.0}; ///< each axis' noise diffusion coefficient (m^2/s^3)
    Covar matrix(double dt) const noexcept;
    Covar covar(double dt) const noexcept;
};

/// Stone Soup's CartesianToElevationBearingRangeRate: the sensor's frame is the world's rotated by `rotation` (world
/// to sensor), its origin at `translation`, moving at `velocity`.
struct RadarMeasurement {
    Mat<3, 3> rotation = identity<3>();
    double translation[3] = {0, 0, 0};
    double velocity[3] = {0, 0, 0};
    MeasCovar noise; ///< R: diagonal of (sigma_el^2, sigma_bearing^2, sigma_range^2, sigma_rate^2)

    Meas function(const State& x) const noexcept;
    Mat<4, 6> jacobian(const State& x) const noexcept;
    /// The state a measurement stands for (its position; its velocity the range rate along the line of sight, plus
    /// the sensor's) - inverse_function.
    State inverse(const Meas& z) const noexcept;
    /// z - prediction, the angles wrapped as Stone Soup's typed vectors wrap them.
    static Meas difference(const Meas& z, const Meas& prediction) noexcept;
};

} // namespace fsim::ms::track
