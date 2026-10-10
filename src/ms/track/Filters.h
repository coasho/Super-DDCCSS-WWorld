#pragma once
// The trackers' filters (ADR-30, MS-5), as Stone Soup's (predictor/kalman.py,
// updater/kalman.py, functions: gauss2sigma, unscented_transform,
// gm_reduce_single): a Kalman prediction through the linear motion model; the
// measurement predicted by linearisation (EKF) or by sigma points (UKF, alpha
// 0.5, beta 2, kappa 3 - n); the update P - K S K^T with K = C S^-1.

#include "ms/track/Models.h"

#include <vector>

namespace fsim::ms::track {

struct Gaussian {
    State x;
    Covar p;
    double t = 0.0; ///< seconds
};

/// A measurement predicted from a state: its mean, the innovation covariance S (noise included) and the cross
/// covariance C = cov(x, z).
struct MeasurementPrediction {
    Meas z;
    MeasCovar s;
    Mat<6, 4> cross;
};

enum class Filter : unsigned char { Ekf, Ukf };

Gaussian predict(const Gaussian& g, double t, const ConstantVelocity& model) noexcept;
MeasurementPrediction predictMeasurement(const Gaussian& prediction, const RadarMeasurement& m, Filter f) noexcept;
/// The Kalman update of a prediction by a measurement; false if S is singular.
bool update(const Gaussian& prediction, const MeasurementPrediction& mp, const Meas& z, Gaussian& out) noexcept;
/// Moment-matched reduction of a weighted Gaussian mixture (weights normalised here).
Gaussian reduce(const std::vector<Gaussian>& states, const std::vector<double>& weights) noexcept;

} // namespace fsim::ms::track
