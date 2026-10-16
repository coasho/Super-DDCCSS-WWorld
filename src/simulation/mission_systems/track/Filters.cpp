#include "simulation/mission_systems/track/Filters.h"

#include <algorithm>

namespace fsim::ms::track {

Gaussian predict(const Gaussian& g, double t, const ConstantVelocity& model) noexcept {
    const double dt = t - g.t;
    const Covar f = model.matrix(dt);
    Gaussian out;
    out.x = f * g.x;
    out.p = f * g.p * transpose(f) + model.covar(dt);
    out.t = t;
    return out;
}

namespace {

/// Weighted circular mean (Stone Soup's Angle.average): atan2 of the weighted sines and cosines.
double circularMean(const double* angles, const double* w, std::size_t n) noexcept {
    double s = 0.0, c = 0.0, ws = 0.0;
    for (std::size_t i = 0; i < n; ++i) s += std::sin(angles[i]) * w[i], c += std::cos(angles[i]) * w[i], ws += w[i];
    return std::atan2(s / ws, c / ws);
}

} // namespace

MeasurementPrediction predictMeasurement(const Gaussian& g, const RadarMeasurement& m, Filter f) noexcept {
    MeasurementPrediction mp;
    if (f == Filter::Ekf) {
        const Mat<4, 6> h = m.jacobian(g.x);
        mp.z = m.function(g.x);
        mp.cross = g.p * transpose(h);
        mp.s = h * mp.cross + m.noise;
        return mp;
    }
    // the unscented transform: 2n + 1 sigma points (gauss2sigma), through the measurement function
    constexpr std::size_t n = 6, k = 2 * n + 1;
    const double alpha = 0.5, beta = 2.0, kappa = 3.0 - static_cast<double>(n);
    const double lamda = alpha * alpha * (static_cast<double>(n) + kappa) - static_cast<double>(n), c = static_cast<double>(n) + lamda;
    Covar l;
    if (!cholesky(g.p, l)) { // (Stone Soup: cholesky_eps - a jitter on the diagonal)
        Covar jittered = g.p;
        for (std::size_t i = 0; i < n; ++i) jittered(i, i) += 1e-10 * std::max(std::fabs(jittered(i, i)), 1.0);
        cholesky(jittered, l);
    }
    std::array<State, k> pts;
    pts[0] = g.x;
    const double sc = std::sqrt(c);
    for (std::size_t j = 0; j < n; ++j)
        for (std::size_t i = 0; i < n; ++i) pts[1 + j].a[i] = g.x.a[i] + l(i, j) * sc, pts[1 + n + j].a[i] = g.x.a[i] - l(i, j) * sc;
    std::array<double, k> wm, wc;
    wm[0] = lamda / c;
    for (std::size_t i = 1; i < k; ++i) wm[i] = 0.5 / c;
    wc = wm;
    wc[0] = lamda / c + (1.0 - alpha * alpha + beta);
    std::array<Meas, k> zs;
    for (std::size_t i = 0; i < k; ++i) zs[i] = m.function(pts[i]);
    // the mean: the angles circular (Stone Soup's typed elevation and bearing), the rest weighted
    double el[k], br[k];
    for (std::size_t i = 0; i < k; ++i) el[i] = zs[i].a[0], br[i] = zs[i].a[1];
    mp.z.a[0] = circularMean(el, wm.data(), k);
    mp.z.a[1] = circularMean(br, wm.data(), k);
    for (std::size_t d = 2; d < 4; ++d) {
        double s = 0.0, ws = 0.0;
        for (std::size_t i = 0; i < k; ++i) s += wm[i] * zs[i].a[d], ws += wm[i];
        mp.z.a[d] = s / ws;
    }
    mp.s = m.noise;
    for (std::size_t i = 0; i < k; ++i) {
        const Meas dz = RadarMeasurement::difference(zs[i], mp.z);
        for (std::size_t r = 0; r < 4; ++r) {
            for (std::size_t q = 0; q < 4; ++q) mp.s(r, q) += wc[i] * dz.a[r] * dz.a[q];
            for (std::size_t q = 0; q < n; ++q) mp.cross(q, r) += wc[i] * (pts[i].a[q] - pts[0].a[q]) * dz.a[r];
        }
    }
    return mp;
}

bool update(const Gaussian& prediction, const MeasurementPrediction& mp, const Meas& z, Gaussian& out) noexcept {
    MeasCovar sInv;
    if (!inverse(mp.s, sInv)) return false;
    const Mat<6, 4> gain = mp.cross * sInv;
    out.x = prediction.x + gain * RadarMeasurement::difference(z, mp.z);
    out.p = prediction.p - gain * mp.s * transpose(gain);
    out.t = prediction.t;
    return true;
}

Gaussian reduce(const std::vector<Gaussian>& states, const std::vector<double>& weights) noexcept {
    Gaussian out;
    if (states.empty()) return out;
    double total = 0.0;
    for (double w : weights) total += w;
    for (std::size_t i = 0; i < states.size(); ++i) out.x = out.x + (weights[i] / total) * states[i].x;
    for (std::size_t i = 0; i < states.size(); ++i) {
        const double w = weights[i] / total;
        const State d = states[i].x - out.x;
        out.p = out.p + w * states[i].p + w * (d * transpose(d));
    }
    out.t = states[0].t;
    return out;
}

} // namespace fsim::ms::track
