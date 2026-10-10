#include "ms/radar/Detection.h"

#include <algorithm>
#include <cmath>

namespace fsim::ms::radar {

Fluctuation fluctuationOf(double ratio) noexcept {
    if (!(ratio > 1.25)) return Fluctuation::Steady;
    if (ratio < 1.75) return Fluctuation::DominantScatterer;
    return Fluctuation::Rayleigh;
}

double marcumQ(double a, double b) noexcept {
    // Q1(a, b) = P(a noncentral chi-square of 2 degrees, noncentrality a^2, exceeds b^2): the Poisson mixture of central
    // ones, sum_j Pois(j; a^2/2) Q(j + 1, b^2/2), Q the upper regularised gamma - summed from the Poisson mode out, each
    // Q from its neighbour by one term (Q(j + 1, x) = Q(j, x) + x^j e^-x / j!)
    if (b <= 0.0) return 1.0;
    if (a <= 0.0) return std::exp(-0.5 * b * b);
    if (a - b > 9.0) return 1.0; // (the Gaussian tail beyond 9 sigma)
    const double lambda = 0.5 * a * a, x = 0.5 * b * b;
    const auto mode = static_cast<long>(std::floor(lambda));
    auto logTerm = [&](long i) { return i * std::log(x) - x - std::lgamma(i + 1.0); }; // ln(x^i e^-x / i!)
    auto logPois = [&](long j) { return j * std::log(lambda) - lambda - std::lgamma(j + 1.0); };
    // Q(mode + 1, x): its sum from the largest term down (x^i/i! peaks at i = x)
    double qMode = 0.0;
    for (long i = 0; i <= mode; ++i) qMode += std::exp(logTerm(i));
    qMode = std::min(qMode, 1.0);
    double total = std::exp(logPois(mode)) * qMode;
    double q = qMode;
    for (long j = mode + 1; j < mode + 100000; ++j) { // upwards
        q = std::min(1.0, q + std::exp(logTerm(j)));
        const double p = std::exp(logPois(j));
        total += p * q;
        if (j > mode + 5 && p < 1e-17) break;
    }
    q = qMode;
    for (long j = mode - 1; j >= 0; --j) { // downwards
        q = std::max(0.0, q - std::exp(logTerm(j + 1)));
        const double p = std::exp(logPois(j));
        total += p * q;
        if (mode - j > 5 && p < 1e-17) break;
    }
    return std::clamp(total, 0.0, 1.0);
}

double probabilityOfDetection(double snr, double pfa, Fluctuation f) noexcept {
    snr = std::max(snr, 0.0);
    pfa = std::clamp(pfa, 1e-15, 0.5);
    const double t = -std::log(pfa);
    switch (f) {
    case Fluctuation::Rayleigh: return std::pow(pfa, 1.0 / (1.0 + snr));
    case Fluctuation::DominantScatterer: {
        const double h = 1.0 + 0.5 * snr;
        return std::min(1.0, (1.0 + snr * t / (2.0 * h * h)) * std::exp(-t / h));
    }
    case Fluctuation::Steady: break;
    }
    return marcumQ(std::sqrt(2.0 * snr), std::sqrt(2.0 * t));
}

double requiredSnr(double pd, double pfa, Fluctuation f) noexcept {
    pd = std::clamp(pd, 1e-6, 0.999999);
    double lo = 0.0, hi = 1e9;
    for (int i = 0; i < 200; ++i) { // (bisection in log: Pd rises with S)
        const double mid = lo <= 0.0 ? hi * 1e-12 : std::sqrt(lo * hi);
        if (probabilityOfDetection(mid, pfa, f) < pd) lo = mid;
        else hi = mid;
        if (lo > 0.0 && hi / lo < 1.0 + 1e-9) break;
    }
    return hi;
}

} // namespace fsim::ms::radar
