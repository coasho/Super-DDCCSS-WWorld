#include "ms/rf/Rain.h"

#include <algorithm>
#include <cmath>

namespace fsim::ms::rf {
namespace {

// Recommendation ITU-R P.838-3 (03/2005), tables 1-4.
struct Fit4 { double a[4], b[4], c[4], m, cc; };
struct Fit5 { double a[5], b[5], c[5], m, cc; };

constexpr Fit4 kH{{-5.33980, -0.35351, -0.23789, -0.94158}, {-0.10008, 1.26970, 0.86036, 0.64552}, {1.13098, 0.45400, 0.15354, 0.16817},
                  -0.18961, 0.71147};
constexpr Fit4 kV{{-3.80595, -3.44965, -0.39902, 0.50167}, {0.56934, -0.22911, 0.73042, 1.07319}, {0.81061, 0.51059, 0.11899, 0.27195},
                  -0.16398, 0.63297};
constexpr Fit5 aH{{-0.14318, 0.29591, 0.32177, -5.37610, 16.1721}, {1.82442, 0.77564, 0.63773, -0.96230, -3.29980},
                  {-0.55187, 0.19822, 0.13164, 1.47828, 3.43990}, 0.67849, -1.95537};
constexpr Fit5 aV{{-0.07771, 0.56727, -0.20238, -48.2991, 48.5833}, {2.33840, 0.95545, 1.14520, 0.791669, 0.791459},
                  {-0.76284, 0.54039, 0.26809, 0.116226, 0.116479}, -0.053739, 0.83433};

template <class Fit, int N>
double sum(const Fit& f, double lf) noexcept {
    double s = 0.0;
    for (int j = 0; j < N; ++j) {
        const double x = (lf - f.b[j]) / f.c[j];
        s += f.a[j] * std::exp(-x * x);
    }
    return s + f.m * lf + f.cc;
}

} // namespace

void rainCoefficientsHV(double fGhz, double& kh, double& alphaH, double& kv, double& alphaV) noexcept {
    const double lf = std::log10(std::clamp(fGhz, 1.0, 1000.0));
    kh = std::pow(10.0, sum<Fit4, 4>(kH, lf)); // (2)
    kv = std::pow(10.0, sum<Fit4, 4>(kV, lf));
    alphaH = sum<Fit5, 5>(aH, lf);              // (3)
    alphaV = sum<Fit5, 5>(aV, lf);
}

RainCoefficients rainCoefficients(double fGhz, double elevationRad, double tiltRad) noexcept {
    double kh, ah, kv, av;
    rainCoefficientsHV(fGhz, kh, ah, kv, av);
    const double c = std::cos(elevationRad);
    const double geometry = c * c * std::cos(2.0 * tiltRad);
    RainCoefficients r;
    r.k = (kh + kv + (kh - kv) * geometry) / 2.0;                              // (4)
    r.alpha = (kh * ah + kv * av + (kh * ah - kv * av) * geometry) / (2.0 * r.k); // (5)
    return r;
}

double rainSpecificDbPerKm(double fGhz, double rateMmH, double elevationRad, double tiltRad) noexcept {
    if (!(rateMmH > 0.0)) return 0.0;
    const RainCoefficients c = rainCoefficients(fGhz, elevationRad, tiltRad);
    return c.k * std::pow(rateMmH, c.alpha); // (1)
}

} // namespace fsim::ms::rf
