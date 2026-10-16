#include "simulation/mission_systems/rf/Antenna.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace fsim::ms::rf {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLight = 299792458.0;
constexpr int kIntervals = 2048; ///< Simpson's rule across the aperture (even)
constexpr double kStep = 1.0 / 32.0; ///< the table's spacing in u: 32 samples a lobe

/// Taylor's line-source illumination (T. T. Taylor, "Design of line-source antennas for narrow beamwidth and low side
/// lobes", IRE Trans. AP-3, 1955): 1 + 2 sum F_m cos(2 pi m x) over x in [-1/2, 1/2].
std::vector<double> taylorCoefficients(double sidelobeDb, int nbar) {
    const double r = std::pow(10.0, sidelobeDb / 20.0);
    const double a = std::acosh(r) / kPi;
    const double sigma2 = nbar * nbar / (a * a + (nbar - 0.5) * (nbar - 0.5));
    std::vector<double> f(static_cast<std::size_t>(std::max(nbar, 1)), 0.0);
    for (int m = 1; m < nbar; ++m) {
        double num = 1.0, den = 1.0;
        for (int n = 1; n < nbar; ++n) {
            num *= 1.0 - m * m / (sigma2 * (a * a + (n - 0.5) * (n - 0.5)));
            if (n != m) den *= 1.0 - static_cast<double>(m * m) / static_cast<double>(n * n);
        }
        f[static_cast<std::size_t>(m)] = ((m + 1) % 2 == 0 ? 1.0 : -1.0) * num / (2.0 * den);
    }
    return f;
}

std::function<double(double)> lineIllumination(const ApertureSpec& s) {
    const double p = std::clamp(s.pedestal, 0.0, 1.0);
    switch (s.taper) {
    case Taper::Cosine: return [p](double x) { return p + (1.0 - p) * std::cos(kPi * x); };
    case Taper::CosineSquared: return [p](double x) { const double c = std::cos(kPi * x); return p + (1.0 - p) * c * c; };
    case Taper::Taylor: {
        auto f = taylorCoefficients(std::max(s.taylorSidelobeDb, 13.27), std::max(s.taylorNbar, 2));
        return [f](double x) {
            double v = 1.0;
            for (std::size_t m = 1; m < f.size(); ++m) v += 2.0 * f[m] * std::cos(2.0 * kPi * static_cast<double>(m) * x);
            return v;
        };
    }
    default: return [](double) { return 1.0; };
    }
}

std::function<double(double)> circularIllumination(const ApertureSpec& s) {
    const double p = std::clamp(s.pedestal, 0.0, 1.0);
    switch (s.taper) {
    case Taper::Parabolic: return [p](double r) { return p + (1.0 - p) * (1.0 - 4.0 * r * r); };
    case Taper::ParabolicSquared: return [p](double r) { const double q = 1.0 - 4.0 * r * r; return p + (1.0 - p) * q * q; };
    default: return [](double) { return 1.0; };
    }
}

/// Simpson's rule of g over [a, b].
double simpson(const std::function<double(double)>& g, double a, double b) {
    const double h = (b - a) / kIntervals;
    double s = g(a) + g(b);
    for (int i = 1; i < kIntervals; ++i) s += g(a + i * h) * (i % 2 ? 4.0 : 2.0);
    return s * h / 3.0;
}

} // namespace

double Pattern::Table::at(double u) const noexcept {
    u = std::fabs(u);
    const double i = u / du;
    if (field.empty() || i >= static_cast<double>(field.size() - 1)) return 0.0;
    const auto k = static_cast<std::size_t>(i);
    const double t = i - static_cast<double>(k);
    return field[k] + (field[k + 1] - field[k]) * t;
}

Pattern::Pattern(const ApertureSpec& spec, double frequencyHz) : spec_(spec), wavelengthM_(kLight / frequencyHz) {
    auto tabulate = [&](Table& t, double extentM, const std::function<double(double)>& transform) {
        t.extentWavelengths = extentM / wavelengthM_;
        t.du = kStep;
        const double uMax = 2.0 * t.extentWavelengths + 2.0; // sine differences reach 2
        const auto n = static_cast<std::size_t>(uMax / kStep) + 2;
        t.field.resize(n);
        const double peak = transform(0.0);
        for (std::size_t i = 0; i < n; ++i) t.field[i] = std::fabs(transform(static_cast<double>(i) * kStep) / peak);
    };
    double areaM2 = 0.0;
    if (spec.shape == ApertureShape::Circular) {
        auto a = circularIllumination(spec);
        tabulate(horizontal_, spec.widthM, [&](double rho) {
            return simpson([&](double r) { return a(r) * std::cyl_bessel_j(0.0, 2.0 * kPi * rho * r) * r; }, 0.0, 0.5);
        });
        vertical_ = horizontal_;
        const double s1 = simpson([&](double r) { return a(r) * 2.0 * kPi * r; }, 0.0, 0.5);
        const double s2 = simpson([&](double r) { return a(r) * a(r) * 2.0 * kPi * r; }, 0.0, 0.5);
        taperEfficiency_ = s1 * s1 / (kPi / 4.0 * s2);
        areaM2 = kPi / 4.0 * spec.widthM * spec.widthM;
    } else {
        auto a = lineIllumination(spec);
        auto transform = [&](double u) { return simpson([&](double x) { return a(x) * std::cos(2.0 * kPi * u * x); }, -0.5, 0.5); };
        tabulate(horizontal_, spec.widthM, transform);
        tabulate(vertical_, spec.heightM, transform);
        const double s1 = simpson(a, -0.5, 0.5);
        const double s2 = simpson([&](double x) { return a(x) * a(x); }, -0.5, 0.5);
        const double line = s1 * s1 / s2;
        taperEfficiency_ = line * line; // the same taper in both planes
        areaM2 = spec.widthM * spec.heightM;
    }
    peakDbi_ = spec.peakGainDbi > 0.0
                   ? spec.peakGainDbi
                   : 10.0 * std::log10(4.0 * kPi * areaM2 * taperEfficiency_ * std::clamp(spec.efficiency, 1e-6, 1.0) /
                                       (wavelengthM_ * wavelengthM_));
    peak_ = std::pow(10.0, peakDbi_ / 10.0), floor_ = std::pow(10.0, spec.floorDbi / 10.0), back_ = std::pow(10.0, spec.backDbi / 10.0);
}

double Pattern::field(double u, bool vertical) const noexcept { return (vertical ? vertical_ : horizontal_).at(u); }

double Pattern::gain(const double d[3], double steerY, double steerZ) const noexcept {
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (!(n > 0.0)) return back_;
    const double cx = d[0] / n, cy = d[1] / n, cz = d[2] / n;
    if (cx <= 0.0) return back_;
    if (!spec_.electronic) steerY = steerZ = 0.0;
    double f2;
    if (spec_.shape == ApertureShape::Circular) {
        const double rho = horizontal_.extentWavelengths * std::hypot(cy - steerY, cz - steerZ);
        const double f = horizontal_.at(rho);
        f2 = f * f;
    } else {
        const double fh = horizontal_.at(horizontal_.extentWavelengths * (cy - steerY));
        const double fv = vertical_.at(vertical_.extentWavelengths * (cz - steerZ));
        f2 = fh * fh * fv * fv;
    }
    double g = peak_ * f2;
    if (spec_.electronic) g *= std::pow(cx, spec_.scanExponent);
    return g + floor_;
}

double Pattern::gainDbi(const double d[3], double steerY, double steerZ) const noexcept {
    const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (!(n > 0.0)) return spec_.backDbi;
    const double cx = d[0] / n, cy = d[1] / n, cz = d[2] / n;
    if (cx <= 0.0) return spec_.backDbi;
    if (!spec_.electronic) steerY = steerZ = 0.0;
    double f2;
    if (spec_.shape == ApertureShape::Circular) {
        const double rho = horizontal_.extentWavelengths * std::hypot(cy - steerY, cz - steerZ);
        const double f = horizontal_.at(rho);
        f2 = f * f;
    } else {
        const double fh = horizontal_.at(horizontal_.extentWavelengths * (cy - steerY));
        const double fv = vertical_.at(vertical_.extentWavelengths * (cz - steerZ));
        f2 = fh * fh * fv * fv;
    }
    double g = std::pow(10.0, peakDbi_ / 10.0) * f2;
    if (spec_.electronic) g *= std::pow(cx, spec_.scanExponent);
    g += std::pow(10.0, spec_.floorDbi / 10.0);
    return 10.0 * std::log10(g);
}

double Pattern::beamwidthRad(bool vertical) const noexcept {
    const Table& t = vertical ? vertical_ : horizontal_;
    const double half = 1.0 / std::sqrt(2.0);
    for (std::size_t i = 1; i < t.field.size(); ++i)
        if (t.field[i] < half) {
            const double u0 = static_cast<double>(i - 1) * t.du;
            const double u = u0 + t.du * (t.field[i - 1] - half) / (t.field[i - 1] - t.field[i]);
            return 2.0 * std::asin(std::min(1.0, u / t.extentWavelengths));
        }
    return kPi;
}

double Pattern::firstSidelobeDb(bool vertical) const noexcept {
    const Table& t = vertical ? vertical_ : horizontal_;
    std::size_t i = 1;
    while (i + 1 < t.field.size() && t.field[i + 1] <= t.field[i]) ++i; // down the main lobe to its first null
    while (i + 1 < t.field.size() && t.field[i + 1] >= t.field[i]) ++i; // up to the first side lobe
    return 20.0 * std::log10(std::max(1e-12, t.field[i]));
}

} // namespace fsim::ms::rf
