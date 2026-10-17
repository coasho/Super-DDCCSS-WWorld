#pragma once

#include <vsg/all.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace fsim::world {

/// Planck's law through the CIE 1931 observer (Wyman, Sloan and Shirley's fit) into linear sRGB: the colour and the
/// luminance (relative) of a blackbody at T kelvin (as world::Exhaust's).
inline void blackbody(double T, double& r, double& g, double& b, double& y) {
    const auto lobe = [](double nm, double mean, double below, double above) {
        const double t = (nm - mean) / (nm < mean ? below : above);
        return std::exp(-0.5 * t * t);
    };
    double X = 0.0, Y = 0.0, Z = 0.0;
    for (double nm = 380.0; nm <= 780.0; nm += 2.0) {
        const double m = nm * 1e-9;
        const double L = 3.7418e-16 / std::pow(m, 5.0) / std::expm1(0.014388 / (m * T));
        X += L * (1.056 * lobe(nm, 599.8, 37.9, 31.0) + 0.362 * lobe(nm, 442.0, 16.0, 26.7) - 0.065 * lobe(nm, 501.1, 20.4, 26.2));
        Y += L * (0.821 * lobe(nm, 568.8, 46.9, 40.5) + 0.286 * lobe(nm, 530.9, 16.3, 31.1));
        Z += L * (1.217 * lobe(nm, 437.0, 11.8, 36.0) + 0.681 * lobe(nm, 459.0, 26.0, 13.8));
    }
    r = std::max(0.0, 3.2406 * X - 1.5372 * Y - 0.4986 * Z);
    g = std::max(0.0, -0.9689 * X + 1.8758 * Y + 0.0415 * Z);
    b = std::max(0.0, 0.0557 * X - 0.204 * Y + 1.057 * Z);
    y = Y;
}

/// How much a turbulent flame's temperature fluctuates about its mean (rms, a share of it): 0.2, the middle of the
/// 0.15 - 0.3 measured in large hydrocarbon fires (Cox and Chitty 1985; Gengembre et al. 1984) - its visible light,
/// steep in temperature, in bright patches over a darker mean, as the hits are photographed.
constexpr double kFlicker = 0.2;

/// A blackbody's light against the sun's disk (5,778 K), worked out once from 500 to 6000 K: its colour (linear sRGB
/// at luminance 1) and its luminance as a share of the sun's - times the atmosphere's solar luminance, what a hot body
/// of that temperature looks like next to the sky.
class SunlitBlackbody {
public:
    SunlitBlackbody() {
        double r, g, b, y;
        blackbody(5778.0, r, g, b, y);
        const double sun = y;
        table_.resize(256);
        for (std::size_t i = 0; i < table_.size(); ++i) {
            const double T = 500.0 + 5500.0 * static_cast<double>(i) / static_cast<double>(table_.size() - 1);
            blackbody(T, r, g, b, y);
            const double l = std::max(0.2126 * r + 0.7152 * g + 0.0722 * b, 1e-30);
            table_[i] = vsg::dvec4(r / l, g / l, b / l, y / sun);
        }
    }
    /// The table as the shaders read it (contrail.glsl's blackbodyAt): kRows rows from `at`, 500 to 6000 K.
    static constexpr std::uint32_t kRows = 256;
    void write(vsg::vec4Array& rows, std::uint32_t at) const {
        for (std::uint32_t i = 0; i < kRows; ++i) {
            const vsg::dvec4& c = table_[i];
            rows.set(at + i, vsg::vec4(static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(c.z), static_cast<float>(c.w)));
        }
    }
    /// A turbulent flame's mean temperature, its temperature fluctuating by `flicker` of it (rms, Gaussian): the one
    /// whose mean sigma T^4 is a black body's at `radiatingK` (what its surface emissive power gives) -
    /// <(1 + f n)^4> = 1 + 6 f^2 + 3 f^4.
    static double meanOfFlicker(double radiatingK, double flicker) {
        const double f2 = flicker * flicker;
        return radiatingK / std::pow(1.0 + 6.0 * f2 + 3.0 * f2 * f2, 0.25);
    }
    /// (colour, luminance against the sun's) at T kelvin; nothing below 500 K.
    vsg::dvec4 at(double T) const {
        if (!(T >= 500.0)) return vsg::dvec4(1.0, 1.0, 1.0, 0.0);
        const double u = std::clamp((T - 500.0) / 5500.0, 0.0, 1.0) * static_cast<double>(table_.size() - 1);
        const auto i0 = static_cast<std::size_t>(u);
        const std::size_t i1 = std::min(i0 + 1, table_.size() - 1);
        const double w = u - static_cast<double>(i0);
        return table_[i0] * (1.0 - w) + table_[i1] * w;
    }

private:
    std::vector<vsg::dvec4> table_;
};

} // namespace fsim::world
