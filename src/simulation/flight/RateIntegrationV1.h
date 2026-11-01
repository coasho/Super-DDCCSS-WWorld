#pragma once

#include <array>
#include <cstddef>

namespace fsim::sim::detail {

// Private JSBSim vector-rate map, in its existing enum order. No SDK ABI.
// The caller supplies the current derivative separately; past[0] is the
// derivative most recently consumed by FGPropagate::Integrate.
struct RateIntegrationV1 {
    std::array<double, 5> coefficients{};
    bool valid = true;

    constexpr double current() const noexcept { return coefficients[0]; }
    double history(const std::array<double, 4>& past) const noexcept {
        double value = 0.;
        for (std::size_t j = 1; j < coefficients.size(); ++j)
            if (coefficients[j] != 0.) value += coefficients[j] * past[j - 1];
        return value;
    }
};

constexpr RateIntegrationV1 rateIntegrationV1(int selector) noexcept {
    switch (selector) {
    case 0: return {}; // explicitly frozen rate
    case 1: return {{{1., 0., 0., 0., 0.}}, true};
    case 2: return {{{.5, .5, 0., 0., 0.}}, true};
    case 3: return {{{1.5, -.5, 0., 0., 0.}}, true};
    case 4: return {{{23./12., -16./12., 5./12., 0., 0.}}, true};
    case 5: return {{{55./24., -59./24., 37./24., -9./24., 0.}}, true};
    case 9: return {{{1901./720., -1387./360., 109./30., -637./360., 251./720.}}, true};
    default: return {{}, false}; // Buss/local-linearization are not rate methods
    }
}

} // namespace fsim::sim::detail
