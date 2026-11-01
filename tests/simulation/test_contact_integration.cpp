#include "simulation/flight/RateIntegrationV1.h"

#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
#include <utility>

using fsim::sim::detail::rateIntegrationV1;

TEST_CASE("contact rate maps retain native quadrature moments and reject position-only methods", "[integration][contact-rate-map]") {
    for (const auto [selector, order] : {std::pair{1, 1}, {2, 1}, {3, 2}, {4, 3}, {5, 4}, {9, 5}}) {
        const auto map = rateIntegrationV1(selector);
        REQUIRE(map.valid);
        for (int degree = 0; degree < order; ++degree) {
            double moment = 0.;
            for (std::size_t j = 0; j < map.coefficients.size(); ++j)
                moment += map.coefficients[j] * std::pow(-static_cast<double>(j), degree);
            CHECK(std::abs(moment - 1. / (degree + 1)) < 1e-12);
        }
    }
    const auto frozen = rateIntegrationV1(0);
    CHECK(frozen.valid);
    CHECK(frozen.current() == 0.);
    CHECK(frozen.history({1., 2., 3., 4.}) == 0.);
    for (int selector : {6, 7, 8, -1, 10}) CHECK_FALSE(rateIntegrationV1(selector).valid);
}

TEST_CASE("mixed rate integrators stop one coupled contact using their actual history response", "[integration][contact-rate-map]") {
    const std::array<double, 4> translationPast{.3, -.2, .5, -.4};
    const std::array<double, 4> rotationPast{-.7, .1, .2, .6};
    for (int translation : {1, 2, 3, 4, 5, 9}) for (int rotation : {1, 2, 3, 4, 5, 9}) {
        const auto v = rateIntegrationV1(translation), w = rateIntegrationV1(rotation);
        for (double dt : {.001, .008333333333333333, .02}) {
            const double velocity = .8, omega = -.2, mass = 2., inertia = 3., arm = .4;
            const double linearFree = .3, angularFree = -.1;
            const double freePoint = velocity + arm * omega + dt * (v.current() * linearFree + v.history(translationPast)
                + arm * (w.current() * angularFree + w.history(rotationPast)));
            const double response = dt * (v.current() / mass + w.current() * arm * arm / inertia);
            const double force = -freePoint / response;
            const double nextV = velocity + dt * (v.current() * (linearFree + force / mass) + v.history(translationPast));
            const double nextW = omega + dt * (w.current() * (angularFree + arm * force / inertia) + w.history(rotationPast));
            CHECK(std::abs(nextV + arm * nextW) < 1e-12);
        }
    }
}
