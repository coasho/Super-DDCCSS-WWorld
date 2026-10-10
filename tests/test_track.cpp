// The trackers (ADR-30, MS-5) against Stone Soup itself: the measurements,
// sensor poses and answers tools/track/crosscheck.py had Stone Soup 1.9.1
// write (tests/data/track/crosscheck.json), replayed through the C++ port -
// one target's EKF and UKF posteriors step by step, and the GNN and JPDA
// multi-target trackers' tracks at every step (their number, each track's
// state and covariance) through crossing targets and false alarms. Then the
// pieces on their own: SciPy's assignment, the chi-square quantile, the
// clusters JPDA factorises over.
#include "core/Json.h"
#include "ms/track/Assignment.h"
#include "ms/track/Tracker.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::ms::track;

namespace {

namespace fs = std::filesystem;

const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();

core::Json fixture() {
    std::ifstream in(kSource / "tests" / "data" / "track" / "crosscheck.json");
    std::stringstream text;
    text << in.rdbuf();
    return core::Json::parse(text.str());
}

template <std::size_t N>
void numbers(const core::Json& a, double (&out)[N]) {
    const auto& v = a.asArray();
    REQUIRE(v.size() == N);
    for (std::size_t i = 0; i < N; ++i) out[i] = v[i].asNumber();
}

template <std::size_t R, std::size_t C>
Mat<R, C> matrix(const core::Json& a) {
    const auto& v = a.asArray();
    REQUIRE(v.size() == R * C);
    Mat<R, C> m;
    for (std::size_t i = 0; i < R * C; ++i) m.a[i] = v[i].asNumber();
    return m;
}

RadarMeasurement model(const core::Json& j) {
    RadarMeasurement m;
    m.rotation = matrix<3, 3>(j.child("rotation"));
    numbers(j.child("translation"), m.translation);
    numbers(j.child("velocity"), m.velocity);
    double noise[4];
    numbers(j.child("noise"), noise);
    for (std::size_t i = 0; i < 4; ++i) m.noise(i, i) = noise[i];
    return m;
}

TrackerSettings settings(const core::Json& s) {
    TrackerSettings t;
    t.filter = s.string("filter", "ekf") == "ukf" ? Filter::Ukf : Filter::Ekf;
    numbers(s.child("q"), t.motion.q);
    t.prior.x = matrix<6, 1>(s.child("prior").child("x"));
    t.prior.p = matrix<6, 6>(s.child("prior").child("p"));
    t.prior.t = s.child("prior").number("t", 0.0);
    return t;
}

/// The worst difference between two Gaussians, relative to the larger of a value and the state's spread there.
double worst(const Gaussian& g, const State& x, const Covar& p) {
    double w = 0.0;
    for (std::size_t i = 0; i < 6; ++i) w = std::max(w, std::fabs(g.x.a[i] - x.a[i]) / std::max(1.0, std::sqrt(std::fabs(p(i, i)))));
    for (std::size_t i = 0; i < 36; ++i) w = std::max(w, std::fabs(g.p.a[i] - p.a[i]) / std::max(1.0, std::fabs(p.a[i])));
    return w;
}

} // namespace

TEST_CASE("trackers against Stone Soup: one target's EKF and UKF posteriors, step by step", "[ms][track]") {
    const auto doc = fixture();
    INFO("Stone Soup " << doc.string("stonesoup", "?"));
    int checked = 0;
    for (const auto& s : doc.child("scenarios").asArray()) {
        if (s.has("kind")) continue;
        INFO(s.string("name", ""));
        const TrackerSettings ts = settings(s);
        Gaussian g = ts.prior;
        double most = 0.0;
        for (const auto& step : s.child("steps").asArray()) {
            const RadarMeasurement m = model(step.child("model"));
            const Meas z = matrix<4, 1>(step.child("z"));
            const Gaussian prediction = predict(g, step.number("t", 0.0), ts.motion);
            REQUIRE(update(prediction, predictMeasurement(prediction, m, ts.filter), z, g));
            most = std::max(most, worst(g, matrix<6, 1>(step.child("posterior").child("x")), matrix<6, 6>(step.child("posterior").child("p"))));
            ++checked;
        }
        WARN(s.string("name", "") << ": the C++ posterior within " << most << " (of a standard deviation, or relative) of Stone Soup's");
        CHECK(most < 1e-6);
    }
    CHECK(checked >= 50);
}

TEST_CASE("trackers against Stone Soup: GNN and JPDA multi-target trackers, their tracks at every step", "[ms][track]") {
    const auto doc = fixture();
    int compared = 0;
    for (const auto& s : doc.child("scenarios").asArray()) {
        if (!s.has("kind")) continue;
        INFO(s.string("name", ""));
        TrackerSettings ts = settings(s);
        ts.association = s.string("kind", "") == "jpda" ? Association::Jpda : Association::Gnn;
        ts.missedDistance = ts.initiatorMissedDistance = s.number("missed_distance", 3.0);
        ts.pd = s.number("pd", 0.9);
        ts.pg = s.number("pg", 0.95);
        ts.clutterDensity = s.number("clutter_density", -1.0);
        ts.minPoints = static_cast<int>(s.number("min_points", 2));
        ts.initiatorDeleter.steps = static_cast<int>(s.number("initiator_steps", 2));
        ts.deleter.steps = static_cast<int>(s.number("deleter_steps", 3));
        MultiTargetTracker tracker(ts);
        double most = 0.0;
        for (const auto& step : s.child("steps").asArray()) {
            const double t = step.number("t", 0.0);
            const RadarMeasurement m = model(step.child("model"));
            std::vector<Detection> dets;
            for (const auto& d : step.child("detections").asArray()) {
                Detection det;
                det.t = t;
                det.z = matrix<4, 1>(d.child("z"));
                det.model = m;
                det.id = dets.size() + 1;
                det.truth = static_cast<std::uint32_t>(d.number("truth", 0));
                dets.push_back(det);
            }
            tracker.step(t, dets);
            const auto& expected = step.child("tracks").asArray();
            INFO("t " << t << ": " << tracker.tracks().size() << " tracks, Stone Soup's " << expected.size());
            REQUIRE(tracker.tracks().size() == expected.size());
            // each of Stone Soup's tracks (a set: no order) matched to the nearest of ours
            std::vector<char> used(tracker.tracks().size(), 0);
            for (const auto& e : expected) {
                const State x = matrix<6, 1>(e.child("x"));
                const Covar p = matrix<6, 6>(e.child("p"));
                std::size_t best = 0;
                double bestW = 1e300;
                for (std::size_t k = 0; k < tracker.tracks().size(); ++k) {
                    if (used[k]) continue;
                    const double w = worst(tracker.tracks()[k].state(), x, p);
                    if (w < bestW) bestW = w, best = k;
                }
                used[best] = 1;
                most = std::max(most, bestW);
                ++compared;
            }
        }
        WARN(s.string("name", "") << ": every track within " << most << " of Stone Soup's");
        CHECK(most < 1e-6);
    }
    CHECK(compared > 200);
}

TEST_CASE("trackers: SciPy's assignment, the gate's chi-square quantile, JPDA's clusters", "[ms][track]") {
    // a small problem with a known optimum (each row's best column taken by another)
    const double inf = std::numeric_limits<double>::infinity();
    const std::vector<double> cost = {4, 1, 3, 2, 0, 5, 3, 2, 2};
    const auto a = assign(cost, 3, 3);
    REQUIRE(a.size() == 3);
    CHECK(a[0] == 1);
    CHECK(a[1] == 0);
    CHECK(a[2] == 2);
    // rectangular, with forbidden pairs
    const std::vector<double> rect = {inf, 1, 9, 9, 2, inf, 9, 9};
    const auto b = assign(rect, 2, 4);
    REQUIRE(b.size() == 2);
    CHECK(b[0] == 1);
    CHECK(b[1] == 0);
    CHECK(assign({inf, inf}, 1, 2).empty()); // (infeasible)
    // scipy.stats.chi2.ppf(p, 4)
    CHECK_THAT(chiSquare4Quantile(0.95), Catch::Matchers::WithinAbs(9.487729036781154, 1e-9));
    CHECK_THAT(chiSquare4Quantile(0.99), Catch::Matchers::WithinAbs(13.276704135987622, 1e-9));
    // JPDA: two tracks sharing a detection and a third on its own - the third's marginals are its own hypotheses'
    std::vector<std::vector<Hypothesis>> h(3);
    auto add = [&](std::size_t k, int det, double p) {
        Hypothesis x;
        x.detection = det;
        x.logProbability = std::log(p);
        h[k].push_back(x);
    };
    add(0, -1, 0.1), add(0, 0, 0.6), add(0, 1, 0.3);
    add(1, -1, 0.2), add(1, 0, 0.8);
    add(2, -1, 0.25), add(2, 2, 0.75);
    const auto m = associateJpda(h, 3);
    // tracks 0 and 1: events (m,m) .02, (m,0) .08, (0,m) .12, (1,m) .06, (1,0) .24 - total .52
    CHECK_THAT(m[0][0], Catch::Matchers::WithinAbs((0.02 + 0.08) / 0.52, 1e-12));
    CHECK_THAT(m[0][1], Catch::Matchers::WithinAbs(0.12 / 0.52, 1e-12));
    CHECK_THAT(m[1][1], Catch::Matchers::WithinAbs((0.08 + 0.24) / 0.52, 1e-12));
    CHECK_THAT(m[2][0], Catch::Matchers::WithinAbs(0.25, 1e-12));
    CHECK_THAT(m[2][1], Catch::Matchers::WithinAbs(0.75, 1e-12));
}
