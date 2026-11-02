// The trackers (ADR-30, MS-5) against Stone Soup itself: the measurements,
// sensor poses and answers tools/validation/track/crosscheck.py had Stone Soup 1.9.1
// write (tests/simulation/data/track/crosscheck.json), replayed through the C++ port -
// one target's EKF and UKF posteriors step by step, and the GNN and JPDA
// multi-target trackers' tracks at every step (their number, each track's
// state and covariance) through crossing targets and false alarms. Then the
// pieces on their own: SciPy's assignment, the chi-square quantile, the
// clusters JPDA factorises over.
#include "foundation/core/Json.h"
#include "simulation/mission_systems/track/Assignment.h"
#include "simulation/mission_systems/track/Tracker.h"
#include "simulation/mission_systems/track/Fusion.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::ms::track;

TEST_CASE("track IMM: identity modes preserve the ordinary UKF and coast covariance", "[ms][track][imm]") {
    Gaussian initial;initial.x.a={10000,100,2000,-20,3000,5};initial.p=100.0*identity<6>();
    ConstantVelocity motion;for(double& q:motion.q)q=2;
    Imm imm(initial,motion,motion,5);
    RadarMeasurement measurement;measurement.noise=identity<4>();
    imm.predictTo(1.0);
    const auto single=predict(initial,1.0,motion);
    for(std::size_t i=0;i<36;++i) CHECK(std::fabs(imm.state().p.a[i]-single.p.a[i])<1e-10);
    const Meas z=measurement.function(single.x);
    Gaussian expected;REQUIRE(update(single,predictMeasurement(single,measurement,Filter::Ukf),z,expected));
    REQUIRE(imm.observe(z,measurement,Filter::Ukf));
    for(std::size_t i=0;i<6;++i) CHECK(std::fabs(imm.state().x.a[i]-expected.x.a[i])<1e-10);
    for(std::size_t i=0;i<36;++i) CHECK(std::fabs(imm.state().p.a[i]-expected.p.a[i])<1e-10);
    CHECK(imm.probabilities()[0]==.5);
    CHECK_THROWS_AS(imm.predictTo(.5),std::invalid_argument);
}

TEST_CASE("track covariance: nonlinear updates preserve covariance symmetry in Earth coordinates", "[ms][track][sim-m01]") {
    Gaussian g;g.x.a={-4000000,100,4000000,50,2500000,-20};g.p=100.*identity<6>();
    ConstantVelocity motion;for(double& q:motion.q)q=100;
    RadarMeasurement sensor;sensor.translation[0]=-4010000;sensor.translation[1]=3997000;sensor.translation[2]=2498500;
    sensor.noise(0,0)=sensor.noise(1,1)=1e-9;sensor.noise(2,2)=sensor.noise(3,3)=1;
    for(int step=1;step<=50;++step) {
        const auto prior=predict(g,.05*step,motion);auto z=sensor.function(prior.x);
        z.a[0]+=1e-5*std::sin(step);z.a[1]+=1e-5*std::cos(step);z.a[2]+=.1*std::sin(step);
        REQUIRE(update(prior,predictMeasurement(prior,sensor,Filter::Ukf),z,g));
        for(std::size_t i=0;i<6;++i)for(std::size_t j=0;j<i;++j)REQUIRE(g.p(i,j)==g.p(j,i));
    }
}

TEST_CASE("track fusion V1: duplicate knowledge cannot manufacture information and loss retains identity", "[ms][track][fusion][sim-m01]") {
    Fusion fusion;Fusion::Observation a;
    a.source.track=1;a.source.system=10;a.source.sampleTimeS=a.source.receivedTimeS=1;
    a.estimate.t=1;a.estimate.x.a={10000,100,2000,0,3000,0};a.estimate.p=100.*identity<6>();
    auto b=a;b.source.kind=ms::FusionSourceKindV1::DeliveredLink;b.source.system=20;b.source.diagonalLinkCovariance=true;
    fusion.advance(1,{a,b});const auto first=fusion.snapshot();REQUIRE(first.size()==1);
    CHECK(first[0].sources.size()==2);CHECK_FALSE(first[0].identityKnown);
    for(std::size_t i=0;i<6;++i)CHECK(std::fabs(first[0].covarianceEcef[i*6+i]-100)<1e-10);
    fusion.advance(1,{a,b});const auto duplicate=fusion.snapshot();CHECK(duplicate[0].id==first[0].id);
    for(std::size_t i=0;i<36;++i)CHECK(duplicate[0].covarianceEcef[i]==first[0].covarianceEcef[i]);
    fusion.advance(14,{});const auto lost=fusion.snapshot();REQUIRE(lost.size()==1);
    CHECK(lost[0].state==ms::FusionStateV1::Lost);CHECK(lost[0].id==first[0].id);
    CHECK(lost[0].lastUpdateS==1);CHECK(lost[0].sources[1].receivedTimeS==1);
    auto future=a;future.source.sampleTimeS=future.source.receivedTimeS=15;future.estimate.t=15;
    fusion.advance(14,{future});CHECK(fusion.rejected()==1);
}

TEST_CASE("track fusion V1: distinct same-sensor tracks stay distinct and capacity refuses explicitly", "[ms][track][fusion][sim-m01]") {
    Fusion::Settings settings;settings.capacity=2;Fusion fusion(settings);
    Fusion::Observation a;a.source.track=1;a.source.sampleTimeS=a.source.receivedTimeS=1;
    a.estimate.t=1;a.estimate.x.a={10000,0,2000,0,3000,0};a.estimate.p=identity<6>();
    auto b=a;b.source.track=2;b.estimate.x.a[0]+=.1;
    auto c=b;c.source.track=3;c.estimate.x.a[0]+=.1;
    fusion.advance(1,{a,b,c});CHECK(fusion.snapshot().size()==2);CHECK(fusion.capacityRefusals()==1);
    auto invalid=a;invalid.estimate.p(0,1)=std::numeric_limits<double>::quiet_NaN();
    fusion.advance(1,{invalid});CHECK(fusion.rejected()==1);
}

TEST_CASE("track IMM: a maneuver changes model probability and empty looks retain finite estimates", "[ms][track][imm]") {
    Gaussian initial;initial.x.a={10000,0,1000,0,3000,0};initial.p=identity<6>();
    ConstantVelocity quiet,maneuver;for(double& q:quiet.q)q=.01;for(double& q:maneuver.q)q=100;
    Imm imm(initial,quiet,maneuver,10);
    RadarMeasurement sensor;sensor.noise=.000001*identity<4>();sensor.noise(2,2)=1;sensor.noise(3,3)=1;
    auto truth=initial.x;
    for(int i=1;i<=4;++i) {
        truth.a[0]=10000+10*i*i;truth.a[1]=20*i;
        imm.predictTo(i);REQUIRE(imm.observe(sensor.function(truth),sensor,Filter::Ukf));
    }
    CHECK(imm.probabilities()[1]>.5);
    const double before=trace(imm.state().p);imm.predictTo(5);
    CHECK(trace(imm.state().p)>before);
    for(double p:imm.probabilities()) CHECK(std::isfinite(p));
    CHECK_THROWS_AS(Imm(initial,quiet,maneuver,0),std::invalid_argument);
}

TEST_CASE("track fusion V1: expired retention releases capacity before accepting this tick", "[ms][track][fusion][sim-m01]") {
    Fusion::Settings settings;settings.capacity=1;settings.maximumAgeS=1;Fusion fusion(settings);
    Fusion::Observation a;a.source.track=1;a.source.sampleTimeS=a.source.receivedTimeS=1;
    a.estimate.t=1;a.estimate.x.a={10000,0,2000,0,3000,0};a.estimate.p=identity<6>();
    fusion.advance(1,{a});const auto first=fusion.snapshot();REQUIRE(first.size()==1);
    auto b=a;b.source.track=2;b.source.sampleTimeS=b.source.receivedTimeS=b.estimate.t=3.01;
    fusion.advance(3.01,{b});const auto current=fusion.snapshot();
    REQUIRE(current.size()==1);CHECK(current[0].id!=first[0].id);
    CHECK(current[0].sources.front().track==2);CHECK(fusion.capacityRefusals()==0);
}

TEST_CASE("track fusion V1: stale contributors cannot permanently exhaust a live entity", "[ms][track][fusion][sim-m01]") {
    Fusion::Settings settings;settings.sourcesPerEntity=2;settings.maximumAgeS=1;settings.processNoise=0;
    Fusion fusion(settings);Fusion::Observation a;
    a.source.track=1;a.source.system=1;a.source.sampleTimeS=a.source.receivedTimeS=a.estimate.t=1;
    a.estimate.x.a={10000,0,2000,0,3000,0};a.estimate.p=identity<6>();
    auto b=a;b.source.system=2;fusion.advance(1,{a,b});
    const auto id=fusion.snapshot().front().id;
    a.source.sampleTimeS=a.source.receivedTimeS=a.estimate.t=1.75;fusion.advance(1.75,{a});
    auto c=a;c.source.system=3;c.source.sampleTimeS=c.source.receivedTimeS=c.estimate.t=2.25;
    fusion.advance(2.25,{c});const auto current=fusion.snapshot();REQUIRE(current.size()==1);
    CHECK(current[0].id==id);CHECK(current[0].lastUpdateS==2.25);
    REQUIRE(current[0].sources.size()==2);CHECK(current[0].sources[0].system==1);
    CHECK(current[0].sources[1].system==3);CHECK(fusion.capacityRefusals()==0);
    fusion.advance(2.25,{b});CHECK(fusion.rejected()==1); // stale replay cannot reclaim its slot
}

TEST_CASE("track fusion V1: sample age governs association even after delayed delivery", "[ms][track][fusion][sim-m01]") {
    Fusion::Settings settings;settings.maximumAgeS=1;settings.processNoise=0;Fusion fusion(settings);
    Fusion::Observation a;a.source.track=1;a.source.sampleTimeS=1;a.source.receivedTimeS=a.estimate.t=1.9;
    a.estimate.x.a={10000,0,2000,0,3000,0};a.estimate.p=identity<6>();
    fusion.advance(1.9,{a});const auto id=fusion.snapshot().front().id;
    auto b=a;b.source.kind=ms::FusionSourceKindV1::DeliveredLink;b.source.system=2;
    b.source.sampleTimeS=b.source.receivedTimeS=b.estimate.t=2.1;
    fusion.advance(2.1,{b});const auto current=fusion.snapshot();REQUIRE(current.size()==2);
    CHECK(current[0].id==id);CHECK(current[0].state==ms::FusionStateV1::Lost);
    CHECK(current[1].id!=id);CHECK(current[1].state==ms::FusionStateV1::Live);
}

namespace {

namespace fs = std::filesystem;

const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();

core::Json fixture() {
    std::ifstream in(kSource / "tests" / "simulation" / "data" / "track" / "crosscheck.json");
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

TEST_CASE("track IMM: nonlinear posteriors against independently generated FilterPy1.4.5", "[ms][track][imm]") {
    std::ifstream in(kSource/"tests/simulation/data/track/imm-crosscheck-v3.json");REQUIRE(in.good());
    std::stringstream text;text<<in.rdbuf();const auto doc=core::Json::parse(text.str());
    Gaussian g;g.x=matrix<6,1>(doc.child("initial").child("x"));g.p=matrix<6,6>(doc.child("initial").child("p"));
    ConstantVelocity quiet,maneuver;
    for(double& q:quiet.q)q=doc.child("process_noise").asArray()[0].asNumber();
    for(double& q:maneuver.q)q=doc.child("process_noise").asArray()[1].asNumber();
    Imm imm(g,quiet,maneuver,doc.number("sojourn_s",0));
    RadarMeasurement sensor;sensor.noise=matrix<4,4>(doc.child("measurement_noise"));
    int checked=0;
    for(const auto& step:doc.child("steps").asArray()) {
        INFO("time "<<step.number("t",0));imm.predictTo(step.number("t",0));
        REQUIRE(imm.observe(matrix<4,1>(step.child("z")),sensor,Filter::Ukf));
        const auto x=matrix<6,1>(step.child("x"));const auto p=matrix<6,6>(step.child("p"));
        for(std::size_t i=0;i<6;++i) CHECK(std::fabs(imm.state().x.a[i]-x.a[i])<=1e-8);
        for(std::size_t i=0;i<36;++i) CHECK(std::fabs(imm.state().p.a[i]-p.a[i])<=1e-7);
        for(std::size_t i=0;i<2;++i) CHECK(std::fabs(imm.probabilities()[i]-step.child("probabilities").asArray()[i].asNumber())<=1e-10);
        ++checked;
    }
    CHECK(checked==5);
}

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

TEST_CASE("track fusion V1: covariance intersection against Stone Soup 1.9.1", "[ms][track][fusion][sim-m01]") {
    std::ifstream in(kSource/"tests/simulation/data/track/ci-crosscheck-v1.json");REQUIRE(in.good());
    std::stringstream text;text<<in.rdbuf();const auto doc=core::Json::parse(text.str());
    REQUIRE(doc.string("reference_version","")=="1.9.1");
    int count=0;
    for(const auto& row:doc.child("cases").asArray()) {
        INFO(row.string("name",""));Gaussian a,b,result;
        const auto& inputs=row.child("inputs").asArray();REQUIRE(inputs.size()==2);
        a.x=matrix<6,1>(inputs[0].child("x"));a.p=matrix<6,6>(inputs[0].child("p"));
        b.x=matrix<6,1>(inputs[1].child("x"));b.p=matrix<6,6>(inputs[1].child("p"));
        REQUIRE(intersect(a,b,result));
        const auto x=matrix<6,1>(row.child("x"));const auto p=matrix<6,6>(row.child("p"));
        for(std::size_t i=0;i<6;++i)CHECK_THAT(result.x.a[i],Catch::Matchers::WithinAbs(x.a[i],1e-7));
        for(std::size_t i=0;i<36;++i)CHECK_THAT(result.p.a[i],Catch::Matchers::WithinAbs(p.a[i],1e-9));
        CHECK(std::isfinite(logDet(result.p)));++count;
    }
    CHECK(count==12);
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
