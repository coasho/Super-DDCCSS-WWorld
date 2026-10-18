// Air search radars (ADR-30, MS-4): the probability of detection against its
// closed forms, the PRF schedule's M of N, the radar equation, every radar
// card against the ranges published for it (checked, never fitted: each
// class's calibration leaves its mean residual at zero), the pulse-Doppler
// clutter (the main beam's notch, the side lobes' band), and a search
// through the World - commanded, detecting, ending with its payload, and
// the same to the bit from the same seed.
#include "mode_flights.h"

#include "foundation/core/Attitude.h"
#include "foundation/core/Json.h"
#include "foundation/transport/WorldMirror.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/radar/Detection.h"
#include "simulation/mission_systems/radar/Radar.h"
#include "simulation/mission_systems/track/Tracker.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace radar = fsim::ms::radar;

namespace {

namespace fs = std::filesystem;

const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path(); // (third_party/jsbsim -> the source tree)

std::shared_ptr<const ms::Card> card(const std::string& id) {
    std::vector<std::string> errors;
    auto c = ms::readCard(kSource / "assets" / "payloads" / (id + ".json"), errors);
    REQUIRE(c != nullptr);
    return c;
}

double db(double x) { return 10.0 * std::log10(x); }

/// A scene of fixed targets: each its state, one RCS for all, free space between.
struct FixedScene final : radar::Scene {
    const sim::VehicleState* own = nullptr;
    std::vector<std::uint32_t> ids;
    std::vector<sim::VehicleState> states;
    double sigmaM2 = 3.0, fluctuation = 1.0;
    bool ground = true; ///< (false: free space - no clutter at the signal level)
    bool hasGround() const override { return ground; }
    std::size_t targets(std::uint32_t* out, const sim::VehicleState** s, std::size_t max) const override {
        std::size_t n = 0;
        for (; n < ids.size() && n < max; ++n) out[n] = ids[n], s[n] = &states[n];
        return n;
    }
    double sigma(std::uint32_t, double, double, double& f) const override { return f = fluctuation, sigmaM2; }
    double pathLossDb(std::uint32_t target, double frequencyHz) const override {
        const sim::VehicleState& t = states[target - 1];
        const double d = std::sqrt(std::pow(t.positionEcef[0] - own->positionEcef[0], 2) + std::pow(t.positionEcef[1] - own->positionEcef[1], 2) +
                                   std::pow(t.positionEcef[2] - own->positionEcef[2], 2));
        return 20.0 * std::log10(4.0 * kPi * d * frequencyHz / 299792458.0);
    }
};

/// The highest SNR among a target's detections, and how many there were.
std::pair<double, int> best(const radar::Radar& r, std::uint32_t truth) {
    double top = -1e9;
    int n = 0;
    for (const auto& d : r.detections())
        if (d.truth == truth) top = std::max(top, d.snrDb), ++n;
    return {top, n};
}

} // namespace

TEST_CASE("radar detection: one look's Pd against its closed forms", "[ms][radar]") {
    // Marcum's Q at its known points: Q1(0, b) = exp(-b^2/2); Q1(a, 0) = 1
    CHECK_THAT(radar::marcumQ(0.0, 2.0), WithinAbs(std::exp(-2.0), 1e-12));
    CHECK_THAT(radar::marcumQ(3.0, 0.0), WithinAbs(1.0, 1e-12));
    // no signal: the false-alarm rate
    for (auto f : {radar::Fluctuation::Steady, radar::Fluctuation::Rayleigh, radar::Fluctuation::DominantScatterer})
        CHECK_THAT(radar::probabilityOfDetection(0.0, 1e-6, f), WithinRel(1e-6, 1e-6));
    // Pd 0.9 at Pfa 1e-6 (Meyer and Mayer; Richards, table 6.1): steady 13.2 dB, Rayleigh 21.1 dB, between them a
    // dominant scatterer's 17.3 dB
    CHECK_THAT(db(radar::requiredSnr(0.9, 1e-6, radar::Fluctuation::Steady)), WithinAbs(13.2, 0.1));
    CHECK_THAT(db(radar::requiredSnr(0.9, 1e-6, radar::Fluctuation::Rayleigh)), WithinAbs(21.1, 0.1));
    CHECK_THAT(db(radar::requiredSnr(0.9, 1e-6, radar::Fluctuation::DominantScatterer)), WithinAbs(17.3, 0.1));
    // Pd 0.5: steady 11.2 dB, Rayleigh 12.8 dB
    CHECK_THAT(db(radar::requiredSnr(0.5, 1e-6, radar::Fluctuation::Steady)), WithinAbs(11.2, 0.1));
    CHECK_THAT(db(radar::requiredSnr(0.5, 1e-6, radar::Fluctuation::Rayleigh)), WithinAbs(12.8, 0.1));
    // the table's fluctuation ratio to its case
    CHECK(radar::fluctuationOf(1.0) == radar::Fluctuation::Steady);
    CHECK(radar::fluctuationOf(1.5) == radar::Fluctuation::DominantScatterer);
    CHECK(radar::fluctuationOf(2.0) == radar::Fluctuation::Rayleigh);
}

TEST_CASE("radar detection: the PRF schedule's M of N keeps the dwell's false-alarm rate", "[ms][radar]") {
    const radar::Spec spec = radar::specOf(*card("an-apg-68v9"));
    REQUIRE(spec.ok);
    const radar::Radar r(spec, 1);
    for (const auto mode : {radar::Mode::RangeWhileSearch, radar::Mode::VelocitySearch}) {
        // the dwell's threshold: one false alarm in the card's false-alarm time among its cells
        const double pfa = r.dwellPfa(mode, r.nominalDwellS(), 150e3, 250.0);
        CHECK_THAT(pfa * r.resolutionCells(mode, r.nominalDwellS(), 150e3, 250.0), WithinRel(r.nominalDwellS() / spec.falseAlarmTimeS, 1e-12));
        CHECK(pfa < 1e-8); // (some 10^6 cells a dwell, some 30 dwells a second)
        // no signal: that rate, whatever the schedule
        CHECK_THAT(r.dwellPd(-60.0, radar::Fluctuation::Rayleigh, mode, pfa), WithinRel(pfa, 0.02));
        // more signal, more detection
        double last = 0.0;
        for (double snr = 0.0; snr <= 40.0; snr += 2.0) {
            const double pd = r.dwellPd(snr, radar::Fluctuation::Rayleigh, mode, pfa);
            CHECK(pd >= last);
            last = pd;
        }
        CHECK(last > 0.99);
    }
    // splitting the energy over 8 PRFs costs: 3 of 8 needs more than a single look at the same false-alarm rate on a
    // steady target, Pd 0.5
    const double pfa = r.dwellPfa(radar::Mode::RangeWhileSearch, r.nominalDwellS(), 150e3, 250.0);
    const double single = db(radar::requiredSnr(0.5, pfa, radar::Fluctuation::Steady));
    double lo = 0.0, hi = 40.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (r.dwellPd(mid, radar::Fluctuation::Steady, radar::Mode::RangeWhileSearch, pfa) < 0.5 ? lo : hi) = mid;
    }
    INFO("3 of 8: " << hi << " dB, one look: " << single << " dB");
    CHECK(hi > single);
    CHECK(hi < single + 7.0);
}

TEST_CASE("radar equation: SNR from its terms, the range a fourth power of the RCS", "[ms][radar]") {
    radar::Spec spec;
    spec.frequencyHz = 10e9;
    spec.aperture.shape = ms::rf::ApertureShape::Circular;
    spec.aperture.widthM = spec.aperture.heightM = 0.7;
    spec.aperture.peakGainDbi = 35.0;
    spec.averagePowerW = 1000.0;
    spec.noiseFigureDb = 3.0;
    spec.systemLossDb = 8.0;
    spec.aperture.electronic = true;
    spec.dwellS = 0.01;
    spec.ok = true;
    const radar::Radar r(spec, 1);
    REQUIRE_THAT(r.pattern().peakGainDbi(), WithinAbs(35.0, 1e-9));
    // by hand: 30 + 70 + 10 log(4 pi 1 / 0.03^2) - 2 x 20 log(4 pi 1e5 / 0.03) - 20 + 203.98 - 3 - 8
    const double lambda = 299792458.0 / 10e9;
    const double want = 30.0 + 70.0 + db(4 * kPi / (lambda * lambda)) - 40.0 * std::log10(4 * kPi * 1e5 / lambda) - 20.0 + 203.977 - 11.0;
    CHECK_THAT(r.snrDb(1.0, 100e3), WithinAbs(want, 0.01));
    // 16 times the RCS, twice the range
    CHECK_THAT(r.detectionRangeM(16.0, 0.5, 1) / r.detectionRangeM(1.0, 0.5, 1), WithinRel(2.0, 1e-6));
    // the fluctuation costs range at a high Pd: a steady target is seen further than a Rayleigh one
    CHECK(r.detectionRangeM(1.0, 0.9, 0) > r.detectionRangeM(1.0, 0.9, 1));
}

TEST_CASE("radar cards: each against its published ranges, every class calibrated alike", "[ms][radar]") {
    // every radar card describes a radar; every published range is a check (free space, the beam's axis, Pd 0.5 on a
    // Rayleigh target), never fitted - only each class's mean residual is taken out, the same for every radar of it
    std::map<double, std::vector<double>> byClass; // (the class's calibration identifies it)
    int checks = 0;
    for (const auto& file : fs::directory_iterator(kSource / "assets" / "payloads")) {
        if (file.path().extension() != ".json") continue;
        std::vector<std::string> errors;
        const auto c = ms::readCard(file.path(), errors);
        if (!c || c->family != ms::Family::AirRadar) continue;
        INFO(c->id);
        const radar::Spec spec = radar::specOf(*c);
        REQUIRE(spec.ok);
        const radar::Radar r(spec, 1);
        std::ifstream in(file.path());
        std::stringstream text;
        text << in.rdbuf();
        const auto doc = core::Json::parse(text.str());
        if (!doc.has("validation")) continue;
        for (const auto& v : doc.child("validation").asArray()) {
            const double got = r.detectionRangeM(v.number("rcs_m2", 3.0), 0.5, 1), published = v.number("range_m", 0.0);
            REQUIRE(published > 0.0);
            const double residual = 40.0 * std::log10(got / published);
            INFO("published " << published / 1e3 << " km, modelled " << got / 1e3 << " km: " << residual << " dB");
            if (v.string("note", "").find("pursuit") != std::string::npos) continue; // (clutter-limited: the clutter test's)
            CHECK(std::fabs(residual) < 12.0); // (the published claims' own scatter: none of them fitted)
            byClass[spec.classCalibrationDb].push_back(residual);
            ++checks;
        }
    }
    CHECK(checks >= 12);
    for (const auto& [cal, residuals] : byClass) {
        double mean = 0.0;
        for (double x : residuals) mean += x / static_cast<double>(residuals.size());
        INFO("class calibration " << cal << " dB: mean residual " << mean << " dB over " << residuals.size());
        CHECK(std::fabs(mean) < 0.3);
    }
}

TEST_CASE("radar false alarms: one in its false-alarm time, whatever its mode's cells", "[ms][radar]") {
    // a threshold set as a designer sets it (Stimson: a fighter radar's false-alarm time, a minute or so): searching
    // empty sky, its false alarms come one in that time - a long range scale's or velocity search's cells alike
    session::World w(options("radar-false-alarms"));
    const auto own = wing(w, "f16c", 5000.0, 250.0);
    const sim::VehicleState self = *w.vehicleState(own);
    for (const char* id : {"an-apg-68v9", "an-apg-77", "an-apy-2"}) {
        const radar::Spec spec = radar::specOf(*card(id));
        REQUIRE(spec.ok);
        CHECK(spec.falseAlarmTimeS == 60.0);
        FixedScene scene;
        scene.own = &self;
        for (const double scale : {40e3, 150e3}) {
            for (const auto mode : {radar::Mode::RangeWhileSearch, radar::Mode::VelocitySearch}) {
                if (spec.rotationRpm > 0.0 && mode == radar::Mode::VelocitySearch) continue; // (a rotodome's surveillance)
                radar::Radar r(spec, 11);
                radar::Search s;
                s.mode = mode;
                s.rangeScaleM = scale;
                r.search(s, 0.0);
                const double seconds = 3600.0;
                for (double t = 1.0; t <= seconds; t += 1.0) r.step(t, self, scene); // (a step at most 100000 dwells)
                const double expected = seconds / spec.falseAlarmTimeS;
                const auto got = static_cast<double>(r.falseAlarmCount());
                INFO(id << " " << scale / 1e3 << " km, mode " << static_cast<int>(mode) << ": " << got << " false alarms in an hour, expected " << expected);
                CHECK(std::fabs(got - expected) < 4.0 * std::sqrt(expected));
            }
        }
    }
}

TEST_CASE("radar clutter: the main beam's notch and the side lobes' band", "[ms][radar]") {
    session::World w(options("radar-clutter"));
    const auto own = wing(w, "f16c", 5000.0, 250.0);
    // a target where it is put, its velocity set apart from it
    auto at = [&](double northKm, double eastKm, double altitudeM, int index) {
        session::VehicleSpec s;
        s.name = "t" + std::to_string(index);
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg += northKm * 1e3 / kEarthM / kDeg;
        s.initial.longitudeDeg += eastKm * 1e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
        s.initial.altitudeMslM = altitudeM;
        s.initial.headingDeg = 90.0;
        s.initial.airspeedTrueMs = 200.0;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        return *w.vehicleState(id);
    };
    const sim::VehicleState self = *w.vehicleState(own);
    const radar::Spec spec = radar::specOf(*card("n001"));
    REQUIRE(spec.ok);

    SECTION("a slow target meets the side lobes' clutter: its SNR down by the card's penalty, gone in velocity search") {
        FixedScene scene;
        scene.own = &self;
        scene.ids = {1, 2};
        scene.states = {at(0.5, 20.0, 5000.0, 1), at(0.5, 20.0, 5000.0, 2)}; // (one place: the same beam positions, the same gains)
        for (int i = 0; i < 3; ++i) scene.states[0].velocityNedMs[i] = -self.velocityNedMs[i]; // 1 head on: closing at twice its speed
        for (int i = 0; i < 3; ++i) scene.states[1].velocityNedMs[i] = self.velocityNedMs[i];  // 2 in pursuit: the same speed
        // (stepped a dwell or so at a time: the scene's places are the step's, and a dwell takes each back along its
        // velocity to its own time - a still scene looked at across 30 s in one step would not be the same place twice)
        auto look = [&](radar::Radar& radar) {
            for (double t = 0.05; t <= 30.0 + 1e-9; t += 0.05) radar.step(t, self, scene);
        };
        radar::Radar r(spec, 7);
        radar::Search s;
        s.azimuthHalfWidthRad = 20 * kDeg;
        s.bars = 1;
        r.search(s, 0.0);
        look(r);
        const auto [headOn, nHead] = best(r, 1);
        const auto [pursuit, nPursuit] = best(r, 2);
        REQUIRE(nHead > 0);
        REQUIRE(nPursuit > 0);
        CHECK_THAT(headOn - pursuit, WithinAbs(spec.sidelobeClutterDb, 0.2));
        // velocity search: only the closing target
        radar::Radar v(spec, 7);
        s.mode = radar::Mode::VelocitySearch;
        v.search(s, 0.0);
        look(v);
        CHECK(best(v, 1).second > 0);
        CHECK(best(v, 2).second == 0);
    }
    SECTION("a target hovering below, in the main beam's clutter, is notched; above, it is seen") {
        FixedScene scene;
        scene.own = &self;
        scene.ids = {1, 2};
        scene.states = {at(0.0, 15.0, 1000.0, 1), at(0.0, 15.0, 9000.0, 2)};
        for (auto& t : scene.states) std::fill(std::begin(t.velocityNedMs), std::end(t.velocityNedMs), 0.0);
        radar::Radar r(spec, 7);
        radar::Search look;
        look.azimuthHalfWidthRad = 10 * kDeg;
        look.bars = 1;
        look.rangeScaleM = 80e3;
        look.elevationCentreRad = std::atan2(-4000.0, 15e3); // down at it
        r.search(look, 0.0);
        r.step(30.0, self, scene);
        CHECK(best(r, 1).second == 0);
        radar::Radar up(spec, 7);
        look.elevationCentreRad = std::atan2(4000.0, 15e3);
        up.search(look, 0.0);
        up.step(30.0, self, scene);
        CHECK(best(up, 2).second > 0);
    }
}

namespace {

/// Two F-16Cs 40 km apart head on, the first searching with its radar for `seconds`: its detections.
std::vector<ms::AmtiDetection> search(session::World& w, std::uint32_t& own, std::uint32_t& target, double seconds) {
    own = wing(w, "f16c", 5000.0, 200.0);
    session::VehicleSpec s;
    s.name = "target";
    s.type = "jsbsim:f16c";
    s.initial.altitudeMslM = 5000.0;
    s.initial.headingDeg = 270.0;
    s.initial.airspeedTrueMs = 200.0;
    s.initial.longitudeDeg += 40e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
    target = w.createVehicle(s);
    REQUIRE(target != 0);
    REQUIRE(w.submit(own, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    for (int i = 0; i < 600 && w.missionSystems(own)->payloads()[0].state != ms::PayloadState::Operate; ++i) w.step();
    REQUIRE(w.missionSystems(own)->payloads()[0].state == ms::PayloadState::Operate);
    ms::AmtiCommand c;
    c.azimuthHalfWidthRad = 30 * kDeg;
    REQUIRE(w.submit(own, c).accepted());
    const double end = w.simTime() + seconds;
    while (w.simTime() < end) w.step();
    return w.amtiDetections(own, 0);
}

} // namespace

TEST_CASE("radar through the World: commanded, detecting, ending with its payload; the same from the same seed", "[ms][radar]") {
    {
        session::World w0(options("radar-refusals"));
        const auto first = wing(w0, "f16c", 3000.0, 200.0, 3);
        // refused while off; and what the radar cannot do
        CHECK(w0.submit(first, ms::AmtiCommand{}).reason == ms::Reason::PayloadOff);
        CHECK(w0.submit(first, ms::AmtiCommand{1}).reason == ms::Reason::NotSupported); // (its RWR)
        CHECK(w0.submit(first, ms::AmtiCommand{9}).reason == ms::Reason::UnknownPayload);
        REQUIRE(w0.submit(first, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
        for (int i = 0; i < 600 && w0.missionSystems(first)->payloads()[0].state != ms::PayloadState::Operate; ++i) w0.step();
        ms::AmtiCommand bad;
        bad.mode = ms::AmtiMode::Surveillance; // (a rotodome's)
        CHECK(w0.submit(first, bad).reason == ms::Reason::InvalidParameter);
        bad = {};
        bad.bars = 0;
        CHECK(w0.submit(first, bad).reason == ms::Reason::InvalidParameter);
    }
    session::World w(options("radar-world"));
    std::uint32_t own = 0, target = 0;

    const auto detections = search(w, own, target, 12.0);
    const ms::RadarStatus st = w.radarStatus(own, 0);
    CHECK(st.searching);
    CHECK(st.mode == ms::AmtiMode::RangeWhileSearch);
    CHECK(st.dwells > 0);
    CHECK(st.framePeriodS > 0.5);
    CHECK(st.framePeriodS < 12.0);
    CHECK(!st.electronic);
    CHECK(st.detections == detections.size());
    const ms::ActivityRecord* a = w.missionSystems(own)->activity(st.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == control::ActivityState::Active);
    // it sees the target head on, where it is; and false alarms at the card's rate (truth 0: anywhere in its beam,
    // range and range rates)
    REQUIRE(!detections.empty());
    const sim::VehicleState& o = *w.vehicleState(own);
    const sim::VehicleState& t = *w.vehicleState(target);
    std::vector<ms::AmtiDetection> real;
    std::size_t alarms = 0;
    for (const auto& d : detections) {
        if (d.truth == 0) {
            ++alarms;
            CHECK(d.rangeM <= 150e3);
            continue;
        }
        CHECK(d.truth == target);
        CHECK(std::fabs(d.azimuthRad) < 3 * kDeg);
        CHECK(d.rangeRateMs < -300.0); // (closing at some 400 m/s)
        CHECK(d.rangeM > 20e3);
        CHECK(d.rangeM < 41e3);
        real.push_back(d);
    }
    REQUIRE(!real.empty());
    // the expected false alarms: one in the card's false-alarm time, the time it searched
    const auto spec = ms::radar::specOf(*w.missionSystems(own)->card(0));
    const ms::radar::Radar model(spec, 1);
    const double expected = static_cast<double>(st.dwells) * model.nominalDwellS() / spec.falseAlarmTimeS;
    INFO("false alarms " << alarms << ", expected " << expected << " in " << st.dwells << " dwells");
    CHECK(std::fabs(static_cast<double>(alarms) - expected) < 4.0 * std::sqrt(expected) + 2.0);
    const auto& last = real.back();
    const double truthRange = std::sqrt(std::pow(t.positionEcef[0] - o.positionEcef[0], 2) + std::pow(t.positionEcef[1] - o.positionEcef[1], 2) +
                                        std::pow(t.positionEcef[2] - o.positionEcef[2], 2));
    CHECK(std::fabs(last.rangeM - truthRange) < 400.0 * (w.simTime() - last.timeS) + 300.0); // (it closed since that dwell)
    for (std::size_t i = 1; i < detections.size(); ++i) CHECK(detections[i].serial == detections[i - 1].serial + 1);
    // only what is new
    CHECK(w.amtiDetections(own, 0, detections.back().serial).size() == 1);
    CHECK(w.amtiDetections(own, 0, detections.back().serial + 1).empty());

    // the same seed, the same looks, to the bit
    session::World w2(options("radar-world-2"));
    std::uint32_t own2 = 0, target2 = 0;
    const auto again = search(w2, own2, target2, 12.0);
    REQUIRE(again.size() == detections.size());
    for (std::size_t i = 0; i < again.size(); ++i) {
        CHECK(again[i].timeS == detections[i].timeS);
        CHECK(again[i].rangeM == detections[i].rangeM);
        CHECK(again[i].azimuthRad == detections[i].azimuthRad);
    }

    // a new search supersedes; the payload to standby ends it
    ms::AmtiCommand narrow;
    narrow.azimuthHalfWidthRad = 10 * kDeg;
    narrow.mode = ms::AmtiMode::VelocitySearch;
    const auto next = w.submit(own, narrow);
    REQUIRE(next.accepted());
    CHECK(w.missionSystems(own)->activity(st.activity)->reason == ms::Reason::Superseded);
    CHECK(w.radarStatus(own, 0).mode == ms::AmtiMode::VelocitySearch);
    REQUIRE(w.submit(own, ms::StateCommand{0, ms::PayloadState::Standby}).accepted());
    const ms::ActivityRecord* ended = w.missionSystems(own)->activity(next.activity);
    CHECK(ended->state == control::ActivityState::Canceled);
    CHECK(ended->reason == ms::Reason::PayloadStandby);
    CHECK(!w.radarStatus(own, 0).searching);
    const auto dwells = w.radarStatus(own, 0).dwells;
    w.step(30);
    CHECK(w.radarStatus(own, 0).dwells == dwells);
    // and a failure fails it
    REQUIRE(w.submit(own, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    w.step();
    const auto third = w.submit(own, ms::AmtiCommand{});
    REQUIRE(third.accepted());
    w.failPayload(own, 0, true);
    CHECK(w.missionSystems(own)->activity(third.activity)->state == control::ActivityState::Failed);
}

TEST_CASE("radar for the viewers: each vehicle's payloads and radar picture in the shared memory, as the SDK has them", "[ms][radar][ipc]") {
    session::WorldOptions o = options("radar-mirror");
    o.publish = true;
    session::World w(o);
    std::uint32_t own = 0, target = 0;
    const auto detections = search(w, own, target, 6.0);
    REQUIRE(!detections.empty());
    w.publishNow();
    ipc::WorldMirror mirror;
    REQUIRE(mirror.open("radar-mirror"));
    ipc::MissionRecord m;
    REQUIRE(mirror.mission(0, m));
    CHECK(m.vehicleId == own);
    const ms::Host& host = *w.missionSystems(own);
    REQUIRE(m.payloadCount == host.payloads().size());
    CHECK(std::string(m.payloads[0].name) == "radar");
    CHECK(std::string(m.payloads[0].designation) == "AN/APG-68(V)9");
    CHECK(m.payloads[0].state == static_cast<std::uint8_t>(ms::PayloadState::Operate));
    CHECK(std::string(m.payloads[0].capabilities) == "amti");
    CHECK(std::string(m.payloads[1].capabilities) == "esm"); // (its warning receiver: built, MS-9a)
    CHECK(std::string(m.payloads[2].capabilities).find("(MS-9)") != std::string::npos); // (its dispenser: not yet)
    REQUIRE(m.radarCount == 1);
    const ipc::MissionRadar& r = m.radars[0];
    const ms::RadarStatus st = w.radarStatus(own, 0);
    CHECK(r.searching == 1);
    CHECK(r.dwells == st.dwells);
    CHECK(r.detections == st.detections);
    REQUIRE(r.recentCount == std::min<std::size_t>(detections.size(), ipc::kMissionDetections));
    CHECK(r.recent[r.recentCount - 1].timeS == detections.back().timeS);
    CHECK(r.recent[r.recentCount - 1].truth == detections.back().truth);
    CHECK_THAT(r.recent[r.recentCount - 1].rangeM, WithinRel(detections.back().rangeM, 1e-6));
    // the target carries the same fit: its record too, its radar off
    ipc::MissionRecord t;
    REQUIRE(mirror.mission(1, t));
    CHECK(t.vehicleId == target);
    CHECK(t.payloads[0].state == static_cast<std::uint8_t>(ms::PayloadState::Off));
    CHECK(t.radars[0].searching == 0);
    // unchanged: nothing new to copy; after a step, the scan moved on
    CHECK_FALSE(mirror.mission(0, m, m.revision));
    const auto revision = m.revision;
    const auto dwellsBefore = r.dwells; // (r is m's: read over below)
    w.step(10);
    w.publishNow();
    REQUIRE(mirror.mission(0, m, revision));
    CHECK(m.radars[0].dwells > dwellsBefore);
    // a vehicle removed: its slot's record empty
    w.removeVehicle(target);
    REQUIRE(mirror.mission(1, t));
    CHECK(t.vehicleId == 0);
    CHECK(t.payloadCount == 0);
}

TEST_CASE("radars in parallel: every radar of a world looks on the workers, the same looks to the bit as on one", "[ms][radar]") {
    // two groups of four F-16Cs 50 km apart head on, every radar searching: the same detections whatever the workers
    auto run = [](unsigned workers) {
        session::WorldOptions o = options(workers == 1 ? "radar-par-1" : "radar-par-4");
        o.workers = workers;
        session::World w(o);
        std::vector<std::uint32_t> ids;
        for (int side = 0; side < 2; ++side)
            for (int i = 0; i < 4; ++i) {
                session::VehicleSpec s;
                s.name = "f" + std::to_string(side) + std::to_string(i);
                s.type = "jsbsim:f16c";
                s.initial.latitudeDeg += (i - 1.5) * 2e3 / kEarthM / kDeg;
                s.initial.longitudeDeg += side * 50e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
                s.initial.altitudeMslM = 5000.0 + 300.0 * i;
                s.initial.headingDeg = side == 0 ? 90.0 : 270.0;
                s.initial.airspeedTrueMs = 220.0;
                ids.push_back(w.createVehicle(s));
                REQUIRE(ids.back() != 0);
            }
        for (const auto id : ids) REQUIRE(w.submit(id, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
        w.step();
        for (const auto id : ids) REQUIRE(w.submit(id, ms::AmtiCommand{}).accepted());
        w.step(stepsFor(w, 10.0));
        std::vector<std::vector<ms::AmtiDetection>> out;
        for (const auto id : ids) out.push_back(w.amtiDetections(id, 0));
        return out;
    };
    const auto one = run(1), four = run(4);
    std::size_t total = 0;
    REQUIRE(one.size() == four.size());
    for (std::size_t k = 0; k < one.size(); ++k) {
        REQUIRE(one[k].size() == four[k].size());
        for (std::size_t i = 0; i < one[k].size(); ++i) {
            CHECK(one[k][i].timeS == four[k][i].timeS);
            CHECK(one[k][i].truth == four[k][i].truth);
            CHECK(one[k][i].rangeM == four[k][i].rangeM);
            CHECK(one[k][i].snrDb == four[k][i].snrDb);
        }
        total += one[k].size();
    }
    CHECK(total > 50);
}

TEST_CASE("radar tracks (MS-5): each target one track, where it is, no false ones; the same on four workers", "[ms][radar][track]") {
    auto run = [](unsigned workers, std::vector<std::uint32_t>& targets, std::uint32_t& own) {
        session::WorldOptions o = options(workers == 1 ? "radar-tracks-1" : "radar-tracks-4");
        o.workers = workers;
        auto w = std::make_unique<session::World>(o);
        own = wing(*w, "f16c", 6000.0, 230.0);
        auto at = [&](const char* type, double northKm, double eastKm, double alt, double hdg, double tas) {
            session::VehicleSpec s;
            s.name = std::string(type) + std::to_string(targets.size());
            s.type = std::string("jsbsim:") + type;
            s.initial.latitudeDeg += northKm * 1e3 / kEarthM / kDeg;
            s.initial.longitudeDeg += eastKm * 1e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
            s.initial.altitudeMslM = alt;
            s.initial.headingDeg = hdg;
            s.initial.airspeedTrueMs = tas;
            targets.push_back(w->createVehicle(s));
            REQUIRE(targets.back() != 0);
        };
        at("f16c", 3.0, 55.0, 7000.0, 270.0, 230.0);   // head on
        at("b52h", 20.0, 70.0, 10500.0, 180.0, 220.0); // crossing, high
        at("c130j", -12.0, 40.0, 4000.0, 0.0, 140.0);  // crossing, low
        REQUIRE(w->submit(own, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
        w->step();
        ms::AmtiCommand c;
        c.elevationCentreRad = -1.0 * kDeg;
        c.bars = 4;
        REQUIRE(w->submit(own, c).accepted());
        w->step(stepsFor(*w, 60.0));
        return w;
    };
    std::vector<std::uint32_t> targets;
    std::uint32_t own = 0;
    auto w = run(1, targets, own);
    const auto tracks = w->amtiTracks(own, 0);
    const ms::RadarStatus st = w->radarStatus(own, 0);
    INFO(st.detections << " detections (" << st.falseAlarms << " false alarms), " << st.tracks << " tracks, " << st.tentativeTracks << " tentative");
    CHECK(st.falseAlarms <= 6); // (one a minute: its false-alarm time's; a clutter-heavy load is test_track's)
    std::size_t falseTracks = 0;
    for (const auto& tr : tracks) falseTracks += tr.truth == 0 ? 1 : 0;
    CHECK(falseTracks == 0);
    WARN(st.detections << " detections (" << st.falseAlarms << " false alarms), " << tracks.size() << " tracks (" << falseTracks << " false), "
                       << st.tentativeTracks << " tentative");
    const sim::VehicleState& o = *w->vehicleState(own);
    for (const auto target : targets) {
        std::vector<ms::AmtiTrack> mine;
        for (const auto& tr : tracks)
            if (tr.truth == target) mine.push_back(tr);
        INFO("target " << target);
        REQUIRE(mine.size() == 1); // (one track, no duplicate)
        const auto& tr = mine[0];
        const sim::VehicleState& t = *w->vehicleState(target);
        const double err = std::sqrt(std::pow(tr.stateEcef[0] - t.positionEcef[0], 2) + std::pow(tr.stateEcef[2] - t.positionEcef[1], 2) +
                                     std::pow(tr.stateEcef[4] - t.positionEcef[2], 2));
        INFO("position error " << err << " m, its sigma " << tr.positionSigmaM << " m; updates " << tr.updates);
        WARN("target " << target << ": track " << tr.id << ", " << tr.updates << " updates, position error " << err << " m (sigma " << tr.positionSigmaM
                       << " m), speed error " << std::hypot(tr.velocityNedMs[0] - t.velocityNedMs[0], tr.velocityNedMs[1] - t.velocityNedMs[1]) << " m/s");
        CHECK(err < 4.0 * std::sqrt(3.0) * tr.positionSigmaM + 50.0); // (within its own uncertainty)
        CHECK(tr.updates >= 8);
        CHECK(w->simTime() - tr.lastUpdateS < 12.0);
        // its velocity: within 60 m/s of the truth
        double vt[3];
        for (int i = 0; i < 3; ++i) vt[i] = t.velocityNedMs[i];
        CHECK(std::hypot(tr.velocityNedMs[0] - vt[0], tr.velocityNedMs[1] - vt[1]) < 60.0);
    }
    (void)o;
    // the same tracks on four workers
    std::vector<std::uint32_t> targets4;
    std::uint32_t own4 = 0;
    auto w4 = run(4, targets4, own4);
    const auto tracks4 = w4->amtiTracks(own4, 0);
    REQUIRE(tracks4.size() == tracks.size());
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        CHECK(tracks4[i].id == tracks[i].id);
        CHECK(tracks4[i].stateEcef[0] == tracks[i].stateEcef[0]);
        CHECK(tracks4[i].covarianceEcef[0] == tracks[i].covarianceEcef[0]);
    }
}

TEST_CASE("radar tracks (MS-5): the E-3G's looks at a B-52H, replayed - one track with its unscented filter, split by an EKF", "[ms][radar][track]") {
    // its detections as the radar gave its tracker (tests/simulation/data/track/aew_bomber_detections.txt): 2 to 4 looks a pass,
    // 13 ms apart, a 10 s rotation between. A new track's velocity across the line of sight is unknown (300 m/s): after
    // a rotation the target is 3 km across from where it was predicted, too curved a range rate for one linearisation -
    // an EKF's update lands outside its own covariance (27 m off in range, 4 m claimed) and the pass's next looks
    // start a second track
    std::vector<ms::track::Detection> dets;
    std::ifstream in(kSource / "tests" / "simulation" / "data" / "track" / "aew_bomber_detections.txt");
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        ms::track::Detection d;
        ss >> d.t >> d.truth;
        for (double& z : d.z.a) ss >> z;
        for (std::size_t i = 0; i < 4; ++i) ss >> d.model.noise(i, i);
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) ss >> d.model.rotation(i, j);
        for (double& x : d.model.translation) ss >> x;
        for (double& x : d.model.velocity) ss >> x;
        d.id = dets.size() + 1;
        dets.push_back(d);
    }
    REQUIRE(dets.size() == 18);
    const radar::Spec spec = radar::specOf(*card("an-apy-2"));
    REQUIRE(spec.ok);
    const ms::track::TrackerSettings settings = radar::Radar(spec, 1).trackerSettings();
    CHECK(settings.filter == ms::track::Filter::Ukf);
    auto replay = [&](ms::track::Filter f) {
        ms::track::TrackerSettings s = settings;
        s.filter = f;
        ms::track::MultiTargetTracker tracker(s);
        std::set<std::uint64_t> confirmed;
        for (const auto& d : dets) {
            tracker.step(d.t, {d});
            for (const auto& tr : tracker.tracks()) confirmed.insert(tr.id);
        }
        return std::make_pair(confirmed.size(), tracker.tracks().size());
    };
    const auto [ukfEver, ukfNow] = replay(ms::track::Filter::Ukf);
    CHECK(ukfEver == 1);
    CHECK(ukfNow == 1);
    const auto [ekfEver, ekfNow] = replay(ms::track::Filter::Ekf);
    INFO("the EKF: " << ekfEver << " tracks confirmed, " << ekfNow << " at the end");
    CHECK(ekfEver > 1);
}

TEST_CASE("radar tracks (MS-5): an AEW radar's several looks a pass make one track; a track the scan left is dropped", "[ms][radar][track]") {
    session::World w(options("radar-tracks-aew"));
    int made = 0;
    auto at = [&](const char* type, double northKm, double eastKm, double alt, double hdg, double tas) {
        session::VehicleSpec s;
        s.name = std::string(type) + "-t" + std::to_string(++made);
        s.type = std::string("jsbsim:") + type;
        s.initial.latitudeDeg += northKm * 1e3 / kEarthM / kDeg;
        s.initial.longitudeDeg += eastKm * 1e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
        s.initial.altitudeMslM = alt;
        s.initial.headingDeg = hdg;
        s.initial.airspeedTrueMs = tas;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        // it holds its speed and heading, level: unpiloted, an A-10C with its stick neutral and its engines at idle is in
        // the ground within 30 s (it once stayed up by tumbling, on a jump in its roll damping's table: hangar.md, Methods)
        control::VelocityCommand hold;
        hold.airspeedMs = tas;
        REQUIRE(w.submit(id, hold).accepted());
        return id;
    };
    // the E-3G sees each target on both its bars and on neighbouring azimuths, a few dwells apart: one track each
    // (at its cruise, Mach 0.76: at 170 m/s it flies 7 deg nose up, and its scan - in its body axes - over what is ahead)
    const auto sentry = at("e3g", 0.0, 0.0, 9000.0, 0.0, 230.0);
    const std::uint32_t targets[] = {at("b52h", 25.0, -165.0, 10500.0, 180.0, 220.0), at("f16c", 0.0, -90.0, 6000.0, 270.0, 230.0),
                                     at("c130j", -18.0, -135.0, 4000.0, 0.0, 140.0), at("a10c", 6.0, -122.0, 2500.0, 30.0, 150.0),
                                     at("f16c", 60.0, 40.0, 7000.0, 200.0, 230.0), at("c17a", -80.0, 120.0, 9500.0, 300.0, 210.0)};
    REQUIRE(w.submit(sentry, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    for (int i = 0; i < 6000 && w.missionSystems(sentry)->payloads()[0].state != ms::PayloadState::Operate; ++i) w.step();
    ms::AmtiCommand c;
    c.mode = ms::AmtiMode::Surveillance;
    c.bars = 2;
    c.rangeScaleM = 400e3;
    REQUIRE(w.submit(sentry, c).accepted());
    w.step(stepsFor(w, 120.0));
    const auto tracks = w.amtiTracks(sentry, 0);
    std::map<std::uint32_t, int> per;
    for (const auto& tr : tracks) ++per[tr.truth];
    for (const auto target : targets) {
        INFO(tracks.size() << " tracks; target " << target << ": " << per[target]);
        CHECK(per[target] == 1);
    }
    CHECK(per[0] == 0);

    // the fighter's radar, flying at them: its tracks; then its scan moved off them - each dropped in its drop looks'
    // frames and one more (Stone Soup's UpdateTimeDeleter), never missed again where it no longer looks
    const auto own = at("f16c", 0.0, -250.0, 6000.0, 90.0, 230.0); // (the bomber, the C-130J and the A-10C ahead)
    REQUIRE(w.submit(own, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    w.step();
    ms::AmtiCommand ahead;
    ahead.azimuthHalfWidthRad = 60.0 * kDeg;
    ahead.bars = 4;
    REQUIRE(w.submit(own, ahead).accepted());
    w.step(stepsFor(w, 40.0));
    const std::size_t before = w.amtiTracks(own, 0).size();
    REQUIRE(before >= 1);
    ms::AmtiCommand aside = ahead;
    aside.azimuthCentreRad = 50.0 * kDeg; // (40 to 60 deg right: nothing there)
    aside.azimuthHalfWidthRad = 10.0 * kDeg;
    aside.bars = 1;
    REQUIRE(w.submit(own, aside).accepted());
    const double frame = w.radarStatus(own, 0).framePeriodS;
    w.step(stepsFor(w, 4.0 * frame + 1.0));
    INFO(before << " tracks before; the frame now " << frame << " s");
    CHECK(w.amtiTracks(own, 0).empty());
}

TEST_CASE("radar signal level (MS-6): the same radar, scan and target as the detection level - detected alike", "[ms][radar][signal]") {
    // a Rayleigh fighter head on, level, at the range the detection level detects it half the time on the beam's axis
    // (clear of its clutter rules: closing faster than the radar flies, the beam level): each level's share of frames
    // that detect it. The signal level is given back its chain's mean processing loss at a dwell's Pd of 0.5 - its mean
    // over where a target's range folds in each PRF's interval - so with the beam staring at it (a dwell a frame, on its
    // axis, at that Pd) they agree on average over ranges and speeds: ten across the longest PRI (its RCS scaled with
    // range to the fourth, the SNR the same at each), closing 450 to 585 m/s, the signal level the better where none of its PRFs eclipses the target and the
    // worse where some do (medium PRF's blind ranges). Scanning, the dwells' SNR is lower (the beam's positions either
    // side of the target): there the blind ranges' spread of losses tells more, the signal level detecting less often
    session::World w(options("radar-signal"));
    const auto own = wing(w, "f16c", 5000.0, 250.0);
    const sim::VehicleState self = *w.vehicleState(own);
    const radar::Spec spec = radar::specOf(*card("an-apg-68v9"));
    REQUIRE(spec.ok);
    const double range = radar::Radar(spec, 1).detectionRangeM(3.0, 0.5, 1);
    session::VehicleSpec ts;
    ts.name = "target";
    ts.type = "jsbsim:f16c";
    ts.initial.longitudeDeg += range / (kEarthM * std::cos(ts.initial.latitudeDeg * kDeg)) / kDeg;
    ts.initial.altitudeMslM = 5000.0;
    ts.initial.headingDeg = 270.0;
    ts.initial.airspeedTrueMs = 200.0;
    REQUIRE(w.createVehicle(ts) != 0);
    FixedScene scene;
    scene.own = &self;
    scene.ids = {1};
    scene.states = {*w.vehicleState(2)};
    scene.states[0].velocityNedMs[0] = 0.0, scene.states[0].velocityNedMs[1] = -200.0, scene.states[0].velocityNedMs[2] = 0.0;
    scene.sigmaM2 = 3.0, scene.fluctuation = 2.0;
    scene.ground = false; // (free space: the levels compared on the radar equation, the signal level's clutter apart)
    const double r0 = std::sqrt(std::pow(scene.states[0].positionEcef[0] - self.positionEcef[0], 2) +
                                std::pow(scene.states[0].positionEcef[1] - self.positionEcef[1], 2) +
                                std::pow(scene.states[0].positionEcef[2] - self.positionEcef[2], 2));
    const sim::VehicleState at0 = scene.states[0];
    double truth = r0;
    auto place = [&](double rangeM) { // (along the same line of sight, the same SNR)
        for (int i = 0; i < 3; ++i)
            scene.states[0].positionEcef[i] = self.positionEcef[i] + (at0.positionEcef[i] - self.positionEcef[i]) * rangeM / r0;
        scene.sigmaM2 = 3.0 * std::pow(rangeM / r0, 4);
        truth = rangeM;
    };
    constexpr int kRanges = 40, kFrames = 30;
    double span = 0.0; // (the longest PRI's unambiguous range: its waveform's, planned by a search)
    {
        radar::Radar planner(spec, 1);
        radar::Search s;
        s.azimuthHalfWidthRad = 4.0 * kDeg;
        s.bars = 1;
        s.signalLevel = true;
        planner.search(s, 0.0);
        REQUIRE(!planner.waveform().intervals.empty());
        span = planner.waveform().intervals.front().ambiguousRangeM;
    }
    auto frames = [&](bool signalLevel, double halfWidthRad, double& rangeError, double& resolved) {
        int found = 0, n = 0, good = 0;
        double sumErr = 0.0;
        for (int k = 0; k < kRanges; ++k) {
            place(r0 + span * ((k + 0.5) / kRanges - 0.5));
            scene.states[0].velocityNedMs[1] = -200.0 - 3.75 * k; // (its Doppler across the filters too: 150 m/s, a blind speed)
            radar::Radar r(spec, 5 + static_cast<std::uint64_t>(k));
            radar::Search s;
            s.azimuthHalfWidthRad = halfWidthRad;
            s.bars = 1;
            s.signalLevel = signalLevel;
            r.search(s, 0.0);
            const double period = r.framePeriodS();
            std::uint64_t seen = 0;
            for (int f = 1; f <= kFrames; ++f) {
                r.step(f * period, self, scene);
                bool hit = false;
                for (const auto& d : r.detections())
                    if (d.serial >= seen && d.truth == 1) {
                        hit = true, ++n;
                        if (std::fabs(d.rangeM - truth) < 3.0 * spec.rangeResolutionM) sumErr += std::fabs(d.rangeM - truth), ++good;
                    }
                seen = r.detectionCount();
                found += hit ? 1 : 0;
            }
        }
        rangeError = good ? sumErr / good : 0.0;
        resolved = n ? static_cast<double>(good) / n : 0.0;
        return found / static_cast<double>(kRanges * kFrames);
    };
    double errDetection = 0.0, errSignal = 0.0, okDetection = 0.0, okSignal = 0.0, unused[4] = {};
    const double pDetection = frames(false, 0.0, errDetection, okDetection), pSignal = frames(true, 0.0, errSignal, okSignal);
    const double scanDetection = frames(false, 4.0 * kDeg, unused[0], unused[1]), scanSignal = frames(true, 4.0 * kDeg, unused[2], unused[3]);
    // the signal level's ambiguities resolved: within 3 gates of the truth, or a ghost elsewhere (the PRFs' aliases of
    // its own detections agreeing by chance - its tracker's gate keeps them out)
    WARN("at " << r0 / 1e3 << " +- " << span / 2e3 << " km, staring: frames detecting it - detection level " << pDetection << ", signal level "
               << pSignal << "; resolved " << okDetection << " and " << okSignal << ", their mean range error " << errDetection << " m and "
               << errSignal << " m; scanning +-4 deg: " << scanDetection << " and " << scanSignal);
    CHECK(std::fabs(pSignal - pDetection) < 0.06); // (each a share of 1,200 frames: some 0.015 either)
    CHECK(scanSignal > 0.5 * scanDetection);
    CHECK(scanSignal < 1.2 * scanDetection);
    CHECK(okSignal > 0.9);
    CHECK(errSignal < spec.rangeResolutionM / 2.0);
}

namespace {

/// A FixedScene over the sea or land, or with no ground.
struct SurfaceScene final : radar::Scene {
    const FixedScene* f = nullptr;
    int surface = 0; ///< 0 land, 1 sea, 2 none
    std::size_t targets(std::uint32_t* o, const sim::VehicleState** s, std::size_t m) const override { return f->targets(o, s, m); }
    double sigma(std::uint32_t t, double fr, double p, double& fl) const override { return f->sigma(t, fr, p, fl); }
    double pathLossDb(std::uint32_t t, double fr) const override { return f->pathLossDb(t, fr); }
    double surfaceHeightM(double, double) const override { return surface == 1 ? -10.0 : std::numeric_limits<double>::quiet_NaN(); }
    bool hasGround() const override { return surface != 2; }
};

/// A radar's frames over a scene: the share in which it detected entity 1, and its false alarms.
std::pair<double, int> frames(const radar::Spec& spec, const radar::Scene& scene, const sim::VehicleState& self, radar::Search s, int count) {
    radar::Radar r(spec, 5);
    r.search(s, 0.0);
    int found = 0, alarms = 0;
    std::uint64_t seen = 0;
    for (int f = 1; f <= count; ++f) {
        r.step(f * r.framePeriodS(), self, scene);
        bool hit = false;
        for (const auto& d : r.detections())
            if (d.serial >= seen) {
                if (d.truth == 1) hit = true;
                else ++alarms;
            }
        seen = r.detectionCount();
        found += hit ? 1 : 0;
    }
    return {static_cast<double>(found) / count, alarms};
}

} // namespace

TEST_CASE("radar signal level (MS-6b): the ground's clutter - no false alarms from it, the main beam's notch, land worse than sea", "[ms][radar][signal]") {
    // the APG-68 at 5 km over flat ground, 250 m/s: its signal-level clutter (constant gamma, land -15 dB, sea -40 dB at
    // 10 GHz) folded into every gate and filter by its medium PRFs
    session::World w(options("radar-signal-clutter"));
    const auto own = wing(w, "f16c", 5000.0, 250.0);
    const sim::VehicleState self = *w.vehicleState(own);
    const radar::Spec spec = radar::specOf(*card("an-apg-68v9"));
    auto target = [&](double northKm, double eastKm, double altitudeM, double eastSpeed) {
        session::VehicleSpec ts;
        ts.name = "t" + std::to_string(static_cast<int>(eastKm * 10)) + "_" + std::to_string(static_cast<int>(altitudeM)) + "_" +
                  std::to_string(static_cast<int>(-eastSpeed));
        ts.type = "jsbsim:f16c";
        ts.initial.latitudeDeg += northKm * 1e3 / kEarthM / kDeg;
        ts.initial.longitudeDeg += eastKm * 1e3 / (kEarthM * std::cos(ts.initial.latitudeDeg * kDeg)) / kDeg;
        ts.initial.altitudeMslM = altitudeM;
        ts.initial.headingDeg = 270.0;
        ts.initial.airspeedTrueMs = 200.0;
        const auto id = w.createVehicle(ts);
        REQUIRE(id != 0);
        sim::VehicleState st = *w.vehicleState(id);
        st.velocityNedMs[0] = 0.0, st.velocityNedMs[1] = eastSpeed, st.velocityNedMs[2] = 0.0;
        return st;
    };
    radar::Search level;
    level.azimuthHalfWidthRad = 4.0 * kDeg;
    level.bars = 1;
    level.signalLevel = true;
    SECTION("looking level over land, sea and free space: detected less where the clutter is stronger, no false alarms") {
        FixedScene fs;
        fs.own = &self;
        fs.ids.push_back(1);
        fs.states.push_back(target(0.0, 60.0, 5000.0, -200.0)); // (head on, 60 km)
        fs.sigmaM2 = 3.0, fs.fluctuation = 2.0;
        double pd[3];
        int alarms[3];
        for (int surface : {0, 1, 2}) {
            SurfaceScene scene;
            scene.f = &fs, scene.surface = surface;
            std::tie(pd[surface], alarms[surface]) = frames(spec, scene, self, level, 150);
        }
        WARN("60 km head on - land " << pd[0] << " (" << alarms[0] << " false alarms), sea " << pd[1] << " (" << alarms[1] << "), free space " << pd[2] << " ("
                                     << alarms[2] << ")");
        CHECK(pd[2] >= pd[1] - 0.05);
        CHECK(pd[1] > pd[0]);
        for (const int a : alarms) CHECK(a <= 3); // (150 frames of 0.4 s: a minute of them - one designed)
    }
    SECTION("looking down: a target hovering in the main beam's clutter is notched; one closing fast there is seen") {
        radar::Search down = level;
        down.elevationCentreRad = std::atan2(-3000.0, 20e3); // (at the ground 30 km ahead; the targets 20 km out, 3 km below)
        double pd[2];
        for (int moving : {0, 1}) {
            FixedScene fs;
            fs.own = &self;
            fs.ids.push_back(1);
            fs.states.push_back(target(0.0, 20.0, 2000.0, moving ? -250.0 : 0.0));
            fs.sigmaM2 = 3.0, fs.fluctuation = 2.0;
            SurfaceScene scene;
            scene.f = &fs, scene.surface = 0;
            pd[moving] = frames(spec, scene, self, down, 100).first;
        }
        WARN("looking down at 20 km over land: hovering " << pd[0] << ", closing at 500 m/s " << pd[1]);
        CHECK(pd[0] < 0.05);
        CHECK(pd[1] > 0.5);
    }
}

// Measurement-error floors (SIM-M01, docs/sdk/radar-measurement-floors.md, section 1.4's R1-R7): a look's errors
// do not shrink below its card's floors - the PS-05/A's, assumed - added in quadrature to the thermal term, with the
// same draws at the detection level; every other card has none.
namespace {

constexpr double kLightMs = 299792458.0;

/// One detection of the target: its four errors against the truth (elevation, azimuth, range, range rate), the
/// standard deviations it reported, its SNR and when.
struct Look {
    double error[4] = {}, sigma[4] = {};
    double snrDb = 0.0, timeS = 0.0;
};

struct Stared {
    std::vector<Look> looks;
    std::vector<radar::Detection> all; ///< every detection, false alarms too
    double positionSigmaM = 0.0;       ///< the target's track at the end (0: none)
    double bwEl = 0.0, bwAz = 0.0, dwellS = 0.0;
    int intervals = 0;                 ///< the signal level's coherent intervals a dwell
};

/// `frames` dwells of `spec` (seeded `seed`) staring along `carrier`'s nose at a steady target `rangeM` ahead of `rcsM2`:
/// both still (no clutter band, range rate 0, a range that does not move), free space - so the truth is the geometry's.
Stared stare(const radar::Spec& spec, std::uint64_t seed, const sim::VehicleState& carrier, double rangeM, double rcsM2, int frames,
             bool signalLevel = false) {
    sim::VehicleState self = carrier;
    self.velocityNedMs[0] = self.velocityNedMs[1] = self.velocityNedMs[2] = 0.0;
    double r[9]; // ecef = r body: the nose (body x) is r's first column
    core::attitude::bodyToEcefFromQuaternion(self.attitudeEcefToBody, r);
    sim::VehicleState t = self;
    for (std::size_t i = 0; i < 3; ++i) t.positionEcef[i] = self.positionEcef[i] + rangeM * r[3 * i];
    FixedScene scene;
    scene.own = &self;
    scene.ids = {1};
    scene.states = {t};
    scene.sigmaM2 = rcsM2, scene.fluctuation = 1.0, scene.ground = false;
    double d[3], range = 0.0;
    for (std::size_t i = 0; i < 3; ++i) d[i] = t.positionEcef[i] - self.positionEcef[i], range += d[i] * d[i];
    range = std::sqrt(range);
    for (double& c : d) c /= range;
    const double db[3] = {r[0] * d[0] + r[3] * d[1] + r[6] * d[2], r[1] * d[0] + r[4] * d[1] + r[7] * d[2], r[2] * d[0] + r[5] * d[1] + r[8] * d[2]};
    const double truth[4] = {std::atan2(-db[2], std::hypot(db[0], db[1])), std::atan2(db[1], db[0]), range, 0.0};
    radar::Radar radar_(spec, seed);
    radar::Search s;
    s.azimuthHalfWidthRad = 0.0;
    s.bars = 1;
    s.signalLevel = signalLevel;
    radar_.search(s, 0.0);
    Stared out;
    out.bwEl = radar_.pattern().beamwidthRad(true), out.bwAz = radar_.pattern().beamwidthRad(false);
    out.dwellS = radar_.nominalDwellS();
    out.intervals = static_cast<int>(radar_.waveform().intervals.size());
    const double period = radar_.framePeriodS();
    std::uint64_t seen = 0;
    double now = 0.0;
    for (int f = 1; f <= frames; ++f) {
        now = f * period;
        radar_.step(now, self, scene);
        for (const auto& det : radar_.detections()) {
            if (det.serial < seen) continue;
            out.all.push_back(det);
            if (det.truth != 1) continue;
            Look k;
            const double measured[4] = {det.elevationRad, det.azimuthRad, det.rangeM, det.rangeRateMs};
            for (std::size_t i = 0; i < 4; ++i) k.error[i] = measured[i] - truth[i], k.sigma[i] = det.sigma[i];
            k.snrDb = det.snrDb, k.timeS = det.timeS;
            out.looks.push_back(k);
        }
        seen = radar_.detectionCount();
    }
    for (const auto& tr : radar_.tracks()) {
        const auto rep = radar_.report(tr, now, self);
        if (rep.truth == 1) out.positionSigmaM = rep.positionSigmaM;
    }
    return out;
}

/// Each quantity's thermal standard deviation (the radar's own law, written out here) at a look's SNR; `dopplerMs`
/// the level's Doppler resolution.
std::array<double, 4> thermal(const radar::Spec& spec, const Stared& run, double snrDb, double dopplerMs) {
    const double root = std::sqrt(2.0 * std::max(std::pow(10.0, snrDb / 10.0), 1.0));
    return {run.bwEl / (spec.monopulseSlope * root), run.bwAz / (spec.monopulseSlope * root), spec.rangeResolutionM / root, dopplerMs / root};
}

std::array<double, 4> floorsOf(const radar::Spec& spec, const Stared& run, double dopplerMs) {
    return {spec.angleFloorBeamwidths * run.bwEl, spec.angleFloorBeamwidths * run.bwAz, spec.rangeFloorResolutions * spec.rangeResolutionM,
            spec.rangeRateFloorResolutions * dopplerMs};
}

/// The errors' sample mean and standard deviation, quantity `i`.
std::pair<double, double> spread(const std::vector<Look>& looks, std::size_t i) {
    double m = 0.0, v = 0.0;
    for (const auto& k : looks) m += k.error[i];
    m /= static_cast<double>(looks.size());
    for (const auto& k : looks) v += (k.error[i] - m) * (k.error[i] - m);
    return {m, std::sqrt(v / static_cast<double>(looks.size() - 1))};
}

/// The RCS that puts a target `rangeM` away at `snrDb` on the radar's axis, free space (its radar equation alone).
double rcsFor(const radar::Spec& spec, double rangeM, double snrDb) { return std::pow(10.0, (snrDb - radar::Radar(spec, 1).snrDb(1.0, rangeM)) / 10.0); }

const char* const kQuantity[4] = {"elevation", "azimuth", "range", "range rate"};

} // namespace

TEST_CASE("radar floors (SIM-M01): the cards' floors - the PS-05/A's alone, every other none; an invalid one refused", "[ms][radar][floor]") {
    // R5 and R6: the PS-05/A's assumed floors as its card gives them; no other air radar's card gives any
    const radar::Spec ps05 = radar::specOf(*card("ps-05a"));
    REQUIRE(ps05.ok);
    CHECK(ps05.angleFloorBeamwidths == 0.1);
    CHECK(ps05.rangeFloorResolutions == 0.05);
    CHECK(ps05.rangeRateFloorResolutions == 0.05);
    int others = 0;
    for (const auto& entry : fs::directory_iterator(kSource / "assets" / "payloads")) {
        std::vector<std::string> errors;
        const auto c = ms::readCard(entry.path(), errors);
        if (!c || c->family != ms::Family::AirRadar || c->id == "ps-05a") continue;
        const radar::Spec s = radar::specOf(*c);
        INFO(c->id);
        CHECK(s.ok);
        CHECK(s.angleFloorBeamwidths == 0.0);
        CHECK(s.rangeFloorResolutions == 0.0);
        CHECK(s.rangeRateFloorResolutions == 0.0);
        ++others;
    }
    CHECK(others == 16);
    // a floor below zero or not a number describes no radar; zero is none
    for (const char* name : {"angle_floor_beamwidths", "range_floor_resolutions", "range_rate_floor_resolutions"})
        for (const double bad : {-0.1, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(), 0.0}) {
            ms::Card c = *card("ps-05a");
            c.parameters[name].value = bad;
            INFO(name << " = " << bad);
            CHECK(radar::specOf(c).ok == (bad == 0.0));
        }
}

TEST_CASE("radar floors (SIM-M01): the PS-05/A's errors at the detection level - its floors in quadrature, the same draws", "[ms][radar][floor]") {
    session::World w(options("radar-floors"));
    const sim::VehicleState carrier = *w.vehicleState(wing(w, "f16c", 5000.0, 200.0));
    const radar::Spec spec = radar::specOf(*card("ps-05a"));
    REQUIRE(spec.ok);
    radar::Spec bare = spec;
    bare.angleFloorBeamwidths = bare.rangeFloorResolutions = bare.rangeRateFloorResolutions = 0.0;
    constexpr double kRange = 20e3;
    // the dwell's SNR is the radar equation's less what the card's class calibration and the look take: the high-SNR
    // run measures that offset, so the near-threshold run is put at 15 dB as dwelled - where its 3 of 8 detects a
    // steady target some 9 % of the time (so ten times the frames for the same count)
    double offset = 0.0;
    for (const double snr : {45.0, 15.0}) { // (R1 the floors' own; R2 near the threshold, the quadrature's)
        const double rcs = rcsFor(spec, kRange, snr - offset);
        const int frames = snr > 30.0 ? 4000 : 40000;
        const Stared floored = stare(spec, 11, carrier, kRange, rcs, frames), plain = stare(bare, 11, carrier, kRange, rcs, frames);
        INFO("SNR " << snr << " dB: " << floored.looks.size() << " detections");
        REQUIRE(floored.looks.size() > 3000);
        if (snr > 30.0) offset = floored.looks.front().snrDb - snr;
        const double doppler = kLightMs / spec.frequencyHz / 2.0 / floored.dwellS;
        const auto floors = floorsOf(spec, floored, doppler);
        // R3: the same detections, each error the floorless one scaled by the standard deviations' ratio
        REQUIRE(plain.looks.size() == floored.looks.size());
        REQUIRE(plain.all.size() == floored.all.size());
        for (std::size_t n = 0; n < floored.looks.size(); ++n) {
            const Look &a = floored.looks[n], &b = plain.looks[n];
            REQUIRE(a.timeS == b.timeS);
            REQUIRE(a.snrDb == b.snrDb);
            const auto t = thermal(spec, floored, a.snrDb, doppler);
            for (std::size_t i = 0; i < 4; ++i) {
                REQUIRE_THAT(b.sigma[i], WithinRel(t[i], 1e-12));                         // (no floor: the thermal term)
                REQUIRE_THAT(a.sigma[i], WithinRel(std::hypot(t[i], floors[i]), 1e-12)); // (with it: in quadrature)
                REQUIRE_THAT(a.error[i], WithinAbs(b.error[i] * a.sigma[i] / b.sigma[i], 1e-9 * std::fabs(a.error[i]) + 1e-9));
            }
        }
        // R1/R2: the errors spread as the reported standard deviations say, about the truth
        const double snrMean = floored.looks.front().snrDb; // (a steady target: every look the same)
        const auto t = thermal(spec, floored, snrMean, doppler);
        for (std::size_t i = 0; i < 4; ++i) {
            const double expect = std::hypot(t[i], floors[i]);
            const auto [mean, sd] = spread(floored.looks, i);
            WARN("detection level, SNR " << snrMean << " dB, " << floored.looks.size() << " looks, " << kQuantity[i] << ": spread " << sd << " against "
                 << expect << " (thermal " << t[i] << ", floor " << floors[i] << "), mean " << mean);
            CHECK(std::fabs(sd / expect - 1.0) < 0.06);
            CHECK(std::fabs(mean) < 4.0 * expect / std::sqrt(static_cast<double>(floored.looks.size())));
        }
        if (snr > 30.0) {
            CHECK(snrMean > 40.0);
            for (std::size_t i = 0; i < 4; ++i) CHECK(floors[i] > 2.0 * t[i]); // (the floors' own regime)
        } else {
            CHECK(snrMean < 20.0);
            CHECK(t[1] > 0.5 * floors[1]); // (the thermal term no longer negligible)
        }
    }
}

TEST_CASE("radar floors (SIM-M01): the PS-05/A at the signal level - its floors added after the processing's own errors", "[ms][radar][floor][signal]") {
    session::World w(options("radar-floors-signal"));
    const sim::VehicleState carrier = *w.vehicleState(wing(w, "f16c", 5000.0, 200.0));
    const radar::Spec spec = radar::specOf(*card("ps-05a"));
    REQUIRE(spec.ok);
    constexpr double kRange = 20e3;
    const Stared run = stare(spec, 13, carrier, kRange, rcsFor(spec, kRange, 45.0), 900, true);
    REQUIRE(run.intervals > 0);
    const double doppler = kLightMs / spec.frequencyHz / 2.0 / (run.dwellS / run.intervals);
    const auto floors = floorsOf(spec, run, doppler);
    // R4: what it reports, the floorless report and the floor in quadrature
    std::vector<Look> resolved; // (its ambiguities resolved: within 3 gates of the truth)
    for (const auto& k : run.looks) {
        const auto t = thermal(spec, run, k.snrDb, doppler);
        for (std::size_t i = 0; i < 4; ++i) REQUIRE_THAT(k.sigma[i], WithinRel(std::hypot(t[i], floors[i]), 1e-9));
        if (std::fabs(k.error[2]) < 3.0 * spec.rangeResolutionM) resolved.push_back(k);
    }
    INFO(resolved.size() << " of " << run.looks.size() << " detections resolved");
    REQUIRE(resolved.size() >= 600);
    for (std::size_t i = 0; i < 2; ++i) { // (the angles: a strong return's are the floor's)
        double mean = 0.0;
        for (const auto& k : resolved) mean += k.sigma[i];
        mean /= static_cast<double>(resolved.size());
        const auto [m, sd] = spread(resolved, i);
        WARN("signal level, " << resolved.size() << " looks, " << kQuantity[i] << ": spread " << sd << " against the reported " << mean << " (floor "
             << floors[i] << "), mean " << m);
        CHECK(std::fabs(sd / mean - 1.0) < 0.12);
        CHECK(std::fabs(m) < 4.0 * mean / std::sqrt(static_cast<double>(resolved.size())));
    }
}

TEST_CASE("radar floors (SIM-M01): across the assumption's band - the angle errors follow the floor, the track's uncertainty grows with it", "[ms][radar][floor][track]") {
    // R7: angle_floor_beamwidths at the band's ends and its value (0.01, 0.1, 0.19); its tracker given the looks
    session::World w(options("radar-floors-band"));
    const sim::VehicleState carrier = *w.vehicleState(wing(w, "f16c", 5000.0, 200.0));
    const radar::Spec card05 = radar::specOf(*card("ps-05a"));
    REQUIRE(card05.ok);
    constexpr double kRange = 20e3;
    double previous = 0.0;
    for (const double f : {0.01, 0.1, 0.19}) {
        radar::Spec spec = card05;
        spec.angleFloorBeamwidths = f;
        const Stared run = stare(spec, 17, carrier, kRange, rcsFor(spec, kRange, 45.0), 4000);
        REQUIRE(run.looks.size() > 3000);
        const double doppler = kLightMs / spec.frequencyHz / 2.0 / run.dwellS;
        const auto t = thermal(spec, run, run.looks.front().snrDb, doppler);
        const auto floors = floorsOf(spec, run, doppler);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto [m, sd] = spread(run.looks, i);
            WARN("floor " << f << " beamwidths, " << kQuantity[i] << ": spread " << sd << " against " << std::hypot(t[i], floors[i]));
            CHECK(std::fabs(sd / std::hypot(t[i], floors[i]) - 1.0) < 0.06);
        }
        WARN("floor " << f << ": the track's position uncertainty " << run.positionSigmaM << " m");
        REQUIRE(run.positionSigmaM > 0.0);
        CHECK(run.positionSigmaM > previous);
        previous = run.positionSigmaM;
    }
}
