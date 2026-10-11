// A radar's engagement modes (ADR-30, MS-4b; A-GRA's AMTI_CapabilityEnum): single-target track - every dwell on its
// track, the beam following, and the track gone when the radar can no longer see the target (the notch, the antenna's
// reach) for its memory time; track-while-scan kept on a track, an electronic array's looks at it between its search
// dwells; a cued search's acquisition (a fighter's boresight and vertical scan); and all of it through the World - the
// commands, the refusals, the status, the activity that fails when the target is lost, the viewers' record.
#include "mode_flights.h"

#include "ipc/WorldMirror.h"
#include "ms/Cards.h"
#include "ms/radar/Radar.h"
#include "sim/Attitude.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;
namespace radar = fsim::ms::radar;

namespace {

namespace fs = std::filesystem;
const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();

radar::Spec specOf(const std::string& id) {
    std::vector<std::string> errors;
    const auto c = ms::readCard(kSource / "assets" / "payloads" / (id + ".json"), errors);
    REQUIRE(c != nullptr);
    const radar::Spec s = radar::specOf(*c);
    REQUIRE(s.ok);
    return s;
}

void ned(double lat, double lon, double n[3], double e[3], double d[3]) {
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    n[0] = -sl * co, n[1] = -sl * so, n[2] = cl;
    e[0] = -so, e[1] = co, e[2] = 0.0;
    d[0] = -cl * co, d[1] = -cl * so, d[2] = -sl;
}

/// A carrier and its targets on straight lines, free space between them: each a state moved on along its velocity (the
/// north, east and down of where it started - as the radar takes them).
struct Rig final : radar::Scene {
    sim::VehicleState self;
    std::vector<sim::VehicleState> states;
    double time = 0.0;
    double sigmaM2 = 3.0;

    /// The carrier: an F-16C's state at 5,000 m, 250 m/s east, level.
    explicit Rig(session::World& w) : world(&w) {
        const auto id = wing(w, "f16c", 5000.0, 250.0);
        self = *w.vehicleState(id);
        self.velocityNedMs[0] = 0.0, self.velocityNedMs[1] = 250.0, self.velocityNedMs[2] = 0.0;
    }
    /// A target `northKm` and `eastKm` from the carrier's start at `altitudeM`, its velocity as given: its number (1, 2 ...).
    std::uint32_t target(double northKm, double eastKm, double altitudeM, double vNorth, double vEast) {
        session::VehicleSpec s;
        s.name = "t" + std::to_string(states.size() + 1);
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg += northKm * 1e3 / kEarthM / kDeg;
        s.initial.longitudeDeg += eastKm * 1e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
        s.initial.altitudeMslM = altitudeM;
        s.initial.headingDeg = 90.0;
        s.initial.airspeedTrueMs = 200.0;
        const auto id = world->createVehicle(s);
        REQUIRE(id != 0);
        states.push_back(*world->vehicleState(id));
        states.back().velocityNedMs[0] = vNorth, states.back().velocityNedMs[1] = vEast, states.back().velocityNedMs[2] = 0.0;
        return static_cast<std::uint32_t>(states.size());
    }
    static void move(sim::VehicleState& s, double dt) {
        double n[3], e[3], d[3];
        ned(s.latitudeRad, s.longitudeRad, n, e, d);
        for (int i = 0; i < 3; ++i) s.positionEcef[i] += dt * (s.velocityNedMs[0] * n[i] + s.velocityNedMs[1] * e[i] + s.velocityNedMs[2] * d[i]);
    }
    /// Everything `dt` on, then the radar's dwells to that time.
    void step(radar::Radar& r, double dt) {
        time += dt;
        move(self, dt);
        for (auto& t : states) move(t, dt);
        r.step(time, self, *this);
    }
    void run(radar::Radar& r, double seconds, double dt = 0.05) {
        const double end = time + seconds - 1e-9;
        while (time < end) step(r, dt);
    }
    /// A target from the carrier now: its range, and its azimuth and elevation in the carrier's body axes.
    void where(std::uint32_t target, double& range, double& az, double& el) const {
        const sim::VehicleState& t = states[target - 1];
        const double d[3] = {t.positionEcef[0] - self.positionEcef[0], t.positionEcef[1] - self.positionEcef[1], t.positionEcef[2] - self.positionEcef[2]};
        double r[9];
        sim::attitude::bodyToEcefFromQuaternion(self.attitudeEcefToBody, r);
        const double b[3] = {r[0] * d[0] + r[3] * d[1] + r[6] * d[2], r[1] * d[0] + r[4] * d[1] + r[7] * d[2], r[2] * d[0] + r[5] * d[1] + r[8] * d[2]};
        range = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        az = std::atan2(b[1], b[0]);
        el = std::atan2(-b[2], std::hypot(b[0], b[1]));
    }
    /// The radar's confirmed track of a target (the one its detections last updated); null for none.
    static const ms::track::Track* trackOf(const radar::Radar& r, std::uint32_t target) {
        for (const auto& t : r.tracks())
            for (auto it = t.points.rbegin(); it != t.points.rend(); ++it)
                if (it->update) {
                    if (it->truth == target) return &t;
                    break;
                }
        return nullptr;
    }
    /// How far a track's place is from its target's, now (m).
    double error(const radar::Radar& r, const ms::track::Track& t, std::uint32_t target) const {
        const ms::AmtiTrack a = r.report(t, time, self);
        const sim::VehicleState& s = states[target - 1];
        return std::sqrt(std::pow(a.stateEcef[0] - s.positionEcef[0], 2) + std::pow(a.stateEcef[2] - s.positionEcef[1], 2) +
                         std::pow(a.stateEcef[4] - s.positionEcef[2], 2));
    }

    // the radar's scene
    std::size_t targets(std::uint32_t* ids, const sim::VehicleState** s, std::size_t max) const override {
        std::size_t n = 0;
        for (; n < states.size() && n < max; ++n) ids[n] = static_cast<std::uint32_t>(n + 1), s[n] = &states[n];
        return n;
    }
    double sigma(std::uint32_t, double, double, double& f) const override { return f = 1.0, sigmaM2; }
    double pathLossDb(std::uint32_t target, double frequencyHz) const override {
        double range, az, el;
        where(target, range, az, el);
        return 20.0 * std::log10(4.0 * kPi * range * frequencyHz / 299792458.0);
    }

private:
    session::World* world;
};

/// A command's search, as the host gives it to the radar.
radar::Search searchOf(const ms::AmtiCommand& c) {
    radar::Search s;
    s.mode = c.mode, s.task = c.task, s.track = c.track, s.illuminate = c.illuminate;
    s.azimuthCentreRad = c.azimuthCentreRad, s.azimuthHalfWidthRad = c.azimuthHalfWidthRad;
    s.elevationCentreRad = c.elevationCentreRad, s.elevationHalfWidthRad = c.elevationHalfWidthRad;
    s.bars = c.bars, s.rangeScaleM = c.rangeScaleM;
    return s;
}

/// A scan about a place until the radar has a confirmed track of the target: the track's number.
std::uint64_t scanFor(Rig& rig, radar::Radar& r, std::uint32_t target, double elevationRad = 0.0) {
    radar::Search s;
    s.azimuthHalfWidthRad = 30 * kDeg;
    s.elevationCentreRad = elevationRad;
    s.bars = 1;
    s.rangeScaleM = 80e3;
    r.search(s, rig.time);
    for (int i = 0; i < 400 && !Rig::trackOf(r, target); ++i) rig.step(r, 0.05);
    const auto* t = Rig::trackOf(r, target);
    REQUIRE(t != nullptr);
    return t->id;
}

radar::Search trackOn(std::uint64_t id, radar::Mode mode, bool illuminate = false) {
    radar::Search s;
    s.task = radar::Task::Track;
    s.mode = mode;
    s.track = id;
    s.illuminate = illuminate;
    s.rangeScaleM = 80e3;
    return s;
}

} // namespace

TEST_CASE("radar modes (MS-4b): single-target track - every dwell on its track, the beam following it, its track far better than a scan's",
          "[ms][radar][radar-modes]") {
    session::World w(options("radar-stt"));
    Rig rig(w);
    const auto target = rig.target(5.0, 40.0, 5000.0, 0.0, -200.0); // head on, 5 km to the left
    radar::Radar r(specOf("an-apg-68v9"), 11);
    const auto id = scanFor(rig, r, target);
    rig.run(r, 6.0); // (a few more frames: the scan's track as good as its looks make it)
    REQUIRE(r.hasTrack(id));
    const double scanSigma = r.report(*Rig::trackOf(r, target), rig.time, rig.self).positionSigmaM;

    CHECK(r.task() == radar::Task::Volume);
    CHECK(r.designated() == 0);
    r.search(trackOn(id, radar::Mode::SingleTargetTrack), rig.time);
    CHECK(r.task() == radar::Task::Track);
    CHECK(r.designated() == id);
    CHECK_FALSE(r.illuminating()); // (not told to)
    const auto dwells = r.dwells();
    const auto firstNew = r.detectionCount();
    const double bw = r.pattern().beamwidthRad(false);
    double worst = 0.0;
    int stray = 0;
    const double start = rig.time;
    while (rig.time < start + 5.0 - 1e-9) {
        rig.step(r, 0.05);
        double range, az, el;
        rig.where(target, range, az, el);
        if (rig.time > start + 0.5) worst = std::max(worst, std::hypot(r.beamAzimuthRad() - az, r.beamElevationRad() - el));
        stray += r.tentativeTracks() > 0 ? 1 : 0;
    }
    std::printf("radar modes: %d steps with a tentative track beside the one it follows\n", stray);
    // every dwell on the target: the beam within a fraction of its width of it, and nearly every dwell a detection
    CHECK(worst < 0.25 * bw);
    const double expected = 5.0 / r.nominalDwellS();
    CHECK(static_cast<double>(r.dwells() - dwells) > expected - 2.0);
    CHECK(static_cast<double>(r.dwells() - dwells) < expected + 2.0);
    int hits = 0;
    for (const auto& d : r.detections())
        if (d.serial >= firstNew && d.truth == target) ++hits;
    CHECK(hits > 0.8 * expected);
    // the same track, held to the target far closer than the scan held it
    REQUIRE(r.active());
    REQUIRE(r.hasTrack(id));
    const auto* t = Rig::trackOf(r, target);
    REQUIRE(t != nullptr);
    CHECK(t->id == id);
    const ms::AmtiTrack a = r.report(*t, rig.time, rig.self);
    INFO("scan " << scanSigma << " m, single-target track " << a.positionSigmaM << " m, error " << rig.error(r, *t, target) << " m");
    std::printf("radar modes: at %.0f km a scan's track to %.0f m, single-target track's to %.1f m (%.1f m off, %.2f m/s); %d of %.0f dwells detections\n",
                a.rangeM / 1e3, scanSigma, a.positionSigmaM, rig.error(r, *t, target), a.velocitySigmaMs, hits, expected);
    // one track of it, not several: each dwell's detection falls inside the track the last ones made
    int tracksOfIt = 0;
    for (const auto& other : r.tracks())
        if (other.points.back().truth == target) ++tracksOfIt;
    CHECK(tracksOfIt == 1);
    CHECK(r.tentativeTracks() == 0);
    CHECK(a.positionSigmaM < scanSigma / 3.0);
    CHECK(rig.error(r, *t, target) < 3.0 * std::sqrt(3.0) * a.positionSigmaM + 5.0); // (within what it says of itself)
    // told to light the target, it does - while it tracks
    r.search(trackOn(id, radar::Mode::SingleTargetTrack, true), rig.time);
    rig.run(r, 0.5);
    CHECK(r.illuminating());
    r.stop();
    CHECK_FALSE(r.illuminating());
    CHECK(r.designated() == 0);
}

TEST_CASE("radar modes (MS-4b): a target that turns into the notch is lost when the memory time is up; above the horizon it is kept",
          "[ms][radar][radar-modes]") {
    // The carrier at 5,000 m. A target 25 km ahead, head on, turns through 90 deg at 6 g to fly across the line of sight.
    // Below (1,000 m): the beam on it meets the ground within the range scale, and once its range rate is the ground's
    // there the pulse-Doppler notch takes it out - no detection, the track kept for the memory time, then gone and the
    // radar stopped. Above (9,000 m): no main-beam clutter, the same turn is followed and the track kept.
    for (const bool below : {true, false}) {
        INFO((below ? "below" : "above"));
        session::World w(options(below ? "radar-notch-below" : "radar-notch-above"));
        Rig rig(w);
        const double altitude = below ? 1000.0 : 9000.0;
        const auto target = rig.target(0.0, 25.0, altitude, 0.0, -200.0);
        const radar::Spec spec = specOf("an-apg-68v9");
        radar::Radar r(spec, 5);
        const auto id = scanFor(rig, r, target, std::atan2(altitude - 5000.0, 25e3));
        r.search(trackOn(id, radar::Mode::SingleTargetTrack, true), rig.time);
        rig.run(r, 2.0);
        REQUIRE(r.active());
        // the turn: 6 g at 200 m/s, to the north
        const double rate = 6.0 * 9.80665 / 200.0;
        double heading = -90.0 * kDeg; // (west)
        double notchedAt = std::numeric_limits<double>::quiet_NaN(), lostAt = notchedAt;
        const double turnStart = rig.time;
        while (rig.time < turnStart + 16.0 && !r.lost()) {
            if (heading < 0.0) heading = std::min(0.0, heading + rate * 0.02);
            auto& t = rig.states[target - 1];
            t.velocityNedMs[0] = 200.0 * std::cos(heading), t.velocityNedMs[1] = 200.0 * std::sin(heading);
            rig.step(r, 0.02);
            // the target's range rate against the ground's in the beam (the notch: within the card's half width)
            double range, az, el;
            rig.where(target, range, az, el);
            double n[3], e[3], d[3];
            ned(rig.self.latitudeRad, rig.self.longitudeRad, n, e, d);
            double los[3], along = 0.0;
            for (int i = 0; i < 3; ++i) los[i] = (t.positionEcef[i] - rig.self.positionEcef[i]) / range;
            for (int i = 0; i < 3; ++i) along += (t.velocityNedMs[0] * n[i] + t.velocityNedMs[1] * e[i]) * los[i];
            if (std::isnan(notchedAt) && std::fabs(along) < spec.notchMs) notchedAt = rig.time; // (its own speed along the line of sight: what sets it apart from the ground)
            if (r.lost()) lostAt = rig.time;
        }
        if (below) {
            REQUIRE(std::isfinite(notchedAt));
            REQUIRE(r.lost());
            CHECK_FALSE(r.active());
            CHECK_FALSE(r.illuminating());
            CHECK(r.designated() == 0);
            INFO("notched at " << notchedAt - turnStart << " s into the turn, lost at " << lostAt - turnStart << " s");
            std::printf("radar modes: a 6 g turn across the line of sight, below: notched %.1f s into it, lost %.2f s later\n", notchedAt - turnStart,
                        lostAt - notchedAt);
            CHECK(lostAt - notchedAt > spec.trackMemoryS - 0.3);
            CHECK(lostAt - notchedAt < spec.trackMemoryS + 0.5);
        } else {
            CHECK_FALSE(r.lost());
            REQUIRE(r.active());
            CHECK(r.designated() == id);
            CHECK(r.illuminating());
            const auto* t = Rig::trackOf(r, target);
            REQUIRE(t != nullptr);
            CHECK(t->id == id); // (the same track through the turn)
            CHECK(rig.error(r, *t, target) < 100.0);
        }
    }
}

TEST_CASE("radar modes (MS-4b): a target that passes out of the antenna's reach is lost there", "[ms][radar][radar-modes]") {
    session::World w(options("radar-gimbal"));
    Rig rig(w);
    const auto target = rig.target(6.0, 20.0, 5000.0, 0.0, -200.0); // a parallel course 6 km to the left, head on
    const radar::Spec spec = specOf("an-apg-68v9");
    radar::Radar r(spec, 3);
    const auto id = scanFor(rig, r, target);
    r.search(trackOn(id, radar::Mode::SingleTargetTrack), rig.time);
    const double limit = spec.azimuthLimitDeg * kDeg;
    double outAt = std::numeric_limits<double>::quiet_NaN(), lostAt = outAt;
    while (rig.time < 80.0 && !r.lost()) {
        rig.step(r, 0.02);
        double range, az, el;
        rig.where(target, range, az, el);
        if (std::isnan(outAt) && std::fabs(az) > limit) outAt = rig.time;
        if (r.lost()) lostAt = rig.time;
    }
    REQUIRE(r.lost());
    REQUIRE(std::isfinite(outAt));
    CHECK(std::fabs(std::fabs(r.beamAzimuthRad()) - limit) < 1e-9); // (the beam at its stop)
    INFO("out of reach at " << outAt << " s, lost at " << lostAt << " s");
    std::printf("radar modes: past the antenna's %.0f deg stop, lost %.2f s later\n", spec.azimuthLimitDeg, lostAt - outAt);
    CHECK(lostAt - outAt > spec.trackMemoryS - 1.0); // (its last detections a little past the stop: the beam's skirt)
    CHECK(lostAt - outAt < spec.trackMemoryS + 1.5);
}

TEST_CASE("radar modes (MS-4b): track-while-scan kept on a track - its volume follows the target; an electronic array looks at it between "
          "its search dwells",
          "[ms][radar][radar-modes]") {
    SECTION("the volume follows: a target that crosses out of a fixed volume is kept by one that is told to follow it") {
        double keptCentre = 0.0, trueAz = 0.0;
        bool kept = false, fixed = true;
        for (const bool follow : {true, false}) {
            session::World w(options(follow ? "radar-tws-follow" : "radar-tws-fixed"));
            Rig rig(w);
            const auto target = rig.target(0.0, 30.0, 5000.0, 177.0, -177.0); // north-west: it crosses to the left
            radar::Radar r(specOf("an-apg-68v9"), 9);
            const auto id = scanFor(rig, r, target);
            radar::Search s = trackOn(id, radar::Mode::TrackWhileScan);
            s.azimuthHalfWidthRad = 10 * kDeg;
            s.bars = 2;
            if (!follow) s.task = radar::Task::Volume, s.track = 0; // (the same scan, where it was put)
            r.search(s, rig.time);
            double worst = 0.0;
            const double start = rig.time;
            while (rig.time < start + 35.0) {
                rig.step(r, 0.05);
                double range, az, el;
                rig.where(target, range, az, el);
                trueAz = az;
                if (follow && rig.time > start + 3.0) worst = std::max(worst, std::fabs(r.searching().azimuthCentreRad - az));
            }
            if (follow) {
                kept = r.active() && r.hasTrack(id) && r.designated() == id;
                keptCentre = r.searching().azimuthCentreRad;
                INFO("the target at " << trueAz / kDeg << " deg, the volume's centre " << keptCentre / kDeg << " deg, worst " << worst / kDeg);
                CHECK(worst < 4.0 * kDeg); // (a frame's motion, and the track's own error)
                CHECK(std::fabs(trueAz) > 15 * kDeg); // (well outside the +-10 deg it started in)
            } else {
                fixed = Rig::trackOf(r, target) != nullptr;
            }
        }
        CHECK(kept);
        CHECK_FALSE(fixed); // (the scan left where it was: the target crossed out of it, its track dropped)
    }
    SECTION("an electronic array's track dwells: the designated track updated every second, not once a frame") {
        double gap[2] = {0.0, 0.0}, frame = 0.0;
        for (const int designated : {1, 0}) {
            session::World w(options(designated ? "radar-tws-esa" : "radar-tws-esa-plain"));
            Rig rig(w);
            const auto target = rig.target(3.0, 60.0, 5000.0, 0.0, -200.0);
            const radar::Spec spec = specOf("an-apg-77");
            REQUIRE(spec.aperture.electronic);
            radar::Radar r(spec, 4);
            const auto id = scanFor(rig, r, target);
            radar::Search s = trackOn(id, radar::Mode::TrackWhileScan);
            s.azimuthHalfWidthRad = 60 * kDeg;
            s.bars = 4;
            s.rangeScaleM = 150e3;
            if (!designated) s.task = radar::Task::Volume, s.track = 0;
            r.search(s, rig.time);
            frame = r.framePeriodS();
            double last = rig.time;
            const double start = rig.time;
            while (rig.time < start + 30.0) {
                rig.step(r, 0.02);
                const auto* t = Rig::trackOf(r, target);
                REQUIRE(t != nullptr);
                if (t->lastUpdateT > last) {
                    if (rig.time > start + 2.0) gap[designated] = std::max(gap[designated], t->lastUpdateT - last);
                    last = t->lastUpdateT;
                }
            }
            if (designated) CHECK(spec.trackUpdateS < 0.5 * frame); // (the case shows something: its frame is several seconds)
        }
        INFO("frame " << frame << " s; the longest wait for an update: designated " << gap[1] << " s, not " << gap[0] << " s");
        std::printf("radar modes: an electronic array's %.1f s frame: its designated track's longest wait for an update %.2f s, a track's in the same "
                    "scan %.2f s\n", frame, gap[1], gap[0]);
        CHECK(gap[1] < 2.5); // (a second apart, a miss or two allowed)
        CHECK(gap[0] > 0.7 * frame);
        CHECK(gap[1] < 0.6 * gap[0]);
    }
}

TEST_CASE("radar modes (MS-4b): a cued search acquires what it finds - boresight and vertical scan, the nearest target, none where none is",
          "[ms][radar][radar-modes]") {
    const radar::Spec spec = specOf("an-apg-68v9");
    SECTION("boresight: a target on the nose is looked at again, its track confirmed, then tracked alone") {
        session::World w(options("radar-acm-bore"));
        Rig rig(w);
        const auto target = rig.target(0.0, 8.0, 5000.0, 0.0, 200.0); // ahead, flying away slower
        radar::Radar r(spec, 21);
        const ms::AmtiCommand c = ms::acmCommand(0, ms::AcmPattern::Boresight);
        CHECK(c.task == ms::AmtiTask::Cued);
        CHECK(c.rangeScaleM == 18520.0);
        r.search(searchOf(c), rig.time);
        CHECK(r.task() == radar::Task::Cued);
        const auto dwells = r.dwells();
        bool looked = false;
        while (rig.time < 3.0 && r.task() == radar::Task::Cued) {
            rig.step(r, r.nominalDwellS());
            looked = looked || r.acquiring();
        }
        REQUIRE(r.task() == radar::Task::Track);
        CHECK(looked);
        CHECK(r.searching().mode == radar::Mode::SingleTargetTrack);
        CHECK(static_cast<int>(r.dwells() - dwells) >= spec.trackConfirm);
        CHECK(rig.time < 1.5);
        const auto* t = Rig::trackOf(r, target);
        REQUIRE(t != nullptr);
        const auto id = t->id;
        CHECK(r.designated() == id);
        rig.run(r, 3.0);
        CHECK(r.active());
        CHECK(r.designated() == id);
    }
    SECTION("boresight: nothing on the nose, nothing acquired; a target beyond its 10 nmi, not either") {
        session::World w(options("radar-acm-none"));
        Rig rig(w);
        rig.target(3.0, 8.0, 5000.0, 0.0, 200.0);  // 20 deg off the nose
        rig.target(0.0, 25.0, 5000.0, 0.0, 250.0); // on the nose, 25 km
        radar::Radar r(spec, 22);
        r.search(searchOf(ms::acmCommand(0, ms::AcmPattern::Boresight)), rig.time);
        rig.run(r, 5.0);
        CHECK(r.active());
        CHECK(r.task() == radar::Task::Cued);
        CHECK(r.designated() == 0);
    }
    SECTION("boresight: of two on the nose, the nearer") {
        session::World w(options("radar-acm-two"));
        Rig rig(w);
        rig.target(0.0, 10.0, 5000.0, 0.0, 200.0);
        const auto near = rig.target(0.0, 6.0, 5000.0, 0.0, 220.0);
        radar::Radar r(spec, 23);
        r.search(searchOf(ms::acmCommand(0, ms::AcmPattern::Boresight)), rig.time);
        rig.run(r, 3.0);
        REQUIRE(r.task() == radar::Task::Track);
        const auto* t = Rig::trackOf(r, near);
        REQUIRE(t != nullptr);
        CHECK(r.tracks().size() == 2); // (one track of each: both are in the beam)
        CHECK(r.designated() == t->id);
    }
    SECTION("vertical scan: a target 30 deg above the nose, out of a level scan's reach, is acquired") {
        session::World w(options("radar-acm-vertical"));
        Rig rig(w);
        const auto target = rig.target(0.0, 6.93, 9000.0, 0.0, 250.0); // 8 km away, 30 deg up, keeping its place
        radar::Radar r(spec, 24);
        const ms::AmtiCommand c = ms::acmCommand(0, ms::AcmPattern::VerticalScan);
        r.search(searchOf(c), rig.time);
        const double stepEl = 0.8 * r.pattern().beamwidthRad(true);
        CHECK(r.bars() == static_cast<int>(std::ceil(60.0 * kDeg / stepEl - 1e-9)));
        const double frame = r.framePeriodS();
        while (rig.time < 4.0 * frame && r.task() == radar::Task::Cued) rig.step(r, 0.02);
        INFO("a frame of " << frame << " s, acquired at " << rig.time << " s");
        std::printf("radar modes: vertical scan, %d bars, a frame of %.1f s: a target 30 deg up acquired %.1f s in\n", r.bars(), frame, rig.time);
        REQUIRE(r.task() == radar::Task::Track);
        const auto* t = Rig::trackOf(r, target);
        REQUIRE(t != nullptr);
        CHECK(r.designated() == t->id);
        CHECK(r.beamElevationRad() > 25 * kDeg);
    }
    SECTION("cued on a track: the volume put about it, and that track the one acquired") {
        session::World w(options("radar-cue-track"));
        Rig rig(w);
        const auto target = rig.target(8.0, 30.0, 5000.0, 0.0, -200.0); // 15 deg to the left
        rig.target(-8.0, 30.0, 5000.0, 0.0, -200.0);                    // and one as far to the right
        radar::Radar r(spec, 25);
        const auto id = scanFor(rig, r, target);
        radar::Search s;
        s.task = radar::Task::Cued;
        s.track = id;
        s.azimuthHalfWidthRad = 3 * kDeg;
        s.bars = 1;
        s.rangeScaleM = 80e3;
        r.search(s, rig.time);
        rig.run(r, 3.0);
        REQUIRE(r.task() == radar::Task::Track);
        CHECK(r.designated() == id);
        const auto* t = Rig::trackOf(r, target);
        REQUIRE(t != nullptr);
        CHECK(t->id == id);
    }
}

namespace {

/// A world of two F-16Cs, the second `aheadKm` ahead of the first on the heading given; the first's radar operating.
struct Pair {
    session::World w;
    std::uint32_t own = 0, target = 0;
    Pair(const char* name, double aheadKm, double headingDeg, bool publish = false) : w(with(name, publish)) {
        own = wing(w, "f16c", 5000.0, 200.0);
        session::VehicleSpec s;
        s.name = "target";
        s.type = "jsbsim:f16c";
        s.initial.altitudeMslM = 5000.0;
        s.initial.headingDeg = headingDeg;
        s.initial.airspeedTrueMs = 200.0;
        s.initial.longitudeDeg += aheadKm * 1e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
        target = w.createVehicle(s);
        REQUIRE(target != 0);
        REQUIRE(w.submit(own, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
        for (int i = 0; i < 600 && w.missionSystems(own)->payloads()[0].state != ms::PayloadState::Operate; ++i) w.step();
        REQUIRE(w.missionSystems(own)->payloads()[0].state == ms::PayloadState::Operate);
    }
    static session::WorldOptions with(const char* name, bool publish) {
        session::WorldOptions o = options(name);
        o.publish = publish;
        return o;
    }
    void run(double seconds) {
        const double end = w.simTime() + seconds;
        while (w.simTime() < end) w.step();
    }
    /// The radar's track of the target (0: none).
    std::uint64_t track() const {
        for (const auto& t : w.amtiTracks(own, 0))
            if (t.truth == target) return t.id;
        return 0;
    }
};

} // namespace

TEST_CASE("radar modes through the World (MS-4b): track and illuminate, the status and the viewers' record, the activity failed when the "
          "target is lost",
          "[ms][radar][radar-modes][ipc]") {
    Pair p("radar-modes-world", 40.0, 270.0, true);
    session::World& w = p.w;
    // what is refused: a track it does not hold, modes that do not go with the task
    ms::AmtiCommand c;
    c.task = ms::AmtiTask::Track, c.mode = ms::AmtiMode::SingleTargetTrack, c.track = 77;
    CHECK(w.submit(p.own, c).reason == ms::Reason::UnknownTrack);
    CHECK(std::string(ms::reasonName(ms::Reason::UnknownTrack)) == "unknown_track");
    CHECK(std::string(ms::reasonName(ms::Reason::TrackLost)) == "track_lost");
    c.track = 0;
    CHECK(w.submit(p.own, c).reason == ms::Reason::InvalidParameter); // (a track task names its track)
    c = {};
    c.mode = ms::AmtiMode::SingleTargetTrack;
    CHECK(w.submit(p.own, c).reason == ms::Reason::InvalidParameter); // (a volume is not tracked)
    c = {};
    c.illuminate = true;
    CHECK(w.submit(p.own, c).reason == ms::Reason::InvalidParameter); // (a search lights nothing)
    c = {};
    c.task = ms::AmtiTask::Cued, c.mode = ms::AmtiMode::VelocitySearch;
    CHECK(w.submit(p.own, c).reason == ms::Reason::InvalidParameter); // (high PRF: no range, no track)
    CHECK(std::string(ms::amtiModeName(ms::AmtiMode::SingleTargetTrack)) == "single_target_track");
    CHECK(std::string(ms::amtiTaskName(ms::AmtiTask::Cued)) == "cued");

    // a search, a track of the target, then: track it alone and light it
    c = {};
    c.azimuthHalfWidthRad = 30 * kDeg;
    REQUIRE(w.submit(p.own, c).accepted());
    for (int i = 0; i < 4000 && !p.track(); ++i) w.step();
    const std::uint64_t id = p.track();
    REQUIRE(id != 0);
    ms::AmtiCommand bad;
    bad.task = ms::AmtiTask::Track, bad.mode = ms::AmtiMode::TrackWhileScan, bad.track = id, bad.illuminate = true;
    CHECK(w.submit(p.own, bad).reason == ms::Reason::InvalidParameter); // (a scan lights nothing)
    ms::AmtiCommand track;
    track.task = ms::AmtiTask::Track, track.mode = ms::AmtiMode::SingleTargetTrack, track.track = id, track.illuminate = true;
    const auto accepted = w.submit(p.own, track);
    REQUIRE(accepted.accepted());
    p.run(2.0);
    ms::RadarStatus st = w.radarStatus(p.own, 0);
    CHECK(st.searching);
    CHECK(st.activity == accepted.activity);
    CHECK(st.mode == ms::AmtiMode::SingleTargetTrack);
    CHECK(st.task == ms::AmtiTask::Track);
    CHECK(st.designatedTrack == id);
    CHECK(st.illuminating);
    CHECK_FALSE(st.acquiring);
    CHECK(st.trackMemoryS == 3.0);
    CHECK(st.bars == 1);
    CHECK(std::fabs(st.azimuthCentreRad - st.beamAzimuthRad) < 1e-12); // (its volume is its beam)
    CHECK(p.track() == id);
    // the viewers' record: the track it follows first and flagged, its task and its illumination
    w.publishNow();
    ipc::WorldMirror mirror;
    REQUIRE(mirror.open("radar-modes-world"));
    ipc::MissionRecord m;
    REQUIRE(mirror.mission(0, m));
    REQUIRE(m.radarCount == 1);
    CHECK(m.radars[0].mode == static_cast<std::uint8_t>(ms::AmtiMode::SingleTargetTrack));
    CHECK(m.radars[0].task == static_cast<std::uint8_t>(ms::AmtiTask::Track));
    CHECK(m.radars[0].illuminating == 1);
    REQUIRE(m.radars[0].trackCount >= 1);
    CHECK(m.radars[0].tracks[0].id == static_cast<std::uint32_t>(id));
    CHECK(m.radars[0].tracks[0].flags == ipc::kMissionTrackDesignated);

    // the target gone (removed from the world: nothing comes back): for the memory time it is still tracked and lit,
    // then the activity has failed - track_lost - and the radar has stopped
    REQUIRE(w.removeVehicle(p.target));
    p.run(1.5);
    st = w.radarStatus(p.own, 0);
    CHECK(st.searching);
    CHECK(st.illuminating);
    p.run(2.5);
    st = w.radarStatus(p.own, 0);
    CHECK_FALSE(st.searching);
    CHECK_FALSE(st.illuminating);
    CHECK(st.designatedTrack == 0);
    CHECK(st.activity == 0);
    const ms::ActivityRecord* a = w.missionSystems(p.own)->activity(accepted.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == control::ActivityState::Failed);
    CHECK(a->reason == ms::Reason::TrackLost);
    w.publishNow();
    REQUIRE(mirror.mission(0, m));
    CHECK(m.radars[0].searching == 0);
    CHECK(m.radars[0].illuminating == 0);
}

TEST_CASE("radar modes through the World (MS-4b): an air-combat acquisition - boresight finds the aircraft ahead and tracks it; the same from "
          "the same seed",
          "[ms][radar][radar-modes]") {
    std::uint64_t dwells[2] = {0, 0};
    double range[2] = {0.0, 0.0};
    for (int k = 0; k < 2; ++k) {
        Pair p(k ? "radar-acm-world-b" : "radar-acm-world-a", 8.0, 90.0); // 8 km ahead, the same way
        session::World& w = p.w;
        const auto accepted = w.submit(p.own, ms::acmCommand(0, ms::AcmPattern::Boresight));
        REQUIRE(accepted.accepted());
        ms::RadarStatus st = w.radarStatus(p.own, 0);
        CHECK(st.task == ms::AmtiTask::Cued);
        CHECK(st.designatedTrack == 0);
        for (int i = 0; i < 2000 && w.radarStatus(p.own, 0).task == ms::AmtiTask::Cued; ++i) w.step();
        st = w.radarStatus(p.own, 0);
        REQUIRE(st.task == ms::AmtiTask::Track);
        CHECK(st.mode == ms::AmtiMode::SingleTargetTrack);
        CHECK(st.activity == accepted.activity); // (the same activity: the acquisition became the track)
        CHECK(st.designatedTrack != 0);
        CHECK(st.designatedTrack == p.track());
        p.run(3.0);
        st = w.radarStatus(p.own, 0);
        CHECK(st.searching);
        dwells[k] = st.dwells;
        for (const auto& t : w.amtiTracks(p.own, 0))
            if (t.id == st.designatedTrack) range[k] = t.rangeM;
        CHECK(range[k] > 7000.0);
        CHECK(range[k] < 9000.0);
        // a rotodome acquires nothing: an early-warning radar searches
        if (k == 0) {
            const auto sentry = wing(w, "e3g", 9000.0, 200.0, 5);
            REQUIRE(w.submit(sentry, ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
            for (int i = 0; i < 6000 && w.missionSystems(sentry)->payloads()[0].state != ms::PayloadState::Operate; ++i) w.step();
            ms::AmtiCommand cue = ms::acmCommand(0, ms::AcmPattern::Boresight);
            CHECK(w.submit(sentry, cue).reason == ms::Reason::InvalidParameter);
        }
    }
    CHECK(dwells[0] == dwells[1]);
    CHECK(range[0] == range[1]);
}
