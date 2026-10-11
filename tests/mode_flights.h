#pragma once

// Flights for the Vehicle Interface's modes (docs/vehicle-interface.md): a
// wing at its speed, a rotorcraft settled in its hover, the wind, the track.

#include "fsim/Control.h"
#include "session/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fsim::modes {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDeg = kPi / 180.0;
inline constexpr double kEarthM = 6371000.0;

inline session::WorldOptions options(const char* name) {
    session::WorldOptions o;
    o.name = name;
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 17;
    return o;
}

inline unsigned stepsFor(const session::World& w, double seconds) { return static_cast<unsigned>(std::lround(seconds / (w.dt() * w.frameSkip()))); }

/// A fixed wing flying east at its altitude and speed; `index` spaces vehicles 3 km apart north.
/// `fuelFraction`: its tanks each that full; not given, its design's own fill.
inline std::uint32_t wing(session::World& w, const std::string& type, double altitudeM, double tasMs, int index = 0,
                          double fuelFraction = std::numeric_limits<double>::quiet_NaN()) {
    session::VehicleSpec s;
    s.initial.fuelFraction = fuelFraction;
    s.name = type + "-" + std::to_string(index);
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += 0.03 * index;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = tasMs;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

/// The fuel a fixed wing of that type starts with to have room for `loadKg` under its maximum take-off weight
/// (docs/hangar.md, The most an aircraft may weigh): its design's own (NaN) where that leaves the room, else the
/// fraction of its tanks that does - 50 kg to spare.
inline double fuelWithRoomFor(session::World& w, const std::string& type, double loadKg) {
    const auto probe = wing(w, type, 5000.0, 200.0, 97);
    w.step();
    const auto* profile = w.profile(probe);
    const double most = profile ? profile->envelope.maxTakeoffKg : std::numeric_limits<double>::quiet_NaN();
    const double mass = w.massKg(probe);
    const auto nav = w.navigationReport(probe);
    REQUIRE(w.removeVehicle(probe));
    if (!std::isfinite(most) || mass + loadKg + 50.0 <= most) return std::numeric_limits<double>::quiet_NaN();
    REQUIRE(nav.capacity > 0.0);
    const double fraction = (most - 50.0 - loadKg - (mass - nav.fuelKg)) / nav.capacity;
    REQUIRE(fraction > 0.0);
    return fraction;
}

/// A rotorcraft hovering at 100 m, spawned at its hover attitude and held still over the ground for `settleS`.
inline std::uint32_t rotor(session::World& w, const std::string& type, double settleS = 15.0, int index = 0) {
    double pitchDeg = 0.0, rollDeg = 0.0;
    {
        session::VehicleSpec probe;
        probe.name = type + "-probe";
        probe.type = "jsbsim:" + type;
        probe.initial.latitudeDeg += 1.0;
        probe.initial.altitudeMslM = 100.0;
        const auto id = w.createVehicle(probe);
        REQUIRE(id != 0);
        const auto& hover = w.profile(id)->hover;
        if (std::isfinite(hover.pitchAttitudeRad)) pitchDeg = hover.pitchAttitudeRad / kDeg;
        if (std::isfinite(hover.rollAttitudeRad)) rollDeg = hover.rollAttitudeRad / kDeg;
        REQUIRE(w.removeVehicle(id));
    }
    session::VehicleSpec s;
    s.name = type + "-" + std::to_string(index);
    s.type = "jsbsim:" + type;
    s.initial.latitudeDeg += 0.02 * index;
    s.initial.altitudeMslM = 100.0;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 0.0;
    s.initial.pitchDeg = pitchDeg;
    s.initial.rollDeg = rollDeg;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    control::VelocityCommand still;
    still.verticalSpeedMs = 0.0;
    still.northMs = still.eastMs = 0.0;
    REQUIRE(w.submit(id, still).accepted());
    if (settleS > 0.0) w.step(stepsFor(w, settleS));
    return id;
}

/// The wind, blowing from `fromDeg` true.
inline void setWind(session::World& w, double fromDeg, double speedMs) {
    sim::EnvironmentState e = w.environment();
    e.windDirectionDeg = fromDeg;
    e.windSpeedMs = speedMs;
    w.setEnvironment(e);
}

/// The track over the ground, rad; the ground speed, m/s.
inline double track(const sim::VehicleState& s) { return std::atan2(s.velocityNedMs[1], s.velocityNedMs[0]); }
inline double groundSpeed(const sim::VehicleState& s) { return std::hypot(s.velocityNedMs[0], s.velocityNedMs[1]); }
/// a - b, wrapped to +-180 deg, in degrees.
inline double degreesApart(double a, double b) { return std::remainder(a - b, 2.0 * kPi) / kDeg; }

/// North and east of (lat0, lon0), m.
inline void offset(const sim::VehicleState& s, double lat0, double lon0, double& north, double& east) {
    north = (s.latitudeRad - lat0) * kEarthM;
    east = (s.longitudeRad - lon0) * kEarthM * std::cos(lat0);
}

/// A taxi's path as the host draws it (docs/flight-autonomy.md, 4.51), drawn again here: the legs through `points` (north,
/// east; the first where it starts), each corner an arc of `radius` tangent to both legs. How far (north, east) is from it.
inline double taxiPathDistance(const std::vector<std::pair<double, double>>& points, double radius, double north, double east) {
    const std::size_t n = points.size() - 1;
    std::vector<double> course(n), length(n), tangent(n + 1, 0.0), turn(n + 1, 0.0);
    for (std::size_t k = 0; k < n; ++k) {
        course[k] = std::atan2(points[k + 1].second - points[k].second, points[k + 1].first - points[k].first);
        length[k] = std::hypot(points[k + 1].second - points[k].second, points[k + 1].first - points[k].first);
    }
    for (std::size_t j = 1; j < n; ++j) {
        turn[j] = std::remainder(course[j] - course[j - 1], 2.0 * kPi);
        tangent[j] = radius * std::tan(0.5 * std::abs(turn[j]));
    }
    double best = 1e18;
    for (std::size_t k = 0; k < n; ++k) {
        const double c = course[k], un = std::cos(c), ue = std::sin(c);
        const double n0 = points[k].first + tangent[k] * un, e0 = points[k].second + tangent[k] * ue, len = length[k] - tangent[k] - tangent[k + 1];
        const double t = std::clamp((north - n0) * un + (east - e0) * ue, 0.0, len);
        best = std::min(best, std::hypot(north - (n0 + t * un), east - (e0 + t * ue)));
        if (k + 1 < n && std::abs(turn[k + 1]) > 1e-9) {
            const double side = turn[k + 1] > 0.0 ? 1.0 : -1.0, an = points[k + 1].first - tangent[k + 1] * un, ae = points[k + 1].second - tangent[k + 1] * ue;
            const double cn = an - side * radius * ue, ce = ae + side * radius * un;
            for (int i = 0; i <= 200; ++i) { // (the arc sampled every half percent of its sweep)
                const double h = c + side * std::abs(turn[k + 1]) * i / 200.0;
                best = std::min(best, std::hypot(north - (cn + side * radius * std::sin(h)), east - (ce - side * radius * std::cos(h))));
            }
        }
    }
    return best;
}

/// Ground that rises `gradient` metres per metre eastward from `longitudeRad` (flat to its west).
class RisingGround final : public sim::GroundProvider {
public:
    RisingGround(double longitudeRad, double latitudeRad, double gradient) noexcept
        : lon0_(longitudeRad), metresPerRad_(kEarthM * std::cos(latitudeRad)), gradient_(gradient) {}
    double heightAboveEllipsoidM(double, double longitudeRad) const override {
        const double east = (longitudeRad - lon0_) * metresPerRad_;
        return east > 0.0 ? gradient_ * east : 0.0;
    }

private:
    double lon0_, metresPerRad_, gradient_;
};

} // namespace fsim::modes
