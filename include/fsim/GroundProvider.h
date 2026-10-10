#pragma once

#include <atomic>
#include <cmath>
#include <limits>
#include <optional>

namespace fsim::sim {

/// Terrain height source for physics (design 7.3). Must be thread-safe: it is
/// called concurrently from every simulation worker, several times per step.
/// The v1 implementation samples the same elevation tile pyramid the viewer
/// renders (design 8.2); `FlatGround` is the M0 stand-in and the unit-test
/// fixture.
class GroundProvider {
public:
    virtual ~GroundProvider() = default;

    /// Terrain height above the WGS-84 ellipsoid, metres, at a geodetic position.
    virtual double heightAboveEllipsoidM(double latitudeRad, double longitudeRad) const = 0;
    /// The height where the provider has data, empty where it has none (a
    /// terrain tile it cannot load, which heightAboveEllipsoidM reads as sea
    /// level): what the terrain query answers (docs/flight-autonomy.md, 4.19).
    /// Every point, unless overridden.
    virtual std::optional<double> knownHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
        return heightAboveEllipsoidM(latitudeRad, longitudeRad);
    }
    /// The spacing of its data, m: a commanded path is checked against the
    /// ground this far apart (4.19). Infinite for ground that is flat.
    virtual double resolutionM() const noexcept { return 30.0; }

    /// A terrain profile (the radio path's, MS-2): the heights at `n` points (n >= 2) evenly spaced along the great circle
    /// from one position to another, both ends included, into `out`; `spacingM` their spacing. A provider may read its
    /// data at the detail that spacing resolves (TerrainTiles: the pyramid level whose texel is no larger than it).
    /// Unless overridden, heightAboveEllipsoidM at each point.
    virtual void profile(double lat0, double lon0, double lat1, double lon1, int n, double spacingM, double* out) const {
        (void)spacingM;
        greatCircle(lat0, lon0, lat1, lon1, n, [&](int i, double lat, double lon) { out[i] = heightAboveEllipsoidM(lat, lon); });
    }

    /// heightAboveEllipsoidM's value at a point a sensor asks about (the radar's clutter, MS-6b), read apart from the
    /// ground the vehicles stand on: a provider may answer it without making or keeping what its vehicles' queries
    /// would (TerrainTiles: a missing level's tile made from its ancestor). The same value, to the bit. Unless
    /// overridden, heightAboveEllipsoidM.
    virtual double pointHeightAboveEllipsoidM(double latitudeRad, double longitudeRad) const {
        return heightAboveEllipsoidM(latitudeRad, longitudeRad);
    }

    /// The `n` points evenly spaced along the great circle from one position to another (the ends included), each
    /// handed to `at(i, latitudeRad, longitudeRad)`: a rotation by a constant angle, as a three-term recurrence (two
    /// atan2 a point; drift ~n x 1e-16 rad).
    template <class F>
    static void greatCircle(double lat0, double lon0, double lat1, double lon1, int n, F&& at) {
        greatCircleUnit(lat0, lon0, lat1, lon1, n, [&](int i, const double* p) {
            if (i == n - 1) at(i, lat1, lon1); // (the far end exactly)
            else at(i, std::atan2(p[2], std::hypot(p[0], p[1])), std::atan2(p[1], p[0]));
        });
    }

    /// The same points as unit vectors (x towards 0 deg N 0 deg E, z north), handed to `at(i, p)`: for a caller that
    /// wants something other than the latitude and longitude of each (no trigonometry a point).
    template <class F>
    static void greatCircleUnit(double lat0, double lon0, double lat1, double lon1, int n, F&& at) {
        const double c0 = std::cos(lat0), c1 = std::cos(lat1);
        const double a[3] = {c0 * std::cos(lon0), c0 * std::sin(lon0), std::sin(lat0)};
        const double b[3] = {c1 * std::cos(lon1), c1 * std::sin(lon1), std::sin(lat1)};
        const double cross[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
        const double sinW = std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
        const double omega = std::atan2(sinW, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
        if (n < 2 || !(sinW > 1e-15)) { // (one point, or the two ends together)
            for (int i = 0; i < n; ++i) at(i, a);
            return;
        }
        // the unit vector in the circle's plane at right angles to a, towards b
        const double u[3] = {(b[0] - a[0] * std::cos(omega)) / sinW, (b[1] - a[1] * std::cos(omega)) / sinW, (b[2] - a[2] * std::cos(omega)) / sinW};
        const double delta = omega / static_cast<double>(n - 1), twoCos = 2.0 * std::cos(delta);
        double prev[3], cur[3] = {a[0], a[1], a[2]};
        const double sd = std::sin(delta), cd = std::cos(delta);
        for (int k = 0; k < 3; ++k) prev[k] = a[k] * cd - u[k] * sd; // (the point a step before a)
        for (int i = 0; i < n; ++i) {
            at(i, i == n - 1 ? b : cur);
            for (int k = 0; k < 3; ++k) {
                const double next = twoCos * cur[k] - prev[k];
                prev[k] = cur[k], cur[k] = next;
            }
        }
    }
};

/// Constant-elevation ground.
class FlatGround final : public GroundProvider {
public:
    explicit FlatGround(double elevationM = 0.0) noexcept : elevationM_(elevationM) {}

    double heightAboveEllipsoidM(double, double) const override { return elevationM_.load(std::memory_order_relaxed); }
    double resolutionM() const noexcept override { return std::numeric_limits<double>::infinity(); }
    void setElevationM(double m) noexcept { elevationM_.store(m, std::memory_order_relaxed); }

private:
    std::atomic<double> elevationM_;
};

} // namespace fsim::sim

namespace fsim {
using sim::FlatGround;
using sim::GroundProvider;
}
