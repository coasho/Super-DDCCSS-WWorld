// The radio path's terrain profile (ADR-30, MS-2/MS-4): the great circle's
// points as the recurrence makes them against geo::destination's; a profile
// read a tile at a time against the same points asked one by one; and over
// the map set's real relief, the profile read from its own data (each point
// from the deepest level there) against the old reading - each point asked of the vehicles' level 15 - its
// differences held to that old reading's own sampling noise, at a fraction of
// its cost.
#include "core/Geodesy.h"
#include "io/TerrainTiles.h"
#include "ms/rf/Path.h"
#include "platform/Paths.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <random>
#include <vector>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

/// A provider that answers point by point only: the profile as it was read before (each point asked of its level).
class PointByPoint final : public sim::GroundProvider {
public:
    explicit PointByPoint(const sim::GroundProvider& g) : g_(g) {}
    double heightAboveEllipsoidM(double lat, double lon) const override { return g_.heightAboveEllipsoidM(lat, lon); }
    double resolutionM() const noexcept override { return g_.resolutionM(); }

private:
    const sim::GroundProvider& g_;
};

double percentile(std::vector<double> v, double p) {
    std::sort(v.begin(), v.end());
    return v.empty() ? 0.0 : v[std::min(v.size() - 1, static_cast<std::size_t>(p * static_cast<double>(v.size())))];
}

} // namespace

TEST_CASE("terrain profile: the great circle's points, and a profile read a tile at a time", "[rf][terrain]") {
    // the recurrence's points against geo::destination's along the same course
    const double lat0 = 37.0 * kDeg, lon0 = -120.0 * kDeg, lat1 = 38.4 * kDeg, lon1 = -117.1 * kDeg;
    const int n = 2001;
    const double d = geo::distanceM(lat0, lon0, lat1, lon1), course = geo::bearingRad(lat0, lon0, lat1, lon1);
    double worst = 0.0;
    sim::GroundProvider::greatCircle(lat0, lon0, lat1, lon1, n, [&](int i, double lat, double lon) {
        double la, lo;
        geo::destination(lat0, lon0, course, d * i / (n - 1), la, lo);
        worst = std::max({worst, std::fabs(lat - la), std::fabs(lon - lo)});
    });
    CHECK(worst < 1e-10); // (under a millimetre)

    // a profile at the vehicles' level: the same ground as its points asked one at a time
    const auto maps = platform::mapsDir();
    if (!std::filesystem::is_directory(maps / "s3.amazonaws.com" / "elevation-tiles-prod" / "terrarium")) SKIP("no map set");
    io::TerrainTiles::Options o;
    o.zoom = 15;
    o.offline = true;
    io::TerrainTiles tiles(o);
    std::vector<double> profile(static_cast<std::size_t>(n)), points(static_cast<std::size_t>(n));
    const double spacing = d / (n - 1);
    tiles.profile(lat0, lon0, lat1, lon1, n, spacing, profile.data());
    sim::GroundProvider::greatCircle(lat0, lon0, lat1, lon1, n, [&](int i, double lat, double lon) { points[static_cast<std::size_t>(i)] = tiles.heightAboveEllipsoidM(lat, lon); });
    // (the profile reads the data a missing level is made from; a point, that level's tile made from it - bilinear of
    // a bilinear resample, extrapolated over the outer half of its 4.8 m texel: the same ground to within that texel)
    double most = 0.0, sum = 0.0;
    for (std::size_t i = 0; i < profile.size(); ++i) most = std::max(most, std::fabs(profile[i] - points[i])), sum += std::fabs(profile[i] - points[i]);
    const double mean = sum / static_cast<double>(profile.size());
    std::printf("terrain profile against its points: mean %.3f m, most %.3f m\n", mean, most);
    CHECK(mean < 0.05);
    CHECK(most < 2.0);
}

TEST_CASE("terrain profile: read from the map set's own data, as faithful as the old reading and far cheaper", "[rf][terrain][mapset]") {
    const auto maps = platform::mapsDir();
    if (!std::filesystem::is_directory(maps / "s3.amazonaws.com" / "elevation-tiles-prod" / "terrarium")) SKIP("no map set");
    io::TerrainTiles::Options o; // the vehicles' ground, as the World makes it
    o.zoom = 15;
    o.offline = true;
    io::TerrainTiles physics(o), separate(o); // (each its own caches: neither reading warmed by the other)
    const PointByPoint old(separate);
    // paths across the Coast Ranges, the Central Valley and the Sierra Nevada: aircraft 100 m - 3 km up, 20 - 350 km apart
    std::mt19937_64 rng(11);
    std::uniform_real_distribution<double> lat(35.0, 39.5), lon(-123.0, -117.0), up(100.0, 3000.0), len(20e3, 350e3), dir(0.0, 6.283185307);
    std::vector<double> change, noise;
    double oldS = 0.0, newS = 0.0;
    int obstructed = 0;
    const int paths = 400;
    for (int k = 0; k < paths; ++k) {
        const double la = lat(rng) * kDeg, lo = lon(rng) * kDeg;
        double lb, lob;
        geo::destination(la, lo, dir(rng), len(rng), lb, lob);
        const ms::rf::Terminal a{la, lo, physics.heightAboveEllipsoidM(la, lo) + up(rng)};
        const ms::rf::Terminal b{lb, lob, physics.heightAboveEllipsoidM(lb, lob) + up(rng)};
        ms::rf::PathOptions p;
        p.frequencyHz = 9.5e9;
        p.ground = &old;
        auto t0 = std::chrono::steady_clock::now();
        const auto before = ms::rf::computePath(a, b, p);
        auto t1 = std::chrono::steady_clock::now();
        p.ground = &physics;
        const auto now = ms::rf::computePath(a, b, p);
        auto t2 = std::chrono::steady_clock::now();
        oldS += std::chrono::duration<double>(t1 - t0).count(), newS += std::chrono::duration<double>(t2 - t1).count();
        // the old reading's own noise: the same path sampled a tenth of a spacing along (as its endpoints move each look)
        const double spacing = std::max(30.0, before.groundRangeM / 2000.0);
        double ja, joa, jb, job;
        const double course = geo::bearingRad(la, lo, lb, lob);
        geo::destination(la, lo, course, 0.1 * spacing, ja, joa);
        geo::destination(lb, lob, course, 0.1 * spacing, jb, job);
        p.ground = &old;
        const auto jittered = ms::rf::computePath({ja, joa, a.heightM}, {jb, job, b.heightM}, p);
        if (before.terrainDb <= 0.0 && now.terrainDb <= 0.0 && jittered.terrainDb <= 0.0) continue; // (clear of it: alike)
        ++obstructed; // (the statistics over the paths the terrain is in)
        change.push_back(std::fabs(now.terrainDb - before.terrainDb));
        noise.push_back(std::fabs(jittered.terrainDb - before.terrainDb));
    }
    std::printf("terrain profile: %d of %d paths with terrain loss; there |new - old| p50 %.2f p95 %.2f max %.2f dB; the old reading's own "
                "noise p50 %.2f p95 %.2f max %.2f dB; cost %.2f ms a path, was %.2f\n",
                obstructed, paths, percentile(change, 0.5), percentile(change, 0.95), percentile(change, 1.0), percentile(noise, 0.5),
                percentile(noise, 0.95), percentile(noise, 1.0), newS / paths * 1e3, oldS / paths * 1e3);
    CHECK(obstructed >= 60); // (the comparison means something: relief in the way)
    CHECK(percentile(change, 0.5) <= percentile(noise, 0.5) + 0.1);
    CHECK(percentile(change, 0.95) <= percentile(noise, 0.95) + 0.5);
    CHECK(newS < 0.25 * oldS);

    // warm: what a path costs once its tiles are decoded, and the profile's part of it
    std::mt19937_64 again(11);
    double profileS = 0.0, pathS = 0.0;
    std::vector<double> heights(2001);
    for (int k = 0; k < paths; ++k) {
        const double la = lat(again) * kDeg, lo = lon(again) * kDeg;
        double lb, lob;
        geo::destination(la, lo, dir(again), len(again), lb, lob);
        const ms::rf::Terminal a{la, lo, physics.heightAboveEllipsoidM(la, lo) + up(again)};
        const ms::rf::Terminal b{lb, lob, physics.heightAboveEllipsoidM(lb, lob) + up(again)};
        ms::rf::PathOptions p;
        p.frequencyHz = 9.5e9;
        p.ground = &physics;
        const double d = geo::distanceM(la, lo, lb, lob);
        const int n = std::clamp(static_cast<int>(std::ceil(d / 30.0)) + 1, 3, 2001);
        (void)ms::rf::computePath(a, b, p); // (its tiles decoded)
        auto t0 = std::chrono::steady_clock::now();
        physics.profile(la, lo, lb, lob, n, d / (n - 1), heights.data());
        auto t1 = std::chrono::steady_clock::now();
        const auto r = ms::rf::computePath(a, b, p);
        auto t2 = std::chrono::steady_clock::now();
        profileS += std::chrono::duration<double>(t1 - t0).count(), pathS += std::chrono::duration<double>(t2 - t1).count();
        CHECK(std::isfinite(r.totalDb));
    }
    std::printf("terrain profile, warm: a path %.1f us, of which the profile %.1f us\n", pathS / paths * 1e6, profileS / paths * 1e6);
}
