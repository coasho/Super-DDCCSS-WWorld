#include "ms/rf/Path.h"

#include "core/Geodesy.h"
#include "fsim/GroundProvider.h"
#include "ms/rf/Ntia.h"
#include "ms/rf/Rain.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace fsim::ms::rf {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLight = 299792458.0;
constexpr double kA = 6378137.0;               // WGS-84
constexpr double kE2 = 6.69437999014e-3;
constexpr double kEffectiveRadiusM = 6371e3 * 4.0 / 3.0; // the median effective earth (k = 4/3)
constexpr double kP528MinM = 1.5, kP528MaxM = 20000.0;
constexpr double kItmMinM = 0.5, kItmMaxM = 3000.0;

/// Unit east, north, up at a geodetic point.
void enu(double lat, double lon, double e[3], double n[3], double u[3]) noexcept {
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    e[0] = -so, e[1] = co, e[2] = 0.0;
    n[0] = -sl * co, n[1] = -sl * so, n[2] = cl;
    u[0] = cl * co, u[1] = cl * so, u[2] = sl;
}

double dot(const double a[3], const double b[3]) noexcept { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

void lookAngles(const Terminal& from, const double d[3], double& elevation, double& azimuth) noexcept {
    double e[3], n[3], u[3];
    enu(from.latitudeRad, from.longitudeRad, e, n, u);
    const double de = dot(d, e), dn = dot(d, n), du = dot(d, u);
    elevation = std::atan2(du, std::hypot(de, dn));
    azimuth = geo::wrapTwoPi(std::atan2(de, dn));
}

} // namespace

void geodeticToEcef(double lat, double lon, double h, double out[3]) noexcept {
    const double s = std::sin(lat), c = std::cos(lat);
    const double nr = kA / std::sqrt(1.0 - kE2 * s * s);
    out[0] = (nr + h) * c * std::cos(lon);
    out[1] = (nr + h) * c * std::sin(lon);
    out[2] = (nr * (1.0 - kE2) + h) * s;
}

double knifeEdgeLossDb(double v) noexcept {
    if (v <= -0.78) return 0.0;
    const double w = v - 0.1;
    return 6.9 + 20.0 * std::log10(std::sqrt(w * w + 1.0) + w); // P.526-15 (31)
}

double bullingtonDb(const double* h, int n, double spacing, double hts, double hrs, double lambda, double re) noexcept {
    if (n < 3) return 0.0;
    const double d = spacing * (n - 1);
    const double ce = 1.0 / re;
    // (the effective earth's bulge added to the terrain: c_e d_i (d - d_i) / 2)
    auto bulged = [&](int i) { const double di = spacing * i; return h[i] + ce * di * (d - di) / 2.0; };
    double stim = -std::numeric_limits<double>::infinity();
    for (int i = 1; i < n - 1; ++i) stim = std::max(stim, (bulged(i) - hts) / (spacing * i)); // (22)
    const double str = (hrs - hts) / d;                                                        // (23)
    double srim = -std::numeric_limits<double>::infinity();
    for (int i = 1; i < n - 1; ++i) srim = std::max(srim, (bulged(i) - hrs) / (d - spacing * i));
    double luc;
    if (stim < str || stim + srim <= 0.0) { // line of sight (or grazing): the edge of the largest v (24), (25)
        double vmax = -std::numeric_limits<double>::infinity();
        for (int i = 1; i < n - 1; ++i) {
            const double di = spacing * i;
            const double above = bulged(i) - (hts * (d - di) + hrs * di) / d;
            vmax = std::max(vmax, above * std::sqrt(2.0 * d / (lambda * di * (d - di))));
        }
        luc = knifeEdgeLossDb(vmax);
    } else { // beyond: the Bullington point where the two horizon rays cross (26)-(29)
        const double dbp = std::clamp((hrs - hts + srim * d) / (stim + srim), 1e-3, d - 1e-3);
        const double vb = (hts + stim * dbp - (hts * (d - dbp) + hrs * dbp) / d) * std::sqrt(2.0 * d / (lambda * dbp * (d - dbp)));
        luc = knifeEdgeLossDb(vb);
    }
    return luc + (1.0 - std::exp(-luc / 6.0)) * (10.0 + 0.02 * d / 1000.0); // (30)
}

namespace {

/// Bullington's ending once the profile's statistics are known: the two horizon slopes, the line of sight's largest
/// height above it (`amax`, its knife edge's), and how the diffraction parameter is found where that is needed.
template <class VMax>
double bullingtonEnd(double stim, double srim, double amax, double d, double hts, double hrs, double lambda, VMax&& vmaxOf) noexcept {
    const double str = (hrs - hts) / d; // (23)
    double luc;
    if (stim < str || stim + srim <= 0.0) { // line of sight (or grazing): the edge of the largest v (24), (25)
        // v_i = above_i sqrt(2 d / (lambda d_i (d - d_i))): its weight is least at the middle, sqrt(8 / (lambda d)); with
        // every point below the line (amax < 0) v <= amax x that least weight - below -0.78, no loss, whatever the rest
        const double least = std::sqrt(8.0 / (lambda * d));
        luc = amax < 0.0 && amax * least <= -0.78 ? 0.0 : knifeEdgeLossDb(vmaxOf());
    } else { // beyond: the Bullington point where the two horizon rays cross (26)-(29)
        const double dbp = std::clamp((hrs - hts + srim * d) / (stim + srim), 1e-3, d - 1e-3);
        const double vb = (hts + stim * dbp - (hts * (d - dbp) + hrs * dbp) / d) * std::sqrt(2.0 * d / (lambda * dbp * (d - dbp)));
        luc = knifeEdgeLossDb(vb);
    }
    return luc + (1.0 - std::exp(-luc / 6.0)) * (10.0 + 0.02 * d / 1000.0); // (30)
}

} // namespace

double bullingtonFastDb(const double* h, int n, double spacing, double hts, double hrs, double lambda, double re) noexcept {
    if (n < 3) return 0.0;
    const double d = spacing * (n - 1);
    const double ce = 1.0 / re;
    // one pass: each slope's largest ratio kept as numerator / denominator (both denominators positive: compared by
    // cross-multiplying, no division a point), and the largest height above the line of sight
    double sNum = -std::numeric_limits<double>::infinity(), sDen = 1.0, rNum = sNum, rDen = 1.0, amax = sNum;
    const double invD = 1.0 / d;
    for (int i = 1; i < n - 1; ++i) {
        const double di = spacing * i, rest = d - di;
        const double b = h[i] + ce * di * rest / 2.0;
        const double sn = b - hts, rn = b - hrs;
        if (sn * sDen > sNum * di) sNum = sn, sDen = di;
        if (rn * rDen > rNum * rest) rNum = rn, rDen = rest;
        amax = std::max(amax, b - (hts * rest + hrs * di) * invD);
    }
    return bullingtonEnd(sNum / sDen, rNum / rDen, amax, d, hts, hrs, lambda, [&] {
        double vmax = -std::numeric_limits<double>::infinity();
        for (int i = 1; i < n - 1; ++i) {
            const double di = spacing * i;
            const double above = h[i] + ce * di * (d - di) / 2.0 - (hts * (d - di) + hrs * di) / d;
            vmax = std::max(vmax, above * std::sqrt(2.0 * d / (lambda * di * (d - di))));
        }
        return vmax;
    });
}

double bullingtonFlatDb(int n, double spacing, double hts, double hrs, double lambda, double re) noexcept {
    if (n < 3) return 0.0;
    const double d = spacing * (n - 1);
    const double ce = 1.0 / re;
    auto bulge = [&](int i) { const double di = spacing * i; return ce * di * (d - di) / 2.0; };
    // the grid maximum of a function concave in d_i: at one of the samples about its continuous optimum x (or an end)
    auto best = [&](double x, auto&& f) {
        const int lo = std::clamp(static_cast<int>(std::floor(x / spacing)), 1, n - 2);
        double m = std::max(f(1), f(n - 2));
        for (int i = std::max(1, lo - 1); i <= std::min(n - 2, lo + 2); ++i) m = std::max(m, f(i));
        return m;
    };
    // (b_i - h) / d_i = ce (d - d_i) / 2 - h / d_i: concave for h > 0, its optimum at sqrt(2 h / ce); the receiver's mirrored
    auto slope = [&](int i) { return (bulge(i) - hts) / (spacing * i); };
    auto slopeR = [&](int i) { return (bulge(i) - hrs) / (d - spacing * i); };
    auto above = [&](int i) { const double di = spacing * i; return bulge(i) - (hts * (d - di) + hrs * di) / d; };
    const double stim = hts > 0.0 ? best(std::sqrt(2.0 * hts / ce), slope) : slope(1);
    const double srim = hrs > 0.0 ? best(d - std::sqrt(2.0 * hrs / ce), slopeR) : slopeR(n - 2);
    const double amax = best(d / 2.0 + (hts - hrs) / (ce * d), above); // (a concave quadratic in d_i)
    return bullingtonEnd(stim, srim, amax, d, hts, hrs, lambda, [&] {
        double vmax = -std::numeric_limits<double>::infinity();
        for (int i = 1; i < n - 1; ++i) {
            const double di = spacing * i;
            vmax = std::max(vmax, above(i) * std::sqrt(2.0 * d / (lambda * di * (d - di))));
        }
        return vmax;
    });
}

void preparePath(double frequencyHz) {
    if (frequencyHz > 0.0) ntia::prepare(std::clamp(frequencyHz / 1e6, 100.0, 30000.0)); // (as computePath hands P.528 its frequency)
}

PathResult computePath(const Terminal& a, const Terminal& b, const PathOptions& o) {
    PathResult r;
    const double lambda = kLight / o.frequencyHz;
    double pa[3], pb[3];
    geodeticToEcef(a.latitudeRad, a.longitudeRad, a.heightM, pa);
    geodeticToEcef(b.latitudeRad, b.longitudeRad, b.heightM, pb);
    const double dv[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]};
    r.slantRangeM = std::sqrt(dot(dv, dv));
    const double slant = std::max(r.slantRangeM, 1e-3);
    for (int i = 0; i < 3; ++i) r.directionEcef[i] = dv[i] / slant;
    const double back[3] = {-r.directionEcef[0], -r.directionEcef[1], -r.directionEcef[2]};
    lookAngles(a, r.directionEcef, r.elevationRad[0], r.azimuthRad[0]);
    lookAngles(b, back, r.elevationRad[1], r.azimuthRad[1]);
    r.groundRangeM = geo::distanceM(a.latitudeRad, a.longitudeRad, b.latitudeRad, b.longitudeRad);
    r.freeSpaceDb = 20.0 * std::log10(4.0 * kPi * slant / lambda);

    // The terrain profile along the great circle (sea level without a ground).
    const double d = r.groundRangeM;
    double spacing = o.profileSpacingM > 0.0 ? o.profileSpacingM : 30.0;
    if (o.profileSpacingM <= 0.0 && o.ground) spacing = std::max(30.0, std::min(o.ground->resolutionM(), 1000.0));
    const int n = std::clamp(static_cast<int>(std::ceil(d / spacing)) + 1, 3, 2001);
    spacing = d / (n - 1);
    std::vector<double> profile(static_cast<std::size_t>(n), 0.0);
    if (o.ground && d > 0.0) o.ground->profile(a.latitudeRad, a.longitudeRad, b.latitudeRad, b.longitudeRad, n, spacing, profile.data());
    // One pass: the sea's surface under the ground, the straight ray's clearance of the terrain in the 4/3 earth, and the
    // least-squares line's sums (smoothSurface's, in its order)
    double sy = 0.0, sxy = 0.0;
    const double rise = (b.heightM - a.heightM) / std::max(d, 1e-9), bulge = 1.0 / (2.0 * kEffectiveRadiusM);
    for (int i = 0; i < n; ++i) {
        double& h = profile[static_cast<std::size_t>(i)];
        h = std::max(0.0, h);
        if (i > 0 && i < n - 1) {
            const double di = spacing * i;
            const double ray = a.heightM + rise * di - di * (d - di) * bulge;
            r.clearanceM = std::min(r.clearanceM, ray - h);
        }
        sy += h, sxy += static_cast<double>(i) * h;
    }
    r.terrainMasked = r.clearanceM < 0.0;

    // The terminals over the profile's smooth surface (P.526-15 4.5.4) - P.528's heights.
    double s0, s1;
    {
        const double nn = static_cast<double>(n);
        const double sx = nn * (nn - 1.0) / 2.0, sxx = (nn - 1.0) * nn * (2.0 * nn - 1.0) / 6.0; // (sums of i and i^2: exact)
        const double den = nn * sxx - sx * sx;
        const double slope = den > 0.0 ? (nn * sxy - sx * sy) / den : 0.0;
        const double intercept = (sy - slope * sx) / nn;
        s0 = intercept, s1 = intercept + slope * (nn - 1.0);
    }
    s0 = std::min(s0, profile.front()), s1 = std::min(s1, profile.back()); // (no terminal below the surface's ground)
    const double hte = std::max(a.heightM - s0, kP528MinM), hre = std::max(b.heightM - s1, kP528MinM);
    const double groundA = a.heightM - profile.front(), groundB = b.heightM - profile.back();

    const double fMhz = o.frequencyHz / 1e6;
    const bool itmFits = o.model == PathModel::Itm && fMhz >= 20.0 && fMhz <= 20000.0 && d >= 1000.0 && d <= 2000e3 &&
                         groundA >= kItmMinM && groundA <= kItmMaxM && groundB >= kItmMinM && groundB <= kItmMaxM;
    if (itmFits) {
        std::vector<double> pfl(static_cast<std::size_t>(n) + 2);
        pfl[0] = n - 1, pfl[1] = spacing;
        std::copy(profile.begin(), profile.end(), pfl.begin() + 2);
        ntia::ItmResult itm;
        r.code = ntia::itmP2p(groundA, groundB, pfl.data(), o.climate, o.surfaceRefractivity, fMhz, o.polarization, o.permittivity,
                              o.conductivity, 0x0 /* single message */, o.timePercent, 50.0, 50.0, itm);
        r.viaItm = true;
        r.mode = static_cast<PathMode>(std::clamp(itm.mode, 1, 3));
        r.gasDb = ntia::p676Specific(o.frequencyHz / 1e9, (a.heightM + b.heightM) / 2000.0) * slant / 1000.0;
        r.basicDb = itm.lossDb + r.gasDb;
    } else {
        // P.528: the lower terminal first
        const double lo = std::min(hte, hre), hi = std::max(hte, hre);
        ntia::P528Result p;
        r.code = ntia::p528(d / 1000.0, std::min(lo, kP528MaxM), std::min(hi, kP528MaxM), std::clamp(fMhz, 100.0, 30000.0),
                            o.polarization, std::clamp(o.timePercent, 1.0, 99.0), p);
        if (r.code == 0 || r.code == 11) {
            r.mode = static_cast<PathMode>(std::clamp(p.mode, 1, 3));
            r.basicDb = p.lossDb;
            r.gasDb = p.gasDb;
        } else { // (outside P.528's inputs: free space and the gases)
            r.gasDb = ntia::p676Specific(o.frequencyHz / 1e9, (a.heightM + b.heightM) / 2000.0) * slant / 1000.0;
            r.basicDb = r.freeSpaceDb + r.gasDb;
        }
        // The terrain's excess over the smooth earth: delta-Bullington (P.526-15 4.5.4)
        const double actual = bullingtonFastDb(profile.data(), n, spacing, a.heightM, b.heightM, lambda, kEffectiveRadiusM);
        const double smooth = bullingtonFlatDb(n, spacing, hte, hre, lambda, kEffectiveRadiusM);
        r.terrainDb = std::max(0.0, actual - smooth);
    }

    // Rain below the rain height, along the straight path.
    if (o.rainRateMmH > 0.0) {
        const double lowH = std::min(a.heightM, b.heightM), highH = std::max(a.heightM, b.heightM);
        double fraction = 0.0;
        if (lowH < o.rainHeightM) fraction = highH <= o.rainHeightM ? 1.0 : (o.rainHeightM - lowH) / (highH - lowH);
        const double elevation = std::fabs(a.heightM <= b.heightM ? r.elevationRad[0] : r.elevationRad[1]); // (at the lower end)
        const double tilt = o.polarization == 1 ? kPi / 2.0 : 0.0;
        r.rainDb = rainSpecificDbPerKm(o.frequencyHz / 1e9, o.rainRateMmH, elevation, tilt) * slant * fraction / 1000.0;
    }
    r.totalDb = r.basicDb + r.terrainDb + r.rainDb;
    return r;
}

} // namespace fsim::ms::rf
