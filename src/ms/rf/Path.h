#pragma once
// The radio path between two points in the air (ADR-30, sections 4 and 5;
// MS-2): its geometry, the terrain under it and whether that masks it, and
// its loss -
//
//  - ITU-R P.528-5 (NTIA's reference implementation): the basic transmission
//    loss over a smooth earth - free space, the ground reflection's
//    multipath in line of sight, smooth-earth diffraction past the horizon,
//    troposcatter, P.676's gaseous absorption along the refracted ray;
//  - the terrain's excess over that smooth earth: ITU-R P.526-15's
//    delta-Bullington (section 4.5), the Bullington loss of the real profile
//    less that of its least-squares smooth surface;
//  - rain: P.838-3's specific attenuation along the part of the path below
//    the rain height;
//  - or, asked for, the ITM (Longley-Rice) over the profile, within its
//    limits (both terminals 0.5 .. 3000 m above the ground, 20 MHz .. 20 GHz),
//    with P.676's gases added.
//
// Every model is deterministic: a median (or the time percentage asked for).

#include <cstdint>
#include <limits>

namespace fsim::sim {
class GroundProvider;
}

namespace fsim::ms::rf {

/// A point in the air: geodetic, its height above the WGS-84 ellipsoid.
struct Terminal {
    double latitudeRad = 0.0;
    double longitudeRad = 0.0;
    double heightM = 0.0;
};

enum class PathModel : std::uint8_t {
    Auto, ///< P.528 with the terrain's delta-Bullington excess
    Itm,  ///< the ITM over the profile where it applies; else as Auto
};

enum class PathMode : std::uint8_t { LineOfSight = 1, Diffraction = 2, Troposcatter = 3 };

struct PathOptions {
    double frequencyHz = 10e9;
    int polarization = 0;                ///< 0 horizontal, 1 vertical (P.528, the ITM, P.838's tilt)
    double timePercent = 50.0;           ///< the loss not exceeded for this percentage of time (P.528, the ITM)
    double rainRateMmH = 0.0;            ///< rain along the path below `rainHeightM`
    double rainHeightM = 3360.0;         ///< P.839's rain height: the 0 deg C isotherm plus 360 m
    const sim::GroundProvider* ground = nullptr; ///< null: sea level everywhere
    double profileSpacingM = 0.0;        ///< 0: the ground's resolution, at least 30 m, at most 2000 samples a path
    PathModel model = PathModel::Auto;
    // The ITM's ground and climate (its defaults: continental temperate, average ground)
    int climate = 5;
    double surfaceRefractivity = 301.0;
    double permittivity = 15.0;
    double conductivity = 0.005;
};

/// Build what a path at this frequency needs before it is first asked (P.676's layer tables: tens of ms): a payload's
/// creation, not the step its first look lands in.
void preparePath(double frequencyHz);

struct PathResult {
    int code = 0;                         ///< the model's: 0 success; P.528 11 or the ITM 1, with warnings; else invalid
    bool viaItm = false;                  ///< the ITM gave the basic loss
    PathMode mode = PathMode::LineOfSight;
    double groundRangeM = 0.0;            ///< great circle
    double slantRangeM = 0.0;             ///< straight line
    double elevationRad[2] = {0, 0};      ///< at each end, towards the other (geodetic local level)
    double azimuthRad[2] = {0, 0};        ///< likewise, clockwise from north
    double directionEcef[3] = {0, 0, 0};  ///< unit, from the first to the second
    double clearanceM = std::numeric_limits<double>::infinity(); ///< the ray's least height above the terrain (4/3 earth); < 0 masked
    bool terrainMasked = false;           ///< the terrain cuts the straight ray
    double freeSpaceDb = 0.0;             ///< 20 log(4 pi R / lambda)
    double basicDb = 0.0;                 ///< P.528's (or the ITM's) basic transmission loss, gases included
    double gasDb = 0.0;                   ///< of which atmospheric absorption
    double terrainDb = 0.0;               ///< the terrain's excess over the smooth earth (delta-Bullington)
    double rainDb = 0.0;
    double totalDb = 0.0;                 ///< basic + terrain + rain
};

/// The path from `a` to `b` (reciprocal: the same loss either way).
PathResult computePath(const Terminal& a, const Terminal& b, const PathOptions& options);

// --- Its parts, for the tests and the payloads ---------------------------------

/// WGS-84 geodetic to ECEF (m).
void geodeticToEcef(double latitudeRad, double longitudeRad, double heightM, double out[3]) noexcept;
/// ITU-R P.526-15 equation (31): the knife-edge diffraction loss J(v) (dB), 0 below v = -0.78.
double knifeEdgeLossDb(double v) noexcept;
/// ITU-R P.526-15 section 4.5.1, the Bullington loss (dB) of a profile: `heightsM[0..n-1]` the terrain at equal
/// spacing from the transmitter (the ends included), `txM` and `rxM` the terminals' heights in the same datum,
/// `effectiveRadiusM` the effective earth's radius.
/// The same, in one pass over the profile (computePath's): the two horizon slopes by cross-multiplication, and the
/// line-of-sight case's diffraction parameter only where a bound does not already rule the loss out (v <= -0.78 at every
/// point: no loss). Equal to bullingtonDb (test_rf_path.cpp).
double bullingtonFastDb(const double* heightsM, int n, double spacingM, double txM, double rxM, double wavelengthM,
                        double effectiveRadiusM) noexcept;
/// bullingtonDb over a flat profile (P.526's smooth-earth term): the slopes and the line of sight's height are
/// concave in the distance, so each grid maximum is at one of the two samples about its continuous optimum - a few
/// points, not a pass.
double bullingtonFlatDb(int n, double spacingM, double txM, double rxM, double wavelengthM, double effectiveRadiusM) noexcept;
double bullingtonDb(const double* heightsM, int n, double spacingM, double txM, double rxM, double wavelengthM,
                    double effectiveRadiusM) noexcept;

} // namespace fsim::ms::rf
