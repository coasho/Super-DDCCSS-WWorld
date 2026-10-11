#include "world/WingVapour.h"

#include "world/Exhaust.h"

#include "core/Log.h"
#include "platform/Threads.h"
#include "world/Frames.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

// The vapour round an aircraft, ported from aeronautic's @aeronautic/wing-vapor (MIT, (c) 2026 Romain Le Gall,
// https://github.com/Romainlg29/aeronautic - see THIRD_PARTY_NOTICES.md): the lift and its vortices
// (src/aerodynamics.ts), the moist air's condensation (src/condensation.ts, @aeronautic/core's atmosphere), the wing
// as a table and its loading (src/wing-shape.ts), the trails laid in the air flown through (src/trails.ts), the
// pressure field (src/vapor-field.ts, src/tsl/field.ts) and its march (src/vapor-material.ts), translated from
// TypeScript and TSL to C++ and GLSL. What it does is theirs; what is changed is said where it is.
//
// Everything is in the airframe's canonical frame, as theirs: x aft, y up, z out along the right wing, metres from
// the model's origin; the field is symmetric left to right, so z is folded and one evaluation draws both wings.

namespace fsim::world {

namespace {

constexpr double kPi = 3.14159265358979323846, kDegree = kPi / 180.0;
constexpr double kGamma = 1.4, kGasConstant = 287.05, kCp = 1004.6, kVapourRatio = 0.622, kLatentHeat = 2.501e6;

double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double smoothstep(double e0, double e1, double x) {
    const double t = clampd((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
double finite(double v, double fallback = 0.0) { return std::isfinite(v) ? v : fallback; }

// --- the air (@aeronautic/core atmosphere.ts, condensation.ts) -------------------------------------------------------
double saturationPressure(double temperatureK) {
    const double c = temperatureK - 273.15;
    return 610.94 * std::exp(17.625 * c / (c + 243.04));
}
double mixingRatio(double vapourPa, double pressurePa) { return kVapourRatio * vapourPa / std::max(pressurePa - vapourPa, 1.0); }

void standardAtmosphere(double altitudeM, double& temperatureK, double& pressurePa) {
    // each layer's base and lapse rate (K/m); above the last, isothermal enough
    const double layers[4][2] = {{0.0, -0.0065}, {11000.0, 0.0}, {20000.0, 0.001}, {32000.0, 0.0028}};
    const double g = 9.80665, h = clampd(altitudeM, -500.0, 47000.0);
    double T = 288.15, p = 101325.0;
    for (int i = 0; i < 4; ++i) {
        const double base = i == 0 ? 0.0 : layers[i][0], lapse = layers[i][1], top = i < 3 ? layers[i + 1][0] : 1e9;
        if (i > 0 && h <= base) break;
        const double rise = std::min(h, top) - base; // (below sea level, the first layer's continued down)
        if (rise == 0.0) continue;
        const double next = T + lapse * rise;
        p *= lapse == 0.0 ? std::exp(-g * rise / (kGasConstant * T)) : std::pow(next / T, -g / (kGasConstant * lapse));
        T = next;
    }
    temperatureK = T;
    pressurePa = p;
}

// The lowest pressure ratio the condensation table covers, its entries, and how much drier and moister the day's
// patches are than its average
constexpr double kMinRatio = 0.3, kHumiditySpread = 0.06, kMoistest = 0.995;
constexpr int kTexels = 128;

double moistest(const WingVapour::Air& air, double spread) {
    return std::max(std::min(1.0 + std::max(spread, 0.0), kMoistest / std::max(air.relativeHumidity, 1e-3)), 1.0);
}

// A parcel expanded to a share of the pressure: the temperature it settles at (the latent heat given back
// included) and the liquid water per cubic metre
void condensate(double ratio, const WingVapour::Air& air, double humidityScale, double& temperatureK, double& waterKgM3) {
    ratio = clampd(ratio, 1e-3, 1.0);
    const double pressure = air.pressurePa * ratio;
    const double dry = air.temperatureK * std::pow(ratio, kGasConstant / kCp);
    const double water = air.mixingRatio * std::max(humidityScale, 0.0);
    const auto saturated = [&](double T) { return mixingRatio(saturationPressure(T), pressure); };
    if (water <= saturated(dry)) {
        temperatureK = dry;
        waterKgM3 = 0.0;
        return;
    }
    double lo = dry, hi = std::max(air.temperatureK, dry + 1e-3);
    for (int i = 0; i < 40; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (kCp * (mid - dry) - kLatentHeat * (water - saturated(mid)) > 0.0) hi = mid;
        else lo = mid;
    }
    temperatureK = 0.5 * (lo + hi);
    const double liquid = std::max(water - saturated(temperatureK), 0.0);
    waterKgM3 = liquid * pressure / (kGasConstant * temperatureK);
}

double solveCondensingDeficit(const WingVapour::Air& air, double scale) {
    double T, w;
    double lo = kMinRatio, hi = 1.0;
    condensate(lo, air, scale, T, w);
    if (w <= 0.0) return 1.0;
    for (int i = 0; i < 32; ++i) {
        const double mid = 0.5 * (lo + hi);
        condensate(mid, air, scale, T, w);
        if (w > 0.0) lo = mid;
        else hi = mid;
    }
    return 1.0 - lo;
}
// The same day asks the same question every frame (vapor-field.ts condensing_memo)
double condensingDeficit(const WingVapour::Air& air, double scale) {
    thread_local double key[4] = {-1.0, -1.0, -1.0, -1.0}, deficit = 1.0;
    if (key[0] != air.temperatureK || key[1] != air.pressurePa || key[2] != air.mixingRatio || key[3] != scale) {
        deficit = solveCondensingDeficit(air, scale);
        key[0] = air.temperatureK;
        key[1] = air.pressurePa;
        key[2] = air.mixingRatio;
        key[3] = scale;
    }
    return deficit;
}

// --- compressible flow (aerodynamics.ts) -----------------------------------------------------------------------------
constexpr double kMinBeta = 0.15;
double beta(double mach) { return std::max(std::sqrt(std::abs(1.0 - mach * mach)), kMinBeta); }
double karmanTsien(double incompressible, double mach) {
    const double b = beta(mach);
    return incompressible / std::max(b + mach * mach / (1.0 + b) * (incompressible / 2.0), b * 0.5);
}
double criticalPressure(double mach) {
    const double m2 = std::max(mach * mach, 1e-6), g = kGamma;
    return 2.0 / (g * m2) * (std::pow((2.0 + (g - 1.0) * m2) / (g + 1.0), g / (g - 1.0)) - 1.0);
}
constexpr double kMaxLocalMach = 1.4;
double mostDeficit(double mach) {
    const double h = 0.5 * (kGamma - 1.0);
    return 1.0 - std::pow((1.0 + h * mach * mach) / (1.0 + h * kMaxLocalMach * kMaxLocalMach), kGamma / (kGamma - 1.0));
}
double cappedDeficit(double deficit, double mach) {
    const double cap = mostDeficit(mach);
    return deficit > 0.0 ? cap * (1.0 - std::exp(-deficit / cap)) : deficit;
}

// --- the field's constants (vapor-field.ts) --------------------------------------------------------------------------
constexpr double kCanardLoading = 1.2, kMaxBias = 0.7, kSideLo = 0.2, kSideHi = 1.8, kBoundFloor = 0.003, kMaxSecondShare = 0.4;
constexpr double kEddyViscosity = 2e-4, kRollupCoefficient = 0.28, kRollupLo = 0.5, kRollupHi = 4.0;
constexpr double kTipStartCore = 0.05, kTipStartShare = 0.4, kMaxTrailM = 400.0, kEdgeVortexHeight = 1.0, kEdgeSurface = 0.375;
constexpr double kBurstLength = 0.08, kBurstSwell = 2.0, kEvaporationS = 0.003, kRootFade = 0.12, kRootBlur = 1.0;
// fsim: where along the span the tip's relief starts to take the wing's suction (the flow round the tip makes the
// outer span's pocket weaker - a swept wing's supercritical flow ends short of its tip): ended over the last 10 %, the
// vapour over the wing stood up in walls at the tips
constexpr double kTipRelief = 0.7;
constexpr double kNoseRadius = 0.03, kNoseReach = 0.3, kWingFieldHeight = 0.1, kWingHeightMinBeta = 0.4, kShockTravel = 0.85;
constexpr double kShockWidth = 0.015, kConeStrength = 4.0, kConeReach = 0.25, kConeShockSubsonic = 0.55, kConeShockSonic = 0.92;
constexpr double kConeShockSupersonic = 0.98, kSupersonicFadeLo = 1.03, kSupersonicFadeHi = 1.15, kConeShockLean = 0.08;
constexpr double kPocketGrowth = 0.03, kConeRecompression = 0.3, kConeShockThickness = 0.01, kConePocketLength = 0.45;
constexpr double kVortexSpacing = kPi / 4.0, kEdgeVortexSpan = 0.7, kSectionEfficiency = 0.95, kLiftSlopeMaxMach = 0.9;
// the airframe's vortex cores (types.ts default_vapor_airframe)
constexpr double kTipCoreRadius = 0.02, kTipCoreShare = 0.35, kEdgeCoreRadius = 0.02;
constexpr int kStations = 64, kTrailPoints = 64;

/// A fixed table read by station or point: indexed by int, as the shader's are
template <class T, int N> struct Table {
    std::array<T, static_cast<std::size_t>(N)> v{};
    T& operator[](int i) { return v[static_cast<std::size_t>(i)]; }
    const T& operator[](int i) const { return v[static_cast<std::size_t>(i)]; }
    void fill(const T& x) { v.fill(x); }
    auto begin() const { return v.begin(); }
    auto end() const { return v.end(); }
};
using Stations = Table<double, kStations>;
using Points = Table<vsg::dvec3, kTrailPoints>;

// --- the march (vapor-material.ts) and the look (types.ts) -----------------------------------------------------------
constexpr double kDropletRadiusM = 1e-6, kAnisotropy = 0.75, kEddyM = 0.6, kShutterS = 1.0 / 60.0;
constexpr int kMaxSteps = 160;
// the budget (wing-vapor-core.ts): under DETAIL_HEIGHT of the screen's height the moisture's patches are dropped.
// (fsim: theirs also coarsened the steps past a share of the screen, up to three times; the pass's budget of rays
// holds the cost here, and coarse steps across the thin layer over a wing left it speckled)
constexpr double kDetailHeight = 0.06, kRollResponseS = 0.15;
constexpr std::size_t kInstances = 8;      ///< vapours drawn at once: the nearest
// The rays the vapour is marched with at most (the window's pixels its boxes cover, all together): past them it is
// marched at a texel for so many pixels each way (any share, up to kMaxScale) and laid on the window from the texels
// round each pixel - the same rays, so the same cost and the same detail for the same view, whatever the window's
// size. (Where it costs, it fills the screen and its features span many pixels; an airframe's edge is marched pixel
// by pixel.)
constexpr double kRayBudget = 150000.0;
constexpr double kMaxScale = 4.0;
constexpr double kRange = 4000.0;          ///< m from the eye
// An airframe's edge pixels, marched one by one: so many listed at most (past them, a texel's are marched by itself)
constexpr std::uint32_t kMaxEdges = 1u << 19;
// The engines' exhausts marched in the same pass (Exhaust::marchSource): so many at once, the nearest; per flame its
// view -> own frame (four rows), then its row in the exhaust's numbers, its detail and its share of the near steps
constexpr std::uint32_t kMaxFlames = 32, kFlameRows = 5;
// (as the exhaust's own box had them, afterburner-batch.ts: the detail by distance, the near steps shared out past
// a share of the screen, too small to read below a share of its height)
constexpr double kDetailDistance = 900.0, kCheapDistance = 3000.0, kPlumeStepArea = 0.3, kMinScreenFraction = 0.001;
// The Screen block's rows before the boxes' outlines (screen, info, the inverse projection's four, depth terms, the
// march's region, the composite's), and what the outlines and the regions are widened by: a pixel round an outline
// (its corners' rounding), kRegionTexels texels round the march's region for the composite's filter (a pixel's four by
// four texels, and an edge pixel's neighbours), kLaidTexels round where the composite lays it on.
constexpr std::uint32_t kScreenRects = 9;
constexpr double kOutlinePx = 1.0;
constexpr int kLaidTexels = 3, kRegionTexels = 6;

} // namespace

// --- the wing as a table (wing-shape.ts), from the model's manifest --------------------------------------------------
struct WingVapour::Model {
    double semispan = 1.0, rootSpan = 0.0;
    Stations leading{}, chord{}, mid{}, thickness{}, loading{};
    bool secondary = false; ///< a canard
    double secondSemispan = 1.0, secondTipLeading = 0.0, secondTipChord = 1.0, secondTipHeight = 0.0, secondArea = 0.0;
    // the planform the lift reads (shape_planform)
    double area = 1.0, span = 2.0, sweep = 0.0, rootChord = 1.0, tipChord = 1.0, sharpness = 1.0;
    // the body, for the cone
    double nose = 1.0, bodyLength = 2.0, bodyRadius = 0.2, bodyHeight = 0.0;
    // the leading-edge vortex's path (edge_path): its span, the surface under it, the local semispan
    double edgeApex = 0.0, edgeLength = 0.0;
    Stations pathSpan{}, pathSurface{}, pathReach{};

    struct Station {
        double leading, chord, mid, thickness, loading;
    };
    Station at(double out) const {
        const double u = std::min(std::abs(out) / semispan, 1.0) * (kStations - 1);
        const int i = std::min(static_cast<int>(u), kStations - 1), j = std::min(i + 1, kStations - 1);
        const double t = clampd(u - i, 0.0, 1.0);
        const auto l = [&](const Stations& v) { return v[i] + (v[j] - v[i]) * t; };
        return {l(leading), l(chord), l(mid), l(thickness), l(loading)};
    }
    double stationSpan(int k) const { return semispan * k / (kStations - 1); }
};

namespace {

// A manifest's surface at a span station: its leading edge (aft of the origin), its chord and its height (up),
// false outside it
bool surfaceAt(const VehicleVisuals::Surface& s, double y, double& leading, double& chord, double& height) {
    const auto& v = s.sections;
    if (v.empty() || y > v.back().y + 1e-6) return false;
    std::size_t k = 0;
    while (k + 1 < v.size() && v[k + 1].y < y) ++k;
    if (k + 1 >= v.size() || y <= v[k].y) {
        const auto& e = y <= v.front().y ? v.front() : v.back();
        leading = -e.x;
        chord = e.w;
        height = -e.z;
        return true;
    }
    const double t = (y - v[k].y) / std::max(v[k + 1].y - v[k].y, 1e-9);
    leading = -(v[k].x + (v[k + 1].x - v[k].x) * t);
    chord = v[k].w + (v[k + 1].w - v[k].w) * t;
    height = -(v[k].z + (v[k + 1].z - v[k].z) * t);
    return true;
}

// Weissinger's lifting line over the table (lattice_loading): its spanwise loading over its mean
void latticeLoading(WingVapour::Model& m) {
    const std::size_t panels = 48;
    const double semispan = m.semispan;
    const auto edge = [&](std::size_t i) { return -semispan * std::cos(kPi * static_cast<double>(i) / static_cast<double>(panels)); };
    const auto quarter = [&](double span) {
        const auto s = m.at(span);
        return s.leading + 0.25 * s.chord;
    };
    std::vector<std::array<double, 2>> control(panels);
    for (std::size_t i = 0; i < panels; ++i) {
        const double span = 0.5 * (edge(i) + edge(i + 1));
        const auto s = m.at(span);
        control[i] = {s.leading + 0.75 * s.chord, span};
    }
    const auto segment = [](double px, double pz, double ax, double az, double bx, double bz) {
        const double r1x = px - ax, r1z = pz - az, r2x = px - bx, r2z = pz - bz;
        const double cross = r1x * r2z - r1z * r2x;
        if (std::abs(cross) < 1e-10) return 0.0;
        const double r1 = std::hypot(r1x, r1z), r2 = std::hypot(r2x, r2z);
        const double along = (bx - ax) * (r1x / r1 - r2x / r2) + (bz - az) * (r1z / r1 - r2z / r2);
        return along / cross / (4.0 * kPi);
    };
    const double far = semispan * 1e3;
    std::vector<std::vector<double>> a(panels, std::vector<double>(panels));
    std::vector<double> b(panels, 1.0);
    for (std::size_t i = 0; i < panels; ++i)
        for (std::size_t j = 0; j < panels; ++j) {
            const double za = edge(j), zb = edge(j + 1), xa = quarter(za), xb = quarter(zb);
            const double px = control[i][0], pz = control[i][1];
            a[i][j] = segment(px, pz, far, za, xa, za) + segment(px, pz, xa, za, xb, zb) + segment(px, pz, xb, zb, far, zb);
        }
    // Gaussian elimination with partial pivoting
    for (std::size_t c = 0; c < panels; ++c) {
        std::size_t p = c;
        for (std::size_t r = c + 1; r < panels; ++r)
            if (std::abs(a[r][c]) > std::abs(a[p][c])) p = r;
        std::swap(a[c], a[p]);
        std::swap(b[c], b[p]);
        const double lead = a[c][c] != 0.0 ? a[c][c] : 1e-12;
        for (std::size_t r = c + 1; r < panels; ++r) {
            const double f = a[r][c] / lead;
            if (f == 0.0) continue;
            for (std::size_t k = c; k < panels; ++k) a[r][k] -= f * a[c][k];
            b[r] -= f * b[c];
        }
    }
    std::vector<double> x(panels);
    for (std::size_t r = panels; r-- > 0;) {
        double sum = b[r];
        for (std::size_t k = r + 1; k < panels; ++k) sum -= a[r][k] * x[k];
        x[r] = sum / (a[r][r] != 0.0 ? a[r][r] : 1e-12);
    }
    const auto interpolate = [&](double at) {
        if (at <= control.front()[1]) return x.front();
        if (at >= control.back()[1]) return x.back();
        std::size_t k = 1;
        while (control[k][1] < at) ++k;
        const double t = (at - control[k - 1][1]) / (control[k][1] - control[k - 1][1]);
        return x[k - 1] + (x[k] - x[k - 1]) * t;
    };
    for (int k = 0; k < kStations; ++k) {
        const double span = m.stationSpan(k);
        m.loading[k] = 0.5 * (interpolate(span) + interpolate(-span));
    }
    m.loading[kStations - 1] = 0.0;
    double mean = 0.0;
    for (int k = 1; k < kStations; ++k) mean += 0.5 * (m.loading[k - 1] + m.loading[k]);
    mean /= kStations - 1;
    for (int k = 0; k < kStations; ++k) m.loading[k] = std::max(m.loading[k] / (mean != 0.0 ? mean : 1.0), 0.0);
}

// The leading-edge vortex's path along the table's leading edge, from where it leaves the body (edge_path)
void edgePath(WingVapour::Model& m) {
    m.edgeApex = m.at(m.rootSpan).leading;
    m.edgeLength = std::max(m.leading[kStations - 1] - m.edgeApex, 0.0);
    const double out = std::max(m.semispan - m.rootSpan, 1e-3);
    for (int index = 0; index < kStations; ++index) {
        const double share = static_cast<double>(index) / (kStations - 1);
        const double x = m.edgeApex + share * m.edgeLength;
        double reach = share * out;
        if (m.edgeLength > 1e-3) {
            reach = out;
            for (int station = 1; station < kStations; ++station) {
                const double inner = m.stationSpan(station - 1), outer = m.stationSpan(station);
                if (outer <= m.rootSpan) continue;
                const double from = std::max(inner, m.rootSpan);
                const double x0 = m.at(from).leading, x1 = m.leading[station];
                if (x <= x1) {
                    const double t = x1 > x0 ? clampd((x - x0) / (x1 - x0), 0.0, 1.0) : 0.0;
                    reach = from + (outer - from) * t - m.rootSpan;
                    break;
                }
            }
        }
        const double span = m.rootSpan + kEdgeVortexSpan * reach;
        const auto s = m.at(span);
        m.pathSpan[index] = span;
        m.pathSurface[index] = s.mid + kEdgeSurface * s.thickness * s.chord;
        m.pathReach[index] = std::max(reach, 0.0);
    }
}

// The model's wing from its manifest - the wing and a strake as one table (a strake is the wing's leading edge
// inboard, its vortex the leading-edge vortex), a canard as the second surface - and its body from its bounding box.
// (aeronautic measures a model from six depth views; this reads hangar's manifest, which says the same.) Lacking a
// wing: a trapezoid to its widest point.
std::unique_ptr<WingVapour::Model> makeModel(const VehicleVisuals::Shape& shape) {
    auto m = std::make_unique<WingVapour::Model>();
    const VehicleVisuals::Surface *wing = nullptr, *strake = nullptr, *canard = nullptr;
    for (const auto& s : shape.surfaces) {
        if (s.sections.size() < 2) continue;
        if (s.kind == "wing" && !wing) wing = &s;
        else if (s.kind == "strake" && !strake) strake = &s;
        else if (s.kind == "canard" && !canard) canard = &s;
    }
    const double length = std::max(shape.hi.x - shape.lo.x, 1.0);
    m->nose = shape.hi.x;
    m->bodyLength = length;
    m->bodyRadius = clampd(0.07 * length, 0.15, 3.0);
    m->bodyHeight = 0.0;
    VehicleVisuals::Surface trapezoid;
    if (!wing) {
        // from the widest point: a moderately swept wing a quarter of the length deep at its root
        const double semispan = std::max(shape.tip.y, 0.5), root = 0.25 * length, tip = 0.1 * length;
        trapezoid.kind = "wing";
        trapezoid.sections = {vsg::dvec4(shape.tip.x + tip + 0.35 * semispan, 0.0, shape.tip.z, root),
                              vsg::dvec4(shape.tip.x + tip, semispan, shape.tip.z, tip)};
        wing = &trapezoid;
    }
    m->semispan = std::max(wing->sections.back().y, 0.25);
    m->rootSpan = std::min(m->bodyRadius, 0.4 * m->semispan);
    for (int k = 0; k < kStations; ++k) {
        const double y = m->stationSpan(k);
        double le = 0.0, chord = 0.0, h = 0.0;
        surfaceAt(*wing, y, le, chord, h);
        double te = le + chord;
        double sle, schord, sh;
        if (strake && surfaceAt(*strake, y, sle, schord, sh) && schord > 0.0) {
            te = std::max(te, sle + schord);
            le = std::min(le, sle);
        }
        m->leading[k] = le;
        m->chord[k] = std::max(te - le, 0.05);
        m->mid[k] = h;
    }
    // planform
    double area = 0.0;
    for (int k = 1; k < kStations; ++k) area += 0.5 * (m->chord[k - 1] + m->chord[k]) * m->semispan / (kStations - 1);
    m->area = 2.0 * area;
    m->span = 2.0 * m->semispan;
    const double out = m->semispan - m->rootSpan;
    m->sweep = out > 1e-3 ? std::atan((m->leading[kStations - 1] - m->at(m->rootSpan).leading) / out) : 0.0;
    m->rootChord = m->chord[0];
    m->tipChord = m->chord[kStations - 1];
    // a fighter's thin wing, a transport's thicker; a strake's or a highly swept edge is sharp
    const double tc = m->sweep > 30.0 * kDegree ? 0.05 : 0.12;
    m->thickness.fill(tc);
    m->sharpness = strake ? 1.0 : m->sweep > 45.0 * kDegree ? 0.8 : 0.4;
    if (canard) {
        m->secondary = true;
        const auto& tip = canard->sections.back();
        m->secondSemispan = std::max(tip.y, 0.1);
        m->secondTipLeading = -tip.x;
        m->secondTipChord = std::max(tip.w, 0.05);
        m->secondTipHeight = -tip.z;
        double a = 0.0;
        for (std::size_t k = 1; k < canard->sections.size(); ++k)
            a += 0.5 * (canard->sections[k - 1].w + canard->sections[k].w) * (canard->sections[k].y - canard->sections[k - 1].y);
        m->secondArea = 2.0 * a;
    }
    latticeLoading(*m);
    edgePath(*m);
    return m;
}

std::vector<double> modelKey(const VehicleVisuals::Shape& shape) {
    std::vector<double> key = {shape.lo.x, shape.lo.y, shape.lo.z, shape.hi.x, shape.hi.y, shape.hi.z, shape.tip.x, shape.tip.y, shape.tip.z};
    for (const auto& s : shape.surfaces) {
        key.push_back(static_cast<double>(std::hash<std::string>{}(s.kind) % 100003u));
        for (const auto& v : s.sections) key.insert(key.end(), {v.x, v.y, v.z, v.w});
    }
    return key;
}

// --- the lift and its vortices (aerodynamics.ts) ---------------------------------------------------------------------
struct FlightState {
    double mach = 0.0, liftCoefficient = 0.0, potential = 0.0, vortex = 0.0, tipCirculation = 0.0, edgeCirculation = 0.0, breakdown = 1.0;
};
FlightState flightState(const WingVapour::Model& m, double airspeed, double alpha, const WingVapour::Air& air) {
    FlightState s;
    const double speed = std::max(airspeed, 0.0);
    s.mach = speed / air.soundMs;
    const double q = 0.5 * air.densityKgM3 * speed * speed;
    const double aspect = m.span * m.span / std::max(m.area, 1e-6), taper = m.tipChord / std::max(m.rootChord, 1e-6);
    const auto sweepAt = [&](double fraction) {
        return std::atan(std::tan(m.sweep) - 4.0 * fraction * (1.0 - taper) / (aspect * (1.0 + taper)));
    };
    // DATCOM's lift slope (Helmbold's, swept) and Polhamus's vortex lift
    const double mm = clampd(s.mach, 0.0, kLiftSlopeMaxMach), b = std::sqrt(1.0 - mm * mm), half = std::tan(sweepAt(0.5));
    const double kp = 2.0 * kPi * aspect /
                      (2.0 + std::sqrt(aspect * aspect * b * b / (kSectionEfficiency * kSectionEfficiency) * (1.0 + half * half / (b * b)) + 4.0));
    const double kv = std::max(kp - kp * kp / (kPi * aspect), 0.0) / std::max(std::cos(m.sweep), 0.05) * clampd(m.sharpness, 0.0, 1.0);
    // vortex breakdown: Wentz and Kohlman's deltas
    const double sweepDeg = m.sweep / kDegree, te = std::max(1.3 * (sweepDeg - 47.0), 3.0) * kDegree, apex = te + 15.0 * kDegree;
    s.breakdown = clampd((apex - std::abs(alpha)) / (apex - te), 0.0, 1.0);
    const double sn = std::sin(alpha), cs = std::cos(alpha);
    s.potential = kp * sn * cs * cs;
    s.vortex = kv * sn * std::abs(sn) * cs * (0.4 + 0.6 * s.breakdown);
    s.liftCoefficient = s.potential + s.vortex;
    const double lift = q * m.area * s.liftCoefficient;
    s.tipCirculation = speed > 0.0 ? std::abs(lift) / (air.densityKgM3 * speed * kVortexSpacing * m.span) : 0.0;
    s.edgeCirculation = speed * m.area * std::max(s.vortex, 0.0) / (2.0 * kEdgeVortexSpan * m.span);
    return s;
}

// --- the field (vapor-field.ts) --------------------------------------------------------------------------------------
// Every number the shader rebuilds the field from: the CPU's field and the constants only the Mach number sets, in the
// order the shader reads them (kFieldNames)
#define FSIM_VAPOUR_FIELD(X)                                                                                                \
    X(mach) X(speed) X(cosAlpha) X(sinAlpha) X(flowZ) X(rollBias) X(slipBias) X(vortexK) X(saturationDeficit) X(semispan)   \
    X(rootSpan) X(tipLeading) X(tipChord) X(tipHeight) X(sectionLift) X(reachChord) X(separation) X(cos2Sweep)              \
    X(normalMach) X(tipCirculation) X(tipCore2) X(tipStart2) X(tipRollup) X(tipGrowth) X(tipDescent) X(tipReach)           \
    X(tipBound) X(trailSpacing) X(secondSemispan) X(secondTipLeading) X(secondTipChord) X(secondTipHeight)                \
    X(secondCirculation) X(secondCore2) X(secondStart2) X(secondRollup) X(secondGrowth) X(secondDescent) X(secondReach)    \
    X(secondBound) X(secondSpacing) X(edgeApex) X(edgeGradient) X(edgeLength) X(edgeCore) X(edgeHeight) X(edgeBurst)       \
    X(edgeBound) X(wingBound) X(wingBoundBelow) X(evaporation) X(nose) X(bodyLength) X(bodyRadius) X(bodyHeight) X(coneStrength)            \
    X(coneShock) X(coneShockWidth) X(coneFade) X(coneBound) X(coneTail) X(wingScale) X(wingSonic) X(wingBeta) X(wingKt) X(wingCap)    \
    X(wingReach) X(wingTravel) X(wingSupersonic) X(coneScale) X(coneBeta) X(coneKt) X(coneCap) X(coneReach)

struct Field {
#define FSIM_DECLARE(name) double name = 0.0;
    FSIM_VAPOUR_FIELD(FSIM_DECLARE)
#undef FSIM_DECLARE
};

constexpr std::size_t kFieldFloats = 0
#define FSIM_COUNT(name) +1
    FSIM_VAPOUR_FIELD(FSIM_COUNT)
#undef FSIM_COUNT
    ;
constexpr std::size_t kFieldRows = (kFieldFloats + 3) / 4;
static_assert(kFieldRows <= 20, "the field's rows");

// The tip vortices' centrelines, laid in the air (trails.ts): +z, -z; the wing's and the canard's
struct Trails {
    Points positive{}, negative{};
    double spacing = 1.0;
};
struct Layout {
    double tipX, tipY, semispan, cosAlpha, sinAlpha, flowZ, descent, rollup, length;
};

vsg::dvec3 shedOffset(const Layout& l, double along, double side) {
    const double rolled = 1.0 - std::exp(-along / std::max(l.rollup, 1e-3));
    return {l.tipX, l.tipY - along * l.descent, side * l.semispan * (1.0 - (1.0 - kVortexSpacing) * rolled)};
}
Trails straightTrails(const Layout& l) {
    Trails t;
    t.spacing = l.length / (kTrailPoints - 1);
    for (int side = 0; side < 2; ++side)
        for (int i = 0; i < kTrailPoints; ++i) {
            const double along = i * t.spacing;
            vsg::dvec3 p = shedOffset(l, along, side == 0 ? 1.0 : -1.0);
            p += vsg::dvec3(l.cosAlpha, l.sinAlpha, l.flowZ) * along;
            (side == 0 ? t.positive : t.negative)[i] = p;
        }
    return t;
}
vsg::dvec3 trailPoint(const Points& points, double spacing, double along) {
    const double position = clampd(along / std::max(spacing, 1e-6), 0.0, kTrailPoints - 1);
    const int i = std::min(static_cast<int>(std::floor(position)), kTrailPoints - 2);
    return points[i] + (points[i + 1] - points[i]) * (position - i);
}
double nearestAlong(const Points& points, double spacing, const Layout& l, const vsg::dvec3& p) {
    const double length = spacing * (kTrailPoints - 1);
    double along = vsg::dot(p - points[0], vsg::dvec3(l.cosAlpha, l.sinAlpha, l.flowZ));
    if (along < 0.0) return along;
    for (int step = 0; step < 2; ++step) {
        const double here = std::min(along, length);
        const vsg::dvec3 at = trailPoint(points, spacing, here), next = trailPoint(points, spacing, std::min(here + spacing, length));
        vsg::dvec3 tangent = next - at;
        const double n = vsg::length(tangent);
        if (n < 1e-6) break;
        tangent /= n;
        along = std::max(here + vsg::dot(p - at, tangent), 0.0);
    }
    return along;
}

struct Tip {
    double leading, chord, height, semispan, circulation, core2, start2, rollup, growth, descent, reach, positive, negative;
};

struct Geometry {
    Field f;
    const WingVapour::Model* m = nullptr;
    Trails trails, second;
};

double sideLoading(const Field& f, double span) {
    return clampd(1.0 + f.rollBias * span / std::max(f.semispan, 1e-3) + f.slipBias * (span > 0.0 ? 1.0 : span < 0.0 ? -1.0 : 0.0), kSideLo, kSideHi);
}
Tip tipOf(const Field& f, bool wing) {
    return wing ? Tip{f.tipLeading, f.tipChord, f.tipHeight, f.semispan, f.tipCirculation, f.tipCore2, f.tipStart2, f.tipRollup,
                      f.tipGrowth, f.tipDescent, f.tipReach, sideLoading(f, f.semispan), sideLoading(f, -f.semispan)}
                : Tip{f.secondTipLeading, f.secondTipChord, f.secondTipHeight, f.secondSemispan, f.secondCirculation, f.secondCore2,
                      f.secondStart2, f.secondRollup, f.secondGrowth, f.secondDescent, f.secondReach, sideLoading(f, f.secondSemispan),
                      sideLoading(f, -f.secondSemispan)};
}
Layout layoutOf(const Field& f, bool wing) {
    const Tip t = tipOf(f, wing);
    return {t.leading + t.chord, t.height, t.semispan, f.cosAlpha, f.sinAlpha, f.flowZ, t.descent, t.rollup, std::max(t.reach, 1.0)};
}

double vortexDeficit(const Field& f, double circulation, double r2, double core2) {
    return f.vortexK * circulation * circulation / std::max(r2 + core2, 1e-6);
}

double tipDeficit(const Geometry& g, const vsg::dvec3& p) {
    double sum = 0.0;
    for (int which = 0; which < 2; ++which) {
        const Tip tip = tipOf(g.f, which == 0);
        if (tip.circulation <= 0.0) continue;
        const Layout l = layoutOf(g.f, which == 0);
        const Trails& t = which == 0 ? g.trails : g.second;
        for (int side = 0; side < 2; ++side) {
            const double sign = side == 0 ? 1.0 : -1.0;
            const auto& points = side == 0 ? t.positive : t.negative;
            const double le = tip.leading, te = le + tip.chord;
            const double along = nearestAlong(points, t.spacing, l, p);
            const double circulation = tip.circulation * (side == 0 ? tip.positive : tip.negative);
            if (along < 0.0) {
                const double formed = clampd((p.x - le) / std::max(tip.chord, 1e-3), 0.0, 1.0);
                const double ax = clampd(p.x, le, te);
                const double r2 = (p.x - ax) * (p.x - ax) + (p.y - tip.height) * (p.y - tip.height) +
                                  (p.z - sign * tip.semispan) * (p.z - sign * tip.semispan);
                sum += vortexDeficit(g.f, circulation * kTipStartShare * formed, r2, tip.start2);
                continue;
            }
            const vsg::dvec3 at = trailPoint(points, t.spacing, along);
            const double r2 = vsg::length2(p - at);
            const double rolled = 1.0 - std::exp(-along / std::max(tip.rollup, 1e-3));
            sum += vortexDeficit(g.f, circulation * (kTipStartShare + (1.0 - kTipStartShare) * rolled), r2,
                                 tip.start2 + (tip.core2 - tip.start2) * rolled + tip.growth * along);
        }
    }
    return sum;
}

double edgeDeficit(const Geometry& g, const vsg::dvec3& p) {
    const Field& f = g.f;
    const auto& m = *g.m;
    const double along = p.x - f.edgeApex;
    if (along <= 0.0 || f.edgeGradient <= 0.0) return 0.0;
    const double onWing = std::min(along, f.edgeLength), past = along - onWing;
    const double u = clampd(onWing / std::max(f.edgeLength, 1e-3), 0.0, 1.0) * (kStations - 1);
    const int i = std::min(static_cast<int>(u), kStations - 2);
    const double t = u - i;
    const double span = m.pathSpan[i] + (m.pathSpan[i + 1] - m.pathSpan[i]) * t;
    const double surface = m.pathSurface[i] + (m.pathSurface[i + 1] - m.pathSurface[i]) * t;
    const double reach = m.pathReach[i] + (m.pathReach[i + 1] - m.pathReach[i]) * t;
    const double vy = surface + f.edgeHeight * reach + past * (f.sinAlpha / std::max(f.cosAlpha, 0.1));
    const double r2 = (p.y - vy) * (p.y - vy) + (std::abs(p.z) - span) * (std::abs(p.z) - span);
    const double burst = smoothstep(f.edgeBurst, f.edgeBurst + kBurstLength * f.edgeLength, along);
    const double core = std::max(f.edgeCore * reach, 0.02) * (1.0 + kBurstSwell * burst);
    const double leaving = std::exp(-past / (0.3 * std::max(f.edgeLength, 1e-3)));
    const double sign = p.z < 0.0 ? -1.0 : 1.0;
    return vortexDeficit(f, f.edgeGradient * sideLoading(f, sign * span) * onWing * (1.0 - 0.3 * burst) * leaving, r2, core * core);
}

double liftShape(double xi, double separation) {
    return std::sqrt((1.0 - xi) / (xi + kNoseRadius)) * (1.0 - separation) + kPi * (1.0 - xi) * separation;
}

double shockStation(const Field& f, double section, double sonic, double mach) {
    if (mach >= 1.0 || sonic >= 0.0) return 1.0;
    const double b = beta(mach), k = mach * mach / (2.0 * (1.0 + b));
    const double target = sonic * b / (1.0 - k * sonic) * f.cos2Sweep;
    const double q = -target * kPi / std::max(section, 1e-6), q2 = q * q;
    const double attached = (1.0 - q2 * kNoseRadius) / (1.0 + q2), separated = 1.0 - q / kPi;
    const double station = clampd(attached + (separated - attached) * f.separation, 0.0, 1.0);
    return station + (1.0 - station) * kShockTravel * smoothstep(0.75, 1.0, mach);
}

double wingDeficit(const Geometry& g, const vsg::dvec3& p) {
    const Field& f = g.f;
    const double span = std::abs(p.z), eta = span / f.semispan;
    if (eta >= 1.0) return 0.0;
    const auto s = g.m->at(span);
    const double chord = std::max(s.chord, 0.05);
    const double xi = (p.x - s.leading) / chord, along = clampd(xi, 0.0, 1.0);
    // over the upper surface, or under the lower: thickness draws suction on both, lift on the upper (and presses
    // the lower), so near Mach 1 the lower surface's pocket is there too, weaker
    const double bulge = 2.0 * s.thickness * chord * along * (1.0 - along);
    const double above = p.y - (s.mid + bulge), below = (s.mid - bulge) - p.y;
    if (above < 0.0 && below < 0.0) return 0.0;
    const double height = std::max(above, below), lift = above >= 0.0 ? 1.0 : -1.0;
    const double section = f.sectionLift * s.loading * sideLoading(f, p.z) / chord;
    const double mach = f.normalMach;
    const auto normal = [&](double at) {
        const double incompressible = -lift * (section / kPi) * liftShape(at, f.separation) - 1.3 * s.thickness * std::sin(kPi * at);
        return karmanTsien(incompressible / f.cos2Sweep, mach);
    };
    double coefficient = normal(along);
    const double sonic = criticalPressure(mach), peak = normal(0.1);
    if (peak < sonic) {
        const double shock = shockStation(f, section, sonic, mach);
        const double ahead = std::max(1.0 - smoothstep(shock - kShockWidth, shock + kShockWidth, along),
                                      std::exp(-std::max(along - shock, 0.0) * chord / std::max(f.evaporation, 1e-3)));
        coefficient = std::min(coefficient, peak) * ahead + std::max(coefficient, sonic) * (1.0 - ahead);
    }
    const double reach = kWingFieldHeight * std::min(chord, f.reachChord) / std::max(beta(mach), kWingHeightMinBeta) *
                         std::min((along + kNoseRadius) / kNoseReach, 1.0);
    const double blur = kRootBlur * height / f.semispan, root = f.rootSpan / f.semispan;
    const double fade = smoothstep(root - blur, root + kRootFade + blur, eta) * (1.0 - smoothstep(kTipRelief, 1.0, eta)) *
                        smoothstep(-0.04, 0.0, xi) * (1.0 - smoothstep(1.0, 1.06, xi));
    return cappedDeficit(-(kGamma / 2.0) * mach * mach * coefficient, mach) * std::exp(-height / reach) * fade;
}

double bodyRadiusAt(const Field& f, double xi) { return f.bodyRadius * std::pow(std::max(4.0 * xi * (1.0 - xi), 0.0), 0.75); }

double coneDeficit(const Geometry& g, const vsg::dvec3& p) {
    const Field& f = g.f;
    if (f.coneStrength <= 0.0) return 0.0;
    const double xi = (p.x + f.nose) / f.bodyLength;
    // (on past the tail: near Mach 1 the shock stands at the tail and, swept aft away from the body, behind it - the
    // pocket ahead of it with it; ended at the tail, the vapour was cut off flat there as it grew towards Mach 1)
    if (xi <= 0.0) return 0.0;
    const double r = std::hypot(p.y - f.bodyHeight, p.z), radius = bodyRadiusAt(f, std::min(xi, 1.0 - 1e-3));
    const double shock = f.coneShock + kConeShockLean * r / f.bodyLength;
    const double ahead = std::max(1.0 - smoothstep(shock - kConeShockThickness, shock + f.coneShockWidth, xi),
                                  std::exp(-std::max(xi - shock, 0.0) * f.bodyLength / std::max(f.evaporation, 1e-3)));
    const double pocket = smoothstep(shock - kConePocketLength, shock, xi);
    const double b = beta(f.mach);
    const double reach = kConeReach * f.bodyLength / b * (0.2 + 0.8 * pocket);
    const double lateral = std::log(1.0 + reach / std::max(r, radius)) / std::log(1.0 + reach / std::max(radius, 1e-3));
    const double coefficient = karmanTsien(-f.coneStrength * pocket, f.mach);
    // (faded to nothing by its bound, should the vapour ever reach past it - the search's 80 m: never cut off hard)
    const double edge = f.coneBound > 0.0 ? 1.0 - smoothstep(0.75 * f.coneBound, f.coneBound, r) : 1.0;
    return cappedDeficit(-(kGamma / 2.0) * f.mach * f.mach * coefficient, f.mach) * lateral * lateral * ahead * f.coneFade * edge;
}

double fieldDeficit(const Geometry& g, const vsg::dvec3& p) {
    return tipDeficit(g, p) + edgeDeficit(g, p) + wingDeficit(g, p) + coneDeficit(g, p);
}

double slenderSuction(double strength, double ratio, double mach) {
    return strength * (1.0 + std::log(1.0 / beta(mach)) / std::max(std::log(1.0 / std::max(ratio, 1e-3)), 1.0));
}
double bodyCriticalMach(double strength, double ratio) {
    if (strength <= 0.0) return 1.0;
    double lo = 0.3, hi = 1.0;
    for (int i = 0; i < 24; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (-slenderSuction(strength, ratio, mid) > criticalPressure(mid)) lo = mid;
        else hi = mid;
    }
    return 0.5 * (lo + hi);
}
double coneStrengthAt(double strength, double ratio, double mach, double growth) {
    if (strength <= 0.0) return 0.0;
    const double slender = -slenderSuction(strength, ratio, mach), transonic = karmanTsien(-strength, mach);
    const double wanted = slender + (transonic - slender) * growth;
    const double b = beta(mach), k = mach * mach / (2.0 * (1.0 + b));
    return -(wanted * b) / (1.0 - k * wanted);
}

struct State {
    Geometry g;
    vsg::dvec3 lo, hi;
    bool visible = false;
};

/// The trails' history: laid by the caller once the reaches are known; straight when there is none.
using LayTrails = std::function<Trails(const Layout&)>;

// The field and its bounds for one moment of flight (vapor_state)
State vapourState(const WingVapour::Model& m, const WingVapour::Flight& flight, const WingVapour::Air& air, const LayTrails& lay) {
    State st;
    Geometry& g = st.g;
    Field& f = g.f;
    g.m = &m;
    const double speed = std::max(flight.airspeedMs, 1e-3), alpha = flight.alphaRad, sideslip = flight.betaRad;
    const FlightState fs = flightState(m, flight.airspeedMs, alpha, air);
    const double semispan = m.semispan, span = 2.0 * semispan;
    const double aspect = span * span / std::max(m.area, 1e-6), taper = m.tipChord / std::max(m.rootChord, 1e-6);
    const double halfSweep = std::atan(std::tan(m.sweep) - 4.0 * 0.5 * (1.0 - taper) / (aspect * (1.0 + taper)));
    const double effectiveCos = std::sqrt(std::cos(halfSweep));
    const double vortexK = air.densityKgM3 / (8.0 * kPi * kPi * air.pressurePa);
    const double saturation = condensingDeficit(air, moistest(air, kHumiditySpread));
    const double share = m.secondary ? std::min(m.secondArea / std::max(m.area, 1e-3) * kCanardLoading, kMaxSecondShare) : 0.0;
    const double wingShare = m.secondary ? 1.0 - share : 1.0;
    const double wingCirculation = fs.tipCirculation * wingShare;
    const double secondSpan = m.secondary ? 2.0 * m.secondSemispan : 1.0;
    const double secondCirculation = m.secondary ? fs.tipCirculation * share * span / secondSpan : 0.0;
    const double tipCore = kTipCoreRadius * span, secondCore = kTipCoreRadius * secondSpan;
    const double rollupSpans =
        clampd(kRollupCoefficient * span * span / std::max(m.area, 1e-3) / std::max(std::abs(fs.liftCoefficient), 1e-3), kRollupLo, kRollupHi);
    const int tip = kStations - 1;

    f.mach = fs.mach;
    f.speed = speed;
    f.cosAlpha = std::cos(alpha) * std::cos(sideslip);
    f.sinAlpha = std::sin(alpha) * std::cos(sideslip);
    f.flowZ = -std::sin(sideslip);
    f.rollBias = clampd(flight.rollRateRadS * semispan / (speed * std::max(std::abs(alpha), 2.0 * kDegree)), -kMaxBias, kMaxBias);
    f.slipBias = clampd(2.0 * sideslip * std::tan(m.sweep), -kMaxBias, kMaxBias);
    f.vortexK = vortexK;
    f.saturationDeficit = saturation;
    f.semispan = semispan;
    f.rootSpan = m.rootSpan;
    f.tipLeading = m.leading[tip];
    f.tipChord = std::max(m.chord[tip], 1e-3);
    f.tipHeight = m.mid[tip];
    f.separation = clampd(m.sharpness, 0.0, 1.0) * smoothstep(3.0 * kDegree, 8.0 * kDegree, std::abs(alpha));
    f.sectionLift = std::max(fs.potential, 0.0) * (m.area / span);
    f.reachChord = std::max(m.area / span, 0.05);
    f.cos2Sweep = std::max(effectiveCos, 0.05);
    f.normalMach = fs.mach * std::sqrt(effectiveCos);
    f.tipCirculation = kTipCoreShare * wingCirculation;
    f.tipCore2 = tipCore * tipCore;
    f.tipStart2 = std::min(std::pow(kTipStartCore * std::max(m.chord[tip], 1e-3), 2.0), tipCore * tipCore);
    f.tipRollup = rollupSpans * span;
    f.tipGrowth = 4.0 * kEddyViscosity * wingCirculation / speed;
    f.tipDescent = wingCirculation / (2.0 * kPi * kVortexSpacing * span) / speed;
    f.trailSpacing = 1.0;
    f.secondSemispan = m.secondary ? m.secondSemispan : 1.0;
    f.secondTipLeading = m.secondary ? m.secondTipLeading : 0.0;
    f.secondTipChord = m.secondary ? m.secondTipChord : 1.0;
    f.secondTipHeight = m.secondary ? m.secondTipHeight : 0.0;
    f.secondCirculation = kTipCoreShare * secondCirculation;
    f.secondCore2 = secondCore * secondCore;
    f.secondStart2 = std::min(std::pow(kTipStartCore * (m.secondary ? m.secondTipChord : 0.0), 2.0), secondCore * secondCore);
    f.secondRollup = rollupSpans * secondSpan;
    f.secondGrowth = 4.0 * kEddyViscosity * secondCirculation / speed;
    f.secondDescent = secondCirculation / (2.0 * kPi * kVortexSpacing * secondSpan) / speed;
    f.secondSpacing = 1.0;
    f.edgeApex = m.edgeApex;
    f.edgeGradient = alpha > 0.0 && m.edgeLength > 1e-3 ? fs.edgeCirculation / m.edgeLength : 0.0;
    f.edgeLength = m.edgeLength;
    f.edgeCore = kEdgeCoreRadius;
    f.edgeHeight = kEdgeVortexHeight * std::max(std::sin(alpha), 0.0);
    f.edgeBurst = fs.breakdown * m.edgeLength;
    f.evaporation = speed * kEvaporationS;
    f.nose = m.nose;
    f.bodyLength = std::max(m.bodyLength, 1e-3);
    f.bodyRadius = m.bodyRadius;
    f.bodyHeight = m.bodyHeight;
    f.coneShockWidth = kConeShockThickness;
    f.coneFade = 1.0 - smoothstep(kSupersonicFadeLo, kSupersonicFadeHi, fs.mach);

    g.trails = straightTrails(layoutOf(f, true));
    g.second = straightTrails(layoutOf(f, false));

    // the wing's extent outboard of the body
    double front = std::numeric_limits<double>::infinity(), back = -front, low = front, high = -front, deepest = 0.0;
    for (int k = 0; k < kStations; ++k) {
        if (m.stationSpan(k + 1) < m.rootSpan) continue;
        const double half = 0.05 * m.chord[k] * 0.5;
        front = std::min(front, m.leading[k]);
        back = std::max(back, m.leading[k] + m.chord[k]);
        low = std::min(low, m.mid[k] - half);
        high = std::max(high, m.mid[k] + half);
        deepest = std::max(deepest, m.chord[k]);
    }
    const double needed = std::max(saturation, kBoundFloor);
    vsg::dvec3 lo(1e18, 1e18, 1e18), hi(-1e18, -1e18, -1e18);
    const auto grow = [&](const vsg::dvec3& a, const vsg::dvec3& b) {
        lo = vsg::dvec3(std::min(lo.x, a.x), std::min(lo.y, a.y), std::min(lo.z, a.z));
        hi = vsg::dvec3(std::max(hi.x, b.x), std::max(hi.y, b.y), std::max(hi.z, b.z));
        st.visible = true;
    };

    // tip vortices
    for (int which = 0; which < 2; ++which) {
        const Tip t = tipOf(f, which == 0);
        const double strongest = t.circulation * std::max(t.positive, t.negative);
        const double well = vortexK * strongest * strongest;
        const double widest = well / needed - t.core2, first = well * kTipStartShare * kTipStartShare / needed - t.start2;
        if (!(needed < 1.0 && (widest > 0.0 || first > 0.0) && t.circulation > 0.0)) continue;
        const double bound = std::sqrt(std::max(well / needed - t.start2, 0.0)) * 1.15 + 0.1;
        const double reach = std::min(widest > 0.0 ? widest / std::max(t.growth, 1e-9) : t.rollup, kMaxTrailM);
        (which == 0 ? f.tipBound : f.secondBound) = bound;
        (which == 0 ? f.tipReach : f.secondReach) = reach;
        Trails trails = lay ? lay(layoutOf(f, which == 0)) : straightTrails(layoutOf(f, which == 0));
        vsg::dvec3 a(t.leading - bound, t.height - bound, -t.semispan - bound), b(t.leading + t.chord + bound, t.height + bound, t.semispan + bound);
        for (const auto* points : {&trails.positive, &trails.negative})
            for (const auto& q : *points) {
                a = vsg::dvec3(std::min(a.x, q.x - bound), std::min(a.y, q.y - bound), std::min(a.z, q.z - bound));
                b = vsg::dvec3(std::max(b.x, q.x + bound), std::max(b.y, q.y + bound), std::max(b.z, q.z + bound));
            }
        (which == 0 ? g.trails : g.second) = trails;
        grow(a, b);
    }
    // leading-edge vortices: the deepest well along the unburst core
    {
        double steepest = 0.0;
        for (int k = 1; k < kStations; ++k)
            steepest = std::max(steepest, (static_cast<double>(k) / (kStations - 1)) * m.edgeLength / std::max(m.pathReach[k], 1e-3));
        const double heaviest = std::max(sideLoading(f, semispan), sideLoading(f, -semispan));
        const double ratio = f.edgeGradient * heaviest * steepest / std::max(f.edgeCore, 1e-3);
        const double well = vortexK * ratio * ratio;
        if (needed < 1.0 && well > needed && f.edgeGradient > 0.0) {
            const double endReach = m.pathReach[tip], endCore = f.edgeCore * endReach;
            const double circulation = f.edgeGradient * heaviest * m.edgeLength;
            const double widest = std::sqrt(std::max(vortexK * circulation * circulation / needed - endCore * endCore, 0.0));
            f.edgeBound = std::max(widest, (1.0 + kBurstSwell) * endCore) * 1.15 + 0.1;
            const double b = f.edgeBound, trailing = m.edgeLength * 0.6;
            const double top = *std::max_element(m.pathSurface.begin(), m.pathSurface.end()) + f.edgeHeight * endReach +
                               trailing * (f.sinAlpha / std::max(f.cosAlpha, 0.1));
            grow(vsg::dvec3(m.edgeApex - b, low - b, -semispan - b), vsg::dvec3(m.edgeApex + m.edgeLength + trailing + b, top + b, semispan + b));
        }
    }
    // the wing's surfaces: over the upper, and under the lower (where it fogs at all - near Mach 1)
    {
        double strongest = 0.0, strongestBelow = 0.0;
        for (double xi : {0.0, 0.02, 0.05, 0.1, 0.2, 0.35, 0.5, 0.7, 0.9})
            for (int j = 0; j <= 6; ++j) {
                const double s = m.rootSpan + (semispan - m.rootSpan) * j / 6.0;
                const auto at = m.at(s);
                const double x = at.leading + at.chord * xi, bulge = 2.0 * at.thickness * at.chord * xi * (1.0 - xi);
                for (double side : {1.0, -1.0}) {
                    const double z = side * std::min(s, semispan * 0.999);
                    strongest = std::max(strongest, wingDeficit(g, vsg::dvec3(x, at.mid + bulge + 1e-3, z)));
                    strongestBelow = std::max(strongestBelow, wingDeficit(g, vsg::dvec3(x, at.mid - bulge - 1e-3, z)));
                }
            }
        if (needed < 1.0 && strongest > needed && std::isfinite(front)) {
            const double reach = kWingFieldHeight * std::min(deepest, f.reachChord) / std::max(beta(f.normalMach), kWingHeightMinBeta);
            f.wingBound = reach * std::log(strongest / needed) * 1.1 + 0.05;
            // (under the wing only where it fogs: marched through for nothing, the clear air there cost 10 % in a pull)
            f.wingBoundBelow = strongestBelow > needed ? reach * std::log(strongestBelow / needed) * 1.1 + 0.05 : 0.0;
            grow(vsg::dvec3(front - 0.05 * deepest, low - std::max(f.wingBoundBelow, 0.05), -semispan),
                 vsg::dvec3(back + 0.1 * f.tipChord, high + f.wingBound, semispan));
        }
    }
    // the cone
    {
        const double ratio = m.bodyRadius / f.bodyLength;
        const double strength = ratio > 0.0 ? kConeStrength * ratio * ratio * std::log(1.0 / ratio) : 0.0;
        const double critical = bodyCriticalMach(strength, ratio);
        const double growth = smoothstep(critical, critical + kPocketGrowth, fs.mach);
        f.coneStrength = coneStrengthAt(strength, ratio, fs.mach, growth);
        f.coneShockWidth = kConeRecompression + (kConeShockThickness - kConeRecompression) * growth;
        f.coneShock = kConeShockSubsonic + (kConeShockSonic - kConeShockSubsonic) * smoothstep(critical, 1.01, fs.mach) +
                      (kConeShockSupersonic - kConeShockSonic) * smoothstep(1.0, kSupersonicFadeLo + 0.03, fs.mach);
        double strongest = 0.0;
        for (int i = 1; i < 40; ++i) {
            const double xi = i / 40.0;
            if (xi < f.coneShock)
                strongest = std::max(strongest, coneDeficit(g, vsg::dvec3(xi * f.bodyLength - f.nose, f.bodyHeight + bodyRadiusAt(f, xi), 0.0)));
        }
        // (sized at what condenses, not the floor the other parts are sized at: in near-saturated air - 95 % - that
        // floor stood well above it, the cone's vapour reached half as far again as its bound, and was cut off on a
        // circle round the aircraft)
        const double fogs = saturation;
        if (fogs < 1.0 && strongest > fogs && f.coneFade > 0.0) {
            // how far out it fogs: at each distance from the axis, the strongest the deficit is along the body there -
            // just ahead of the shock, which sweeps aft the further out (kConeShockLean). Sought at the one station
            // the shock has on the body, the radius came short wherever the swept shock let the pocket reach further
            // out, and the field was cut there: a circle round the cone, plain as the vapour grew near Mach 1
            const auto strongestOut = [&](double r) {
                const double shock = f.coneShock + kConeShockLean * r / f.bodyLength;
                double most = 0.0;
                for (int k = 0; k <= 24; ++k) {
                    const double xi = shock - kConePocketLength + (kConePocketLength + 0.01) * k / 24.0;
                    if (xi <= 0.0 || xi >= 1.0) continue;
                    most = std::max(most, coneDeficit(g, vsg::dvec3(xi * f.bodyLength - f.nose, f.bodyHeight + r, 0.0)));
                }
                return most;
            };
            double a = bodyRadiusAt(f, f.coneShock - 0.02), b = 80.0;
            if (strongestOut(b) > fogs) a = b;
            for (int i = 0; i < 32 && a < b; ++i) {
                const double mid = 0.5 * (a + b);
                if (strongestOut(mid) > fogs) a = mid;
                else b = mid;
            }
            f.coneBound = a * 1.15 + 0.2;
            // behind the shock the droplets evaporate over a distance (f.evaporation, at the speed): what fogs runs on
            // until the deficit, falling by e every that far, is under what condenses - ended 2 % of the length behind
            // the shock, the cone was cut off flat there, before its vapour had gone
            f.coneTail = f.evaporation * std::log(std::max(strongest / fogs, 1.0)) * 1.15 + 0.2;
            const double bb = f.coneBound;
            const double shockX = (f.coneShock + f.coneShockWidth + kConeShockLean * bb / f.bodyLength + 0.02) * f.bodyLength - f.nose + f.coneTail;
            grow(vsg::dvec3(-f.nose, f.bodyHeight - bb, -bb), vsg::dvec3(shockX, f.bodyHeight + bb, bb));
        } else {
            f.coneStrength = 0.0;
        }
    }
    f.trailSpacing = g.trails.spacing;
    f.secondSpacing = g.second.spacing;
    // the shader's constants (vapor_constants)
    const double wm = f.normalMach, wb = beta(wm), cb = beta(f.mach);
    f.wingScale = kGamma / 2.0 * wm * wm;
    f.wingSonic = criticalPressure(wm);
    f.wingBeta = wb;
    f.wingKt = wm * wm / (2.0 * (1.0 + wb));
    f.wingCap = mostDeficit(wm);
    f.wingReach = kWingFieldHeight / std::max(wb, kWingHeightMinBeta);
    f.wingTravel = kShockTravel * smoothstep(0.75, 1.0, wm);
    f.wingSupersonic = wm >= 1.0 ? 1.0 : 0.0;
    f.coneScale = kGamma / 2.0 * f.mach * f.mach;
    f.coneBeta = cb;
    f.coneKt = f.mach * f.mach / (2.0 * (1.0 + cb));
    f.coneCap = mostDeficit(f.mach);
    f.coneReach = kConeReach * f.bodyLength / cb;
    st.lo = lo;
    st.hi = hi;
    return st;
}

// body (x forward, y right, z down) <-> canonical (x aft, y up, z right)
vsg::dvec3 toCanonical(const vsg::dvec3& b) { return {-b.x, -b.z, b.y}; }
vsg::dvec3 toBody(const vsg::dvec3& c) { return {-c.x, c.z, -c.y}; }

} // namespace

// --- the path through the air (trails.ts TrailHistory) ---------------------------------------------------------------
// Positions in the air (m, ECEF axes), attitudes as unit quaternions body -> ECEF: kept in the body frame, a proper
// rotation (the canonical frame is a mirror of it), and turned into the canonical frame as the trails are laid
struct WingVapour::History {
    struct Q {
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    };
    struct Sample {
        vsg::dvec3 position;
        Q attitude;
        double flown;
    };
    std::deque<Sample> samples;
    vsg::dvec3 position;
    double flown = 0.0;

    static Q fromMatrix(const double* r) { // row-major, columns the body axes in ECEF
        Q q;
        const double t = r[0] + r[4] + r[8];
        if (t > 0.0) {
            const double s = std::sqrt(t + 1.0) * 2.0;
            q = {0.25 * s, (r[7] - r[5]) / s, (r[2] - r[6]) / s, (r[3] - r[1]) / s};
        } else if (r[0] > r[4] && r[0] > r[8]) {
            const double s = std::sqrt(1.0 + r[0] - r[4] - r[8]) * 2.0;
            q = {(r[7] - r[5]) / s, 0.25 * s, (r[1] + r[3]) / s, (r[2] + r[6]) / s};
        } else if (r[4] > r[8]) {
            const double s = std::sqrt(1.0 + r[4] - r[0] - r[8]) * 2.0;
            q = {(r[2] - r[6]) / s, (r[1] + r[3]) / s, 0.25 * s, (r[5] + r[7]) / s};
        } else {
            const double s = std::sqrt(1.0 + r[8] - r[0] - r[4]) * 2.0;
            q = {(r[3] - r[1]) / s, (r[2] + r[6]) / s, (r[5] + r[7]) / s, 0.25 * s};
        }
        return q;
    }
    static Q nlerp(const Q& a, Q b, double t) {
        if (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0.0) b = {-b.w, -b.x, -b.y, -b.z};
        Q q{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t};
        const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        return n > 0.0 ? Q{q.w / n, q.x / n, q.y / n, q.z / n} : a;
    }
    static vsg::dvec3 rotate(const Q& q, const vsg::dvec3& v) {
        const vsg::dvec3 u(q.x, q.y, q.z);
        const vsg::dvec3 t = vsg::cross(u, v) * 2.0;
        return v + t * q.w + vsg::cross(u, t);
    }
    static vsg::dvec3 unrotate(const Q& q, const vsg::dvec3& v) { return rotate(Q{q.w, -q.x, -q.y, -q.z}, v); }

    void reset() {
        samples.clear();
        flown = 0.0;
        position = vsg::dvec3();
    }
    /// `flow`: the free stream in the canonical frame
    void advance(double dt, const Q& attitude, double airspeed, const vsg::dvec3& flow, double keep) {
        constexpr double kMaxGap = 2.0, kMaxStep = 1.0 / 30.0;
        if (dt > kMaxGap) reset();
        const bool had = !samples.empty();
        const Q previous = had ? samples.back().attitude : attitude;
        const double step = std::max(dt, 0.0);
        const int steps = had ? std::clamp(static_cast<int>(std::ceil(step / kMaxStep)), 1, 120) : 1;
        for (int k = 1; k <= steps; ++k) {
            const Q turned = had ? nlerp(previous, attitude, static_cast<double>(k) / steps) : attitude;
            // the aircraft flies against the free stream
            const vsg::dvec3 velocity = rotate(turned, -toBody(flow)) * airspeed;
            position += velocity * (step / steps);
            flown += airspeed * step / steps;
            if (!samples.empty() && flown - samples.back().flown < 1e-6) {
                samples.back().attitude = turned;
                continue;
            }
            samples.push_back({position, turned, flown});
        }
        while (samples.size() > 2 && (flown - samples[1].flown > keep || samples.size() > 1024)) samples.pop_front();
    }
    Trails paths(const Layout& layout) const {
        if (samples.size() < 2) return straightTrails(layout);
        const Sample& now = samples.back();
        Trails t;
        t.spacing = layout.length / (kTrailPoints - 1);
        for (int side = 0; side < 2; ++side) {
            std::size_t older = samples.size() - 1;
            for (int i = 0; i < kTrailPoints; ++i) {
                const double along = i * t.spacing, at = now.flown - along;
                while (older > 0 && samples[older].flown > at) --older;
                const Sample& before = samples[older];
                const Sample& after = samples[std::min(older + 1, samples.size() - 1)];
                const double span = after.flown - before.flown;
                const double u = span > 1e-6 ? clampd((at - before.flown) / span, 0.0, 1.0) : 0.0;
                const Q attitude = nlerp(before.attitude, after.attitude, u);
                vsg::dvec3 where = before.position + (after.position - before.position) * u;
                const double beyond = std::max(before.flown - at, 0.0);
                if (beyond > 0.0) where += rotate(before.attitude, toBody(vsg::dvec3(layout.cosAlpha, layout.sinAlpha, layout.flowZ))) * beyond;
                const vsg::dvec3 p =
                    unrotate(now.attitude, rotate(attitude, toBody(shedOffset(layout, along, side == 0 ? 1.0 : -1.0))) + where - now.position);
                (side == 0 ? t.positive : t.negative)[i] = toCanonical(p);
            }
        }
        return t;
    }
};

// --- the static rules (for the tests and the docs) -------------------------------------------------------------------
WingVapour::Air WingVapour::moistAir(double altitudeM, double relativeHumidity, double temperatureOffsetK) {
    double T, p;
    standardAtmosphere(altitudeM, T, p);
    Air a;
    a.temperatureK = std::max(T + temperatureOffsetK, 150.0);
    a.relativeHumidity = clampd(relativeHumidity, 0.0, 1.0);
    const double vapour = std::min(a.relativeHumidity * saturationPressure(a.temperatureK), p * 0.5);
    a.pressurePa = p;
    a.densityKgM3 = (p - 0.378 * vapour) / (kGasConstant * a.temperatureK);
    a.soundMs = std::sqrt(kGamma * kGasConstant * a.temperatureK);
    a.mixingRatio = mixingRatio(vapour, p);
    return a;
}

double WingVapour::defaultHumidity(double altitudeM) {
    // aeronautic's default day (types.ts default_vapor_air): "a humid summer day, low over the sea", 85 %
    (void)altitudeM;
    return 0.85;
}

double WingVapour::liquidWater(double pressureRatio, const Air& air, double humidityScale) {
    double T, w;
    condensate(pressureRatio, air, humidityScale, T, w);
    return w;
}

double WingVapour::saturationDeficit(const Air& air) { return condensingDeficit(air, moistest(air, kHumiditySpread)); }

double WingVapour::deficit(const VehicleVisuals::Shape& shape, const Flight& flight, const Air& air, const vsg::dvec3& body) {
    const auto m = makeModel(shape);
    const State st = vapourState(*m, flight, air, {});
    return fieldDeficit(st.g, toCanonical(body));
}

bool WingVapour::bounds(const VehicleVisuals::Shape& shape, const Flight& flight, const Air& air, vsg::dvec3& lo, vsg::dvec3& hi) {
    const auto m = makeModel(shape);
    const State st = vapourState(*m, flight, air, {});
    if (!st.visible) return false;
    const vsg::dvec3 a = toBody(st.lo), b = toBody(st.hi);
    lo = vsg::dvec3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
    hi = vsg::dvec3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
    return true;
}

// --- the drawing (vapor-material.ts, tsl/field.ts) -------------------------------------------------------------------
namespace {

// Per vapour drawn, in one storage buffer: the field's rows, the frame's (the box, the light, the budget), the
// wing's tables, the trails and the condensation table
constexpr int kRowField = 0, kRowFrame = 20, kRowShape = 32, kRowLoading = 96, kRowPath = 160, kRowTrails = 224, kRowTable = 480;
constexpr int kRowsPer = 608;


// The dither the gas is marched with (fsim: aeronautic's interleaved gradient noise is a lattice, and under a thin
// layer's error it stood on the screen as a pattern of dots): takram's blue noise, its first slice, held still.
const char* kDither = R"(
layout(set = 0, binding = 15) uniform sampler3D blueNoise;
float gasDither(vec2 pixel) { return texelFetch(blueNoise, ivec3(ivec2(floor(pixel)) & 127, 0), 0).r; }
)";

// The field and the march (aeronautic's wing-vapor material, as the fragment shader of a box per aircraft had them),
// for the compute pass that marches the vapour at a share of the window's pixels (kCompute).
const char* kField = R"(
const float PI = 3.14159265;
const int STATIONS = 64, TRAIL_POINTS = 64, TEXELS = 128;
const float FAR = 1e5, BOUND_SAFETY = 0.85, TRAIL_SAFETY = 0.85, BOUND_FLOOR = 0.003, TIP_START_SHARE = 0.4;
const float BURST_LENGTH = 0.08, BURST_SWELL = 2.0, NOSE_RADIUS = 0.03, NOSE_REACH = 0.3, ROOT_BLUR = 1.0, ROOT_FADE = 0.12;
const float CONDENSATION_ONSET = 1.0;
const float TIP_RELIEF = 0.7; // (kTipRelief)
const float SHOCK_WIDTH = 0.015, CONE_POCKET_LENGTH = 0.45, CONE_SHOCK_LEAN = 0.08, CONE_SHOCK_THICKNESS = 0.01, MIN_RATIO = 0.3;
const float OPAQUE = 0.01, MULTIPLE_SCATTERING = 4.0, ISOTROPIC_SHARE = 0.5, MULTIPLE_SHADE = 0.25, PIXEL_STEP = 0.0015;

vec4 F[FIELD_ROWS];
int B;

vec4 shapeRow(int row, float share) {
    float p = clamp(share, 0.0, 1.0) * float(STATIONS - 1);
    int i = min(int(floor(p)), STATIONS - 2);
    return mix(data[B + row + i], data[B + row + i + 1], p - float(i));
}
vec3 trailTexel(int row, int i) { return data[B + ROW_TRAILS + row * TRAIL_POINTS + i].xyz; }
vec3 trailAt(int row, float along, float spacing) {
    float p = clamp(along / max(spacing, 1e-6), 0.0, float(TRAIL_POINTS - 1));
    int i = min(int(floor(p)), TRAIL_POINTS - 2);
    return mix(trailTexel(row, i), trailTexel(row, i + 1), p - float(i));
}
void trailSpan(int row, float along, float spacing, out vec3 a, out vec3 b) {
    float p = clamp(along / max(spacing, 1e-6), 0.0, float(TRAIL_POINTS - 1));
    int i = min(int(floor(p)), TRAIL_POINTS - 2);
    float t = p - float(i);
    vec3 x0 = trailTexel(row, i), x1 = trailTexel(row, i + 1), x2 = trailTexel(row, min(i + 2, TRAIL_POINTS - 1));
    a = mix(x0, x1, t);
    b = mix(x1, x2, t);
}
float sideLoading(float span) {
    return clamp(f_rollBias * span / max(f_semispan, 1e-3) + f_slipBias * sign(span) + 1.0, 0.2, 1.8);
}
float vortexDeficit(float circulation, float r2, float core2) { return f_vortexK * circulation * circulation / max(r2 + core2, 1e-6); }
float capped(float d, float cap) { return d > 0.0 ? cap * (1.0 - exp(-d / max(cap, 1e-4))) : d; }
float karmanTsien(float incompressible, float b, float kt) { return incompressible / max(b + kt * incompressible, b * 0.5); }

// one tip vortex: its deficit, how far outside the tube it can fog in, the step there
vec3 tipSide(bool wing, vec3 p, float side) {
    float leading = wing ? f_tipLeading : f_secondTipLeading, chord = wing ? f_tipChord : f_secondTipChord;
    float tipY = wing ? f_tipHeight : f_secondTipHeight, semispan = wing ? f_semispan : f_secondSemispan;
    float circulation0 = wing ? f_tipCirculation : f_secondCirculation, tipCore2 = wing ? f_tipCore2 : f_secondCore2;
    float start2 = wing ? f_tipStart2 : f_secondStart2, rollup = wing ? f_tipRollup : f_secondRollup;
    float growth = wing ? f_tipGrowth : f_secondGrowth, reach = wing ? f_tipReach : f_secondReach;
    float bound = wing ? f_tipBound : f_secondBound, spacing = wing ? f_trailSpacing : f_secondSpacing;
    int row = (wing ? 0 : 2) + (side > 0.0 ? 0 : 1);
    float te = leading + chord, lengthM = spacing * float(TRAIL_POINTS - 1);
    vec3 start = trailTexel(row, 0);
    float along = dot(p - start, vec3(f_cosAlpha, f_sinAlpha, f_flowZ));
    bool ahead = along < 0.0;
    vec3 axis = vec3(clamp(p.x, leading, te), tipY, semispan * side);
    if (!ahead) {
        for (int k = 0; k < 2; ++k) {
            float here = clamp(along, 0.0, lengthM);
            vec3 at, next;
            trailSpan(row, here, spacing, at, next);
            vec3 tangent = normalize(next - at);
            along = max(here + dot(p - at, tangent), 0.0);
        }
        axis = trailAt(row, along, spacing);
    }
    float trailed = max(along, 0.0);
    vec3 offset = p - axis;
    float radius2 = dot(offset, offset);
    float rolled = 1.0 - exp(-trailed / max(rollup, 1e-3));
    float formed = ahead ? clamp((p.x - leading) / chord, 0.0, 1.0) * TIP_START_SHARE : rolled * (1.0 - TIP_START_SHARE) + TIP_START_SHARE;
    float core2 = start2 + (tipCore2 - start2) * rolled + growth * trailed;
    float circulation = circulation0 * formed * sideLoading(semispan * side);
    float deficit = vortexDeficit(circulation, radius2, core2);
    float needed = max(f_saturationDeficit, BOUND_FLOOR);
    float wide2 = f_vortexK * circulation * circulation / needed - core2;
    float wide = min(sqrt(max(wide2, 0.0)) * 1.15 + 0.1, bound);
    float outside = min((sqrt(radius2) - wide + max(along - reach, 0.0)) * TRAIL_SAFETY, bound * 4.0 + 1.0);
    return vec3(deficit, bound > 0.0 ? outside : FAR, clamp(sqrt(core2) * 0.4, 0.03, 1.5));
}
vec3 tipVortex(vec3 p) {
    float deficit = 0.0, outside = FAR, step = FAR;
    for (int w = 0; w < 2; ++w) {
        bool wing = w == 0;
        if ((wing ? f_tipBound : f_secondBound) <= 0.0) continue;
        for (int s = 0; s < 2; ++s) {
            vec3 part = tipSide(wing, p, s == 0 ? 1.0 : -1.0);
            deficit += part.x;
            step = part.y < outside ? part.z : step;
            outside = min(outside, part.y);
        }
    }
    return vec3(deficit, outside, step);
}

// a leading-edge vortex; p folded, side which wing
vec3 edgePart(vec3 p, float side) {
    float along = p.x - f_edgeApex, lengthM = max(f_edgeLength, 1e-3);
    float onWing = clamp(along, 0.0, f_edgeLength), past = max(along - f_edgeLength, 0.0);
    vec4 path = shapeRow(ROW_PATH, onWing / lengthM);
    float vz = path.x, reach = path.z;
    float vy = path.y + f_edgeHeight * reach + past * (f_sinAlpha / max(f_cosAlpha, 0.1));
    vec2 across = vec2(p.y - vy, p.z - vz);
    float radius2 = dot(across, across);
    float burst = smoothstep(f_edgeBurst, f_edgeBurst + lengthM * BURST_LENGTH, along);
    float core = max(f_edgeCore * reach, 0.02) * (burst * BURST_SWELL + 1.0);
    float leaving = exp(-past / (lengthM * 0.3));
    float deficit = along > 0.0 ? vortexDeficit(f_edgeGradient * sideLoading(vz * side) * onWing * (1.0 - 0.3 * burst) * leaving, radius2, core * core) : 0.0;
    float outside = (sqrt(radius2) - f_edgeBound + max(-along, 0.0) + max(past - lengthM * 0.6, 0.0)) * BOUND_SAFETY;
    return vec3(deficit, f_edgeBound > 0.0 ? outside : FAR, clamp(core * 0.4, 0.03, 1.0));
}

float liftShape(float at) { return mix(sqrt((1.0 - at) / (at + NOSE_RADIUS)), (1.0 - at) * PI, f_separation); }

// the wing's upper surface; p folded
vec3 wingPart(vec3 p, float side) {
    float span = p.z, eta = span / f_semispan;
    vec4 station = shapeRow(ROW_SHAPE, eta);
    float loading = shapeRow(ROW_LOADING, eta).x;
    float leading = station.x, chord = max(station.y, 0.05), thickness = station.w;
    float xi = (p.x - leading) / chord, along = clamp(xi, 0.0, 1.0);
    // fsim: over the upper surface, or under the lower - thickness draws suction on both, lift on the upper (and
    // presses the lower), so near Mach 1 the lower surface's pocket is there too, weaker; drawn over the upper alone,
    // the vapour ended in a plane at the wing, a hard straight edge across the span seen from ahead or astern
    float bulge = thickness * chord * 2.0 * along * (1.0 - along);
    float surface = station.z + bulge, lower = station.z - bulge;
    float above = p.y - surface, below = lower - p.y;
    float height = max(above, below), lift = above >= 0.0 ? 1.0 : -1.0;
    float section = lift * f_sectionLift * loading * sideLoading(span * side) / chord;
    float here = karmanTsien((section / -PI * liftShape(along) - thickness * 1.3 * sin(along * PI)) / f_cos2Sweep, f_wingBeta, f_wingKt);
    float peak = karmanTsien((section / -PI * liftShape(0.1) - thickness * 1.3 * sin(0.1 * PI)) / f_cos2Sweep, f_wingBeta, f_wingKt);
    float target = f_wingSonic * f_wingBeta / (1.0 - f_wingKt * f_wingSonic) * f_cos2Sweep;
    float q = target * -PI / max(abs(section), 1e-6), q2 = q * q;
    float sonicAt = clamp(mix((1.0 - q2 * NOSE_RADIUS) / (q2 + 1.0), 1.0 - q / PI, f_separation), 0.0, 1.0);
    float shock = (f_wingSupersonic > 0.5 || f_wingSonic >= 0.0) ? 1.0 : sonicAt + (1.0 - sonicAt) * f_wingTravel;
    float ahead = max(1.0 - smoothstep(shock - SHOCK_WIDTH, shock + SHOCK_WIDTH, along),
                      exp(-max(along - shock, 0.0) * chord / max(f_evaporation, 1e-3)));
    float pocket = mix(max(here, f_wingSonic), min(here, peak), ahead);
    float coefficient = peak < f_wingSonic ? pocket : here;
    float reach = f_wingReach * min(chord, f_reachChord) * min((along + NOSE_RADIUS) / NOSE_REACH, 1.0);
    float root = f_rootSpan / f_semispan;
    float blur = max(height, 0.0) * ROOT_BLUR / f_semispan; // (held off negative: inside the wing nothing shows)
    float fade = smoothstep(root - blur, root + ROOT_FADE + blur, eta) * (1.0 - smoothstep(TIP_RELIEF, 1.0, eta)) *
                 smoothstep(-0.04, 0.0, xi) * (1.0 - smoothstep(1.0, 1.06, xi));
    float atSurface = capped(-coefficient * f_wingScale, f_wingCap) * fade;
    float top = reach * log(max(atSurface, 1e-6) / max(f_saturationDeficit, 1e-6));
    float deficit = (height < 0.0 || eta >= 1.0 || (below >= 0.0 && f_wingBoundBelow <= 0.0)) ? 0.0 : atSurface * exp(-height / reach);
    float vertical = max(lower - f_wingBoundBelow - p.y, p.y - surface - f_wingBound);
    float outside = max(max(vertical, max(-xi, xi - 1.06) * chord), max(f_rootSpan - span, span - f_semispan)) * BOUND_SAFETY;
    return vec3(deficit, f_wingBound > 0.0 ? outside : FAR, clamp(max(reach * 0.25, (height - top) * 0.5), 0.04, 1.0));
}

// the slender body: the vapour cone
vec3 conePart(vec3 p) {
    float xi = (p.x + f_nose) / f_bodyLength, along = clamp(xi, 1e-3, 1.0 - 1e-3);
    float r = length(vec2(p.y - f_bodyHeight, p.z));
    float radius = f_bodyRadius * pow(max(along * (1.0 - along) * 4.0, 1e-3), 0.75);
    float shock = f_coneShock + r * CONE_SHOCK_LEAN / f_bodyLength;
    float ahead = max(1.0 - smoothstep(shock - CONE_SHOCK_THICKNESS, shock + f_coneShockWidth, xi),
                      exp(-max(xi - shock, 0.0) * f_bodyLength / max(f_evaporation, 1e-3)));
    float pocket = smoothstep(shock - CONE_POCKET_LENGTH, shock, xi);
    float reach = f_coneReach * (pocket * 0.8 + 0.2);
    float lateral = log(reach / max(r, radius) + 1.0) / log(reach / max(radius, 1e-3) + 1.0);
    float coefficient = karmanTsien(-f_coneStrength * pocket, f_coneBeta, f_coneKt);
    float edge = 1.0 - smoothstep(0.75 * f_coneBound, f_coneBound, r); // (faded to nothing by its bound: never cut off hard)
    float deficit = xi > 0.0 ? capped(-coefficient * f_coneScale, f_coneCap) * lateral * lateral * ahead * f_coneFade * edge : 0.0;
    float shockX = (shock + f_coneShockWidth + 0.02) * f_bodyLength - f_nose + f_coneTail;
    float outside = max(r - f_coneBound, max(-f_nose - p.x, p.x - shockX)) * BOUND_SAFETY;
    return vec3(deficit, f_coneBound > 0.0 ? outside : FAR, clamp(f_coneBound / 14.0, 0.1, 1.5));
}

// the whole field: its deficit, how far the point is from anything that can fog, the step there, and how far from
// anything but the tips
vec4 field(vec3 point, bool tips) {
    vec3 folded = vec3(point.x, point.y, abs(point.z));
    float side = point.z < 0.0 ? -1.0 : 1.0;
    vec3 parts[4];
    int n = 0;
    parts[n++] = edgePart(folded, side);
    parts[n++] = wingPart(folded, side);
    parts[n++] = conePart(point);
    float untipped = min(min(parts[0].y, parts[1].y), parts[2].y);
    if (tips) parts[n++] = tipVortex(point);
    float deficit = 0.0, outside = FAR, step = FAR;
    for (int i = 0; i < n; ++i) {
        deficit += parts[i].x;
        outside = min(outside, parts[i].y);
        step = min(step, parts[i].y <= 0.0 ? parts[i].z : FAR);
    }
    return vec4(deficit, outside, step, untipped);
}

// the condensation table: grams of water per cubic metre, drier, the day's and moister
vec4 waterAt(float deficit) {
    float ratio = 1.0 - clamp(deficit, 0.0, 1.0 - MIN_RATIO);
    float u = clamp((ratio - MIN_RATIO) / (1.0 - MIN_RATIO), 0.0, 1.0) * float(TEXELS - 1);
    int i = min(int(floor(u)), TEXELS - 2);
    return mix(data[B + ROW_TABLE + i], data[B + ROW_TABLE + i + 1], u - float(i));
}

// three.js r180 tonemapping_pars_fragment: AgX (Filament's, after Blender's), linear sRGB in and out, exposure 1
vec3 agx(vec3 color) {
    const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(vec3(0.6274, 0.0691, 0.0164), vec3(0.3293, 0.9195, 0.0880), vec3(0.0433, 0.0113, 0.8956));
    const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(vec3(1.6605, -0.1246, -0.0182), vec3(-0.5876, 1.1329, -0.1006), vec3(-0.0728, -0.0083, 1.1187));
    const mat3 AgXInsetMatrix = mat3(vec3(0.856627153315983, 0.137318972929847, 0.11189821299995),
                                     vec3(0.0951212405381588, 0.761241990602591, 0.0767994186031903),
                                     vec3(0.0482516061458583, 0.101439036467562, 0.811302368396859));
    const mat3 AgXOutsetMatrix = mat3(vec3(1.1271005818144368, -0.1413297634984383, -0.14132976349843826),
                                      vec3(-0.11060664309660323, 1.157823702216272, -0.11060664309660294),
                                      vec3(-0.016493938717834573, -0.016493938717834257, 1.2519364065950405));
    const float AgxMinEv = -12.47393, AgxMaxEv = 4.026069;
    color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
    color = AgXInsetMatrix * color;
    color = max(color, 1e-10);
    color = log2(color);
    color = (color - AgxMinEv) / (AgxMaxEv - AgxMinEv);
    color = clamp(color, 0.0, 1.0);
    vec3 x2 = color * color, x4 = x2 * x2;
    color = 15.5 * x4 * x2 - 40.14 * x4 * color + 31.96 * x4 - 6.868 * x2 * color + 0.4298 * x2 + 0.1191 * color - 0.00232;
    color = AgXOutsetMatrix * color;
    color = pow(max(vec3(0.0), color), vec3(2.2));
    color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
    return clamp(color, 0.0, 1.0);
}

float hashCell(vec3 c) {
    vec3 p = fract(c * 0.3183099 + vec3(0.1, 0.2, 0.3)) * 17.0;
    return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}
float valueNoise(vec3 x) {
    vec3 c = floor(x), o = fract(x), e = o * o * (3.0 - 2.0 * o);
    return mix(mix(mix(hashCell(c), hashCell(c + vec3(1, 0, 0)), e.x), mix(hashCell(c + vec3(0, 1, 0)), hashCell(c + vec3(1, 1, 0)), e.x), e.y),
               mix(mix(hashCell(c + vec3(0, 0, 1)), hashCell(c + vec3(1, 0, 1)), e.x), mix(hashCell(c + vec3(0, 1, 1)), hashCell(c + vec3(1, 1, 1)), e.x), e.y), e.z);
}
// fsim: the air's humidity patches, -1..1 (a few per cent drier .. moister): four octaves of eddies from `eddy` up, each
// as long along the flow as the shutter smears it, weighted as a passive scalar's turbulence has them (Obukhov and
// Corrsin: a -5/3 spectrum, an octave's amplitude growing with its size to the 1/3) - most of the variation in the
// larger eddies. aeronautic's two octaves at the smallest size, smeared nine times longer along the flow than across,
// stood on the wing as even stripes; the contrast is theirs (the sum's spread scaled to their two octaves')
float patchiness(float downstream, vec2 across, float eddy, float smear) {
    float sum = 0.0, weights = 0.0, size = eddy, weight = 1.0;
    for (int o = 0; o < 4; ++o) {
        sum += weight * valueNoise(vec3(downstream / (size + smear), across / size) + vec3(5.2, 1.3, 7.7) * float(o));
        weights += weight;
        size *= 2.37;
        weight *= 1.333; // (2.37^(1/3))
    }
    return (sum / weights - 0.5) * 3.4;
}
float henyeyGreenstein(float c, float g) {
    float g2 = g * g;
    return (1.0 - g2) / pow(max(g2 + 1.0 - 2.0 * g * c, 1e-4), 1.5) / (4.0 * PI);
}

// One aircraft's vapour along a ray (its frame: the camera, the direction), ending at `solid` (m along it): the light
// as aeronautic's material leaves it (premultiplied, tone mapped) and its opacity; `first` where the cloud starts.
vec4 march(int base, vec3 camera, vec3 direction, float solid, float jitter, out float first) {
    first = -1.0;
    B = base;
    vec4 r0 = data[B + ROW_FRAME], r1 = data[B + ROW_FRAME + 1], r2 = data[B + ROW_FRAME + 2], r3 = data[B + ROW_FRAME + 3];
    vec4 r4 = data[B + ROW_FRAME + 4], r5 = data[B + ROW_FRAME + 5], r6 = data[B + ROW_FRAME + 6];
    vec3 boxMin = r0.xyz, boxMax = r1.xyz, sunRay = r2.xyz, sunRgb = r3.rgb, sky = r4.rgb;
    int maxSteps = int(r0.w);
    float stepScale = r1.w, detail = r2.w, exposure = r3.w, extinction = r4.w;
    float anisotropy = r5.x, eddy = r5.y, shutter = r5.z, seed = r5.w, time = r6.x;

    // the box's slab test
    vec3 inv = 1.0 / vec3(abs(direction.x) < 1e-6 ? 1e-6 : direction.x, abs(direction.y) < 1e-6 ? 1e-6 : direction.y,
                          abs(direction.z) < 1e-6 ? 1e-6 : direction.z);
    vec3 tl = (boxMin - camera) * inv, th = (boxMax - camera) * inv;
    vec3 tn = min(tl, th), tf = max(tl, th);
    float enter = max(max(max(tn.x, tn.y), tn.z), 0.0), leave = min(min(tf.x, tf.y), tf.z);
    if (leave <= enter) return vec4(0.0);
    // the airframe (or another vehicle) in the way: the ray ends on it
    leave = min(leave, solid);
    if (leave <= enter) return vec4(0.0);
    for (int i = 0; i < FIELD_ROWS; ++i) F[i] = data[B + i]; // (the field's numbers, now the ray crosses its box)

    // the light: the droplets' forward peak, which cloud in the way blocks, and what has scattered many times
    float cosine = dot(direction, sunRay);
    vec3 sun = sunRgb * MULTIPLE_SCATTERING;
    vec3 sunSingle = sun * henyeyGreenstein(cosine, anisotropy) * (1.0 - ISOTROPIC_SHARE);
    vec3 sunMultiple = sun * (ISOTROPIC_SHARE / (4.0 * PI));
    // the air moves aft past the aircraft along the free stream, the patches with it, streaked by the shutter
    vec3 flow = vec3(f_cosAlpha, f_sinAlpha, f_flowZ);
    float drift = f_speed * time, smear = f_speed * shutter;

    vec3 light = vec3(0.0);
    float T = 1.0, t = enter, inside = 0.0;
    for (int i = 0; i < 256; ++i) {
        if (i >= maxSteps || t > leave) break;
        vec3 point = camera + direction * t;
        vec4 s = field(point, true);
        float footprint = t * PIXEL_STEP;
        if (s.y > 0.0) { // clear air: on to the nearest part that could fog
            t += max(s.y, max(footprint, 0.05));
            inside = 0.0;
            continue;
        }
        float here = min(max(s.z * stepScale, footprint), leave - t + 1e-3);
        if (inside < 0.5) { // in from clear air: a share of a step first, so the steps after are offset by it and an
            // edge leaves noise, not bands - taken where the ray comes in, so a layer thinner than a step (the vapour
            // over a wing) is not stepped over (fsim: aeronautic's skipped the share, and left holes in thin layers)
            inside = 1.0;
            here *= max(jitter, 0.05);
        }
        if (s.x > f_saturationDeficit) {
            vec4 water = waterAt(s.x);
            vec3 air = point - flow * drift + vec3(37.1, 11.3, 23.7) * seed;
            float downstream = dot(air, flow);
            vec3 across = air - flow * downstream;
            float patchy = detail > 0.5 ? clamp(patchiness(downstream, across.yz, eddy, smear), -1.0, 1.0) : 0.0;
            float grams = patchy < 0.0 ? mix(water.y, water.x, -patchy) : mix(water.y, water.z, patchy);
            // fsim: the droplets' onset. The air crosses the pocket in a few tens of milliseconds, and its droplets need
            // supersaturation past the dew point to nucleate and grow in that time: the cloud thickens over a range of
            // expansion beyond saturation (CONDENSATION_ONSET of the deficit saturation takes again), its equilibrium
            // water reached only inside it. Laid down at equilibrium the moment the air saturates, every cloud ended
            // opaque on the surface where it saturates - round the cone, a hard circle - where those photographed thin
            // out and their edge is see-through
            grams *= smoothstep(f_saturationDeficit, f_saturationDeficit * (1.0 + CONDENSATION_ONSET), s.x);
            float through = exp(-max(grams, 0.0) * extinction * here);
            // how much cloud the sunlight crossed to get here (the tips left out: a thin tube shades nothing)
            float shade = 0.0;
            if (detail > 0.5) {
                const vec2 samples[3] = vec2[3](vec2(0.4, 0.8), vec2(1.4, 1.6), vec2(4.0, 4.0));
                for (int k = 0; k < 3; ++k)
                    if (s.w < samples[k].x) {
                        vec4 toward = field(point + sunRay * samples[k].x, false);
                        if (toward.y <= 0.0) shade += waterAt(toward.x).y * extinction * samples[k].y;
                    }
            }
            vec3 source = sunSingle * exp(-shade) + sunMultiple * exp(-shade * MULTIPLE_SHADE) + sky;
            light += source * T * (1.0 - through);
            T *= through;
            if (first < 0.0) first = t;
        }
        if (T < OPAQUE) break;
        t += here;
    }
    float coverage = 1.0 - T;
    if (coverage < 1e-4 || first < 0.0) {
        first = -1.0;
        return vec4(0.0);
    }
    // through three.js's AgX tone mapping, as aeronautic's examples draw it (stage.tsx); its own light mapped and laid
    // over the scene, where theirs maps the whole frame
    return vec4(agx(light * exposure), coverage);
}
)";

// The compute pass: a ray per texel of an image at a share of the window's pixels (`scale` of them each way: 1, or 2
// when the vapour fills enough of the screen to cost - kHalfArea), through every aircraft's vapour, nearest first,
// composited front to back; where the cloud starts (m along the ray, 0 none).
const char* kCompute = R"(
layout(set = 0, binding = 1) uniform sampler2D solid; // the vehicles' depth (render::Viewer::depthPrepass)
// (region: the march's texels this frame, x0, y0, x1, y1; laid: the window's pixels the composite lays it on;
// rects: where on the window each vapour's box and then each exhaust's can be, x0, y0, x1, y1 px)
layout(set = 0, binding = 2) uniform Screen { vec4 screen; vec4 info; mat4 inverseProjection; vec4 depthTerms; vec4 region; vec4 laid;
                                              vec4 rects[SCREEN_RECTS]; };
layout(set = 0, binding = 3, rgba16f) uniform writeonly image2D colorOut;
layout(set = 0, binding = 4, r32f) uniform writeonly image2D frontOut;
layout(set = 0, binding = 7, rgba16f) uniform writeonly image2D edgeColorOut;
layout(set = 0, binding = 8, r32f) uniform writeonly image2D edgeFrontOut;
// the heat haze the exhausts give each texel and each edge pixel: where the scene behind is seen (pixels from where it
// is), where the hot gas starts (m; 0 none)
layout(set = 0, binding = 16, rgba16f) uniform writeonly image2D hazeOut;
layout(set = 0, binding = 17, rgba16f) uniform writeonly image2D edgeHazeOut;
// the edge pixels to march one by one: their count, then each (x, y << 16)
layout(std430, set = 0, binding = 10) buffer Edges { uint edges[]; };

// the distance along a pixel's ray to what is at a depth (reverse Z), and the ray
vec3 viewRay(vec2 pixel) {
    vec4 h = inverseProjection * vec4(pixel / screen.xy * 2.0 - 1.0, 1.0, 1.0);
    return normalize(h.xyz / h.w);
}
float distanceAt(vec2 pixel, float z) {
    vec4 h = inverseProjection * vec4(pixel / screen.xy * 2.0 - 1.0, z, 1.0);
    return length(h.xyz / h.w);
}
// the vehicles' depth at a window pixel (the depth image is the screen's size: where the pixel falls in it)
float solidAt(ivec2 p) {
    ivec2 size = textureSize(solid, 0);
    return texelFetch(solid, clamp(ivec2((vec2(p) + 0.5) / screen.xy * vec2(size)), ivec2(0), size - 1), 0).r;
}

// the exhausts marched with the vapour: per flame, view -> its frame, then (its row, detail, near steps' share, -)
layout(std430, set = 0, binding = 14) readonly buffer Flames { vec4 flameRows[]; };
const int MAX_LAYERS = 8;

// what each vapour and exhaust leaves along the ray, kept to be laid front to back by where each starts (past
// MAX_LAYERS, laid on in the order met)
void addLayer(inout vec4 layers[MAX_LAYERS], inout float starts[MAX_LAYERS], inout int n, inout vec4 rest, vec4 one, float start) {
    if (n < MAX_LAYERS) {
        layers[n] = one;
        starts[n] = start;
        ++n;
    } else {
        rest += (1.0 - rest.a) * one;
    }
}

// every aircraft's vapour and every engine's exhaust along the ray through a window point, front to back; where the
// first starts; the heat haze - how far the exhausts' hot gas moves the scene behind (pixels), where it starts
vec4 shade(vec2 pixel, float solidZ, out float front, out vec3 haze) {
    vec3 bentView = vec3(0.0);
    float hotFront = 1e30;
    vec3 dirView = viewRay(pixel);
    float solidM = solidZ > 0.0 ? distanceAt(pixel, solidZ) : 1e30;
    float jitter = gasDither(pixel);
    vec4 layers[MAX_LAYERS];
    float starts[MAX_LAYERS];
    int n = 0;
    vec4 rest = vec4(0.0);
    front = 1e30;
    int count = int(info.y);
    for (int k = 0; k < count; ++k) {
        vec4 where = rects[k]; // (the ray passes its box only inside the box's own outline on the window)
        if (pixel.x < where.x || pixel.y < where.y || pixel.x > where.z || pixel.y > where.w) continue;
        int base = k * ROWS_PER;
        mat4 fromView = mat4(data[base + ROW_FRAME + 7], data[base + ROW_FRAME + 8], data[base + ROW_FRAME + 9], data[base + ROW_FRAME + 10]);
        vec3 camera = fromView[3].xyz;
        vec3 direction = normalize(mat3(fromView) * dirView);
        float first;
        vec4 one = march(base, camera, direction, solidM, jitter, first);
        if (first >= 0.0) {
            addLayer(layers, starts, n, rest, one, first);
            front = min(front, first);
        }
    }
    int flames = int(screen.z);
    for (int f = 0; f < flames; ++f) {
        vec4 where = rects[VAPOUR_RECTS + f];
        if (pixel.x < where.x || pixel.y < where.y || pixel.x > where.z || pixel.y > where.w) continue;
        int row = f * FLAME_ROWS;
        mat4 fromView = mat4(flameRows[row], flameRows[row + 1], flameRows[row + 2], flameRows[row + 3]);
        vec4 how = flameRows[row + 4];
        vec3 local = mat3(fromView) * dirView;
        float perMetre = max(length(local), 1e-9); // (the flame's frame's units per metre)
        float first, hot;
        vec3 bend;
        vec4 one = marchFlame(int(how.x), fromView[3].xyz, local / perMetre, how.y, how.z, pixel, solidM * perMetre, first, bend, hot);
        if (first >= 0.0) {
            addLayer(layers, starts, n, rest, one, first / perMetre);
            front = min(front, first / perMetre);
        }
        if (hot >= 0.0) { // (its turn from the flame's frame into the view's)
            bentView += transpose(mat3(fromView)) * bend / perMetre;
            hotFront = min(hotFront, hot / perMetre);
        }
    }
    // front to back
    for (int i = 1; i < n; ++i)
        for (int j = i; j > 0 && starts[j] < starts[j - 1]; --j) {
            vec4 l = layers[j];
            layers[j] = layers[j - 1];
            layers[j - 1] = l;
            float s = starts[j];
            starts[j] = starts[j - 1];
            starts[j - 1] = s;
        }
    vec4 sum = vec4(0.0);
    for (int i = 0; i < n; ++i) sum += (1.0 - sum.a) * layers[i];
    sum += (1.0 - sum.a) * rest;
    if (front > 1e29) front = 0.0;
    // the turn as pixels: the scene behind is seen where the ray, turned, goes
    haze = vec3(0.0);
    if (hotFront < 1e29) {
        vec3 right = viewRay(pixel + vec2(1.0, 0.0)) - dirView, down = viewRay(pixel + vec2(0.0, 1.0)) - dirView;
        haze = vec3(dot(bentView, right) / max(dot(right, right), 1e-20), dot(bentView, down) / max(dot(down, down), 1e-20), hotFront);
    }
    return sum;
}

// one pixel marched by itself into the full-size images
void marchPixel(ivec2 p) {
    float front;
    vec3 haze;
    vec4 color = shade(vec2(p) + 0.5, solidAt(p), front, haze);
    imageStore(edgeColorOut, p, color);
    imageStore(edgeFrontOut, p, vec4(front));
    imageStore(edgeHazeOut, p, vec4(haze, 0.0));
}
)";

// The march's texels. An edge's pixels are listed for kEdges rather than marched here: a thread marching several
// rays while the others in its group wait for it cost the whole group as much.
const char* kMarchMain = R"(
layout(local_size_x = 8, local_size_y = 8) in;
void main() {
    // (the texels the boxes can reach, and those round them the composite reads: the rest are not marched, not read)
    ivec2 c = ivec2(gl_GlobalInvocationID.xy) + ivec2(region.xy);
    if (c.x >= int(region.z) || c.y >= int(region.w)) return;
    float scale = info.x;
    if (scale < 1.001) {
        float front;
        vec3 haze;
        vec4 color = shade(vec2(c) + 0.5, solidAt(c), front, haze);
        imageStore(colorOut, c, color);
        imageStore(frontOut, c, vec4(front));
        imageStore(hazeOut, c, vec4(haze, 0.0));
        return;
    }
    // a texel for the pixels whose centres fall in its square, scale each way - unless they lie on different surfaces
    // (an airframe's edge): those are marched one by one into the full-size images, and the texel says so (-1)
    // (on different surfaces: some on a vehicle and some not, or half a metre apart - vapour finer than that is not
    // drawn - so a slanting skin is not an edge)
    ivec2 lo = ivec2(ceil(vec2(c) * scale - 0.5)), hi = min(ivec2(ceil(vec2(c + 1) * scale - 0.5)), ivec2(screen.xy));
    ivec2 span = max(hi - lo, ivec2(1));
    float zMin = 1.0, zMax = 0.0, mMin = 1e30, mMax = 0.0;
    for (int k = 0; k < span.x * span.y; ++k) {
        ivec2 p = lo + ivec2(k % span.x, k / span.x);
        float z = solidAt(p);
        zMin = min(zMin, z);
        zMax = max(zMax, z);
        if (z > 0.0) {
            float m = distanceAt(vec2(p) + 0.5, z);
            mMin = min(mMin, m);
            mMax = max(mMax, m);
        }
    }
    bool edge = zMax > 0.0 && (zMin <= 0.0 || mMax - mMin > 0.5);
    if (edge) {
        uint n = uint(span.x * span.y), at = atomicAdd(edges[0], n);
        for (int k = 0; k < span.x * span.y; ++k) {
            ivec2 p = lo + ivec2(k % span.x, k / span.x);
            if (p.x >= int(screen.x) || p.y >= int(screen.y)) {
                if (at + uint(k) < uint(MAX_EDGES)) edges[1u + at + uint(k)] = 0xffffffffu;
                continue;
            }
            if (at + uint(k) < uint(MAX_EDGES)) edges[1u + at + uint(k)] = uint(p.x) | (uint(p.y) << 16);
            else marchPixel(p); // (the list full)
        }
        imageStore(colorOut, c, vec4(0.0));
        imageStore(frontOut, c, vec4(-1.0));
        return;
    }
    float front;
    vec3 haze;
    vec4 color = shade((vec2(c) + 0.5) * scale, zMax, front, haze);
    imageStore(colorOut, c, color);
    imageStore(frontOut, c, vec4(front));
    imageStore(hazeOut, c, vec4(haze, 0.0));
}
)";

// The edges' pixels, a thread each
const char* kEdgesMain = R"(
layout(local_size_x = 64) in;
void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i >= min(edges[0], uint(MAX_EDGES))) return;
    uint packed = edges[1u + i];
    if (packed == 0xffffffffu) return;
    marchPixel(ivec2(int(packed & 0xffffu), int(packed >> 16)));
}
)";

// The list emptied for the frame
const char* kEdgesReset = R"(#version 450
layout(local_size_x = 1) in;
layout(std430, set = 0, binding = 10) buffer Edges { uint edges[]; };
void main() { edges[0] = 0u; }
)";

// The composite: the vapour image laid on the window's pixels - each from the texels round it (bilinear), those whose
// cloud starts behind a vehicle covering this pixel left out - and its depth where it starts, so what stands in
// front of it (terrain, an airframe) hides it.
const char* kCompositeVertex = R"(
out gl_PerVertex { vec4 gl_Position; };
void main() {
    vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";
const char* kCompositeFragment = R"(
layout(set = 0, binding = 1) uniform sampler2D solid;
// (region: the march's texels this frame, x0, y0, x1, y1; laid: the window's pixels the composite lays it on;
// rects: where on the window each vapour's box and then each exhaust's can be, x0, y0, x1, y1 px)
layout(set = 0, binding = 2) uniform Screen { vec4 screen; vec4 info; mat4 inverseProjection; vec4 depthTerms; vec4 region; vec4 laid;
                                              vec4 rects[SCREEN_RECTS]; };
layout(set = 0, binding = 5) uniform sampler2D colorTex;
layout(set = 0, binding = 6) uniform sampler2D frontTex;
layout(set = 0, binding = 7) uniform sampler2D edgeColorTex;
layout(set = 0, binding = 8) uniform sampler2D edgeFrontTex;
layout(set = 0, binding = 16) uniform sampler2D hazeTex;
layout(set = 0, binding = 17) uniform sampler2D edgeHazeTex;
layout(set = 0, binding = 18) uniform sampler2D sceneTex; // the scene as drawn (render::Viewer::sceneColour)
layout(location = 0) out vec4 outColor;

void main() {
    vec2 pixel = gl_FragCoord.xy;
    // (beyond what the boxes can reach nothing was marched: nothing to lay on)
    if (pixel.x < laid.x || pixel.y < laid.y || pixel.x >= laid.z || pixel.y >= laid.w) discard;
    vec4 h = inverseProjection * vec4(pixel / screen.xy * 2.0 - 1.0, 1.0, 1.0);
    vec3 ray = normalize(h.xyz / h.w);
    float scale = info.x;
    ivec2 used = ivec2(info.zw);
    // a vehicle on this pixel: how far
    ivec2 size = textureSize(solid, 0);
    float z = texelFetch(solid, clamp(ivec2(pixel / screen.xy * vec2(size)), ivec2(0), size - 1), 0).r;
    float solidM = 1e30;
    if (z > 0.0) {
        vec4 hs = inverseProjection * vec4(pixel / screen.xy * 2.0 - 1.0, z, 1.0);
        solidM = length(hs.xyz / hs.w);
    }
    vec4 sum = vec4(0.0);
    float front = 1e30;
    vec2 haze = vec2(0.0);
    float hot = 1e30;
    ivec2 own = clamp(ivec2(floor(pixel / scale)), ivec2(0), used - 1);
    if (scale >= 1.001 && texelFetch(frontTex, own, 0).r < 0.0) {
        // an edge: this pixel was marched on its own - with those round it marched so on the same surface (within 5 %
        // of its distance; nothing behind it), weighted as the texels are, the march's dither filtered out
        const float EDGE_FILTER = 0.75;
        float weights = 0.0;
        for (int k = 0; k < 9; ++k) {
            ivec2 o = ivec2(k % 3 - 1, k / 3 - 1);
            ivec2 p = clamp(ivec2(pixel) + o, ivec2(0), ivec2(screen.xy) - 1);
            if (texelFetch(frontTex, clamp(ivec2(floor((vec2(p) + 0.5) / scale)), ivec2(0), used - 1), 0).r >= 0.0) continue;
            float zn = texelFetch(solid, clamp(ivec2((vec2(p) + 0.5) / screen.xy * vec2(size)), ivec2(0), size - 1), 0).r;
            float mn = 1e30;
            if (zn > 0.0) {
                vec4 hn = inverseProjection * vec4((vec2(p) + 0.5) / screen.xy * 2.0 - 1.0, zn, 1.0);
                mn = length(hn.xyz / hn.w);
            }
            if ((mn > 1e29) != (solidM > 1e29) || (solidM < 1e29 && abs(mn - solidM) > 0.05 * solidM)) continue;
            float w = exp(-0.5 * dot(vec2(o), vec2(o)) / (EDGE_FILTER * EDGE_FILTER));
            weights += w;
            float start = texelFetch(edgeFrontTex, p, 0).r;
            if (start <= 0.0) continue;
            sum += w * texelFetch(edgeColorTex, p, 0);
            front = min(front, start);
        }
        sum /= max(weights, 1e-6);
        vec4 hz = texelFetch(edgeHazeTex, ivec2(pixel), 0);
        if (hz.z > 0.0 && hz.z < solidM) {
            haze = hz.xy;
            hot = hz.z;
        }
    } else {
        // the texels round it, weighted by a Gaussian of their distance (sigma GAS_FILTER texels: the march's dither
        // - a pattern two or three pixels across - filtered out, what is wider kept), over those that stand for one
        // surface (not edges), those whose cloud starts behind a vehicle covering this pixel left out
        const float GAS_FILTER = 0.75;
        vec2 q = pixel / scale - 0.5;
        ivec2 b = ivec2(floor(q)) - 1;
        float weights = 0.0;
        for (int k = 0; k < 16; ++k) {
            ivec2 t = clamp(b + ivec2(k & 3, k >> 2), ivec2(0), used - 1);
            vec2 d = vec2(b + ivec2(k & 3, k >> 2)) - q;
            float w = exp(-0.5 * dot(d, d) / (GAS_FILTER * GAS_FILTER));
            float start = texelFetch(frontTex, t, 0).r;
            if (start < 0.0) continue; // an edge texel: nothing for its neighbours
            weights += w;
            vec4 hz = texelFetch(hazeTex, t, 0);
            if (hz.z > 0.0 && hz.z < solidM) {
                haze += w * hz.xy;
                hot = min(hot, hz.z);
            }
            if (start == 0.0 || start >= solidM) continue; // no cloud, or it starts behind the vehicle here
            sum += w * texelFetch(colorTex, t, 0);
            front = min(front, start);
        }
        sum /= max(weights, 1e-6);
        haze /= max(weights, 1e-6);
    }
    // (light without cover counts: up a dry nozzle the clear gas hides nothing, the turbine and the pipe glow through it)
    bool gas = (sum.a >= 1e-4 || max(max(sum.r, sum.g), sum.b) >= 1.0 / 1024.0) && front < 1e29;
    bool hazy = hot < 1e29 && dot(haze, haze) > 1e-4;
    if (!gas && !hazy) discard;
    if (hazy) {
        // the heat haze: the scene behind seen where the hot gas turns the ray, the gas over it
        vec3 behind = textureLod(sceneTex, (pixel + haze) / vec2(textureSize(sceneTex, 0)), 0.0).rgb;
        sum = vec4((gas ? sum.rgb : vec3(0.0)) + (1.0 - (gas ? sum.a : 0.0)) * behind, 1.0);
        front = gas ? min(front, hot) : hot;
    }
    float zView = ray.z * front;
    gl_FragDepth = clamp((depthTerms.x * zView + depthTerms.y) / (depthTerms.z * zView + depthTerms.w), 0.0, 1.0);
    outColor = sum;
}
)";

std::string header() {
    std::string s = "#version 450\nlayout(push_constant) uniform PushConstants { mat4 projection; mat4 modelView; } pc;\n"
                    "layout(std430, set = 0, binding = 0) readonly buffer Data { vec4 data[]; };\n";
    s += "#define MAX_EDGES " + std::to_string(kMaxEdges) + "\nconst int FLAME_ROWS = " + std::to_string(kFlameRows) + ";\n";
    s += "#define SCREEN_RECTS " + std::to_string(kInstances + kMaxFlames) + "\n#define VAPOUR_RECTS " + std::to_string(kInstances) + "\n";
    s += "const int ROWS_PER = " + std::to_string(kRowsPer) + ", ROW_FRAME = " + std::to_string(kRowFrame) +
         ", ROW_SHAPE = " + std::to_string(kRowShape) + ", ROW_LOADING = " + std::to_string(kRowLoading) +
         ", ROW_PATH = " + std::to_string(kRowPath) + ", ROW_TRAILS = " + std::to_string(kRowTrails) +
         ", ROW_TABLE = " + std::to_string(kRowTable) + ";\n#define FIELD_ROWS " + std::to_string(kFieldRows) + "\n";
    std::size_t k = 0;
    const char* xyzw = "xyzw";
#define FSIM_DEFINE(name)                                                                                          \
    s += std::string("#define f_") + #name + " F[" + std::to_string(k / 4) + "]." + xyzw[k % 4] + "\n"; \
    ++k;
    FSIM_VAPOUR_FIELD(FSIM_DEFINE)
#undef FSIM_DEFINE
    return s;
}

void pack(const Field& f, vsg::vec4Array& data, std::uint32_t base) {
    std::array<float, kFieldRows * 4> v{};
    std::size_t k = 0;
#define FSIM_PACK(name) v[k++] = static_cast<float>(f.name);
    FSIM_VAPOUR_FIELD(FSIM_PACK)
#undef FSIM_PACK
    for (std::size_t r = 0; r < kFieldRows; ++r) data.set(base + static_cast<std::uint32_t>(r), vsg::vec4(v[4 * r], v[4 * r + 1], v[4 * r + 2], v[4 * r + 3]));
}

} // namespace

WingVapour::WingVapour(std::size_t slots, vsg::ref_ptr<const vsg::Options> options) {
    (void)options;
    root_ = vsg::Group::create();
    compute_ = vsg::Group::create();
    slots_.resize(std::max<std::size_t>(slots, 1));
}

WingVapour::~WingVapour() = default;

bool WingVapour::build(vsg::ref_ptr<vsg::ImageInfo> depth, std::uint32_t width, std::uint32_t height, const VehicleVisuals::Compiler& compiler,
                       const Exhaust* exhaust, vsg::ref_ptr<vsg::ImageInfo> blueNoise, vsg::ref_ptr<vsg::ImageInfo> sceneColour) {
    if (state_) return true;
    const std::string gas = header() + kDither + kField + Exhaust::marchSource() + kCompute;
    auto march = vsg::ShaderStage::create(VK_SHADER_STAGE_COMPUTE_BIT, "main", gas + kMarchMain);
    auto edgeMarch = vsg::ShaderStage::create(VK_SHADER_STAGE_COMPUTE_BIT, "main", gas + kEdgesMain);
    auto edgeReset = vsg::ShaderStage::create(VK_SHADER_STAGE_COMPUTE_BIT, "main", std::string(kEdgesReset));
    auto vertex = vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", std::string("#version 450\n") + kCompositeVertex);
    auto fragment = vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", std::string("#version 450\n#define SCREEN_RECTS ") +
                                                                                      std::to_string(kInstances + kMaxFlames) + "\n" + kCompositeFragment);
    auto shaderCompiler = vsg::ShaderCompiler::create();
    vsg::ShaderStages one{march}, two{vertex, fragment}, three{edgeMarch}, four{edgeReset};
    if (!shaderCompiler->supported() || !shaderCompiler->compile(one) || !shaderCompiler->compile(two) || !shaderCompiler->compile(three) ||
        !shaderCompiler->compile(four)) {
        LOG_WARN("world") << "wing vapour shaders did not build; no vapour drawn";
        return false;
    }
    data_ = vsg::vec4Array::create(static_cast<std::uint32_t>(kInstances * kRowsPer), vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    data_->properties.dataVariance = vsg::DYNAMIC_DATA;
    auto edges = vsg::uintArray::create(1 + kMaxEdges, 0u); // (written by the GPU)
    flames_ = vsg::vec4Array::create(kMaxFlames * kFlameRows, vsg::vec4(0.0f, 0.0f, 0.0f, 0.0f));
    flames_->properties.dataVariance = vsg::DYNAMIC_DATA;
    // the scene behind the heat haze (none: a black one, never read - no haze is drawn without it)
    if (!sceneColour) {
        auto black = vsg::ubvec4Array2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R8G8B8A8_UNORM});
        black->set(0, 0, vsg::ubvec4(0, 0, 0, 255));
        sceneColour = vsg::ImageInfo::create(vsg::Sampler::create(), black, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
    // the dither (none: a flat one)
    if (!blueNoise) {
        auto flat = vsg::ubyteArray3D::create(1, 1, 1, vsg::Data::Properties{VK_FORMAT_R8_UNORM});
        flat->set(0, 0, 0, 128);
        blueNoise = vsg::ImageInfo::create(vsg::Sampler::create(), flat, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
    // the exhausts' numbers (none: a stand-in, never read)
    exhaust_ = exhaust;
    vsg::ref_ptr<vsg::Data> exhaustRows = exhaust ? vsg::ref_ptr<vsg::Data>(exhaust->rows()) : vsg::ref_ptr<vsg::Data>(vsg::vec4Array::create(Exhaust::kRows));
    vsg::ref_ptr<vsg::Data> blackbody = exhaust ? vsg::ref_ptr<vsg::Data>(exhaust->blackbodyTable()) : vsg::ref_ptr<vsg::Data>(vsg::vec4Array::create(256));
    vsg::ref_ptr<vsg::Data> exhaustFrame = exhaust ? vsg::ref_ptr<vsg::Data>(exhaust->frame()) : vsg::ref_ptr<vsg::Data>(vsg::vec4Value::create());
    // screen, info, inverse projection, depth terms; the march's region, the composite's, each box's outline
    screen_ = vsg::vec4Array::create(static_cast<std::uint32_t>(kScreenRects + kInstances + kMaxFlames), vsg::vec4(1.0f, 1.0f, 0.0f, 0.0f));
    screen_->properties.dataVariance = vsg::DYNAMIC_DATA;
    if (!depth) {
        auto sampler = vsg::Sampler::create();
        auto nothing = vsg::floatArray2D::create(1, 1, vsg::Data::Properties{VK_FORMAT_R32_SFLOAT});
        nothing->set(0, 0, 0.0f);
        depth = vsg::ImageInfo::create(sampler, nothing);
    }
    // the images: the screen's size (a window maximised later uses a part of them)
    unsigned sw = 0, sh = 0;
    platform::screenSize(sw, sh);
    alloc_ = vsg::uivec2(std::max({width, sw, 1u}), std::max({height, sh, 1u}));
    const auto image = [&](VkFormat format) {
        auto i = vsg::Image::create();
        i->imageType = VK_IMAGE_TYPE_2D;
        i->format = format;
        i->extent = VkExtent3D{alloc_.x, alloc_.y, 1};
        i->mipLevels = 1;
        i->arrayLayers = 1;
        i->samples = VK_SAMPLE_COUNT_1_BIT;
        i->tiling = VK_IMAGE_TILING_OPTIMAL;
        i->usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        return i;
    };
    auto color = image(VK_FORMAT_R16G16B16A16_SFLOAT), front = image(VK_FORMAT_R32_SFLOAT);
    auto edgeColor = image(VK_FORMAT_R16G16B16A16_SFLOAT), edgeFront = image(VK_FORMAT_R32_SFLOAT);
    auto haze = image(VK_FORMAT_R16G16B16A16_SFLOAT), edgeHaze = image(VK_FORMAT_R16G16B16A16_SFLOAT);
    const auto view = [](vsg::ref_ptr<vsg::Image> i) { return vsg::ImageView::create(i, VK_IMAGE_ASPECT_COLOR_BIT); };
    auto nearest = vsg::Sampler::create();
    nearest->minFilter = nearest->magFilter = VK_FILTER_NEAREST;
    nearest->addressModeU = nearest->addressModeV = nearest->addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    auto colorView = view(color), frontView = view(front), edgeColorView = view(edgeColor), edgeFrontView = view(edgeFront);
    auto hazeView = view(haze), edgeHazeView = view(edgeHaze);
    const auto storage = [](vsg::ref_ptr<vsg::ImageView> v, std::uint32_t binding) {
        return vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, v, VK_IMAGE_LAYOUT_GENERAL), binding, 0,
                                            VK_DESCRIPTOR_TYPE_STORAGE_IMAGE);
    };

    // the march
    const VkShaderStageFlags cs = VK_SHADER_STAGE_COMPUTE_BIT, fs = VK_SHADER_STAGE_FRAGMENT_BIT;
    auto marchLayout = vsg::PipelineLayout::create(
        vsg::DescriptorSetLayouts{vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr},
            {3, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr},
            {4, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr},
            {7, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr},
            {8, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr},
            {10, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {11, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {12, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {13, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, cs, nullptr},
            {14, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, cs, nullptr},
            {15, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, cs, nullptr},
            {16, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr},
            {17, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, cs, nullptr}})},
        vsg::PushConstantRanges{});
    auto marchSet = vsg::DescriptorSet::create(
        marchLayout->setLayouts[0],
        vsg::Descriptors{vsg::DescriptorBuffer::create(data_, 0, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                         vsg::DescriptorImage::create(depth, 1, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorBuffer::create(screen_, 2, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, colorView, VK_IMAGE_LAYOUT_GENERAL), 3, 0,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_IMAGE),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(vsg::ref_ptr<vsg::Sampler>{}, frontView, VK_IMAGE_LAYOUT_GENERAL), 4, 0,
                                                      VK_DESCRIPTOR_TYPE_STORAGE_IMAGE),
                         storage(edgeColorView, 7), storage(edgeFrontView, 8),
                         vsg::DescriptorBuffer::create(edges, 10, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                         vsg::DescriptorBuffer::create(exhaustRows, 11, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                         vsg::DescriptorBuffer::create(blackbody, 12, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                         vsg::DescriptorBuffer::create(exhaustFrame, 13, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                         vsg::DescriptorBuffer::create(flames_, 14, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER),
                         vsg::DescriptorImage::create(blueNoise, 15, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         storage(hazeView, 16), storage(edgeHazeView, 17)});
    auto pass = vsg::Group::create();
    {
        // the last frame's composite done with the images, the vehicles' depth written; the images' old contents not kept
        auto begin = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
                                                  VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
        begin->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT));
        for (const auto& i : {color, front, edgeColor, edgeFront, haze, edgeHaze})
            begin->imageMemoryBarriers.push_back(vsg::ImageMemoryBarrier::create(
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_WRITE_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL, VK_QUEUE_FAMILY_IGNORED,
                VK_QUEUE_FAMILY_IGNORED, i, VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1}));
        pass->addChild(begin);
        const auto computeBarrier = [] {
            auto b = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0);
            b->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT,
                                                                   VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT));
            return b;
        };
        const auto stage = [&](vsg::ref_ptr<vsg::ShaderStage> shader, vsg::ref_ptr<vsg::Dispatch> d) {
            auto g = vsg::StateGroup::create();
            g->add(vsg::BindComputePipeline::create(vsg::ComputePipeline::create(marchLayout, shader)));
            g->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_COMPUTE, marchLayout, 0, marchSet));
            g->addChild(d);
            return g;
        };
        // the edge list emptied; the texels marched, the edges listed; the edges marched
        pass->addChild(stage(edgeReset, vsg::Dispatch::create(1, 1, 1)));
        pass->addChild(computeBarrier());
        dispatch_ = vsg::Dispatch::create(1, 1, 1);
        pass->addChild(stage(march, dispatch_));
        pass->addChild(computeBarrier());
        pass->addChild(stage(edgeMarch, vsg::Dispatch::create((kMaxEdges + 63) / 64, 1, 1)));
        auto done = vsg::PipelineBarrier::create(VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0);
        done->memoryBarriers.push_back(vsg::MemoryBarrier::create(VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT));
        pass->addChild(done);
    }
    computeSwitch_ = vsg::Switch::create();
    computeSwitch_->addChild(true, pass);

    // the composite: premultiplied over, depth tested where the cloud starts (not written: vapour hides nothing)
    auto drawLayout = vsg::PipelineLayout::create(
        vsg::DescriptorSetLayouts{vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
            {1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {2, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, fs, nullptr},
            {5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {7, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {8, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {16, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr},
            {18, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, fs, nullptr}})},
        vsg::PushConstantRanges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}});
    auto drawSet = vsg::DescriptorSet::create(
        drawLayout->setLayouts[0],
        vsg::Descriptors{vsg::DescriptorImage::create(depth, 1, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorBuffer::create(screen_, 2, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, colorView, VK_IMAGE_LAYOUT_GENERAL), 5, 0,
                                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, frontView, VK_IMAGE_LAYOUT_GENERAL), 6, 0,
                                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, edgeColorView, VK_IMAGE_LAYOUT_GENERAL), 7, 0,
                                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, edgeFrontView, VK_IMAGE_LAYOUT_GENERAL), 8, 0,
                                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, hazeView, VK_IMAGE_LAYOUT_GENERAL), 16, 0,
                                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorImage::create(vsg::ImageInfo::create(nearest, edgeHazeView, VK_IMAGE_LAYOUT_GENERAL), 17, 0,
                                                      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER),
                         vsg::DescriptorImage::create(sceneColour, 18, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER)});
    auto raster = vsg::RasterizationState::create();
    raster->cullMode = VK_CULL_MODE_NONE;
    auto depthState = vsg::DepthStencilState::create();
    depthState->depthWriteEnable = VK_FALSE;
    depthState->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;
    auto blend = vsg::ColorBlendState::create();
    blend->attachments = {VkPipelineColorBlendAttachmentState{
        VK_TRUE, VK_BLEND_FACTOR_ONE, VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD, VK_BLEND_FACTOR_ONE,
        VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA, VK_BLEND_OP_ADD,
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT}};
    vsg::GraphicsPipelineStates states{vsg::VertexInputState::create(), vsg::InputAssemblyState::create(), raster,
                                       vsg::MultisampleState::create(), blend, depthState};
    auto state = vsg::StateGroup::create();
    state->add(vsg::BindGraphicsPipeline::create(vsg::GraphicsPipeline::create(drawLayout, two, states)));
    state->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, drawLayout, 0, drawSet));
    state->addChild(vsg::Draw::create(3, 1, 0, 0));
    drawSwitch_ = vsg::Switch::create();
    drawSwitch_->addChild(true, state);
    if (compiler && (!compiler(computeSwitch_) || !compiler(drawSwitch_))) {
        LOG_WARN("world") << "could not compile the wing vapour";
        return false;
    }
    if (compiler) { // (without one it is compiled with the scene: left on until then - views switched off are not compiled for)
        computeSwitch_->setAllChildren(false);
        drawSwitch_->setAllChildren(false);
    }
    compute_->addChild(computeSwitch_);
    state_ = state;
    root_->addChild(drawSwitch_);
    setScreen(width, height);
    return true;
}

void WingVapour::setScreen(std::uint32_t width, std::uint32_t height) {
    window_ = vsg::vec2(static_cast<float>(std::max<std::uint32_t>(width, 1)), static_cast<float>(std::max<std::uint32_t>(height, 1)));
}

void WingVapour::compiled() {
    if (computeSwitch_) computeSwitch_->setAllChildren(false);
    if (drawSwitch_) drawSwitch_->setAllChildren(false);
}

void WingVapour::setVisible(bool on) {
    visible_ = on;
    if (!on) {
        wantsSolid_ = false;
        if (computeSwitch_) computeSwitch_->setAllChildren(false);
        if (drawSwitch_) drawSwitch_->setAllChildren(false);
        for (auto& s : slots_)
            if (s.history) s.history->reset();
    }
}

void WingVapour::setAir(double seaLevelK, double humidity) noexcept {
    seaLevelK_ = std::isfinite(seaLevelK) ? seaLevelK : 288.15;
    humidity_ = std::isfinite(humidity) ? clampd(humidity, 0.0, 1.0) : 0.0;
}

void WingVapour::update(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive, const VehicleVisuals& visuals,
                        const vsg::dmat4& view, const vsg::dmat4& projection, const vsg::dvec3& eye, const vsg::dvec3& sun,
                        float daylight) {
    if (!state_ || !visible_) return;
    const std::size_t n = std::min({states.size(), alive.size(), visuals.count(), slots_.size()});
    // nearest first: they get the vapours drawn
    std::vector<std::pair<double, std::size_t>> order;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& st = states[i];
        if (!alive[i] || !visuals.visible(i) || st.diverged || !visuals.shape(i).valid) {
            if (slots_[i].history) slots_[i].history->reset();
            continue;
        }
        const double d = vsg::length(positionEcef(st) - eye);
        if (d < kRange) order.emplace_back(d, i);
    }
    std::sort(order.begin(), order.end());

    const vsg::dvec3 sunDir = vsg::length(sun) > 0.0 ? vsg::normalize(sun) : vsg::dvec3(0.0, 0.0, 1.0);
    const double day = clampd(daylight, 0.0, 1.0);
    std::size_t shown = 0;
    double coveredSum = 0.0;
    // where on the window each vapour's box and each exhaust's can be (clip space: left, bottom, right, top): a ray
    // meets a box only inside the box's own outline, so the march looks for each only there - and not at all where no
    // outline is
    std::array<vsg::dvec4, kInstances + kMaxFlames> outlines;
    for (const auto& [distance, i] : order) {
        if (shown >= kInstances) break;
        const auto& st = states[i];
        Slot& slot = slots_[i];
        const auto& shape = visuals.shape(i);
        auto key = modelKey(shape);
        if (!slot.model || slot.key != key) {
            slot.model = makeModel(shape);
            slot.key = std::move(key);
            slot.history.reset();
        }
        if (!slot.history) slot.history = std::make_unique<History>();
        const Model& m = *slot.model;

        // the canonical frame in ECEF: x aft, y up, z right
        const vsg::dvec3 fwd = bodyAxisEcef(st, 0), right = bodyAxisEcef(st, 1), down = bodyAxisEcef(st, 2);
        const vsg::dvec3 cx = -fwd, cy = -down, cz = right;

        // the air: the standard atmosphere from the world's sea-level temperature, and its humidity
        const double altitude = finite(st.altitudeMslM);
        const double rh = humidity_ > 0.0 ? humidity_ : defaultHumidity(altitude);
        const double rounded = std::round(altitude / 25.0) * 25.0;
        const Air air = moistAir(rounded, rh, seaLevelK_ - 288.15);

        // the roll rate about the flight path, eased (or the simulation's own: its body rate)
        Flight flight;
        flight.airspeedMs = std::max(finite(st.airspeedTrueMs), 0.0);
        flight.alphaRad = finite(st.alphaRad) + 0.5 * std::max(finite(st.flapsRad), 0.0);
        flight.betaRad = finite(st.betaRad);
        const double now = finite(st.simTime);
        const double dt = slot.lastTime >= 0.0 ? now - slot.lastTime : 0.0;
        if (dt < -0.5) slot.history->reset(); // a reset or a replay's seek
        const double follow = dt > 0.0 ? 1.0 - std::exp(-dt / kRollResponseS) : 0.0;
        slot.rollRate += (finite(st.angularRateBodyRadS[0]) - slot.rollRate) * follow;
        flight.rollRateRadS = slot.rollRate;
        slot.lastTime = now;

        if (flight.airspeedMs < 30.0) {
            slot.history->reset();
            continue;
        }
        // one more step of flight through the air, then the trails laid back along it
        const vsg::dvec3 flow(std::cos(flight.alphaRad) * std::cos(flight.betaRad), std::sin(flight.alphaRad) * std::cos(flight.betaRad),
                              -std::sin(flight.betaRad));
        slot.history->advance(std::clamp(dt, 0.0, 2.5), History::fromMatrix(st.rotationBodyToEcef), flight.airspeedMs, flow, slot.keep);
        State s = vapourState(m, flight, air, [&](const Layout& l) { return slot.history->paths(l); });
        slot.keep = std::max({s.g.f.tipReach, s.g.f.secondReach, 1.0});
        if (!s.visible) continue;

        // the box rides on the canonical frame: canonical -> ECEF
        vsg::dmat4 c;
        c(0, 0) = cx.x; c(0, 1) = cx.y; c(0, 2) = cx.z; c(0, 3) = 0.0;
        c(1, 0) = cy.x; c(1, 1) = cy.y; c(1, 2) = cy.z; c(1, 3) = 0.0;
        c(2, 0) = cz.x; c(2, 1) = cz.y; c(2, 2) = cz.z; c(2, 3) = 0.0;
        const vsg::dvec3 at = positionEcef(st);
        c(3, 0) = at.x; c(3, 1) = at.y; c(3, 2) = at.z; c(3, 3) = 1.0;

        // the budget (wing-vapor-core.ts budget()): the box's eight corners on screen and the rectangle round them,
        // clipped to it - all of it when the camera is in or beside the box
        const vsg::dmat4 toClip = projection * view * c;
        double sLeft = 1e30, sRight = -1e30, sBottom = 1e30, sTop = -1e30;
        bool behind = false;
        for (int k = 0; k < 8 && !behind; ++k) {
            const vsg::dvec4 q = toClip * vsg::dvec4((k & 1) ? s.hi.x : s.lo.x, (k & 2) ? s.hi.y : s.lo.y, (k & 4) ? s.hi.z : s.lo.z, 1.0);
            if (q.w <= 0.0) {
                behind = true;
                break;
            }
            sLeft = std::min(sLeft, q.x / q.w);
            sRight = std::max(sRight, q.x / q.w);
            sBottom = std::min(sBottom, q.y / q.w);
            sTop = std::max(sTop, q.y / q.w);
        }
        const double width = behind ? 2.0 : std::max(std::min(sRight, 1.0) - std::max(sLeft, -1.0), 0.0);
        const double height = behind ? 2.0 : std::max(std::min(sTop, 1.0) - std::max(sBottom, -1.0), 0.0);
        const double covered = width * height / 4.0;
        const double stepScale = 1.0;
        outlines[shown] = behind ? vsg::dvec4(-1.0, -1.0, 1.0, 1.0) : vsg::dvec4(sLeft, sBottom, sRight, sTop);
        const double detail = height / 2.0 > kDetailHeight ? 1.0 : 0.0;

        // the light (types.ts default_vapor_look by day; apps/docs stage.tsx's NIGHT look by night): the sun's colour
        // and intensity, the sky's, mixed with the daylight
        const vsg::dvec3 sunC(vsg::dot(sunDir, cx), vsg::dot(sunDir, cy), vsg::dot(sunDir, cz));
        const vsg::dvec3 sunRgb = vsg::dvec3(1.0, 0.96, 0.9) * (3.0 * day) + vsg::dvec3(0.7, 0.78, 1.0) * (0.6 * (1.0 - day));
        const vsg::dvec3 sky = vsg::dvec3(0.55, 0.68, 0.9) * day + vsg::dvec3(0.08, 0.1, 0.18) * (0.15 * (1.0 - day));

        const auto base = static_cast<std::uint32_t>(shown * kRowsPer);
        const auto row = [&](int first, int k) { return base + static_cast<std::uint32_t>(first + k); };
        pack(s.g.f, *data_, base + static_cast<std::uint32_t>(kRowField));
        const auto F = [](double v) { return static_cast<float>(v); };
        data_->set(row(kRowFrame, 0), vsg::vec4(F(s.lo.x), F(s.lo.y), F(s.lo.z), F(kMaxSteps)));
        data_->set(row(kRowFrame, 1), vsg::vec4(F(s.hi.x), F(s.hi.y), F(s.hi.z), F(stepScale)));
        data_->set(row(kRowFrame, 2), vsg::vec4(F(sunC.x), F(sunC.y), F(sunC.z), F(detail)));
        data_->set(row(kRowFrame, 3), vsg::vec4(F(sunRgb.x), F(sunRgb.y), F(sunRgb.z), 1.0f));
        // a gram per cubic metre of micron droplets: 3 w / 2 rho r
        data_->set(row(kRowFrame, 4), vsg::vec4(F(sky.x), F(sky.y), F(sky.z), F(3e-3 / (2.0 * 1000.0 * kDropletRadiusM))));
        data_->set(row(kRowFrame, 5), vsg::vec4(F(kAnisotropy), F(kEddyM), F(kShutterS), F(static_cast<double>(i % 17) * 0.618)));
        data_->set(row(kRowFrame, 6), vsg::vec4(F(std::fmod(std::max(now, 0.0), 120.0)), 0.0f, 0.0f, 0.0f));
        for (int k = 0; k < kStations; ++k) {
            data_->set(row(kRowShape, k), vsg::vec4(F(m.leading[k]), F(m.chord[k]), F(m.mid[k]), F(m.thickness[k])));
            data_->set(row(kRowLoading, k), vsg::vec4(F(m.loading[k]), 0.0f, 0.0f, 0.0f));
            data_->set(row(kRowPath, k), vsg::vec4(F(m.pathSpan[k]), F(m.pathSurface[k]), F(m.pathReach[k]), 0.0f));
        }
        const std::array<const Points*, 4> trails{&s.g.trails.positive, &s.g.trails.negative, &s.g.second.positive, &s.g.second.negative};
        for (int r = 0; r < 4; ++r)
            for (int k = 0; k < kTrailPoints; ++k) {
                const auto& p = (*trails[static_cast<std::size_t>(r)])[k];
                data_->set(row(kRowTrails, r * kTrailPoints + k), vsg::vec4(F(p.x), F(p.y), F(p.z), 0.0f));
            }
        // the condensation table: the same day asks the same question every frame
        if (slot.table.empty() || slot.tableAir != vsg::dvec3(air.temperatureK, air.pressurePa, air.mixingRatio)) {
            slot.tableAir = vsg::dvec3(air.temperatureK, air.pressurePa, air.mixingRatio);
            slot.table.resize(kTexels);
            for (int k = 0; k < kTexels; ++k) {
                const double ratio = kMinRatio + (1.0 - kMinRatio) * k / (kTexels - 1);
                double T, w;
                condensate(ratio, air, 1.0, T, w);
                slot.table[static_cast<std::size_t>(k)] = vsg::vec4(F(1000.0 * liquidWater(ratio, air, 1.0 - kHumiditySpread)), F(1000.0 * w),
                                                                    F(1000.0 * liquidWater(ratio, air, moistest(air, kHumiditySpread))),
                                                                    F(air.temperatureK - T));
            }
        }
        for (int k = 0; k < kTexels; ++k) data_->set(row(kRowTable, k), slot.table[static_cast<std::size_t>(k)]);
        // view -> the box's frame (canonical), for the rays
        const vsg::dmat4 fromView = vsg::inverse(view * c);
        for (std::size_t col = 0; col < 4; ++col)
            data_->set(row(kRowFrame, 7 + static_cast<int>(col)), vsg::vec4(F(fromView[col][0]), F(fromView[col][1]), F(fromView[col][2]), F(fromView[col][3])));
        coveredSum += covered;
        ++shown;
    }
    // the exhausts showing, the nearest first: each one's frame from the view, its detail and its share of the near
    // steps (as the exhaust's own box worked them out), and how much of the screen it covers (for the budget)
    std::uint32_t flameCount = 0;
    if (exhaust_ && exhaust_->rows()) {
        const auto& rows = *exhaust_->rows();
        const double screenScale = 2.0 / std::max(std::abs(projection[1][1]), 1e-9);
        std::vector<std::pair<double, const VehicleVisuals::ShownFlame*>> near;
        for (const auto& f : visuals.flames()) {
            const vsg::dmat4 modelView = view * f.world;
            near.emplace_back(vsg::length(vsg::dvec3(modelView[3][0], modelView[3][1], modelView[3][2])), &f);
        }
        std::sort(near.begin(), near.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [distance, f] : near) {
            if (flameCount >= kMaxFlames) break;
            const std::uint32_t at = Exhaust::kRows * f->instance;
            if (at + 8 >= rows.size()) continue;
            const double reach = rows.at(at).z, outerNear = rows.at(at + 8).y, outerFar = rows.at(at + 5).w;
            if (!(reach > 0.0) || reach < distance * screenScale * kMinScreenFraction) continue; // too small to read
            const vsg::dmat4 modelView = view * f->world;
            // on the screen at all: its box's corners, and the rectangle round them clipped to the screen
            const vsg::dmat4 toClip = projection * modelView;
            double sLeft = 1e30, sRight = -1e30, sBottom = 1e30, sTop = -1e30;
            bool behind = false;
            for (int k = 0; k < 8 && !behind; ++k) {
                const double r = (k & 1) ? outerFar : outerNear;
                const vsg::dvec4 q = toClip * vsg::dvec4((k & 1) ? reach : 0.0, (k & 2) ? r : -r, (k & 4) ? r : -r, 1.0);
                if (q.w <= 0.0) {
                    behind = true;
                    break;
                }
                sLeft = std::min(sLeft, q.x / q.w);
                sRight = std::max(sRight, q.x / q.w);
                sBottom = std::min(sBottom, q.y / q.w);
                sTop = std::max(sTop, q.y / q.w);
            }
            const double width = behind ? 2.0 : std::max(std::min(sRight, 1.0) - std::max(sLeft, -1.0), 0.0);
            const double height = behind ? 2.0 : std::max(std::min(sTop, 1.0) - std::max(sBottom, -1.0), 0.0);
            if (width <= 0.0 || height <= 0.0) continue;
            coveredSum += width * height / 4.0;
            // its outline for the march: round the cylinder it is marched in (marchFlame: its far radius all along)
            vsg::dvec4 outline(-1.0, -1.0, 1.0, 1.0);
            if (!behind) {
                const double rc = std::max(outerFar, outerNear);
                bool back = false;
                double oLeft = 1e30, oRight = -1e30, oBottom = 1e30, oTop = -1e30;
                for (int k = 0; k < 8 && !back; ++k) {
                    const vsg::dvec4 q = toClip * vsg::dvec4((k & 1) ? reach : 0.0, (k & 2) ? rc : -rc, (k & 4) ? rc : -rc, 1.0);
                    if (q.w <= 0.0) {
                        back = true;
                        break;
                    }
                    oLeft = std::min(oLeft, q.x / q.w), oRight = std::max(oRight, q.x / q.w);
                    oBottom = std::min(oBottom, q.y / q.w), oTop = std::max(oTop, q.y / q.w);
                }
                if (!back) outline = vsg::dvec4(oLeft, oBottom, oRight, oTop);
            }
            outlines[kInstances + flameCount] = outline;
            const vsg::dmat4 fromView = vsg::inverse(modelView);
            const vsg::dvec3 eyeLocal(fromView[3][0], fromView[3][1], fromView[3][2]);
            const double tier = distance > kCheapDistance ? 2.0 : distance > kDetailDistance ? 1.0 : 0.0;
            const double along = std::clamp(eyeLocal.x, 0.0, reach);
            const double viewHeight = std::max(vsg::length(eyeLocal - vsg::dvec3(along, 0.0, 0.0)), 1e-3) * screenScale;
            const double plume = reach * (outerNear + outerFar) / (viewHeight * viewHeight);
            const double share = std::clamp(std::sqrt(kPlumeStepArea / std::max(plume, 1e-6)), 0.0, 1.0);
            const auto F = [](double v) { return static_cast<float>(v); };
            const std::uint32_t base = flameCount * kFlameRows;
            for (std::size_t col = 0; col < 4; ++col)
                flames_->set(base + static_cast<std::uint32_t>(col), vsg::vec4(F(fromView[col][0]), F(fromView[col][1]), F(fromView[col][2]), F(fromView[col][3])));
            flames_->set(base + 4, vsg::vec4(F(static_cast<double>(f->instance)), F(tier), F(share), 0.0f));
            ++flameCount;
        }
        if (flameCount > 0) flames_->dirty();
    }

    // the outlines on the window (px, a pixel wider), and the rectangle round them all: beyond it nothing is marched
    const std::uint32_t ww = std::min(alloc_.x, static_cast<std::uint32_t>(window_.x)), wh = std::min(alloc_.y, static_cast<std::uint32_t>(window_.y));
    double uLeft = 1e30, uBottom = 1e30, uRight = -1e30, uTop = -1e30;
    const auto outlined = [&](std::size_t at, const vsg::dvec4& o) {
        const double x0 = std::max((o.x * 0.5 + 0.5) * ww - kOutlinePx, 0.0), x1 = std::min((o.z * 0.5 + 0.5) * ww + kOutlinePx, static_cast<double>(ww));
        const double y0 = std::max((o.y * 0.5 + 0.5) * wh - kOutlinePx, 0.0), y1 = std::min((o.w * 0.5 + 0.5) * wh + kOutlinePx, static_cast<double>(wh));
        screen_->set(kScreenRects + static_cast<std::uint32_t>(at), vsg::vec4(static_cast<float>(std::floor(x0)), static_cast<float>(std::floor(y0)),
                                                                             static_cast<float>(std::ceil(x1)), static_cast<float>(std::ceil(y1))));
        if (x1 > x0 && y1 > y0) uLeft = std::min(uLeft, x0), uBottom = std::min(uBottom, y0), uRight = std::max(uRight, x1), uTop = std::max(uTop, y1);
    };
    for (std::size_t k = 0; k < shown; ++k) outlined(k, outlines[k]);
    for (std::size_t k = 0; k < flameCount; ++k) outlined(kInstances + k, outlines[kInstances + k]);
    const bool any = (shown > 0 || flameCount > 0) && uRight > uLeft && uTop > uBottom;
    wantsSolid_ = any;
    computeSwitch_->setAllChildren(any);
    drawSwitch_->setAllChildren(any);
    if (any) {
        // the march's share of the window's pixels, and its size
        const double rays = std::min(coveredSum, 1.0) * static_cast<double>(ww) * static_cast<double>(wh);
        const double scale = std::clamp(std::sqrt(rays / kRayBudget), 1.0, kMaxScale);
        const auto uw = static_cast<std::uint32_t>(std::ceil(ww / scale)), uh = static_cast<std::uint32_t>(std::ceil(wh / scale));
        // the texels marched: those the outlines reach, and kRegionTexels round them (all zero, and marched so: the
        // composite's filter reads them); the pixels it is laid on: kLaidTexels round - every texel those read is one
        // marched this frame, and no pixel outside them could have had anything laid on it
        const int tx0 = static_cast<int>(std::floor(uLeft / scale)), ty0 = static_cast<int>(std::floor(uBottom / scale));
        const int tx1 = static_cast<int>(std::ceil(uRight / scale)), ty1 = static_cast<int>(std::ceil(uTop / scale));
        const int rx0 = std::max(tx0 - kRegionTexels, 0), ry0 = std::max(ty0 - kRegionTexels, 0);
        const int rx1 = std::min(tx1 + kRegionTexels, static_cast<int>(uw)), ry1 = std::min(ty1 + kRegionTexels, static_cast<int>(uh));
        dispatch_->groupCountX = static_cast<std::uint32_t>(rx1 - rx0 + 7) / 8;
        dispatch_->groupCountY = static_cast<std::uint32_t>(ry1 - ry0 + 7) / 8;
        screen_->set(7, vsg::vec4(static_cast<float>(rx0), static_cast<float>(ry0), static_cast<float>(rx1), static_cast<float>(ry1)));
        screen_->set(8, vsg::vec4(static_cast<float>(std::floor((tx0 - kLaidTexels) * scale)), static_cast<float>(std::floor((ty0 - kLaidTexels) * scale)),
                                  static_cast<float>(std::ceil((tx1 + kLaidTexels) * scale)), static_cast<float>(std::ceil((ty1 + kLaidTexels) * scale))));
        const vsg::dmat4 inverseProjection = vsg::inverse(projection);
        screen_->set(0, vsg::vec4(static_cast<float>(ww), static_cast<float>(wh), static_cast<float>(flameCount), 0.0f));
        screen_->set(1, vsg::vec4(static_cast<float>(scale), static_cast<float>(shown), static_cast<float>(uw), static_cast<float>(uh)));
        for (std::size_t col = 0; col < 4; ++col)
            screen_->set(static_cast<std::uint32_t>(2 + col), vsg::vec4(static_cast<float>(inverseProjection[col][0]), static_cast<float>(inverseProjection[col][1]),
                                                                       static_cast<float>(inverseProjection[col][2]), static_cast<float>(inverseProjection[col][3])));
        screen_->set(6, vsg::vec4(static_cast<float>(projection[2][2]), static_cast<float>(projection[3][2]), static_cast<float>(projection[2][3]),
                                  static_cast<float>(projection[3][3])));
        screen_->dirty();
        data_->dirty();
    }
}

} // namespace fsim::world
