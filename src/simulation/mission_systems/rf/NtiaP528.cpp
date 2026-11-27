// NTIA's U.S. reference implementation of ITU-R P.528-5 with P.676 and P.835
// (third_party/ntia/p528, public domain; third_party/ntia/README.md): built
// here as one translation unit, inside a namespace of its own, so that its
// names and macros reach nothing else. Then the wrappers Ntia.h declares.
#include "simulation/mission_systems/rf/Ntia.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <math.h>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <stdlib.h>
#include <vector>

namespace fsim_ntia_p528 {
#include "p528/src/p528/CombineDistributions.cpp"
#include "p528/src/p528/FindKForYpiAt99Percent.cpp"
#include "p528/src/p528/GetPathLoss.cpp"
#include "p528/src/p528/InverseComplementaryCumulativeDistributionFunction.cpp"
void LineOfSightExact(Path *path, Terminal *terminal_1, Terminal *terminal_2, LineOfSightParams *los_params,
    double f__mhz, double A_dML__db, double p, double d__km, int T_pol, Result *result, double *K_LOS);
#define LineOfSight LineOfSightExact
#include "p528/src/p528/LineOfSight.cpp"
#undef LineOfSight
#include "p528/src/p528/LinearInterpolation.cpp"
#include "p528/src/p528/LongTermVariability.cpp"
#include "p528/src/p528/NakagamiRice.cpp"
#include "p528/src/p528/P528.cpp"
#include "p528/src/p528/RayOptics.cpp"
#include "p528/src/p528/ReflectionCoefficients.cpp"
#include "p528/src/p528/SmoothEarthDiffraction.cpp"
#include "p528/src/p528/TerminalGeometry.cpp"
#include "p528/src/p528/TranshorizonSearch.cpp"
#include "p528/src/p528/Troposcatter.cpp"
#include "p528/src/p528/ValidateInputs.cpp"
#include "p528/src/p528/data.cpp"
#include "p528/src/p676/GlobalWetPressure.cpp"
#include "p528/src/p676/LineShapeFactor.cpp"
#include "p528/src/p676/NonresonantDebyeAttenuation.cpp"
#include "p528/src/p676/OxygenData.cpp"
void RayTraceExact(double f__ghz, double h_1__km, double h_2__km, double beta_1__rad,
                   RayTraceConfig config, SlantPathAttenuationResult* result);
#define RayTrace RayTraceExact
#include "p528/src/p676/RayTrace.cpp"
#undef RayTrace
#include "p528/src/p676/RefractiveIndex.cpp"
#include "p528/src/p676/Refractivity.cpp"
#include "p528/src/p676/SlantPath.cpp"
#include "p528/src/p676/SpecificAttenuation.cpp"
#include "p528/src/p676/TerrestrialPath.cpp"
#include "p528/src/p676/WaterVapourData.cpp"
#include "p528/src/p676/WaterVapourDensityToPartialPressure.cpp"
#include "p528/src/p835/Conversions.cpp"
#include "p528/src/p835/MeanAnnualGlobalReferenceAtmosphere.cpp"

// P.676's layer properties - the refractive index and the specific
// attenuation of the reference atmosphere at a height - tabulated per
// frequency over P.676's own layer coordinate (equation 14: the layer index
// s = 100 ln(1e4 h (e^0.01 - 1) + 1)), four points a layer, from the exact
// GetLayerPropertiesExact; between them, linear. A ray trace is then its
// geometry alone (a P.528 path ~50x faster); the losses agree with the exact
// to a thousandth of a dB (test_rf_path.cpp).
namespace {
constexpr double kPerLayer = 4.0;
constexpr double kTopKm = 100.0;
struct LayerTable {
    double fGhz = 0.0;
    std::vector<double> n, gamma;
};
double layerIndex(double hKm) { return 100.0 * log(1e4 * hKm * (exp(0.01) - 1.0) + 1.0); }
double layerHeight(double s) { return (exp(s / 100.0) - 1.0) / (1e4 * (exp(0.01) - 1.0)); }
std::mutex tablesMutex;
std::map<double, std::shared_ptr<const LayerTable>> tables;
std::atomic<bool> tabulate{true};
thread_local const LayerTable* lastTable = nullptr;

const LayerTable* tableFor(double fGhz) {
    if (lastTable && lastTable->fGhz == fGhz) return lastTable;
    std::lock_guard<std::mutex> lock(tablesMutex);
    auto it = tables.find(fGhz);
    if (it == tables.end()) {
        if (tables.size() >= 256) return nullptr; // (a sweep across frequencies: exact beyond that many)
        auto t = std::make_shared<LayerTable>();
        t->fGhz = fGhz;
        const auto count = static_cast<std::size_t>(layerIndex(kTopKm) * kPerLayer) + 2;
        t->n.resize(count), t->gamma.resize(count);
        RayTraceConfig config{GlobalTemperature, GlobalPressure, GlobalWetPressure};
        for (std::size_t i = 0; i < count; ++i)
            GetLayerPropertiesExact(fGhz, layerHeight(static_cast<double>(i) / kPerLayer), config, &t->n[i], &t->gamma[i]);
        it = tables.emplace(fGhz, std::move(t)).first;
    }
    lastTable = it->second.get();
    return lastTable;
}
} // namespace

// The grazing ray's trace from the ground to a terminal (TerminalGeometry: a P.676 ray trace through hundreds of layers,
// ~50 us, twice a path - ~90 % of P.528's time): a function of the frequency and the terminal's height alone,
// tabulated per frequency over u = sqrt(h) - in which the horizon's geometry is smooth (d_r, the angle, the length and
// the bending each go as sqrt(h) near the ground) - at nodes filled from the exact trace the first time a height
// between them is asked, and read between them by a cubic (Catmull-Rom) through four nodes.
namespace {
constexpr double kGeometryU0 = 0.0387298334620741688; // sqrt(0.0015 km): P.528's lowest terminal
constexpr double kGeometryDu = 1.0 / 512.0;
constexpr int kGeometryNodes = 2300;                  // (to u = 4.53: above P.528's 20 km)
struct GeometryNode {
    std::atomic<bool> ready{false};
    double v[4] = {0, 0, 0, 0}; // the trace's arrival angle, gas loss, length and bending
};
struct GeometryTable {
    double fMhz = 0.0;
    std::mutex fill;
    std::unique_ptr<GeometryNode[]> nodes{new GeometryNode[kGeometryNodes]};
    GeometryNode minHeightNode; // exact cache for P.528's 1.5 m (0.0015 km) minimum terminal height
};
std::atomic<bool> tabulateGeometry{true};
std::mutex geometryMutex;
std::map<double, std::unique_ptr<GeometryTable>> geometryTables;
thread_local GeometryTable* lastGeometry = nullptr;

GeometryTable* geometryFor(double fMhz) {
    if (lastGeometry && lastGeometry->fMhz == fMhz) return lastGeometry;
    std::lock_guard<std::mutex> lock(geometryMutex);
    auto it = geometryTables.find(fMhz);
    if (it == geometryTables.end()) {
        if (geometryTables.size() >= 256) return nullptr; // (a sweep across frequencies: exact beyond that many)
        auto t = std::make_unique<GeometryTable>();
        t->fMhz = fMhz;
        it = geometryTables.emplace(fMhz, std::move(t)).first;
    }
    lastGeometry = it->second.get();
    return lastGeometry;
}

const double* geometryNode(GeometryTable& t, int k) {
    GeometryNode& g = t.nodes[static_cast<std::size_t>(k)];
    if (!g.ready.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(t.fill);
        if (!g.ready.load(std::memory_order_relaxed)) {
            const double u = kGeometryU0 + kGeometryDu * k;
            SlantPathAttenuationResult r{};
            SlantPathAttenuation(t.fMhz / 1000, 0, u * u, PI / 2, &r); // (TerminalGeometryExact's trace)
            g.v[0] = r.angle__rad, g.v[1] = r.A_gas__db, g.v[2] = r.a__km, g.v[3] = r.bending__rad;
            g.ready.store(true, std::memory_order_release);
        }
    }
    return g.v;
}
const double* geometryMinHeightNode(GeometryTable& t) {
    GeometryNode& g = t.minHeightNode;
    if (!g.ready.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(t.fill);
        if (!g.ready.load(std::memory_order_relaxed)) {
            SlantPathAttenuationResult r{};
            SlantPathAttenuation(t.fMhz / 1000, 0, 0.0015, PI / 2, &r);
            g.v[0] = r.angle__rad, g.v[1] = r.A_gas__db, g.v[2] = r.a__km, g.v[3] = r.bending__rad;
            g.ready.store(true, std::memory_order_release);
        }
    }
    return g.v;
}
} // namespace

void TerminalGeometry(double f__mhz, Terminal* terminal) {
    GeometryTable* t = tabulate.load(std::memory_order_relaxed) && tabulateGeometry.load(std::memory_order_relaxed) ? geometryFor(f__mhz) : nullptr;
    if (t && terminal->h_r__km == 0.0015) {
        const double* v = geometryMinHeightNode(*t);
        const double theta_tx__rad = 0;
        terminal->theta__rad = PI / 2 - v[0];
        terminal->A_a__db = v[1];
        terminal->a__km = v[2];
        const double central_angle = ((PI / 2 - v[0]) - theta_tx__rad + v[3]);
        terminal->d_r__km = a_0__km * central_angle;
        terminal->phi__rad = terminal->d_r__km / a_e__km;
        terminal->h_e__km = (a_e__km / cos(terminal->phi__rad)) - a_e__km;
        terminal->delta_h__km = terminal->h_r__km - terminal->h_e__km;
        return;
    }
    const double u = std::sqrt(std::max(terminal->h_r__km, 0.0));
    const double s = (u - kGeometryU0) / kGeometryDu;
    if (!t || !(s >= 1.0) || s >= kGeometryNodes - 2.0) return TerminalGeometryExact(f__mhz, terminal); // (the table's ends: exact)
    const int k = static_cast<int>(s);
    const double w = s - k;
    const double *p0 = geometryNode(*t, k - 1), *p1 = geometryNode(*t, k), *p2 = geometryNode(*t, k + 1), *p3 = geometryNode(*t, k + 2);
    double v[4];
    for (int i = 0; i < 4; ++i) // (Catmull-Rom)
        v[i] = p1[i] + 0.5 * w * (p2[i] - p0[i] + w * (2.0 * p0[i] - 5.0 * p1[i] + 4.0 * p2[i] - p3[i] + w * (3.0 * (p1[i] - p2[i]) + p3[i] - p0[i])));
    // as TerminalGeometryExact goes on from its trace
    const double theta_tx__rad = 0;
    terminal->theta__rad = PI / 2 - v[0];
    terminal->A_a__db = v[1];
    terminal->a__km = v[2];
    const double central_angle = ((PI / 2 - v[0]) - theta_tx__rad + v[3]);
    terminal->d_r__km = a_0__km * central_angle;
    terminal->phi__rad = terminal->d_r__km / a_e__km;
    terminal->h_e__km = (a_e__km / cos(terminal->phi__rad)) - a_e__km;
    terminal->delta_h__km = terminal->h_r__km - terminal->h_e__km;
}

void GetLayerProperties(double f__ghz, double h_i__km, RayTraceConfig config, double* n, double* gamma) {
    const bool reference = config.temperature == GlobalTemperature && config.dry_pressure == GlobalPressure &&
                           config.wet_pressure == GlobalWetPressure;
    const LayerTable* t = reference && tabulate.load(std::memory_order_relaxed) && h_i__km >= 0.0 && h_i__km < kTopKm ? tableFor(f__ghz) : nullptr;
    if (!t) return GetLayerPropertiesExact(f__ghz, h_i__km, config, n, gamma);
    const double s = layerIndex(h_i__km) * kPerLayer;
    const auto i = static_cast<std::size_t>(s);
    const double w = s - static_cast<double>(i);
    *n = t->n[i] + (t->n[i + 1] - t->n[i]) * w;
    *gamma = t->gamma[i] + (t->gamma[i + 1] - t->gamma[i]) * w;
}

void RayTrace(double f__ghz, double h_1__km, double h_2__km, double beta_1__rad,
              RayTraceConfig config, SlantPathAttenuationResult* result) {
    const bool reference = config.temperature == GlobalTemperature && config.dry_pressure == GlobalPressure &&
                           config.wet_pressure == GlobalWetPressure;
    const LayerTable* t = reference && tabulate.load(std::memory_order_relaxed) && h_1__km >= 0.0 && h_2__km > h_1__km && h_2__km < kTopKm - 2.0
                              ? tableFor(f__ghz)
                              : nullptr;
    if (!t) return RayTraceExact(f__ghz, h_1__km, h_2__km, beta_1__rad, config, result);

    const double kExp01 = std::exp(0.01);
    const double kExp01m1 = kExp01 - 1.0;
    const double kInvExp01m1 = 1.0 / kExp01m1;
    const double kLayerScale = 1e4 * kExp01m1;
    const double kIndexFactor = 100.0 * kPerLayer;

    const int i_lower = static_cast<int>(std::floor(100.0 * std::log(kLayerScale * h_1__km + 1.0) + 1.0));
    const int i_upper = static_cast<int>(std::ceil(100.0 * std::log(kLayerScale * h_2__km + 1.0) + 1.0));
    const double expLower = std::exp((i_lower - 1) / 100.0);
    const double expUpper = std::exp(i_upper / 100.0);
    const double expLowerPlus1 = expLower * kExp01;
    const double m = ((std::exp(0.02) - kExp01) / (expUpper - expLowerPlus1)) * (h_2__km - h_1__km);

    result->A_gas__db = 0.0;
    result->bending__rad = 0.0;
    result->a__km = 0.0;
    result->delta_L__km = 0.0;
    result->angle__rad = beta_1__rad;

    const double* const nTab = t->n.data();
    const double* const gTab = t->gamma.data();
    const std::size_t tabMax = t->n.size() - 2;
    auto lookup = [&](double hKm, double expRef, double lnExpRef, double& nOut, double& gOut) noexcept {
        const double x = kLayerScale * hKm + 1.0;
        const double r = x / expRef;
        double lnX;
        if (r > 0.90 && r < 1.11) {
            const double u = (r - 1.0) / (r + 1.0);
            const double u2 = u * u;
            lnX = lnExpRef + 2.0 * u * (1.0 + u2 * (1.0 / 3.0 + u2 * (1.0 / 5.0)));
        } else {
            lnX = std::log(x);
        }
        const double s = kIndexFactor * lnX;
        const std::size_t idx = std::min(static_cast<std::size_t>(s), tabMax);
        const double w = s - static_cast<double>(idx);
        nOut = nTab[idx] + (nTab[idx + 1] - nTab[idx]) * w;
        gOut = gTab[idx] + (gTab[idx + 1] - gTab[idx]) * w;
    };

    double expCur = expLower;
    double lnExpCur = (i_lower - 1) * 0.01;
    double delta_i__km = m * expCur;
    double h_i__km = h_1__km;
    double n_i = 0.0, gamma_i = 0.0;
    lookup(h_i__km + 0.5 * delta_i__km, expCur, lnExpCur, n_i, gamma_i);
    double r_i__km = a_0__km + h_i__km;

    const double n1r1Sin = n_i * r_i__km * std::sin(beta_1__rad);
    double lastSinAlpha = std::sin(beta_1__rad);

    for (int i = i_lower; i <= i_upper - 1; ++i) {
        const double expNext = expCur * kExp01;
        const double lnExpNext = lnExpCur + 0.01;
        const double delta_ii__km = m * expNext;
        const double h_ii__km = h_1__km + m * ((expNext - expLower) * kInvExp01m1);
        double n_ii = 0.0, gamma_ii = 0.0;
        lookup(h_ii__km + 0.5 * delta_ii__km, expNext, lnExpNext, n_ii, gamma_ii);
        const double r_ii__km = a_0__km + h_ii__km;

        const double sinBetaI = std::min(1.0, n1r1Sin / (n_i * r_i__km));
        const double cosBetaI = std::sqrt(std::max(0.0, 1.0 - sinBetaI * sinBetaI));
        const double rCos = r_i__km * cosBetaI;
        const double a_i__km = -rCos + std::sqrt(rCos * rCos + delta_i__km * (2.0 * r_i__km + delta_i__km));

        result->a__km += a_i__km;
        result->A_gas__db += a_i__km * gamma_i;
        result->delta_L__km += a_i__km * (n_i - 1.0);

        const double sinAlphaI = std::min(1.0, n1r1Sin / (n_i * r_ii__km));
        lastSinAlpha = sinAlphaI;
        if (i != i_upper - 1) {
            const double sinBetaII = std::min(1.0, (n_i / n_ii) * sinAlphaI);
            const double dSin = sinBetaII - sinAlphaI;
            if (sinAlphaI < 0.999 && sinBetaII < 0.999 && std::fabs(dSin) < 1e-4) {
                const double sMid = 0.5 * (sinAlphaI + sinBetaII);
                result->bending__rad += dSin / std::sqrt(1.0 - sMid * sMid);
            } else {
                result->bending__rad += std::asin(sinBetaII) - std::asin(sinAlphaI);
            }
        }

        expCur = expNext;
        lnExpCur = lnExpNext;
        delta_i__km = delta_ii__km;
        h_i__km = h_ii__km;
        n_i = n_ii;
        gamma_i = gamma_ii;
        r_i__km = r_ii__km;
    }

    if (i_lower <= i_upper - 1) result->angle__rad = std::asin(lastSinAlpha);
}

namespace {
struct LosHeaderKey {
    double fMhz = 0.0, h1Km = 0.0, h2Km = 0.0, dDKm = 0.0, dMlKm = 0.0, aDmlDb = 0.0;
    int tPol = 0;
    bool operator==(const LosHeaderKey& o) const noexcept {
        return fMhz == o.fMhz && h1Km == o.h1Km && h2Km == o.h2Km &&
               dDKm == o.dDKm && dMlKm == o.dMlKm && aDmlDb == o.aDmlDb && tPol == o.tPol;
    }
};
struct LosHeaderEntry {
    bool valid = false;
    LosHeaderKey key{};
    double psiLimit = 0.0;
    double d0Km = 0.0;
    double aLosD0Db = 0.0;
};
constexpr std::size_t kLosHeaderCacheSize = 16;
thread_local std::array<LosHeaderEntry, kLosHeaderCacheSize> losHeaderCache{};
thread_local std::size_t losHeaderNextSlot = 0;
} // namespace

void LineOfSight(Path *path, Terminal *terminal_1, Terminal *terminal_2, LineOfSightParams *los_params,
    double f__mhz, double A_dML__db, double p, double d__km, int T_pol, Result *result, double *K_LOS) {
    if (!tabulate.load(std::memory_order_relaxed)) {
        LineOfSightExact(path, terminal_1, terminal_2, los_params, f__mhz, A_dML__db, p, d__km, T_pol, result, K_LOS);
        return;
    }
    const double lambda__km = 0.2997925 / f__mhz;
    const LosHeaderKey key{f__mhz, terminal_1->h_r__km, terminal_2->h_r__km, path->d_d__km, path->d_ML__km, A_dML__db, T_pol};
    const LosHeaderEntry* hit = nullptr;
    for (const auto& e : losHeaderCache) {
        if (e.valid && e.key == key) { hit = &e; break; }
    }
    double psi_limit = 0.0;
    double aLosD0 = 0.0;
    double R_Tg = 0.0;
    if (hit) {
        psi_limit = hit->psiLimit;
        path->d_0__km = hit->d0Km;
        aLosD0 = hit->aLosD0Db;
    } else {
        const double terminate = lambda__km / 1e6;
        psi_limit = FindPsiAtDeltaR(lambda__km / 2, path, terminal_1, terminal_2, terminate);
        const double d_y6__km = FindDistanceAtDeltaR(lambda__km / 6, path, terminal_1, terminal_2, terminate);
        if (terminal_1->d_r__km >= path->d_d__km || path->d_d__km >= path->d_ML__km) {
            if (terminal_1->d_r__km > d_y6__km || d_y6__km > path->d_ML__km)
                path->d_0__km = terminal_1->d_r__km;
            else
                path->d_0__km = d_y6__km;
        } else if (path->d_d__km < d_y6__km && d_y6__km < path->d_ML__km) {
            path->d_0__km = d_y6__km;
        } else {
            path->d_0__km = path->d_d__km;
        }
        double d_temp__km = path->d_0__km;
        while (true) {
            const double psiWalk = FindPsiAtDistance(d_temp__km, path, terminal_1, terminal_2);
            LineOfSightParams los_result;
            RayOptics(terminal_1, terminal_2, psiWalk, &los_result);
            if (los_result.d__km >= path->d_0__km || (d_temp__km + 0.001) >= path->d_ML__km) {
                path->d_0__km = los_result.d__km;
                break;
            }
            d_temp__km += 0.001;
        }
        const double psi_d0 = FindPsiAtDistance(path->d_0__km, path, terminal_1, terminal_2);
        RayOptics(terminal_1, terminal_2, psi_d0, los_params);
        GetPathLoss(psi_d0, path, f__mhz, psi_limit, A_dML__db, 0, T_pol, los_params, &R_Tg);
        aLosD0 = los_params->A_LOS__db;
        losHeaderCache[losHeaderNextSlot] = LosHeaderEntry{true, key, psi_limit, path->d_0__km, aLosD0};
        losHeaderNextSlot = (losHeaderNextSlot + 1) % kLosHeaderCacheSize;
    }

    const double psi = FindPsiAtDistance(d__km, path, terminal_1, terminal_2);
    RayOptics(terminal_1, terminal_2, psi, los_params);
    GetPathLoss(psi, path, f__mhz, psi_limit, A_dML__db, aLosD0, T_pol, los_params, &R_Tg);

    SlantPathAttenuationResult result_slant;
    SlantPathAttenuation(f__mhz / 1000, terminal_1->h_r__km, terminal_2->h_r__km, PI / 2 - los_params->theta_h1__rad, &result_slant);
    result->A_a__db = result_slant.A_gas__db;
    result->A_fs__db = 20.0 * log10(los_params->r_0__km) + 20.0 * log10(f__mhz) + 32.45;

    double f_theta_h;
    if (los_params->theta_h1__rad <= 0.0) f_theta_h = 1.0;
    else if (los_params->theta_h1__rad >= 1.0) f_theta_h = 0.0;
    else f_theta_h = MAX(0.5 - (1 / PI) * (atan(20.0 * log10(32.0 * los_params->theta_h1__rad))), 0);

    double Y_e__db, Y_e_50__db, A_Y;
    LongTermVariability(terminal_1->d_r__km, terminal_2->d_r__km, d__km, f__mhz, p, f_theta_h, los_params->A_LOS__db, &Y_e__db, &A_Y);
    LongTermVariability(terminal_1->d_r__km, terminal_2->d_r__km, d__km, f__mhz, 50, f_theta_h, los_params->A_LOS__db, &Y_e_50__db, &A_Y);

    double F_AY;
    if (A_Y <= 0.0) F_AY = 1.0;
    else if (A_Y >= 9.0) F_AY = 0.1;
    else F_AY = (1.1 + (0.9 * cos((A_Y / 9.0) * PI))) / 2.0;

    double F_delta_r;
    if (los_params->delta_r__km >= (lambda__km / 2.0)) F_delta_r = 1.0;
    else if (los_params->delta_r__km <= lambda__km / 6.0) F_delta_r = 0.1;
    else F_delta_r = 0.5 * (1.1 - (0.9 * cos(((3.0 * PI) / lambda__km) * (los_params->delta_r__km - (lambda__km / 6.0)))));

    const double R_s = R_Tg * F_delta_r * F_AY;
    const double Y_pi_99__db = 10.0 * log10(f__mhz * pow(result_slant.a__km, 3)) - 84.26;
    const double K_t = FindKForYpiAt99Percent(Y_pi_99__db);
    const double W_a = pow(10.0, K_t / 10.0);
    const double W_R = R_s * R_s + 0.0001;
    const double W = W_R + W_a;
    if (W <= 0.0) {
        *K_LOS = -40.0;
    } else {
        *K_LOS = 10.0 * log10(W);
        if (*K_LOS < -40.0) *K_LOS = -40.0;
    }
    const double Y_pi_50__db = 0.0;
    const double Y_pi__db = NakagamiRice(*K_LOS, p);
    const double Y_total__db = -CombineDistributions(Y_e_50__db, Y_e__db, Y_pi_50__db, Y_pi__db, p);

    result->d__km = los_params->d__km;
    result->A__db = result->A_fs__db + result->A_a__db - los_params->A_LOS__db + Y_total__db;
    result->theta_h1__rad = los_params->theta_h1__rad;
}
} // namespace fsim_ntia_p528

namespace fsim::ms::ntia {

int p528(double dKm, double h1M, double h2M, double fMhz, int pol, double timePercent, P528Result& out) {
    fsim_ntia_p528::Result r{};
    const int rc = fsim_ntia_p528::P528(dKm, h1M, h2M, fMhz, pol, timePercent, &r);
    out.mode = r.propagation_mode;
    out.warnings = r.warnings;
    out.dKm = r.d__km;
    out.lossDb = r.A__db;
    out.freeSpaceDb = r.A_fs__db;
    out.gasDb = r.A_a__db;
    out.thetaH1Rad = r.theta_h1__rad;
    return rc;
}

double p676Slant(double fGhz, double h1Km, double h2Km, double elevationRad) {
    fsim_ntia_p528::SlantPathAttenuationResult r{};
    // (its beta is the angle from the zenith: above 90 deg, the ray leaves downwards)
    if (fsim_ntia_p528::SlantPathAttenuation(fGhz, h1Km, h2Km, 1.5707963267948966 - elevationRad, &r) != 0) return std::nan("");
    return r.A_gas__db;
}

void tabulateLayers(bool on) { fsim_ntia_p528::tabulate.store(on); }

void tabulateTerminals(bool on) { fsim_ntia_p528::tabulateGeometry.store(on); }

void prepare(double fMhz) { fsim_ntia_p528::tableFor(fMhz / 1000); } // (P.528's own f__mhz / 1000: the same key)

double p676Specific(double fGhz, double hKm) {
    const double t = fsim_ntia_p528::GlobalTemperature(hKm);
    const double p = fsim_ntia_p528::GlobalPressure(hKm);
    const double e = fsim_ntia_p528::GlobalWetPressure(hKm);
    return fsim_ntia_p528::SpecificAttenuation(fGhz, t, e, p);
}

} // namespace fsim::ms::ntia
