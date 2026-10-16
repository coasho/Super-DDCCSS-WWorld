// NTIA's U.S. reference implementation of ITU-R P.528-5 with P.676 and P.835
// (third_party/ntia/p528, public domain; third_party/ntia/README.md): built
// here as one translation unit, inside a namespace of its own, so that its
// names and macros reach nothing else. Then the wrappers Ntia.h declares.
#include "simulation/mission_systems/rf/Ntia.h"

#include <algorithm>
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
#include "p528/src/p528/LineOfSight.cpp"
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
#include "p528/src/p676/RayTrace.cpp"
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
} // namespace

void TerminalGeometry(double f__mhz, Terminal* terminal) {
    GeometryTable* t = tabulate.load(std::memory_order_relaxed) && tabulateGeometry.load(std::memory_order_relaxed) ? geometryFor(f__mhz) : nullptr;
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
