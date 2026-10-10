// NVIDIA Flow's fires measured against the observed ones (MS-7h, step 3): the sources world::FlowVolumes gives the
// solver (world/FlowFires.h, its calibration in its comment), each case run alone on the solver's own device,
// stepped at its rate and waited on, its fields read back each step.
//
//   spill M       a holed tank's M kg of kerosene at once, the hole moving at `path` m/s: against CCPS's fireball
//   detonation W  W kg TNT: against the photographed fireball (its duration an estimate)
//   leak R        R kg/s from a hole at `path` m/s: against API RP 521's jet flame
//   check         all the calibration's cases, each within its tolerance (the exit code: how many are not)
//
// What is measured: the luminous fireball - the voxels above 1,200 K (the flame's edge against the day; the solver's
// temperature 1 is the flame's, FlowFires::kFlameK) - as the sphere of its volume at its largest (its diameter) and
// from its first luminous step to its last (its duration); a leak's, its luminous length behind the hole at the end.
// Also the smoke left (the solver's units times m^3: its total extinction, m^2) against the soot put in.
//
//   flightsim-flow-calibrate spill 100 [pressure=<Pa>] [path=<m/s>] [seconds=<s>] [calls=<per s>] [csv=<file>]

#include "world/FlowFires.h"
#include "world/FlowSolver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

using fsim::world::FlowSolver;
namespace ff = fsim::world::flowfires;

struct Case {
    std::string kind;
    double amount = 0.0, pressure = 101325.0, path = 0.0, seconds = 0.0, calls = 60.0; ///< (calls: the solver's a second, as a viewer's frames)
    std::string csv;
};
struct Result {
    double measured = 0.0, observed = 0.0, duration = 0.0, observedDuration = 0.0, smoke = 0.0, soot = 0.0, gpuMs = 0.0;
};

Case of(const char* kind, double amount, double pressure = 101325.0, double path = 0.0) {
    Case c;
    c.kind = kind, c.amount = amount, c.pressure = pressure, c.path = path;
    return c;
}

Result measure(const Case& c) {
    FlowSolver::Settings settings;
    settings.cellM = 0.5;
    settings.exportFuel = true;
    settings.combustion = ff::combustion();
    FlowSolver solver;
    Result r;
    if (!solver.start(settings)) {
        std::fprintf(stderr, "NVIDIA Flow did not start (nvflow.dll, nvflowext.dll beside this program?)\n");
        std::exit(1);
    }
    const bool spill = c.kind == "spill", leak = c.kind == "leak";
    const double seconds = c.seconds > 0.0 ? c.seconds : spill ? 2.5 * ff::ccpsDuration(c.amount) + 0.5 : leak ? 3.0 : 1.0;
    std::FILE* csv = c.csv.empty() ? nullptr : std::fopen(c.csv.c_str(), "w");
    if (csv) std::fprintf(csv, "t,diameter,length,height,middle_z,smoke,hot,leaves,gpu_ms\n");
    const double dt = 1.0 / c.calls, cell3 = std::pow(settings.cellM, 3.0);
    const double visible = (1200.0 - ff::kAmbientK) / (ff::kFlameK - ff::kAmbientK);
    double vMax = 0.0, firstLit = -1.0, lastLit = 0.0, gpu = 0.0;
    int steps = 0;
    std::vector<FlowSolver::Sphere> sources;
    for (double t = 0.0; t < seconds; t += dt, ++steps) {
        sources.clear();
        const double origin[3] = {0.0, 0.0, 0.0}, path[3] = {c.path, 0.0, 0.0}, none[3] = {0.0, 0.0, 0.0};
        if (spill) ff::spill(c.amount, origin, path, c.pressure, t, dt, 1, sources);
        else if (leak) {
            const double at[3] = {c.path * t, 0.0, 0.0};
            ff::leak(c.amount, at, path, c.pressure, dt, 1, sources);
        } else ff::detonation(c.amount, origin, none, c.pressure, t, dt, 1, sources);
        solver.step(dt, sources, steps == 0, true);
        gpu += solver.gpuMs();
        const FlowSolver::Fields& f = solver.fields();
        std::uint64_t hot = 0;
        double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30}, zSum = 0.0, smoke = 0.0;
        for (const auto& leaf : f.leaves)
            for (std::uint32_t n = 0; n < 512u; ++n) {
                smoke += static_cast<double>(leaf.smoke[n]);
                if (!(static_cast<double>(leaf.temperature[n]) >= visible)) continue;
                const int ijk[3] = {leaf.origin[0] + static_cast<int>(n >> 6), leaf.origin[1] + static_cast<int>((n >> 3) & 7u),
                                    leaf.origin[2] + static_cast<int>(n & 7u)};
                for (int a = 0; a < 3; ++a) {
                    const double x = f.zero[a] + ijk[a] * f.voxel;
                    lo[a] = std::min(lo[a], x), hi[a] = std::max(hi[a], x + f.voxel);
                }
                zSum += f.zero[2] + (ijk[2] + 0.5) * f.voxel;
                ++hot;
            }
        const double volume = static_cast<double>(hot) * cell3, diameter = std::cbrt(6.0 * volume / 3.141592653589793);
        const double length = hot ? hi[0] - lo[0] : 0.0;
        if (leak) r.measured = length;
        else if (volume > vMax) vMax = volume, r.measured = diameter;
        if (hot) {
            if (firstLit < 0.0) firstLit = t;
            lastLit = t;
        }
        r.smoke = smoke * cell3;
        if (csv)
            std::fprintf(csv, "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%llu,%zu,%.3f\n", t, diameter, length, hot ? hi[2] - lo[2] : 0.0,
                         hot ? zSum / static_cast<double>(hot) : 0.0, r.smoke, static_cast<unsigned long long>(hot), f.leaves.size(),
                         solver.gpuMs());
    }
    if (csv) std::fclose(csv);
    r.duration = firstLit < 0.0 ? 0.0 : lastLit - firstLit;
    r.gpuMs = steps ? gpu / steps : 0.0;
    if (spill) {
        r.observed = ff::ccpsDiameter(c.amount, c.pressure), r.observedDuration = ff::ccpsDuration(c.amount);
        r.soot = ff::kSootExtinction * ff::kSootYield * c.amount;
    } else if (leak) {
        r.observed = ff::jetFlameLength(c.amount);
        r.soot = ff::kSootExtinction * ff::kSootYield * c.amount * seconds;
    } else {
        r.observed = ff::detonationDiameter(c.amount, c.pressure), r.observedDuration = ff::detonationDuration(c.amount);
        r.soot = ff::kSootExtinction * ff::kTntSoot * c.amount;
    }
    return r;
}

void print(const Case& c, const Result& r) {
    std::printf("%-10s %7g %s at %6.0f Pa, %5.0f m/s: %s %6.2f m against %6.2f (%.2f)", c.kind.c_str(), c.amount,
                c.kind == "leak" ? "kg/s" : c.kind == "spill" ? "kg  " : "kg TNT", c.pressure, c.path,
                c.kind == "leak" ? "flame" : "diameter", r.measured, r.observed, r.observed > 0.0 ? r.measured / r.observed : 0.0);
    if (r.observedDuration > 0.0) std::printf(", %5.2f s against %5.2f (%.2f)", r.duration, r.observedDuration, r.duration / r.observedDuration);
    std::printf(", smoke %.2f of the soot's, %.2f ms GPU a step\n", r.soot > 0.0 ? r.smoke / r.soot : 0.0, r.gpuMs);
    std::fflush(stdout);
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: flightsim-flow-calibrate spill|detonation|leak <amount> [key=value ...] | check\n");
        return 2;
    }
    const std::string kind = argv[1];
    if (kind == "check") {
        // the calibration's cases, and how near each must come: its size within 15 % (a leak's 20 %), its duration
        // 20 % (a smallest case 40 %: the cells' and the steps' floor)
        struct Checked {
            Case c;
            double size, time;
        };
        const std::vector<Checked> cases = {
            {of("spill", 3), 0.15, 0.4},
            {of("spill", 10), 0.15, 0.2},
            {of("spill", 100), 0.15, 0.2},
            {of("spill", 1000), 0.15, 0.2},
            {of("spill", 50, 47181.0, 200.0), 0.15, 0.2},
            {of("detonation", 1), 0.15, 0.4},
            {of("detonation", 8), 0.15, 0.2},
            {of("detonation", 30), 0.15, 0.2},
            {of("detonation", 8, 47181.0), 0.15, 0.2},
            {of("leak", 5, 47181.0, 200.0), 0.2, 0.0},
            {of("leak", 20, 47181.0, 200.0), 0.2, 0.0},
            {of("leak", 5, 47181.0, 100.0), 0.2, 0.0},
        };
        int outside = 0;
        for (const Checked& k : cases) {
            const Result r = measure(k.c);
            print(k.c, r);
            const bool sizeOk = std::abs(r.measured / r.observed - 1.0) <= k.size;
            const bool timeOk = k.time <= 0.0 || std::abs(r.duration / r.observedDuration - 1.0) <= k.time;
            if (!sizeOk || !timeOk) ++outside, std::printf("    ^ outside its tolerance\n");
        }
        std::printf("%d of %zu cases outside their tolerance\n", outside, cases.size());
        return outside;
    }
    if (kind != "spill" && kind != "detonation" && kind != "leak") {
        std::fprintf(stderr, "spill, detonation, leak or check\n");
        return 2;
    }
    if (argc < 3) {
        std::fprintf(stderr, "how much?\n");
        return 2;
    }
    Case c;
    c.kind = kind;
    c.amount = std::atof(argv[2]);
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        const auto eq = a.find('=');
        const std::string key = a.substr(0, eq), value = eq == std::string::npos ? "" : a.substr(eq + 1);
        if (key == "pressure") c.pressure = std::atof(value.c_str());
        else if (key == "path") c.path = std::atof(value.c_str());
        else if (key == "seconds") c.seconds = std::atof(value.c_str());
        else if (key == "calls") c.calls = std::atof(value.c_str());
        else if (key == "csv") c.csv = value;
        else {
            std::fprintf(stderr, "unknown: %s\n", a.c_str());
            return 2;
        }
    }
    print(c, measure(c));
    return 0;
}
