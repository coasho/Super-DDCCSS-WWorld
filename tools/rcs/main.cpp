// flightsim-rcs: an aircraft type's radar cross section table (ADR-30, MS-3),
// computed by shooting and bouncing rays over its hangar model on the GPU.
//
//   flightsim-rcs <type>... [--bands 1.3e9,3e9,9.5e9,16e9] [--bin 3] [--sub 3] [--rays 8] [--bounces 5]
//                           [--phong 0.75] [--out <file>] [--no-yield]   writes aircraft/<type>/rcs.bin
//   (it waits between dispatches while a flightsim viewer runs; --no-yield shares the GPU instead)
//   flightsim-rcs <type>... --verify [--elevations -45,-15,0,15,45] [--tolerance-db 0.1]
//                           the table against the model as it is now: those rows recomputed, every bin's tt and pp
//                           compared - and against the method's own noise, the model moved by a fraction of a ray tube
//                           (its RCS the same; only where the tubes fall on it changes); no further from the table than
//                           from itself moved (its 95th, 99th percentiles and largest, each within the tolerance over
//                           the noise's), the table stands and its provenance records the check, else it is left and the
//                           run fails
//   flightsim-rcs <type|file.glb> --at <az>,<el> --frequency <hz>  one aspect's scattering matrix
//   flightsim-rcs --shape sphere|plate|dihedral|trihedral|cylinder --size <m> --at ... --frequency ...
#include "Engine.h"
#include "Model.h"
#include "Shapes.h"

#include "core/Json.h"
#include "ms/rcs/Table.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <tlhelp32.h>
#endif

namespace fs = std::filesystem;
using namespace fsim;

namespace {

/// A batch job that keeps the GPU busy for hours must not take frames from the viewer: the process's GPU scheduling
/// class to idle (WDDM's D3DKMTSetProcessSchedulingPriorityClass, gdi32), so the graphics scheduler runs anyone
/// else's work first - with nobody else, the tables take as long as before. Its CPU class below normal likewise.
void yieldToInteractive() {
#ifdef _WIN32
    SetPriorityClass(GetCurrentProcess(), BELOW_NORMAL_PRIORITY_CLASS);
    using SetClass = LONG(APIENTRY*)(HANDLE, int);
    if (HMODULE gdi = LoadLibraryW(L"gdi32.dll"))
        if (auto set = reinterpret_cast<SetClass>(reinterpret_cast<void*>(GetProcAddress(gdi, "D3DKMTSetProcessSchedulingPriorityClass"))))
            if (set(GetCurrentProcess(), 0 /* D3DKMT_SCHEDULINGPRIORITYCLASS_IDLE */) != 0)
                std::fprintf(stderr, "flightsim-rcs: the GPU scheduling class stays normal\n");
#endif
}

/// Whether a flightsim viewer is running (its frames are the GPU's first call).
bool viewerRunning() {
#ifdef _WIN32
    const HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return false;
    PROCESSENTRY32W e{};
    e.dwSize = sizeof e;
    bool found = false;
    for (BOOL ok = Process32FirstW(snap, &e); ok && !found; ok = Process32NextW(snap, &e))
        found = _wcsicmp(e.szExeFile, L"flightsim-viewer.exe") == 0;
    CloseHandle(snap);
    return found;
#else
    return false;
#endif
}

/// Before each dispatch: while a viewer runs, wait (a dispatch of a large aircraft's high band holds the GPU for a
/// second or more, and even at the idle class a viewer with little slack misses frames to it). Checked twice a
/// second at most.
void standAside() {
    static auto checked = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (now - checked < std::chrono::milliseconds(500)) return;
    checked = now;
    if (!viewerRunning()) return;
    std::fprintf(stderr, "\nflightsim-rcs: a viewer is open - waiting until it closes (--no-yield: share the GPU)\n");
    const auto t0 = std::chrono::steady_clock::now();
    while (viewerRunning()) std::this_thread::sleep_for(std::chrono::seconds(1));
    std::fprintf(stderr, "flightsim-rcs: carrying on after %.0f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    checked = std::chrono::steady_clock::now();
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

std::vector<double> numbers(const std::string& s) {
    std::vector<double> v;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, ',')) v.push_back(std::stod(x));
    return v;
}

/// The repository's aircraft/ (beside this program's tree: build/<preset>/bin/.. or the source tree), else the working directory's.
fs::path aircraftDir(const char* argv0) {
    for (fs::path p = fs::absolute(argv0).parent_path(); !p.empty() && p != p.root_path(); p = p.parent_path())
        if (fs::exists(p / "aircraft" / "models.txt")) return p / "aircraft";
    return fs::current_path() / "aircraft";
}

std::uint64_t fnv(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    std::uint64_t h = 1469598103934665603ull;
    char buf[65536];
    while (in.read(buf, sizeof buf) || in.gcount())
        for (std::streamsize i = 0; i < in.gcount(); ++i) h = (h ^ static_cast<unsigned char>(buf[i])) * 1099511628211ull;
    return h;
}

/// The treatments' values, the same for every aircraft (assets/signature/treatments.json).
struct Treatments {
    double ramFullDb = 15.0, ramPartialDb = 3.0, ramLowHz = 4e9, intakeConcealedDb = 16.0;
};

core::Json readJson(const fs::path& file) {
    std::ifstream in(file);
    if (!in) return core::Json();
    std::stringstream ss;
    ss << in.rdbuf();
    try {
        return core::Json::parse(ss.str(), file.string());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "flightsim-rcs: %s\n", e.what());
        return core::Json();
    }
}

Treatments treatmentsAt(const fs::path& file) {
    Treatments t;
    const core::Json doc = readJson(file);
    const core::Json& p = doc.child("parameters");
    t.ramFullDb = p.child("ram_full_loss_db").number("value", t.ramFullDb);
    t.ramPartialDb = p.child("ram_partial_loss_db").number("value", t.ramPartialDb);
    t.ramLowHz = p.child("ram_low_frequency").number("value", t.ramLowHz);
    t.intakeConcealedDb = p.child("intake_concealed_loss_db").number("value", t.intakeConcealedDb);
    return t;
}

/// An aircraft's documented low-observable features (aircraft/<type>/signature.json): none without a card.
struct Treatment {
    std::string ram = "none"; ///< "full", "partial", "none"
    bool concealed = false, lined = false, exhaust = false;
};

Treatment treatmentOf(const fs::path& dir) {
    Treatment t;
    const core::Json doc = readJson(dir / "signature.json");
    const core::Json& f = doc.child("features");
    if (!f.isObject()) return t;
    t.ram = f.child("ram").string("value", "none");
    auto flag = [&](const char* key) { const core::Json* v = f.child(key).find("value"); return v && v->isBool() && v->asBool(); };
    t.concealed = flag("intake_concealed"), t.lined = flag("duct_lined"), t.exhaust = flag("exhaust_treated");
    return t;
}

/// The treatments an aircraft's features bring, at a frequency:
///  - its skin: the coating's loss (full or on its largest reflectors), its full value from ramLowHz up, below it in
///    proportion to the frequency;
///  - its intake cavity: a concealed engine face's loss ((f / 9.5 GHz)^0.3 of it below 9.5 GHz), and a lined duct's
///    coating on top;
///  - its exhaust: coated as its skin where its treatment is stated, else bare metal and an open cavity.
void applyTreatment(rcs::Settings& s, const Treatment& t, const Treatments& v, double f) {
    const double scale = std::min(1.0, f / v.ramLowHz);
    const double ram = (t.ram == "full" ? v.ramFullDb : t.ram == "partial" ? v.ramPartialDb : 0.0) * scale;
    const double lining = t.lined ? v.ramFullDb * scale : 0.0;
    const double concealed = t.concealed ? v.intakeConcealedDb * std::min(1.0, std::pow(f / 9.5e9, 0.3)) : 0.0;
    s.skinLossDb = ram;
    s.cavityReflectivity = std::pow(10.0, -(concealed + lining) / 10.0);
    s.exhaustLossDb = t.exhaust ? ram : 0.0;
    s.exhaustReflectivity = t.exhaust ? std::pow(10.0, -ram / 10.0) : 1.0;
}

std::string describe(const Treatment& t) {
    return "RAM " + t.ram + (t.concealed ? ", engine face concealed" : "") + (t.lined ? ", duct lined" : "") + (t.exhaust ? ", exhaust treated" : "");
}

int usage() {
    std::fprintf(stderr, "usage: flightsim-rcs <type>... [--bands hz,..] [--bin deg] [--sub n] [--rays per-lambda] [--bounces n] "
                         "[--phong a] [--out file]\n       flightsim-rcs <type|file.glb> --at az,el --frequency hz\n"
                         "       flightsim-rcs --shape sphere|plate|dihedral|trihedral|cylinder --size m --at az,el --frequency hz\n");
    return 2;
}

} // namespace

int main(int argc, char** argv) {
    yieldToInteractive();
    bool yield = true;
    for (int i = 1; i < argc; ++i) yield = yield && std::strcmp(argv[i], "--no-yield") != 0;
    if (yield) rcs::setDispatchGate(standAside);
    std::vector<std::string> targets;
    std::vector<double> bands = {1.3e9, 3.0e9, 9.5e9, 16e9}, at, patch;
    double binDeg = 3.0, frequency = 10e9, size = 1.0;
    int sub = 3;
    std::string shape, outPath;
    bool parts = false;
    std::string ramOverride;
    int exhaustTreated = -1;
    int rings = 256;
    std::vector<double> elevationsDeg; // (--elevations: only these rows - a measurement's slice, not a table to use)
    std::string dumpPath;              // (--dump: every aspect's scattering, to compare two engines)
    bool verify = false;               // (--verify: the table against the model as it is now)
    double toleranceDb = 0.1;
    rcs::Settings settings;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) std::exit(usage());
            return argv[++i];
        };
        if (a == "--no-yield") continue; // (above)
        else if (a == "--bands") bands = numbers(next());
        else if (a == "--bin") binDeg = std::stod(next());
        else if (a == "--sub") sub = std::stoi(next());
        else if (a == "--rays") settings.raysPerWavelength = std::stod(next());
        else if (a == "--bounces") settings.maxBounces = std::stoi(next());
        else if (a == "--phong") settings.phongAlpha = std::stod(next());
        else if (a == "--cavity") settings.cavityReflectivity = std::stod(next());
        else if (a == "--skin-loss") settings.skinLossDb = std::stod(next());
        else if (a == "--ram") ramOverride = next(); // full, partial, none: in place of the card's
        else if (a == "--exhaust-treated") exhaustTreated = std::stoi(next());
        else if (a == "--out") outPath = next();
        else if (a == "--at") at = numbers(next());
        else if (a == "--patch") patch = numbers(next());
        else if (a == "--frequency") frequency = std::stod(next());
        else if (a == "--shape") shape = next();
        else if (a == "--size") size = std::stod(next());
        else if (a == "--parts") parts = true;
        else if (a == "--rings") rings = std::stoi(next());
        else if (a == "--elevations") elevationsDeg = numbers(next());
        else if (a == "--dump") dumpPath = next();
        else if (a == "--verify") verify = true;
        else if (a == "--tolerance-db") toleranceDb = std::stod(next());
        else if (!a.empty() && a[0] == '-') return usage();
        else targets.push_back(a);
    }
    if (targets.empty() && shape.empty()) return usage();
    if (verify && elevationsDeg.empty()) elevationsDeg = {-45.0, -15.0, 0.0, 15.0, 45.0};

    std::string error;
    auto engine = rcs::Engine::create(&error);
    if (!engine) {
        std::fprintf(stderr, "flightsim-rcs: %s\n", error.c_str());
        return 1;
    }
    std::printf("device: %s\n", engine->deviceName().c_str());

    rcs::ModelReport lastReport;
    const Treatments treatments = treatmentsAt(aircraftDir(argv[0]).parent_path() / "assets" / "signature" / "treatments.json");
    // the card's treatment, or the one given on the command line
    auto chosen = [&](const fs::path& file) {
        Treatment t = treatmentOf(file.parent_path());
        if (!ramOverride.empty()) t.ram = ramOverride;
        if (exhaustTreated >= 0) t.exhaust = exhaustTreated != 0;
        return t;
    };
    auto loadTarget = [&](const std::string& t, rcs::Mesh& mesh, fs::path& file) {
        if (!shape.empty()) {
            mesh = shape == "sphere" ? rcs::shapes::sphere(size, rings) : shape == "plate" ? rcs::shapes::plate(size)
                 : shape == "dihedral" ? rcs::shapes::dihedral(size) : shape == "trihedral" ? rcs::shapes::trihedral(size)
                 : rcs::shapes::cylinder(size, 4 * size, 2 * rings);
            return true;
        }
        file = fs::path(t).extension() == ".glb" ? fs::path(t) : aircraftDir(argv[0]) / t / (t + ".glb");
        rcs::ModelReport report;
        if (!rcs::readModel(file, mesh, &report, &error)) {
            std::fprintf(stderr, "flightsim-rcs: %s\n", error.c_str());
            return false;
        }
        std::printf("%s: %zu triangles (%zu nodes, %zu triangles left out)\n", file.string().c_str(), report.triangles, report.skippedNodes,
                    report.skippedTriangles);
        lastReport = report;
        return true;
    };

    if (!at.empty()) { // one aspect
        if (at.size() != 2) return usage();
        rcs::Mesh mesh;
        fs::path file;
        if (!loadTarget(targets.empty() ? shape : targets[0], mesh, file) || !engine->load(mesh, &error)) {
            std::fprintf(stderr, "flightsim-rcs: %s\n", error.c_str());
            return 1;
        }
        if (patch.size() == 2) { // the mean over a patch of n x n aspects, `width` degrees wide, round the one given
            if (shape.empty()) applyTreatment(settings, chosen(file), treatments, frequency);
            const int n = static_cast<int>(patch[1]);
            std::vector<rcs::Aspect> aspects;
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    aspects.push_back({(at[0] + ((i + 0.5) / n - 0.5) * patch[0]) * kDeg, (at[1] + ((j + 0.5) / n - 0.5) * patch[0]) * kDeg});
            const auto t0 = std::chrono::steady_clock::now();
            const auto all = engine->evaluate(aspects, frequency, settings);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
            double tt = 0, pp = 0, d = 0, cross = 0;
            std::vector<double> samples;
            for (const auto& x : all) {
                tt += std::norm(x.tt) + 0.5 * x.diffuse, pp += std::norm(x.pp) + 0.5 * x.diffuse, d += x.diffuse;
                cross += std::norm(0.5 * (x.tp + x.pt)) + 0.5 * x.diffuse;
                samples.push_back(std::norm(x.tt) + 0.5 * x.diffuse);
            }
            const double k = static_cast<double>(all.size());
            std::sort(samples.begin(), samples.end());
            std::printf("az %.2f el %.2f +-%.2f deg (%d x %d) at %.4g Hz: mean tt %.2f dBsm (%.3g m^2, median %.3g)  pp %.2f dBsm (%.3g m^2)  "
                        "cross %.2f dBsm  cavities %.3g m^2  (%.0f ms)\n",
                        at[0], at[1], patch[0] / 2, n, n, frequency, 10 * std::log10(tt / k), tt / k, samples[samples.size() / 2], 10 * std::log10(pp / k),
                        pp / k, 10 * std::log10(cross / k), d / k, ms);
            return 0;
        }
        const auto t0 = std::chrono::steady_clock::now();
        if (shape.empty()) applyTreatment(settings, chosen(file), treatments, frequency);
        const auto s = engine->evaluate({rcs::Aspect{at[0] * kDeg, at[1] * kDeg}}, frequency, settings)[0];
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        auto db = [](std::complex<double> c) { return 10 * std::log10(std::max(std::norm(c), 1e-30)); };
        if (parts) { // each node's own return there (alone: no shadowing by, or bounces off, the others)
            for (const rcs::ModelPart& p : lastReport.parts) {
                if (!p.triangles) continue;
                rcs::Mesh one;
                one.positions = mesh.positions, one.normals = mesh.normals;
                one.indices.assign(mesh.indices.begin() + static_cast<std::ptrdiff_t>(3 * p.firstTriangle),
                                   mesh.indices.begin() + static_cast<std::ptrdiff_t>(3 * (p.firstTriangle + p.triangles)));
                if (!engine->load(one, &error)) continue;
                const auto x = engine->evaluate({rcs::Aspect{at[0] * kDeg, at[1] * kDeg}}, frequency, settings)[0];
                std::printf("  %-40s %7zu tris  tt %7.2f dBsm  pp %7.2f\n", p.name.c_str(), p.triangles, db(x.tt), db(x.pp));
            }
            engine->load(mesh, &error);
        }
        std::printf("az %.2f el %.2f at %.4g Hz: tt %.2f dBsm (%.4g m^2)  pp %.2f  tp %.2f  pt %.2f  cavities %.3g m^2 (%.1f ms, %llu rays)\n", at[0], at[1],
                    frequency, db(s.tt), std::norm(s.tt), db(s.pp), db(s.tp), db(s.pt), s.diffuse, ms,
                    static_cast<unsigned long long>(engine->raysTraced()));
        return 0;
    }

    for (const std::string& type : targets) {
        rcs::Mesh mesh;
        fs::path file;
        if (!loadTarget(type, mesh, file) || !engine->load(mesh, &error)) {
            std::fprintf(stderr, "flightsim-rcs: %s: %s\n", type.c_str(), error.c_str());
            return 1;
        }
        ms::rcs::Table table;
        table.aircraft = type;
        table.binDeg = binDeg;
        table.azimuthBins = static_cast<int>(std::lround(360.0 / binDeg));
        table.elevationBins = static_cast<int>(std::lround(180.0 / binDeg)) + 1;
        const Treatment chosenTreatment = chosen(file);
        std::printf("features: %s\n", describe(chosenTreatment).c_str());
        const auto t0 = std::chrono::steady_clock::now();
        std::FILE* dump = dumpPath.empty() ? nullptr : std::fopen(dumpPath.c_str(), "wb");
        auto wanted = [&](int e) {
            if (elevationsDeg.empty()) return true;
            for (double d : elevationsDeg)
                if (std::lround((d + 90.0) / binDeg) == e) return true;
            return false;
        };
        // the bands' bins over the rows wanted, for the model the engine holds
        auto computeBands = [&](ms::rcs::Table& into, std::FILE* dumpTo) {
            for (double f : bands) {
                applyTreatment(settings, chosenTreatment, treatments, f);
                ms::rcs::Band band;
                band.frequencyHz = f;
                band.bins.resize(static_cast<std::size_t>(into.azimuthBins * into.elevationBins));
                for (int e = 0; e < into.elevationBins; ++e) {
                    if (!wanted(e)) continue;
                    std::vector<rcs::Aspect> aspects;
                    for (int az = 0; az < into.azimuthBins; ++az)
                        for (int i = 0; i < sub; ++i)
                            for (int j = 0; j < sub; ++j) {
                                const double da = ((i + 0.5) / sub - 0.5) * binDeg, de = ((j + 0.5) / sub - 0.5) * binDeg;
                                const double el = std::clamp(-90.0 + e * binDeg + de, -90.0, 90.0);
                                aspects.push_back({(az * binDeg + da) * kDeg, el * kDeg});
                            }
                    const auto s = engine->evaluate(aspects, f, settings);
                    if (dumpTo)
                        for (std::size_t k = 0; k < s.size(); ++k) { // f, az, el, tt, tp, pt, pp (re, im), diffuse
                            const double row[12] = {f, aspects[k].azimuthRad, aspects[k].elevationRad, s[k].tt.real(), s[k].tt.imag(), s[k].tp.real(),
                                                    s[k].tp.imag(), s[k].pt.real(), s[k].pt.imag(), s[k].pp.real(), s[k].pp.imag(), s[k].diffuse};
                            std::fwrite(row, sizeof row, 1, dumpTo);
                        }
                    const auto n = static_cast<std::size_t>(sub * sub);
                    for (int az = 0; az < into.azimuthBins; ++az) {
                        double m[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}, s2 = 0;
                        for (std::size_t k = 0; k < n; ++k) {
                            const rcs::Scattering& x = s[static_cast<std::size_t>(az) * n + k];
                            // (monostatic: S_pt and S_tp are one, as reciprocity has it - their mean taken)
                            const std::complex<double> tp = 0.5 * (x.tp + x.pt);
                            const std::complex<double> ttpp = x.tt * std::conj(x.pp), tttp = x.tt * std::conj(tp), pptp = x.pp * std::conj(tp);
                            // the cavities' diffuse power, incoherent: half co-polarised, half crossed
                            m[0] += std::norm(x.tt) + 0.5 * x.diffuse, m[1] += std::norm(x.pp) + 0.5 * x.diffuse, m[2] += std::norm(tp) + 0.5 * x.diffuse;
                            m[3] += ttpp.real(), m[4] += ttpp.imag(), m[5] += tttp.real(), m[6] += tttp.imag(), m[7] += pptp.real(), m[8] += pptp.imag();
                            s2 += (std::norm(x.tt) + 0.5 * x.diffuse) * (std::norm(x.tt) + 0.5 * x.diffuse);
                        }
                        ms::rcs::Moments& b = band.bins[static_cast<std::size_t>(e * into.azimuthBins + az)];
                        const double inv = 1.0 / static_cast<double>(n);
                        b.tt = float(m[0] * inv), b.pp = float(m[1] * inv), b.tp = float(m[2] * inv);
                        b.ttppRe = float(m[3] * inv), b.ttppIm = float(m[4] * inv), b.tttpRe = float(m[5] * inv), b.tttpIm = float(m[6] * inv);
                        b.pptpRe = float(m[7] * inv), b.pptpIm = float(m[8] * inv);
                        b.fluctuation = m[0] > 0 ? float(s2 * inv / (m[0] * inv * m[0] * inv)) : 1.0f;
                    }
                    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                    std::printf("\r%s %.3g GHz: elevation %d/%d (%.0f s, %.2f Grays)   ", type.c_str(), f / 1e9, e + 1, into.elevationBins, elapsed,
                                static_cast<double>(engine->raysTraced()) / 1e9);
                    std::fflush(stdout);
                }
                into.bands.push_back(std::move(band));
            }
        };
        computeBands(table, dump);
        if (dump) std::fclose(dump);
        std::printf("\n%s: %.1f s, %.2f Grays (%.2f Grays/s)\n", type.c_str(),
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), static_cast<double>(engine->raysTraced()) / 1e9,
                    static_cast<double>(engine->raysTraced()) / 1e9 / std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        if (verify) {
            // the rows recomputed against the table's: every bin's tt and pp (the moments' co-polarised powers) in dB
            const fs::path tablePath = !outPath.empty() ? fs::path(outPath) : file.parent_path() / "rcs.bin";
            const auto old = ms::rcs::readTable(tablePath, &error);
            if (!old) {
                std::fprintf(stderr, "flightsim-rcs: %s: no table to verify: %s\n", type.c_str(), error.c_str());
                return 1;
            }
            bool same = old->bands.size() == table.bands.size() && old->azimuthBins == table.azimuthBins && old->elevationBins == table.elevationBins;
            for (std::size_t b = 0; same && b < table.bands.size(); ++b) same = old->bands[b].frequencyHz == table.bands[b].frequencyHz;
            if (!same) {
                std::fprintf(stderr, "flightsim-rcs: %s: the table's bands or bins are not these settings'\n", type.c_str());
                return 1;
            }
            // the bins' differences between two computations (dB, every bin's tt and pp where both are above nothing)
            auto differences = [&](const ms::rcs::Table& x, const ms::rcs::Table& y) {
                std::vector<double> d;
                for (std::size_t b = 0; b < x.bands.size(); ++b)
                    for (int e = 0; e < x.elevationBins; ++e) {
                        if (!wanted(e)) continue;
                        for (int az = 0; az < x.azimuthBins; ++az) {
                            const auto k = static_cast<std::size_t>(e * x.azimuthBins + az);
                            const auto& p = x.bands[b].bins[k];
                            const auto& q = y.bands[b].bins[k];
                            for (const auto [u, v] : {std::pair{p.tt, q.tt}, std::pair{p.pp, q.pp}})
                                if (u > 1e-10f && v > 1e-10f) d.push_back(std::abs(10.0 * std::log10(static_cast<double>(u) / static_cast<double>(v))));
                        }
                    }
                std::sort(d.begin(), d.end());
                return d;
            };
            // the method's own noise: the same model moved by a fraction of a ray tube (at the highest band) - its RCS the
            // same, as a translation leaves it, only where the tubes fall on it changed
            rcs::Mesh moved = mesh;
            const double tube = 299792458.0 / *std::max_element(bands.begin(), bands.end()) / settings.raysPerWavelength;
            const double shift[3] = {0.37 * tube, 0.23 * tube, 0.31 * tube};
            for (std::size_t i = 0; i < moved.positions.size(); ++i) moved.positions[i] += static_cast<float>(shift[i % 3]);
            if (!engine->load(moved, &error)) {
                std::fprintf(stderr, "flightsim-rcs: %s\n", error.c_str());
                return 1;
            }
            ms::rcs::Table jittered = table;
            jittered.bands.clear();
            computeBands(jittered, nullptr);
            std::printf("\n");
            const auto noise = differences(table, jittered);
            const auto diffs = differences(table, *old);
            const auto pct = [](const std::vector<double>& d, int p) { return d.empty() ? 0.0 : d[d.size() * static_cast<std::size_t>(p) / 100]; };
            const double worst = diffs.empty() ? 0.0 : diffs.back(), p99 = pct(diffs, 99), p95 = pct(diffs, 95);
            const double noiseWorst = noise.empty() ? 0.0 : noise.back(), noiseP99 = pct(noise, 99), noiseP95 = pct(noise, 95);
            std::printf("%s: %zu bins; against the table: largest %.3f dB, 99 %% within %.3f, 95 %% within %.3f; the method's own noise "
                        "(moved %.2g mm): largest %.3f, 99 %% within %.3f, 95 %% within %.3f; tolerance %.3g dB over it\n",
                        type.c_str(), diffs.size(), worst, p99, p95, 1e3 * tube * 0.37, noiseWorst, noiseP99, noiseP95, toleranceDb);
            // it holds where it differs from the model no more than the model from itself moved
            if (diffs.empty() || p95 > noiseP95 + toleranceDb || p99 > noiseP99 + toleranceDb || worst > noiseWorst + toleranceDb) {
                std::printf("%s: the table does not hold for this model: compute it again\n", type.c_str());
                return 1;
            }
            ms::rcs::Table kept = *old;
            core::Json p;
            try {
                p = core::Json::parse(kept.provenance);
            } catch (const std::exception&) {
                p = core::Json();
            }
            core::Json v, rows, hz;
            char hash[24];
            std::snprintf(hash, sizeof hash, "%016llx", static_cast<unsigned long long>(fnv(file)));
            for (double d : elevationsDeg) rows.push(d);
            for (const auto& band : table.bands) hz.push(band.frequencyHz);
            v.set("model", file.filename().string());
            v.set("model_fnv1a", std::string(hash));
            v.set("elevations_deg", rows);
            v.set("bands_hz", hz);
            v.set("bins", static_cast<double>(diffs.size()));
            v.set("max_db", worst);
            v.set("p99_db", p99);
            v.set("p95_db", p95);
            v.set("noise_max_db", noiseWorst);
            v.set("noise_p99_db", noiseP99);
            v.set("noise_p95_db", noiseP95);
            v.set("noise", "the model moved by a fraction of a ray tube: where the tubes fall on it, which leaves its RCS the same");
            v.set("tolerance_db", toleranceDb);
            v.set("device", engine->deviceName());
            p.set("verified", v);
            kept.provenance = p.dump();
            if (!ms::rcs::writeTable(kept, tablePath, &error)) {
                std::fprintf(stderr, "flightsim-rcs: %s\n", error.c_str());
                return 1;
            }
            std::printf("%s: the table holds; its provenance records the check (%s)\n", type.c_str(), tablePath.string().c_str());
            continue;
        }
        char provenance[2048];
        std::snprintf(provenance, sizeof provenance,
                      "{\"method\":\"shooting and bouncing rays, physical optics at every bounce, perfect conductor (tools/rcs)\","
                      "\"model\":\"%s\",\"model_fnv1a\":\"%016llx\",\"rays_per_wavelength\":%g,\"max_bounces\":%d,\"phong_alpha\":%g,"
                      "\"sub_aspects\":%d,\"features\":\"%s\",\"treatments\":{\"ram_full_db\":%g,\"ram_partial_db\":%g,\"ram_low_hz\":%g,\"intake_concealed_db\":%g},\"cavity\":\"diffuse, 4 rho A\",\"device\":\"%s\"}",
                      file.filename().string().c_str(), static_cast<unsigned long long>(fnv(file)), settings.raysPerWavelength, settings.maxBounces,
                      settings.phongAlpha, sub * sub, describe(chosenTreatment).c_str(), treatments.ramFullDb, treatments.ramPartialDb, treatments.ramLowHz,
                      treatments.intakeConcealedDb, engine->deviceName().c_str());
        table.provenance = provenance;
        const fs::path out = !outPath.empty() ? fs::path(outPath) : file.parent_path() / "rcs.bin";
        if (!ms::rcs::writeTable(table, out, &error)) {
            std::fprintf(stderr, "flightsim-rcs: %s\n", error.c_str());
            return 1;
        }
        std::printf("%s\n", out.string().c_str());
    }
    return 0;
}
