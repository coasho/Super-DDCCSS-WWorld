// fsim_radar_bench: a radar's cost and what it detected (ADR-30, MS-4..MS-6), for A/B
// measurements of the radar's code (docs/sdk/mission-systems.md, Signal level).
//
//   fsim_radar_bench [type] [level] [seconds] [altitude_m]
//       type      the carrier (default f16c; e3g: its rotodome)
//       level     signal (default) or detection
//       seconds   simulated (default 60)
//       altitude  above the ellipsoid (default 600 m: the testbench's)
//
// The testbench's case: the carrier over San Francisco Bay off SFO, heading
// south-east across it, on the physics terrain from the local map set (never
// fetched), a C172 crossing ahead of it. Its radar searches as the testbench
// commands it - a fighter's whole field of regard in range-while-search, 4
// bars; a rotodome's 360 deg in surveillance, 2 bars - to its instrumented
// range. Printed: the wall time of the run (and of a run with the radar off,
// the rest of the world's cost), the signal level's split between the
// ground's clutter map and the dwell's processing, and a digest of every
// detection (FNV-1a over each one's fields, bit for bit): two builds detect
// alike when their digests are equal.

#include "simulation/mission_systems/Host.h"
#include "simulation/mission_systems/radar/Radar.h"
#include "simulation/runtime/world/World.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using namespace fsim;

namespace {

struct Digest {
    std::uint64_t h = 1469598103934665603ull;
    template <class T> void add(const T& v) {
        unsigned char b[sizeof(T)];
        std::memcpy(b, &v, sizeof(T));
        for (unsigned char c : b) h = (h ^ c) * 1099511628211ull;
    }
};

struct Result {
    double wallS = 0.0;
    std::uint64_t detections = 0, digest = 0;
    ms::radar::signal::Stats stats;
    std::uint64_t dwells = 0;
};

Result run(const std::string& type, int level, double seconds, double altitudeM) {
    session::WorldOptions o;
    o.name = "radar-bench-" + type + std::to_string(level);
    o.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.seed = 7;
    o.terrain = true;
    o.terrainOffline = true;
    session::World w(o);
    session::VehicleSpec own;
    own.name = "own";
    own.type = "jsbsim:" + type;
    own.initial.latitudeDeg = 37.62, own.initial.longitudeDeg = -122.36;
    own.initial.altitudeMslM = altitudeM;
    own.initial.headingDeg = 140.0;
    own.initial.airspeedTrueMs = 150.0;
    const std::uint32_t id = w.createVehicle(own);
    session::VehicleSpec other;
    other.name = "other";
    other.type = "jsbsim:c172";
    other.initial.latitudeDeg = 37.58, other.initial.longitudeDeg = -122.30;
    other.initial.altitudeMslM = altitudeM;
    other.initial.headingDeg = 20.0;
    other.initial.airspeedTrueMs = 50.0;
    w.createVehicle(other);
    if (id == 0) {
        std::fprintf(stderr, "could not create a %s\n", type.c_str());
        return {};
    }
    const ms::Host* host = w.missionSystems(id);
    std::uint16_t payload = 0;
    const ms::radar::Radar* radar = nullptr;
    for (std::uint16_t p = 0; host && p < 16 && !radar; ++p)
        if ((radar = host->radar(p))) payload = p;
    if (!radar) {
        std::fprintf(stderr, "a %s has no air search radar\n", type.c_str());
        return {};
    }
    if (level > 0) {
        w.submit(id, ms::StateCommand{payload, ms::PayloadState::Operate});
        const ms::radar::Spec& spec = radar->spec();
        ms::AmtiCommand c;
        c.payload = payload;
        const bool rotodome = spec.rotationRpm > 0.0;
        c.mode = rotodome ? ms::AmtiMode::Surveillance : ms::AmtiMode::RangeWhileSearch;
        c.azimuthHalfWidthRad = rotodome ? 3.14159265358979323846 : spec.azimuthLimitDeg * 3.14159265358979323846 / 180.0;
        c.bars = rotodome ? 2 : 4;
        c.rangeScaleM = spec.instrumentedRangeM;
        c.signalLevel = level == 2;
        const auto r = w.submit(id, c);
        if (r.status == control::CommandStatus::Rejected) {
            std::fprintf(stderr, "the search was refused\n");
            return {};
        }
    }
    const unsigned steps = static_cast<unsigned>(std::lround(seconds / (w.dt() * w.frameSkip())));
    w.step(120); // (settled: its tables built, the terrain loaded)
    const auto t0 = std::chrono::steady_clock::now();
    w.step(steps);
    Result out;
    out.wallS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    Digest d;
    for (const auto& det : w.amtiDetections(id, payload)) {
        d.add(det.serial), d.add(det.timeS), d.add(det.truth), d.add(det.rangeM), d.add(det.azimuthRad), d.add(det.elevationRad);
        d.add(det.rangeRateMs), d.add(det.snrDb);
        ++out.detections;
    }
    for (const auto& t : w.amtiTracks(id, payload)) d.add(t.id), d.add(t.updates), d.add(t.truth), d.add(t.latitudeRad), d.add(t.longitudeRad);
    out.digest = d.h;
    out.stats = radar->signalStats();
    out.dwells = radar->dwells();
    return out;
}

} // namespace

int main(int argc, char** argv) {
    const std::string type = argc > 1 ? argv[1] : "f16c";
    const std::string levelName = argc > 2 ? argv[2] : "signal";
    const double seconds = argc > 3 ? std::atof(argv[3]) : 60.0;
    const double altitudeM = argc > 4 ? std::atof(argv[4]) : 600.0;
    const int level = levelName == "detection" ? 1 : levelName == "off" ? 0 : 2;
    const Result off = run(type, 0, seconds, altitudeM);
    const Result r = run(type, level, seconds, altitudeM);
    std::printf("%s %s, %.0f s simulated at %.0f m\n", type.c_str(), levelName.c_str(), seconds, altitudeM);
    std::printf("  wall %.3f s (radar off %.3f s): the radar %.1f %% of real time\n", r.wallS, off.wallS, 100.0 * (r.wallS - off.wallS) / seconds);
    std::printf("  dwells %llu, %.3f ms each\n", static_cast<unsigned long long>(r.dwells),
                r.dwells ? 1e3 * (r.wallS - off.wallS) / static_cast<double>(r.dwells) : 0.0);
    if (level == 2)
        std::printf("  clutter map %.3f s, dwell processing %.3f s (cells %llu, peaks %llu, hits %llu)\n", r.stats.clutterS, r.stats.processS,
                    static_cast<unsigned long long>(r.stats.cells), static_cast<unsigned long long>(r.stats.peaks),
                    static_cast<unsigned long long>(r.stats.hits));
    std::printf("  detections %llu, digest %016llx\n", static_cast<unsigned long long>(r.detections), static_cast<unsigned long long>(r.digest));
    return 0;
}
