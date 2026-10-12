// flightsim-showcase: a scripted recording for a store's model, moving parts and rendering effects
// (ADR-30, MS-7e: docs/mission-systems.md). The carrier and missile motion is kinematic: level carriage, release,
// motor phases and commanded fin deflections are written directly into the recording's vehicle states. The viewer
// plays those states back through its normal rendering path (flightsim-viewer --replay). An optional target aircraft
// is flown by World through its JSBSim flight model and damage chain. This fixture checks rendering from supplied
// states; it does not validate native missile flight, guidance, seekers or fuze outcomes.
//
//   flightsim-showcase <script.json> <out.fsrec>
//
// The script (written by `fsim hangar showcase <missile>` from the design):
//   { "name": ..., "duration_s": 30, "rate_hz": 60,
//     "start": { "lat_deg", "lon_deg", "alt_m", "heading_deg" },
//     "carrier": { "type": "jsbsim:f16c", "name", "speed_ms", "thrust_n", "n2" },
//     "missiles": [ { "type": "jsbsim:aim120c", "card": "aim-120c", "station_number": 9, "name",
//                     "station": [x fwd, y right, z down] (m from the carrier's CG),
//                     "launch_s", "launch": "rail" | "eject", "eject_ms", "ignition_delay_s", "mass_kg",
//                     "diameter_m", "cd", "phases": [[duration_s, thrust_n], ...], "fin_deg", "bay": false,
//                     "dive_s", "dive_deg" (optional: from dive_s after its launch it pitches down to dive_deg),
//                     "detonate_s", "explosive_kg", "warhead_kg" (optional: detonate_s after its launch its warhead
//                     goes off - an engagement event, MS-7h's detonation, as MS-7d's fuze will write it - and it ends
//                     there, as World ends a weapon) } ],
//     "target": { "type", "name", "speed_ms", "heading_deg", "miss_m": [x, y, z] (where the first detonation is
//                 from it, NED m), "seed" }
//                (optional: an aircraft flown by its own flight model - a World's vehicle, under its autopilot - on
//                 the line through the first missile's detonation, found in a first pass; at it the world's
//                 detonation (MS-7g's damage chain: World::detonate) breaks what its fragments and blast reach, and
//                 the damaged aircraft flies on as the damage makes it - its fires, its sections parting, their
//                 pieces falling - the world's events and pieces written as it writes them),
//     "ground": false (optional: true, the ground under the missiles, from the map set's terrain - the tiles the
//                      physics stands on - ends each where it strikes it, as World ends a weapon that does: its
//                      vehicle removed that step, no state of it at the ground),
//     "environment": { "humidity" (0..1, over water), "temperature_sea_level_k", "pressure_sea_level_pa",
//                      "wind_from_deg", "wind_ms", "cloud_cover", "cloud_base_m" } }   (optional: the world's air and
//                      weather the replay draws its contrails, smoke and clouds in; none, the viewer's defaults)
// A missile in a weapon bay ("bay": true) has its bay's doors open over the half second before its launch, held open a
// second after it, shut over eight tenths - the store management's sequence (src/ms/HostStores.cpp).

#include "core/Json.h"
#include "io/TerrainTiles.h"
#include "ipc/Recording.h"
#include "session/World.h"
#include "sim/Attitude.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

using fsim::core::Json;
constexpr double kPi = 3.14159265358979323846;
constexpr double kA = 6378137.0, kF = 1.0 / 298.257223563, kE2 = kF * (2.0 - kF);
constexpr double kG = 9.80665;

struct Vec {
    double x = 0, y = 0, z = 0;
};
Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec operator*(double s, Vec a) { return {s * a.x, s * a.y, s * a.z}; }
double norm(Vec a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

/// Body -> NED (columns: the body axes in NED) from roll, pitch, yaw.
void bodyToNed(double roll, double pitch, double yaw, double m[3][3]) {
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw), sy = std::sin(yaw);
    m[0][0] = cp * cy, m[0][1] = sr * sp * cy - cr * sy, m[0][2] = cr * sp * cy + sr * sy;
    m[1][0] = cp * sy, m[1][1] = sr * sp * sy + cr * cy, m[1][2] = cr * sp * sy - sr * cy;
    m[2][0] = -sp, m[2][1] = sr * cp, m[2][2] = cr * cp;
}

/// The ECEF -> body quaternion (w, x, y, z; sim/Attitude.h's convention) of a body -> ECEF matrix R: its transpose,
/// C = R^T, is the matrix bodyToEcefFromQuaternion's m (Shepperd's method, stable for any attitude).
void quaternionFromBodyToEcef(const double R[3][3], double q[4]) {
    double m[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m[i][j] = R[j][i];
    const double tr = m[0][0] + m[1][1] + m[2][2];
    double w, x, y, z;
    if (tr > 0.0) {
        const double s = 2.0 * std::sqrt(1.0 + tr);
        w = 0.25 * s, x = (m[1][2] - m[2][1]) / s, y = (m[2][0] - m[0][2]) / s, z = (m[0][1] - m[1][0]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
        x = 0.25 * s, w = (m[1][2] - m[2][1]) / s, y = (m[0][1] + m[1][0]) / s, z = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
        y = 0.25 * s, w = (m[2][0] - m[0][2]) / s, x = (m[0][1] + m[1][0]) / s, z = (m[1][2] + m[2][1]) / s;
    } else {
        const double s = 2.0 * std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
        z = 0.25 * s, w = (m[0][1] - m[1][0]) / s, x = (m[0][2] + m[2][0]) / s, y = (m[1][2] + m[2][1]) / s;
    }
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    q[0] = w / n, q[1] = x / n, q[2] = y / n, q[3] = z / n;
}

/// The ISA troposphere and lower stratosphere: temperature (K) and density (kg/m3) at an altitude.
void isa(double h, double& T, double& rho) {
    if (h < 11000.0) {
        T = 288.15 - 0.0065 * h;
        rho = 1.225 * std::pow(T / 288.15, 4.2559);
    } else {
        T = 216.65;
        rho = 0.36392 * std::exp(-(h - 11000.0) / 6341.62);
    }
}

/// A point of the scripted motion: where (NED from the start), how fast (NED), its attitude.
struct Pose {
    Vec ned, vel;
    double roll = 0, pitch = 0, yaw = 0;
};

class Earth {
public:
    Earth(double latDeg, double lonDeg, double altM) : lat0_(latDeg * kPi / 180.0), lon0_(lonDeg * kPi / 180.0), h0_(altM) {
        const double s = std::sin(lat0_);
        const double w = std::sqrt(1.0 - kE2 * s * s);
        rn_ = kA / w;
        rm_ = kA * (1.0 - kE2) / (w * w * w);
    }
    /// The state a flight model would write for this pose.
    void fill(const Pose& p, fsim::sim::VehicleState& st) const {
        const double lat = lat0_ + p.ned.x / (rm_ + h0_);
        const double lon = lon0_ + p.ned.y / ((rn_ + h0_) * std::cos(lat0_));
        const double h = h0_ - p.ned.z;
        const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
        const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
        st.positionEcef[0] = (n + h) * cl * co, st.positionEcef[1] = (n + h) * cl * so;
        st.positionEcef[2] = (n * (1.0 - kE2) + h) * sl;
        st.latitudeRad = lat, st.longitudeRad = lon, st.altitudeMslM = h, st.altitudeAglM = h;
        const double en[3][3] = {{-sl * co, -so, -cl * co}, {-sl * so, co, -cl * so}, {cl, 0.0, -sl}}; // NED -> ECEF
        double bn[3][3];
        bodyToNed(p.roll, p.pitch, p.yaw, bn);
        double R[3][3] = {};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) R[i][j] += en[i][k] * bn[k][j];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) st.rotationBodyToEcef[i * 3 + j] = R[i][j];
        quaternionFromBodyToEcef(R, st.attitudeEcefToBody);
        st.eulerRad[0] = p.roll, st.eulerRad[1] = p.pitch, st.eulerRad[2] = p.yaw;
        st.velocityNedMs[0] = p.vel.x, st.velocityNedMs[1] = p.vel.y, st.velocityNedMs[2] = p.vel.z;
        const double v[3] = {p.vel.x, p.vel.y, p.vel.z};
        for (int j = 0; j < 3; ++j) st.velocityBodyMs[j] = bn[0][j] * v[0] + bn[1][j] * v[1] + bn[2][j] * v[2];
        double T, rho;
        isa(h, T, rho);
        st.airspeedTrueMs = norm(p.vel);
        st.airspeedCalibratedMs = st.airspeedTrueMs * std::sqrt(rho / 1.225);
        st.mach = st.airspeedTrueMs / std::sqrt(1.4 * 287.05 * T);
        st.loadFactor = 1.0;
    }

    /// Where a state's place is, from the start (NED, m: the inverse of fill's).
    Vec ned(const fsim::sim::VehicleState& st) const {
        return {(st.latitudeRad - lat0_) * (rm_ + h0_), (st.longitudeRad - lon0_) * (rn_ + h0_) * std::cos(lat0_), h0_ - st.altitudeMslM};
    }

private:
    double lat0_, lon0_, h0_, rn_, rm_;
};

double smoothstep(double a, double b, double t) {
    const double x = std::clamp((t - a) / (b - a), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

struct Missile {
    std::string type, name, launch, card;
    int stationNumber = 1;
    bool bay = false;
    /// its bay's doors at a time (0 shut .. 1 open), as the store management opens them round its release
    double doors(double t) const {
        if (!bay) return 0.0;
        constexpr double open = 0.5, hold = 1.0, shut = 0.8;
        if (t < launchS - open) return 0.0;
        if (t < launchS) return (t - (launchS - open)) / open;
        if (t < launchS + hold) return 1.0;
        return std::max(0.0, 1.0 - (t - launchS - hold) / shut);
    }
    Vec station;
    double launchS = 5.0, ejectMs = 0.0, ignitionDelayS = 0.0, massKg = 100.0, diameterM = 0.2, cd = 0.35, finDeg = 15.0;
    double diveS = -1.0, diveDeg = -30.0; ///< (diveS < 0: no dive)
    double detonateS = -1.0, explosiveKg = 0.0, warheadKg = 0.0; ///< (detonateS < 0: none)
    bool ended = false;                    ///< struck the ground: its vehicle removed
    std::vector<std::pair<double, double>> phases; ///< (duration s, thrust N)
    // the free flight, from its release
    bool released = false;
    Pose pose;
    double speed = 0.0;
    double thrust(double sinceIgnition) const {
        if (sinceIgnition < 0.0) return 0.0;
        double t = 0.0;
        for (const auto& [d, f] : phases) {
            if (sinceIgnition < t + d) return f;
            t += d;
        }
        return 0.0;
    }
};

void fail(const std::string& what) {
    std::fprintf(stderr, "flightsim-showcase: %s\n", what.c_str());
    std::exit(1);
}

} // namespace

namespace {

struct Pass {
    bool detonated = false;
    double time = 0.0;
    Vec ned;         ///< where the first missile with a detonation went off
    bool targetSeen = false;
    Vec targetError; ///< a target's place then less the line's (its flight model's own way from it)
};

} // namespace

static int run(const Json& script, const char* outPath, Pass& pass, const Pass* first, Vec correction = {}, bool trial = false) {
    const double rate = script.number("rate_hz", 60.0), duration = script.number("duration_s", 30.0);
    const Json& start = script.child("start");
    const Earth earth(start.number("lat_deg", 36.0), start.number("lon_deg", -115.0), start.number("alt_m", 6000.0));
    const double heading = start.number("heading_deg", 90.0) * kPi / 180.0;
    const Json& c = script.child("carrier");
    const double carrierSpeed = c.number("speed_ms", 230.0);
    std::vector<Missile> missiles;
    for (const Json& m : script.child("missiles").asArray()) {
        Missile s;
        s.type = m.string("type", ""), s.name = m.string("name", s.type), s.launch = m.string("launch", "rail");
        s.card = m.string("card", ""), s.stationNumber = static_cast<int>(m.number("station_number", 1.0));
        s.bay = m.boolean("bay", false);
        const auto& st = m.child("station").asArray();
        if (st.size() != 3) fail("a missile's station is [x, y, z]");
        s.station = {st[0].asNumber(), st[1].asNumber(), st[2].asNumber()};
        s.launchS = m.number("launch_s", 5.0), s.ejectMs = m.number("eject_ms", 0.0);
        s.ignitionDelayS = m.number("ignition_delay_s", 0.0), s.massKg = m.number("mass_kg", 100.0);
        s.diameterM = m.number("diameter_m", 0.2), s.cd = m.number("cd", 0.35), s.finDeg = m.number("fin_deg", 15.0);
        s.diveS = m.number("dive_s", -1.0), s.diveDeg = m.number("dive_deg", -30.0);
        s.detonateS = m.number("detonate_s", -1.0), s.explosiveKg = m.number("explosive_kg", 0.0);
        s.warheadKg = m.number("warhead_kg", 0.0);
        for (const Json& p : m.child("phases").asArray()) s.phases.emplace_back(p.asArray().at(0).asNumber(), p.asArray().at(1).asNumber());
        if (s.type.empty()) fail("a missile needs its type (jsbsim:<design>)");
        missiles.push_back(std::move(s));
    }

    // the target: flown through the first detonation (found in the first pass), its fires after it
    const bool hasTarget = script.has("target") && first && first->detonated;
    const Json& tj = script.has("target") ? script.child("target") : script;
    const std::uint32_t targetSlot = static_cast<std::uint32_t>(1 + missiles.size());
    Pose target;
    if (hasTarget) {
        const double h = tj.number("heading_deg", 270.0) * kPi / 180.0, v = tj.number("speed_ms", 220.0);
        target.yaw = h;
        target.vel = {v * std::cos(h), v * std::sin(h), 0.0};
        Vec miss;
        if (tj.has("miss_m")) {
            const auto& mm = tj.child("miss_m").asArray();
            if (mm.size() == 3) miss = {mm[0].asNumber(), mm[1].asNumber(), mm[2].asNumber()};
        }
        target.ned = first->ned + (-1.0) * miss; // (at the detonation's time: the miss is the burst from the target)
    }
    const std::uint32_t count = static_cast<std::uint32_t>(1 + missiles.size() + (hasTarget ? 1 : 0));
    // (a target's pieces, as they part, in slots of their own above the vehicles': the world's, counted down)
    constexpr std::uint32_t kPieceSlots = 8;
    fsim::ipc::RecordingHeader header;
    header.capacity = count + (hasTarget ? kPieceSlots : 0), header.dt = 1.0 / rate, header.frameSkip = 1;
    std::snprintf(header.name, sizeof(header.name), "%s", script.string("name", "showcase").c_str());
    fsim::ipc::RecordWriter out;
    if (!out.open(outPath, header)) fail(std::string("cannot write ") + outPath);
    if (script.has("environment")) {
        // the air and the weather as a World records its environment (in force from the first snapshot)
        const Json& e = script.child("environment");
        fsim::sim::EnvironmentState env;
        env.humidity = std::clamp(e.number("humidity", env.humidity), 0.0, 1.0);
        env.temperatureSeaLevelK = e.number("temperature_sea_level_k", env.temperatureSeaLevelK);
        env.pressureSeaLevelPa = e.number("pressure_sea_level_pa", env.pressureSeaLevelPa);
        env.windDirectionDeg = e.number("wind_from_deg", env.windDirectionDeg);
        env.windSpeedMs = e.number("wind_ms", env.windSpeedMs);
        env.cloudCover = std::clamp(e.number("cloud_cover", env.cloudCover), 0.0, 1.0);
        env.cloudBaseM = e.number("cloud_base_m", env.cloudBaseM);
        env.revision = 1;
        out.writeEnvironment(env);
    }
    std::unique_ptr<fsim::io::TerrainTiles> ground; // (offline: the map set's tiles, their nearest level where one is missing)
    if (script.boolean("ground", false)) ground = std::make_unique<fsim::io::TerrainTiles>(fsim::io::TerrainTiles::Options{});
    auto row = [&](std::uint32_t slot, const std::string& name, const std::string& type, bool alive = true) {
        fsim::ipc::RecordedVehicle r;
        r.id = slot + 1, r.generation = 1, r.alive = alive ? 1 : 0;
        std::snprintf(r.name, sizeof(r.name), "%s", name.c_str());
        std::snprintf(r.type, sizeof(r.type), "%s", type.c_str());
        out.writeVehicle(slot, r);
    };
    row(0, c.string("name", "carrier"), c.string("type", "jsbsim:f16c"));
    if (hasTarget) row(targetSlot, tj.string("name", "target"), tj.string("type", "jsbsim:mig29a"));
    // the target's world: it flies its own flight model, one world step a frame, from where the line through the
    // detonation has it at the start
    std::unique_ptr<fsim::session::World> world;
    std::uint32_t targetId = 0, weaponSlot = 0;
    struct Separation {
        std::string section;
        double at = 0.0;
        bool done = false;
    };
    std::vector<Separation> separations; // (a scenario's injected structural failures, after the burst)
    if (hasTarget && !trial && tj.has("separate"))
        for (const Json& s : tj.child("separate").asArray())
            separations.push_back({s.string("section", ""), first->time + s.number("after_s", 0.05), false});
    std::vector<std::uint32_t> piecesWritten;
    if (hasTarget) {
        fsim::session::WorldOptions o;
        o.name = "showcase", o.workers = 1, o.pinWorkers = false, o.publish = false;
        o.seed = static_cast<std::uint64_t>(tj.number("seed", 1.0));
        o.frameSkip = std::max(1, static_cast<int>(std::lround(1.0 / (rate * o.dt))));
        o.capacity = header.capacity;
        world = std::make_unique<fsim::session::World>(o);
        Pose p0 = target;
        p0.ned = target.ned + (0.0 - first->time) * target.vel + correction; // (its start moved by what its flight strays)
        fsim::ipc::VehicleSample s0{};
        earth.fill(p0, s0.state);
        fsim::session::VehicleSpec vs;
        vs.name = tj.string("name", "target");
        vs.type = tj.string("type", "jsbsim:mig29a");
        vs.initial.latitudeDeg = s0.state.latitudeRad * 180.0 / kPi;
        vs.initial.longitudeDeg = s0.state.longitudeRad * 180.0 / kPi;
        vs.initial.altitudeMslM = s0.state.altitudeMslM;
        vs.initial.headingDeg = target.yaw * 180.0 / kPi;
        vs.initial.airspeedTrueMs = norm(target.vel);
        targetId = world->createVehicle(vs);
        if (!targetId) fail("the target's flight model would not load: " + vs.type);
    }
    // the world's events in this recording's slots: its vehicle the target, its pieces where it put them
    auto worldEvents = [&] {
        for (auto ev : world->takeEngagementEvents()) {
            const auto kind = static_cast<fsim::ipc::EngagementKind>(ev.kind);
            if (kind == fsim::ipc::EngagementKind::Detonation) {
                ev.slot = weaponSlot;
                if (ev.target) ev.target = targetSlot + 1;
            } else {
                ev.slot = targetSlot;
                if (kind == fsim::ipc::EngagementKind::Hit) ev.target = weaponSlot + 1;
            }
            out.writeEvent(ev);
            if (kind != fsim::ipc::EngagementKind::PanelTorn)
                std::printf("  %.2f s: %s %s%s\n", ev.simTime,
                            kind == fsim::ipc::EngagementKind::Detonation ? "detonation" : kind == fsim::ipc::EngagementKind::Hit ? "fragments struck"
                            : kind == fsim::ipc::EngagementKind::Fire ? "fire" : kind == fsim::ipc::EngagementKind::EngineOut ? "engine out"
                            : kind == fsim::ipc::EngagementKind::ControlLost ? "control lost" : kind == fsim::ipc::EngagementKind::CrewLost ? "crew lost"
                            : kind == fsim::ipc::EngagementKind::Separated ? "separated" : "event",
                            ev.name, kind == fsim::ipc::EngagementKind::Fire ? (" (" + std::to_string(ev.rateKgS) + " kg/s, " +
                                                                                std::to_string(ev.amountKg) + " kg at once)").c_str() : "");
        }
    };
    // the carrier's stores: each missile on its station until it is launched, then released
    auto stores = [&](double t) {
        std::vector<fsim::ipc::MissionStore> list;
        for (const auto& m : missiles) {
            fsim::ipc::MissionStore st;
            st.station = static_cast<std::uint8_t>(m.stationNumber);
            st.status = static_cast<std::uint8_t>(t < m.launchS ? 0 : 4); // ms::StoreStatus Loaded, Released
            st.bayDoors = static_cast<std::uint8_t>(std::lround(255.0 * m.doors(t)));
            std::snprintf(st.mnemonic, sizeof st.mnemonic, "%s", m.name.c_str());
            std::snprintf(st.store, sizeof st.store, "%s", m.card.c_str());
            list.push_back(st);
        }
        return list;
    };
    out.writeStores(0, stores(0.0));
    std::vector<unsigned char> launched(missiles.size(), 0);
    std::vector<std::uint8_t> doorsWritten(missiles.size(), 0);

    double worst = 0.0; // the quaternion's round trip through the viewer's own conversion
    const int frames = static_cast<int>(std::lround(duration * rate));
    const double dt = 1.0 / rate;
    for (int k = 0; k <= frames; ++k) {
        const double t = k * dt;
        std::vector<std::pair<std::uint32_t, fsim::ipc::VehicleSample>> samples;
        // the carrier: level, straight, at its speed
        Pose carrier;
        carrier.yaw = heading;
        carrier.vel = {carrierSpeed * std::cos(heading), carrierSpeed * std::sin(heading), 0.0};
        carrier.ned = t * carrier.vel;
        fsim::ipc::VehicleSample cs{};
        earth.fill(carrier, cs.state);
        cs.state.simTime = t, cs.state.stepCount = static_cast<std::uint32_t>(k), cs.state.engineCount = 1;
        cs.state.gearPosition = 0.0;
        cs.state.thrustN[0] = c.number("thrust_n", 50000.0), cs.state.engineN2[0] = c.number("n2", 92.0);
        cs.state.throttlePosition[0] = cs.inputs.throttle[0] = 0.8;
        cs.inputs.gearDown = 0.0;
        samples.emplace_back(0u, cs);
        if (hasTarget) {
            // the target: its world a step on (one a frame), its state as its flight model has it, its pieces'
            if (k > 0) world->step(1);
            for (auto& s : separations)
                if (!s.done && t >= s.at) {
                    s.done = true;
                    if (!world->separateSection(targetId, s.section)) std::printf("  (no %s to fail)\n", s.section.c_str());
                }
            fsim::ipc::VehicleSample ts{};
            ts.state = *world->vehicleState(targetId);
            ts.inputs = *world->inputs(targetId);
            ts.state.simTime = t, ts.state.stepCount = static_cast<std::uint32_t>(k);
            samples.emplace_back(targetSlot, ts);
            if (!pass.targetSeen && t >= first->time - 1e-9) {
                pass.targetSeen = true;
                pass.targetError = earth.ned(ts.state) + (-1.0) * target.ned;
            }
            for (const std::uint32_t id : world->vehicleIds()) {
                const auto* info = world->info(id);
                if (!info || info->role != fsim::ms::Role::Debris) continue;
                // a piece that left it: its own flight model's, in a slot of its own after the vehicles'
                auto at = std::find(piecesWritten.begin(), piecesWritten.end(), id);
                if (at == piecesWritten.end()) {
                    if (piecesWritten.size() >= kPieceSlots) continue;
                    piecesWritten.push_back(id);
                    at = piecesWritten.end() - 1;
                    row(count + static_cast<std::uint32_t>(at - piecesWritten.begin()), info->name, info->type);
                }
                fsim::ipc::VehicleSample ds{};
                ds.state = *world->vehicleState(id);
                ds.state.simTime = t, ds.state.stepCount = static_cast<std::uint32_t>(k);
                samples.emplace_back(count + static_cast<std::uint32_t>(at - piecesWritten.begin()), ds);
            }
            worldEvents();
        }
        double bn[3][3];
        bodyToNed(carrier.roll, carrier.pitch, carrier.yaw, bn);
        for (std::size_t i = 0; i < missiles.size(); ++i) {
            Missile& m = missiles[i];
            const auto doors = static_cast<std::uint8_t>(std::lround(255.0 * m.doors(t)));
            if (t >= m.launchS && !launched[i]) {
                // released: off its station, a vehicle of its own from here
                launched[i] = 1;
                row(static_cast<std::uint32_t>(i + 1), m.name, m.type);
                out.writeStores(0, stores(t));
                doorsWritten[i] = doors;
            } else if (doors != doorsWritten[i]) { // (its bay's doors moving)
                out.writeStores(0, stores(t));
                doorsWritten[i] = doors;
            }
            fsim::ipc::VehicleSample ms{};
            ms.state.engineCount = 1;
            if (t < m.launchS) {
                // carried: on its station, moving with the carrier
                Pose p = carrier;
                const double o[3] = {m.station.x, m.station.y, m.station.z};
                p.ned = carrier.ned + Vec{bn[0][0] * o[0] + bn[0][1] * o[1] + bn[0][2] * o[2],
                                          bn[1][0] * o[0] + bn[1][1] * o[1] + bn[1][2] * o[2],
                                          bn[2][0] * o[0] + bn[2][1] * o[1] + bn[2][2] * o[2]};
                m.pose = p;
                m.speed = carrierSpeed;
                if (m.launch == "eject") m.pose.vel = m.pose.vel + Vec{0.0, 0.0, m.ejectMs}; // the ejector's push, at release
            } else {
                // free: the motor's thrust along its axis, its drag, gravity; a scripted manoeuvre - pull up, then a
                // turn - that the fins are deflected for (an autopilot's commands, not a flight model's response)
                const double since = t - m.launchS, burn = since - m.ignitionDelayS;
                double T, rho;
                isa(start.number("alt_m", 6000.0) - m.pose.ned.z, T, rho);
                const double area = 0.25 * kPi * m.diameterM * m.diameterM;
                const double drag = 0.5 * rho * m.speed * m.speed * m.cd * area;
                m.speed = std::max(0.0, m.speed + dt * (m.thrust(burn) - drag) / m.massKg);
                double pitchCmd = 6.0 * kPi / 180.0 * smoothstep(0.8, 2.8, since);
                if (m.diveS >= 0.0) pitchCmd += (m.diveDeg * kPi / 180.0 - pitchCmd) * smoothstep(m.diveS, m.diveS + 2.0, since);
                const double yawCmd = heading + 12.0 * kPi / 180.0 * smoothstep(4.0, 7.0, since);
                const double pitchRate = (pitchCmd - m.pose.pitch) / dt, yawRate = (yawCmd - m.pose.yaw) / dt;
                m.pose.pitch = pitchCmd, m.pose.yaw = yawCmd;
                // before the motor lights, an ejected missile falls clear; then it flies along its axis
                Vec along{std::cos(m.pose.pitch) * std::cos(m.pose.yaw), std::cos(m.pose.pitch) * std::sin(m.pose.yaw), -std::sin(m.pose.pitch)};
                if (burn < 0.0) m.pose.vel = m.pose.vel + Vec{0.0, 0.0, kG * dt};
                else m.pose.vel = m.speed * along + Vec{0.0, 0.0, std::max(0.0, m.pose.vel.z - m.speed * along.z) * std::exp(-4.0 * dt)};
                m.pose.ned = m.pose.ned + dt * m.pose.vel;
                const double limit = m.finDeg * kPi / 180.0;
                // the fins as the turn rates ask for them (JSBSim's signs: elevator + nose down, rudder + nose left)
                ms.state.elevatorRad = std::clamp(-pitchRate * 0.9, -limit, limit);
                ms.state.rudderRad = std::clamp(-yawRate * 0.9, -limit, limit);
                ms.state.aileronRad = std::clamp(0.25 * limit * std::sin(2.0 * kPi * 0.7 * since) * smoothstep(7.5, 8.5, since) *
                                                     (1.0 - smoothstep(10.0, 11.0, since)), -limit, limit);
                ms.state.thrustN[0] = m.thrust(burn);
                ms.inputs.throttle[0] = ms.state.throttlePosition[0] = ms.state.thrustN[0] > 0.0 ? 1.0 : 0.0;
            }
            if (m.ended) continue;
            earth.fill(m.pose, ms.state);
            ms.state.simTime = t, ms.state.stepCount = static_cast<std::uint32_t>(k);
            ms.inputs.elevator = ms.state.elevatorRad, ms.inputs.rudder = ms.state.rudderRad, ms.inputs.aileron = ms.state.aileronRad;
            const std::optional<double> under =
                launched[i] && ground ? ground->knownHeightAboveEllipsoidM(ms.state.latitudeRad, ms.state.longitudeRad) : std::nullopt;
            if (launched[i] && ground && !under) fail("no terrain under the missile in the map set (fetch-maps)");
            if (launched[i] && m.detonateS >= 0.0 && t - m.launchS >= m.detonateS) {
                // its warhead goes off: the event where it is, moving as it was; it ends there
                fsim::ipc::EngagementEvent e;
                e.kind = static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Detonation);
                e.slot = static_cast<std::uint32_t>(i + 1);
                e.simTime = t;
                const double sl = std::sin(ms.state.latitudeRad), cl = std::cos(ms.state.latitudeRad);
                const double so = std::sin(ms.state.longitudeRad), co = std::cos(ms.state.longitudeRad);
                const double* v = ms.state.velocityNedMs; // NED -> ECEF
                e.velocityEcef[0] = -sl * co * v[0] - so * v[1] - cl * co * v[2];
                e.velocityEcef[1] = -sl * so * v[0] + co * v[1] - cl * so * v[2];
                e.velocityEcef[2] = cl * v[0] - sl * v[2];
                for (int j = 0; j < 3; ++j) e.positionEcef[j] = ms.state.positionEcef[j];
                e.amountKg = m.explosiveKg, e.massKg = m.warheadKg;
                if (!pass.detonated) pass.detonated = true, pass.time = t, pass.ned = m.pose.ned;
                std::snprintf(e.name, sizeof e.name, "%s", m.card.c_str());
                if (hasTarget && !trial) {
                    // the world's detonation: what its fragments and blast break on the target (its events its own)
                    fsim::session::Detonation d;
                    d.store = m.card;
                    for (int j = 0; j < 3; ++j) d.positionEcef[j] = e.positionEcef[j], d.velocityEcef[j] = d.axisEcef[j] = e.velocityEcef[j];
                    weaponSlot = static_cast<std::uint32_t>(i + 1);
                    world->detonate(d);
                    const auto& ts = *world->vehicleState(targetId);
                    const double dx = ts.positionEcef[0] - e.positionEcef[0], dy = ts.positionEcef[1] - e.positionEcef[1],
                                 dz = ts.positionEcef[2] - e.positionEcef[2];
                    std::printf("%s burst %.1f m from %s\n", m.name.c_str(), std::sqrt(dx * dx + dy * dy + dz * dz), tj.string("name", "target").c_str());
                    worldEvents();
                } else {
                    out.writeEvent(e);
                }
                m.ended = true;
                row(static_cast<std::uint32_t>(i + 1), m.name, m.type, false);
                std::printf("%s detonated at %.2f s, %.1f kg of explosive, %.0f m up\n", m.name.c_str(), t, m.explosiveKg,
                            ms.state.altitudeMslM);
                continue;
            }
            if (under && ms.state.altitudeMslM <= *under) {
                // struck the ground: ended that step, as World ends a weapon (its row dead, no state of it there)
                m.ended = true;
                row(static_cast<std::uint32_t>(i + 1), m.name, m.type, false);
                std::printf("%s struck the ground at %.2f s, %.0f m/s\n", m.name.c_str(), t, norm(m.pose.vel));
                continue;
            }
            if (launched[i]) samples.emplace_back(static_cast<std::uint32_t>(i + 1), ms); // (carried: the carrier's store)
        }
        for (const auto& [slot, s] : samples) {
            double R[9];
            fsim::sim::attitude::bodyToEcefFromQuaternion(s.state.attitudeEcefToBody, R);
            for (int j = 0; j < 9; ++j) worst = std::max(worst, std::abs(R[j] - s.state.rotationBodyToEcef[j]));
        }
        out.writeSnapshot(t, samples);
    }
    out.close();
    if (worst > 1e-9) fail("the attitude's quaternion does not give back its matrix (" + std::to_string(worst) + ")");
    std::printf("%s: %d frames, %u vehicles, %.1f s\n", outPath, frames + 1, count, duration);
    return 0;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: flightsim-showcase <script.json> <out.fsrec>\n");
        return 2;
    }
    std::ifstream in(argv[1], std::ios::binary);
    if (!in) fail(std::string("cannot read ") + argv[1]);
    std::stringstream text;
    text << in.rdbuf();
    const Json script = Json::parse(text.str(), argv[1]);
    if (!script.has("target")) {
        Pass pass;
        return run(script, argv[2], pass, nullptr);
    }
    // a target: a first pass finds where the missile goes off; trials fly the target's own flight model to it - its
    // start moved by what it strays from the line until it is there within half a metre - the last flies it through
    // the burst
    Pass first, second;
    const std::string dry = std::string(argv[2]) + ".dry";
    run(script, dry.c_str(), first, nullptr);
    if (!first.detonated) fail("a target needs a missile that detonates (detonate_s)");
    Vec correction;
    for (int trial = 0; trial < 5; ++trial) {
        Pass p;
        run(script, dry.c_str(), p, &first, correction, true);
        std::printf("  the target %.1f m from its line at the burst's time\n", norm(p.targetError));
        if (!p.targetSeen || norm(p.targetError) < 0.5) break;
        correction = correction + (-1.0) * p.targetError;
    }
    std::remove(dry.c_str());
    return run(script, argv[2], second, &first, correction);
}
