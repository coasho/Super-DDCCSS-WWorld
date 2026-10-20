// A part leaving an aircraft, made physically consistent (sim::FlightModel's mass change; MS-7a's releases, MS-7g's
// parting sections): the airframe stays where it is, its CG and state move to what is left, momentum is kept.
#include "mode_flights.h"
#include "foundation/io/AssetResolver.h"
#include "contracts/world_stream/WorldLayout.h"
#include "foundation/transport/WorldMirror.h"
#include "simulation/mission_systems/Host.h"
#include "simulation/flight/JsbsimModel.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;

namespace {

struct V {
    double x = 0.0, y = 0.0, z = 0.0;
    V operator+(const V& o) const { return {x + o.x, y + o.y, z + o.z}; }
    V operator-(const V& o) const { return {x - o.x, y - o.y, z - o.z}; }
    V operator*(double k) const { return {x * k, y * k, z * k}; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
};
V cross(const V& a, const V& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

/// An aircraft's rigid body now: its mass, CG, velocity (ECEF) and its model's origin (ECEF: its CG less its offset)
struct Body {
    double kg = 0.0;
    V cg, velocity, origin;
    sim::JsbsimModel::Rigid r;
};

Body body(sim::FlightModel& m) {
    auto& j = dynamic_cast<sim::JsbsimModel&>(m);
    Body b;
    b.r = j.rigid();
    b.kg = b.r.massKg;
    b.cg = {b.r.cgEcef[0], b.r.cgEcef[1], b.r.cgEcef[2]};
    b.velocity = {b.r.velocityEcef[0], b.r.velocityEcef[1], b.r.velocityEcef[2]};
    double off[3];
    m.cgFromOriginBodyM(off);
    const double* T = b.r.bodyToEcef;
    const V offset{T[0] * off[0] + T[1] * off[1] + T[2] * off[2], T[3] * off[0] + T[4] * off[1] + T[5] * off[2],
                   T[6] * off[0] + T[7] * off[1] + T[8] * off[2]};
    b.origin = b.cg - offset;
    return b;
}

} // namespace

TEST_CASE("a mass change: the airframe stays where it is, its CG and state move to what is left (v + w x d), a reaction on it",
          "[damage][stores][mass]") {
    session::World w(options("mass-change"));
    const auto id = wing(w, "mig29a", 6000.0, 220.0);
    w.step(stepsFor(w, 1.0));
    sim::FlightModel& m = *w.model(id);
    auto& j = dynamic_cast<sim::JsbsimModel&>(m);
    // turning (so the part's point moves as more than the CG does)
    const double rates[3] = {0.5, 0.2, -0.1};
    j.setBodyRates(rates);
    const Body before = body(m);
    double cg0[3];
    m.cgFromOriginBodyM(cg0);
    const double* c0 = before.r.cgStructural;
    // 500 kg off beside the left wing's tip (structural frame: x aft, y right, z up), a sideways push on what is left
    const double kg = 500.0, at[3] = {c0[0] + 0.5, c0[1] - 5.0, c0[2] - 0.3}, impulse[3] = {0.0, 2000.0, -500.0};
    REQUIRE(m.beginMassChange());
    REQUIRE(m.removeMass(kg, at));
    REQUIRE(m.endMassChange(nullptr, impulse));
    const Body after = body(m);
    // the airframe where it was
    CHECK((after.origin - before.origin).length() < 1e-6);
    // the CG where the rest's is: d = -m r / (M - m), r the part's place from the CG (body axes: x and z flipped)
    double cg1[3];
    m.cgFromOriginBodyM(cg1);
    const double M = before.kg;
    const double r[3] = {-(at[0] - c0[0]), at[1] - c0[1], -(at[2] - c0[2])};
    for (int k = 0; k < 3; ++k) CHECK(std::abs((cg1[k] - cg0[k]) - (-kg * r[k] / (M - kg))) < 1e-4);
    CHECK(std::abs(after.kg - (M - kg)) < 0.05);
    // its velocity: the new CG's (w x d) and the reaction's, in body axes turned to ECEF
    const V d{cg1[0] - cg0[0], cg1[1] - cg0[1], cg1[2] - cg0[2]};
    const V dvBody = cross(V{rates[0], rates[1], rates[2]}, d) + V{impulse[0], impulse[1], impulse[2]} * (1.0 / after.kg);
    const double* T = before.r.bodyToEcef;
    const V dv{T[0] * dvBody.x + T[1] * dvBody.y + T[2] * dvBody.z, T[3] * dvBody.x + T[4] * dvBody.y + T[5] * dvBody.z,
               T[6] * dvBody.x + T[7] * dvBody.y + T[8] * dvBody.z};
    CHECK(((after.velocity - before.velocity) - dv).length() < 1e-6);
}

TEST_CASE("a MiG-29A's left wing parts with its missiles: it and its piece carry the aircraft's momentum, its airframe does not jump",
          "[damage][stores][mass][world]") {
    session::World w(options("mass-wing"));
    const auto id = wing(w, "mig29a", 6000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "air-to-air") == ms::Reason::None);
    w.step(stepsFor(w, 2.0));
    const Body before = body(*w.model(id));
    REQUIRE(w.separateSection(id, "left wing"));
    std::uint32_t piece = 0;
    for (const auto v : w.vehicleIds())
        if (w.info(v)->role == ms::Role::Debris) piece = v;
    REQUIRE(piece != 0);
    const Body rest = body(*w.model(id)), part = body(*w.model(piece));
    // the mass: the aircraft's, the piece's - its wing and its three missiles (two R-73s, an R-27R)
    CHECK(std::abs(rest.kg + part.kg - before.kg) < 0.1);
    CHECK(part.kg > 600.0);
    // the momentum (the break's spring pushes them apart: nothing from outside)
    const V p0 = before.velocity * before.kg, p1 = rest.velocity * rest.kg + part.velocity * part.kg;
    CHECK((p1 - p0).length() < 1e-5 * p0.length());
    // the airframe where it was, and their centre of mass where the aircraft's was
    CHECK((rest.origin - before.origin).length() < 1e-4);
    const V centre = (rest.cg * rest.kg + part.cg * part.kg) * (1.0 / (rest.kg + part.kg));
    CHECK((centre - before.cg).length() < 0.01);
    // the stores went with it: the aircraft's lost, none released
    int lost = 0;
    for (const auto& s : w.stores(id)) lost += s.status == ms::StoreStatus::Lost ? 1 : 0;
    CHECK(lost == 3);
}

TEST_CASE("stores jettisoned: the carrier's airframe flies on without a jump, though its CG moved", "[stores][mass][world]") {
    session::World w(options("mass-release"));
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "mk82 x4") == ms::Reason::None);
    w.step(stepsFor(w, 1.0));
    std::uint16_t station = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) station = pl.index;
    ms::JettisonCommand all;
    all.payload = station;
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    // the origin's path step by step: its second difference its acceleration's (under a millimetre a step), not the CG's
    // shift (centimetres): what the ejectors push it by aside
    std::vector<V> path;
    double cgMoved = 0.0, last[3];
    w.model(id)->cgFromOriginBodyM(last);
    for (int k = 0; k < 40; ++k) {
        w.step(1);
        path.push_back(body(*w.model(id)).origin);
        double cg[3];
        w.model(id)->cgFromOriginBodyM(cg);
        cgMoved = std::max(cgMoved, std::sqrt((cg[0] - last[0]) * (cg[0] - last[0]) + (cg[1] - last[1]) * (cg[1] - last[1]) +
                                              (cg[2] - last[2]) * (cg[2] - last[2])));
        for (int a = 0; a < 3; ++a) last[a] = cg[a];
    }
    CHECK(cgMoved > 0.03); // (four Mk 82s off: the CG moved centimetres)
    double worst = 0.0;
    for (std::size_t k = 2; k < path.size(); ++k) worst = std::max(worst, (path[k] - path[k - 1] * 2.0 + path[k - 2]).length());
    if (std::getenv("FSIM_MASS_SHOW")) std::printf("the CG moved %.4f m; the origin's worst second difference %.5f m\n", cgMoved, worst);
    // (the ejectors' reaction changes its velocity at once - some 2.3 mm a step - and its acceleration adds under one:
    // well under a tenth of the CG's shift, which the airframe jumped by before)
    CHECK(worst < 0.1 * cgMoved);
}

TEST_CASE("an F-16's external tanks feed its own as it burns - fuel moved, none made or lost; jettisoned, each flies on with what was left in it",
          "[stores][mass][fuel][world]") {
    session::World w(options("mass-tanks"));
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(1);
    auto external = [&] {
        double kg = 0.0;
        for (const auto& s : w.stores(id))
            if (s.status == ms::StoreStatus::Loaded) kg += s.fuelKg;
        return kg;
    };
    const sim::EnergyOnBoard e0 = w.model(id)->energy();
    const double x0 = external(), m0 = body(*w.model(id)).kg;
    CHECK(std::abs(x0 - 0.8 * (2 * 370 + 300) * 3.785411784) < 0.5); // (three tanks full: the cards' 0.80 kg/L)
    w.step(stepsFor(w, 30.0));
    const sim::EnergyOnBoard e1 = w.model(id)->energy();
    const double x1 = external(), m1 = body(*w.model(id)).kg;
    const double burnt = (e0.fuelKg + x0) - (e1.fuelKg + x1);
    if (std::getenv("FSIM_MASS_SHOW"))
        std::printf("internal %.2f -> %.2f of %.2f kg, external %.2f -> %.2f kg, mass %.2f -> %.2f kg\n", e0.fuelKg, e1.fuelKg, e1.fuelCapacityKg, x0, x1,
                    m0, m1);
    CHECK(burnt > 1.0);                           // (its engine burnt some)
    CHECK(x1 < x0 - 1.0);                         // the tanks gave it ...
    CHECK(e1.fuelCapacityKg - e1.fuelKg < 1.0);   // ... its own kept topped up (it started full) ...
    CHECK(std::abs((m0 - m1) - burnt) < 0.05);    // ... and the aircraft is lighter by the fuel burnt alone
    // the wing tanks drawn alike (a symmetric load stays so)
    double left = -1.0, right = -2.0;
    for (const auto& s : w.stores(id)) {
        if (s.station == 4) left = s.fuelKg;
        if (s.station == 6) right = s.fuelKg;
    }
    CHECK(std::abs(left - right) < 1e-6);
    // jettisoned: each tank is a vehicle of its own, as heavy as it was on its station; the aircraft that much lighter
    std::vector<double> carried;
    for (const auto& s : w.stores(id)) carried.push_back(s.massKg);
    std::uint16_t station = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) station = pl.index;
    ms::JettisonCommand all;
    all.payload = station;
    all.emergency = true;
    const double before = body(*w.model(id)).kg;
    REQUIRE(w.submit(id, all).accepted());
    w.step(stepsFor(w, 0.5));
    double gone = 0.0;
    std::vector<std::uint32_t> tanks;
    const auto stores = w.stores(id);
    for (std::size_t k = 0; k < stores.size(); ++k) {
        const auto& s = stores[k];
        REQUIRE(s.status == ms::StoreStatus::Jettisoned);
        REQUIRE(s.weapon != 0);
        const double kg = body(*w.model(s.weapon)).kg;
        CHECK(std::abs(kg - carried[k]) < 0.05);
        gone += kg;
        tanks.push_back(s.weapon);
    }
    const double after = body(*w.model(id)).kg;
    CHECK(std::abs((before - after) - gone) < 0.5); // (and half a second's burn)
    // it falls away below the aircraft (some 40 m in 3 s), behind it as its drag slows it
    w.step(stepsFor(w, 3.0));
    const Body tank = body(*w.model(tanks[0])), plane = body(*w.model(id));
    const double drop = plane.cg.length() - tank.cg.length();
    const V ahead = plane.velocity * (1.0 / plane.velocity.length());
    const V apart = plane.cg - tank.cg;
    const double behind = apart.x * ahead.x + apart.y * ahead.y + apart.z * ahead.z;
    if (std::getenv("FSIM_MASS_SHOW")) std::printf("3 s after: the tank %.1f m below the aircraft, %.1f m behind it\n", drop, behind);
    CHECK(drop > 20.0);
    CHECK(behind > 1.0);
}

TEST_CASE("the viewer's picture: an airframe hung from its model's origin has each store on its station where the simulation "
          "releases it - and the published sample says where that origin is",
          "[stores][mass][world][viewer]") {
    // what the viewer draws (app/viewer_main.cpp, drawnState): each model's origin at its state's position less its
    // CG's place from the origin (ipc::originEcef, the sample's `derived`), each carried store's model at its
    // aircraft's manifest's `station` line. Released, the store is a vehicle of its own: it must be where it hung.
    session::WorldOptions o = options("mass-picture");
    o.publish = true;
    session::World w(o);
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "mk82 x4") == ms::Reason::None);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(stepsFor(w, 1.0));
    // loaded, its CG is not at its origin: decimetres
    double cg[3];
    w.model(id)->cgFromOriginBodyM(cg);
    CHECK(std::sqrt(cg[0] * cg[0] + cg[1] * cg[1] + cg[2] * cg[2]) > 0.1);
    // the sample the viewer reads says so, and its origin is the model's
    w.publishNow();
    ipc::WorldMirror mirror;
    REQUIRE(mirror.open("mass-picture"));
    sim::SnapshotBatch batch;
    REQUIRE(mirror.pollSnapshot(batch));
    REQUIRE(!mirror.status().empty());
    const double* d = mirror.status()[0].derived;
    for (int k = 0; k < 3; ++k) CHECK(std::abs(d[ipc::kDerivedCgFromOriginM + static_cast<std::size_t>(k)] - cg[k]) < 1e-9);
    double origin[3];
    ipc::originEcef(batch.states[0], d, origin);
    const Body before = body(*w.model(id));
    CHECK((V{origin[0], origin[1], origin[2]} - before.origin).length() < 1e-6);

    // the stations' places as the viewer has them: its model's manifest
    std::map<int, V> stations;
    for (const auto& dir : io::AssetResolver{}.aircraftDirs()) {
        std::ifstream in(dir / "f16c" / "f16c.glb.manifest");
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string key;
            int number = 0;
            V at;
            if (fields >> key >> number >> at.x >> at.y >> at.z && key == "station") stations.emplace(number, at);
        }
        if (!stations.empty()) break;
    }
    REQUIRE(stations.size() == 9);

    std::uint16_t payload = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) payload = pl.index;
    ms::JettisonCommand all;
    all.payload = payload;
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    std::map<std::uint32_t, bool> seen;
    int released = 0;
    double worst = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 4.0) && released < 5; ++k) {
        w.step(1);
        for (const auto& rec : w.weapons()) {
            if (rec.launcher != id || seen[rec.id]) continue;
            seen[rec.id] = true;
            ++released;
            // the store's origin from its aircraft's, in the aircraft's body axes, the step it became a vehicle
            const Body a = body(*w.model(id)), s = body(*w.model(rec.id));
            const V r = s.origin - a.origin;
            const double* T = a.r.bodyToEcef;
            const V at{T[0] * r.x + T[3] * r.y + T[6] * r.z, T[1] * r.x + T[4] * r.y + T[7] * r.z, T[2] * r.x + T[5] * r.y + T[8] * r.z};
            REQUIRE(stations.count(rec.station));
            INFO("station " << rec.station << " (" << rec.store << "): " << at.x << ", " << at.y << ", " << at.z);
            // within what its ejector moved it in that step (some 5 m/s for a 120th of a second) and a centimetre
            const double off = (at - stations.at(rec.station)).length();
            CHECK(off < 0.08);
            worst = std::max(worst, off);
        }
    }
    CHECK(released == 5); // (two bombs, three tanks: the tanks' loadout had the inboard stations first)
    // (hung from its CG instead - as the viewer drew it before - each would have left its station's picture by the
    // CG's place from the origin as it was loaded: the decimetres checked above)
    if (std::getenv("FSIM_MASS_SHOW")) std::printf("stores from their stations as they leave: %.4f m at most\n", worst);
}

TEST_CASE("a missile's model origin is its launch CG: full, its CG is at its origin; as its motor burns its CG moves ahead of it",
          "[stores][mass][world][viewer]") {
    // its file's empty CG is its structure's, burnt out - some decimetres ahead of its CG as it hangs on a station.
    // Its picture's origin and what a station holds is its launch CG, which its file says (fsim/model/origin-x-in:
    // hangar's storebodies.py): a missile jettisoned full is drawn where it is, not a third of a metre off
    session::World w(options("mass-missile-origin"));
    const auto lit = wing(w, "aim120c", 8000.0, 250.0);
    // A standalone flight-model fixture is an aircraft endpoint whose neutral
    // default commands zero throttle. Request ignition explicitly; model load
    // itself must not burn propellant, and changing neutral defaults is invalid.
    control::ActuatorCommand ignition;
    ignition.throttle = 1;
    REQUIRE(w.submit(lit, ignition).accepted());
    const double initialFuel = w.vehicleState(lit)->fuelKg;
    double cg[3];
    w.model(lit)->cgFromOriginBodyM(cg);
    CHECK(std::abs(cg[0]) < 0.002);
    CHECK(std::abs(cg[1]) < 1e-6);
    CHECK(std::abs(cg[2]) < 1e-6);
    w.step(stepsFor(w, 10.0)); // (its 8 s of burn, and on)
    REQUIRE(w.vehicleState(lit)->fuelKg < initialFuel);
    w.model(lit)->cgFromOriginBodyM(cg);
    // burnt out: its structure's CG, 1.497 m from its nose where its launch CG was 1.83 m (its design's and its grain's)
    CHECK(cg[0] > 0.31);
    CHECK(cg[0] < 0.36);
    // a tank's, a bomb's: its CG is its origin (nothing burns)
    for (const char* type : {"tank370gal", "mk82", "463lpallet"}) {
        const auto id = wing(w, type, 6000.0, 200.0, 2);
        w.model(id)->cgFromOriginBodyM(cg);
        INFO(type);
        CHECK(std::sqrt(cg[0] * cg[0] + cg[1] * cg[1] + cg[2] * cg[2]) < 0.002);
        w.removeVehicle(id);
        w.step();
    }
}

TEST_CASE("a load is held to the aircraft's maximum take-off weight: what would take it past is refused, nothing changed; "
          "a bomber carries its bombs",
          "[stores][mass][world]") {
    session::World w(options("mass-most"));
    // a C-17A as it spawns: 235,996 kg of its 265,352 kg. Eighteen 463L pallets (40,824 kg) would take it past: refused
    // as a whole, none loaded. Its airdrop bundles (17,964 kg) fit.
    const auto c17 = wing(w, "c17a", 6000.0, 200.0);
    w.step(stepsFor(w, 0.5));
    REQUIRE(w.profile(c17) != nullptr);
    const double most = w.profile(c17)->envelope.maxTakeoffKg;
    CHECK(std::abs(most - 265352.0) < 1.0);
    const double spawned = w.massKg(c17), full = w.navigationReport(c17).fuelKg;
    REQUIRE(spawned + 40824.0 > most);
    CHECK(w.loadLoadout(c17, "pallets") == ms::Reason::OverMaxWeight);
    CHECK(std::string(ms::reasonName(ms::Reason::OverMaxWeight)) == "over_max_weight");
    for (const auto& s : w.stores(c17)) CHECK(std::string(s.store).empty());
    REQUIRE(w.loadLoadout(c17, "airdrop") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    CHECK(std::abs(w.massKg(c17) - spawned - 17964.0) < 30.0); // (less the fuel of a second)
    // station by station to the limit: pallets go on until the next would pass it (two loads between steps are both
    // counted: the flight model has not weighed the first yet)
    REQUIRE(!w.stores(c17).empty());
    const std::uint16_t hold = w.stores(c17).front().payload;
    int taken = 0;
    for (int station = 1; station <= 18; ++station) { // (each pallet replaces a 998 kg bundle: 1,270 kg more)
        const ms::Reason r = w.loadStore(c17, hold, station, "463l-pallet");
        if (r == ms::Reason::None) ++taken;
        else CHECK(r == ms::Reason::OverMaxWeight);
    }
    w.step(stepsFor(w, 0.5));
    CHECK(w.massKg(c17) <= most + 1.0);
    CHECK(w.massKg(c17) > most - 1300.0);
    CHECK(taken > 0);
    CHECK(taken < 18);
    // with half its fuel it takes all eighteen: what a start's tanks hold is the scenario's to say
    // (InitialConditions::fuelFraction - each tank that fraction of what it holds)
    const auto light = wing(w, "c17a", 6000.0, 200.0, 2, 0.5);
    w.step(stepsFor(w, 0.5));
    const auto nav = w.navigationReport(light);
    CHECK(std::abs(nav.fuelKg - 0.5 * nav.capacity) < 0.001 * nav.capacity);
    CHECK(std::abs((spawned - w.massKg(light)) - (full - nav.fuelKg)) < 30.0); // (lighter by the fuel it has not)
    REQUIRE(w.loadLoadout(light, "pallets") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    CHECK(w.massKg(light) < most);
    REQUIRE(w.resetVehicle(light)); // (a reset is to the same start: its tanks as they were given)
    w.step(stepsFor(w, 0.5));
    CHECK(std::abs(w.navigationReport(light).fuelKg - nav.fuelKg) < 30.0);

    // the B-52H: six bombs on each pylon and eight on its bay's launcher, 4,536 kg - one pylon's drag a pylon
    const auto buff = wing(w, "b52h", 6000.0, 220.0);
    w.step(stepsFor(w, 0.5));
    const double clean = w.massKg(buff);
    REQUIRE(w.loadLoadout(buff, "mk82 x20") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    CHECK(std::abs(w.massKg(buff) - clean - 20.0 * 226.8) < 60.0);
    int carried = 0;
    for (const auto& s : w.stores(buff)) carried += std::string(s.store) == "mk82" ? 1 : 0;
    CHECK(carried == 20);
}
