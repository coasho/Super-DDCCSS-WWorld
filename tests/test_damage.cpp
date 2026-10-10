// MS-7g's damage part (src/ms/damage): the warhead each card and the rule make, the blast against an independent fit,
// THOR's penetration, every aircraft's damage data read, and a burst beside an F-16C - where the fragments go,
// what they strike, deterministic from the damage stream.
#include "ms/damage/Damage.h"
#include "ms/damage/State.h"
#include "mode_flights.h"
#include "sim/JsbsimModel.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace fsim::ms;
using namespace fsim::ms::damage;

namespace {

const std::filesystem::path kAircraft = FSIM_TEST_AIRCRAFT_DIR;
const std::filesystem::path kStores = kAircraft.parent_path() / "assets" / "stores";

bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

std::shared_ptr<const StoreCard> card(const char* id) {
    std::vector<std::string> errors;
    auto c = readStoreCard(kStores / (std::string(id) + ".json"), errors);
    INFO((errors.empty() ? std::string() : errors.front()));
    REQUIRE(errors.empty());
    return c;
}

// Kinney and Graham's free-air fits (Explosive Shocks in Air, 1985): independent of Kingery-Bulmash's
double kinneyPressure(double z) {
    return 101325.0 * 808.0 * (1.0 + (z / 4.5) * (z / 4.5)) /
           (std::sqrt(1.0 + (z / 0.048) * (z / 0.048)) * std::sqrt(1.0 + (z / 0.32) * (z / 0.32)) * std::sqrt(1.0 + (z / 1.35) * (z / 1.35)));
}
double kinneyImpulse(double z, double w) { // Pa s
    return 1e5 * 1e-3 * 0.067 * std::sqrt(1.0 + std::pow(z / 0.23, 4.0)) / (z * z * std::cbrt(1.0 + std::pow(z / 1.55, 3.0))) * std::cbrt(w);
}

} // namespace

TEST_CASE("a warhead's fragments: Gurney's velocity, the rule's case and fragments, from each card", "[damage]") {
    // Gurney's cylinder: M/C = 1 throws at sqrt(2E) / sqrt(1.5)
    CHECK(near(gurneyCylinder(2700.0, 1.0, 1.0), 2700.0 / std::sqrt(1.5), 1e-9));
    CHECK(gurneyCylinder(2700.0, 1.0, 0.0) == 0.0);
    // the AIM-120C's WDU-41/B: 20 kg, 7.17 kg of PBXN-110 - its case (0.85 of the rest) thrown at some 1.8 km/s,
    // in cubes as thick as the case, a few grams each, more than a thousand of them
    const auto aim120 = warheadOf(*card("aim-120c"));
    REQUIRE(aim120);
    CHECK(aim120->kind == Warhead::Kind::Fragmentation);
    CHECK(near(aim120->caseKg, 0.85 * (20.0 - 7.17), 1e-9));
    CHECK(aim120->speedMs > 1600.0);
    CHECK(aim120->speedMs < 2100.0);
    CHECK(aim120->lengthM > 0.15);
    CHECK(aim120->lengthM < 0.6);
    CHECK(aim120->fragmentKg > 0.002);
    CHECK(aim120->fragmentKg < 0.02);
    CHECK(aim120->fragments > 500);
    CHECK(near(aim120->fragments * aim120->fragmentKg, aim120->caseKg, 0.01 * aim120->caseKg));
    CHECK(aim120->tntKg > aim120->explosiveKg); // (an HMX fill: more than its weight of TNT)
    double shares = 0.0;
    for (const Zone& z : aim120->zones) shares += z.share;
    CHECK(near(shares, 1.0, 1e-12));
    // the R-27's rods: fewer, heavier, slower (a continuous rod's speed at most Talos's)
    const auto r27 = warheadOf(*card("r-27er"));
    REQUIRE(r27);
    CHECK(r27->kind == Warhead::Kind::Rod);
    CHECK(r27->fragmentKg > 10.0 * aim120->fragmentKg);
    CHECK(r27->speedMs <= 1400.0);
    // every missile's card gives one; a bomb's card none yet
    for (const auto& f : std::filesystem::directory_iterator(kStores)) {
        std::vector<std::string> errors;
        const auto c = readStoreCard(f.path(), errors);
        INFO(f.path().string() << (errors.empty() ? std::string() : ": " + errors.front()));
        REQUIRE(c);
        CHECK(errors.empty());
        if (c->storeClass == "aam") {
            const auto w = warheadOf(*c);
            REQUIRE(w);
            CHECK(w->fragments > 10);
            CHECK(w->speedMs > 800.0);
        }
    }
}

TEST_CASE("a burst's blast: Kingery-Bulmash in free air against Kinney-Graham, scaled for altitude", "[damage]") {
    // 2 kg of TNT in free air (the surface fits' 1 kg): the two fits agree within some 12 % from Z = 4 out, within
    // 25 % close in (Z 1.6: the fits part near the fireball, where neither is measured well)
    for (double r : {2.0, 5.0, 10.0}) {
        const Blast b = blastAt(2.0, r, 101325.0, 288.15);
        const double z = r / std::cbrt(2.0), tol = z < 3.0 ? 0.25 : 0.12;
        INFO("r " << r << " m: " << b.overpressurePa << " Pa against " << kinneyPressure(z) << ", " << b.impulsePas << " Pa s against "
                  << kinneyImpulse(z, 2.0));
        CHECK(near(b.overpressurePa, kinneyPressure(z), tol * kinneyPressure(z)));
        CHECK(near(b.impulsePas, kinneyImpulse(z, 2.0), (tol + 0.03) * kinneyImpulse(z, 2.0)));
    }
    // Sachs: in air at half the pressure, the overpressure at the scaled distance a half
    const Blast sea = blastAt(8.0, 6.0, 101325.0, 288.15), high = blastAt(8.0, 6.0 / std::cbrt(0.5), 0.5 * 101325.0, 288.15);
    CHECK(near(high.overpressurePa, 0.5 * sea.overpressurePa, 1e-6 * sea.overpressurePa));
    // the reflected peak: twice a weak shock's, up to eight times a strong one's
    CHECK(near(reflected(10.0, 101325.0), 20.0, 0.01));
    CHECK(reflected(1e9, 101325.0) / 1e9 > 7.9);
}

TEST_CASE("a fragment through the skin: THOR's residual velocity", "[damage]") {
    // a 6 g steel cube (9 mm) at 1.8 km/s through 3 mm of aluminium: on, slower; it slows more through a thicker plate,
    // and obliquely; a slow one through a thick plate stops
    const double m = 0.006, a = 1.5 * 0.009 * 0.009;
    const double thin = thorResidual(0.003, a, m, 1800.0, 0.0);
    CHECK(thin > 1000.0);
    CHECK(thin < 1800.0);
    CHECK(thorResidual(0.008, a, m, 1800.0, 0.0) < thin);
    CHECK(thorResidual(0.003, a, m, 1800.0, 1.0) < thin);
    CHECK(thorResidual(0.02, a, m, 300.0, 0.0) == 0.0);
    CHECK(thorResidual(0.0, a, m, 300.0, 0.0) == 300.0);
}

TEST_CASE("every aircraft's damage data reads", "[damage]") {
    int read = 0;
    for (const auto& d : std::filesystem::directory_iterator(kAircraft)) {
        const auto file = d.path() / (d.path().filename().string() + ".damage.json");
        if (!std::filesystem::is_regular_file(file)) continue;
        std::string error;
        const auto a = readAirframe(file, error);
        INFO(error);
        REQUIRE(a);
        CHECK(a->sections.size() == 5);
        CHECK(!a->panels.empty());
        CHECK(!a->components.empty());
        for (const auto& p : a->panels) CHECK(p.section >= 0);
        ++read;
    }
    CHECK(read >= 30);
}

TEST_CASE("an AIM-120C bursting 6 m to an F-16C's left: its fragments strike the left of it", "[damage]") {
    std::string error;
    const auto f16 = readAirframe(kAircraft / "f16c" / "f16c.damage.json", error);
    REQUIRE(f16);
    const auto w = warheadOf(*card("aim-120c"));
    REQUIRE(w);
    // the target level, flying north (body x north, y east, z down: ECEF here a local frame), at 220 m/s; the missile
    // from behind and below at 800 m/s, bursting 6 m to its left by its wing
    Target t;
    t.bodyToEcef = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    t.velocity = {220.0, 0.0, 0.0};
    t.airPressurePa = 47181.0, t.airTemperatureK = 249.2, t.airDensity = 0.66;
    Burst b;
    b.at = {-1.0, -6.0, 0.5};
    b.velocity = {800.0, 50.0, -40.0};
    b.axis = b.velocity;
    fsim::Rng rng(7);
    const Impact hit = assess(*w, b, *f16, t, rng);
    CHECK(near(hit.missM, std::sqrt(1.0 + 36.0 + 0.25), 1e-9));
    if (std::getenv("FSIM_DAMAGE_SHOW")) {
        std::printf("AIM-120C: %d fragments of %.1f g at %.0f m/s, %.2f m long, case %.1f mm; %d strike\n", w->fragments,
                    1000.0 * w->fragmentKg, w->speedMs, w->lengthM, 1000.0 * w->caseThicknessM, hit.fragmentsStriking);
        for (const auto& c : hit.components)
            std::printf("  %-28s %3d hits %8.0f J, fastest %4.0f m/s\n", f16->components[static_cast<std::size_t>(c.component)].name.c_str(),
                        c.hits, c.energyJ, c.fastestMs);
        for (const auto& p : hit.panels)
            if (p.hits || p.impulsePas > 100.0)
                std::printf("  panel %3d %-10s %3d hits %3d through, hole %5.0f cm2, %6.0f kPa, %5.0f Pa s\n", p.panel,
                            f16->sections[static_cast<std::size_t>(f16->panels[static_cast<std::size_t>(p.panel)].section)].name.c_str(),
                            p.hits, p.perforations, 1e4 * p.holeAreaM2, 1e-3 * p.overpressurePa, p.impulsePas);
        std::printf("  impulse %.0f %.0f %.0f N s\n", hit.impulseBody.x, hit.impulseBody.y, hit.impulseBody.z);
    }
    CHECK(hit.fragmentsStriking > 20);
    CHECK(hit.fragmentsStriking < hit.fragmentsThrown);
    int left = 0, right = 0;
    for (const auto& p : hit.panels) {
        const auto& s = f16->sections[static_cast<std::size_t>(f16->panels[static_cast<std::size_t>(p.panel)].section)].name;
        if (s == "left wing") left += p.hits;
        if (s == "right wing") right += p.hits;
    }
    INFO("left wing " << left << " hits, right " << right);
    CHECK(left > right);
    CHECK(!hit.components.empty());
    CHECK(hit.impulseBody.y > 0.0); // (pushed away from the burst: to the right)
    // the same stream, the same hits
    fsim::Rng again(7);
    const Impact twice = assess(*w, b, *f16, t, again);
    CHECK(twice.fragmentsStriking == hit.fragmentsStriking);
    CHECK(twice.components.size() == hit.components.size());
    // far off, nothing strikes and the blast is weak
    Burst far = b;
    far.at = {0.0, -300.0, 0.0};
    fsim::Rng rng3(7);
    const Impact miss = assess(*w, far, *f16, t, rng3);
    CHECK(miss.components.empty());
}

TEST_CASE("a hit aircraft's structure: an intact wing fails at its ultimate load, a weakened one sooner", "[damage]") {
    std::string error;
    auto f16 = readAirframe(kAircraft / "f16c" / "f16c.damage.json", error);
    REQUIRE(f16);
    const auto af = std::make_shared<const Airframe>(*f16);
    const double g = 9.80665, W = af->designMassKg;
    // flown level, pulling n at its design mass, its tanks empty (the case its root is sized by)
    auto pull = [&](double n) {
        Flight f;
        f.aeroForceN = {0.0, 0.0, -n * W * g};
        f.specificForceMs2 = {0.0, 0.0, -n * g};
        f.tankKg.assign(8, 0.0);
        return f;
    };
    const int left = af->section("left wing");
    VehicleDamage intact(af);
    const double nu = af->nUltimate[1];
    CHECK(near(std::abs(intact.bending(left, pull(nu))[0]), af->sections[static_cast<std::size_t>(left)].strengthNm[0],
               2e-3 * af->sections[static_cast<std::size_t>(left)].strengthNm[0]));
    CHECK(intact.loads(pull(0.99 * nu)).empty());
    VehicleDamage pulled(af);
    const auto broke = pulled.loads(pull(1.02 * nu));
    REQUIRE(!broke.empty());
    CHECK(broke.front().kind == Change::Kind::Separated);
    // its torn panels' face: a blast at a thin skin tears it from some 600 Pa s (4 mm of 2024-T3)
    CHECK(tearImpulse(0.004) > 400.0);
    CHECK(tearImpulse(0.004) < 900.0);
    // a 10 cm2 hole in a tank at 1 g and 30 kPa of dynamic pressure: some kilograms a second
    VehicleDamage holed(af);
    Impact hit;
    for (std::size_t c = 0; c < af->components.size(); ++c)
        if (af->components[c].kind == "fuel") {
            Impact::ComponentHits h;
            h.component = static_cast<int>(c), h.hits = 1, h.energyJ = 0.0, h.fastestMs = 500.0, h.areaM2 = 1e-3;
            hit.components.push_back(h);
            break;
        }
    fsim::Rng rng(3);
    holed.apply(hit, 101325.0, rng);
    REQUIRE(holed.tanks().size() == 1);
    const int tank = holed.tanks().begin()->first;
    const double leak = holed.leakKgS(tank, 500.0, 1.0, 30000.0);
    CHECK(leak > 1.0);
    CHECK(leak < 10.0);
    CHECK(holed.leakKgS(tank, 0.0, 1.0, 30000.0) == 0.0);
}

TEST_CASE("an AIM-120C's burst beside an F-16C: what it breaks, deterministic", "[damage]") {
    std::string error;
    auto f16 = readAirframe(kAircraft / "f16c" / "f16c.damage.json", error);
    REQUIRE(f16);
    const auto af = std::make_shared<const Airframe>(*f16);
    const auto w = warheadOf(*card("aim-120c"));
    REQUIRE(w);
    Target t;
    t.bodyToEcef = {V3{1, 0, 0}, V3{0, 1, 0}, V3{0, 0, 1}};
    t.velocity = {220.0, 0.0, 0.0};
    t.airPressurePa = 47181.0, t.airTemperatureK = 249.2, t.airDensity = 0.66;
    Burst b;
    b.velocity = {800.0, 50.0, -40.0};
    b.axis = b.velocity;
    std::vector<std::vector<int>> runs;
    for (int k = 0; k < 2; ++k) {
        b.at = {-1.0, -3.0, 0.5}; // (3 m off its left side)
        fsim::Rng rng(11);
        const Impact hit = assess(*w, b, *af, t, rng);
        VehicleDamage d(af);
        const auto changes = d.apply(hit, t.airPressurePa, rng);
        std::vector<int> kinds;
        for (const auto& c : changes) kinds.push_back(static_cast<int>(c.kind));
        runs.push_back(kinds);
        if (k == 0 && std::getenv("FSIM_DAMAGE_SHOW"))
            for (const auto& c : changes)
                std::printf("change %d component %d tank %d engine %d channel %s panel %d section %d spill %.1f\n", static_cast<int>(c.kind),
                            c.component, c.tank, c.engine, c.channel.c_str(), c.panel, c.section, c.spillKg);
        CHECK(!changes.empty());
    }
    CHECK(runs[0] == runs[1]);
}

namespace {
// a burst `offset` metres from the vehicle's origin (body axes), the missile closing at `closing` m/s (body axes)
fsim::session::Detonation burstBeside(const fsim::sim::VehicleState& s, V3 offset, V3 closing) {
    fsim::session::Detonation d;
    d.store = "aim-120c";
    const double* R = s.rotationBodyToEcef;
    auto toEcef = [&](const V3& b) { return V3{R[0] * b.x + R[1] * b.y + R[2] * b.z, R[3] * b.x + R[4] * b.y + R[5] * b.z, R[6] * b.x + R[7] * b.y + R[8] * b.z}; };
    const V3 off = toEcef(offset), own = toEcef({s.velocityBodyMs[0], s.velocityBodyMs[1], s.velocityBodyMs[2]}), close = toEcef(closing);
    for (int k = 0; k < 3; ++k) {
        d.positionEcef[k] = s.positionEcef[k] + (&off.x)[k];
        d.velocityEcef[k] = (&own.x)[k] + (&close.x)[k];
        d.axisEcef[k] = d.velocityEcef[k];
    }
    return d;
}
} // namespace

TEST_CASE("an AIM-120C beside a MiG-29A in the world: what it breaks, flown on, deterministic", "[damage][world]") {
    std::vector<std::vector<int>> kinds;
    for (int run = 0; run < 2; ++run) {
        fsim::session::World w(fsim::modes::options("damage"));
        const auto id = fsim::modes::wing(w, "mig29a", 6000.0, 220.0);
        w.step(fsim::modes::stepsFor(w, 3.0));
        const auto& s = *w.vehicleState(id);
        // 4 m off its left side, overtaking from behind and below at 600 m/s
        const auto events = w.detonate(burstBeside(s, {-1.0, -4.0, 0.5}, {600.0, 80.0, -30.0}));
        REQUIRE(!events.empty());
        CHECK(events.front().kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Detonation));
        CHECK(events.front().target != 0); // (its target: the MiG)
        const auto* dmg = w.damage(id);
        REQUIRE(dmg);
        std::vector<double> before;
        auto fuel = [&] {
            std::vector<double> out;
            for (int t = 0; t < 3; ++t) out.push_back(w.model(id)->property("propulsion/tank[" + std::to_string(t) + "]/contents-lbs").get());
            return out;
        };
        before = fuel();
        w.step(fsim::modes::stepsFor(w, 5.0));
        const auto later = w.takeEngagementEvents();
        const auto after = fuel();
        std::vector<int> k;
        for (const auto& e : later) k.push_back(e.kind);
        kinds.push_back(k);
        if (run == 0 && std::getenv("FSIM_DAMAGE_SHOW")) {
            for (const auto& e : later)
                std::printf("event %d %-24s t %.2f index %u spill %.1f rate %.2f\n", e.kind, e.name, e.simTime, e.index, e.amountKg, e.rateKgS);
            for (std::size_t t = 0; t < before.size(); ++t) std::printf("tank %zu: %.0f -> %.0f lbs\n", t, before[t], after[t]);
            const auto& v = *w.vehicleState(id);
            std::printf("after 5 s: alt %.0f m, roll %.0f, pitch %.0f deg, tas %.0f m/s\n", v.altitudeMslM, v.eulerRad[0] * 57.3, v.eulerRad[1] * 57.3,
                        v.airspeedTrueMs);
        }
        // its holed tanks lost fuel faster than its engines burn it
        bool leaked = false;
        for (const auto& [tank, state] : dmg->tanks())
            if (tank >= 0 && tank < 3 && state.holeM2 > 0.0 && after[static_cast<std::size_t>(tank)] < before[static_cast<std::size_t>(tank)]) leaked = true;
        CHECK((dmg->tanks().empty() || leaked));
        CHECK(std::isfinite(w.vehicleState(id)->altitudeMslM));
    }
    CHECK(kinds[0] == kinds[1]);
}

TEST_CASE("a MiG-29A's left wing parts: it flies on as a JSBSim piece of its own, the aircraft rolls", "[damage][world]") {
    fsim::session::World w(fsim::modes::options("piece"));
    const auto id = fsim::modes::wing(w, "mig29a", 6000.0, 220.0);
    w.step(fsim::modes::stepsFor(w, 2.0));
    const double before = w.model(id)->property("inertia/weight-lbs").get();
    REQUIRE(w.separateSection(id, "left wing"));
    CHECK_FALSE(w.separateSection(id, "left wing")); // (gone already)
    std::uint32_t piece = 0;
    for (const auto v : w.vehicleIds())
        if (w.info(v)->role == fsim::ms::Role::Debris) piece = v;
    REQUIRE(piece != 0);
    CHECK(w.info(piece)->type.rfind("debris:mig29a:left wing:", 0) == 0);
    const auto& p0 = *w.vehicleState(piece);
    const auto& a0 = *w.vehicleState(id);
    const double alt0 = p0.altitudeMslM;
    // it leaves where its centre was, as fast as the aircraft
    CHECK(std::abs(p0.airspeedTrueMs - a0.airspeedTrueMs) < 15.0);
    // the aircraft lighter by the wing's structure
    std::string error;
    const auto af = readAirframe(kAircraft / "mig29a" / "mig29a.damage.json", error);
    REQUIRE(af);
    const double wing = af->sections[static_cast<std::size_t>(af->section("left wing"))].massKg;
    w.step(1);
    CHECK(std::abs((before - w.model(id)->property("inertia/weight-lbs").get()) * 0.45359237 - wing) < 0.05 * wing + 20.0);
    // its first seconds: unstable in pitch (its centre of mass behind its centre of pressure) it pitches up, its
    // lift lifting it, and tumbles, its broadside drag taking its speed; then it falls at a tumbling plate's speed
    w.step(fsim::modes::stepsFor(w, 8.0));
    const auto& p1 = *w.vehicleState(piece);
    const auto& a1 = *w.vehicleState(id);
    CHECK(p1.airspeedTrueMs < 0.6 * a1.airspeedTrueMs); // (its drag a tumbling plate's)
    CHECK(std::abs(a1.eulerRad[0]) > 0.5);              // (one wing gone: the aircraft rolls off)
    const double rollAt8 = a1.eulerRad[0] * 57.2958, climbed = p1.altitudeMslM - alt0;
    w.step(fsim::modes::stepsFor(w, 22.0));
    const auto& p2 = *w.vehicleState(piece);
    if (std::getenv("FSIM_DAMAGE_SHOW"))
        std::printf("the piece: up %.0f m by 8 s (the aircraft rolled to %.0f deg), %.0f m below its start by 30 s at %.0f m/s\n", climbed,
                    rollAt8, alt0 - p2.altitudeMslM, p2.airspeedTrueMs);
    CHECK(p2.altitudeMslM < alt0 - 300.0);
    CHECK(p2.airspeedTrueMs < 120.0);
    const auto events = w.takeEngagementEvents();
    bool separated = false;
    for (const auto& e : events) separated = separated || e.kind == static_cast<std::uint8_t>(fsim::ipc::EngagementKind::Separated);
    CHECK(separated);
}

TEST_CASE("what a JSBSim piece costs a step", "[.][bench][damage]") {
    // two worlds alike, each a MiG-29A whose left wing parted; one flies its piece, the other's is removed at once:
    // their steps timed in alternating second-long rounds while both fly, the medians compared
    fsim::session::World with(fsim::modes::options("with")), without(fsim::modes::options("without"));
    const auto a = fsim::modes::wing(with, "mig29a", 9000.0, 220.0), b = fsim::modes::wing(without, "mig29a", 9000.0, 220.0);
    REQUIRE(with.separateSection(a, "left wing"));
    REQUIRE(without.separateSection(b, "left wing"));
    for (const auto v : without.vehicleIds())
        if (without.info(v)->role == fsim::ms::Role::Debris) without.removeVehicle(v);
    std::vector<double> tw, to;
    const unsigned n = fsim::modes::stepsFor(with, 1.0);
    auto timed = [&](fsim::session::World& w) {
        const auto t0 = std::chrono::steady_clock::now();
        w.step(n);
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() / (n * static_cast<double>(w.frameSkip()));
    };
    for (int round = 0; round < 9; ++round) tw.push_back(timed(with)), to.push_back(timed(without));
    std::sort(tw.begin(), tw.end());
    std::sort(to.begin(), to.end());
    CHECK(!with.vehicleState(a)->crashed);
    std::printf("a 120 Hz step: %.1f us with its piece, %.1f us without - a piece %.1f us (medians of 9 rounds)\n", 1e6 * tw[4], 1e6 * to[4],
                1e6 * (tw[4] - to[4]));
}

TEST_CASE("every aircraft's pieces load and fall in JSBSim", "[damage]") {
    // hangar's pieces (aircraft/<type>/pieces/<type>-<section>/), each loaded, started at 6 km at 200 m/s and stepped
    // 5 s: it stays finite and slows (a tumbling piece's drag)
    auto ground = std::make_shared<fsim::sim::FlatGround>(0.0);
    int loaded = 0;
    for (const auto& d : std::filesystem::directory_iterator(kAircraft)) {
        const auto dir = d.path() / "pieces";
        if (!std::filesystem::is_directory(dir)) continue;
        for (const auto& p : std::filesystem::directory_iterator(dir)) {
            const std::string name = p.path().filename().string();
            INFO(name);
            fsim::sim::JsbsimModel m(1.0 / 120.0, ground);
            fsim::sim::InitialConditions ic;
            ic.altitudeMslM = 6000.0, ic.airspeedTrueMs = 200.0;
            REQUIRE(m.load(fsim::sim::AircraftSpec{name, FSIM_TEST_JSBSIM_ROOT, dir}, ic));
            fsim::sim::ControlInputs in;
            for (int k = 0; k < 600; ++k) m.step(in);
            fsim::sim::VehicleState s;
            m.state(s);
            CHECK(std::isfinite(s.altitudeMslM));
            CHECK(s.airspeedTrueMs < 200.0);
            ++loaded;
        }
    }
    CHECK(loaded >= 120);
}
