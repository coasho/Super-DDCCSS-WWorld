// A6 characterization through World's existing production path. Floating values are recorded as their
// scalar IEEE bits, never as VehicleState's object representation (which includes padding).
#include "mode_flights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;

namespace {

class Canonical {
public:
    Canonical() { out_.imbue(std::locale::classic()); }

    void integer(const std::string& name, std::uint64_t value) {
        out_ << name << "=i:" << std::hex << value << '\n';
    }
    void number(const std::string& name, double value) {
        std::uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        out_ << name << "=f64:" << std::hex << std::setfill('0') << std::setw(16) << bits << '\n';
    }
    void number(const std::string& name, float value) {
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(value));
        std::memcpy(&bits, &value, sizeof(bits));
        out_ << name << "=f32:" << std::hex << std::setfill('0') << std::setw(8) << bits << '\n';
    }
    template <std::size_t N> void numbers(const std::string& name, const double (&values)[N]) {
        for (std::size_t i = 0; i < N; ++i) number(name + "[" + std::to_string(i) + "]", values[i]);
    }
    void string(const std::string& name, const char* value) { out_ << name << "=s:" << std::quoted(value) << '\n'; }
    std::string str() const { return out_.str(); }

private:
    std::ostringstream out_;
};

void stateFields(Canonical& c, const sim::VehicleState& s) {
    c.number("state.simTime", s.simTime);
    c.numbers("state.positionEcef", s.positionEcef);
    c.numbers("state.attitudeEcefToBody", s.attitudeEcefToBody);
    c.number("state.latitudeRad", s.latitudeRad);
    c.number("state.longitudeRad", s.longitudeRad);
    c.number("state.altitudeMslM", s.altitudeMslM);
    c.number("state.altitudeAglM", s.altitudeAglM);
    c.numbers("state.eulerRad", s.eulerRad);
    c.numbers("state.velocityBodyMs", s.velocityBodyMs);
    c.numbers("state.velocityNedMs", s.velocityNedMs);
    c.numbers("state.angularRateBodyRadS", s.angularRateBodyRadS);
    c.numbers("state.accelerationBodyMs2", s.accelerationBodyMs2);
    c.number("state.airspeedTrueMs", s.airspeedTrueMs);
    c.number("state.airspeedCalibratedMs", s.airspeedCalibratedMs);
    c.number("state.mach", s.mach);
    c.number("state.alphaRad", s.alphaRad);
    c.number("state.betaRad", s.betaRad);
    c.number("state.loadFactor", s.loadFactor);
    c.number("state.aileronRad", s.aileronRad);
    c.number("state.elevatorRad", s.elevatorRad);
    c.number("state.rudderRad", s.rudderRad);
    c.number("state.flapsRad", s.flapsRad);
    c.number("state.gearPosition", s.gearPosition);
    c.integer("state.engineCount", static_cast<std::uint64_t>(s.engineCount));
    c.number("state.speedbrakePosition", s.speedbrakePosition);
    c.numbers("state.throttlePosition", s.throttlePosition);
    c.numbers("state.thrustN", s.thrustN);
    c.number("state.fuelKg", s.fuelKg);
    c.integer("state.stepCount", s.stepCount);
    c.integer("state.onGround", s.onGround);
    c.integer("state.diverged", s.diverged);
    c.integer("state.crashed", s.crashed);
    c.numbers("state.rotationBodyToEcef", s.rotationBodyToEcef);
    c.numbers("state.engineRpm", s.engineRpm);
    c.numbers("state.engineN2", s.engineN2);
    c.numbers("state.afterburner", s.afterburner);
    c.numbers("state.nozzlePosition", s.nozzlePosition);
    c.number("state.leadingEdgeFlapRad", s.leadingEdgeFlapRad);
    c.integer("state.wheelCount", static_cast<std::uint64_t>(s.wheelCount));
    c.numbers("state.wheelCompressionM", s.wheelCompressionM);
    c.numbers("state.wheelSteerRad", s.wheelSteerRad);
    c.numbers("state.wheelSpeedMs", s.wheelSpeedMs);
}

std::string inventoryFields(const ms::Host& host) {
    Canonical c;
    c.integer("stores.revision", host.storeRevision());
    c.integer("stores.massRevision", host.massRevision());
    c.number("stores.carriedMassKg", host.storesMassKg());
    c.number("stores.externalFuelKg", host.externalFuelKg());
    c.integer("stores.count", host.stores().size());
    for (const auto& s : host.stores()) {
        const std::string p = "stores[" + std::to_string(s.index) + "].";
        c.integer(p + "index", s.index);
        c.integer(p + "payload", s.payload);
        c.integer(p + "station", static_cast<std::uint64_t>(s.station));
        c.string(p + "stationName", s.stationName);
        c.string(p + "store", s.store);
        c.string(p + "mnemonic", s.mnemonic);
        c.string(p + "storeClass", s.storeClass);
        c.string(p + "carriage", s.carriage);
        c.integer(p + "powerable", s.powerable);
        c.integer(p + "jettisonable", s.jettisonable);
        c.integer(p + "flyable", s.flyable);
        c.integer(p + "status", static_cast<std::uint64_t>(s.status));
        c.number(p + "massKg", s.massKg);
        c.number(p + "fuelKg", s.fuelKg);
        c.integer(p + "weapon", s.weapon);
        c.number(p + "bayDoors", s.bayDoors);
    }
    return c.str();
}

// Deliberately omit entity identity and world time here: a fresh aircraft can be compared with a reused model.
std::string vehicleFields(session::World& w, std::uint32_t id) {
    const auto* state = w.vehicleState(id);
    auto* model = w.model(id);
    const auto* host = w.missionSystems(id);
    const auto* inputs = w.inputs(id);
    const auto* performance = w.performance(id);
    REQUIRE(state != nullptr);
    REQUIRE(model != nullptr);
    REQUIRE(host != nullptr);
    REQUIRE(inputs != nullptr);
    REQUIRE(performance != nullptr);
    Canonical c;
    stateFields(c, *state);
    const auto energy = model->energy();
    c.number("model.massKg", w.massKg(id));
    c.number("model.internalFuelKg", energy.fuelKg);
    c.number("model.fuelCapacityKg", energy.fuelCapacityKg);
    c.number("model.fuelFlowKgS", energy.fuelFlowKgS);
    c.number("model.chargeJ", energy.chargeJ);
    c.number("model.chargeCapacityJ", energy.chargeCapacityJ);
    c.number("model.powerW", energy.powerW);
    c.integer("model.starved", energy.starved);
    double cg[3] = {};
    c.integer("model.hasCg", model->cgFromOriginBodyM(cg));
    c.numbers("model.cgFromOriginBodyM", cg);
    c.number("inputs.aileron", inputs->aileron);
    c.number("inputs.elevator", inputs->elevator);
    c.number("inputs.rudder", inputs->rudder);
    c.numbers("inputs.throttle", inputs->throttle);
    c.number("inputs.flaps", inputs->flaps);
    c.number("inputs.gearDown", inputs->gearDown);
    c.number("inputs.brakeLeft", inputs->brakeLeft);
    c.number("inputs.brakeRight", inputs->brakeRight);
    c.integer("control.revision", w.controlRevision(id));
    c.integer("performance.revision", performance->revision);
    c.number("performance.loadKg", performance->loadKg);
    c.number("performance.loadSpeedRatio", performance->loadSpeedRatio);
    c.number("performance.minCasMs", performance->minCasMs);
    c.number("performance.maxCasMs", performance->maxCasMs);
    return c.str() + inventoryFields(*host);
}

std::uint16_t stationPayload(const session::World& w, std::uint32_t id) {
    const auto* host = w.missionSystems(id);
    REQUIRE(host != nullptr);
    for (const auto& payload : host->payloads())
        if (payload.family == ms::Family::StoreStation) return payload.index;
    FAIL("aircraft has no store station payload");
    return 0;
}

void checkHealthy(const session::World& w, std::uint32_t id) {
    const auto* s = w.vehicleState(id);
    REQUIRE(s != nullptr);
    CHECK_FALSE(s->diverged);
    CHECK_FALSE(s->crashed);
}

void snapshot(std::ostringstream& trace, const char* label, session::World& w) {
    trace << "phase=" << label << '\n';
    Canonical c;
    c.number("world.simTime", w.simTime());
    c.integer("world.vehicleCount", w.vehicleCount());
    trace << c.str();
    auto ids = w.vehicleIds();
    std::sort(ids.begin(), ids.end());
    for (const auto id : ids) {
        Canonical identity;
        identity.integer("entity.id", id);
        identity.integer("entity.generation", w.info(id)->generation);
        identity.integer("entity.role", static_cast<std::uint64_t>(w.info(id)->role));
        identity.integer("entity.launcher", w.info(id)->launcher);
        trace << identity.str() << vehicleFields(w, id);
    }
}

std::string replay(unsigned workers, int frameSkip, bool drag) {
    auto o = options("store-binding-replay");
    o.workers = workers;
    o.frameSkip = frameSkip;
    o.dt = 1.0 / 60.0;
    o.storeDrag = drag;
    o.seed = 17;
    session::World w(o);
    // Two active aircraft ensure the two-worker case traverses more than one model before any release.
    const auto carrier = wing(w, "f16c", 5000.0, 220.0, 0, 0.5);
    const auto companion = wing(w, "f16c", 5000.0, 220.0, 1, 0.5);
    const auto payload = stationPayload(w, carrier);
    auto* retainedModel = w.model(carrier);
    const auto initialStoreRevision = w.missionSystems(carrier)->storeRevision();
    const auto initialMassRevision = w.missionSystems(carrier)->massRevision();
    std::ostringstream trace;
    trace.imbue(std::locale::classic());
    snapshot(trace, "spawn", w);

    REQUIRE(w.loadLoadout(carrier, "external tanks") == ms::Reason::None);
    REQUIRE(w.loadStore(carrier, payload, 5, "") == ms::Reason::None);
    REQUIRE(w.loadStore(carrier, payload, 5, "tank-300gal") == ms::Reason::None);
    snapshot(trace, "between-step-load-unload-reload", w);
    const auto inventory = inventoryFields(*w.missionSystems(carrier));
    const auto refusal = w.loadStore(carrier, payload, 1, "mk82");
    REQUIRE(refusal == ms::Reason::StationCannotTake);
    CHECK(inventoryFields(*w.missionSystems(carrier)) == inventory);
    Canonical result;
    result.integer("refusedLoad.reason", static_cast<std::uint64_t>(refusal));
    trace << result.str();
    snapshot(trace, "between-step-refused", w);

    const auto storeRevision = w.missionSystems(carrier)->storeRevision();
    const auto massRevision = w.missionSystems(carrier)->massRevision();
    const double fuelBefore = w.missionSystems(carrier)->externalFuelKg();
    w.step();
    snapshot(trace, "tick-1", w);
    CHECK(w.missionSystems(carrier)->externalFuelKg() < fuelBefore);
    CHECK(w.missionSystems(carrier)->massRevision() > massRevision);
    CHECK(w.missionSystems(carrier)->storeRevision() == storeRevision);
    REQUIRE(w.loadStore(carrier, payload, 3, "mk82") == ms::Reason::None);
    REQUIRE(w.loadStore(carrier, payload, 4, "") == ms::Reason::None);
    REQUIRE(w.loadStore(carrier, payload, 4, "tank-370gal") == ms::Reason::None);
    snapshot(trace, "after-tick-1-edits", w);
    w.step(6);
    snapshot(trace, "tick-7", w);
    REQUIRE(w.loadStore(carrier, payload, 3, "") == ms::Reason::None);
    REQUIRE(w.loadStore(carrier, payload, 3, "mk82") == ms::Reason::None);
    snapshot(trace, "after-tick-7-edits", w);
    w.step();
    snapshot(trace, "tick-8", w);
    w.step();
    snapshot(trace, "tick-9", w);

    ms::JettisonCommand release;
    release.payload = payload;
    release.emergency = true;
    for (const auto& s : w.stores(carrier))
        if (s.station != 6) release.stores.push_back(s.index);
    REQUIRE(release.stores.size() == 3);
    const auto command = w.submit(carrier, release);
    REQUIRE(command.accepted());
    Canonical accepted;
    accepted.integer("jettison.activity", command.activity);
    accepted.integer("jettison.status", static_cast<std::uint64_t>(command.status));
    trace << accepted.str();
    snapshot(trace, "asymmetric-jettison-submitted", w);
    w.step(2);
    snapshot(trace, "asymmetric-jettison-flown", w);
    unsigned jettisoned = 0, stillLoaded = 0;
    for (const auto& s : w.stores(carrier)) {
        if (s.station == 6) {
            CHECK(s.status == ms::StoreStatus::Loaded);
            CHECK(s.weapon == 0);
            ++stillLoaded;
        } else {
            CHECK(s.status == ms::StoreStatus::Jettisoned);
            CHECK(s.weapon != 0);
            ++jettisoned;
        }
    }
    CHECK(jettisoned == 3);
    CHECK(stillLoaded == 1);
    checkHealthy(w, carrier);
    checkHealthy(w, companion);

    const auto beforeReset = inventoryFields(*w.missionSystems(carrier));
    REQUIRE(w.resetVehicle(carrier, nullptr, o.seed));
    CHECK(inventoryFields(*w.missionSystems(carrier)) == beforeReset);
    snapshot(trace, "loaded-reset", w);
    w.step(2);
    snapshot(trace, "loaded-reset-after-steps", w);

    REQUIRE(w.removeVehicle(carrier));
    CHECK(w.missionSystems(carrier) == nullptr);
    const auto reused = wing(w, "f16c", 5000.0, 220.0, 0, 0.5);
    CHECK(w.model(reused) == retainedModel);
    CHECK(w.stores(reused).empty());
    CHECK(w.missionSystems(reused)->storeRevision() == initialStoreRevision);
    CHECK(w.missionSystems(reused)->massRevision() == initialMassRevision);
    snapshot(trace, "loaded-history-reused-spawn", w);

    // These paired field-bit records characterize the existing reuse transient without demanding a clean reset.
    session::World fresh(o);
    const auto clean = wing(fresh, "f16c", 5000.0, 220.0, 0, 0.5);
    trace << "comparison=fresh-clean-spawn\n" << vehicleFields(fresh, clean);
    trace << "comparison=reused-spawn\n" << vehicleFields(w, reused);
    w.step(2);
    fresh.step(2);
    trace << "comparison=fresh-clean-after-steps\n" << vehicleFields(fresh, clean);
    trace << "comparison=reused-after-steps\n" << vehicleFields(w, reused);
    CHECK(std::abs(w.massKg(reused) - fresh.massKg(clean)) < 0.01);
    CHECK(w.stores(reused).empty());
    snapshot(trace, "reused-after-steps", w);
    const auto newPayload = stationPayload(w, reused);
    REQUIRE(w.loadStore(reused, newPayload, 4, "tank-370gal") == ms::Reason::None);
    w.step();
    CHECK(w.missionSystems(reused)->externalFuelKg() > 0.0);
    CHECK(w.massKg(reused) > fresh.massKg(clean) + 1000.0);
    checkHealthy(w, reused);
    snapshot(trace, "reused-reloaded", w);
    return trace.str();
}

} // namespace

TEST_CASE("store binding: a loaded reset retains inventory fuel and revisions", "[stores][mass][fuel][world][store-binding]") {
    auto o = options("store-binding-reset");
    o.frameSkip = 1;
    session::World w(o);
    const auto id = wing(w, "f16c", 5000.0, 220.0, 0, 0.5);
    const double initialFuel = w.model(id)->energy().fuelKg;
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    const double external = w.missionSystems(id)->externalFuelKg();
    w.step(9);
    REQUIRE(w.missionSystems(id)->externalFuelKg() < external);
    const auto before = inventoryFields(*w.missionSystems(id));
    const auto storeRevision = w.missionSystems(id)->storeRevision();
    const auto massRevision = w.missionSystems(id)->massRevision();
    const auto controlRevision = w.controlRevision(id);
    const auto performanceRevision = w.performance(id)->revision;
    const auto generation = w.info(id)->generation;
    REQUIRE(w.resetVehicle(id, nullptr, o.seed));
    CHECK(inventoryFields(*w.missionSystems(id)) == before);
    CHECK(w.missionSystems(id)->storeRevision() == storeRevision);
    CHECK(w.missionSystems(id)->massRevision() == massRevision);
    CHECK(w.controlRevision(id) == controlRevision);
    CHECK(w.performance(id)->revision == performanceRevision);
    CHECK(w.model(id)->energy().fuelKg == initialFuel);
    CHECK(w.info(id)->generation == generation + 1);
    CHECK(w.vehicleState(id)->stepCount == 0);
    CHECK(w.vehicleState(id)->simTime == 0.0);
    const double retainedFuel = w.missionSystems(id)->externalFuelKg();
    w.step();
    CHECK(w.missionSystems(id)->externalFuelKg() < retainedFuel);
    CHECK(w.missionSystems(id)->storeRevision() == storeRevision);
    CHECK(w.missionSystems(id)->massRevision() > massRevision);
    checkHealthy(w, id);
}

TEST_CASE("store binding: loaded-history slot reuse starts a fresh host and synchronizes station mass", "[stores][mass][world][store-binding]") {
    auto o = options("store-binding-reuse");
    o.frameSkip = 1;
    session::World fresh(o), used(o);
    const auto clean = wing(fresh, "f16c", 5000.0, 220.0, 0, 0.5);
    const auto previous = wing(used, "f16c", 5000.0, 220.0, 0, 0.5);
    auto* retainedModel = used.model(previous);
    REQUIRE(used.loadLoadout(previous, "external tanks") == ms::Reason::None);
    used.step(9);
    REQUIRE(used.massKg(previous) > fresh.massKg(clean) + 3000.0);
    REQUIRE(used.removeVehicle(previous));
    const auto reused = wing(used, "f16c", 5000.0, 220.0, 0, 0.5);
    CHECK(reused != previous);
    CHECK(used.model(reused) == retainedModel);
    CHECK(used.stores(reused).empty());
    CHECK(used.missionSystems(reused)->storeRevision() == fresh.missionSystems(clean)->storeRevision());
    CHECK(used.missionSystems(reused)->massRevision() == fresh.missionSystems(clean)->massRevision());
    CHECK(used.missionSystems(reused)->externalFuelKg() == 0.0);
    CHECK(used.missionSystems(reused)->storesMassKg() == 0.0);
    const auto freshStart = vehicleFields(fresh, clean);
    const auto reusedStart = vehicleFields(used, reused);
    INFO("canonical reused/fresh spawn equal: " << (reusedStart == freshStart));
    INFO("fresh spawn mass kg: " << fresh.massKg(clean) << ", reused: " << used.massKg(reused));
    used.step();
    fresh.step();
    CHECK(std::abs(used.massKg(reused) - fresh.massKg(clean)) < 0.01);
    CHECK(used.stores(reused).empty());
    used.step(8);
    fresh.step(8);
    const auto freshLater = vehicleFields(fresh, clean);
    const auto reusedLater = vehicleFields(used, reused);
    INFO("canonical reused/fresh after nine steps equal: " << (reusedLater == freshLater));
    CHECK(std::abs(used.massKg(reused) - fresh.massKg(clean)) < 0.01);
    // Rebinding must resolve the retained model's station properties for the new inventory too.
    REQUIRE(used.loadStore(reused, stationPayload(used, reused), 6, "tank-370gal") == ms::Reason::None);
    used.step();
    CHECK(used.massKg(reused) > fresh.massKg(clean) + 1000.0);
    checkHealthy(used, reused);
}

TEST_CASE("store binding: deterministic replay preserves edit tick fuel release reset and reuse boundaries", "[stores][mass][fuel][drag][world][determinism][store-binding]") {
    std::ostringstream transcript;
    transcript.imbue(std::locale::classic());
    transcript << "store-binding-trace-v1; scalar hexadecimal IEEE floating bits and hexadecimal integers\n";
    for (const int frameSkip : {1, 4})
        for (const bool drag : {false, true}) {
            std::string serial;
            for (const unsigned workers : {1u, 2u}) {
                const std::string label = "workers=" + std::to_string(workers) + ";frameSkip=" + std::to_string(frameSkip) +
                                          ";storeDrag=" + (drag ? "true" : "false");
                INFO(label);
                const auto run = replay(workers, frameSkip, drag);
                if (workers == 1) serial = run;
                else {
                    CHECK(run.size() == serial.size());
                    if (run != serial) {
                        const auto mismatch = std::mismatch(run.begin(), run.end(), serial.begin(), serial.end());
                        const auto at = static_cast<std::size_t>(mismatch.first - run.begin());
                        const auto start = at == 0 ? std::string::npos : run.rfind('\n', at - 1);
                        const auto line = start == std::string::npos ? 0 : start + 1;
                        UNSCOPED_INFO("first differing replay byte: " << at << "; field: " << run.substr(line, run.find('\n', line) - line));
                    }
                    CHECK(run.compare(serial) == 0);
                }
                transcript << "case=" << label << '\n' << run;
            }
        }
    // Only this test writes a trace. Root uses the same executable/filter before and after extraction.
    if (const char* path = std::getenv("FSIM_STORE_BINDING_TRACE"); path && *path) {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        REQUIRE(out.is_open());
        out << transcript.str();
        out.close();
        REQUIRE(out.good());
    }
}
