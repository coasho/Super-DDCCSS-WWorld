// Own-platform equipment access and report provenance through installed public headers and DLLs alone.
#include <fsim/Agent.h>
#include <fsim/MAReports.h>
#include <fsim/World.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <set>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace fs = std::filesystem;
using namespace fsim;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void checkModule(const fs::path& prefix, const wchar_t* name) {
#ifdef _WIN32
    const HMODULE module = GetModuleHandleW(name);
    require(module != nullptr, "required installed DLL was not loaded");
    wchar_t buffer[32768];
    const DWORD size = GetModuleFileNameW(module, buffer, 32768);
    require(size > 0 && size < 32768, "cannot inspect loaded DLL path");
    const fs::path actual(std::wstring(buffer, size));
    require(fs::equivalent(actual, prefix / "bin" / name), "a runtime DLL came from outside the installed SDK");
    std::printf("installed DLL: %s\n", actual.u8string().c_str());
#else
    (void)prefix;
    (void)name;
    throw std::runtime_error("this installed gate requires the Windows UCRT64 runtime");
#endif
}

WorldOptions options(const fs::path& prefix) {
    WorldOptions result;
    result.name = "installed-equipment-cpp";
    result.publish = false;
    result.workers = 1;
    result.pinWorkers = false;
    result.seed = 19;
    result.jsbsimRoot = (prefix / "share/flightsim/jsbsim").generic_string();
    return result;
}

Vehicle aircraft(World& world) {
    VehicleSpec spec;
    spec.name = "viper";
    spec.type = "jsbsim:f16c";
    spec.initial.latitudeDeg = 40.0;
    spec.initial.longitudeDeg = 0.0;
    spec.initial.altitudeMslM = 6000.0;
    spec.initial.headingDeg = 90.0;
    spec.initial.airspeedTrueMs = 230.0;
    return world.createVehicle(spec);
}

void ownMeta(const ma::ReportMeta& meta, double now) {
    require(meta.source == ma::Source::OwnPlatform && meta.validity == ma::Validity::Valid,
            "equipment report is not valid own-platform data");
    require(meta.sampleTimeS == now && meta.receiveTimeS == now && meta.ageS == 0.0,
            "equipment report times differ from its native read");
}

void smoke(World& world) {
    Vehicle viper = aircraft(world);
    require(viper.loadLoadout("mk82 x4") == ms::Reason::None, "installed equipment loadout was not found");
    Agent agent = world.createAgent(viper, "equipment");
    std::uint16_t dispenser = 0xFFFF;
    for (const auto& payload : agent.payloads())
        if (payload.family == ms::Family::Countermeasures) dispenser = payload.index;
    require(dispenser != 0xFFFF, "installed aircraft has no countermeasure payload");
    require(agent.submit(ms::StateCommand{dispenser, ms::PayloadState::Operate}).accepted(),
            "own countermeasure payload could not be powered");
    bool operating = false;
    for (unsigned step = 0; step < 1200 && !operating; ++step) {
        world.step();
        for (const auto& payload : agent.payloads())
            if (payload.index == dispenser) operating = payload.state == ms::PayloadState::Operate;
    }
    require(operating, "native countermeasure warmup did not complete");
    const auto inventory = agent.countermeasures();
    require(inventory.chaff == 30 && inventory.flare == 15, "native initial dispenser inventory differs");
    const auto before = agent.reportCountermeasures();
    require(before.availability == ma::Availability::Available, "dispenser report is unavailable");
    ownMeta(before.meta, world.time());
    require(before.value.chaff == inventory.chaff && before.value.flare == inventory.flare,
            "dispenser report differs from the own-platform read");
    require(agent.dispense(false) == ms::Reason::None, "own-platform chaff release failed");
    const auto after = agent.reportCountermeasures();
    require(after.value.chaff == 29 && after.value.flare == 15, "release did not consume exactly one native chaff");
    require(agent.dispense(true) != ms::Reason::None, "shared native cooldown allowed an immediate flare");
    require(agent.reportCountermeasures().value.flare == 15, "refused release consumed flare inventory");
    ma::ReportOptions stale;
    stale.staleAfterS = -1.0;
    require(agent.reportCountermeasures(stale).meta.validity == ma::Validity::Stale, "strict age threshold was lost");

    const auto stores = agent.reportStores();
    require(stores.availability == ma::Availability::Available && stores.items.size() == 4,
            "installed loadout inventory was not reported");
    for (const auto& item : stores.items) {
        ownMeta(item.meta, world.time());
        require(std::strcmp(item.value.store, "mk82") == 0 && item.value.weapon == 0,
                "inventory contains another store or premature release");
    }
    require(agent.reportStores().items.size() == stores.items.size(), "report read consumed inventory");
    const auto strikes = agent.reportStrikes();
    require(strikes.availability == ma::Availability::Available && strikes.items.empty(), "idle platform reports a strike");
    require(!agent.launchZone("mk82", 0).has_value(), "zero designation produced an available launch zone");
    const auto consents = agent.reportConsents();
    require(consents.items.empty(), "an undelivered consent request was reported");
    std::printf("installed C++ equipment: chaff=%u flare=%u stores=%zu strikes=%zu consents=%zu\n",
                after.value.chaff, after.value.flare, stores.items.size(), strikes.items.size(), consents.items.size());
}

void guidedControls(const fs::path& prefix) {
    auto setup = options(prefix);
    setup.name = "installed-fin-cpp";
    setup.terrain = false;
    World world(setup);
    auto carrier = aircraft(world);
    require(carrier.loadLoadout("air superiority") == ms::Reason::None, "installed missile loadout missing");
    Agent launcher = world.createAgent(carrier, "launcher");
    control::VelocityCommand hold;
    hold.airspeedMs = 250; hold.verticalSpeedMs = 0; hold.headingRad = 90 * 3.14159265358979323846 / 180;
    require(launcher.submit(hold).accepted(), "launcher flight hold refused");
    comm::C2NodeSpec node;
    node.name = "approver"; node.latitudeRad = 39.9 * 3.14159265358979323846 / 180; node.altitudeMslM = 200;
    const auto c2 = world.createC2Node(node);
    const auto net = world.createNet({});
    require(world.joinNet(carrier.id(), net) && world.joinNet(c2, net), "installed consent net could not be joined");
    Agent approver = world.createAgent(c2, "approver");
    require(approver.declareRole(2, "APPROVER"), "approver role could not be declared");
    std::uint16_t stations = 0xffff, radar = 0xffff;
    for (const auto& payload : launcher.payloads()) {
        if (payload.family == ms::Family::StoreStation) stations = payload.index;
        if (payload.family == ms::Family::AirRadar) radar = payload.index;
    }
    require(stations != 0xffff && radar != 0xffff, "installed radar/stations missing");
    require(launcher.submit(ms::StateCommand{stations, ms::PayloadState::Operate}).accepted(), "station power refused");
    require(launcher.submit(ms::StateCommand{radar, ms::PayloadState::Operate}).accepted(), "radar power refused");
    bool operating = false;
    for (unsigned i = 0; i < 1200 && !operating; ++i) {
        world.step();
        unsigned ready = 0;
        for (const auto& payload : launcher.payloads())
            if ((payload.index == stations || payload.index == radar) && payload.state == ms::PayloadState::Operate) ++ready;
        operating = ready == 2;
    }
    require(operating, "installed radar/station warmup failed");
    const auto own = carrier.state();
    VehicleSpec targetSpec;
    targetSpec.name = "opponent"; targetSpec.type = "jsbsim:f16c";
    targetSpec.initial.latitudeDeg = own.latitudeRad * 180 / 3.14159265358979323846 + .01;
    targetSpec.initial.longitudeDeg = own.longitudeRad * 180 / 3.14159265358979323846 + .15;
    targetSpec.initial.altitudeMslM = 6000; targetSpec.initial.airspeedTrueMs = 250; targetSpec.initial.headingDeg = 270;
    auto target = world.createVehicle(targetSpec);
    hold.headingRad = 270 * 3.14159265358979323846 / 180;
    require(target.submit(hold).accepted(), "target flight hold refused");
    ms::AmtiCommand scan;
    scan.payload = radar; scan.bars = 1; scan.azimuthHalfWidthRad = 15 * 3.14159265358979323846 / 180;
    scan.rangeScaleM = 60000;
    require(launcher.submit(scan).accepted(), "installed radar search refused");
    std::uint64_t track = 0;
    for (unsigned i = 0; i < 600 && !track; ++i) {
        world.step();
        for (const auto& measured : launcher.amtiTracks(radar)) if (measured.updates >= 3) { track = measured.id; break; }
    }
    require(track != 0, "installed radar did not confirm a track");
    const auto designation = launcher.designate(radar, track);
    require(designation != 0, "own measured designation refused");
    scan.task = ms::AmtiTask::Track; scan.mode = ms::AmtiMode::SingleTargetTrack; scan.track = track;
    require(launcher.submit(scan).accepted(), "installed radar track refused");
    ms::StrikeCommand strike;
    strike.payload = stations; strike.store = "aim-120c"; strike.target = designation;
    const auto accepted = launcher.submit(strike);
    require(accepted.accepted() && world.weapons().empty(), "legal strike did not wait for delivered consent");
    bool answered = false;
    for (unsigned i = 0; i < 1200 && !answered; ++i) {
        world.step();
        for (const auto& report : approver.reportConsents().items) if (report.value.request.strike == accepted.activity) {
            require(report.meta.source == ma::Source::Dms && world.weapons().empty(), "consent bypassed modeled delivery");
            require(approver.answerConsent(report.value.from, report.value.request.requestId, true), "delivered consent answer refused");
            answered = true; break;
        }
    }
    require(answered, "installed approver received no consent request");
    for (unsigned i = 0; i < 600 && world.weapons().empty(); ++i) world.step();
    require(world.weapons().size() == 1, "delivered consent did not release one armed missile");
    const auto missile = world.weapons().front().id;
    bool moved = false;
    for (unsigned i = 0; i < 20; ++i) {
        world.step();
        const auto sample = world.weaponControlsV1(missile);
        require(sample && sample->available && sample->controllerAvailable && sample->fins.size() == 4,
                "installed DLL did not expose finite native controller/fins");
        require(sample->sampleTimeS == world.time() && sample->controllerTimeS <= sample->sampleTimeS,
                "installed control sample timing differs");
        std::set<std::uint32_t> ids;
        for (const auto& fin : sample->fins) {
            require(fin.valid && ids.insert(fin.id).second && fin.actualRad >= fin.minimumRad && fin.actualRad <= fin.maximumRad,
                    "installed actual fin is invalid, duplicated or outside its stops");
            moved = moved || std::abs(fin.actualRad) > 1e-8;
        }
    }
    require(moved, "armed native guidance never moved a physical fin");
    const auto before = *world.weaponControlsV1(missile);
    const auto& fin = before.fins.front();
    world.vehicle(missile).property("fcs/fin-" + std::to_string(fin.nativeIndex) + "-actuator/malfunction/fail_stuck").set(1);
    world.step(5);
    const auto after = world.weaponControlsV1(missile);
    require(after && after->fins.front().failedStuck && after->fins.front().actualRad == fin.actualRad,
            "installed query hid the actual jammed fin");
    std::printf("installed C++ guided controls: armed=1 delivered_consent=1 fins=%zu jam=1\n", after->fins.size());
}
} // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 3, "usage: equipment_consumer_cpp <installed SDK> <smoke|data_probe>");
        const fs::path prefix = fs::absolute(argv[1]);
        checkModule(prefix, L"libfsim.dll");
        checkModule(prefix, L"libJSBSim.dll");
        World world(options(prefix));
        const std::string mode = argv[2];
        if (mode == "data_probe") {
            try {
                (void)aircraft(world);
            } catch (const Error& error) {
                std::printf("installed aircraft data rejected as expected: %s\n", error.what());
                return 42;
            }
            throw std::runtime_error("corrupt installed aircraft data was bypassed");
        }
        require(mode == "smoke", "unknown installed equipment consumer mode");
        smoke(world);
        guidedControls(prefix);
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed C++ equipment consumer: %s\n", error.what());
        return 1;
    }
}
