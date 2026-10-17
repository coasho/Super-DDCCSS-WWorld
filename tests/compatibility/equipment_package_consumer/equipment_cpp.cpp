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
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed C++ equipment consumer: %s\n", error.what());
        return 1;
    }
}
