// Exercises only the installed public SDK. No internal headers or source paths.
#include <fsim/World.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void checkModule(const fs::path& prefix, const wchar_t* name) {
#ifdef _WIN32
    const HMODULE module = GetModuleHandleW(name);
    require(module != nullptr, "required runtime DLL was not loaded");
    wchar_t path[32768];
    const DWORD size = GetModuleFileNameW(module, path, 32768);
    require(size != 0 && size < 32768, "cannot inspect loaded runtime DLL");
    const fs::path actual(std::wstring(path, size));
    require(fs::equivalent(actual, prefix / "bin" / name), "loaded DLL came from outside the installed SDK");
    std::printf("installed DLL: %s\n", actual.u8string().c_str());
#else
    (void)prefix;
    (void)name;
    throw std::runtime_error("this package gate requires the Windows UCRT64 runtime");
#endif
}
fsim::Vehicle aircraft(fsim::World& world, const char* type, const char* name, double speed) {
    fsim::VehicleSpec spec;
    spec.type = std::string("jsbsim:") + type;
    spec.name = name;
    spec.initial.altitudeMslM = 1500.0;
    spec.initial.airspeedTrueMs = speed;
    spec.initial.fuelFraction = 0.5;
    return world.createVehicle(spec);
}
int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: consumer <installed-prefix> <smoke|stock|design|weapon>\n");
        return 1;
    }
    const std::string mode = argv[2];
    if (mode != "smoke" && mode != "stock" && mode != "design" && mode != "weapon") return 1;
    try {
        const fs::path prefix = fs::canonical(argv[1]);
        checkModule(prefix, L"libfsim.dll");
        checkModule(prefix, L"libJSBSim.dll");
        const fs::path data = prefix / "share" / "flightsim";
        const char* paths = std::getenv("FSIM_AIRCRAFT_PATH");
        require(paths && std::string(paths) == (data / "aircraft").generic_string() + ";" +
                    (data / "aircraft" / "stores").generic_string(),
                "FSIM_AIRCRAFT_PATH must select installed aircraft and store models");
        fsim::WorldOptions options;
        options.name = "installed-consumer";
        options.publish = false;
        options.workers = 1;
        options.pinWorkers = false;
        options.jsbsimRoot = (data / "jsbsim").generic_string();
        fsim::World world(options);
        // Only the relevant public API call can return the negative-probe code.
        // A crash, missing DLL or unrelated failure is never a passing probe.
        if (mode != "smoke") {
            if (mode == "stock" || mode == "design") {
                try {
                    aircraft(world, mode == "stock" ? "c172x" : "f16c", "probe", 150.0);
                } catch (const fsim::Error& error) {
                    std::printf("installed %s model rejected: %s\n", mode.c_str(), error.what());
                    return 42;
                }
            } else {
                auto launcher = aircraft(world, "f16c", "launcher", 150.0);
                try {
                    world.launchWeapon(launcher);
                } catch (const fsim::Error& error) {
                    std::printf("installed weapon model rejected: %s\n", error.what());
                    return 42;
                }
            }
            std::fprintf(stderr, "corrupted installed model unexpectedly loaded\n");
            return 1;
        }
        auto stock = aircraft(world, "c172x", "stock", 60.0);
        auto design = aircraft(world, "f16c", "design", 150.0);
        require(design.loadLoadout("mk82 x4") == fsim::ms::Reason::None, "installed loadout refused");
        const auto stores = design.stores();
        require(stores.size() == 4, "installed payload/store cards did not produce four bombs");
        for (const auto& store : stores)
            require(std::string(store.store) == "mk82" && store.flyable, "installed Mk 82 store card/model unavailable");
        fsim::ms::WeaponLaunch launch;
        launch.offsetBodyM[2] = 2.0;
        launch.ejectionBodyMs[2] = 2.0;
        auto weapon = world.launchWeapon(design, launch);
        require(weapon.role() == fsim::ms::Role::Weapon && weapon.launcher() == design.id(), "weapon role/launcher mismatch");
        fsim::control::AttitudeCommand attitude;
        attitude.pitchRad = 0.03;
        require(stock.command(attitude), "installed SDK rejected stock aircraft control");
        const double initialWeaponAltitude = weapon.state().altitudeMslM;
        world.step(30);
        for (const auto& vehicle : {stock, design, weapon})
            require(vehicle.valid() && std::isfinite(vehicle.state().altitudeMslM) && vehicle.state().altitudeMslM > 1000.0,
                    "installed model did not fly with a finite airborne state");
        require(world.time() > 0.0 && weapon.state().altitudeMslM != initialWeaponAltitude, "native world/weapon did not advance");
        require(world.weapons().size() == 1, "native world did not retain the launched weapon");
        std::printf("installed data: %s\n", data.u8string().c_str());
        std::printf("fsim %s: stock c172x, designed f16c, four carried Mk 82s and one flying weapon at %.2f s\n",
                    fsim::version(), world.time());
#ifdef HAVE_FSIM_VISION
        std::printf("optional vision target exported (GPU rendering not exercised)\n");
#endif
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "installed SDK consumer failed: %s\n", error.what());
        return 1;
    }
}
