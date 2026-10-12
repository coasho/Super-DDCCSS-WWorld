# fsim SDK guide

Start with the [root build/run guide](../../README.md), [current capabilities and limits](../status.md),
and the C++ tour below. After a render build, `.\fsim.cmd --preset ucrt64-release demo` opens a visual demonstration;
headless consumers can run `build/ucrt64-headless/examples/minimal_trainer.exe` directly.
The launcher uses only `build/<preset>`: explicit `--preset` before the command, then `FSIM_PRESET`,
then `ucrt64-headless`. `.\fsim.cmd build` configures the selected preset on its first build.

The `fsim` SDK is how a training application drives the platform. You write a
C++ program (or a Python one, `import fsim`, or any language with a C FFI,
through `fsim_c.h`), link `libfsim.dll`, create a **World**, create **vehicles** by name, type and initial
state, command them at whichever **control level** your experiment needs, and
step the world. Start `flightsim-viewer.exe` from a render build and it mirrors a
published world through shared memory (see [viewer.md](viewer.md)).

| Document | Contents |
| --- | --- |
| [world.md](world.md) | `World`, `Vehicle`, `VehicleSpec`, stepping, time, state and sensed state |
| [control.md](control.md) | the multi-level control stack: levels, commands, built-in loops and behaviours, writing your own |
| [environment.md](environment.md) | real-time environment control, effects (disturbances, sensor models), communication |
| [viewer.md](viewer.md) | transparent visualisation: how the prebuilt viewer discovers and mirrors a world |
| [scenarios.md](scenarios.md) | scenario files: the world, environment, vehicles, commands and effects as a JSON document |
| [agents.md](agents.md) | agents and the referee: an MA's own objects, what they refuse, the referee's view (ADR-31) |
| [boundary.md](boundary.md) | which principal each entry point belongs to - agent (an MA's decisions), MA core, range, referee, platform - and the calls an agent may not yet trust (ADR-31) |
| [mission-systems.md](mission-systems.md) | the payloads each aircraft carries (radars, warning receivers, jammers, sensors, guns, stations): discovery, power states, BIT, grants (ADR-30) |
| [engagement.md](engagement.md) | native missile release, flight, seekers, countermeasures, events, damage, compatibility and modeling assumptions |
| [engagement-validation.md](engagement-validation.md) | dated local regression and encounter evidence, reproduction paths and validation limits |
| [vision.md](vision.md) | cameras on vehicles: offscreen RGB images of the same Earth the viewer draws (`fsim_vision.dll`) |
| [vecenv.md](vecenv.md) | the batch (gym-style) layer for vectorised RL, built on the same object model; registering your own task, observation space and action space |
| [c_abi.md](c_abi.md) | the C ABI for other languages |
| [python.md](python.md) | the Python SDK: the same object model, batch layer and cameras as numpy arrays, Gymnasium and Stable-Baselines3 adapters, and what it costs |

## C++ tour

```cpp
#include <fsim/World.h>
#include <fsim/BuiltinEffects.h>

int main() {
    fsim::World world({.name = "my-experiment"});            // published for viewers under this name
    world.environment().setWind({.directionDeg = 270, .speedMs = 8, .turbulence = 0.2});

    fsim::VehicleSpec spec;
    spec.name = "red-1"; spec.type = "jsbsim:c172x";
    spec.initial.latitudeDeg = 37.62; spec.initial.longitudeDeg = -122.38;
    spec.initial.altitudeMslM = 1500; spec.initial.headingDeg = 90; spec.initial.airspeedTrueMs = 60;
    fsim::Vehicle red = world.createVehicle(spec);

    red.command(fsim::control::VelocityCommand{.airspeedMs = 65, .verticalSpeedMs = 2, .headingRad = 0.0});
    red.addEffect<fsim::effects::GaussianSensorNoise>();

    for (int k = 0; k < 3000; ++k) {                          // 100 s at the default 30 Hz world step
        world.step();
        const auto& s = red.state();                          // truth
        const auto& z = red.sensed();                         // through the sensor effects
        // ... your learner reads z, writes red.command(...) at any level ...
    }
}
```

## Building against the SDK

- Headers: `include/fsim/*.h` (C++17; the `{.field = ...}` designated initialisers in these pages are C++20 sugar - assign the fields instead in C++17). Only these are stable; anything under `src/` may change.
- Library: `libfsim.dll` (+ import library), `libJSBSim.dll` and their runtime dependencies in the installed `bin/` directory. Place your executable there or arrange DLL lookup for that directory.
- Data: the standalone SDK installs stock JSBSim data under `share/flightsim/jsbsim`, designed aircraft and released-store models under `share/flightsim/aircraft`, and payload/store cards under `share/flightsim`. Damage definitions and JSBSim piece models accompany the designs. Notices and JSBSim source changes are under `share/doc/flightsim`. A render SDK also includes optional vision model/cloud resources; map tiles require separate provision. See the [resource guide](../resources.md).
- CMake in this repository: `target_link_libraries(my_trainer PRIVATE fsim::sdk)` (see `examples/`); from an installed package: `find_package(fsim CONFIG REQUIRED COMPONENTS sdk)` gives `fsim::sdk`. Request `vision` when needed; `fsim::vision` is available only when built with rendering. See [Install and package](../../README.md#install-and-package) for `dist`, component installation and CPack.
- Platform development and validation: [development guide](../development.md). Current capability and verification boundaries: [status](../status.md).

The installed CMake configuration exports `fsim_BIN_DIR`, `fsim_DATA_DIR`, `fsim_JSBSIM_ROOT` and `fsim_AIRCRAFT_DIR`, resolved relative to its installed prefix. Pass `fsim_JSBSIM_ROOT` to your application's `WorldOptions::jsbsimRoot`, and set `FSIM_AIRCRAFT_PATH` to `<prefix>/share/flightsim/aircraft;<prefix>/share/flightsim/aircraft/stores`. This explicitly selects installed stock, custom and released-store data even when the development checkout is present. Development defaults also search source paths; package validation must prove the selected data is used.

After building, `.\fsim.cmd --preset ucrt64-headless test -L package` installs only the SDK into a separate prefix containing spaces and builds the [consumer example](../../tests/package_consumer/CMakeLists.txt) against installed `fsim::sdk`. It checks loaded DLL locations and runtime-data hashes, creates stock `c172x` and custom `f16c`, loads four Mk 82s and releases a store. Three probes poison installed stock/custom/store XML and require public-API rejection exit 42, then restore exact bytes and repeat the positive run. Logs and the manifest are retained under `build/<preset>/package-consumer`. This acceptance gate does not exercise GPU rendering or establish fleet stability.

## Conventions

- SI units, radians in the C++ API (degrees only where a field says `Deg`), WGS-84 geodetic latitude/longitude, altitudes above the ellipsoid ("MSL" in JSBSim's sense), body frame x forward / y right / z down, NED velocities.
- `fsim::control::kHold` (NaN) in an optional command field means "keep the current value / let the controller decide".
- Every `World` call runs on the caller's thread; `step()` is the only call that does work. Nothing is asynchronous, nothing is hidden.
- Load-time failures throw `fsim::Error`; runtime calls return `bool`/handles and log through the platform logger.
- Determinism is a contract for the same seed, fixed simulation step and call sequence across worker counts; validate changes with the relevant determinism checks described in the [development guide](../development.md).
