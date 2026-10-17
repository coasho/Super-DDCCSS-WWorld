# flightsim

A Windows C++17 flight-simulation platform for reinforcement learning, flight autonomy and air-combat experiments. JSBSim integrates vehicle flight; the native SDK owns flight controls, mission systems and missile engagements. Training applications and game clients submit commands and read SDK state and events. An optional VulkanSceneGraph viewer attaches to a running world, and optional vision cameras render observations.

Start with [current capabilities, milestones and validation limits](docs/status.md). The [documentation index](docs/README.md) separates usage guides, API contracts, design decisions and recorded evidence.

## Build and test

The supported repository presets use **Windows x64, MSYS2 UCRT64 GCC, Ninja and CMake 3.25+**. The toolchain defaults to `D:/ENV/DevLanguages/Cpp/msys2/ucrt64`; keep toolchains, Python environments and caches under `D:/ENV`. Run these PowerShell commands from the repository root with the UCRT64 tools available:

```powershell
$env:PATH = "D:\ENV\DevLanguages\Cpp\msys2\ucrt64\bin;$env:PATH"
New-Item -ItemType Directory -Force D:\ENV\tmp | Out-Null
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:\ENV\tmp'
.\fsim.cmd --preset ucrt64-headless build
.\fsim.cmd --preset ucrt64-headless test
```

The launcher defaults to `ucrt64-headless`, and `build` configures that preset automatically when its cache is absent. This is the canonical headless path: no Vulkan stack or GPU is required. Tests fetch Catch2 on first configuration. Configuration can also fetch a sample visual model. The Python SDK is enabled when a python.org or conda **CPython 3.11+** with development headers and `python3.dll` is found; MSYS2 Python is not suitable. Use `.\fsim.cmd --preset ucrt64-headless configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/<environment>/python.exe` before building if discovery does not find yours. For Python/Hangar tests, provision numpy, scipy, matplotlib and Pillow in that environment before testing. See [development setup and validation](docs/development.md) for dependencies and focused checks.

JSBSim and the VSG stack are tracked vendored sources in `third_party/`; this checkout has no Git submodules to initialize.

For the optional viewer and vision module, install the Vulkan/glslang and VSG dependency prerequisites first, and retain the vendored NVIDIA Flow headers and DLLs. The VSG superbuild supplies the prefix used by the render presets:

```powershell
cmake -S deps --preset deps-ucrt64
cmake --build build/deps-ucrt64
.\fsim.cmd --preset ucrt64-release configure
.\fsim.cmd --preset ucrt64-release build
.\fsim.cmd --preset ucrt64-release test
```

`ucrt64-debug` also enables rendering and needs the same dependencies. These commands describe the checked-in build configuration; recorded results and outstanding failures are linked from [status](docs/status.md).

## Run

Use one selected preset for every command. The syntax is `fsim.cmd [--preset NAME] COMMAND [ARGS]`; `--preset` is a global option before the command. Selection is explicit `--preset`, then `FSIM_PRESET`, then `ucrt64-headless`. All paths come from `build/<preset>`; the launcher never searches other builds. `help` and `where` work before configuration or compilation.

```powershell
.\fsim.cmd --preset ucrt64-headless where
.\fsim.cmd --preset ucrt64-headless headless --help
.\fsim.cmd --preset ucrt64-headless fly
.\fsim.cmd --preset ucrt64-headless python examples/simulation/python/world_tour.py
.\fsim.cmd --preset ucrt64-release demo
.\fsim.cmd --preset ucrt64-release viewer
```

The demo and viewer require a built render preset. The demo's simulation runs in its own publisher process, `flightsim-demo`, which the viewer starts beside itself and attaches to read-only, as to any published world. With the selected preset's Python SDK, `.\fsim.cmd --preset ucrt64-release testbench` opens the scenario testbench with its viewer, and `.\fsim.cmd --preset ucrt64-headless testbench --headless "fa9.,fa10."` runs headless suites. See the [testbench guide](src/applications/testbench/README.md) for options and the `Testbench.cmd` double-click entrypoint. Application arguments and exit codes pass through. `test` requires an already configured build and reports the matching build command when it is absent.

## Layout

| Directory | Purpose |
| --- | --- |
| `include/fsim/` | Public C++ headers and C ABI contracts |
| `src/` | Native implementation, applications (`src/applications/`: headless, demo publisher, viewer, testbench), optional renderer and vision module |
| `python/` | Python bindings and training adapters over the native SDK |
| `examples/` | SDK consumers by category (`training/`, `simulation/`, `visualization/`): trainers, Python examples and scenario JSON |
| `tests/` | Tests by category (`contracts/`, `simulation/`, `training/`, `visualization/`, `compatibility/`, `integration/`): native/C ABI and Python tests, fixtures and installed-package consumers; architecture checks register independently of optional Python bindings |
| `src/applications/testbench/` | Scenario runner, local browser UI and recorded flight-stability results |
| `aircraft/` | Hangar design inputs, generated models, JSBSim definitions and reports |
| `assets/` | Runtime configuration, payload/store cards, textures, shaders and map assets |
| `tools/` | Tools by category: `models/` (Hangar, RCS), `data/` (resource generators), `validation/` (checks, benchmarks, inspection), `launch/` and `architecture/` |
| `cmake/`, `deps/` | Toolchain, asset staging, installation and VSG dependency superbuild |
| `third_party/` | Vendored dependencies and upstream data with their own licenses |
| `docs/` | Guides, SDK reference, decisions, backlog and generated evidence |
| `build/<preset>/` | Build outputs: `bin/`, `examples/`, `tests/`, `python/`, runtime `share/` and packages |

[Resource generation and runtime consumption](docs/resources.md) explains how designs and cards produce flight models, visual models and runtime data. Generated reports are evidence of their recorded run, not a substitute for flight or visual validation.

## Install and package

Package the selected preset; `dist` configures an empty build tree and builds the enabled install targets:

```powershell
.\fsim.cmd --preset ucrt64-headless dist
```

This stages separate components under `build/ucrt64-headless/dist/`: `sdk`, `tools`, `examples` when enabled, and `python` when available. `.\fsim.cmd --preset ucrt64-release dist` adds the `viewer` component. Tests are not installed. The viewer package carries local map assets from `assets/maps`; see the [map assets guide](assets/config/maps.README.md) before preparing an offline viewer.

To install all components together or create a ZIP with CPack:

```powershell
cmake --install build/ucrt64-headless --prefix D:/ENV/flightsim
cpack --config build/ucrt64-headless/CPackConfig.cmake
```

ZIP output goes to `build/<preset>/package/`. The install example above installs all components together. To install only the standalone SDK, add `--component sdk`; `dist/sdk` uses the same component rules. It includes stock JSBSim data, designed aircraft and released-store models, payload/store cards, damage definitions and piece models, and notices. An installed CMake consumer uses `find_package(fsim CONFIG REQUIRED COMPONENTS sdk)` and links `fsim::sdk`; `fsim::vision` and its model/cloud resources are available only in a package built with rendering.

Run `.\fsim.cmd --preset ucrt64-headless test -L package` after building to check an SDK-only installation under a separate prefix containing spaces. The gate checks runtime-data hashes, loaded DLL paths, stock/custom aircraft and store release, then poisons installed XML to verify load rejection before restoring and rerunning. This checks package consumption; GPU rendering and aircraft stability require their own validation. The [SDK guide](docs/sdk/README.md) and [package consumer example](tests/compatibility/package_consumer/CMakeLists.txt) describe explicit runtime-data selection and consumer interfaces.

## Documentation

- [Current status](docs/status.md): capabilities, milestone boundaries and verification limits.
- [Development guide](docs/development.md): supported setup, contribution rules and checks.
- [Resource guide](docs/resources.md): inputs, generators, outputs and runtime consumers.
- [SDK guide](docs/sdk/README.md): world, vehicles, controls, agents, scenarios and language bindings.
- [Native engagement contract](docs/sdk/engagement.md): missile lifecycle, units, access and modeling assumptions.
- [Documentation index](docs/README.md): architecture decisions, domain designs, backlog and evidence.

## License

First-party code and documentation are [MIT licensed](LICENSE). Dependencies and incorporated resources retain their own licenses; see [third-party notices](THIRD_PARTY_NOTICES.md), including JSBSim's LGPL terms and shipped source changes.
