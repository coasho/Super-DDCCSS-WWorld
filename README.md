# flightsim

A Windows C++17 flight-simulation platform for reinforcement learning, flight autonomy and air-combat experiments. JSBSim integrates vehicle flight; the native SDK owns flight controls, mission systems and missile engagements. Training applications and game clients submit commands and read SDK state and events. An optional VulkanSceneGraph viewer attaches to a running world, and optional vision cameras render observations.

Start with [current capabilities, milestones and validation limits](docs/status.md). The [documentation index](docs/README.md) separates usage guides, API contracts, design decisions and recorded evidence.

## Build and test

The supported repository presets use **Windows x64, MSYS2 UCRT64 GCC, Ninja and CMake 3.25+**. The toolchain defaults to `D:/ENV/DevLanguages/Cpp/msys2/ucrt64`; keep toolchains, Python environments and caches under `D:/ENV`. Run these PowerShell commands from the repository root with the UCRT64 tools available:

```powershell
$env:PATH = "D:\ENV\DevLanguages\Cpp\msys2\ucrt64\bin;$env:PATH"
New-Item -ItemType Directory -Force D:\ENV\tmp | Out-Null
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:\ENV\tmp'
cmake --preset ucrt64-headless
cmake --build --preset ucrt64-headless
ctest --preset ucrt64-headless
```

This is the canonical headless path: no Vulkan stack or GPU is required. Tests fetch Catch2 on first configuration. Configuration can also fetch a sample visual model. The Python SDK is enabled when a python.org or conda **CPython 3.11+** with development headers and `python3.dll` is found; MSYS2 Python is not suitable. Set `-DFSIM_PYTHON_EXECUTABLE=D:/ENV/<environment>/python.exe` at configuration if discovery does not find yours. For Python/Hangar tests, provision numpy, scipy, matplotlib and Pillow in that environment before testing. See [development setup and validation](docs/development.md) for dependencies and focused checks.

JSBSim and the VSG stack are tracked vendored sources in `third_party/`; this checkout has no Git submodules to initialize.

For the optional viewer and vision module, install the Vulkan/glslang and VSG dependency prerequisites first, and retain the vendored NVIDIA Flow headers and DLLs. The VSG superbuild supplies the prefix used by the render presets:

```powershell
cmake -S deps --preset deps-ucrt64
cmake --build build/deps-ucrt64
cmake --preset ucrt64-release
cmake --build --preset ucrt64-release
ctest --preset ucrt64-release
```

`ucrt64-debug` also enables rendering and needs the same dependencies. These commands describe the checked-in build configuration; recorded results and outstanding failures are linked from [status](docs/status.md).

## Run

To select the headless build explicitly:

```powershell
.\build\ucrt64-headless\bin\flightsim.exe --help
.\build\ucrt64-headless\examples\minimal_trainer.exe
```

After a render build, `./fsim.cmd demo` opens a demonstration and `./fsim.cmd viewer` attaches to a published SDK world. With the staged Python SDK, `./fsim.cmd python examples/python/world_tour.py` runs the Python tour and `./fsim.cmd testbench` opens the scenario testbench; see the [testbench guide](testbench/README.md) for headless suites and options. `Testbench.cmd` is the double-click entrypoint.

The launcher requires an existing build. It searches Release, headless, packaged Release viewer, then Debug, stopping at the first directory with a platform executable; it does not select a build per command. Its `build`, `test` and `dist` commands are fixed to `ucrt64-release`. Use the direct CMake commands above, or a specific executable/staged Python launcher, when the preset matters. `./fsim.cmd where` prints the selected paths.

## Layout

| Directory | Purpose |
| --- | --- |
| `include/fsim/` | Public C++ headers and C ABI contracts |
| `src/` | Native implementation, applications, optional renderer and vision module |
| `python/` | Python bindings and training adapters over the native SDK |
| `examples/` | SDK consumers, trainers, Python examples and scenario JSON |
| `tests/` | Native/C ABI tests, fixtures and an explicit package-consumer example; Python/boundary checks are registered from `python/` |
| `testbench/` | Scenario runner, local browser UI and recorded flight-stability results |
| `aircraft/` | Hangar design inputs, generated models, JSBSim definitions and reports |
| `assets/` | Runtime configuration, payload/store cards, textures, shaders and map assets |
| `tools/` | Hangar, resource generators, benchmarks and inspection tools |
| `cmake/`, `deps/` | Toolchain, asset staging, installation and VSG dependency superbuild |
| `third_party/` | Vendored dependencies and upstream data with their own licenses |
| `docs/` | Guides, SDK reference, decisions, backlog and generated evidence |
| `build/<preset>/` | Build outputs: `bin/`, `examples/`, `tests/`, `python/`, runtime `share/` and packages |

[Resource generation and runtime consumption](docs/resources.md) explains how designs and cards produce flight models, visual models and runtime data. Generated reports are evidence of their recorded run, not a substitute for flight or visual validation.

## Install and package

After building the selected preset:

```powershell
cmake --build --preset ucrt64-headless --target dist
```

This stages separate components under `build/ucrt64-headless/dist/`: `sdk`, `tools`, `examples` when enabled, and `python` when available. Using `ucrt64-release` instead adds the `viewer` component. Tests are not installed. The viewer package carries local map assets from `assets/maps`; see the [map assets guide](assets/config/maps.README.md) before preparing an offline viewer.

To install all components together or create a ZIP with CPack:

```powershell
cmake --install build/ucrt64-headless --prefix D:/ENV/flightsim
cpack --config build/ucrt64-headless/CPackConfig.cmake
```

ZIP output goes to `build/<preset>/package/`. The install example above installs all components together. An installed CMake consumer uses `find_package(fsim CONFIG REQUIRED)` and links `fsim::sdk`; `fsim::vision` is available only in a package built with rendering. The separate `--component sdk`/`dist/sdk` tree currently omits runtime data that its CMake configuration requires, so it is not a verified standalone consumer path; see the [known data-contract gap](docs/status.md#known-workflow-and-reporting-gaps). The [SDK guide](docs/sdk/README.md) and [package consumer example](tests/package_consumer/CMakeLists.txt) describe the consumer interfaces.

## Documentation

- [Current status](docs/status.md): capabilities, milestone boundaries and verification limits.
- [Development guide](docs/development.md): supported setup, contribution rules and checks.
- [Resource guide](docs/resources.md): inputs, generators, outputs and runtime consumers.
- [SDK guide](docs/sdk/README.md): world, vehicles, controls, agents, scenarios and language bindings.
- [Native engagement contract](docs/sdk/engagement.md): missile lifecycle, units, access and modeling assumptions.
- [Documentation index](docs/README.md): architecture decisions, domain designs, backlog and evidence.

## License

First-party code and documentation are [MIT licensed](LICENSE). Dependencies and incorporated resources retain their own licenses; see [third-party notices](THIRD_PARTY_NOTICES.md), including JSBSim's LGPL terms and shipped source changes.
