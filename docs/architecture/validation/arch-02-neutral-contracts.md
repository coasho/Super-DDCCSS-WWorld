# ARCH-02 neutral contracts

Status: implemented, awaiting independent acceptance (2026-10-16). ARCH-03 has not started.
Starting main HEAD: `7625529` (ARCH-01 with its review and recheck corrections). No numerical, RNG, timestep or default
behavior changed: definitions moved with their names, namespaces, fields and order. The runtime applies the same
scenario values in the same order.

Allowlist:
- **Contracts:** `src/contracts/**` (new), and `src/CMakeLists.txt` for its registration.
- **Transport:** `src/ipc/{CMakeLists.txt,Recording.h,Recording.cpp,VisionSegment.h,WorldMirror.h,WorldPublisher.h,WorldRegistry.cpp}`;
  `src/ipc/WorldLayout.h` moved to the contracts.
- **Simulation:** `src/sim/{CMakeLists.txt,SnapshotBuffer.h}`, and
  `src/session/{CMakeLists.txt,World.h,Scenario.h,Scenario.cpp,ScenarioRecipe.h}`.
- **Training and SDK:** `src/env/{CMakeLists.txt,ScenarioVecEnv.cpp}` and `src/sdk/Scenario.cpp`.
- **Public headers:** `include/fsim/{Scenario.h,ScenarioDocument.h}`.
- **Includes only:** `src/app/StrikeCapture.h`, `src/ui/{MissionGui.h,MonitorGui.h}` and `src/world/FlowVolumes.h`.
- **Tests:** `tests/{CMakeLists.txt,test_contracts.cpp,test_state_layout.cpp,test_scenario.cpp,test_mass_change.cpp,test_load_limits.cpp}`.
- **Policy and checker:** `tools/architecture/{policy.json,check.py,baseline.py,test_check.py,README.md}`.
- **Documentation:**
  - `docs/sdk/viewer.md` (two path references);
  - this record and [arch-02-baseline.json](arch-02-baseline.json);
  - the ARCH-02 status and index lines of the [roadmap](../migration/roadmap.md) and [architecture index](../README.md);
  - a `src/contracts/**` row (P070) in the [path map](../migration/path-map.csv), since the contracts were created in their destination;
  - `docs/status.md`.

The other owner's uncommitted wording edits in `tools/architecture/check.py` and `README.md` stay outside this
package.

## 1. Contracts below producer and consumer (roadmap tasks 3 and 5)

`fsim_contracts` (`fsim::contracts`) is a header-only interface target in its reserved root `src/contracts`. It links
nothing, includes only the standard library and public contract headers, and is a `contract` module in the
architecture policy:

| Contract | Taken from | Who includes it |
| --- | --- | --- |
| `world_stream/WorldLayout.h` (shared-memory layout, mission/engagement records, registry) | `src/ipc/WorldLayout.h` (moved whole) | ipc, ui, world, viewer, tests |
| `world_stream/SnapshotBatch.h` (`sim::SnapshotBatch`) | `src/sim/SnapshotBuffer.h` (the batch; the triple buffer stays in sim) | sim's buffer, ipc's mirror |
| `vision_stream/VisionLayout.h` (camera segment layout) | `src/ipc/VisionSegment.h` (the publisher/mirror stay in ipc) | ipc, Vision, the viewer's preview |
| `recording/RecordingFormat.h` (FSREC records, magic, version history, snapshot framing) | `src/ipc/Recording.h` and the format constants hidden in `Recording.cpp` | ipc's writer/reader, the SDK, session |
| `simulation/WorldOptions.h` (`session::WorldOptions`) | `src/session/World.h` | session (runtime) and, after ARCH-04, the lower facade's mapper |

The transport (`ipc`) now links the contracts, core and platform, and includes no simulation header. The policy's
`ipc -> sim` exception is removed, and the gate holds the transport to the foundation rule. `sim` and `session` link the
contracts they include. The public state values (`VehicleState`, `ControlInputs`, `EnvironmentState`) were already
neutral headers in `include/fsim`; the forwarding `sim/*.h` names remain for simulation code, and the transport uses the
public names directly.

## 2. Public header inventory (task 1)

Every `include/fsim` header (41) has an owner domain and a kind in the policy's `public_headers`, recorded in the
baseline with its hash and including modules:

| Domain | Kind | Headers |
| --- | --- | --- |
| contracts | value / interface | `Export`, `Span`, `Rng`, `VehicleState`, `ControlInputs`, `EnvironmentState`, `InitialConditions` (value); `GroundProvider` (interface) |
| simulation | value | `Pid`, `MAReports` |
| simulation | interface | `Altimeter`, `Magnetic`, `Frames`, `Property`, `Capability`, `MissionSystems`, `Control`, `ControlStack`, `ControllerRegistry`, `BuiltinControllers`, `RotorControllers`, `GuidanceModes`, `VehicleProfile`, `PerformanceProfile`, `Effects`, `BuiltinEffects`, `Comm`, `ScenarioDocument` |
| simulation | facade | `World`, `Agent`, `MAService`, `Recording` |
| training | interface / facade | `VecEnvPlugins` (interface); `VecEnv`, `MissionEpisode`, `MissionBatch`, `Scenario` (facade) |
| adapters | c_abi / facade | `fsim_c`, `fsim_mission_env`, `fsim_vision_c` (C ABI); `Vision` (the Vision DLL's World-handle API) |

The checker holds every module-to-public-header and public-to-public include to the domain direction rules. The
inventory found one violation, the A-03 finding: simulation runtime (`session`) included the training-owned
`Scenario.h` and, through it, `VecEnv.h`. Section 3 removes it.

Three existing public includes are listed as exact exceptions with the package that removes them. Viewer widgets
(`ui`) read `Capability.h` and `MissionSystems.h` (ARCH-06). `VecEnv.h` includes `fsim_c.h` because the C++ VecEnv is
a pimpl over the C handle (ARCH-07). A stale public exception fails like a native one.

## 3. Scenario and options values apart from facade and training aggregates (task 2)

- `fsim/ScenarioDocument.h` (new, simulation) holds the document's format, its value types (`EffectSpec`,
  `ScenarioVehicle`, `ScenarioVecEnv`, `Scenario`) and `loadScenario` / `parseScenario` / `dumpScenario`.
  `fsim/Scenario.h` includes it and keeps `applyScenario` and `vecEnvOptions`. Every includer of `Scenario.h` sees the
  same declarations; the types, their layouts and the exported functions are unchanged (section 5).
- The runtime applies its own `session::ScenarioRecipe` (`session/ScenarioRecipe.h`). `session::scenarioRecipe` maps
  the document onto it field for field: environment, world-wide effects, and each vehicle's name, type, model, control
  divider, initial state, count, spacing, command, target and effects. The document's `world` and `vecenv` sections are
  not the runtime's. The apply steps, their order and their error messages are the same. The document overloads keep
  every caller as it was.
- `vecEnvOptions` (document to `VecEnvOptions`) moved from the runtime to training (`src/env/ScenarioVecEnv.cpp`). It
  is the same exported symbol, still linked into `libfsim.dll` through the whole-archive `env`.
- `session::WorldOptions` (native construction values) is a contract (section 1). The mapper from the public options
  (`session/WorldOptions.h`) stays where it is until ARCH-04 moves it to the lower simulation facade.

## 4. Regressions added before the includes changed (task 6)

| Regression | What it holds |
| --- | --- |
| `tests/test_state_layout.cpp`: the recording writer's framing and fields for a fixed world (FSREC v7) | Taken from the writer before any definition moved: a 2,928-byte file. Each record's kind, slot, count and size prefix, and every field the world sets, are checked at their literal offsets; the file reads back. Padding is not compared (see section 7). The existing frozen offsets, strides and v1-v7 old-recording reads are kept |
| `tests/test_contracts.cpp` (CTest `contracts`, label `architecture`) | Built against `fsim::contracts` and Catch2 alone. The frozen sizes and constants of every moved definition, the native options' defaults, and the inline model-origin function |
| `tools/architecture/test_check.py` (12 cases) | Transport depending on simulation fails once its exception is gone. Transport on the contracts passes and cannot reach simulation headers. A contract cannot include or link an implementation. Unclassified and stale public headers fail. A simulation module or public header cannot include a training one, and the scenario split passes. A contract header cannot include a domain API. Only assemblies and applications include the C ABI. Public exceptions are exact and expire. Classification errors are input errors |

## 5. Compatibility evidence

| Check | Result (2026-10-16) |
| --- | --- |
| `tools/ma_contract/baseline.py compare` against the frozen baseline (`e1945b9`), headless and Release | 19 comparisons, 0 failures. No baseline export or vtable removed; `libfsim.dll` exports 1,456 symbols (the same count as the M8 record), 68 vtable/typeinfo symbols unchanged. 267 public C++ and 120 C types with unchanged layouts, 718 and 415 enumerators unchanged, 50 public vtables unchanged. The Python inventories (620 headless, 629 Release names) keep their contracts. `applyScenario`, `loadScenario`, `parseScenario`, `dumpScenario` and `vecEnvOptions` are exported with their signatures |
| `baseline.py verify`, then `run` with this build's installed SDK and Python package | 66 artifacts unchanged. The frozen old-header C++ and C99 binaries and the frozen Python consumers (`agent.py`, `links.py`, `c2.py`) pass on the new library and package |
| Recordings | The v1-v7 old-recording reads and the writer's frozen framing pass. The recording, mirror, stores, events, environment and vision-segment round trips pass |

## 6. Verification

| Check | Result (2026-10-16) |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings each; Release compiled every render consumer of the moved headers (`ui`, `world`, `vision`, the viewer) |
| `tools/architecture/check.py --repo .` | Accepted: 20 native modules, 374 source/public header files. Native exceptions: `ui -> sim`, `world -> sim` (ARCH-06); public exceptions: the three listed |
| `tools/architecture/test_check.py` | 122 cases passed (12 added) |
| Targeted headless CTest (46 tests) | 46/46 passed in 103.99 s. Scenario, recording, mirror, vision segment, the viewer's picture, VecEnv, layout/boundary, contracts, C ABI, mission episode/actions/joint/C ABI, MA binding, batch recording, the SDK, mission and equipment installed-package consumers, `sdk_boundary`, architecture and contract checks, and the Python world/batch-recording/VecEnv/mission/MA/vision modules |
| Targeted Release CTest (15 tests) | 15/15 passed in 94.60 s: contracts, layout boundary, strike capture, native showcase, viewer screenshot, Vision (C++, C ABI, Python), viewer-facing shared-memory cases and the three installed-package consumers |
| Planning document checker | Accepted |

No full suite ran, as the request directs. The change-scoped selector would escalate this package (shared simulation
and build-system paths) to the full headless suite. That run stays with the next full-regression milestone.

## 7. Findings outside this package

- **Recording padding.** `RecordWriter` writes its callers' structs and its own snapshot head byte for byte. Their
  padding is unspecified, so two recordings of the same world can differ in padding bytes (memory contents reach the
  file). Readers are unaffected. This predates ARCH-02 and was found while writing the framing regression. A writer
  that writes padding as zero would not change the format; it is left for its own reviewed change.

## 8. Next

ARCH-03 (foundation and domain leaf moves) may start after independent acceptance of this package. The remaining
exceptions are removed by the packages named in them.
