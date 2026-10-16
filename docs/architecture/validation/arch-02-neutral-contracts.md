# ARCH-02 neutral contracts

Status: implemented; the [independent review](arch-02-review-2026-10-16.md)'s finding (HC-00) and the recording
padding defect found here (HC-01) are corrected (section 9), awaiting independent acceptance (2026-10-16). ARCH-03 has
not started.
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

- **Recording padding (HC-01).** `RecordWriter` wrote its callers' structs and its own snapshot head byte for byte.
  Their padding is unspecified, so two recordings of the same world could differ in padding bytes, and memory contents
  reached the file. Readers were unaffected. This predates ARCH-02 and was found while writing the framing regression.
  It is corrected in its own change (section 9).

## 8. Next

ARCH-03 (foundation and domain leaf moves) may start after independent acceptance of this package. The remaining
exceptions are removed by the packages named in them.

## 9. Corrections: HC-00 and HC-01

Taken from the [independent review](arch-02-review-2026-10-16.md) of `6c0de98` and the
[handoff carryover register](handoff-carryover-register.md). They are separate corrections: neither closes the other.

### HC-00: public include files of any suffix

The public inventory scanned only `.h`-like suffixes, and an include of an unclassified public file was skipped. The
review's chain was `session -> fsim/Bridge.inc -> fsim/VecEnv.h`; the checker accepted it with `Bridge.inc` neither
classified nor scanned.
- Every regular file under `include/fsim` is now public, whatever its suffix, because the install copies the directory
  whole. Each is classified and scanned like a header.
- Source scanning adds the fragment suffixes `.inc`, `.ipp`, `.tpp`, `.ixx` and `.hxx`.
- Explicit failures:
  - an include of an unclassified public file is reported where it is included (`unclassified-public-include`),
    beside `unclassified-public-header`;
  - a resolved private include whose suffix is still not scanned fails (`unscanned-include`);
  - a public file that is not UTF-8 text fails (`unsupported-public-file`).
- Regressions in `tools/architecture/test_check.py`:
  - the review's unclassified `.inc` chain, rejected at its include and as unclassified;
  - the indirect simulation-to-training chain through a classified fragment;
  - the indirect contract-to-domain chain through a `.ipp` (`Public.h -> Values.ipp -> World.h`);
  - a valid `.tpp` control that passes;
  - an unscanned private include and a non-text public file.

  The five negative cases fail on `6c0de98`. The real tree is unchanged: 41 classified headers and the same three
  public exceptions.

### HC-01: recordings carry no unspecified padding

`RecordWriter` (`src/ipc/Recording.cpp`, the only recording writer) now writes each record as its own bytes with its
padding zero.

| Record | Padding written as zero |
| --- | --- |
| Header | bytes 4-7 and 84-87 |
| Table row | bytes 4-7 and 402-407 |
| Snapshot head | bytes 12-15 |
| Sample | state byte 391 (after its three flags) and bytes 604-607 (after its wheel count) |
| Stores, environment, events | none (no padding) |

- **Where the gaps come from:** the frozen member lists in `tests/test_state_layout.cpp` cover every member of these
  records. The writer derives each gap from the members on either side. Compile-time checks hold it to the frozen
  sizes and offsets, and a sample's state, inputs and derived values follow each other without padding.
- **What did not change:** record sizes, offsets, the format version (7), every member's bytes and the readers.
- **Regressions in `tests/test_state_layout.cpp`:**
  - The same world is recorded twice, its structs made over different background bytes (0xA5, 0x5A). The files must
    be identical, with every padding byte zero at its literal file offset. On the unfixed writer this failed at the
    header's padding (file offset 10).
  - The frozen-framing case now also pins the whole file's FNV-1a digest (`0x25f7a36e4eb3062e`, 2,928 bytes), possible
    now that the bytes are deterministic. It was identical on three runs.
  - The v1-v7 old-recording reads are unchanged and pass.

| Verification (2026-10-16) | Result |
| --- | --- |
| Builds, headless and Release, warnings as errors | 0 warnings each |
| `tools/architecture/test_check.py`; `check.py` on the repository | 128 cases passed (6 added); accepted, unchanged counts and exceptions |
| Targeted headless CTest | 17/17 passed in 46 s: the recording round trips, old-recording reads, stores/events/environment records, the SDK's recording reader, strike capture, native showcase, layout boundary, contracts, architecture checks, batch-recording adapter, and the Python world/recording binding modules |
| Targeted Release CTest | 6/6 passed in 47 s: strike capture, native showcase, layout boundary, contracts, viewer screenshot, the SDK's recording reader |
| Frozen-baseline compare, both builds | 19 comparisons, 0 failures; 1,456 exports, layouts, enumerators and vtables unchanged |

No full suite ran.

**Observed, not changed (HC-10 in the register).** The shared-memory publisher (`WorldPublisher::publish`) assigns each
whole `VehicleState` into the segment, so its 5 padding bytes carry the caller's unspecified bytes to local readers.
That is shared memory, not a file, and outside HC-01's recording scope. It is registered for the transport owner.


## 10. Correction: HC-10 (shared-memory padding)

Taken from the [handoff carryover register](handoff-carryover-register.md): verify the raw published bytes, and correct
them only if padding is confirmed to propagate.

**Measured.** A generated probe compiled `offsetof` and `sizeof` of every member of every plain record the world
publisher writes: `VehicleState`, `ControlInputs`, `EnvironmentState`, the table row, the slot header, the sample, the
mission record with all its nested records, the store, the engagement event and its ring. Only two have padding:
- `VehicleState`: byte 391 and bytes 604-607, reaching the segment through each sample's whole-state assignment.
- `MissionRangeDoppler`: bytes 52-55, at mission-record bytes 13,220-13,223, reaching it through `setMission`'s
  whole-record copy.

The table rows and slot headers are written member by member into the zeroed segment, and the environment and events
have no padding. The camera segment's header is value-initialized (its padding zero) and its records are written
member by member.

**Confirmed.** The new raw regression in `tests/test_state_layout.cpp` publishes two vehicles and a mission record
whose padding bytes are explicitly poisoned (0xA5). It maps the segment by its name, as a viewer does, and reads the
latest slot's bytes. On the unfixed publisher, all 14 padding bytes failed (5 per vehicle and the 4 range-Doppler
bytes): the caller's padding reached shared memory. Every member byte matched the caller's.

**Corrected.** The layout contract (`contracts/world_stream/WorldLayout.h`) now defines the two padded layouts' gaps
once (`kVehicleStatePadding`, `kRangeDopplerPadding`) and a `zeroPadding` helper.
- The gaps are derived from the members on either side and held to the frozen offsets at compile time.
- `WorldPublisher::publish` zeroes each published state's padding, and `setMission` the range-Doppler block's, inside
  the existing seqlock window. Readers see the same protocol.
- The recording writer (HC-01) now takes its sample gaps from the same definition.
- Unchanged: layouts, member values, publication timing and sequence protocol, the recording bytes (the frozen digest
  `0x25f7a36e4eb3062e` still matches) and the exports.

| Verification (2026-10-16) | Result |
| --- | --- |
| Raw shared-memory regression | Failed on the unfixed publisher at all 14 padding bytes; passes now (three runs), with every member byte equal to the caller's |
| Builds, headless and Release, warnings as errors | 0 warnings each |
| Targeted headless CTest | 21/21 passed in 47 s: publish/mirror, the viewers' radar, load-limit and model-origin records, the recording round trips and old reads, strike capture, native showcase, layout boundary, contracts, architecture checks, Python world/recording bindings |
| Targeted Release CTest | 9/9 passed in 47 s: the same publication cases, strike capture, native showcase, layout boundary, contracts, viewer screenshot |
| Frozen-baseline compare, both builds | 19 comparisons, 0 failures; 1,456 exports, layouts, enumerators and vtables unchanged |
| Architecture gate | Accepted; the same counts and exceptions |

No full suite ran.
