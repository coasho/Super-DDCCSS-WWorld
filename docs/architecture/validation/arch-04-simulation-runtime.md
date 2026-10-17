# ARCH-04 simulation runtime, services and lower facade

Status: implemented, awaiting independent acceptance (2026-10-16). ARCH-05 has not started.
Starting main HEAD: `ad12a44` (ARCH-03 accepted, HC-12 closed).

This is a relocation and target split. No source line changed except includes and path comments, and no model, RNG,
timestep, default or registration changed. Every first-party object file both presets build is unchanged in code, data
and symbols, including the 21 that moved (section 6).

## 1. Ownership by state (task 1)

The session target was classified file by file before anything moved. Every file already had a single state owner, so
no file was split and no shared helper was extracted.

| Destination | Files | State owned |
| --- | --- | --- |
| `src/simulation/runtime/world` (`fsim_session`) | `World.h/.cpp`; the World member files `WorldDamage`, `WorldEngagements`, `Frames`, `Geometry`, `Marshalls`, `Intercepts`, `Navigation`, `PerformanceProfile`, `StateData`; the native scenario `Scenario.h/.cpp`, `ScenarioRecipe.h` | The authoritative World: vehicles, stepping, environment, frames and geometry, engagements and damage; the native recipe and its application |
| `src/simulation/runtime/services` (`fsim_session`) | `MaCore`, `MaConsentLedger.h`, `MaServiceBindings`, `MaReports`, `AgentRules` | MaCore owns C2 tasks and consent; MaServiceBindings owns service identity bindings; MaReports and AgentRules are stateless rules and builders over the World |
| `src/simulation/runtime/stores` (`fsim_session`) | `StoreFlightBinding.h/.cpp` | Store-to-flight binding state |
| `src/simulation/communications/dms` (new `fsim_modeled_dms`) | `Dms.h/.cpp` | Modeled delivery: links, queues, routing, provenance |
| `src/simulation/api` (new `fsim_simulation_api`) | `WorldOptions.h/.cpp` (from session); `World.cpp`, `Agent.cpp`, `Recording.cpp` (from `src/sdk`) | None of its own: the public World, Agent and recording facade and the public-to-native options mapping |

The 32 source files and session's `CMakeLists.txt` (now the runtime's) were moved with `git mv`, each a rename.
Target names, aliases and namespaces are unchanged: `fsim_session` keeps its name at its destination.

## 2. Targets and graph

| Target | Links (direct) | Notes |
| --- | --- | --- |
| `fsim_modeled_dms` | `comm`, `ms` | Its symbols come from MS RF propagation (`rf::computePath`, `rf::geodeticToEcef`). The `comm` edge is declared as the simulation plan requires; the current DMS sources use no generic-network symbol. Its source root is disjoint from the network target's |
| `fsim_session` | `contracts`, `modeled_dms`, `ms`, `control`, `effects`, `comm`, `ipc`, `sim`, `io`, `core` | As before, plus `modeled_dms` |
| `fsim_simulation_api` | `session`, `ipc`, `core` | Below training and the final SDK assembly; links without training or graphics (section 4) |
| `fsim_env` | `simulation_api` added | Mission episodes call `session::makeWorldOptions`, now simulation_api's (task 2) |
| `fsim` (final SDK) | `simulation_api`, `modeled_dms` added to the `WHOLE_ARCHIVE` list | Exports the facade's classes as before |

Further details:
- **Include views:** every target listed above except `fsim` and `fsim_env` exports a generated include view
  (`cmake/IncludeViews.cmake`), not the shared `src` root.
- **Compile settings:** `fsim_simulation_api` compiles as the SDK compiles its sources: `FSIM_BUILDING`, hidden
  visibility and hidden inlines.
- **Registration:** the `WHOLE_ARCHIVE` list keeps the DLL's object set; registration and per-target `FSIM_BUILDING`
  are unchanged (task 5).
- **One definition (task 4):** the SDK assembly no longer compiles `World.cpp`, `Agent.cpp` or `Recording.cpp`. Its
  link line takes them only from `libfsim_simulation_api.a`. The facade's four sources need no symbol from the rest of
  the assembly, while the mission code, `fsim_world_c`, `fsim_recording_c` and `fsim_vecenv_c` consume theirs.

### Deviation from path-map row P022: `applyScenario` stays in the assembly

`src/sdk/Scenario.cpp` (the 15-line `fsim::applyScenario`) was not moved:
- **Mixed header:** its public header `include/fsim/Scenario.h` also declares training's `vecEnvOptions` and includes
  `fsim/VecEnv.h`. The architecture gate rejects that include from a simulation module (`forbidden-public`), so
  moving the file would need a new domain exception.
- **Not needed by mission code:** task 3 asks for the facade implementations mission code needs. No mission or
  training object references `fsim::applyScenario`; its only caller is the public `scenario_runner` example.
- **Native scenario:** the native scenario implementation (`session::applyScenario`, parsing and the recipe) did move,
  with the runtime.

The register records this as HC-15 for an explicit decision.

## 3. Roadmap tasks

| Task | Done |
| --- | --- |
| 1. Split state owners, then relocate runtime/world/services/stores and modeled DMS; DMS its own target over generic comm and MS RF | Sections 1 and 2 |
| 2. Construction mapping owned at the API boundary; runtime takes native options; env's mapper call through an explicit env -> simulation_api edge; scenario/options/seed parity | The mapper is simulation_api's; the runtime still takes only the native contract (`contracts/simulation/WorldOptions.h`). Parity: `test_episode_world_options`, `mission_episode`, `native_state_reset_fixture` pass, and the mapper's object is unchanged |
| 3. Private lower simulation_api with the facade implementations mission code needs | World, Agent and Recording implementations and the mapper; `applyScenario` excepted (section 2) |
| 4. No dependency on the final SDK assembly; exactly one definition of exported World/Agent methods | Section 2; the link gate in section 4 |
| 5. Registration, `WHOLE_ARCHIVE` and per-target `FSIM_BUILDING` preserved | Section 2; export lists identical (section 6) |
| 6. Characterize authority, C2/DMS, store/flight and output-only behavior before extracting shared helpers | No helper was extracted. The characterization tests ran on the moved code (section 6), and object identity shows the compiled behavior is unchanged |

## 4. Policy, gate and tests

- **Policy:** `modeled_dms` and `simulation_api` are registered modules, no longer planned. Their allowed dependencies
  are exactly their links. `session` lists no legacy directory and owns `src/simulation/runtime`. `env` may use
  `simulation_api`; `sdk` may use `simulation_api` and `modeled_dms`. The architecture gate accepts 22 modules and 374
  files, with the same exceptions as before.
- **Gate fixture:** `test_the_lower_simulation_facade_stays_below_training_and_the_sdk_assembly` registers the facade
  at its destination below the runtime. It then shows that a link to training (`env`) or to the final assembly
  (`sdk`) is rejected by direction, and that an include of either's headers is rejected as undeclared.
- **Link gate:** `simulation_api_link` (`tests/simulation_api_link.cpp`) links every object of `fsim_simulation_api`
  (`WHOLE_ARCHIVE`) into a program with only that target's declared closure, and runs it. A symbol needed from
  training, graphics or the final SDK assembly would not resolve. This is the roadmap's gate: simulation_api links
  without training or graphics.
- **Include views:**

  | Probe set | Allowed headers compile | Forbidden headers not found |
  | --- | --- | --- |
  | `include_view_modeled_dms` | DMS, MS RF path, network | the runtime's services, physical effects, transport |
  | `include_view_session` | world, services, stores, DMS, transport | the facade's `WorldOptions.h`, `env/Registry.h`, `sdk/LastError.h` |
  | `include_view_simulation_api` | the mapper, the World, transport recording | `env/Registry.h`, `sdk/Handles.h` |

  Three existing probes named `session/...` headers that moved and now name their new paths:
  - `platform`: the include-free `simulation/api/WorldOptions.h`;
  - `comm`: `simulation/communications/dms/Dms.h`, so the network still cannot see modeled DMS;
  - `ms`: `simulation/runtime/services/MaServiceBindings.h`.
- **Selector:** the shared-core scope claims `src/simulation/runtime/**`, `src/simulation/communications/dms/**` and
  `src/simulation/api/**` in place of the old paths, so the same changes escalate as before.

## 5. References

- **Includes:** 86 literal includes in 62 files (`src`, `tests`, `tools`, `examples`, `python/src`) name the new
  paths.
- **Comments:** source comments naming the moved files (contracts, MS damage and model headers, flight control, the
  World, the facade, hangar's carriage model) and the CMake descriptions follow them.
- **Maintained documents:** `docs/development.md`, `docs/flight-autonomy.md`, the architecture design document's
  source-tree table, `docs/agent-boundary.md` and the MA engagement contract's links. In the last three, which carry
  other owners' uncommitted edits, only these path lines are committed.
- **Kept as written:** dated plans and status entries, the contracts header's "had in `session/World.h`" history,
  `tools/architecture/README.md`'s ARCH-02 text, and the architecture and selector tests' synthetic fixture paths.

## 6. Verification (2026-10-16)

| Check | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings; 142 and 164 units compiled |
| Object identity: per source, every section's contents and the symbol table, against the `ad12a44` builds | All 287 headless and 335 Release first-party objects unchanged, 21 of them moved; the only new object is the link gate's |
| `tools/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures; all 22 export, vtable and Python-inventory captures byte-identical to the ARCH-03 capture |
| Frozen baseline (`e1945b9`): `verify`; `run` of the frozen C++, C99 and Python consumers against this build's installed SDK and package | 66 artifacts unchanged; all five consumers exit 0 on the new `libfsim.dll` |
| Fast tier (ARCH-03's test names: every test measured under 5 s plus the named gates, which include the installed C++/C/Python package consumers, `simulation_interface_boundary`, `c_abi` and the Python SDK tests in the tier) plus the four new tests | Headless 572/572 (95 s); Release 637/637 (114 s) |
| Related gates outside the tier, headless (section 7) | 27/27 (112 s) |
| Architecture: gate; `test_check.py`; selector `test_scope.py`; planning document checker | Accepted (22 modules); 131 cases; 49 cases (1 skipped); accepted |
| Include environments ([arch-04-include-views.json](arch-04-include-views.json)) | All 119 compiled sources of the 12 compiled migrated targets, per preset, use exactly the recorded roots; none holds the shared `src` root |

The dated architecture baseline is [arch-04-baseline.json](arch-04-baseline.json). No full suite ran: the selector
escalates this package to the full suite, which stays HC-11's deferred evidence. Logs are in `D:/ENV/tmp/arch04/`.

## 7. Characterization (task 6)

These existing tests exercise the moved behavior and passed after the move. The before results are ARCH-03's runs of
the same code: every object has been unchanged since.

| Behavior | Tests |
| --- | --- |
| Agent authority and service identity | `ma_actions`, `ma_service_binding`, `sdk_boundary`, `python_sdk test_agents`, `test_ma_service` |
| C2 tasks, consent and DMS delivery | `ma_joint`, `ma_reports`, `ma_observation`, `ma_c_abi`, `python_sdk test_ma_joint`, `test_ma_reports`, `test_ma_composition`, `test_ma_equipment` |
| Store/flight binding | `store binding: deterministic replay ...`, `python_sdk test_mass_accounting`, `test_launch`, the tier's store and loadout cases |
| Engagements and damage | `engagement_native` and the tier's damage, guidance and countermeasure cases |
| Facade, mapper and episodes | `mission_episode`, `mission_training`, `native_state_reset_fixture`, `mission_c_abi`, `python_sdk test_world`, `test_mission`, `test_mission_training`, `test_isolation`, `test_adapters`, `test_vecenv` |
| Output only | `simulation_interface_boundary` (output boundary, state layout, recording digest), in the tier |

## 8. Rollback

Move the 32 source files and the runtime's `CMakeLists.txt` back with `git mv`, and restore its content and `src/sdk`'s
source list. Then
revert:
- the include rewrite;
- the two new targets' CMake files, env's and the SDK's link lists, and `src/CMakeLists.txt`;
- the policy's modules;
- the probes and link gate, and the gate fixture;
- the selector paths;
- the listed comment and document lines.

The compiled objects are identical either way. Verify the export lists after a rollback.

## 9. Next

ARCH-05 (training environment and mission extraction) may start after independent acceptance. The open carryovers,
including HC-15's placement decision, are in the [register](handoff-carryover-register.md).
