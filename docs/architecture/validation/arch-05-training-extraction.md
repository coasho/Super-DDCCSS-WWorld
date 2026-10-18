# ARCH-05 training environment and mission extraction

Status: independently accepted at `58f9297` (2026-10-16); see the
[independent review](arch-05-review-2026-10-16.md). ARCH-06 has not started.
Starting main HEAD: `6e79873` (ARCH-04 accepted, with HC-15's placement decided).

This is a relocation and target split. No source line changed except includes and path comments, and no model, RNG,
timestep, default, schema, receipt, cost or lifecycle rule changed. Every first-party object file both presets build is
unchanged in code, data and symbols, including the 11 that moved. The installed composition example's traces are
byte-identical to those before the move (section 5).

## 1. What moved

| Destination | Files | Target |
| --- | --- | --- |
| `src/training/native/vecenv` | `VecEnv`, `Task`, `Observation`, `Registry`, `Scenario.h`, `ScenarioVecEnv.cpp` (from `src/env`) | `fsim_env` |
| `src/training/native/episodes/runtime` | `MissionEpisode.h/.cpp` (from `src/env`): the native episode owner | `fsim_env` |
| `src/training/native/mission` | `MissionConfig`, `MissionJson`, `MissionGateway` (from `src/sdk`) | new `fsim_training_mission` |
| `src/training/native/batch` | `MissionBatch.cpp` (from `src/sdk`) | `fsim_training_mission` |
| `src/training/native/episodes/api` | `MissionEpisode.cpp`, `MissionEpisodeImpl.h` (from `src/sdk`): the public mission facade | `fsim_training_mission` |

Further details:
- **Moves:** 21 source files and env's `CMakeLists.txt` (now `src/training/native`'s) moved with `git mv`, each a
  rename.
- **Names:** `fsim_env` keeps its name and alias, and namespaces are unchanged. The policy module `mission` is
  registered as `fsim_training_mission` (alias `fsim::mission`) and is no longer planned.
- **Includes:** 43 literal includes in 24 files name the new paths.
- **Two episode owners kept apart:** the native episode owner (`episodes/runtime`) and the public mission facade
  (`episodes/api`) stay in separate directories and targets.
- **Stays in the SDK:** the C ABI adapter `fsim_mission_env_c.cpp` stays in the SDK assembly; it moves with the C
  adapters in ARCH-07.

## 2. Targets and graph

| Target | Links (direct) | Notes |
| --- | --- | --- |
| `fsim_env` | `simulation_api`, `session`, `control`, `sim`, `io`, `core` (unchanged) | VecEnv and the native episode owner |
| `fsim_training_mission` | `env`, `simulation_api`, `session`, `core` | Uses the World and Agent through the lower facade, the native episode owner and the runtime's MaCore. Needs no symbol from the SDK assembly |
| `fsim` (final SDK) | `mission` added to its `WHOLE_ARCHIVE` list; the mission sources leave its own list | Only `fsim_mission_env_c` (C ABI) consumes the mission code |

Further details:
- **Include views:** both training targets export include views, not the shared `src` root. `fsim_include_view`
  gained an optional `DIRECTORIES` list for a target whose destinations are several directories: `env` covers
  `vecenv` and `episodes/runtime`; `mission` covers `mission` and `episodes/api` (`batch` holds no header).
- **Compile settings:** `fsim_training_mission` compiles as the SDK compiles its sources (`FSIM_BUILDING`, hidden
  visibility).
- **One definition:** the SDK's link line takes mission and env objects only from their archives, so each exported
  mission method has one definition.

There is no training-to-SDK-to-training cycle:
- **Symbols:** the mission objects reference nothing that the rest of the SDK assembly defines.
- **Policy:** the policy lets the SDK use `mission`, never the reverse.
- **Fixture:** a gate fixture rejects the reverse link (as a forbidden kind and a cycle) and an include of the
  assembly's private headers.
- **Link gate:** `training_link` links every object of both training targets with only their declared closure.

## 3. Roadmap tasks

| Task | Done |
| --- | --- |
| 1. VecEnv separate from independent mission episode ownership | `vecenv/` and `episodes/runtime/` in separate directories; the public facade `episodes/api/` sits in its own target |
| 2. Config, gateway, batch and lifecycle in training, over the lower simulation facade | `fsim_training_mission`, linking `simulation_api` and no SDK target |
| 3. EpisodeHold, all-slot preflight, final-document ownership and correct Busy retry | No code changed, and every object is identical. `mission_training` and its three stress cases, `mission_c_abi` and its two stress cases, and the Python Busy/batch concurrency tests pass |
| 4. Public mission wrappers and `_native` runtime type owners compatible | Exports, vtables and Python inventories identical; frozen C++/C99/Python consumers pass on the new installed SDK. No Python source changed |
| 5. Actor/referee separation, non-consuming fanout, deduplication, native refusal meaning, seeds and costs | Objects identical; `test_isolation` (costs refused before the actor), `mission_training`, `ma_*`, `mission_episode` and `native_state_reset_fixture` pass |
| 6. The real installed M6 composition traces and the preopened-child regressions | Section 5 |

Python helpers, examples and workload tools are in the write scope, but no task needed them moved. The private Python
separation is ARCH-07's (training plan, section 5), and nothing in Python changed here.

## 4. Policy, gate and tests

- **Policy:** `mission` is registered (target `fsim_training_mission`, destinations `src/training/native/mission`,
  `batch` and `episodes/api`), with allowed dependencies `env`, `simulation_api`, `session` and `core`, equal to its
  links. `env` lists no legacy directory. `sdk` may use `mission`. The architecture gate accepts 23 modules and 374
  files, with the same exceptions as before.
- **Gate fixture:** `test_mission_training_stays_below_the_sdk_assembly_that_links_it` registers the mission module
  at its destination over the episode owner, with the assembly linking it, and is accepted. It rejects a link back to
  the assembly (forbidden kind, and the cycle `mission -> sdk -> mission`) and an include of the assembly's headers.
- **Link gate:** `training_link` (`tests/training_link.cpp`) links `fsim_training_mission` and `fsim_env` whole, with
  only their declared closure, and runs the program.
- **Include views:**

  | Probe set | Allowed headers compile | Forbidden headers not found |
  | --- | --- | --- |
  | `include_view_env` | VecEnv registry, native episode owner, mapper, World | `MissionJson.h` (the mission implementation above it), `sdk/LastError.h` |
  | `include_view_mission` | gateway, facade implementation, native episode owner, mapper | `sdk/Handles.h`, `sdk/CAbiSupport.h`, `sdk/LastError.h` |

  The `session` and `simulation_api` probes name `env/Registry.h`'s new path.
- **Selector:** the MA gateway, episode-owner and concurrency scopes, the shared-core scope and the one selector test
  that reads the real rules name the new paths. `src/training/native/CMakeLists.txt` is nested, so it falls outside
  the flat build-file pattern and escalates through the fail-closed fallback (HC-13), as env's CMake file did through
  the build-system scope.

## 5. Verification (2026-10-16)

| Check | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings |
| Object identity: per source, every section's contents and the symbol table, against the `6e79873` builds | All 288 headless and 336 Release first-party objects unchanged, 11 of them moved; the only new object is the link gate's |
| `tools/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures; all 22 export, vtable and Python-inventory captures byte-identical to ARCH-04's |
| Frozen baseline (`e1945b9`): `verify`; `run` of the frozen C++, C99 and Python consumers on this build's installed SDK and package | 66 artifacts unchanged; all five consumers exit 0 |
| Installed M6 composition (`mission_package_consumer`, both presets): its traces before and after the move | Both runs' traces byte-identical to the pre-move traces on each preset; the test itself requires same-after-reset and same under the training adapter (`same_step`, `next_step`) |
| Fast tier (ARCH-03's test names, ARCH-04's four tests and ARCH-05's three) | Headless 575/575 (116 s); Release 640/640 (121 s) |
| Related training gates outside the tier, headless: `mission_episode`, `native_state_reset_fixture`, `mission_training` and its stress cases, `mission_c_abi` and its stress cases, `sdk_boundary`, the `ma_*` suites, the Python mission, training, isolation, composition, MA, agents, adapters, VecEnv and world tests, and the five Python mission-concurrency cases, including the three preopened-handle regressions | 32/32 (213 s) |
| Architecture: gate; `test_check.py`; selector `test_scope.py`; planning document checker | Accepted (23 modules); 132 cases; 49 cases (1 skipped); accepted |
| Include environments ([arch-05-include-views.json](arch-05-include-views.json)) | All 130 compiled sources of the 14 compiled migrated targets, per preset, use exactly the recorded roots; none holds the shared `src` root |

The dated architecture baseline is [arch-05-baseline.json](arch-05-baseline.json). No full suite ran: the selector
escalates this package to the full suite, which stays HC-11's deferred evidence. Logs are in `D:/ENV/tmp/arch05/`.

## 6. References

- **Comments and documents:** the runtime's scenario comment, `docs/development.md`, the mission-training page's
  owner reference and the architecture design document's source-tree row name the new paths. In the design document,
  which carries another owner's uncommitted edits, only that row is committed.
- **Kept as written:** dated status entries and plans, `tools/architecture/README.md`'s ARCH-02 text, and the
  architecture and selector tests' synthetic fixtures.

## 7. Rollback

Move the 21 source files and the CMake file back with `git mv`, and restore env's CMake content and the SDK's source
and whole-archive lists. Then revert:
- the include rewrite;
- the policy;
- the probes, the link gate and the gate fixture;
- the selector paths;
- `fsim_include_view`'s `DIRECTORIES` option;
- the listed comment and document lines.

Keep one runtime and type owner throughout: do not substitute `Vehicle.reset` or a Python fallback. The compiled
objects are identical either way; verify the export lists after a rollback.

## 8. Next

ARCH-06 (visualization core, effects and source separation) requires the next user-relayed assignment after
independent acceptance. The open carryovers are in the [register](handoff-carryover-register.md).
