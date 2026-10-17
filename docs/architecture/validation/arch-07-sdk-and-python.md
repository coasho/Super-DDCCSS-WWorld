# ARCH-07 SDK assembly and Python import cleanup

Status: implemented, awaiting independent acceptance (2026-10-16). ARCH-08 has not started.
Starting main HEAD: `74efb49` (ARCH-06 accepted).

This package leaves the final SDK assembly thin and moves the Python extension sources into the adapters. It moves the
Python implementation behind the public modules into private packages and removes both deferred import cycles. The
camera core no longer imports the VecEnv, and no public header exception remains. One path-map row is blocked by
ownership: P056, `fsim.isolation`'s codec and proxy, whose public module carries another owner's uncommitted edits
(register HC-21, section 9).

No model, RNG, timestep, layout, default or public identity changed:
- **Native objects:** every first-party object both presets build is unchanged in code, data and symbols, apart from
  the split `fsim.cpp` (section 2). That includes the 20 moved headless and 21 moved Release objects.
- **Exported surface:** exports, vtables, typeinfo and the C and C++ public layouts are byte-identical to the
  pre-change capture.
- **Python:** every public Python name keeps its owner, qualified name, signature, fields, defaults, members and
  pickling (section 6).

## 1. What moved

| Destination | From | Target or package |
| --- | --- | --- |
| `src/adapters/sdk/cpp` | `src/sdk/Scenario.cpp`; `src/sdk/fsim.cpp`'s C++ part, now `VecEnv.cpp` | `fsim` (the DLL) |
| `src/adapters/sdk/c` | `src/sdk/fsim_*_c.cpp` and the private headers; `fsim.cpp`'s C ABI core, now `fsim_c.cpp` | `fsim` |
| `src/adapters/sdk/python` | `python/src/_native*.{c,h}` | `fsim_python` (`fsim._native`) |
| `src/adapters/sdk/python/abi` | `python/src/fsim_py.h` | new `fsim_python_abi` |
| `src/adapters/vision/python` | `python/src/_vision.c` | `fsim_python_vision` (`fsim._vision`) |
| `python/fsim/_training` | `training.py`'s implementation | `fsim._training` |
| `python/fsim/_simulation` | `world.py`'s 16 records World and Agent share; new agent and referee factories | `fsim._simulation` |
| `python/fsim/_visualization` | `vision.py`'s `Camera` and `Vision` | `fsim._visualization` |
| `python/fsim/_contracts` | `world._options`; the owner declaration the private packages use | `fsim._contracts` |

Further details:
- **Renames:** 33, all with `git mv`: 16 SDK sources and headers with its CMake file, and the 16 extension sources.
- **Names:** target names, aliases, DLL and module names are unchanged.
- **Includes:** the SDK's private headers are spelled by name (`"Handles.h"`): HEAD's 25 `sdk/...` includes, two of
  them now split between `VecEnv.cpp` and `fsim_c.cpp`.

## 2. SDK assembly (tasks 1 and 2)

- **Thin assembly:** `src/adapters/sdk` holds exports, handles, error and principal conversion, and composition only:
  - `cpp/` is the C++ compatibility facade: the VecEnv pimpl and `applyScenario`'s delegation (HC-15);
  - `c/` holds the sized C ABI adapters with their handles, error string and principal guards.

  The World, Agent and recording classes stay simulation's (`simulation/api`), and the mission facade stays training's,
  linked whole as before.
- **The `fsim.cpp` split:** P019 is a split, then a move. The C ABI core moved to `c/fsim_c.cpp`: the version, the
  per-thread error string every adapter writes, the log level and the registry listing. The C++ VecEnv facade is
  `cpp/VecEnv.cpp`. HEAD's `fsim.cpp` was compiled with the new unit's exact command and compared function by function
  (disassembly with relocations, section offsets and padding normalized):
  - all 45 symbols it defined are defined by exactly one of the two new objects;
  - every function is unchanged except the throw paths of the VecEnv constructor, `reset` and `step`. These call the
    exported `fsim_last_error()` out of line, where they inlined its thread-local read in the same unit before. The
    message thrown is the same string;
  - the compiler's local clone of that read (`.part.0`) is gone;
  - the typeinfo both units now emit is COMDAT and folded by the linker.
- **HC-15:** `applyScenario` is at `src/adapters/sdk/cpp/Scenario.cpp` with its public signature, single definition and
  delegation to `session::applyScenario`. Its object is unchanged. No simulation-to-training exception was added.
- **Include paths (HC-14):** the SDK compiles with `PRIVATE src/adapters/sdk/c` and the views of the modules it links,
  no longer with the `src` root. A new probe, `include_view_sdk`, shows the effect on both presets:
  - the runtime, the lower facade and training are visible;
  - its own header by a `src`-root spelling (`adapters/sdk/c/Handles.h`), the demo application and a scene header are
    not.

  Five existing probes named the old `sdk/...` spellings as forbidden headers, which the probe check would now report
  as stale. They name `adapters/sdk/c/...` instead.
- **Public value split:** `VecEnv.h` (training facade) included the C ABI only for the handle `handle()` returns. That
  handle is now its own contracts value header, `fsim/fsim_vecenv_handle.h`, which `fsim_c.h` includes, and the
  `VecEnv.h -> fsim_c.h` public exception is removed. Code that includes `VecEnv.h` and calls C functions on
  `handle()` now includes `fsim_c.h` itself. No repository consumer did; the VecEnv reference says so (HC-22).

## 3. Extension modules

The gate reads `src` CMake only and refuses helper functions and non-literal links, so the extensions are declared
in their adapter directories with plain CMake:
- **Declarations:** `python/CMakeLists.txt` finds CPython and defines `cpython::abi3`, an imported target carrying the
  interpreter's headers (system, as before) and `python3.dll`. It adds both adapter directories (`fsim._vision` only
  with the renderer) and stages the package as before.
- **`fsim_python_abi`** (module `python_abi`) owns `fsim_py.h` and the stable-ABI settings both modules compiled with:
  `Py_LIMITED_API`, the C warning set and the CPython link.
- **Unchanged builds:** the compiled objects of all 11 extension sources are unchanged, and `fsim._native` remains the
  one native type-registration owner. `fsim._vision` is never registered in it.
- **Baseline record:** `tools/architecture/baseline.py` follows `python/CMakeLists.txt`'s `add_subdirectory` into the
  declaring files, keeping the enclosing `if()` branches. The record names each extension's declaring file
  (`declared_in`).

## 4. Python packages (tasks 3 to 6)

- **Owner declaration:** a public class, record or function whose code moved keeps its public module as owner.
  `fsim._contracts.public(module)` sets `__module__`, records use `namedtuple(..., module=...)`, and the public module
  re-exports the name.
- **`fsim._training`** (P051) holds the training adapter and section 8's seed streams. It takes `Error` from its owner,
  `fsim._native` (`fsim.Error` is `_native.Error`), so the `fsim.training -> fsim` backedge is gone. `fsim.training` is
  the docstring and the re-exports.
- **`fsim._simulation`** (P054, P055):
  - `values` holds the 16 records World and Agent both build, owned by `fsim.world`. `fsim.agent` takes them from there.
  - `factories`: `World.create_agent` and `World.referee` build through it, and `fsim.agent` registers `Agent` and
    `Referee` when it loads. The package root loads it with `fsim.world`, before any World exists. The
    `fsim.world -> fsim.agent` backedge is gone.
- **`fsim._visualization`** (P053): `cameras` holds `Camera` and `Vision`, owned by `fsim.vision`. It imports neither the
  VecEnv nor the simulation wrappers: `fsim.vision` binds the public World and Vehicle types it accepts (`bind`), so
  `Vision(world)`'s type check and the vehicle-or-id arguments behave as before.
- **Training-image composition** (task 5, HC-17.4): `VisionBatch`, the VecEnv's batch with a camera per vehicle, is
  composed in `fsim.vision`, outside both the camera core and the training core. The native `BatchCameras` stays in the
  Vision adapter as the compatible public C++ and C API. The native visualization core had no VecEnv after ARCH-06.
- **Kept as they were:** `fsim.mission` (P052, already thin); the eager imports of `fsim/__init__.py`, which the public
  `fsim.World`, `fsim.Agent` and others need (HC-04); and `fsim.isolation` (P056, HC-21).
- **Gate:** no Python exception remains. The gate scans 26 modules with 58 imports and finds no cycle, deferred
  included.

## 5. Policy, gate and tests

- **Policy:**
  - `sdk` holds no legacy directory;
  - `python_abi`, `python_native` and `vision_native` are registered, no longer planned;
  - `cpython::abi3` is an external target;
  - `fsim/fsim_vecenv_handle.h` is classified (contracts value);
  - the public exceptions and the Python exceptions are empty.

  The gate accepts 28 modules and 396 files. With this policy, HEAD's tree is rejected (29 issues):
  - both deferred Python cycles;
  - `VecEnv.h -> fsim_c.h`;
  - the SDK's `src` root;
  - the unowned `src/sdk`;
  - the missing extension targets.
- **Tests:**
  - **Gate:** `test_check.py` adds `test_an_added_adapter_directory_declares_its_extension_under_the_enclosing_branch`,
    which fails with HEAD's `baseline.py`.
  - **Python SDK:** `python/tests/test_package_imports.py` (`python_sdk test_package_imports`):
    - the core modules import in a fresh interpreter with Gymnasium, Stable-Baselines3, PyTorch and `fsim._vision`
      blocked, and load none of them;
    - the optional modules then fail with ImportError;
    - the training names and the shared records keep their owners and pickle;
    - the training adapter's `Error` is `fsim.Error`;
    - the cameras keep `fsim.vision` and the camera core holds no VecEnv. This case skips where `fsim.vision` is not
      built.
  - **Probes:** `include_view_sdk`, both presets.
- **Selector:** `scopes.toml` names the moved paths. The `_training` implementation joins the stress and MA Python
  scopes, and `_visualization` joins the shared renderer scope. `python/src/` left the escalating roots. SDK assembly
  sources stay unclaimed and escalate, as `src/sdk` did.

## 6. Verification (2026-10-16)

| Check | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings |
| Object identity: per source, every section's contents and the symbol table, against the `74efb49` builds | Headless: 269 unchanged in place, 20 moved unchanged; Release: 317 and 21. Only `cpp/VecEnv.cpp` differs and `c/fsim_c.cpp` is new (the split, section 2) |
| The split, function by function against HEAD's `fsim.cpp` compiled with the same command | 45 symbols, each in one new object; differences only on the three throw paths (section 2) |
| `tools/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures. Against the `74efb49` capture: exports, vtables, Python call lists and the C and C++ layouts byte-identical; the class dump has the same 445 classes, reordered; the Python inventory adds only the new private packages |
| Python identity: every public name of 12 (headless) and 13 (Release) modules: owner, qualified name, signature, fields, defaults, docs, enum members, class members and a pickle round trip | 494 and 501 names, 0 differ |
| Frozen baseline (`e1945b9`): `verify`; the frozen C++, C99 and three Python consumers on the new installed Release SDK, then the frozen Python scripts on the new installed package | 66 artifacts unchanged; 5 and 5 exit 0. The second run's provenance names the installed package (`install/python/fsim`, its `_native.pyd` and `libfsim.dll`) |
| Installed Python package and wheel | `fsim/_contracts`, `_simulation`, `_training` and `_visualization` installed and in the wheel (`fsim-0.1.0-cp311-abi3-win_amd64.whl`, 1,291 entries) |
| Related fast tests (section 7) | Headless 622 tests: 621 passed, and `include_view_sdk` failed on a forbidden header that needs VSG, which a probe cannot test without it. With a GPU-free scene header (`visualization/scene/Interpolator.h`) it passes; it was rerun with the four tests whose end overlapped that rebuild, 5/5. Release 682/682 (257 s). No skips |
| Include environments ([arch-07-include-views.json](arch-07-include-views.json)) | The SDK's 12 compiled sources use exactly the recorded roots on each preset: its own `c/`, the public `include` and the views of the modules it links; no `src` root. The earlier targets are unchanged |
| Architecture: gate; `test_check.py`; selector `test_scope.py`; planning document checker | Accepted (28 modules, 396 files); 134 cases; 74 cases (1 skipped; 74/74 with the Release inventory); accepted (25 documents, 74 source units, 420 files) |

The dated architecture baseline is [arch-07-baseline.json](arch-07-baseline.json). Logs are in `D:/ENV/tmp/arch07/`.

Limits:
- **Full suite:** none ran (HC-11).
- **Order of work:** the SDK probe's correction and rebuild ran while the headless set's last four tests were still
  running, against the one-build-or-test-job rule. The rebuild compiled and linked nothing (a reconfigure and the package's
  copy-if-different staging). Those four tests were rerun alone and pass.
- **Toolchain:** builds and the ABI comparison ran from PowerShell. The Bash tool's compiler invocations exit 1
  without a diagnostic in this environment, whatever TMP is.

## 7. Test scope

The selector escalates this package to both full suites: SDK assembly sources and `fsim_c.h` are unclaimed, as before.
As the package's instructions and HC-11 require, the related fast set ran instead:
- **The fast tier:** ARCH-06's names, every test measured under 5 s plus the named gates. These include the installed
  C++, C and Python package consumers, `simulation_interface_boundary`, `c_abi` and the Python SDK tests.
- **Outside the tier:**
  - every include-view probe, including the new `include_view_sdk`, with `include_view_checker` and
    `simulation_api_link`;
  - `sdk_boundary` and `ma_baseline_checker`;
  - the MA and training gates (`mission_episode`, `native_state_reset_fixture`, `mission_training`, `mission_c_abi`,
    `ma_joint` and the other `ma_*` suites) with their stress cases;
  - `dist_components` and `rust_example`;
  - every Python SDK module but five flight and radar physics modules (`test_fleet`, `test_launch`, `test_load_limits`,
    `test_mass_accounting`, `test_radar`), with the four mission-concurrency stress cases.

Headless ran 622 of 712 tests and Release 682 of 771. Not run: the long flight, fleet, radar and conformance suites,
hangar's tests, the launcher contract, the native engagement suite and the Flow calibration. This package does not
change their code: their objects are unchanged.

## 8. References

- **Updated:**
  - `docs/development.md`'s adapter owners;
  - the Python reference (`docs/sdk/python.md`, the package layout and imports);
  - the VecEnv reference (`docs/sdk/vecenv.md`, the C handle header);
  - the engagement contract's link to the C mission adapters;
  - the architecture design document's two source-tree rows;
  - the architecture tool README's Python-cycle and public-exception text;
  - `boundary.py`'s comment on where adapters live.

  The last four files carry another owner's edits; only these lines are committed.
- **Kept as written:**
  - the dated maintenance-slice prose naming `src/sdk` files in the design document;
  - dated records and plans;
  - synthetic fixture paths.

## 9. Open items

- **HC-21:** P056 is done by section 13, with the other owner's uncommitted edits to `fsim.isolation` kept in place.
- **HC-22:** `VecEnv.h` no longer includes `fsim_c.h` (section 2), a source-level change for a consumer that relied on
  it. Binary compatibility and every repository consumer are unaffected.
- **HC-04:** revisited, still open. The package root still imports its public modules eagerly, so a change to any
  `fsim` module still selects every Python test that imports `fsim`.

## 10. Rollback

1. Move the 33 files back with `git mv`, restore `fsim.cpp` from `fsim_c.cpp` and `VecEnv.cpp`, and delete the two
   extension CMake files and `fsim_vecenv_handle.h`.
2. Restore `python/CMakeLists.txt`'s helper and `src/sdk/CMakeLists.txt`'s `src` root.
3. Restore `training.py`, `world.py`, `agent.py` and `vision.py`, and delete the four private packages and the new test.
4. Revert:
   - the policy, restoring its `VecEnv.h` and two Python exceptions and the planned modules;
   - the probes;
   - the selector paths;
   - `baseline.py` and its test;
   - the listed document lines.

No native model parameter is involved.

## 11. Next

ARCH-08 (applications, tests, examples and packaging) requires the next user-relayed assignment after independent
acceptance.

## 13. HC-21 completion: P056, and HC-23 (2026-10-17)

Status: awaiting independent acceptance. ARCH-08 has not started.

The 2026-10-17 assignment released `python/fsim/isolation.py` for P056, provided its pending uncommitted edits are
preserved. They remain uncommitted and unchanged, in the lines they were made on.

**The split:**
- **`fsim._adapters.codec`** holds the values that cross the actor's local pipe: the named-tuple and enum registry,
  `_encode` and `_decode`, `_dump` and `_load`, exceptions as data and their rebuild. It also holds `AgentError` (owner
  `fsim.isolation`) and the reference an activity decodes to in the MA's process. The function bodies are unchanged
  except in two respects:
  - `BatchCommand` and `AgentActivity` are bound by `fsim.isolation` (`bind`), no longer imported from `fsim.world` and
    `fsim.agent` inside the functions;
  - the registry takes the loaded package from `sys.modules`, no longer `import fsim`.
- **`fsim._adapters.actor`** holds the actor-side proxies, `RemoteAgent` and `RemoteMissionEndpoint` (owner
  `fsim.isolation`), their channel, and the actor process's loop, `serve`. `serve` is the registered dynamic-import site;
  it takes the callable-method lists from `fsim.isolation` instead of computing them.
- **`fsim.isolation`** (662 to 396 lines) keeps:
  - its docstring and the callable-method lists;
  - the Range-side checks (`_carried`, `_own_endpoint`);
  - the Range-side process hosts `AgentProcess` and `MissionEndpointProcess`, which hold the Range's agent, episode and
    endpoint. The SDK keeps actor endpoint proxies separate from Range handles.

  `-m fsim.isolation` still starts the actor process, through `_serve_ma`, which calls `serve`. The module re-exports
  the moved public names and the codec names its hosts and tests use. The two bound classes are imported under private
  aliases, so its namespace is unchanged: in a first version, the inventory comparison showed `fsim.isolation.BatchCommand`
  and `AgentActivity` appearing.

**Preserved edits:** the other owner's four hunks lie in the regions that stay: the docstring's title and two paragraphs,
`mission_endpoint_calls`' docstring, and a comment and a docstring in `MissionEndpointProcess` (16 changed lines). The
same edits were applied to HEAD's text, which is committed, and to the working file. The difference between the two is
exactly those 16 lines, unchanged before and after. The other files carrying uncommitted edits were handled the same
way: their edits to `isolated_mission_actor.py` are untouched, and those to `check.py`, the architecture tool README and
this record are kept beside the committed blobs.

**HC-23:**
- **Gate:** a planned module's phase may name a module plan's package (`SIM-x`, `TRN-x`, `VIS-x`) as well as a roadmap
  package. Exceptions still expire with a roadmap package.
- **Policy:** the native `training_vision` reservation waits for VIS-F and stays planned.
- **Planning target graph:** the native entry's status is `planned-VIS-F`, as `oms` records its feature phase, with a
  follow-up note. The Python entry notes that the accepted `VisionBatch` composition is `fsim.vision`'s (HC-17.4).
- **Consistency:** the reservation's destination, planned status and optional render build agree with the planning
  target. No provider was built.

| Check | Result |
| --- | --- |
| Gate | Accepted: 28 native modules, 29 Python modules, 63 imports, no cycle; the dynamic-import site is `fsim._adapters.actor:serve` |
| `test_check.py` | 135 cases. The new `test_a_planned_module_may_wait_for_a_module_plan_package` fails 3 subtests with HEAD's `check.py`, and HEAD's gate rejects the new policy's `VIS-F` |
| Python identity, both presets | 494 and 501 names, 0 differ; `fsim.isolation`'s 7 names unchanged |
| `baseline.py compare`, both builds | 19 comparisons, 0 failures. Against ARCH-07's capture, exports, vtables, Python call lists and the C and C++ layouts are byte-identical. The Python inventory adds only `fsim._adapters`. The full class dump differs only in compiler addresses in standard-library construction-vtable lines |
| Frozen C++/C99/Python consumers on the new installed package | 5/5 exit 0; provenance names the installed package |
| Wheel | Contains `fsim/_adapters` (1,294 entries) |
| Related fast tests, headless | 22/22 (132 s): the isolation, mission, MA joint and equipment modules, the four mission-concurrency cases, `test_package_imports`, `test_mission_training`, the installed mission consumer, `sdk_boundary`, `ma_baseline_checker`, the architecture, selector and documentation gates. Then the rest of the selector's 41-test plan for these paths, 26/26 (14 s) |
| Related fast tests, Release | 6/6 (16 s): isolation, mission, package imports, `sdk_boundary`, the architecture and selector gates |
| Selector `test_scope.py`; planning document checker | 74 cases (1 skipped; 74/74 with the Release inventory); accepted (26 documents, 74 source units, 423 files) |

No skips, no native object compiled or linked, and no full suite (HC-11). Logs are in `D:/ENV/tmp/hc21/`.
