# Development workflow

This is the working guide for this checkout. [status.md](status.md) is the sole current capability and milestone summary; [resources.md](resources.md) identifies editable inputs and generated outputs. Design rationale remains in the architecture and ADR documents. Commands below run from the repository root in PowerShell unless stated otherwise.

## Branch and delivery policy

Use the existing `main` branch for implementation, documentation, commits and pushes. Create another branch or
worktree only when the user explicitly requests it. Fetch `origin` and inspect local/remote ancestry before
integrating existing work; use fast-forward updates when possible and push `main` after appropriate validation.
Branch names in dated delivery records describe their original execution context, not the current workflow.

## Environment and supported build

The maintained Windows build uses MSYS2 **UCRT64**, GCC, Ninja and CMake 3.25 or newer, with C++17. The presets and [toolchain](../cmake/toolchains/ucrt64.cmake) are the implementation of this contract. MSVC branches in CMake are not evidence of a verified alternative toolchain.

Keep installed environments, toolchains and dependency caches under `D:\ENV`. Reuse existing installations. Before compiling, set all temporary and extension-build directories explicitly:

```powershell
$env:TMP = 'D:\ENV\tmp'
$env:TEMP = 'D:\ENV\tmp'
$env:TORCH_EXTENSIONS_DIR = 'D:\ENV\tmp'
$env:PATH = 'D:\ENV\DevLanguages\Cpp\msys2\ucrt64\bin;' + $env:PATH
```

The toolchain defaults to `D:/ENV/DevLanguages/Cpp/msys2/ucrt64`. Override `FSIM_UCRT64_ROOT` for another UCRT64 installation under `D:\ENV`, and adjust PATH too: the preset's PATH still contains the default location. In an MSYS2 shell, the toolchain can fall back to `MSYSTEM_PREFIX` when the default compiler is absent.

The Python SDK requires 64-bit **CPython >= 3.11 from python.org or conda**, with `python3.dll` and `Python.h`. MSYS2 Python does not satisfy this stable-ABI integration. [python/CMakeLists.txt](../python/CMakeLists.txt) probes the interpreter named by `FSIM_PYTHON_EXECUTABLE`, or the Windows `py -3` launcher if none is supplied. A failed probe skips the Python SDK even when `FSIM_BUILD_PYTHON=ON`; read the configure output. Its native modules use the CPython 3.11 stable ABI. Hangar and its tests use numpy, scipy, matplotlib and Pillow; provision them in the selected environment under `D:\ENV`, with caches there too.

JSBSim and the VSG stack are tracked vendored sources in `third_party/`, as recorded in `.gitmodules`; there are no Git submodules to initialize. Configure a fresh checkout with an explicit Python interpreter:

```powershell
.\fsim.cmd --preset ucrt64-headless configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe
.\fsim.cmd --preset ucrt64-headless build
.\fsim.cmd --preset ucrt64-headless test
```

The interpreter path is an example matching the local installation; select the actual supported interpreter. The three main presets are:

| Preset | Configuration | Renderer |
| --- | --- | --- |
| `ucrt64-headless` | Release | Off |
| `ucrt64-release` | Release | On |
| `ucrt64-debug` | Debug | On |

Tests, examples, command-line tools and the Python SDK are enabled by default. `FSIM_WITH_RENDER` defaults off outside presets. Use the build's own preset throughout; do not mix binaries from different build directories. The equivalent direct commands are `cmake --preset <preset>`, `cmake --build --preset <preset>` and `ctest --preset <preset>`.

For rendering, the actual dependency entry point is the [deps superbuild](../deps/CMakeLists.txt), not a vcpkg recipe:

```powershell
cmake -S deps --preset deps-ucrt64
cmake --build build/deps-ucrt64
.\fsim.cmd --preset ucrt64-release configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe
.\fsim.cmd --preset ucrt64-release build
.\fsim.cmd --preset ucrt64-release test
```

It builds the checked-out vsg, vsgXchange and vsgImGui sequentially into `build/deps-ucrt64/install`, which the main presets put on `CMAKE_PREFIX_PATH`. The UCRT64 Vulkan headers/loader, glslang and the dependencies required by those upstream projects must be available. Consult each checked-out upstream README and its version support matrix before changing compiler, library or language-standard versions. Rendering also requires the bundled NVIDIA Flow headers and DLLs; [cmake/Flow.cmake](../cmake/Flow.cmake) rejects a renderer configure without them. An alternate built checkout can be selected with `FSIM_FLOW_DIR`; rebuilding Flow uses `tools/data/flow/build-flow.sh` and its pinned upstream version. Do not treat a render configure or a headless test as GPU validation.

## Preset selection and launch

The Windows entry point is `fsim.cmd [--preset NAME] COMMAND [ARGS]`, delegated to [tools/launch/fsim.ps1](../tools/launch/fsim.ps1). The global `--preset` option belongs before the command; options after the command are forwarded to that command. Selection is explicit `--preset`, then `FSIM_PRESET`, then `ucrt64-headless`. It selects only `build/<preset>` and never scans or falls back to another build. `help` and `where` work with no cache or executable.

`configure` forwards CMake configure flags, such as `-DFSIM_PYTHON_EXECUTABLE=...`. `build` and `dist` configure the selected preset when `CMakeCache.txt` is absent; configure first when you need custom options. Their remaining arguments go to `cmake --build`. `test` forwards arguments to CTest and requires an existing configuration; an unconfigured tree produces the matching build hint. Application commands use that preset's executable or configured Python interpreter and return its exit code. Python configuration writes `build/<preset>/python/fsim-python.json`; the shared launcher reads this metadata and invokes the interpreter directly with the selected package on PYTHONPATH.

For CTest expressions containing shell operators such as `|`, invoke `ctest --preset <preset>` directly from PowerShell. The batch entry passes through CMD, which can reinterpret such expressions even when the PowerShell argument is quoted. For example, `ctest --preset ucrt64-headless -LE 'hangar|architecture' -j 10` selects the functional suite without either label.

```powershell
.\fsim.cmd --preset ucrt64-headless help
.\fsim.cmd --preset ucrt64-headless where
.\fsim.cmd --preset ucrt64-headless build --parallel 8
.\fsim.cmd --preset ucrt64-headless test -L launcher
.\fsim.cmd --preset ucrt64-headless hangar list
.\fsim.cmd --preset ucrt64-release demo
.\fsim.cmd --preset ucrt64-release dist
cmake --install build/ucrt64-release --component viewer --prefix D:/ENV/fsim-viewer
```

For direct staged Python use, `build/<preset>/python/fsim-python.cmd` selects that package; Hangar also needs `tools/models/hangar` on PYTHONPATH. See [hangar.md](hangar.md) for stages; resource regeneration is a separate task from writing documentation. `Testbench.cmd` uses the same preset selection: explicit arguments run synchronously and return the testbench exit code; a no-argument launch opens a minimized console. The headless preset opens the testbench window alone; a viewer requires a render preset. See [testbench setup](../src/applications/testbench/README.md#how-to-open-it).

The `dist` target builds enabled install targets and produces separate SDK, tools, examples, viewer (when enabled) and Python (when available) trees. The standalone SDK component includes DLLs, headers, CMake exports, runtime JSBSim data, designed aircraft/store models, payload/store cards and notices. Render SDKs also include optional vision model/cloud resources. Installation copies generated products rather than running Hangar. See [resources.md](resources.md#staging-and-runtime-search) and the [installed-consumer guidance](sdk/README.md#building-against-the-sdk).

## Change invariants and verification

Keep the existing public C++ API, C ABI and Python ownership/lifetime contracts aligned. A public surface change must update its principal classification in `docs/sdk/boundary.toml`, regenerate the Markdown inventory, and exercise the corresponding SDK and ABI tests. Agent permissions are part of behavior: do not broaden access to range operations or referee truth while adding convenience methods. [agent-boundary.md](architecture/design/agent-boundary.md) records the boundary design.

Preserve deterministic simulation streams, control trajectories and generated bytes outside the intended change. For flight/control/model work, compare the affected aircraft and the rest of the fleet, identifying named failures and digest differences. A clean process exit does not prove stability or conformance. For native mesher changes, compare generated GLB bytes across repeated runs and thread counts, and check the relevant corpus; do not reset unexpected generated differences to conceal them.

Before extending simulation state, construction inputs or Viewer telemetry, follow the
[simulation/interface evolution boundary](architecture/design/simulation-interface-boundaries.md).
Run `simulation_interface_boundary` alongside the architecture gates. Existing unsized state/control layouts are
fixed; use versioned extensions for additional data. New World options need explicit native mapping and episode
admission decisions. Viewer publication, reading and recording must remain output-only. New mutable subsystems must
participate in complete World ownership/reset; a rendering or gateway convenience must not introduce shared mutable
simulation state. Keep actor reports distinct from diagnostic IPC/referee truth.

Tests, examples and tools are kept in categories, which the architecture gate's layout rule holds (ARCH-08):
- **Tests:** `tests/contracts`, `simulation`, `training`, `visualization`, `compatibility` and `integration`.
  `compatibility` holds the installed-package consumers, the C ABI, adapter-compatibility and launcher tests, and the
  Python SDK's tests (`tests/compatibility/python`).
- **Examples:** `examples/simulation`, `training` and `visualization`.
- **Tools:** `tools/architecture`, `models` (Hangar, RCS, propellants), `data` (map, cloud and ocean generators, tile
  prefetch, Flow assets) and `validation` (checks, cross-checks and benchmarks). `tools/launch` keeps the launchers'
  implementation (a declared layout exception, HC-25).

CTest names and labels, executable names and the flat `build/<preset>/examples` output are unchanged. Add a file to
its category: a file beside them fails `architecture_boundary`.

Use the existing layers in [tests/CMakeLists.txt](../tests/CMakeLists.txt) and [python/CMakeLists.txt](../python/CMakeLists.txt):

| Layer | Existing entry point | What it establishes |
| --- | --- | --- |
| Core, mission systems, integration | CTest cases, focused `-R` selection | Native behavior and scenario regressions |
| Public C++ and C ABI | SDK cases and `c_abi` | Consumer contracts against the shared library |
| Python and principals | `-L python_sdk` (a CTest test per module: `python_sdk test_world`, ...), `sdk_boundary` | Python behavior and classified exports |
| Native architecture and scanner regressions | `.\fsim.cmd test -L architecture` | Module ownership/dependencies, public/private and GPU boundaries, principal scanner coverage and negative fixtures |
| Mission-training contract | `ctest --preset ucrt64-headless -L ma-training --no-tests=error` | The [contract](sdk/mission-training.md)'s schema dialect, fixtures, page tables and local links ([check.py](../tools/validation/ma_contract/check.py)); not evidence of any implemented mission interface |
| Control conformance | `ctest --preset ucrt64-headless -L conformance` | Lifecycle, allocation gate and adapter checks |
| Advertised fleet capabilities | `ctest --preset ucrt64-headless -L fleet` | Aircraft-specific acceptance cases; includes shards |
| Hangar methods and pipeline | `ctest --preset ucrt64-headless -L hangar` | Theory/data comparisons and stage checks |
| Windows launchers | `.\fsim.cmd --preset ucrt64-headless test -L launcher` | Preset precedence, first-build configuration, argument forwarding and exit codes |
| Installed SDK package | `.\fsim.cmd --preset ucrt64-headless test -L package` | SDK-only install, installed DLL/data provenance and API load-rejection probes |
| Renderer, RCS and cameras | Render preset cases; viewer screenshots/performance tools | Visual and GPU behavior on capable hardware |
| Contention (stress) cases | `-L stress` | Calls from other threads meeting mission-batch transitions, hundreds of steps each; selected when concurrency code changes |
| Change-scoped selection | `testing_scope_checker` (label `testing`) | The selector's rules, and `tools/validation/testing/scopes.toml` against the repository and the build's inventory |

Choose the tier by the change, as the next section says. Record skips: camera tests may skip without a Vulkan device, and RCS tests may skip without ray queries. All native viewer acceptance runs must use `--fullscreen`, including screenshots, live/replay lifecycle checks and performance scenes. Confirm the captured dimensions against the display's full resolution, read the image and confirm the model used in logs. Vision/RCS render into offscreen GPU targets, and scene-graph tests have no native viewer window. The [viewer validation record](sdk/viewer-validation.md) identifies actual runtime evidence and exclusions. A historical green suite, a workflow file, or a current test listing is not a fresh pass. Check actual workflow-run evidence before claiming CI success.

The `package_consumer` CTest gate installs only `sdk` into `build/<preset>/package-consumer/installed SDK`, builds the [external consumer](../tests/compatibility/package_consumer/CMakeLists.txt) using only installed `fsim::sdk`, checks DLL locations and a SHA256 runtime-data manifest, then exercises stock `c172x`, custom `f16c`, its Mk 82 loadout and a released store. It poisons the installed stock, custom and released-store XML in turn; each probe must produce API rejection exit 42. The gate restores exact bytes before its final positive run and keeps its logs/manifest in the private package-consumer directory. Optional vision export checks do not open a GPU device.

The `mission_package_consumer` gate (label `ma-training`) installs `sdk` and `python` into prefixes of its own under `build/<preset>/mission-package-consumer`, checks the installed public headers, library and Python package against this tree and build byte for byte (`provenance.sha256`), builds the [C++ and C99 consumers](../tests/compatibility/mission_package_consumer/CMakeLists.txt) against installed `fsim::sdk` alone, and runs them and the Python consumer with only the installed files on any path. Each checks where its `libfsim.dll` was loaded from, then exercises MA-service binding in both Agent creation orders and the legal reports.

The `dist_components` gate builds tiny independent targets directly through `dist`, checks that an enabled example is compiled before installation, and removes obsolete executables/wheels when components are disabled. It checks the fixed `bin`, `lib` and `share/flightsim` package layout and protects cleanup outside the owned build's distribution tree. SDK CMake exports describe that actual layout and remain relocatable with the installation prefix.

Architecture acceptance is registered by [cmake/ArchitectureChecks.cmake](../cmake/ArchitectureChecks.cmake), independently of `FSIM_BUILD_PYTHON` or native Python module discovery. It needs only Python 3.11+ standard-library support; `FSIM_CHECK_PYTHON` can select its interpreter. When none is available, configure explicitly warns that these checks are unavailable; do not count that configuration as architecture acceptance. The [architecture checker guide](../tools/architecture/README.md) documents the permitted CMake subset and module/source policy. Extend its negative fixtures alongside any new native dependency or declaration form.

When splitting C adapters, preserve the first principal guard, shared last-error state, borrowed output lifetime and `struct_size` behavior. `sdk_guard_checker` exercises source-discovery/coverage failures without loading the SDK; `sdk_boundary` still checks the actual DLL/Python surface and generated inventory. `launch_eligibility` and the stores' `[launch-policy]` cases characterize the private weapon admission seam; native engagement acceptance retains worker/event determinism. Avoid changing defaults or refusal order while moving ownership.

DMS/C2 adapter owners are `src/adapters/sdk/c/fsim_links_c.cpp` and `src/adapters/sdk/python/_native_dms.c`. C control/VI belongs to `src/adapters/sdk/c/fsim_control_c.cpp`, and Mission Systems belongs to `src/adapters/sdk/c/fsim_mission_c.cpp`; only the unchanged parameter conversion is shared by world/control through private `CParameterSupport.h`. C Scenario belongs to `fsim_scenario_c.cpp`, and Environment/terrain/magnetic calls belong to `fsim_environment_c.cpp`, including five definitions moved from control. World lifecycle and remaining operations stay in `fsim_world_c.cpp`.

C VecEnv belongs to `fsim_vecenv_c.cpp`, sharing the original opaque layout through private `VecEnvHandles.h` with the C++ wrappers in `fsim.cpp`. Keep VecEnv's original void-return guard; CAbiSupport's int-return guard has different semantics. C Recording belongs to `fsim_recording_c.cpp`, with state/control layout assertions and owned views into recording data. World recording setup/query and their existing principal guards stay in World core.

Python control and Mission handlers belong to `_native_control.c` and `_native_mission.c`; World, Scenario and Environment handlers belong to `_native_world.c`, `_native_scenario.c` and `_native_environment.c`. VecEnv and Recording handlers belong to `_native_vecenv.c` and `_native_recording.c`, using the exact layouts/prototypes in focused private headers. Keep all method/type/module registration, Error, BufferType and runtime type ownership in `_native.c`, alongside World lifecycle, agent creation/sealing, state buffers, scenario application and the coupled VecEnv buffers/borrowed World and Recording samples callbacks. Public identities remain in their existing Python modules. `_native_world.h` shares existing private layouts/call support; `_native_world_options.h` shares immutable option metadata only between core and Scenario. Compare moved/shared bodies, field metadata, original remainders, registrations, native exports and World/Scenario/VecEnv/Recording descriptors alongside all four binding compatibility suites and agent/isolation/buffer acceptance when splitting another domain.

`adapter_compatibility` pins C control/Mission error and result-ordering differences against the shared SDK. Direct control success can retain LastError while guarded success clears it. A null Mission result permits submission, and a short result can reject after submission; Mission does not overwrite control's last detail. Preserve these existing distinctions during source extraction rather than normalizing them.

`world_adapter_compatibility` pins C Scenario borrowing/versioned writes/truncated ID counts, Environment error retention and validation, terrain NaN gaps, magnetic short outputs and principal refusal order. `test_world_binding_compatibility` pins Python defaults, conversion errors, native identities/descriptor flags, Scenario application and World buffer/borrowed-owner lifetime. Keep the existing Scenario partial-application tests: a later command failure retains earlier creation and environment side effects.

`batch_recording_adapter_compatibility` pins C borrowed World caching/no-op destruction, stable VecEnv result pointers, short-struct/out-pointer validation and direct versus guarded LastError behavior. Recording cases pin output clearing, null/out-of-range defaults and borrowed sample/event/string stability. `test_batch_recording_binding_compatibility` pins native type/descriptor identity, float32/float64 narrowing and failed action buffer release, read-only result/sample buffers, owner retention and frame conversion errors. Keep existing VecEnv busy/GIL/autoreset and Recording round-trip tests; source extraction must preserve their current policies.

MA removal has a narrower contract than deleting its State: publicly queryable tasks, authority, overrides and borrowed RBAC names remain in place, as do controller mappings. `MaConsentLedger` owns private pending requests; answered/automatic records are reclaimed while unanswered ended strikes retain late-reply semantics. Retirement releases private consent storage and stops maintenance after the existing observable transitions settle. Reset retains MA state. `[ma][lifecycle]` and `[agents][c2][lifecycle]` cases exercise these boundaries; see the [A2 design](architecture/migration/history/2026-10-12-architecture-lifecycle-design.md) before changing history retention.

World's private `src/simulation/runtime/stores/StoreFlightBinding.h/.cpp` owns carried-store property bindings and their mass/drag,
weight-room and reported-load caches. Keep `loadTick_`, aircraft pointmass-name caching, entity/model lifecycle,
permission checks, traversal, host publication and mass-change transactions in World. The binding borrows property
nodes and takes explicit Host/model inputs; it must not retain model pointers or add reset invalidation. Preserve
fuel transfer before mass application before pool stepping, the single-dt transfer cap, separate store/fuel revisions,
lazy qbar binding and force summation after damage/before guidance. `store binding:` cases pin loaded reset and slot
reuse, and their optional `FSIM_STORE_BINDING_TRACE` replay emits explicit field bits without structure padding.
Compare replay bytes against the pre-extraction build alongside existing mass/drag/load, determinism and principal
gates. See the [A6 design](architecture/migration/history/2026-10-12-architecture-store-binding-design.md) for the preservation contract.

### Change-scoped testing

Measured on `ucrt64-headless` (2026-10-16, 16 jobs): the suite is 690 CTest tests with 10,044 s of test time,
and a full run takes about 19 minutes of wall time. Its floor is its longest test,
`hangar test_vulnerability.Dispensers` (1,037 s under the suite's load); the 54 tests of 30 s or more hold about 87%
of the test time, the 556 under 5 s about 5%. `fleet` and `hangar` alone hold nearly two thirds. Tests marked `RUN_SERIAL`
(`mission_episode`, `native_state_reset_fixture`, `equipment_package_consumer`, `simulation_interface_boundary`) run
alone, so any selection that includes them adds their time to its wall time. A routine change needs a small part of
this; take the tier its change needs:

| Tier | When | Command | Wall time |
| --- | --- | --- | --- |
| Targeted | Every routine change, before each commit | `python tools/validation/testing/scope.py` (the plan), then `--run` once the presets it names are built (`--build` builds them) | Measured: this correction's own selection (the selector, the comparator and the installed-consumer gate, after a build) 68 s. Estimated (lower bounds): documentation 5 s; `tests/training/test_ma_joint.cpp` 30 s; an MA gateway source 62 s; MA concurrency code, a shared Python test fixture or any `python/fsim` module (stress cases included) 303 s |
| Integration | Before handing off a feature package; public-surface, packaging or installed-consumer changes | `python tools/validation/testing/scope.py --tier integration --run` | Estimated: an MA gateway source, at least 383 s |
| Full | At a milestone handoff or acceptance; before a push; when `scope.py` escalates (shared simulation core, build system or test registration, native code no scope claims); when a targeted or integration failure is unexplained | `ctest --preset ucrt64-headless --no-tests=error` | 19 minutes |
| Release and visual | Render, vision, viewer or public-surface changes; anything only the renderer builds plans `ucrt64-release` alone; release preparation; Viewer acceptance | `scope.py` runs Release where its plan says so; full: `ctest --preset ucrt64-release --no-tests=error` | |

[tools/validation/testing/scope.py](../tools/validation/testing/scope.py) reads the working tree's changes (or `--paths`), the reviewed
rules in [tools/validation/testing/scopes.toml](../tools/validation/testing/scopes.toml) and each preset's own CTest inventory. It selects the
tests the matching scopes name, and beyond them the tests that depend on the changed file:

- every CTest test built from a changed C/C++ test or example source, or from one including a changed local header
  (each preset's `compile_commands.json` names the target a source is compiled into);
- every CTest test whose command names the changed file or a fixture directory holding it, and the tests of a test
  source or script that names a changed data file;
- every `python_sdk` test whose module imports the changed Python file, directly or through other modules, shared
  test fixtures (`test_mission_concurrency` imports `test_mission_training`'s) and the scripts it runs in another
  process: the import closure of `tests/compatibility/python` and `python/fsim`. `fsim/__init__.py` imports the package's modules,
  so a change to any of them selects every module's test - the contention cases among them.

It fails closed. A changed path nothing claims (no scope, no test source, command, reader or import), a selection that
comes out empty and no changes at all are errors: it exits 3 and names the reason and the paths (`--json` gives them
as data), and runs nothing. So is a Python test module the changes need - the changed module itself, or one importing
the change - with no CTest test registered: a module added since the last configure is registered only by a build, and
the rest of the selection (`sdk_boundary`, say) passing would not have run it. `--build` registers it first; `--run`
or a plan without it stops, naming the module. A changed native source or build input nothing claims escalates to the full suite instead.
Legacy environment code (`src/training/native/vecenv`) is linked into `libfsim` and run by native, C,
Python and plugin tests alike, so it escalates too. Repository metadata and tools without tests stay unclaimed: their
change needs the full tier or a reviewed scope.

An escalation applies to the presets its own path plans (HC-19). A shared-core change plans `ucrt64-headless` and
escalates it; a renderer change beside it adds `ucrt64-release` with that change's own selection - every
`render`-labelled test among it - not the Release full suite. Shared build inputs (the root and `src` CMake files,
`CMakePresets.json`, `cmake/**` other than the test and package scripts that their own scopes claim, `deps/**`,
`third_party/**`, `tests/CMakeLists.txt` and `python/CMakeLists.txt`) configure both presets, the renderer's branches
included, so they plan and escalate both (HC-20). A plan that escalates is a statement of what the change needs; it never
authorizes running a full suite before the required milestone (HC-11).

What only the renderer builds (`FSIM_WITH_RENDER`) is planned on `ucrt64-release`, which compiles it; `ucrt64-headless`
does not. This covers visualization with its nested CMake files, the Vision adapter and its Python extension, the
viewer, the renderer's tools and example, and the render tests. The `render-only roots` scope claims these and selects
the `render`-labelled tests, which are every test only the renderer builds. For these paths alone, `ucrt64-headless` is
not planned. Independently of the rules, a changed path that the presets' compile databases show is compiled only in
`ucrt64-release` also plans that preset: a source, or a file in a directory holding only such sources. A missing
`ucrt64-release` inventory or a failed build of it is an error, not a headless pass. Such a plan then requires and
selects every `render`-labelled test of the `ucrt64-release` inventory, as listed after any build. So does a path that
the shared `render, vision and viewer` scope claims (the public Vision header, `python/fsim/vision.py`, `assets/**`),
whose headless selection is kept. An inventory without one (a stale or renderer-disabled configure) is a
`no-renderer-coverage` error before any test starts; `sdk_boundary` and the Python Vision and camera tests that these
scopes also name cannot stand in for it. Release scopes without the `render` label need no renderer tests. `testing_scope_checker` holds the
rules to the real builds: every source compiled only with the renderer is claimed by the render-only scope, and every
test only the renderer builds carries the label.

A scope can also name the tests a preset must have (`[scope.requires]`, by name). Each preset the scope plans selects
them, whatever the tier, and an inventory missing any one is a `missing-required-tests` error before any test runs,
with or without `--build`; the scope's other tests cannot stand in for them. The `demo publisher` scope requires the
publisher's process tests (`demo_publisher`, `demo_publisher_steps`, `demo_publisher_lifecycle`) on `ucrt64-release`,
and its single-step schedules (`demo_publisher_schedules`), include-view probe and control-block contract test on both
presets (HC-31). `testing_scope_checker` holds every requirement to the configured inventories.

With `--build` it builds the presets first, then lists each inventory afresh - a reconfigure may add, remove or
reorder tests (the Python test modules are a `CONFIGURE_DEPENDS` glob) - and resolves the selected names to that
inventory's `-I` numbers immediately before running. After the run it requires that exactly the selected tests ran
(exit 4 otherwise). It prints the plan, an estimate and the exact command, and with `--run` runs at the preset's jobs
with `--no-tests=error`, keeping the raw output in `build/<preset>/scope-logs` and recording each test's measured
time ([tools/validation/testing/timing.py](../tools/validation/testing/timing.py); `timing.py report` shows where the time goes). The
estimate is a lower bound from measured passing times: `RUN_SERIAL` tests are added one after another, the rest
packed over the jobs by their `PROCESSORS`; load and CTest's ordering are not modelled. A failed or timed-out run
never replaces a test's passing time. Tests labelled `stress` run in the targeted tier only when concurrency code
changed (a scope naming that label) or what they are built from or import did; the integration and full tiers always
include them.

Acceptance commands select by label, not by an old exact name. `python_sdk` is a label over one CTest test per
module (`python_sdk test_world`, ...); there is no single test of that name. `mission_training` and `mission_c_abi`
leave their contention cases to `mission_training_stress batch|SameStep|NextStep` and `mission_c_abi_stress same|next`
(label `stress`). `-L ma-training` still selects all of them, the Python contention module included.

Rules for every tier:

- Run CTest in parallel at the preset's jobs. `-j 1` is for diagnosing an order-dependent failure, not for gates:
  the exclusive build/test slot is one agent at a time, not one test at a time.
- One run per tier and preset. Do not repeat the same selection gate by gate or preset by preset; run
  `ucrt64-release` only where the plan or the change requires it.
- A passing run stands for the build it ran on. A documentation edit afterwards needs only its own checks
  (`scope.py` selects them; no build), not the suites again.
- Never narrow coverage to save time: the full and integration tiers keep every test, the stress cases among them.
  Report the plan, command, log and wall time with the results.
- Keep `scopes.toml` current. Add a scope when a domain gains targeted coverage; `testing_scope_checker` fails on a
  path pattern no tracked file matches, or on a label or test pattern that selects nothing.

## Long-running work and evidence

A background command must write raw output directly to its log. Never end it with `| grep`, `| sed`, `| awk` or `| head`: block buffering can hide completed work until the process exits. Read and filter the saved log separately. If an unavoidable live pipeline needs filtering, use explicit line buffering (`grep --line-buffered`, `awk '{print; fflush()}'`, or `python -u`); `head` does not fix buffering.

Keep one background task entry per operation. Do not layer another background polling/wait task on a task that already reports completion. Use its existing completion notification and retain the original raw log.

Report the checkout/commit, preset, command, environment, elapsed time, tested scope, exit code, failures and skips. Keep before/after measurements under comparable conditions; separate implemented, measured, visually checked, and still unverified results. Documentation-only edits should be checked for links, command correctness, generated-file ownership and diff scope; they do not justify builds, asset generation or heavy flight campaigns.

## Documentation and skill maintenance

[status.md](status.md) owns current capability and milestone status. Keep architecture documents as design/rationale, roadmap documents as sequencing/history, [backlog.md](backlog.md) as unresolved work, and generated reports as dated evidence. Update cross-links instead of copying another live status table. A documented limitation stays unresolved until implementation and acceptance evidence support changing it.

`docs/sdk/boundary.toml` is the editable principal inventory; `docs/sdk/boundary.md` is rendered output. `docs/hangar-stores-build.md` and its JSON companion are generated build evidence; regenerate them with their owning tool rather than hand-editing rows. [resources.md](resources.md) lists the other owners.

Regenerate the canonical boundary page from the render build with both native DLLs and its Python package:

```powershell
python tools/validation/boundary/boundary.py render --fsim-dll build/ucrt64-release/bin/libfsim.dll --vision-dll build/ucrt64-release/bin/libfsim_vision.dll --python-path build/ucrt64-release/python
ctest --preset ucrt64-release -R '^sdk_boundary$'
ctest --preset ucrt64-headless -R '^sdk_boundary$'
```

Headless checking verifies its available API subset, but headless rendering would remove optional Vision
groups from the canonical page. Retain the complete renderer inventory even when the implementation change
was validated primarily with a headless build.

The aircraft design skill is maintained at `.agents/skills/aircraft-design/SKILL.md`. `.claude/skills/aircraft-design/SKILL.md` is a byte-identical synchronized copy for Claude discovery. Change the maintenance source, sync the copy in the same change, and verify:

```powershell
Get-FileHash .agents/skills/aircraft-design/SKILL.md, .claude/skills/aircraft-design/SKILL.md -Algorithm SHA256
git diff --no-index .agents/skills/aircraft-design/SKILL.md .claude/skills/aircraft-design/SKILL.md
```

The hashes must match and the no-index diff must be empty (exit 0). Retain both frontmatter fields and the shared provenance note. This synchronization preserves the existing workflow; it does not authorize generating aircraft or change project acceptance criteria.
