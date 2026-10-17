# ARCH-08 applications, tests, examples and packaging

Status: implemented on main (2026-10-17); awaiting independent acceptance. ARCH-09 has not started.
Starting main HEAD: `a59b46f` (ARCH-07 accepted).

This package moves the remaining applications into `src/applications` and classifies every test, example and tool by
category. It separates the Viewer's demo into a publisher process of its own (HC-16) and takes the Viewer's
control-level names from a neutral stream contract (HC-17.2). It removes the last shared `src` include roots (HC-14)
and adds a layout rule to the architecture gate that holds the new categories.

Nothing a user or consumer sees changed:
- **Names:** every CTest name and label, executable, DLL and module name, launcher command and installed path is
  unchanged. The one new executable is `flightsim-demo`, installed beside the Viewer.
- **Native objects:** every first-party library and application object is unchanged, apart from the Viewer's entry
  point (section 2). A moved test's object differs from ARCH-07's only by the source path it embeds, except where this
  package edited the test (section 6).
- **Behavior:** the demo flies exactly as it did inside the Viewer. `demo_publisher` compares the in-process and the
  published demo over 240 steps, to the bit.

## 1. What moved

| Destination | From | Path-map rows |
| --- | --- | --- |
| `src/applications/headless` | `src/app` (`flightsim`) | P043 |
| `src/applications/testbench` | `testbench`: the Python testbench, its suites, UI and recorded results | P065 |
| `tests/contracts` | `test_contracts.cpp`, `test_state_layout.cpp` | P061 |
| `tests/simulation` | the other native tests and benchmarks, `control_flights.h`, `mode_flights.h`, `crash_check.cmake` and `tests/data` | P061 |
| `tests/training` | `test_vecenv`, `test_mission_*`, `test_ma_*`, `test_native_state_reset_fixture`, `training_link.cpp`, `MissionTestSupport.h` and `tests/fixtures` | P057, P058, P060 |
| `tests/visualization` | the terrain, vehicle-visual, contrail, Viewer-screenshot, Vision and strike-capture tests | P061 |
| `tests/compatibility` | the C ABI, SDK, agent and adapter-compatibility tests, `test_launchers.ps1`, the three installed-package consumers, and `python/tests` as `tests/compatibility/python` | P059, P062 |
| `tests/integration` | `test_showcase_native.cpp`, and the new `test_demo_publisher.cpp` | P061 |
| `examples/training` | `minimal_trainer`, `ppo_trainer`, `rust_trainer`, `examples/python/ma_composition`; the plugin and the training scripts as `examples/training/python` | P063, P064 |
| `examples/simulation` | `multi_level_control`, `scenario_runner`, `scenarios`, `udp_peer`; eleven Python examples as `examples/simulation/python` | P064 |
| `examples/visualization` | `vision_capture`; `cameras.py` as `examples/visualization/python` | P064 |
| `tools/models` | `hangar`, `rcs`, `propellants` | P067, P068 |
| `tools/data` | `cloud_map`, `ocean_map`, `tile_prefetch`, `flow` | |
| `tools/validation` | `ma_contract`, `ma_engagement`, `boundary`, `testing`, `contrails`, `fers`, `track`, `plume_check`, `flow_calibrate`, `viewer_perf`, `showcase` | P069 |

Further details:
- **Renames:** 609 files, all with `git mv`. `tools/architecture` stays (P066). `tools/launch` stays too, as the
  layout's one declared exception (section 4, HC-25).
- **Inventories:** P061 and P064 are classified. Nothing remains at the top of `tests` or `examples` but their
  `CMakeLists.txt`.
- **Build outputs:** unchanged. Example executables still build into the flat `build/<preset>/examples`, test
  executables into `build/<preset>/tests`, and the tools into `bin`.

## 2. Applications (task 7)

- **Headless (P043):** `flightsim`'s sources are in `src/applications/headless`. The target sees its own directory
  and its links' include views, not the shared `src` root. Its command line is unchanged.
- **Demo publisher (HC-16):** `flightsim-demo` (`src/applications/demo/main.cpp`) flies the unchanged
  `DemoSimulation` in a process of its own and publishes it on the existing read-only world stream (`WorldPublisher`).
  Its vehicles are named `demo N`, of type `jsbsim:<aircraft>`, each with its CG offset in its flight status.
  - **Launch:** `flightsim-viewer --demo` starts it from the Viewer's own directory, passing the Viewer's demo and
    terrain options (`DemoLink`, `src/applications/viewer/DemoLink.{h,cpp}`).
  - **Control:** the Viewer attaches to its world as to any other, and drives it through the demo control block
    (`src/contracts/world_stream/DemoControl.h`, shared memory `<world>.control`): pause, single step, time factor
    and quit. The block also carries the throughput the Viewer shows. The keys are unchanged.
  - **Start:** the publisher starts held at its first state. The Viewer releases it after compiling its scene, where it
    used to start the in-process simulation, so the demo's timing is as before.
  - **End:** the publisher stops when the Viewer asks, or when the Viewer's process has gone (`--parent-pid`). The
    Viewer waits for it to exit and ends it by force after a few seconds.
  - **Dependencies:** the Viewer links no simulation, control or demo code. The `viewer -> demo` exception is gone and
    the policy lists no exception. `flightsim-demo` installs beside the Viewer (component `viewer`) and is staged with
    its DLLs in `bin`.
- **Control-level names (HC-17.2):** the stream contract `contracts/world_stream/WorldLayout.h` names the levels
  (`kControlLevelNames`, `controlLevelName`). The Viewer shows them from there, not from `fsim/Control.h`, and its
  labels are unchanged:
  - `World.cpp` asserts that the stream and the control module count the same levels;
  - `test_control.cpp` checks every name against `control::levelName`;
  - the `include_view_viewer` probe holds the Viewer's view. Its own headers, the demo control block, the world mirror,
    scene frames and UI colours are reachable. `DemoSimulation.h`, the control stack, `SimRunner.h` and `World.h` are
    not.
- **Testbench (P065):** a Python application without a native target, in `src/applications/testbench`.
  `Testbench.cmd`, `fsim.cmd testbench`, its module name, UI and results directory work as before. The planned
  `testbench` reservation is removed as fulfilled.

## 3. Tests, examples and tools (tasks 1 and 2)

- **Names kept:** CTest names and labels, executables and Python test names are unchanged. One Catch2 case names a
  path in its title (`the propellant classes are NASA CEA's (tools/propellants/classes.json)`). The title is kept so
  the CTest name stays; the file the case reads is the moved one (HC-26).
- **Public examples:** each example links only the installed package's targets: `fsim::sdk`, and `fsim::vision` for
  the Vision example. None sees a private header. The composition example still runs as the installed mission
  consumer's copy.
- **Layout rule:** section 5.

## 4. References, launchers and packaging (tasks 3 to 6)

- **References:** the moves' references were rewritten in three passes:
  - the first pass rewrote old paths at the start of a word in 179 files: 516 references and 39 recomputed Markdown
    links;
  - a second pass fixed 25 references after `/` or `\` (`${CMAKE_SOURCE_DIR}/tools/...`) in 11 files;
  - path segments joined in code (`os.path.join(ROOT, "tests", "fixtures")`, `REPO / "tools" / ...`) were fixed by
    hand.

  The CMake subdirectories and the Python tests' glob and working directory name the categories. Dated records keep
  their paths.
- **Depth:** every moved script that finds the repository from its own location now walks one level further. These
  are the Python SDK tests, the testbench, `boundary.py`, the MA contract and engagement checkers, the selector, the
  FERS and Stone Soup cross-checks, the cloud-map tools, and Hangar's repository search, tests and `rebuild_all.py`.
  So do the Viewer performance tools, `test_launchers.ps1` and the Rust example's build script.
- **Launchers:** `fsim.ps1` puts `tools/models/hangar` and `src/applications/testbench` on `PYTHONPATH`. Example
  routing is unchanged (the flat `build/<preset>/examples`), and `test_launchers.ps1` finds the repository two levels
  up. `tools/launch` keeps its path: `fsim.cmd`, `Testbench.cmd` and the launcher contract name it, and the layout has
  no launch category. It is the policy's one layout exception, until ARCH-10 (HC-25).
- **Packaging:** the install rules and layouts are unchanged, except that `flightsim-demo` joins the Viewer component.
  The scenarios install from `examples/simulation/scenarios` to the same `share/flightsim/scenarios`.
- **Copied consumer scripts and fixtures:**
  - the installed C++, C, Python and mission consumers configure from their new directories and copy their scripts,
    the joint fixtures and the composition example from there;
  - the MA compatibility comparison copies its frozen consumers from `examples/simulation/python`;
  - the fixture readers (`FSIM_TEST_DATA_DIR`, the mission fixtures, and the FERS and Stone Soup cross-checks) read
    the moved data.
- **No hidden fallback:**
  - no old path is left on disk or named as a runtime path, and the empty `src/app` is removed;
  - Hangar's repository search tries its new depths only;
  - `ma_engagement` imports the generic checker as `tools.validation.ma_contract`;
  - runtime code in `src`, `include` and `python/fsim` names no source-tree path at all;
  - the installed components name neither the checkout nor a build tree (section 6).
- **Provenance (P067, P068):** generated data keeps the provenance text it was written with. The generated
  `aircraft/**`, the classified `assets/**`, Hangar's output templates and `flightsim-rcs`'s table method string are not
  edited, so a regeneration writes what is committed (HC-26). Where a tool's output names the tool and both changed
  together, the output is checked:
  - `docs/hangar-effects.md` regenerates byte for byte;
  - `sdk_boundary` re-renders `boundary.md`;
  - the recorded joint chains and FERS cross-check name their tools' new paths and replay unchanged (`ma_joint`,
    `test_signal`).

## 5. Policy, gate and tests

- **Policy:**
  - `app` holds no legacy directory;
  - `demo` is `flightsim-demo`, an application linking contracts, ipc, sim, io, core and platform;
  - `viewer` no longer depends on `control` or `demo`;
  - the exceptions are empty, and the planned `testbench` reservation is removed;
  - the new `layout` names the categories of `tests`, `examples` and `tools`, with the one exception `tools/launch`.
- **Gate:** the layout rule has three outcomes:
  - a file under `tests`, `examples` or `tools` outside its root's categories, other than the root's `CMakeLists.txt`,
    is `uncategorized`;
  - an exception with no file under it is stale;
  - an exception naming a category, or a file, is an input error.

  Ignored `__pycache__` directories are skipped. `LayoutFixtures` (6 cases) hold the rule, and the baseline records the
  layout.
- **Tests:**
  - `demo_publisher` (Release, label `render`): the in-process `DemoSimulation` and `flightsim-demo`, launched and
    stepped by `DemoLink`, compared state by state. A seed mutation was confirmed to fail it;
  - `test_contracts.cpp`: the control block's layout and the stream's level names;
  - `test_control.cpp`: the stream's names against the control module's;
  - probes: the new `include_view_viewer`; `include_view_demo` now probes `flightsim-demo`.
- **Selector:** `scopes.toml` names the moved paths, and the render-only scope claims `test_demo_publisher.cpp`.
  `scope.py` reads the Python tests from `tests/compatibility/python`.

## 6. Verification (2026-10-17)

| Check | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings |
| Object identity: per source, every section's contents and the symbol table, against ARCH-07's final builds | Headless: 154 unchanged in place, 26 moved unchanged, 111 test objects differ. Release: 187, 32 and 121. New: `flightsim-demo`'s `main.cpp`, and in Release `DemoLink.cpp` and the new test. Every library and application object is unchanged but the Viewer's `main.cpp`. `DemoSimulation.cpp`'s object is unchanged, now built into `flightsim-demo` and the test |
| The differing test objects, compiled again with their old path (`-fmacro-prefix-map`, and the moved data paths in `-D` definitions) | Headless: 96 identical to ARCH-07's, and 4 more whose edits are comments only. The other 11 carry reviewed edits: two new test cases (`test_contracts`, `test_control`), path strings (`bench_control`, `test_mission_episode`, `test_ma_joint`, `test_radar`, `test_signal`, `test_track`), and three tests including the edited `MissionTestSupport.h`. Release: 101, 6 and 12, adding `test_vehicle_visuals`' propellant path |
| `tools/validation/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures. Against HC-21's capture, byte-identical: exports, vtables, the Python call lists and inventories, the C and C++ layouts and the public class dump. The full class dump differs only in 14 standard-library construction-vtable addresses |
| Python identity: every public name of 12 (headless) and 13 (Release) modules: owner, qualified name, signature, fields, defaults, members and a pickle round trip | 494 and 501 names, 0 differ |
| Frozen baseline (`e1945b9`): `verify`; the frozen C++, C99 and three Python consumers on the installed Release SDK, then on the installed package | 66 artifacts unchanged; 5 and 5 exit 0. The second run's provenance names `install/python/fsim`, its `_native.pyd` and `libfsim.dll` |
| Installed provenance: the `sdk`, `python`, `viewer` and `examples` components, every file scanned | No file names the checkout. No CMake package file, Python module, JSON or TOML names a build tree; only installed Markdown guides show `build/<preset>` command examples. No file sits at a moved source path. The Viewer component holds `flightsim-demo.exe`, and the scenarios install to `share/flightsim/scenarios` |
| Related fast tests (section 7) | Headless 627 of 713: 624 passed, and 3 failed (`testing_scope_checker`, `ma_engagement_contract`, `ma_engagement_checker`); after their fixes 3/3. Release 688/688 of 774 (306 s). No skips |
| Launcher, distribution and package matrices (in the counts above) | `launcher_contract` on both presets; `dist_components`; `package_consumer`, `equipment_package_consumer` and `mission_package_consumer`; `rust_example`: all pass |
| Demo | `demo_publisher`: 240 of 240 steps identical, and a different seed fails it. GUI through `fsim.cmd viewer --demo` (fullscreen, 3840 x 2160), twice: the publisher starts, the Viewer mirrors it and exits, and no publisher is left. Against ARCH-06's capture: the same simulated time (4.9 s), pose and list values. The mean differences are 1.79 and 0.52, and the two runs differ from each other by 1.52 (ARCH-06's in-process runs: 0.185) |
| Include environments ([arch-08-include-views.json](arch-08-include-views.json)) | The demo and Viewer compile with exactly their recorded roots: their own views and their links'. None of the 506 headless and 559 Release compiled sources, tests, tools and examples included, has the shared `src` root or an ancestor of it |
| Architecture: gate; `test_check.py`; planning document checker | Accepted (28 modules, 400 files; layout 598 files in 13 categories, exception `tools/launch`); 141 cases; accepted. With this policy, HEAD's tree is rejected (478 issues): 448 uncategorized files, the Viewer's demo and control edges, the headless application's `src` root and the missing `flightsim-demo` |
| Generated record | `docs/hangar-effects.md` regenerates byte for byte (88 designs) |

The dated architecture baseline is [arch-08-baseline.json](arch-08-baseline.json). Logs are in `D:/ENV/tmp/arch08/`.

Limits:
- **Full suite:** none ran (HC-11).
- **Screenshot timing:** a demo screenshot's sub-frame phase now depends on two processes' clocks. Run to run, it
  varies more than ARCH-06's in-process capture did; the flight itself is bit-identical (`demo_publisher`).
- **`demo_publisher` runs on Release only:** it uses the Viewer's own client, which only render builds compile. The
  headless build compiles `flightsim-demo` but neither tests nor installs it.
- **Toolchain:** builds, the object checks and the ABI comparison ran from PowerShell, as in ARCH-07.

## 7. Test scope

The selector escalates this package to both full suites: it changes build inputs and unclaimed native code. As the
package's instructions and HC-11 require, the related fast set ran instead:
- **The fast tier:** ARCH-07's names, which are every test measured under 5 s plus the named gates. These include the
  include-view probes, the installed C++, C and Python package consumers, `dist_components`, `rust_example`,
  `sdk_boundary`, `ma_baseline_checker`, the MA and training gates with their stress cases, and every native case
  that reads moved data.
- **Added, because this package changes what they exercise:**
  - `launcher_contract`, both presets;
  - `demo_publisher` and `include_view_viewer`;
  - `python_sdk test_mass_accounting`, whose repository path changed;
  - `hangar test_stores.StoresTest` and `hangar test_methods.Aircraft_`, which use Hangar's repository search and
    path helper;
  - headless `hangar test_pipeline.TurbopropStart`, which reads a design through `test_pipeline`'s root.

Headless ran 627 of 713 tests and Release 688 of 774. Not run: the long flight, fleet, radar and conformance suites,
the remaining long Hangar cases, the native engagement suite, the Flow calibration and four long Python modules
(`test_fleet`, `test_launch`, `test_load_limits`, `test_radar`). Their objects differ from ARCH-07's only by their
embedded source path, and they read no moved path.

## 8. References

- **Updated:**
  - the README's layout table and its demo paragraph;
  - `docs/development.md`'s new layout paragraph;
  - every current guide's moved paths, including `docs/hangar.md`, `docs/sdk/*`, `docs/vehicle-interface.md` and
    `docs/mission-systems.md`;
  - the architecture design document's source-map row for `src/applications`;
  - `docs/status.md`'s current-state rows;
  - the architecture tool README (planned modules, walk roots and an ARCH-08 section);
  - the Viewer's help text;
  - CI and the aircraft-design skills' Hangar paths.

  Twelve of these files carry another owner's uncommitted edits, and only this package's lines are committed: the
  design document, `docs/agent-boundary.md`, five `docs/sdk` files, `docs/status.md`, the architecture tool README,
  `check.py`, `boundary.py` and `ma_engagement/test_check.py`. `isolated_mission_actor.py` moves with its owner's
  edits, which stay uncommitted and unchanged.
- **Kept as written:**
  - dated records and plans, including `docs/status.md`'s dated entries;
  - synthetic fixture paths in the architecture and selector tests;
  - the generator text and classified data listed under HC-26.

## 9. Open items

- **HC-25:** `tools/launch` stays outside the tool categories, as the layout's only exception, until ARCH-10.
- **HC-26:** text naming the old tool paths is left in place on purpose: provenance in generated outputs (Hangar's
  templates and outputs, the RCS tables' method string), two shader comments in classified assets, and one CTest case
  title.
- **HC-27:** the Viewer's demo link launches and supervises the publisher process itself, not through
  `foundation/platform`.
- **HC-14, HC-16 and HC-17's item (2):** done by this package, awaiting independent acceptance.
- **HC-05:** its unclaimed testbench result data now lies under `src/applications/testbench/results`.

## 10. Rollback

1. Move the 609 files back with `git mv`, and restore their references, depths and CMake subdirectories.
2. Delete `flightsim-demo`'s `main.cpp`, `DemoLink`, `DemoControl.h` and `test_demo_publisher.cpp`. Restore the
   Viewer's in-process demo, the demo library target `fsim_demo_publisher` (`fsim::demo`) and the `viewer -> demo`
   exception.
3. Point `controlLevelName`'s users back to `control::levelName`, and remove the stream's names and the probe.
4. Revert the policy's layout, `check.py`'s rule, its fixtures and `baseline.py`.

No native model parameter is involved.

## 11. Next

ARCH-09 (relocating the architecture authority) requires the next user-relayed assignment.

## 13. HC-28 and HC-29 correction (2026-10-17)

Corrects the two defects of the [independent review](arch-08-review-2026-10-17.md). Starting HEAD: `8c8bd8d`.

- **HC-28, counted single steps:** the publisher (`src/applications/demo/main.cpp`, `SingleSteps`) counts the control
  block's step requests and takes every one:
  - it hands the runner one step at a time. The runner's single step stays the existing boolean latch, unchanged, and
    the next step is handed over only once the one before it is published;
  - a step starts once the held simulation has settled: nothing published for a tenth of a second and for one and a
    half paced periods. A step the runner was taking as it was held, or a latch a resume left set, is not counted as a
    requested one;
  - a resume drops the requests not yet taken, and quit abandons them;
  - the Viewer writes the pause before a request, both with release ordering, and the publisher reads the count first,
    acquiring, so a request made after a pause is read with it.

  `DemoControl.h` states the contract. Its layout and version, the simulation's timestep and the read-only world stream
  are unchanged.
- **HC-29, process handles:** `DemoLink`'s child process closes its handle whatever the child's state: when it is seen
  to have exited, when it is finished, and on destruction.
- **Selector (HC-05):** the new `demo publisher` scope claims the demo application and the control block. It selects
  `demo_publisher*`, `include_view_demo` and `contracts`, so a change there no longer escalates to the full suite.

| Check | Result |
| --- | --- |
| The new regressions first, on the unchanged publisher and `DemoLink` | `demo_publisher_steps` and `demo_publisher_lifecycle` fail. 20 requests made at once take 1 step: the published simulation time is 0.0167 s against the in-process 0.333 s. Three failed starts raise the process handles from 114 to 117. `demo_publisher` passes |
| The same after the fix | `demo_publisher`: 240/240 steps identical. `demo_publisher_steps`: 20 requests made at once are the in-process demo's 20 steps to the bit, and none follows in the next second. Of 1000 requests, 13 were taken before a resume, and the pause after it holds for a second. A request after that is taken once. The publisher stops on request with 100,000 pending, in 0.02 s. `demo_publisher_lifecycle`: 113 process handles before and after three failed starts and a start and stop. Each 5/5 when repeated |
| Builds | Headless and Release, warnings as errors: 0 warnings |
| The selector's targeted plan for this correction's paths | Headless 8/8 (8.0 s); Release 71/71 (128 s): every renderer test, with `include_view_viewer` and `include_view_demo`. No skips |
| Architecture gate; GUI demo | Accepted. `fsim.cmd viewer --demo` starts the publisher, mirrors it, takes its screenshot and exits, and no publisher is left |

Logs are in `D:/ENV/tmp/hc28/`.

Limits:
- **Full suite:** none ran (HC-11). With `tests/CMakeLists.txt`'s two new test registrations, the selector escalates the
  whole tree.
- **Manual interaction:** these tests do not establish manual pause, step, time-factor and quit use (HC-07).
- **Steps while running:** a step requested while the demo runs is dropped. The in-process demo's latch took one such
  step at the next pause (HC-30).

## 15. HC-28, HC-30 and HC-31 correction (2026-10-17)

Corrects the three items the [recheck at `ecf58a3`](arch-08-review-2026-10-17.md#6-correction-recheck-at-ecf58a3)
left open. Starting HEAD: `ecf58a3`.

- **HC-28, an exact hold acknowledgment:** the publisher's step accounting is `DemoSteps`
  (`src/applications/demo/DemoSteps.h`), and no timing remains in it:
  - a hold pauses the runner, then stops it, which joins its thread, and starts it again, held. The join returns only
    once a step the thread was taking is published, and nothing published until then counts as a requested step;
  - from then on the held runner steps only when handed a step. Steps are handed one at a time, each once the one
    before is published, so every later snapshot is a handed step's;
  - a resume waits until the step handed over is published, so the runner's latch is never left set while it runs.

  The runner, its latch and timestep, and the world stream are unchanged. The join and restart are `SimRunner`'s own
  `stop()` and `start()`.
- **HC-30, requests kept:** a request made while the demo runs, or still pending when it resumes, waits for the next
  hold and is taken there, one step each. Quit abandons them.
- **Acknowledgment:** the control block gains `stepsTaken` (publisher to Viewer): the requests taken, each a published
  step. Its version is 2, and `DemoLink::stepsTaken()` reads it. The existing version check refuses a mismatched
  Viewer and publisher.
- **HC-31, required coverage:** a scope can name the tests a preset must have (`[scope.requires]`):
  - each preset the scope plans selects them, in every tier;
  - an inventory missing any one is a `missing-required-tests` error before any test runs, with or without a build.

  The `demo publisher` scope requires `demo_publisher`, `demo_publisher_steps`, `demo_publisher_lifecycle`,
  `demo_publisher_schedules`, `include_view_demo` and `contracts` on `ucrt64-release`, and the last three on
  `ucrt64-headless`.

| Check | Result |
| --- | --- |
| `demo_publisher_schedules` (both presets): `DemoSteps` against a model of the runner's loop as written. A step is decided at the loop's top and published when it completes; `stop` joins | The review's delayed-work schedule takes 2 requested steps for 2 requests, beside the demo's own step under way. Requests made while running are all taken, and only after the hold. A backlog through a resume and a pause: 12 requests, 12 steps. In 2,000 seeded random schedules every request is one requested step, no own step is counted, and the latch is never set while running |
| Mutations of `DemoSteps`, each detected by that test | Without the hold's join, the delayed-work schedule takes 1 requested step for 2 requests (the review's failure), and seed 2 fails. Resuming with a step handed over fails the backlog schedule |
| The real runner in-process (same test) | 10 requests made while running: none taken before the hold, then exactly 10 agent steps after the hold's state. 9 pending through a resume and a pause: exactly 9 after the next hold |
| Process tests (Release) | `demo_publisher`: 240/240 steps identical. `demo_publisher_steps`: 20 requests made at once are the in-process demo's 20 steps to the bit, acknowledged as 20. 30 held, a resume after 5, 4 more while running, then a hold: all 28 pending are taken, none beyond. That is 29 agent steps after the last running snapshot: the requested 28 and the demo's own step under way. One more request is exactly one agent step. The publisher stops with 100,000 pending in 0.03 s. `demo_publisher_lifecycle`: 113 handles before and after. Each of the four passes 5/5 when repeated |
| Selector | The new cases on HEAD's selector first: 5 of 6 fail (22 sub-tests), and the valid control passes. Then all 82 cases pass, none skipped. These include the review's probe, with the real headless inventory standing in for Release: every tier, and a run with or without a build, exits 3 and starts no runner. On the real inventories every required test is selected |
| The selector's targeted plan for this correction's paths | Headless 9/9 (8.3 s), Release 72/72 (122 s), every required publisher test included, no skips |
| Builds; gates | Headless and Release, warnings as errors: 0 warnings. Architecture and selector checks 3/3 on each preset; the planning checker accepts |
| GUI demo | Scripted `fsim.cmd viewer --demo`: the publisher starts held, and the Viewer mirrors it, takes its screenshot and exits. No publisher is left |

Logs are in `D:/ENV/tmp/hc28b/`.

Limits:
- **Full suite:** none ran (HC-11). With `tests/CMakeLists.txt` changed, the selector escalates the whole tree.
- **Pause latency:** a hold joins the runner's thread, so a pause takes effect after at most one paced period
  (16.7 ms in real time, longer at a low time factor). The publisher publishes nothing during that wait, and nothing is
  lost.
- **Bounds in the process test:** across a resume, `demo_publisher_steps` bounds the demo's own steps at the hold to
  two. The exact counts are the model's and the in-process test's.
- **Manual interaction:** manual pause, step, time-factor and quit use remains HC-07.
- **Test helpers:** two test helpers first demanded an exact count where held stepping goes on. In-process, it failed
  the first run; in the process test, one of five repeats. Both now wait for a floor, and the totals stay exact; both
  were corrected before the commit.
