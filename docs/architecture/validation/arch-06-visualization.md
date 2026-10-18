# ARCH-06 visualization core, effects and source separation

Status: independently accepted at `74efb49` (2026-10-16). The source split at `61830e4` and the subsequent
selector corrections complete this package; HC-13 is closed.
See the [final review](arch-06-review-2026-10-16.md#9-final-hc-13-acceptance-at-74efb49). No rendering regression is established
by the small image differences. ARCH-07 has not started.
Starting main HEAD: `58f9297` (ARCH-05 accepted).

This package moves visualization into `src/visualization`, separates the Vision World-handle adapter from offscreen
rendering, and separates the Viewer from the demo simulation it ran. The scene and UI now read only contracts and
read-only frames: both visualization-to-simulation domain exceptions and both UI public-header exceptions are removed.

No model, RNG, timestep, layout or default changed:
- every first-party object both presets build is unchanged in code, data and symbols, except the two files that were
  restructured (`Vision.cpp`, the Viewer's `main.cpp`) and the two new ones;
- the restructured code's output is unchanged: the Vision cameras' images are byte-identical, and the live native
  strike, the replay and the demo render as before (section 6).

## 1. What moved

| Destination | From | Target |
| --- | --- | --- |
| `src/visualization/render` | `src/render` | `fsim_render` |
| `src/visualization/scene` | `src/world`: Earth, terrain, sky and clouds, vehicle visuals, cameras, frames and interpolation | `fsim_world` (policy module `scene`) |
| `src/visualization/vfx/effects` | `src/world`: `FlowVolumes`, `Exhaust`, `Trails`, `Airflow`, `WingVapour`, `Cocip` | `fsim_world` |
| `src/visualization/vfx/flow` | `src/world`: `FlowSolver`, `FlowFires` (the optional GPU solver) | `fsim_flow` |
| `src/visualization/ui` | `src/ui` | `fsim_ui` |
| `src/visualization/vision` | `src/vision/Offscreen.*`, and the new `CameraScene.*` | new `fsim_vision_core` |
| `src/adapters/vision` | `src/vision/Vision.cpp`, `fsim_vision_c.cpp` | `fsim_vision` (the DLL) |
| `src/applications/viewer` | `src/app/viewer_main.cpp` (now `main.cpp`), `StrikeCapture.h` | `flightsim-viewer` |
| `src/applications/demo` | `src/app/DemoAutopilot.h`, and the new `DemoSimulation.*` | new `fsim_demo_publisher` |

Further details:
- **Renames:** 74, all with `git mv`: 70 source files and the render, world, UI and Vision CMake files.
- **Names:** target names and aliases are unchanged (`fsim_world` keeps its name and alias).
- **Includes:** 129 literal includes in 50 files name the new paths. The contrail test generator
  (`tools/contrails/make_cocip_test.py`) emits the new include too.
- **Separation:** physical effects stay simulation's (`simulation/environment`). Rendered VFX are now `vfx/effects`,
  and Flow keeps its own optional target and dynamic loader in `vfx/flow` (task 2).

## 2. Read-only inputs (task 3)

- **Scene:** includes the public value header `fsim/VehicleState.h` (contracts domain; it defines `sim::VehicleState`)
  in place of the simulation's alias header. It takes snapshot batches from `contracts/world_stream/SnapshotBatch.h`
  in place of the simulation's triple buffer, and the attitude math from `foundation/core/Attitude.h`, which the
  simulation's header only re-exported. The types and functions are the same ones, and the objects are unchanged.
- **UI:** reads the same contracts. Two values it named from simulation public headers are now stated on the wire
  contract (`contracts/world_stream/WorldLayout.h`), beside the fields that carry them:
  - `kMissionFamilies` (13): the payload record's family codes;
  - `kConstraintLowEnergy`, `kConstraintDeviating`, `kConstraintUpset`: the `kDerivedConstraints` bits the monitor
    names.

  The producer (`simulation/runtime/world/World.cpp`) holds `ms::Family` and `control::kActivity*` to them with
  `static_assert`s. Layout, version and values are unchanged.
- **Vision:** `vision::CameraScene` (`vision_core`) renders read-only vehicle frames: each vehicle's state, model, CG
  offset, carried stores and bay doors, plus the clock's UTC for the sun. It never reads or steps a World. The moved
  scene-building, slot, mount and aiming code is unchanged apart from its inputs.
- **Removed policy:** the `world -> sim` and `ui -> sim` domain exceptions, and the `ui -> Capability.h` and
  `ui -> MissionSystems.h` public exceptions.

## 3. Vision adapter (task 4)

`fsim_vision.dll` (`src/adapters/vision`) keeps the public World-handle API and its C ABI. `Sensors` reads the World
into frames and passes them to `CameraScene`. It also keeps:
- the store-status filter (`ms::StoreStatus`, a public value);
- camera-table publishing;
- image access and PNG saving;
- `BatchCameras`.

The offscreen-device failure keeps its `fsim::Error` and message through a dedicated `CameraSceneError`, which the
adapter converts. Other scene exceptions propagate unchanged, so the C ABI's error mapping is unaffected. Exports,
vtables and Python inventories are identical (section 6).

The C API source moved here with the DLL in this package, although path-map row P042 had scheduled it for ARCH-07.
The DLL target moves as a whole, so splitting its sources across a legacy and a destination directory for one phase
would have left a legacy claim (register HC-17).

## 4. Viewer and demo (tasks 5 and 6)

The demo's simulation now lives in the demo application:
- `app::DemoSimulation` (`fsim_demo_publisher`) holds the moved demo setup: the vehicle pool, JSBSim models, initial
  conditions, autopilot and simulation thread. Call order, log text and error exits are unchanged.
- Its header exposes only contracts (`SnapshotBatch`).
- The Viewer calls it through the same six operations it used on the runner.
- The Viewer's sources include no simulation implementation header.

Two links remain on the Viewer executable:
- **Temporary compatibility adapter (task 6):** `flightsim-viewer` keeps its `--demo` command line by linking the demo
  application. Nothing may depend on an application, so this edge is an exact, expiring exception: `viewer -> demo`,
  rule `kind`, until ARCH-08. Its removal gate is a separate demo publisher process whose World the viewer attaches
  to live, with `--demo` either launching it or retired (register HC-16).
- **Control link:** the Viewer still links `fsim_control` for `control::levelName` (public `Control.h`), as before, to
  name control levels in its vehicle list. This is neither training nor FDM stepping. A presentation contract for
  level names would remove it (HC-17).

The demo composes the flight module directly, as the demo always has. The planned graph's `demo -> simulation_api`
would make it fly a World instead, which is a behavior change outside a structural package (HC-17).

## 5. Policy, gate and tests

- **Policy:**
  - `scene` (formerly module `world`), `render`, `ui`, `flow`, `vision` and `viewer` hold no legacy directory or
    file claim;
  - `vision_core` and `demo` are registered, no longer planned;
  - allowed dependencies equal links;
  - the only native exception is `viewer -> demo` (kind); the only public exception is ARCH-07's
    `VecEnv.h -> fsim_c.h`;
  - the gate accepts 25 modules and 378 files.
- **Gate fixture:** `test_a_temporary_application_composition_needs_its_exact_kind_exception`. An application linking
  another is rejected; a `domain`-rule exception does not cover it and is reported stale; the exact `kind` exception
  is accepted. Existing fixtures already reject a scene or UI include or link to simulation without an exception.
- **Include views:**
  - render, Flow, scene (with effects), UI, `vision_core` and demo export include views;
  - their probes (`include_view_render`, `_flow`, `_scene`, `_ui`, `_vision_core`; `_demo` in both presets) show that
    no simulation header, including the old `SnapshotBuffer.h`, is visible to the visualization targets;
  - the demo cannot see the runtime, training or the SDK;
  - the Viewer has no probe, because `fsim_control`, which it links, exports the flight module's view.
- **Probe check:**
  - **Vendor roots:** `cmake/CheckIncludeView.cmake` now searches vendor and imported roots (VSG, JSBSim, stb) as
    system roots, as the build does.
  - **Implicit directories:** it leaves the compiler's implicit include directories to the compiler. The generated
    check records them (`IMPLICIT`); naming one again broke the standard library's search.
  - **Visibility semantics:** repository roots and views are still `-I`, so visibility is tested as before. The
    checker's 13 cases still pass.
- **Selector:** the render/Vision/Viewer scope names the new render, Vision and Viewer paths. Scene, UI, Flow and demo
  sources stay unclaimed and escalate, as before.

## 6. Verification (2026-10-16)

| Check | Result |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | 0 warnings |
| Object identity: per source, every section's contents and the symbol table, against the `58f9297` builds | Headless: all 289 objects unchanged. Release: 335 unchanged (31 of them moved); only the restructured `Vision.cpp` and Viewer `main.cpp` differ; `CameraScene.cpp` and `DemoSimulation.cpp` are new. The final build's objects equal those the visual evidence below ran on |
| Vision images (`fsim_vision_test`, six cameras: RGB and segmentation, own-vehicle hiding) | All six PNGs byte-identical to the pre-change run; both before and after runs are deterministic |
| Live native strike, fullscreen 3840 x 2160 (`tools/viewer_perf/strike_acceptance.py`), before and after | Both pass. Same scene hash, launch 13.03 s and hit 19.53 s, the same target outcome (crew lost, 614 post-hit samples) and capture times. Both after-captures inspected |
| Replay of the same live recording, replay clock, 3 frames | Independent recheck: before/after differences are 82-964 of 8,294,400 pixels (at most 0.01162%); baseline before/before counts are 278-531. Three of six comparisons exceed that observed count range. Differences are small, but these runs do not establish a statistical noise bound or a rendering regression |
| Demo, fullscreen | Runs: 8 x c172x on 5 workers. With the sun fixed to the pre-change run's UTC, mean difference 0.19 levels and 1,132 pixels over 16 levels (a threaded demo). An unfixed run differs only by the 15-minute later sun |
| `tools/ma_contract/baseline.py compare`, both builds | 19 comparisons, 0 failures; all 22 captures byte-identical, including `libfsim_vision.dll` and `_vision` |
| Frozen baseline (`e1945b9`): `verify`; frozen C++/C99/Python consumers on the new installed SDK | 66 artifacts unchanged; five consumers exit 0 |
| Fast tier (ARCH-05's names and the six new probes); it includes `vision`, `vision_c_abi`, `python_sdk test_vision`, `viewer_screenshot`, `strike_capture`, `showcase_native`, the scene/terrain/contrail render tests and the installed-package gates | Headless 576/576 (107 s); Release 646/646 (116 s) |
| NVIDIA Flow calibration against observed fires (outside the tier) | 1/1 (11 s) |
| Include-view probes | 17/17 headless, 22/22 Release |
| Architecture: gate; `test_check.py`; selector `test_scope.py`; planning document checker | Accepted (25 modules, 378 files); 133 cases; 49 cases (1 skipped); accepted |
| Include environments ([arch-06-include-views.json](arch-06-include-views.json)) | All compiled sources of the migrated targets use exactly the recorded roots: 131 headless (16 targets), 162 Release (21 targets); none holds the shared `src` root |

The dated architecture baseline is [arch-06-baseline.json](arch-06-baseline.json). Logs and captures are in
`D:/ENV/tmp/arch06/`.

Limits:
- **Full suite:** none ran (HC-11).
- **Performance:** the 4K performance gate was not remeasured. The render, scene, UI and Flow objects are unchanged,
  and the Viewer's change is the demo call indirection. The dated performance limits in
  [viewer-validation](../../sdk/viewer-validation.md) stand.
- **Interactive acceptance:** CLI captures do not establish manual interaction acceptance.

## 7. References

- **Updated:** the third-party notices, the Flow vendor README, the viewer page and the architecture design document's
  source-tree table name the new paths. In the design document, which carries another owner's edits, only those rows
  are committed.
- **Shared tool text:** the architecture tool README's description of shared legacy file claims, and the matching
  `check.py` comment, now say that Flow and the Viewer held such claims until ARCH-06.
- **Kept as written:**
  - the shader comments in `assets/clouds/shaders`: assets are classified data with hashed provenance and change only
    with a resource change;
  - module-relative `world/...` names in prose;
  - dated records;
  - synthetic fixture paths.

## 8. Rollback

Move the 74 files back with `git mv`; delete `CameraScene.*`, `DemoSimulation.*` and the new CMake files; and restore
`Vision.cpp`'s and the Viewer `main.cpp`'s pre-change content. Then revert:
- the include rewrite;
- the scene and UI include swaps and the two contract constants with their `static_assert`s;
- the policy (restoring its four exceptions);
- the probes, the probe-check search rules and the gate fixture;
- the selector paths;
- the listed document lines.

Restore resource lookup paths as they were; no native model parameter is involved.

## 9. Next

ARCH-07 (SDK assembly and Python import cleanup) requires the next user-relayed assignment. ARCH-06 is independently
accepted at `74efb49`, and HC-13 is closed. The [review](arch-06-review-2026-10-16.md) resolves HC-16/17's scope choices
and retains their assigned follow-ups; other open items remain in the register.

## 10. HC-13 correction: Release selection for renderer changes (2026-10-16)

The [review](arch-06-review-2026-10-16.md#1-p2--hc-13-graphics-changes-can-pass-without-a-release-build-or-test) found
that a change to scene, UI or Flow code was planned on `ucrt64-headless` alone, which does not compile it. Its selection
then passed without the changed code being built or tested. The gap predates ARCH-06 (the old `src/world` and `src/ui`
paths had it).

The correction is in the test selector (`tools/testing`) and the render tests' labels; no product source changed:
- **Render-only scope:** the `render-only roots` scope (`render_only = true`, `release = true`) claims everything only
  the renderer builds. That covers visualization with its nested CMake files, the Vision adapter and
  `python/src/_vision*`, the viewer, `tools/rcs`, `tools/flow_calibrate`, `examples/vision_capture`, the seven
  render-only test sources and `cmake/Flow.cmake`.
- **Selection:** it selects the `render` label plus `sdk_boundary` and the Python Vision/camera tests. Every test only
  the renderer builds now carries that label: the render scene tests, `viewer_screenshot`, `rcs`, `vision`,
  `vision_c_abi`, the Flow calibration and the render include-view probes (59 tests).
- **Presets:** `scope.py` plans `ucrt64-release` for such paths and does not plan `ucrt64-headless` for them alone;
  another claiming scope or path still plans it. `cmake/Flow.cmake`, which every configure reads, keeps its
  build-system escalation too.
- **Compile-database evidence:** independently of the rules, a changed path that the presets' compile databases show
  is compiled only in `ucrt64-release` also plans that preset. That is a source, or a file in a directory holding only
  such sources, such as a header or a nested CMake file. Without both databases the rules decide.
- **Fail-closed:**
  - a missing `ucrt64-release` inventory (`no-inventory`) or a failed build of it is an error, and nothing runs;
  - the submitted negative fixture rejects an inventory containing only `flight core`; independent recheck below
    shows that a Release inventory without render tests can still pass when generic fallback tests remain;
  - an escalation still covers every planned preset.

Regressions, in `tools/testing/test_scope.py` (`testing_scope_checker`, 62 cases on each preset):

| Kind | Cases |
| --- | --- |
| Pure plans on the real rules | 16 render-only roots (scene, UI, Flow, effects, nested CMake files, `vision_core`, the adapter, the viewer, `_vision`, rcs, Flow calibration, the example and two render-test sources) plan Release alone with the render tests. `cmake/Flow.cmake` plans both. Controls: a headless change plans headless only; a mixed change plans both |
| Evidence | An unclaimed Release-only source and a header beside it plan Release alone; a scope that forgot the renderer still plans Release; without databases the old escalation applies. `release_only` reads both databases and ignores a directory that also holds headless sources |
| Lifecycle (`select`, effects injected) | Scene, UI and Flow changes build, list and run `ucrt64-release` only. A missing Release inventory exits 3 and runs nothing. A failed Release build stops before listing. A Release inventory without render tests is an empty selection. A mixed change plans both presets; a headless change only its own |
| Real builds | Every source compiled only with the renderer, and every tracked file in a renderer-only directory, is claimed by a render-only scope; no headless-compiled source is claimed by render-only scopes alone. Every test only the Release inventory has carries `render`, and none in headless does. On both real inventories, scene, UI and Flow changes plan Release alone with every render test, targeted, planned and not run |

Verification (2026-10-16):

| Check | Result |
| --- | --- |
| The review's probe, the real rules with an injected two-preset inventory, planned only. HEAD's selector against the corrected one | HEAD: UI, scene and Flow changes each plan `ucrt64-headless` only, status 0 (the false pass); the render `Viewer.cpp` control plans both. Corrected: all four plan `ucrt64-release` alone, status 0 |
| The new regressions against HEAD's `scope.py` and `scopes.toml` | All seven new cases fail |
| `scope.py --paths`, the real inventories and compile databases, planned only (no build or run) | UI, scene, Flow, the scene's CMake file and the render backend: `ucrt64-release` only, 64 of 769 tests. `Pid.cpp`: the headless suite (shared core, unchanged). UI with `SimRunner.h`: both presets |
| `testing_scope_checker` | 62/62 headless, 62/62 Release, no skips |
| The `render`-labelled tests on `ucrt64-release` | 59/59 (121 s) |
| Headless `architecture_boundary`, `architecture_checker`, `include_view_checker` | 3/3 |

Only the selector, its rules and tests, the render tests' CTest labels and `docs/development.md` changed. No build
input of a product target changed, so no object was rebuilt, and no full suite ran (HC-11). Logs are in
`D:/ENV/tmp/hc13/`.

### Independent HC-13 recheck at 9e66811

The [recheck](arch-06-review-2026-10-16.md#7-hc-13-correction-recheck-at-9e66811) leaves HC-13 open. The genuine
Release inventory plans correctly, but injecting the real headless inventory as Release yields exit 0 with no
`render` tests: a UI change selects 5 generic tests, or all 710 in a full/mixed escalated plan. Stubbed run lifecycles
with and without a build also return 0. Required renderer coverage must be validated separately from the overall
selection being nonempty. The existing negative fixture removes fallback matches and does not cover this case.

Independent `testing_scope_checker` gates pass on both presets (62 cases each, 5.19 s and 5.16 s). No product build,
full suite or GUI scenario was rerun. HC-19 and HC-20 in the register retain the mixed-suite efficiency and shared
build-input Release-policy limits reported by Claude. ARCH-07 remains unreleased.

## 11. HC-13 renderer-coverage requirement (2026-10-16)

The [recheck at `9e66811`](arch-06-review-2026-10-16.md#7-hc-13-correction-recheck-at-9e66811) found the remaining
P2: a renderer change's `ucrt64-release` plan passed on an inventory without one renderer test, because the render
scope's generic names (`sdk_boundary`, the Python Vision and camera tests) still matched. Section 10's claim that such
an inventory is an empty selection held only for its fixture, which had none of those names.

The correction, in `tools/testing/scope.py` only:
- **Requirement:** a path planned on `ucrt64-release` because only the renderer builds it (a `render_only` scope or the
  compile-database evidence) makes that preset's plan require the `render` label. The plan selects every
  `render`-labelled test of the inventory, whatever the tier or the claiming scope. Without one, it is a
  `no-renderer-coverage` error naming those paths, and generic matches cannot satisfy it.
- **Lifecycle:** `select()` checks the inventory listed after any build, so a build that registers the renderer tests
  passes, and a stale pre-build inventory is not the one held. A plan error exits 3 before any runner starts.
- **Retained:** the build-before-inventory refresh, `no-inventory`, the failed-build stop and the escalation of every
  planned preset. A headless plan never asks for the label.

Regressions in `tools/testing/test_scope.py` (`testing_scope_checker`, 69 cases on each preset). The realistic
negative fixture is a Release inventory of `sdk_boundary`, `python_sdk test_vision`, `python_sdk test_camera` and
`flight core`, without any renderer test:

| Kind | Cases |
| --- | --- |
| Pure plan, preset given | Targeted, integration and full: the error without renderer tests, every renderer test selected with them; a headless plan of a mixed change has no such error |
| Lifecycle, the negative fixture | A UI change in all three tiers; a mixed escalated change (UI with `SimRunner.h`); a compile-database Release-only path claimed by a generic scope: exit 3, `no-renderer-coverage`. A run with and without a build: exit 3, nothing run, only the build and listing called |
| Lifecycle, controls | A pre-build inventory without renderer tests whose build registers them passes on the post-build inventory. A valid inventory selects every renderer test in every tier. The generic-scope evidence path then gets every renderer test too |
| Real builds | The review's probe: the real headless inventory given as the Release one fails all three tiers, the mixed escalated change and runs with and without a build, before any runner starts |

The previous single-name fixture (`flight core` only) now also reports `no-renderer-coverage`, and its case was
replaced by these.

Verification (2026-10-16):

| Check | Result |
| --- | --- |
| The review's probe (`D:/ENV/tmp/hc13/probe2*.log`): real rules, the real 710-test headless inventory as Release, builds and runners injected, nothing built or run | HEAD: UI targeted/integration/full and the mixed plan exit 0 with 5, 5, 710 and 710 selected and no render test; both runs start their runner. Corrected: all six exit 3 with `no-renderer-coverage`, no runner started |
| Positive control, the real 769-test Release inventory, the same cases | Both selectors: 64 selected targeted and integration, 769 full and mixed, all 59 render tests in each; the runs reach the runner |
| The new regressions against HEAD's `scope.py` | The six negative cases fail; the two controls pass on both |
| `testing_scope_checker` | 69/69 headless, 69/69 Release, no skips |
| Headless `architecture_boundary`, `architecture_checker`, `sdk_guard_checker`, `ma_contract`, `ma_engagement_contract` | 5/5 |

Only the selector, its tests and `docs/development.md` changed. No build input of a product target changed, so nothing
was rebuilt, and no full suite ran (HC-11).

### Independent recheck at 5f8c139

The [current review](arch-06-review-2026-10-16.md#8-hc-13-recheck-at-5f8c139) confirms the six previous source-path
cases now reject with `no-renderer-coverage` (exit 3) before any runner starts. The genuine Release control still
selects 64 of 769, including all 59 render tests. Both selector gates pass 69/69 cases with no skips (5.91 s and 5.76 s).

One P2 remains under HC-13: the existing `render, vision and viewer` scope requests renderer tests for
`fsim_vision_c.h`, `python/fsim/vision.py` and `assets/**`, but is not `render_only`. Those paths leave the renderer
requirement empty and bypass the new check. The real headless inventory supplied as Release returns 0 with no render
tests, in all tiers; stubbed shader run lifecycles with and without builds also return 0. Require renderer coverage
independently of render-only routing, and preserve these shared paths' headless coverage. The valid Release control
for these paths selects 69 of 769, including all 59 render tests.

No product build, render rerun, full suite or GUI job was executed. HC-19/20 and other open carryovers remain assigned.
ARCH-06 is not fully accepted; ARCH-07 has not started.

## 12. HC-13 shared renderer scope (2026-10-16)

The [recheck at `5f8c139`](arch-06-review-2026-10-16.md#8-hc-13-recheck-at-5f8c139) found the last P2: the shared
`render, vision and viewer` scope (`include/fsim/fsim_vision_c.h`, `python/fsim/vision.py`, `assets/**`) asks for the
`render` label on `ucrt64-release` and keeps headless coverage. Section 11 required renderer tests only for a
`render_only` claim or compile-database evidence, so these paths passed a Release inventory without renderer tests on
generic names.

The correction, in `tools/testing/scope.py`:
- **Requirement:** `renders(rule)` (a `release` scope whose labels include `render`) decides that a path needs renderer
  tests. That covers the render-only and the shared scope alike, and the compile-database evidence still applies. A
  Release scope without the label (the MA public surface, SDK principals) needs none.
- **Routing:** `render_only` keeps deciding only whether `ucrt64-headless` is planned. The shared scope's paths still
  plan both presets, and headless keeps their own selection; a headless plan never asks for the label.
- **Rules:** `testing_scope_checker` checks that every render-only scope `renders`, and every scope selecting the
  `render` label runs on Release. The `scopes.toml` key comment and `docs/development.md` state the requirement.

Regressions in `tools/testing/test_scope.py` (`testing_scope_checker`, 74 cases on each preset). The paths are the
shader `assets/clouds/shaders/bruneton/runtime.glsl`, the public Vision header and the Python wrapper:

| Kind | Cases |
| --- | --- |
| Pure plan, preset given | On Release the three paths need renderer tests and fail on generic tests; their headless plans have no error and keep `sdk_boundary`. `MissionBatch.h` (a generic Release scope) needs none. `renders` on its own |
| Lifecycle, the negative fixture (both presets generic) | Each path in all three tiers; each with `SimRunner.h` (escalated): exit 3, `no-renderer-coverage` on Release only, with headless still planned and selecting. The shader with a run, with and without a build: exit 3, both presets built when asked, no runner started |
| Lifecycle, controls | A valid Release inventory: each path in each tier passes with the renderer tests on Release and `sdk_boundary` on headless. A Release build that registers the renderer tests passes the header's run on both presets. The MA public surface and SDK principals pass on generic inventories |
| Real builds | The real headless inventory as the Release one: the three paths in all tiers, the mixed change and both run lifecycles fail before any runner starts. On both real inventories the three paths plan both presets, headless keeping `sdk_boundary`, Release selecting every render test |

Verification (2026-10-16):

| Check | Result |
| --- | --- |
| The recheck's probe (`D:/ENV/tmp/hc13c/probe*.log`): real rules and compile-database evidence, the real 710-test headless inventory as Release, builds and runners injected, nothing built or run | HEAD `5f8c139`: ten plans (three paths in three tiers, the mixed change) exit 0 with no render test; the two runs start both runners, then exit 4 only because the stub runner writes no log. Corrected: all twelve exit 3 with `no-renderer-coverage`, no runner started |
| Controls, the same probe | Both selectors: on the real 769-test Release inventory the three paths select 69 targeted and integration (769 full and mixed), all 59 render tests, and headless 10 (710 full and mixed). `MissionBatch.h` selects 27 on each preset without renderer tests and passes |
| The new regressions against HEAD's `scope.py` | Every new negative case fails (three cases also by the missing `renders`); the valid and generic-scope controls pass on both |
| `testing_scope_checker` | 74/74 headless, 74/74 Release, no skips |
| Headless `architecture_boundary`, `architecture_checker`, `sdk_guard_checker`, `ma_contract`, `ma_engagement_contract` | 5/5 |

A correction to section 11's wording: in its saved HEAD probe, four plans exited 0 and the two runs reached their
runner, then exited 4 (`ran-other-tests`) from the stub. "Exit 0 in all six" in the earlier handoff was imprecise; both
showed the missing check. Only the selector, its rules and tests and `docs/development.md` changed. Nothing was rebuilt
and no full suite ran (HC-11).

### Independent acceptance at 74efb49

The [final review](arch-06-review-2026-10-16.md#9-final-hc-13-acceptance-at-74efb49) closes HC-13 and accepts ARCH-06.
The unchanged shared-scope fixtures now reject all nine path/tier plans, both run lifecycles and all three mixed
plans when Release lacks renderer tests. Genuine Release controls still select 69 of 769, including all 59 render
tests, and preserve 10 headless tests; ordinary SDK Release controls pass without a renderer requirement.
The registered selector gates pass 74/74 cases each, with no skips (7.16 s headless, 7.08 s Release).

No product build, renderer rerun, full suite or GUI job was executed. Open carryovers, including HC-19/20, retain
their owners and closure conditions. ARCH-07 has not started and awaits the next user-relayed assignment.
