# ARCH-10 final three-domain acceptance

Status: implemented on main (2026-10-17); awaiting independent acceptance. The structural, compatibility and
installed-consumer gates pass on both required presets. The full suites (HC-11), the fullscreen Viewer scenarios and
the joint/combat gates (HC-07) stay pending, as the user directed, so this record reports migration progress: it does
not declare the three-domain architecture complete.
Starting main HEAD: `6bb79cc` (ARCH-09 accepted).

The user's decisions for this package (2026-10-17):
- **HC-19:** escalation is bounded per preset.
- **HC-20:** shared build inputs plan and escalate both presets.
- **Configured build graph:** implemented as a CTest of each preset.
- **HC-25:** the launchers' implementation becomes a declared tools category.

## 1. Carryovers resolved (awaiting independent acceptance)

| ID | Resolution | Evidence |
| --- | --- | --- |
| HC-04 | **Retained by decision.** The package root imports its modules eagerly because `fsim.World`, `fsim.Agent` and the others are its public API (ARCH-07). Every test importing `fsim` therefore depends on each module, and selecting them all is dependency coverage, not over-selection. No selector change | The existing import-closure regression (`test_the_python_import_closure`); [development guide](../../development.md#change-scoped-testing) |
| HC-19 | **Bounded per preset.** An escalation applies to the presets its own path plans. A shared-core change escalates `ucrt64-headless`; a renderer change beside it adds `ucrt64-release` with its own selection, every `render` test among it. Example: World.cpp plus a UI source drops the Release full suite (778 tests, an estimated 881 s) to the renderer selection | `PresetEscalationTest` (8 cases) and the updated mixed-change lifecycle case. On the old selector 5 of the 9 fail (15 subtests); the 4 controls pass on both |
| HC-20 | **Both presets.** The `build system` scope is `release = true`: root and `src` CMake files, `CMakePresets.json`, `cmake/**` (less the test and package scripts their own scopes claim), `deps/**`, `third_party/**`, `tests/CMakeLists.txt` and `python/CMakeLists.txt` plan and escalate both. Planning never authorizes a full run (HC-11) | `test_shared_build_inputs_plan_and_escalate_both_presets` (9 paths, each failing on the old rules) |
| HC-24 | **Aligned and enforced.** The planning target graph's native section now uses the policy's names, targets, domains and directories: `transport` became `ipc`; `env`, `mission` and `sdk` list their destinations; `python_abi`, `python_native` and `vision_native` were added; `app` gained its registered `sim` edge; `demo`'s target is `flightsim-demo`. The path map's P004 owner is `ipc`. The planning checker now fails any drift | Planning checker: 30 native and 9 Python owners. Three planted drifts (a rename, a dropped edge, a merged directory) each fail |
| HC-25 | **Declared.** `launch` is the fifth tools category in the policy and the design; the layout exception is gone, and no file moved | Gate: 601 files in 14 categories, no layout exception. `FinalPolicy` (3 cases) holds the committed policy to no exception of any kind and no reservation waiting for an architecture phase |

## 2. Structural acceptance (tasks 1 and 2)

Every required graph has no unknown owner, forbidden edge or cycle:

| Graph | Result |
| --- | --- |
| Native source graph (`check.py`, every CMake branch) | Accepted: 28 modules, 401 source and public header files, 42 classified public headers. No module, public, Python or layout exception remains |
| Configured graph, `ucrt64-headless` (`configured.py`, the preset's own CMake File API code model 2.11) | Accepted: 86 targets; 20 module targets and 67 module links, each a source-graph edge, with every source edge between configured modules configured. The 8 renderer modules are absent because `FSIM_WITH_RENDER` is off |
| Configured graph, `ucrt64-release` | Accepted: 105 targets; all 28 module targets and all 97 source edges configured. Include directories lie within each module's closure, with no `src` or ancestor root |
| Python imports | 29 modules (27 sources and 2 extension leaves), 63 literal imports. No eager or deferred cycle (2026-10-16 had two deferred cycles); optional packages stay out of the eager closure |
| Planning target DAG and catalogs | Accepted: 30 native and 9 Python owners, no cycle, 77 source units covering 442 files, 25 document units, 51 documents reachable |
| Every local Markdown link in the repository (the scratch checker; `.codex` excluded in the Git listing, before access) | 106 files, 1,324 links, none broken, after the records were written; the documentation tests then passed 3/3 |
| Refreshed scoped audit (`dependency-audit.json`) | 28 modules, 401 files, no native or Python cycle |

Further details:
- **Records:**
  - [arch-10-baseline.json](arch-10-baseline.json) holds the final inventories with hashes: owners, links, includes, public headers, Python imports and declared package outputs;
  - [arch-10-provenance.json](arch-10-provenance.json) holds the toolchain, the configuration, the inventories and the artifacts.
- **Configured graph gate:**
  - `cmake/ArchitectureChecks.cmake` requests each configure's own code model and cache, with `cmake_file_api()`;
  - `architecture_configured` holds them to the source graph in each preset;
  - its 27 negative fixtures (`architecture_configured_checker`) cover the rules: unregistered targets, foreign sources, hidden links and dependencies, dropped edges, unconfigured modules, broad, foreign and unknown include roots, and an absent or unusable reply (pending, not passed). The rules are in [the tools README](../../../tools/architecture/README.md#reconciling-with-the-configured-build-arch-10).
- **Tests compiling application sources (HC-37):** `fsim_demo_publisher_test` compiles the demo publisher's `DemoSimulation.cpp` and the Viewer's `DemoLink.cpp`; `fsim_demo_steps_test` compiles `DemoSimulation.cpp`. Applications have no library to link, so the gate allows a test target to compile an application's sources and names each one. A library's source never compiles outside its own target.

## 3. Native, compatibility and installed gates (task 3)

| Run (2026-10-17, exclusive slot; exact names, `--no-tests=error`) | Result |
| --- | --- |
| Configure and incremental build of both presets | Both pass. Nothing was compiled: the native objects are unchanged since HC-32's builds, and only data, DLLs and the Python package were staged |
| Selector plan for this package's 21 paths (architecture checker, documentation, test selection) | `ucrt64-headless` 29/29 (25.8 s), `ucrt64-release` 35/35 (26.9 s): the `architecture` label, `ma_contract`, `sdk_guard_checker`, `testing_scope_checker` |
| Compatibility and installed consumers, on each preset | 7/7 headless (143 s) and 7/7 Release (148 s): `c_abi`, `sdk_boundary`, `package_consumer`, `dist_components`, `mission_package_consumer`, `equipment_package_consumer` and `launcher_contract` |
| Checker suites (direct) | `test_check.py` 144, `test_configured.py` 27, `test_scope.py` 90 (1 skipped without a build directory) |

Pending, as directed:
- **Full suites:** `ucrt64-headless` has 717 tests and `ucrt64-release` 780. Their labels are in the provenance record, and they include the 69 and 68 Python SDK tests. They run on frozen code after this review (HC-11).
- **Runtime matrix:** the flight, MS, communications, episode and batch cases of the acceptance matrix are covered by those suites, not by this package.

## 4. Visual evidence (task 4)

No GUI slot was used. The dated [Viewer evidence](../../sdk/viewer-validation.md), ARCH-06's before/after fullscreen
runs and ARCH-08's scripted captures keep their diagnostic scope.

Pending under HC-07:
- complete fullscreen scenarios on the current build;
- manual pause, step, time-factor and quit;
- the 4K performance gate;
- installed public-endpoint joint acceptance.

## 5. Provenance and pending functional goals (task 5)

[arch-10-provenance.json](arch-10-provenance.json) records the build's provenance:
- **Toolchain:** g++ 16.2.0 (MSYS2 UCRT64), CMake and CTest 4.4.3, Ninja 1.13.2, Python 3.13.12 for the checks.
- **GPU:** NVIDIA GeForce RTX 5070 Ti, driver 596.36.
- **Each preset:**
  - its cache options and CTest inventory by label;
  - SHA-256 of every staged DLL, executable and extension: 17 headless, 72 Release;
  - the staged `share/flightsim` data: 234 files, with one digest;
  - the configured-graph facts;
  - the installed mission consumer's provenance manifest.
- **Working tree:** each uncommitted change, other owners' included, with its hash.

The functional model goals stay outside this migration:
- **Simulation:** MS calibration, DLZ/WEZ, sensors and electronic warfare, and the OMS adapter (MS-12, a planned reservation).
- **Visualization:** the native training-image provider (VIS-F, a planned reservation), and Viewer quality with the 4K 160 fps target.
- **Training:** the full M5-M8 combat criteria, rewards and joint gates (HC-06, HC-07).
- **Upper layer:** the formal MA runtime and DRL training, which follow this program.

## 6. Final checklist

| Item ([acceptance](acceptance.md#9-final-acceptance-record)) | State |
| --- | --- |
| All three domain implementations in their assigned roots | Met: every module has destinations only, and no legacy directory remains |
| No unexplained legacy implementation outside the path register | Met: 442 files mapped, and the gate finds no unowned source |
| No forbidden link, import or package edge, and no cycle | Met in the source, configured (both presets), Python and planning graphs |
| Thin classified adapters with one public type or state owner | Met as accepted in ARCH-07 (HC-15, HC-21, HC-22); unchanged |
| Headless simulation and training independent of graphics and ML | Met: the headless preset configures no renderer module, and the optional packages stay out of the eager import closure |
| Simulation authority, concurrency, reset, RNG and outcome meanings preserved | No native object changed in this package; current proof needs the full suites (HC-11). **Pending** |
| Existing binaries, import paths, resources and recordings compatible | Installed C++, C99 and Python consumers pass on both presets; the full compatibility suites are **pending** (HC-11) |
| Current required native, installed and graphics evidence complete | **Pending:** the full suites (HC-11), and the GUI and joint gates (HC-07) |
| Historical documentation with one canonical owner and valid redirects | Met in ARCH-09 |
| Every handoff's self-reported items reconciled in the register | Done for this handoff ([register](handoff-carryover-register.md)) |
| Independent review accepted, slots released, only reviewed files committed | Slots released. Independent review **pending** |

## 7. Self-reported items

- **HC-37 (new, documented arrangement):** two test targets compile application sources, as section 2 says.
- **Architecture scope on Release:** the `architecture checker` scope now plans `ucrt64-release` too, because the
  configured graph differs per preset. This goes beyond HC-19 and HC-20, and costs about 27 s.
- **Gate assumptions:**
  - a Python extension module is identified by its destinations under a `python` directory;
  - an absent module is allowed only when its branch's cache option is off;
  - a configure without Python (`FSIM_BUILD_PYTHON` on but no suitable CPython) is rejected rather than passed.
- **Build trees:** the native objects come from HC-32's builds; this package compiled nothing. The trees include the
  other owners' uncommitted edits (Python files among them), as the provenance records.
- **Slot check:** no build job of this repository was running. An unrelated long-running process named `GCC`, started
  on 2026-10-02, was left alone.
- **Owners' edits:** `tools/architecture/README.md` and `docs/status.md` carry other owners' uncommitted edits. Only
  this package's changes are committed (HEAD-plus-mine blobs), and copies taken before any edit are in
  `D:/ENV/tmp/arch10/before`.
- **Provenance capture:** the first capture dropped the leading dot of `.claude/CLAUDE.md`, from a stripped status
  line. It was fixed and recaptured before the commit, and all 36 changed paths carry hashes.
- **HC-35** still awaits the user's acknowledgment.

## 8. Rollback

1. Revert this package's commit.
2. Reconfigure both presets: `cmake/ArchitectureChecks.cmake` stops requesting the File API reply, and the two tests disappear.
3. Restore the planning files from the before copies: the target graph, the planning checker, the path map, the design, the index, the roadmap, the register and the acceptance page.

## 9. Next

Independent acceptance of this package. Then, on the user's assignment, the frozen-code full suites (HC-11) and the
GUI and joint gates (HC-07) complete the final-program evidence.

## 11. HC-38 to HC-43 correction (2026-10-17)

Corrects the six items of the [independent review](arch-10-review-2026-10-17.md). Starting HEAD: `197a059`.

The user's decision for HC-43 (2026-10-17):
- native builds keep CMake 3.25 support;
- `architecture_configured` requires CMake 4.2 or later;
- with an older CMake it fails with an actionable diagnostic and leaves the preset's acceptance pending;
- there is no closure-mode fallback.

| ID | Correction | Regressions |
| --- | --- | --- |
| HC-38 | `configured.py` classifies owned paths first, whether or not the include is SYSTEM: a directory under `src` must lie in the destinations of the module or of a module in its closure, and an include view must belong to the same set. The SYSTEM allowance applies only afterwards, to vendor and dependency-prefix directories in the checkout | The two SYSTEM refusals (`src/sim`; `fsim_sim`'s view), and a SYSTEM control with owned, vendor, dependency-prefix and toolchain directories |
| HC-39 | The `build system` scope claims every CMake file both configures read, at its real depth: `src/**`, `examples/**` and `tools/**` `CMakeLists.txt`. Two kinds stay out: what only the renderer's branch configures, which the render-only roots keep on Release alone, and the test and package scripts their own scopes claim. The installed SDK package scope now claims the equipment consumer project. HC-20's decision now holds at every depth | 14 nested paths (core, platform, flight, DMS, SDK, its Python extension, training, the headless application, contracts, examples, tools, Hangar's native library) plan and escalate both presets. Core's CMake file plus a UI source escalates both suites. 8 render-only CMake files keep their Release-only routing. `test_every_tracked_cmake_input_is_routed` checks all 131 CMake inputs in the repository |
| HC-40 | The architecture checker's scope requires `architecture_boundary`, `architecture_checker`, `architecture_configured` and `architecture_configured_checker` on each preset it plans (`[scope.requires]`) | `ArchitectureGatesRequiredTest` (7 cases): the review's legacy-gate-only inventories, both configured gates missing, each one missing on either preset, a gate still missing after the build (stops before any runner), registration by the build (passes), a valid inventory, and a documentation change that needs none |
| HC-41 | A new [provenance record](arch-10-provenance.json), committed after this correction against it as its base. Its source manifest lists: each package input hashed at the base commit, the two configured-checker files included; every working-tree path differing from the base, which is the other owners' edits and the 28 untracked planning inputs; and its own path, excluded by rule. A verifier recomputes all three | The verifier (`D:/ENV/tmp/arch10-fix/provenance.py`, SHA-256 `c35a69cc33915f2e`) rejects a changed hash, a dropped entry and the review's stale `scope.py` hash |
| HC-42 | The reply's build directory must be the requested one. A module is configured exactly where its branch is on: a renderer or Python extension module configured with its option off fails as `disabled-branch-module`, and an absent one with its option on still fails | A wrong-build reply is an input error. Render-off replies with all, or only one, renderer module present are rejected, as is Python-off with the extension present. The absent-renderer and Python controls are kept |
| HC-43 | `configured.py` reads the CMake version from the reply, or from the cache when there is none, and requires 4.2. The configure warns below 4.2, and the development guide and the tools README state the requirement | Replies from CMake 3.25.0, 3.31.6 and 4.1.2 are pending, with the diagnostic; 4.2.0 is accepted; with no reply, the diagnostic names the configuring version |

Further details:
- **Red evidence:**
  - on the gate before this correction, 10 of the 37 configured-checker cases fail (12 with subtests);
  - on the rules before it, 6 of the 10 new selector cases fail (25 with subtests);
  - the controls pass on both.
- **Count corrections (HC-41):**
  - both inventories have 67 `python_sdk` tests, where section 3 says 69 and 68;
  - the configured checker had 27 cases in total at `197a059`, acceptance controls included, where section 2 calls them negative. It now has 37.
  - The 17 headless and 72 Release artifact hashes and both data digests are unchanged, and the new record retains them.
- **Listing:** `RulesTest` now lists files with `--exclude=.codex` as well, so Git never descends into a `.codex` directory (HC-35's corrective measure).

| Check (exclusive slot; exact names, `--no-tests=error`) | Result |
| --- | --- |
| Selector for the 7 changed code paths, with `--build`. Both presets reran CMake; nothing compiled | `ucrt64-headless` 29/29 (25.6 s), `ucrt64-release` 35/35 (28.1 s); the four architecture gates required and present on each |
| Checker suites | `test_configured.py` 37, `test_scope.py` 101 (1 skipped without a build directory), `test_check.py` 144 |
| Real configured graphs (CMake 4.4.3) | Both accepted: headless 20 modules and 67 links, the renderer modules absent; Release 28 and 97 |
| Documentation | Planning checker: 52 documents, 30 native and 9 Python owners. Repository links: 107 files and 1,339 local links, none broken. Documentation tests: 3/3 |

No full suite, GUI run or package gate was repeated (HC-11, HC-07). Logs and the before copies are in `D:/ENV/tmp/arch10-fix/`.

## 13. HC-11 final regression (2026-10-17)

The [HC-11 record](hc-11-final-regression.md) has the details.

After the review closed HC-38 to HC-43 and the related fast checks passed (29/29 headless, 35/35 Release), the
source tree was frozen and fingerprinted:
- HEAD `f1bb879` plus 58 differing paths;
- snapshot digest `e26241d67a879d4e`;
- both build trees' artifacts, data, inventories and cache options.

Each required full suite then ran once on that snapshot:

| Preset | Result | Wall time |
| --- | --- | --- |
| `ucrt64-headless` | 717/717 passed | 1,123 s |
| `ucrt64-release` | 780/780 passed | 1,138 s |

Further details:
- **Snapshot:** `verify` after each run found it unchanged.
- **Coverage:** the Python SDK (67 per preset), contention, package, launcher and renderer tests are among the passed
  tests. `viewer_screenshot` ran rather than skipping.
- **Checklist (section 6):** current full-suite proof now exists for the rows on simulation authority, concurrency,
  reset, RNG and outcome meanings and on compatible binaries, import paths, resources and recordings.
- **Still pending (HC-07):** the graphics evidence, meaning the complete fullscreen scenarios, manual controls, the
  4K performance gate and joint acceptance. Final-program acceptance is not claimed.

## 15. HC-07 verification (2026-10-17)

The [HC-07 record](hc-07-viewer-and-joint.md) has the details. Verified on the HC-11-tested binaries at 3840 x 2160
fullscreen:
- the continuous live strike;
- the replay frames;
- the demo;
- live reconnection to a replaced world;
- the demo's pause, step, time factor and quit through the window's key input (15/15).

**Not met:**
- **The 4K performance gate:** it fails for fleet, strike and hit in two runs (HC-45, new).
- **The combat cases:** J5, J9, the combat parts of J3, J8 and J10, the combat Viewer run and the seed/aspect matrix
  still lack their prerequisites (HC-06).
- **The rest:** a human-operated session, mouse, the timeline slider and resize remain unverified.

Section 6's row "current required native, installed and graphics evidence complete" therefore stays pending, and HC-07
stays open.
