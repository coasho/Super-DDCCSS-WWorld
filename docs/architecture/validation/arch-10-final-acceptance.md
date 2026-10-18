# ARCH-10 final three-domain acceptance

Status: structural/correction subset accepted at `f1bb879`, and HC-11 final regression independently accepted at
evidence commit `66dc43d` (2026-10-17). HC-38 to HC-43 and HC-20 remain closed. HC-07's fullscreen Viewer, manual,
performance and joint gates remain pending, so complete ARCH-10/final-program acceptance is not claimed. HC-44
records nonblocking future fingerprint hardening. See sections 12-14; earlier sections retain dated evidence.
HC-07's diagnostic subset is independently reviewed at `465b7af` (section 16); HC-45 performance, combat prerequisites
and remaining input coverage keep HC-07 open. HC-46/47 are recorded nonblocking Viewer/evidence limits.
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
| Simulation authority, concurrency, reset, RNG and outcome meanings preserved | HC-11 accepted at `66dc43d`: all registered cases pass on the frozen effective source; no model/feature evidence beyond those assertions is inferred |
| Existing binaries, import paths, resources and recordings compatible | Installed consumers and both required complete inventories pass; HC-11 accepted at `66dc43d`, with the recorded source/artifact scope |
| Current required native, installed and graphics evidence complete | Native/full-suite and installed evidence accepted under HC-11; graphics/manual/performance/joint evidence still **pending** HC-07 |
| Historical documentation with one canonical owner and valid redirects | Met in ARCH-09 |
| Every handoff's self-reported items reconciled in the register | Done for this handoff ([register](handoff-carryover-register.md)) |
| Independent review accepted, slots released, only reviewed files committed | Structural/correction and HC-11 reviews accepted, slots released; complete program still awaits HC-07. Planning/review inputs retain explicit uncommitted status |

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

## 10. Independent review at 197a059

The [independent review](arch-10-review-2026-10-17.md) accepts HC-19's bounded escalation, HC-24's planning/policy
alignment, HC-25's declared launch category, HC-04's retained dependency rule and HC-37's test arrangement.
The disclosed Release architecture run is appropriate. Complete ARCH-10 acceptance is blocked by six P2 items:

- **HC-38:** SYSTEM includes bypass owned source/include-view closure checks.
- **HC-39:** nested shared module CMake files omit Release; HC-20 remains open.
- **HC-40:** missing configured-gate registrations can still produce successful selector plans.
- **HC-41:** the source manifest has a stale selector hash and omits the two new checker files.
- **HC-42:** wrong-build replies and render-off replies still containing renderer modules are accepted.
- **HC-43:** the mandatory codemodel 2.9 requirement conflicts with the declared CMake 3.25 support contract.

Existing exact gates pass 5/5 headless (35.82 s) and 2/2 Release (11.57 s). Bounded synthetic/plan-only probes expose
the gaps despite those green gates. The current real headless graph does omit renderer modules; no native
simulation or public ABI defect is alleged. Both inventories contain 67 `python_sdk` tests, correcting section 3's
69/68. The configured checker has 27 total cases including controls, correcting the negative-only wording.

Before-copy verification preserves other owners' README/status payloads. The 17 headless/72 Release artifact
hashes match current binaries, but they do not repair HC-41's source-provenance gap. HC-35 remains pending human
disposition; all handoff slips and stated limits are reconciled in the review/register. Evidence is under
`build/architecture-reviews/arch10-197a059/`. No configure/build, consumer rebuild, GUI or full suite was repeated;
no files were staged, committed or pushed. Stop for bounded corrections and independent recheck. HC-07/HC-11
retain their existing gates; a full run is not authorized by this review.

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

## 12. Independent correction acceptance at f1bb879

The [correction recheck](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) closes all
six P2 findings and HC-20. SYSTEM ownership, recursive shared-CMake routing, per-preset required gates, source
provenance, build/branch validation and the explicit native-3.25/strict-gate-4.2 contract now satisfy their recorded
closure conditions. No new blocking finding was established. Current exact gates pass 3/3 headless (17.50 s) and
2/2 Release (11.63 s); the original-hole probes and controls pass without running any selected complete inventory.

Source provenance was independently verified before review writes: 13 committed inputs at `872f561`, 58 working
paths (30 tracked/28 planning) and the expected package-input set match. Commit `f1bb879` changes only the manifest,
excluded by its stated rule. All 17/72 binary hashes, both 234-file data digests and consumer manifests match.
The manifest remains dated evidence; seven later review-document edits form an explicit overlay rather than a
newly recaptured source snapshot. Logs, before copies and the overlay are under
`build/architecture-reviews/arch10-f1bb879/`.

The scratch verifier is accepted as a one-off evidence helper. It does not establish expected-set completeness
or artifact freshness on its own; those were checked independently. Before future capture/reuse, its owner must
replace post-filtered `rglob` with exclusions before descent and validate any claimed required input set. The
helper was not run in capture mode here. Its limits, corrected development slips, Git-listing measure, original
owner payload preservation and pending HC-35 disposition are reconciled in the review/register.

HC-04/19/24/25 and HC-37 stay accepted. Full suites (HC-11), current fullscreen/manual/performance and joint gates
(HC-07), and functional/model carryovers remain pending. No configure/build, package gate, GUI or full suite was
repeated; nothing was staged, committed or pushed. This accepts the bounded correction, not the complete program,
and authorizes no pending run.

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

## 14. Independent HC-11 acceptance at 66dc43d

The [HC-11 review](hc-11-review-2026-10-17.md) accepts the single full run of each required inventory on the
effective frozen source at `f1bb879`: 717/717 headless and 780/780 Release CTest entries pass, with exact name and
raw-result matches. No CTest entry skipped or went unrun. Headless's three expected internal optional-Vision skips
are exercised successfully on Release; they do not represent three missing CTest entries. All 63 renderer entries
pass, including actual Viewer presented-frame checks.

The committed fingerprint and raw log hashes match; before/after-run checks report the covered snapshot unchanged.
Post-run source differences are the declared documentation overlay. HC-44 records future helper improvements for
uncaptured test executables/plugins/staged Python, full inventory metadata and pre-query tracked exclusions.
These are evidence-scope limits with no demonstrated changed runtime input; they do not require another full run.

HC-11 is closed. HC-07's complete fullscreen/manual/performance/joint evidence and the separately assigned feature
prerequisites remain pending. Clean rebuild/Debug were outside this assignment. No native build, GUI or suite was
repeated during review; no files were staged, committed or pushed. Continue only on the next user assignment.

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

## 16. Independent HC-07 diagnostic-evidence review at 465b7af

The [review](hc-07-review-2026-10-17.md) accepts the dated live-strike, replay/demo visual samples, programmatic
keyboard/setup/lifecycle checks and observed same-process World replacement. Native event/readback evidence and
all supplied hashes match. Standalone-demo exit/cleanup summary fields are author observations, and the reconnect
helper is not a fail-closed gate (HC-47); direct log/image review supports the observed subset.

HC-45 is confirmed: fleet/strike/hit fail both unchanged 4K gates, reheat passes. Current load and changed hit
workload do not establish migration regression or its absence. HC-06 combat handlers/registration/reduction inputs,
HC-02 and mouse/replay/resize coverage remain pending; no human-operated session is claimed. HC-46's --no-gui
panels are pre-existing and nonblocking. HC-07 and complete program acceptance stay open. HC-11 remains closed;
no GUI, native build, full suite or performance run was repeated during review.

## 17. Independent HC-45 evidence review at 0a635d9

The [review](hc-45-review-2026-10-17.md) accepts the raw matched-input measurements, with no unconditional
no-regression or causal acceptance. HC-48/50 require evidence corrections; HC-49 is a nonblocking diagnostic
query-gap issue. The saved 68/68 Release selection covers the changed diagnostic source, while HC-11's full-run
coverage remains dated to its original frozen source. Four staged artifacts changed; no new full run is requested.
HC-45 performance and HC-07 remain open, HC-47 was not fixed, and other assigned gates retain their owners.
No Viewer, benchmark or full suite was repeated; the review used a GPU-free production-source probe and parser/
document checks only. Performance optimization remains deferred under the user's record-and-stop decision.

## 18. HC-47 to HC-50 correction acceptance at 1e9433b

The [recheck](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b) closes HC-48/49/50.
HC-47's new measured evidence is accepted, but missing-observation and event-order false accepts keep its helper
correction open. This does not establish a current Viewer failure. HC-07/full-program acceptance remains open,
HC-45 optimization is deferred, and HC-11 retains its dated frozen-source coverage. Newly selected full suites
remain a pending plan, not authority for another run. Only exact GPU-timer cases and pure helper/document checks
ran in review; no build, GUI or performance rerun. Other carryovers and the human HC-35 decision remain unchanged.

## 19. HC-47 helper-check acceptance at d6bfb54

The [acceptance](hc-45-review-2026-10-17.md#7-hc-47-version-2-correction-acceptance-at-d6bfb54) closes HC-47's
check-logic correction after 149 expected pure results and evidence/owner preservation checks. HC-48/49/50 stay
closed. V2's live run path remains unexercised and needs validation before future automated-gate reuse; no new live
Viewer evidence is claimed and no GUI/performance/full run is requested now. HC-07 and complete program acceptance
remain open, HC-45 optimization stays deferred, and HC-35/other owners and HC-11's dated snapshot scope remain.

## 20. HC-07 sampled interactions and HC-51 acceptance at 5c80892

The [review](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892) accepts the dated window-input mouse/replay/resize subset at `2a09756` and closes HC-51's
guide correction. This supersedes broad earlier interaction-pending labels for those sampled cases only. Human
operation, other panels/cameras/options, HC-45, HC-06 and HC-02 remain pending. HC-52 records nonblocking resize-
helper completeness work before automated reuse; seven real saved images remain accepted. HC-07 and full-program
acceptance stay open. No GUI, performance scene, native build or full suite was repeated; only pure/document checks.
