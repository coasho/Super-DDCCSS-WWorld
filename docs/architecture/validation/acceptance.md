# Architecture and migration acceptance

## 1. Evidence classes

Keep four independent evidence classes:

1. Static ownership/dependency: source, CMake, Python and package graphs.
2. Public compatibility: headers, layouts, vtables, exported names, Python identities and installed consumers.
3. Runtime semantics: native physics/execution, episode/role lifecycle, RNG and output parity.
4. Graphics/visual quality: render/Vision build, complete Viewer runs, image/lifetime/performance evidence.

A pass in one class does not establish another. Historical 617/617 or other totals are prior evidence; discover and report the current inventory after every relevant move.

This planning package has static/document evidence only. It does not claim that the target directory tree already exists.

## 2. Structural matrix

| Gate | Required result | Failure example |
| --- | --- | --- |
| Source ownership | Every maintained implementation has exactly one owner | Same helper listed under SDK and training |
| Native source graph | All conditional branches scanned; no unknown target, forbidden edge or SCC | simulation links env; render imports training |
| Configured build graph | File API/link graph agrees with admitted source graph per required preset | A macro/conditional creates a hidden upward link |
| Header visibility | Private imports reachable through target dependencies; public headers implementation-independent | Parent-relative include bypasses an owner |
| Contracts | No domain implementation or graphics/OS registry in neutral definitions | Wire header imports World runtime |
| Python imports | No eager or deferred source-module or owner SCC; dynamic imports explicit/registered | training locally imports root fsim |
| CPython ownership | One established runtime owner per type | Duplicate MissionEndpoint registration |
| Package graph | Optional graphics/ML do not become core requirements | import fsim loads Vulkan/PyTorch |
| SDK assembly | Final assembly depends on domains; domains do not depend on assembly | training -> final SDK -> training |
| Directory policy | All moved sources and docs are in assigned categories | New implementation left in a compatibility shim |
| Generated resources | Source/generator/runtime ownership and paths remain consistent | Hand-edited aircraft/stores XML |
| Local links/catalog | Canonical documents reachable, no missing target or duplicate authority | Moved plan still linked at a removed path |

Cycles are checked at declared native-target and Python source-module/owner granularity. Python modules inside the same domain must also be acyclic. Mere mutual conceptual references or compatible forward declarations within one native owner are not a native link cycle. Deferred Python imports are real architecture edges, even when initialization happens to work.

A literal-import AST scan is not a proof about arbitrary dynamic import code. Maintain explicit dynamic import/provider allowlists and runtime import tests.

## 3. Required negative fixtures

Each fixture must fail for its intended boundary, with a meaningful diagnostic:

- simulation -> training; simulation -> visualization.
- training core -> render/Flow/Vulkan or final SDK assembly.
- visualization core -> training or native mutable World implementation.
- foundation/contracts -> domain implementation.
- unregistered source/target/import owner.
- two owners for one source or two definitions/registrations for one public symbol/type.
- CMake cycle through aliases, conditional branches or WHOLE_ARCHIVE.
- Python eager and function-local facade backedges.
- unknown dynamic dependency/provider or optional-package eager import.
- public header private/vendor leakage.
- source/build/installed fallback hiding a missing installed file.
- changed IPC/recording stride or unsupported old recording.
- skipped/empty required test selection.

An allowed-dependency table alone is not a test. Plant the violating source/link/import and verify the checker rejects it.

## 4. Runtime and compatibility matrix

| Domain/boundary | Required cases |
| --- | --- |
| Flight/control | Fixed-seed native trajectory/control/event comparisons at unchanged dt/frameSkip; control/authority/refusal parity |
| Mission Systems | Radar/tracking/RF, stores, seeker/guidance/fuze/damage/countermeasure semantics under existing accepted inputs |
| Communications | Modeled delivery, C2 roles/tasks, DMS provenance/ingestion/queue behavior and ownership |
| World construction | Exhaustive native mapping/admission; scenario values independent of training wrappers |
| Simulation output | Publication/recording/focus/observer/capture changes do not change native state or events |
| Legacy VecEnv | Existing action/observation shapes, task/plugin vtables, reset/default behavior |
| Mission episodes | Fresh World/reset fixture, lifetime/move/close/generation/seed and cross-episode independence |
| Mission batches | Worker/reset schedule matrix, role loss/C2 roles, SameStep/NextStep finals and reset order |
| Preopened handles | Native C++/C99/Python reads/submits versus complete batch transitions; Busy changes nothing; later progress |
| Training adapter | No double tick/stream/hook on Busy retry; per-role final snapshot; real M6 trace equivalence |
| Actor/referee boundary | Costs/outcome/truth refusal before actor invocation, legal snapshot equivalence under hidden truth |
| Public ABI | Old-header consumers, sizes/offsets/vtables/enums/errors, exports, C buffer semantics |
| Python | Stable ABI and runtime type owners; signatures/qualified names/pickling; buffer/GIL/lifetime and optional imports |
| Packages | Installed C++/C99/Python, exact DLL/data/native-module paths, poisoned/missing-resource refusal |
| Recording | Existing format readers and byte/layout fixtures; live/replay parity |
| Viewer/Vision | Read-only source, correct camera/images and installed assets; complete native scene and GPU evidence |

New physics/model behavior uses separate numerical/reference acceptance. Pure relocation is rejected if it changes results beyond an explicitly reviewed baseline tolerance.

## 5. Discovery and representative commands

Use the existing launcher and actual required presets. Compilation environment variables TMP, TEMP and TORCH_EXTENSIONS_DIR point at D:/ENV/tmp. Do not create a new toolchain for a directory move.

Representative commands, after the implementation owner acquires the exclusive slot:

~~~powershell
D:/ENV/python/3.13/python.exe -B tools/architecture/check.py --repo .
D:/ENV/python/3.13/python.exe -B tools/architecture/test_check.py
ctest --preset ucrt64-headless -N
ctest --preset ucrt64-headless -L architecture --no-tests=error --output-on-failure
ctest --preset ucrt64-headless -L ma-training --no-tests=error --output-on-failure
ctest --preset ucrt64-headless -R "^(simulation_interface_boundary|c_abi|python_sdk|package_consumer|mission_package_consumer)$" --no-tests=error --output-on-failure
ctest --preset ucrt64-headless --no-tests=error --output-on-failure
~~~

Retain finite test-property timeouts and discover every required test. During a test-source migration preserve names/labels first, then change the selection only with an updated inventory.

Use UCRT64 Release for optional render/Vision build and its current required tests. The exact GPU commands/scenarios come from the retained Viewer/SDK references and current registered inventory; an absent test is pending or an explicit failure, not an empty pass.

A restricted interpreter that cannot discover source paths is invalid evidence. Re-run the scoped read-only check in the appropriate host context; do not report missing-target artifacts as real source violations.

## 6. Installed-consumer provenance

For each current package:
1. Install into an owned isolated prefix.
2. Remove source/build DLL and data fallbacks from the consumer environment.
3. Compile C++/C99 consumers with installed headers and link/load installed libraries.
4. Import Python with the installed package/native extension and verify loaded DLL paths.
5. Exercise both existing simulation and mission/batch APIs where applicable.
6. Compare copied model/resource hashes with source owners.
7. Poison/remove a required owned resource and verify explicit failure.
8. Record compiler/interpreter/package/DLL/data hashes and exact paths.

Do not reuse another consumer's mutable staging directory without the same explicit CTest resource lock.

## 7. Viewer evidence

One GUI slot, one complete native scenario. Record native event/recording evidence and the actual Viewer source. Confirm reset/replacement/reconnect, camera/images and required event-relative captures.

Record GPU/driver, build/compiler, resolution, fixed workload, frame times, native dt/cadence and asset hashes. Distinguish diagnostics from strict isolated public-endpoint joint acceptance.

Current unresolved Viewer quality/performance cases remain pending. The [HC-07 review](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892) accepts the diagnostic subset at `465b7af` and sampled window-input mouse/replay/resize subset at `2a09756`, and closes HC-51 at `5c80892`. HC-45 performance, HC-06 combat prerequisites, HC-02 J6, a human-operated session and the explicitly uncovered panel/camera/option variants keep HC-07 open. HC-52 is a nonblocking reusable-helper completeness limitation; HC-47 to HC-50 and HC-51 are closed with their dated scope. HC-11 remains closed for its frozen source; no repeat full suite is requested.

## 8. Document-package validation

Run without configuring or building native code:

~~~powershell
D:/ENV/python/3.13/python.exe -B docs/architecture/validation/check_documents.py
~~~

This checks planning links/catalogs, target DAG consistency, forbidden domain directions, source-rule uniqueness and expanded destination collisions (case-insensitive for Windows). It validates the plan's internal consistency, not implementation compliance or runtime behavior.

Planning verification on 2026-10-16 at main d344e2b: 9 Markdown documents, 27 native and 9 Python target owners, 69 source mapping units, 22 document registration units, and 399 implementation/public files covered; no target graph cycle or destination collision. The refreshed existing native checker accepts 19 modules/367 source-public files with no native cycle. The Python source audit covers 16 modules and retains the two deferred cycles listed in the current-state audit. No native/runtime/GPU acceptance is inferred from these static results.

At ARCH-10 (2026-10-17, main 6bb79cc working tree), the planning check accepts 50 Markdown documents, 30 native and 9 Python target owners, 77 source units covering 442 files and 25 document units. Its native owners now equal the policy's modules (HC-24). The refreshed audit finds 28 native modules and 401 files with no native or Python cycle. The configured graph of each required preset is checked by `architecture_configured` (`tools/architecture/configured.py`); see the [ARCH-10 record](arch-10-final-acceptance.md). That gate needs CMake 4.2 or later (HC-43); with an older CMake it fails with an actionable diagnostic and the preset's configured-graph acceptance is pending, while native builds stay supported from CMake 3.25.

The static audit snapshot is dated. Re-run source evidence after implementation changes before making a new current-state claim.

The scoped audit can be refreshed separately (this updates only its evidence JSON; use the intended client date):

~~~powershell
D:/ENV/python/3.13/python.exe -B docs/architecture/validation/audit_sources.py --date 2026-10-16
~~~

## 9. Final acceptance record

Record exact HEAD/diff, ownership/path inventory, actual graph/hash, nonzero test inventories, commands/timeouts/results, installed provenance, complete visual evidence and explicit pending functional work.

The final checklist is:
- All three domain implementations in assigned roots.
- No unexplained legacy implementation outside the path register.
- No forbidden link/import/package edge or SCC.
- Thin classified compatibility adapters with one public type/state owner.
- Headless simulation/training independent of graphics and ML.
- Simulation authority, concurrency/reset/RNG and native outcome meanings preserved.
- Existing binaries/import paths/resources/recordings remain compatible.
- Current required native/installed/graphics evidence complete.
- Historical documentation has one canonical owner and valid redirects.
- Every handoff's self-reported problem, limitation and deferred task is reconciled in the [carryover register](handoff-carryover-register.md); each remains open with an owner/package and closure condition until correction evidence closes it.
- Independent review accepted, slots released, only reviewed files committed.

Until these gates are met, report migration progress rather than saying the three-domain architecture is already complete.

The [ARCH-10 record](arch-10-final-acceptance.md) holds this record. The [HC-11 independent review](hc-11-review-2026-10-17.md) closes the final-regression gap at evidence `66dc43d`: the required complete CTest inventories pass once each on the frozen effective source, with exact name/result matches. Expected internal headless Vision skips and fingerprint scope are disclosed; HC-44 tracks nonblocking future helper improvements. The [correction recheck at f1bb879](arch-10-review-2026-10-17.md#9-hc-38-to-hc-43-correction-acceptance-at-f1bb879) and earlier accepted decisions remain valid. HC-07's visual/manual/performance/joint gates remain pending, so final-program/complete ARCH-10 acceptance is not claimed. This review requests no repeated suite.
