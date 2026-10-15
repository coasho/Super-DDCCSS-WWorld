# Architecture A3 Implementation Plan

> **For agentic workers:** Use the existing parallel adapter workflow with disjoint file ownership, root-owned
> builds/integration and an independent final review. Follow executing-plans and verification-before-completion.

**Goal:** Give control and Mission language adapters focused source owners without changing their public contracts.

**Architecture:** Extract existing C/Python definitions literally and retain shared handles, errors, conversions
and Python registration. Add characterization before extraction and compare source/runtime surfaces afterward.

**Tech Stack:** C++17, C11, CPython 3.11 stable ABI, CMake 3.25+, Windows UCRT64 GCC/Ninja.

**Spec:** [A3 design](../specs/2026-10-12-architecture-adapters-design.md).

## Global constraints

- Baseline `34e5516`. Leave A3 uncommitted until requested.
- Preserve public API/ABI, principal permissions, simulation behavior, error order, struct-size semantics and borrowed lifetimes.
- Native simulation/domain sources, public/object-model files, resources and prior adapter owners remain hash-identical.
- Keep World/module/type tables, Error, BufferType and PyInit centrally owned; no new GIL/busy checks or public mixins.
- Tools/caches under `D:/ENV`; compilation temporary directories `D:/ENV/tmp`. Files use English.

## Review focus

- Direct versus guarded success retains the original LastError clearing behavior.
- Mission submission does not overwrite control's last detail; null/short results retain their original side-effect order.
- Native tuple placeholders, public value identity/pickle, FASTCALL and argument conversion order remain unchanged.
- Activity ownership, range/referee restrictions and borrowed cached strings/results retain their guards and lifetimes.
- Shared conversions/constants and macro-generated handlers remain single owners without extra native exports.

## Task 1: baseline characterization

**Files:** new `tests/test_adapter_compatibility.cpp` and `python/tests/test_domain_binding_compatibility.py`;
root registers a standalone C++ consumer target in `tests/CMakeLists.txt`.

**Interfaces:** Consume the existing public C ABI and Python native/public object model. No production interfaces added.

- [x] Add C cases for control/Mission LastError/detail and null/short-result side-effect ordering, plus principal restrictions where needed.
- [x] Add Python cases for extracted descriptor/module ownership and no-keyword flags, argument errors, tuple placeholders, record identity/pickle and representative cross-domain operations.
- [x] Build/run these cases against A2 before any production extraction; preserve initial fixture failures and corrections.
- [x] Capture source, all 229 World descriptors/32 module functions, exports and protected-file hashes. Observe extraction-verifier failure before the new owner files exist.

## Task 2: C control extraction

**Files:** new `src/sdk/fsim_control_c.cpp`, new `src/sdk/CParameterSupport.h`, `src/sdk/fsim_world_c.cpp`, SDK CMake.

**Interfaces:** Existing 233 `FSIM_API` definitions and 55 local helper definitions link into the same SDK DLL;
the unchanged `params(const char* const*, const double*, uint32_t)` definition serves control and world privately.

- [x] Move the main actuator-to-field-count section, hold/batch entry points, control-only helpers and five layout assertions literally.
- [x] Move only params into private support, retain its namespace/linkage and include the support in both users.
- [x] Preserve 157 first principal guards and 32 source-authorization macros; keep CAbiSupport, Handles, Mission and links unchanged.
- [x] Register the source; compare all signatures/bodies, guard positions, macro definitions and original-file remainder against A2.

## Task 3: Python control/Mission extraction

**Files:** new `python/src/_native_control.c` and `_native_mission.c`, `_native.c`, `_native_world.h`, Python CMake.

**Interfaces:** Existing control 112 World/13 module handlers and 46 helpers, and Mission 37 World/12 module handlers
and four helpers. Private support adds declarations for existing as_int/get_array and keeps FSIM_PY_VALUES at 40.

- [x] Extract audited ranges, retaining as_u64/as_doubles in core and preserving the control name macro's body/arguments.
- [x] Add private prototypes and only required linkage changes; define FSIM_PY_NO_BUFFER_TYPE in the new owners.
- [x] Retain terrain/property/state/lifecycle and all tables/type/module registration in core; register both new sources.
- [x] Compare every moved/shared body, original-file remainder, constants/macros, busy/GIL/buffer operations and runtime descriptors/exports.

## Task 4: integration, review and documentation

**Files:** root-owned architecture/development/status/navigation documents and this plan; evidence under `D:/ENV/tmp`.

- [x] Build the full headless preset with warnings as errors and run C/Python characterization plus architecture gates.
- [x] Independently review both adapter changes and contract tests; fix actionable findings with evidence.
- [x] Run the full applicable functional suite, including C ABI, control/Mission, agent/principal, isolation/thread/buffer, determinism and package gates.
- [x] Refresh dist; compare all runtime manifest hashes, tested DLL bytes, public exports and 3,514 protected hashes.
- [x] Check English content, Markdown links/anchors, command syntax, diff scope and exact registration/source comparisons.
- [x] Record the A2 commit, A3 acceptance, limitations and remaining queue; leave A3 uncommitted.

## Progress and decisions

- A2 committed as `34e5516` after fresh architecture/export/scope/distribution/documentation verification; clean tree before branching.
- Captured 3,514 protected file hashes, 1,338 SDK exports, all 229 World descriptors and 32 module functions from the A2 binaries.
- Root owns all configure/build/test/dist operations and test-source registration. C/Python implementers own disjoint adapter files; the characterization implementer owns only the two new test files.
- CParameterSupport is a separate focused private header, keeping prior CAbiSupport and Mission/links owners unchanged.
- Pre-flight: C and Python adapters consume unchanged public C declarations; only the test-registration and documentation tasks share root-owned files. No conflicting production edits are assigned.
- The user's explicit next-round instruction authorizes the stated continuation. Reuse the clean feature branch and existing build cache; no additional worktree or intermediate approval gate is needed.
- Baseline C consumer passed 1/1 and Python domain characterization passed 6/6 before production edits. Four C fixture failures were preserved: an unrequestable STARTING target, activity ordering, unconfigured forged claims and the boolean return of agent_identity. The fixtures were corrected against public contracts and existing implementation, without production changes.
- The standalone C main was initially registered with the Catch SDK target; root separated its target before baseline acceptance and restored SDK test discovery. No Catch cases were removed.
- C extraction passed literal whole-file comparisons for 233 exported definitions, 55 local helpers, the unchanged parameter helper and the original-file remainder. Root applied the prepared C extraction after its implementer encountered a model-capacity failure.
- Python source comparisons passed for 171 control and 53 Mission definitions, the name macro, all eight shared helpers, 14 registration initializers, four type specs, PyInit, WorldObject layout and the original core remainder. Only moved handler linkage and the two existing shared-helper declarations change.
- Full headless build passed with FSIM_WARNINGS_AS_ERRORS=ON; all adapter translation units compiled into their existing SDK/Python binaries and the package was restaged.
- Focused CTest acceptance passed 4/4 in 82.72 seconds; architecture checks passed 3/3 in 2.29 seconds. All 229 World descriptors, 32 module functions, World layout and native exports match A2.
- Independent read-only review found no actionable regression in the complete C/Python/test diff. It confirmed the 51 existing control busy checks, 10 buffer releases, Mission's existing busy policy, reference/memory operations, cached readbacks and result-ordering distinctions.
- The formatting check found extra EOF blank lines in both new Python owners. Root removed only those blank lines, repeated literal checks successfully and rebuilt before the full functional suite.
- Full functional suite passed 548/548 in 323.33 seconds, including both package gates. Together with the three architecture checks this covers 551 distinct CTest cases.
- Distribution refreshed successfully. All 1,230 runtime manifest entries and tested installed-consumer DLL bytes match dist; the staged and distributed Python module/object-model bytes match. The two binding compatibility suites passed 13/13 against dist, and all runtime descriptors/exports and 3,514 protected hashes match A2.
- Documentation acceptance passed: seven changed/new Markdown files, 158 local links, six anchors and five PowerShell blocks. English, exact file endings, protected scope and diff checks passed; synchronized development skill copies were unchanged. Status recorded A2's commit, A3 acceptance, explicit untested configurations and the remaining queue. A3 was left uncommitted at that stopping point.
- Subsequently committed as e79f268 at the user's request after fresh architecture/C compatibility, source/export,
  protected-scope, resource/distribution and documentation verification. No push was performed.
