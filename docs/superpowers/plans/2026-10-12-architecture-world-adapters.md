# Architecture A4 Implementation Plan

> **For agentic workers:** Use the existing parallel adapter workflow with disjoint file ownership, root-owned
> builds/integration and independent final review. Follow executing-plans and verification-before-completion.

**Goal:** Separate World, Scenario and Environment language adapters while preserving their observable contracts.

**Architecture:** Move literal bodies into focused C/Python source owners, retaining shared handles, errors, layouts,
field metadata and central Python registration. Characterize A3 first, then compare source and runtime surfaces.

**Tech Stack:** C++17, C11, CPython 3.11 stable ABI, CMake 3.25+, Windows UCRT64 GCC/Ninja.

**Spec:** [A4 design](../specs/2026-10-12-architecture-world-adapters-design.md).

## Global constraints

- Baseline e79f268; branch codex/architecture-world-adapters. Leave A4 uncommitted until requested.
- Public API/ABI, principal permissions, simulation behavior, validation order, borrowed lifetimes and defaults stay unchanged.
- Only the five Environment definitions may leave C control; all its remaining source and prior domain owners stay literal.
- Python runtime type ownership and all registration stay core; no public object-model edits or extra busy/GIL checks.
- Files English, conversation Chinese. Existing tools/caches D:/ENV; compilation temporary directories D:/ENV/tmp.

## Review focus

- Scenario borrowed strings survive other calls; short structs retain conditional writes.
- Truncated IDs still report the full count; later application errors retain earlier side effects.
- Direct Environment calls preserve LastError and NaN/short-result behavior; first principal guards retain refusal order.
- World/Scenario type identity, descriptor flags, buffer lifetime and conversion errors survive new translation units.
- Shared immutable metadata and ScenarioObject layout stay literal; constructor/deallocator and registration stay coherent.

## Task 1: baseline characterization

**Files:** new tests/test_world_adapter_compatibility.cpp and python/tests/test_world_binding_compatibility.py;
root registers a standalone C consumer in tests/CMakeLists.txt. Evidence under D:/ENV/tmp.

**Interfaces:** Existing public C ABI, native Python and public object model only.

- [x] Add focused public C/Python cases after inspecting existing contracts and fixtures.
- [x] Build/run new cases against committed A3 before production changes; correct assumptions with preserved failure logs.
- [x] Capture protected hashes, SDK exports, native descriptors/module functions and type layouts.
- [x] Prepare exact extraction verifiers and observe failure while new owners are absent.

## Task 2: C Scenario/Environment extraction

**Files:** new src/sdk/fsim_scenario_c.cpp and fsim_environment_c.cpp; world/control source remainders and SDK CMake.

**Interfaces:** Same six Scenario and eight Environment FSIM_API definitions; existing shared CAbiSupport and Handles.

- [x] Move world Scenario lines 609-675 and Environment 121-138/479-500 literally.
- [x] Move control Environment lines 1630-1640/2897-2923; preserve adjacent control definitions and both original remainders.
- [x] Preserve all five moved principal guards, linkage, short output and error/side-effect ordering.
- [x] Register both sources; verify inventories/bodies/remainders and unchanged prior helpers/owners.

## Task 3: Python World/Scenario/Environment extraction

**Files:** new _native_world.c, _native_scenario.c, _native_environment.c and _native_world_options.h;
_native.c, _native_world.h and Python CMake.

**Interfaces:** Existing 23 World, four Scenario and eight Environment callable definitions, shared layouts/conversions.

- [x] Move audited bodies and private spec/Environment metadata literally, changing only required static linkage.
- [x] Share literal World fields and ScenarioObject privately; move COUNT to private support without new accessor functions.
- [x] Preserve core runtime ownership and all registrations; add matching private prototypes and CMake sources.
- [x] Compare moved/shared bodies, metadata, original remainder, all registration/type layouts and runtime descriptors/exports.

## Task 4: integration and acceptance

**Files:** root-owned architecture/development/status/navigation documents and this plan; evidence D:/ENV/tmp.

- [x] Build full headless preset with warnings as errors; run new characterization, C/Python and architecture gates.
- [x] Independently review complete C/Python/test diff and resolve actionable findings.
- [x] Run full applicable functional suite with lifecycle, determinism, principal, buffer and package coverage.
- [x] Refresh dist; verify SDK exports, protected hashes, consumer DLL bytes/runtime manifest and distributed Python compatibility.
- [x] Verify English, file endings, links/anchors, PowerShell blocks, source comparisons and diff scope.
- [x] Record A3 commit, A4 acceptance, limitations and remaining queue; leave A4 uncommitted.

## Progress and decisions

- A3 committed as e79f268 after fresh architecture/C compatibility, source/export/resource and documentation checks.
- Clean tree before creating the next-round branch. The user's explicit continuation authorizes the established parallel
  workflow in this checkout; no additional worktree or intermediate approval gate is required.
- Root owns all native configure/build/CTest/dist operations. Adapter implementers own disjoint C and Python files;
  characterization owns only the two new tests. Existing native/domain/object-model/resource files are protected.
- Read-only audits identified Scenario borrow/version/partial-apply contracts, Environment error/NaN/revision behavior,
  and Python static type/buffer coupling. The selected design retains those owners and avoids new accessor wrappers.
- Captured 3,692 protected file hashes and 1,338 SDK export names. Runtime baseline covers 229 World, two Scenario,
  11 VecEnv and four Recording descriptors, 32 module functions, all four type layouts/flags and the sole native export.
- Baseline C consumer passed 1/1 and Python characterization passed 6/6 before production edits. The final C fixture
  refinement checks borrowed string content instead of unspecified pointer identity; it passed again on A3.
  Both missing-owner extraction checks failed as expected before application. No baseline behavior fixture failed.
- Literal extraction checks pass for the two C owners and three Python owners. C World/control remainders and
  prior helpers/owners are unchanged. Python core remainder, shared fields/layouts, 14 registration initializers,
  four type specs, PyInit and eight shared helper bodies match A3. Runtime comparison passes all captured identities.
- Full headless build passed with FSIM_WARNINGS_AS_ERRORS=ON. Five focused C/Python compatibility/boundary gates
  passed in 83.69 seconds; three architecture gates passed in 2.32 seconds. Independent complete-diff review found
  no actionable regression. Existing native Scenario tests retain partial-application coverage; the new public C
  fixture covers success, borrowing/versioning, truncated output and principal refusal.
- Full applicable functional suite passed 549/549 in 325.02 seconds, including 18 fleet cases and both package
  gates. Together with the three architecture checks this covers 552 distinct CTest cases. Inventory comparison
  retains all 591 A3 cases and adds only world_adapter_compatibility, for 592 registered cases including Hangar.
- Distribution refreshed. All 1,230 runtime manifest entries and tested installed-consumer SDK/JSBSim DLL bytes
  match dist; staged/distributed Python native and object-model files match. Three binding compatibility suites
  passed 19/19 against dist. All 1,338 SDK exports, the sole Python export and 3,692 protected file hashes match A3.
- Acceptance reached on codex/architecture-world-adapters; A4 remains uncommitted. Architecture/development/status
  and navigation documents describe the retained runtime owners and remaining queue. No simulation/domain, public
  object-model, generated-resource or dependency work was added. Hangar generation, renderer/GPU, render SDK
  installation, CPack ZIP and remote CI were not exercised.
