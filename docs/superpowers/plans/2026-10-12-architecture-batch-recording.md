# Architecture A5 Implementation Plan

> **For agentic workers:** Use the established parallel adapter workflow with disjoint ownership, root-owned
> integration/builds and independent review. Apply verification-before-completion to every acceptance claim.

**Goal:** Separate VecEnv and Recording C/Python adapters while preserving their observable contracts.

**Architecture:** Literal source extraction with private handle/layout headers. Keep C++ wrappers and Python runtime
type/buffer/borrowed-World ownership in their original core owners.

**Tech Stack:** C++17, C11, CPython 3.11 stable ABI, CMake 3.25+, Windows UCRT64 GCC/Ninja.

**Spec:** [A5 design](../specs/2026-10-12-architecture-batch-recording-design.md).

## Constraints

- Baseline e708b87. Initially stop before commit; the user subsequently
  requested commit and push after acceptance.
- Public API/ABI, principal checks, native behavior, resources and existing object models/tests stay unchanged.
- Do not replace VecEnv's void-return guard with the int-return shared guard or normalize LastError semantics.
- Keep C++ VecEnv bodies, core type ownership, all registration and the three coupled Python callbacks literal.
- Files use English. Existing tools/caches D:/ENV, compilation temporary directories D:/ENV/tmp.

## Review focus

- Invalid VecEnv options retain the caller's out pointer; validated creation/load failures may clear it.
- Direct range/buffer/null calls retain LastError while guarded reset/step/create success clears it.
- Borrowed World caching/no-op destruction and Python owner retention preserve the original handle lifetime.
- VecEnv result pointers stay stable; float64 scratch narrowing and failed action buffers retain cleanup/order.
- Recording event strings/sample arrays stay borrowed; empty/out-of-range results and owner-retaining sample buffers
  preserve their representation. Type identities, descriptor call flags and static runtime ownership remain core.

## Task 1: characterization and baseline

**Files:** New tests/test_batch_recording_adapter_compatibility.cpp and
python/tests/test_batch_recording_binding_compatibility.py; root registers the standalone C target.

- [x] Add focused public C/Python tests using existing contracts/fixtures; keep existing tests frozen.
- [x] Run them against A4 before production extraction; retain any fixture failure/correction evidence.
- [x] Capture protected files, SDK/Python exports, type layouts/descriptors/functions and CTest inventory.
- [x] Prepare literal extraction expectations and observe missing-owner failures.

## Task 2: C VecEnv/Recording extraction

**Files:** fsim.cpp, fsim_world_c.cpp, SDK CMake; new fsim_vecenv_c.cpp, fsim_recording_c.cpp and VecEnvHandles.h.

- [x] Move opaque VecEnv layout privately, plus 14 C definitions and three original local helpers to the VecEnv owner.
- [x] Preserve C++ Impl/methods/conversions, lastError, registry/log/version and original C linkage.
- [x] Move Recording layout/nine definitions; copy layout assertions and retain World recording/lifecycle guards.
- [x] Register sources and compare all owners/remainders/layouts/helpers and prior protected owners byte-for-byte.

## Task 3: Python VecEnv/Recording extraction

**Files:** _native.c and Python CMake; new _native_vecenv.c/.h and _native_recording.c/.h.

- [x] Move 11 VecEnv callbacks, fields and names_list; move five Recording callbacks, with matching private prototypes.
- [x] Share unchanged object layouts privately; retain core buffers/borrowed World/sample callbacks and runtime state.
- [x] Preserve all registration/type specs/PyInit, shared helpers and existing busy/GIL/buffer/reference operations.
- [x] Compare exact bodies/remainders/metadata and native descriptors/layouts/functions/exports against A4.

## Task 4: integration and acceptance

**Files:** Architecture/development/status/navigation documents and this plan; local evidence D:/ENV/tmp.

- [x] Build full headless preset with warnings as errors; run focused C/Python and architecture gates.
- [x] Independently review the complete adapter/test diff; resolve actionable findings.
- [x] Run full applicable functional suite with principal, determinism, lifecycle, fleet and package coverage.
- [x] Refresh dist; compare protected files/exports, consumer DLL bytes and runtime manifest; test distributed Python.
- [x] Verify English, exact endings, local links/anchors, PowerShell examples and exact diff scope.
- [x] Record A4 commit, A5 acceptance, limits and remaining queue; retain the initial review stop before commit.

## Progress and decisions

- A4 committed as e708b87 after fresh literal source/runtime, scope/export/distribution, documentation and focused
  architecture/C checks. The clean checkout is reused on the new A5 branch; no push or dependency installation.
- Read-only C/Python audits support the selected minimal ownership split. Private handle/layout sharing avoids moving
  C++ wrappers or adding runtime accessors. Three Python callbacks retain their static Buffer/borrowed-World coupling.
- Root owns all native configure/build/CTest/dist and commits. C/Python workers own disjoint source sets; a third
  worker owns only the two new characterization files. Native domains, public files, resources and prior owners freeze.
- Captured 3,701 protected file hashes and 1,338 SDK export names. Native runtime baseline includes 246 descriptors,
  32 module functions, four type layouts/flags and the sole PyInit__native export; CTest inventory is captured before
  adding the new standalone consumer.
- Public C characterization passed 1/1 in 0.30 seconds against unchanged A4. Python passed 6/6 in 0.206 seconds.
  The first Python fixture incorrectly used World's nine pointer words for VecEnv; the actual A4 VecEnv layout has
  eight. Its failure log is retained as D:/ENV/tmp/a5-binding-baseline-first.log, followed by corrected/final logs.
  Local runs set TMP/TEMP to D:/ENV/tmp; committed tests discover the standard temporary directory for portability.
- Both missing-owner verification checks failed as expected before production extraction. C expectations cover
  six source/CMake files and 48 frozen public/prior support hashes; Python expectations cover six files, nine frozen
  owners/support headers, exact metadata/layouts/remainders, all registration and the three retained callbacks.
- Both extractions applied. C expectations match all six files, preserving C++ wrappers and three local helpers;
  all 48 captured SDK/header hashes match. Python exact comparisons preserve moved bodies/layouts/fields, the core
  remainder, three retained callbacks, 14 registration tables, four type specs, eight helpers and nine prior sources.
- C tail removal exposed two extra EOF newlines in World. Only those bytes were removed before rebuilding;
  original failure is D:/ENV/tmp/architecture-a5-formatting-failure.log. Original baseline/expected metadata remains
  available and the extraction verifier explicitly permits only this EOF normalization. Diff formatting now passes.
- Full headless build passed with warnings as errors. Runtime comparison retains all 246 descriptors, 32 module
  functions, four type layouts/flags and PyInit__native. SDK export comparison and all 3,701 protected hashes pass.
  Inventory comparison retains all 592 A4 cases and adds only the batch_recording_adapter_compatibility gate.
- Three architecture gates passed in 2.41 seconds. Complete independent adapter/test review found no actionable
  regression; it independently checked expected/frozen source hashes, linkage/layouts, ownership, cleanup/order
  and the retained core callbacks/registration. All six focused C/Python gates passed in 83.51 seconds.
- Full applicable functional suite passed 550/550 in 324.74 seconds, including 18 fleet cases and both package gates.
  Together with three architecture checks this covers 553 distinct CTest cases. All 593 registered names match the
  A4 inventory plus the new standalone compatibility consumer, including 40 unchanged Hangar cases.
- Distribution refreshed. All 1,230 runtime manifest entries and tested installed-consumer fsim/JSBSim DLL bytes
  match dist; native and public Python wrapper bytes match staging, including VecEnv/Recording. Four binding
  compatibility suites passed 25/25 against distributed Python. All 1,338 SDK exports and 3,701 protected hashes match.
- Acceptance reached for A5, which was initially left uncommitted. English architecture/development,
  navigation and status describe current owners, acceptance and the remaining queue. No dependency installation,
  resource generation or new simulation/domain feature was added. Hangar generation, renderer/GPU, render SDK
  installation, CPack ZIP and remote CI were not exercised.
- At the user's subsequent request, A5 was committed as 2efea05 after fresh exact-source/runtime, protected-file,
  export, distribution and documentation comparisons passed. Eight focused ABI, boundary, adapter and architecture
  gates passed again before commit. This delivery does not change the validated native source or simulation behavior.
