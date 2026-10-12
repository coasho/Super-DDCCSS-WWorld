# Architecture A2 Implementation Plan

**Goal:** Retire MA private maintenance safely and give C/Python DMS/C2 adapters focused source owners.

**Architecture:** Retain MA public history and map/controller ownership. Extract private consent bookkeeping, introduce explicit retirement and move adapter bodies into separately compiled domain files without altering registrations.

**Tech Stack:** C++17, C11, CPython 3.11 stable ABI, CMake 3.25+, Windows UCRT64 GCC.

**Spec:** [A2 design](../specs/2026-10-12-architecture-lifecycle-design.md).

## Global constraints

- Baseline `6652584`; branch `codex/architecture-lifecycle`. Leave A2 reviewable without committing it until requested.
- Preserve API/ABI, permissions, error/struct-size semantics, borrowed storage, controller IDs and step/event ordering.
- Retain public headers, Python object-model files, aircraft/store/generated and third-party bytes.
- Tools/caches stay under `D:/ENV`; compilation temporary directories use `D:/ENV/tmp`.
- Files English; conversation Chinese. No runtime framework or new simulator feature.

## Review focus

- Removed entity history remains readable, including borrowed RBAC names and task answer semantics.
- Unanswered ended strikes remain distinguishable from unauthorized replies; multiple approvers retain first-answer behavior.
- Reset preserves MA and slot reuse never inherits another entity's records.
- Native method registration, tuple order, FASTCALL, Error identity, GIL/busy checks and buffer lifetimes remain unchanged.
- Splitting source files must not omit principal scanning or add DLL/Python exports.

## Task 1: MA private state lifecycle

**Files:** `src/session/MaCore.{h,cpp}`, new `src/session/MaConsentLedger.h`, `src/session/World.cpp`, `src/session/CMakeLists.txt`, new `tests/test_ma_lifecycle.cpp`, selected additions to `tests/test_agents.cpp`.

**Interfaces:** Produce `void MaCore::retireEntity(std::uint32_t) noexcept`; the internal ledger owns strike/request/approver records with lookup, completion and storage-release operations. Consume existing World/DMS removal and MA step phases.

- [x] Add pre-refactor public characterization cases for retained history, removal/reset timing, first answers and late unanswered replies; run them on the baseline.
- [x] Add ledger lifecycle regression tests and observe failure before its implementation.
- [x] Implement the ledger and retirement call in both successful removal paths; retain map entries/controllers and existing pending transitions.
- [x] Verify storage reclamation and all lifecycle/agent/C2 cases. Keep infinite/NaN records on their existing path.

## Task 2: C links adapter

**Files:** new `src/sdk/fsim_links_c.cpp`, `src/sdk/fsim_world_c.cpp`, `src/sdk/CMakeLists.txt`; declaration annotation in `src/sdk/CAbiSupport.h` only if required.

**Interfaces:** The existing 70 `FSIM_API` definitions and three local conversions remain unchanged and link into `fsim`. No public declaration or principal inventory changes.

- [x] Capture the contiguous 659-line source block, signatures/bodies and DLL export baseline.
- [x] Move the block literally, preserve its C linkage and register the new source.
- [x] Compare bodies and original-file remainder; verify boundary scanning and C ABI/agent acceptance after integration build.

## Task 3: Python native links adapter

**Files:** new `python/src/_native_dms.c` and `_native_world.h`, `python/src/_native.c`, `python/src/fsim_py.h`, `python/CMakeLists.txt`, new `python/tests/test_binding_compatibility.py`.

**Interfaces:** Keep existing WorldObject fields/order, WORLD_IDLE and native registration; private support provides existing conversions and the original Error helper. The 47 World handlers and two local converters move with unchanged bodies.

- [x] Add and run baseline runtime characterization of native descriptors, public value identity/pickle and argument/error behavior; capture 49 bodies and registration tables.
- [x] Extract the native block into a separately compiled C file and a minimal private support header.
- [x] Verify exact body/registration comparisons, unchanged native exports and Python agent/isolation/thread/buffer acceptance after integration build.

## Task 4: Integration and evidence

**Files:** root-owned `tests/CMakeLists.txt`, architecture/development/status/navigation documentation and this plan.

- [x] Register new lifecycle cases, build the headless preset with warnings as errors and run targeted acceptance.
- [x] Independently review the complete diff and resolve behavioral or coverage findings with regression evidence.
- [x] Run architecture gates and the complete applicable functional suite, including installed consumers.
- [x] Refresh dist; compare manifest and DLL bytes; verify API/source/data scope, documentation links and English content.
- [x] Update status with commit baseline, exact commands/counts/logs, limitations and remaining architecture queue.

## Progress

- A1 committed as `6652584` after fresh architecture, export/manifest and documentation checks; clean tree before branching.
- Three read-only audits established MA compatibility constraints and exact C/Python extraction seams.
- Implementation uses three disjoint file ownership tasks; root owns test registration, integration verification and final documentation.
- Completed all three bounded tasks. Independent review found no actionable issues in the integrated diff.
- Fresh acceptance passed: 547 functional CTest cases in 325.12 seconds and three architecture gates in 2.51 seconds, covering 550 distinct cases. Targeted MA acceptance passed six internal cases with 4,134 assertions and three public cases with 105 assertions; seven Python compatibility tests passed on the baseline, rebuilt adapter and final staged module.
- C extraction is literal, and all 49 Python bodies plus shared helpers/registrations match their baseline. All 1,338 SDK exports and the sole Python export match. All 3,204 protected file hashes and 1,230 distribution runtime hashes match; distributed DLLs match the independently tested installed consumer.
- [Status and evidence](../../status.md#architecture-a2-verification-2026-10-12) records commands, logs, pre-refactor fixture corrections, review and skipped configurations. Whole-State public history and unanswered ended-strike requests remain retained by design; map scanning and controller ownership are unchanged.
- Stopping point reached after acceptance on `codex/architecture-lifecycle`. A2 was subsequently committed as `34e5516` at the user's request before A3; no push or new feature milestone.
