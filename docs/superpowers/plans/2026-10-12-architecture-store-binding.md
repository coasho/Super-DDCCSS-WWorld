# Architecture A6 Store Binding Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development for the disjoint characterization and source
> tasks; root owns native builds, integration, evidence and the final independent review.

**Goal:** Give World's carried-store flight bindings a private state owner while preserving observable behavior.

**Architecture:** Entry embeds StoreFlightBinding; explicit Host/model inputs keep borrowed property lifetime clear.
World retains lifecycle, traversal, permissions, loadTick, publication and release/damage transactions.

**Tech Stack:** C++17, CMake 3.25+, Windows x64 UCRT64 GCC/Ninja and existing CPython.

**Spec:** [A6 design](../specs/2026-10-12-architecture-store-binding-design.md).

## Global constraints and review focus

- Baseline 39be3c1; branch codex/architecture-world-store-binding. Initial review preceded user-authorized delivery.
- No public API/ABI, permission, resource, flight-model, Mission rule, Python or existing-test changes.
- Tools/caches D:/ENV; TMP/TEMP/TORCH_EXTENSIONS_DIR D:/ENV/tmp. Files English, conversation Chinese.
- Preserve loaded reset, same-aircraft reused model handles and the separate drag/mass revisions.
- Preserve first-load stamp arithmetic, non-finite early return, rejected-load side effects and tick wrap semantics.
- Preserve fuel dt/threshold/draw/refuel order, drag summation and control-host publication cadence.
- Never add reset invalidation, retained model pointers, new locking/allocation phases or model behavior fixes.

## Task 1: characterize and capture the baseline

**Files:** New tests/test_store_binding.cpp; tests/CMakeLists.txt; local baseline/evidence under D:/ENV/tmp.

- [x] Add focused production-path tests for loaded reset, loaded-history slot reuse and deterministic sequences.
- [x] Register cases through existing Catch infrastructure; build/run them against unchanged A5 production source.
- [x] Save explicit field-bit replay for sequential load/unload/refusal, tick boundaries, external fuel, asymmetric
      release, reset and reuse; include workers/frameSkip/drag variants without raw-structure padding.
- [x] Capture protected files, original World/CMake bytes, SDK/Python exports/runtime metadata and CTest inventory.
- [x] Prepare baseline-derived extraction expectations and observe the missing-owner verification failure.

## Task 2: extract the private state owner

**Files:** New src/session/StoreFlightBinding.h/.cpp; World.h/.cpp; session/CMakeLists.txt.

**Interface:** The signatures and ownership in the spec are binding; the characterization file consumes only
existing World/Host/model interfaces. Source workers do not configure/build, modify tests/docs or commit.

- [x] Move the exact station/mass/stamp/drag/load-revision state with unchanged defaults into StoreFlightBinding.
- [x] Extract station discovery, synchronization, weight-room math, fuel arithmetic and drag accumulation.
- [x] Replace only the matching World field/operation seams; retain outer guards, loops and publication order.
- [x] Compare baseline-derived bodies/remainders and frozen native domains, adapters, public files and resources.
- [x] Build warnings-as-errors headless; repeat characterization and compare replay bytes exactly.

## Task 3: review and integration acceptance

**Files:** Architecture/development/navigation/status references and this plan; local evidence D:/ENV/tmp.

- [x] Independently review the full source/test diff, including every review-focus condition; resolve findings.
- [x] Run new store-binding characterization, ABI/principal/binding and three architecture gates.
- [x] Run full applicable functional suite, including fleet/determinism/lifecycle/Python and installed consumers.
- [x] Refresh dist and compare tested consumer DLL/data and staged Python provenance; verify protected hashes,
      unchanged exports/runtime identities, existing CTest inventory and exact diff scope.
- [x] Update English ownership/status references with actual acceptance and limits; validate links/examples/format.

## Progress and decisions

- A5 is committed as 2efea05, delivery documentation as 39be3c1; both are pushed on the A5 branch.
- Reuse the clean task-owned checkout/build on a new A6 branch. Read-only parallel audits confirm the exact state
  cluster and update ordering. No dependency installation or worktree duplication is required.
- Ruling: preserve existing loaded-reset behavior; this ownership refactor does not authorize its normalization.
- Ruling: use baseline characterization plus missing-owner source verification for the refactor's red/green evidence;
  behavioral characterization is expected to pass on the original source rather than demand new behavior.
- Captured 3,720 protected hashes, 1,338 SDK exports, 246 native Python descriptors, 32 module functions and 593 CTest
  cases. Original source and runtime evidence are in D:/ENV/tmp/a6-baseline. Source expectations/remainders and
  missing-owner RED evidence are in D:/ENV/tmp/a6-owner; original World.cpp CRLF and other source LF remain distinct.
- First characterization run had 18 assertions using an incorrect zero initial Mission Host revision. Host::bind
  advances it to one; tests now compare the original/fresh Host revisions. No production change was made to pass.
  The first failure log is architecture-a6-characterization-baseline.log; corrected baseline passed all three cases
  and 2,655 assertions. Its 67,553-line field-bit transcript captures all eight worker/frameSkip/drag variants.
- Exact extraction GREEN matches all five owned files and complete World/CMake remainders; WorldDamage remains
  unchanged. The full headless build passes with warnings as errors. Post-extraction characterization passes the
  same 2,655 assertions; all 2,641,139 transcript bytes match baseline (SHA-256
  2d6e061a2dd7b7b84f0ec02ba32d83ff6b1e6ec9a80b99204861d731b7f32df5).
- Eight ABI/principal/adapter/architecture gates pass in 7.41 seconds. Preflight retains all protected hashes,
  exports and Python identities; all 593 existing CTest names remain, plus the three store-binding cases (596 total).
- Independent read-only review found no actionable Critical, Important or Minor issues. It compared actual baseline
  source, lifetime/reset/reuse semantics, revision and force ordering, permission boundaries and scalar replay
  coverage. The report is D:/ENV/tmp/architecture-a6-independent-review.md; runtime acceptance remains root-owned.
- The full applicable functional suite passed 553/553 in 325.59 seconds, including all 18 fleet cases, lifecycle,
  native engagement determinism, Python, launcher and both package/installed-consumer gates. Together with the three
  architecture checks, this validates 556 distinct CTest cases; the 40 unchanged Hangar cases were not rerun.
- Distribution refreshed successfully. All 1,230 runtime manifest entries and SDK/JSBSim DLL bytes match the tested
  installed consumer; staged/distributed Python native and wrapper bytes match. All four distributed binding
  compatibility suites passed 25/25 in 1.195 seconds using the actual dist module.
- Final scope remains exactly 13 files; all 3,720 protected hashes, 1,338 SDK export names and Python runtime
  identities match A5. Source/test bytes remain those independently reviewed and tested. English ownership/status
  references, six Markdown files, 175 local links, five anchors, five PowerShell blocks and diff formatting pass.
  A6 was initially left uncommitted for review; the user subsequently requested delivery and fresh validation.
  No renderer/GPU, render SDK installation, CPack ZIP or remote CI acceptance is claimed.
