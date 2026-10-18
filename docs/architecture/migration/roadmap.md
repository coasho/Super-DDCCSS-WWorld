# Staged architecture migration roadmap

## 1. Execution contract

The architecture package defines the destination; it does not authorize overlapping an active feature's shared paths. At the original planning baseline, source relocation was pending and M8 generic had just been committed at d344e2b. The dated package records below supersede that initial state: ARCH-01 through ARCH-09 are accepted and ARCH-10 retains the stated final-program gates. Before each new package, release its shared window and record the current HEAD, source hashes and pending gates. Full combat/Viewer feature completion is separate from safe source classification.

Work on main only. Preserve unrelated edits. Do not create branches/worktrees, clean/stash/reset another owner's files or stage unrelated hunks. Every package records its exact path/symbol allowlist, owner, prerequisites, current HEAD/diff, fixtures, affected targets, compatibility gates and rollback procedure.

Each package is a complete vertical change: implementation, public adapters, build/install paths, examples, tests, policy and documentation together. Current Claude assignments remain with Claude. An architecture review does not transfer their feature ownership.

No native configure/build/test jobs compete. No GUI scenario competes with another. Raw background output goes directly to one log/job without a trailing buffered filter. Toolchain/cache/temporary compilation paths stay under D:/ENV and D:/ENV/tmp.

## 2. Work package order

~~~mermaid
flowchart TD
  A0[ARCH-00 current audit and plans] --> A1[ARCH-01 ownership and policy baseline]
  M8[Review M8 generic handoff and release window] --> A1
  A1 --> A2[ARCH-02 neutral contracts]
  A2 --> A3[ARCH-03 foundation and leaf moves]
  A3 --> A4[ARCH-04 simulation runtime and facade]
  A4 --> A5[ARCH-05 training extraction]
  A3 --> A6[ARCH-06 visualization separation]
  A4 --> A6
  A5 --> A7[ARCH-07 SDK and Python adapters]
  A6 --> A7
  A7 --> A8[ARCH-08 applications tests and packaging]
  A8 --> A9[ARCH-09 documentation relocation]
  A9 --> A10[ARCH-10 final structural acceptance]
~~~

This is dependency order, not permission for simultaneous writes. ARCH-05 and ARCH-06 can have independent read-only analysis, but shared contracts/SDK/CMake/transport files retain one active owner.

## 3. Detailed packages

### ARCH-00 — current-state evidence and target plan

Status: delivered by this planning request.

Allowlist: docs/architecture/**; derived audit helper under build/architecture-planning-20261016.

Output: current native/Python evidence, target boundaries and layouts, all three module plans, path/document registers, target DAG policy and acceptance/checking instructions.

Gate: document links/catalogs and proposed graphs validate; current native graph findings are reported accurately. No code relocation or runtime acceptance is claimed.

### ARCH-01 — freeze source ownership and enforce the destination policy

Status: independently accepted on main at `7625529` (2026-10-16); see the [ARCH-01 record](../validation/arch-01-ownership-baseline.md) and [independent review](../validation/arch-01-review-2026-10-16.md). This package moved no domain source; later package progress is recorded below.

Write scope: tools/architecture checker/policy/negative fixtures, relevant CMake architecture registration, docs/architecture validation records. Do not move domain source in this package.

Tasks:
1. Record the exact current native target graph, include owners, public headers, Python imports, package outputs and baseline hashes.
2. Extend ownership checks to the future nested directories without allowing unowned legacy code.
3. Add negative fixtures for each forbidden reverse edge, deferred Python backedge, graphics leakage, duplicate source ownership and SDK extraction cycle.
4. Distinguish contracts, implementations, final assembly and external application consumers.
5. Compare actual targets against policy; an allowed edge is not evidence that a dependency exists.
6. Define how the source parser and configured CMake File API/link graph will be reconciled after actual moves.

Gate: current accepted tree remains valid, mutations fail for their intended reasons, unsupported declarations fail explicitly. Do not run a future-only policy against unmigrated directories and call its expected path failures implementation defects.

Rollback: revert only this package's reviewed changes, preserving feature and wording edits. Retain its evidence.

### ARCH-02 — extract neutral values and transport contracts

Status: independently accepted on main at `dc09e8f` (2026-10-16); see the [ARCH-02 record](../validation/arch-02-neutral-contracts.md) and [independent review](../validation/arch-02-review-2026-10-16.md). HC-00 and HC-01 are closed; the HC-10 transport follow-up is independently accepted at `565ef3e`. ARCH-03 has not started.

Write scope: include/fsim value/header ownership, internal scenario/options/state contracts, src/ipc layout headers, relevant source includes/tests and ownership policy.

Tasks:
1. Inventory which public headers are neutral values and which are simulation/training/visualization facades.
2. Separate native scenario/options values from legacy VecEnv/public compatibility aggregates.
3. Place shared stream/recording definitions below producer and consumer; eliminate implementation-only sim headers from transport.
4. Preserve existing public header names, type identity, field order/alignment, enum numbers and plugin vtables.
5. Retain the exact existing World native facade exception; do not add private namespaces to new public APIs.
6. Add layout/old-recording and negative-dependency regressions before changing includes.

Gate: old headers/binaries and serialized formats remain compatible; no contract target depends on domain/foundation implementation.

Rollback: restore original definition owner and include path in one reviewed package; never keep duplicate definitions as a fallback.

### ARCH-03 — migrate named foundation and domain leaves

Status: independently accepted at `ad12a44` (2026-10-16). Relocation at `30bc5ef`, ten-target include narrowing at `e7ee67e` and the ancestor-root correction at `ad12a44` complete this package; HC-12 is closed. See the [ARCH-03 record](../validation/arch-03-foundation-and-leaves.md) and [final acceptance](../validation/arch-03-review-2026-10-16.md#6-final-hc-12-acceptance-at-ad12a44). HC-11, HC-13 and HC-14 remain separate carryovers. ARCH-04 has not started.

Write scope: current platform/core/io/ipc mechanisms, sim/control/effects/ms/comm leaf CMake/includes and corresponding tests/install/policy paths.

Tasks:
1. Move platform/core/io to foundation subdirectories; move shared transport mechanisms to foundation/transport.
2. Move sim/control/effects/ms/comm leaves to their simulation destinations.
3. Keep physical effects separate from rendered VFX.
4. Update literal includes, CMake source lists, tests, generators and documentation references with each move.
5. Retain target names/aliases initially; narrow compiler include paths one target at a time.
6. Keep generic network/OS transport separate from modeled DMS.

Gate: current native/architecture regressions and headless build remain green; no model, RNG, timestep or default behavior changes are mixed in.

Rollback: move the exact package back with its references; no recursive relocation of an unchecked computed path.

### ARCH-04 — simulation runtime, services and lower facade

Status: independently accepted at `6e79873` (2026-10-16); see the [ARCH-04 record](../validation/arch-04-simulation-runtime.md) and [independent review](../validation/arch-04-review-2026-10-16.md). HC-15 explicitly retains the thin `applyScenario` compatibility wrapper in the final assembly and assigns its relocation to ARCH-07; P022 is amended rather than counted as already moved. Native scenario execution is in the simulation runtime. ARCH-05 has not started.

Write scope: session World/services/store-binding/DMS/construction mapping; selected SDK World/Agent/scenario implementations; private target registration and affected tests/adapters.

Tasks:
1. Split logical state owners first, then relocate runtime/world/services/stores and modeled DMS.
   Give modeled DMS its own target consuming generic comm and MS RF propagation; keep the network target's source root disjoint.
2. Give construction mapping an API boundary owner; runtime consumes native options, not a training/public aggregate. Retain the episode's mapper call through an explicit env-to-simulation_api dependency and verify scenario/options/seed parity.
3. Create the private lower simulation_api target containing existing facade implementations needed by mission code.
4. Remove its dependency on the final SDK assembly; keep exactly one definition of exported World/Agent methods.
5. Preserve registration/WHOLE_ARCHIVE and per-target FSIM_BUILDING behavior.
6. Characterize authority, C2/DMS, store/flight and output-only behavior before extracting shared helpers.

Gate: simulation_api links without training or graphics; old C++/C/Python consumers and native output parity pass.

Rollback: restore original facade assembly only for this package; verify exports after rollback.

### ARCH-05 — training environment and mission extraction

Status: independently accepted at `58f9297` (2026-10-16); see the [ARCH-05 record](../validation/arch-05-training-extraction.md) and [independent review](../validation/arch-05-review-2026-10-16.md). The training targets are below the final SDK, and installed composition traces retain byte parity. Other carryovers remain open; ARCH-06 has not started.

Write scope: src/env, mission-specific SDK implementations, public mission/VecEnv adapters, training Python helpers, examples, workload tools and their tests/build/policy references.

Tasks:
1. Move existing VecEnv implementation separately from independent mission episode ownership.
2. Move config/gateway/batch/lifecycle implementation to training, consuming the lower simulation facade.
3. Retain EpisodeHold, all-slot preflight, final-document ownership and correct Busy retry.
4. Keep public mission wrappers and _native runtime type owners compatible.
5. Preserve actor/referee separation, non-consuming fanout, deduplication, native refusal meaning, seeds and costs.
6. Re-run the real installed M6 composition traces and preopened-child regressions.

Gate: no training-to-final-SDK-to-training cycle; current episode/batch/actor/lifetime/installed acceptance passes.

Rollback: restore the original target/assembly mapping while retaining one runtime/type owner. Do not substitute Vehicle.reset or a Python physics fallback.

### ARCH-06 — visualization core, effects and source separation

Status: independently accepted at `74efb49` (2026-10-16). The source split at `61830e4` and subsequent selector corrections complete this package; HC-13 is closed. See the [ARCH-06 record](../validation/arch-06-visualization.md) and [final acceptance](../validation/arch-06-review-2026-10-16.md#9-final-hc-13-acceptance-at-74efb49). HC-16's exact temporary exception is allowed through ARCH-08. HC-17 retains legacy demo behavior, accepts P042's early move and assigns remaining display/composition cleanup explicitly. HC-19/20 retain the selector's reported limits. Small image differences are not established rendering regressions. ARCH-07 has not started and awaits the next assignment.

Write scope: render/world/ui/vision, Flow loader boundary, Viewer source adapters and shared transport consumers; corresponding render tests/assets/install references.

Tasks:
1. Rename scene ownership from world to visualization/scene.
2. Split Flow/VFX from physical effects and retain the optional GPU target.
3. Make scene/vision rendering consume read-only frames/contracts.
4. Keep compatible World-handle and native Vision exports in adapters.
5. Separate live/replay viewing from demo simulation composition.
6. Record any temporary executable compatibility adapter with an explicit removal gate.

Gate: headless graph/build remains GPU-free; render/Vision/Viewer builds and complete visual scenarios have current evidence. Do not relabel diagnostic evidence as joint actor acceptance.

Rollback: restore source/target mapping and resource lookup without touching native model parameters.

### ARCH-07 — SDK assembly and Python import cleanup

Status: independently accepted at `a59b46f` (2026-10-17). The `ebd3a09` subset plus P056 completion and planned-phase
correction close HC-21 and HC-23; HC-22's explicit C-header include remains accepted. HC-24 retains planning/policy
graph reconciliation before ARCH-10. See the [record](../validation/arch-07-sdk-and-python.md) and
[final review](../validation/arch-07-review-2026-10-17.md#6-final-hc-21-and-hc-23-acceptance-at-a59b46f).
ARCH-08 has not started and awaits its next assignment.

Write scope: src/adapters/sdk, adapters/vision, Python source/staging/registration, public compatibility modules and corresponding tests/policy.

Tasks:
1. Leave final SDK assembly thin: exports, handles, error/principal conversion and product composition only.
   Under the accepted HC-15/P022 placement adjustment, move the thin `applyScenario` compatibility adapter to `src/adapters/sdk/cpp/Scenario.cpp`; preserve its delegation to native scenario execution, public signature and single definition. Do not add a simulation-to-training exception for its mixed compatibility header.
2. Keep public C++/C/Python names, native runtime type owners and stable ABI.
3. Move private Python implementations under _simulation/_training/_visualization/_adapters.
4. Remove both deferred import cycles; shared values/backend factories live below their consumers.
5. Remove VecEnv from visualization core; optional training-image composition lives outside both cores.
   HC-17.4's optional training/Vision composition remains in this package; compatible BatchCameras in the outer adapter is not completion of that work.
6. Verify optional ML/GPU imports, pickling/qualified names, method signatures, buffers and GIL/lifetime behavior.

Gate: native, Python owner/import and package graphs are acyclic; core imports work without ML/GPU packages; installed old consumers pass.

Rollback: restore public compatibility modules without retaining parallel type registries or state stores.

### ARCH-08 — applications, tests, examples and packaging

Status: independently accepted at `dd7dc9f` (2026-10-17). HC-32 and complete HC-16 publisher lifecycle are closed; HC-14/17.2 and HC-28 to HC-31 stay accepted. The original 30-cycle native probe now has stable handles (89/89); exact headless gates pass 3/3 and Release publisher cases 4/4. HC-25 to HC-27 and required final-program gates retain their assigned follow-ups. See the [ARCH-08 record](../validation/arch-08-applications-tests-and-packaging.md) and [final acceptance](../validation/arch-08-review-2026-10-17.md#8-final-hc-32-and-arch-08-acceptance-at-dd7dc9f). No full suite was repeated. ARCH-09 has not started and awaits its next assignment.

Write scope: applications, test/example/tool source locations, launchers and CMake staging/install/distribution definitions; generators only for path references.

Tasks:
1. Classify tests into contracts/simulation/training/visualization/compatibility/integration while retaining CTest names initially.
2. Classify examples and tools; public examples must consume installed SDK interfaces.
3. Preserve launcher/preset behavior, DLL names, package layouts and asset resolution.
4. Update copied consumer scripts, fixture paths, recording fixtures and command-line source adapters.
5. Check installed source/build fallback rejection and every required resource's provenance.
6. Ensure no removed path remains a hidden runtime fallback.
7. Remove HC-16's in-process Viewer/demo link through a separate publisher using the existing read-only stream, preserving legacy demo flight behavior. Remove HC-17.2's Viewer control-level-name dependency through a neutral presentation contract, preserve displayed labels and add the resulting Viewer include-boundary check.

Gate: package/distribution/launcher, current full regression and installed consumer matrices pass; graphics acceptance remains separately visible.

### ARCH-09 — relocate existing architecture authority

Status: independently accepted at main `6bb79cc` plus the inspected uncommitted planning/index package (2026-10-17), following relocation `e3e95bb` and redirects `a3f327d`. HC-33 is closed: D23-D25 register and directly index ADR-26, ADR-27 and the packaging plan with category/owner/status; all three bodies are unchanged. ADR-27 remains in force for rotorcraft. D07-D09 retain the approved M8 deferral (HC-34). HC-35 retains the pending human/instruction-owner disposition, and HC-36 tracks later comment-reference maintenance. See the [ARCH-09 record](../validation/arch-09-architecture-authority.md) and [final independent review](../validation/arch-09-review-2026-10-17.md#7-final-hc-33-and-arch-09-acceptance-at-6bb79cc). Earlier evidence limits and final-program gates remain; ARCH-10 has not started.

Write scope: only the registered documents and their inbound links, including development/README references and project instructions when their owner releases them.

Tasks:
1. Apply the document register in dependency order after active feature documents are stable.
2. Move one canonical body; update inbound links and retain compatibility redirects where needed.
3. Preserve historical dates, achieved evidence and pending gates.
4. Keep SDK API reference under docs/sdk; use links rather than duplicate editable contracts.
5. Eliminate new architecture plans outside docs/architecture.
6. Validate all local links and index reachability.

Gate: every authoritative architecture document has a category/owner/status and is reachable from the index; no stale dual authority.

### ARCH-10 — final three-domain acceptance

HC-45 (2026-10-17): the [review and correction acceptances](../validation/hc-45-review-2026-10-17.md) close HC-48/49/50 at `1e9433b` and HC-47 check logic at `d6bfb54`. Equivalence/causality remain unresolved and performance optimization is deferred. V2 helpers need live-path validation before future automated reuse; no GUI/performance/full run requested now. Task 4 remains open.

HC-07 (2026-10-17): the [review and interaction acceptance](../validation/hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892) accept the diagnostic subset at `465b7af` and sampled window-input mouse/replay/resize cases at `2a09756`; HC-51 closed at `5c80892`. HC-45, HC-06, HC-02, human operation and uncaptured panel/camera/option variants keep task 4 open. HC-52 is nonblocking reusable-helper completeness work. No GUI, benchmark, build or full suite was repeated for review.

HC-11 (2026-10-17, independently accepted at evidence `66dc43d`): each required full CTest inventory ran once on the frozen effective source at `f1bb879`, 717/717 headless and 780/780 Release passed. See the [record](../validation/hc-11-final-regression.md) and [independent review](../validation/hc-11-review-2026-10-17.md). Expected internal headless Vision skips and fingerprint scope are qualified; HC-44 is nonblocking future helper work. Task 4's GUI/manual/performance/joint scenarios (HC-07) remain pending.

Status: structural/correction subset accepted at `f1bb879`; task 3's HC-11 final regression is now accepted at `66dc43d`. HC-38 to HC-43 and HC-20 remain closed. Tasks 1/2/5 retain their accepted source/configured/planning evidence, dated source manifest and explicit documentation overlays. HC-04/19/24/25 and HC-37 stay accepted. HC-43 preserves native CMake 3.25 support with the strict configured gate from 4.2. Task 4's HC-07 evidence remains pending; complete ARCH-10/final-program acceptance is not claimed. See the [record](../validation/arch-10-final-acceptance.md). HC-44 is future fingerprint work, not a reason to repeat these accepted suites. No additional run is requested by this review.

Write scope: evidence/summary/policy only, plus narrowly scoped fixes required by failed gates.

Tasks:
1. Capture the final HEAD/diff and full source/target/import/package inventories.
2. Prove no unknown source owner, forbidden edge or SCC in all required graphs.
3. Run current native headless/full/compatibility/installed gates in the exclusive slot.
4. Build optional render/Vision and complete required fullscreen scenarios in the GUI slot.
5. Record source/DLL/data/package/GPU provenance and pending functional model goals.
6. Release slots and stop for independent acceptance.

Gate: all required structural and compatibility cases achieved; selected visual evidence current; remaining MS/model/upper-layer work explicitly distinguished.

## 4. Safe package handoff format

A handoff records:
- Starting and ending main HEAD, reviewed diff/commit and exact allowlist.
- Changed source owners, symbols, target/include/import edges and public interfaces.
- Target/include/source/package path changes and remaining compatibility shims.
- Baseline and current exports/layout/type identities.
- Actual discovered nonzero test counts, results, timeouts and raw logs.
- Native/GUI slot release; installation/asset provenance.
- Achieved, failed, unverified and pending gates.
- Every self-reported problem, limitation and deferred task, reconciled with the [handoff carryover register](../validation/handoff-carryover-register.md); include its ID, owner/package, closure condition and current disposition even when it is pre-existing or outside this package.
- Next permitted package and its dependencies.

Commit only reviewed feature-owned files; push only when authorized. Do not mark a package complete while its logs still show a running repetition or while failed final gates lack a later clean result.

Independent acceptance reviews the complete handoff, not only the requested
code changes or final test totals. Read all supplied text, account for new
self-reported items and prior open carryovers, and record them before issuing
the verdict. A deferred item stays open with its correction destination;
successful phase acceptance does not delete it.

## 5. Stop conditions

Stop implementation at a concrete ownership conflict, missing frozen interface, unclassified dependency, changed numerical behavior, ABI/layout/type mismatch or unexplained test failure. Preserve the reviewable work and resolve that specific prerequisite; do not silently widen scope.

The planning request itself ends with this package and its validation. Formal source migration starts in its own exclusive window after the M8 generic handoff review. Physical model upgrades and production MA/DRL remain separate assignments.
