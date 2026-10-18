# Project Instructions

Repository: Super-DDCCSS-WWorld.
Branch: main only.
Implementation baseline: baf476f841970f1f3c90b8de0bf630e8a5478738.

## Context and filesystem boundaries

Use these project instructions, tracked project documentation and explicitly relevant project source as persistent context. The accepted [design](docs/superpowers/specs/2026-10-12-ma-system-training-readiness-design.md), [Claude plan](docs/superpowers/plans/2026-10-12-ma-system-training-readiness.md) and [Claude work allocation](docs/superpowers/plans/2026-10-12-ma-system-training-work-allocation.md) govern MA training work. The linked plan and allocation describe Claude's assignments only.

Do not read, search, enumerate, traverse or modify any directory named .codex or anything inside it, at any depth. This restriction includes repository-local directories, user-profile locations and their memories, skills and configuration.

Do not inspect or modify other worktrees. Scope searches to explicit allowed project paths; avoid broad filesystem or repository enumeration that could enter an excluded directory. Exclude every .codex directory before a permitted recursive source search. Use the tracked project documents and this file instead of profile memory lookup.

Work on main. Do not create branches or worktrees. Verify the current branch before changing files and preserve unrelated working-tree changes. Stage only reviewed feature-owned files and inspect the staged diff.

Maintained code, documentation, tests, UI text, metadata and commit messages are English. All communication with Claude, including prompts and handoffs, is English.

## Feature ownership

Preserve the agreed vertical allocation. Each owner delivers a complete feature across native code, public interfaces, examples, tests, documentation and acceptance evidence; do not split ownership by implementation layer.

Claude owns complete generic features: generic contracts/schema; explicit MA-service/controller binding; noncombat reports available within Agent permissions; P2P and C2 task/control/package/lost-link behavior; independent mission episode lifecycle; the mission gateway/transport and actor isolation; composition examples; batch/training lifecycle, noncombat task criteria, referee-stream transport and communication costs; installed consumers, generic integration and compatibility checks.

For those features, Claude owns native C++, C++/C/Python interfaces, examples, tests, documentation and acceptance evidence together. Future generic integration and compatibility builds remain Claude work. Reviewing a handoff or running independent verification does not transfer feature ownership.

Implement only explicitly assigned feature packages during the active write window. The documented assignments do not expand an implementation window's scope. Project documentation records public contracts, assigned tasks and acceptance evidence rather than personal execution notes.

The reviewed Viewer baseline was delivered as a separate verified main commit and push: baf476f841970f1f3c90b8de0bf630e8a5478738. Generic M0, M1 and noncombat M2 have been delivered on main; the latest M1 follow-up is 2bad550. Progress reviewed on 2026-10-15 at main 6e8b4aa: M4 independently accepted at 54b7fb9; simulation/interface compatibility prerequisite accepted at 08e3afe; M5 core implemented in 6e8b4aa and awaiting independent acceptance. Full M5 extension handlers, first-package acceptance, M6-M8 and joint/Viewer acceptance remain pending. See docs/status.md for current status and dated evidence. Preserve the frozen generic M0 schemas, native mappings, lifetime/error and ownership contracts when adding downstream interfaces.

The first complete Claude package is the noncombat mission episode gateway: M1 + M4 + noncombat reports + minimal M5 core through installed public C++, C99 and Python. Only an interim gateway smoke check may precede full M4 fixture acceptance; complete M5 core and first-package acceptance require full M4. The supplied native-state reset fixture uses already accepted SDK behavior and need not wait for new policy implementation. M5 core is independent of M3; full M5 requires both core and combat gates. M6 noncombat composition consumes M5 core; full composition scenarios consume M5 full. M7 core consumes M6 noncombat and remains separate from combat outcome/reward gates; full M7 also requires full M6.

Claude owns the generic schema and facade. Integrate frozen extension definitions, handler registrations, native receipt/outcome contracts and fixtures at agreed seams with their owner. Preserve native semantics and authority. Missing inputs remain pending or fail explicitly; do not invent provisional behavior. Project allocation and implementation documents contain Claude assignments only.

## Architecture and compatibility

SDK is the sole simulation authority for flight, sensing, communication delivery, release, weapon guidance, fuzes, collisions and damage. Client code uses SDK interfaces for execution and handles presentation/interaction. Never duplicate simulation in a client.

The formal complex MA runtime belongs in the upper-layer project. Its actors, BT/planners, blackboards, internal services and arbitration are not an SDK production framework. SDK examples are minimal replacement/lifecycle demonstrations.

An SDK Agent is a permission-scoped endpoint. MA service, platform, component and DRL actor are distinct identities; their counts need not match. C2 services may exist without aircraft.

Internal components may share state available through their SDK permissions inside the same platform/system boundary. Cross-platform/C2 information passes through modeled sensors/links. A shared gateway does not expand authority or make another platform's blackboard available outside those modeled paths.

Use SDK terminology for these contracts: service identity binding, Agent permissions, callable-method lists, actor/referee data visibility, generation-checked handle lifetime, request validation and ABI compatibility. Process isolation means an actor process receives only its SDK proxy and permitted reports; operating-system file/process access is outside that interface contract. Describe the local pipe's HMAC connection check separately from modeled C2 roles and endpoint binding. Preserve established API identifiers, schema fields and native refusal semantics when correcting explanatory wording. The terminology reference is docs/sdk/agents.md, SDK access terminology.

Preserve existing public signatures, class/struct layouts, plugin vtables, enum numbers, status/error semantics, observation/action shapes, default permissions, behavior and ABI. Use additive versioned interfaces. Old-header binaries must continue to work. Preserve established native/Python runtime type owners and CPython stable-ABI conventions.

Keep flight VecEnv compatible. New mission episodes initially own one World each, including mutable task/judge, MaCore, DMS, guidance/damage state and cursors. Reconstruct the owned World for a full reset; do not substitute Vehicle.reset.

Existing Vehicle.reset preserves authority, tasks, overrides and C2 activity ownership. Do not change those semantics. Generation-checked episode reset is a separate contract.

A retained generation/closed control block protects child handles: reset rejects prior generations, and parent destruction closes children before releasing the World, so access fails before dereferencing expired state. Parent movement preserves identity and live child handles. Stale requests, activities, targets and process endpoints require rejection or explicit rebind.

Derive RNG streams from base seed, stable env ID, local episode index and stable stream/service/component IDs. Another episode's resets or worker schedule must not change a sequence.

## Gateway and document contracts

Encode 64-bit identities/correlations as unsigned decimal strings. Unknown numeric reports use null with availability/validity, not zero or NaN. DMS bytes use bounded base64 with decoded-byte/native-limit checks.

Build one immutable legal gateway snapshot per decision tick. Re-reading and internal fanout are non-consuming. The configured platform/system gateway ingests DMS once and routes it internally within that same boundary; legacy Agent.receive remains consuming.

Actor reports contain only bound Agent knowledge, with provenance/sample/receive times, age and validity. Preserve stale/unknown/unavailable values. Referee truth, arbitrary task custom state and reward cannot populate actor snapshots or action masks.

submit validates and queues. Range.step closes the tick, dispatches by stable service ID/request ID, advances configured native steps and freezes the next snapshot. The endpoint binds each request to its configured service identity; request documents cannot change sender/controller.

Use scoped edge-triggered request IDs. Identical retries return original acknowledgment without repeating effects; changed-body reuse fails. Keep live correlations until explicit closure and bound terminal tombstones/capacity without re-executing expired retries.

Serialize reset, step and submission at the Range boundary. Reject busy/lifetime failures, wrong generations and late/future ticks explicitly. Native refusals remain authoritative receipts with native reasons/IDs.

Keep actor endpoint proxies separate from Range/referee handles. Strict actor process isolation disables truth publication and recording and exposes only the endpoint's callable-method list. Recording/IPC truth, Viewer and critic/referee data stay outside actor observations and action masks. Diagnostic visual runs are labeled non-isolated.

## Scheduling, files and verification

Record exact path/symbol allowlists and main HEAD before each feature. Shared World/MaCore/Agent/adapters/schema references/CMake/boundary files have one active feature owner per exclusive write window. Integrate reviewed patches at recorded handoffs; never edit shared paths concurrently. Record interfaces, changed symbols, fixtures, pending gates and commit/diff at each handoff.

Use exclusive implementation windows. Stop and release the write/build slots after each handoff. Resume only when the user relays the next assignment and its dependencies are accepted. The user relays prompts and handoffs; do not contact external workers directly.

Acquire the exclusive serial native configure/build/test slot before execution; pause other writes to its build inputs. Each feature owner may execute its own native gates while holding that slot. Use one GUI slot for a complete Viewer scenario. Do not run competing viewers or CMake writers.

Use existing UCRT64 presets/launcher. Tools, environments and caches belong under D:/ENV. Set TMP, TEMP and TORCH_EXTENSIONS_DIR to D:/ENV/tmp before compilation. Read upstream README support matrices before varying dependency/toolchain versions.

One task has one background entry. Send raw background output directly to a file without a trailing buffered grep/sed/awk/head filter; filter while reading. Do not add a second polling job around an already reporting background task.

Use actual discovered nonzero test counts, bounded timeouts and --no-tests=error. Report current evidence and pending gates accurately; do not reuse old totals as current verification. Broaden tests when changed behavior or unresolved failures justify it.

Noncombat acceptance includes delivered P2P tracks, typed C2 roles/tasks, configured ownership, independent episodes, generation/lifetime safety, submit deduplication, non-consuming C buffer reads, fresh full reset and installed-consumer provenance. The full M4 fixture covers crew loss, fire, an in-flight weapon, a decoy, a consumed store, pending C2 tasks/consent, queued messages and a removed entity. Full native-state reset inputs come from a supplied fixture using existing SDK behavior. Range APIs may configure the scenario and inspect referee outcomes; actor combat requests/releases use existing Agent/C2 permission paths, never Range consent injection to fulfill actor requests.

Use the dated [Viewer evidence](docs/sdk/viewer-validation.md) as existing diagnostic evidence. Do not relabel a diagnostic baseline as installed public-endpoint joint acceptance. Consume current required visual evidence at full-program integration; record missing cases as pending. Preserve native outcome meanings without inventing mission success, breakup or crash.

Stop this SDK architecture program after J1-J10, current compatibility/full regression, installed consumer provenance and complete Viewer acceptance. Formal MA implementation and DRL training are subsequent upper-layer work.
