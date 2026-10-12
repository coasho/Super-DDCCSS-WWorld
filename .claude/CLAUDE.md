# Project Memory for Claude

Repository: Super-DDCCSS-WWorld.
Branch: main only.
Implementation baseline: baf476f841970f1f3c90b8de0bf630e8a5478738.

## Context and filesystem boundaries

Use this project memory, tracked project documentation and explicitly relevant project source as persistent context. The accepted [design](../docs/superpowers/specs/2026-10-12-ma-system-training-readiness-design.md), [plan](../docs/superpowers/plans/2026-10-12-ma-system-training-readiness.md) and [work allocation](../docs/superpowers/plans/2026-10-12-ma-system-training-work-allocation.md) govern MA training work.

Do not read, search, enumerate, traverse or modify any directory named .codex or anything inside it. This includes repository-local directories, user-profile directories such as C:/Users/Administrator/.codex, its memories/skills/configuration, and .codex directories anywhere else.

Do not inspect or modify other worktrees. Scope searches to explicit allowed project paths; avoid broad filesystem or repository enumeration that could enter an excluded directory. Exclude every .codex directory before a permitted recursive source search. Use the tracked project documents and this file instead of profile memory lookup.

Work on main. Do not create branches or worktrees. Verify the current branch before changing files and preserve unrelated working-tree changes. Stage only reviewed feature-owned files and inspect the staged diff.

Conversation is Chinese. Maintained code, documentation, tests, UI text, metadata and commit messages are English.

## Feature ownership

Claude owns complete generic features: generic contracts/schema; explicit MA-service/controller binding; legal noncombat reports; P2P and C2 task/control/package/lost-link behavior; independent mission episode lifecycle; the mission gateway/transport and actor isolation; composition examples; batch/training lifecycle, noncombat task criteria, referee-stream transport and communication costs; installed consumers, generic integration and compatibility checks.

For those features, Claude owns native C++, C++/C/Python interfaces, examples, tests, documentation and acceptance evidence together. Future generic integration and compatibility builds are Claude work.

Codex owns complete weapon/guidance features: equipment/countermeasure/launch-zone access; engagement policy and strike consent; combat payload handlers/receipts; combat outcomes/rewards; combat joint scenarios and actual-strike Viewer acceptance. Their public interfaces, adapters, tests, examples and documentation follow Codex ownership.

Codex delivered the reviewed Viewer baseline as a separate verified main commit and push: baf476f841970f1f3c90b8de0bf630e8a5478738. Claude implementation has not started. Begin with generic M0 contract completion, then the first noncombat feature package.

Freeze generic M0 schemas, native mappings, lifetime/error and ownership contracts before editing downstream public bindings.

The first Claude package is the complete noncombat mission episode gateway: M1 + M4 + noncombat reports + minimal M5 core through installed public C++, C99 and Python. Only an interim gateway smoke check may precede full M4 fixture acceptance; complete M5 core and first-package acceptance require full M4. The Codex-owned combat-state reset fixture uses already accepted SDK behavior and need not wait for M3. M5 core is independent of M3; full M5 requires both core and combat gates. M6 noncombat composition consumes M5 core; full composition scenarios consume M5 full. M7 core consumes M6 noncombat and remains separate from combat outcome/reward gates; full M7 also requires full M6.

Claude owns the generic schema. Codex owns engagement extension definitions in separate files. The generic facade binds frozen request kinds to handlers; Codex implements combat validation/submission/correlation through existing SDK authority.

## Architecture and compatibility

SDK is the sole simulation authority for flight, sensing, communication delivery, release, weapon guidance, fuzes, collisions and damage. Client code uses SDK interfaces for execution and handles presentation/interaction. Never duplicate simulation in a client.

The formal complex MA runtime belongs in the upper-layer project. Its actors, BT/planners, blackboards, internal services and arbitration are not an SDK production framework. SDK examples are minimal replacement/lifecycle demonstrations.

An SDK Agent is a permission-scoped endpoint. MA service, platform, component and DRL actor are distinct identities; their counts need not match. C2 services may exist without aircraft.

Internal components may share legally obtained state inside the same platform/system boundary. Cross-platform/C2 information passes through modeled sensors/links. Sharing a gateway does not expand authority or permit a cross-platform blackboard bypass.

Preserve existing public signatures, class/struct layouts, plugin vtables, enum numbers, status/error semantics, observation/action shapes, default permissions, behavior and ABI. Use additive versioned interfaces. Old-header binaries must continue to work. Preserve established native/Python runtime type owners and CPython stable-ABI conventions.

Keep flight VecEnv compatible. New mission episodes initially own one World each, including mutable task/judge, MaCore, DMS, guidance/damage state and cursors. Reconstruct the owned World for a full reset; do not substitute Vehicle.reset.

Existing Vehicle.reset preserves authority, tasks, overrides and C2 activity ownership. Do not change those semantics. Generation-checked episode reset is a separate contract.

A retained generation/closed control block protects child handles: reset rejects prior generations, and parent destruction closes children before releasing the World, so access fails before dereferencing expired state. Parent movement preserves identity and live child handles. Stale requests, activities, targets and process endpoints require rejection or explicit rebind.

Derive RNG streams from base seed, stable env ID, local episode index and stable stream/service/component IDs. Another episode's resets or worker schedule must not change a sequence.

## Gateway and document contracts

Encode 64-bit identities/correlations as unsigned decimal strings. Unknown numeric reports use null with availability/validity, not zero or NaN. DMS bytes use bounded base64 with decoded-byte/native-limit checks.

Build one immutable legal gateway snapshot per decision tick. Re-reading and internal fanout are non-consuming. The configured platform/system gateway ingests DMS once and routes it internally within that same boundary; legacy Agent.receive remains consuming.

Actor reports contain only bound Agent knowledge, with provenance/sample/receive times, age and validity. Preserve stale/unknown/unavailable values. Referee truth, arbitrary task custom state and reward cannot populate actor snapshots or action masks.

submit validates and queues. Range.step closes the tick, dispatches by stable service ID/request ID, advances configured native steps and freezes the next snapshot. The endpoint authenticates service identity; request documents cannot change sender/controller.

Use scoped edge-triggered request IDs. Identical retries return original acknowledgment without repeating effects; changed-body reuse fails. Keep live correlations until explicit closure and bound terminal tombstones/capacity without re-executing expired retries.

Serialize reset, step and submission at the Range boundary. Reject busy/lifetime failures, wrong generations and late/future ticks explicitly. Native refusals remain authoritative receipts with native reasons/IDs.

Keep actor endpoint proxies separate from Range/referee handles. Strict isolation disables truth publication and recording and uses an endpoint-only whitelist. Recording/IPC truth, Viewer and critic/referee data stay outside actor training. Diagnostic visual runs are labeled non-isolated.

## Scheduling, files and verification

Record exact path/symbol allowlists and main HEAD before each feature. Shared World/MaCore/Agent/adapters/schema references/CMake/boundary files have one active feature owner per exclusive write window. Other owners supply reviewed patches or wait for handoff. Record interfaces, changed symbols, fixtures, pending gates and commit/diff at each handoff.

Acquire the exclusive serial native configure/build/test slot before execution; pause other writes to its build inputs. Each feature owner may execute its own native gates while holding that slot. Use one GUI slot for a complete Viewer scenario. Do not run competing viewers or CMake writers.

Use existing UCRT64 presets/launcher. Tools, environments and caches belong under D:/ENV. Set TMP, TEMP and TORCH_EXTENSIONS_DIR to D:/ENV/tmp before compilation. Read upstream README support matrices before varying dependency/toolchain versions.

One task has one background entry. Send raw background output directly to a file without a trailing buffered grep/sed/awk/head filter; filter while reading. Do not add a second polling job around an already reporting background task.

Use actual discovered nonzero test counts, bounded timeouts and --no-tests=error. Report current evidence and pending gates accurately; do not reuse old totals as current verification. Broaden tests when changed behavior or unresolved failures justify it.

Noncombat acceptance includes delivered P2P tracks, typed C2 roles/tasks, configured ownership, independent episodes, generation/lifetime safety, submit deduplication, non-consuming C buffer reads, fresh full reset and installed-consumer provenance. The full M4 fixture covers crew loss, fire, an in-flight weapon, a decoy, a consumed store, pending C2 tasks/consent, queued messages and a removed entity. Combat-state reset inputs come from an agreed Codex-owned fixture using existing SDK behavior. Range APIs may configure the scenario and inspect referee outcomes; actor combat requests/releases use existing Agent/C2 permission paths, never Range consent injection to fulfill actor requests.

The native strike snapshot at simulation time 40.0 s records crewLost, engineOut and three torn panels while the target remains airborne. It does not establish separated sections or a crash. Do not interpret an entity/crew-loss label as visual breakup or a crash. The existing Viewer baseline diagnostic uses explicit Range-supplied consent; preserve that behavior and keep it separate from formal MA J5 approval acceptance and installed public-endpoint joint acceptance.

Combat Viewer acceptance runs fullscreen to scenario completion, with carrier -> missile -> actually damaged target focus and exactly launch + 1.0 s and actual hit + 0.3 s captures. No timed self-destruction or false hit capture. This is Codex's complete combat feature.

Stop this SDK architecture program after J1-J10, current compatibility/full regression, installed consumer provenance and complete Viewer acceptance. Formal MA implementation and DRL training are subsequent upper-layer work.
