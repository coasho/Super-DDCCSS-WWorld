# Claude MA Training Work Allocation

Status: Generic M0, M1 and noncombat M2 delivered; the first complete mission episode gateway package remains pending. M4/M5 have not started.
Date: 2026-10-12.
Progress reviewed: 2026-10-14, main 616327a.
Delivered baseline: baf476f841970f1f3c90b8de0bf630e8a5478738.

This document lists Claude's assignments only. Follow the accepted [design](../specs/2026-10-12-ma-system-training-readiness-design.md), [Claude implementation plan](2026-10-12-ma-system-training-readiness.md) and [project memory](../../../.claude/CLAUDE.md). The design retains the complete architecture and J1-J10 program acceptance. Package identifiers follow the implementation plan.

## 1. Complete feature assignments

Each assigned feature includes native C++, public C++/C/Python interfaces, example usage, tests, schema definitions, boundary documentation and acceptance evidence. Ownership follows the feature across these layers.

| Claude feature | Packages and acceptance |
| --- | --- |
| Generic contracts, identities, configuration, errors, lifetime and compatibility vocabulary | Generic M0; schema fixtures and documented public contracts |
| Explicit MA-service/controller/capability ownership and DMS ingress/egress binding | M1; deterministic configured ownership and legacy compatibility |
| Navigation, passive sensing/contacts, tracks, link provenance, tasks and authority reports | Noncombat M2; equivalent legal reports across languages |
| Independent episode construction, scenario lifecycle, generation/closed handles and seed streams | M4; one World per episode and complete fresh reset |
| Mission gateway, immutable observations, envelopes, queues, deduplication and receipt transport | M5 core; public episode/endpoint/referee family and buffer semantics |
| Flight/sensing/P2P/C2 task, control, override, lost-link and package behavior | Noncombat M5; existing permission-scoped execution paths |
| Actor endpoint proxy and strict process isolation | M5 core; actor-only whitelist and publication/recording guard |
| Example gateway, components, BT/planner fixtures, arbitration and lifecycle | M6 noncombat; installed SDK usage and deterministic legal composition |
| Batch scheduling, role topology, autoreset, final observations, noncombat task criteria, referee-stream transport, communication costs and training adapter | M7 core; independent episode/role lifecycle and component hooks |
| Consumer packaging, public facade assembly, ABI checks, provenance and generic joint acceptance | M8 generic; installed C++, C99 and Python consumers |

Claude owns the full P2P and generic C2 transport feature packages. The gateway consumes frozen extension payload/receipt contracts through agreed registration seams. Implement generic authentication, generation/tick checks, queueing and stable dispatch; preserve the supplied native handler's validation, authority and correlation semantics.

## 2. Baseline and required integration inputs

The Viewer/engagement baseline was separately verified, committed and pushed on main as baf476f841970f1f3c90b8de0bf630e8a5478738. Its evidence and limits are in the [Viewer validation record](../../sdk/viewer-validation.md). It remains a native diagnostic with explicit Range-supplied consent, separate from formal MA J5 approval and installed endpoint joint acceptance.

Generic M0 was delivered in 1874091 and corrected in 182ac58. It consumed the partial Viewer compatibility artifacts and captured the complete generic baseline under D:/ENV/tmp/ma-training-baseline, including public type/vtable metadata, native/Python exports and frozen C++, C99 and Python consumers. The dated [compatibility baseline](../../sdk/ma-compatibility-baseline.md) records provenance and limitations; later packages must verify compatibility against it.

M1 was delivered in 990554b and noncombat M2 in fb23a9b. Follow-ups 9b06c79 and 2bad550 cover binding transitions, foreign-World Agent rejection, peer provenance and the qualified activity-preservation contract. These deliveries do not implement independent mission episodes or the gateway. The first complete package still requires full M4 and M5 core.

Required inputs at the integration gates are frozen extension schemas/handler contracts, a supplied full native-state reset fixture, and dated combat joint/Viewer acceptance evidence. Core work proceeds independently of new engagement-policy implementation. Record absent inputs as pending; never substitute fabricated success or mark full-program acceptance complete.

| Claude gate | Required inputs | Result |
| --- | --- | --- |
| Generic M0 | Delivered baseline and compatibility artifacts | Frozen generic contracts and complete compatibility baseline |
| M1 and noncombat M2 | Frozen generic M0; M1 for multi-service reports | Explicit ownership and legal noncombat reports |
| Full M4 | Generic M0, M1 and supplied existing-native-state fixture | Independent episode ownership and full fresh reset |
| M5 core | Generic M0, M1, noncombat M2 and full M4 | Legal noncombat gateway in all public languages |
| M6 noncombat | M5 core | Legal composition and component lifecycle |
| M7 core | M4, M5 core and M6 noncombat | Batch/role/reset/training lifecycle |
| M8 generic | Claude packages and current compatibility evidence | Installed consumers, isolation and generic joint results |

Full M5 additionally requires accepted extension handlers. Full M6 requires M5 full. Full M7 additionally requires accepted outcome/criterion extensions and M6 full. Full M8 requires every accepted core and extension gate, J1-J10, current compatibility/full regression, installed provenance and current combat Viewer evidence.

Freeze request-kind names, schema references and handler signatures before facade integration. Unavailable families fail explicitly until installed. Full M4 uses existing SDK behavior and need not wait for M3. Interim noncombat smoke checks do not complete full M4, M5 core or the first Claude package.

## 3. Claude feature file allowlists

Existing paths identify integration seams; new filenames are proposed. Preserve established runtime type owners and adapter conventions. Record exact path/symbol allowlists before each package.

| Claude feature | Primary paths |
| --- | --- |
| Generic contracts | docs/sdk/ma-contract.schema.json, mission-training.md, ma-composition.md; generic design/plan sections |
| Service binding | include/fsim/MAService.h; src/session/MaServiceBindings.h/.cpp; binding portions of World and MaCore; src/sdk/World.cpp |
| Legal noncombat reports | Navigation/contact/track/task/authority portions of MAReports and Agent; corresponding C/Python wrappers and tests |
| Gateway/lifecycle | src/env/MissionEpisode, MissionActions, MissionObservation; public MissionEpisode family; fsim_mission_env adapters; Python mission.py and isolation.py |
| Composition/batch | examples/python/ma_composition; MissionBatch; noncombat MissionTask criteria/lifecycle, referee-stream transport and communication costs; language adapters and tests |
| Generic delivery | Generic joint tests; installed consumers; package CMake/run gates; compatibility and validation documentation |

Claude owns the generic schema envelope and integrates frozen external schema references at agreed seams. Keep extension definitions in separate files. Document the JSON Schema dialect without a new runtime dependency.

## 4. Shared-file and execution protocol

1. Record main HEAD, the owned feature, exact paths/symbols, required inputs and expected gates before a package.
2. Acquire an exclusive write window for each shared path. World, MaCore, Agent, adapters, schema references, CMake registrations and boundary tables have one active writer per path.
3. Integrate each Claude feature across native code, language interfaces, examples, tests and documentation. Consume reviewed extension patches at recorded handoffs; do not edit shared paths concurrently.
4. Record changed symbols, frozen interfaces, schema fixtures, pending issues and the commit/diff at every handoff. Contract changes identify affected consumers and compatibility behavior first.
5. Acquire one exclusive native configure/build/test slot; pause other changes to its build inputs. Record the tested HEAD/diff and release the slot after capturing evidence.
6. Use one GUI slot for any complete visual scenario. No competing viewers, CMake writers or capture polling jobs.
7. Stage only reviewed feature-owned files, inspect the staged diff and commit on main. Follow authorized delivery instructions for pushing.

Use discovered nonzero test counts, bounded timeouts and --no-tests=error. Broaden testing for changed behavior, failures or unresolved concerns. A final full-suite claim requires the final current build and results.

All tools/environments/caches use D:/ENV. Set TMP, TEMP and TORCH_EXTENSIONS_DIR to D:/ENV/tmp before compilation. One background job has one entry and writes raw output to a file; filter only while reading it.

## 5. First Claude package: noncombat mission episode gateway

Inputs: delivered baseline, frozen generic M0 and accepted architecture. Scope: M1, full M4, noncombat M2 and minimal M5 core, delivered through installed public C++, C99 and Python.

Deliver explicit service ownership, one-World episodes, fresh reset, safe endpoint/referee handles, immutable snapshots, wait/flight/P2P/typed C2-task actions, queued receipts, deduplication and strict endpoint isolation. Complete native code, adapters, example, tests, boundary tables and English documentation together.

Required end-to-end acceptance:

- Reverse Agent/service creation order and obtain equivalent configured grants, DMS ownership and activity permissions; reject duplicate exclusive or foreign-controller bindings.
- Create two independent episodes. Resetting or messaging one does not change the other's state or sampled sequence.
- A lead shares a measured track. A radar-off wing receives it only after modeled delivery, then creates its own legal link designation. Hidden referee truth does not change actor snapshots.
- A C2 role without an aircraft sends a typed task; MA accepts/rejects and reports progress/completion. Unauthorized roles have no execution effect.
- Freeze one snapshot per tick. Repeated reads and component fanout are non-consuming; the same platform/system gateway ingests DMS once and routes it within that boundary.
- Submit queues only. Range.step dispatches in stable service/request order; late/future/stale ticks fail. Identical retries and C buffer-size queries do not repeat side effects or consume messages.
- Parent movement preserves episode identity and live children. Reset rejects prior generations; parent destruction closes children before releasing the World. Fresh same-seed construction matches reset after normalizing generation metadata only.
- Consume the supplied existing-native-state fixture and verify complete reconstruction after crew loss, fire, an in-flight weapon, a decoy, a consumed store, pending C2 tasks/consent, queued messages and a removed entity. Range may configure/inspect referee state; actor requests/releases retain existing Agent/C2 permission paths, without Range consent injection to fulfill actor requests.
- Installed C++, C99 and Python consumers use only the owned prefix, with source/build DLL/data fallbacks removed. Existing APIs, binaries, permissions, defaults and Vehicle.reset remain compatible.
- Strict endpoint-process and in-process traces agree with publication/recording disabled. Proxy access to Range/referee or stale generations fails.

An interim noncombat gateway smoke check may precede full fixture acceptance. Complete first-package and M5 core acceptance require full M4, including the supplied native-state fixture. Record missing inputs as pending gates.

## 6. Later Claude packages and program exit

Complete M6 noncombat composition, M7 core batch/training and M8 generic delivery next. Claude delivers J1/J2/J4/J6/J7, generic J10 infrastructure and the task/message/lifecycle portions of J3/J8. Consume supplied evidence for J5/J9, combat J10 and the remaining J3/J8 cases before declaring full-program acceptance.

SDK remains the sole simulation authority. Formal complex MA execution belongs in the upper layer. Agent/service counts do not prescribe actor/component counts.

Keep recording, IPC truth publication, Viewer access and referee/critic streams outside actor training. Diagnostic visual acceptance is explicitly non-isolated and separate from strict actor evidence.

Claude's assignment is complete after its packages and integration handoffs have current evidence. The whole SDK program ends only after the complete design's J1-J10 and M8 requirements pass. Real MA implementation and DRL training then proceed in the upper-layer project.
