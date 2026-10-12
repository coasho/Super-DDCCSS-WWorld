# MA System Training: Vertical Work Allocation

Status: Planned ownership and handoff contract. Claude implementation has not started.
Date: 2026-10-12.
Baseline commit: baf476f841970f1f3c90b8de0bf630e8a5478738.

This allocation supplements the accepted [design](../specs/2026-10-12-ma-system-training-readiness-design.md) and [implementation plan](2026-10-12-ma-system-training-readiness.md). Their architecture, compatibility constraints and finite M8 exit remain authoritative. Package numbers below refer to that plan. Claude context and filesystem rules are in the [project memory](../../../.claude/CLAUDE.md).

## 1. Complete feature ownership

Each feature owner delivers its native implementation, public C++/C/Python interfaces, example usage, tests, schema definitions, boundary documentation and acceptance evidence. Native C++ work belongs to the owner of the feature it implements. Interfaces and tests follow that same feature owner.

| Complete feature | Owner | Packages and acceptance |
| --- | --- | --- |
| Generic contracts, identities, configuration, errors, lifetime and compatibility vocabulary | Claude | Generic M0; schema fixtures and documented public contracts |
| Combat report/request/policy/outcome contracts and engagement schema extensions | Codex | Combat M0; native mappings, permissions and equipment availability |
| Explicit MA-service/controller/capability ownership and DMS ingress/egress binding | Claude | M1; deterministic configured ownership and legacy compatibility |
| Navigation, passive sensing/contacts, tracks, link provenance, tasks and authority reports | Claude | Noncombat M2; equivalent legal reports across languages |
| Weapon/store/support reports, countermeasure inventory/dispense and launch-zone queries | Codex | Combat M2; equivalent equipment access across languages |
| Engagement authorization profiles, policy activation and correlated strike consent | Codex | M3; approved/denied/delayed/expired/canceled consent and release revalidation |
| Independent episode construction, scenario lifecycle, generation/closed handles and seed streams | Claude | M4; one World per episode and complete fresh reset |
| Mission gateway, immutable observations, envelopes, queues, deduplication and receipts transport | Claude | M5 core; public episode/endpoint/referee family and buffer semantics |
| Flight/sensing/P2P/C2 task, control, override, lost-link and package behavior | Claude | Noncombat M5; existing permission-scoped execution paths |
| Combat request payloads, strike/countermeasure/policy handlers and native combat receipts | Codex | Combat M5; authorized native release and accurate activity correlation |
| Actor endpoint proxy and strict process isolation | Claude | M5 core; actor-only whitelist and publication/recording guard |
| Example gateway, components, BT/planner fixtures, arbitration and lifecycle | Claude | M6; installed SDK usage and deterministic legal composition |
| Batch scheduling, role topology, autoreset, final observations, noncombat task criteria, referee-stream transport, communication costs and training adapter | Claude | M7 core; independent episode/role lifecycle and component hooks |
| Combat outcomes, combat task criteria, termination causes and combat reward definitions | Codex | Combat M7; native damage/crew-loss/collision/friendly-damage evidence |
| Consumer packaging, ABI checks, provenance and generic joint acceptance | Claude | M8 generic; installed C++, C99 and Python consumers |
| Strike/guidance/outcome joint scenarios and combat viewer acceptance | Codex | M8 combat; complete native strike chain and event-relative captures |

Necessary changes to native weapon release, guidance, fuzes or damage within the accepted combat scope also belong to Codex, with their interfaces and acceptance. Model/card/control calibration remains outside this plan.

P2P and generic C2 transport are Claude features. Engagement policy payloads and strike-consent correlation are Codex features, including their DMS adapters. Claude's gateway transports the frozen combat payloads and receipts; Codex defines and implements their meaning.

Generic integration, public facade assembly, compatibility builds and package-consumer infrastructure belong to Claude. Combat integration and combat acceptance belong to Codex. The active feature owner performs shared-file integration during its exclusive write window.

## 2. Baseline delivery and dependency gates

Codex verified and delivered the Viewer/engagement baseline as a separate main commit and push: baf476f841970f1f3c90b8de0bf630e8a5478738. Keep that baseline separate from MA packages. Its current evidence and limits are in the [Viewer validation record](../../sdk/viewer-validation.md).

The native strike snapshot at simulation time 40.0 s records crewLost, engineOut and three torn panels. The target remains airborne; separated sections and a crash are not established by that snapshot. Crew loss must not be described as physical breakup or a crash. The final baseline evidence records the actual inspected result and its limits.

The existing Viewer baseline is a native diagnostic with explicit Range-supplied consent. Preserve that baseline behavior; it is not formal MA J5 approval acceptance or installed public-endpoint joint acceptance.

Claude owns generic M0 completion, baseline compatibility artifact consumption and the first noncombat feature package. Partial baseline artifacts are under D:/ENV/tmp/viewer-strike-delivery-20261012/compatibility: archived public headers, native/Vision exports, 2,357 protected input hashes and a frozen installed C++ consumer with 31 binaries and its data manifest. Complete public type/vtable metadata, Python export and old C99/Python consumer coverage remain M0 work; this artifact set does not complete that gate.

| Gate | Required inputs | Result |
| --- | --- | --- |
| M5 core | Frozen generic M0, M1, M4 and noncombat M2 reports | Legal noncombat gateway in all public languages |
| M5 combat | M5 core, combat M2 and M3 | Combat payloads/receipts through the same gateway |
| M5 full | M5 core and M5 combat | Complete accepted mission action/report contract |
| M6 noncombat | M5 core | Legal noncombat composition and component lifecycle |
| M6 full | M5 full | Complete composition scenarios through the full endpoint contract |
| M7 core | M4, M5 core and M6 noncombat | Batch/role/reset/training lifecycle |
| M7 combat | M5 combat, M7 core and frozen combat outcome criteria | Versioned combat outcomes/rewards |
| M7 full | M7 core, M7 combat and M6 full | Complete mission training contract |
| M8 complete | All core/combat gates and current compatibility evidence | J1-J10 and finite delivery |

M5 core does not depend on M3. Freeze combat request-kind names and the handler binding contract before facade integration; implement their payload validation and behavior in the Codex package. Unavailable families fail explicitly until installed, rather than returning fabricated success.

## 3. Feature file allowlists

Existing paths below are integration seams from the accepted plan; new filenames are proposed. Preserve established runtime type owners and adapter conventions. Each package records an exact path/symbol allowlist before editing.

| Feature | Primary paths |
| --- | --- |
| Claude contracts | docs/sdk/ma-contract.schema.json, mission-training.md, ma-composition.md; design/plan generic contract sections |
| Claude binding | include/fsim/MAService.h; src/session/MaServiceBindings.h/.cpp; binding portions of World and MaCore; src/sdk/World.cpp |
| Claude legal reports | Navigation/contact/track/task/authority portions of MAReports and Agent; corresponding C/Python wrappers and tests |
| Codex equipment reports | Equipment/weapon/countermeasure/launch-zone portions of MAReports and Agent; corresponding C/Python wrappers and tests |
| Codex authorization | include/fsim/EngagementPolicy.h; src/session/EngagementPolicy.h/.cpp; MaConsentLedger; relevant MaCore/AgentRules and DMS adapter symbols |
| Claude gateway/lifecycle | src/env/MissionEpisode, MissionActions, MissionObservation; public MissionEpisode family; fsim_mission_env adapters; Python mission.py and isolation.py |
| Codex combat gateway extension | Dedicated combat payload/handler/receipt files; explicitly handed-off combat dispatch registration and report append points |
| Claude composition/batch | examples/python/ma_composition; MissionBatch; noncombat MissionTask criteria/lifecycle, referee-stream transport and communication costs; associated language adapters and tests |
| Codex combat judge | Dedicated combat outcome/criterion/reward files; combat MissionTask integration points and fixtures |
| Claude delivery | Generic joint tests; installed consumers; package CMake/run gates; generic compatibility and validation documentation |
| Codex combat delivery | Combat joint fixtures/tests; combat consumers/examples; combat validation documentation and viewer strike runner |

Claude owns the generic schema envelope. Codex owns engagement extension definitions in a separate proposed docs/sdk/ma-engagement-contract.schema.json and corresponding dedicated native definitions. The generic schema references the frozen extension contract only at an agreed integration point. Both files use the documented JSON Schema dialect without a new runtime dependency.

The facade binds fixed request kinds to frozen handlers. Generic scheduling owns authentication, generation/tick checks, queueing and stable dispatch; a combat handler owns native combat validation, submission and correlation. Each request still maps to the accepted SDK operation and native permission rules.

## 4. Shared-file and execution protocol

1. Before a package, record main HEAD, owned feature, exact paths/symbols, required inputs and expected acceptance gates.
2. Acquire an exclusive write window for every shared path. Shared World, MaCore, Agent, adapters, schema references, CMake registrations and boundary tables have one active writer per path.
3. The active feature owner integrates its feature across all layers. The other owner supplies a reviewed patch or waits for the recorded handoff; it does not edit that path concurrently.
4. A handoff records changed symbols, frozen interfaces, schema fixtures, pending issues and the commit/diff to consume. Contract changes name affected consumers and compatibility behavior before implementation continues.
5. Acquire one exclusive native configure/build/test slot. Pause other changes to its build inputs, record the tested HEAD/diff and release the slot after evidence is captured. Either owner may run its own native gates while holding this slot.
6. Hold one GUI slot for the complete viewer scenario. No competing viewers, CMake writers or capture polling jobs.
7. Stage only the reviewed package allowlist, check the staged diff and commit on main. Keep unrelated modifications outside that commit. Follow the authorized delivery instructions for pushing.

New tests use discovered nonzero test counts, bounded timeouts and --no-tests=error. Broaden testing when changed behavior, failures or unresolved concerns justify it. A final full-suite claim requires the final current build and current results.

All tools/environments/caches use D:/ENV. Set TMP, TEMP and TORCH_EXTENSIONS_DIR to D:/ENV/tmp before compilation. One background job has one background entry and writes raw output directly to a file; filter only while reading it.

## 5. First Claude package: noncombat mission episode gateway

Inputs: pushed baf476f841970f1f3c90b8de0bf630e8a5478738, frozen generic M0 contracts and the accepted architecture. Scope: M1, M4, noncombat reports and minimal M5 core, delivered through installed public C++, C99 and Python interfaces.

Deliver explicit service ownership, one-World episode construction, fresh reset, safe endpoint/referee handles, immutable snapshots, wait/flight/P2P/typed C2-task actions, queued receipts, deduplication and strict endpoint isolation. Complete native code, language adapters, example, tests, boundary tables and English documentation together.

Required end-to-end acceptance:

- Reverse Agent/service creation order and obtain equivalent configured grants, DMS ownership and activity permissions; reject duplicate exclusive or foreign-controller bindings.
- Create two independent episodes. Resetting or messaging one does not change the other's state or sampled sequence.
- Lead shares a measured track. A radar-off wing receives it only after simulated delivery, then creates its own legal link designation. Hidden referee truth does not change actor snapshots.
- A C2 role without an aircraft sends a typed task; MA accepts/rejects and reports progress/completion. Unauthorized roles have no execution effect.
- Freeze one snapshot per decision tick. Repeated reads and internal component fanout are non-consuming; the same platform/system gateway ingests DMS once and routes it within that boundary.
- Submit queues only. Range.step dispatches in stable service/request order; late/future/stale ticks fail. Identical request retries and C buffer-size queries do not repeat side effects or consume messages.
- Parent movement preserves episode identity and live child handles. Reset rejects prior-generation children/intents; parent destruction closes live children before releasing the World, so access fails before dereferencing expired state. Fresh same-seed construction matches reset after normalizing generation metadata only.
- Verify full native-state reconstruction after crew loss, fire, an in-flight weapon, a decoy, a consumed store, pending C2 tasks/consent, queued messages and a removed entity. The agreed Codex-owned fixture uses already accepted SDK behavior. Range APIs may configure the scenario and inspect referee outcomes; actor combat requests/releases use existing Agent/C2 permission paths, never Range consent injection to fulfill actor requests. Claude's gateway does not implement tactical strike decisions for that fixture.
- Installed C++, C99 and Python consumers use only the owned install prefix, with source/build DLL/data fallbacks removed. Existing APIs, binaries, permission rules, defaults and Vehicle.reset behavior remain compatible.
- Strict endpoint-process and in-process traces agree with truth publication/recording disabled; proxy access to Range/referee or stale generations fails.

An interim noncombat gateway smoke check may precede full M4 fixture acceptance. Complete M5 core and complete first-package acceptance require full M4, including the Codex-owned combat-state reset fixture, before combat handler integration. That fixture uses already accepted SDK behavior and need not wait for M3. Record absent inputs as pending gates, never as completed acceptance.

## 6. Later joint acceptance and finite exit

Claude owns J1/J2/J4/J6/J7 and generic J10 infrastructure. Codex owns J5/J9 and combat J10 cases. J3 and J8 are joint gates with Claude task/message/lifecycle cases and Codex combat fixtures. Each owner supplies reproducible evidence for its complete feature.

SDK remains the sole authority for simulation, sensing, link delivery, release, guidance, fuzes, collisions and damage. Formal complex MA execution belongs in the upper layer. SDK Agent/service count does not prescribe actor/component count.

Keep recording, IPC truth publication, Viewer access and referee/critic streams outside actor training observations and actions. Diagnostic recording/viewer acceptance is explicitly non-isolated and separate from strict actor isolation evidence.

Stop after complete J1-J10, current compatibility/full regression, installed-consumer provenance and complete combat viewer acceptance. Real MA runtime implementation and DRL training proceed in the upper-layer project.
