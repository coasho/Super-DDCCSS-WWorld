# MA System Integration and Training Readiness Implementation Plan

> **For agentic workers:** Use the applicable plan-execution workflow for the current tool environment. Follow the [vertical work allocation](2026-10-12-ma-system-training-work-allocation.md): each feature owner delivers native implementation, language adapters, tests, examples, documentation and evidence. Shared paths and native/GUI execution use exclusive windows. Claude context is in the [project memory](../../../.claude/CLAUDE.md).

**Goal:** Make the SDK support upper-layer composite MA systems and correct local/P2P/C2 training episodes through public, compatible interfaces.

**Architecture:** Keep formal MA decision runtime in the upper layer. Add explicit MA-service binding, legal reports and a separate mission-episode API over the existing execution core. Initially isolate one World per episode, and validate with deterministic composite fixtures and installed consumers.

**Tech Stack:** C++17, C99/C11 adapters, CPython stable ABI 3.11+, existing CMake 3.25+ UCRT64/Ninja presets, native JSBSim and existing communication/mission simulation.

**Spec:** [MA system integration design](../specs/2026-10-12-ma-system-training-readiness-design.md).

**Delivered baseline:** baf476f841970f1f3c90b8de0bf630e8a5478738, verified and pushed to main. See [current Viewer evidence](../../sdk/viewer-validation.md). MA contract and implementation packages remain pending.

## Global Constraints

- Work on main. Do not create branches or worktrees.
- Formal MA, BT engines, planners and learned policies belong in the upper layer. SDK examples demonstrate composition only.
- Preserve existing public signatures, layouts, vtables, enum numbers, errors, legacy defaults and flight VecEnv shapes.
- New interfaces are additive and versioned; old-header binaries must run with the new library.
- Preserve Vehicle.reset authority/task/override/C2 ownership semantics and existing MA retirement/history behavior.
- SDK owns sensing, links, release, flight, guidance, fuzes, collisions and damage. Do not add client-side physics or self-destruction scripts.
- Existing physics/cards/control/equipment defaults are unchanged. New report/policy/environment configuration is explicit.
- All maintained files and commit messages are English; conversation is Chinese.
- Tools/caches are under D:/ENV; TMP, TEMP and TORCH_EXTENSIONS_DIR are D:/ENV/tmp.
- Read upstream support matrices before changing a toolchain or dependency. This plan requires no new environment installation.
- Raw background output goes to a file without a trailing buffered filter. One job has one background entry.
- Viewer tests are fullscreen, run complete scenarios, and retain exactly two event-relative strike captures.
- Only the separately delivered Viewer baseline is complete below. MA contracts and M1-M8 implementation/acceptance remain pending; proposed interfaces are not implemented APIs.

## Review Focus

- An old episode reference arrives after reset: reject it without affecting new state.
- A second MA service or internal component consumes shared state: explicit ownership must prevent traversal-order grants and destructive inbox races.
- A report is unknown, stale or not delivered: preserve that condition; never fill it from referee truth.
- A consent answer is duplicated, reordered, canceled, expired or for another policy/activity: no unauthorized or repeated release.
- One team member ends while others continue, or a next-step reset is pending: correct final observations, bootstrap flags and zero old-episode side effects.

## Dependency and stop gates

| Package | Dependencies | Reviewable output |
| --- | --- | --- |
| M0 Baseline/contracts | None | Current compatible baseline and signed-off interface/spec vocabulary |
| M1 MA-service ownership | M0 | Explicit owner/binding path for new mission scenarios |
| M2 Agent parity/reports | M0; M1 for multi-service cases | C++/C/Python legal equipment access and report provenance |
| M3 Authorization profile | M1; M2 for report-dependent checks | Bounded opt-in policy and correlated request lifecycle |
| M4 Episode lifecycle | M0-M1 | Fresh independent World per episode; stale-reference rejection |
| M5 core / combat / full | Core: generic M0, M1, noncombat M2, full M4; combat adds combat M2 and M3; full requires both | Public role-scoped observations and typed asynchronous requests |
| M6 noncombat / full | Noncombat: M5 core; full: M5 full | Upper-layer gateway/components/BT/planner fixture with deterministic arbitration |
| M7 core / combat / full | Core: M4, M5 core, M6 noncombat; combat adds M5 combat and outcome contracts; full adds both streams and M6 full | Independent batch sampling, team outcomes and training lifecycle |
| M8 Joint acceptance/delivery | M0-M7 | Installed consumers, isolated actor path, native/viewer evidence and final stop |

M2 and M4 can proceed independently after their inputs are frozen. M3 and M4 share identity/generation semantics, not mutable source ownership. Do not start downstream work against undocumented provisional signatures.

M5 core and noncombat composition do not require M3. Full M4 still requires the agreed Codex-owned combat-state reset fixture using existing SDK behavior; that fixture need not wait for M3. Interim noncombat smoke checks do not complete full M4 or M5 core. Full M5/M7/M8 gates retain all accepted combat and noncombat cases.

## Planned file map

| Area | Existing files | New files |
| --- | --- | --- |
| Contracts | docs/agent-boundary.md, docs/sdk/agents.md, docs/sdk/boundary.toml | docs/sdk/ma-composition.md, docs/sdk/mission-training.md, docs/sdk/ma-contract.schema.json |
| Binding | src/session/World.h/.cpp, MaCore.h/.cpp, src/sdk/World.cpp | include/fsim/MAService.h, src/session/MaServiceBindings.h/.cpp |
| Safe equipment reports | include/fsim/Agent.h, src/sdk/Agent.cpp, fsim_mission_c.cpp, python/fsim/agent.py, python/src/_native_mission.c | include/fsim/MAReports.h, src/session/MaReports.h/.cpp |
| Policy | src/session/MaCore.h/.cpp, MaConsentLedger.h, AgentRules.cpp, SDK/native DMS adapters | include/fsim/EngagementPolicy.h, src/session/EngagementPolicy.h/.cpp |
| Episode/actions | src/env/CMakeLists.txt, src/sdk/CMakeLists.txt | src/env/MissionEpisode.h/.cpp, MissionActions.h/.cpp, MissionObservation.h/.cpp |
| Public environment | SDK/native registration files, python/CMakeLists.txt, python/fsim/__init__.py | include/fsim/MissionEpisode.h, fsim_mission_env.h, src/sdk/MissionEpisode.cpp, fsim_mission_env_c.cpp, python/src/_native_mission_env.c/.h, python/fsim/mission.py |
| Composition | Existing AgentProcess proxy and examples documentation | examples/python/ma_composition/contracts.py, components.py, system.py, run.py, README.md |
| Training batch | Environment/SDK/native build registration | src/env/MissionBatch.h/.cpp, MissionTask.h/.cpp, include/fsim/MissionBatch.h |
| Verification | tests/CMakeLists.txt, tests/package_consumer/CMakeLists.txt, cmake/RunPackageConsumer.cmake, tools/boundary | New focused tests, joint scenario fixtures and installed C/Python consumers described below |

This map identifies integration paths; the [vertical allocation](2026-10-12-ma-system-training-work-allocation.md) assigns feature owners. New file names define ownership seams, not an instruction to move unrelated code. Keep existing runtime type owners and adapter conventions. Generic schema envelopes and combat extensions have separate owners/files; use explicit shared-path handoffs for registrations and facade integration.

## Shared proposed interfaces

These are new interfaces. Existing Agent/World/VecEnv interfaces remain source and binary compatible.

### Explicit service binding

In include/fsim/MAService.h:

    struct MAServiceBindingV1 {
        std::uint64_t serviceId;
        std::uint32_t platform;
        control::ControllerId controller;
        std::vector<std::string> commandCapabilities;
        bool ownsDmsIngress;
        bool ownsDmsEgress;
    };
    bool bindMAService(World&, const MAServiceBindingV1&, std::string* error);

This free function is Range-only. It verifies a Range-minted controller and supports explicit, non-overlapping ownership. It does not create arbitrary controller identities. Report sharing and authority ownership are separate.

### Environment family

In include/fsim/MissionEpisode.h, export new movable PIMPL classes:

    class MissionEpisode {
        explicit MissionEpisode(std::string_view configJsonV1);
        std::string reset(std::uint64_t localEpisodeIndex);
        MissionEndpoint endpoint(std::uint64_t serviceId);
        MissionReferee referee();
        void step();
    };
    class MissionEndpoint {
        std::string observationJsonV1() const;
        std::string submitJsonV1(std::string_view actionsJsonV1);
    };
    class MissionReferee {
        std::string outcomeJsonV1() const;
        std::string truthJsonV1() const;
    };

reset returns an owned initial episode-key document. step advances exactly the configured integer number of native World steps. Endpoint methods return owned documents, never borrowed pointers to a World or buffers invalidated during reset. Stale endpoint/referee generations fail explicitly. Range obtains endpoint/referee handles; actors receive endpoint proxies only.

All three classes export explicit destructors, noexcept move construction/assignment and deleted copy operations. Parent movement preserves the episode; reset invalidates prior child generations. A retained closed/generation control block makes child access after parent destruction fail before World access. New C++ APIs throw std::invalid_argument for malformed input and std::runtime_error for closed/stale/busy contexts; C adapters set last error and return existing status values. Ordinary native command refusal stays in a receipt. Python raises fsim.Error for API/lifetime failures and returns rejection receipts for ordinary native refusal.

In include/fsim/fsim_mission_env.h, define separate opaque fsim_mission_episode, fsim_mission_endpoint and fsim_mission_referee types. Return int using existing enum fsim_status values from fsim_c.h:

    int fsim_mission_episode_create(const char*, size_t, fsim_mission_episode**);
    void fsim_mission_episode_destroy(fsim_mission_episode*);
    int fsim_mission_episode_reset(fsim_mission_episode*, uint64_t local_episode_index);
    int fsim_mission_episode_step(fsim_mission_episode*);
    int fsim_mission_endpoint_open(fsim_mission_episode*, uint64_t service_id, fsim_mission_endpoint**);
    int fsim_mission_referee_open(fsim_mission_episode*, fsim_mission_referee**);
    void fsim_mission_endpoint_destroy(fsim_mission_endpoint*);
    void fsim_mission_referee_destroy(fsim_mission_referee*);
    int fsim_mission_endpoint_submit(fsim_mission_endpoint*, const char*, size_t);
    int fsim_mission_endpoint_observation_json(fsim_mission_endpoint*, char*, size_t, size_t*);
    int fsim_mission_endpoint_receipts_json(fsim_mission_endpoint*, char*, size_t, size_t*);
    int fsim_mission_referee_outcome_json(fsim_mission_referee*, char*, size_t, size_t*);
    int fsim_mission_referee_truth_json(fsim_mission_referee*, char*, size_t, size_t*);

Configuration/actions are UTF-8 pointer plus byte count. Document reads report required bytes including the NUL terminator; null buffer/zero capacity is a successful non-consuming size query, insufficient capacity returns FSIM_INVALID_ARGUMENT without a partial document. Destroy accepts null; closed/stale contexts return FSIM_INVALID_ARGUMENT, busy returns FSIM_ERROR and a live handle of the wrong principal kind returns FSIM_NOT_ALLOWED. Each sets explicit last error. Destroyed C handles must not be reused; separately live child handles remain safe after parent destruction. reset's initial episode key is in the first endpoint observation. submit queues and stores an owned acknowledgment document, read through receipts_json without re-submitting.

At tick T, endpoint submissions validate generation/tick/schema and enqueue only. step closes T, dispatches in stable service-ID/request-ID order, advances the configured native steps, and freezes T+1. Late/future ticks reject; absent services wait. One serialized Range boundary covers reset, step and submission. A duplicate returns the original queued acknowledgment; latest native status arrives in the snapshot/receipt stream.

JSON IDs are unsigned decimal strings; unavailable numbers are null plus validity; DMS bytes are bounded base64. Native action defaults apply only to explicitly omitted optional fields. Upper-layer arbitration is recorded in example logs, not claimed by SDK receipts.

In python/fsim/mission.py mirror MissionEpisode, MissionEndpoint and MissionReferee. No actor endpoint property returns World or Referee. Extend python/fsim/isolation.py's existing AgentProcess proxy with a mission-endpoint whitelist and retained truth-side-channel guard. Strict isolation refuses published/recorded episodes and Range/referee handles. Viewer/recording opt-in is diagnostic and explicitly non-isolated.

### Composition fixture, upper layer

In examples/python/ma_composition/contracts.py:

    class MAComponent:
        def reset(self, context): ...
        def propose(self, snapshot): ...
        def feedback(self, receipts): ...
    class MASystem:
        def begin_episode(self, context): ...
        def tick(self, endpoint): ...
        def end_episode(self, outcome): ...

These are example-only Python contracts, not new SDK plugin vtables. The system owns legal blackboard, component timing, arbitration and lifecycle. An outcome handed to a decision component contains its permitted mission feedback, not raw referee truth.

### Opt-in policy interface

In include/fsim/EngagementPolicy.h define EngagementPolicyV1 with policyId/revision (uint64), strikePolicy (existing comm::StrikePolicy), requestValidityS (positive double), authorizerRoleIds and activationRoleIds (vectors of uint32). Eligibility always uses existing own-designation and native selected-store checks.

    bool loadEngagementPolicy(World&, std::uint32_t platform,
                              const EngagementPolicyV1&, bool initiallyActive,
                              std::string* error);
    std::uint64_t activateEngagementPolicy(Agent& c2, comm::SystemId destination,
                                          std::uint64_t policyId, std::uint64_t revision,
                                          comm::DmsReason* why);

Loading is Range-only. Activation requires an authenticated role with existing kWeapons permission and explicit membership in the profile's activationRoleIds. Approving roles must both approve strikes under current RBAC and belong to authorizerRoleIds. Reserve local native DMS topic 18 for the versioned activation command, with normal CommandStatus responses; this is not a normative ASK wire-topic number.

C/Python mirror these operations through a Range-only JSON profile loader and a typed C2 activation helper. The mission action kind c2.activate_policy calls the same helper, never raw free-form native-topic injection. No profile or initiallyActive=false retains legacy behavior until an authorized activation arrives.

## M0: Freeze baseline and complete contract tables

**Files:** Design/plan above; docs/agent-boundary.md; docs/sdk/agents.md; new docs/sdk/ma-composition.md, mission-training.md and ma-contract.schema.json; baseline artifacts under D:/ENV/tmp/ma-training-baseline.

**Consumes:** Delivered main baseline baf476f841970f1f3c90b8de0bf630e8a5478738 and its evidence. Partial compatibility artifacts are under D:/ENV/tmp/viewer-strike-delivery-20261012/compatibility; their remaining coverage is listed in the vertical allocation.
**Produces:** Frozen public compatibility artifact, schema v1, service binding and lifetime/error contracts.

- [x] Review, verify, separately commit and push the Viewer/engagement baseline on main: baf476f841970f1f3c90b8de0bf630e8a5478738. Full Release run: 652/653, with the sole module-registration failure corrected and architecture checks revalidated in both presets; final headless selection 11/11 and complete fullscreen native strike passed. See the dated evidence for counts, limits and reproduction.
- [ ] Capture HEAD, dirty diff, protected model/card/control hashes, native/Python exports, CTest inventory, existing public type/vtable metadata, and old-header consumer binaries with dependency/data manifests.
- [ ] Define MA system/service/component/actor/platform IDs, scopes and episode generations. The existing MaCore entity ownership and moved local ASK reference path are clarified in ADR-31; explicit configured service contracts remain to be frozen.
- [ ] Finish every observation/action kind's fields, availability rules, native mapping, unknown-field/error behavior and sizes. Add round-trip fixtures for IDs above 2^53, unknown numeric reports and non-UTF-8 message bytes. Use JSON Schema 2020-12 as a documented dialect; no new runtime dependency is required.
- [ ] Specify configuration v1: base_seed, env_id, scenario recipe, dt/frame_skip/decision_steps, explicit service bindings, role/net/package setup, completed-request retention ticks, maximum in-flight requests/document bytes, authorization profile and task criterion. Initial defaults are workers=1 and publish=false; diagnostic names/paths include env and generation; batch worker budget is explicit.
- [ ] Define public PIMPL ownership/move/error behavior and C functions before workers edit bindings. All new symbol names enter explicit SDK boundary classifications.
- [ ] Validate English documents, local links and schema fixtures; classify every proposed endpoint method as Range, Agent or Referee. Do not call this formal ASK compliance.
- [ ] Review and commit the accepted contract slice separately on main.

**Gate:** A fresh implementer can construct each request/report and identify its authority without guessing. Current baseline failures are listed rather than attributed to this plan.

## M1: Add explicit MA-service ownership

**Files:** New MAService.h and MaServiceBindings.h/.cpp; src/session/World.h/.cpp, MaCore.h/.cpp; src/sdk/World.cpp; session build registration; tests/test_ma_service_binding.cpp.

**Consumes:** M0 identities and bindings.
**Produces:** bindMAService and deterministic capability/DMS ownership for configured mission episodes.

- [ ] Add failing tests and register ma_service_binding in tests/CMakeLists.txt with ma-training label and timeout: reverse Agent creation order and obtain identical configured grants; reject duplicate exclusive capability/DMS ownership; reject unminted/foreign controllers; preserve existing single-service behavior.
- [ ] Run the focused production-path test and capture the traversal-order or missing-binding failure before implementation.
- [ ] Store validated bindings with the owning platform/system, resolve C2 designations through them and remove traversal-order selection only from the explicitly configured path.
- [ ] Keep activities tied to the configured owner; unauthorized update/cancel and FA/C2-held capability requests retain existing rejection reasons.
- [ ] Verify platform-shared reports versus service-owned activity access, system removal, endpoint invalidation and re-creation.
- [ ] Run ctest --preset ucrt64-headless -R "^ma_service_binding$" --no-tests=error --output-on-failure and existing Agent/C2/ABI gates. Review and commit independently.

**Gate:** Multiple services have explicit semantics; many internal components behind one service do not require new SDK identities.

## M2: Complete Agent parity and legal reports

**Files:** New MAReports.h and session/MaReports.h/.cpp; Agent.h; sdk/Agent.cpp and fsim_mission_c.cpp; Python agent.py/_native_mission.c; tests/test_ma_reports.cpp; python/tests/test_ma_reports.py; boundary tables.

**Consumes:** M0 report schema; M1 configured ownership.
**Produces:** Equivalent own contacts/countermeasure/launch-zone interfaces and actor-safe report snapshots.

- [ ] Add failing parity/permission cases and register ma_reports with label/timeout for own contacts, inventory, dispense and own-designation launch zone across C++/C/Python.
- [ ] Test foreign platform/designation/controller refusal and continued rejection of global engagement/decoy/seeker-truth APIs.
- [ ] Add Agent wrappers without changing its data layout or existing C/Python type owners. Reports come from equipment/Agent paths, not filtered copies of global truth records.
- [ ] Add report times, provenance and availability. Available v1 fields are launcher-known release/inventory/support reports; absent weapon telemetry stays unavailable.
- [ ] Test that changing an undetected entity's truth or a global terminal outcome cannot change actor reports before a modeled observation reaches it. A common upper-layer wrapper may not leak another platform/C2 view before modeled delivery. Delivered sensor/link changes must change the correct report.
- [ ] Run ctest --preset ucrt64-headless -R "^ma_reports$" --no-tests=error --output-on-failure and python_sdk with the staged CPython. Refresh/render/check boundary classifications with both optional Vision and core exports.
- [ ] Review and commit the parity/report package.

**Gate:** All languages have the same safe capability set; no invented flyout or kill knowledge.

## M3: Implement the bounded authorization profile

**Files:** New EngagementPolicy.h and session/EngagementPolicy.h/.cpp; MaCore, MaConsentLedger and AgentRules; C++/C/Python DMS adapters; tests/test_ma_authorization.cpp.

**Consumes:** M0 authorization.v1 schema, M1 identities and M2 legal report semantics.
**Produces:** Preloaded opt-in criteria, typed authorized activation and correlated pending-request lifecycle.

- [ ] Add failing tests and register ma_authorization with label/timeout for unauthorized profile activation, invalid/unsupported criteria, unknown required evidence, and incorrect request/activity/profile/generation association.
- [ ] Implement initial role/authorizer, valid designation/store, approval mode and simulation-time validity checks. Require Range-configured policy-activation authority; do not enlarge existing role bitsets/defaults.
- [ ] Specify and implement cancel/expiry/profile-replacement behavior: requests awaiting approval become terminal when invalidated; active releases keep their accepted native transaction semantics.
- [ ] Test denial, delayed delivery, loss, duplicate/late answers, canceled requests, profile revision changes and ended strikes. Revalidate after approval delay, including lapsed designation and lost/expended store. Exactly one valid answer may authorize exactly one pending activity.
- [ ] Run ctest --preset ucrt64-headless -R "^ma_authorization$" --no-tests=error --output-on-failure plus existing permission/consent tests.
- [ ] Compare unconfigured scenario traces with the baseline; document the supported subset and review/commit.

**Gate:** All policy changes are explicit and authorized. No IFF/geographic/full-ASK capability is claimed without its model and tests.

## M4: Own complete independent mission episodes

**Files:** New src/env/MissionEpisode.h/.cpp; env build registration; tests/test_mission_episode.cpp; tests/fixtures/mission_training/episode.json.

**Consumes:** M0 configuration/generation/seed rules and M1 service bindings.
**Produces:** Internal episode owner, full fresh-world reset and generation-checked endpoint contexts.

- [ ] Add failing tests and register mission_episode with label/timeout: A's weapon/damage/message cannot affect B; resetting A never changes B's sampled sequence; stale handles/intents fail after reset.
- [ ] Construct one World per episode and apply the deterministic scenario/role/net/store/binding recipe. Each episode owns its task state and cursors.
- [ ] On reset reconstruct the entire owned World. Do not call legacy resetVehicle as a substitute or clear existing vehicle authority semantics.
- [ ] Test fresh-equivalence after crew loss, fire, in-flight weapon, decoy, consumed store, pending C2 task/consent, queued message and removed entity. Compare the same env/local episode seed, normalizing only generation metadata, not native outcomes.
- [ ] Test parent destruction with live children, parent move, two uniquely published diagnostic episodes and reset-safe recording paths; neither handle access nor artifact naming may rely on freed World state.
- [ ] Derive RNG per env/local episode/stable stream; test worker and asynchronous-reset schedule invariance.
- [ ] Run ctest --preset ucrt64-headless -R "^mission_episode$" --no-tests=error --output-on-failure plus the legacy reset-preservation case.
- [ ] Measure initialization time and memory without optimizing ownership yet. Review and commit.

**Gate:** Every episode is independently reproducible and has no mutable cross-episode state.

## M5: Expose mission snapshots and asynchronous actions

**Files:** MissionEpisode public/C headers and adapters; MissionActions/MissionObservation; new Python mission module/native adapter; existing native registration files; tests/test_ma_observation.cpp, test_ma_actions.cpp, test_mission_c_abi.c; python/tests/test_mission.py.

**Consumes:** Core: frozen generic M0, M1, noncombat M2 reports and full M4 episode owner. Combat handlers additionally require combat M2 and M3.
**Produces:** Public MissionEpisode/Endpoint/Referee interfaces and schema-valid receipts.

Claude owns the core gateway and noncombat handlers; Codex owns combat payloads, native validation/correlation and combat receipts. Freeze handler contracts before facade integration. A core-only pass does not complete the full M5 gate below.

- [ ] Add failing actor/referee handle-kind and generation tests before exposing public entrypoints; register ma_observation, ma_actions and mission_c_abi with ma-training labels/timeouts in this package.
- [ ] Freeze one legal snapshot per decision tick, ingest the DMS inbox once and return owned immutable documents on repeated reads.
- [ ] Queue requests at submit and dispatch through the bound Agent's existing VI/MS/DMS/C2 method at Range.step in fixed order. Test reversed service/proxy arrival order, wrong decision tick and missing-service wait. Preserve role/target checks; never use Range consent or entity truth to fulfill actor requests.
- [ ] Implement scoped request deduplication and bounded terminal retry retention, preserving pending/executing correlations until explicit closure. Test approval beyond retry window, late cancellation of long activities, terminal tombstone expiry, capacity refusal, identical retry, changed-body reuse, stale retry, unknown kind, non-finite arguments and repeated strike/send.
- [ ] Map native IDs and reasons into receipts. Test that pending, accepted and release Done cannot be mistaken for actual mission success.
- [ ] Implement C two-call buffer reads and handle-kind validation, Python wrappers and isolated endpoint proxy with publication/recording guard. Test buffer size queries do not consume messages or submit twice. Keep existing plugin vtables/native type owners and the non-free-threaded abi3 target Py_LIMITED_API=0x030B0000 intact.
- [ ] Run ctest --preset ucrt64-headless -R "^(ma_observation|ma_actions|mission_c_abi)$" --no-tests=error --output-on-failure and python_sdk. Check exports/boundary and old consumers.
- [ ] Review and commit the public environment/action package.

**Gate:** Upper-layer MA can execute local, peer and C2 actions through one consistent public contract without acquiring referee access.

## M6: Demonstrate a composite upper-layer MA

**Files:** examples/python/ma_composition directory; python/tests/test_ma_composition.py; composition documentation.

**Consumes:** M5 core endpoint contract and receipts for noncombat composition; M5 full for complete composition scenarios.
**Produces:** Replaceable component fixture, deterministic reference coordinator and lifecycle hooks.

- [ ] Add failing example tests for shared snapshot identity, single inbox ingestion, conflicting intents, activity update/cancel ownership and reset of all component state.
- [ ] Implement two policy fixtures, a small BT executive and a pure planning function using example-only contracts.
- [ ] Implement configured resource priority/ownership and stable component-ID tie-breaks; retain proposal/arbitration/submission/receipt provenance separately.
- [ ] Run components at declared decision periods. Feed SDK refusals back to their originating components; no internal coordinator may override FA/C2/MS authority.
- [ ] Reset blackboard, BT execution state, planner/RNN placeholder memory, RNG streams, queues and activity mappings at generation changes.
- [ ] Run the example with an installed public SDK and compare repeated traces with identical inputs. Run python_sdk composition cases.
- [ ] Review and commit the example; do not add a BT/ML dependency or production MA runtime.

**Gate:** MA composition and policy replacement are demonstrated without equating one platform, one service and one actor.

## M7: Add batch scheduling, team outcomes and reward contracts

**Files:** New MissionBatch/MissionTask files and public header; C/Python environment adapters; tests/test_mission_training.cpp; python/tests/test_mission_training.py; training documentation.

**Consumes:** Core: M4 independent episodes, M5 core endpoints and M6 noncombat lifecycle example. Combat criteria additionally consume M5 combat and frozen native outcome definitions; full M7 requires both streams and M6 full.
**Produces:** Batch mission sampling with per-role/episode results and separate referee reward/critic streams.

Claude owns scheduling, role lifecycle, noncombat task criteria, referee-stream transport, communication costs and the training adapter. Codex owns combat outcomes, combat task criteria and combat rewards. The full gate below requires their joint acceptance.

- [ ] Add failing tests and register mission_training with label/timeout for team reset flags, dead member with continuing teammates, C2 role without aircraft, variable service topology and one task instance per episode.
- [ ] Implement batch scheduling over independent episode owners. No task/plugin mutable object or RNG is shared between episodes.
- [ ] Implement per-role active/terminated/truncated/final snapshot/bootstrap eligibility and separate episode_done/reason.
- [ ] Handle NextStep reset before any old action or world step; test no old release/message/damage. SameStep keeps old final results distinct from new initial observations.
- [ ] Define versioned referee outcomes/rewards for miss, damage, crew loss, incapacitation, collision, friendly damage and mission success. Optional shaping/resource/communication costs are explicit configuration.
- [ ] Keep critic truth separate from actor observations/infos and mask construction. Test actor-view equivalence under hidden-truth changes.
- [ ] Add a dependency-free Python training adapter contract with component registration/reset hooks. Leave framework-specific optimizers/adapters to the upper layer.
- [ ] Run ctest --preset ucrt64-headless -R "^mission_training$" --no-tests=error --output-on-failure, Python tests and deterministic worker/reset schedule matrix. Measure throughput, initialization, memory and episode reclamation over a fixed recorded workload.
- [ ] Review and commit.

**Gate:** Batch, service and actor counts are distinct; terminal transitions and rewards are reliable enough for learning.

## M8: Prove the installed joint chain and deliver

**Files:** tests/test_ma_joint.cpp; tests/fixtures/mission_training joint scenarios; tests/package_consumer/ma_cpp.cpp, ma_c.c, ma_python.py; package CMake/run gates; new docs/sdk/ma-training-validation.md; status/navigation documentation; viewer acceptance tool only where public source selection is needed.

**Consumes:** All previous packages.
**Produces:** J1-J10 evidence, compatibility report, reproducibility manifest and finite completion.

- [ ] Register ma_joint and mission_package_consumer with ma-training labels and bounded timeouts. Earlier packages have already registered their own focused gates; verify actual discovery and reject empty matches.
- [ ] Run J1-J9 using public SDK endpoints: measured P2P tracks, typed C2 task/RBAC/consent, native flyout and actual outcomes. Include approved/denied/delayed/lost approval, direct-control ownership and package/lost-link cases.
- [ ] Compare in-process and strict isolated actor endpoint traces with truth publication/recording disabled. Test the proxy rejects Range/referee handles and stale generations. Never send World/Referee, scenario answers or preflight truth into the policy process.
- [ ] Install core SDK/runtime data and Python into a separate build/mission-package-consumer owned prefix, never the existing package-consumer work directory. Compile C++/C99 consumers and run Python using only that prefix with source/build fallbacks removed. If shared staging is unavoidable, apply the same explicit CTest resource lock to both gates.
- [ ] Check DLL/data/wrapper/native-module provenance, all retained exports/layouts/vtables and old-header binaries against the new library.
- [ ] Run the current full ucrt64-headless CTest suite. Build ucrt64-release and run relevant renderer regressions plus one complete fullscreen public-SDK-driven joint strike scenario as a separately labeled non-isolated diagnostic.
- [ ] Preserve carrier -> missile -> actually damaged target focus; capture exactly launch + 1.0 s and hit + 0.3 s; finish the scenario. Miss/denied scenarios finish without a false target-hit capture or explosion.
- [ ] Decode/inspect captures and native recording/events, verify no early destructive effects and compare live/headless native results. No repeated screenshot polling.
- [ ] Record seed/aspect matrix, role/service/component counts, dt/cadence, build/compiler/package hashes, test inventory/results, failures and measurement limits in English.
- [ ] Update current capability/status documents only with achieved evidence. Independently review, commit bounded final changes on main, and push only when requested/authorized.

**Gate:** J1-J10 and compatibility pass. Stop architecture expansion and move real MA/DRL integration to the upper-layer project.

## Verification commands and scheduling

Use the existing launcher/presets and actual discovered test inventory. Focused names in this plan are new registrations, not claims that they exist today.

    .\fsim.cmd --preset ucrt64-headless configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe -DFSIM_WARNINGS_AS_ERRORS=ON
    .\fsim.cmd --preset ucrt64-headless build --parallel 12
    ctest --preset ucrt64-headless -L ma-training --no-tests=error --output-on-failure
    ctest --preset ucrt64-headless -R "^(sdk_boundary|c_abi|adapter_compatibility|world_adapter_compatibility|batch_recording_adapter_compatibility|python_sdk|package_consumer|mission_package_consumer)$" --no-tests=error --output-on-failure
    ctest --preset ucrt64-headless --no-tests=error --output-on-failure
    .\fsim.cmd --preset ucrt64-release build --parallel 12
    ctest --preset ucrt64-release -R "^(strike_capture|showcase_native|viewer_screenshot|vision|vision_c_abi)$" --no-tests=error --output-on-failure
    D:/ENV/python/3.13/python.exe tools/viewer_perf/strike_acceptance.py --build build/ucrt64-release --out D:/ENV/tmp/ma-training-viewer

The last command is the existing native visual baseline. M8 adds a public-SDK joint source option or a separate compatible public-source runner; do not label the current internal Showcase as public consumer acceptance.

Every focused command in M1-M7 also requires --no-tests=error and an exact discovered test count greater than zero. The active feature owner holds the exclusive native configure/build/test and GUI execution slots and records the tested HEAD/diff. Claude owns generic integration and compatibility gates; Codex owns combat integration and acceptance. Run independent read-only reviews in parallel; never run competing viewers or CMake writers. Broaden tests only after changed behavior or a new failure justifies it. No final current full-suite claim until the final current build completes.

## Deferred scope and exit rule

Not required: production MA orchestration, neural-network training, universal BT/planner packages, distributed rollout cluster, shared-World partitioning, world-pool/snapshot optimization, new missile downlink equipment, full ASK wire certification, cryptographic identity, aircraft/missile calibration, shadow/material changes or unrelated source splitting.

Record performance instead of inventing a throughput target. Optimize only a measured bottleneck in a separately scoped follow-up that preserves the accepted episode and permission semantics.

Each package ends with reviewed tests and a main commit when execution is authorized. Do not reuse unfinished visual work as a clean baseline or combine unrelated existing modifications into MA commits.

The program ends after M8: the SDK can host upper-layer composite MA decision systems, P2P/C2 interactions and independently reproducible training episodes. Upper-layer MA implementation/training is a subsequent project deliverable.
