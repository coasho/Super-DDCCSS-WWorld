# Claude MA Training Implementation Plan

> **For Claude:** This plan contains Claude assignments only. Follow the [Claude work allocation](2026-10-12-ma-system-training-work-allocation.md): each assigned feature includes native implementation, language adapters, tests, examples, documentation and evidence. Shared paths and native/GUI execution use exclusive windows. Persistent context is in the [project memory](../../../.claude/CLAUDE.md).

**Goal:** Make the SDK support upper-layer composite MA systems and correct local/P2P/C2 training episodes through public, compatible interfaces.

**Architecture:** Keep formal MA decision runtime in the upper layer. Add explicit MA-service binding, legal reports and a separate mission-episode API over the existing execution core. Initially isolate one World per episode, and validate with deterministic composite fixtures and installed consumers.

**Tech Stack:** C++17, C99/C11 adapters, CPython stable ABI 3.11+, existing CMake 3.25+ UCRT64/Ninja presets, native JSBSim and existing communication/mission simulation.

**Spec:** [MA system integration design](../specs/2026-10-12-ma-system-training-readiness-design.md).

**Delivered baseline:** baf476f841970f1f3c90b8de0bf630e8a5478738, verified and pushed to main. See [current Viewer evidence](../../sdk/viewer-validation.md).

**Progress reviewed:** 2026-10-15, main `6e8b4aa`. Generic M0 was delivered in 1874091 and corrected in 182ac58; M1 in 990554b; noncombat M2 in fb23a9b. Follow-ups 9b06c79 and 2bad550 address binding transitions, foreign-World Agent rejection, peer provenance and the qualified activity-preservation contract. The native-state reset fixture was supplied in 4406ff1. M4 is implemented as the internal episode owner (`8254d7e`, `c507978`, `54b7fb9`) and has passed independent acceptance. M5 core was implemented in `6e8b4aa` after its prerequisites were verified at `08e3afe` and awaits independent acceptance; full M5 (extension handlers), the first-package acceptance and M6-M8 remain pending.

## Global Constraints

- Work on main. Do not create branches or worktrees.
- Formal MA, BT engines, planners and learned policies belong in the upper layer. SDK examples demonstrate composition only.
- Preserve existing public signatures, layouts, vtables, enum numbers, errors, legacy defaults and flight VecEnv shapes.
- New interfaces are additive and versioned; old-header binaries must run with the new library.
- Preserve Vehicle.reset authority/task/override/C2 ownership semantics and existing MA retirement/history behavior.
- SDK owns sensing, links, release, flight, guidance, fuzes, collisions and damage. Do not add client-side physics or self-destruction scripts.
- Existing physics/cards/control/equipment defaults are unchanged. New report/policy/environment configuration is explicit.
- All communication with Claude, maintained files and commit messages are English.
- Tools/caches are under D:/ENV; TMP, TEMP and TORCH_EXTENSIONS_DIR are D:/ENV/tmp.
- Read upstream support matrices before changing a toolchain or dependency. This plan requires no new environment installation.
- Raw background output goes to a file without a trailing buffered filter. One job has one background entry.
- Viewer tests are fullscreen, run complete scenarios, and retain exactly two event-relative strike captures.
- Distinguish delivered M0/M1/noncombat M2 and accepted M4 from implemented M5 core awaiting independent acceptance, pending full M5, composition, training and final acceptance. Contract fixtures alone do not establish runtime gateway behavior.

## Review Focus

- An old episode reference arrives after reset: reject it without affecting new state.
- A second MA service or internal component consumes shared state: explicit ownership must prevent traversal-order grants and destructive inbox races.
- A report is unknown, stale or not delivered: preserve that condition; never fill it from referee truth.
- An external native receipt is delayed, duplicated or stale: preserve its correlation and authority; transport never fabricates execution or success.
- One team member ends while others continue, or a next-step reset is pending: correct final observations, bootstrap flags and zero old-episode side effects.

## Dependency and stop gates

| Package | Dependencies | Reviewable output |
| --- | --- | --- |
| M0 Baseline/contracts | None | Current compatible baseline and signed-off interface/spec vocabulary |
| M1 MA-service ownership | M0 | Explicit owner/binding path for new mission scenarios |
| Noncombat M2 Agent parity/reports | Generic M0; M1 for multi-service cases | C++/C/Python legal navigation/contact/track/task/authority reports |
| M4 Episode lifecycle | M0-M1 | Fresh independent World per episode; stale-reference rejection |
| M5 core | Generic M0, M1, noncombat M2 and full M4 | Public role-scoped observations, generic gateway and noncombat asynchronous requests |
| M6 noncombat | M5 core | Upper-layer gateway/components/BT/planner fixture with deterministic arbitration |
| M7 core | M4, M5 core and M6 noncombat | Independent batch sampling, role lifecycle, noncombat criteria and referee-stream transport |
| M8 generic delivery | Claude packages and current compatibility inputs | Installed consumers, isolated actor path and generic joint evidence |

Noncombat M2 and M4 can proceed independently after their inputs are frozen. Do not start downstream work against undocumented provisional signatures.

M5 core and noncombat composition do not require M3. Full M4 consumes a supplied native-state reset fixture using existing SDK behavior; that fixture need not wait for new policy implementation. Interim smoke checks do not complete full M4 or M5 core. Full M5/M6/M7/M8 acceptance retains every case in the neutral design and consumes the required integration inputs below.

## Planned file map

| Area | Existing integration files | Added or planned files |
| --- | --- | --- |
| Contracts | docs/agent-boundary.md, docs/sdk/agents.md, docs/sdk/boundary.toml | docs/sdk/ma-composition.md, docs/sdk/mission-training.md, docs/sdk/ma-contract.schema.json |
| Binding | src/session/World.h/.cpp, MaCore.h/.cpp, src/sdk/World.cpp | include/fsim/MAService.h, src/session/MaServiceBindings.h/.cpp |
| Legal noncombat reports | Navigation/contact/track/task/authority portions of include/fsim/Agent.h, src/sdk/Agent.cpp, fsim_mission_c.cpp, python/fsim/agent.py, python/src/_native_mission.c | include/fsim/MAReports.h, src/session/MaReports.h/.cpp |
| Episode/actions | src/env/CMakeLists.txt, src/sdk/CMakeLists.txt | src/env/MissionEpisode.h/.cpp, MissionActions.h/.cpp, MissionObservation.h/.cpp |
| Public environment | SDK/native registration files, python/CMakeLists.txt, python/fsim/__init__.py | include/fsim/MissionEpisode.h, fsim_mission_env.h, src/sdk/MissionEpisode.cpp, fsim_mission_env_c.cpp, python/src/_native_mission_env.c/.h, python/fsim/mission.py |
| Composition | Existing AgentProcess proxy and examples documentation | examples/python/ma_composition/contracts.py, components.py, system.py, run.py, README.md |
| Training batch | Environment/SDK/native build registration | src/env/MissionBatch.h/.cpp, MissionTask.h/.cpp, include/fsim/MissionBatch.h |
| Verification | tests/CMakeLists.txt, tests/mission_package_consumer/CMakeLists.txt, cmake/RunMissionPackageConsumer.cmake, tools/boundary | Focused tests, joint scenario fixtures and installed C++/C99/Python consumers described below |

This map contains Claude integration paths; the [work allocation](2026-10-12-ma-system-training-work-allocation.md) records its complete feature scope. M0/M1/noncombat M2 files now exist; later filenames remain planned seams. The delivered package sections below identify actual files. Keep existing runtime type owners and adapter conventions. Consume frozen external contracts at agreed schema/handler registration points, using exclusive shared-file handoffs.

## Shared proposed interfaces

The service binding interface is delivered in M1. The mission environment family and composition contracts below remain proposed. Existing Agent/World/VecEnv interfaces remain source and binary compatible. The [public contract](../../sdk/mission-training.md) and exported headers are authoritative for implemented signatures.

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

In python/fsim/mission.py mirror MissionEpisode, MissionEndpoint and MissionReferee. No actor endpoint property returns World or Referee. Extend python/fsim/isolation.py's existing AgentProcess proxy with the mission endpoint's callable-method list and existing diagnostic truth-output checks. Strict actor process isolation refuses published/recorded episodes and Range/referee handles. Viewer/recording opt-in is diagnostic and explicitly non-isolated.

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

## Required integration inputs

Consume frozen extension schemas, request-kind/handler registrations, native receipt/outcome contracts, full native-state reset fixtures and current joint/visual evidence at the agreed integration seams. These are inputs, not Claude implementation assignments.

Missing extensions fail explicitly. Missing fixtures/evidence remain pending gates. Do not implement provisional semantics or manufacture acceptance to satisfy an unavailable input. Core work proceeds independently of new policy implementation; full-program gates follow the complete neutral design.

## M0: Freeze baseline and complete generic contract tables

**Status:** Delivered in 1874091; contract corrections in 182ac58. This is a contract and compatibility baseline, not a mission gateway implementation.

**Files:** docs/sdk/mission-training.md, ma-composition.md, ma-contract.schema.json and ma-compatibility-baseline.md; ADR-31 and SDK references; tools/ma_contract; tests/fixtures/mission_training/contract. Baseline artifacts are under D:/ENV/tmp/ma-training-baseline.

**Consumes:** Delivered main baseline baf476f841970f1f3c90b8de0bf630e8a5478738 and its evidence, including the supplied partial compatibility artifacts.
**Produces:** Frozen public compatibility artifact, schema v1, service binding and lifetime/error contracts.

- [x] Capture the dated compatibility baseline: protected inputs, native/Python exports, public layouts/vtables, test inventory and frozen C++, C99 and Python consumers with provenance.
- [x] Freeze identities, generations, service binding, ownership/move/lifetime/errors, configuration, native mappings, document limits and principal classifications.
- [x] Deliver the documented JSON Schema 2020-12 subset, round-trip/refusal fixtures and standard-library checker, including large IDs, unknown numeric reports and binary messages.
- [x] Commit the generic contracts and corrections separately on main. Runtime implementation and later compatibility verification remain package-specific gates.

**Gate:** Delivered contract vocabulary and dated evidence are in the [public contract](../../sdk/mission-training.md) and [compatibility baseline](../../sdk/ma-compatibility-baseline.md). Hand-written contract fixtures do not establish runtime endpoint behavior or full-program acceptance.

## M1: Add explicit MA-service ownership

**Status:** Delivered in 990554b; transition, foreign-World and provenance fixes in 9b06c79; contract qualification and mixed-controller regression in 2bad550.

**Files:** include/fsim/MAService.h and fsim_c.h; src/session/MaServiceBindings.h/.cpp, World.h/.cpp, MaCore.cpp and AgentRules; src/sdk/World.cpp, Agent.cpp and fsim_links_c.cpp; Python world.py/_native_dms.c; service-binding/C ABI/Python tests and the installed consumer.

**Consumes:** M0 identities and bindings.
**Produces:** bindMAService and deterministic capability/DMS ownership for configured mission episodes.

- [x] Bind explicit capability/DMS owners through public C++/C/Python interfaces; retain legacy behavior on unbound platforms.
- [x] Enforce native grants and service-owned command/update/cancel/DMS paths; reject conflicting, unminted and foreign-World bindings.
- [x] Cover binding transitions, configured creation-order independence, platform removal, peer provenance and installed consumers with registered regressions.
- [x] Qualify activity preservation: a capability-wide revoke cancels the owner's activities too when another controller held a grant or had a live/waiting policy activity on that capability. Bind all services before they act.
- [x] Commit the implementation and reviewed follow-ups on main. Subsequent packages must preserve these native semantics.

**Gate:** Multiple services have explicit semantics; many internal components behind one service do not require new SDK identities.

## M2: Complete noncombat Agent parity and legal reports

**Status:** Noncombat slice delivered in fb23a9b; peer/heartbeat provenance corrected in 9b06c79. This does not complete equipment parity or mission gateway snapshots.

**Files:** include/fsim/MAReports.h, Agent.h and fsim_c.h; src/session/MaReports.h/.cpp and AgentRules; src/sdk/Agent.cpp, fsim_mission_c.cpp and fsim_links_c.cpp; Python agent.py/_native_mission.c; tests/test_ma_reports.cpp, test_ma_c_abi.c, python/tests/test_ma_reports.py and tests/mission_package_consumer; boundary tables.

**Consumes:** M0 report schema; M1 configured ownership.
**Produces:** Equivalent legal navigation/contact/track/task/authority interfaces and actor-safe report snapshots.

- [x] Export permission-scoped navigation, passive contacts, tracks, link tracks, peers/heartbeats, tasks and authority reports across C++/C/Python.
- [x] Preserve sample/receive times, provenance, age and availability, including independent position and heartbeat provenance. Unknown/stale/unavailable reports remain explicit.
- [x] Add permission/parity, hidden-truth isolation and installed public C++/C99/Python consumer regressions; classify the added SDK boundary entries.
- [x] Commit the noncombat report package and reviewed provenance corrections on main. Equipment and full mission acceptance remain pending integration gates.

**Gate:** All languages expose the delivered noncombat report set without referee truth or invented flyout/kill knowledge. This gate is distinct from full M2 and full-program acceptance.

## M4: Own complete independent mission episodes

**Files:** New src/env/MissionEpisode.h/.cpp; env build registration; tests/test_mission_episode.cpp; tests/fixtures/mission_training/episode.json.

**Consumes:** M0 configuration/generation/seed rules and M1 service bindings.
**Produces:** Internal episode owner, full fresh-world reset and generation-checked endpoint contexts.

- [x] Add failing tests and register mission_episode with label/timeout: A's weapon/damage/message cannot affect B; resetting A never changes B's sampled sequence; stale internal contexts fail after reset. Public submitted-intent rejection is an M5 gate.
- [x] Construct one World per episode and apply the deterministic scenario/role/net/store/binding recipe. Each episode owns its task state and cursors.
- [x] On reset reconstruct the entire owned World. Do not call legacy resetVehicle as a substitute or clear existing vehicle authority semantics.
- [x] Consume the supplied full native-state fixture and test fresh-equivalence after crew loss, fire, in-flight weapon, decoy, consumed store, pending C2 task/consent, queued message and removed entity. Range may configure/inspect referee state; actor requests/releases retain existing Agent/C2 permissions, without Range consent injection to fulfill actor requests. Compare the same env/local episode seed, normalizing only generation metadata, not native outcomes.
- [x] Test parent destruction with live children, parent move, two uniquely published diagnostic episodes and reset-safe recording paths; neither handle access nor artifact naming may rely on freed World state.
- [x] Derive RNG per env/local episode/stable stream; test worker and asynchronous-reset schedule invariance.
- [x] Run ctest --preset ucrt64-headless -R "^mission_episode$" --no-tests=error --output-on-failure plus the legacy reset-preservation case.
- [x] Measure initialization time and memory without optimizing ownership yet. Review and commit.

**Gate:** Every episode is independently reproducible and has no mutable cross-episode state.

## M5 core: Expose mission snapshots and noncombat asynchronous actions

**Files:** MissionEpisode public/C headers and adapters; MissionActions/MissionObservation; new Python mission module/native adapter; existing native registration files; tests/test_ma_observation.cpp, test_ma_actions.cpp, test_mission_c_abi.c; python/tests/test_mission.py.

**Consumes:** Frozen generic M0, M1, noncombat M2 reports and full M4 episode owner; frozen external registration/receipt contracts where applicable.
**Produces:** Public MissionEpisode/Endpoint/Referee interfaces and schema-valid receipts.

Implement the core gateway and noncombat handlers. Freeze extension contracts before facade integration; consume supplied handlers without changing their native semantics. A core-only pass does not complete full M5 program acceptance.

Before exporting the facade, independently verify the accepted M4 inputs and the
[simulation/interface boundary prerequisites](../../architecture/design/simulation-interface-boundaries.md). Keep the
native setup callbacks internal and public state/transport layouts fixed. Do not implement formal decision systems
or training algorithms; fixtures only exercise the public interfaces. Disagreements or missing prerequisites must
be reported with evidence before dependent implementation proceeds.

- [x] Add failing actor/referee handle-kind and generation tests before exposing public entrypoints; register ma_observation, ma_actions and mission_c_abi with ma-training labels/timeouts in this package.
- [x] Freeze one legal snapshot per decision tick, ingest the DMS inbox once and return owned immutable documents on repeated reads.
- [x] Queue requests at submit and dispatch through the bound Agent's existing VI/MS/DMS/C2 method at Range.step in fixed order. Test reversed service/proxy arrival order, wrong decision tick and missing-service wait. Preserve role/target checks; never use Range consent or entity truth to fulfill actor requests.
- [x] Implement scoped request deduplication and bounded terminal retry retention, preserving pending/executing correlations until explicit closure. Test long-lived tasks beyond retry windows, late cancellation, terminal tombstone expiry, capacity refusal, identical retry, changed-body reuse, stale retry, unknown kind, non-finite arguments and repeated task/send requests. Consume supplied extension correlation fixtures at full integration.
- [x] Transport native IDs, states and reasons through versioned receipts. Test that queued/pending/accepted or native activity completion does not imply mission success; preserve frozen extension event/receipt semantics.
- [x] Implement C two-call buffer reads and handle-kind validation, Python wrappers and isolated endpoint proxy with publication/recording guard. Test buffer size queries do not consume messages or submit twice. Keep existing plugin vtables/native type owners and the non-free-threaded abi3 target Py_LIMITED_API=0x030B0000 intact.
- [x] Run ctest --preset ucrt64-headless -R "^(ma_observation|ma_actions|mission_c_abi)$" --no-tests=error --output-on-failure and python_sdk. Check exports/boundary and old consumers.
- [x] Review and commit the public environment/action package (committed on main; awaiting independent acceptance).

**Gate:** Upper-layer MA can execute local, peer and C2 actions through one consistent public contract without acquiring referee access.

## M6: Demonstrate a composite upper-layer MA

**Files:** examples/python/ma_composition directory; python/tests/test_ma_composition.py; composition documentation.

**Consumes:** M5 core endpoint contract and receipts for noncombat composition; accepted integration inputs for full composition scenarios.
**Produces:** Replaceable component fixture, deterministic reference coordinator and lifecycle hooks.

- [x] Add failing example tests for shared snapshot identity, single inbox ingestion, conflicting intents, activity update/cancel ownership and reset of all component state.
- [x] Implement two noncombat navigation/coordination policy fixtures, a small BT executive and a pure planning function using example-only contracts.
- [x] Implement configured resource priority/ownership and stable component-ID tie-breaks; retain proposal/arbitration/submission/receipt provenance separately.
- [x] Run components at declared decision periods. Feed SDK refusals back to their originating components; no internal coordinator may override FA/C2/MS authority.
- [x] Reset blackboard, BT execution state, planner/RNN placeholder memory, RNG streams, queues and activity mappings at generation changes.
- [x] Run the example with an installed public SDK and compare repeated traces with identical inputs. Run python_sdk composition cases.
- [x] Review and commit the example; do not add a BT/ML dependency or production MA runtime (M6 noncombat; awaiting independent acceptance; full composition scenarios consume full M5).

**Gate:** MA composition and policy replacement are demonstrated without equating one platform, one service and one actor.

## M7 core: Add batch scheduling, role lifecycle and noncombat criteria

**Files:** New MissionBatch/MissionTask files and public header; C/Python environment adapters; tests/test_mission_training.cpp; python/tests/test_mission_training.py; training documentation.

**Consumes:** M4 independent episodes, M5 core endpoints and M6 noncombat lifecycle example. Full M7 additionally consumes accepted extension outcomes/criteria and full M6 evidence.
**Produces:** Batch mission sampling with per-role/episode results and separate referee reward/critic streams.

Implement scheduling, role lifecycle, noncombat task criteria, referee-stream transport, communication costs and the training adapter. Extension outcome/criterion contracts are read-only integration inputs. Full program acceptance requires the complete joint evidence.

- [x] Add failing tests and register mission_training with label/timeout for team reset flags, dead member with continuing teammates, C2 role without aircraft, variable service topology and one task instance per episode.
- [x] Implement batch scheduling over independent episode owners. No task/plugin mutable object or RNG is shared between episodes.
- [x] Implement per-role active/terminated/truncated/final snapshot/bootstrap eligibility and separate episode_done/reason.
- [x] Handle NextStep reset before any old action or world step; test no old release/message/damage. SameStep keeps old final results distinct from new initial observations.
- [x] Define noncombat task criteria, completion/reason semantics and explicit communication/resource costs. Transport supplied versioned referee outcomes/rewards without redefining their native meaning. Keep every reward and critic stream outside actor observations and masks.
- [x] Keep critic truth separate from actor observations/infos and mask construction. Test actor-view equivalence under hidden-truth changes.
- [x] Add a dependency-free Python training adapter contract with component registration/reset hooks. Leave framework-specific optimizers/adapters to the upper layer.
- [x] Run ctest --preset ucrt64-headless -R "^mission_training$" --no-tests=error --output-on-failure, Python tests and deterministic worker/reset schedule matrix. Measure throughput, initialization, memory and episode reclamation over a fixed recorded workload.
- [x] Review and commit (M7 core; awaiting independent acceptance; rewards are pending because no reward definition is installed, and full M7 consumes accepted extension outcomes/criteria and full M6).

**Gate:** Batch, service and actor counts are distinct; terminal transitions and rewards are reliable enough for learning.

## M8 generic: Prove installed consumers and joint integration

**Files:** Generic portions of tests/test_ma_joint.cpp and tests/fixtures/mission_training; tests/mission_package_consumer/ma_cpp.cpp, ma_c.c, ma_python.py; package CMake/run gates; new docs/sdk/ma-training-validation.md; status/navigation documentation.

**Consumes:** Claude packages, accepted extension interfaces/fixtures and required current integration evidence.
**Produces:** Claude joint results, installed-consumer compatibility/provenance and the complete program evidence manifest.

- [x] Register ma_joint and extend the existing mission_package_consumer gate, retaining ma-training labels and bounded timeouts. Earlier packages have already registered their own focused gates; verify actual discovery and reject empty matches.
- [x] Run J1/J2/J4/J6/J7, generic J10 infrastructure and task/message/lifecycle cases for J3/J8 through public SDK endpoints. Cover measured P2P tracks, typed C2 tasks/RBAC, direct-control ownership and package/lost-link behavior. Consume supplied evidence for the remaining joint cases; do not mark absent cases complete (J5, J9 and the combat parts of J3/J8/J10 recorded pending in docs/sdk/ma-training-validation.md).
- [x] Compare in-process and strict isolated actor endpoint traces with truth publication/recording disabled. Test the proxy rejects Range/referee handles and stale generations. Never send World/Referee, scenario answers or preflight truth into the policy process.
- [x] Install core SDK/runtime data and Python into a separate build/mission-package-consumer owned prefix, never the existing package-consumer work directory. Compile C++/C99 consumers and run Python using only that prefix with source/build fallbacks removed. If shared staging is unavoidable, apply the same explicit CTest resource lock to both gates.
- [x] Check DLL/data/wrapper/native-module provenance, all retained exports/layouts/vtables and old-header binaries against the new library.
- [ ] Run the current full ucrt64-headless suite in the exclusive native slot. Build ucrt64-release and run generic renderer/lifecycle regressions when changed boundaries justify them. Consume current complete visual acceptance evidence before closing the full M8 gate. (M8 generic: full headless suite and Release build and gates done; current complete visual acceptance evidence pending.)
- [x] Record seed/aspect matrix, role/service/component counts, dt/cadence, build/compiler/package hashes, test inventory/results, failures and measurement limits in English.
- [x] Update current capability/status documents only with achieved evidence. Independently review, commit bounded final changes on main, and push only when requested/authorized (M8 generic; awaiting independent acceptance).

**Gate:** Claude delivery and handoffs pass. Full-program completion additionally requires every design J1-J10 case, current compatibility/full regression, installed provenance and complete visual acceptance. Keep missing external evidence pending; move real MA/DRL integration to the upper-layer project after full M8.

## Verification commands and scheduling

Use the existing launcher/presets and actual discovered test inventory. M0/M1/noncombat M2 contract, binding, report, C ABI and installed-consumer gates are registered. Later mission episode/gateway/composition/training test names remain planned registrations; a listed command is not evidence that a gate exists or has passed.

    .\fsim.cmd --preset ucrt64-headless configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe -DFSIM_WARNINGS_AS_ERRORS=ON
    .\fsim.cmd --preset ucrt64-headless build --parallel 12
    ctest --preset ucrt64-headless -L ma-training --no-tests=error --output-on-failure
    ctest --preset ucrt64-headless -R "^(sdk_boundary|c_abi|adapter_compatibility|world_adapter_compatibility|batch_recording_adapter_compatibility|python_sdk|package_consumer|mission_package_consumer)$" --no-tests=error --output-on-failure
    ctest --preset ucrt64-headless --no-tests=error --output-on-failure
    .\fsim.cmd --preset ucrt64-release build --parallel 12

Every focused command in Claude packages requires --no-tests=error and an exact discovered test count greater than zero. Hold the exclusive native configure/build/test slot and any required GUI slot; record the tested HEAD/diff. Run independent read-only reviews in parallel; never run competing viewers or CMake writers. Broaden tests only after changed behavior or a new failure justifies it. No final current full-suite claim until the final current build completes.

## Deferred scope and exit rule

Not required: production MA orchestration, neural-network training, universal BT/planner packages, distributed rollout cluster, shared-World partitioning, world-pool/snapshot optimization, new missile downlink equipment, full ASK wire certification, cryptographic identity, aircraft/missile calibration, shadow/material changes or unrelated source splitting.

Record performance instead of inventing a throughput target. Optimize only a measured bottleneck in a separately scoped follow-up that preserves the accepted episode and permission semantics.

Each package ends with reviewed tests and a main commit when execution is authorized. Do not reuse unfinished visual work as a clean baseline or combine unrelated existing modifications into MA commits.

The program ends after M8: the SDK can host upper-layer composite MA decision systems, P2P/C2 interactions and independently reproducible training episodes. Upper-layer MA implementation/training is a subsequent project deliverable.
