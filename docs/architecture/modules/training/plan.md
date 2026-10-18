# Training module plan

## 1. Responsibility

Training owns correct experiment lifecycle over the native simulator: independent mission episodes, seeds, snapshots, queued requests/receipts, per-role lifecycle, task/referee transport, batch scheduling and framework-independent adapter behavior.

The destination is src/training/native and the private Python implementation under python/fsim/_training. Public fsim.mission, fsim.training, VecEnv and the native type-registration owner remain compatible.

Production MA actors/BT/planners/blackboards/arbitration and learning optimizers belong in the upper-layer project. This repository provides minimal replaceable composition examples and installed-consumer contracts.

## 2. Current accepted basis and pending gates

| Area | Current basis | Migration treatment |
| --- | --- | --- |
| M0 generic contracts | Frozen versioned documents and fixtures | Preserve schemas/native mappings; add extensions separately |
| M1/noncombat M2 | Explicit service binding and legal reports | Consume native simulation authority |
| M4 | Independent World ownership, generation/lifetime/reset and seed behavior | Preserve EpisodeHold and full reset fixture |
| M5 core | Public episode/endpoint/referee, generic requests/receipts and process proxy | Move implementation only after lower simulation facade exists |
| M6 noncombat | Installed composition example with deterministic arbitration/lifecycle | Preserve trace equivalence |
| M7 core at 0cf0150 | Independent batches, role finals, costs, autoresets and adapter; independently accepted | Preserve all five corrected findings and preopened-handle regressions |
| M8 generic at d344e2b | Implemented; awaiting independent acceptance | Review its handoff and release its exclusive window before shared source migration |
| Full M5/M6/M7/M8 | Combat handlers/outcomes/criteria and complete joint/Viewer evidence have pending inputs | Remain pending until actual owner inputs and gates are available |

The current feature [plan](../../../superpowers/plans/2026-10-12-ma-system-training-readiness.md) and [allocation](../../../superpowers/plans/2026-10-12-ma-system-training-work-allocation.md) remain authoritative for Claude's assigned features. This architecture package does not reassign their implementation.

## 3. Native ownership split

| Implementation | Current | Destination |
| --- | --- | --- |
| Episode owner, generation/lifetime/holds and seed streams | src/env/MissionEpisode* | training/native/episodes/runtime |
| Legacy VecEnv/task/observation/action implementation | Remaining src/env | training/native/vecenv |
| Config and strict JSON/document validation | src/sdk/MissionConfig*, MissionJson* | training/native/mission |
| Gateway/judge/snapshots/receipts/costs | src/sdk/MissionGateway* | training/native/mission |
| Mission batch and final-document lifecycle | src/sdk/MissionBatch.cpp | training/native/batch |
| Public mission facade/impl | src/sdk/MissionEpisode* | training/native/episodes/api, assembled into the same SDK |
| C ABI/Python registration | SDK C adapters, python/src | adapters/sdk |
| World/Agent implementation needed by gateway | src/sdk/World.cpp, Agent.cpp | Lower simulation/api, not training or final assembly |

A training target may depend on the lower simulation facade and runtime. It must not depend on fsim's final SDK assembly when that assembly links the training target.

The existing native episode calls the construction-options mapper. Its env target explicitly depends on simulation_api after ARCH-04 relocates that mapper. Native option values are extracted below runtime in ARCH-02; the mapper and its public compatibility inputs do not enter the runtime target. Preserve scenario/seed/options parity across this split.

## 4. Contract boundaries

### Actor endpoint

An actor receives only its configured endpoint and permitted reports. Identity binding is configured outside request documents. No Range/referee handle, costs/outcome/truth document or arbitrary task custom state crosses into actor input.

Preserve 64-bit decimal-string identities, null/validity for unknown values, bounded base64 and decoded/native limits. Do not simplify unavailable values to zero.

### Submission and native execution

Submit validates and queues. The Range closes the tick, dispatches by stable service/request order and runs the native steps. Native refusals remain native receipts; training does not grant consent or manufacture an outcome to make a request succeed.

Preserve scoped request deduplication, changed-body refusal, live correlations, terminal retention and non-consuming reads.

### Episode lifecycle

One World and one mutable task/judge/gateway instance per mission episode. Fresh reset rebuilds the World. Retained generation/closed blocks protect old endpoints, activities, targets and process connections.

A compound transition must include native step, outcome read, final capture and autoreset under one episode hold. Preopened endpoints/referees are part of that contention contract.

### Batch and roles

Batch, platform, service, component and actor counts remain independent. A C2 service may have no aircraft.

Keep role active/terminated/truncated/final-observation/bootstrap apart from episode_done/reason. An ended role stops ticking while teammates continue, and has one terminal snapshot. SameStep and NextStep keep old finals apart from new initial observations.

The adapter must tick each system once per decision tick even when batch dispatch refuses busy; post-step records/hooks change only after the required output is read coherently.

### Referee and reward

Truth, task outcome, communication/resource costs and supplied reward inputs are separate Range/referee streams. No reward definition is currently installed merely because the adapter has a critic stream.

Full combat outcome/reward semantics require separately frozen, versioned owner input. Missing equipment reports remain unavailable; launch correlations do not imply a modeled weapon-feedback link.

## 5. Python separation

1. Remove the deferred fsim-to-training-to-fsim cycle: use the existing native Error/backend interface directly, not the root facade.
2. Keep public class/enum/named-tuple qualified identities and _native ownership stable.
3. Keep private lifecycle implementations independent of package root reexports.
4. Move shared values needed by world and agent below both wrappers, eliminating their deferred cycle.
5. Keep process transport/encoding in an adapter with explicit simulation and mission type registries.
6. Keep Gymnasium/SB3 integrations optional and uneager. Core import must work without ML packages.
7. Remove VecEnv from visualization implementation. An optional outer image-provider composition owns that dependency.
8. Preserve stdlib-only mission training behavior and installed public-interface examples.

## 6. Work packages

| ID | Prerequisite | Output | Acceptance |
| --- | --- | --- | --- |
| TRN-A | Reviewed M8 generic handoff | Reviewed feature baseline, interfaces, current pending-gate list | No unfinished shared write/build window |
| TRN-B | Neutral contracts + lower simulation_api | Training native target extracted from SDK implementation | No training-to-final-SDK cycle; registered exports unchanged |
| TRN-C | TRN-B | Episode and VecEnv directories separated | Existing flight shapes/defaults and mission full-reset/lifetime regressions |
| TRN-D | TRN-B/C | Gateway/config/receipts/batch moved with a single state owner | Requests/dedup/cost/referee and both autoreset regressions |
| TRN-E | Shared Python contracts | Private Python training and compatibility wrappers separated | No deferred import SCC; class/pickling/proxy/type identity unchanged |
| TRN-F | TRN-D/E | Installed composition/workload/in-process and strict-process comparisons | Same permitted traces; package/DLL/data provenance |
| TRN-G | Supplied combat schemas/handlers/native evidence | Full M5 and full composition integration at registered seams | Actor equipment permissions + native outcomes, no provisional behavior |
| TRN-H | Accepted TRN-G plus versioned criteria/reward definitions | Full M7 transport/reduction integration | Native meanings preserved; hidden-truth equivalence for actor/masks |
| TRN-I | Required owner/Viewer inputs | Full M8 joint/program evidence | J1-J10 and current compatibility/full regression/Viewer requirements |

TRN-G through TRN-I are functional completion packages, not prerequisites for every safe directory move. Keep their missing inputs explicit.

## 7. Required behavioral evidence

- Worker-count and reset-schedule determinism using actual recorded matrices.
- Independent task/judge/RNG ownership and fresh/reset equivalence.
- Parent move/close, stale generation and busy refusal through C++/C99/Python.
- Native preopened endpoint/referee read/submit contention against SameStep and NextStep.
- No old-generation action/message/release/damage after an episode ends, with combat cases pending until their required interfaces are supplied.
- Lost role with continuing teammates; C2-only roles; variable topology; complete team end.
- One snapshot per tick and non-consuming internal fanout/C reads.
- Deduplication and native refusal correlation without duplicate execution.
- Costs/outcome/truth refusal before actor-process invocation; ordinary data still crosses.
- Real M6 system under the adapter, both autoresets and installed imports.
- Hidden-truth changes do not change legal actor observations or masks.
- Measured initialization, throughput, memory and reclamation; no invented performance target.

## 8. Framework and upper-layer handoff

The upper-layer project receives installed episode/batch endpoints, lifecycle signals, legal reports, seed conventions and separate critic transport. It owns learner-specific tensor layouts, optimizers, policy checkpoints, production MA orchestration and distributed rollout scheduling.

An optional framework adapter can translate these public outputs. It cannot become an SDK simulation authority or require PyTorch/Gymnasium for headless native consumers.

The SDK architecture program stops at its agreed joint/compatibility/installed/Viewer acceptance. Starting formal MA or DRL training is a separate upper-layer assignment.

## 9. Exit

Structural acceptance requires clean source roots and target/import graphs, retained ABI/type identity, current installed consumers and the [acceptance matrix](../../validation/acceptance.md). Functional acceptance additionally requires the explicit full-program inputs above.

Record achieved, unverified and pending items separately. Do not turn a passing noncombat core or a migration result into full combat training acceptance.
