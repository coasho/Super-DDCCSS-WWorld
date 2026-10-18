# Three-domain boundaries and dependency rules

## 1. Architectural contract

The three domains are simulation, training and visualization. They have separate implementation roots, explicit public/internal contracts and one-way dependencies. Shared contracts and foundation services are named support modules, not a fourth domain or an unrestricted common-code bucket.

A dependency means a target link, a private-header import, a Python import, a runtime plugin/provider dependency or an installed-package requirement. Runtime data flowing from a producer to a consumer is not permission for the producer to call back into the consumer.

The following are forbidden:

- Simulation depending on training or visualization implementation.
- Training core depending on graphics libraries or visualization implementation.
- Visualization core depending on training or stepping/resetting simulation.
- A domain depending on the final SDK assembly that aggregates that same domain.
- A shared support module importing a domain implementation.
- Private implementations importing the root Python compatibility facade.
- Avoiding the policy with deferred/dynamic imports, whole-archive assembly, parent-directory includes or a generic shared-state registry.

## 2. Domain ownership

| Domain | Owns | Consumes | Does not own |
| --- | --- | --- | --- |
| Simulation | World clock/steps; flight; physical environment and disturbances; sensors/communications; guidance/fuzes/collision/damage; native Agent/C2 permissions and execution | Neutral contracts, platform/math/assets/transport services, JSBSim and admitted model data | Learner/reward algorithms, production MA runtime, presentation timing |
| Training | Independent episode/batch lifecycle; legal gateway snapshots/requests/receipts; task criteria/referee transport; seeds; adapter scheduling and communication costs | Simulation facade/runtime contracts and shared services | Physical outcomes, invented sensor knowledge, Viewer/GPU state, production MA/optimizer |
| Visualization | Scene construction, camera rendering, live/replay presentation, UI, visual effects and image output | Immutable presentation/stream contracts, assets, graphics backend | World stepping, actor knowledge expansion, guidance/damage adjudication |
| Contracts | Stable values, command/report documents, immutable views and stream schemas | Standard-library value facilities | Registries, World ownership, OS access, graphics or domain implementations |
| Foundation | OS services, math algorithms, assets, transport/replay mechanisms | Contracts and lower foundation modules | Domain policy or cross-domain mutable state |
| Adapters/applications | Final exports, compatibility wrappers and explicit product composition | Declared domain APIs | Duplicate domain implementation |

## 3. Target dependency graph

Arrows mean depends on. The machine-readable [target policy](../validation/target-dependencies.json) is authoritative for individual edges; this diagram is a simplified view.

~~~mermaid
flowchart TD
  Apps[Applications and optional training-image composition] --> SDK[SDK compatibility assembly]
  Apps --> Viewer[Viewer application]
  Apps --> VisionAPI[Vision compatibility adapter]
  SDK --> Training[Training mission and environment]
  SDK --> SimAPI[Simulation facade]
  Training --> SimAPI
  Training --> Sim[Simulation runtime]
  SimAPI --> Sim
  Sim --> Foundation[Named foundation services]
  Sim --> Contracts[Neutral contracts]
  Foundation --> Contracts
  Viewer --> Viz[Visualization core]
  Viewer --> Transport[Shared stream and recording services]
  VisionAPI --> Viz
  VisionAPI --> SDK
  Viz --> Foundation
  Viz --> Contracts
  Transport --> Contracts
~~~

The final Vision adapter may consume the legacy SDK for compatible World handles. Visualization core does not. A camera source is read-only; segmentation/truth output stays on the Range side unless a separate permitted image-report contract exists.

An application may compose training and visualization through an image-provider interface. This outer application is the owner of the optional graphics dependency. Neither training core nor simulation gains a reverse dependency.

## 4. SDK extraction decision

Current mission gateway code calls public World/Agent implementations while the final SDK links the mission gateway. A direct extraction that leaves all World/Agent implementation in that final SDK would create a cycle.

The selected sequence is:

1. Classify neutral public values separately from public facade declarations.
2. Give the World/Agent/recording implementations and construction mapper a private simulation_api target below training. Native scenario execution belongs in the simulation runtime. Under the HC-15 placement adjustment accepted at ARCH-04, the thin public `applyScenario` compatibility wrapper stays in the final assembly and moves with the SDK adapters in ARCH-07; it owns no independent scenario algorithm and creates no reverse dependency.
3. Let the training mission target consume simulation_api and the native episode/runtime contracts.
4. Keep the existing fsim DLL as the final assembly of simulation_api, training mission and C/C++ compatibility exports.
5. Keep the existing CPython _native module as the single established runtime type-registration owner.
6. Isolate rendered Vision implementation from its legacy World-handle adapter.

Private implementation targets are not new installed SDK libraries by default. Existing DLL names, exports and installed headers remain. Per-target export definitions, WHOLE_ARCHIVE registration and object ownership must be proven before assembly changes are accepted.

## 5. Data and control flow

### Authoritative execution

An actor produces a request through its configured SDK endpoint. The endpoint validates service identity, generation, tick, fields and request IDs. Submission queues; the Range closes the tick, dispatches in deterministic order and invokes native execution. Simulation alone advances physical state and delivers modeled messages. Training reads native results and freezes the next legal snapshot.

No training component independently moves a missile, creates a hit, changes consent to satisfy an actor or invents missing equipment feedback.

### Observation and referee channels

An actor snapshot contains only information available to its bound Agent through modeled sensors/links and permissions. Referee truth, reward inputs, arbitrary task state, visual diagnostic buffers and segmentation identifiers remain separate.

A configured platform/system gateway consumes DMS once for internal fanout. Legacy Agent.receive remains consuming. Preserve the full provenance, validity, unknown/null and sample/receive-time contracts.

### Presentation and images

Simulation publishes neutral immutable output or recording frames. Visualization reads them. Focus requests and wall-clock presentation rate cannot change the native trajectory or event sequence. Interpolation is presentation-only.

Vision returns a timestamped, generation/tick-associated image buffer. An outer observation provider must define whether that image is permitted for an actor; camera access to Range state is not itself that permission.

### UI controls

Display widgets cannot access mutable native internals. Operator commands use a separately classified command adapter with explicit authority, not the state-view channel. The passive Viewer core remains read-only.

## 6. Lifecycle, concurrency and determinism

- A mission episode owns one World and all mutable mission/judge/gateway/RNG state.
- Full episode reset reconstructs that World. Existing Vehicle.reset keeps its documented authority/task/override/C2 behavior.
- Generation-checked retained control blocks protect children after reset, move and parent close.
- Preserve EpisodeHold: step, outcome, final capture and autoreset share one episode boundary.
- A batch preflights all required holds before mutation; conflicting batch/child calls fail explicitly as busy.
- SameStep keeps old finals apart from the new initial observation. NextStep resets before old-generation work can dispatch.
- The adapter retries a busy batch dispatch without ticking systems twice.
- Seed streams derive from stable environment/local-episode/service/component IDs; worker count and another slot's resets do not select a sequence.
- Graphics frame rate, publication frequency and reader count do not consume simulation RNG or choose its native timestep.

## 7. Public compatibility

Preserve names, signatures, public struct/class layouts, vtables, enum values, refusal/error precedence, C buffer behavior, native ownership and Python qualified identities. Add versioned interfaces when a new contract is needed.

Public include/fsim names are installation contracts. An internal source move must not require users to include src paths. The World header's existing native facade seam is a documented compatibility exception, not a license for new private-type exposure.

IPC layout/version and recording readers remain byte compatible, including old recording fixtures. Moving a definition does not authorize adding a field.

## 8. Evidence and ownership

Each source has one owner. Each shared path has one active writer per feature window. A complete package includes native code, adapters, examples/tests/docs and acceptance evidence together. Existing Claude feature ownership remains unchanged.

Native graph checks, source-import checks, Python/import-owner checks, installed package checks and runtime behavior checks are independent evidence. Passing one cannot replace another.

Model calibration, structural cleanup, training integration and rendering performance use separate work packages and acceptance. A successful directory migration cannot mark an unfinished physical model or Viewer scenario complete.
