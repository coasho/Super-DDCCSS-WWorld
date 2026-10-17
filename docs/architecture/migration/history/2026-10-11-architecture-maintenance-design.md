# Architecture maintenance: boundaries and domain ownership

## Intent and baseline

The third maintenance pass starts from `f8cfec9`, after the documentation and build/package passes. The requested outcome is an easier-to-maintain project structure and an initial code architecture improvement. This pass preserves public C++/C/Python APIs, ABI layouts, principal permissions, simulation behavior and aircraft/resource bytes.

The first delivery is **Architecture A1**: executable module boundaries, a separate Mission Systems C adapter, and weapon-owned launch eligibility. Its stopping point is verified internal separation; it does not introduce new simulator capabilities or close the physical-model roadmap.

## Findings and alternatives

The native target graph is broadly consistent with the existing architecture, but every internal library exposes the whole `src` include root. CMake linking alone cannot prevent an undeclared header dependency. `ms/radar/Radar.cpp` includes pure quaternion helpers from `sim/Attitude.h` outside its dependency closure. Rendered `world/FlowVolumes.h` exposes IPC engagement events without declaring the IPC dependency.

`src/sdk/fsim_world_c.cpp` has 5,787 lines and mixes lifecycle, flight autonomy, agents, communication, recording and Mission Systems. Its final Mission Systems block is a coherent adapter with local conversions. The existing principal checker scans only two fixed C implementation files, so splitting adapters without updating that checker would weaken coverage.

`session::World::submit(StrikeCommand)` resolves entities and inventory, but also owns weapon FOV/seeker/illumination rules. These rules are stateless and can move to the weapon module while World continues assembling facts and submitting activities.

Three approaches were considered:

1. Rename/move directories only. This improves navigation but leaves responsibility and dependency problems intact.
2. Enforce existing boundaries and extract two established domain seams. Selected: existing consumers keep working and current tests can establish behavior preservation.
3. Replace World/control orchestration or introduce a general event/entity framework. Deferred: this changes lifecycle and numerical execution semantics before their contracts are sufficiently isolated.

## A1 design

### Module dependency enforcement

Register native modules and source ownership explicitly. Check the real declared internal target dependency graph, reject cycles and unknown modules, and require private-header imports to be reachable through the target's dependency closure. Retain legitimate transitive platform dependencies.

Scan optional renderer sources even in a headless checkout. Keep public SDK headers independent of private `src` and third-party implementation headers, and keep the headless layers independent of VSG/Vulkan. Account separately for `fsim_flow`, whose sources share `src/world` with `fsim_world`.

Move quaternion helper implementations unchanged into core. Keep the existing sim include/namespace forwarding interface so internal consumers retain their names. Radar uses the core header. Declare the renderer's IPC dependency explicitly.

### Mission Systems C adapter

Move the existing Mission Systems section into private `src/sdk/fsim_mission_c.cpp`, compiled into the same `fsim` DLL. Its local MS conversions stay with that adapter. Share only existing error/guard/size-aware copy helpers through a private header; do not create a second error store or duplicate conversion bodies.

Preserve exported names, declarations, error text, `struct_size` behavior, pointer lifetimes and the first principal guard in each function. Update principal checking to cover every compiled C adapter and reject exported world-handle definitions absent from its inspected sources. Public headers and rendered boundary inventory stay unchanged.

### Weapon launch eligibility

Create a two-phase pure eligibility check in `ms/weapon`: first evaluate the selected card and own/target state, then evaluate illumination facts only when target/FOV acceptance requires them. Keep the existing target-existence, FOV and illumination rejection precedence, defaults of 25/55 degrees, short-circuit behavior and numeric expressions.

World retains agent authorization, ordered store selection, entity/designation/radar lookup and Host submission. It still checks only the first selected AAM. No new state, allocation, RNG, worker callback or event phase is added. Add characterization of defaults/override FOV, behind/coincident targets, missing targets and each illumination mismatch, plus rejection side effects through the existing World path.

## Acceptance

- Existing public headers, aircraft/store sources and third-party content remain unchanged; SDK export names match a captured baseline.
- Boundary checker negative fixtures reject undeclared/upward imports, cycles, public/private leaks, headless GPU dependencies, unknown modules and missing C adapter guard coverage.
- Existing quaternion/JSBSim comparison, radar, C ABI, principal, agent, native engagement and worker determinism checks remain green.
- New eligibility tests cover refusal precedence and unchanged inventory/activity/event state on rejected World submissions.
- The full applicable headless functional suite, installed SDK consumer and launcher/distribution gates pass. Unchanged Hangar model generation retains prior evidence; no GPU/render-install acceptance is claimed.
- Current architecture/development/status guidance identifies the new owners and the remaining work.

## Subsequent work

| Priority | Candidate | Prerequisite and boundary |
| --- | --- | --- |
| High | MA state retirement | Establish which tasks, consents, authority and history must survive vehicle removal before adding detach semantics. Current MaCore retains per-entity state after World removal. |
| High | C ABI and Python domain separation | Continue with DMS/C2 and VI adapters. Preserve Python `__module__`/qualified identities, tuple layouts, FASTCALL, GIL/busy flags and buffer ownership. |
| Medium | Store-to-flight-model binding | Group property handles, mass/drag revisions and transfer bookkeeping without changing `loadTick`, dt or operation order. |
| Medium | Operational geometry ownership | Extract owner/common storage while retaining shadowing, revision, pointer stability and visibility. Keep frame truth policy separate. |
| Medium | Weapon observation generation | Separate seeker applicability/observation rules from World orchestration with fixed-scene encounter evidence. |
| Medium | Viewer and platform boundaries | Split viewer configuration/scene orchestration and isolate Flow's Win32 loading after a render build and visual acceptance path are available. |
| Medium | DLL export policy | Replace broad transitive `FSIM_BUILDING` propagation with explicit per-target ownership after dependency checks are stable. |

## Constraints

Windows x64 UCRT64 GCC, C++17 and CMake 3.25+ remain the supported path. Reuse installed tools under `D:/ENV`; use `D:/ENV/tmp` for compilation temporary files. Written files use English. No new runtime or third-party framework is introduced.
