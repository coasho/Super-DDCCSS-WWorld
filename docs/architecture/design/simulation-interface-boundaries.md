# Simulation and interface evolution boundaries

Date: 2026-10-15. Baseline: `main` at `54b7fb9`.

## Scope and stop point

Protect the SDK's construction, ownership, permission and compatibility boundaries before exposing the M5 mission
facade. This is a bounded prerequisite, not a new simulation framework. Formal decision systems and training
algorithms remain upper-layer work; test fixtures may call existing SDK interfaces. Existing flight VecEnv task
utilities remain compatible. Physical model fidelity and Viewer visual/performance acceptance remain separate work.

## Dependency direction

The environment/session adapter depends on native simulation services. Native simulation never depends on mission
episodes, gateways, Viewer, GPU state or rendering frame callbacks. Viewer consumes diagnostic IPC/recording output;
it does not advance or correct authoritative physics. Keep the executable module policy in
[`tools/architecture`](../../../tools/architecture/README.md).

Materials, shadows, cameras and visual effects must not alter native simulation, actor knowledge or permissions.
Image-based sensors may use an explicit vision provider with defined measurements and availability. That provider
must not depend on an interactive Viewer's camera, frame rate or presentation state. Physical terrain and weather
belong to the simulation even when Viewer draws the same resources.

## Construction and reset ownership

`session::makeWorldOptions` is the shared internal mapping for public C++ World construction and episode construction.
Exact-arity aggregate decomposition makes a new public, native or admitted episode option a compile failure until its mapping/default
decision is reviewed. Episode scenario admission has a separate exhaustive decomposition: every public field is
accepted, name-checked or explicitly refused. Native-only terrain prefetch retains its native default. The legacy C
adapter keeps its size-gated defaults; it must not inherit newer C++ defaults for absent option tails.

An episode accepts the scenario's capacity, JSBSim root and terrain settings. Timing, workers, seed and diagnostic
outputs remain episode-owned. A scenario's flight VecEnv configuration is refused, rather than silently ignored.
Construction mapping does not change the public World constructor's wall-clock epoch initialization.

An episode owns a complete World. New mutable simulation state must be owned beneath that World or by that
episode, and full reset reconstructs it. Immutable model/resource sharing is allowed. Process-global mutable
guidance, sensor, communication or damage state must not become a cross-episode shortcut. Existing Vehicle reset
semantics remain distinct. Each subsystem addition must extend fresh-reset, seed-isolation and ownership tests.

## Actor facade and trusted native access

M4's native setup/range/service callbacks are trusted internal integration seams. Their generation lock protects
lifetime; possession of a native callback is not an actor permission boundary. Keep these seams internal.

M5 actor endpoints expose only bound Agent requests and legal, immutable reports. They must not expose native
callbacks, `session::World`, public range `World`/Vehicle handles, MissionEpisode/Range/referee handles, raw IPC,
recording truth, or an alternate path to acquire them. Endpoint identity comes from its binding, not request data.
The source guard rejects private namespace declarations and references in public headers, retaining only exact
existing World facade seams. It is a lexical check, not a sandbox: public-handle escape, buffer reads, generations,
authentication and process isolation still require M5 acceptance tests in C++, C99 and Python.

## Fixed ABI and independent output formats

The existing unsized public state/control records have fixed layout and array stride. Growing C and C++ mirrors
together does not preserve old binaries. Future engine, sensor or damage telemetry uses additive, versioned
interfaces with explicit sizes/capability discovery, rather than growing existing state records.

IPC layout 9 and recording format 7 embed raw state/control/sample records. Their fixed sizes, offsets and types are
guarded separately from mirror equality, including IPC header, table, mission and event record extents. Literal recording fixtures retain versions 1 through 7 and adjacent sample
strides. A new transport/file representation needs an explicit version, reader/writer adaptation or deliberate
incompatibility rejection; bumping a number or appending bytes alone is not a compatibility design. Viewer-only
telemetry does not belong in the actor report contract merely because it is available in a diagnostic snapshot.

New simulation capabilities may add versioned request/report families. Preserve availability, provenance, timestamps
and native refusal/outcome meanings. Improved physics can legitimately change measured trajectories and outcomes;
interface compatibility does not require old numerical results after an intentional model change. Do not turn a
provisional native outcome into universal tactical success or a fixed reward definition.

## Executable acceptance

- `architecture_boundary` and `architecture_checker`: native module direction, headless/GPU separation and public
  private-type exposure, including negative fixtures for native-to-episode and Viewer-to-episode dependencies.
- `simulation_interface_boundary`: every public construction field, native defaults, episode refusals, frozen public
  and output layouts, literal legacy recordings, and captured native-state parity with diagnostic publication,
  recording, forced publication, focus requests and IPC observer reconnects across complete resets.
- `mission_episode` and `native_state_reset_fixture`: complete fresh reset, generation/lifetime safety and independent
  seeds using existing native state, including pending equipment and communication activity.
- Existing C ABI, adapters, Python, installed consumers and principal checks retain their own acceptance scope.

The output parity case uses the supplied fixture's fieldwise capture and active replay; its documented private-state
observability limits still apply. Mission payload/radar component guards pin their strides; they do not individually
pin every internal member. The case uses Viewer's actual IPC reader without creating a window or GPU. It verifies the simulation
output boundary, not materials, shadows, camera behavior, frame rate or fullscreen visual acceptance. Current results
belong in [`docs/status.md`](../../status.md); missing model and Viewer acceptance remain open there.

M5 may proceed after these prerequisite gates and independent review pass. Its own public facade/isolation gates
must pass before it can be declared delivered. This prerequisite adds no M9/M10 milestone.
