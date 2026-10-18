# Directory layout and classification

## 1. Required final shape

The following is the destination layout. These directories are planned; their appearance here is not evidence that source has moved.

~~~text
src/
  contracts/                  neutral private/shared value and wire contracts
  foundation/
    platform/                 operating-system services
    core/                     math, deterministic utility algorithms
    io/                       asset and format infrastructure
    transport/
      world_stream/           shared-memory producer/reader mechanisms
      recording/              versioned recording readers/writers
  simulation/
    flight/                   FDM integration and native model stepping
    flight_control/           VI/controller execution
    environment/              physical disturbances, noise and forces
    mission_systems/          radar, sensors, stores, weapon and damage models
    communications/
      network/                generic network/UDP mechanisms
      dms/                    modeled delivery and RF integration
    runtime/
      services/               native Agent/C2/MA ownership and reports
      stores/                 store-to-flight binding state
      world/                  authoritative World assembly/lifecycle
    api/                      private lower World/Agent/scenario facade
  training/
    native/
      episodes/
        runtime/              retained ownership, reset, seed streams and holds
        api/                  public mission facade, distinct implementation files
      vecenv/                 existing flight VecEnv compatibility implementation
      mission/                configuration/gateway/documents/receipts
      batch/                  mission scheduling and final-document lifecycle
  visualization/
    render/                   graphics backend
    scene/                    Earth, terrain, vehicle, interpolation and camera builders
    vfx/
      flow/                   separate optional Flow target
      effects/                exhaust, trails, airflow and vapour
    ui/                       presentation widgets
    vision/                   offscreen image generation from read-only inputs
  adapters/
    sdk/
      cpp/                    final C++ compatibility facade assembly
      c/                      sized C ABI/error/principal adapters
      python/                 _native registration and domain C adapters
    vision/                   legacy fsim_vision World-handle adapter
    oms/                      separately gated MS-12 protocol adapter
  applications/
    headless/                 native simulation runner
    viewer/                   passive live/replay viewer
    demo/                     scenario publisher using simulation APIs
    training_visualization/   optional image/provider composition
    testbench/                classified test application and compatible launcher
include/fsim/                 stable public installed headers
python/fsim/                  stable public package and compatibility declarations
  _contracts/                 internal shared Python values/protocols
  _simulation/                simulation wrapper implementation
  _training/                  dependency-free training implementation
  _visualization/             optional camera wrapper implementation
  _adapters/                  typed process/compatibility transport
tests/
  contracts/
  simulation/
  training/
  visualization/
  compatibility/
  integration/
examples/
  simulation/
  training/
  visualization/
tools/
  architecture/
  models/
  data/
  validation/
  launch/                     the launchers' implementation (fsim.cmd and Testbench.cmd run it)
docs/
  architecture/               this planning and architecture authority
  sdk/                        public API reference
  ...                         retained domain/reference records until registered relocation
~~~

## 2. Assignment rules

1. The business implementation belongs in exactly one domain root.
2. Public headers remain under include/fsim even when implementations move. Their declared type owner is recorded in the dependency policy.
3. Shared contracts contain definitions that both sides need, not helpers that happen to be convenient to share.
4. Foundation submodules have individually declared targets and dependencies. Their common directory does not permit arbitrary imports between them.
5. Final exports, handle/error translation and Python registration belong in adapters. Domain algorithms do not accumulate there.
6. Applications compose APIs and own optional integrations. They do not become a second simulation engine or production MA framework.
7. A new file outside these assigned categories fails source-ownership acceptance.
8. Empty placeholder domain directories are not created as a substitute for a real migration.

## 3. Important current-to-target moves

The [path map](../migration/path-map.csv) supplies the machine-readable migration units and precedence.

| Current | Destination owner | Special handling |
| --- | --- | --- |
| src/sim | simulation/flight | Keep JSBSim numerical execution unchanged |
| src/control | simulation/flight_control | Preserve command refusal/grant behavior |
| src/effects | simulation/environment | These effects alter physical/sensed state |
| src/ms | simulation/mission_systems | Preserve native model meanings and pending-fidelity status |
| src/comm | simulation/communications/network | Separate generic transport from modeled DMS |
| src/session/Dms* | simulation/communications/dms | Preserve routing, provenance, queues and RNG |
| Other session World/native services | simulation/runtime | Split by state ownership before moving shared helpers |
| World/Agent/scenario SDK implementation | simulation/api | Create lower bridge before mission extraction |
| src/env | training/native | Keep legacy VecEnv separate from independent mission episodes |
| MissionGateway/Config/Json/Batch SDK files | training/native | Consume lower simulation facade; never final SDK |
| src/render, scene parts of src/world | visualization/render and scene | Rename world so it cannot be mistaken for simulation authority |
| Flow/VFX parts of src/world | visualization/vfx | Preserve separate Flow target and loader ownership |
| src/ipc | contracts plus foundation/transport | Never put the shared publisher protocol inside visualization |
| src/vision | visualization/vision plus adapters/vision | Split read-only rendering from compatible World-handle access |
| src/app | applications | Separate live/replay viewer from demo simulation composition |

## 4. Python and CPython ownership

Public imports remain valid: fsim, fsim.world, fsim.agent, fsim.vecenv, fsim.mission, fsim.training, fsim.isolation and optional fsim.vision/gym/sb3.

Public named tuples, enums, native runtime types and serialization identities keep their established owners and qualified names. Do not duplicate a native type in a new extension or move a public class declaration in a way that silently changes __module__ or pickling.

Private implementation helpers can move under _simulation, _training, _visualization and _adapters. Public modules retain thin declarations/reexports as needed. A compatibility facade imports the implementation; the implementation cannot import that facade back.

Move shared World/Agent value types below both wrappers. Replace local imports used to break circular initialization with explicit lower factories/backend interfaces. Obtain Error from its existing native owner or inject the backend exception type; training implementation must not import fsim to reach Error.

The camera module must not import VecEnv. The optional training-image composition imports both public APIs from an outer integration.

The existing _native extension remains one runtime type-registration owner. Its C source files are classified into domain adapter subdirectories without replacing the stable ABI or creating duplicate state owners.

## 5. Assets and generated artifacts

Keep the authoritative resource trees stable unless a separate resource migration is approved:

- aircraft design sources and generated runtime models retain their generator ownership.
- aircraft/stores XML is generated; it is never repaired manually during a source move.
- assets/payloads, assets/stores, models, maps, shaders and licenses remain classified data.
- third_party remains vendor code; tools/environments/caches stay under D:/ENV.
- build and distribution outputs remain derived artifacts and are not module sources.

Path ownership is an API for launchers, asset resolution, installed consumers, fixture loading and generators. Every move updates its consumers and tests in the same package.

## 6. Documentation organization

All new cross-domain plans are in docs/architecture. Module plans are under modules/simulation, modules/training and modules/visualization. Current-state evidence, design decisions, migration and validation have their own directories.

Public API reference stays under docs/sdk. Historical evidence is retained with its date and scope. Existing ADRs and accepted plans are listed in the [document register](../migration/document-register.csv); they cannot be relocated during an active feature window merely for tidiness.

When relocating a document: move its canonical body once, fix inbound links, keep a small redirect at the old path where compatibility requires it, and record its supersession/authority. Do not maintain two editable copies or replace historical evidence with current claims.

## 7. Tests, examples and tools

Tests follow ownership; installed-consumer and cross-domain cases remain integration/compatibility tests. Preserve existing CTest names and labels initially even when source files move.

Examples use installed public interfaces. The composition example remains a replaceable demonstration, not a production MA runtime. Testbench is an application/test harness and may compose APIs without becoming a lower simulation dependency.

Architecture, model generation, asset preparation, validation and launcher tools have explicit categories. The launch category holds the repository launchers' implementation, which `fsim.cmd` and `Testbench.cmd` run; it was declared at ARCH-10 (HC-25) in place of a layout exception. A tool can depend on a domain API; the domain cannot depend on that tool at runtime.
