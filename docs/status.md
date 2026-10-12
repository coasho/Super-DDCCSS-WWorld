# Current capabilities and milestone status

Reviewed: 2026-10-11. Starting implementation revision: `32e78a6`; this maintenance pass is a working-tree change.

This is the entry point for what exists now, which delivery it belongs to, and what remains open. The first
maintenance pass consolidates documentation and development guidance, corrects outdated descriptions, and retires
the obsolete tactical prototype and orphan visual/compiler artifacts. It does not refactor native modules, change
the SDK's ABI or physics, repair packaging/launcher behavior, or regenerate models.

Use the [documentation index](README.md) to find reference material, the [development guide](development.md) for
build and contribution procedures, and the [resource guide](resources.md) before editing generated assets.

## How to read status

- **Implemented** means the current source provides the named behavior. It does not imply that every aircraft,
  configuration, hardware target, or physical performance claim has passed acceptance.
- **Recorded delivery** means a named validation record reports completed checks on its stated build. It is not a
  fresh test run on the reader's checkout.
- **Partial** means an implementation exists but the original stage's scope or acceptance criteria remain open.
- **Planned** means an accepted objective or candidate exists without evidence here that it has been delivered.
- **Historical** identifies dated decisions, plans, measurements, and migration records. Their dates and scope
  remain attached to the original evidence.

API contracts belong in `docs/sdk/`; accepted architectural constraints remain in their ADRs. This page summarizes
their current relationship. A newer implementation or demonstration does not silently close every older roadmap
criterion. Update this page and the affected reference together when a capability changes.

## Capability map

| Area | Current state | Contract and evidence | Limits and remaining work |
| --- | --- | --- | --- |
| Native simulation and SDK | Implemented: caller-driven `World` stepping, vehicles, JSBSim flight, environments, effects, scenarios, recording, C++/C/Python interfaces and vectorized environments. | [SDK index](sdk/README.md), [world](sdk/world.md), [scenarios](sdk/scenarios.md), [VecEnv](sdk/vecenv.md). | Public interfaces are under `include/fsim`; `src/` is internal. Test success is scoped to its configuration and cases. |
| Vehicle Interface and Flight Autonomy | Implemented control contracts, activities, grants, performance/navigation reports, routes, patterns, launch and recovery. Support and applicability vary by aircraft. | [Control API](sdk/control.md), [Vehicle Interface](vehicle-interface.md), [Flight Autonomy record](flight-autonomy.md), [testbench](../testbench/README.md). | Retain the open control and stability findings in the [backlog](backlog.md). An advertised capability or a successful process exit is not blanket full-load flight acceptance. |
| Agent, MA core, range and referee separation | Implemented agent handles, authority checks, measured designations, messaging and process-isolation interfaces; the ADR records AB-1 through AB-6. | [Agent API](sdk/agents.md), [ADR-31](agent-boundary.md), generated [boundary table](sdk/boundary.md). | Truth snapshots remain range/referee interfaces. New exports require classification and boundary checks; original engagement-scoring acceptance remains distinct from API access classification. |
| Mission Systems host and payload data | Implemented discovery, payload state/BIT, capabilities, activities, grants, cards and aircraft fits. | [Mission Systems API](sdk/mission-systems.md), [ADR-30](mission-systems.md). | A card or installed payload does not prove its whole physical model exists. Per-capability support and per-aircraft fits remain relevant. |
| RF, RCS, radar and tracking | Implemented RF propagation, mesh-derived RCS tables, detection radar, trackers and signal processing. RWS/TWS/STT and ACM presets are present. The roadmap records MS-1 through MS-6 and MS-4b deliveries. | [Mission Systems API](sdk/mission-systems.md), [dated MS progress](mission-systems.md), `tests/test_radar_modes.cpp`, `tests/test_signal.cpp`, `tests/test_track.cpp`. | Signal-level processing has a different cost from detection-level processing. Loaded-store RCS and other model gaps remain in the backlog. |
| Aircraft and store generation | Implemented Hangar designs, models, JSBSim files, manifests, carriage and the recorded store build. | [Resource ownership](resources.md), [Hangar](hangar.md), [store build record](hangar-stores-build.md). | Source inputs, generated designs and runtime products have different editing rules. Reproducible generation does not establish calibrated real-world flight performance. |
| Native missile engagements | Implemented release, native motor/flight state, radar and infrared seeker observations, guidance, countermeasures, fuzes, damage, terminal records and replayable events. Recorded delivery at the implementation baseline. | [Engagement contract](sdk/engagement.md), [delivery validation](sdk/engagement-validation.md), `tests/test_engagement_native.cpp`. | Guidance, fin response, seekers, decoys and launch-zone parameters are game estimates. This is not completion of the original high-fidelity MS-7c/MS-7d/MS-7f criteria. |
| Launch zones and countermeasures | Implemented advisory launch-zone queries and native chaff/flare inventory, releases, motion and expiry. | [Engagement contract](sdk/engagement.md), `tests/test_countermeasure_core.cpp`, `tests/test_weapon_guidance.cpp`. | Advisory envelopes do not guarantee intercepts or satisfy MS-8's original published-range acceptance. Direct dispenser APIs do not mean every electronic-attack capability is implemented. |
| Warning receivers and other sensors | ESM/RWR contacts are implemented. Other payload families retain their own support states and staged modeling work. | [Mission Systems API](sdk/mission-systems.md), `src/ms/esm/`, [remaining MS stages](mission-systems.md). | Native seeker and dispenser support is not a claim that full IRST imagery, IFF, jamming, all guns, PNT, or the OMS bridge is complete. |
| Native viewer, effects and vision | Optional Vulkan/VSG viewer and vision modules exist, with terrain, aircraft/store visuals and engagement effects. | [Viewer](sdk/viewer.md), [vision](sdk/vision.md), [Hangar effects](hangar-effects.md). | The native Vulkan viewer was not rebuilt for the native-engagement delivery. Older GPU observations retain their original hardware and build scope. |
| Testbench | Flight Autonomy, platform and stability scenarios remain active. The obsolete `tactics.bvr` prototype and its independent Python simulation have been removed. | [Testbench guide](../testbench/README.md), `testbench/fsim_testbench/registry.py`. | Native engagement acceptance is provided by SDK tests and its delivery record; no replacement testbench combat scenario is introduced here. |

## Milestones

Use the domain-qualified names below in status reports. Existing `M1` identifiers in historical documents are
preserved, but never use an unqualified `M1` to select work.

| Name used here | Original scope | Current disposition |
| --- | --- | --- |
| **Platform M0-M4** | Foundation milestones in the [system architecture record](FlightSim_System_Architecture_and_Design.md): simulation, terrain, SDK/RL interfaces, viewer, vision and external training. | Simulation, SDK/VecEnv, viewer and camera implementations exist with dated progress records. The original generic `ext/env_server` proposal is not an implemented module. Original throughput goals and target matrices are not asserted complete or freshly verified. |
| **Platform M5** | Public v1 release, packaging, licensing, benchmarks and frozen interfaces. | Open release milestone. No local `v*` release tag was present at review; packaging code alone is not public release acceptance. |
| **Mission Systems MS-1 to MS-6, MS-4b** | Host/data, RF, RCS, radar, trackers, signal processing and engagement radar modes. | Recorded implementations and dated stage evidence exist. Outstanding fixes or model limits remain open individually. |
| **Mission Systems MS-7 / MS-8** | Store/weapon lifecycle, high-fidelity flight models, seekers, guidance, lethality, validation and launch zones. | Partial against the original criteria: stores, assets, damage and a native game-assumption engagement path exist; higher-fidelity calibration and published-performance acceptance remain open. See B-36 and related [backlog](backlog.md) items. |
| **Mission Systems M1** | The autonomous end-to-end loop in [ADR-30](mission-systems.md), including the original modeling, sensor, scoring and demonstration acceptance. | Keep open as the original acceptance target. The native-engagement delivery supplies a narrower working path; it does not close every M1 dependency or authorize the next stage automatically. |
| **Native engagement delivery** | SDK ownership of game-assumption missile engagements and countermeasures; native integration and client-state consumption. | Recorded local delivery: [contract](sdk/engagement.md), [validation record](sdk/engagement-validation.md), [design](superpowers/specs/2026-10-11-sdk-missile-engagement-design.md). Its browser-client results do not cover the retired testbench prototype or a rebuilt native viewer. |
| **Agent boundary AB-1 to AB-6** | Classification, scoped handles, designations, mission data, messaging/authority, and process isolation. | Recorded completed migration in [ADR-31](agent-boundary.md). Later engagement records do not by themselves prove every original referee/reward-scoring requirement. |
| **First maintenance pass** | README/navigation, current capability and milestone status, development guidance, resource-generation ownership and removal of confirmed obsolete content. | Completed and locally verified below. Retired the tactical prototype, deleted orphan visual/compiler artifacts and corrected descriptions/tests/report wording. No architecture refactor, launcher/packaging repair, model regeneration or additional feature milestone is included. |

The next implementation milestone has not been selected by this documentation pass. Preserve the original roadmap
and its acceptance criteria; select a bounded next task explicitly before expanding the queue.

## Known workflow and reporting gaps

These are documented observations for follow-up, not fixes delivered by this pass.

| Gap | Observed source behavior | Consequence and follow-up |
| --- | --- | --- |
| Launcher bootstrap/configuration | `fsim.cmd` checks for an existing executable before dispatching `build` or `help`; its build/test/dist commands use Release while runtime discovery can select other builds. `Testbench.cmd` also expects Release. | Use the explicit preset commands in [development](development.md). A future launcher change should make first-build and selected-preset behavior consistent. |
| Independent SDK data contract | The SDK install component does not install `share/flightsim`, but `cmake/fsimConfig.cmake.in` requires that directory. | Static packaging inconsistency; an independent SDK consumer failure has not been reproduced in this pass. Define data ownership and verify a clean component install before claiming standalone SDK delivery. |
| Direct `dist` dependencies and consumers | `dist` depends on deployment and optional Python staging, while its installation includes examples. The existing `tests/package_consumer` is not registered in current CTest/CI. | Clean direct packaging and component-consumer acceptance require follow-up; a prior whole-tree build can hide missing dependencies. |
| Optional Python checks | Python discovery can return from `python/CMakeLists.txt` before registering Python, Hangar and `sdk_boundary` checks. | Check that required tests actually exist in the configured tree before interpreting a green CTest summary. |

## Cleanup decisions

- Removed the old `tactics.bvr` suite and its independent Python sensor/missile/kill-scoring implementation. The
  testbench no longer offers that scene. Native SDK encounter tests remain the engagement acceptance route.
- Removed the unused root `comp.spv`, three agent-process PNGs and seven unreferenced README/viewer screenshots.
  Referenced design/source photographs and dated control/performance evidence remain attached to their scope.
- Corrected strike/store capability descriptions to name the remaining calibrated weapon and launch-zone fidelity
  rather than claiming native guidance, seekers and fuzes do not exist. `Partial` support remains intentional.
- Kept showcase/plume fixtures because current model tests and viewer-performance tools consume them. Their
  carrier/missile poses and detonation timing are scripted; native target flight, damage and Flow fire do not
  establish native missile-flight acceptance.
- Clarified the generated effects audit: zero missing entries is a result under its current exclusions, not proof
  that every physical effect exists. Rotorcraft exhaust/wake gaps remain B-59 in the [backlog](backlog.md).

## Verification records and update rules

### Maintenance verification, 2026-10-11

Fresh local checks for this working-tree change:

| Check | Result |
| --- | --- |
| `cmake --build --preset ucrt64-headless -j 10` | Passed; native libraries, tests and Python package restaged |
| `ctest --preset ucrt64-headless -j 10 --timeout 3600` | All 572 tests passed, including 40 Hangar and 18 fleet-labeled cases; elapsed 886.00 seconds |
| Documentation links and commands | Checked all 22 changed/new Markdown files, local file links and anchors; no missing targets. All 13 PowerShell command blocks parsed |
| Development skill | Both frontmatter validations passed; canonical and discovery copies are byte-identical |
| Testbench catalogue | 89 scenarios remain; FA recovery and all 35 stability cases remain registered; no Tactics suite/scenario |
| Generated effects report | Regenerated with its owner; existing inventory rows and totals unchanged. Audit rules and scripted Python fixture behavior unchanged |
| Change scope | Diff formatting clean; no changes under `aircraft/`, `assets/stores/` or `third_party/` |

Raw build and test logs are local artifacts at `D:/ENV/tmp/project-cleanup-build.log` and
`D:/ENV/tmp/project-cleanup-tests.log`. This maintenance check did not rebuild the Vulkan viewer, perform GPU
visual acceptance, reproduce the external browser-client checks, or install/test standalone SDK components.

### Earlier delivery evidence

The [native-engagement validation record](sdk/engagement-validation.md) reports a repaired functional suite, an
unchanged model suite, Python and boundary checks, native encounters, and an external browser integration. It also
records the initial allocation failure and the limits of the final checks. Preserve that distinction rather than
rewriting it as one uninterrupted green run.

The checked-in [CI workflow](../.github/workflows/ci.yml) configures a Windows UCRT64 headless build, runs CTest, and
rebuilds store/effect outputs plus one aircraft. It does not build the Vulkan viewer or perform component-install
consumer acceptance. Its header records that it had not yet run on GitHub when written; no remote run was inspected
for this documentation review.

When updating status:

1. Name the implementation revision, build/preset and the commands actually run.
2. Keep source inspection, recorded checks and fresh measurements separate.
3. Update the SDK reference and affected current-status statements together; mark replaced plans as historical.
4. Close a backlog/stage item only against its own acceptance criteria and evidence.
5. Keep agent/range/referee access, ABI compatibility, deterministic behavior and resource provenance explicit.
