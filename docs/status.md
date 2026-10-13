# Current capabilities and milestone status

Reviewed: 2026-10-12. First maintenance pass: `2b83f24` (implementation baseline `32e78a6`).
Second maintenance pass: `f8cfec9`; Architecture A1: `6652584`; Architecture A2: `34e5516`; Architecture A3: `e79f268`;
Architecture A4: `e708b87`; Architecture A5: `2efea05`; Architecture A6: `b57947d`, based on `39be3c1`.
Current development and delivery use `main`, including both maintenance passes and Architecture A1 through A6.
Branch names in the dated verification records identify their original execution context.

This is the entry point for what exists now, which delivery it belongs to, and what remains open. The first
maintenance pass consolidated documentation and development guidance, corrected outdated descriptions, and retired
the obsolete tactical prototype and orphan visual/compiler artifacts. The second pass aligns build/run/test presets
and repairs standalone SDK packaging and consumer acceptance. Architecture A1 starts the third pass by enforcing
native module dependencies, separating the Mission Systems C adapter and assigning stateless launch eligibility
to the weapon module. A2 retires private MA consent bookkeeping and separates C/Python native DMS/C2 adapters
while retaining public history and original state-transition timing. A3 separates C control/VI and Python native
control/Mission handlers, retaining shared handles, errors and registration. A4 separates C Scenario/Environment
and Python World/Scenario/Environment handlers while retaining core runtime ownership. A5 separates C/Python
VecEnv/Recording handlers, retaining C++ wrappers and coupled type/buffer callbacks. A6 starts World state-owner
decomposition with private store-to-flight-model bindings, retaining lifecycle and simulation order. Public interfaces, ABI layouts,
physics and aircraft model bytes remain unchanged.

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
| Native viewer, effects and vision | Optional Vulkan/VSG viewer and vision modules exist, with terrain, aircraft/store visuals and engagement effects. Native fullscreen lifecycle and screenshot corrections have passed focused GPU checks. | [Viewer](sdk/viewer.md), [vision](sdk/vision.md), [current viewer validation](sdk/viewer-validation.md), [Hangar effects](hangar-effects.md). | The viewer-correction baseline failed the 4K performance gate for three of four scenes; performance of the replacement native hit fixture has not been remeasured. Aircraft materials retain existing detail; aircraft cast shadows are not enabled. Current acceptance is scoped by the linked record; older GPU observations retain their original hardware and build scope. |
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
| **Second maintenance pass** | Consistent configure/build/run/test/dist presets, first-build entry, independent SDK runtime data and installed-consumer acceptance. | Completed and locally verified below: 535 functional CTest cases, real launch/build/dist commands and independent installed consumers. Native module architecture and simulation models are unchanged. |
| **Architecture A1** | Executable native dependency boundaries, Mission Systems C adapter separation and weapon-owned launch eligibility. | Implemented and locally verified below: 538 functional cases plus three architecture checks; 1,338 DLL export names match the baseline. Internal ownership changes preserve public interfaces and simulation behavior. The broader architecture queue remains open. |
| **Architecture A2** | MA private consent retirement and C/Python native DMS/C2 adapter separation. | Committed as `34e5516` and locally verified below: 547 functional cases plus three architecture checks, unchanged exports and installed-consumer/distribution acceptance. Public history remains queryable; aggregate history reclamation is deferred. |
| **Architecture A3** | C control/VI and Python control/Mission adapter separation. | Committed as `e79f268` and locally verified below against the [A3 design](superpowers/specs/2026-10-12-architecture-adapters-design.md): 548 functional cases plus three architecture checks, unchanged exports/runtime descriptors and installed-consumer/distribution acceptance. |
| **Architecture A4** | C Scenario/Environment and Python World/Scenario/Environment adapter separation. | Committed as `e708b87` and locally verified below against the [A4 design](superpowers/specs/2026-10-12-architecture-world-adapters-design.md): 549 functional cases plus three architecture checks, unchanged public exports/native identities and installed-consumer/distribution acceptance. |
| **Architecture A5** | C/Python VecEnv and Recording adapter separation with retained C++ and runtime type owners. | Committed as `2efea05` and locally verified below against the [A5 design](superpowers/specs/2026-10-12-architecture-batch-recording-design.md): 550 functional cases plus three architecture checks, unchanged exports/native identities and installed-consumer/distribution acceptance. |
| **Architecture A6** | World store-to-flight-model binding state ownership. | Committed as `b57947d` and pushed at the user's request. Locally verified below against the [A6 design](superpowers/specs/2026-10-12-architecture-store-binding-design.md): all 596 headless CTest cases, including 40 Hangar cases, exact eight-variant baseline replay, unchanged exports/native identities and installed-consumer/distribution acceptance. |
| **MA training M0** | Generic contracts and compatibility baseline of the [MA training plan](superpowers/plans/2026-10-12-ma-system-training-readiness.md). | Contract frozen: the [mission training contract](sdk/mission-training.md), its [schema](sdk/ma-contract.schema.json) and fixtures, the [composition vocabulary](sdk/ma-composition.md) and the dated [compatibility baseline](sdk/ma-compatibility-baseline.md) (599 of 599 headless cases at the baseline; C++, C99 and Python consumers frozen). M0 itself changed no SDK header, export, adapter or behaviour. |
| **MA training M1** | Explicit MA-service binding: which Agent of a platform owns each command capability and each side of its DMS. | Implemented in C++ (`fsim/MAService.h`), C (ABI 1.72) and Python, additive: configured ownership is the same whatever order the Agents were made or bound in, overlapping ownership and foreign controllers are refused, and a platform with no binding behaves as before ([contract](sdk/mission-training.md#9-service-binding), [agents.md](sdk/agents.md#ma-services)). |
| **MA training noncombat M2** | Legal reports with their provenance: navigation, own tracks, passive contacts, link tracks, peers, tasks and authority. | Implemented as Agent calls in C++ (`fsim/MAReports.h`), C (ABI 1.73) and Python, additive: each report is the Agent's existing read with its source, sample and receive times, age and validity; unknown, stale, invalid, unavailable and not-applicable are kept apart, and nothing the referee alone knows enters one ([contract](sdk/mission-training.md#legal-report-calls-noncombat-m2), [agents.md](sdk/agents.md#legal-reports)). Verified by `ctest -L ma-training`, which includes the installed C++, C99 and Python consumers. The mission episode, the gateway and its documents (M4, M5 and later), every engagement extension runtime and all joint acceptance remain open. |

The third maintenance pass includes bounded A1,
[A2 lifecycle/adapter](superpowers/specs/2026-10-12-architecture-lifecycle-design.md) and
[A3 control/Mission](superpowers/specs/2026-10-12-architecture-adapters-design.md) and
[A4 World/Scenario/Environment](superpowers/specs/2026-10-12-architecture-world-adapters-design.md) and
[A5 VecEnv/Recording](superpowers/specs/2026-10-12-architecture-batch-recording-design.md) and
[A6 store-binding slices](superpowers/specs/2026-10-12-architecture-store-binding-design.md).
The remaining [architecture queue](superpowers/specs/2026-10-11-architecture-maintenance-design.md#subsequent-work)
includes further World state-owner decomposition, a public-history retention contract, retained C World and Python core/type/buffer
ownership, finer control ownership, operational geometry, weapon observations, viewer/platform
boundaries and DLL export ownership. A2 retains map traversal,
controller mappings and post-removal history; it does not bound aggregate State storage or unanswered ended-strike
requests. These candidates require their own acceptance; architecture maintenance does not close a flight,
sensor or weapon-model milestone.

## Known workflow and reporting gaps

The first pass recorded these workflow gaps. The second pass repairs the first three; Python SDK availability remains
an environment-dependent condition. A1 makes architecture acceptance independent of SDK binding discovery.

| Workflow | Current behavior | Acceptance and limits |
| --- | --- | --- |
| Launcher bootstrap/configuration | Explicit `--preset` precedes `FSIM_PRESET`, then `ucrt64-headless`. Configure/build/run/test/dist and Testbench use the selected build. First build/dist configure automatically; help/where work before binaries exist. | `launcher_contract` exercises the real Windows batch entry, argument preservation, failures, first build and configuration isolation. Testbench viewer/profile regressions run inside `python_sdk`. |
| Independent SDK data contract | SDK installs stock JSBSim, designed aircraft/store models, cards, damage/pieces, notices and applicable vision resources. Exports describe the actual relocatable `bin` and `share/flightsim` layout. | `package_consumer` verifies complete runtime-file hashes, installed DLL locations, stock/custom/store flight and poisoned-XML rejection. The development resolver's source fallback remains; probes prove use of existing installed models. Vision resources are checked without GPU rendering. |
| Direct `dist` dependencies and consumers | `dist` builds all enabled installed targets, selects present components and removes inactive components' obsolete generated files. Only viewer maps are retained as reusable data. | `dist_components` builds a small independent fixture directly through dist, checks disabled-component cleanup, CMake exports and cleanup ownership. CTest/CI register both package gates. |
| Optional Python checks | Python discovery can return from `python/CMakeLists.txt` before registering Python, Hangar and `sdk_boundary` checks. | Check that required tests actually exist in the configured tree before interpreting a green CTest summary. |
| Native architecture checks | Three standard-library checks register independently of the Python SDK, using Python 3.11+ or `FSIM_CHECK_PYTHON`. | Repository imports and declaration policy, module checker fixtures and C guard scanner fixtures are exercised. Configure warns when no supported interpreter is available; that configuration has no architecture acceptance. |

## Cleanup decisions

- Removed the old `tactics.bvr` suite and its independent Python sensor/missile/kill-scoring implementation. The
  testbench no longer offers that scene. Native SDK encounter tests remain the engagement acceptance route.
- Removed the unused root `comp.spv`, three agent-process PNGs and seven unreferenced README/viewer screenshots.
  Referenced design/source photographs and dated control/performance evidence remain attached to their scope.
- Corrected strike/store capability descriptions to name the remaining calibrated weapon and launch-zone fidelity
  rather than claiming native guidance, seekers and fuzes do not exist. `Partial` support remains intentional.
- Replaced the maintained showcase, plume-check and viewer `hit` scene's scripted missile motion with SDK
  release and native JSBSim flight. Their scenes use observations and consent; SDK guidance, seekers, fuzes and
  damage determine the outcome. Missing support fails instead of falling back to supplied poses or timed bursts.
  Dated scripted-fixture images and performance records keep their original scope; they are not evidence for
  the replacement scenes.
  The optional native `--live` run publishes and paces the same scene for continuous fullscreen observation;
  captured frames alone do not establish full-flight acceptance. Completed live runs are scoped by the
  [viewer validation record](sdk/viewer-validation.md).
- Clarified the generated effects audit: zero missing entries is a result under its current exclusions, not proof
  that every physical effect exists. Rotorcraft exhaust/wake gaps remain B-59 in the [backlog](backlog.md).

## Verification records and update rules

### First maintenance verification, 2026-10-11

Local checks for the change committed as `2b83f24`:

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

### Second maintenance verification, 2026-10-11

Local checks for the change committed as `f8cfec9`, based on `2b83f24`, using UCRT64 Release headless and
CPython 3.13.12. All compiler temporary directories use `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| `.\fsim.cmd --preset ucrt64-headless configure -DFSIM_PYTHON_EXECUTABLE:FILEPATH=D:/ENV/python/3.13/python.exe` and `build -j 10` | Passed; typed CMake flags preserved and current Python entry metadata regenerated |
| `.\fsim.cmd --preset ucrt64-headless test -LE hangar -j 10 --timeout 3600` | All 535 functional cases passed, including 18 fleet cases, Python SDK, SDK boundary, launcher and both package gates; elapsed 346.02 seconds |
| `launcher_contract` | 79 checks passed through real CMD/PowerShell entry points; includes empty/quoted/metacharacter arguments, preset precedence, first-build bootstrap, process failures and stale Python entry invalidation using real CMake |
| `package_consumer` | SDK-only install under a prefix containing spaces; 1,230 runtime-file hashes, installed fsim/JSBSim DLL provenance, stock/custom flight, four Mk 82 stores and native weapon stepping passed. All three poisoned-model probes rejected with exit 42; restored positive run passed |
| `dist_components` | Direct dist compiled an initially unbuilt example; disabling components removed obsolete executables/wheels, retained viewer map data and rejected cleanup outside the owned distribution tree. Independent CMake exports matched actual fixed paths with nondefault GNUInstallDirs cache values |
| Actual launch commands | Headless simulator stepped two C172s; direct configured Python created/stepped an SDK world; synchronous Testbench `fa1.support` passed |
| `.\fsim.cmd --preset ucrt64-headless dist -j 10` | Passed; SDK, tools, examples and Python trees plus wheel produced. Actual SDK distribution matched all 1,230 consumer manifest entries and tested runtime DLL bytes |
| Documentation and review | Nine changed/new Markdown files, local links/anchors, PowerShell blocks, preset JSON and CI YAML checked; skill copies matched. Independent review passed after export-layout and stale-entry fixes |
| Scope | No native implementation/public-header, aircraft, store-source or third-party changes; no architecture refactor |

Logs are `D:/ENV/tmp/preset-sdk-configure.log`, `preset-sdk-build.log`, `preset-sdk-functional-tests.log`,
`preset-sdk-headless-run.log`, `preset-sdk-python-run.log`, `preset-sdk-testbench.log` and `preset-sdk-dist.log`.
Installed-consumer logs and `runtime-data.sha256` are under `build/ucrt64-headless/package-consumer`.
The unchanged 40 Hangar cases retain the first pass's full-suite evidence and were not rerun in this pass.
The renderer/GPU, render SDK installation, CPack ZIP and remote GitHub workflow were not exercised. Headless
package acceptance does not close Platform M5's public-release or GPU target matrix.

### Architecture A1 verification, 2026-10-11

Fresh local checks on `codex/architecture-boundaries`, based on `f8cfec9`, using UCRT64 Release headless and
CPython 3.13.12. Compiler temporary directories remain under `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| Headless configure and `build -j 10` through `fsim.cmd` | Passed; libraries, native tests and Python package rebuilt |
| `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600` | All 538 functional cases passed, including 18 fleet cases, Python/principal checks, launcher and both installed-package gates; elapsed 325.27 seconds |
| `.\fsim.cmd --preset ucrt64-headless test -L architecture --output-on-failure` | All three checks passed in 2.94 seconds: repository policy covers 19 modules and 332 source/header files, with 33 module fixtures and 22 C guard scanner test methods |
| Architecture checks with `FSIM_BUILD_PYTHON=OFF` | All three passed in 5.24 seconds in an independent headless configuration. This establishes registration without Python SDK bindings; the independent configuration was not used for a native build |
| World launch characterization and focused native acceptance | Before extraction, two new World cases passed with 443 assertions. After extraction, all 17 selected cases passed in 65.97 seconds, covering launch eligibility, stores, quaternion/JSBSim comparisons, C ABI and native engagement/worker determinism |
| Adapter and ABI comparison | The extracted 710-line MS block contains 78 exported definitions and is byte-identical to its baseline. Existing error/guard/copy helpers are unchanged; all 1,338 rebuilt DLL export names, including 466 C exports, match the captured baseline |
| `.\fsim.cmd --preset ucrt64-headless dist -j 10` | Passed; distribution refreshed from the rebuilt SDK. All 1,230 runtime manifest entries and both fsim/JSBSim DLL bytes match the independent installed consumer |
| Independent review | Three checker gaps were reproduced and fixed: target include-search imports, qualified world-handle inputs and non-enforcing guard macros. Final negative fixtures and repository checks passed; no remaining actionable finding in the reviewed A1 scope |
| Documentation | Eight changed/new Markdown files, 164 local links and five anchors checked with no errors; all 11 PowerShell command blocks parsed and development skill copies remained byte-identical |
| Scope | No changes under public `include/`, aircraft, store-source or third-party paths; no generated model changes |

The functional and architecture runs cover 541 distinct CTest cases. Raw logs are local artifacts at
`D:/ENV/tmp/architecture-a1-configure.log`, `architecture-a1-build.log`, `architecture-a1-functional-tests.log`,
`architecture-a1-architecture-checks.log`, `architecture-a1-focused-tests.log` and
`architecture-a1-no-python-sdk-checks.log`. Distribution output is in `D:/ENV/tmp/architecture-a1-dist.log`;
manifest, DLL and scope comparison results are in `D:/ENV/tmp/architecture-a1-final-verification.json`.
Adapter comparison evidence is in
`D:/ENV/tmp/architecture-a1-sdk-extraction-check.json`; installed-consumer logs and runtime hashes are under
`build/ucrt64-headless/package-consumer`.

The first functional command through the batch wrapper failed before CTest started because CMD interpreted the
combined label expression's pipe. The successful recorded run invoked CTest directly with the same preset and
selection; the initial shell failure remains in `D:/ENV/tmp/architecture-a1-functional-tests-shell.log`.
Unchanged Hangar generation retains the first pass's 40-case evidence. Renderer/GPU behavior, render SDK
installation, CPack ZIP and remote CI were not exercised. The dependency gate scans renderer source declarations
in a headless checkout; that is source acceptance, not a renderer build.

### Architecture A2 verification, 2026-10-12

Fresh local checks for A2, committed as `34e5516`, collected on `codex/architecture-lifecycle` and based on A1
`6652584`, using UCRT64 Release headless, warnings as errors and CPython 3.13.12. Compiler temporary directories
remain under `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| Headless `build -j 10` through `fsim.cmd` | Passed; native libraries/tests and both Python adapter translation units compiled and the Python package restaged |
| `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600` | All 547 functional cases passed, including 18 fleet cases, C ABI, Python agent/isolation/thread/buffer checks, native engagement determinism, launcher and both package gates; elapsed 325.12 seconds |
| Architecture gates | All three passed in 2.51 seconds; repository policy covers 19 modules and 334 source/header files, with 33 module fixtures and 22 C guard scanner test methods |
| MA baseline characterization | Before MA edits, three public cases passed with 105 assertions and one borrowed-storage case passed with four assertions. They cover removal/reset history, C2 settlement, finite/NaN overrides, lost communication, slot reuse, first answers and authorized late replies |
| MA storage and public acceptance | The new ledger tests first failed for the absent implementation. After integration, all six internal cases passed with 4,134 assertions, including 512 answer and 512 retirement/reuse cycles; all three public characterization cases passed with 105 assertions |
| C adapter comparison | The 659-line DMS/C2 block with 70 exported definitions is byte-identical to the A1 baseline. The original-file remainder, three conversions and shared guard body are unchanged; only the guard declaration gains an unused-helper annotation |
| Python adapter comparison | All 49 moved handler/converter bodies, six shared helpers, 14 registration tables, WorldObject layout, type registration and 23 existing busy checks match the baseline. No GIL phase is added. All 47 native descriptors retain their names, qualified names, owners, types and documentation; seven compatibility tests passed before extraction, after rebuilding and after final distribution staging |
| Exports and protected scope | All 1,338 SDK DLL export names, including 466 C exports, match A1. The Python native module still exports only `PyInit__native`. SHA-256 checks match all 3,204 captured public-header, Python object-model, aircraft/store/generated, third-party and boundary-inventory files |
| Installed consumer and `.\fsim.cmd --preset ucrt64-headless dist -j 10` | Both package gates passed in the functional suite. Distribution refreshed successfully; all 1,230 runtime manifest entries and both fsim/JSBSim DLL bytes match the tested installed consumer. Distributed `_native.pyd` matches the rebuilt, tested module |
| Independent review and documentation | Complete A2 diff independently reviewed with no actionable findings. Six changed/new Markdown files, 151 local links and six anchors checked with no errors; all five PowerShell blocks parsed. Development skill copies remain byte-identical |

The functional and architecture runs cover 550 distinct CTest cases. Raw logs are local artifacts at
`D:/ENV/tmp/architecture-a2-build.log`, `architecture-a2-functional-tests.log`,
`architecture-a2-architecture-tests.log`, `architecture-a2-ma-characterization.log`,
`architecture-a2-ma-borrowed-characterization.log`, `architecture-a2-ma-ledger-red.log`,
`architecture-a2-ma-ledger-green.log`, `architecture-a2-ma-public-green.log`,
`architecture-a2-python-compatibility.log`, `architecture-a2-python-dist-compatibility.log` and
`architecture-a2-dist.log`. C extraction evidence is in `architecture-a2-c-links-extraction-check.json`;
Python source/runtime comparison evidence is under `D:/ENV/tmp/a2-python-native`.
Final export, scope and distribution comparison results are in
`D:/ENV/tmp/architecture-a2-final-verification.json`. Installed-consumer logs and runtime hashes are under
`build/ucrt64-headless/package-consumer`.

Two initial pre-refactor fixture runs exposed assumptions about message arrival order and cancel success:
network jitter can reorder simultaneous replies, and successful cancellation returns `Canceled`. The fixtures
were corrected before production edits; their original failure logs remain as
`architecture-a2-ma-characterization-baseline-fixture-failure.log` and
`architecture-a2-ma-characterization-baseline-cancel-fixture-failure.log` under `D:/ENV/tmp`.
Final formatting normalized one new Python source file's trailing newline; distribution rebuilt that module,
and the literal body checks plus seven compatibility tests passed again.

A2 reclaims answered private consent records and releases their storage when an entity is removed. It retains
public State/history, borrowed strings, controller mappings and unanswered ended-strike requests needed for late
answers. Retired entries still occupy the map; settled entries skip maintenance, but traversal remains a map scan.
No aggregate history bound or performance improvement was measured. Unchanged Hangar generation retains the first
pass's 40-case evidence and was not rerun. Renderer/GPU behavior, render SDK installation, CPack ZIP and remote CI
were not exercised. This architecture slice does not close a flight, sensor, weapon or public-release milestone.

### Architecture A3 verification, 2026-10-12

Fresh local checks for A3, subsequently committed as `e79f268`, were collected on `codex/architecture-adapters`
against A2 commit `34e5516`.
The [A3 design](superpowers/specs/2026-10-12-architecture-adapters-design.md) and
[implementation plan](superpowers/plans/2026-10-12-architecture-adapters.md) record the extraction seams and scope.
All commands used the existing Windows x64 UCRT64 headless Release preset and CPython 3.13.12, with compilation
temporary directories under `D:/ENV/tmp` and warnings treated as errors. No dependency was installed.

| Check | Result |
| --- | --- |
| Baseline characterization | Standalone C adapter consumer passed 1/1 and six new Python domain cases passed against A2 before production extraction. Missing-owner extraction checks failed as expected before the new files existed |
| Headless build through `fsim.cmd` | Passed; the C control and Python control/Mission translation units link into the existing binaries. After normalizing two EOF blank lines, the affected Python sources rebuilt and the package was restaged before the full suite |
| `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` | Passed 548/548 in 323.33 s, including control/Mission, agent/principal, MA lifecycle, isolation/thread/buffer, determinism, fleet and both package gates |
| Focused C/Python and architecture acceptance | C ABI, Python SDK, SDK boundary and new C compatibility consumer passed 4/4 in 82.72 s. Three architecture checks passed in 2.29 s |
| C extraction comparison | All 233 exported control/VI definitions, 55 local helpers, five layout assertions, 157 first principal guards and 32 source-authorization macros match A2. The parameter helper and original-file remainder retain their bodies/linkage; prior Mission/DMS owners and shared handle/error support are unchanged |
| Python source/runtime comparison | Control has 112 World and 13 module handlers; Mission has 37 World and 12 module handlers. Moved bodies/macros, eight shared helpers, 14 registration initializers, four type specs, PyInit and WorldObject layout match A2. All 229 World descriptors and 32 module functions retain their names, owners, qualified names, types and documentation. Existing busy/GIL/buffer/reference operations retain their positions |
| Exports and protected scope | All 1,338 SDK DLL export names match A2; the Python native module exports only `PyInit__native`. SHA-256 checks match all 3,514 captured public/object-model, resource, native domain, third-party and prior adapter files |
| Installed consumer, dist and distributed Python | Both package gates passed. Dist refreshed; all 1,230 runtime manifest entries and fsim/JSBSim DLL bytes match the tested installed SDK consumer. Distributed Python matches the staged package, and both binding compatibility suites passed 13/13 against dist |
| Independent review and documentation | Complete C/Python/test diff independently reviewed with no actionable regression. Seven changed/new Markdown files, 158 local links, six anchors and five PowerShell blocks checked. English content, exact file endings and diff scope passed; development skill copies remain byte-identical |

The functional and architecture runs cover 551 distinct CTest cases. Raw logs are under `D:/ENV/tmp`:
`architecture-a3-build.log`, `architecture-a3-final-build.log`, `architecture-a3-focused-tests.log`,
`architecture-a3-architecture-tests.log`, `architecture-a3-functional-tests.log`,
`architecture-a3-c-characterization-baseline.log`, `architecture-a3-python-characterization-baseline.log`,
`architecture-a3-c-extraction-check.log`, `architecture-a3-python-source-check.log`,
`architecture-a3-runtime-check.log`, `architecture-a3-dist.log` and `architecture-a3-python-dist-compatibility.log`.
The final export/scope/distribution results are in `architecture-a3-final-verification.json`; source/runtime
baselines are in `a3-baseline` and `a3-c-control`. Installed-consumer evidence remains under
`build/ucrt64-headless/package-consumer`.

Four initial C fixture failures were corrected before production edits: STARTING is not a requestable target;
activities return newest live entries before ended entries; controller zero is an agent's own identity; and
agent_identity returns boolean one on success. Their original logs remain as
`architecture-a3-c-characterization-fixture-{state,order,claims,identity}-failure.log` under `D:/ENV/tmp`.
Root also corrected initial test registration by separating the standalone main from the Catch SDK target,
restoring all existing test discovery. The EOF formatting failure is retained in `architecture-a3-formatting-failure.log`.

A3 preserves direct/guarded error clearing, null/short-result side effects, command readback and borrowed lifetime
distinctions. Mission's existing busy-check policy and simulation/domain behavior are unchanged. The working tree
was left uncommitted for review, then committed at the user's request. Unchanged Hangar generation retains the first
pass's 40-case evidence and was not rerun. Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This slice does not close
the separate physics, sensor, weapon or public-release milestones.

### Architecture A4 verification, 2026-10-12

Fresh checks were collected on `codex/architecture-world-adapters`, against committed A3 `e79f268`.
The [A4 design](superpowers/specs/2026-10-12-architecture-world-adapters-design.md) and
[implementation plan](superpowers/plans/2026-10-12-architecture-world-adapters.md) record the selected ownership
and stopping point. All builds used the existing UCRT64 headless Release preset, warnings as errors and CPython
3.13.12, with TMP/TEMP/TORCH_EXTENSIONS_DIR under `D:/ENV/tmp`. No dependencies were installed.

| Check | Result |
| --- | --- |
| A3 characterization before extraction | Public C consumer passed 1/1 and six new Python cases passed 6/6. A final C assertion refinement checks borrowed string content instead of unspecified pointer identity; its A3 baseline passed again. Both missing-owner source verifiers failed as expected before the new files existed |
| Headless build through `fsim.cmd` | Passed with warnings as errors; both new C owners and all three new Python owners compiled and linked into the existing SDK/module, and Python staging refreshed |
| Focused C/Python and architecture acceptance | C ABI, Python SDK, SDK boundary and both C adapter compatibility gates passed 5/5 in 83.69 s. Three architecture gates passed in 2.32 s |
| Full applicable functional suite | `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` passed 549/549 in 325.02 s, including 18 fleet cases, principal/isolation/thread/buffer checks, lifecycle, native engagement determinism, launcher and both package gates |
| C source comparison | Six Scenario and eight Environment exports are literal extractions, including all five moved first principal guards. World retains 56 definitions and control retains 228. Original remainders and prior Mission/DMS owners, Handles, CAbiSupport and CParameterSupport remain byte-identical to A3 |
| Python source/runtime comparison | The 23 World, four Scenario and eight Environment callbacks, field metadata and shared layouts retain their bodies. Core remainder, eight shared helpers, 14 registration initializers, four type specs and PyInit match A3. All 246 native descriptors across World/Scenario/VecEnv/Recording, 32 module functions and four type layouts/flags match; busy/GIL/buffer operations retain their original positions |
| Exports, protected scope and inventory | All 1,338 SDK export names match A3; Python exports only PyInit__native. SHA-256 matches all 3,692 protected files. The exact 22-file diff scope passes. All 591 A3 CTest names remain; the new standalone compatibility gate brings registration to 592, including 40 unchanged Hangar cases |
| Installed consumer, dist and distributed Python | Both package gates passed. Dist refreshed successfully; all 1,230 resource manifest entries and both SDK/JSBSim DLL bytes match the tested installed consumer. Distributed native/object-model bytes match staging, and all three binding compatibility suites passed 19/19 against the distributed module |
| Independent review and documentation | Complete C/Python/test diff independently reviewed with no actionable regression. English content, local links/anchors, PowerShell examples, exact new-file endings and diff checks pass; development skill copies remain byte-identical |

The functional and architecture runs cover 552 distinct CTest cases. Raw logs are under `D:/ENV/tmp`:
`architecture-a4-build.log`, `architecture-a4-focused-tests.log`, `architecture-a4-architecture-tests.log`,
`architecture-a4-functional-tests.log`, `architecture-a4-c-characterization-baseline.log`,
`architecture-a4-python-characterization-baseline.log`, `architecture-a4-characterization-final-baseline.log`,
`architecture-a4-dist.log` and `architecture-a4-python-dist-compatibility.log`. Final scope/export/resource results
are in `architecture-a4-final-verification.json`. Baselines are under `a4-baseline`; literal extraction evidence
and expected missing-owner failures are under `a4-c-adapters` and `a4-python-adapters`. Installed-consumer logs and
runtime hashes remain under `build/ucrt64-headless/package-consumer`.

No baseline behavior fixture or post-extraction acceptance failed. Existing native Scenario tests cover a later
command error retaining earlier creation/environment side effects; the new C consumer separately covers success,
borrowed strings, versioned options, truncated IDs and principal refusal. Scenario's borrowed C strings, Python
type ownership, partial application, Environment error/NaN distinctions and existing busy/GIL behavior remain
unchanged. Session World state ownership is unchanged. Python runtime construction/destruction, agent
creation/sealing, state buffers, scenario application, VecEnv/Recording and public object identities remain in
their established owners.

A4 was left uncommitted for review, then committed as e708b87 at the user's request after fresh source/runtime,
scope/export/distribution, documentation and focused architecture/C verification.
Unchanged Hangar generation retains first-pass evidence and was not rerun.
Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This adapter slice does not close
separate physics, sensor, weapon, history-retention or public-release milestones.

### Architecture A5 verification, 2026-10-12

Fresh checks were collected on `codex/architecture-vecenv-recording`, against committed A4 `e708b87`.
The [A5 design](superpowers/specs/2026-10-12-architecture-batch-recording-design.md) and
[implementation plan](superpowers/plans/2026-10-12-architecture-batch-recording.md) record the selected split and
retained ownership. Builds used the existing UCRT64 headless Release preset with warnings as errors and CPython
3.13.12. Compilation TMP/TEMP/TORCH_EXTENSIONS_DIR remained under `D:/ENV/tmp`; no dependencies were installed.

| Check | Result |
| --- | --- |
| A4 characterization before extraction | New public C consumer passed 1/1 in 0.30 s; six Python cases passed 6/6 in 0.206 s. Both missing-owner source verifiers failed as expected before production moves |
| Headless build through `fsim.cmd` | Passed with warnings as errors. New C/Python owners linked into the existing binaries, C++ wrappers remained in fsim.cpp and the Python package restaged |
| Focused C/Python and architecture acceptance | C ABI, Python SDK, SDK boundary and all three C adapter compatibility gates passed 6/6 in 83.51 s. Three architecture gates passed in 2.41 s |
| Full applicable functional suite | `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` passed 550/550 in 324.74 s, including 18 fleet cases, principals, thread/buffer/lifecycle checks, native engagement determinism, launcher and both package gates |
| C source comparison | All 14 VecEnv and nine Recording C definitions retain their original bodies. VecEnv's three local helpers and opaque layout match A4; C++ Impl/methods/toOptions/span/toResult, lastError, log/version/registry and original C linkage are retained. Recording's layout/assertions and borrowed views are preserved. World recording/lifecycle guards remain core; only exposed EOF blank lines were normalized |
| Python source/runtime comparison | All 11 VecEnv and five Recording callbacks, option metadata, names_list and exact private layouts match A4. Core retains three coupled callbacks, 14 registration initializers, four type specs, eight shared helpers and PyInit. All 246 descriptors, 32 module functions and four type layouts/flags match; existing GIL/busy/buffer/reference operations retain their positions |
| Exports, protected scope and inventory | All 1,338 SDK export names match A4; Python exports only PyInit__native. SHA-256 matches all 3,701 protected files. Exact 22-file diff scope passes. All 592 A4 CTest names remain; the new standalone consumer brings registration to 593, including 40 unchanged Hangar cases |
| Installed consumer, dist and distributed Python | Both package gates passed. Dist refreshed; all 1,230 runtime manifest entries and fsim/JSBSim DLL bytes match the tested installed consumer. Staged/distributed native and public Python wrappers, including VecEnv/Recording, match. All four binding compatibility suites passed 25/25 against distributed Python |
| Independent review and documentation | Complete adapter/test diff independently reviewed with no actionable regression, including independent expected/frozen hash checks. English content, links/anchors, PowerShell examples, exact new-file endings and diff checks pass; development skill copies remain byte-identical |

The functional and architecture runs cover 553 distinct CTest cases. Raw logs are under `D:/ENV/tmp`:
`architecture-a5-build.log`, `architecture-a5-c-characterization-baseline.log`, `a5-binding-baseline-final.log`,
`architecture-a5-focused-tests.log`, `architecture-a5-architecture-tests.log`, `architecture-a5-functional-tests.log`,
`architecture-a5-c-source-check.log`, `architecture-a5-python-source-check.log`, `architecture-a5-runtime-check.log`,
`architecture-a5-dist.log` and `architecture-a5-python-dist-compatibility.log`. Final scope/export/resource results
are in `architecture-a5-final-verification.json`. Baselines and exact source expectations are under `a5-baseline`,
`a5-c-adapters` and `a5-python-adapters`. Installed-consumer logs and runtime hashes remain under
`build/ucrt64-headless/package-consumer`.

The first Python fixture used World's nine pointer words as VecEnv's size; A4's VecEnv actually has eight. It was
corrected before production moves; the first failure remains in `a5-binding-baseline-first.log`, followed by
corrected/final baseline logs. Committed tests use standard temporary-directory discovery; local runs select
`D:/ENV/tmp` through the environment. C extraction exposed two extra EOF newlines in World; only those bytes were
normalized before rebuilding. The original formatting failure remains in `architecture-a5-formatting-failure.log`,
with pre-normalization expected bytes/metadata retained by the C verifier. Post-extraction acceptance had no failures.

VecEnv's original void-return guard remains distinct from the shared int-return guard; direct LastError retention
and guarded success clearing are unchanged. The cached borrowed World and zero-copy VecEnv/Recording views retain
their owner lifetime. C++ VecEnv wrappers and shared LastError remain core. Python buffers/borrowed World/sample
callbacks, all runtime type pointers and registration remain core; other callbacks have focused owners. Public
headers/object models, native simulation/domain sources and resource bytes remain unchanged.

A5 was left uncommitted for review, then committed as `2efea05` at the user's request after fresh source/runtime,
scope/export/distribution and documentation checks. Eight focused ABI, boundary, adapter and architecture gates
passed again before commit. Unchanged Hangar generation retains first-pass evidence and was not rerun.
Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This adapter slice does not close
separate physics, sensor, weapon, history-retention or public-release milestones.

### Architecture A6 verification, 2026-10-12

Fresh checks were collected on `codex/architecture-world-store-binding`, against A5 delivery baseline `39be3c1`.
The [A6 design](superpowers/specs/2026-10-12-architecture-store-binding-design.md) and
[implementation plan](superpowers/plans/2026-10-12-architecture-store-binding.md) describe the first World state-owner
slice. Builds used the existing UCRT64 headless Release preset, warnings as errors and CPython 3.13.12, with
TMP/TEMP/TORCH_EXTENSIONS_DIR under `D:/ENV/tmp`. No dependencies were installed.

| Check | Result |
| --- | --- |
| A5 characterization before extraction | Three new production-path cases passed with 2,655 assertions after correcting the fixture's initial Host revision assumption. Eight worker/frameSkip/drag variants recorded reset, reuse, between-step inventory edits, external fuel, tick boundaries and asymmetric release |
| Source extraction | The missing-owner verifier failed before implementation. After extraction, all five owned production files and complete World/CMake remainders match explicit baseline-derived expectations; WorldDamage remains unchanged |
| Headless build | Passed with warnings as errors; the new private owner compiles into the existing session target, without new libraries or external dependencies |
| Post-extraction characterization and replay | The same three cases and 2,655 assertions passed. All eight before/after traces match exactly: 67,553 lines, 2,641,139 bytes, SHA-256 `2d6e061a2dd7b7b84f0ec02ba32d83ff6b1e6ec9a80b99204861d731b7f32df5`. State fields use explicit scalar encodings, including float speedbrake, without structure padding |
| ABI, principal, adapter and architecture gates | All eight passed in 7.41 s: C ABI, SDK boundary, three C adapter compatibility consumers and three architecture checks |
| Full applicable functional suite | `ctest --preset ucrt64-headless -LE 'hangar\|architecture' -j 10 --timeout 3600 --output-on-failure` passed 553/553 in 325.59 s, including 18 fleet cases, lifecycle/principal/thread/buffer checks, native engagement determinism, Python, launcher and both package/installed-consumer gates |
| Exports, protected scope and inventory | SHA-256 matches all 3,720 protected files; all 1,338 SDK export names, sole Python `PyInit__native` export, 246 native descriptors, 32 module functions and four type layouts match A5. The exact 13-file diff scope passes. All 593 existing CTest names remain, with three new store-binding cases bringing registration to 596 |
| Installed consumer, dist and distributed Python | Both package gates passed in the functional suite. `cmake --build --preset ucrt64-headless --target dist -j 10` passed; all 1,230 runtime manifest entries and SDK/JSBSim DLL bytes match the tested installed consumer. Staged/distributed native and public Python wrapper bytes match; all four binding compatibility suites passed 25/25 against the distributed module |
| Independent review and documentation | Complete source/test diff independently reviewed with no actionable findings. Six changed/new Markdown files, 175 local links and five anchors pass; all five PowerShell blocks parse. English text, new-file endings and diff checks pass; development skill copies remain byte-identical |

Raw logs and replay files are under `D:/ENV/tmp`: `architecture-a6-build.log`,
`architecture-a6-characterization-baseline.log`, `architecture-a6-characterization-baseline-corrected.log`,
`architecture-a6-characterization-after.log`, `architecture-a6-replay-baseline.txt`, `architecture-a6-replay-after.txt`,
`architecture-a6-focused-tests.log`, `architecture-a6-functional-tests.log`, `architecture-a6-dist.log` and
`architecture-a6-python-dist-compatibility.log`. Final scope/export/resource results are in
`architecture-a6-final-verification.json`; documentation checks are in `architecture-a6-doc-verification.log` and
`architecture-a6-doc-commands.log`. The functional and architecture runs cover 556 distinct CTest cases.
Baselines are under `a6-baseline`;
explicit extraction expectations, original bytes and RED/GREEN evidence are under `a6-owner`. Independent source
review is in `architecture-a6-independent-review.md`. Installed-consumer evidence remains under
`build/ucrt64-headless/package-consumer`.

The first fixture incorrectly expected zero fresh Host revisions; Host::bind advances storeRevision. Its 18 failed
assertions were corrected against unchanged A5 source by comparing original/fresh Host revisions, then the baseline
passed. The first failure remains in its log. No production initialization or reset behavior was changed to pass;
post-extraction acceptance had no failures.

StoreFlightBinding owns Entry's station property handles, mass/drag revisions, between-step carried-mass stamp,
cached drag points/qbar and load-report revision. It borrows model property nodes and receives Host/model inputs
explicitly, retaining no World, Host or model pointer. World retains model/Entry lifetime, aircraft station-name
cache, traversal, permission checks, loadTick, control publication and release/damage mass transactions.
Separate store/fuel revisions, lazy qbar acquisition, refused-load synchronization, non-finite weight-room return,
fuel dt/threshold/draw/refuel order and force summation remain unchanged. Loaded reset retains inventory and
binding state; same-aircraft reuse creates a fresh Entry/Host/binding around the retained model, including its
original pre-synchronization mass transient.

A6 was initially left uncommitted for review, then delivered at the user's request. This initial acceptance retained
the first pass's unchanged Hangar generation evidence and did not rerun it.
Renderer/GPU, render SDK installation, CPack ZIP and remote CI were not exercised. This slice does not close the
remaining World state-owner work or separate physics, sensor, weapon, history-retention and public-release milestones.

### Architecture A6 post-push validation, 2026-10-12

The user requested delivery followed by fresh validation. Implementation commit
`b57947d75ea85e998e0af40fa9daf7828b5f9cd5` was pushed to
`origin/codex/architecture-world-store-binding`; an independent remote-head query matched local HEAD.
The checkout was clean. The existing UCRT64 headless Release build retained warnings as errors and CPython 3.13.12.
TMP/TEMP/TORCH_EXTENSIONS_DIR and Matplotlib's configuration/cache directory were under `D:/ENV/tmp`.

| Check | Result |
| --- | --- |
| Before commit/push | `fsim.cmd` build passed; 11 focused CTest cases passed in 7.22 s. Literal extraction, protected files, exports/runtime identities, exact scope and documentation checks passed |
| Build after push | `.\fsim.cmd --preset ucrt64-headless build -j 10` passed |
| Full registered headless suite | `ctest --preset ucrt64-headless -j 10 --timeout 3600 --output-on-failure` passed 596/596 in 887.51 s: 553 functional cases, three architecture checks and all 40 Hangar classes. This includes 18 fleet cases, native engagement determinism, lifecycle/principals, Python, launcher and both installed-consumer/package gates |
| Fresh production-path replay | All three store-binding cases passed with 2,655 assertions. Eight worker/frameSkip/drag variants reproduce the original 67,553-line, 2,641,139-byte baseline exactly, with the same SHA-256 recorded above |
| Distribution after the full suite | `.\fsim.cmd --preset ucrt64-headless dist -j 10` passed. All 1,230 runtime manifest entries and SDK/JSBSim DLL bytes match the freshly tested installed consumer; distributed Python native/wrapper bytes match staging |
| Distributed Python compatibility | All four binding compatibility suites passed 25/25 in 1.195 s against the actual distributed native module |
| Protected scope and documentation | All 3,720 protected hashes, 1,338 SDK export names, sole Python export, 246 descriptors, 32 functions and four type layouts match the A5 baseline. The committed A6 slice retains its exact 13-file scope. Six Markdown files, 175 local links, five anchors and five PowerShell blocks pass |

The Hangar run validates existing geometry/manifests, mass, carriage, effects, vulnerability and temporary-copy
generation/startup cases. It does not rebuild all canonical aircraft assets or validate GPU rendering.
Post-push logs are under `D:/ENV/tmp`: `architecture-a6-delivery-build.log`,
`architecture-a6-delivery-full-tests.log`, `architecture-a6-delivery-characterization.log`,
`architecture-a6-delivery-dist.log` and `architecture-a6-delivery-python-dist-compatibility.log`. Remote identity is in
`architecture-a6-delivery-push.json`; the fresh trace is `architecture-a6-delivery-replay-after.txt`. Committed-scope
results are in `architecture-a6-delivery-final-verification.json`; documentation results are in
`architecture-a6-delivery-doc-verification.log` and `architecture-a6-delivery-doc-commands.log`.

### Main integration, 2026-10-12

At the user's request, the existing `main` was fast-forwarded to the fully validated `03f512f` delivery, bringing
both maintenance passes and Architecture A1 through A6 into `main`. Remote main was an ancestor with no divergent
commits. Before the workflow documentation update, main's complete tree was identical to `03f512f`:
`29fc31b2513fd9e36be5e8b7ac2d15baad8d8c21`.

Only this status page and the [development workflow](development.md) change after integration. Source, tests,
public headers, Python, build configuration and resources retain the exact validated tree. The 596/596 full-suite
and 25/25 distributed Python results above remain applicable; they are not represented as new runs on main.
Fresh ABI, boundary, adapter and architecture gates passed 8/8 in 7.27 seconds. Protected source/resource hashes,
exports, Python identities and distribution provenance match the recorded A6 delivery. Both changed Markdown
files, 82 local links, four anchors and five PowerShell blocks pass, together with source identity and diff checks.
Logs are under `D:/ENV/tmp`: `architecture-a6-main-focused-tests.log`,
`architecture-a6-main-protected-verification.log`, `architecture-a6-main-doc-verification.log` and
`architecture-a6-main-doc-commands.log`.
Future implementation, documentation, commits and pushes use the existing `main`; another branch or worktree
requires an explicit user request.

### Earlier delivery evidence

The [native-engagement validation record](sdk/engagement-validation.md) reports a repaired functional suite, an
unchanged model suite, Python and boundary checks, native encounters, and an external browser integration. It also
records the initial allocation failure and the limits of the final checks. Preserve that distinction rather than
rewriting it as one uninterrupted green run.

The checked-in [CI workflow](../.github/workflows/ci.yml) configures a Windows UCRT64 headless build, runs CTest, and
rebuilds store/effect outputs plus one aircraft. The second maintenance pass adds launcher, distribution and SDK-only
component-install consumer acceptance through the same preset entry point. It does not build the Vulkan viewer.
Its header records that it had not yet run on GitHub when written; no remote run was inspected for this maintenance work.

When updating status:

1. Name the implementation revision, build/preset and the commands actually run.
2. Keep source inspection, recorded checks and fresh measurements separate.
3. Update the SDK reference and affected current-status statements together; mark replaced plans as historical.
4. Close a backlog/stage item only against its own acceptance criteria and evidence.
5. Keep agent/range/referee access, ABI compatibility, deterministic behavior and resource provenance explicit.
