# Current capabilities and milestone status

Reviewed: 2026-10-12. First maintenance pass: `2b83f24` (implementation baseline `32e78a6`).
Second maintenance pass: `f8cfec9`; Architecture A1: `6652584`. Architecture A2 is a working-tree change based on
`6652584`, on `codex/architecture-lifecycle`.

This is the entry point for what exists now, which delivery it belongs to, and what remains open. The first
maintenance pass consolidated documentation and development guidance, corrected outdated descriptions, and retired
the obsolete tactical prototype and orphan visual/compiler artifacts. The second pass aligns build/run/test presets
and repairs standalone SDK packaging and consumer acceptance. Architecture A1 starts the third pass by enforcing
native module dependencies, separating the Mission Systems C adapter and assigning stateless launch eligibility
to the weapon module. A2 retires private MA consent bookkeeping and separates C/Python native DMS/C2 adapters
while retaining public history and original state-transition timing. Public interfaces, ABI layouts, physics and
aircraft model bytes remain unchanged.

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
| **Second maintenance pass** | Consistent configure/build/run/test/dist presets, first-build entry, independent SDK runtime data and installed-consumer acceptance. | Completed and locally verified below: 535 functional CTest cases, real launch/build/dist commands and independent installed consumers. Native module architecture and simulation models are unchanged. |
| **Architecture A1** | Executable native dependency boundaries, Mission Systems C adapter separation and weapon-owned launch eligibility. | Implemented and locally verified below: 538 functional cases plus three architecture checks; 1,338 DLL export names match the baseline. Internal ownership changes preserve public interfaces and simulation behavior. The broader architecture queue remains open. |
| **Architecture A2** | MA private consent retirement and C/Python native DMS/C2 adapter separation. | Implemented and locally verified below: 547 functional cases plus three architecture checks, unchanged exports and installed-consumer/distribution acceptance. Public history remains queryable; aggregate history reclamation is deferred. |

The third maintenance pass includes bounded A1 and
[A2 lifecycle/adapter slices](superpowers/specs/2026-10-12-architecture-lifecycle-design.md).
The remaining [architecture queue](superpowers/specs/2026-10-11-architecture-maintenance-design.md#subsequent-work)
includes a public-history retention contract, further C ABI/Python domain separation, store bindings, operational
geometry, weapon observations, viewer/platform boundaries and DLL export ownership. A2 retains map traversal,
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
- Kept showcase/plume fixtures because current model tests and viewer-performance tools consume them. Their
  carrier/missile poses and detonation timing are scripted; native target flight, damage and Flow fire do not
  establish native missile-flight acceptance.
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

Fresh local checks on `codex/architecture-lifecycle`, based on committed A1 `6652584`, using UCRT64 Release
headless, warnings as errors and CPython 3.13.12. A2 remains an uncommitted working-tree change. Compiler temporary
directories remain under `D:/ENV/tmp`.

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
