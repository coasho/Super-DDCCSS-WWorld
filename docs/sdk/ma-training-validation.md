# MA training validation

The evidence manifest of the MA-training program's joint acceptance ([design](../superpowers/specs/2026-10-12-ma-system-training-readiness-design.md),
section 12; [plan](../superpowers/plans/2026-10-12-ma-system-training-readiness.md), M8). It records what was
run, on which build, with which result, and what is still pending. It does not restate the contract: that is
[mission-training.md](mission-training.md).

| Part | State |
| --- | --- |
| M8 generic: joint chains, generic joint cases, installed consumers, isolation, compatibility checks | **Implemented, corrected after the independent review of `d344e2b`, awaiting independent acceptance** (2026-10-16) |
| Full M8 and the program's completion | **Pending**: J5, J9, the combat parts of J3, J8 and J10, and complete Viewer acceptance consume inputs that are not supplied yet (section 6) |

First recorded on 2026-10-16 from `main` at `0cf0150` with the M8 changes (`d344e2b`); corrected the same day after the
independent review of `d344e2b` (`docs/architecture/modules/training/reviews/2026-10-16-m8-generic-d344e2b.md`): the
Python contracts the comparison holds and the artifacts it requires (section 4), the installed aircraft data the Python
replay flies (section 3), J8's pending state at its reset and J6's evidence (section 1). Corrected again after the
review of `3802e7a` (`docs/architecture/modules/training/reviews/2026-10-16-m8-3802e7a-test-scoping.md`): a record's
fields are held exactly and an added parameter's kind and default are read, not guessed (section 4); the installed
runtime data is held to an inventory the staging rules give, and missing data is refused before any World is made
(section 3).

## 1. Joint acceptance matrix

| ID | State | Evidence (current runs, section 5) |
| --- | --- | --- |
| J1 Composition | **Delivered** (M6 noncombat, accepted) | `python_sdk`'s `test_ma_composition`; `mission_package_consumer` runs the [composition example](ma-composition.md) twice on the installed package (identical traces, a reset generation equal to a fresh one) and under the training adapter with `same_step` and `next_step` autoreset |
| J2 P2P perception | **Delivered** | `ma_joint` J2 on the joint chain: the wing's radar stays off and its own track list empty; it holds the lead's track only from the tick after the lead's MA published it, received after the publication's time; it then designates it through the link (`sensor.designate_link`) and holds that designation, with the lead as its source |
| J3 Peer coordination | **Message and task part delivered**; releases pending | `ma_joint` J3 on the lossy chain (60% of attempts lost): the proposal and its acknowledgment are retransmitted until delivered and each appears in exactly one snapshot; the resubmitted request is answered `duplicate` and sends nothing; its id with another body is refused `request_id_reuse`; J4 on the joint chain: C2's resubmitted task request makes no second task. Duplicate releases need the combat extensions |
| J4 Typed C2 task | **Delivered** | `ma_joint` J4: the AVC role's task reaches the wing; the wing accepts, reports 50% and completion; C2's receipt goes `pending`, `executing`, `completed`; the LRE role's task is answered `unauthorized` and the wing never holds it |
| J5 Strike approval | **Pending** | Consent request/answer extension handlers and their fixtures are not installed ([section 15](mission-training.md#15-extension-seam-and-pending-gates)) |
| J6 Control ownership | **Delivered** for C2's direct command, its control designation (FA revocation) across a platform's services and its overrides; one exact gap pending (below) | `ma_joint`, on the joint chain: while C2's HSA command flies, the wing's flight requests from components 1 and 2 are both refused `authority_held`, naming C2's activity; the wing may not cancel C2's activity; the LRE role's control designation is `unauthorized`; once C2 cancels its own command the wing's next flight request executes as its service's activity. `ma_joint`, directly: C2 designates the lead's waypoint mode alone - `fsim.guidance.hsa` is then not allowed in the snapshots of both of the lead's services (lead-flight and lead-sensors), and lead-flight's own HSA request is refused `not_granted`; C2 designates HSA back and the next request executes. C2's speed override is held in both of the lead's services' snapshots and binds lead-flight's own 260 m/s HSA request: true airspeed changes by -12.8 m/s in 12 s under it, +25.3 m/s in the same episode without it; C2's cancellation removes it from both. `ma_service_binding` (current run) covers the native paths beneath: "an activity is its owner's; C2's override reaches the owner's flight, not the agent made last", "the default policy holds no grant on a bound platform, and the platform stays under grants" and "a capability two controllers act on is taken back whole - the owner's activity too". **Pending:** whether the MA's own commanded speed resumes after C2 cancels an override is not established by these cases |
| J7 Link/package change | **Delivered** | `ma_joint` J7: a shared link track's `age_s` is simulated time since its report, `stale` past 10 s and dropped 20 s after it; the lost-link contingency is raised at the first tick C2 has been silent for its 4 s threshold and cleared when C2 is heard again; the wing's leader proposal reaches C2 once; the package the AVC role forms (leader: the lead) is both MAs'; the LRE role's package change is answered `unauthorized` by both |
| J8 Episode lifecycle | **Generic part delivered**; the combat part through endpoints pending | `ma_joint` J8, pending state at the reset (on a 2.5 s link): established before it - a task accepted by the wing whose receipt is `executing` at C2, the wing's reliable message still on the modelled link (its receipt `pending`, its destination's delivery `pending`, C2 not holding it), and a request held in the gateway's queue (`queued`, submitted but not stepped); after it, for 8 ticks, every service's snapshot equals a fresh episode's, no receipt, task, operator or transmission of the old generation remains and neither message ever arrives; truth and costs equal the fresh episode's. `ma_joint` J8, chain: a reset at the chain's tick 23 (the package requests submitted, not stepped) replays the chain to the same trace, truth and costs as a fresh episode. Two episodes stepped together, one reset midway, each keep their solo trace. Native weapons, decoys and stores at a reset: the [native-state reset fixture](native-state-reset-fixture.md) through `mission_episode` (M4, accepted). Policy memory: M6's composition reset cases. Weapons and decoys released by an actor's request need the combat extensions |
| J9 Outcome validity | **Pending** | Combat outcome reduction and its evidence are not supplied ([section 15](mission-training.md#15-extension-seam-and-pending-gates)) |
| J10 Public consumers | **Generic part delivered**; the combat chain pending | `mission_package_consumer` (section 3): installed C++, C99 and Python replay both joint chains to byte-identical traces, and a strictly isolated Python actor to the same trace; `python_sdk`'s `test_ma_joint` the same on the build tree, and the isolated proxy's refusals |

## 2. The joint chains

`tests/training/fixtures/mission_training/joint` holds two recorded chains and their configurations. `tools/validation/ma_contract/joint_chain.py
author` ran each scenario once through the public Python endpoints, deciding as it went (which task id the wing
answers, which activity C2's command flies), and recorded every submission as `<tick> <service id> <requests>`. The
simulation is deterministic, so every replay meets the recorded ids; `ma_joint` checks what each case means on the
replay, and every consumer must write the same trace. A trace is, for each tick, each service's snapshot in ascending
service id, then each submission's acknowledgment - every document as the SDK wrote it.

| Chain | Platforms | Services (ids) | Ticks | Submission lines | Seeds (base, env, local -> world) | Trace SHA-256 |
| --- | --- | --- | --- | --- | --- | --- |
| `chain.joint.txt` | lead, wing, bogey (F-16C, JSBSim); C2 nodes crc (AVC), obs (LRE) | lead-flight (1), lead-sensors (2), crc-c2 (4), obs-c2 (5), wing-ma (9007199254740995) | 29 | 22 | 18446744073709551615, 8001, 0 -> 14940144872676502023 | `3d78fdd36042803bed6e758d1637ca809894410f192b86c9933e43efb1cf5bf5` |
| `chain.lossy.txt` | lead, wing; crc; one net losing 60% of attempts | lead-flight, lead-sensors, crc-c2, wing-ma | 17 | 4 | 18446744073709551615, 8002, 0 -> 4365543517816406494 | `9e7ec1481c5c823e7fc662ad8ec750b1df56eeae923a1d3d2d2525b513a9fc25` |

Timing: `dt_s` 1/120 s, `frame_skip` 4, `decision_steps` 30 - one decision tick is 1.0 s of simulated time. Roles,
services and components are not one to one: the joint chain has five scenario platforms (lead, wing and bogey
aircraft; crc and obs C2 nodes) and five services bound to four of them - the lead has two (lead-flight, lead-sensors),
the wing and each C2 node one, the bogey none; two components act behind the wing's service in J6; the composition
example (J1) runs four components behind one service. The lossy chain has three platforms and four services. Aspect: one deterministic geometry per chain - the bogey 0.25
degree north of the lead, head-on at 7,000 m; the C2 nodes on the ground. A published seed/aspect matrix is the
combat scenes' (J5, J9) and is pending with them.

## 3. Installed consumers and isolation (J10, generic)

`mission_package_consumer` installs the SDK and the Python package into its own prefixes under
`build/<preset>/mission-package-consumer` (never the `package-consumer` work directory), compiles the C++ and C99
consumers against the installed headers alone (checked in the consumers' `compile_commands.json`), runs every
consumer with only the installed `bin` and the Windows system directories on its DLL path, and the Python consumer
with only the installed package on `PYTHONPATH`; each reports the library and package it loaded. The C++ and C99
consumers fly the installed SDK's aircraft (`FSIM_AIRCRAFT_PATH` its `share/flightsim/aircraft` and `stores`); the
Python consumer, its isolated actor's process and the composition example fly the installed Python package's own -
without it, a development library's compiled-in source aircraft directory comes first (`src/foundation/io/AssetResolver.cpp`,
unchanged). The runtime data the chains fly is held to an expected inventory, not to whatever was installed: the gate
applies the staging rules (`cmake/StageDesigns.cmake`) to the source tree's F-16C and released stores in a private
directory, and each of the 113 files that gives - `f16c.xml`, its `Engines/`, model, payloads, radar cross section,
damage data and pieces, and the stores' flight models - must be installed in both packages, byte for byte; an omitted
file fails the gate instead of dropping out of the list. The Python consumer checks the same inventory first
(`runtime-data.sha256`), before it makes any World, and its refusal probes share that preflight. With the installed
Python package's `f16c.xml` missing, or the engine file it names (`Engines/f16c_engine0.xml`) missing, the preflight
refuses (exit 4) naming the installed path - although a development library would find the source tree's copies after
the installed ones (`src/foundation/io/AssetResolver.cpp`, kept as it is). With `f16c.xml` corrupted it refuses for the hash; and
with the presence check alone the corrupted file reaches the SDK, which refuses the joint chain's episode (exit 3):
the installed copy is the one read. Each file is restored and the whole inventory re-checked. The provenance manifest
lists 275 installed files with their SHA-256. On the Release run below: `libfsim.dll` `ccdcdbaf16dc49a6a760a87643cf7f7a9a9fe430974a07f14be22b2beb363623` (the same in the SDK and the Python package),
`_native.pyd` `a385bb357d38c469c889ee3269f70aebc41ee8c7a560098ac8c49b60d2a1dc1f`.

Each consumer replays both chains: C++ through `fsim/MissionEpisode.h`, C99 through `fsim_mission_env.h` (an
acknowledgment is the receipts document a submission answers with), Python in-process and with the wing's actor in
a process of its own (`fsim.isolation.MissionEndpointProcess`: its endpoint's three calls alone, truth publication and
recording off). The gate requires the four traces of each chain to be byte-identical; on both presets they were, with
the digests above - the headless and Release builds' traces the same, and the same as a build-tree replay
(`joint_chain.py replay`). `test_ma_joint` holds the isolated
proxy to its boundary: no range member on it, no referee handle, episode or referee document across, and no answer
from an earlier generation until the range re-binds it.

## 4. Compatibility

`tools/validation/ma_contract/baseline.py compare` captures the current builds' exports and public types - into a work directory it
clears first, so nothing an earlier run left is read - and holds them to the frozen
[compatibility baseline](ma-compatibility-baseline.md) (`e1945b9`), which it never writes. Every artifact of each
selected build is required: the core build's `libfsim.dll`, `_native.pyd` and package always, the render build's with
`--release-build`. Each recorded Python description keeps its contract - signatures with their defaults, enumerator
values, a record's fields exactly (a record is unpacked by position, so a new field - defaulted or not - is a new
record type), class members with their kinds and signatures, exception bases, constants and the runtime owner of each
name. A signature may only gain parameters an old call never passes: defaulted positional or keyword-only ones,
`*args` or `**kwargs`. Each added parameter's kind and default are read from its text outside quotes and brackets -
`c: '='` has no default - and an added `/`, or an addition that cannot be classified, is refused. All 2,326 recorded
signatures classify. Result: 19
comparisons, 0 failures; 620 (headless) and 629 (Release) Python descriptions keep their contracts -
`libfsim.dll` 1,338 baseline exports all present (1,456 now, headless and Release), 68 vtable/typeinfo symbols
unchanged, `libfsim_vision.dll` 57 exports unchanged, the Python call surface a superset (565 -> 639 headless,
580 -> 654 Release), every recorded Python module name present; 267 public C++ types and 120 C structs with
unchanged layouts, 718 and 415 enumerators with unchanged values, 50 public vtables with unchanged slots. Planted
against a doctored copy of the baseline - an export removed, a type's size, an enumerator and a vtable slot changed -
the same command fails on each of the four. `ma_baseline_checker` (17 cases) holds the comparison to its own
refusals: a changed signature or default, enumerator value, record field, member kind or signature, runtime owner,
exception base or constant fails it, and so do a record's added field, a required keyword-only addition hidden behind
an `=` in its annotation and an added `/`; an addition old calls never pass does not. Every admitted or refused
addition is checked against real functions: each admitted one still takes every call written against the frozen
signature, each refused one breaks one; and a missing DLL, Python package or extension
module, an incomplete selected Release build, a reused work directory and the baseline as the work directory each fail
the real command. `baseline.py verify` re-hashed the baseline (66 artifacts, unchanged) and `baseline.py run` ran its
frozen old-header C++, C99 and Python consumers against this build's installed library.

## 5. Verification records

| Check | Result (2026-10-16) |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| `ma_joint` | 10 cases, 696 assertions |
| `python_sdk test_ma_joint` | 2 cases |
| `ma_baseline_checker` | 17 cases |
| Affected headless gates at `3802e7a` (`ma_joint`, `ma_baseline_checker`, `ma_service_binding`, the M4-M7 gates, contract, boundary and architecture checks, `c_abi`, `python_sdk`, installed consumers) | 20/20 passed in 663.31 s |
| Affected Release gates at `3802e7a` (the same with `equipment_package_consumer`) | 21/21 passed in 683.43 s |
| After the second correction: targeted tiers (`testing_scope_checker`, `ma_baseline_checker`, `mission_package_consumer`) | 3/3 headless, 3/3 Release; documentation checks 4/4 |
| Full headless suite, once on the frozen revision of the second correction | 690/690 passed in 1227.65 s (619/619 in 1066.28 s at `3802e7a`) |
| Baseline | `verify`: 66 artifacts, 0 changed or missing; `run`: the frozen C++, C99 and Python (`agent.py`, `links.py`, `c2.py`) consumers pass on this build's installed library; `compare`: 19 comparisons, 0 failures (section 4) |
| Test inventory | 690 tests headless, 744 Release; 37 labelled `ma-training` (the contention cases among them, label `stress`; `python_sdk` is one CTest test per module) |

Environment: Windows, MSYS2 UCRT64 GCC 16.2.0 (Rev3), CMake 4.4.3, CPython 3.13.12, presets `ucrt64-headless` and
`ucrt64-release` (both Release builds), `TMP`, `TEMP` and `TORCH_EXTENSIONS_DIR` under `D:/ENV/tmp`. Logs:
`D:/ENV/tmp/m8/logs/`, `D:/ENV/tmp/m8fix/logs/` and, for the second correction, `D:/ENV/tmp/testing/logs/`. No renderer, viewer or lifecycle boundary changed in M8, so no renderer regression beyond the
Release build and its suite was required.

## 6. Pending, and limits

- J5 and J9, the combat parts of J3 (duplicate releases), J8 (weapons and decoys an actor's request released) and J10
  (the combat chain): they consume the frozen extension handlers, outcome reduction and their fixtures, none of which
  is installed.
- Complete Viewer acceptance: the [Viewer evidence](viewer-validation.md) of 2026-10-12 is a dated diagnostic
  baseline, not the installed public-endpoint joint acceptance; the full-scenario combat Viewer run is pending with
  the combat chain.
- 2026-10-17 (HC-07): the prerequisites above are still not supplied. The engagement registry says
  `runtime_installed: false`, and no outcome reduction exists, so J5, J9 and the combat parts of J3, J8 and J10
  remain pending with the combat Viewer run and the seed/aspect matrix. The generic joint cases passed in the HC-11
  full suites. See the [HC-07 record](../architecture/validation/hc-07-viewer-and-joint.md).
- The joint chains are one deterministic seed and aspect each. Their recorded ids hold because the simulation is
  deterministic; a change to native behaviour that moves an id or a tick fails the replay checks, and the chains are
  then authored again with `joint_chain.py author` and reviewed.
- J6: whether the MA's own commanded speed resumes after C2 cancels its override is not established (section 1).
- A development library still resolves a missing installed aircraft from its source tree (section 3, kept as it is);
  the gate's preflight refuses that case before any World is made, for the files of the expected inventory.
- On the lossy net, C2's commands and the MAs' answers are sent once, not retried: a lost one is simply lost. The
  lossy chain therefore exercises the reliable DMS sends; C2's task and its resubmission are exercised on the main
  chain.
- Nothing here is a policy, a learner or a formal MA runtime: those are the upper layer's, after full M8.
