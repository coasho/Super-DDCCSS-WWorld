# MA training validation

The evidence manifest of the MA-training program's joint acceptance ([design](../superpowers/specs/2026-10-12-ma-system-training-readiness-design.md),
section 12; [plan](../superpowers/plans/2026-10-12-ma-system-training-readiness.md), M8). It records what was
run, on which build, with which result, and what is still pending. It does not restate the contract: that is
[mission-training.md](mission-training.md).

| Part | State |
| --- | --- |
| M8 generic: joint chains, generic joint cases, installed consumers, isolation, compatibility checks | **Implemented, awaiting independent acceptance** (2026-10-16) |
| Full M8 and the program's completion | **Pending**: J5, J9, the combat parts of J3, J8 and J10, and complete Viewer acceptance consume inputs that are not supplied yet (section 6) |

Recorded on 2026-10-16 from `main` at `0cf0150` plus the M8 changes this record is committed with.

## 1. Joint acceptance matrix

| ID | State | Evidence (current runs, section 5) |
| --- | --- | --- |
| J1 Composition | **Delivered** (M6 noncombat, accepted) | `python_sdk`'s `test_ma_composition`; `mission_package_consumer` runs the [composition example](ma-composition.md) twice on the installed package (identical traces, a reset generation equal to a fresh one) and under the training adapter with `same_step` and `next_step` autoreset |
| J2 P2P perception | **Delivered** | `ma_joint` J2 on the joint chain: the wing's radar stays off and its own track list empty; it holds the lead's track only from the tick after the lead's MA published it, received after the publication's time; it then designates it through the link (`sensor.designate_link`) and holds that designation, with the lead as its source |
| J3 Peer coordination | **Message and task part delivered**; releases pending | `ma_joint` J3 on the lossy chain (60% of attempts lost): the proposal and its acknowledgment are retransmitted until delivered and each appears in exactly one snapshot; the resubmitted request is answered `duplicate` and sends nothing; its id with another body is refused `request_id_reuse`; J4 on the joint chain: C2's resubmitted task request makes no second task. Duplicate releases need the combat extensions |
| J4 Typed C2 task | **Delivered** | `ma_joint` J4: the AVC role's task reaches the wing; the wing accepts, reports 50% and completion; C2's receipt goes `pending`, `executing`, `completed`; the LRE role's task is answered `unauthorized` and the wing never holds it |
| J5 Strike approval | **Pending** | Consent request/answer extension handlers and their fixtures are not installed ([section 15](mission-training.md#15-extension-seam-and-pending-gates)) |
| J6 Control ownership | **Delivered** | `ma_joint` J6: while C2's HSA command flies, the wing's flight requests from components 1 and 2 are both refused `authority_held`, naming C2's activity; the wing may not cancel C2's activity; the LRE role's control designation is `unauthorized`; once C2 cancels its own command the wing's next flight request executes as its service's activity |
| J7 Link/package change | **Delivered** | `ma_joint` J7: a shared link track's `age_s` is simulated time since its report, `stale` past 10 s and dropped 20 s after it; the lost-link contingency is raised at the first tick C2 has been silent for its 4 s threshold and cleared when C2 is heard again; the wing's leader proposal reaches C2 once; the package the AVC role forms (leader: the lead) is both MAs'; the LRE role's package change is answered `unauthorized` by both |
| J8 Episode lifecycle | **Generic part delivered**; the combat part through endpoints pending | `ma_joint` J8: stopped midway - a task open, the proposal, package commands and C2's status on the link - and reset, the episode replays the chain to the same trace, truth and costs as a fresh one; two episodes stepped together, one reset midway, each keep their solo trace. Native weapons, decoys and stores at a reset: the [native-state reset fixture](native-state-reset-fixture.md) through `mission_episode` (M4, accepted). Policy memory: M6's composition reset cases. Weapons and decoys released by an actor's request need the combat extensions |
| J9 Outcome validity | **Pending** | Combat outcome reduction and its evidence are not supplied ([section 15](mission-training.md#15-extension-seam-and-pending-gates)) |
| J10 Public consumers | **Generic part delivered**; the combat chain pending | `mission_package_consumer` (section 3): installed C++, C99 and Python replay both joint chains to byte-identical traces, and a strictly isolated Python actor to the same trace; `python_sdk`'s `test_ma_joint` the same on the build tree, and the isolated proxy's refusals |

## 2. The joint chains

`tests/fixtures/mission_training/joint` holds two recorded chains and their configurations. `tools/ma_contract/joint_chain.py
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
services and components: five services on five platforms in the joint chain (the lead's two services on one
aircraft, two C2 services without an aircraft), two components behind the wing's service in J6; the composition
example (J1) runs four components behind one service. Aspect: one deterministic geometry per chain - the bogey 0.25
degree north of the lead, head-on at 7,000 m; the C2 nodes on the ground. A published seed/aspect matrix is the
combat scenes' (J5, J9) and is pending with them.

## 3. Installed consumers and isolation (J10, generic)

`mission_package_consumer` installs the SDK and the Python package into its own prefixes under
`build/<preset>/mission-package-consumer` (never the `package-consumer` work directory), compiles the C++ and C99
consumers against the installed headers alone (checked in the consumers' `compile_commands.json`), runs every
consumer with only the installed `bin` and the Windows system directories on its DLL path, and the Python consumer
with only the installed package on `PYTHONPATH`; each reports the library and package it loaded. Its provenance
manifest lists 49 installed files with their SHA-256. On the Release run below: `libfsim.dll`
`ccdcdbaf16dc49a6a760a87643cf7f7a9a9fe430974a07f14be22b2beb363623` (the same in the SDK and the Python package),
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

`tools/ma_contract/baseline.py compare` captures the current builds' exports and public types and holds them to the
frozen [compatibility baseline](ma-compatibility-baseline.md) (`e1945b9`): 19 comparisons, 0 failures -
`libfsim.dll` 1,338 baseline exports all present (1,456 now, headless and Release), 68 vtable/typeinfo symbols
unchanged, `libfsim_vision.dll` 57 exports unchanged, the Python call surface a superset (565 -> 639 headless,
580 -> 654 Release), every recorded Python module name present; 267 public C++ types and 120 C structs with
unchanged layouts, 718 and 415 enumerators with unchanged values, 50 public vtables with unchanged slots. Planted
against a doctored copy of the baseline - an export removed, a type's size, an enumerator and a vtable slot changed -
the same command fails on each of the four. `baseline.py verify` re-hashed the baseline (66 artifacts, unchanged) and
`baseline.py run` ran its frozen old-header C++, C99 and Python consumers against this build's installed library.

## 5. Verification records

| Check | Result (2026-10-16) |
| --- | --- |
| UCRT64 headless and Release builds, warnings as errors | Passed, 0 warnings each |
| `ma_joint` | 8 cases, 438 assertions |
| `python_sdk` `test_ma_joint` | 2 cases; `python_sdk` 261 cases in all |
| Affected headless gates (`ma_joint`, the M4-M7 gates, contract, boundary and architecture checks, `c_abi`, `python_sdk`, installed consumers) | 18/18 passed in 609.02 s |
| Affected Release gates (the same with `equipment_package_consumer`) | 19/19 passed in 707.93 s |
| Full headless suite | 618/618 passed in 1086.22 s |
| Baseline | `verify`: 66 artifacts, 0 changed or missing; `run`: the frozen C++, C99 and Python (`agent.py`, `links.py`, `c2.py`) consumers pass on this build's installed library; `compare`: 19 comparisons, 0 failures (section 4) |
| Test inventory | 618 tests headless, 672 Release; 18 labelled `ma-training` |

Environment: Windows, MSYS2 UCRT64 GCC 16.2.0 (Rev3), CMake 4.4.3, CPython 3.13.12, presets `ucrt64-headless` and
`ucrt64-release` (both Release builds), `TMP`, `TEMP` and `TORCH_EXTENSIONS_DIR` under `D:/ENV/tmp`. Logs:
`D:/ENV/tmp/m8/logs/`. No renderer, viewer or lifecycle boundary changed in M8, so no renderer regression beyond the
Release build and its suite was required.

## 6. Pending, and limits

- J5 and J9, the combat parts of J3 (duplicate releases), J8 (weapons and decoys an actor's request released) and J10
  (the combat chain): they consume the frozen extension handlers, outcome reduction and their fixtures, none of which
  is installed.
- Complete Viewer acceptance: the [Viewer evidence](viewer-validation.md) of 2026-10-12 is a dated diagnostic
  baseline, not the installed public-endpoint joint acceptance; the full-scenario combat Viewer run is pending with
  the combat chain.
- The joint chains are one deterministic seed and aspect each. Their recorded ids hold because the simulation is
  deterministic; a change to native behaviour that moves an id or a tick fails the replay checks, and the chains are
  then authored again with `joint_chain.py author` and reviewed.
- On the lossy net, C2's commands and the MAs' answers are sent once, not retried: a lost one is simply lost. The
  lossy chain therefore exercises the reliable DMS sends; C2's task and its resubmission are exercised on the main
  chain.
- Nothing here is a policy, a learner or a formal MA runtime: those are the upper layer's, after full M8.
