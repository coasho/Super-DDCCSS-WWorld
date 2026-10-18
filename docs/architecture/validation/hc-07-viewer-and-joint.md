# HC-07 Viewer, manual-control, performance and joint verification

Status: diagnostic evidence subset independently accepted at `465b7af` (2026-10-17). **HC-07 stays open.** Remaining
gates include:
- **4K performance gate (HC-45, new):** it fails on three of four scenes.
- **Missing prerequisites:** the combat joint cases, the combat Viewer run and the seed/aspect matrix depend on
  inputs that are not supplied (HC-06).
- **Interaction coverage:** the sampled programmatic mouse, replay and resize cases at `2a09756` are independently accepted. A human-operated session, other-panel input, chase/overview variants and `--no-zoom-to-cursor` remain unverified. See the [interaction acceptance](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892).

Final-program acceptance is not claimed. The [summary](hc-07-evidence.json) holds the measurements and hashes.

**Latest independent acceptance at `d6bfb54`: HC-47's two helper-check gaps are closed.** All eight prior
false accepts are rejected by version 2; 149 pure results are as expected, including saved-run/real-log controls.
HC-48/49/50 remain closed. Version 2's live path is not claimed verified; exercise it before future automated-gate
reuse, not in this GUI-excluded assignment. See [acceptance](hc-45-review-2026-10-17.md#7-hc-47-version-2-correction-acceptance-at-d6bfb54).
The dated historical findings below remain preserved. No full suite, GUI or performance scene was repeated.

**Latest interaction acceptance at `5c80892`:** sampled mouse/replay/resize evidence at `2a09756` is accepted and
HC-51 is closed. HC-52 retains a nonblocking resize-helper completeness gap; actual saved frames are valid. See
the [review](hc-07-review-2026-10-17.md#6-interaction-subset-and-hc-51-acceptance-at-5c80892). HC-07 remains open on the listed performance, prerequisite and human/uncaptured gates.

## 1. What ran, on what

- **Source:** main `66dc43d` (HC-11 accepted).
- **Binaries:** the `ucrt64-release` build. All 72 staged executables, DLLs and extensions are byte for byte the ones
  HC-11's full suites tested, and both presets' staged data still has HC-11's digest (234 files). Neither build was
  rebuilt, and no full suite ran (as directed).
- **Hardware:** NVIDIA GeForce RTX 5070 Ti, driver 596.36; 3840 x 2160 fullscreen.
- **GUI slot:** one Viewer at a time, and no build, test or other Viewer alongside.
- **Background load:** other applications stayed open, at 4-37 % CPU and 16 % GPU before the runs. They are not this
  repository's and were left alone.
- **Tools:** the maintained tools (`viewer_perf.py`, `strike_acceptance.py`), plus two scratch checks in
  `D:/ENV/tmp/hc07/`, `manual_controls.py` and `reconnect.py`, whose hashes are in the summary.

## 2. Results

| Case | Result |
| --- | --- |
| 4K performance gate, run once and repeated once (`viewer_perf.py`, 160 fps target) | **Fails on 3 of 4 scenes in both runs** (section 3); reheat passes |
| Replay at 3840 x 2160 (`viewer_perf.py --shots`) | 26 frames of the four scenes, every one 3840 x 2160 and inspected: models, effects, fire and smoke as recorded, no black or broken frame |
| Complete live fullscreen strike (`strike_acceptance.py`) | **Passes.** Native and Viewer exit 0 after 40.03 and 41.95 wall seconds. Its records match the dated 2026-10-12 run (details below) |
| Demo, fullscreen (`--demo --fullscreen --screenshot`) | **Visual evidence accepted:** 3840 x 2160 frame (8 aircraft, 160 fps on its panel). Standalone exit/cleanup are author observations, not parsed measurements in the summary; the separate controls run measures lifecycle (HC-47) |
| Manual controls through the Viewer's window input (`manual_controls.py`) | **15/15 pass** for the demo's pause, single step, time factor and quit (section 4) |
| A world replaced under the same name (`reconnect.py`) | **Passes.** The Viewer logs "world 'hc07-reconnect' was replaced; attaching to the new one". Frames 000-002 show `old-one` (1,525-1,556 m), frames 003-007 the fresh `new-one` (from 2,345 m), and it exits 0. Checked from its log and frames; the HC-47 run asserts them (see [HC-47 correction](#hc-47-correction-2026-10-17)) |
| Generic joint cases | Pass in HC-11's accepted full suites on both presets: `ma_joint`, `python_sdk test_ma_joint`, `mission_package_consumer`, `ma_service_binding`, isolation and composition. They were not run again |

Live strike details:
- **Timeline:** launch 13.033 s, hit 19.533 s, captures at 14.033 and 19.833 s; 1,200 native frames.
- **Target:** crew lost, one engine out, three torn panels, airborne at the end, 614 post-hit samples.
- **Consistency:** the live result matches its preflight.
- **Captures:** they are visually equivalent to 2026-10-12's. The pixels differ by a mean of 1.9/255 at launch and
  4.9/255 at the hit, where the fire and clouds vary from run to run, so image parity is not claimed.

## 3. The 4K performance gate (HC-45, new)

The target: after five seconds of warmup, every second must reach at least 158.4 fps, and no frame may take more than
12.5 ms.

| Scene | Run | Mean fps | Lowest second (at) | Worst frame, ms (at) | Frames over 12.5 ms | Gate |
| --- | --- | --- | --- | --- | --- | --- |
| fleet | 1 / 2 | 160.0 / 160.0 | 159.7 / 159.6 | 18.70 (25.6 s) / 18.99 (25.7 s) | 2 / 2 | **fail** (slow frames) |
| strike (Flow) | 1 / 2 | 156.2 / 156.1 | 124.8 (10 s) / 123.8 (10 s) | 14.76 / 14.79 | 32 / 24 | **fail** |
| hit | 1 / 2 | 152.9 / 158.8 | 93.8 (27.1 s) / 132.8 (27.1 s) | 62.20 / 58.80 (27.1 s) | 65 / 24 | **fail** |
| reheat | 1 / 2 | 160.0 / 160.0 | 159.9 / 159.9 | 8.35 / 6.91 | 0 / 0 | pass |

Further details:
- **Where the drops fall:** strike falls short from 8 to 11 s, at the impact and fireball; hit falls short in its
  aftermath, from 20 to 32 s, with NVIDIA Flow solving.
- **What the frames show:** in those seconds the CPU-side recording time is most of the frame.
- **Dated baseline:** the same three scenes already failed the gate on 2026-10-12, before the native showcase
  replaced the scripted hit.
- **Today's lowest seconds:** for strike and hit they are lower than that baseline. The hit scene is a different
  workload now, and this machine was not idle, so a regression is not established, and neither is its absence.
- **Not done:** no optimisation and no quality change was made, and no threshold was relaxed.
- **Closure (HC-45):**
  - an idle-machine measurement;
  - a before/after comparison with a Viewer from before the migration (`viewer_perf.py --exe`) to settle whether
    it regressed;
  - the visualization plan's performance work, without lowering quality.

## 4. Manual controls through the window's input

`manual_controls.py` starts `flightsim-viewer --demo --fullscreen` and posts each key to the Viewer's own window as
`WM_KEYDOWN`/`WM_KEYUP` with its scan code, so the keys never reach another application. They travel the Viewer's
event path: VSG's window events, ImGui and the key handler. The script reads each effect, read-only, from shared memory:
- the demo's control block;
- the newest published snapshot's simulation time, read under its sequence lock.

| Check | Measured |
| --- | --- |
| Running at time factor 1 | 0.9999 simulated s per wall s |
| Space pauses | 0 s of simulated time in 2 s |
| `.` three times, held | 3 requests taken; the simulation advanced 0.05 s, three agent steps of 1/60 s; still held afterwards |
| Space resumes | 0.9998 |
| `[` | time factor 0.5; 0.49997 measured |
| `]` twice | time factor 2.0; 1.99974 measured |
| Escape | the Viewer exits 0 in 0.43 s; the demo publisher stops; both shared-memory segments are released |

This is the keyboard's own path, but not a human at the machine.

The first attempt passed 11 of 15. Its four rate checks read the world header's step counter, which the demo publisher
does not maintain, while the key effects were already right in the control block. The script was corrected to read
the snapshot's simulation time and run again. Both attempts' logs are kept.

## 5. Not verified, and why

| Case | Missing prerequisite | Owner |
| --- | --- | --- |
| J5 strike approval, J9 outcome validity; the combat parts of J3 (duplicate releases), J8 (actor-released weapons and decoys) and J10 (the combat chain) | The runtime handlers for the five request kinds and their registration seam (`ma-engagement-registry.json`: `runtime_installed: false`), and the versioned outcome-reduction contract. Neither is supplied ([mission-training.md, section 15](../../sdk/mission-training.md#15-extension-seam-and-pending-gates)) | HC-06: the extension and native-input owners, then Claude's integration |
| The published seed/aspect matrix | It belongs to the combat scenes (J5, J9) | With HC-06 |
| The installed public-endpoint combat Viewer run | The combat chain. A strictly isolated run publishes no truth, so its Viewer evidence is a labelled diagnostic run of that chain | With HC-06 |
| A human-operated session | A person at the machine; the checks in section 4 are input-driven | The user, or whoever accepts HC-07 |
| Mouse (rotate, pan, zoom), the replay timeline slider and replay keys, and window resize | Verified through the window's input on 2026-10-17 (see [Mouse, replay and resize](#mouse-replay-and-resize-2026-10-17)). Not covered: mouse input over the other panels, the chase and overview cameras under the mouse, `--no-zoom-to-cursor` | Viewer owner, at the next Viewer acceptance, for what is not covered |
| The 4K performance gate | Failing (section 3) | HC-45 |
| J6: does the MA's own speed resume after C2 cancels its override? | A separate evidence gap | HC-02 |

## 6. Also found

- **HC-46 (new, nonblocking):** `--no-gui` is documented as "just the rendered scene, no panels or labels". Yet the
  strike captures show the Mission Systems windows, and so did 2026-10-12's, so this is not a regression.
- **Demo control column:** the demo's vehicle list leaves its control column empty. The ARCH-08 capture did too, and the
  built-in demo's aircraft have no SDK command. A published SDK world shows its level (`actuator` in the reconnect
  frames).

## 7. Independent partial acceptance at 465b7af

The [independent review](hc-07-review-2026-10-17.md) accepts the dated diagnostic subset: live native event/readback
consistency, sampled replay/strike/demo images, the 15 programmatic keyboard/setup/lifecycle checks and observed
same-process World replacement. It independently confirms the performance failures and missing prerequisites.
No GUI, performance run, native build or full suite was repeated during review.

All eight raw performance streams match the reports: fleet/strike/hit fail both runs and reheat passes both under
the unchanged thresholds. Background load and changed hit workload leave migration regression unresolved; matched
workload/assets/settings and an idle machine are required for a controlled comparison. Temporal correlation with
impact/fire is a profiling lead, not proof of a bottleneck. HC-45 remains open.

All 20 listed evidence hashes, the 17/72 HC-11 binary hashes, both 234-file data digests and eight before-copy
hashes match. Other owners' payloads remain preserved and this commit changes documentation/evidence only.
Visual samples show the old/new vehicle switch and the documented --no-gui panels; HC-46 remains nonblocking.

**HC-47, nonblocking helper limit:** `summarize.py` hardcodes the standalone-demo exit/remaining-publisher fields,
so those are author observations for this run. The separate control run measures exit/cleanup. `reconnect.py`
records data without fail-closed assertions, so its exit alone is not acceptance; reviewed logs/images establish
this replacement. Before reuse as automated gates, record actual process outcomes and assert expected identities,
frames and cleanup with negative controls. No current Viewer failure is inferred.

HC-06 combat integration/reduction inputs, HC-02's J6 proof and remaining interaction coverage are still pending.
HC-07 and complete final-program acceptance are not claimed. Harness fixes, digest-order correction, blank demo
control cells and the HC-35/44 limits are reconciled in the review/register. Only documentation fast checks run;
nothing was staged, committed or pushed. Stop for the next user assignment.

## HC-47 correction (2026-10-17)

The [review](hc-07-review-2026-10-17.md) found that the summary gave the standalone demo's exit and remaining
publishers, and the staged data's match with HC-11, as constants, and that `reconnect.py` asserted nothing. Both checks
now have fail-closed helpers in `D:/ENV/tmp/hc4750/helpers/`. The [summary](hc-07-evidence.json), now
`fsim.hc07.evidence.v2`, reads only saved measurements; its other values are unchanged.

- **`demo_capture.py`:** runs the demo fullscreen and saves one frame. It measures and asserts:
  - the Viewer's exit code, 0;
  - that its log names the demo world `demo-<Viewer pid>`, mirrored from a separate publisher process;
  - no demo publisher before the run, and none left after it;
  - that the world's control block and segment are both released;
  - the saved frame's size, 3840 x 2160, and the log's line for it.
- **`reconnect_check.py`:** publishes `hc47-reconnect` with `old-one`, closes it, and publishes it again with
  `new-one`. It asserts:
  - each phase's published vehicle table, read from shared memory while it flies: `old-one` alone, then `new-one`
    alone, in different incarnations;
  - the Viewer's log order: attach, one replacement, attach again;
  - frames 0-7 in order, each 3840 x 2160, with frames on both sides of the replacement;
  - the Viewer's exit code, 0, and the world's segment released afterwards.
- **Negative controls:** each helper's `--self-test` gives doctored measurements to the same checks. Each one must
  fail, and a positive control must pass: 14 for the demo and 13 for the reconnect, all as expected.
- **Runs:** one of each, on the current Release Viewer (`dd875197b212f60c`, rebuilt for HC-49, so not the HC-11-tested
  binary). Both passed:
  - demo: exit 0 after 10.42 s; publisher process 24740 for `demo-19196`, gone afterwards; both segments released;
    3840 x 2160;
  - reconnect: exit 0; incarnations 4147846233 and 308058473; frames 000-002 before the replacement and 003-007 after,
    every one 3840 x 2160; the segment released.

  Each helper's first run passed too. Each was run again after a change to the helper, so that the recorded hash is
  the one that ran; both attempts are kept.
- **The original runs:** HC-07's own demo run did not record its exit or cleanup, so the summary now gives them as
  null, and the frame's size is read from the file. The original reconnect's log order and frame sizes are read from
  its log and frames; that run asserted nothing.
- **Staged data:** recomputed with HC-11's tree digest. Both presets still have HC-11's 234 files and digest
  (`603ea0553e18673f`).

These are short GUI checks, not performance runs, and no full suite ran.

### Version 2 checks (2026-10-17)

The [recheck](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b) found eight invalid records
that the checks above still accepted: four missing or partial demo measurements, three missing reconnect
measurements, and a reattach logged after all of the replacement's frames. The helpers that ran stay as they were.
Version 2, in `D:/ENV/tmp/hc47b/helpers/`, keeps their runs and corrects their checks:
- **Every measurement required:** each must be present, not null and correctly typed:
  - an integer exit code, the Viewer's run time and process ID;
  - both segment results, each true or false;
  - publisher lists of process IDs;
  - both phases' vehicle, step count and table, with a nonzero 32-bit incarnation, the two distinct;
  - each frame's size.

  A probe that cannot tell now gives null, never the passing answer: a failed `tasklist` query, or a segment that
  Windows does not report as not found.
- **Event order:** the log must attach before any frame, then replace exactly once after at least one frame, then
  attach again before any of the replacement's frames. The frames must run 0-7 in order.
- **Regressions** (`test_checks.py`, with no Viewer and no process):
  - **red/green:** version 1's checks accept all eight of the review's records; version 2 rejects them;
  - **self-tests:** 47 demo and 54 reconnect negatives, each rejected for its own reason, and both valid controls;
  - **valid controls:** both saved outcomes above pass version 2. The order check passes three real logs (the
    reconnect run above, its first attempt and HC-07's), and rejects each once its reattach is moved last;
  - **probes:** `tasklist` output captured on this machine; version 1's and version 2's probes under stubs (a failed
    query, an access-denied segment); and Windows' answer for a missing segment.

  149 results, all as expected.
- **Not run:** version 2's run path has not run against a Viewer, since no GUI run was in scope. Its probes' decisions
  are covered by the pure checks above. The saved outcomes were measured with version 1's probes.

The [summary](hc-07-evidence.json) adds these results and their hashes (`helper_checks_v2`).

## Mouse, replay and resize (2026-10-17)

The remaining interaction cases, through the Viewer's own window. Keys, mouse buttons, drags and the wheel were posted
to it: VSG reads a mouse message's position, and a key's virtual-key code, from the message, as a device's messages
carry them. The window was resized, minimised, maximised and restored with the ordinary window calls on it. This is
input-driven, not a person at the machine. The checks are in `D:/ENV/tmp/hc07i/`; the [summary](hc-07-evidence.json)
(`interaction`, schema v3) holds their outcomes, readings and hashes.

- **Binaries:** the current Release build: Viewer `dd875197b212f60c`, demo `ca518158b90cb81a`. The Viewer was rebuilt for
  HC-49 and differs from HC-11; the demo still matches its HC-11 hash. The runs are 3840 x 2160 fullscreen at 144 dpi, except the resize run.
- **Measured:**
  - the demo's pause flag, from its control block;
  - how much each frame's scene differs from the one before;
  - frame and client sizes;
  - the demo world's simulation time;
  - exit codes and cleanup.
- **Read by inspection:** from each frame's panel, the camera, eye and distance, and the replay's time, time factor,
  pause box and timeline. These are checked against what each input must do, by `readings_check.py` and the replay
  checks; 18 altered-reading controls are each rejected.

| Case | Result |
| --- | --- |
| Mouse, demo paused (21 frames) | Left and middle drags rotate at the same distance; the right button changes nothing. 20 wheel notches out take 40 m to 516 m, 5 in take it to 272 m (12 % a notch), and 5 out return it exactly. Space + left drag changes nothing while locked to a vehicle, and pans the ground at the same distance in the free camera. The farthest is 50,000,000 m and the nearest, detached, 25 m; at the nearest, drags keep the view above the ground. Space alone pauses; space with a drag does not |
| Wheel, free camera (9 frames) | The picture scales about the pointer when it is off the middle (error 9.8; about the middle, 15.9), and about the middle when the pointer is there (17.6; unscaled, 32.9). Zooming back out returns the eye |
| Replay keys, timeline and buttons (16 and 5 frames) | Space pauses and holds. From the first frame, `.` five times shows 0.5 s and two step clicks 0.7 s: one recorded frame each. `]` and `[` set x4 (12.0 s played in 3 s) and x0.5 (1.5 s). Home goes to the start. A click on the timeline seeks (30.8 s; 4.9 s while paused). At x8 the replay loops at its end. The pause box pauses |
| Window resize, demo windowed (7 frames) | Every frame is its client area at that moment: 1600 x 900; 2560 x 1440, also after minimising (client 0 x 0) and restoring; 3836 x 2050 maximised; 2560 x 1440 restored; 1200 x 900; 1600 x 900. None is stretched or blank, and the demo kept simulating (8.2 to 32.0 s) |

All five final runs exited 0, and each demo run cleaned up. The only warning is the one every start prints
(`GraphicsPipelineConfigurator`).

- **Found (HC-51, nonblocking):** the [Viewer guide](../../sdk/viewer.md)'s mouse description differs from the Viewer:
  - when the camera is detached, the wheel zooms towards the pointer (on unless `--no-zoom-to-cursor`). The guide says
    it changes nothing but the distance, and names a `--zoom-to-cursor` option that does not exist;
  - locked to a vehicle, space + left drag does nothing;
  - a detached camera's nearest distance is 25 m; 6 m is a followed vehicle's.

  The guide's description was corrected the same day (HC-51), from the camera code and these measurements.
- **Earlier attempts, kept:** four runs were repeated after a fault in the check, not in the Viewer:
  - the first mouse run began the free camera 40 m from the ground, and posted 130 wheel notches more slowly than
    its frames came;
  - the first zoom run judged zooming back out against a still frame's noise, while sharper tiles had loaded;
  - the first resize run read the demo's single-step count as its simulation measure;
  - the first replay probe's folder did not exist; the Viewer reported it and exited 1.
- **Not covered:**
  - a person at the machine;
  - mouse input over the other panels;
  - the chase and overview cameras under the mouse;
  - `--no-zoom-to-cursor`.
- **HC-47 version 2:** the mouse, zoom and resize runs used its probes for the demo's cleanup. Its own demo and
  reconnect run paths were not run.
