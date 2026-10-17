# HC-07 Viewer, manual-control, performance and joint verification

Status: verified where the inputs exist (2026-10-17); awaiting independent acceptance. **HC-07 stays open.** Two
things block it:
- **4K performance gate (HC-45, new):** it fails on three of four scenes.
- **Missing prerequisites:** the combat joint cases, the combat Viewer run and the seed/aspect matrix depend on
  inputs that are not supplied (HC-06).

Final-program acceptance is not claimed. The [summary](hc-07-evidence.json) holds the measurements and hashes.

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
| Demo, fullscreen (`--demo --fullscreen --screenshot`) | **Frame accepted:** a 3840 x 2160 frame inspected (8 aircraft, 160 fps on its panel). This run's exit and cleanup were not recorded; the HC-47 run measures and asserts them (see [HC-47 correction](#hc-47-correction-2026-10-17)) |
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
| Mouse (rotate, pan, zoom), the replay timeline slider and replay keys, and window resize | Not part of this run, and not covered by the checks above | Viewer owner, at the next Viewer acceptance |
| The 4K performance gate | Failing (section 3) | HC-45 |
| J6: does the MA's own speed resume after C2 cancels its override? | A separate evidence gap | HC-02 |

## 6. Also found

- **HC-46 (new, nonblocking):** `--no-gui` is documented as "just the rendered scene, no panels or labels". Yet the
  strike captures show the Mission Systems windows, and so did 2026-10-12's, so this is not a regression.
- **Demo control column:** the demo's vehicle list leaves its control column empty. The ARCH-08 capture did too, and the
  built-in demo's aircraft have no SDK command. A published SDK world shows its level (`actuator` in the reconnect
  frames).

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
