# HC-45 4K performance gate: idle-host measurement and before/after comparison

Status: evidence recorded (2026-10-17); its review's corrections, HC-48 to HC-50, made the same day (section 5);
awaiting independent acceptance. **HC-45 stays open.** By the user's decision, no performance change was made: the
hypotheses in section 3 are left for a dedicated performance assignment.

The [HC-07 review](hc-07-review-2026-10-17.md#2-hc-45-performance-failure-is-confirmed) asked for three things. Two
are done; the third is not:

| Requirement | Result |
| --- | --- |
| An idle-host measurement | Done: CPU 1-6 % and GPU 0-1 % before each run, against CPU 4-37 % and GPU 16 % during HC-07's runs |
| A controlled before/after comparison with matched workload, assets and settings | Done, two rounds. The current Viewer and the Viewer of `7948dc4`, from before the architecture migration, fail the same three scenes. **Neither a regression nor equivalence is established**: at strike, the current Viewer's lowest seconds are 1.4-1.5 fps lower (section 1) |
| Meeting the target through the visualization plan's work, without lowering quality | **Not done**, by the user's decision. Section 3 gives hypotheses for that work, not demonstrated causes |

The [summary](hc-45-evidence.json) holds the measurements and hashes.

**Independent review at `0a635d9`:** the raw measurement subset is accepted with qualifications. The absolute
"no regression" conclusion and some causal/frame associations below are author conclusions not accepted by the
[review](hc-45-review-2026-10-17.md). HC-48/50 track those P2 evidence corrections; HC-49 is a nonblocking P3
query-gap diagnostic issue. HC-45 stays open, and HC-47 was not fixed by this handoff. No benchmark was repeated.

**Historical independent correction acceptance at `1e9433b`:** HC-48/49/50 are closed; the following earlier review
paragraph is historical. HC-47's new measurements are credible, but its helpers still accept missing observations
and invalid reattach/frame ordering. It stays open. See [recheck](hc-45-review-2026-10-17.md#6-hc-47-to-hc-50-correction-recheck-at-1e9433b).
HC-45 performance remains deferred; no scene or full suite was repeated.

**Latest HC-47 disposition at `d6bfb54`:** its v2 helper-check correction is independently accepted and closed;
HC-48/49/50 remain closed. Future v2 live-path validation is required before automated reuse, with no run requested
now. HC-45 performance remains open/deferred. See [acceptance](hc-45-review-2026-10-17.md#7-hc-47-version-2-correction-acceptance-at-d6bfb54).

## 1. The comparison

- **Viewers:**
  - **A (current):** the HC-11-tested Release Viewer (`flightsim-viewer.exe` `1ede4fcb9a153cf8`).
  - **B (before):** `7948dc4`, 2026-10-16 14:53, the parent of ARCH-01's first commit. It was exported with
    `git archive` to `D:/ENV/tmp/hc45/before-src` (not a worktree), configured with the `ucrt64-release` preset against
    the same dependency prefix (tests, examples, tools and Python off) and built in 54 s.
- **Matched inputs:**
  - the world-stream layout (9) and the recording format (7) are the same;
  - the dependencies, toolchain, aircraft and assets, shaders included, are unchanged since `7948dc4` (a README aside);
  - the two builds' `config/` directories are identical;
  - B's `maps` junction was pointed at the same tiles as A's.
- **Workload:** `viewer_perf.py`'s scenes, publishers, native hit recording and arguments are the current ones, so only
  `--exe` differs.
- **Order:** A, B, A, B, so drift in the host's load falls on both alike.
- **Note:** B's configure downloaded the project's sample model `Cesium_Air.glb` from the URL its CMake names, as A's
  first configure did.

The gate: after a 5 s warmup, every second at least 158.4 fps and no frame over 12.5 ms.

| Scene | A run 1 | B run 1 | A run 2 | B run 2 |
| --- | --- | --- | --- | --- |
| fleet | 159.2 fps, 17.66 ms, 2 over | 160.0, 17.47, 2 | 159.0, 17.92, 2 | 159.4, 17.71, 1 |
| strike | **144.5**, 14.31, 3 | **146.0**, 13.27, 5 | **145.7**, 12.93, 2 | **147.1**, 12.35, 0 |
| hit | 159.4, 19.10, 6 | **158.0**, 18.91, 8 | **158.0**, 19.10, 12 | **158.0**, 19.28, 5 |
| reheat | pass | pass | pass | pass |

Each cell gives the lowest second, the worst frame and the frames over 12.5 ms. Every run fails fleet, strike and hit,
and passes reheat: on this workload, those three failures occur before the migration.

Paired by round, A minus B, the lowest seconds differ by -0.8 and -0.4 fps at fleet, -1.5 and -1.4 fps at strike, and
+1.4 and 0.0 fps at hit. At strike, A's two lowest seconds are 144.5 and 145.7 fps and B's 146.0 and 147.1, a range of
1.2 and 1.1 fps within each Viewer. Two rounds, with no declared equivalence tolerance and no uncertainty analysis,
can neither dismiss that difference as noise nor show it to be a regression. **Neither a migration regression nor
performance equivalence is established** (HC-48).

## 2. Diagnostics added

`--stats` now also prints the slowest GPU frame of each second (`render::GpuTimer`). The line, `gpu worst | ...`,
gives:
- that frame's passes and their total;
- `gap before it`: from the previous frame's last timestamp to this frame's first. The interval is outside the marked
  passes: the GPU may have been idle, waiting on presentation, or busy with other work, and the timestamps do not say
  which;
- `largest gap`: the largest such interval in the second, in general before another frame;
- `unread`: the frames whose timestamps could not be read in the second.

A gap is measured only between two frames read one after the other; after a frame that was not read, the next gap is
`n/a`. The first version bridged an unread frame and counted its passes as part of a gap (HC-49, corrected after the
review, with five fast regression cases). The `diag1` run below used that first version, whose line ended in `gaps max`
and had no unread count; the review found no evidence that a skipped query affected it.

`viewer_perf.py` reads only `gpu |` lines, so its reports are unchanged.

- **First version:** the related Release selection passed 68/68 (121.5 s), `viewer_screenshot` among them.
- **HC-49 fix:** the related selection passed 74/74 in Release (121.86 s), the five new `[gpu-timer]` cases among them,
  and 6/6 headless.
- **Binaries:** four staged Release artifacts now differ from HC-11's, so they are no longer the HC-11-tested
  binaries: the Viewer (`dd875197b212f60c`), the Vision DLL in `bin` and `python/fsim/bin` (`855e87f78f4aa242`), and
  `_vision.pyd` (`412bd8343590f3e1`). The change is diagnostic output only.

## 3. What the slowest frames show (run `diag1`, idle host)

Each second's report selects four things separately: the GPU-slowest frame (the largest total of marked passes), the
gap just before that frame, the largest gap in the second, and the longest frame time on the Viewer's clock. They are
in general different frames, and the report has no identity joining them (HC-50).

| Scene, second | GPU-slowest frame: total (largest pass groups) | Gap before that frame | Largest gap in the second | Longest frame time in the second |
| --- | --- | --- | --- | --- |
| fleet, 25.5 s | 4.468 ms | 0.685 ms | 9.459 ms | 17.52 ms |
| hit, 11.0 s | 6.051 ms (clouds march 5.730) | 0.354 ms | 7.768 ms | 19.17 ms |
| hit, 25.1 s | 4.685 ms (pass end, scene copy, gas 2.525) | 5.887 ms | 6.558 ms | 12.73 ms |
| hit, 27.1 s | 4.793 ms (vehicles+exhaust 2.152) | 0.591 ms | 8.035 ms | 18.27 ms |
| hit, 29.1 s | 4.800 ms (pass end, scene copy, gas 2.701) | 0.406 ms | 8.124 ms | 19.26 ms |
| strike, 9.0 s | 7.704 ms (sky, earth, rest 3.108; vehicles+exhaust 1.753) | 0.487 ms | 3.006 ms | 8.83 ms |
| strike, 10.0 s | 8.348 ms (sky, earth, rest 3.230; vehicles+exhaust 2.415) | 0.408 ms | 1.250 ms | 8.90 ms |
| strike, 12.0 s | 7.695 ms (vehicles+exhaust 2.487; pass end, scene copy, gas 2.312) | 5.237 ms | 6.030 ms | 13.17 ms |
| strike, 14.0 s | 5.683 ms (vehicles+exhaust 2.592) | 0.471 ms | 7.931 ms | 17.17 ms |

What the timestamps show, and no more:
- **Frame budget:** among these seconds, the GPU-slowest frames' marked passes exceed the 6.25 ms budget only at
  strike 9.0, 10.0 and 12.0 s.
- **Long gaps:** gaps of 6.0-9.5 ms occur in fleet, hit and strike. Only at hit 25.1 s and strike 12.0 s is the gap
  just before the GPU-slowest frame long (5.9 and 5.2 ms).
- **What fills them:** a gap is unmarked time, so it may be idle time, presentation pacing or other work.
- **The pass groups:** these do not isolate one object. "sky, earth, rest", "vehicles+exhaust" and "pass end, scene
  copy, gas" each cover several draws, so they do not measure the fire or the breakup pieces alone.

Hypotheses for the dedicated assignment. None is a demonstrated cause; attributing one needs correlated measurements
or a controlled experiment:

| Observation | Hypothesis, not confirmed |
| --- | --- |
| fleet, 25.5 s, a slow second in every run of both Viewers: a 9.46 ms gap in it | Tile textures compiled and uploaded on the shared queue; or presentation pacing |
| hit, 11.0 s, a slow second in every run: one frame's clouds march 5.73 ms, against about 1 ms | A full cloud re-march, as after a camera change |
| hit, 25-29 s and strike, 12-14 s: largest gaps of 6.0-8.1 ms | NVIDIA Flow's solver, which runs on its own Vulkan device on the same GPU, taking the GPU between the Viewer's frames. No Flow timing was recorded |
| strike, 9-10 s: marked passes 7.7-8.35 ms, the largest groups "sky, earth, rest" and "vehicles+exhaust" | The fire volume's shading and the breakup pieces' draws within those groups |

Possible picture-preserving work, each following one of these hypotheses:
- **The fire's shading:** the sun's and the sky's transmittance per voxel, once per solver step, instead of eight
  extra lookups per sample.
- **The breakup pieces:** their draw cost at the impact.
- **The solver's contention:** a lower GPU priority or paced submissions, or the solver on the Viewer's own device as
  async compute, a larger change.
- **Tile uploads:** spread over frames.
- **The cloud march:** its full re-march after a camera change.

Before any of these, profile which hypothesis holds. None has been tried, and none is known to reach 160 fps.

## 4. Closure, still open

Meet the gate on an idle host in the current Viewer, through picture-preserving work. Check each change against
reference frames (`viewer_perf.py --shots` and `--compare`) and measure it again at 3840 x 2160 fullscreen. No threshold
is relaxed. If a scene cannot meet 160 fps without a quality trade-off, that is reported, not traded.

## 5. The review's corrections (2026-10-17)

The [review](hc-45-review-2026-10-17.md) accepted the raw measurements with qualifications and asked for three
corrections:
- **HC-48:** the comparison's conclusion is narrowed (section 1). The same three failures occur before the migration,
  the paired differences are disclosed, and neither a regression nor equivalence is established.
- **HC-49:** the GPU timer no longer measures a gap across a frame it did not read. It reports `n/a` and counts the
  unread frames (section 2).
- **HC-50:** the slow-frame table separates the four selections and labels the leads as hypotheses (section 3). The
  [Viewer guide](../../sdk/viewer.md) now says what a gap can and cannot show.

No benchmark or performance scene was rerun. The raw [summary](hc-45-evidence.json) is unchanged: its diagnostic
lines keep the first version's `gaps max` wording.
