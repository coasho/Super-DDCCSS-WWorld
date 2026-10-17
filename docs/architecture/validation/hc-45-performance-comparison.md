# HC-45 4K performance gate: idle-host measurement and before/after comparison

Status: evidence recorded (2026-10-17); awaiting independent acceptance. **HC-45 stays open.** By the user's decision,
no performance change was made: the causes found here are left for a dedicated performance assignment.

The [HC-07 review](hc-07-review-2026-10-17.md#2-hc-45-performance-failure-is-confirmed) asked for three things. Two
are done; the third is not:

| Requirement | Result |
| --- | --- |
| An idle-host measurement | Done: CPU 1-6 % and GPU 0-1 % before each run, against CPU 4-37 % and GPU 16 % during HC-07's runs |
| A controlled before/after comparison with matched workload, assets and settings | Done: **no regression**. The current Viewer and the Viewer of `7948dc4`, from before the architecture migration, fail the same three scenes, narrowly and alike |
| Meeting the target through the visualization plan's work, without lowering quality | **Not done**, by the user's decision. The causes are named in section 3 |

The [summary](hc-45-evidence.json) holds the measurements and hashes.

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
and passes reheat. B's strike is about 1.5 fps higher at its lowest second, which is within the spread of these runs.
Nothing points to the migration.

## 2. Diagnostics added

`--stats` now also prints the slowest GPU frame of each second (`render::GpuTimer`). The new line, `gpu worst | ...`,
gives:
- that frame's passes and their total;
- the GPU's gap before it: from the previous frame's last mark to its first, while the GPU was idle or busy with work
  outside the Viewer's marked passes;
- the largest such gap.

`viewer_perf.py` reads only `gpu |` lines, so its reports are unchanged.

The related Release selection passed 68/68 (121.5 s), `viewer_screenshot` among them. The rebuilt Viewer is
`2915dce58514c3f3`, so it is no longer the HC-11-tested binary; the change is diagnostic output only.

## 3. What the slowest frames show (run `diag1`, idle host)

| Scene, time | Slowest frames | Cause found |
| --- | --- | --- |
| fleet, 25.5 s (every run, both Viewers) | The Viewer's passes total 4.47 ms, but the GPU's largest gap is 9.46 ms | GPU work outside the Viewer's frame, most likely tile textures compiled and uploaded on the shared queue. Not yet confirmed |
| hit, 11.0 s (every run) | One frame's cloud march takes 5.73 ms, against about 1 ms | The clouds marched in full in one frame, as after a camera change. Not yet confirmed |
| hit, 25-29 s; strike, 12-14 s | Gaps of 5.2-8.1 ms before the slowest frames, with the passes at 4.7-7.7 ms | NVIDIA Flow's solver, on its own Vulkan device on the same GPU, stepping 60 times a second: its work takes the GPU between the Viewer's frames, and refreshes are missed |
| strike, 9-10 s | GPU-bound throughout: the slowest frames' passes total 7.7-8.35 ms, against a 6.25 ms budget | The fire volume's ray-march (in "sky, earth, rest") 3.1-3.2 ms; the vehicles, breakup pieces among them, 1.75-2.4 ms |

What could close the gap without lowering the picture, for the dedicated assignment:
- **The fire's shading:** the sun's and the sky's transmittance per voxel, once per solver step, instead of eight
  extra lookups per sample.
- **The breakup pieces:** their draw cost at the impact.
- **The solver's contention:** a lower GPU priority or paced submissions, or the solver on the Viewer's own device as
  async compute, a larger change.
- **Tile uploads:** spread over frames.
- **The cloud march:** its full re-march after a camera change.

None has been tried, and none is known to reach 160 fps. The hit's aftermath in particular puts the solver's steps
and the Viewer's frames on one GPU.

## 4. Closure, still open

Meet the gate on an idle host in the current Viewer, through picture-preserving work. Check each change against
reference frames (`viewer_perf.py --shots` and `--compare`) and measure it again at 3840 x 2160 fullscreen. No threshold
is relaxed. If a scene cannot meet 160 fps without a quality trade-off, that is reported, not traded.
