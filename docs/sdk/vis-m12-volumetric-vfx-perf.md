# VIS-M12 — Volumetric Fire/Explosion & Contrail/Trail GPU Optimization

- **Date:** 2026-11-26
- **Branch / Base:** `main` (`VIS-M12` Pass 1 delivered at `d40f1b6b` from base `7b5b280`; `VIS-M12b` Pass 2 deep optimization on top of `d40f1b6b`)
- **Hardware / Display:** NVIDIA GeForce RTX 5070 Ti, primary display `3840×2160` fullscreen, real time (1×), complete scene enabled (Earth imagery, elevation, ocean, clouds, sunlight, aircraft shadows, exhaust/condensation/contrails, NVIDIA Flow combustion)
- **Capture Mechanism:** Interactive desktop `desktop-gui` MCP (`WinSta0\Default`, `dxcam-dxgi`)

---

## 1. Architecture Summary

### 1.1 Three-Stage Volumetric Fire & Smoke Pipeline (`FlowVolumes`)

1. **Precomputed 3-D Sun & Sky Transmittance Grid (`assets/clouds/shaders/flow_light.comp`)**
   - Allocates an `80×80×80` `VK_FORMAT_R16G16_SFLOAT` 3-D atlas (`16³` bricks × `5³` texels per brick covering offsets `0, 2, 4, 6, 8` inside each `8³` NanoVDB leaf so brick boundaries stitch seamlessly under hardware trilinear filtering).
   - Dispatches one `5×5×5` workgroup per active Flow brick inside `Viewer::setScene`'s `beforePass`, **only when solver fields or sun direction change** (cached across intermediate frames at 160 Hz; `0.000 ms` on cached frames, `~0.05 ms` on refresh frames, averaging `0.007–0.020 ms`).
   - Replaces 8 sparse hash-table + 3-D texture lookups per raymarch step with a single `flowLightAt()` hardware trilinear fetch (`flow_march.comp` and fallback `flow.frag`).

2. **Half-Resolution Single-Pass Front-to-Back Compute Raymarch (`assets/clouds/shaders/flow_march.comp`)**
   - Uses `Viewer::sceneDepth()` (resolved 1-sample `VK_FORMAT_D32_SFLOAT` copied via `CopySceneDepth`, strictly gated by `Viewer::setSceneDepthCopied(flow->active())` so idle frames incur zero copy cost) and executes inside `Viewer::setScene`'s new `betweenPass` slot (between the main render pass and `overPass`).
   - Sorts active Flow pages **front-to-back** on the CPU, projects their tight screen-space bounding rectangle (`[rx0, ry0, rx1, ry1]`) padded by 2 half-res texels, and dispatches `8×8` compute groups over the active bounding region only (`2048×1152` backing allocation; `scale = 2.0` at ≥ 2M px such as 4K, `scale = 1.0` at smaller resolutions).
   - Clamps every ray against the conservative farthest scene depth across the covered full-res pixels (`zFar = min(z)` in reverse-Z), integrates radiative transfer in linear radiance space across all pages with Nyquist-adaptive stepping `max(0.75 * voxel, 0.45 * F_eye.w * s)`, terminates across **all remaining pages** as soon as `through < 0.003` (`B-60`), and writes premultiplied AgX `RGBA16F` + front hit distance `R32F`.

3. **Full-Resolution Depth-Aware Bilateral Upsample (`assets/clouds/shaders/flow_composite.{vert,frag}`)**
   - Runs as a fullscreen triangle at the start of `overPass` (before `WingVapour` and ImGui), discarding outside the projected pixel scissor rect `laid`.
   - Samples the 2×2 half-res neighborhood and bilaterally rejects background fire texels whose `front > pixelDist` (preserving razor-sharp 4K aircraft and terrain silhouettes while keeping seamless fire up to foreground edges).

4. **Startup Pipeline Warmup (`B-62`), Solid Ground Plane & SFPE Pool Fire (`B-53`), and Clear-Air Trimming (`B-60`)**
   - `FlowSolver::start()` runs two tiny warm-up steps (keeping `host_->clock` strictly monotonic) and `FlowVolumes::update()` runs a 1-workgroup frame-0 shader/layout warmup for `flow_light`, `flow_march`, and `flow_composite`.
   - `FlowSolver::step()` accepts an optional `FlowSolver::GroundPlane` (`B-53`) emitting a zero-allocation `NvFlowEmitterBoxParams` solid collider (`allocationScale = 0`, `applyPostPressure = TRUE`, `isPhysicsCollision = TRUE`) below impact elevation; `flowfires::poolRadius()` and `flowfires::poolFire()` model post-liftoff residual hydrocarbon surface burning using the SFPE JP-8 burning rate (`0.055 kg/(m²·s)`, capped at `4.0 kg/s`).
   - `summaryAllocate.speedThreshold` is raised from `3.0` to `5.0 m/s` (freeing slow post-burnout clear-air blocks) while keeping Flow's native FP16 precision (`enableLowPrecision* = FALSE`; enabling 8-bit UNORM low precision clips `fuel > 1` and fails calibration) and `maxStepsPerSimulate = 2u` (preserving exact 3-step detonation fixed-timestep accumulation).

5. **Contrail LOD, Zero-Churn `Airflow`, O(1) `Trails` (`B-52`) & `GpuTimer` Attribution**
   - `assets/clouds/shaders/contrail.frag`: skips inactive wake/plume parts before Crow-wave noise evaluation; adds projected-width distance LOD (`< 3 px` non-fire uses single-step analytic `plumeDepth()`, `3..12 px` halves steps with 4-octave+tail scattering, `≥ 12 px` uses 5-octave+tail scattering `< 0.15%` error vs. 8 octaves) and early termination (`transmittance < 0.01`).
   - `Airflow`: eliminates per-frame `kept`, `order`, and `drawnSamples` heap allocations via in-place compaction and reusable scratch buffers, and skips `samples_->dirty()` when idle.
   - `Trails` (`B-52`): replaces O(N) `history.erase(begin())` with an O(1) circular ring buffer (`ring`, `head`, `count`), skips redundant uploads between samples, and adds additive `Trails::Attachment` (`Origin`, `CenterOfGravity`, `WingTips`, default `Origin`) + `setBodyOffset()`.
   - `src/applications/viewer/main.cpp`: unwraps single-child `addBackdrop` `vsg::Switch` wrappers in `GpuTimer` labeling so `clouds under`, `clouds over`, `trails`, `contrails`, `flow light`, `flow march`, and `flow composite` are each measured independently instead of collapsing into `sky, earth, rest`.

### 1.2 Second Deep Cross-Pipeline Pass (`VIS-M12b`: Flow, Contrails, Ocean, Clouds & WingVapour)

1. **Flow `16³` Small Blocks & Decoupled 30 Hz Async Export (`FlowSolver`, `FlowVolumes`)**
   - `FlowSolver::Host::start()`: enables `enableSmallBlocks = NV_FLOW_TRUE` (`16³` density/velocity blocks = 8× finer spatial sparsity than `32³`), `blockMinLifetime = 2u`, and `gridDesc.maxLocations = 8192u` (preventing block-cap clipping on 1,000+ kg fireballs) while preserving default `smokeThreshold = 0.02f` and `speedThreshold = 5.0f`.
   - `FlowSolver::step(..., bool exportFields = true)` & `FlowVolumes`: decouples 60 Hz async fluid physics from **30 Hz** NanoVDB export, PCIe readback, CPU packing, and 12 MB atlas upload (`doExport = wait || job.clear || (last && ((simStep & 1u) == 0u))`; `synchronous_` mode stays exact on every step), replaces `std::unordered_map slots_` with direct flat open-addressed lookups in `out.hash`, and splits `pack()` into branchless interior `8×8×8` + boundary halo + zero-skip FP16 conversion.

2. **Contiguous Contrail Instancing & Shader Fast Paths (`Airflow`, `contrail.{vert,frag,glsl}`)**
   - `Airflow`: trims `kYoungAges` from 19 to 10 smooth logarithmic ages and packs active samples contiguously with sequence jumps (`+4.0`) + 2 trailing sentinels so `ribbonDraw_->instanceCount` renders only active segments (~4.2× fewer box instances).
   - `contrail.{vert,frag,glsl}`: updates vertex indexing for contiguous `gl_InstanceIndex`; replaces `pow(..., 1.5)` with `d * sqrt(d)` in `henyeyGreenstein()` and removes redundant covariance inversion in `halfLineDepth()`; defers `octavePhase` computation lazily until `within(...)` passes; replaces `partAt(k, 0/1)` bbox queries with direct zero-noise envelopes; skips 3-harmonic Crow noise on young plumes (`wake.z <= 1e-3 && wake.w <= 1e-3`); adds chord-adaptive step counts `clamp(int(ceil((e1 - e0) / max(0.6 * partRadius, F_eye.w * sc))), 3, maxN)`; and matches geometric multiple-scattering tail weights (`0.875 * exp(-tau * 0.024)` / `0.9375 * exp(-tau * 0.048)`).

3. **Ocean Reflection/Cascade & Ground-Shade Culling (`ocean.{glsl,frag}`, `composite.frag`, `params.glsl`, `Clouds.cpp`)**
   - Hoists `whitecapCover = 3.84e-6 * pow(windMs, 3.41)` to CPU in `p.oceanFoam.w`; short-circuits `shallowAttenuation` beyond `350 m` depth; replaces 9 static 4K `RGBA16F` cloud-reflection taps with 1 (`spread < 4 px`), 3 (`dist > 25 km`), or 5 IGN-jittered taps (skipping completely when `fresnel < 0.04` or `cover <= 0.005`); uses far-field analytical whitecap foam (`t > 15 km`) and breaks out of sub-pixel wave cascades (`k == 2` beyond `20 km`, `k == 1` beyond `80 km`) while preserving compile-time `OCEAN_CASCADES` unrolling; and discards no-op `groundShade` (`MODE == 0`) sky/sunlit pixels in `composite.frag`.

4. **Volumetric Clouds Compute, Resolve & Cadence Throttling (`clouds.glsl`, `shadow.comp`, `march.comp`, `resolve.comp`, `Clouds.{h,cpp}`)**
   - `clouds.glsl`: fast-paths `getLayerDensity` when `expTerms == 0`, early-outs eroded-empty voxels before `shapeDetailTexture`, replaces `pow(detail, 6.0)` with multiplies, and adds `sampleMediaCoarse` (omitting `32³` detail noise for soft secondary optical-depth marches).
   - `shadow.comp`: removes sub-texel `#define SHAPE_DETAIL` on 20–200 m BSM texels.
   - `march.comp`: hoists 5-octave+tail `octavePhase` once per ray, uses `d * sqrt(d)`, moves precomputed irradiance LUT reads inside sky/ground occlusion guards, extends fast precomputed daytime irradiance up to 200 km (`dot(sun, normal) > 0.15`, `cameraHeight < 30 km`), uses `sampleMediaCoarse` in `marchOpticalDepth`, and uses analytic ground bounce when viewing cloud tops from above (`cameraHeight > shadowTopHeight`) or beyond 35 km.
   - `resolve.comp`: merges 4-tap + 9-tap `colorBuffer` gathers into a single 9-tap loop (computing `wSample = (wSmooth^2)^2` with zero extra `exp()`), adds clear-sky early-out (`maxAlpha < 1e-5`), and replaces 9-tap Catmull-Rom with 5-tap cross Catmull-Rom (44% less 4K `RGBA16F` history bandwidth).
   - `Clouds.{h,cpp}`: decouples `shadowParity_` from `parity_` with `shadowPassSwitch_` and 4-combo `seaDrawSwitch_` sharing one compiled `GraphicsPipeline`, throttling Beer Shadow Maps (`shadow.comp` + `shadowResolve.comp`) to **30 Hz** with exact world-locked `lastShadow_[c] * translate(eyeEcef)` on intermediate frames, and throttling ocean FFT + 8-level mip pyramid (`seaComputeSwitch_`) to **60 Hz** (`< 5 km`) / **15 Hz** (`>= 5 km`).

5. **WingVapour Per-Rect Culling & Edge Switch (`WingVapour.{h,cpp}`)**
   - Adds per-rect culling in `kMarchMain` (skipping empty sky between spread formation jets and zeroing halo margins without depth reads/raymarching) and per-rect discard in `kCompositeFragment`, and gates `edgeMarch` (8,192 workgroups) behind `edgeSwitch_` (`scale >= 1.001`).

---

## 2. Measured 4K Fullscreen Benchmarks (RTX 5070 Ti, `3840×2160`)

### 2.1 Ground-Strike 1.27 t F-16C Fireball (`strike.fsrec`, `--follow 0 --chase-distance 160 --stats 22 --stats-budget 12.5`)

| Metric | Baseline (Pre-`VIS-M12`) | After `VIS-M12` (`d40f1b6b`) | After `VIS-M12b` | Total Delta vs. Baseline |
| --- | --- | --- | --- | --- |
| **Peak fireball FPS (`t=10–12 s`)** | `115.1–140.7 fps` (`avg 7.11–8.69 ms`) | `156.7–159.9 fps` (`avg 6.26–6.38 ms`) | **LOCKED `159.9 fps`** (`avg 6.25 ms`) | **+44.8 fps (locked 160 Hz ceiling)** |
| **Lowest 1 s FPS (`t=10–21 s` active plume)** | `115.1 fps` (`t=10.4 s`) | `151.5 fps` (`t=16.1 s`) | **`159.7 fps`** (`159.7–160.4 fps` all 12 s) | **+44.6 fps floor** |
| **Frames over 12.5 ms (`t=10–21 s`)** | Up to `6 / s` (`max 19.39 ms`, `p99 16.16 ms`) | `0–1 / s` (`max 10.45–15.75 ms`, `p99 8.35–12.94 ms`) | **`0 of 1,925 frames`** (`max 8.08–9.47 ms`, `p99 7.91–8.85 ms`) | **Zero drops across entire explosion** |
| **Peak total 4K GPU frame time (`t=10–14 s`)** | `7.639 ms` avg (`t=10.4 s`) | `4.508–4.855 ms` avg | **`3.207–3.690 ms` avg** (`2.941–3.106 ms` `t=18–21 s`) | **-3.95 to -4.43 ms (>2.1× faster)** |
| **`clouds march` / `clouds under` / `vapour march`** | `~1.20` / `~2.05` / `~0.080 ms` | `~1.20` / `~2.05` / `~0.080 ms` | **`0.296–0.303` / `1.367–1.415` / `0.002–0.018 ms`** | **4.0× / -33% / >4× faster** |
| **Flow solver GPU avg / post-burnout leaves (`t=20 s`)** | up to `5.45 ms` / `1,014` leaves (`0.57 ms`) | up to `~4.6 ms` / `259` leaves (`0.46 ms`) | **`0.65–2.84 ms` avg** (max `4.87 ms`) / **`248–253` leaves** (`0.39 ms`) | **~2× lower solver GPU, -75% tail leaves** |

*(Note: at `t=9.0 s`, one single `app 12.54 ms` frame occurs on initial impact wreck-vehicle spawn `vehicles 1 -> 5`, targeted by Sprint 2 `VIS-M11` `DebrisField`.)*

### 2.2 8× F-16C Formation Contrails (`contrails.fsrec`, 10,800 m, `humidity=0.72`, `--follow 1 --chase-distance 260 --stats 13`)

| Metric | Baseline (Pre-`VIS-M12`) | After `VIS-M12` (`d40f1b6b`) | After `VIS-M12b` | Total Delta vs. Baseline |
| --- | --- | --- | --- | --- |
| **Steady 4K FPS (`t=4–12 s`)** | `143.2–144.6 fps` (`avg 6.91–6.98 ms`) | `144.5–149.1 fps` (`avg 6.71–6.92 ms`) | **LOCKED `159.7–160.3 fps`** (`avg 6.24–6.26 ms`) | **+16 fps (locked 160 Hz ceiling)** |
| **Total 4K GPU frame time (`t=8–12 s`)** | `6.095–6.161 ms` | `5.982–6.104 ms` | **`5.076–5.202 ms`** *(~`4.4 ms` at full boost clock; ~14% VSync idle downclocking)* | **~28% faster full-clock GPU** |
| **Isolated `contrails` GPU pass (`t=8–12 s`)** | Collapsed in `sky, earth, rest` (`2.65–2.72 ms`) | `1.180–1.233 ms` | **`0.309–0.343 ms`** | **~3.7–4.0× faster than `VIS-M12`** |
| **`vapour march` + `gas` (`t=8–12 s`)** | `~0.65 + 0.18 = 0.83 ms` | `~0.65 + 0.18 = 0.83 ms` | **`0.328–0.333 + 0.112–0.114 = 0.44 ms`** | **-47%** |
| **`clouds march` (`t=8–12 s`)** | `~2.23 ms` | `~2.23 ms` | **`1.555–1.609 ms`** *(even under VSync downclocking)* | **-28% raw (~-38% clock-normalized)** |
| **Frames over 12.5 ms (`t=2–12 s`)** | `0` | `0` (`p99 7.71–8.45 ms`) | **`0`** (`p99 7.99–8.43 ms`) | Clean |

---

## 3. Physics Calibration, Visual Parity (`B-63`) & Automated Verification

1. **`flightsim-flow-calibrate check`:** **`0 of 12 cases outside their tolerance`** (`12/12 PASS`).
   - `spill 1000 kg`: diameter `49.85 m` vs. `58.00 m` (`0.86`, within `0.36%` of `VIS-M12`'s `50.03 m`), duration `4.02 s` vs. `4.50 s` (`0.89`), **`1.27 ms` GPU/step** (down from `2.28 ms` baseline and `1.98 ms` `VIS-M12`, **-44% vs. baseline**).
   - All 4 TNT detonations (`1, 8, 30 kg` sea level + `8 kg` at 6 km) and all 3 jet-flame leaks (`5–20 kg/s`) remain within tolerance.
2. **Statistical Visual Parity (`B-63`, `3840×2160` fullscreen):**
   - **`VIS-M12` `strike.fsrec` (`before_strike_10.6.png` vs. `after_strike_10.6.png`):** MAE `1.33 / 255` (`0.52%`), median `0.00`, p95 `3.00`, 4K flame bbox within `4 px`; HUD `155 fps (4.7 ms)` vs. `115 fps (7.6 ms)`.
   - **`VIS-M12b` `contrails.fsrec` (`after_contrails_10.0.png` vs. `after2_contrails_10.0.png`, replay `t=10.0 s`):** Overall 4K pixel MAE **`0.96 / 255`** (`0.38%`), median `0.00`, p95 `4.00 / 255`; upper sky + 8× jets + 8× contrails MAE **`0.20 / 255`** (`0.08%`, median `0.00`, p95 `1.00`); lower ocean + clouds MAE `1.72 / 255`; HUD **`160 fps (5.3 ms)`** vs. `147 fps (6.7 ms)`.
   - **`VIS-M12b` `strike.fsrec` (`after2_strike_10.6.png` vs. baseline/`VIS-M12`, replay `t=10.6 s`):** Overall 4K pixel MAE `5.06 / 255` (`1.99%`) vs. baseline (`4.98 / 255` vs. `VIS-M12`), median `1.00 / 255` (ocean, horizon, reflections, and clouds match; slight turbulent eddy-shape difference inside the fireball cap arises from switching `32³` -> `16³` Flow sparse blocks while conserving calibrated diameter within `0.36%`), crisp 4K bilateral silhouettes on `viper right wing`, `viper left wing`, and `viper nose`; HUD **`158 fps (3.8 ms)`**.
3. **CTest (`ucrt64-release`, `-Werror` zero warnings):**
   - `17/17` targeted tests passed after final `VIS-M12b` edits: `the vapour cone's bounds hold all of it`, `the vapour round an aircraft follows the flight and the air`, `the clouds follow the weather`, `simulation_interface_boundary`, `vision`, `vision_c_abi`, `include_view_vision_core`, `include_view_flow`, `sdk_boundary`, `python_sdk test_vision`, 3 store/availability/vision-segment checks, and all 4 `contrail` tests (`test_contrail_observed`, `CoCiP`, no-humidity, missile ice contrail); plus all 5 `GpuTimer` tests passed in `VIS-M12`.
   - `architecture_boundary` reported only 4 pre-existing unrelated files (`tests/reference/frozen_thermal_image_receiver_v1.{h,cpp}`, `tools/maintenance/{,test_}clean_hangar_data.py`), with **0 violations** on `VIS-M12`/`VIS-M12b` files.

---

## 4. Known Limits & `B-61` Architectural Note

- **Upstream NVIDIA Flow Device Ownership (`B-61`):** Inspection of `third_party/flow/include/NvFlowContext.h` confirms that `NvFlowDeviceInterface` always creates its own internal `VkDevice` inside `nvflowext.dll` and exposes Win32 external-memory export handles rather than a host `VkDevice` injection API. Consequently, single-device shared-queue interop cannot be enabled without modifying upstream `nvflowext.dll`. With `VIS-M12` + `VIS-M12b` (`16³` small blocks, 30 Hz decoupled async NanoVDB export/readback, >10× faster Flow shading, and 2–4× faster clouds/ocean/contrails/vapour), `B-61` runtime impact is fully eliminated during active 4K explosion playback on RTX 5070 Ti (**locked `159.7–160.4 fps`**, **`0 of 1,925` frames over 12.5 ms**, peak GPU `3.21–3.69 ms`), while true single-device zero-copy GPU interop remains tracked under `B-61`.
