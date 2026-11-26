# VIS-M12 — Volumetric Fire/Explosion & Contrail/Trail GPU Optimization

- **Date:** 2026-11-26
- **Branch / Base:** `main` (base `7b5b280`)
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

---

## 2. Measured 4K Fullscreen Benchmarks (RTX 5070 Ti, `3840×2160`)

### 2.1 Ground-Strike 1.27 t F-16C Fireball (`strike.fsrec`, `--follow 0 --chase-distance 160 --stats 22 --stats-budget 12.5`)

| Metric | Baseline (Before `VIS-M12`) | After `VIS-M12` | Delta |
| --- | --- | --- | --- |
| **Peak fireball FPS (`t=10–12 s`)** | `115.1–140.7 fps` (`avg 7.11–8.69 ms`) | **`156.7–159.9 fps`** (`avg 6.26–6.38 ms`) | **+44.8 fps at peak (+39%)** |
| **Lowest 1 s window FPS (`t=9–21 s`)** | `115.1 fps` (`t=10.4 s`) | **`151.5 fps`** (`t=16.1 s`) | **+36.4 fps floor** |
| **Peak volumetric fire GPU shading** | `~3.82 ms` avg / `~5.62–7.25 ms` worst (`sky, earth, rest` `2.09 -> 5.91 ms` avg, `9.34 ms` worst) | **`0.336–0.405 ms` avg** (`flow light 0.007–0.020` + `flow march 0.213–0.319` + `flow composite 0.073–0.119 ms`) | **~9.5× to 11.4× faster** |
| **Peak total 4K GPU frame time** | `7.639 ms` avg (`t=10.4 s`) | **`4.508–4.855 ms` avg** | **-2.78 to -3.13 ms (-37% to -41%)** |
| **Worst frame spike (`t=10–18 s`)** | `19.39 ms` (`p99 16.16 ms`, `6` over 12.5 ms at `t=12.4 s`) | **`10.45–15.75 ms`** (`p99 8.35–12.94 ms`, `0` over 12.5 ms in 9 of 11 active seconds) | **-3.6 to -8.9 ms worst spike** |
| **Post-burnout tail leaves (`t=20 s`)** | `1,014` leaves (`0.57 ms` solver GPU) | **`259` leaves** (`0.46 ms` solver GPU) | **-74% tail leaves** |

### 2.2 8× F-16C Formation Contrails (`contrails.fsrec`, 10,800 m, `humidity=0.72`, `--follow 1 --chase-distance 260 --stats 13`)

| Metric | Baseline (Before `VIS-M12`) | After `VIS-M12` | Delta |
| --- | --- | --- | --- |
| **Steady 4K FPS (`t=8–12 s`)** | `143.2–144.6 fps` (`avg 6.91–6.98 ms`) | **`144.5–149.1 fps`** (`avg 6.71–6.92 ms`) | **+1.3 to +4.5 fps** |
| **Total 4K GPU frame time** | `6.095–6.161 ms` | **`5.982–6.104 ms`** | **-0.06 to -0.14 ms** |
| **Isolated `contrails` GPU pass** | Collapsed in `sky, earth, rest` (`2.65–2.72 ms`) | **`1.180–1.233 ms`** (8 close-up F-16Cs) | Attributed & LOD-ready |
| **Frames over 12.5 ms (`t=2–12 s`)** | `0` | **`0`** (`p99 7.71–8.45 ms`) | Clean |

---

## 3. Physics Calibration, Visual Parity (`B-63`) & Automated Verification

1. **`flightsim-flow-calibrate check`:** **`0 of 12 cases outside their tolerance`** (12/12 PASS).
   - `spill 1000 kg`: diameter `50.03 m` vs. `58.00 m` (`0.86`), duration `4.07 s` vs. `4.50 s` (`0.90`), **`1.98 ms` GPU/step** (down from `2.28 ms` baseline, `-13%`).
   - All 4 TNT detonations (`1, 8, 30 kg` sea level + `8 kg` at 6 km): `0.89–1.03` diameter ratio, `0.97–1.11` duration ratio.
   - All 3 jet-flame leaks (`5–20 kg/s`): `0.94–1.13` length ratio.
2. **Statistical Visual Parity (`B-63`, `before_strike_10.6.png` vs. `after_strike_10.6.png` at `3840×2160`, replay `t=10.6 s`):**
   - Overall pixel MAE: **`1.33 / 255`** (`0.52%`), median `0.00`, p95 `3.00 / 255`.
   - Mean RGB: `[110.5, 119.0, 134.2]` vs. `[110.2, 118.9, 134.0]`.
   - Flame bounding box: `(1092, 0, 2711, 1185)` vs. `(1090, 0, 2707, 1185)` (within 4 px at 4K); HUD render readout `155 fps (4.7 ms)` vs. `115 fps (7.6 ms)`.
3. **CTest (`ucrt64-release`, `-Werror` zero warnings):**
   - `14/14` targeted tests passed: `simulation_interface_boundary`, `vision`, `vision_c_abi`, `include_view_flow`, `sdk_boundary`, all 5 `GpuTimer` tests, and all 4 `contrail` tests (`test_contrail_observed`, `CoCiP`, no-humidity, missile ice contrail).
   - `architecture_boundary` reported only 4 pre-existing unrelated files (`tests/reference/frozen_thermal_image_receiver_v1.{h,cpp}`, `tools/maintenance/{,test_}clean_hangar_data.py`), with **0 violations** on `VIS-M12` files.

---

## 4. Known Limits & `B-61` Architectural Note

- **Upstream NVIDIA Flow Device Ownership (`B-61`):** Inspection of `third_party/flow/include/NvFlowContext.h` confirms that `NvFlowDeviceInterface` always creates its own internal `VkDevice` inside `nvflowext.dll` and exposes Win32 external-memory export handles rather than a host `VkDevice` injection API. Consequently, single-device shared-queue interop cannot be enabled without modifying upstream `nvflowext.dll`. With `VIS-M12`, `B-61` impact is heavily mitigated (cutting Viewer volumetric shading by >10× and solver tail leaves by 74% keeps 4K peak fireball playback at `151.5–160.9 fps`), while occasional `~13–15 ms` solver-preemption frames at `4,096` leaves remain documented under `B-61`.
