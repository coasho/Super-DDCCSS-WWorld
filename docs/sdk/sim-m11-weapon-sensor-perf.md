# SIM-M11 (HC-45): Multi-Missile & On-Rail IR Reticle Runtime Acceleration

Date: 2026-11-27 (`main` base `15a47490`)

## 1. Motivation & HC-45 Carryover Scope

The [SIM-M09 4v4 real-time benchmark](sim-m09-encounters.md) demonstrated that
4v4 all-sensors runs at **46–47× real time** and 4v4 + 1 live `AIM-120C` runs at
**1.01× real time**, but two severe runtime hotspots remained as **HC-45**:
1. **Multi-missile serial bottleneck**: Two concurrent live `AIM-120C` missiles
   dropped 4v4 RTF to **0.47×** due to serial per-weapon `WeaponSensor`
   evaluation, expensive 512-patch ground-clutter `computePath` / P.528 /
   `RadioReceiverV1` signal processing, per-substep `VehicleState` copies,
   serial multi-missile JSBSim XML loads, and $O(N_{\text{weapons}} \times
   N_{\text{vehicles}})$ engagement sweeps.
2. **On-rail IR reticle bottleneck**: Carrying Ready on-rail IR missiles
   (`aim-9m`, `aim-9x`, `r-73`) dropped single-aircraft RTF to **0.29×** (1×
   `AIM-9M`) / **0.101×** (2× `AIM-9M`, `19.72 s` for `2.0 s` sim) because
   `sampleOnRailSeekers()` executed `ThermalReticleReceiverV1` /
   `ThermalImageReceiverV1` serially with unaccelerated full-frame background
   rasterization, repeated null-model horizon projections, and scalar
   pixel/template inner loops.

**SIM-M11** closes **HC-45** across the entire RF/radar, IR imaging/reticle,
flight-pool/JSBSim, and `World` engagement pipeline while preserving every
frozen public ABI, bit-exact `FrozenThermalImageReceiverV1` oracle, and
bit-identical cross-worker determinism (`workers=1 == workers=8`).

---

## 2. Engineering Optimizations Delivered

### 2.1 RF Propagation, NTIA P.528, Radar Signal & `RadioReceiverV1`
- `src/simulation/mission_systems/rf/{Antenna.cpp,NtiaP528.cpp,Path.cpp}`:
  - Added exact $O(1)$ constant-height `FlatGround` clearance and
    delta-Bullington fast path in `Path.cpp` (zero profile-sampling overhead
    when terrain is flat while preserving exact terrain-profile results when a
    heightfield is supplied).
  - Eliminated redundant trig/transcendental recomputation and inner-loop
    overhead in `NtiaP528.cpp` and `Antenna.cpp` (P.528 verified at
    `~3.9 µs/path` and within `0.016 dB` vs. exact reference over 300 random
    paths).
- `src/simulation/mission_systems/radar/Signal.cpp`,
  `src/simulation/mission_systems/components/implementations/seeker/{EngineeringSeekerV3.{h,cpp},RadioReceiverV1.cpp}`:
  - Hoisted invariant waveform/Doppler factors and tight inner clutter/CFAR
    loops inside `Signal.cpp` and `RadioReceiverV1.cpp` (512-patch
    `RadioReceiverV1::expose` down to **`2.20–2.26 ms`** total across
    `114,030` range-Doppler cells).

### 2.2 `ThermalImageReceiverV1` & `ThermalDualBandReceiverV1`
- `src/simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.{h,cpp}` & `ThermalDualBandReceiverV1.h`:
  - **Bit-exact oracle preservation**: `expose(...)` stays **0-ULP bit-exact**
    to `FrozenThermalImageReceiverV1` across all 4 IEEE-754 rounding modes
    (`FE_TONEAREST` + MXCSR guard on cached solid-angle `omega` table, 1-ULP
    `x0 + pitch == (x + 1 - halfW) * pitch` guard on hoisted `uniformBg` patch
    loop, and signed-zero/nearest guard on zero-patch skips).
  - **Fast background & SIMD median**: Added `exposeFastBackgroundV1(...)`
    backed by `ThermalReticleReceiverV1::fastBackgroundRadianceV1(...)`,
    runtime-dispatched AVX2 (`__attribute__((target("avx2")))` +
    `__builtin_cpu_supports("avx2")`) 4-pixel-wide 16+16 bitonic-split median
    for interior and border annuli, active-only `sourceContrast` attribution,
    and sorted `componentPixels` in `detectorScore`.
  - **Nested dual-band parallelism without ABI change**: Added free RAII
    `DualBandParallelGuardV1` (zero `sizeof` change;
    `sizeof(ThermalImageReceiverV1) == sizeof(FrozenThermalImageReceiverV1)`
    preserved) enabling join-before-return 2-channel parallel execution when
    spare `VehiclePool` workers exist.

### 2.3 `ThermalReticleReceiverV1` & `MovingRadiatorInverseV2`
- `src/simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.{h,cpp}` & `MovingRadiatorInverseV2.cpp`:
  - Shared immutable `PrecomputedV1` (projected/weighted/symmetric weights, row
    prefixes, x-moment waves, uniform/transient scalar dots,
    `templateInvNorm2`, lazy `supportRects`).
  - `gaussianIntervalFastV1` (`< 4e-12` cubic Hermite LUT) across `spot()`,
    `spotJacobianV1()`, `fit()`, and `tryFastDeltaPhase()`.
  - Runtime-dispatched AVX2+FMA (`__attribute__((target("avx2,fma")))`) kernels
    (`reticleFmadd1/2/3`, `reticleAddDiff`, `reticleFmaddDiff`, `reticleDot`,
    4-template `reticleDot4`, `reticleNormDot`, `reticleResidualNorm2`) with
    identical scalar fallback.
  - Zero-allocation `evaluateProjectedNullV1` (100% hit rate, row-major
    remaining-row contraction) + generation-stamped `thread_local`
    `nullTable[4096]` in `fitHorizonWithMeasuredPriorV2`, hoisted
    `FastDeltaScratch`, and single-patch direct row-major fast path in
    `tryFastDeltaPhase`.
  - Join-before-return `DirectReticleLauncherV1` + `ReticleNestedPoolGuardV1`
    with zero persistent thread-local/static destructors (Windows DLL
    Loader-Lock safe).

### 2.4 `VehiclePool`, `JsbsimModel`, `World` & `WorldEngagements`
- `src/simulation/flight/{VehiclePool.{h,cpp},JsbsimModel.{h,cpp},JsbsimLog.cpp}`:
  - Dynamic atomic work-stealing in `VehiclePool` (`reverseOrder = true` so
    heavy high-index missile slots start first).
  - Cached `generatedIntegrationSubsteps` by `(path, size, mtime)`; non-virtual
    `JsbsimModel::readNavigationStateV2(NavigationReadV2&)` (5 fields only, zero
    vtable/layout impact) consumed by `weaponNavigationObserverV2` in
    `WorldWeaponLinks.cpp`; airborne gear-loop skip (`deepestFt() > 10.0`) +
    lazy `tangSpd`; `checkDivergence(bool updateLastGood)` skipping log-only
    snapshot updates on intermediate substeps; `thread_local` JSBSim log bridge
    in `JsbsimLog.cpp` for parallel loader threads.
- `src/simulation/runtime/world/{World.{h,cpp},WorldEngagements.cpp,WorldWeaponLinks.cpp}`:
  - Parallel `JsbsimModel::load()` preload in `World::flyReleases()` when
    `>= 2` fresh missile loads occur in the same step (serial entity/ID/RNG/event
    commit order strictly preserved).
  - Parallel `prepareEngagements()` and `sampleOnRailSeekers()` over immutable
    scene snapshots with deterministic serial merge by weapon/station ID.
  - Exact $O(1)$ `FlatGround` `opticalClear()`, combined 3×3
    `projectedSeekerExtentWithCgV1()`, exact triangle-inequality broad-phase
    bounding-sphere reject in `finishEngagements()`, and join-before-return
    2-way range-band split for 512-patch active-radar ground clutter when spare
    workers exist (preserving exact `(band, az)` accumulation order).

---

## 3. Measured Performance & Cross-Worker Determinism

All timings below are clean builds (`-Werror`, zero profiling hooks,
`dt = 1/120 s`, `frameSkip = 4`, 30 Hz world step) via
`tests/simulation/test_sim_m11_weapon_perf.cpp` (CTest `sim_m11_weapon_perf`)
and `tests/integration/test_sim_m09_encounters.cpp` (`sim_m09_encounters`):

| Scenario | Pre-SIM-M11 Baseline | `ucrt64-release` (After) | `ucrt64-headless` (After) | Speedup / Determinism |
| --- | ---: | ---: | ---: | --- |
| `F-16C + 2× on-rail AIM-9M Ready` (`2.0 s`, 4w) | `19.72 s` (`0.101×` RTF; `0.29×` 1×) | **`0.1648 s` (`12.14×` RTF)** (`maxStep 9.03 ms`, `slow=0`) | **`0.1662 s` (`12.03×` RTF)** (`maxStep 8.75 ms`, `slow=0`) | **~120× faster** |
| `4v4 + 2× AIM-120C + on-rail AIM-9X/R-73` (`2.0 s` det) | `0.29×–0.47×` RTF | `w1: 0.8968 s (2.23×)` / **`w8: 0.3802 s (5.26×)`** | `w1: 0.9036 s (2.21×)` / **`w8: 0.3811 s (5.25×)`** | **`w1 == w8` bit-identical** |
| `4v4 + 2× AIM-120C + on-rail AIM-9X/R-73` (`10.0 s`, 8w) | `0.29×–0.47×` RTF | **`1.4732 s` (`6.79×` RTF)** | **`1.4855 s` (`6.73×` RTF)** | **14×–23× faster** (gate `>= 2.0×`) |
| `4v4 + 4 live missiles (2× AIM-120C + 2× AIM-9X) + 6 on-rail IR` (`2.0 s`, incl. 4 cold JSBSim XML loads) | `< 0.25×` RTF | `w1: 1.6818 s (1.19×)` / **`w8: 0.9491 s (2.11×)`** | `w1: 1.6816 s (1.19×)` / **`w8: 0.9315 s (2.15×)`** | **`w1 == w8` bit-identical**, `>= 2.0×` RTF |
| `SIM-M09 4v4 + 1 live AIM-120C + chaff` (`10.0 s`, 4w) | `9.927 s` (`1.007×` R) / `9.856 s` (`1.015×` H) | **`1.0735 s` (`9.32×` RTF)** | **`1.2065 s` (`8.29×` RTF)** | **8.2×–9.2× faster** |
| `SIM-M09 4v4 all-sensors` (`10.0 s`, 4w) | `0.211 s` (`47.39×` R) / `0.217 s` (`46.07×` H) | `0.2185 s` (`45.76×` RTF) | `0.2494 s` (`40.09×` RTF) | Unregressed (`40–46×` RTF) |
| Full CTest `sim_m09_encounters` (DLZ + 1v1 + 4v4) | `192.71 s` (R) / `191.72 s` (H) | **`33.13 s`** | **`33.23 s`** | **5.8× faster** |
| Full CTest `thermal_reticle_receiver` (56 cases / 325,635 assertions) | `263.97 s` | **`~118 s`** | **`118.32 s`** | **2.23× faster**, 0-ULP oracle |

### Micro-Profile Reference (`test_sim_m11_weapon_perf`, clean builds)
- `computePath(512 flat ground patches)`: `5.525 ms` (R) / `5.538 ms` (H)
- `RadioReceiverV1::expose(512 ground patches, 114,030 cells)`: `2.262 ms` (R) / `2.199 ms` (H)
- `ThermalReticleReceiverV1(aim-9m level horizon empty sky)`: `expose = 3.107 ms` (R) / `3.111 ms` (H)
- `ThermalReticleReceiverV1(aim-9m mid-FOV horizon)`: `expose = 55.675 ms` (R) / `55.499 ms` (H)
- `ThermalReticleReceiverV1(aim-9m pure sky)`: `expose = 0.264 ms` (R) / `0.256 ms` (H)

---

## 4. Dual-Preset Verification Summary

Both `ucrt64-release` and `ucrt64-headless` compiled cleanly with **0 compiler
warnings (`-Werror`)**:

| Suite | `ucrt64-release` | `ucrt64-headless` |
| --- | --- | --- |
| `sim_m11_weapon_perf` (2/2 cases, 154/154 assertions) | PASS (`22.92 s`) | PASS (`22.96 s`) |
| `thermal_reticle_receiver` (56/56 cases, 325,635/325,635 assertions, all 4 IEEE-754 rounding modes & bit-exact oracle) | PASS | PASS (`118.32 s`) |
| `weapon_thermal_image` (24/24 cases, 8,334/8,334 assertions) | PASS | PASS (`0.31 s`) |
| `weapon_radio_receiver` (15/15 cases, 4,459/4,459 assertions) | PASS | PASS (`0.15 s`) |
| `weapon_radio_temporal` (5/5 cases, 142/142 assertions) | PASS | PASS (`0.03 s`) |
| `representative_weapon_models` (17/17 cases, 800/800 assertions) | PASS (`2.50 s`) | PASS (`2.54 s`) |
| `representative_weapon_native` (V2 fixture) | PASS (`3.95 s`) | PASS (`3.89 s`) |
| `sim_m09_encounters` (3/3 cases, 512/512 assertions) | PASS (`33.13 s`) | PASS (`33.23 s`) |
| `aam_full_chain` representative 5-family (`aim-120c`, `aim-7m`, `aim-9x`, `aim-9m`, `r-73`) | PASS (`5/5`: `16.93 s`, `17.96 s`, `26.38 s`, `324.17 s`, `278.32 s`) | covered via `representative_weapon_native` + `sim_m11_weapon_perf` |
| `fsim_tests "[rf]"` (11/11 cases, 6,516/6,516 assertions; P.528 `3.90–3.91 µs/path`) | PASS | PASS |
| `combat_vfx` | PASS (`1.47 s`) | PASS (`1.45 s`) |
| `sdk_boundary` (0 errors) | PASS (`4.79 s`) | PASS (`2.29 s`) |
| `architecture_boundary` | 0 violations from SIM-M11 files (4 pre-existing unrelated files) | — |

### Preserved Invariants
- Zero changes to public C++/C99/Python ABIs, `WorldLayout v9`, `VehicleState`,
  `fsrec v8`, `SeekerSceneClockV1` vtable, `sizeof(ThermalImageReceiverV1)`, or
  `fsim::Agent` `truth == 0` boundaries.
- Zero persistent background thread pools destroyed under Windows DLL
  Loader-Lock (`DirectReticleLauncherV1`, `DualBandParallelGuardV1`, and
  `VehiclePool` parallel regions all strictly join before returning).
