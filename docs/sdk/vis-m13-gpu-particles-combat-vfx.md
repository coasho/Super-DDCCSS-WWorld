# VIS-M13 — Unified Compute GPU Particle Engine & Combat VFX

Date: 2026-11-26  
Branch: `main` (base `21ef76f0`, following delivered `VIS-M12`/`VIS-M12b` and `VIS-M11`/`VIS-M11b`)  
Status: **Delivered**

---

## 1. Overview

`VIS-M13` introduces a **unified, compute-driven Vulkan GPU particle and combat-ribbon renderer** (`GpuParticles`) backed by a **strictly SDK-driven additive combat VFX wire contract, wait-free IPC segment, and lazy companion `.vfx` replay sidecar**. Every visual projectile, countermeasure decoy, muzzle flash, impact spark, and supersonic blast wave originates in native `fsim::session::World` simulation authority — the Viewer duplicates zero ballistics, decoy, or damage physics, while preserving `WorldLayout v9`, `VehicleState`, `fsrec v8`, `DebrisLayoutV1`, `LegacyCompatible` defaults, and all public C++/C99/Python ABIs unchanged.

---

## 2. Architecture & Deliverables

### 2.1 Frozen Additive Wire Contract (`src/contracts/world_stream/CombatVfxLayoutV1.h`)
Trivially copyable, zero-implicit-padding, strict finite/bounds-validated wire structures:
- **`ProjectileWireV1` (`80 B`, align 8)**: live native ballistic gun rounds (`positionEcef[3]`, `velocityEcef[3]`, monotonic `id`, `shooterVehicleId`, `calibreM`, `ageS`, `maxAgeS`, `explosiveKg`, `flags` including `kProjectileFlagTracer` and `kProjectileFlagHe`).
- **`DecoyWireV1` (`80 B`, align 8)**: live native countermeasures (`positionEcef[3]`, `velocityEcef[3]`, monotonic `id`, `ownerVehicleId`, `ageS`, `lifeS`, `intensityOrSigma`, `bloomRadiusM`, `kind` = `Chaff` / `Flare`).
- **`CombatVfxEventWireV1` (`80 B`, align 8)**: discrete authoritative combat events deduplicated by monotonic `serial` (`MuzzleFlash`, `BulletAirframeHit`, `BulletSurfaceImpact`, `DetonationShockwave`, `CountermeasureEject`).
- **`CombatVfxSlotV1` / `CombatVfxHeaderV1`**: fixed capacities (`kMaxProjectilesV1 = 512`, `kMaxDecoysV1 = 256`, `kMaxCombatVfxEventsV1 = 128`, ~`71.6 KB`/slot) inside a triple-buffered seqlock shared-memory segment (`kCombatVfxMagicV1`, `kCombatVfxVersionV1 = 1`).

### 2.2 Wait-Free IPC & Companion Replay Sidecar (`src/foundation/transport/CombatVfxSegmentV1.{h,cpp}`)
- **`CombatVfxSegmentV1`**: triple-buffered wait-free shared memory (`fsim.vfx.v1.<world>`) wired into `WorldPublisher::setCombatVfx` and `WorldMirror::combatVfx()`.
- **`CombatVfxRecorderV1` & `CombatVfxReplayV1`**: lazy `<recordPath>.vfx` (`FSVFX1`) companion created only when combat projectiles, decoys, or VFX events occur. `atTime(simTimeS, out, interpolate)` linearly interpolates matching-ID live projectiles and decoys between bracketing frames and collects deduplicated discrete events in `[simTimeS - 0.12, simTimeS]` for glitch-free seeking and deterministic replay playback.

### 2.3 Native Simulation Authority (`src/simulation/runtime/world/World.{h,cpp}`, `WorldEngagements.cpp`, `WorldDamage.cpp`)
- Assigns deterministic monotonic IDs and 1-in-3 tracer flags (`kProjectileFlagTracer`) in `World::stepGunProjectiles`.
- Emits authoritative discrete events directly from native simulation transitions:
  - `MuzzleFlash` at exact gun barrel ECEF pose and firing axis when rounds spawn.
  - `BulletAirframeHit` on `impact.fragmentsStriking > 0` with true ECEF impact point, relative velocity, and kinetic/HE energy.
  - `BulletSurfaceImpact` on terrain/water strike with native `SurfaceKindV1` (`Water`, `Soil`, `Rock`).
  - `CountermeasureEject` in `World::dispense` for flare and chaff cartridges.
  - `DetonationShockwave` on warhead detonation / catastrophic fuel-ullage explosion (plus fallback Viewer hook on legacy `ipc::EngagementKind::Detonation` so older recordings without `.vfx` sidecars also render shockwaves).
- `World::collectCombatVfxFrame()` publishes live `gunProjectiles_`, live `Dispenser::decoys()` (flares with `6000 * exp(-age/2) W/sr`; chaff with `10 * (1 - exp(-age/0.35)) * exp(-age/5.0) m²` RCS and `0.6 + 5.5*(1 - exp(-age/0.65)) m` bloom radius), and bounded recent `combatVfxEvents_`.

### 2.4 Unified Vulkan Compute Particle & Combat Ribbon Engine (`src/visualization/vfx/effects/GpuParticles.{h,cpp}`, `assets/clouds/shaders/combat_*`)
- **Floating-Origin Precision**: CPU maintains `originEcef_` rebased only beyond `25 km` from camera, passing `originToEye` computed in double precision for `< 1.5 mm` world precision worldwide with zero per-frame particle rebasing.
- **Up-Front Device-Local Allocation & Zero-Cost Idle Gating**: `GpuParticles::build()` allocates device-local (`VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT`) SSBO + indirect draw buffers (`particles[65536]`, `aliveAlpha[32768]`, `aliveAdditive[32768]`, `indirect[2]`) directly on the Viewer `VkDevice`. Reset uses `vkCmdFillBuffer(..., 0xBF800000u)` (IEEE-754 `-1.0f`) and `vkCmdUpdateBuffer` for the 32-byte indirect draw counts, preserving the persistent ring-buffer write cursor. `vsg::Switch` nodes gate compute and draw passes to zero GPU cost when idle.
- **Compute Simulation (`betweenPass`, after `CopySceneDepth`)**:
  - `combat_particle_spawn.comp`: O(1) atomic ring-buffer ingestion (`<= 2,048` spawns/frame).
  - `combat_particle_sim.comp`: `256` workgroups updating `65,536` particles in parallel (gravity, exponential aero drag, helical flare curl, buoyancy) with 6-plane eye-relative reverse-Z frustum culling and dual-bucket indirect draw compaction (`aliveAlpha` + `aliveAdditive`).
- **Soft-Depth + Flow-Interleaved Rendering (`overScene`)**:
  - `combat_particle.{vert,frag}` (2× `vsg::DrawIndirect`: premultiplied alpha pass → additive HDR pass): Kind 0 seamless sub-stepped flare corkscrew white MgO smoke (`0.008 s` step, up to `32` sub-steps), Kind 1 molten amber-gold metallic ricochet sparks & burning debris embers (`2600 → 900 K` cooling), Kind 2 translucent silver-grey chaff cloud haze, Kind 3 muzzle gas & HEI hit puffs, Kind 4 soil/rock/water impact ejecta, Kind 5 detonation condensation mist, Kind 6 pinpoint tumbling metallic chaff glints.
  - `combat_ribbon.{vert,frag}` (direct exact-SDK-pose HDR primitives): Kind 0 velocity-aligned golden-amber gun tracer dashes (`7–12 m`) + subtle ball rounds, Kind 1 directional muzzle flash cones, Kind 2 blazing IR flare magnesium cores with 6-point optical diffraction spikes, Kind 3 translucent Wilson-condensation Mach shockwave Fresnel shell (`alpha <= 0.30` with organic angular limb breakup).
  - Full reverse-Z `sceneDepthTex` soft occlusion, `flowFrontTex` volumetric interleaving, and `GpuTimer` passes `"combat compute"` and `"combat vfx"` in `src/applications/viewer/main.cpp`.

---

## 3. Automated Verification (Dual-Preset, Zero `-Werror` Warnings)

| Suite / Check | `ucrt64-headless` | `ucrt64-release` | Details |
| --- | --- | --- | --- |
| `combat_vfx` (`tests/simulation/test_combat_vfx.cpp`) | **PASS** (`1.48 s`, **4/4 cases, 264/264 assertions**) | **PASS** (`1.49 s`, **4/4 cases, 264/264 assertions**) | Wire contract & validators, live IPC seqlock roundtrip, `.vfx` interpolation & event-window seek, native M61A1 gun burst + flare/chaff salvo determinism |
| `fsim_render_tests ~[gpu]` | n/a | **PASS** (**63/63 cases, 5,296/5,296 assertions**) | Complete offline render/visuals/manifest regression |
| `sdk_boundary` | **PASS** (`2.41 s`, 0 errors) | **PASS** (`4.81 s`, 0 errors) | Frozen public C++/C99/Python ABI & boundary enforcement |
| `mandatory_presentation` | n/a | **PASS** (`0.02 s`) | Viewer fullscreen/real-time/complete-scene contract |
| `destruction_debris` | — | **PASS** (`14.00 s`) | `VIS-M11`/`VIS-M11b` regression |
| `sim_m10_guns_rcs_ballistics` | — | **PASS** (`12.49 s`) | `SIM-M10` gun ballistics & terminal damage regression |
| `esm_ea_comint` | — | **PASS** (`28.49 s`) | `SIM-M05` countermeasure dispenser & EW regression |
| `include_view_ipc` / `include_view_render` | — | **PASS** (`0.95 s` / `2.10 s`) | Module header-isolation checks |
| `architecture_boundary` | — | **0 new violations** | Only the 4 pre-existing unrelated uncategorized files (`frozen_thermal_image_receiver_v1.{h,cpp}`, `{,test_}clean_hangar_data.py`) |

---

## 4. Native 4K Fullscreen Visual & GPU Telemetry Evidence

Captured on physical interactive desktop (`WinSta0\Default` via `desktop-gui` MCP), **NVIDIA GeForce RTX 5070 Ti**, **`3840×2160` fullscreen**, **1× real time**, **`--sun-utc 19` daylight**, complete scene layers enabled (Earth imagery, terrain, clouds, sunlight, aircraft shadows, exhaust, contrails, NVIDIA Flow volumetric fire/smoke, and JoltPhysics `.glb` debris).

### 4.1 Showcase 1 — M61A1 20 mm Gun Burst Tail-Chase (`gun_tail_chase.fsrec` + `.vfx` 225 f + `.debris` 260 f)
- **Visual inspection (`gun_v2_000..003.jpg`)**: Crisp golden-amber 20 mm tracer dashes stream from `viper-1` into `bandit-1`, producing tight molten-amber ricochet sparks, dark HEI puffs, defensive IR flare ejections, trailing smoke, and Flow fuel-bay fireballs (`HUD 151 fps`).
- **4K Fullscreen `--stats 5` GPU telemetry**:

| Sim Window | FPS | `p50` / `p99` ms | `combat compute` (avg) | `combat vfx` (avg) | Combined `VIS-M13` GPU | Total 4K GPU | Flow Leaves |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `t = 2.0 s` (active burst + hits + fire start) | **133.9 fps** | `7.37 / 10.70 ms` | `0.052 ms` | `0.077 ms` | **`0.129 ms`** | `6.566 ms` | `500` |
| `t = 3.0 s` (sustained tracers + sparks + fire) | **123.2 fps** | `8.09 / 11.04 ms` | `0.041 ms` | `0.129 ms` | **`0.170 ms`** | `7.096 ms` | `1,105` |
| `t = 4.0 s` (heavy fire + tracers + debris) | **120.2 fps** | `8.24 / 13.06 ms` | `0.036 ms` | `0.114 ms` | **`0.150 ms`** | `7.296 ms` | `1,588` |

### 4.2 Showcase 2 — Countermeasure Volley & AIM-120C Proximity-Fuze Kill (`cm_missile_kill.fsrec` + `.vfx` 320 f + `.debris` 340 f)
- **Visual inspection (`cm_v3_000..003.jpg`, `cm_v3_002_crop.jpg`)**: Blazing HDR golden-white IR flare cores with 6-point optical diffraction spikes, seamless helical sunlit white MgO corkscrew smoke trails, delicate shimmering silver chaff clouds, translucent Wilson-condensation Mach compression shockwave sphere (`alpha <= 0.30` with organic angular breakup), supersonic golden spark burst, massive NVIDIA Flow fireball, and tumbling severed F-16C `.glb` airframe wreckage (`HUD 160 fps`).
- **4K Fullscreen `--stats 6` GPU telemetry**:

| Sim Window | FPS | `p50` / `p99` ms | `combat compute` (avg) | `combat vfx` (avg) | Combined `VIS-M13` GPU | Total 4K GPU | Flow Leaves |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `t = 2.0 s` (flare curtains + chaff volley) | **160.3 fps** | `6.00 / 8.18 ms` | `0.020 ms` | `0.023 ms` | **`0.043 ms`** | `5.750 ms` | `0` |
| `t = 3.0 s` (full countermeasure curtain) | **~160 fps** | `5.93 / 9.02 ms` | `0.015 ms` | `0.024 ms` | **`0.039 ms`** | `5.844 ms` | `0` |
| `t = 4.0 s` (detonation Mach shockwave + fireball) | **138.6 fps** | `7.21 / 13.45 ms` | `0.029 ms` | `0.030 ms` | **`0.059 ms`** (worst `0.132 ms`) | `6.530 ms` | `1,381` |
| `t = 5.0–6.0 s` (tumbling burning wreckage + embers) | **136.8–139.6 fps** | `7.16–7.39 / 10.01–10.20 ms` | `0.020–0.021 ms` | `0.000–0.002 ms` | **`0.020–0.023 ms`** | `6.436–6.498 ms` | `2,430–2,828` |

Combined `VIS-M13` GPU cost stays within **`0.020–0.170 ms`** at native 4K (`3840×2160`), comfortably under half of the `< 0.35 ms` budget, maintaining **`120.2–160.3 fps`** across all active combat phases.
