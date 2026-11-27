# VIS-M13 — Unified Compute GPU Particle Engine & Combat VFX Plan

Date: 2026-11-26
Branch: `main` (base `21ef76f0`, following delivered `VIS-M12`/`VIS-M12b` and `VIS-M11`/`VIS-M11b`)
Status: **Delivered (2026-11-26)** — see [VIS-M13 Evidence](../../sdk/vis-m13-gpu-particles-combat-vfx.md) (`combat_vfx` 4/4 cases / 264/264 assertions dual-preset, `fsim_render_tests` 63/63 / 5,296/5,296, 4K fullscreen `120.2–160.3 fps`, combined `combat compute + combat vfx` `0.020–0.170 ms` GPU)

---

## 1. Objective

Deliver a **unified, compute-driven Vulkan GPU particle & ribbon engine** (`GpuParticles`, 65,536-particle GPU-resident ring buffer + frustum-culled indirect draw) and a **strictly SDK-driven additive combat VFX transport** (`CombatVfxLayoutV1` / `CombatVfxSegmentV1` + companion `.vfx` replay sidecar) so every Phase 1 combat subsystem (`SIM-M05` flares/chaff, `SIM-M10` 20/30 mm aircraft guns, proximity-fuze detonations, and `VIS-M11` burning debris) renders at **cinema-grade 4K quality** (`< 0.35 ms` GPU pass time, target **120–160 FPS** at `3840×2160` fullscreen) with **zero simulation duplicated in the Viewer** and **zero breaks** to `WorldLayout v9`, `VehicleState`, `fsrec v8`, `DebrisLayoutV1`, or public C++/C99/Python ABIs.

---

## 2. Architecture & Strict Layering

### Layer A — Additive Frozen Wire Contract (`src/contracts/world_stream/CombatVfxLayoutV1.h`)
Zero implicit padding, trivially copyable, strict finite validators:
- **`ProjectileWireV1` (`80 B`, align 8)**: live native ballistic gun projectiles (`positionEcef[3]`, `velocityEcef[3]`, `id`, `shooterVehicleId`, `calibreM`, `ageS`, `maxAgeS`, `explosiveKg`, `flags` = tracer/HE).
- **`DecoyWireV1` (`80 B`, align 8)**: live native countermeasure decoys (`positionEcef[3]`, `velocityEcef[3]`, `id`, `ownerVehicleId`, `ageS`, `lifeS`, `intensityOrSigma`, `bloomRadiusM`, `kind` = `Chaff` / `Flare`).
- **`CombatVfxEventWireV1` (`80 B`, align 8)**: discrete authoritative combat events deduplicated by monotonic `serial` (`MuzzleFlash`, `BulletAirframeHit`, `BulletSurfaceImpact`, `DetonationShockwave`, `CountermeasureEject`).
- Capacities: `kMaxProjectilesV1 = 512`, `kMaxDecoysV1 = 256`, `kMaxCombatVfxEventsV1 = 128` (~`71.6 KB`/slot, wait-free triple-buffered seqlock).

### Layer B — Transport & Companion Replay Sidecar (`src/foundation/transport/CombatVfxSegmentV1.{h,cpp}`)
- **`CombatVfxSegmentV1`**: wait-free triple-buffered shared-memory segment (`fsim.vfx.v1.<world>`) wired into `WorldPublisher` (`setCombatVfx`) and `WorldMirror` (`combatVfx()`).
- **`CombatVfxRecorderV1` / `CombatVfxReplayV1`**: lazy companion `<recordPath>.vfx` (`FSVFX1` header) opened only when combat projectiles, decoys, or VFX events occur; `atTime(simTimeS, out, interpolate)` smoothly interpolates matching-ID projectiles & decoys across bracketing frames and gathers discrete events in `[simTimeS - 0.12, simTimeS]` for glitch-free seeking and deterministic replay captures.

### Layer C — Native World Authority (`src/simulation/runtime/world/World.{h,cpp}`, `WorldEngagements.cpp`)
- Assign deterministic monotonic IDs & 1-in-3 tracer flags to `GunProjectile` rounds in `World::stepGunProjectiles`.
- Record discrete `CombatVfxEventWireV1` directly inside native simulation hooks:
  - `MuzzleFlash` on burst spawn at exact muzzle ECEF & firing axis.
  - `BulletAirframeHit` on `impact.fragmentsStriking > 0` at exact impact ECEF, velocity, kinetic/HE energy.
  - `BulletSurfaceImpact` on `altM <= groundM` with terrain/water surface classification (`Soil`/`Water`/`Rock`).
  - `CountermeasureEject` in `World::dispense` for flare & chaff launches.
- Collect live `gunProjectiles_`, all live `Dispenser::decoys()` (flares with `6000 * exp(-age/2)` W/sr; chaff with `10 * (1 - exp(-age/0.35)) * exp(-age/5.0)` m² RCS & physical `0.6 + 5.5*(1 - exp(-age/0.65))` m bloom radius), and bounded recent `combatVfxEvents_` in `World::collectCombatVfxFrame()` for both live `WorldPublisher` and `.vfx` recording.

### Layer D — Unified Compute GPU Particle & Combat Ribbon Engine (`src/visualization/vfx/effects/GpuParticles.{h,cpp}` + GLSL Shaders)
- **Floating-Origin Double Precision**: CPU maintains `originEcef_` rebased only beyond 25 km from camera, giving `< 1.5 mm` float precision worldwide with zero per-frame particle rebasing churn.
- **Vulkan Compute Simulation (`betweenPass`, after `CopySceneDepth`)**:
  - `PrepareGpuParticleBuffers` allocates GPU-local `VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT` SSBOs (`particles[65536]`, `aliveAlpha[32768]`, `aliveAdditive[32768]`, `indirect[2]` with `STORAGE_BUFFER | INDIRECT_BUFFER | TRANSFER_DST`) and resets the two `VkDrawIndirectCommand` instance counts via `vkCmdUpdateBuffer`.
  - **`combat_particle_spawn.comp`**: ingests up to `2048` new particles/frame into the GPU ring buffer via O(1) atomic write cursor.
  - **`combat_particle_sim.comp`**: updates all 65,536 particles in parallel (`256` workgroups) with gravity, exponential aero drag, helical curl/buoyancy, camera view-frustum culling, and dual-bucket atomic compaction into `aliveAlpha` + `aliveAdditive` indirect draw buffers (`< 0.15 ms` GPU).
- **Soft-Depth + 360° Flow-Interleaved Rendering (`overScene`)**:
  - **`combat_particle.{vert,frag}`** (2× `vsg::DrawIndirect`: alpha smoke/chaff/dust/shockwave pass -> additive HDR sparks/embers/muzzle-gas pass):
    - Kind 0: Flare corkscrew pyrotechnic white smoke puffs (6-way sunlit, helical turbulent wake).
    - Kind 1: Incandescent metallic sparks & embers (velocity-stretched HDR streaks, blackbody cooling `2600 -> 900 K`).
    - Kind 2: Chaff metallic dipole twinkling glints + realistic translucent silver-grey radar-dipole haze (`alpha <= 0.24`).
    - Kind 3: Muzzle propellant gas puff & HEI airframe hit flash/smoke.
    - Kind 4: Ground/rock/water bullet impact ejecta plumes.
    - Kind 5: Expanding detonation condensation mist rings.
  - **`combat_ribbon.{vert,frag}`** (direct exact-SDK-pose HDR combat primitives):
    - Velocity-aligned glowing **gun tracer streaks** (`14–22 m` motion-blurred white-gold core -> amber tail) & subtle close-range non-tracer slugs.
    - Multi-petal **muzzle flash cone** right on the gun port during firing.
    - Ultra-bright **IR flare magnesium core** with 6-point optical diffraction star glare driven strictly by native `intensityOrSigma`.
    - Rapidly expanding translucent **supersonic detonation shockwave sphere** (Fresnel Mach compression rim + brief Wilson condensation flash, triggered from both `CombatVfxEventKindV1::DetonationShockwave` and `ipc::EngagementKind::Detonation` so legacy replays benefit too!).
  - Full reverse-Z `sceneDepthTex` soft occlusion + `flowFrontTex` volumetric interleaving + `GpuTimer` telemetry (`combat compute`, `combat vfx`).

---

## 3. Verification & 4K Visual Acceptance Gates

1. **Automated Dual-Preset Tests (`tests/simulation/test_combat_vfx.cpp` + `fsim_render_tests`)**:
   - Wire contract layout & validator checks (`80 B`, align 8, zero padding, finite/NaN rejection).
   - `CombatVfxSegmentV1` live IPC roundtrip + `CombatVfxRecorderV1`/`CombatVfxReplayV1` interpolation & event-window seek parity.
   - Native `World` gun burst (`M61A1` tail chase) -> verifies `ProjectileWireV1` tracers, `MuzzleFlash`, `BulletAirframeHit`, and `BulletSurfaceImpact` events + bit-identical seeded determinism.
   - Native `World` flare & chaff salvo -> verifies `DecoyWireV1` intensities, bloom radii, lifetimes, and `CountermeasureEject` events.
   - Full regression: `combat_vfx`, `destruction_debris`, `sim_m10_guns_rcs_ballistics`, `esm_ea_comint`, `sdk_boundary`, `fsim_render_tests`, `mandatory_presentation`, `architecture_boundary` (0 new violations), zero `-Werror` warnings on both `ucrt64-headless` and `ucrt64-release`.
2. **Native 4K Fullscreen Visual & Performance Verification (`desktop-gui` MCP, RTX 5070 Ti, `3840×2160`, `--sun-utc 19` daylight, 1× real time)**:
   - **Showcase 1 — Gun Burst Tail-Chase (`gun_tail_chase.fsrec` + `.vfx` + `.debris`)**: F-16C firing M61A1 20 mm bursts into target F-16C -> verify muzzle flash, blazing tracer stream, metallic ricochet sparks & HEI skin hits, progressive smoke/fire/breakup, and 4K `--stats` (`>= 120–160 fps`, `combat compute` + `combat vfx` `< 0.35 ms`).
   - **Showcase 2 — BVR/WVR Countermeasure & Missile Kill (`cm_missile_kill.fsrec` + `.vfx` + `.debris`)**: F-16C dispensing blazing IR flare curtains (corkscrew white smoke + HDR star cores) and chaff clouds (metallic glints + translucent dipole bloom) against incoming AIM-120C -> proximity-fuze shockwave sphere + Flow fireball + dense GPU burning ember shower.
