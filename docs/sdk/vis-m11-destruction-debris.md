# VIS-M11 — Vehicle Destruction & Lightweight Debris Architecture Redesign

Date: 2026-11-26  
Branch: `main` (base `0795eedc`)  
Backlog closed: **B-54**, **B-55**, **B-58**

---

## 1. Architecture Summary

`VIS-M11` replaces heavyweight JSBSim child-vehicle breakup spawning (`jsbsim:<aircraft>-<section>`) in opt-in `World::DestructionModeV1::LightweightDebris` mode with a physics-driven multi-tier destruction state machine, a deterministic fixed-capacity 6-DOF tumbling rigid-body debris field, an additive zero-break IPC/replay sidecar, and a GPU instanced destruction renderer — while keeping `World::DestructionModeV1::LegacyCompatible` as default so all frozen `SIM-M02`/`SIM-M03` tests, `WorldLayout v9`, `VehicleState`, `fsrec v8`, and public C++/C99/Python ABIs remain 100% unchanged.

### Core Modules & Wire Contracts

| Layer | Files | Responsibility |
| --- | --- | --- |
| **Wire Contract** | `src/contracts/world_stream/DebrisLayoutV1.h` | Frozen POD wire structs: `DebrisBodyWireV1` (104 B, align 8), `VehicleDestructionWireV1` (64 B, align 8), `DestructionTierV1`, `DebrisArchetypeV1`, `SurfaceKindV1`, header/slot structs, and strict finite/quaternion validators. |
| **IPC & Replay Sidecar** | `src/foundation/transport/DebrisSegmentV1.{h,cpp}`, `WorldPublisher.{h,cpp}`, `WorldMirror.{h,cpp}` | Wait-free triple-buffered seqlock shared-memory segment (`fsim.debris.v1.<world>`) + lazy companion `<recordPath>.debris` recorder (`DebrisRecorderV1`) and slerped/interpolated reader (`DebrisReplayV1`). |
| **Destruction & Surface Physics** | `src/simulation/mission_systems/damage/Destruction.{h,cpp}`, `Impact.{h,cpp}` | Multi-tier native state machine (`Intact -> Damaged -> StructuralBreakup -> CatastrophicKill -> GroundWreck / WaterImpact`), MIL-HDBK-2069 / Ball fuel-tank ullage overpressure ignition, sympathetic carried-HE store cook-off (direct strike & sustained fire soak), hydrodynamic-ram spray exit speed (`B-58`), early gear-overload / belly-scrape impact trigger (`B-54`), and Soil / Water / Rock compliance, friction, restitution, and pool-fire quenching (`B-55`). |
| **Flight Contact Seam** | `src/simulation/flight/FlightModel.h`, `JsbsimModel.{h,cpp}` | Additive `TerrainContactLoadV1` / `TerrainContactLoadSourceV1` exposing bogey gear sink speed and structural belly compression depth prior to 1.0 m penetration without altering the `FlightModel` vtable or `VehicleState`. |
| **Lightweight 6-DOF Debris Pool** | `src/simulation/runtime/world/DebrisField.{h,cpp}`, `World.{h,cpp}`, `WorldDamage.cpp` | Fixed-capacity 512-body deterministic 6-DOF tumbling rigid-body pool (flat-plate incidence drag, pitch-tumbling torque, ground/water bounce & slide, fuel burn-down, radiative cooling, exact linear momentum conservation). Zero `jsbsim:<piece>` vehicles spawned in `LightweightDebris` mode; Coulomb belly-slide deceleration with surface CG clearance and live JSBSim tank fuel draw-down; debris-cloud centroid pose tracking after `CatastrophicKill`. |
| **Viewer Destruction Renderer** | `src/visualization/vfx/effects/DebrisVisuals.{h,cpp}`, `src/visualization/scene/VehicleVisuals.{h,cpp}`, `assets/clouds/shaders/debris*.{vert,frag}`, `src/applications/viewer/main.cpp` | 4 procedural faceted aerospace debris meshes (`AirframeHull`, `WingFragment`, `PanelShard`, `BurningChunk`) rendered in 4 instanced draw calls with charred metallic PBR + fracture-edge blackbody glow; single-draw instanced premultiplied translucent pass for razor-thin velocity-stretched incandescent sparks, velocity-aligned 3-octave FBM dark hydrocarbon soot ribbons, turbulent flame cores, and hydrodynamic ocean spray plumes; clean intact-hull retirement (`VehicleVisuals::setHullRetired`) on `CatastrophicKill`. |

---

## 2. Physics Grounding & Backlog Closures

- **B-58 (Catastrophic Mid-Air Disintegration, Ullage Overpressure, Sympathetic Cook-Off & Hydrodynamic Ram)**:
  - `ullageFlammability(ullageFraction)` gates confined vapour deflagration strictly to flammable headspace (`0.12 <= u <= 0.96`; liquid-full `< 0.12` and bone-dry `> 0.96` tanks cannot explode from ullage vapour).
  - High-energy fragment/incendiary deposition (`>= 18 kJ`) or blast rupture (`>= 240 kPa`) into flammable ullage generates confined overpressure up to `720 kPa`; exceeding structural collapse (`350 kPa`), catastrophic centre-fuselage skin removal (`>= 65%`), blast crush (`>= 450 kPa`), or direct/thermal sympathetic detonation of carried HE stores (`>= 12 kJ` strike, `>= 250 kPa` blast, or `>= 6.0 s` section fuel-fire soak) triggers Tier 3 `CatastrophicKill`.
  - Secondary blast TNT equivalent and aerosolised fireball fuel derive strictly from real headspace vapour combustion energy (`2.8 MJ/m^3`), actual carried HE mass, and actual remaining tank fuel.
  - `hydrodynamicRamSpraySpeedMs(E_frag, v_frag, m_fuel)` derives exit spray speed physically from kinetic energy deposited in kerosene ($\rho = 800\text{ kg/m}^3$) instead of an arbitrary velocity fraction.
- **B-54 (Early Gear/Belly Contact, Belly Slide & Fuel Burn-Down)**:
  - `exceedsGearOrBellyContact(sinkMs, bellyDepthM, tangentMs)` triggers impact when bogey gear sink speed exceeds `6.5 m/s` (MIL-A-8863C fighter limit) or structural belly contact depth reaches `0.05 m`, replacing the hard 1.0 m penetration wait.
  - Shallow strikes transition into `GroundWreck` Coulomb belly-slide deceleration ($\mu g$ with belly CG clearance and continuous scrape sparks), while fireball fuel is immediately deducted from JSBSim tanks and residual pool fire draws down live tank fuel step-by-step until exhaustion.
- **B-55 (Surface-Aware Impact Physics & Debris/Splash Clouds)**:
  - Distinct `SurfaceKind::Soil`, `SurfaceKind::Water` (NACA TN-2929 deeper `1.6 m` hydrodynamic give, lower `0.22` skimming friction, `0.04` restitution, `0.35x` slick-fire quenching, displaced `SplashPlume` rooster-tail spray), and `SurfaceKind::Rock` (`0.05 m` rigid give, `0.75` friction, higher peak g-load).

---

## 3. Automated Test & Regression Evidence

All builds compiled with zero warnings (`-Werror`) on both `ucrt64-headless` and `ucrt64-release`:

| Verification Suite | Preset | Result | Notes |
| --- | --- | --- | --- |
| `destruction_debris` (`tests/simulation/test_destruction_debris.cpp`) | `ucrt64-headless` | **PASS** (6/6 cases, 383/383 assertions, `9.04 s`) | Wire contracts, ullage envelope, hydrodynamic ram, B-54/B-55 surfaces, zero-`jsbsim:<piece>` breakup + momentum conservation, catastrophic `AIM-120C` kill, belly-slide + tank fuel burn-down, steep dive, water impact, bit-identical determinism, `.debris` replay + `DebrisSegmentV1` live IPC. |
| `destruction_debris` | `ucrt64-release` | **PASS** (6/6 cases, 383/383 assertions, `8.93 s`) | Exact parity with headless. |
| `sdk_boundary` | `ucrt64-headless` & `ucrt64-release` | **PASS** (`2.33 s` H / `2.78 s` R) | 0 boundary/ABI regressions. |
| `liquid_native_binding` & `liquid_vessels_native` | `ucrt64-headless` | **PASS** (`78.48 s`, `74.81 s`) | Full SIM-M03 liquid/piece preservation verified in `LegacyCompatible` default mode. |
| `fsim_tests "[impact],[damage],[mass]"` | `ucrt64-headless` | **121/122 cases (12,555/12,556 assertions)** | All `test_impact` & `test_mass_change` pass 100%; 1 pre-existing unrelated `mig29a` tail break assertion (`test_damage.cpp:1979`, unchanged by `VIS-M11` diffs). |
| `fsim_render_tests` & `mandatory_presentation` | `ucrt64-release` | **PASS** (62/62 cases, 5,251/5,251 assertions) | Full `VehicleVisuals` / terrain / CoCiP / contrail / `GpuTimer` verification. |
| `architecture_boundary` | `ucrt64-release` | **0 new violations** | Only the 4 pre-existing unrelated uncategorized files (`frozen_thermal_image_receiver_v1.{h,cpp}`, `{,test_}clean_hangar_data.py`). |

---

## 4. Native 4K Fullscreen Visual & Performance Evidence (`desktop-gui` MCP)

Executed on the physical interactive desktop (`WinSta0\Default`, NVIDIA GeForce RTX 5070 Ti, **`3840×2160` fullscreen**, **1× real time**, complete Earth/terrain/clouds/ocean/sunlight/shadows/Flow/debris scene enabled) across three native `World` + `.debris` replays generated by `fsim_destruction_debris_test "[visual-gen]"`:

1. **Mid-air Catastrophic Kill (`midair_kill.fsrec` + `.debris`)**:
   - `F-16C` struck at `2,500 m`, `230 m/s` (`55%` fuel) by close-in `AIM-120C` -> `ullage_explosion`, intact hull cleanly retired (`hullRetired = 1`, zero floating Lego wings), `60` active tumbling charred metal chunks & glowing embers erupting alongside the Flow fireball, continuous velocity-aligned FBM soot ribbons darkening to charcoal-black smoke behind every burning shard, camera tracking live debris-cloud centroid as airspeed decays `214 -> 172 -> 147 m/s`.
   - **Real-time 4K `--stats 6` telemetry** (`vehicles 1` constant throughout):
     - Sustained FPS during active disintegration (`t = 2..5 s`): **`155.7 – 159.5 fps`** (`p50 = 6.32–6.43 ms`, total 4K GPU `5.26 – 5.71 ms`, `app` CPU `0.13 – 0.20 ms`).
     - **`debris solid` GPU pass**: **`0.003 – 0.004 ms`** (40+ 3-D faceted sunlit/incandescent chunks at 4K).
     - **`debris trails` GPU pass**: **`0.525 – 0.947 ms`** (up to 4,096 velocity-stretched FBM soot ribbons, flame cores & sparks at 4K).
     - **`flow march + composite`**: **`0.006 – 0.097 ms`** (up to `3,969` active Flow leaves).
2. **Ground Shallow Belly-Slide (`ground_slide.fsrec` + `.debris`)**:
   - `F-16C` impacts at `151.9 m/s`, `7.5°` -> `GroundWreck` leveled belly-slide decelerating smoothly across terrain with fiery Flow skid trail, razor-thin white-hot scraping sparks, flying charred wing/panel debris, and live tank fuel burn-down (`159–161 fps` at 4K).
3. **Pacific Ocean Water Impact (`water_impact.fsrec` + `.debris`)**:
   - `F-16C` impacts water off Golden Gate at `168.8 m/s`, `18.3°` (early `B-54` arrest prior to 1.0 m penetration) -> `WaterImpact` with massive billowing white hydrodynamic FBM ocean rooster-tail spray plumes over FFT waves, tumbling sheared metal debris, and quenched surface slick burn.
