# VIS-M11 — Vehicle Destruction & Lightweight Debris Architecture Redesign

Date: 2026-11-26  
Branch: `main` (v1 base `0795eedc`, v2 upgrade on `c8e6e882`)  
Backlog closed: **B-54**, **B-55**, **B-58**

---

## 1. Architecture Summary

`VIS-M11` replaces heavyweight JSBSim child-vehicle breakup spawning (`jsbsim:<aircraft>-<section>`) in opt-in `World::DestructionModeV1::LightweightDebris` mode with a physics-driven multi-tier destruction state machine, a deterministic JoltPhysics 5.3.0 fixed-capacity 6-DOF tumbling rigid-body debris field, immediate simulation-instance retirement on catastrophic/shatter destruction, an additive zero-break IPC/replay sidecar, and a 360° depth-interleaved GPU instanced destruction renderer — while keeping `World::DestructionModeV1::LegacyCompatible` as default so all frozen `SIM-M02`/`SIM-M03` tests, `WorldLayout v9`, `VehicleState`, `fsrec v8`, and public C++/C99/Python ABIs remain 100% unchanged.

### Core Modules & Wire Contracts

| Layer | Files | Responsibility |
| --- | --- | --- |
| **Wire Contract** | `src/contracts/world_stream/DebrisLayoutV1.h` | Frozen POD wire structs: `DebrisBodyWireV1` (104 B, align 8), `VehicleDestructionWireV1` (64 B, align 8), `DestructionTierV1`, `DebrisArchetypeV1`, `SurfaceKindV1`, header/slot structs, and strict finite/quaternion validators. |
| **IPC & Replay Sidecar** | `src/foundation/transport/DebrisSegmentV1.{h,cpp}`, `WorldPublisher.{h,cpp}`, `WorldMirror.{h,cpp}` | Wait-free triple-buffered seqlock shared-memory segment (`fsim.debris.v1.<world>`) + lazy companion `<recordPath>.debris` recorder (`DebrisRecorderV1`) and slerped/interpolated reader (`DebrisReplayV1`). |
| **Destruction & Surface Physics** | `src/simulation/mission_systems/damage/Destruction.{h,cpp}`, `Impact.{h,cpp}` | Multi-tier native state machine (`Intact -> Damaged -> StructuralBreakup -> CatastrophicKill -> GroundWreck / WaterImpact`), MIL-HDBK-2069 / Ball fuel-tank ullage overpressure ignition, sympathetic carried-HE store cook-off (direct strike & sustained fire soak), hydrodynamic-ram spray exit speed (`B-58`), early gear-overload / belly-scrape impact trigger (`B-54`), and Soil / Water / Rock compliance, friction, restitution, and pool-fire quenching (`B-55`). |
| **Flight Contact Seam** | `src/simulation/flight/FlightModel.h`, `JsbsimModel.{h,cpp}` | Additive `TerrainContactLoadV1` / `TerrainContactLoadSourceV1` exposing bogey gear sink speed and structural belly compression depth prior to 1.0 m penetration without altering the `FlightModel` vtable or `VehicleState`. |
| **JoltPhysics 6-DOF Debris Pool & Retirement** | `src/simulation/runtime/world/DebrisField.{h,cpp}`, `World.{h,cpp}`, `WorldDamage.cpp`, `cmake/Dependencies.cmake` | Fixed-capacity 512-body deterministic single-threaded JoltPhysics (`Jolt::Jolt`) 6-DOF rigid-body pool over local ENU terrain heightfields and hydrodynamic ocean drag arrest (`~25–45 m`), with aerodynamic flat-plate drag & tumble torque, Coulomb belly-slide clearance (`>= 1.55 m` AGL), 24-curtain conical ocean spray plumes + sparse lacy seafoam rings, steep-dive crater scorch & dust skirt, fuel burn-down, radiative cooling, and exact linear momentum conservation. `World::retireDestroyedVehicles()` removes destroyed JSBSim instances (`hullRetired == true`) immediately in `LightweightDebris` mode while preserving `retiredDestruction_` metadata for referee/API queries; water-entry flash is quenched to `min(fireballKg * 0.015, 5.5)` kg. |
| **360° Interleaved Destruction Renderer & Camera** | `src/visualization/vfx/effects/DebrisVisuals.{h,cpp}`, `FlowVolumes.{h,cpp}`, `FlowFires.cpp`, `src/visualization/scene/VehicleVisuals.{h,cpp}`, `CameraController.{h,cpp}`, `assets/clouds/shaders/debris*.{vert,frag}`, `flow_march.comp`, `src/applications/viewer/main.cpp` | Real airframe `.glb` section fragments (`manifold::manifold` break-cap/section extraction, `Joint::BreakCap` hidden on intact hulls) + procedural charred metallic shards; 360° 3-D volumetric depth interleaving (`debris.trailNode()` back pass -> `flow->compositeNode()` -> `debris.frontTrailNode()` front pass) using bilinear `VK_FORMAT_R32G32_SFLOAT` `(frontDist, alpha)` `flowFrontTex` feathering; neutral-cool charcoal soot albedo (`vec3(0.58, 0.61, 0.66)`) with `3.2 m / 1.4 m` Worley-Perlin sculpting in `flow_march.comp`; densely overlapping (`0.52 * baseR`, `kMaxSamplesPerBody = 64`) billowing volumetric charcoal smoke puffs; 1:1 world-meter isotropic seawater spray (`hNorm * 2.0 * aspect * 0.76`) and sparse seafoam (`alpha <= 0.40`); smooth debris-follow camera glide with interpolation-lag bridging and altitude hold (`CameraController::holdAt` / `driftFocus`) easing to a full stop after wreckage dispersion (`spreadM > 95 m` or `deadAge > 3.25 s`). |

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
| `destruction_debris` (`tests/simulation/test_destruction_debris.cpp`) | `ucrt64-headless` | **PASS** (7/7 cases, 456/456 assertions, `13.86 s`) | Wire contracts, ullage envelope, hydrodynamic ram, B-54/B-55 surfaces, zero-`jsbsim:<piece>` breakup + momentum conservation, catastrophic `AIM-120C` kill + immediate vehicle retirement (`vehicleIds().empty()`), belly-slide + tank fuel burn-down, steep dive + retirement, water impact + retirement, bit-identical JoltPhysics determinism (`std::memcmp == 0`), `.debris` replay + `DebrisSegmentV1` live IPC, and full `VIS-M11b` 4-grade mid-air combat damage progression + aspect classification + dynamic thermal weakening/runaway escalation + Grade 3→4 section deduplication. |
| `destruction_debris` | `ucrt64-release` | **PASS** (7/7 cases, 456/456 assertions, `13.88 s`) | Exact parity with headless. |
| `sdk_boundary` | `ucrt64-headless` & `ucrt64-release` | **PASS** (`2.46 s` H / `3.00 s` R) | 0 boundary/ABI regressions. |
| `liquid_native_binding` | `ucrt64-headless` & `ucrt64-release` | **PASS** (`79.55 s` H / `79.68 s` R) | Full SIM-M03 liquid/piece preservation verified in `LegacyCompatible` default mode. |
| `fsim_tests "[impact],[mass]"` | `ucrt64-headless` | **28/29 cases (5,070/5,071 assertions)** | All `test_impact` & `test_mass_change` pass 100%; 1 pre-existing unrelated `mig29a` tail break assertion (`test_damage.cpp:1979`, unchanged by `VIS-M11`/`VIS-M11b` diffs). |
| `fsim_render_tests` & `mandatory_presentation` | `ucrt64-release` | **PASS** (63/63 cases, 5,296/5,296 assertions; `mandatory_presentation` `0.02 s`) | Full `VehicleVisuals` (including wingtip rail reparenting on wing detachment) / terrain / CoCiP / contrail / `GpuTimer` verification. |
| `architecture_boundary` | `ucrt64-release` | **0 new violations** | `Jolt::Jolt` and `manifold::manifold` declared in `tools/architecture/policy.json`; only the 4 pre-existing unrelated uncategorized files (`frozen_thermal_image_receiver_v1.{h,cpp}`, `{,test_}clean_hangar_data.py`). |

---

## 4. Native 4K Fullscreen Visual & Performance Evidence (`desktop-gui` MCP)

Interactively reviewed and accepted across all four native `World` + `.debris` replays generated by `fsim_destruction_debris_test "[visual-gen]"` (`FSIM_VIS_M11_RECORD_DIR`) on the physical interactive desktop (`WinSta0\Default`, NVIDIA GeForce RTX 5070 Ti, **`3840×2160` fullscreen**, **1× real time**, `--sun-utc 19` daylight, complete Earth/terrain/clouds/ocean/sunlight/shadows/Flow/debris scene enabled):

1. **Mid-air Catastrophic Kill (`midair_kill.fsrec` + `.debris`)**:
   - `F-16C` struck at `2,500 m`, `230 m/s` (`55%` fuel) by close-in `AIM-120C` -> `ullage_explosion`, simulation vehicle retired immediately (`vehicles 1 -> 0` at `t = 1.4 s`), real airframe `.glb` sections & charred tumbling debris erupt alongside the Flow fireball with 360° depth-interleaved billowing charcoal smoke puffs, while the camera holds altitude, glides smoothly with the separating major airframe sections, and eases to a halt upon dispersion.
   - **Real-time 4K `--stats 6` telemetry (`t = 2..5 s`)**: **`158.1 – 160.3 fps`** (`p50 = 6.18–6.23 ms`, total 4K GPU `5.223–5.422 ms`, `app` CPU `0.11–0.13 ms`); `debris solid` **`0.018–0.038 ms`**, `debris trails back` **`0.042–0.264 ms`**, `flow composite` **`0.017–0.034 ms`**, `debris trails front` **`0.000–0.243 ms`**, Flow solver GPU avg `0.35–0.73 ms` (`730–2,559` active leaves).
2. **Ground Shallow Belly-Slide (`ground_slide.fsrec` + `.debris`)**:
   - `F-16C` shallow belly-skid (`7.3°`) decelerates across terrain (`vehicles 1` retained while skidding) with fiery Flow skid trail, scraping sparks, tumbling airframe fragments, and live tank fuel burn-down.
   - **Real-time 4K `--stats 6` telemetry (`t = 2..5 s`)**: **`159.7 – 160.2 fps`** (`p50 = 6.22–6.24 ms`, total 4K GPU `2.445–3.238 ms`, `debris solid 0.003–0.006 ms`, `debris trails back 0.083–0.609 ms`, `flow composite 0.029–0.067 ms`, `debris trails front 0.064–0.148 ms`).
3. **Pacific Ocean Water Impact (`water_impact.fsrec` + `.debris`)**:
   - `F-16C` impacts water off Golden Gate (`~20°`, early `B-54` arrest) -> simulation instance retired immediately (`vehicles 1 -> 0` at `t = 1.24 s`), hydrodynamic drag arrests debris travel, towering 24-curtain 1:1 isotropic white seawater spray spire & sparse lacy seafoam surround a compact water-quenched `5.04 kg` impact flash with seamless 360° Flow/spray volumetric interleaving.
   - **Real-time 4K `--stats 6` telemetry (`t = 2..5 s`)**: **`159.8 – 160.2 fps`** (`p50 = 6.19–6.25 ms`, total 4K GPU `3.765–4.417 ms`, `debris solid 0.206–0.317 ms`, `debris trails back 0.193–0.410 ms`, `flow composite 0.013–0.025 ms`, `debris trails front 0.110–0.559 ms`).
4. **Steep Dive Ground Disintegration (`steep_dive.fsrec` + `.debris`)**:
   - `F-16C` impacts terrain at `~65°`, `203.5 m/s` -> simulation instance retired immediately (`vehicles 1 -> 0` at `t = 1.22 s`), `913.95 kg` fuel spill fireball, crater scorch, dust skirt, bouncing JoltPhysics airframe chunks & billowing charcoal smoke trails, smooth camera glide & stop.
   - **Real-time 4K `--stats 6` telemetry**: **`156.4 – 159.9 fps`** across impact (`t = 2..4 s`, total GPU `2.919–5.252 ms`), and **`122.7 – 137.2 fps`** during close-up viewport-filling `3,347–3,766`-leaf rising plume (`t = 5..6 s`, total GPU `6.493–7.254 ms`, `debris solid 0.023–0.036 ms`, `debris trails back 0.231–0.678 ms`, `debris trails front 0.091–0.404 ms`, `flow composite 0.020–0.077 ms`).

---

## 5. VIS-M11b — Multi-Grade Progressive Combat Damage & Dynamic Escalation

Extending `VIS-M11 v2`, `VIS-M11b` maps native warhead fragment hit location, 3-D aspect (`HitAspect`), fragment count/energy, per-tank local blast attenuation, and in-flight thermal/structural evolution into a physical 5-grade combat damage spectrum (`DamageGrade`: `Intact`, `LightPerforation`, `FireAndStall`, `StructuralBreakup`, `CatastrophicKill`, `SurfaceImpact`) with zero changes to `LegacyCompatible` default mode or frozen wire/ABI layouts:

- **Physical Location & Aspect Discrimination (`Destruction.{h,cpp}`, `State.{h,cpp}`, `WorldDamage.cpp`)**:
  - `World::evaluateEntryDestruction` aggregates multi-box fuel cells by real physical tank index using authored section tank capacity (`sec.tanks`), computes per-tank local blast overpressure from adjacent struck panels (attenuating internal baffled centre fuselage tanks behind intact skin vs. wet outboard wing cells), and evaluates centre-fuselage blast collapse from area-weighted centre-panel blast rather than distant tail/wingtip peak overpressure.
  - Outboard wet wing-tank ullage ignition sets `DestructionOutcome::rupturedWingSection` to shear **that wing** cleanly into Grade 3 `StructuralBreakup` (with forced burning on the departing `.glb` wing and a localized root fiery snap) instead of instantaneously vaporizing the centre fuselage.
  - Sustained in-flight bay fires thermally weaken load-bearing roots (`kThermalWeakenOnsetS = 1.2 s`, `kThermalWeakenRatePerS = 0.085 /s` via `VehicleDamage::weakenSectionThermal`) and intense centre-bay fires (`>= 2.4 kg/s`) accumulate `centreThermalRunawayS` toward delayed secondary Grade 4 `CatastrophicKill`, while `CatastrophicSpawn::alreadySeparatedMask` guarantees already-detached `.glb` sections are never duplicated during Grade 3→4 escalation.
  - Non-catastrophic hits emit localized metallic sparks & aluminium flakes (`DebrisField::spawnHitBurst`) at the struck panel/component centroid; in-flight burning engine/fuel bays shed periodic trailing embers (`DebrisField::spawnDamageEmbers`) and draw down live tank fuel.
- **Viewer Visual Discrimination (`Airflow.cpp`, `VehicleVisuals.cpp`, `DebrisField.cpp`)**:
  - `Airflow::ignite` and `Airflow::update` render un-ignited perforated fuel-tank leaks (`rateKgS = 0, massKg > 0`) as crisp, streamlined white vaporizing kerosene mist ribbons (`0.36 * kFireWidth`) even with NVIDIA Flow active.
  - `VehicleVisuals` tags `Joint::BreakCap` with its parent section so Grade 3 partial breakup reveals only the `"centre"` fracture cap while keeping intact attached seams clean, and `reparentOutboardWingtipIslands` dynamically moves outboard wingtip appendage triangles (`|y| > 0.88 * halfSpan` within wing X/Z bounds, such as LAU-129 wingtip missile rails, wingtip EW pods, and winglets left in `centre`/`tail` by Hangar's 2-D planform slicer) onto `fsim:section:left-wing` / `right-wing` at load time while discarding their artificial rail cut-caps — leaving zero floating wingtip hardware on the broken parent airframe and keeping the full launcher rail intact on the severed `.glb` wing without modifying on-disk `.glb` hashes. `DebrisField` uses physically averaged tumbling wing drag area (`clamp(0.24 * span^2, 0.9, 3.8) m^2`) so severed `.glb` wings peel off and tumble visibly inside the chase view.
- **Verified Native 4K Fullscreen Replays (`desktop-gui` MCP, RTX 5070 Ti, `3840×2160`, `--sun-utc 19` daylight, 1× real time)**:
  1. **Grade 1 — Distant Flank Graze (`grade1_light_leak.fsrec`)**: `aim-9m` at `12.1 m` (`HitAspect::Right`, `56` fragments) -> localized metallic flakes/sparks + razor-sharp white vaporizing kerosene mist trails streaming off holed fuel cells (`burningTanks = 0`, `engineFire = 0`), controlled level flight (`137 fps`, `5.6 ms` HUD).
  2. **Grade 2 — Rear-Quarter Burning Stall (`grade2_fire_stall.fsrec`)**: `aim-9m` at `6.5 m` rear quarter (`HitAspect::Rear`, `403` fragments) -> `EngineOut` + `F100-PW-229` engine-bay fire (`0.45–0.78 kg/s`) -> compact roaring tail flame transitioning in frame into thick billowing charcoal-black smoke + glowing embers + wing-root mist leak while airspeed bleeds (`140 -> 115 m/s`), AoA climbs (`4.9° -> 9.5°`), and aircraft banks into a stalling descent (`137–145 fps`, `6.0–6.4 ms` HUD).
  3. **Grade 3 — Left-Wing Structural Breakup (`grade3_wing_breakup.fsrec`)**: `aim-9m` at `4.29 m` off left wing (`HitAspect::Left`, `744` fragments) -> wet wing-cell ullage rupture shears real `.glb` `left wing` off its root cleanly (`vehicles 1` alive, `hullRetired = false`), tumbling visibly through the chase frame while the crippled one-winged `F-16C` rolls violently inverted (`roll -13° -> -80° -> -155° -> 129°`) trailing engine fire, thick black smoke, sparks, and kerosene mist.
     - **Real-time 4K `--stats 6` telemetry (`t = 2..5 s`)**: **`158.5 – 160.8 fps`** (`p50 = 6.14–6.37 ms`, total 4K GPU **`4.861–5.140 ms`**, `debris solid 0.000–0.007 ms`, `debris trails back 0.014–0.023 ms`, `debris trails front 0.001–0.084 ms`, `flow march 0.069–0.152 ms`, `flow composite 0.018–0.044 ms`, `contrails 0.055–0.105 ms`, Flow solver `569–1,977` leaves, GPU avg `0.39–1.14 ms`).
  4. **Grade 4 — Instant Mid-Air Catastrophic Disintegration (`midair_kill.fsrec`)**: Close-in `aim-120c` at `2.96 m` (`396` heavy fragments saturating centre main tanks) -> unchanged instant `CatastrophicKill`, immediate vehicle retirement, and smooth debris-follow camera glide (`158.1–160.3 fps`).

---

## 6. Public `DestructionModeV1` API & Follow-Up Hardening (2026-11-27)

- **Additive Public API (`include/fsim/{World.h,fsim_c.h}`, `src/simulation/api/World.cpp`, `src/adapters/sdk/{c/fsim_world_c.cpp,python/_native*.{c,h}}`, `python/fsim/{__init__.py,world.py}`, `docs/sdk/boundary.{toml,md}`)**: Exposed `DestructionModeV1` (`LegacyCompatible = 0` default, `LightweightDebris = 1`) across public C++ (`World::setDestructionModeV1` / `destructionModeV1`), C99 (`fsim_world_set_destruction_mode_v1` / `fsim_world_destruction_mode_v1`, returning `FSIM_NOT_ALLOWED` for `Agent` handles and `FSIM_INVALID_ARGUMENT` for unknown modes), and Python (`fsim.DestructionModeV1`, `World(..., destruction_mode=...)`, `set_destruction_mode_v1`, `destruction_mode_v1`). Verified in `world_adapter_compatibility`, `test_world_binding_compatibility.py`, and `sdk_boundary` on both `ucrt64-release` and `ucrt64-headless`.
- **Steep-Impact Disintegration & Smoke/Flow Polish (`Destruction.cpp`, `Airflow.cpp`, `contrail.frag`, `FlowVolumes.cpp`)**: Lowered land steep-dive disintegration threshold in `applyGroundOrWaterDestruction()` to `>= 45°` with `>= 32 m/s` normal speed (or `>= 55 m/s` normal at any angle) so stalled high-sink-rate terrain crashes detonate & retire cleanly on the surface rather than clipping intact; fixed 30 Hz NanoVDB export phase-lock via `stepsSinceExport` in `FlowVolumes.cpp`; gated `Airflow.cpp` fire/mist wake trails to airborne non-crashed hulls with `>= 1.5 m` sample spacing and `18 m` radius cap plus smooth `3.2σ` zero-density radial falloff in `contrail.frag`. Verified in `destruction_debris` (`5.17 s` R / `5.19 s` H) and `fsim_render_tests "~[gpu]"` (63/63 cases, 5,296/5,296 assertions).
