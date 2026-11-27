# VIS-M11b — Multi-Grade Progressive Combat Damage & Dynamic Escalation Plan

Date: 2026-11-26
Branch: `main` (base `4eeda5ea`, extending delivered `VIS-M11` + `VIS-M11 v2`)
Status: **Delivered & Verified on `main` (Dual-Preset Gates + 4K Daylight Captures + Wingtip Rail Reparenting Passed)**

---

## 1. Objective

Turn native warhead fragment ray-casting (`Damage.cpp` / `FragmentFieldV1` / Project THOR / Kingery-Bulmash blast) and `VIS-M11` destruction physics into a **true multi-grade, location-, aspect-, and fragment-density-dependent combat damage spectrum** — strictly driven by physical hit geometry and thermal/structural evolution (never scripted), while keeping `LegacyCompatible` default mode, `WorldLayout v9`, `DebrisLayoutV1` (104 B / 64 B), and all public C++/C99/Python ABIs 100% unchanged.

---

## 2. Physical Damage Grade Spectrum (`DamageGrade`)

| Grade | Native Enum (`ms::damage::DamageGrade`) | Typical Physical Trigger (Position × Aspect × Fragment Count/Energy) | Native Simulation & Flight Consequence | 4K Viewer Presentation |
| --- | --- | --- | --- | --- |
| **Grade 0** | `Intact` | Zero perforations / far miss | Normal flight | Intact airframe |
| **Grade 1** | `LightPerforation` | Distant/glancing burst (~9–14 m), sparse fragments (~5–50 hits); skin perforated, optional un-ignited fuel tank hole | Slight `aeroLoss` drag rise, continuous un-ignited fuel leak (`leakKgS`), aircraft stays controllable | Localized metallic impact spark puff + tiny aluminium flakes at struck point; continuous **white vaporizing kerosene mist trail** (`Airflow` mist, `rateKgS = 0, massKg > 0`); light local scorch |
| **Grade 2** | `FireAndStall` | Mid-distance burst (~4.5–8.5 m) or **rear/aft-quarter aspect** striking engine, stabilators/rudder, or aft fuel bay (~40–180 hits) without instant root shear or centre-ullage saturation | `EngineOut` (thrust -> 0), `ControlLost` (elevator/rudder/aileron stuck), sustained engine-bay or fuel-tank fire (`Fire`), progressive airspeed loss -> **aerodynamic stall / nose-down spiral**; ongoing fire thermally weakens burning section roots and soaks centre fuel bays / carried ordnance | Persistent **NVIDIA Flow volumetric flame + thick charcoal-black smoke plume** anchored on burning engine/tank + far-field `Airflow` smoke wake + periodic trailing incandescent embers; camera follows stalling burning fighter |
| **Grade 3** | `StructuralBreakup` | **Concentrated lateral/dorsal wing-root or tail-root hit** (~2.5–4.5 m off wing/tail, ~100–350 hits) shearing 1 section root OR wing-tank ullage blow-off, OR delayed in-flight thermal/load failure of a burning wing/tail | Single major section (`left wing`, `right wing`, `tail`, or `nose`) detaches cleanly without spawning `jsbsim:<piece>`: mass/fuel/stores/lift removed from JSBSim parent (violent asymmetric roll/spin); real `.glb` section + Voronoi shards + sparks enter JoltPhysics `DebrisField`; ongoing fuselage fire can escalate further to Grade 4 or terminate in Grade 5 ground/water crash | Parent hides broken section (`hideSection`), exposes **only centre-attached fracture `BreakCap`** (keeping intact seams spotless), emits wing-root rupture flame/spark burst + Flow fire/smoke, while detached real `.glb` wing/tail tumbles away leaving a curved black smoke trail |
| **Grade 4** | `CatastrophicKill` | **Close-in high-density core strike** (< 3.5–4.5 m beam/dorsal/ventral onto centre fuselage main tanks: concentrated multi-fragment saturation + blast overpressure), direct/thermal ordnance cook-off, >= 2 simultaneous major sections sheared, centre fuselage collapse, or **sustained uncontrolled centre-fuselage fire reaching thermal runaway** | `CatastrophicKill` triggered: TNT-equivalent ullage/cook-off blast + atomized fuel fireball; all **remaining non-separated** `.glb` airframe sections (strictly deduplicated via `alreadySeparatedMask`) + shards + sparks launched conserving linear momentum; simulation instance immediately retired via `World::retireDestroyedVehicles()` | Multi-lobe NVIDIA Flow fireball & rolling charcoal smoke, 360° depth-interleaved real `.glb` breakup, altitude-held smooth debris-follow camera glide easing to a stop |
| **Grade 5** | `SurfaceImpact` | Stricken or diving aircraft impacts Soil / Rock / Water | `GroundWreck` (belly-slide or steep crater disintegration) / `WaterImpact` (hydrodynamic shatter + seawater fountain + quenched flash) | Existing 4K-accepted `VIS-M11 v2` surface impact presentation |

---

## 3. Architectural Changes (Strict Vertical Ownership & Zero ABI Breaks)

1. **Native Damage & Destruction Physics (`src/simulation/mission_systems/damage/`)**:
   - `Destruction.h/.cpp`:
     - Add `DamageGrade` and `HitAspect` enums and additive tracking fields on `VehicleDestructionState` (`grade`, `hitAspect`, `hitFragmentsTotal`, `hitEnergyTotalJ`, `sectionThermalS`, `centreThermalRunawayS`, `engineFireActive`, `engineFireRateKgS`, `engineFireSection`).
     - Extend `TankUllageInput` with `int hitCount = 0` and `bool isCentreSection = false`; extend `DestructionOutcome` with `int rupturedWingSection = -1` and `double rupturedWingSpillKg = 0.0` so **outboard wing-tank ullage ignition ruptures that wing into Grade 3 `StructuralBreakup`** instead of instantaneously vaporizing the entire centre fuselage!
     - Calibrate centre-fuselage immediate ullage detonation so stray distant grazes (1–4 fragments) ignite a controllable/progressive tank fire (Grade 2) while concentrated close-in core penetration (`>= 6` hits & `>= 42 kJ` or high blast overpressure) triggers immediate Grade 4 `CatastrophicKill`.
     - Add progressive in-flight **thermal weakening** (`sectionThermalS`) and **centre-tank thermal runaway** (`centreThermalRunawayS`) so burning fighters dynamically escalate (Grade 2 -> Grade 3 delayed wing/tail breakoff -> Grade 4 delayed secondary explosion) when fire burns intensely enough.
   - `State.h/.cpp`:
     - Add `VehicleDamage::weakenSectionThermal(int section, double deltaRemoved)` (called exclusively in `LightweightDebris` mode by `World::stepDamage()`).
2. **World Runtime & JoltPhysics Debris (`src/simulation/runtime/world/`)**:
   - `WorldDamage.cpp` & `World.h`:
     - Populate `TankUllageInput::{hitCount, isCentreSection}`, `HitAspect` from `impact.fromBody`, and `CatastrophicSpawn::alreadySeparatedMask` (preventing duplicate `.glb` sections during Grade 3 -> Grade 4 delayed escalation!).
     - Handle `outcome.rupturedWingSection >= 0`: cleanly separate that wing section + emit wing-root rupture flame/spark burst without retiring the aircraft.
     - Emit un-ignited fuel-mist leak events (`EngagementKind::Fire` with `rateKgS = 0.0, massKg = leak`) in `LightweightDebris` mode for Grade 1 holed tanks; track persistent burning engine-bay fires (`engineFireActive`) and emit periodic burning embers (`DebrisField::spawnDamageEmbers`) for Grade 2/3 burning flight.
     - Emit localized fragment-impact sparks & aluminium skin flakes (`DebrisField::spawnHitBurst`) at the actual struck panel/component centroid on non-catastrophic warhead hits.
   - `DebrisField.h/.cpp`:
     - Add `CatastrophicSpawn::alreadySeparatedMask` (skipping sections already separated in Grade 3 while still conserving linear momentum across remaining mass).
     - Add `spawnHitBurst(HitBurstSpawn)` (tight directional sparks + small aluminium flakes at actual struck airframe location) and `spawnDamageEmbers(DamageEmberSpawn)` (periodic hot sparks trailing from in-flight engine/fuel fires).
3. **Viewer Visuals (`src/visualization/`)**:
   - `Airflow.cpp`: Enable un-ignited kerosene mist trails (`fire.rateKgS <= 0.0 && fire.mistKgS > 0.0`) even when `fireHandOver_ > 0` (NVIDIA Flow active), so Grade 1 holed fuel tanks stream crisp white vaporizing mist.
   - `VehicleVisuals.{h,cpp}`: Record parent section name on `Joint::BreakCap`; during Grade 3 partial breakup, reveal break caps ONLY on `"centre"` (and `sectionOnly_`) so intact wings/tail/nose retain spotless zero-seam exteriors.
   - `FlowVolumes.cpp`: Ensure burning engine-bay leaks (`F100-PW-229` etc.) anchor accurately to the engine component and keep emitting crisp Flow flames + dark soot while the engine bay burns.

---

## 4. Verification & 4K Visual Acceptance Gates

1. **Automated Unit & Native `World` Tests (`tests/simulation/test_destruction_debris.cpp`)**:
   - Same `AIM-120C` warhead vs. `F-16C` across 4 physical burst geometries (distant glancing flank, rear-quarter engine/tail, concentrated left-wing root, close-in centre fuselage) deterministically produces `LightPerforation` (Grade 1), `FireAndStall` (Grade 2), `StructuralBreakup` (Grade 3), and `CatastrophicKill` (Grade 4).
   - Dynamic escalation test: sustained fire thermally weakens burning wing root -> delayed Grade 3 separation -> delayed Grade 4 secondary explosion with **zero duplicate `.glb` section bodies** and clean vehicle retirement.
   - Bit-identical Jolt determinism, zero `-Werror` warnings on both `ucrt64-headless` & `ucrt64-release`, `sdk_boundary`, `fsim_render_tests`, `mandatory_presentation`, `liquid_native_binding`, and `architecture_boundary` (0 new violations).
2. **Native 4K Fullscreen Visual Acceptance (`desktop-gui` MCP, `3840×2160`, `--sun-utc 19` daylight, 1× real time)**:
   - Generate and visually inspect all 4 mid-air combat grade replays (`grade1_light_leak.fsrec`, `grade2_fire_stall.fsrec`, `grade3_wing_breakup.fsrec`, `midair_kill.fsrec`), verifying distinct visual progression from white fuel-mist leak -> burning stall-spin -> single real `.glb` wing detachment + tumbling -> rapid mid-air explosion & disintegration.
