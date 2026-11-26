# SIM-M10: Aircraft Guns, B-33 Mass Conservation, B-37 Loaded-Store RCS & B-50 Bomb Ballistics

Date: 2026-11-26 (`main` base `93a34a2`)
Alignment reference: [open-source-alignment.md](../research/sim-m10/open-source-alignment.md)

## 1. Scope Delivered

SIM-M10 completes the final milestone on the simulation roadmap across three independent physics and mission-systems areas, preserving all frozen SDK/C99/Python ABI contracts, `fsim::Agent` permission boundaries, and clean-airframe compatibility:

1. **Aircraft Guns, B-33 Ammunition Mass Conservation & Ballistic Air-to-Air Damage (`MS-9c` / `B-33`)**:
   - Implemented `fsim::ms::gun::GunModel` (`src/simulation/mission_systems/gun/Gun.h`, `Gun.cpp`) wired into `ms::makeModel` (`Model.cpp`) and `ms::Host` (`CapabilityType::Strike` promoted to `Supported` for `Family::Gun`, `TaskKind::Fire` accepted on operating guns with `ControlMode::Granted` authority checks).
   - Covers all **11/11** maintained `Family::Gun` cards (`m61a1`, `m61a2`, `gau-22a`, `gau-8a`, `bk-27`, `defa-554`, `giat-30m791`, `gsh-23`, `gsh-30-1`, `gsh-30-2`, `m60d`) with published/derived calibre, cyclic rate of fire (scaled by installed mount count, e.g., dual `DEFA-554` on Mirage 2000), muzzle velocity, projectile mass, total shed mass per round, HEI explosive fill, dispersion, and McCoy/STANAG 4355 quadratic atmospheric drag retardation $k = \frac{1}{2}\rho_0 C_D A / m$.
   - **B-33 ammunition mass conservation**: `Host::collectShedMass()` accumulates shed cartridge/projectile mass and bumps `Host::massRevision()`; `StoreFlightBinding` binds the JSBSim `"gun ammunition"` point mass and reduces it in real time as rounds leave the aircraft, updating flight-model mass and CG automatically.
   - **Ballistic projectile flight & native damage**: `World::stepGunProjectiles(double dt)` integrates every fired round in WGS-84 ECEF under standard-atmosphere quadratic drag and gravity with seeded Gaussian dispersion (`gunRng_`), performs swept segment intersection against target `Airframe` panels and internal components via `ms::damage::assessProjectileSegment(...)` (Project THOR penetration + optional HEI Kingery-Bulmash blast), applies native `VehicleDamage` through `World::applyDamagePreparedV1(...)`, and emits `EngagementEvent` (`kind == "damage"`, `reason == "gun"`) and `ipc::EngagementKind::Hit` records without fabricating `crewLost`.
   - Exposed `Vehicle.fire_gun` / `Agent.fire_gun` in Python (`python/fsim/world.py`, `python/fsim/agent.py`, `src/adapters/sdk/python/_native_mission.c`) and registered both in `docs/sdk/boundary.toml` / `docs/sdk/boundary.md`.

2. **Loaded-Store Radar Cross Section & Stealth Bay Shielding (`B-37`)**:
   - Implemented physical-optics external store RCS estimation (`src/simulation/mission_systems/rcs/StoreRcs.{h,cpp}`, Knott / Ruck cylinder broadside + ogive/pylon dihedral model) and wired external store summation into `World::radarCrossSection(...)` (with `rcsTablesMutex_` thread-safety for parallel `sense()` workers).
   - Uses precomputed per-store `rcs.bin` tables whenever available (e.g., `aircraft/mk82/rcs.bin`), falling back to `externalStoreSigmaM2(...)` for fuel tanks, jamming pods, bombs, and missiles without precomputed tables.
   - **Stealth bay rule**: stores housed inside closed internal weapon bays (`!station->bay.empty() && bayDoors <= 0.1`) contribute **0.0 m²**, preserving clean low-observable RCS on `F-35A` / `F-22A` bit-identically.

3. **Mk 82 Free-Fall Bomb Ballistic Validation (`B-50`)**:
   - Added live 6-DOF JSBSim `mk82` release cross-check from an `F-16C` at 3,000 m MSL / 220 m/s level flight (calm and 20 m/s crosswind) against an independent RK4 standard-atmosphere point-mass ballistic ODE using `assets/stores/mk82.json` parameters (`mass_kg = 226.8`, `drag_area_m2 = 0.00952846`).

## 2. Measured Verification Evidence (2026-11-26)

All code compiled with `-Werror` and **0 warnings** on both `ucrt64-headless` and `ucrt64-release`.

### SIM-M10 Quantitative Results (`sim_m10_guns_rcs_ballistics`, 5/5 cases, 244/244 assertions)

| Check | Measured Result | Reference / Expectation | Basis |
| --- | --- | --- | --- |
| Gun cards constructed | **11 / 11** (`m61a1`..`m60d`) | All maintained `Family::Gun` cards | `published` / `derived` |
| B-33 mass delta (`F-16C` `M61A1`, 100 rounds vs. clean twin) | **15.50 kg** | `100 × 0.155 = 15.50 kg` | `simulated` |
| B-33 mass delta (`F-16C` `M61A1`, 500 rounds expended) | **77.50 kg** | `500 × 0.155 = 77.50 kg` | `simulated` |
| Live A2A gun tail-chase (120 rds at 220 m) | **8 hits / 8 native damage events**, bit-identical across seeded runs | Deterministic `VehicleDamage` + `reason="gun"` | `simulated` |
| B-37 `F-35A` nose RCS (clean vs. `internal air-to-air` bay closed) | `1.63004 m²` vs. `1.63004 m²` | Bit-identical stealth preservation | `simulated` |
| B-37 `F-16C` nose RCS (clean / `mk82 x4` / `external tanks`) | `2.59113` / `2.59144` / `4.14665 m²` | Strictly increasing with external carriage | `simulated` |
| B-37 `EA-18G` nose RCS (clean / `ALQ-99` jamming pods) | `3.52875` / `5.22604 m²` | Strictly increasing with pods | `simulated` |
| B-50 `Mk 82` calm drop (`h0 = 2994.2 m`) TOF | `24.44 s` vs. RK4 `24.34 s` (**0.42% err**) | Independent standard-atmosphere RK4 ODE | `simulated` |
| B-50 `Mk 82` calm drop downrange East | `5089.17 m` vs. RK4 `5058.35 m` (**0.61% err**) | Independent standard-atmosphere RK4 ODE | `simulated` |
| B-50 `Mk 82` calm drop impact speed | `302.52 m/s` vs. RK4 `300.53 m/s` (**0.66% err**) | Independent standard-atmosphere RK4 ODE | `simulated` |
| B-50 `Mk 82` 20 m/s northward crosswind drift | **+42.06 m** north vs. calm | Physical wind drift sensitivity | `simulated` |

### Dual-Preset Test Matrix

| Suite | `ucrt64-headless` | `ucrt64-release` |
| --- | --- | --- |
| `sim_m10_guns_rcs_ballistics` | PASS (5/5, 244 assertions, `12.52 s`) | PASS (5/5, 244 assertions, `12.30 s`) |
| `oms_adapter` | PASS (`6.17 s`) | PASS (`5.99 s`) |
| `ms11_families` | PASS (`7.63 s`) | PASS (`7.60 s`) |
| `optical_ir` | PASS (`5.92 s`) | PASS (`5.75 s`) |
| `esm_ea_comint` | PASS (`28.67 s`) | PASS (`28.05 s`) |
| `launch_zone` | PASS (`29.07 s`) | PASS (`28.31 s`) |
| `ma_equipment` / `ma_equipment_c` | PASS (`3.19 s` / `1.94 s`) | PASS (`3.20 s` / `1.99 s`) |
| `c_abi` | PASS (`30.03 s`) | PASS (`29.35 s`) |
| `sdk_boundary` (2,055 entry points, 0 errors) | PASS (`2.42 s`) | PASS (`2.90 s`) |
| `fsim_tests "[ms]"` (95/95 cases, 2,131,065 assertions; 3 legacy gun-NotImplemented assertions updated) | PASS | PASS |
| Python `test_ma_equipment.py` (9/9 incl. `Agent.fire_gun` + B-33) | PASS (`10.32 s`) | PASS (`10.25 s`) |

## 3. Honest Boundaries & Open Follow-On Items

- **Empirical ballistic calibration (B-50)**: The B-50 verification validates the live 6-DOF JSBSim `mk82` drop against an independent RK4 standard-atmosphere point-mass ODE using the store's own VLM-derived drag area (`0.42%–0.66%` agreement) plus crosswind drift sensitivity. Comparison against a matched published military range table (e.g., JMEM / USAF bombing tables with exact release attitude and wind profile) remains an empirical calibration item when such a dataset is curated.
- **Per-store high-resolution `rcs.bin` tables (B-37)**: Runtime loaded-store RCS uses exact `rcs.bin` tables where built (`mk82`) and physical-optics cylinder/ogive estimates (`StoreRcs`) for all other external stores; precomputing full `flightsim-rcs` meshes for every external tank/pod/missile remains an optional offline asset enhancement.
- **Fullscreen visual acceptance**: Fullscreen primary-display Vulkan Viewer verification remains skipped/pending in this isolated-desktop environment (`WinSta0\exebox-*`).
