# SIM-M04 (MS-8): Flyout-Derived DLZ and WEZ

SIM-M04 replaces the assumed heuristic air-to-air launch-zone calculation with deterministic 5-D dynamic launch zone (DLZ) and aspect-polar weapon engagement zone (WEZ) envelopes derived offline from native 6-DOF JSBSim missile flyouts, with propagated 1-sigma uncertainty intervals, cryptographic provenance, and honest published-performance comparisons.

## Method

`tools/models/dlz/dlz_generator.cpp` (`fsim-dlz-generator`) generates one immutable envelope table `aircraft/stores/<model>/<model>.dlz-v1.json` (schema `fsim.ms.dlz.v1`) for each of the 24 maintained air-to-air missiles:

1. **Native 6-DOF JSBSim reach trajectories (`36` launch conditions per missile)**:
   For every `(launch_altitude_m, launch_mach, target_altitude_delta_m)` knot across
   - `launch_altitude_m`: `[1000, 6000, 12000, 18000]` m
   - `launch_mach`: `[0.6, 0.9, 1.4]`
   - `target_altitude_delta_m`: `[-4000, 0, 4000]` m
   the generator steps the missile's actual JSBSim flight model (`dt = 1/120 s` with each store XML's authored `fsim/integration-v1/substeps`, `frameSkip = 4`), native `ms::weapon::Guidance` (preserving authentic `acquireM`, midcourse loft, proportional navigation, `maxG`, `armingS`, and `lifeS`), native fin autopilot (`actuatorAwareCommands` / `finCommands`), thrust-vector control where authored, and `PulseManagerV1` second-pulse ignition for dual-pulse motors (`pl-15`, `r-37m`) as well as `meteor`'s ducted-rocket V2 propulsion.
2. **Measured off-boresight attenuation (`4` flyouts per missile)**:
   Native 6-DOF flyouts at `0..60 deg` off-boresight (clamped to seeker gimbal limit) measure the missile's turn-induced energy loss curve (`off_boresight_deg`, `off_boresight_factor`).
3. **5-D grid intercept and 7 g no-escape evaluation (`540` knots per missile)**:
   Across `target_mach = [0.7, 1.0, 1.4]` and `aspect_deg = [0, 45, 90, 135, 180]` (`0 deg` = head-on, `90 deg` = beam, `180 deg` = tail chase), each knot computes:
   - `r_max_m`, `tof_max_s`: maximum kinematic intercept range and time-of-flight against a non-maneuvering constant-velocity target (`age >= arming_s`, terminal speed `>= 120 m/s`), applying the missile's own measured off-boresight factor for crossing lead angles.
   - `r_ne_m`, `tof_ne_s`: no-escape range against a target executing an immediate sustained `7 g` level break turn to `180 deg` tail-away escape at launch.
   - `r_min_m`, `tof_min_s`: minimum armed/settled engagement range.
4. **Propagated 1-sigma uncertainty (`8` perturbed 6-DOF flyouts per missile)**:
   Four representative altitude/Mach anchors are re-flown under lower (`-10%` thrust, `+15%` drag, `-5%` battery life) and upper (`+10%` thrust, `-15%` drag, `+5%` battery life) perturbations via `JsbsimModel::setExternalForceBody`. The RMS half-spread across `0/90/180 deg` aspects produces `uncertainty_fraction.{r_max, r_ne, r_min, tof}` (bounded in `[0.05, 0.50]`).

Every table records `basis: "simulated"`, `equipment_calibrated: false`, and full 64-hex SHA-256 checksums (`model_xml_sha256`, `card_sha256`, plus runtime-computed `table_sha256`). **No aerodynamic drag, lift, or motor impulse parameters are tuned or fit to published figures.**

## Runtime Evaluation and Public Interfaces

Runtime evaluation (`src/simulation/mission_systems/weapon/LaunchZone.{h,cpp}`) resolves `<aircraftDir>/<model>/<model>.dlz-v1.json` through `io::AssetResolver`, validates schema/monotonicity once, caches the immutable table per `World`, and evaluates 5-D multilinear interpolation in real time (zero JSBSim calls at query time). When no `.dlz-v1.json` table is present, an explicit legacy fallback (`flyoutDerived = false`, `basis = "assumed"`) is returned.

| Layer | DLZ V2 | WEZ V1 | Legacy DLZ |
| --- | --- | --- | --- |
| C++ `World` | `World::launchZoneV2(vehicle, store, target)` | `World::launchZoneWezV1(vehicle, store, target)` | `World::launchZone(...)` (delegates to V2) |
| C++ `Agent` | `Agent::launchZoneV2(store, designation)` | `Agent::launchZoneWezV1(store, designation)` | `Agent::launchZone(...)` |
| C99 ABI | `fsim_world_launch_zone_v2` (`fsim_launch_zone_v2_init`) | `fsim_world_launch_zone_wez_v1` (`fsim_launch_zone_wez_v1_init`) | `fsim_world_launch_zone` |
| Python `World` | `world.launch_zone_v2(vehicle, store, target)` | `world.launch_zone_wez(vehicle, store, target)` | `world.launch_zone(...)` |
| Python `Agent` | `agent.launch_zone_v2(store, designation)` | `agent.launch_zone_wez(store, designation)` | `agent.launch_zone(...)` |

### Key Output Semantics

- `maximumM` (`R_max` / `R_aero`): boresight aerodynamic maximum intercept range at the target's current aspect and kinematics.
- `probabilityInterceptM` (`R_pi`): `R_max` attenuated by the current off-boresight angle factor (`off_boresight_factor`).
- `noEscapeM` (`R_ne`): `7 g` immediate turn-and-run escape range attenuated by off-boresight angle (`0 <= R_min <= R_ne <= R_pi <= R_max`).
- `timeOfFlightS`, `fPoleM`, `aPoleM`: predicted missile time of flight, launcher-to-target separation at intercept (`F-pole`), and launcher-to-target separation at active-radar pitbull (`A-pole`, NaN for non-ARH missiles or out-of-range queries).
- `maximumUncertaintyM`, `noEscapeUncertaintyM`, `minimumUncertaintyM`, `timeOfFlightUncertaintyS`: propagated 1-sigma bounds.
- `inEnvelope`: `true` when all 5 kinematic inputs lie inside the generated grid axes; out-of-envelope queries clamp smoothly and set `inEnvelope = false`.
- **Permission boundary**: Agent handles (`Agent::launchZoneV2`, `Agent::launchZoneWezV1`, C99 on Agent handles, Python `agent.launch_zone_v2` / `agent.launch_zone_wez`) accept only live own-platform designations, rejecting raw entity IDs, foreign designations, and lapsed designations. Query results remain advisory and never grant release authority.

## Packaging and Published-Performance Comparison

- `cmake/StageDesigns.cmake` copies `<model>.dlz-v1.json` alongside `<model>.xml` into build and installed `share/flightsim/aircraft/stores/<model>/` trees so installed C++, C99, and Python consumers resolve the flyout tables without source-tree fallback.
- Full 24-AAM published-performance comparison tables, residuals, and SHA-256 provenance are recorded in [dlz-published-comparison.md](../research/sim-m04/dlz-published-comparison.md) and [dlz-published-comparison.json](../research/sim-m04/dlz-published-comparison.json).
- Public-literature and open-source methodology alignment (JSBSim, Zarchan, Fleeman, ICAO ISA, BVR F-pole/A-pole closure) is documented in [open-source-alignment.md](../research/sim-m04/open-source-alignment.md).
- Notable un-tuned physical findings:
  - `aim-120c` produces `99.0 km` high-altitude head-on `R_max` (`+10.0%` vs published `90 ± 15 km`) with `R_ne = 43.8 km` (`44.2%`).
  - `meteor`'s ducted-rocket V2 sustained thrust naturally yields the highest no-escape ratio (`R_ne = 73.2 km` / `R_max = 128.3 km` = `57.1%`).
  - Short-range IR missiles (`r-60m`, `pl-10`, `magic-2`, `pl-8`) exceed their low-altitude/seeker-limited published ranges at `12 km / M 1.2` because the uncalibrated cards carry a uniform `45 s` battery life and high-altitude drag is low, while ultra-long-range missiles (`r-37m`, `pl-15`) are bounded by their authored `110 s` battery life and assumed propellant impulse. All residuals are preserved honestly without parameter fitting.
