# SIM-M04 (MS-8): Public-Literature and Open-Source Alignment

This note records how SIM-M04's flyout-derived 5-D Dynamic Launch Zone (DLZ) and aspect-polar Weapon Engagement Zone (WEZ) architecture aligns with public aerospace guidance literature and mature open-source flight-simulation practice, and documents the exact physical basis and honest residuals across all 24 maintained air-to-air missiles.

## 1. Reference Baseline and Open-Source Lineage

| Component | Authoritative Public / Open-Source Reference | Alignment in `Super-DDCCSS-WWorld` |
| --- | --- | --- |
| 6-DOF Missile Flight Dynamics | **JSBSim** (open-source 6-DOF flight dynamics engine, `JSBSim-Team/jsbsim`); **Fleeman**, *Tactical Missile Design* (AIAA, 2nd ed.) | Offline table generation (`fsim-dlz-generator`) flies each missile's actual JSBSim 6-DOF model (`dt = 1/120 s` + authored integration-v1 substeps), native fin autopilot, TVC where authored, `PulseManagerV1` dual-pulse motors, and ducted-rocket V2 ramjet propulsion. No separate point-mass surrogate is substituted for the flyout. |
| Midcourse Loft & Terminal Guidance | **Zarchan**, *Tactical and Strategic Missile Guidance* (AIAA, 6th ed.); **Shneydor**, *Missile Guidance and Pursuit* | Uses the native `ms::weapon::Guidance` law (midcourse loft, proportional navigation, `acquireM`, `maxG`, `armingS`, `lifeS`), guaranteeing consistency between advisory DLZ queries and real-time `World` launches. |
| Standard Atmosphere | **ICAO Doc 7488 / U.S. Standard Atmosphere (1976)** | `isaSoundSpeedMs(h)` implements the troposphere lapse rate (`288.15 - 0.0065 h` K) and tropopause isotherm (`216.65 K` above `11 km`), matching ICAO ISA within `0.05 m/s` from `0` to `20 km` (verified in `tests/simulation/test_launch_zone.cpp`). |
| Surrogate Table / 5-D Multilinear Lookup | **AFSIM / WSF** weapon launch-computer tables; open-literature BVR DLZ/WEZ surrogate-grid methods (e.g., *AIAA Journal of Guidance, Control, and Dynamics* DLZ/LAR lookup architectures) | Pre-computes a 5-D grid `(launch_alt, launch_mach, target_dh, target_mach, aspect)` = `540` knots per missile + measured `off_boresight_factor` curve, evaluated in $O(1)$ via 5-D multilinear interpolation at runtime with zero JSBSim calls. |
| Tactical Engagement Metrics | **F-pole, A-pole, $R_{\text{aero}}$, $R_{\text{pi}}$, $R_{\text{ne}}$, $R_{\min}$** (public BVR tactical flight-test definitions; Zarchan kinematic closure) | `evaluateLaunchZoneV2` derives `F-pole` as launcher-to-target separation at intercept (`t_tof`), `A-pole` as separation at active-radar seeker pitbull (`t_act = t_tof * (1 - R_active / R_slant)`, `NaN` for SARH/IR missiles), and `R_ne` against an immediate sustained `7 g` level break turn to `180 deg` tail-away escape. |

## 2. Analytical Verification Against Literature Equations

`tests/simulation/test_launch_zone.cpp` (`[sim-m04][dlz][alignment]`) cross-validates the runtime DLZ evaluator against closed-form reference physics:

1. **ICAO / U.S. Standard Atmosphere 1976**: `isaSoundSpeedMs` verified at `0, 5, 11, 15, 20 km` within `0.05 m/s`.
2. **Zarchan / BVR F-pole and A-pole Kinematic Closure**:
   - Head-on F-pole matches $|R_0 - V_c \cdot t_{\text{TOF}}|$ within `5 m`.
   - Active-radar A-pole matches $|R_0 - V_c \cdot t_{\text{TOF}}(1 - R_{\text{act}}/R_0)|$ within `5 m` and satisfies $R_{\text{A-pole}} > R_{\text{F-pole}}$.
   - Passive IR missiles (`aim-9x`) explicitly output `NaN` for `aPoleM` (no active pitbull phase).
3. **Propulsion-Class Scaling**:
   - Ducted-ramjet sustained thrust (`meteor`: `R_ne / R_max = 62.4%`) and dual-pulse BVR boost-sustain (`pl-15`: `57.9%`) strictly exceed single-motor solid-rocket boost-coast (`aim-120c`: `48.1%`, `aim-120d`: `52.1%`) at identical `12 km / M 1.2` head-on conditions.
   - Reduced atmospheric density at high altitude strictly increases `R_max` over sea-level launches across all 24 AAMs.
4. **Native 6-DOF Closed-Loop Consistency**:
   - A live `World` 6-DOF JSBSim `aim-120c` launch inside `R_ne` (`22 km` head-on, `10 km MSL`) detonates within `25 m` miss distance, confirming runtime alignment between the offline `.dlz-v1.json` envelope and active flight/guidance execution.

## 3. Parameter Provenance and Honest Residuals

Every `.dlz-v1.json` table explicitly sets `basis: "simulated"` and `equipment_calibrated: false`, recording full 64-hex SHA-256 digests of the store XML, store card, and serialized table. Full 24-missile figures live in [dlz-published-comparison.md](dlz-published-comparison.md) and [dlz-published-comparison.json](dlz-published-comparison.json).

Following the 2026-11-28 24-AAM aerodynamic, propulsion, and seeker engineering calibration (`basis: "assumed"` / `"derived"` / `"simulated"`, nonzero screening uncertainties, zero OEM hardware calibration claims), all 24 missiles meet their published peak-Mach bands (`0` outliers) and span physically consistent high-altitude kinematic reach without uniform `45 s / 110 s` battery-life truncation. Two physically transparent residual patterns remain:

1. **Medium- and Long-Range BVR AAMs (`r-77m`, `aim-120d`, `r-37m`, `pl-15`, `aim-120c`, `meteor`, `r-27er`, `r-27et`)**: High-altitude (`12 km / M 1.2`) un-evaded head-on kinematic flyouts span `+5.2%` (`r-77m` `199.9 km` vs `190 km`) to `+65.0%` (`meteor` `329.9 km` vs `200 km`), while immediate `7 g` break-turn `R_ne` sits in the realistic tactical band (`aim-120c` `66.0 km`, `aim-120d` `100.4 km`, `pl-15` `149.0 km`, `meteor` `205.8 km`, `r-37m` `318.8 km`). Positive `R_max` residuals reflect optimal high-altitude lofted aerodynamic glide to `M_min` against a non-maneuvering head-on target versus conservative mixed-altitude tactical brochures.
2. **Short- and Medium-Range IR / SARH AAMs (`r-60m`, `pl-10`, `pl-8`, `asraam`, `iris-t`, `r-73`, `aim-9m`, `aim-9x`, `r-27t`, `super-530d`)**: High-altitude (`12 km / M 1.2`) kinematic glide still exceeds published tactical ranges (`+50.8%` to `+217.6%`, e.g., `r-60m` `25.4 km` vs `8 km`, `aim-9m` `52.8 km` vs `35 km`, `r-73` `60.2 km` vs `30 km`) because published WVR figures are typically quoted at low-to-medium altitude and are bounded in live engagements by passive IR seeker acquisition range (`acquire_m`) rather than pure ballistic/lofted glide.

These residuals remain intentionally documented rather than concealed by curve fitting; any future store-card propulsion or battery-life refinement will automatically update the `.dlz-v1.json` tables and SHA-256 checksums via `fsim-dlz-generator`.
