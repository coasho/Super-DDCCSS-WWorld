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
   - Ducted-ramjet sustained thrust (`meteor`: `R_ne / R_max = 57.1%`) strictly exceeds solid-rocket boost-coast (`aim-120c`: `44.2%`, `aim-120d`: `48.0%`) by `> 5%` at identical `12 km / M 1.2` head-on conditions.
   - Reduced atmospheric density at high altitude strictly increases `R_max` over sea-level launches across all 24 AAMs.
4. **Native 6-DOF Closed-Loop Consistency**:
   - A live `World` 6-DOF JSBSim `aim-120c` launch inside `R_ne` (`22 km` head-on, `10 km MSL`) detonates within `25 m` miss distance, confirming runtime alignment between the offline `.dlz-v1.json` envelope and active flight/guidance execution.

## 3. Parameter Provenance and Honest Residuals

Every `.dlz-v1.json` table explicitly sets `basis: "simulated"` and `equipment_calibrated: false`, recording full 64-hex SHA-256 digests of the store XML, store card, and serialized table. Full 24-missile figures live in [dlz-published-comparison.md](dlz-published-comparison.md) and [dlz-published-comparison.json](dlz-published-comparison.json).

No aerodynamic drag, lift, or rocket-motor impulse parameters were tuned to force published numbers. Three physically transparent residual patterns emerge from the current uncalibrated store cards:

1. **Medium-Range Solid-Rocket BVR AAMs (`aim-120c`, `pl-12`, `mica-em`, `r-27r`)**: Align closely with public open-source ranges (`aim-120c` `99.0 km` vs `90 ± 15 km`, `+10.0%`; `pl-12` `+22.2%`; `mica-em` `+24.9%`).
2. **Short-Range IR AAMs (`r-60m`, `pl-10`, `magic-2`, `pl-8`, `asraam`, `iris-t`)**: Kinematic flyouts at `12 km / M 1.2` exceed published tactical ranges (`+44%` to `+465%`) because:
   - Published WVR ranges are often quoted at medium/low altitude and bounded by passive IR seeker lock-on range rather than pure kinematic glide, and
   - Several uncalibrated IR store cards currently share a uniform `45 s` battery life (`lifeS`) rather than missile-specific short thermal-battery durations.
3. **Ultra-Long-Range / Ramjet / Dual-Pulse AAMs (`r-37m`, `pl-15`, `meteor`, `r-77m`)**: High-altitude head-on flyouts fall `-36%` to `-75%` below upper-bound public brochures because authored card `lifeS` (`110 s` for `r-37m`/`pl-15`) and assumed solid/ramjet propellant total impulse cap maximum lofted glide duration before reaching `200–350 km`.

These residuals remain intentionally documented rather than concealed by curve fitting; any future store-card propulsion or battery-life refinement will automatically update the `.dlz-v1.json` tables and SHA-256 checksums via `fsim-dlz-generator`.
