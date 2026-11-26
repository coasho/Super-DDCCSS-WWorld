# SIM-M05 (MS-9b): Electronic Attack, Enhanced ESM/DF Geolocation, COMINT, and Programmed Countermeasures

SIM-M05 delivers native electronic attack (RF noise jamming against radars and datalinks), SNR-dependent direction-finding and moving-platform passive triangulation for capable ESM/SIGINT suites, modeled COMINT datalink interception without message-payload leakage, and programmed countermeasure salvos across C++, C99 ABI, and Python.

## Native Physics and Subsystem Models

1. **Electronic Attack RF Noise Jammer (`src/simulation/mission_systems/ea/Jammer.{h,cpp}`)**:
   - Models `Family::ElectronicAttack` RF jamming pods and internal suites (`an-alq-99`, `compass-call`, `an-alq-135`, `an-alq-165`, `an-alq-172`, `l402-himalayas`, etc.) with multi-pod power scaling (`count`).
   - Optical/IR directed countermeasures (`an-aaq-24` DIRCM) are explicitly excluded (`makeJammer` returns `nullptr`, reporting `NotImplemented`) rather than fabricating RF noise emissions.
   - Supports:
     - **Preemptive spot/barrage jamming** (`contact == 0`): directional sector or full-azimuth omnidirectional (`azimuthHalfWidthRad >= pi`), explicit centre frequency/bandwidth or full-band barrage (`frequencyHz = NaN`), and optional finite `durationS` (`GoalReached` completion).
     - **Reactive ESM contact-following** (`contact != 0`): tracks an own-platform ESM contact's measured bearing and frequency each tick, automatically ending with `ActivityState::Failed` / `Reason::ContactLost` when the contact expires or leaves the jammer's RF band.
   - Publishes continuous `EmitterMode::Jammer` emissions into `World::emissions_` and exposes telemetry via `payloadReport` (`active`, `power`, `frequency`, `bandwidth`, `erp`, `band_min`, `band_max`).
   - All unpublished jammer transmitter parameters (`power_w`, `peak_gain_dbi`, `sidelobe_dbi`, `band_min/max`, `spot_bandwidth_hz`) are explicitly `basis: assumed` engineering class defaults overridden whenever a payload card declares measured values.

2. **Radar Noise-Jamming Integration (`radar::Scene::jammingDensityWHz`, `Radar::dwell`, `Radar::dwellSignal`)**:
   - External in-band noise jammers raise effective receiver noise power spectral density from $N_0 = k T_0 F$ to $N_0 + J_0$, degrading both detection-level and signal-level radar SNR by $10\log_{10}(1 + J_0 / N_0)\text{ dB}$, where
     $$J_0 = \sum_i \frac{P_i G_{t,i} G_{r,i}}{L_i B_{j,i}} \cdot \text{overlap}(f_i, B_{j,i}; f_r, B_r).$$
   - Victim receive gain $G_{r,i}$ evaluates the radar's actual electronic (single- or multi-face) or mechanical beam pattern along the jammer's line of sight; out-of-band jammers have `overlap == 0` and cause zero degradation, preserving all unjammed radar behavior bit-for-bit. Short-range burn-through emerges naturally from $R^{-4}$ radar return vs $R^{-2}$ one-way jammer scaling.

3. **Enhanced ESM/DF, Moving-Platform Geolocation, and COMINT (`src/simulation/mission_systems/esm/Receiver.cpp`)**:
   - Ordinary amplitude-comparison RWRs (`an-alr-56m`, etc.) preserve legacy constant `10 deg` bearing uncertainty (`bearing_floor_deg = 10 deg`) bit-for-bit while adding `EmitterMode::Jammer` strobe detection.
   - Dedicated SIGINT/ELINT and interferometric fighter EW suites (`rivet-joint`, `an-alq-218`, `e3-esm`, `e7-esm`, `an-alr-94`, `an-asq-239`, `spectra`, `praetorian`, `gripen-ews`) tighten bearing uncertainty with excess SNR down to `bearing_floor_deg` (`0.5–1.0 deg`, `basis: assumed`).
   - **Moving-platform 3-D bearing-only triangulation (`triangulateRays`)**: requires at least 4 separated looks (`>= 400 m` spacing), `>= 2 km` baseline, `>= 4 deg` angular parallax, positive matrix conditioning, and a forward-hemisphere fix in `[500 m, 600 km]`; otherwise `rangeM`, `latitudeRad`, `longitudeRad`, and `heightM` remain `NaN` (no fabricated single-look range).
   - **COMINT (`EmitterMode::Datalink`)**: `Dms::activeCarriers` publishes RF carrier metadata (`entity`, `frequencyHz`, `bandwidthHz`, `powerW`, never message bytes or topics) for active datalink transmitters. COMINT-capable receivers (`rivet-joint`, `an-alq-218`, `e3-esm`, `e7-esm`) intercept those emissions as `EmitterMode::Datalink` contacts without exposing DMS payloads.

4. **Communications Jamming (`Dms::Locator::jammed` & `World::stepDms`)**:
   - Active `EmitterMode::Jammer` emissions overlapping a DMS net's RF carrier degrade link closure at the receiver. When $J/S$ exceeds the assumed tactical link jam margin (`10 dB`, `basis: assumed`) over the terrain-checked radio path, packets are dropped and recorded in `NetStats::lost`. Out-of-band jammers and unjammed worlds are unaffected.

5. **Programmed Countermeasure Salvos (`TaskKind::Dispense`)**:
   - `Host::submit(PayloadTask)` schedules `TaskKind::Dispense` salvos (`chaff`, `flares`, `salvos`, `intervalS`, optional `durationS`) on operating `Family::Countermeasures` payloads, routing every cartridge through `World::dispense` so native dispenser inventory, `0.35 s` shared cooldown, decoys, and engagement events remain single-authority.

## Public Interfaces

| Layer | Submit Payload Task | Payload Report | ESM Contacts |
| --- | --- | --- | --- |
| C++ `Vehicle` | `Vehicle::submit(const ms::PayloadTask&, opts)` | `Vehicle::payloadReport(payload)` | `Vehicle::contacts(payload)` |
| C++ `Agent` | `Agent::submit(const ms::PayloadTask&, opts)` | `Agent::payloadReport(payload)` | `Agent::contacts(payload)` (`truth == 0`) |
| C99 ABI | `fsim_vehicle_payload_task` (`fsim_payload_task_init`, `fsim_task_kind_name`) | `fsim_vehicle_payload_report` (`fsim_payload_value_init`) | `fsim_vehicle_contacts` |
| Python `Vehicle` / `Agent` | `submit_task(...)`, `jam(...)`, `dispense_salvo(...)` (`fsim.TaskKind`) | `payload_report(payload)` (`fsim.PayloadValue`) | `contacts(payload)` (`fsim.EmitterMode`) |

All Agent entry points enforce own-platform scope (`FSIM_NOT_ALLOWED` on foreign vehicles) and redact `Contact::truth` to `0`. All 2,045 exported C++, C, and Python entry points are classified in [boundary.toml](boundary.toml) and verified by `sdk_boundary` ([boundary.md](boundary.md)).

## Verification

Verified with zero compiler warnings (`-Werror`) on both `ucrt64-headless` and `ucrt64-release`:

- `esm_ea_comint` (6 test cases / 166 assertions): EA unit math & DIRCM refusal, mainlobe radar jamming degradation & out-of-band immunity & burn-through & signal-level jamming, reactive ESM contact-following & `ContactLost`, COMINT datalink intercept & Compass Call comms jamming, moving-platform bearing-only triangulation geolocation, programmed countermeasure salvos & Agent boundaries.
- Legacy ESM regression (`fsim_tests "[esm]"`): 3/3 cases (83/83 assertions) on both presets.
- Cross-language & installed consumer gates: `ma_equipment`, `ma_equipment_c` (C99 `fsim_vehicle_payload_task` & `fsim_vehicle_payload_report`), `c_abi`, `sdk_boundary` (2,045 entry points, 0 errors on both presets), Python `test_engagement.py` + `test_ma_equipment.py` (15/15 on both presets), and installed `equipment_package_consumer` on both presets.
