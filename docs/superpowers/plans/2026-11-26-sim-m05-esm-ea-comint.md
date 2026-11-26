# SIM-M05 (MS-9b) Implementation Plan: ESM, COMINT, Electronic Attack & Countermeasures Interaction

- **Date**: 2026-11-26
- **Branch**: `main`
- **Base HEAD**: `88bc459` (after SIM-M03 `3883b58`, `cj-10a` hash sync `3c326e4`, and SIM-M04 `88bc459`)
- **Scope**: Complete vertical delivery of MS-9b across native C++, C99 ABI, Python SDK, payload cards, SDK boundary classification, tests, and documentation.

## 1. Frozen Requirements & Public-Data Honesty

1. **Electronic Attack (`Family::ElectronicAttack`)**:
   - Native RF noise jammer model (`src/simulation/mission_systems/ea/Jammer.{h,cpp}`) implementing `ms::Model` for all RF jammer payload cards (`an-alq-99`, `an-alq-135`, `an-alq-165`, `an-alq-172`, `an-alq-117`, `an-alq-122`, `an-alq-155`, `an-alq-213`, `an-alt-16`, `an-alt-28`, `an-alt-32`, `compass-call`, `l402-himalayas`, `sabre`), while distinguishing optical DIRCM (`an-aaq-24`) explicitly rather than fabricating RF jamming on an IR laser payload.
   - Supports preemptive sector/omni jamming (`contact == 0`) and reactive ESM contact-following spot/barrage jamming (`contact != 0`, ending with `Reason::ContactLost` when the followed contact drops), with finite optional `durationS`, frequency/band validation, and `EmitterMode::Jammer` emission publication.
   - Numeric transmitter powers, gains, and bands without manufacturer-published figures carry explicit `basis: assumed` provenance and wide uncertainties; no fabricated equipment calibration is claimed.
2. **Physical Jammer-to-Radar & Jammer-to-Comms Interaction**:
   - Additive `radar::Scene::jammingNoiseRatio(...)` default virtual method (returning `0.0` when unjammed, preserving bit-for-bit regression parity on all existing radar tests).
   - Computes received jammer spectral power density $J_0 = \sum_j \frac{P_j G_j G_r}{B_j L_{\text{path}}(j, f)} \cdot \text{overlap}(f_r, B_r; f_j, B_j)$ using each radar's own real `rf::Pattern` (mainlobe vs sidelobe) and P.528/P.526 terrain-aware `pathLossDb`, degrading both detection-level (`Radar::dwell`) and signal-level (`Radar::dwellSignal`) SNR by $10\log_{10}(1 + J_0 / N_0)$ dB — yielding real burn-through ranges, degraded $P_d$, inflated measurement noise, and track loss.
   - Comms/DMS RF jamming: active in-band `EmitterMode::Jammer` emissions degrade modeled `Dms` link closure when received $J/S$ exceeds the net's jam margin, dropping packets without leaking truth; out-of-band or terrain-masked jammers have zero effect.
3. **Enhanced ESM/RWR, SNR-Dependent DF, Bearing-Only Geolocation & COMINT**:
   - Preserves existing ESM/RWR warning and contact behavior.
   - Adds SNR-dependent bearing standard deviation (down to `bearing_floor_deg`), moving-platform bearing-only least-squares triangulation (`latitudeRad`, `longitudeRad`, `heightM`, `rangeM` populated only once baseline parallax and conditioning thresholds are met; `NaN` otherwise — never truth substitution), and `EmitterMode::Jammer` detection.
   - Adds modeled RF carrier `EmitterMode::Datalink` emissions from active `Dms` transmissions so wideband/COMINT ESM suites (`rivet-joint`, `an-alq-218`, `e3-esm`, `e7-esm`, etc.) intercept communications emitter bearings/frequencies/strengths without ever exposing decoded DMS message contents.
4. **Programmed Countermeasures Salvo & Public `PayloadTask` / `PayloadReport` Interfaces**:
   - Wires `Host::submit(const PayloadTask&, ...)` for `TaskKind::Jam` and `TaskKind::Dispense` (programmed chaff/flare salvos respecting native dispenser inventory and cooldown).
   - Additive public C++ (`Vehicle::submit(PayloadTask)`, `Agent::submit(PayloadTask)`, `Vehicle::payloadReport`, `Agent::payloadReport`), C99 (`fsim_payload_task`, `fsim_vehicle_payload_task`, `fsim_payload_value`, `fsim_vehicle_payload_report`), and Python (`PayloadTask`, `PayloadValue`, `submit_task`, `jam`, `payload_report`) interfaces preserving all existing ABI, struct layouts, enum values, and Agent permission boundaries.

## 2. Path & Symbol Allowlist

- `src/simulation/mission_systems/ea/Jammer.{h,cpp}` (new)
- `src/simulation/mission_systems/esm/Receiver.cpp`
- `src/simulation/mission_systems/Model.cpp`
- `src/simulation/mission_systems/Host.{h,cpp}`
- `src/simulation/mission_systems/radar/Radar.{h,cpp}`
- `src/simulation/mission_systems/CMakeLists.txt`
- `src/simulation/communications/dms/Dms.{h,cpp}`
- `src/simulation/runtime/world/World.{h,cpp}`
- `src/simulation/runtime/world/WorldEngagements.cpp`
- `src/simulation/api/{World.cpp,Agent.cpp}`
- `include/fsim/{World.h,Agent.h,fsim_c.h}`
- `src/adapters/sdk/c/fsim_mission_c.cpp`
- `src/adapters/sdk/python/{_native_world.h,_native.c,_native_mission.c}`
- `python/fsim/{__init__.py,world.py,agent.py}`
- `assets/payloads/*.json` (ESM/EA cards where band/role parameters distinguish wideband COMINT/ELINT, standoff jammers, and DIRCM)
- `docs/sdk/{boundary.toml,boundary.md,mission-systems.md,sim-m05-esm-ea-comint.md}`, `docs/status.md`
- `tests/simulation/test_esm_ea_comint.cpp` (new), `tests/CMakeLists.txt`, `tests/training/test_ma_equipment_c.c`, `tests/compatibility/python/test_engagement.py`

## 3. Verification Gates (Both `ucrt64-headless` and `ucrt64-release`)

- Zero compiler warnings (`-Werror`) across full headless and release builds.
- `esm_ea_comint` unit + native World tests: preemptive & reactive contact-following jamming, mainlobe vs sidelobe vs out-of-band vs terrain-masked radar jamming, burn-through recovery, signal-level radar jamming, DMS comms jamming, COMINT datalink intercept without payload leakage, moving-platform bearing-only geolocation convergence & single-look `NaN` check, programmed countermeasure salvo depletion, Agent authority & refusal boundaries, worker-count determinism.
- `ma_equipment`, `ma_equipment_c` (including sized C99 `fsim_payload_task` & `fsim_payload_value`), `c_abi`, `sdk_boundary`, existing radar/ESM/DMS regressions, Python `test_engagement.py` & `test_ma_equipment.py`, and installed `equipment_package_consumer`.
