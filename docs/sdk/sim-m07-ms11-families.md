# SIM-M07 (MS-11): Weather Radar, SAR, SMTI, PNT, Comm Terminals / Mode 5 IFF, and Ground-Map Radar

SIM-M07 completes the remaining Mission Systems payload families (`WeatherRadar`, `Sar`, `Smti`, `Pnt`, `CommTerminal`, and `GroundMapRadar`) across all **18** maintained MS-11 payload cards through C++, C99 ABI, and Python without reimplementing native DMS/C2 delivery or breaking any existing ABI.

## Native Physics and Subsystem Models

1. **Airborne Weather Radar (`src/simulation/mission_systems/weather/WeatherRadar.{h,cpp}`)**:
   - Covers `an-apn-241` (`c130j`), `an-aps-133` (`c17a`), and `kc135r-radar` (`kc135r`).
   - Computes Marshall-Palmer (1948) equivalent reflectivity ($\text{dBZ} = 10\log_{10}(200 R^{1.6})$), ITU-R P.838-3 X-band two-way rain attenuation, and Probert-Jones beam-filled minimum detectable reflectivity under `TaskKind::Search`, emitting `ContactKind::Weather` contacts and X-band `EmitterMode::Search` emissions.
2. **Synthetic Aperture Radar (`src/simulation/mission_systems/sar/SarRadar.{h,cpp}`)**:
   - Covers `asars-2` (`u2s`) and `eiss-sar` (`rq4b`).
   - Implements Curlander & McDonough coherent integration time $T_{\text{int}} = \frac{\lambda R}{2 v \rho_a \sin\psi}$ (enforcing side-looking squint geometry with `Reason::OutOfReach` for dead-ahead targets) and Noise Equivalent Sigma Zero ($\sigma^0_{\text{NE}}$, degraded by in-band RF jamming $10\log_{10}(1 + J_0 / N_0)$).
   - Commanded via `TaskKind::Image`; emits `ContactKind::Image` upon completion (`ActivityState::Completed / Reason::GoalReached`).
3. **Surface Moving-Target Indication (`src/simulation/mission_systems/smti/SmtiRadar.{h,cpp}`)**:
   - Implements Skolnik/Richards endo-clutter Minimum Detectable Velocity ($\text{MDV} = \frac{\lambda}{2}\Delta f_{\text{clutter}}$) and `ContactKind::Surface` reporting while honoring the repository's no-ground-entities boundary.
4. **Positioning, Navigation & Timing (`src/simulation/mission_systems/pnt/PntReceiver.{h,cpp}`)**:
   - Covers `an-asn-109` (`f15c`, pure INS: Groves Schuler oscillation + radial drift, completely immune to RF jamming) and `magr-2k` (`kc46a`, dual-frequency L1 `1575.42 MHz` / L2 `1227.60 MHz` SAASM GPS).
   - Evaluates live `EmitterMode::Jammer` interference over `World::pathLossDb` using Groves Eq. 8.35 $(C/N_0)_{\text{eff}}$, dropping GNSS lock when $(C/N_0)_{\text{eff}} < 22\text{ dB-Hz}$, drifting on dead reckoning while jammed, and re-acquiring cleanly when jamming ceases.
5. **Tactical Comm Terminals & Mode 5 IFF (`src/simulation/mission_systems/comm/CommTerminal.{h,cpp}`)**:
   - Covers all 9 `comm_terminal` cards (`a10c-link16`, `an-arc-210`, `an-asq-242`, `gripen-link16`, `ifdl`, `jtids-2h`, `mids-jtrs`, `mids`, `u2s-datalinks`) while preserving native DMS/C2 delivery intact.
   - Evaluates band-specific RF jammer interference with terminal processing gain (Link-16 FHSS, Ku-band MADL/IFDL, VHF/UHF AN/ARC-210) and models ICAO Annex 10 / STANAG 4193 `1030/1090 MHz` Mode 5 IFF link closure on fitted suites (`mids-jtrs`, `an-asq-242`).
6. **Ground-Map Radar (`src/simulation/mission_systems/ground_map/GroundMapRadar.{h,cpp}`)**:
   - Covers `an-apq-166` (`b52h`) and `h6k-radar` (`h6k`), preserving `CapabilityType::Status`-only A-GRA contract while publishing `ContactKind::Terrain` clearance profiles and X-band emissions when operating.

## Public Interfaces

| Layer | Weather Search | SAR Image | Contacts & Reports |
| --- | --- | --- | --- |
| C++ `Vehicle` / `Agent` | `submit(PayloadTask{..., TaskKind::Search})` | `submit(PayloadTask{..., TaskKind::Image})` | `contacts(payload)` (`Weather`, `Image`, `Surface`, `Terrain`), `payloadReport(payload)` |
| C99 ABI | `fsim_vehicle_payload_task` (`FSIM_TASK_SEARCH`) | `fsim_vehicle_payload_task` (`FSIM_TASK_IMAGE`) | `fsim_vehicle_contacts`, `fsim_vehicle_payload_report` |
| Python `Vehicle` / `Agent` | `search_weather(...)`, `submit_task(...)` | `image_sar(...)`, `submit_task(...)` | `contacts(payload)`, `payload_report(payload)` |

## Verification & Open-Source Alignment

- Public-literature and open-source methodology alignment (Marshall-Palmer/Probert-Jones/ITU-R P.838, Curlander-McDonough SAR, Skolnik/Richards SMTI, Groves INS/GNSS, Link-16/ICAO 1030/1090 MHz Mode 5 IFF) is recorded in [open-source-alignment.md](../research/sim-m07/open-source-alignment.md).
- Native C++ suite `ms11_families` (`tests/simulation/test_ms11_families.cpp`) and Python `test_ma_equipment.py::test_ms11_weather_sar_and_pnt_through_python_agent` verified with zero warnings (`-Werror`) on both `ucrt64-headless` and `ucrt64-release` alongside `sdk_boundary`.
