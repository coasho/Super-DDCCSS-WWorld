# SIM-M09 (MS-7f & Mission Systems M1): Public Literature & Open-Source Validation Alignment

This note records the public engineering literature, open-source simulation
benchmarks, quantitative acceptance criteria, and honest boundaries governing
**SIM-M09** (`MS-7f` validation & original Mission Systems `M1` autonomous
engagement loop).

## 1. Authoritative Public References & Methodology

| Topic | Authoritative Reference | How Applied in SIM-M09 |
| --- | --- | --- |
| **Missile Flyout, Time of Flight, F-Pole & A-Pole** | P. Zarchan, *Tactical and Strategic Missile Guidance* (AIAA, 6th ed.); ICAO Doc 7488 ISA; JSBSim 6-DOF flight dynamics | Cross-checks live 6-DOF `World` missile flyouts (active radar `AIM-120C` / `Meteor`, semi-active `AIM-7M`, infrared `AIM-9M`) against SIM-M04 flyout-derived `launchZoneV2` (`maximumM`, `noEscapeM`, `timeOfFlightS`, `fPoleM`, `aPoleM`) and published open-source ranges recorded in [dlz-published-comparison.md](../sim-m04/dlz-published-comparison.md). |
| **Aircraft Combat Survivability & Kill Semantics** | R. E. Ball, *The Fundamentals of Aircraft Combat Survivability: Analysis and Design* (AIAA, 2nd ed.) | Strictly separates release, fuze detonation, miss distance, component damage, and crew loss in `WeaponRecord` and `EngagementEvent`. A detonation or release never fabricates a kill or mission-success verdict. |
| **Agent-Boundary Autonomy & C2 Authority** | Open-Arsenal A-GRA ASK 6.0a (MS L1 Volume, RBAC & Strike Consent); ADR-31 Agent/Referee boundary | Both autonomous fighters in the 1v1 loop hold ONLY `fsim::Agent` handles (`truth == 0`). Weapon release requests traverse the native C2 DMS consent handshake (`declareRole` -> `StrikeCommand(consent=false)` -> `consentRequests` -> `answerConsent`); never Range consent injection or truth reads. |
| **Multi-Ship Real-Time & Determinism Benchmarking** | AFSIM / JSBSim / Stone Soup benchmark discipline | Measures wall-clock real-time factor ($\text{RTF} = T_{\text{sim}} / T_{\text{wall}} \ge 1.0$) on a full 4v4 air-to-air scenario (`4× F-16C` vs `4× Su-27S`) with active radars, RWRs, IRST, Mode 5 IFF comm terminals, DMS track sharing, missiles, and chaff/flares, plus bit-identical repeatability across independent seeded runs. |

## 2. Scope & Honest Boundaries

- **Verified natively in SIM-M09**:
  1. Published range, time-of-flight, and no-escape envelope consistency across
     active-radar (`AIM-120C`, `Meteor`), semi-active (`AIM-7M`), and infrared
     (`AIM-9M`) missiles using live 6-DOF `World` execution vs. SIM-M04 tables.
  2. Autonomous 1v1 end-to-end Mission Systems M1 engagement loop in C++
     (`tests/integration/test_sim_m09_encounters.cpp`) and as a minimal public
     Python SDK example (`examples/simulation/python/autonomous_1v1_encounter.py`):
     radar search -> track -> own-track designation -> `launchZoneV2` gating ->
     C2 DMS release consent -> active/semi-active launch -> RWR warning ->
     defensive crank/notch & chaff dispensing -> native fuze/damage/event audit.
  3. Measured 4v4 real-time air-to-air scenario (`4× F-16C` vs `4× Su-27S`)
     holding $\text{RTF} \ge 1.0$ with Mode 5 IFF telemetry, DMS P2P track
     dissemination, multi-missile launches, countermeasures, and exact seeded
     determinism.
- **Explicitly pending (not claimed closed)**:
  - Fullscreen primary-display Vulkan Viewer visual acceptance (`--live` 1x
    fullscreen pixel review) remains **skipped/pending** under the Antigravity
    Windows isolated-desktop (`WinSta0\exebox-*`) constraint until re-enabled on
    an interactive desktop session.
  - Individual weapon/airframe empirical equipment calibration remains tracked
    in the separate uncertainty ledger (`equipment_calibrated = false`,
    `basis: simulated`).
