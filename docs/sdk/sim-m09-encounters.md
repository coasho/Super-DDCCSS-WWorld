# SIM-M09 (MS-7f & Mission Systems M1): Encounter Validation, 4v4 Real-Time Benchmark & Autonomous 1v1 Loop

**SIM-M09** validates the integrated Mission Systems stack end-to-end across
public SDK `Agent` and C2 DMS boundaries, delivering:
1. Multi-seeker flyout DLZ/WEZ cross-checks against published envelopes;
2. The original Mission Systems **M1** autonomous 1v1 engagement loop (C++ and
   runnable Python SDK example); and
3. A measured **4v4 real-time air-to-air benchmark** per payload family with
   Mode 5 IFF, Link 16 DMS P2P track sharing, active missiles, countermeasures,
   and bit-identical seeded determinism.

See [open-source-alignment.md](../research/sim-m09/open-source-alignment.md) for
the literature references and honest validation boundaries.

## 1. Delivered Artifacts

- **C++ integration & benchmark suite**:
  `tests/integration/test_sim_m09_encounters.cpp` (CTest `sim_m09_encounters`,
  labels `sim-m09;native-engagement;performance`).
- **Python SDK autonomous 1v1 example**:
  `examples/simulation/python/autonomous_1v1_encounter.py` — runnable both
  headless (`--headless`) and viewer-published (default `--world sim-m09-1v1`)
  using strictly `fsim.Agent` + C2 DMS RBAC release consent (`ROLE_QB`).

## 2. Measured Performance per Payload Family (4v4 Scenario)

Measured on `ucrt64-headless` / `ucrt64-release` (`dt = 1/120 s`, `frameSkip = 4`,
30 Hz world step, 4 workers, `2× F-35A` + `2× F-16C` vs `4× Su-27S`):

| Configuration | Sim Window | Wall Time | Real-Time Factor (RTF) | Notes |
| --- | ---: | ---: | ---: | --- |
| 4v4 Airframes only (8 jets, JSBSim + HSA) | `3.0 s` | `0.055 s` | **`54.25x`** | Parallel flight & control stack |
| + 8 Active Radars (`TWS`) | `3.0 s` | `0.060 s` | **`50.11x`** | Detection-level pulse-Doppler + GNN tracker |
| + 8 Active RWRs / ESMs | `3.0 s` | `0.058 s` | **`51.34x`** | Emitter intercept & warning classification |
| + 4 Active IRSTs (`OEPS-27`) | `3.0 s` | `0.059 s` | **`51.07x`** | Planck + atmospheric transmittance |
| Full 4v4 Sensors + Mode 5 IFF + Link 16 DMS (CTest) | `10.0 s` | `0.217 s` (H) / `0.211 s` (R) | **`46.07x` (H) / `47.39x` (R)** | `tracks=20`, `linkTracks=48`, `contacts=21`, `iffMode5=1` |
| + 1 Live 6-DOF `AIM-120C` + Chaff (CTest) | `10.0 s` | `9.856 s` (H) / `9.927 s` (R) | **`1.015x` (H) / `1.007x` (R)** | Holds real time ($\text{RTF} \ge 1.0$) |
| + 2 Concurrent 6-DOF `AIM-120C` (diagnostic) | `20.0 s` | `42.721 s` | `0.47x` | Serial per-weapon `WeaponSensor` hotspot (HC-45 carryover) |
| On-rail IR missile thermal reticle (`aim-9m`, 1 jet) | `2.0 s` | `6.834 s` | `0.29x` | 2-D physical thermal reticle rasterizer (HC-45 carryover) |

## 3. Autonomous 1v1 Mission Systems M1 Verification

Executed identically via both C++ (`sim_m09_encounters` `[1v1]`) and public
Python SDK (`examples/simulation/python/autonomous_1v1_encounter.py --headless`):
- **Blue `F/A-18C`** (`AIM-120C` + `AIM-7M`) and **Red `F-16C`** (`AIM-120C` +
  `AN/ALE-47` CMDS) operate strictly behind `fsim::Agent` (`truth == 0` verified
  on every radar track and RWR contact).
- **Closed loop observed**: radar warm-up -> TWS search -> track acquisition ->
  own-track `designate` -> `launchZoneV2` (`flyoutDerived = true`, `basis = "simulated"`)
  -> `StrikeCommand(consent = false)` -> Blue C2 AWACS (`ROLE_QB`) DMS consent
  grant -> native `AIM-120C` launch -> Red RWR warning -> beam/crank maneuver +
  4 native chaff cartridges -> live 6-DOF `AIM-120C` proximity-fuze **detonation**.
- **Python 1v1 measurement (`45.0 s` sim)**:
  - `ucrt64-headless`: `wall = 35.483 s` (**`1.27x` RTF**), `blue_fired=true`, `c2_consented=true`, `red_defending=true`, `red_chaff_dispensed=4`, `weapon_ends=["detonated"]`.
  - `ucrt64-release`: `wall = 35.093 s` (**`1.28x` RTF**), identical outcomes.
- **Determinism**: Two independent seeded runs of both the autonomous 1v1 and
  4v4 + live-missile scenarios produce **bit-identical** state digests, track
  counts, link tracks, contacts, and weapon records.

## 4. Verification Summary & Honest Pending Items

| Check | `ucrt64-headless` | `ucrt64-release` |
| --- | --- | --- |
| `sim_m09_encounters` (3/3 cases, 506/506 assertions) | Passed (`191.72 s`) | Passed (`192.71 s`) |
| `examples/simulation/python/autonomous_1v1_encounter.py --headless` | Passed (`35.48 s`, `1.27x` RTF) | Passed (`35.09 s`, `1.28x` RTF) |
| `sdk_boundary` (2,053 entry points, 0 errors) | Passed | Passed (`2.93 s`) |

**Explicitly retained pending items**:
1. Fullscreen primary-display Vulkan Viewer visual acceptance (`--live` 1x
   fullscreen pixel verification of the 1v1 loop) remains **skipped/pending**
   under the Antigravity Windows isolated-desktop (`WinSta0\exebox-*`) bug.
2. Optimizing serial per-weapon `WeaponSensor` / on-rail IR thermal-reticle
   rasterization for `2+` concurrent missiles and loaded IR racks remains
   tracked under HC-45 performance work.
