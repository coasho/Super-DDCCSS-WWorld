# Radar measurement-error floors (SIM-M01, first radar package)

Package record and feature description for SIM-M01's first radar package under the
[Mission Systems work allocation](../architecture/modules/simulation/ms-work-allocation.md#11-mandatory-public-data-fidelity-open-source-reuse-and-fleet-coverage)
(§1.1, §3 rows 1 and 2B, §5). Owner: Claude. This page is the package's own radar-only record. The shared allocation,
status, carryover register and SDK guide pages are outside this package's write window. The links that would point to
this page from them remain to be made in a later shared-document window.

## 1. Freeze (2026-10-17, before any source edit)

- **Baseline:** main `14a90f450392d8b737d4b46f9d1bb99cb0661987` (`14a90f4`).
- **Concurrent work:** another owner's uncommitted SIM-M02 slice touches
  `docs/architecture/modules/simulation/ms-work-allocation.md` and `tools/models/hangar/**`. Those files are outside
  this package; it neither reads them for its inputs nor writes them.
- **Scope:** one radar and one named processing gap.
  - **Radar:** Ericsson (now Saab) PS-05/A, card `assets/payloads/ps-05a.json`. Its only fit in the current roster is
    the JAS 39C Gripen (`aircraft/gripen/payloads.json`, payload `radar`).
  - **Gap:** SIM-M01's "sourced measurement-error floors" ([simulation plan](../architecture/modules/simulation/plan.md#5-functional-roadmap-retained-after-structural-work)).
    Today each detection's errors are thermal only: resolution / sqrt(2 SNR) in elevation, azimuth, range and range rate,
    at both the detection level (MS-4) and the signal level (MS-6). They shrink without bound as the SNR grows, and the
    tracker (MS-5) takes them as its measurement noise. This package adds per-look error floors that do not shrink
    with the SNR, carried into the reported standard deviations and so into the tracker.
- **Not in this package:**
  - target glint (it needs the target's extent, which the radar's scene does not give: a shared-runtime change);
  - boresight/radome bias (a correlated error, not a per-look floor);
  - multipath;
  - the refused signal-level CUED/TRACK/STT and velocity-search combinations (refused in `Host.cpp`, shared runtime);
  - any public interface change;
  - any other radar's values.

### 1.1 Admitted public inputs

| Id | Source (exact version and location) | What it supports | Admitted as |
| --- | --- | --- | --- |
| S1 | MIT Lincoln Laboratory, "Tracking and Parameter Estimation" (Introduction to Radar Systems course, slides EST&TRACK-1 to 41, dated 7/31/2008), https://www.ll.mit.edu/media/6981, retrieved 2026-10-17, SHA-256 `8ab4c3dd0a74f28a9efc1cbddd616d28b63f136293f70150781678791521bae9` | Slide 27: monopulse accuracy improves with SNR, about 10:1 beam splitting at a typical ~13 dB threshold (the present thermal law). Slide 28: a bias error is the true value minus the average measured value. Slide 29: Doppler accuracy is about 1 / (coherent time x sqrt(SNR)). Slides 30-31: real-world limits beyond receiver noise - calibration, amplitude fluctuation, angle noise (glint), multipath bias | Method only: it names the mechanisms that bound accuracy at high SNR but gives no value for any radar |
| S2 | MathWorks, `fusionRadarSensor` reference page, release R2026b ("Introduced in R2021a"), https://www.mathworks.com/help/fusion/ref/fusionradarsensor-system-object.html, read 2026-10-17 | A documented measurement-model convention: each bias fraction "sets a lower bound" on that axis's accuracy, as a fraction of its resolution (the azimuth resolution "typically the half-power beamwidth"). Defaults: `AzimuthBiasFraction` and `ElevationBiasFraction` 0.1, `RangeBiasFraction` and `RangeRateBiasFraction` 0.05. No formula is given | The anchor of an explicit assumption: a proprietary tool's documented generic defaults, not data about the PS-05/A |
| Card | The card's existing sources, unchanged: English Wikipedia "PS-05/A", revision 1357555494 (8-10 GHz, 1 kW average, 60 cm antenna, a mechanical antenna/platform), and Stimson (false-alarm time) | The radar's band, power and aperture, hence its beamwidth and resolutions | As labelled on the card today |

Searched for and not found (2026-10-17), so not admitted:
- any published PS-05/A angle, range or range-rate accuracy, or error budget. The English Wikipedia article and the
  Forecast International archived report give modes and processing (pulse compression, narrow Doppler filters, PRF
  switching), not accuracy;
- Military Periscope's page is paywalled and was not used;
- forum posts were not used.

No published floor value exists for this radar, so its floors are explicit assumptions (§1.3), never published values.

### 1.2 Open-source evaluation

| Candidate (version) | Licence | Scope found | Decision |
| --- | --- | --- | --- |
| Stone Soup 1.9.1 (the version the existing MS-5 port is cross-checked against, `tests/simulation/data/track/crosscheck.json`; radar sensors documented through v1.9.1) | MIT | Radar measurement models take a fixed `noise_covar`; no SNR scaling, bias or floor | Keep consuming it as the tracker: its measurement noise is the radar's per-detection standard deviation, now including the floor. Nothing to adopt for the floor itself |
| FERS (SourceForge 0.23, 2013; development mirror github.com/davidbits/FERS) | GPL-2.0 | Signal-level raw returns; no estimation or accuracy model | Not applicable to a measurement floor; not linked (licence). ADR-30's MS-6 cross-check role is unchanged |
| RadarSimPy | GPL-3.0 wrapper over a closed commercial engine | Signal level | Not usable (ADR-30, MS-6 amendment) |
| MathWorks `fusionRadarSensor` (R2026b) | Proprietary | Documented bias-fraction lower bounds | Not reusable code. Its documented convention anchors the assumption (S2) |

No mature open-source implementation provides SNR-scaled accuracy with floors. The custom work is necessary and small:
a quadrature term added to the existing accuracy model.

### 1.3 Model and frozen values

For each measured quantity k (elevation, azimuth, range, range rate):

    sigma_k = sqrt(sigma_thermal,k^2 + sigma_floor,k^2)

- **Thermal:** `sigma_thermal,k` is today's term, unchanged.
- **Floors:**
  - angle: `sigma_floor,angle = f_angle x (that plane's half-power beamwidth)`;
  - range: `sigma_floor,range = f_range x (range resolution)`;
  - range rate: `sigma_floor,rate = f_rate x (Doppler resolution, wavelength / (2 x the level's coherent time))`.

  Each uses the same resolution the thermal term already uses at that level.
- **Detection level:** each measurement is the truth plus `sigma_k` times the same standard normal draw it takes today.
  The floor changes no draw and no detection decision.
- **Signal level:** the processor's own estimate gains an independent zero-mean normal error of `sigma_floor,k`, drawn
  from the signal level's stream after the dwell's own draws, and only when a floor is above zero. The reported
  standard deviation is the root sum of squares of today's reported value and the floor.
- **Cards without the floor parameters:** no floor, exactly today's behaviour.
- **Invalid floors:** a floor parameter that is negative or not finite makes the card's radar invalid (`Spec::ok`
  false), not silently zero.

PS-05/A card parameters (new; basis `assumed`; anchored on S2's documented defaults; S1 for the mechanism):

| Parameter | Value | Uncertainty | Resulting floor for the PS-05/A (approximate until measured in §2) |
| --- | --- | --- | --- |
| `angle_floor_beamwidths` | 0.1 | 0.09 (0.01 to 0.19) | about 6.7 mrad: a 0.6 m, 9 GHz parabolic aperture's ~3.8 deg half-power beamwidth |
| `range_floor_resolutions` | 0.05 | 0.045 | 7.5 m of its 150 m (assumed) range resolution |
| `range_rate_floor_resolutions` | 0.05 | 0.045 | about 0.013 m/s of its detection-level Doppler resolution |

The central angle value may be pessimistic for a fire-control radar. The band reaches 0.01 beamwidth, and the
sensitivity cases below cover it. This floor's fidelity gate stays open until public PS-05/A accuracy evidence is
admitted (§3).

### 1.4 Reference conditions and tolerances (frozen before implementation)

| Case | Condition | Tolerance |
| --- | --- | --- |
| R1 high SNR, detection level | PS-05/A staring at a steady (non-fluctuating) target on its axis, free space, SNR above 40 dB, 4,000 detections | Sample standard deviation of each of the four errors within 6 % of `sigma_k` (sampling error about 1.1 %). Each mean within 4 standard errors of the floorless run's mean |
| R2 near threshold | The same geometry with the SNR near 15 dB | As R1. The thermal term is no longer negligible, so the quadrature is tested, not just the floor |
| R3 same draws | R1 and R2 with the same seed, with and without floors | The same detections (count, truth, time). Each error scales exactly by `sigma_k / sigma_thermal,k` (relative difference below 1e-9) |
| R4 signal level | PS-05/A at the signal level, high SNR, free space, at least 600 detections | The reported `sigma_k` is the root sum of squares of the floorless report and the floor (below 1e-9 relative). The empirical angle-error standard deviation is within 12 % of the mean reported `sigma` |
| R5 no-floor cards | Every other air-radar card, and PS-05/A with its floors removed | `specOf` gives zero floors. Detections are identical (bitwise) to today's for the same seed |
| R6 invalid input | A floor of -0.1, NaN or infinity on a card | `Spec::ok` is false |
| R7 sensitivity | `angle_floor_beamwidths` at 0.01, 0.1 and 0.19 | R1's tolerance at each value. A tracker fed those detections reports position standard deviations that grow with the floor |
| R8 public encounter | Python SDK: a Gripen with a target 5 km ahead in formation, against the same with an F-16C (AN/APG-68(V)9, no floor) | Gripen: range-error standard deviation 4-12 m, RMS position error 15-120 m. F-16C: below 3 m and 10 m. Gripen track `position_sigma_m` above the F-16C's |

### 1.5 Path and symbol allowlist (frozen)

Writes are confined to these paths:

| Path | Symbols / content |
| --- | --- |
| `src/simulation/mission_systems/radar/Radar.h` | `Spec`: three floor fractions; a private helper declaration if needed |
| `src/simulation/mission_systems/radar/Radar.cpp` | `specOf` (read and validate the floors); `Radar::dwell` (the measurement lambda); `Radar::dwellSignal` (the measurement block); a file-local helper |
| `assets/payloads/ps-05a.json` | Three new parameters with basis, uncertainty and note; sources S1 and S2 |
| `tests/simulation/test_radar.cpp` | New test cases R1-R7 (existing cases unchanged) |
| `tests/compatibility/python/test_radar.py` | New public encounter test R8 (existing tests unchanged) |
| `docs/sdk/radar-measurement-floors.md` | This record and its results |

Read-only for this package: everything else. In particular:
- **Missile paths:** `src/simulation/mission_systems/weapon/**`, `assets/stores/**`, missile designs and
  `tools/models/hangar/**`.
- **Shared runtime and data:** `Signal.*`, `Detection.*`, `track/**`, `Host.*`, `Cards.*`, `Model.*`, World, Agent,
  adapters, `include/fsim/**`, `python/fsim/**` and every other card.
- **Build files:** all CMake files and `tools/validation/testing/scopes.toml`.
- **Shared documents:** `docs/architecture/**`, `docs/status.md`, `docs/sdk/mission-systems.md`, `docs/sdk/README.md`
  and the carryover register.

New test cases are discovered by the existing Catch2 and Python registrations, with no build-file edit.

### 1.6 Checks and stop point

- **Checks:** related fast checks only. The change-scoped selector's targeted tests for the changed paths, the radar
  cases above, the card loader's validation, and the planning, link and documentation checks. No full suite, GUI or
  performance run.
- **Native slot:** native configure, build and test run serially, after confirming no other native build is active.
- **Stop:** independent review of this package. No other radar, mode or family is started, and no missile
  calibration is changed.

## 2. Results (2026-10-17)

### 2.1 The feature

- **Card:** `Spec` gains `angleFloorBeamwidths`, `rangeFloorResolutions` and `rangeRateFloorResolutions`, read from
  the card's `angle_floor_beamwidths`, `range_floor_resolutions` and `range_rate_floor_resolutions` (0 where absent).
  `specOf` refuses a negative or non-finite floor.
- **Detection level:** each standard deviation is `floored(thermal, floor)`, the root sum of squares where the floor
  is above zero and the thermal term itself otherwise. The draws are unchanged.
- **Signal level:** each detection gains a zero-mean error of the floor's size from the signal stream, after the
  dwell's own draws and only when a floor is above zero. Its reported standard deviations take the floor in
  quadrature.
- **Interfaces:** no public interface changed. The detections and tracks the SDK already reports carry the effect.
- **For a user:** a radar card may now give these three fractions. The PS-05/A's are assumptions (§1.3), so its
  reported accuracy at high SNR is bounded by them, and its tracker's measurement noise with it.

Measured PS-05/A values: half-power beamwidth 3.632 deg in both planes, so the angle floor is 6.339 mrad; range floor
7.5 m; Doppler resolution 0.275 m/s at the detection level, so the range-rate floor is 0.0138 m/s. The freeze's
"about 3.8 deg / 6.7 mrad" was an estimate made before measurement.

### 2.2 Reference cases (Release, `fsim_tests "[floor]"` and `python_sdk test_radar`)

| Case | Measured | Tolerance | Result |
| --- | --- | --- | --- |
| R1, 45.0 dB, 4,000 looks | Spreads against expected: elevation 6.221 / 6.341 mrad (-1.9 %), azimuth 6.432 / 6.341 mrad (+1.4 %), range 7.543 / 7.524 m (+0.3 %), range rate 0.01383 / 0.01380 m/s (+0.2 %). Every mean inside 4 standard errors | 6 %, 4 SE | Pass |
| R2, 15.0 dB as dwelled, 3,379 looks | Elevation 8.159 / 8.062 mrad (+1.2 %), azimuth 8.134 / 8.062 mrad (+0.9 %), range 20.36 / 20.30 m (+0.3 %), range rate 0.03687 / 0.03723 m/s (-1.0 %). The thermal terms (4.98 mrad, 18.86 m, 0.0346 m/s) are comparable with the floors here | 6 %, 4 SE | Pass |
| R3 | Both SNRs: the same detections, times and SNRs with and without floors. Each floorless standard deviation is the thermal law to 1e-12; each floored one is the quadrature to 1e-12; each error is the floorless one times the ratio to 1e-9 | as frozen | Pass (103,338 assertions) |
| R4, signal level, 900 looks (all resolved) | Reported standard deviations are the quadrature to 1e-9. Elevation spread 6.327 against 6.340 mrad reported (-0.2 %), azimuth 6.150 / 6.340 mrad (-3.0 %); means inside 4 SE | 12 % | Pass |
| R5 | The 16 other air-radar cards give no floor and stay valid | as frozen | Pass |
| R6 | -0.1, NaN and infinity on each floor make the radar invalid; 0 is valid | as frozen | Pass |
| R7, 4,000 looks each | Angle spreads within 2 % of expected at 0.01 / 0.1 / 0.19 beamwidths. The track's position uncertainty is 4.01 / 22.52 / 36.53 m | 6 %, increasing | Pass |
| R8, public SDK, 5 s | Gripen: range-error SD 7.32 m, position RMS 44.9 m, track `position_sigma_m` 8.92 m. F-16C: 2.05 m, 6.0 m, 0.52 m | Gripen 4-12 m, 15-120 m; F-16C below 3 m and 10 m; Gripen track above F-16C's | Pass |

The F-16C's 2.05 m and 6.0 m in R8 are mostly the fixture's own truth, states taken at step times in a formation that
drifts, not its radar's thermal error (about 0.1 m at that SNR). The radar compensates its carrier's and the target's
motion within a step (`Radar::step`, `dwell`). The bounds were frozen with that margin.

### 2.3 Corrections to the test set-up, not to the frozen conditions

- **R2:** asking `snrDb()` for 15 dB gave a dwell SNR about 3.5 dB lower (the card's class calibration and the look),
  where 4,000 frames gave 340 detections. The test now measures that offset in the R1 run and puts R2 at 15.0 dB as
  dwelled. There, the PS-05/A's 3-of-8 medium-PRF schedule detects a steady target about 8.4 % of the time, so R2 runs
  40,000 frames to get 3,379 detections. Condition and tolerance are as frozen.
- **R8:** with one bar over 10 s, the F-16C carrier's own flight took the target out of its scan after about 6 s, and
  its track was dropped. The fixture now uses two bars and 5 s, during which both carriers detect and track
  throughout. Bounds unchanged.

### 2.4 Checks run

- **Builds:** `ucrt64-headless` and `ucrt64-release`, incremental, warnings as errors, both clean. Inventories: 721
  headless (717 + 4) and 789 Release (785 + 4).
- **Related tests,** both presets, 38/38 each (52.9 s and 52.1 s):
  - every radar and radar-mode case, the new four among them;
  - the Stone Soup tracker cross-checks;
  - `mission systems:` card and fit checks;
  - `agents: its radar's tracks`;
  - `python_sdk test_radar`, `test_fleet`, `test_mission_systems` and `test_esm`.
- **Documentation:** `architecture_boundary`, `ma_contract` and `sdk_guard_checker` 3/3. The planning checker
  accepts 59 documents. The local-link audit finds no broken link.
- **Working tree at the time:** the checks ran on a working tree that also held another owner's uncommitted SIM-M02
  edits, outside this package:
  - `assets/stores/*.json`;
  - `tools/models/hangar/**`;
  - the allocation page.

  The card checks read the store cards, and they passed. This package neither depends on those edits nor includes
  them.

### 2.5 Not run, and why

- **Full suites:** the change-scoped selector plans the whole headless suite (717), because no test scope claims
  `src/simulation/mission_systems/radar/**`, and 587 of 785 Release tests, because the card is a staged asset the
  Viewer reads. Only related fast checks were authorized, so the full suites remain a pending plan.
- **Selector follow-up:** a scope for the radar paths would bound such plans. It belongs to the owner of
  `tools/validation/testing/scopes.toml`, which is outside this window.
- **No GUI or performance run.** The Viewer's payload windows read the card unchanged in form.

### 2.6 Commits

- `3cfcbe8`: the freeze (§1), before any source edit.
- The implementation, tests, card and these results follow in one commit.

## 3. Coverage and remaining work

Air-radar fits in the current roster (35 airframes with fits among 89 aircraft directories; the rest are stores and
missiles). After this package:

| Radar (card) | Airframe | Measurement floor |
| --- | --- | --- |
| PS-05/A (`ps-05a`) | JAS 39C Gripen (`gripen`) | This package: assumed floors (§1.3); fidelity gate open until public accuracy evidence is admitted |
| AN/APG-63(V)3, AN/APG-68(V)9, AN/APG-73, AN/APG-77, AN/APG-79, AN/APG-81, AN/APY-2, Captor-M, MESA, N001, N019, N036, RBE2 AA, RDI, Type 1473, Type 1475 | F-15C, F-16C, F/A-18C, F-22A, EA-18G, F-35A, E-3G, Typhoon, E-7A, Su-27S, MiG-29A, Su-57, Rafale, Mirage 2000, J-10A, J-20A | None (thermal accuracy only): open under SIM-M01, one radar at a time |

Weather, SAR and ground-map radar cards remain SIM-M07's. Still open beyond this package:
- glint, which needs the target's extent from the shared scene;
- boresight and radome bias;
- multipath;
- signal-level track and velocity modes;
- the PS-05/A's per-card validation and its remaining assumed parameters.
