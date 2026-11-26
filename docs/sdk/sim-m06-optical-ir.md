# SIM-M06 (MS-10): Passive & Active Optical/IR Sensors

SIM-M06 delivers physical passive optical/IR sensors (IRST, EOTS/FLIR, spherical DAS/MLD/MAWS, solar-blind UV missile warning, and ISR EO/IR) and active optical laser rangefinding/designation (`Family::PassiveOptical` / `CapabilityType::PassiveOptical`) across all 19 maintained `passive_optical` payload cards through C++, C99 ABI, and Python.

## Native Physics and Subsystem Models

1. **Radiometric Target Signatures (`src/simulation/mission_systems/optical/OpticalSensor.{h,cpp}`)**:
   - **Aerodynamic kinetic-heating airframe skin**: computes recovery skin temperature $T_{\text{skin}} = T_{\text{amb}}(h)(1 + 0.85 \cdot 0.2 M^2)$ and integrates Stefan-Boltzmann + exact dimensionless Planck in-band contrast radiation over the sensor's passband $[\lambda_{\min}, \lambda_{\max}]$, capturing high-Mach supersonic head-on detection naturally.
   - **Aspect-dependent turbine tailpipe & exhaust plume**: driven directly by each target's live JSBSim `VehicleState` (`engineCount`, `throttlePosition`, `afterburner`), modeling tail-aspect nozzle cavity exposure and strong $\text{CO}_2$/$\text{H}_2\text{O}$ MWIR afterburner surges.
   - **Rocket plumes & pyrotechnic flares**: `Role::Weapon` burning motors (`Scene::weaponBurning`) emit intense MWIR and solar-blind UV (`0.25–0.28 um`) plumes; coasting weapons retain kinetic-heating skin contrast without UV emission; active flare decoys (`Scene::flares()`) radiate decaying MWIR signatures.
2. **Altitude-Stratified Atmospheric & Weather Optical Transmittance**:
   - Combines LOWTRAN/MODTRAN-class altitude-density-scaled Beer-Lambert clear-air extinction, Kruse/Koschmieder visibility aerosol extinction (`EnvironmentState::visibilityM`), humidity absorption, overcast cloud-layer opacity (`cloudBaseM`, `cloudCover`), precipitation attenuation, and terrain/Earth-horizon line-of-sight masking (`Scene::masked`).
3. **Passive Detection, Angular Tracking, and No-Fabrication Range Policy**:
   - Evaluates aperture irradiance $E = I_{\text{eff}}\tau_{\text{atm}} / R^2$, look-down Earth-background NEI penalty, Gaussian threshold crossing probability $\frac{1}{2}\operatorname{erfc}((\xi_{\min} - \text{SNR})/\sqrt{2})$, and SNR-tightening angular errors down to the sensor's IFOV/stabilization floor.
   - Passive IRST/FLIR/DAS/MAWS contacts ALWAYS report `rangeM = NaN`, `latitudeRad = NaN`, `longitudeRad = NaN`, and `heightM = NaN` unless actively ranged by a fitted laser rangefinder.
   - Spherical DAS/MLD/MAWS sensors (`an-aaq-37`, `j20a-das`, `an-aar-56`, `an-aar-54`, `c17a-mws`, `gripen-maws`) run continuously in full $4\pi$ surveillance upon entering `PayloadState::Operate`, setting `Contact::warning = true` on closing or burning missiles. Solar-blind UV MWS (`an-aar-54`) detects burning rocket plumes while ignoring non-burning jet aircraft.
4. **Active Optical Laser Rangefinder / Designator (`TaskKind::Track`)**:
   - Explicitly fitted only on laser-equipped cards (`an-aaq-40`, `eots-86`, `h6k-eots`, `oeps-27`, `oeps-29`, `osf`, `101ks-atoll`, `klen-ps`); passive-only cards (`pirate`, `eord-31`, `an-aaq-37`, `an-aar-54`, `an-aar-56`, `an-asq-151`, `eiss-eoir`, `syers-2c`, `c17a-mws`, `gripen-maws`, `j20a-das`) report `laser_capable = 0` and never produce laser range.
   - In `TaskKind::Track`, laser-capable payloads fire a two-way Lambertian optical link ($P_r = \frac{P_t A_r \rho \eta_{\text{sys}} \tau_{\text{atm}}^2}{\pi R^2}$ at `1.064 um` or eye-safe `1.54 um`). When inside `laserMaxRangeM` with laser SNR $\ge 10$, populates measured `rangeM` (`2–5 m` 1-sigma), `rangeRateMs`, and 3-D geodetic coordinates (`latitudeRad`, `longitudeRad`, `heightM`). Beyond range or behind clouds, reverts cleanly to `rangeM = NaN` while angular IR tracking continues until track memory expires (`Reason::ContactLost`).

## Public Interfaces

| Layer | Optical Search / Track | Optical Contacts | Telemetry Report |
| --- | --- | --- | --- |
| C++ `Vehicle` / `Agent` | `submit(const ms::PayloadTask&, opts)` (`TaskKind::Search`, `TaskKind::Track`) | `contacts(payload)` (`ContactKind::Infrared`, `truth == 0` on `Agent`) | `payloadReport(payload)` (`contacts`, `warnings`, `searching`, `tracking`, `laser_capable`, `laser_active`, `laser_ranged`, `laser_range`, `wavelength_min`, `wavelength_max`, `nei`) |
| C99 ABI | `fsim_vehicle_payload_task` (`FSIM_TASK_SEARCH`, `FSIM_TASK_TRACK`) | `fsim_vehicle_contacts` | `fsim_vehicle_payload_report` |
| Python `Vehicle` / `Agent` | `search_optical(...)`, `track_optical(...)`, `submit_task(...)` | `contacts(payload)` | `payload_report(payload)` |

## Verification & Open-Source Alignment

- Public-literature and open-source methodology alignment (Hudson, Mahulikar, LOWTRAN/Kruse, AFSIM `WSF_IRST_SENSOR`, Koechner/Kamerman LRF link, and parameter provenance across all 19 cards) is documented in [open-source-alignment.md](../research/sim-m06/open-source-alignment.md).
- Native C++ suite `optical_ir` (`tests/simulation/test_optical_ir.cpp`): radiometric/Planck/Beer-Lambert/LRF analytical alignment, all 19 `passive_optical` card coverage & honest LRF discrimination, fighter IRST/EOTS search + passive-range honesty + active LRF ranging + cloud occultation `ContactLost`, spherical DAS 360-deg coverage + flare visibility + UV MWS discrimination + Agent boundaries.
- Cross-language Python test (`tests/compatibility/python/test_ma_equipment.py::test_optical_ir_search_track_and_laser_ranging`) and `sdk_boundary` boundary check across both `ucrt64-headless` and `ucrt64-release`.
