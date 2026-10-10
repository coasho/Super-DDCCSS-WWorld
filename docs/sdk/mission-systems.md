# Mission Systems: the payloads a vehicle carries

[ADR-30](../mission-systems.md) puts A-GRA's Mission Systems (MS) interface on the platform. Each aircraft carries the
real payloads of its type: radars, warning receivers, jammers, electro-optical sensors, guns, store stations and
countermeasures. The SDK commands them the way the Vehicle Interface commands flight. This page covers what exists
at **MS-4**:

- the payloads each aircraft carries, from its fit;
- the capabilities they give, discovered honestly;
- their power states and built-in test;
- activities, failures and grants;
- weapons in flight as entities, and the radio path between any two of them;
- each aircraft's radar cross section, as one entity's radar sees another (MS-3);
- air search radars: AMTI searches commanded through the SDK, their detections, and the viewer's displays of them (MS-4).

The models behind the other capabilities (tracking, ESM, missiles, ...) come in the later stages. Until a
stage lands, its capability is advertised `not_implemented`, together with the stage that builds it.

## Payloads and capabilities

A vehicle's payloads come from its aircraft's **fit**, `aircraft/<type>/payloads.json`. Each fit entry names a
payload by its role on that aircraft (`radar`, `rwr`, `stations`) and points to the payload type's **card**,
`assets/payloads/<card>.json`. Both files cite their public sources (ADR-30 section 6). In a fit, each payload
quotes the passage of its cited source that names it, under `evidence`. Every passage was checked against the
cited revision.

Each payload gives the capabilities of its family:

| Family | Capabilities |
| --- | --- |
| `air_radar` | `fsim.ms.status`, `fsim.ms.amti` (MS-4) |
| `weather_radar` | status, `fsim.ms.weather_radar` (MS-11) |
| `sar` / `smti` | status, `fsim.ms.sar` / `fsim.ms.smti` (MS-11) |
| `esm` | status, `fsim.ms.esm` (MS-9) |
| `electronic_attack`, `countermeasures` | status, `fsim.ms.ea` (MS-9) |
| `passive_optical` | status, `fsim.ms.po` (MS-10) |
| `store_station` | status, `fsim.ms.store_management`, `fsim.ms.strike` (MS-7) |
| `gun` | status, `fsim.ms.strike` (MS-7) |
| `comm_terminal`, `pnt` | status, `fsim.ms.comm_terminal` / `fsim.ms.pnt` (MS-11) |
| `ground_map_radar` | status only: a real-beam ground-mapping or terrain-avoidance radar, which A-GRA has no capability for |

A capability instance is addressed as `<type>:<payload>`, for example `fsim.ms.amti:radar`. A capability the vehicle
lacks reports `unavailable` with `not_supported` (a known type) or `unknown_capability`. `fsim.ms.status` is
supported now: it powers a payload off, to standby or to operate (from off through the card's `warm_up_s`), and runs
its built-in test (the card's `bit_s`).

Grants and availability use the same calls as flight capabilities. Pass the instance id to `requestControl`,
`releaseControl`, `revokeControl`, `setAllowed`, `controlStatus` and `setAvailability`. Under `ControlMode::Granted`,
a command needs its controller's grant on the payload's `fsim.ms.status`.

The rules for commands:

- A newer command supersedes a payload's live activity.
- Cancelling a warm-up leaves the payload off.
- Cancelling a BIT returns the payload to its state before the test.
- A payload failed by the platform (`failPayload`) refuses commands with `payload_failed` until it is repaired. A
  repaired payload starts off.

Mission Systems reasons (`ms::Reason`) have their own list, separate from `control::Reason`. They include
`payload_off`, `payload_starting`, `payload_standby`, `bit_in_progress`, `payload_failed`, `already_in_state` and
`not_implemented`.

## C++

```cpp
fsim::Vehicle v = world.createVehicle(spec);               // jsbsim:f16c
for (const auto& p : v.payloads())                         // radar AN/APG-68(V)9, rwr AN/ALR-56M, ...
    std::printf("%s %s %s\n", p.name, p.designation, fsim::ms::familyName(p.family));
auto r = v.submit(fsim::ms::StateCommand{0, fsim::ms::PayloadState::Operate});
if (r.status == fsim::control::CommandStatus::Rejected) std::puts(fsim::ms::reasonName(r.reason));
auto esm = v.missionCapabilityStatus("fsim.ms.esm:rwr");     // unavailable, not_implemented (MS-9)
```

## C

```c
fsim_payload_info p; fsim_payload_info_init(&p);
for (uint32_t i = 0; i < fsim_vehicle_payload_count(world, id); ++i)
    if (fsim_vehicle_payload(world, id, i, &p) == FSIM_OK) printf("%s %s\n", p.name, p.designation);
fsim_ms_options o; fsim_ms_options_init(&o); o.command_id = 42;
fsim_ms_result r; fsim_ms_result_init(&r);
fsim_vehicle_payload_state(world, id, 0, FSIM_PAYLOAD_OPERATE, &o, &r);   /* r.activity, r.reason */
```

See [c_abi.md](c_abi.md) (ABI 1.49) for the full list of calls.

## Python

```python
v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0)
[p.name for p in v.payloads]               # ['radar', 'rwr', 'cmds', 'gun', 'stations']
v.payload("radar").designation             # 'AN/APG-68(V)9'
a = v.set_payload_state("radar", "operate", command_id=42)
v.mission_activity(a).state                # ActivityState.COMPLETED (no warm-up on its card yet)
v.payload_bit("radar"); v.fail_payload("radar"); v.fail_payload("radar", failed=False)
v.mission_capability_status("fsim.ms.esm:rwr")     # (Availability.UNAVAILABLE, 'not_implemented', nan): MS-9
```

A refused command raises `fsim.MissionRejected`, whose `.reason` holds the reason's name.

## Weapons in flight (MS-2)

A store released by an aircraft becomes an entity of the world (by the platform's `launchWeapon`, or by its store
management: [Stores and weapon employment](#stores-and-weapon-employment-ms-7a)). It has a vehicle id, which the state
calls read, and the viewer draws it. Sensors will see it too. It is flown by its own model; the flight capabilities report it
`unavailable` with reason `restricted`, and its seeker and guidance arrive with the missiles at MS-7. It ends where
it strikes the ground or the sea, and the world keeps its record: when and where it ended, and how fast it was going.
There are no ground entities (ADR-30, D7).

`launchWeapon` is the platform's release, as `failPayload` is the platform's contingency. A commanded release (A-GRA's
Strike and Store Management) comes with MS-7. The store starts at its station, given as an offset from the launcher's
CG in the launcher's body axes. It takes the launcher's attitude and its velocity over the earth, plus the ejection's
push. Until MS-7 brings real stores, JSBSim's own `mk82` is the store to release. Its flight model describes itself as
built from public data and guesses. Its model in the viewer is written by `python -m hangar store mk82`
(`tools/hangar/hangar/stores.py`) from the published length and diameter; the fins are estimated.

```python
jet = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=37.55, longitude_deg=-122.85, altitude_msl_m=1200.0, airspeed_ms=170.0)
bomb = w.launch_weapon(jet, "jsbsim:mk82", offset_m=(0.5, 2.5, 0.8), ejection_ms=(0.0, 0.0, 3.0))
bomb.role, bomb.launcher                   # ('weapon', 1)
w.focus_viewer(bomb)                       # the viewers follow it down
...
w.weapons                                  # [WeaponRecord(id=2, launcher=1, ..., end='impact', end_speed_ms=...)]
```

In C++, the same calls are `World::launchWeapon(launcher, ms::WeaponLaunch{...})`, `World::weapons()`,
`Vehicle::role()` and `Vehicle::launcher()`. In C, they are `fsim_world_launch_weapon`, `fsim_world_weapon_count` and
`fsim_world_weapon`, and `fsim_vehicle_role` (ABI 1.50).

## The radio path (MS-2)

Every sensor, jammer and link shares one radio path between two points in the air (`src/ms/rf`). The SDK can ask for
it between any two entities:

- **Smooth earth:** ITU-R P.528-5, NTIA's U.S. reference implementation (`third_party/ntia`). Its basic
  transmission loss includes free space, the ground reflection's multipath in line of sight, smooth-earth diffraction
  past the horizon, troposcatter, and P.676's gaseous absorption along the refracted ray.
- **Terrain:** the terrain's excess over that smooth earth, from P.526-15's delta-Bullington method over a profile of
  the world's ground. The straight ray is also checked for masking, in the 4/3 earth.
- **Rain:** P.838-3's specific attenuation along the part of the path below the rain height.
- **Or the ITM:** asked for (`itm`), NTIA's Longley-Rice over the profile replaces the above, within its limits: both
  ends 0.5-3000 m above the ground, 20 MHz-20 GHz. It charges more than a knife edge for a broad, rounded hill, and
  more so as the frequency rises.

```python
p = w.radio_path(jet, bomb, frequency_hz=9.4e9)     # RadioPath(mode=1, total_db=..., free_space_db=..., gas_db=..., terrain_masked=False, ...)
w.radio_path(a, b, frequency_hz=9.4e9, rain_mm_h=25.0, rain_height_m=4000.0).rain_db
```

The loss is a median, or the percentage of time asked for. NTIA's own published examples are reproduced to 0.05 dB
(P.528) and 0.01 dB (the ITM), and P.838-3's table to its digits (`tests/test_rf_path.cpp`). A P.528 path costs
9-27 us. P.676's layers, and each terminal's grazing ray trace from the ground (a function of the frequency and its
height alone, ~90 % of the reference's time), are tabulated per frequency and filled as asked. Over 300 random paths
they agree with the exact computation to 0.016 dB; the exact computation itself moves 0.11 dB when its terminals rise
a centimetre. A path over terrain costs ~70 us more: its profile (up to 2,001 points) is read from the deepest level
of the map set at each point, decoded once (an offline set's missing levels are not made a tile at a time), and
P.526's Bullington terms are found in one pass, the smooth earth's in closed form.

Antenna patterns (`ms/rf/Antenna.h`) come from the aperture's illumination and are tabulated in sine space. An
electronically scanned array's beam broadens and loses gain as it scans. The radars use them at MS-4.

## Radar cross section (MS-3)

Each aircraft type has a table of its radar cross section, `aircraft/<type>/rcs.bin`. The SDK reads it at the aspect
between two entities:

```python
r = w.radar_cross_section(ahead, target, frequency_hz=9.5e9, polarization="vertical")
r.sigma_m2, r.fluctuation, r.aspect_azimuth_rad    # (1.8, 1.9, 0.01): an F-16C nose-on at X band
```

In C++ this is `World::radarCrossSection(observer, target, ms::RcsQuery{...})`; in C, `fsim_world_radar_cross_section`
(ABI 1.51).

- **Aspect:** the radar's line of sight, turned into the target's body axes. The table is binned by azimuth (from the
  nose towards the right wing) and elevation.
- **Polarisation:** the radar's linear polarisation is given from the local vertical towards the horizontal. It is
  carried into the table's own basis at that aspect, so a rolled target is seen in the right polarisation.
- **What a bin holds:**
  - the mean RCS over the bin (3 deg by 3 deg), from the second moments of the scattering matrix, so any pair of
    transmit and receive polarisations follows;
  - how much the RCS fluctuates within the bin: E[sigma^2]/E[sigma]^2, which is 1 for a steady target and 2 for a
    Rayleigh one (Swerling 1/2). The radars at MS-4 draw each look from it.
- **Bands:** L (1.3 GHz), S (3.0), X (9.5) and Ku (16). Between bands the table interpolates in log frequency and log
  sigma.

### How the tables are made

`flightsim-rcs <type>` (`tools/rcs`) computes a table on the GPU by shooting and bouncing rays (SBR; Ling, Chou and
Lee, 1989), through Vulkan ray queries:

- **Rays:** for each radar direction, a grid of ray tubes, 8 to a wavelength each way (3 to 25 million for a fighter),
  covers the model's projection.
- **Bounces:** each tube is traced through up to five reflections off a perfect conductor. At every hit its physical
  optics current is radiated back to the radar, integrated exactly over its footprint.
- **Curved skin:** the faceted model is read as the curved skin it stands for. The current and the reflection follow
  the interpolated normal, and the radiating point lies halfway onto the Phong surface (alpha 0.5, which follows a
  circle to second order).
- **Cavities:** intake ducts and engine faces, the nozzles' liners and turbines (the hangar's cavity materials) send
  back the power that enters them diffusely, depolarised: sigma = 4 rho A (a Lambertian aperture).
- **Configuration:** the gear is retracted as the viewer retracts it, so the doors are closed over the wells.
  Afterburner plumes are left out, and so are a model's breakup caps (the `break` material: inside the airframe, no
  surface of it - read as a cavity's wall they raised an F-16C's diffuse return 12 %). A weapon bay is shut: its cover
  (`fsim:cover:<bay>`, the skin over its whole opening, which the viewer does not draw) stands for its doors, whose
  12 mm gaps are the viewer's, for them to turn - seen through them, the bay was a cavity, where a closed bay's seams
  are sealed and a 12 mm slot is below its cutoff at every band but Ku.
- **Validation:** against the closed forms of canonical targets (`ctest -R rcs`, on a GPU with ray queries).

| Target | Closed form | Computed (theta-theta / phi-phi) |
| --- | --- | --- |
| Sphere, a = 0.5 m, 10 GHz | pi a^2 = -1.05 dBsm | -0.98 / -0.98 |
| Plate 0.5 m, broadside | 4 pi A^2 / lambda^2 = 29.41 dBsm | 29.36 / 29.36; first side lobe at -13.26 dB within 0.5 dB |
| Cylinder a = 0.25 m, l = 1 m, broadside | 2 pi a l^2 / lambda = 17.19 dBsm | 17.14 / 17.14 |
| Dihedral 0.3 m (two bounces) | 8 pi a^4 / lambda^2 = 23.55 dBsm | 23.60 / 23.46, its polarisations opposed |
| Trihedral 0.2 m (three bounces) | 12 pi a^4 / lambda^2 = 18.27 dBsm | 18.14 / 18.20 |

**A table and its model.** A table records the model it was made from (its provenance's `model_fnv1a`).
`flightsim-rcs <type> --verify` holds it to the model as it is now: five elevation rows (-45, -15, 0, 15, 45 deg) in
every band computed again, every bin's tt and pp compared with the table's - against the method's own noise, the same
model moved by a fraction of a ray tube, which leaves its RCS the same (a translation only turns its phase) and changes
only where the tubes fall on it. Differing from the table no more than from itself moved (its 95th and 99th
percentiles and its largest difference each within 0.1 dB of the noise's), the table stands and its provenance records
the check (`verified`); else it is computed again. Measured over the fleet (2026-10-09): the method's own noise is 0.09
- 1.1 dB at its largest bin and 95 % within 0.03 - 0.2 dB, more the larger the airframe (the C-130J's, the EC-130H's,
the C-17A's the most), so a table is not reproduced to the bin unless from the very same mesh on the very same grid
(an F-22A's from its own model: 0.000 dB). And a stealth airframe's bins
follow its facetting: the F-22A's mesh decimated again after its bays were cut moved single bins seen from above,
where no bay is, by up to 6.9 dB (95 % within 1.4 - 1.7 dB at S to Ku band) with every slice's mean unchanged (within
0.1 dB) - the uncertainty a single bin of such a table carries.

A table takes 5-40 minutes per aircraft on an RTX 5070 Ti, at about 2-4 billion rays a second. The tool runs at the
idle GPU scheduling class, and while a flightsim viewer is open it waits between dispatches (`--no-yield` shares the
GPU instead): a table being made never costs the viewer frames.

### What the models carry, and the treatments

The hangar's models carry each aircraft's outline, planform, surfaces and ducts, from its public dimensions. They do not
carry:

- the finer faceting and edge alignment of a stealth design;
- coatings (radar-absorbent material);
- edge diffraction, travelling and creeping waves (outside the reach of PO and SBR);
- the cockpit interior (the canopy is taken as conducting).

**One method for every aircraft, whatever its origin.** An aircraft's low-observable features come from its signature
card (`aircraft/<type>/signature.json`). Each feature quotes the passage of its source that states it. The sources are
the aircraft's own Wikipedia article, the same kind of source for every type, and the S-duct article for the intakes it
lists. A feature its sources do not state is taken as absent. The treatments these features bring have one set of
values, the same for all (`assets/signature/treatments.json`):

| Feature | Treatment | Basis |
| --- | --- | --- |
| Radar-absorbent material over the skin | 15 dB (+-5) off each bounce from 4 GHz up; below, in proportion to the frequency | derived: Chung et al. (a bare F-22 CAD model about 10 dBsm nose on, -10 to -20 with RAM); a 6.25 mm layer gives over 10 dB from 4 to 18 GHz (Sutrakar and Anjana, 2025) |
| On the largest reflectors only | 3 dB (+-2) | assumed |
| Engine face concealed (serpentine duct, vanes, blocker) | the intake cavity's return 16 dB (+-4) lower at 9.5 GHz, (f / 9.5 GHz)^0.3 of it below | derived: CSIR-NAL's S-duct design, 16.9 dB |
| Duct walls lined | the coating's loss on the cavity too | as the coating |
| Exhaust treated | coated as the skin | as the coating |

Nothing is fitted to any aircraft's claimed RCS. The figures reported for each type are listed on its card with who
reported them (the manufacturer, an independent academic simulation, an analyst, the press, an adversary's
assessment). The tests compare them with its table, and the comparison is given here:

| Aircraft (features) | Table, X band, +-10 deg: front / side / rear (m^2) | Reported, frontal unless said |
| --- | --- | --- |
| F-22A (RAM; engine face concealed) | 0.10 / 1.35 / 2.2 | manufacturer 0.0001 "from certain angles" (a best case); press 0.0005; academic, Chung et al.: a bare CAD model about 10 dBsm nose on, -10 to -20 dBsm (0.01-0.1) with RAM; academic, Perotoni et al.: a bare model -8.8 dBsm front, 30 side, 27 rear |
| F-35A (RAM; concealed; exhaust treated) | 0.13 / 1.8 / 0.20 | press about 0.005; manufacturer "lower than a metal golf ball at certain frequencies and angles" (about 0.0015); academic, Mattei et al.: a bare POFACETS model up to a few dBsm nose on, expected at least 10 dB lower in service |
| Su-57 (RAM; concealed; duct lined) | 0.087 / 5.2 / 5.2 | manufacturer's patent: an average of 0.1-1 intended; press 0.1-0.5 |
| J-20A (RAM; S-duct; exhaust treated) | 0.056 / 6.4 / 0.58 | adversary assessment (Taiwanese specialists, as a US journal reports): 0.05 front, 5 side, 20 rear |
| Typhoon (RAM on its largest reflectors; concealed; duct lined) | 0.62 / 154 / 12 | - |
| Rafale (RAM on its largest reflectors; S-duct) | 0.72 / 123 / 28 | - |
| Su-27S (none) | 9.1 / 433 / 14 | press 10-15 |
| F-16C (none) | 1.8 / 228 / 9.4 | - |

Where the table and a claim differ, the method does not bend to the claim. The manufacturers' figures for the F-22 and
F-35 are best cases at "certain angles", 14-30 dB under the tables. The independent simulations of the F-22 and F-35
agree with the tables within their spread. The adversary assessment of the J-20A agrees front and side, and is 15 dB
above the table at the rear (it may predate its present nozzles).

The tables' uncertainty is wide, mainly from the outer shape (5-10 dB for the stealth types), the coating (5 dB) and
the intake (4 dB). Under that resolution the four stealth types come out alike: each at the level its coating leaves
of its outer shape.

## Air search radars (MS-4)

Every fighter's fire-control radar and both AEW radars are modelled at the detection level. Their capability,
`fsim.ms.amti:<payload>`, is `supported` wherever the radar's card describes it.

```python
v.set_payload_state("radar", "operate")                   # (through its warm-up, where its card gives one)
a = v.amti_search("radar", "range_while_search", azimuth_half_width_rad=math.radians(60),
                  elevation_centre_rad=math.radians(-3), bars=4, range_scale_m=150e3)
w.step(300)
for d in v.amti_detections("radar", since=0):            # AmtiDetection
    d.range_m, d.azimuth_rad, d.elevation_rad, d.range_rate_ms, d.snr_db, d.truth
v.radar_status("radar")                                   # RadarStatus: mode, volume, beam, frame period, counts
v.cancel_mission(a)                                       # it stops
```

In C++: `Vehicle::submit(ms::AmtiCommand{...})`, `radarStatus(payload)`, `amtiDetections(payload, since)`. In C (ABI
1.52): `fsim_vehicle_amti_search`, `fsim_vehicle_radar_status`, `fsim_vehicle_amti_detections`. `fsim python
examples\python\radar.py` runs an F-16C and an E-3G searching against four aircraft, for the viewer.

- **The command** (A-GRA's AMTI command): a mode, the volume about the nose (azimuth centre and half width, elevation
  centre, 1-8 bars) and a range scale. The payload must be operating. A new search supersedes the radar's last one.
  The activity runs until it is cancelled, or until the payload stops operating: standby, off or a BIT cancel it, and
  a failure fails it.
  - **Modes:** `range_while_search` (medium PRF: every aspect, range and range rate measured),
    `velocity_search` (high PRF: only targets closing faster than the radar flies, at the longest range),
    `track_while_scan` (as range-while-search) and `surveillance` (a rotodome's 360 deg sweep, its only mode). Every
    mode but velocity search keeps tracks ([below](#tracks-ms-5)).
- **The scan:** an electronic beam steps 0.8 of a beamwidth and dwells its card's time. A mechanical antenna sweeps one
  beamwidth at its scan rate. A rotodome turns at its card's rate, its bars stacked electronically. The bars are in the
  body axes: a banked aircraft tilts its scan with it.
- **Each look** is the radar equation against every entity in the beam:
  - the radar's average power, its aperture's gain towards the target (steered, with scan loss), the target's RCS
    from its table at the aspect and polarisation (MS-3), the dwell's time, noise figure and losses;
  - the radio path both ways, from P.528 with the gases and the terrain's excess (MS-2). That excess is worked out once
    a second per pair and frequency, shared by two radars looking at each other (the loss is the same both ways); the
    free-space part is exact at every look. A radar's frequency tables are built with its vehicle, not in a step;
  - **clutter:** a target whose Doppler falls within the card's notch of the main beam's ground return is lost. One
    closing or opening slower than the radar flies is seen against the side lobes' clutter, at the card's penalty
    (10 dB for a mechanical antenna, 6 dB for an AESA). Velocity search drops it.
- **Detection:** each dwell is split over its PRFs (medium PRF 3 of 8, high PRF 1 of 3), each at its share of the
  energy. Each PRF's false-alarm rate keeps the dwell's at its threshold's (next). A PRF detects with the exact single-look
  probability for the target's fluctuation: Marcum's Q for a steady target, Pfa^(1/(1+S)) for a Rayleigh one, the
  chi-square-4 form for a dominant scatterer, chosen from the table's fluctuation ratio. The draw comes from the radar's
  own stream, seeded from the world's seed and the vehicle's id: the same world gives the same detections, to the bit.
  The radars look in parallel on the world's workers, with the same detections for any number of them.
- **The threshold and false alarms:** set as a radar's designer sets it, for a false-alarm time, 60 s on every card:
  - *the fighters' radars:* published - "a minute or so" for radars in fighter aircraft (G. W. Stimson, *Introduction
    to Airborne Radar*, 2nd ed., SciTech 1998, p. 144; the sentence found in the book by Google Books' full-text
    search of that edition), read as 60 s, the reading inside the card's ±30 s;
  - *the AEW radars (APY-2, MESA):* an engineering assumption - Stimson states the figure for fighters, and no figure
    for an AEW radar has been found; their cards say so.

  Each dwell's false-alarm probability a resolution cell is its share of that: the dwell's time over 60 s, divided
  among its cells (its range bins within the range scale, by its Doppler filters over the range rates it searches). A dwell with more cells, a
  longer range scale or the wider span of a faster carrier, sets a higher threshold, and the false alarms come one a
  minute whatever the mode. They fall in its beam, anywhere in its range scale and range rates, at the threshold's
  SNR, `truth` 0.
- **Measurement:** range, azimuth, elevation and range rate, each with an error that shrinks as sqrt(2 SNR)
  (range resolution, monopulse beam splitting, Doppler resolution). The detection's position follows from them.
  `truth` names the entity that made it: the simulation's knowledge, for scoring, not the radar's.

### The cards, the same method for every radar

The parameters each radar's card (`assets/payloads/<radar>.json`) does not publish come from its class, the same for
every country:
- **Average power:** a tube transmitter's is the PS-05/A's published 1 kW (times a factor its source gives against
  its predecessor, as CAPTOR-M's twice the APG-65). A GaAs AESA's is its modules × 8 W peak × a 0.1 duty, the module
  count published or the aperture's at half a wavelength. An AEW radar's is 10 kW.
- **Noise figure:** 4 dB for designs in service before 1995, else 3 dB.
- **Side-lobe clutter:** 10 dB for a mechanical antenna, 6 dB for an AESA.

Every published detection range is a **check**, never fitted. Each class carries one calibration, minus its radars'
mean residual against their published ranges (on a 3 m² fighter where the source does not say, Pd 0.5, Rayleigh):

| Class | Calibration | Residuals after it (dB, modelled against published range) |
| --- | --- | --- |
| tube | -3.5 dB | APG-68(V)9 +2.9, RDI -9.1, N019 +1.9, N001 +7.6, PS-05/A -3.2 |
| GaAs AESA | +8.9 dB | APG-63(V)3 +8.7, APG-77 -5.2, APG-81 +3.2, APG-79 +3.2, RBE2-AA -9.8 |
| AEW | -4.1 dB | APY-2 -0.9, MESA +0.8 |

The residuals that remain, up to ±10 dB, are the published claims' own scatter: they come from different kinds of
sources, and none is bent to fit. The N001's 40 km against a 3 m² target in pursuit is clutter-limited: the clutter
model's own check (`test_radar.cpp`).

### In the viewer

The viewer shows the Mission Systems of the vehicle it follows (`tab` switches; `p`, or "payloads" in the monitor, shows
or hides the windows; each window collapses from its title bar):
- **mission systems:** each payload, its type and state (a warm-up's or BIT's progress), and the capabilities it
  gives; those not modelled yet show the stage that builds them.
- **one window per radar:** its mode, volume and frame; a B-scope for a fighter's radar (azimuth across its field of
  regard, range up, the volume searched shaded, the beam's position) or a PPI for a rotodome (heading up, the sweep).
  Detections fade over two frames, green closing and amber opening; hover one for its range, angles, rate and SNR.
  Its tracks are boxes with their numbers, a line along each one's course (its length the speed), fading when a
  track has gone two frames without an update; hover one for its updates, range, angles, rates and uncertainty. The
  "hits" box hides the detections, leaving the tracks.

The windows read the world's shared memory: each vehicle's record holds its payloads, its radars' newest 96
detections and their first 48 tracks, and the first radar at the signal level's range-Doppler map, written by the
SDK's own instances when they change (layout 5). Nothing is simulated in the viewer.

## Tracks (MS-5)

Each radar keeps its own tracks from its detections, as its processor does, in every mode but velocity search:

```python
for t in v.amti_tracks("radar"):                          # AmtiTrack, predicted to the world's time
    t.id, t.updates, t.last_update_s, t.latitude_rad, t.longitude_rad, t.height_m, t.velocity_ned_ms
    t.position_sigma_m, t.velocity_sigma_ms, t.range_m, t.azimuth_rad, t.elevation_rad, t.range_rate_ms, t.truth
s = v.radar_status("radar")                               # s.tracks, s.tentative_tracks, s.false_alarms
```

In C++: `Vehicle::amtiTracks(payload)` (`ms::AmtiTrack`, its ECEF state and covariance too). In C (ABI 1.53):
`fsim_vehicle_amti_tracks` (`fsim_amti_track`), and `fsim_radar_status` gains `false_alarms`, `tracks` and
`tentative_tracks`. `examples/python/radar.py` prints each radar's tracks every 10 s.

- **The tracker** is [Stone Soup](https://github.com/dstl/Stone-Soup)'s (Dstl, MIT), ported to C++: its
  nearly-constant-velocity model, the radar's elevation-bearing-range-rate measurement, the extended and unscented
  Kalman filters, the distance and PDA hypothesisers, global nearest neighbour by 2D assignment (SciPy's
  `linear_sum_assignment`, Crouse's algorithm, step for step), JPDA (every joint event, factorised over the clusters
  of tracks that share detections), the mixture tracker, the single- and multi-measurement initiators and the
  deleters. `tools/track/crosscheck.py` runs Stone Soup 1.9.1 on the same measurements: the ported filters, GNN and
  JPDA agree with it to 1e-11 at every step (`test_track.cpp`).
- **A radar's tracker:** GNN with the unscented filter, a dwell at a time, its detections measured from the carrier's
  pose at that dwell. A new track starts from one detection - its position and range rate from it, its velocity across
  the line of sight unknown (300 m/s an axis) - and is held until 3 updates confirm it. An extended filter is not used:
  after a rotation the target is kilometres across from where a new track predicted it, too curved a range rate for
  one linearisation; the EKF's update landed outside its own covariance (27 m off in range, 4 m claimed), and the
  same pass's next look began a second track (`test_radar.cpp` replays the E-3G's own detections of a B-52H).
- **Looks, not steps:** a scanning radar does not look at every track every dwell. A track predicted inside the
  dwell's half-power beam is *expected*: no detection counts as a miss. One within 1.5 beamwidths is *possible*: it
  may take a detection, and a miss does not count. The rest are untouched. A track with 3 expected looks and no update
  is dropped, and so is any track without an update for 3 frames and one more - one that left the scan's volume, never
  missed again where the radar no longer looks (Stone Soup's UpdateTimeDeleter).
- **Capacity:** 20 confirmed tracks for a fighter's radar, 400 for a rotodome; the card can say otherwise
  (`track_capacity`, `track_process_noise`, `track_gate`, `track_confirm`, `track_drop_looks`,
  `track_velocity_sigma`). Tracks are dropped when the payload stops operating.
- **`truth`** names the entity whose detection last updated the track (0: a false alarm) - for scoring.
- **Deferred:** an IMM (Stone Soup 1.9.1 has none to check a port against) and fusion across sensors, which needs a
  second sensor or a data link (MS-9 to MS-11).

## Stores and weapon employment (MS-7a)

A-GRA's Store Management and Weapon Employment sequences as SDK calls: the inventory (its MA_StrikeCapability), strike
with release consent, abort, jettison. A store station payload (`stations` in a fit) gives two capabilities:
`fsim.ms.store_management` (the inventory, consent) and `fsim.ms.strike` (strike, jettison) - both partly built: the
weapons' seekers, guidance and fuzes come with MS-7d, launch zones with MS-8.

```python
v = w.create_vehicle("viper", "jsbsim:f16c", ...)
v.stores                                  # [] - an aircraft starts with its stations empty
v.load_loadout("mk82 x4")                 # its fit's loadout: Mk 82s on stations 3, 4, 6, 7
v.load_store(5, "mk82")                   # one station (a scenario's load, not a Mission Systems command)
v.set_payload_state("stations", "operate")  # the master arm
a = v.strike("mk82", 2, ripple_s=0.5)     # its activity: the stores powered, then asking for consent
v.strike_status(a).phase                  # 'awaiting_consent'
v.strike_consent(a)                       # given: they leave half a second apart, weapons at their stations
w.weapons[0].store, w.weapons[0].station  # ('mk82', 3)
v.jettison(emergency=True)                # the rest, unarmed, at once
```

- **The stations:** each aircraft's store-station card lists its stations: number, name, place (a store's CG from the
  aircraft's, body axes), carriage (`rail`: a missile fires off it; `ejector`: the store is pushed clear, 3 m/s down;
  `rail+ejector`: a missile on its rail, anything else on its rack), the store classes it takes, its limit and
  capacity - each with its basis. A load a station cannot take is refused (`station_cannot_take`).
- **The stores:** a store card (`assets/stores/<id>.json`) is one store type, as a payload card is one payload type:
  its class, its flight model once released, whether it is powered before release (and its `power_up_s`,
  `power_down_s`) and can be jettisoned, its mass and dimensions with their sources.
- **Loadouts:** a fit names the loads its stations may take (`loadouts`), each with its basis. An aircraft starts
  empty; a scenario loads it (`load_loadout`, `load_store`) - the flight model carries the stores' mass from the next
  step (hangar writes each station as a point mass of the aircraft's, the platform sets it to the stores on it).
- **Strike** (A-GRA's Strike task, then its TaskCommand): a store type and a quantity - or the stores by index - and a
  target (for a guided weapon's seeker, MS-7d). The stations must operate (the master arm). Powerable stores power up;
  then the activity asks for release consent (A-GRA's StrikeConsentRequest) unless the command gives it (MA's weapons
  release authority answering itself); then the stores leave a ripple interval apart, each a weapon entity at its
  station (`launchWeapon`), its record naming the store, station, activity and target. The activity completes when the
  last has gone. Refused consent ends it `failed` (`consent_denied`); canceling it before its last release aborts the
  rest (they stay on their stations); the master arm off cancels it. A store in a weapon bay (its station's name
  says which: the stations whose names agree up to "bay" share its doors) waits for the bay's doors: they open in half
  a second, the store leaves, they are held open a second after the bay's last release and shut in eight tenths (all
  three engineering assumptions). `StoreInfo::bayDoors` is how far they are open (0 shut .. 1), for every store in
  the bay.
- **Jettison** (A-GRA's Jettison task): every jettisonable store, or those named, released unarmed (`jettisoned`).
  Standard: the stations powered (standby or operate), a powered store powered down first, the stores leaving 0.1 s
  apart. Emergency: at once, whatever the stations' state. A jettison takes its stores from any strike holding them.
- **The viewer:** the mission systems window lists the stores station by station, those gone dimmed; each carried
  store is drawn on its station, its pylon and launcher once one has been there, a bay's doors as they open
  ([viewer.md](viewer.md#stores-on-their-stations)).
- `examples/python/stores.py` shows it: an F-16C loads four Mk 82s, strikes two with consent given a few seconds
  later, and jettisons the other two.

**Data so far:** the F-16C's nine stations (the article's hardpoints, numbered left to right; their places derived from
hangar's design of it, their limits the figures commonly quoted, which no primary source found confirms - engineering
assumptions, said so on the card) and its loadout of four Mk 82s; the Mk 82's store card (its flight model JSBSim's
`mk82`). The missiles' cards and every aircraft's stations came with MS-7b (below), their flight models come with MS-7c.

**The missiles' cards (MS-7b).** The fighters' air-to-air missiles, 24 cards in `assets/stores` (class `aam`), each with
its seeker, guidance, motor and fuze in words and its numbers with their sources. One method for every nation's: each
figure from the missile's English Wikipedia revision first; an official page where one gives the fitted variant's
figure (the US services' fact sheets: the European, Russian and Chinese makers publish none we could read); then the
article in the maker country's language (Russian, Chinese, French) where the English one lacks it; what none gives, an
engineering assumption, its reasoning in its note. `+` derived, `*` assumed:

| Missile | Seeker | Mass kg | Length m | Diameter m | Range km | Speed Mach |
| --- | --- | --- | --- | --- | --- | --- |
| AIM-120C-7 AMRAAM | active radar | 161.5 | 3.66 | 0.178 | 90 | 4.0 |
| AIM-120D AMRAAM | active radar | 161.5 | 3.66 | 0.178 | 145 | 4.0 |
| AIM-7M Sparrow | semi active radar | 225.0 | 3.64 | 0.203 | 70 | 4.0+ |
| AIM-9M Sidewinder | infrared | 85.5 | 2.87 | 0.127 | 35 | 2.5 |
| AIM-9X Sidewinder | imaging infrared | 84.0 | 3.02 | 0.127 | 35* | 2.5* |
| AIM-132 ASRAAM | imaging infrared | 88.0 | 2.90 | 0.166 | 25 | 3.0 |
| IRIS-T | imaging infrared | 87.4 | 2.94 | 0.127 | 25 | 3.0 |
| R.550 Magic 2 | infrared | 89.0 | 2.72 | 0.157 | 20 | 2.0 |
| Meteor | active radar | 190.0 | 3.65 | 0.178 | 200 | 4.0 |
| MICA EM | active radar | 112.0 | 3.10 | 0.160 | 80 | 4.0 |
| MICA IR | imaging infrared | 112.0 | 3.10 | 0.160 | 60 | 4.0 |
| PL-10 | imaging infrared | 89.0 | 3.00 | 0.160* | 20 | 3.0* |
| PL-12 | active radar | 180.0 | 3.93 | 0.203 | 85 | 4.0 |
| PL-15 | active radar | 210.0 | 4.00 | 0.203 | 175 | 5.0 |
| PL-8 | infrared | 115.0 | 2.95 | 0.160 | 20 | 3.5 |
| R-27ER | semi active radar | 350.0 | 4.78+ | 0.260 | 130 | 4.5 |
| R-27ET | infrared | 348.0 | 4.49+ | 0.260 | 120 | 4.5 |
| R-27R | semi active radar | 253.0 | 4.08 | 0.230 | 73 | 4.5 |
| R-27T | infrared | 253.0+ | 3.79+ | 0.230 | 40 | 4.5 |
| R-37M | active radar | 510.0 | 4.06 | 0.380 | 350 | 6.0 |
| R-60M | infrared | 45.0 | 2.13+ | 0.120 | 8 | 2.5 |
| R-73 | infrared | 105.0 | 2.93 | 0.165 | 30 | 2.5 |
| R-77M | active radar | 190.0* | 3.71* | 0.200 | 190 | 4.0 |
| Super 530D | semi active radar | 270.0 | 3.80 | 0.263 | 40 | 4.6 |

- Derived: the R-27 variants' lengths from the Russian article's table, which gives the basic and extended-range
  missiles' lengths without tying them to a seeker (read against the English article's R-27R); the R-27T's mass, which
  the table gives without a seeker; the R-60M's length (the R-60's plus the 42 mm the article states); the AIM-7M's
  speed (the AIM-7F's, the same motor); the AIM-9M's warhead (the AIM-9X's: the same, the USAF fact sheet says).
- Assumed: the AIM-9X's range and speed (classified: the family's); the PL-10's diameter and speed (its class's); the
  R-77M's mass, length and warhead (the R-77-1's: no source read gives the R-77M's).
- Every missile's power-up and power-down time: 1 s, assumed (no source gives any).
- Their flight models come with MS-7c (until then `flyable` is false: they load, and a strike of them is refused
  `store_not_flyable`); the seekers, guidance and fuzes with MS-7d.

**Every fighter's stations and loadouts (MS-7b).** Each of the 16 other fighters with a store-station payload has its
card's stations - its hardpoints as the English article counts them, a multi-rail pylon's rails each a station of its
own (the F-15C's nine hardpoints are eleven stations) - and one air-to-air loadout of its fitted variant's missiles,
from the article's armament or a sentence naming a typical load:

| Aircraft | Stations | Loadout | Stores |
| --- | ---: | --- | --- |
| A-10C | 11 | self-defence | 2 AIM-9M |
| EA-18G | 9 | self-protection | 2 AIM-120C |
| F-15C | 11 | air superiority | 6 AIM-120C, 2 AIM-9M |
| F-16C | 9 | air superiority; mk82 x4 | 4 AIM-120C, 2 AIM-9X; 4 Mk 82 |
| F-22A | 12 | air superiority | 6 AIM-120C, 2 AIM-9X |
| F-35A | 10 | internal air-to-air | 4 AIM-120C |
| F/A-18C | 9 | air-to-air | 4 AIM-120C, 2 AIM-7M, 2 AIM-9M |
| Gripen | 8 | air-to-air | 4 Meteor, 2 IRIS-T |
| J-10A | 11 | air-to-air | 2 PL-12, 2 PL-8 |
| J-20A | 10 | internal air-to-air | 4 PL-15, 2 PL-10 |
| MiG-29A | 7 | air-to-air | 2 R-27R, 4 R-73 |
| Mirage 2000 | 9 | air defence | 2 Super 530D, 2 Magic 2 |
| Rafale | 14 | air-to-air | 2 MICA EM, 2 MICA IR |
| Su-25 | 11 | self-defence | 2 R-60M |
| Su-27S | 10 | air-to-air | 2 R-27R, 2 R-73 |
| Su-57 | 12 | internal air-to-air | 4 R-77M |
| Typhoon | 13 | air-to-air | 4 AIM-120C, 2 ASRAAM |

- **Estimates, marked as such:** no public source gives a station's place or its limit, so every station's are
  estimates (basis `assumed`, the rule in its note): its place from hangar's design of the aircraft by one rule for
  every type (wing pylons evenly from 25 % to 80 % of the semi-span, 40 % of the local chord aft of the leading edge,
  0.6 m below the wing; wingtip rails 0.15 m beyond the tip; fuselage and bay stations at offsets from the CG), its
  limit by its kind. Where an article gives the load but not the stations, or a mix but not the counts, which station
  carries what and how many are estimates too, said so in the loadout's note.
- **The flight models carry them:** hangar writes each station as a point mass of the aircraft (`station N`, weight 0
  until a store is on it), so every fighter's XML was rebuilt; each flies its 60 s manoeuvring test to the bit as
  before with its stations empty (its states' hash and its mass compared, old XML against new, all 16).
- The fleet test loads each aircraft's every loadout and checks the stations, the stores and the mass they add.

**External fuel tanks** (`external tanks`, a loadout of fourteen of them: A-10C, EA-18G, F-15C, F-16C, F-22A,
F/A-18C, Gripen, J-10A, MiG-29A, Mirage 2000, Rafale, Su-25, Su-57, Typhoon; 19 tank cards in `assets/stores`, class
`tank`). Three have none, on purpose: no tank is certified or has flown on the F-35A (studies, and a 2026 Israeli
development contract); the J-20A's four are photographed but their capacity is analysts' guesses threefold apart; the
Su-27S carries none. A tank is loaded full. Its fuel is part of its mass (`StoreInfo::massKg`; `StoreInfo::fuelKg` what is
left). As the engines burn, the aircraft draws it into its own tanks, from every tank alike, at up to 10 kg/s (an
estimate): its own stay topped up, and it is lighter by the fuel burnt alone. A tank jettisoned flies on as a JSBSim
vehicle of its own (`aircraft/stores/<card id without hyphens>/`, written by `python -m hangar storemodels`: a slender
body's aerodynamics - estimates), with the fuel that was left in it. Unfinned, it tumbles: its body's unstable
moment swings it broadside (143 deg from the air within 6 s of leaving an F-16; a finned Mk 82 stays within 6 deg), and
broadside its drag is tens of kN (61 m below and 77 m behind the aircraft 3 s after). Its drawn model is
`assets/models/<name>.glb` (`python -m hangar store`). One rule sets every card:
- the capacity is the published one, and the fuel 0.80 kg/L of it;
- the empty mass is published, else derived from published figures, else 0.10 kg per litre (the mean of the three
  published dry masses per litre); a forum's or a game's figure is noted, never used;
- the size is published, else 8 diameters long with the fuel 0.68 of the cylinder (the four published tanks' means).
Where a station's assumed limit was below the full tank seen on it, the limit was raised to that tank (its note says
so); the F-22A's is FAS's 5,000 lb.

**Pods** (`pods`, a loadout of six of them; five pod cards, class `pod`): the AN/ALQ-184 (F-16C centreline), the
AN/ALQ-131 and AN/AAQ-28 LITENING (A-10C), the AN/ASQ-228 ATFLIR (F/A-18C, on its left fuselage station), the
LITENING on the Gripen and the Typhoon, the Damocles (Rafale). A pod is a store: its mass on its station, its drag and
its sideslip moment on the aircraft, and jettisoned it flies on as a tank does (its model by the same writer). What
it senses or jams is its payload's, not built with it yet. Its mass and size are the published ones; a figure seen
only as a search result's quote is `assumed` and says so; a boxy pod's diameter is the circle's of its section's
area. Not carded, for want of public figures or of a station on our models: the LANTIRN and Sniper pods (the F-16's
chin stations are not modelled), the HARM targeting pod, the J-10A's and the Su-25's pods, the Sorbtsiya. The
EA-18G's ALQ-99 pods are part of its airframe's model.

**No flight models for the air-to-air missiles.** Their cards have no `model`, so a strike with them is refused
(`store_not_flyable`). Building them - their motors, their powered flight - was stopped by the assistant's safety
filter on 2026-10-10 and has not been taken up again. Their mass, their drag and their sideslip figures in carriage
are on their cards like every other store's.

**What leaves an aircraft keeps the physics whole** (sim::FlightModel's mass change). A store released or jettisoned,
a section parting, a tank's fuel drawn: the airframe stays where it is while its CG moves to what is left, and its
velocity becomes v + w x d. The departing part's inertia comes off the base. The launcher takes the ejector's
reaction. A store or piece starts at the velocity of its own point (v + w x r) with the parent's body rates. A
blast's push and its moment about the CG act on the airframe. tests/test_mass_change.cpp measures all of it.

**Their drag on the aircraft** (`WorldOptions::storeDrag`, `store_drag`: on by default). Each store pushes against
the airflow at its station, so its moment about the CG comes with it: a store below the CG pitches the nose down, and a
load on one wing alone yaws the aircraft toward it. A store adds its own drag while it is on. Its carriage adds its own
from the first store onward, and stays after a release, as the drawn pylon does. Its CD S (m^2) below Mach 0.8, by
one rule for every aircraft:
- the store's own: its card's `drag_area_m2`, an isolated build-up from the geometry hangar holds for it
  (`python -m hangar storedrag`: Hoerner's body of revolution, its base, its fins' friction; Re 1e7 per metre). A
  tank is about 0.065 on its frontal area, the Mk 82 0.13 (DSTO-TR-0554 measured 0.150), a missile 0.2 - 0.45. The
  same figure is a jettisoned tank's axial drag in its own flight model;
- under a pylon, interference 1.0 (a single pylon store flew at its free drag: AGARD-CP-373; NASA's Shrout & Kelly
  1966), its lugs 0.007 (DSTO-TR-0554), the pylon and rack 0.024 (the F-16's pylon drag indices 7, 8 and 11);
- on a rail, the rail 0.0068 (RAND's F-15C LAU-128), none on a wingtip; in a fuselage well (an F-15's AIM-120), the
  missile's own alone; in a bay, nothing;
- a pod's as a tank's, its rounded tail taken as a blunt base (too steep for the air to follow).

Above Mach 0.8 it rises as the Mk 82's measured drag does (DSTO-TR-0554: 1.28 times at Mach 1.0, 2.43 at 1.2).
Compared with RAND MR-1216-AF's drag indices, not fitted to them (tests/test_store_drag.cpp):

| Load | Ours | Published |
| --- | ---: | ---: |
| F-16C, 2 x 370 US gal tanks | 39.2 | 35.0 |
| F-15C, 3 x 610 US gal tanks | 34.0 | 33.1 |
| F-15C, 4 x AIM-120 in the wells | 5.3 | 5.2 |
| F-16C, ALQ-184 on its centreline pylon | 22.4 | 29 (18 + 11) |

The drag slows the aircraft by its drag over its mass, to 2 %. The pod is low by a quarter: a body of revolution has
none of a jamming pod's antennas, scoop or slab sides.

**In a sideslip** a store also turns and pushes the aircraft, at its station (the same switch):
- its body's unstable moment, q x `munk_volume_m3` x sin(2 beta): Munk's couple (NACA Report 184) - its volume times
  Lamb's k2 - k1 for its fineness (0.916 at 8). It turns the nose away from the wind: why tanks cost directional
  stability. Three tanks take 0.029 per radian (0.0005 per degree) off an F-16C's Cn-beta. No published figure for
  an F-16 or F-15 was found to hold that to; the reports found are in words (a centreline tank reduces it; one
  model's small stores cost a third of it at Mach 2). Munk's is the perfect fluid's moment, an upper figure;
- its fins' and base's side force, q x `side_slope_m2` x sin(beta), `side_arm_m` behind the station: one fin pair's
  Helmbold slope carried onto the body (NACA Report 1307), slender-body theory's base. In carriage at half the card's
  figure, as carried fins measure (NASA TN D-3582).

A missile in a fuselage well adds neither. A jettisoned tank or pod flies with the same figures in pitch and yaw.

**Not yet:** the stores' effect in pitch (under a wing or fuselage the flow is held along the surface: no figure to
give it); a pylon's own side force; a pylon store's earlier drag rise (NACA RM L7K20); multiple racks' larger
interference; open bay doors' drag (nothing published for any stealth fighter; cavity papers give ratios, not a
coefficient to use); hung stores and release failures; release
envelopes (a store released outside its carriage limits); multiple racks' release sequences.

## Signal level (MS-6)

A radar can search at the signal level instead: each dwell through the samples its receiver would give and the
processing its processor would do, the ground's clutter among them. The same search, one flag:

```python
v.amti_search("radar", "range_while_search", azimuth_half_width_rad=math.radians(30), signal_level=True)
v.radar_status("radar").signal_level                       # True
```

In C++: `AmtiCommand::signalLevel`, `RadarStatus::signalLevel`. In C (ABI 1.55): `fsim_amti_command.signal_level` and
`fsim_radar_status.signal_level`, read and written where the caller's `struct_size` has them. Medium-PRF modes only
(range-while-search, track-while-scan, surveillance); velocity search at the signal level is refused
(`invalid_parameter`). Its detections and tracks come through the same calls as the detection level's.

**The chain** (`src/ms/radar/Signal`, the platform's own: ADR-30's MS-6 amended):
- **The waveform:** a dwell split into 8 coherent intervals, one per PRF, the same design for every radar in speed:
  blind speeds 150 to 300 m/s, geometric (an X-band radar's PRFs 10 to 20 kHz, an S-band one's a third of that). The
  transmitter on 10 % of the time (the cards' class duty), the gates its range resolution apart, those while it
  transmits blanked.
- **The samples, in the Doppler filters directly:** receiver noise as white complex Gaussian convolved with the
  window's DFT taps (a 4-term Blackman-Harris: what an FFT of windowed pulses gives, exactly, without the FFT), each
  gate hearing the share of it its matched window does (the last gates' windows reach into the next transmission); a
  target as the window's transform at its folded Doppler, in the gates its folded range straddles, each gate's response
  what of its chips' overlap the receiver hears (the straddle and the eclipse together, gate by gate), its amplitude
  drawn per interval from its fluctuation; the ground's clutter as Gaussian power in each gate and filter (below).
- **The processor:** cell-averaging CFAR along range (16 cells, 2 guard), and where the ground's clutter is, a
  clutter-map threshold too (the cell's expected power times -ln p: the design false-alarm probability however the
  clutter varies); the main beam's clutter notched (the card's notch about its range rate) and the altitude line
  blanked; a cell detected that is the largest of its neighbours; the PRFs' 3 of 8, each interval's detection unfolded
  to every range and rate it may be and counted where three agree within a cell, each detection resolving one target
  at most (no ghosts from a target's own aliases); monopulse angles from the difference channels.
- **The threshold:** designed for the same false-alarm time as the detection level's: the M of N's agreements
  counted exactly (19 ways an axis for 3 detections within a cell), the gates lost while transmitting, the share of
  noise's crossings that are peaks (from the window's correlation, given the crossing). Noise alone gives the
  designed rate.
- **One calibration for both levels:** the detection level's SNR is the calibrated one, its losses lumped (processing
  among them). The chain is given back its mean processing loss for its waveform and the target's fluctuation, against
  the detection level as it detects (at its own interval threshold), and takes it look by look - straddle,
  scalloping, CFAR, eclipses. The estimate (semi-analytic) against the chain's own at Pd 0.5, 4,000 dwells a point:
  8.10 against 8.20 dB for a Rayleigh target, 7.87 against 7.89 dB for a dominant scatterer, 7.40 against 7.34 dB for a
  steady one. In free space the two levels then detect alike on average over where a target's range and Doppler fold:
  a 3 m² fighter (a dominant scatterer) at the APG-68's 50 % range, 40 ranges across its longest PRI and their speeds
  across a blind speed - the beam staring, 36.0 % of frames at the detection level and 40.3 % at the signal level;
  scanning +-4 deg, 18.3 % and 20.4 %; 98 % of the signal level's resolved within three gates. At any one range they
  differ, as medium PRF does: a range none of its PRFs eclipses is seen more often than the average, one that falls in
  several eclipses rarely (at 92.4 km, 1 frame in 120 against the detection level's 23).

**The ground's clutter:** range rings on a sphere of 4/3 the Earth's radius from the altitude to the horizon (or 400
km), a gate wide near the aircraft and 1.5 % of their range beyond (a ring's power falls as R^-4); each ring's
patches every 7.5 deg and, in the main beam's footprint, a quarter beamwidth apart; each patch through the radar
equation with its two-way gain (the antenna's pattern: its main lobe, near side lobes, the card's floor in front and
back level behind) and constant-gamma reflectivity, its power spread over the Doppler it spans and folded into every
interval's gates and filters. Clutter changes how the radar behaves, as a medium-PRF radar's does:
- Looking level from 5 km at a fighter head on 60 km out: detected in 4 % of frames over land, 77 % over the sea, all
  in free space; no false alarms from the clutter.
- Blind zones: over land, the frames detecting it fall to none at ranges that fold into several PRFs' eclipses or
  their altitude lines together, and recover between them.
- Looking down: a target hovering in the main beam's clutter is notched (0 frames); one closing at 500 m/s there is
  seen in every frame.

**Assumptions** (engineering, not sourced values): the ground's gamma, -15 dB for land and -40 dB for the sea at 10
GHz (+5 log10(f / 10 GHz)) - the values MATLAB's `surfacegamma` gives farmland (after Nathanson et al.) and sea state 3
(after Long), the primaries not read - constant gamma has no specular term, so the altitude line (the ground straight
below) is as faint as the clutter about it, where a real one is bright; land or sea by the terrain's height (at or
below 0: sea) under the aircraft and in the main beam's footprint, the ground flat at the height below the aircraft (no
relief or masking in the clutter); the antennas' side-lobe floor -5 dBi and back level -15 dBi where the cards give none
(all of them so far).

**Against FERS (MS-6c).** [FERS](https://github.com/davidbits/FERS) (GPL-2.0, run as a separate program by
`tools/fers/crosscheck.py`, nothing of it linked) forms the samples of one 10 GHz interval - 10 kHz PRF, 32 pulses, a 1
m² target moving along the line of sight, receiver noise at 290 K - with a plain 1 us pulse and a Barker-13 code of 1 us
chips. A receiver's processing (blanked while transmitting, matched filtered at the target's Doppler, a gate a chip,
Blackman-Harris and the DFT across the pulses) makes its cells, each over kTB through the same processing
(`tests/data/signal/fers_crosscheck.json`); `tests/test_signal.cpp` gives the chain the same target, its SNR from the
same radar equation, and compares:
- FERS's echo power is the radar equation's (within 0.6 %), its noise kTB's (a raw sample 0.998, a processed cell 1.002
  and 1.007), the neighbouring filters' noise correlated -0.817 and -0.823 (the window's -0.816), the last gate's
  noise 0.98 and 0.075 of the rest (the chain's share: 1 and 1/13).
- The cells about the target, worst over each case: on a gate and a filter 0.006 dB; half a gate and half a filter off
  0.08 dB; opening, folded five times 0.22 dB (one cell on the window's skirt; the rest within 0.1); eclipsed at the
  PRI's end 0.08 dB; folded six times in range and four in Doppler 0.08 dB. The Barker code's two straddled gates
  within 0.12 dB.
- Found by it, and fixed: the chain had eclipsed an echo as a whole (its straddle weight times its received share), 4.4
  dB too weak where it straddles the gate before the next transmission - now each gate's response is what of its own
  window the receiver hears; the last gates now hear less of the noise.
- Differences the chain keeps, recorded: a code's range sidelobes (Barker-13's 1/13 a lag, more for a code cut short by
  an eclipse); a phase code's compression not matched to the Doppler (at 23 kHz the 13 us code loses 1.4 dB and its
  neighbouring gates rise to -10 dB: the chain takes a radar's compression as tolerant of Doppler); range migration
  within an interval (a percent or two of a gate's response pulse to pulse).
- FERS's own: it gives each pulse's echo to the receive window open when the pulse goes out (a radar's own windows
  lose its echoes from beyond the first unambiguous range: the scene has a transmitter and, beside it, a receiver
  whose one window spans every pulse); it renders a pulse in whole samples from its fractional start and drops the last
  of some (the pulse file is padded with zero samples).

**The range-Doppler display (MS-6c).** The viewer draws a radar at the signal level's range-Doppler map beside its
B-scope or PPI: its last dwell's first interval (its lowest PRF), range up and closing speed across, both folded, each
cell's power over the noise (0.25 dB steps to 64 dB, binned to 128 x 64 by the largest), the processor's peaks
circled, the main beam's notch and the altitude line's blanking outlined. The world publishes it with the radar's
record (shared-memory layout 5). `examples/python/radar.py --signal-level` shows it.

**Not yet:** the difference channels' clutter (a clutter-dominated cell's angles are its noise's); a guard channel;
velocity search.

**Cost:** a dwell in noise 0.25 ms (the APG-68's 25 ms dwells: about 1 % of real time per radar; MS-6c's per-gate
response and noise share 1.1 % over MS-6b's, medians of 12 interleaved runs; the display's snapshot nothing
measurable); with the ground's clutter 2-3 ms (its 55 ms mechanical-scan dwells: 4-6 %), the clutter's map 1.1-2.2 ms
of it. Low over land on the physics terrain - the testbench's 600 m over San Francisco Bay - 4.5 % of real time for
the F-16C's radar, 5.6 % for the E-3G's (`fsim_radar_bench`, tests/bench_radar.cpp: the wall time against the same
world with the radar off, and a digest of every detection and track). It was 21.5 % and 11 %: 85 % of it was the
clutter map's ground under the beam, ~200 points a dwell, each through the vehicles' terrain query, which made a missing
level's whole tile from its ancestor for each new point (37 us) and pushed their own tiles out of the cache. The map now
reads them with `GroundProvider::pointHeightAboveEllipsoidM`: the same heights to the bit, from the four texels that
tile would hold (the digests unchanged; tests/test_terrain.cpp compares 4,000 points). A fluctuation's processing loss is worked out the first time a target of it is seen (about a millisecond for a
Rayleigh one, some tens for the others) and kept while the threshold stays. The detection level is untouched.

## The fleet's payloads

Every aircraft has a fit (`aircraft/<type>/payloads.json`), one rule for all of them, whoever made the aircraft:

- **Fitted where a source names it** on the variant the fit is: the aircraft's own Wikipedia article, the payload's
  own article naming the aircraft, or an official source - the operator, its government's record, the maker. Each
  entry quotes its evidence, checked against the source's text at the revision cited (2026-10-06 and 2026-10-07).
- **Described but not named:** a system a source says the aircraft carries without naming its model is fitted under a
  designation that says so, "... (model not named)", its card saying what the source does.
- **Nothing on assumption.** A system no source found says is fitted is not fitted, and the fit's note says what is
  missing.

Pods that hang on a station (targeting, jamming, IRST, data-link pods) join with the stores at MS-7. The exception is
the EA-18G's ALQ-99s: jamming is that aircraft's role.

| Aircraft | Payloads |
| --- | --- |
| A-10C | GAU-8/A Avenger; 11 stations; A-10C Link 16 radio (model not named); A-10C chaff and flare dispensers (model not named) |
| B-52H | AN/ALQ-172(V); AN/ALR-46; AN/ALR-20A; AN/ALE-20 ×12; AN/ALE-24 ×8; 3 stations; AN/ALQ-117 Pave Mint; AN/ALQ-122; AN/ALQ-153; AN/ALQ-155; AN/ALT-16; AN/ALT-28; AN/ALT-32; AN/ASQ-151 EVS; AN/APQ-166; AN/ARC-210 Warrior |
| C-130J | AN/ALR-56M; AN/APN-241 |
| C-17A | AN/APS-133(V); AN/ALE-47; C-17A missile warning system (model not named) |
| C172 | none (a civil light aircraft, no mission payloads) |
| Crazyflie 2.0 | none (a research nano-quadrotor, no mission payloads) |
| E-3G | AN/APY-2; E-3 ESM; JTIDS Class 2H |
| E-7A | MESA; E-7 ESM; E-7A countermeasures (model not named) |
| EA-18G | AN/APG-79; AN/ALQ-218; AN/ALQ-99 ×3; 9 stations |
| EC-130H | Compass Call |
| F-15C | AN/APG-63(V)3; AN/ALR-56C; AN/ALQ-135; AN/ALE-45; M61A1 Vulcan; 9 stations; AN/ALQ-128 EWWS; AN/ASN-109 |
| F-16C | AN/APG-68(V)9; AN/ALR-56M; AN/ALE-47; M61A1 Vulcan; 9 stations; AN/ALQ-213 |
| F-22A | AN/APG-77; AN/ALR-94; AN/AAR-56; M61A2 Vulcan; 8 stations; IFDL; MIDS-JTRS; F-22A flare dispensers (model not named) ×2 |
| F-35A | AN/APG-81; AN/ASQ-239 Barracuda; AN/AAQ-40 EOTS; AN/AAQ-37 DAS; GAU-22/A; 10 stations; AN/ASQ-242 CNI |
| F/A-18C | AN/APG-73; AN/ALR-67(V)3; AN/ALQ-165 ASPJ; M61A1 Vulcan; 9 stations |
| Gripen (JAS 39C) | PS-05/A; Mauser BK-27; 8 stations; JAS 39C Link 16 terminal (model not named); JAS 39C electronic warfare suite (model not named); JAS 39C missile approach warning system (model not named) |
| H-6K | 6 stations; H-6K nose radar (model not named); H-6K electro-optical targeting system (model not named) |
| IRIS+ | none (a consumer quadrotor, no mission payloads; its camera gimbal is an accessory, not fitted) |
| J-10A | Type 1473; GSh-23; 11 stations |
| J-20A | Type 1475 (KLJ-5); 6 stations; EOTS-86; EORD-31; J-20A distributed electro-optical sensors (model not named) |
| KC-135R | KC-135R radar (Pacer CRAG; model not named) |
| KC-46A | AN/ALR-69A; AN/AAQ-24(V) LAIRCM; AN/AAR-54 ×6; MAGR 2K |
| MiG-29A | N019 Sapfir-29; OEPS-29; SPO-15 Beryoza; GSh-30-1; 7 stations |
| Mirage 2000C | Thomson-CSF RDI; Serval; Sabre; Spirale; DEFA 554 ×2; 9 stations |
| Rafale C | RBE2 AA; SPECTRA; OSF (Optronique Secteur Frontal); GIAT 30/M791; 14 stations |
| RC-135W | Rivet Joint |
| RQ-4B (Block 20) | EISS SAR; EISS EO/IR |
| Skua | none (a hypothetical UAV; its design chooses no mission payload) |
| Su-25 | Klen-PS; SPO-15 Beryoza; ASO-2V; GSh-30-2; 11 stations |
| Su-27S | N001 Myech; OEPS-27; SPO-15 Beryoza; GSh-30-1; 10 stations |
| Su-57 | N036 Byelka; L402 Himalayas; 101KS Atoll; GSh-30-1; 12 stations |
| Typhoon (Tranche 2) | Euroradar Captor-M; PIRATE; Praetorian DASS; Mauser BK-27; 13 stations; MIDS |
| U-2S | ASARS-2; SYERS-2C; U-2S data links (model not named) |
| UH-1H | M60D ×2 |
| UH-60A | M60D ×2; 4 stations; AN/ALE-47 |

What is still not fitted, and why:

- **Not named in a source found:** the A-10C's radar warning receiver; the C-130J's and C-17A's missile warning and
  dispensers beyond those listed; the KC-135R's defensive systems; the UH-60A's radar warning receiver and infrared
  jammer. They join when a source names them.
- **IFF** (the F-15C's AN/APX-119, the IFF inside the F-35A's CNI and the MIDS-JTRS): ADR-30's families have no IFF
  payload yet, though A-GRA has an IFF capability.
- **Navigation** is fitted only where a source names the receiver (the KC-46A's MAGR 2K, the F-15C's AN/ASN-109).
- **Ground-mapping radars** (the B-52H's AN/APQ-166, the H-6K's nose radar) are fitted as `ground_map_radar`: A-GRA
  has no capability for a real-beam ground map or terrain avoidance, so they offer their status only.
- **Upgrades not on the variant fitted:** the B-52H's AN/APQ-188 and B-52J changes, the F-16C's AN/ALR-69A(V) and
  AN/ALQ-257, the RQ-4B's AN/ALR-89 suite (developed for it, not confirmed fitted), the Finnish F/A-18C's Link 16.
- **No mission payloads:** the C172, the Crazyflie 2.0 and the IRIS+; the Skua's design chooses none.

The cards hold identity and sources only so far. Their parameters (power, aperture, bands, published ranges, each
with its uncertainty and basis) are filled stage by stage: the radars at MS-4, the missiles at MS-7.
