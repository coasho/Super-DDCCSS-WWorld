# ADR-30: Mission Systems — high-fidelity payloads behind A-GRA's Mission Systems interface

| | |
| --- | --- |
| Status | Accepted 2026-10-06: the owner approved it with one change - no ground entities (D7) - and asked for implementation in the order of section 8. Transport feasibility was checked (section 3.3) |
| Extends | ADR-28 ([vehicle-interface.md](vehicle-interface.md)) and ADR-29 ([flight-autonomy.md](flight-autonomy.md)): Mission Systems capabilities follow the same pattern in the SDK as the VI's flight capabilities (discovery, commands, command status, activities, grants), and ADR-29's transport items XPT-01 to XPT-08 come into scope (stage MS-12) |
| Scope | Every payload family that A-GRA's Mission Systems (MS) interface defines - sensors (AMTI radar, ESM/RWR, passive optical, weather radar, SAR/SMTI where the aircraft carries them), weapons (stores, launch zones, release consent, fire, abort, jettison, flyout), electronic attack, communications and datalinks, track fusion, PNT, health and BIT - modelled at high fidelity, per aircraft from the public specifications of the payloads it actually carries, commanded through the SDK (C++, C ABI, Python) and, later, over OMS |
| Reference | From public repositories, read for semantics, none in this repository: A-GRA ASK 6.0a (open-arsenal/a-gra: *MS L1 Interface Volume* v6.0a, *A-GRA_MessageDefinitions_v6_0_a.xsd*); OMS 2.5 (open-arsenal/oms: the OMS Standard, the Language-Agnostic CAL specification OMSC-SPC-013); UCI 2.6 (open-arsenal/uci) |
| Related | [sdk/control.md](sdk/control.md), [hangar.md](hangar.md), [design document](FlightSim_System_Architecture_and_Design.md) |

## 1. Context

### 1.1 The goal (the owner, 2026-10-06)

- Implement all of the OMS-related payload modules - radar, missiles and the other mission-system payloads. They work like the VI: the SDK sends MS commands to control them.
- Every payload at a high-fidelity level, built on mature open-source solutions found and studied first, not on simplified versions written from scratch.
- Each vehicle's payloads follow the publicly available real-world specifications of the payloads it actually carries.

### 1.2 What the MS interface asks for

The MS volume groups 67 interactions into ten packages. Its minimum message set has 128 messages: 99 UCI 2.3 messages and 29 A-GRA extensions. "Core" binds every implementation; the other Mission Use Cases (MUCs) bind an implementation that has that capability.

| Package | Interactions | MUCs | What it needs underneath |
| --- | ---: | --- | --- |
| Sensor/Vehicle Tasking | 11 | AMTI, ESM, PO, Weather Radar, Core | Air radar, ESM receivers, EO/IR sensors, weather radar; their capabilities, commands, states and activities |
| Weapon Employment | 13 | Core, Store Management | Stores inventory and descriptions, launch zones (DLZ, WEZ), release consent, fire, abort, jettison, weapon flyout and weapon entities |
| Status Service | 5 | Core, PO | Capability heartbeat (34 messages: every capability's status), queries, health, status and BIT |
| Data Fusion | 2 | Core | Single-sensor tracks, fused tracks, entities |
| Communications Service | 14 | Core, Additional Comms | Comm terminals, gateways, network endpoints, link state |
| Decentralized Messaging | 10 | Core | Wrapped messages to and from other A-GRA systems over the links |
| COP | 4 | Core, AMTI, PO, Store Management | Sensor and weapon status for the common operating picture |
| Mission Plan Behaviors | 2 | Core | Plan validation from the MS side |
| PNT Service | 1 | PNT | Navigation solution and its quality |
| Contingencies | 5 | Core | Sensor, payload, comms and logging failures; malformed messages |

### 1.3 Terms

| Term | Meaning |
| --- | --- |
| Payload | One modelled mission system on an aircraft: a radar, an ESM receiver, a targeting pod, a store station and what it carries, a datalink terminal |
| Payload model | The physics that makes a payload behave: its emissions, detections, measurements, kinematics |
| Detection level | A sensor modelled per look: signal-to-noise from the physics, detection probability and measurement errors from it. No waveform samples |
| Signal level | A sensor modelled through its waveform and processing chain (samples, matched filter, Doppler, CFAR) |
| Payload card | The data for one real payload on one aircraft type: every parameter with its public source and uncertainty (section 6) |

## 2. Decisions

### 2.1 Taken (the owner, 2026-10-06)

| | Decision |
| --- | --- |
| D1 | Scope: all MS payload families, commanded through the SDK as the VI is; OMS transport after the payloads |
| D2 | Fidelity: high, built on mature open-source models where they exist; where none exists the gap is stated and the model implemented from the published method, validated against published results (section 5) |
| D3 | Per aircraft: each type carries its real payloads, with parameters from public specifications |
| D4 | GPL components may be linked. The first one linked makes the distributed package GPL-3.0: `LICENSE` changes from MIT, and every dependency's licence is checked for compatibility at that stage (section 9) |
| D5 | Sensors: detection level by default; signal level selectable per sensor. The radar's signal level is the platform's own chain (amended 2026-10-07: RadarSimPy's engine is closed, see MS-6), cross-checked against an open-source signal-level simulator (FERS) run beside it |
| D6 | Order: the transport and status core, then the AMTI radar and air-to-air weapons end to end; then ESM/RWR and EA, passive optical, comms and datalinks, weather radar, PNT (section 8) |
| D7 | No ground entities: no SAM sites, ships or emitters on the ground. Every sensor's targets and emitters are the world's aircraft and the weapons they release; the terrain and the sea are clutter, masking and the ground air-to-ground weapons fall on |

### 2.2 Approved with this record

| | Proposal |
| --- | --- |
| P1 | The payload engine runs inside the world, stepped with it and deterministic; payload commands take effect between steps as VI commands do |
| P2 | The SDK's MS surface mirrors the VI's: capability discovery with support states, commands with command status, activities with progress, grants per capability (section 4) |
| P3 | No classified, export-controlled or restricted data, ever: public sources only, each cited on its payload card (section 6) |
| P4 | Gaps are stated, not hidden. Where public data does not pin a parameter, the card gives the physical derivation, the assumption and its uncertainty |

## 3. What exists

### 3.1 In the platform

- **Entities:** every vehicle in a world, with its full state. There are no weapons in flight and no emitters.
- **Physics:** JSBSim (LGPL-2.1) for flight; the hangar's geometry pipeline (tools/hangar), with each aircraft's 3D model; terrain from the map set; Bruneton's atmosphere; the exhaust's Planck emission (world/Exhaust.cpp).
- **SDK:** the VI pattern and its machinery (ADR-28/29): capability host, commands, statuses, activities, grants, and C ABI and Python parity.

### 3.2 Mature open-source components studied

| Domain | Component | Licence | Fidelity, and what it covers | Use |
| --- | --- | --- | --- | --- |
| Whole-system reference | **MIXR** (successor to OpenEaagles), C++ | LGPL-2.1+ | Training-simulator level. Radar range equation with antenna gain tables, an integration gain, jamming noise; RWR, jammer, IR sensor, IR seeker, SAR, missiles, stores manager, datalink; RCS by simple shapes and az/el tables. No Swerling statistics, clutter, multipath or propagation | Structural reference for the payload object model and the emission and query flow; not the fidelity source |
| RF propagation, air-to-air and air-to-ground | **NTIA P.528** (the US reference implementation of ITU-R P.528) | US Government work, public domain | High: line of sight, diffraction and troposcatter loss with atmospheric absorption, 100 MHz-30 GHz | Every RF path: radar, ESM, jamming, datalinks |
| Terrain propagation | **NTIA ITM** (Longley-Rice) | Public domain | High, 20 MHz-20 GHz over terrain profiles | Low-altitude and ground links, terrain diffraction |
| Clutter loss | **NTIA P.2108** | Public domain | ITU-R recommendation | Ground terminals |
| Tracking and fusion | **Stone Soup** (Dstl, Five Eyes), Python | MIT | High: Kalman/EKF/UKF/particle filters, IMM, GNN, JPDA, MHT, PHD, data association, metrics | Trackers (ported to C++ for the world's step, cross-checked against Stone Soup on the same data) |
| Signal-level radar | **RadarSimPy** (Python layer GPL-3.0; its C++ engine `radarsimcpp` a closed binary, the free trial limited to 2 targets, one transmit and one receive channel and 8 meshes) | GPL-3.0 wrapper, commercial engine | Signal level: transmitter, receiver, waveform, baseband samples from point targets and meshes, range-Doppler, CFAR, DOA | Not used: closed (found at MS-6, 2026-10-07) |
| Signal-level radar | **FERS**, C++ | GPL-2.0 | Signal level, multistatic: pulsed stable, CW, FMCW in alpha | The cross-check of the platform's own signal-level chain, run beside it (not linked) |
| Weapon flyout | **JSBSim** | LGPL-2.1 | 6-DOF with tables, already in the platform | Missile and bomb flight |
| Motors, rocket aerodynamics | **RocketPy** | MIT | 6-DOF rocket flight; motor models (solid, thrust curves); Barrowman aerodynamics | Motor models; cross-check of missile kinematics |
| Missile aerodynamics | **SU2** (CFD) | LGPL-2.1 | High: Euler/RANS | Offline aero tables per missile, from its public external geometry |
| Missile aerodynamics | **OpenVSP / VSPAERO** | NASA Open Source Agreement | Panel and vortex-lattice methods | Fast tables; cross-check of SU2 |
| GNSS | **gps-sdr-sim** | MIT | Real ephemerides, signal generation | PNT: satellite geometry, DOP, outages |
| OMS transport | **sleet** (LA-CAL server, the Air Force's starter kit), Rust | Apache-2.0 | Reference OWP server with UCI schema validation | Reference server and test peer (MS-12) |
| OMS transport | **la-cal-harness**, Python | Apache-2.0 | LA-CAL compliance tests | Transport tests |

Not available, and why:
- **AFSIM** is not publicly released.
- **Xpatch** (shooting and bouncing rays RCS) and **MODTRAN** (atmospheric radiative transfer) are proprietary.
- **Missile DATCOM** is restricted.

### 3.3 Transport checked (2026-10-06)

`sleet` was built from source with the installed Rust and loaded *A-GRA_MessageDefinitions_v6_0_a.xsd* (860 global elements, 4,967 types). Two OWP clients completed INIT and INFO, subscribed, and published. A malformed `AMTI_Command` was refused with `-ERR Invalid-Message ... UnknownMember`. The bus is ready to carry the MS messages, schema-validated.

## 4. Architecture

```
SDK (C++ / C ABI / Python)            OMS bus (MS-12)
   │ MS commands, discovery,             │ OWP over WebSocket, OMS JSON,
   │ statuses, activities                │ A-GRA 6.0a messages
   ▼                                     ▼
 Mission Systems host  ◄──────────  OMS bridge (LA-CAL client, I/O thread,
   │ per vehicle: payloads,             queues; commands applied between steps)
   │ capability states, grants
   ▼
 Payload engine (stepped with the world, deterministic)
   ├─ Emitters and receivers: radar, ESM/RWR, jammers, datalinks
   │    └─ RF path: geometry, antenna patterns, P.528/ITM/P.676/P.838, terrain masking, multipath
   ├─ Signatures: RCS (GPU, from each model's mesh), IR (skin, plume, background)
   ├─ Electro-optical/IR sensors: atmosphere, detection, imagery (vision renderer)
   ├─ Trackers and fusion
   ├─ Stores and weapons: inventory, release, flyout (JSBSim), seekers, guidance, fuzing
   └─ PNT, health and BIT, failures
```

- **Determinism:**
  - Payloads step with the world at their own rates (a radar's dwell, a seeker's frame).
  - Random draws (Swerling fluctuation, noise) come from a seeded stream per payload.
  - The world's digests cover them.
- **Performance:**
  - Detection-level sensors are per look and per target, batched.
  - RCS and IR signatures are precomputed per aircraft type into tables over aspect, frequency and polarisation, then looked up.
  - Signal level is per sensor, on request, and not real-time.
- **The SDK surface (P2):**
  - **Discovery and status:** capability ids under `fsim.ms.*` (`fsim.ms.amti`, `fsim.ms.strike`, `fsim.ms.store_management` ...), support states, availability and reasons, as ADR-29 section 4 defines them.
  - **Commands and activities:** commands carry the A-GRA command's content as native types, and activities report its progress and results (detections, tracks, weapon state).
  - **Grants:** per capability, as ADR-28's VI-7.
- **Weapons in flight are entities.**
  - Each is a JSBSim instance with its seeker and guidance, a player the sensors see and the viewer draws.
  - Its kill assessment comes from fuze, warhead and target geometry.

## 5. Fidelity per domain

| Domain | Detection level (default) | Signal level (selectable) | Validation |
| --- | --- | --- | --- |
| AMTI radar | Per look: the radar range equation with the card's power, aperture, frequency, losses and noise figure; antenna patterns; Swerling 0-4 fluctuation (Shnidman's detection equations); CFAR and straddle losses; propagation (P.528, gaseous P.676, rain P.838); multipath and terrain masking; main-lobe and side-lobe clutter for look-down (land and sea clutter models from the published literature); Doppler notch and blind speeds; scan patterns and dwell timing; modes (RWS, TWS, STT, ACM); range, angle and Doppler errors from SNR; ECCM against jamming | The platform's own chain (MS-6): each coherent interval's baseband samples (sum and monopulse difference channels; targets from their RCS tables and the radio path, receiver noise, the ground's clutter as scatterers), Doppler filtering, CFAR, the PRFs' M of N resolving range and Doppler, monopulse angles; cross-checked against FERS | Detection range against each radar's published range for a stated RCS; canonical cases against closed forms; detection level against signal level on the same scenarios |
| RCS | GPU shooting and bouncing rays (physical optics plus multiple-bounce geometric optics), implemented from the published method on Vulkan, over each aircraft's mesh, into tables over aspect, frequency and polarisation | - | No mature open-source SBR exists (section 3.2). Validated against closed forms (sphere, flat plate, dihedral, trihedral, cylinder) and against published RCS figures for aircraft where they exist |
| Air-to-air weapons | JSBSim 6-DOF; aero tables from SU2 and VSPAERO on the missile's public external geometry; motor from its public impulse, burn time and mass, through RocketPy's motor model; seekers (active and semi-active radar through the radar model; IR through the IR model); guidance (proportional navigation and its augmented forms, midcourse with datalink updates, loft); proximity fuze and warhead lethality; DLZ and WEZ by flying the model (Rmax, Rne, Rmin over aspect and altitude) | - | Range, time of flight and no-escape range against published figures; flyout against RocketPy's for the motor phase |
| ESM/RWR | One-way equation from every emitter in the world (they are the simulated radars), receiver sensitivity, band coverage, direction-finding accuracy, pulse parameters from the emitter's card | - | Detection ranges against published sensitivities |
| Electronic attack | Noise and deception jamming: jamming-to-signal ratio, burn-through range, side-lobe effects on the victim radar's SNR | In the signal-level chain's samples where selected | Burn-through ranges against the closed form |
| Passive optical (FLIR, IRST, targeting pods) | Target signature (skin from aerodynamic heating and sun, plume from the exhaust model's Planck emission), atmospheric transmission in the sensor's band, sensor NETD and detection criteria (Johnson), track | Imagery from the vision renderer in the sensor's band | Detection ranges against published figures; plume radiance against published measurements |
| Weather radar | Reflectivity from the clouds' water content (the viewer's cloud fields), attenuation | - | Reflectivity against the Z-LWC relations |
| Comms and datalinks | Link budgets over P.528/ITM, antenna patterns, data rates, latency, jamming | - | Link ranges against published figures |
| Tracking and fusion | Stone Soup's algorithms (ported): EKF/UKF, IMM, GNN, JPDA; track quality and continuity | - | Against Stone Soup on the same measurements |
| PNT | GNSS geometry from real ephemerides (gps-sdr-sim), DOP, INS drift from the published error models, outages and jamming | - | Against published accuracies |

## 6. Payload cards (D3, P3, P4)

Each payload of each aircraft type has a card: one file per payload and type, with a citation and an uncertainty for every parameter. These are the hangar's rules for airframes, applied to payloads.

| Field group | Examples |
| --- | --- |
| Identity | Designation, band, role, aircraft and stations |
| Transmitter and receiver | Peak and average power, frequency and agility, PRFs, pulse widths, duty, noise figure, losses |
| Antenna | Aperture size, type (mechanical, PESA, AESA), gain, beamwidths, side lobes, field of regard, scan rates |
| Processing | Modes, integration, CFAR, track capacity, update rates |
| Published performance | Detection range against a stated RCS, the conditions, and the source. These are the validation targets, not inputs |
| Weapons | Mass, dimensions, fins, motor (impulse, burn time, propellant mass), seeker, guidance, fuze, warhead mass, published range |

- **Payload families by aircraft type.** The F-16C carries the AN/APG-68(V)9 (or APG-83), AIM-120, AIM-9, Sniper or LITENING, ALQ-131/184 and Link 16. The E-3G carries the AN/APY-2. The EA-18G carries ALQ-99 and ALQ-249. The RQ-4B's sensors, and so on.
- **What is public.** Public data varies widely:
  - Some types have detailed open literature and fact sheets: the US types, the Gripen and the Rafale.
  - Some have very little: the J-20, the Su-57, the PL-15.
  - Where a parameter is not published, the card derives it from what is (a known aperture and band give the beamwidth). It states the assumption and widens the uncertainty.
  - The fleet report lists each card's confidence.
- **The first cards (MS-4, MS-7):** the fire-control radars and air-to-air missiles of the fighters (F-15C, F-16C, F-22A, F-35A, F/A-18C, EA-18G, Gripen, Rafale, Typhoon, Mirage 2000, J-10A, J-20A, MiG-29A, Su-27S, Su-57), and the AEW radars (E-3G, E-7A).

## 7. Verification

- **Determinism:** the world's digests with payloads, stepped twice from the same seed, bit-identical.
- **Physics:** each model's closed-form cases (section 5) in ctest.
- **Published performance:** every card's published figures reproduced within its stated uncertainty, run as a fleet test like ADR-29's.
- **SDK:** every MS command on every aircraft that has the capability, through C++, the C ABI and Python; discovery honest (nothing advertised that does not work), as ADR-29.
- **Transport (MS-12):** la-cal-harness against the bridge; round trips through sleet with schema validation on every message.
- **Performance:** a 4-ship-against-4 air-to-air scenario with radars, RWRs and missiles holding real time, measured per payload family.

## 8. Stages

| Stage | What | Depends on |
| --- | --- | --- |
| MS-1 | Mission Systems host: `fsim.ms.*` capabilities, discovery, support states, commands, command status, activities, grants, C ABI and Python; payload cards' format and loader | - |
| MS-2 | Entity and emission core: weapons in flight as players (no ground entities, D7); the RF path (geometry, antenna patterns, NTIA P.528 and ITM, P.676, P.838, terrain masking, multipath) | MS-1 |
| MS-3 | RCS engine: GPU shooting and bouncing rays over the hangar's meshes into tables; canonical validation | MS-2 |
| MS-4 | AMTI radar, detection level: modes, scan, Swerling, clutter, errors; the fighters' and AEW radars' cards; published ranges reproduced | MS-2, MS-3 |
| MS-5 | Trackers and fusion: Stone Soup's algorithms ported, cross-checked | MS-4 |
| MS-6 | Signal-level radar option: the platform's own chain (amended 2026-10-07, below), cross-checked against FERS | MS-4 |
| MS-7 | Stores and weapons: inventory per station, release, jettison, abort, consent; air-to-air missiles (aero tables from SU2 and VSPAERO, motors, seekers, guidance, fuzes, lethality) on JSBSim; the missiles' cards | MS-2, MS-4 |
| MS-8 | Launch zones: DLZ and WEZ from the flyout model; published ranges reproduced | MS-7 |
| MS-9 | ESM/RWR and electronic attack | MS-2, MS-4 |
| MS-10 | Passive optical: IR signatures, atmosphere, IRST and targeting pods; imagery through the vision renderer | MS-2 |
| MS-11 | Comms and datalinks, the Decentralized Messaging Service, PNT, health and BIT, contingencies, weather radar; air-to-ground weapons | MS-2 |
| MS-12 | OMS bridge: LA-CAL client (OWP over WebSocket), A-GRA 6.0a messages generated from the schema, the MS minimum message set mapped onto the SDK, ServiceStatus and SubsystemStatus heartbeats; sleet and la-cal-harness in the tests. Closes ADR-29's XPT-01 to XPT-08 | MS-1 ... |

### 8.1 Progress

- **MS-1 done (2026-10-06).**
  - **Code:** `src/ms` (the host, the cards and fits loader, the names). `include/fsim/MissionSystems.h` and
    `Vehicle`'s payload calls. C ABI 1.49. Python's `payloads`, `set_payload_state`, `payload_bit`,
    `mission_activities` and the related calls ([sdk/mission-systems.md](sdk/mission-systems.md)).
  - **Data:** fits for 26 aircraft and 85 identity cards. Each payload in a fit quotes the passage of its cited source
    that names it (Wikipedia revisions for all aircraft except the F-16C). Those passages were checked against the
    cached revision text when the fits were written.
  - **Tests:**
    - `fsim_tests "[ms]"`: every card and fit in the tree reads clean; card validation; the F-16C's discovery; the
      state machine (warm-up, BIT, cancel, supersede, failure); grants through the World.
    - `python/tests/test_mission_systems.py`.
  - **Not fitted yet:**
    - No fit: C-130J, KC-135R, KC-46A, UH-1H, UH-60A. Their cited sources name no payload.
    - Missing from a fit: the H-6K's radar, the Gripen's EW suite and the C-17A's defensive systems.
    - The B-52H's APQ-166 has no capability yet.
- **MS-2 done (2026-10-06).**
  - **The radio path** (`src/ms/rf`):
    - NTIA's P.528 and ITM reference implementations vendored (`third_party/ntia`, public domain, the changes stated
      there). P.676's layers are tabulated per frequency: a path costs 120-260 us instead of ~10 ms (9-27 us since
      MS-4's performance work, below), within 0.001 dB
      of the exact computation.
    - P.526-15's delta-Bullington adds the terrain's excess over a profile of the world's ground; P.838-3 adds rain.
    - Multipath in line of sight is P.528's ground reflection.
    - Antenna patterns come from aperture illuminations (uniform, cosine, Taylor, circular), tabulated in sine
      space, with electronic steering, scan loss, a side-lobe floor and a back lobe.
  - **Weapons in flight as entities:**
    - `launchWeapon` releases a store at its station with its launcher's velocity, attitude and an ejection push.
    - The store is flown by its own model, its flight capabilities `restricted`. It ends at impact, and the world
      keeps its record.
    - The Mk 82 is the store until MS-7: its flight model and its viewer model built from its design (`hangar mk82`; JSBSim's own `mk82` until 2026-10-10).
    - Through C++, the C ABI (1.50) and Python, with `radioPath` between any two entities.
  - **Tests:**
    - `test_rf_path.cpp`: NTIA's examples, P.838's table, the apertures' closed forms, P.526's J(v), masking, rain,
      the layer tables against the exact computation.
    - `test_weapons.cpp`: the release, the fall and the impact, the record, and the same flight twice to the bit.
    - The C ABI test and `python/tests/test_weapons.py`.
  - **The emitter registry** (which payload transmits what, where) comes with the first emitters, the radars at
    MS-4.
- **MS-3 done (2026-10-06).**
  - **The engine** (`tools/rcs`, `flightsim-rcs`): shooting and bouncing rays on the GPU (Vulkan ray queries
    through VSG).
    - Rays: 8 tubes to a wavelength each way, through up to 5 reflections off a perfect conductor. Each hit radiates
      its physical-optics current back to the radar, integrated over its footprint.
    - Curved skin: the facets read as the curved skin they stand for (interpolated normals; the Phong surface at
      alpha 0.5, which follows a circle to second order).
    - Cavities (ducts, engine faces, nozzle interiors): diffuse, sigma = 4 rho A.
    - Gear retracted as the viewer retracts it.
    - Validated against the closed forms of a sphere, a plate (and its side lobe), a cylinder, a dihedral and a
      trihedral, within 0.15 dB (`ctest -R rcs`).
  - **The tables** (`aircraft/<type>/rcs.bin`):
    - L, S, X and Ku bands; 3 deg bins, 9 sub-aspects each.
    - Each bin holds the scattering matrix's second moments and the fluctuation ratio.
    - Signature cards: each aircraft's documented low-observable features, quoted from its sources (its own
      article and the S-duct article, the same kinds for every type), with one set of treatment values for all
      (`assets/signature/treatments.json`: a RAM coating, a concealed engine face, a lined duct, a treated exhaust,
      each derived from independent studies). Nothing is fitted to a claimed RCS. (A first version fitted the stealth
      types to whatever figure each had: manufacturers' best cases for some, a patent's intent or an adversary's
      estimate for others. It was replaced, because that mixed claims of different kinds by origin.)
    - The figures reported for each type, with who reported them, are compared, never fitted (`test_rcs_table.cpp`, docs/sdk/mission-systems.md).
  - **At run time:** `World::radarCrossSection(observer, target, query)` reads the target's table at the aspect and
    polarisation between them. C ABI 1.51; Python `World.radar_cross_section`.
  - **Not modelled:** edge diffraction (PTD/MEC), travelling and creeping waves, the cockpit interior. The stealth
    types' tables carry this, with their finer shaping, in a wide uncertainty.
  - **Performance (2026-10-07, the owner's rule: within each stage, fidelity kept).** A fighter's table took ~5
    minutes, the fleet ~11 hours, and held the GPU. Measured on an F-16C slice (3 elevation rows, 4 bands):
    - Kept: the mesh in the GPU's own memory (every hit reads it), and dispatches sized by their tubes (a band's
      thousands of small aspects each waited on a fence). 13.6 -> 5.9 s, the scattering identical to the reference at
      every aspect (`flightsim-rcs --elevations ... --dump`, compared value by value).
    - Measured and dropped: one trace for all four bands (the paths are frequency-independent, but four bands' PO at
      every Ku-band hit cost more than the L/S/X traversals saved: 6.4 s); mirror symmetry (no hangar mesh is its own
      mirror image - each side is meshed apart, a millimetre off, 0.7 rad of phase at Ku band - so the mirrored half
      would not be this mesh's answer); a coverage mask skipping the two thirds of tubes that cannot strike (exact,
      but those misses were already cheap); fast-trace build flags (no change).
    - Where the time is now: the primary rays' traversal, ~70 % (shading ~3 %, the reflected rays ~25 %). The next
      step would be the first bounce by rasterisation (as GRECO does), estimated ~1.6x more.
- **MS-4 done (2026-10-06).**
  - **The radar** (`src/ms/radar`): detection level. The scan (electronic steps, mechanical sweeps, rotodomes), the
    radar equation with the antenna's steered pattern, the target's RCS table and the radio path's losses, pulse-Doppler
    clutter (the main beam's notch, the side lobes' band), the PRF schedule's M of N, exact single-look Pd for steady,
    Rayleigh and dominant-scatterer targets, and measurement errors that shrink with the SNR. Seeded per vehicle:
    deterministic.
  - **The cards:** the 17 radar cards carry their parameters with the same class method for every country (power by
    transmitter class, noise figure by era, side-lobe clutter by antenna type). Every published range is a check; each
    class has one calibration, its mean residual. The remaining scatter is ±10 dB
    ([sdk/mission-systems.md](sdk/mission-systems.md#air-search-radars-ms-4)).
  - **The SDK:** the AMTI capability is `supported` for every radar its card describes. `Vehicle::submit(AmtiCommand)`,
    `radarStatus`, `amtiDetections`; C ABI 1.52; Python `amti_search`, `radar_status`, `amti_detections`.
  - **The viewer** (the owner's requirement, 2026-10-06): the followed vehicle's payloads and each radar's display (a
    B-scope, or a PPI for a rotodome) in windows that collapse and hide (`p`). They are drawn from a per-vehicle Mission
    record in the world's shared memory (layout 3), written by the SDK's own instances. Externally visible payloads
    are drawn by the airframes' models (the E-3G's rotodome, the E-7A's top hat, the EA-18G's pods, the Typhoon's tip
    pods, the EC-130H's and RC-135W's antennas). The EA-18G's fit now counts the three ALQ-99 pods its model carries.
    Stores on stations come with MS-7.
  - **The emitter registry:** the radars searching are the hosts' (`Host::radar`, `RadarStatus.searching`). ESM reads
    them at MS-9.
  - **Tests:** `test_radar.cpp` covers the Pd closed forms (13.2 / 17.3 / 21.1 dB at Pd 0.9), M of N, the radar
    equation, every card against its published ranges, clutter, the World round trip with determinism, and the shared
    memory. Also the C ABI test and `python/tests/test_radar.py`.
  - **Not modelled yet:** signal level (MS-6), ECM (MS-9), weather and the sea's clutter as a field
    (only its Doppler regions are), multipath on the target, ambiguities beyond what M of N leaves.
  - **Performance (the owner, 2026-10-07: severe and unstable; solved within the stage, fidelity kept).** Measured first:
    the radars' cost was the radio path (96 %); its spikes were the terrain.
    - *The terrain:* the offline map set holds level 9-12 where the physics asks level 15. Each missing tile was made
      from its ancestor by decoding that ancestor again (a level-10 tile decoded once for each of its 1,024
      descendants) on the simulation thread, and a 300 km profile touched ~300 of them, evicting the tiles the
      vehicles stand on. Now each source tile is decoded once (`TerrainTiles`' source cache), and a profile reads the
      deepest level at each point directly, a tile at a time (`GroundProvider::profile`). A level-of-detail reading was
      tried and rejected: 0.73 dB at the 95th percentile against the old reading's own 0.16 dB noise. The native
      reading is within that noise (0.02 dB p95, `test_rf_terrain_profile.cpp`). Cold, a path costs 1.6 ms (was 33
      ms).
    - *P.528:* the terminals' grazing ray traces (~90 % of it) are tabulated per frequency in sqrt(h), filled as asked:
      9-27 us a path (was 110), within 0.016 dB of the exact against its own 0.11 dB jitter for a centimetre.
    - *The pair:* reciprocity shares a path between two radars looking at each other; the frequency's tables are
      built when the radar is.
    - *The profile and Bullington:* Mercator coordinates exactly at nodes 32 points apart, cubic Hermite with their
      analytic rates between (an error ~1e-15 rad); Bullington in one pass (slopes by cross-multiplication, the
      line-of-sight case skipped where a bound shows no loss), the smooth earth's in closed form (its grid maxima of
      concave functions): equal to the reference to 1e-9 dB on 3,000 random profiles.
    - *Parallel:* the radars look on the world's workers (`VehiclePool::forEach`), the same detections for any
      number of workers (`test_radar.cpp`).
    - *Measured* (step time, interleaved medians on a loaded machine): 3 radars, 6 aircraft, terrain - mean 0.85 ->
      0.25 ms, p99 23.4 -> 0.41 ms, max 58.6 -> 1.3 ms (no radars: 0.24 / 0.33 / 0.46). 32 radar fighters over the
      Sierra Nevada, terrain, 1 worker - mean 3.31 -> 1.68 ms, p99 13.0 -> 3.9 ms (no radars 1.31 / 2.06); 4 workers
      0.97 / 1.97 ms (no radars 0.74 / 0.91). Without terrain the radars cost ~5 %.
- **MS-5 done (2026-10-07).**
  - **The trackers** (`src/ms/track`): Stone Soup's components ported to C++ - the nearly-constant-velocity model, the
    radar measurement (elevation, bearing, range, range rate), EKF and UKF, the distance and PDA hypothesisers, GNN by
    SciPy's 2D assignment (Crouse's, step for step), JPDA (exact, factorised over clusters), the mixture tracker,
    initiators and deleters. `tools/track/crosscheck.py` runs Stone Soup 1.9.1 on the same data
    (`tests/data/track/crosscheck.json`): EKF 3.2e-12, UKF 6.8e-12, GNN 6.1e-12, JPDA 5.3e-12 (the largest relative
    difference at any step).
  - **Each radar's tracks:** GNN with the UKF, a dwell at a time, from the carrier's pose at the dwell; looks rather
    than steps (a track in the dwell's half-power beam is expected, within 1.5 beamwidths possible, the rest
    untouched); confirmed at 3 updates; dropped after 3 expected misses, or without an update for 3 frames and one
    more (UpdateTimeDeleter: a track the scan's volume left). Through the SDK (`amtiTracks`, C ABI 1.53, Python
    `amti_tracks`) and drawn on the radar displays (shared-memory layout 4).
  - **Found on the way, fixed:**
    - *The false alarms* came from a fixed 1e-6 a cell: an MPRF dwell's ~600,000 cells gave a fighter's radar 20
      false alarms a second, and the displays a field of them. The threshold is now designed for a false-alarm time,
      as a radar's is, 60 s on every card, each dwell's probability its share among its own cells. For the fighters'
      radars the figure is published: Stimson's "a minute or so" for radars in fighter aircraft (*Introduction to
      Airborne Radar*, 2nd ed., p. 144, the sentence verified in that edition by Google Books' full-text search; it
      was first taken from a forum's quotation and is now checked against the book). For the AEW radars it is an
      engineering assumption, and their cards say so. The class calibrations were taken again by the same method (tube -5.3 -> -3.5,
      GaAs +7.0 -> +8.9, AEW -5.8 -> -4.1 dB; the residuals within 0.1 dB of before). The calibration script had
      added the cards' calibration back with the wrong sign, harmless only while the cards had none.
    - *Duplicate tracks* on the E-3G: an EKF's first update after a rotation landed outside its own covariance (the
      range rate too curved for one linearisation with an unknown cross-range velocity), and the pass's next look
      began a second track. The UKF keeps one; replayed from the E-3G's own detections (`test_radar.cpp`).
    - *Tracks that left the scan* coasted for ever, their uncertainty growing (the A-10C's to 7 km): only a miss
      could delete them, and nothing looked there. The UpdateTimeDeleter drops them.
  - **Performance:** the tracker was 0.58 ms a step at 32 radars, from the held tracks' look predicate and the
    deleter's allocations; checking only the tracks a point was appended to, the deleter without allocating and the
    lists compacted in place took it to ~0.01 ms. The UKF against the EKF, 32 radars over the Sierra Nevada on 4
    workers, interleaved medians: mean 0.843 / 0.851 ms, p99 1.568 / 1.568 ms - no cost.
  - **The frame rate (the owner, 2026-10-07: fluctuations still severe):** measured fullscreen (3840 x 2160) on the
    radar demo, the viewer held 160 fps (its vsync) with its GPU passes at 4-5 ms and ~10 % of a core, but dipped to
    120-140 fps for seconds at a time while the fleet's RCS tables were being made on the same GPU (96 % busy): its
    frames waited in the driver. `flightsim-rcs` now runs at the idle GPU scheduling class and below-normal CPU
    priority: over the sea (4 ms of GPU a frame) the viewer then held 160 fps beside a KC-135R's table, but over land
    (5.2 ms) still 128-157, since a dispatch of a large aircraft's high band holds the GPU for about a second. So
    between dispatches the tool also waits while a flightsim viewer runs (`--no-yield` to share instead): beside a
    C-17A's table, the viewer held 160.0 fps every second of 30 with no late frame, the table waiting 32 s. Its
    tables are the same byte for byte (the CF2's, made by both builds). `--stats` prints each second's worst frame and
    its late frames.
  - **The fleet's first nine tables** (A-10C, B-52H, C-130J, C-172, C-17A, CF2, E-3G, E-7A, EA-18G) were made by the
    tool before the treatments (f694843) and the faster engine (a044eb2): untreated aircraft, so the same physics,
    but an older provenance. They are made again: the C-172's agrees with its old table to float rounding (worst
    7.6e-7 relative in its 29,280 bins - the engine's partial sums grouped differently).
  - **Deferred:** an IMM (Stone Soup 1.9.1 has none to check a port against); fusion across sensors (it needs a second
    sensor or a data link: MS-9 to MS-11).
- **MS-6a and MS-6b done (2026-10-07): the radar at the signal level** (`src/ms/radar/Signal`; the SDK's
  `signal_level`, C ABI 1.55; [sdk/mission-systems.md](sdk/mission-systems.md#signal-level-ms-6)).
  - The chain as amended above: Doppler-domain samples (noise through the window's taps, targets in closed form, the
    ground's clutter as Gaussian power), CA-CFAR and a clutter-map threshold, the main beam's notch and the altitude
    line, peak picking, 3 of 8 with each detection resolving one target, monopulse. Its threshold designed for the
    detection level's false-alarm time (the agreements counted exactly, the blanked gates, the peaks' share); its
    calibration the detection level's, its mean processing loss given back (8.0 dB semi-analytic, 8.4 dB measured).
  - Checked: one interval against CA-CFAR's closed form for a Rayleigh target; noise's peaks against their computed
    share; a dwell's false alarms against the design; a target through every PRF's ambiguities (35 dB: range to 6 m,
    rate to 0.15 m/s, no ghosts); free space against the detection level (19.7 % of frames against 17.7 %); the
    ground's clutter: no false alarms from it, the main beam's notch, land worse than sea, the blind zones medium PRF
    has.
  - Found on the way: the coincidence test missed agreements whose centre held no detection (fixed: every cell within
    the tolerance is a centre); its threshold had counted the agreements as (2 tol + 1)^(2m) (exact count now); a
    patch's power as a spectral line made a comb CFAR fired on (spread over its Doppler now); CA-CFAR alone gave ~2,000
    false alarms a 100 frames over land (the clutter map's threshold: none).
  - Cost: 0.27 ms a dwell in noise, 2-3 ms with the ground's clutter (the map 1.1-2.2 ms after linear gains, shared
    patch edges and skipping what is 30 dB below the noise).
  - Assumptions, said so: constant gamma (land -15, sea -40 dB at 10 GHz), the antennas' side-lobe floor and back
    level where the cards give none, flat ground at the height below.
- **MS-6c done (2026-10-07): FERS's cross-check and the range-Doppler display**
  ([sdk/mission-systems.md](sdk/mission-systems.md#signal-level-ms-6)).
  - FERS built from source beside the platform (GPL-2.0, not linked; its dependencies from MSYS2, owner-approved) and
    run by `tools/fers/crosscheck.py` on one interval, a plain pulse and a Barker-13 code: its samples through a
    receiver's processing against the chain's cells. Echo power, noise, the window's correlation, straddle, scalloping,
    folding in range and Doppler, the eclipse: within 0.12 dB (one skirt cell 0.22 dB).
  - The cross-check found the chain's eclipse wrong where an echo straddles the gate before the next transmission (4.4
    dB): each gate's response is now what of its own matched window the receiver hears, and the last gates hear that
    share of the noise. Recalibrating after it found two more: the processing loss is now per fluctuation, and against
    the detection level's own interval threshold (which counts the PRFs' false agreements as C(n, m) alone). Both
    levels now detect alike averaged over where a target folds (36.0 % and 40.3 % of frames staring, 18.3 % and 20.4 %
    scanning), where one range had been compared before.
  - The chain keeps: a code's range sidelobes, compression tolerant of Doppler, no range migration within an interval.
  - The viewer's range-Doppler map (shared-memory layout 5), checked in fullscreen.
- **The fleet's fits filled in (2026-10-07).** Every aircraft has a fit: 139 payloads where 96 were, on 31 aircraft
  (the C172, Crazyflie 2.0, IRIS+ and Skua carry none, and their fits say so). One rule for every aircraft: fitted
  where a source names it on the variant fitted - its own article, the payload's article naming it, or an official
  source (the KC-46A's from the Congressional Record's arms-sale notification, the C-130J's from BAE's and Northrop
  Grumman's pages) - each quote checked in the source's text; described but unnamed systems under designations that
  say "model not named"; nothing on assumption. New: the C-130J, KC-135R, KC-46A, UH-1H and UH-60A fits; the B-52H's
  whole avionics list; data links (F-22A, F-35A, Typhoon, Gripen, A-10C, U-2S); the J-20A's EOTS-86 and EORD-31.
  A `ground_map_radar` family (C ABI 1.54) carries the B-52H's AN/APQ-166 and the H-6K's radar with their status
  only: A-GRA has no capability for a real-beam ground map. Still missing, and why: [sdk/mission-systems.md](
  sdk/mission-systems.md#the-fleets-payloads) (IFF has no payload family yet).

### MS-6 amended (2026-10-07)

RadarSimPy, the signal-level simulator MS-6 was to link, turned out to be a GPL-3.0 Python layer over a closed,
commercial engine (`radarsimcpp`, distributed as binaries; its free trial allows 2 targets, one transmit and one
receive channel and 8 meshes). The owner chose to build the chain here instead (2026-10-07):

- **What it is:** for each coherent interval of a dwell (one per PRF), the baseband samples a radar's receiver gives -
  range gates by pulses, in its sum and monopulse difference channels - from the targets (their RCS tables, the radio
  path, their fluctuation, range and Doppler folded by the PRF, eclipsed while it transmits), its receiver noise and
  the ground's clutter as scatterers; then its processing: Doppler filtering with a window, the main beam's clutter
  notch, CFAR at the threshold the false-alarm time sets, the PRFs' M of N resolving range and Doppler, monopulse
  angles. Its detections feed the same tracker.
- **Selection:** per radar, by its search command; detection level stays the default and costs nothing more.
- **Validation:** noise alone gives the designed false-alarm rate; Pd against SNR follows the detection level's exact
  forms plus the closed-form CFAR loss; measurement errors against their bounds; blind ranges and speeds where the
  PRFs put them; clutter against the detection level's notch and band; and FERS, run beside it on the same pulsed
  scenes, the received signal compared.
- **Performance:** measured per dwell in the stage, as every stage's.
- **Licensing:** nothing GPL is linked, so the platform stays MIT (section 9). FERS is a separate program the tests
  compare against.

- **MS-7a done (2026-10-07): stores management** ([sdk/mission-systems.md](sdk/mission-systems.md#stores-and-weapon-employment-ms-7a)).
  - Store-station cards list their stations (place, carriage, classes, limit, capacity, basis); store cards
    (`assets/stores`) the store types; fits their loadouts. An aircraft starts empty; a scenario loads it.
  - The host's inventory, strike (power-up, release consent asked for or given, releases a ripple apart as weapon
    entities that name their store, station, activity and target), abort, the master arm, standard and emergency
    jettison; C++, C ABI 1.56, Python; the viewer's stores list; `examples/python/stores.py`.
  - The stores' mass on the flight model: hangar writes each station as an empty point mass (the F-16C's rebuilt:
    its XML's only change), the platform sets each to the stores on it.
  - Data: the F-16C's nine stations (places derived from its design, limits assumed - no primary source confirms
    the commonly quoted figures) and a Mk 82 loadout; the Mk 82's card.
  - Tools for MS-7c built from source (owner, 2026-10-07): SU2 8.5.0 (TecIO off: it does not build with MinGW),
    VSPAERO (OpenVSP 3.54.0's solver alone), RocketPy 1.13.0 (pip). SU2 needs a volume mesh about each missile: a
    mesher is not among them yet.

- **MS-7b, the missiles' cards (2026-10-07; every aircraft's stations and loadouts next).** 24 air-to-air missiles, the
  fitted variants' by the fits' rule (the F-16C Block 52's AIM-9M/X, AIM-120C and AIM-7M - not the Python or IRIS-T of
  other users; the J-10A's PL-8 and PL-12, not the J-10C's; the MiG-29 9.12's R-27R/T, R-73, R-60M, not the later
  variants' R-77). Sources: the English Wikipedia revision, the US fact sheets, the maker country's Wikipedia; gaps
  derived or assumed and said so ([sdk/mission-systems.md](sdk/mission-systems.md#stores-and-weapon-employment-ms-7a)).
  A paywalled page (GlobalSecurity's AA-10 specifications, HTTP 402) was not used.
- **MS-7b, every fighter's stations and loadouts (2026-10-07).** The 16 other fighters' store-station cards (their
  hardpoints by the English article, a multi-rail pylon's rails each a station) and one air-to-air loadout each, from
  the article's armament. The stations' places and limits are estimates by one rule for every type, said so on each;
  so are a loadout's stations or counts where the article gives only the mix. Each fighter's XML rebuilt with its
  stations' point masses: its flight is unchanged to the bit, old XML against new, all 16
  ([sdk/mission-systems.md](sdk/mission-systems.md#stores-and-weapon-employment-ms-7a)).
- **MS-7e1, the missiles' models, and the first of MS-7e2 (2026-10-08).** All 24 missiles as hangar designs, written
  by one rule from one table ([hangar.md](hangar.md#missiles)): each card's length, diameter, span and mass held, its
  shape assumed from photographs and said so; its control surfaces moving (`fsim:` nodes), its motor's nozzle cut in
  and named for the viewer (`rocket` in the manifest), its paint and markings bands. The viewer draws a motor's plume
  from its thrust (the exhaust's jet as a solid rocket's: [sdk/viewer.md](sdk/viewer.md#effects)), and
  `fsim hangar showcase <missile>` shows one on its carrier's station and launched, on a scripted recording
  (`tools/showcase`) until it flies. Every other design's model re-meshed unchanged (the mesh bytes identical; three
  propeller aircraft's gear rotations differ by sign alone, as before - restored). The plume's look was
  held to launch photographs since (below).
- **MS-7e2, the smoke trail (2026-10-08).** A missile's motor lays smoke where it burns, drawn as the contrails'
  plume ([sdk/viewer.md](sdk/viewer.md#effects)): its primary smoke the propellant's particles (an aluminised motor's
  alumina dense, a reduced-smoke motor's residue a haze), its secondary smoke an ammonium perchlorate motor's HCl taking
  up the air's water where the air is humid enough; spread by CoCiP's diffusivities and shear (cocip::disperse),
  carried by the wind, gone at an optical depth of 0.02; a missile lays no aircraft's contrail. Held to the classes'
  physics in tests/test_contrail_observed.cpp; the secondary smoke's onset and condensed share are engineering
  estimates until a published secondary-smoke method (AGARD AR-287, STANAG 6016) is to hand.
- **MS-7e2, the propellants through NASA's equilibrium code (2026-10-09).** Each propellant class's chamber, exit and
  exhaust from CEA (through RocketCEA, installed apart: `tools/propellants`, its numbers in classes.json and
  world/Propellants.h, a test holding the one to the other), one composition a class for every nation's missiles: an
  aluminised motor's chamber 3,456 K, its exit 2,327 K at Mach 2.88 (its alumina freezing, gamma 1.135), a
  reduced-smoke motor's 2,962 K and 1,648 K at Mach 2.79; their exits 3.6 and 3.1 times sea level's pressure at full
  thrust (estimated 1.5 before); their exhaust's water once its hydrogen burns 0.29 and 0.33 kg per kg, their
  afterburning 5.8 and 2.3 MJ/kg, HCl 21 and 27 %, alumina 34 %. What changed with it: a reduced-smoke motor's ice
  contrail forms in air some 3 K warmer than a jet's, an aluminised motor's needs colder (the tropopause's on a typical
  day); the Sparrow's flame is as wide as its photographs'; a reduced-smoke motor's glow runs longer the higher it
  flies, at 6 km brighter at 1 L than the one AIM-120 photograph (1.39 against 1.23 times the sky; its altitude
  unknown) - left open. The minimum-smoke class stays an estimate (no design uses one; its ingredients are not in
  CEA's library).
- **MS-3, the bay fighters' RCS tables and every table held to its model (2026-10-09).** The F-22A's, F-35A's,
  J-20A's and Su-57's tables computed again from their models with the bays (MS-7e1), each bay shut under a cover the
  radar sees and the viewer does not (`fsim:cover:<bay>`, the skin over its whole opening): seen through the doors' 12
  mm gaps, the bay had been a cavity to the rays. The breakup sections' caps (MS-7e1) now have a material of their own
  that the RCS leaves out: read as cavity walls they had raised an F-16C's diffuse return 12 %. Every other table held
  to its current model by `flightsim-rcs --verify` against the method's own noise
  ([sdk/mission-systems.md](sdk/mission-systems.md#how-the-tables-are-made)): all 27 hold, their provenance recording
  the check. The published figures compared move by 0.3 dB at most (the Su-57's frontal); single bins of a stealth
  airframe move with its facetting by up to 7 dB (the F-22A's seen from above, where no bay is), their slices' means by
  0.1 dB.
- **MS-7e2, a humid day's trail in a replay (2026-10-09).** A recording now carries the world's environment as it
  changes (`E` records, format 4: World writes it, the viewer's replay applies it as a live world's - clouds, the air
  the contrails, smoke and vapour form in, the sun by its clock), so a replay's trails form in the recorded air; before,
  every replay was drawn on a typical day. `fsim hangar showcase <missile> --humidity 0.9` records a humid day: the
  AIM-120C's reduced smoke at 6 km, next to nothing on a typical day, lays a white trail of its HCl's droplets there
  ([sdk/viewer.md](sdk/viewer.md#effects)). Test: test_world (the environment recorded as it changes).
- **MS-7e2, the plume held to launch photographs (2026-10-09).** Four launch photographs (three AIM-7s, aluminised;
  an AIM-120, reduced smoke) measured along the missile's axis, and the viewer's plumes measured the same way
  ([sdk/viewer.md](sdk/viewer.md#a-missiles-plume-held-to-launch-photographs), `tools/plume_check`): the old plume was
  dark for half a missile length behind the nozzle (its core drawn as an ideal gas cooling as it expanded) and alike
  for every propellant. Now bright from the nozzle, an aluminised motor's a white flame saturated to 1.7 L (the
  photographs' 1.6 - 2.3) and as wide (4.1 diameters at 1 L, theirs 3.2 - 4.1), a reduced-smoke motor's a short white
  core fading into its trail's haze, as photographed. The propellants' temperatures are typical values until an
  equilibrium code gives each class's.
- **MS-7e2, the launch and the end at the ground (2026-10-09).** Four photographs at ignition (an AIM-9M off an
  F/A-18F's wingtip, an AIM-7M ejected from an F/A-18A, an AIM-9M off an F-22's trapeze, an AIM-120 off an F-14A)
  against the viewer's F/A-18C launches, frame by frame: no flash in any of them, and no smoke cloud apart from the
  trail - the smoke round the Hornet is its Sparrow's trail 0.05 s old, spread 5 - 6 m by the carrier's own flow (the
  F-15s' Sparrows' 1.7 - 2 m clear of them, as the viewer's), so the viewer adds none
  ([sdk/viewer.md](sdk/viewer.md#a-missiles-plume-held-to-launch-photographs)). Open: the carrier's flow on the young
  trail, the flame splashed on a launcher, the reduced-smoke glow at 1 L (now two photographs). To compare them the
  viewer saves screenshots on a replay's clock (`--screenshot-clock replay`): on the window's, a slow frame left the
  interpolator extrapolating and snapping back - a Sidewinder drawn ahead of its rail, then behind it. The missile's end
  at the ground: World already ends a weapon that strikes it (its vehicle removed that step); the showcase dives one
  there (`--dive-s`, the map set's terrain) - the viewer draws it to the strike and no further, its trail left in the
  air. What it does there is MS-7d's fuze and MS-7h's detonation.
- **MS-7e1, the vulnerable components (2026-10-09).** Every fixed-wing design's engines, fuel, crew and flight
  controls placed from its design by one rule ([hangar.md](hangar.md#the-3d-model), `hangar/vulnerability.py`): a
  turbofan by Raymer's statistical sizing from its thrust, bypass ratio and design Mach (within 40 % of five published
  engines; a shaft engine from its mass, an estimate), each tank its capacity of kerosene through its body's section or
  between its wing's spars (a long wing tank a chain of boxes), each crew payload's people seated, each control's
  actuator in the structure that drives it; then each fitted inside the airframe's solid as the mesher measures it -
  all 31 designs' components wholly inside it (within 1 cm), each on the breakup section it parts with. The manifest's
  `component` lines (the viewer reads them; MS-7g's presented areas, MS-7h's fires), outside the model's key: no mesh
  changed, the RCS tables and the flights as they were. Rotorcraft wait (engines, transmission, rotor). Found on the
  way: a design without bays carried an unused `cover` material in its model (the bay covers' change, MS-3) - made only
  with a cover now; the models committed as they are (a re-mesh drops it, byte for byte otherwise).
- **MS-7e1, the chaff and flare dispensers (2026-10-09).** Each type's dispensers placed from what is public
  ([hangar.md](hangar.md#the-3d-model), `hangar/dispensers.py`): 22 Commons photographs of the types releasing flares,
  then - the owner's call, against crude places - encyclopedias, the defence press, other games' documentation (War
  Thunder) and forums, each group's sources and their kind recorded. 19 types placed from sources, several with two
  groups (the A-10C's wingtips and gear pods, the Su-25's tail cone and nacelles, the Typhoon's flaperon fairings and
  outboard rails, the Rafale's flares on top and chaff on its sides, the Su-57's tail sting down and sideways); the
  Hornet's moved forward of its main gear, under its engine trunks, as The War Zone has it (a photograph had only
  shown them below it). The J-20A and H-6K, which nothing public places, keep one rule - an estimate, said so. A
  tanker, AEW or reconnaissance type nothing shows releasing flares carries none. The manifest's `dispenser` lines,
  which the viewer reads; the sources' counts kept for MS-9a's card.
- **MS-7e1, published engine sizes (2026-10-09).** The vulnerable components' engines sized from published figures -
  each engine's Wikipedia specifications, the same kind of source for every nation's; the WS-10's from a forum's quote
  of *Aviation World* and the AL-41F1's from a compiled table (their kind said); the AL-31F's and M53's maximum from
  their inlet (times 1.25, the median of the engines that give both) - in place of Raymer's statistical sizing, which
  was up to 40 % out (the M88's diameter 38 % too wide, the CFM56-7B 28 % too long, the D-30KP-2 42 % short of its length with its reverser).
  Only the R-195, which nothing public sizes, keeps Raymer's.
- **MS-7g's damage chain, G1 - G5 (2026-10-09).** A warhead's burst in the world and what it breaks, flown on:
  - **G1, the warheads:** each missile's card gives its warhead (`warhead`, `warhead_kind`: fragmentation or rod,
    `explosive`) and its charge (`explosive_kg`: War Thunder's datamine, the one source that gives every missile's -
    a game reference, said so; for the six it gives none, 37 %, the median of its shares; `gurney_kms`: each fill
    by its composition's nearest explosive in Cooper's table - estimates). The PL-10's, PL-12's and PL-15's
    warhead masses are analogues and the press's (estimates). ms/damage's one rule makes the rest from the card
    (estimates: no public source gives a modern missile's fragments): its case 85 % of what is not explosive, a
    cylinder 0.9 of the missile wide holding it all, its fragments cubes as thick as its case (rods as long as it),
    thrown at Gurney's cylinder velocity (a rod at 1.4 km/s at most, Talos's) into War Thunder's seven zones (a rod
    warhead's into a 10-degree belt) - the AIM-120C's 2,023 fragments of 5.4 g at 1.83 km/s.
  - **G2, the airframes:** hangar's `<type>.damage.json` for every fixed-wing design ([hangar.md](hangar.md)).
  - **G3, the impact** (`World::detonate`; ms/damage `assess`): each fragment's flight across the target in its
    attitude (thrown from the moving missile, met by the moving target, slowed by the air), through each skin it
    crosses by THOR's residual velocity (aluminium 2024-T3; TR-47's coefficients as Ball gives them - to check
    against the report's Table IV), into what it strikes behind; the blast on each panel by Kingery-Bulmash in free
    air (the surface fits at half the charge, its TNT equivalent by its Gurney energy), scaled for altitude by Sachs,
    reflected as far as a face looks at the burst - within 12 % of Kinney and Graham's fits from 4 scaled metres, 25 %
    close in.
  - **G4, the damage** (ms/damage `VehicleDamage`), each chance drawn from the vehicle's own damage stream: a fuel
    tank holed (a fast fragment's hole petalled five times its size by hydrodynamic ram) leaks by the orifice
    equation (the fuel's head under the load factor and the flow's suction) and catches fire with 1 - exp(-E / 16 kJ)
    per hit, less in thin air, throwing 5 % of its box's fuel out at once (at most 50 kg: the fuel fireball); an
    engine hit is out with 0.4 a hit (seized: no relight), on fire with 0.25 of that; a control actuator's hit loses
    its channel with 0.5 (its actuators stuck); a crew hit faster than 300 m/s incapacitates with 0.6 (the controls
    held); a skin panel tears off when the blast's impulse passes its plastic work (2024-T3's strength and
    elongation, 5 % of it straining), and a section's root weakens by what is torn and holed near it ((1 - f)^1.5,
    its compression side) - every number of these an estimate (no public source gives them for a modern airframe;
    Ball's method, its constants classified).
  - **G5, the forces and loads** (World's step): the damage takes off each section's lost share of the
    aerodynamic force at its centre of pressure, and adds the torn and holed skin's drag, through the world's force
    channel with any effect's; each section's root bending each step - its share of the aerodynamic force at its
    centre of pressure and its own mass's and its tanks' fuel's inertia (the specific force at the CG and the
    rotation's about it) - against its strength left; one that fails separates, its mass taken off its flight model
    (a negative point mass at its centre, JSBSim's own), its tanks' fuel with it. A reset undoes all of it.
  An AIM-120C bursting 4 m off a MiG-29A's left side: 550 of its fragments strike, its left engine is out, 18 panels
  of its left wing and tail are torn and its centre tank holed (some 9 kg/s); under its autopilot it rolls to -77
  degrees in 5 s (tests/test_damage.cpp).
  - **G6, the pieces** (the owner, 2026-10-09: detached objects JSBSim's too): hangar writes each section that can
    part its own flight model (`aircraft/<type>/pieces/`, pieces.py: its mass and inertia, a tumbling plate's or bluff
    body's aerodynamics, its centre of pressure walking aft with alpha, its rotation damped - estimates); a failed
    section flies on as a World vehicle (role Debris, no control), from where its centre was, as it moved there,
    taking its tanks' fuel and its stations' stores (Lost there: off the aircraft's mass) - a MiG-29A's wing leaving
    at 220 m/s pitches up, tumbles, climbs 280 m and falls at a plate's terminal 60 m/s; some 2 us a 120 Hz step
    (a MiG-29A with its control stack 7.5 us). A close burst's blast and fragments strike a section's bending mode:
    its root's peak moment the impulse's moment times the mode's frequency (from its tip bending 5 % of its reach at
    its ultimate load - an estimate), so a burst beside a wing breaks it, one astern only pushes the tail; densely
    holed skin loses what cracks link (1 - exp(-20 n a / A): an estimate).
  - **G7:** every event recorded as it happens; the showcase's world run (`--target`, its trials flying the target's
    own model to the burst within half a metre; `--miss-m`, `--seed`, `--separate` a scenario's injected failure).
  - **G8 begun:** the viewer hides a parted section, draws a piece as its section alone; a spill's fireball is five
    lobes along the path over 0.15 s; a fire burns at most 5 kg/s of its leak, its flame and section growing as the
    square root of what it burns, the rest a white fuel mist (100 um droplets, evaporating over 2 s - estimates).
  Open in G8: the mist and the trails' structure (streaky, ragged, not a smooth tube), the burst's fragment ring and
  sparks, lobed puffs, torn skin drawn.
  - **G9, what leaves an aircraft, physically whole (2026-10-09; the owner: detachable payloads and stores consistent
    across the simulation).**
    - A mass change keeps the airframe in place and its momentum (FlightModel's beginMassChange / endMassChange).
      JSBSim moves the CG and, in the air, would move the body with it: the state is nudged back, and the velocity
      becomes v + w x d plus the reaction over the mass. The part's own inertia comes off the base inertia.
    - A release's ejector pushes back on its launcher. A store starts at its station's velocity, v + w x r, with the
      launcher's rates. A section's piece carries its fuel and the stores still on it (point masses in its model).
      A blast's moment about the CG turns the airframe. The aerodynamic loss's moment is taken about the CG.
    - The F-16C's wingtip stations had been given to its tail section (breakup's tip reach).
    - External fuel tanks: 18 cards, one rule for their figures (capacity published; empty mass and size published,
      derived or by the fleet's means; forum and game figures only noted). A tank's fuel is part of its mass. The
      aircraft draws it in as it burns (10 kg/s at most, an estimate). Jettisoned, it flies on as a JSBSim body with
      what was left in it.
    - Measured (tests/test_mass_change.cpp):
      - a MiG-29A's wing parting conserves mass and momentum to 1e-5 and keeps the airframe within 0.1 mm;
      - an F-16's Mk 82 jettison moves the airframe 3.5 mm against a 6.1 cm CG shift;
      - its three tanks feed 3.2 kg in 30 s with its own kept full, the mass falling by the burn alone;
      - jettisoned, each tank weighs what it did on its station and is 52 m below and 13 m behind the aircraft 3 s later.
    - Left as JSBSim has it: the fuel burnt in the air moves the airframe by its CG's shift (every flight baseline
      kept).
  - **G10, the stores' drag on their aircraft (2026-10-10).** Each store and its carriage push against the airflow at
    their station, through the force channel, so their moments come with them. The store's own drag is an isolated
    build-up from its geometry (`hangar storedrag`, on its card); its lugs, pylon and rail come from measured figures
    (DSTO-TR-0554, AGARD-CP-373, RAND MR-1216-AF, the F-16's pylon drag indices); above Mach 0.8 it rises as the Mk 82's
    measured drag does. Against RAND's drag indices: F-16C 2 x 370 US gal 39.2 / 35, F-15C 3 x 610 US gal 34.0 / 33.1,
    F-15C 4 x AIM-120 in their wells 5.3 / 5.2. A jettisoned tank flies with the same isolated drag. Off with
    `storeDrag` (ABI 1.67 `store_drag`).
  - **G11, stores in a sideslip, the last tank, pods (2026-10-10).**
    - A carried store's body turns the nose from the wind (Munk's couple, on its card as `munk_volume_m3`); its fins
      and base push back behind its station (`side_slope_m2`, `side_arm_m`; half in carriage: NASA TN D-3582). Three
      tanks cost an F-16C 0.029 per radian of Cn-beta. A free tank takes the same figures: it tumbles (143 deg in
      6 s), a finned Mk 82 does not (6 deg).
    - The Su-57's PTB-2000. None for the F-35A (none certified) or the J-20A (capacity unknown).
    - Pods as stores: five cards (ALQ-184, ALQ-131, LITENING, ATFLIR, Damocles), a `pods` loadout on six aircraft,
      their own flight models once jettisoned. An ALQ-184 on an F-16C's centreline: 22.4 against RAND's 29.
    - Every tank and pod of every loadout, jettisoned, flies on with its mass (tests/test_stores.cpp: 46 stores).
    - Not built: flight models for the air-to-air missiles (stopped by the assistant's safety filter, 2026-10-10).
- **MS-7h, the detonation (2026-10-09).** The engagement's events in a recording (`V` records, format 5:
  `ipc::EngagementEvent`, one plain layout for every kind - a detonation now, a component's failure, a breakup and chaff
  to come), as MS-7d's fuze will write them; the showcase scripts one (`--detonate-s`: the missile ends there, as
  World ends a weapon; its explosive 40 % of the card's warhead, an estimate). The viewer draws it
  ([sdk/viewer.md](sdk/viewer.md#effects), world/Detonations): the flash, the fireball (4.67 W^0.31 m across in free air, from a 1 kg
  and a 10 kg TNT charge's measured diameters at the ground; larger in thin air; at TNT's measured 2,000 K, its glow
  Planck's against the sun's disk), and the soot it leaves, a billowing brown-black puff (soot's albedo, reflecting as
  an absorbing half-space) spreading and drifting with the wind, lit as the contrails are - held to six China Lake and
  Point Mugu photographs of hits on target drones (the white flash, the black puff, the AIM-54's glowing embers in
  it). Found on the way: `--view`'s point in the air was put on the ground under it - held now until the view is
  dragged.
- **MS-7h, the fire and smoke as a fluid (2026-10-09).** The owner rejected the fireballs pieced from spheres and
  noise: an engagement's fire and smoke are now solved by NVIDIA Flow 2.2 (BSD-3, NVIDIA-Omniverse/PhysX `flow/`, pinned
  at 6dc5358; built by `tools/flow/build-flow.sh`, configured with `FSIM_FLOW_DIR` - optional, not in the default
  build), a real-time sparse-voxel combustion solver, on its own Vulkan device (world::FlowSolver, its C interface).
  - Its sources are the damage chain's events (world/FlowFires.h): a warhead's detonation, a holed tank's spill, and
    the leak burning on from its component, laid along the hole's path as the aircraft flies; held to what is observed
    of fires - CCPS's fuel fireballs (Roberts 1981: diameter 5.8 M^(1/3) m, 0.45 M^(1/3) s), the photographed TNT
    fireballs (MS-7h's, above), API RP 521's jet flames (L = 0.00326 Q^0.478 m) - by `flightsim-flow-calibrate check`,
    a test where Flow is built: 12 cases, spills 3 - 1000 kg at 0.86 - 1.05 of CCPS's diameter and 0.90 - 1.10 of
    its duration (one at 6 km, its hole at 200 m/s), detonations 1 - 30 kg at 0.88 - 0.97 of the photographs', leaks
    5 - 20 kg/s at 0.94 - 1.04 of API's length. The flame's temperature is CCPS's measured surface emissive power's
    (350 kW/m^2: 1,600 K), its soot Tewarson's yield at fresh soot's extinction, its rise the hot gas's density deficit.
  - What the calibration found: Flow's advection does not dilute what expands, so the expansion its combustion makes
    compounds without bound (the fireball jumped from 0.5 to 7 times the observed between two settings): the products'
    volume and soot are put in by the sources, where the fuel is. A leak's smoke thins at 40 % a second (an estimate;
    Flow does not mix it with the air) - the solver's near field, a trail some 8 s long: without it the solver's
    blocks grew without bound (the frame rate at 4K fell from 160 to 49 in 20 s).
  - Drawn (world::FlowVolumes, `assets/clouds/shaders/flow.*`): its smoke and temperature read back each frame,
    packed as bricks in a 3D texture found by a hash of their place, drawn page by page (32 m), ray-marched as
    radiative transfer through soot - absorbing, scattering little, glowing at Planck's light for its temperature -
    with the world's sun, sky and aerial perspective. `--fire flow|spheres` in the viewer (flow when built with it);
    the old fire trail and its mist are not drawn with Flow. Measured at 3840x2160 fullscreen (RTX 5070 Ti), the
    AIM-120C showcase: 130 - 158 fps through the burst and the two 5 kg/s leaks after it (160 before), the solver
    2.5 - 3.5 ms of GPU, the packing 1.5 - 2 ms of CPU on 4 threads and Flow's own some 4 ms.
  - Found on the way: vsg's ellipsoid gives latitude and longitude in degrees - world::Detonations' wind frame took
    them as radians (fixed), as Flow's frame first did (its gravity then pointed anywhere: the smoke flew off).
  - Then (the owner, 2026-10-09: what still looked wrong): the flame's light was even - Flow holds every burning cell
    at the one flame temperature - so it flickers below the cells as large hydrocarbon fires are measured to (0.2 of
    its temperature rms, Blackbody's kFlicker; its mean lowered so its mean emissive power stays CCPS's), its pattern
    the clouds' noise read at the pixel's footprint, carried by the air; with it the opaque soot's edge no longer steps
    with the cells. The trail was a smooth tube: a burning aircraft's wake is now given to the solver - its vortex
    pair's flow (Gamma = W / (rho V b0), Lamb-Oseen cores 5 % of the span) across the wake, and its near wake's
    turbulence on the leak's spray (5 % of the airspeed rms: an estimate) - which rolls and billows the smoke. The far
    trail: world::Airflow's fire trail draws the smoke the solver's near field has lost (1 - exp(-0.51 age) of the
    soot's), its flame and mist left to Flow. A block holding no smoke freed once slower than 3 m/s (Flow's 1: the
    wake's stirring held blocks). Measured, the AIM-120C showcase at 4K: 126 - 164 fps, the solver 2.5 - 3.7 ms.
  - Open: GPU interop in place of the readback.
- **MS-7e2, the rocket's ice contrail (2026-10-09).** Where the air is cold enough a missile's exhaust's water freezes
  into a contrail in its smoke trail ([sdk/viewer.md](sdk/viewer.md#effects)): the Schmidt-Appleman criterion for a
  rocket's exhaust (cocip::formationOf: its water and the heat it brings the plume per kg of propellant), CoCiP's
  initial ice over the trail's section, then CoCiP's plume stepped as a jet's - without CoCiP's 6 - 13 km bounds, its
  weather data's domain, since a missile climbs past them. It forms where a jet's does (within a kelvin), lasts some
  20 - 45 s on a typical day and persists in ice-supersaturated air (tests/test_contrail_observed.cpp). Two drawing
  faults it showed are fixed for every trail: a store's first frame, interpolated from its empty slot at the Earth's
  centre, began its trail there (world::Interpolator now takes a vehicle new to a snapshot where the newer has it), and
  the young trail between laid samples left a climbing trail's plume at the older sample's height - drawn in steps.
  `fsim hangar showcase <missile> --altitude 10500` flies the shot in cold air.
- **MS-7e1, the carriage and the stores on their stations (2026-10-08).** Each aircraft's stations' pylons, launcher
  rails and racks in its model (fsim:station:<n> nodes, a pylon up to the skin found by a ray against the airframe's
  mesh; [hangar.md](hangar.md#the-carriage-on-the-stations)), shown once a store has been there; the stations' places
  in its manifest. The viewer draws each carried store with its own design's model on its station
  ([sdk/viewer.md](sdk/viewer.md#stores-on-their-stations)), from the live world's mission records (each store's
  card now in them: ipc layout 7) or a recording's store records (format version 3); the showcase carries its missile
  as the carrier's store until it is launched, a vehicle of its own from then.
- **MS-7e1, the weapon bays and their doors (2026-10-08).** The F-22A's, F-35A's, J-20A's and Su-57's bays cut into
  their underside, their doors cut from the skin over them, hinged on their outer edges
  ([hangar.md](hangar.md#the-carriage-on-the-stations)); the store management opens a bay's doors before releasing
  from it and shuts them after ([sdk/mission-systems.md](sdk/mission-systems.md#stores-and-weapon-employment-ms-7a));
  the viewer turns them, the store in the bay behind them. The doors' timings are assumed.
- **MS-7e1, the parts the airstream turns (2026-10-08).** The AIM-9M's rollerons, spun by the airstream and blurring
  into discs, and the Magic 2's tail, free to roll on its bearing ring ([hangar.md](hangar.md#missiles),
  [sdk/viewer.md](sdk/viewer.md#parts-the-airstream-turns)); their sizes and rates assumed. The R-77M's lattice
  (grid) fins: frames round a diamond lattice, all-moving on their spindles ([hangar.md](hangar.md#missiles)).
- **MS-7e1, the breakup sections (2026-10-09).** Every fixed-wing design's airframe split into its nose, centre,
  wings and tail - triangle by triangle, so whole it is as it was - each closed over its break, a node at its break
  with what is mounted on it ([hangar.md](hangar.md#the-3d-model)): the pieces MS-7g's structural kill and MS-7h's
  breakup part. The rule is one estimate for every aircraft (no structural data).

### MS-7 planned (2026-10-07)

MS-7 in six parts, each committed when its tests pass and runnable when it is (a demo for each). What no source gives
is estimated where a value is needed - from the closest published sibling, the class, or the physics - and marked as an
estimate (basis `assumed`, its reasoning in its note) wherever it appears:

- **MS-7a - stores management** (A-GRA's Store Management and Weapon Employment sequences, as SDK calls):
  - The inventory: each aircraft's stations from its store-station card (number, place, carriage, the store types it
    takes), loaded from a loadout the fit names (or the SDK's, before flight); each store a strike capability with its
    station, type, mnemonic, carriage, whether it is powerable and jettisonable, and its status.
  - Strike: a weapon type, a quantity and a target; the host picks the stores, asks for release consent (or the command
    gives it, as MA's weapons release authority may), releases them a ripple interval apart as entities (`launchWeapon`),
    and reports the activity (A-GRA's ActivityState). Abort: the activity canceled before its release.
  - Jettison: all the stores that can be, or those named; standard (a powered store powered down first) or emergency
    (released at once). A jettisoned store flies unarmed.
  - Tested with the Mk 82, the store whose flight model exists; the missiles join as theirs do (MS-7c).
- **MS-7b - the missiles' cards:** every fighter's air-to-air missiles by the same feature-based sourcing as the radars:
  dimensions, mass, fins, motor (impulse, burn time, propellant mass), seeker, guidance, fuze, warhead, published range;
  what no source gives, an engineering assumption said to be one. The fits' loadouts.
- **MS-7c - flight models:** each missile's aero tables from its hangar design's geometry (MS-7e1) - SU2 (Euler, its
  surface the design's own mesh) and VSPAERO (panels), each the other's and hangar's first tables' cross-check, both
  built from source here (owner, 2026-10-07) - its motor through RocketPy's solid-motor model, both written into the
  design's JSBSim model in place of the first ones, its flight against RocketPy's in the motor phase.
- **MS-7d - seekers, guidance, fuzes, lethality** (as the real ones behave):
  - An active radar seeker is a radar of its own through the platform's radar model (MS-4, MS-6): its frequency,
    power, aperture and processing on its card (estimates where unpublished, said so), so it acquires a target where
    its signal-to-noise ratio does against that target's RCS table and the clutter, not at a fixed range; inertial
    midcourse on the launcher's datalink updates, then the seeker's search, acquisition and track.
  - A semi-active seeker homes on the launcher's illumination: the launching radar must keep its target (continuous
    wave or interrupted, as the card says), and the missile loses it when it does not.
  - An infrared seeker sees the target's signature through a first IR model (the engines' plume and hot parts, the
    airframe's aerodynamic heating, the atmosphere's transmission; MS-10 completes it), with its field of regard,
    gimbal and track rate limits; flares come with MS-9.
  - Proportional navigation and its augmented forms, loft, the autopilot's fin commands within the airframe's limits;
    proximity fuze and warhead lethality; the weapons' flyout state (A-GRA's EngagementState).
- **MS-7e - the missiles as models, then their effects in the viewer** (the owner, 2026-10-08, approved: the models
  first, then their effects). Built the way the aircraft are: one hangar design (`aircraft/<name>/<name>.toml`) is both the picture
  and the aerodynamics - its bodies and surfaces give the viewer's model (the signed-distance mesher, each control
  surface cut from the same solid and named `fsim:<channel>`, the engines' kinds in the manifest), the tables (vortex
  lattice, slender bodies, Mach effects), the mass and the JSBSim model; what is drawn and not in the tables says so
  (`aero = false`, its drag in the calibrated extra drag area, as the F-16C's tip launchers and the A-10C's pylons are).

  **The focus now (the owner, 2026-10-08, after the plan's approval): the models, their mechanical animations and the
  visual effects only, for every missile and its variants.** The first flight models, a carried store's drag and the
  stages after MS-7e2 wait. Until a missile flies, its animations and effects are shown on a replay
  (`flightsim-viewer --replay`) whose states - the fins' deflections, the motor's thrust through its burn - are written
  from the card, not flown; the same states a flight model writes later.
  - **MS-7e1 - the models:**
    - each of the 24 missiles a hangar design, its card's length, diameter, span and mass held (a check fails the build
      outside the card's uncertainty), its external shape from public drawings and photographs: the nose (a radome, or
      an infrared seeker's dome), the body's stations, the strakes or wings, the control fins as all-moving surfaces
      on their spindles, each `fsim:` node mixing the pitch, yaw and roll channels with its gains and stops; the
      nozzle's exit (the motor's port, a rocket's line in the manifest); the lugs or hooks and markings bands drawn
      only; what no source gives estimated and said so;
    - (waits: the focus above) from the same design, a first flight model by hangar's own methods, extended where a missile needs them
      (cruciform fins, a body's lift at high incidence, the Mach range to the card's top speed): a solid rocket's
      thrust and propellant from the card, the CG moving as it burns, a fin mixer and limits in its FCS - enough to
      launch and fly it unguided, its fly stage's tests passed; MS-7c replaces its tables and motor with SU2's,
      VSPAERO's and RocketPy's, cross-checked against these;
    - the carriage in each fighter's design, as its fixed launchers already are: the pylons, launch rails and ejector
      launchers (LAU-127, LAU-7, LAU-106 and their counterparts) at the store-station card's places, drawn only; the
      weapon bays and their doors (F-22A, F-35A, J-20A, Su-57) as jointed parts on a channel of their own, as the gear
      doors are. Each changed fighter re-meshed, its RCS table remade, its flight unchanged to the bit (old XML against
      new);
    - the stores on the stations: the viewer draws each loaded store's model at its station from the manifest and
      takes it off when released; its drag on its carrier from its own design's zero-lift drag, added only while it is
      carried (an empty aircraft flies as before, to the bit);
    - for MS-7g and MS-7h: each airframe's breakup sections (wings, tail, forward fuselage) cut from its solid as its
      control surfaces are, its vulnerable components' places from its design (engines, tanks, cockpit), the
      dispensers' places.
  - **MS-7e2 - the models' effects** (as the aircraft engines' and contrails were studied, held to observations), each
    read from the missile's own flight, as the jets' exhaust reads its engine:
    - the motor's plume - its brightness and length through its burn's phases (boost, sustain, a dual pulse's two) from
      the rocket's thrust - and its smoke trail by motor type (a reduced-smoke motor's faint trail, an older one's
      dense one), dispersing and drifting with the wind as the contrails do, condensing into a contrail where the air
      allows; a solid motor's kind in the gas pass's march beside the jet engines';
    - the control surfaces moving: each missile's fins from its FCS (MS-7d's autopilot drives the same channels), the
      bay doors opening for a launch and closing after it;
    - the launch: off its rail under its own power (the rail's flash and smoke), or ejected and lighting a moment
      after; its end at the fuze (until MS-7d's fuze, its end at the ground).
- **MS-7f - validation and a runnable engagement:** range, time of flight and no-escape range against the published
  figures; a 4-against-4 engagement in real time, measured per payload family; `examples/python` demos of each part,
  and one engagement from radar track to launch, flyout, seeker acquisition and the fuze, in the viewer.

### Milestone M1 - the autonomous end-to-end loop (the owner, 2026-10-07; its stages approved)

The owner's direction: build the underlying infrastructure first; when the foundations are in place, a simple autonomous
demo runs the basic end-to-end loop once and marks the milestone. Player interaction and human play (flight controls,
HOTAS, HUD and cockpit displays, networked play) come later and are out of M1.

**The demo:** one fighter against one, both autonomous, each carrying an active and a semi-active radar missile. Search,
track, designate, the launch zone, launch, midcourse updates, the seeker's acquisition (or the illumination held), the
fuze, the target killed or not by the warhead's effect on it, the referee's record, the mission's end; the shot-at
fighter warned by its RWR and defending (notch or crank, chaff). The agents are simple scripted ones in
`examples/python`, behind the agent boundary (ADR-31 B6: no decision logic in the SDK); the run is seen in the viewer -
the missiles, their detonations, the hit aircraft's fire and smoke, the chaff (MS-7e, MS-7h).

**What M1 needs that no stage held** (found comparing the platform with DCS World, War Thunder and Ace Combat):

- **MS-4b - radar engagement modes.** Section 5 lists RWS, TWS, STT and ACM; MS-4 built the search modes only
  (`AmtiMode`). Added: a designated track the radar follows (a TWS volume kept on it), single-target track with
  illumination (what a semi-active missile homes on; the missile loses its target when the radar does), and ACM
  acquisition (boresight and vertical scan). Carried in A-GRA's AMTI command where it has the content, an SDK extension
  where it does not, said so.
- **MS-7g - the target's vulnerability and the kill.** MS-7d gives the warhead and the fuze; nothing said what a hit does
  to the aircraft. Added: each aircraft's vulnerable components (engines, fuel, crew, flight controls) from its hangar
  geometry; the warhead's fragment field from its card (the Gurney velocity, fragment mass and count); the kill
  probability of a detonation's geometry by the published survivability method (Ball, *The Fundamentals of Aircraft
  Combat Survivability*, AIAA), with its kill levels (attrition, mission kill); the consequence in the world - the
  vehicle killed or damaged, its payloads lost, its flight model failed as the components are (an engine out, controls
  lost, the airframe falling) - drawn from its seeded stream, deterministic; an event the referee records.
- **MS-9a - the RWR, launch warning and chaff** ahead of the rest of MS-9: the one-way equation from the world's radars
  (an active seeker going active among them), the launch warning, chaff clouds' RCS from the published dipole-cloud
  forms and their bloom and fall. Jammers and the full ESM stay in MS-9b.
- **AB-7 (ADR-31's range side) - the referee's rules and the scenario's payloads.** The referee's engagement record
  (shots, the weapons' ends, kills and their credit), end conditions, the outcome; the scenario format for loadouts and
  payload states at the start (closes section 10's open question).
- **MS-7h - the engagement's effects in the viewer** (the owner, 2026-10-07). MS-7e draws the missiles up to their
  fuzes; nothing drew what follows. Added, each driven by its model's state and held to observations as the contrails
  were, not judged by screenshots:
  - the detonation: its flash, fireball and smoke puff scaled by the warhead's explosive mass on its card, its
    fragments' spray from MS-7g's fragment field;
  - the hit aircraft: fire where MS-7g's failed components are (an engine, a fuel tank) and the smoke trail it leaves,
    the airframe breaking up when the kill is structural, its debris falling;
  - chaff: each bundle dispensed, its cloud's bloom and fall from MS-9a's model, drifting with the wind.

  Built on the viewer's exhaust glow, trails and contrail code where they fit; each effect's GPU cost measured
  fullscreen, as every viewer pass's. The geometry it needs (breakup sections, components, dispensers) comes with
  MS-7e1; each effect is drawn from its model's state, as MS-7e2's are, so MS-7h follows the stages that make that
  state (MS-7g's kill, MS-9a's chaff).

**Estimates left to refine (the owner, 2026-10-09: documented for later, unless one causes an obvious modelling
error).** Each is said so where it is used:

- the vulnerable components' fuel tanks' shapes (their capacity through the airframe's section, not a manual's tank
  diagram), the control actuators' size (one proportion), the R-195's size (nothing public: Raymer's);
- the J-20A's and H-6K's dispensers (nothing public places them: the rule), the dispensers' cartridge counts (the
  sources', for MS-9a's card);
- a warhead's explosive share (40 % of its mass until each card gives its charge), the fireball's timing (its growth,
  hold and cooling scaled as W^(1/3)), the soot's share (5 %) and the puff's growth (t^(1/4)); the fragments not drawn;
- the plumes' open differences (the reduced-smoke glow at 1 L, the shock-cell beads; after MS-7h, as the owner has it),
  the carrier's flow on a young trail, a flame splashed on its launcher.

**The visual focus kept (the owner, 2026-10-09).** MS-7e's models and effects done but two items (below), the owner
chose to keep the focus on the visuals: finish MS-7e, then build MS-7h's effects ahead of the stages that make their
state - on scripted replays, as MS-7e2's were (the showcase writes the states a flight would: a detonation at a time and
place, a component failed, chaff dispensed), each effect drawn from the state it will read once MS-7g and MS-9a write
it, so wiring them later changes no effect. The flight models, MS-7c and the stages after it wait. The sequence:

1. MS-7e2, the launch: a rail launch's flash and the smoke cloud round the launcher as the motor lights (an ejected
   one already falls clear before it lights); the missile's end - at the ground, until MS-7d's fuze.
2. MS-7e1, for MS-7g and MS-7h: each aircraft's vulnerable components placed from its design (engines, fuel tanks,
   cockpit, flight controls) and its chaff and flare dispensers, in its model's manifest.
3. MS-7h, on scripted replays: the detonation (flash, fireball, smoke puff by the warhead's explosive mass; the
   fragments' spray), the hit aircraft (fire at its failed components, its smoke trail; its breakup sections parting
   and tumbling, debris falling), chaff (the bundles' bloom and fall, drifting with the wind) - each held to
   observations, its GPU cost measured fullscreen.

**MS-7g's damage chain pulled forward (the owner, 2026-10-09).** The first hit-aircraft renders (56e0d13: the fuel's
fireball a sphere, its trail an even tube) looked artificial against the drone-hit photographs, and the owner asked
for the effects to follow how the aircraft is hit - from the forces and moments on it. A warhead's own push on a
fighter is a jolt (a few thousand N s: under 1 m/s on 15 t); what differs from hit to hit is what the fragments and the
blast break, and what the broken aircraft's forces, loads and motion then do. So the damage part of MS-7g comes first,
as a physical chain in the world - the kill probabilities and kill levels stay with the rest of MS-7g:

- **G1, the warheads:** each card's warhead sourced as its type, charge (mass, explosive), case and fragments (count,
  mass, material) or rods, the same kinds of source for every nation, gaps labelled; the explosives' Gurney constants.
- **G2, the airframes' structure for the world:** hangar writes each aircraft's damage data beside its model - the
  vulnerable components (kind, box, section, a tank's fuel, an engine's index, a control's channel), the breakup
  sections (their masses from the mass model, their lift share, spanwise centre of pressure and bending per g from the
  strip model, the root's strength from the design's ultimate load factor), the skin over each section.
- **G3, the impact:** `World::detonate` (what MS-7d's fuze will call; a range call until then) - the fragment field
  (Gurney velocity, the spray's belt tilted by the missile's velocity, fragments slowed by the air) and the blast
  (free-air overpressure and impulse, scaled for altitude) laid across the target's components and skin in its
  attitude; each hit's penetration; the damage drawn from the vehicle's own damage stream, deterministic.
- **G4, the damage state and its consequences:** a holed tank's leak (its hole, the airflow's suction) and fire; an
  engine out (JSBSim's own engine run flag), a control channel failed (the actuator's failure modes; the control
  layer told it is Faulted), the crew; a section's strength cut by the area the hits removed.
- **G5, the forces:** the damaged aircraft flown by its own model - a holed or lost surface's lift and drag taken out
  through the world's external force and moment at its centre of pressure, a lost section's mass and inertia off; so
  it rolls off, yaws or spins as the damage makes it.
- **G6, the loads and the breakup:** each section's root bending each step from its lift share and the load factor
  less its own inertia relief, against its remaining strength; a section that fails separates as a body of its own
  (its mass, inertia, drag; tumbling), the rest flown on.
- **G7, the record:** the world writes the engagement's events (the detonation, each hit, a fire's start, rate and
  end, a section's separation) and the debris' poses into the recording; the referee reads them (AB-7's record).
- **G8, the effects from the chain:** the burst oriented by the fragment belt (the ring photographed off the T-33),
  lobed fireballs and puffs, the fuel released from the holed side streaming in the local airflow (the QF-4B's
  fireball attached and stretched aft), the trail gusting as the leak burns, the skin panels torn off on the burst's
  side, the sections parting and the debris falling - each held to the photographs again.

The showcase becomes a world run: the target flown by its own JSBSim model under its autopilot, the missile scripted
until MS-7d, `World::detonate` at its burst.

Then the flight models (MS-7e1's waiting part) and MS-7c onwards, as below.

**The order to M1** (the owner, 2026-10-08, approved: the 3-D models first, then their visual effects; the models
built as the aircraft are, one design for the picture and the aerodynamics): MS-7e1 (the missiles as hangar designs, their first
flight models, the carriage, the stores on the stations), MS-7e2 (their effects: plumes, smoke trails, the control
surfaces moving, the launch), MS-7c (SU2's, VSPAERO's and RocketPy's tables and motors on the same designs), MS-4b,
MS-7d (active and semi-active seekers, guidance, the midcourse uplink, fuzes; the infrared seeker after M1), MS-7g,
MS-8 (launch zones), MS-9a, MS-7h (the engagement's effects), AB-7, then M1's demo as MS-7f's engagement. Each stage
keeps the verification of section 7. The order before (2026-10-07) put the viewer's stages last: MS-7c, 4b, 7d, 7g,
8, 9a, AB-7, 7e, 7h.

**After M1, not in it:** IFF (no friendlies in a 1-against-1; it gets a payload family before any 2-against-2), guns,
infrared missiles, IRST and flares (MS-10), jammers and ESM (MS-9b), comms and datalinks beyond the launcher's uplink
(MS-11), the OMS bridge (MS-12), human play.

## 9. Licensing

- **Today:** the platform is MIT; JSBSim (LGPL-2.1) ships as a separate library. MS-6 links nothing GPL (its chain
  is the platform's own, amended above), so no GPL step is planned.
- **If a GPL component is linked later:**
  - `LICENSE` becomes GPL-3.0; the MIT notice stays on the files that carry it.
  - `THIRD_PARTY_NOTICES.md` lists every component.
  - Each dependency's compatibility with GPL-3.0 is checked:
    - MIT, BSD, Apache-2.0 and LGPL are compatible.
    - FERS is GPL-2.0, run as a separate program for MS-6's cross-check, not linked.
    - SU2 (LGPL-2.1) and VSPAERO (NASA Open Source Agreement) are run as separate programs to make MS-7's aero
      tables, not linked; RocketPy (MIT) runs in Python beside the build. Their outputs (the tables) are the
      platform's data.
- **The rest:**
  - US Government works (NTIA, A-GRA, OMS, UCI) carry their notices.
  - Stone Soup's MIT notice goes with the ported trackers.

## 10. Open questions

- The order inside MS-7 after air-to-air: guns first, air-to-ground weapons in MS-11 (unless the owner says otherwise).
- The scenario format for payload loadouts per mission (stores per station, radar modes at start).
