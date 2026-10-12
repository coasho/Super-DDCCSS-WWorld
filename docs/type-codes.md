# Internal type codes

Short codes for the kinds of vehicle and payload, for development: discussion, commit messages, the backlog, notes in
the documents. A code names a **type** - "an F vehicle", "an AF payload" - never a model: the F-16C stays the F-16C
and the AIM-120C the AIM-120C, in speech as in the data.

- **Always with its noun.** "E vehicles", "R payloads". No code is used twice across the two tables, so a bare one is
  still unambiguous.
- **Names in the data do not change.** The files, the SDK and the cards keep the original type names of the tables
  below (`category = "fighter"`, `"class": "tank"`, `"family": "air_radar"`); the codes are not identifiers in code.
- **This page is the mapping.** A type added to the data gets its row here in the same change ("Keeping it current").

## Vehicles

A vehicle's type is its design's `[aircraft] category` (`aircraft/<name>/<name>.toml`), which the profile carries as
`fsim::AircraftClass` (`include/fsim/VehicleProfile.h`; hangar's `CLASSES`, `tools/hangar/hangar/profile.py`). Every
design's category is listed in [hangar-effects.md](hangar-effects.md).

| Code | Original type | `category` | `AircraftClass` | For example |
| --- | --- | --- | --- | --- |
| F | fighter | `fighter` | `Fighter` | F-16C, Su-27S, J-20A |
| A | attack aircraft | `attack` | `Attack` | A-10C, Su-25 |
| B | bomber | `bomber` | `Bomber` | B-52H, H-6K |
| C | transport (cargo aircraft) | `transport` | `Transport` | C-130J, C-17A |
| K | tanker | `tanker` | `Tanker` | no design carries it yet |
| E | early warning aircraft | `aew` | `Aew` | E-7A |
| RC | reconnaissance aircraft | `reconnaissance` | `Reconnaissance` | U-2S, RQ-4B |
| EW | electronic warfare aircraft | `electronic warfare` | `ElectronicWarfare` | EC-130H |
| GA | light general aviation aircraft | `light_ga` | `LightGa` | Cessna 172 |
| U | unmanned aircraft (fixed wing) | `uav` | `Uav` | Skua |
| T | trainer | `trainer` | `Trainer` | no design carries it yet |
| H | helicopter | `helicopter` | `Helicopter` | UH-1H, UH-60 |
| MR | multirotor | `multirotor` | `Multirotor` | Iris, Crazyflie 2.0 |

- **The code follows the design's category, not the type's role.** The E-3G, KC-135R, KC-46A and RC-135W are
  `transport` in their designs: they are C vehicles until their designs say otherwise. The EA-18G is `fighter`: an F
  vehicle.
- **`missile` and `store`** are categories too, of the designs of payloads (below), not of vehicles.

## Payloads

A payload is one modelled mission system on an aircraft ([mission-systems.md](mission-systems.md), 1.3). Its type is
its card's `family` (`assets/payloads/*.json`; `fsim::ms::Family`, `include/fsim/MissionSystems.h`). What a store
station carries is a store, whose type is its card's `class` (`assets/stores/*.json`).

### Mission systems

| Code | Original type | `family` | `ms::Family` | For example |
| --- | --- | --- | --- | --- |
| R | radar (air search and fire control) | `air_radar` | `AirRadar` | AN/APG-68(V)9, AN/APY-2 |
| WR | weather radar | `weather_radar` | `WeatherRadar` | AN/APN-241 |
| SR | synthetic aperture radar | `sar` | `Sar` | ASARS-2, EISS SAR |
| SM | surface moving target radar | `smti` | `Smti` | no card carries it yet |
| GR | ground-mapping radar | `ground_map_radar` | `GroundMapRadar` | AN/APQ-166 |
| ES | electronic support: ESM, radar warning | `esm` | `Esm` | AN/ALR-56M, AN/ALQ-218 |
| PO | passive optical: FLIR, IRST, a targeting pod's sensor | `passive_optical` | `PassiveOptical` | AN/AAQ-40 EOTS, AN/AAQ-37 DAS |
| EA | electronic attack: a jammer | `electronic_attack` | `ElectronicAttack` | AN/ALQ-135, AN/ALQ-165 ASPJ |
| CM | countermeasures: chaff, flares, towed decoys | `countermeasures` | `Countermeasures` | AN/ALE-47 |
| CT | communications terminal: a radio, a datalink | `comm_terminal` | `CommTerminal` | MIDS, AN/ARC-210 |
| PN | positioning, navigation and timing | `pnt` | `Pnt` | MAGR 2K |
| G | gun | `gun` | `Gun` | GAU-8/A, BK-27 |
| ST | store station | `store_station` | `StoreStation` | a pylon, a rail, a bay's ejector, a pallet position |

### Stores

| Code | Original type | `class` | For example |
| --- | --- | --- | --- |
| AA | air-to-air missile | `aam` | AIM-120C, R-77M, PL-15 |
| AG | bomb and other air-to-ground store | `bomb` | Mk 82, CJ-10A |
| AF | auxiliary fuel tank | `tank` | 370 gal tank, PTB-1500 |
| PD | pod, as the body a station carries | `pod` | AN/AAQ-28, AN/ALQ-131 |
| SB | supply box: cargo on the floor, for airdrop | `cargo` | 463L pallet, CDS bundle (A-22) |

- **A pod is two things.** Its body on the station is a PD payload (mass, drag, model); the sensor or the jammer in it
  is the PO or EA payload its aircraft's card names.
- **A missile's design** is `category = "missile"`, any other store's `category = "store"`
  ([hangar.md](hangar.md#stores)): the code is the card's class, AA for every missile there is today.

## Keeping it current

| A new ... | is declared in | then |
| --- | --- | --- |
| vehicle type | `AircraftClass`, hangar's `CLASSES` (`profile.py`), the design's `category` | a row under Vehicles |
| mission-system type | `ms::Family` and `familyName` (`src/ms/Names.cpp`), the card's `family` | a row under Mission systems |
| store type | the card's `class` (hangar: `storedesigns.py`, `storebodies.py`) | a row under Stores |

A code is one or two capital letters, unused in every table above, and is not changed once it has been used.
