# Native Missile Engagements

The SDK owns missile release, motor state, guidance, seeker observations, countermeasures, fuzes and damage. A client submits commands and renders SDK snapshots and events. JSBSim is the sole integrator of released weapon flight; there is no client-side missile trajectory or random decoy adjudication.

## Release and observation

Load a flyable air-to-air store, operate its station payload, and submit a strike through the existing vehicle API. Acceptance creates a strike activity. Release occurs after its preparation and consent stages. Read `world.weapons` to discover the released entity; do not consume presentation inventory at command acceptance. Failed strikes leave loaded stores intact. Read `vehicle.stores` to reconcile stores subsequently lost through damage.

A radar shot uses an own-platform radar-track designation. Midcourse guidance reads that designation's measured estimate and extrapolates its last observation after loss of support. A raw target entity supplied by a range script cannot create midcourse sensor updates. Semi-active radar requires the designated track's illumination. Infrared and imaging-infrared seekers use current local observations. `world.seeker_target(vehicle, store)` provides an on-rail infrared observation or zero when none is available. A zero designation is a failed observation, not a valid game shot.

Native seeker observations are filtered by range, aspect, radar cross section where available, terrain propagation, gimbal and Doppler gates. Guidance cannot retain a current tracking flag after its observation disappears. Countermeasures compete within the held contact's association cell. Jettison creates an inert, unguided body and cannot arm its motor or fuze.

Explicit version-1 store capability metadata separates inertial steering from launcher updates. Launch and due-release validation check every selected store before any member of that due activity spawns. A Ready infrared store can supply its actual on-rail observation. Semi-active candidates must pass the actual transmitter beam and both propagation paths. These mechanisms do not calibrate equipment or add a modeled weapon datalink.

The [contact-fuze channel](weapon-contact-fuze.md) can initiate from an exact authored nose-tip crossing of attached finite aircraft geometry after seeker loss. First unarmed contact consumes only that release's contact channel. Contact and proximity share one detonation authority; contact events identify the physical victim with `reason="contact"`, independently of the retained seeker holding.

## Python presentation API

| Interface | Result |
| --- | --- |
| `world.weapons` | Live and terminal `WeaponRecord` snapshots |
| `world.dispense(vehicle, "chaff" or "flare")` | Accepted release; raises `fsim.Error` on refusal |
| `world.countermeasures(vehicle)` | Remaining chaff and flare inventory |
| `world.decoys()` | Decoy ID, owner, kind, age in seconds, ECEF position in metres |
| `world.engagement_events(since=serial)` | Every event with serial strictly greater than the cursor |
| `world.launch_zone(vehicle, store, target)` | Advisory maximum, no-escape and minimum distances in metres |
| `world.seeker_target(vehicle, store)` | On-rail infrared observed entity, or zero |
| `world.weapon_controls(weapon)` | Versioned actual-fin/controller Range diagnostics, or `None` for an unknown/terminal weapon; [availability and C++/C equivalents](weapon-controls.md) |

`WeaponRecord` appends `phase`, `seeker`, `holding`, `miss_m`, `burning`, `tracking`, `supported`, `time_to_go_s` and `time_to_active_s`. `burning` comes from native engine thrust; fuel-flow bookkeeping alone can remain nonzero after rocket burnout. `holding` is the last held seeker observation and may be a decoy; `tracking` says whether it is current. Unknown miss distance and unavailable timing estimates are NaN. An active-radar seeker reports zero time to active after activation; IR and semi-active seekers have no such transition estimate. Terminal records retain the native final position and outcome; the snapshot retains the most recent 256 terminal weapons alongside live weapons. See the dated [SIM-M03 correctness evidence](weapon-fidelity-validation.md) for observation and fuze corrections and remaining physical-fidelity gates.

Events carry monotonic `serial`, step-boundary `time_s`, `kind`, `reason`, `entity`, `launcher`, `target`, ECEF position in metres, ECEF velocity in metres per second and `destroyed`. Kinds include `launch`, `activation`, `acquisition`, `track_loss`, `decoy_capture`, `chaff`, `flare`, `detonation`, `damage`, `expiry`, `impact` and `removal`.

Reads are repeatable and never consume another client's events. History is retained for the world's lifetime. Each reader maintains its own exclusive cursor. Weapon terminal events occur once. Detonation poses use swept encounter interpolation; timestamps remain at the enclosing simulation step boundary. Damage assessments use target positions interpolated to the same encounter fraction.

`damage.destroyed` reflects the native crew-loss outcome. A proximity burst can damage an aircraft that continues to fly. Clients must not infer a kill from a detonation. Later native crashes or divergence remain authoritative. Released decoys continue to fly and expire even after their carrier is removed.

## C and C++ compatibility and access

The corresponding methods are available on `fsim::World`; C uses the `fsim_world_*` engagement functions and sized result structures in `fsim_c.h`. New weapon fields and terminal enums are appended. Initializers set `struct_size`; getters copy no more than the caller's advertised size, preserving older binaries' result buffers. Existing mission reason values are preserved, including the appended `store_not_flyable` reason.

World weapon, event and decoy truth snapshots are range/referee interfaces. C agent handles cannot read these world snapshots or the on-rail query's entity IDs. Dispenser inventory and actions are scoped to the owning platform. Agent launch-zone queries require an own designation. The generated [boundary table](boundary.md) enumerates the exported C++, C and Python surface and is checked by `sdk_boundary`.

### Agent equipment access and reports

Agent equipment calls use the existing native operations on the Agent's own platform:

| C++ | Python | Existing C entry point |
| --- | --- | --- |
| `Agent::countermeasures()` | `agent.countermeasures()` | `fsim_world_countermeasures` on an Agent handle |
| `Agent::dispense(flare)` | `agent.dispense("chaff" or "flare")` | `fsim_world_dispense` on an Agent handle |
| `Agent::launchZone(store, designation)` | `agent.launch_zone(store, designation)` | `fsim_world_launch_zone` on an Agent handle |

Dispenser requests preserve native payload-state, inventory and shared cooldown checks. C++ returns the native
`ms::Reason`; Python raises `fsim.Error` on refusal. Neither installs an automatic defensive response. A launch-zone
query requires a current designation belonging to this platform, never a raw entity or another platform's
designation. C++ returns an empty optional on that refusal; C/Python retain the existing status/error behavior.
Successful results are the native advisory estimates. An unknown store's default distances are not evidence that
the store is carried, supported or able to intercept; query results do not grant release authority.

The additive equipment report methods use the existing `ma::Report` / `ma::Collection` vocabulary:

| C++ | Python | Metadata kind and values in C |
| --- | --- | --- |
| `reportStores(options)` | `report_stores(stale_after_s=None)` | `FSIM_REPORT_STORES` (8), existing store getters |
| `reportStrikes(options)` | `report_strikes(stale_after_s=None)` | `FSIM_REPORT_STRIKES` (9), `fsim_agent_strike_report_get` |
| `reportCountermeasures(options)` | `report_countermeasures(stale_after_s=None)` | `FSIM_REPORT_COUNTERMEASURES` (10), existing inventory getter |
| `reportConsents(options)` | `report_consents(stale_after_s=None)` | `FSIM_REPORT_CONSENTS` (11), `fsim_agent_consent_report_get` |

Store, strike and inventory reports describe this platform's current equipment state, with `own_platform`
provenance, sample/receive time equal to the time read and age zero. A C2 node has no such equipment. An aircraft
without a countermeasure payload reports `not_applicable`, distinct from an installed dispenser with zero cartridges.
A removed platform reports `unavailable`. Inventory values are meaningful only when available.
The existing native store state `Lost` is also named in C as `FSIM_STORE_LOST` (7); C/Python report its status as
`"lost"`, rather than losing the state name. Existing store-state numbers 0-6 are unchanged.

Strike reports retain native activity/phase, ordered store indices and released weapon IDs. Targets are own-platform
designations or zero: a Range-created activity's true entity target is redacted. In Python each value is
`StrikeReport(activity, status)`, with the existing `StrikeStatus` unchanged. The new C strike getter returns the
activity separately and uses the existing sized status and bounded arrays. Reads do not grant another service
permission to update or cancel an activity. Activity completion means the native release transaction ended, not
impact or mission success.

Consent reports contain only requests already delivered to this platform, with `dms` provenance, the requesting MA
system, the request's sample time and its actual receive time. The delivery net is unknown because the native record
does not retain it. On a bound platform only the DMS ingress owner receives this report; other services get
`not_owner` with no items, and the new C value getter refuses them. Legacy consent reads retain their existing
platform-shared behavior. Repeated metadata/count/value reads are non-consuming.

Released IDs are correlations, not weapon telemetry. Weapon/seeker/support/miss-distance/timing feedback remains
`unavailable / no_modeled_feedback` under the [extension contract](ma-engagement-contract.md). World weapon records,
decoy positions, engagement events and on-rail target truth remain outside Agent access. These methods do not
install the future mission gateway handlers.

`examples/training/python/ma_equipment.py` demonstrates a configured service, current inventory metadata and one native
cartridge release through the public SDK, without exposing world weapon or decoy truth.

## Modeling assumptions

The following are game estimates (`basis: assumed`), not calibrated real-world missile performance:

- Proportional navigation gain 3.5 and bounded acceleration demand feed normalized surface commands through the [finite actuator/fin chain](missile-control-validation.md). JSBSim realizes the geometric fin forces/moments together with native drag, propulsion, evolving mass, gravity and motion. Response/control-effectiveness parameters remain engineering estimates.
- Card overrides apply to seeker range, gimbal, acceleration limit, battery life and fuze radius. Native release defaults are 17 km for active radar, 60 km for semi-active radar and 13 km for infrared; gimbal limits are 60 degrees for radar and 45 for infrared; acceleration limits are 35 g and 40 g respectively; battery limits are 110 s and 45 s respectively. Arming takes one second. Radar midcourse loft is 20 degrees; infrared loft is zero. Default fuze radius is `7 + sqrt(warhead_kg)` metres, with 12 kg used when that card parameter is absent.
- Infrared acquisition varies with target aspect and afterburner. Radar uses a simplified look-down Doppler gate of 28 m/s. Relative target strength is normalized to one; decoy strength uses deterministic bloom/fade rather than random rolls.
- The association cell is two degrees. Ordinary seekers require a decoy strength exceeding 1.6 times the held return; imaging seekers require five times. Fresh flare strength peaks at six, then decays with a two-second time constant, so imaging seekers resist moderate flares but are not immune. Chaff blooms over 0.3 s and decays over six seconds.
- Installed dispensers begin with 30 chaff and 15 flares, share a 0.35 s release interval, and use analytic linear drag/gravity relative to each particle's local modeled mean wind. Chaff lasts six seconds and flares four seconds. Dispenser refusal consumes no inventory. Seeker observations precede propagation; owned trajectories retain the last partial living interval through fuze processing.
- Active seeker emissions feed the existing receiver simulation with assumed transmitter power and antenna pattern. Clients display measured receiver contacts, without a geometric warning fallback.
- Launch zones are advisory energy envelopes based on the card, launch altitude/speed and observed target velocity. The `no_escape_m` name expresses an estimate; it is neither an integrated intercept prediction nor a guaranteed hit.

This milestone does not calibrate fin aerodynamics, imaging classification, electronic counter-countermeasures or decoy cartridge aerodynamics/signatures. Decoy gust/turbulence fields remain unsupported. The [physical-chain audit](sim-m03-physical-chain-audit.md) records independent corrections and the remaining source-linked approximations and required inputs.

## Validation

`weapon_guidance` covers activation, stale estimates, illumination, infrared observations, bounded acceleration, arming, swept proximity and deterministic decoy discrimination. `countermeasure_core` covers inventory, interval, invalid inputs, analytic motion and expiry. `engagement_native` runs actual JSBSim releases, head-on encounters, motor burnout/coast, event replay, worker-count determinism, unsupported truth targets and inert jettison. C and Python suites cover telemetry compatibility, dispenser access and carrier-independent decoy lifetime. The [delivery validation record](engagement-validation.md) lists the verified commands, outcomes and remaining modeling limits.

## Equipment interface verification (2026-10-14)

Baseline: main `1325a3c`, plus this equipment access/report slice. UCRT64 Release and headless builds used the existing
presets with warnings as errors, CPython 3.13 and temporary directories under `D:/ENV/tmp`.

| Check | Recorded result |
| --- | --- |
| Missing-interface regression | The new native/C tests failed to build against the baseline before implementation. |
| Headless and Release builds | Both passed with warnings as errors. |
| Native feature gates | `ma_equipment` and `ma_equipment_c`: 2/2 passed, including six C++ cases and the plain C99 consumer. |
| Python feature tests | 6/6 passed, including report values, native lost-store naming and one cartridge release over the existing isolated Agent process channel. |
| Headless focused regression | 23/23 passed in 119.21 s, including the full Python SDK suite, native engagements, ABI, bindings/reports, adapter compatibility, contracts and both installed consumer gates. |
| Headless installed equipment consumer | Included above: 1/1 passed in 25.37 s; C++, C99 and Python used separate installed prefixes. |
| Release focused regression | 8/8 passed in 86.49 s, including native engagements, equipment, bindings/reports, C ABI, SDK boundary and the installed equipment consumer. |
| Frozen consumers on the new installed SDK | All five passed: old C++ and C99 binaries and the three old Python Agent/link/C2 consumers. The 66 baseline artifacts were unchanged. |
| Public ABI metadata against M0 | All 267 existing C++ types, 120 C records, 107 C++ enums, 81 C enums and 50 public vtables retained their layouts, existing values and slots. No old export was removed; current headless/Release core exports match. |
| Protected runtime inputs | All 2,357 recorded aircraft/model/card/control inputs retained their SHA-256 hashes. |
| Installed public example | `examples/training/python/ma_equipment.py` passed against the installed Python package. |

The scenarios exercise off-payload refusal, native shared cooldown, own/foreign/lapsed designation boundaries,
Range-target redaction, repeatable reads, caller-sized C outputs, native lost-store naming and configured ingress ownership. Approval is sent
over modeled C2/DMS delivery; tests continue until native store release, compare nonzero weapon IDs with store and
strike reports, and separately verify actual emergency jettison. No Range consent injection fulfills those actor
requests. Referee records are used only on the test side to check native correlations.

The equipment consumer gate explicitly binds each language to its installed aircraft and JSBSim directories,
checks wrapper/native-module/DLL/header/data provenance, corrupts a private installed model copy and requires
loading to fail, restores it, then repeats the successful smoke checks. Development resource discovery remains
unchanged; this gate does not rely on its source-tree fallback.

Raw logs are under `D:/ENV/tmp/ma-equipment-access-20261014/`; installed provenance manifests are under each build's
`equipment-package-consumer/`. ABI metadata and hash comparisons are in `compatibility.json` beside the logs.
The full headless/Release suites and Viewer/GPU scenarios were not rerun for this adapter-only slice. The dated
[Viewer evidence](viewer-validation.md) retains its existing scope and limits. Mission episodes, gateway handler
registration, policy/outcome integration and full joint acceptance remain pending; no such gate is closed here.
