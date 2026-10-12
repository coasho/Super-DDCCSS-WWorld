# Native Missile Engagements

The SDK owns missile release, motor state, guidance, seeker observations, countermeasures, fuzes and damage. A client submits commands and renders SDK snapshots and events. JSBSim is the sole integrator of released weapon flight; there is no client-side missile trajectory or random decoy adjudication.

## Release and observation

Load a flyable air-to-air store, operate its station payload, and submit a strike through the existing vehicle API. Acceptance creates a strike activity. Release occurs after its preparation and consent stages. Read `world.weapons` to discover the released entity; do not consume presentation inventory at command acceptance. Failed strikes leave loaded stores intact. Read `vehicle.stores` to reconcile stores subsequently lost through damage.

A radar shot uses an own-platform radar-track designation. Midcourse guidance reads that designation's measured estimate and extrapolates its last observation after loss of support. A raw target entity supplied by a range script cannot create midcourse sensor updates. Semi-active radar requires the designated track's illumination. Infrared and imaging-infrared seekers use current local observations. `world.seeker_target(vehicle, store)` provides an on-rail infrared observation or zero when none is available. A zero designation is a failed observation, not a valid game shot.

Native seeker observations are filtered by range, aspect, radar cross section where available, terrain propagation, gimbal and Doppler gates. Guidance cannot retain a current tracking flag after its observation disappears. Countermeasures compete within the held contact's association cell. Jettison creates an inert, unguided body and cannot arm its motor or fuze.

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

`WeaponRecord` appends `phase`, `seeker`, `holding`, `miss_m`, `burning`, `tracking`, `supported`, `time_to_go_s` and `time_to_active_s`. `burning` comes from native engine thrust; fuel-flow bookkeeping alone can remain nonzero after rocket burnout. `holding` is a current seeker observation and may be a decoy. Unknown miss distance is NaN. Unsupported timing estimates are zero. Terminal records retain the native final position and outcome; the snapshot retains the most recent 256 terminal weapons alongside live weapons.

Events carry monotonic `serial`, step-boundary `time_s`, `kind`, `reason`, `entity`, `launcher`, `target`, ECEF position in metres, ECEF velocity in metres per second and `destroyed`. Kinds include `launch`, `activation`, `acquisition`, `track_loss`, `decoy_capture`, `chaff`, `flare`, `detonation`, `damage`, `expiry`, `impact` and `removal`.

Reads are repeatable and never consume another client's events. History is retained for the world's lifetime. Each reader maintains its own exclusive cursor. Weapon terminal events occur once. Detonation poses use swept encounter interpolation; timestamps remain at the enclosing simulation step boundary. Damage assessments use target positions interpolated to the same encounter fraction.

`damage.destroyed` reflects the native crew-loss outcome. A proximity burst can damage an aircraft that continues to fly. Clients must not infer a kill from a detonation. Later native crashes or divergence remain authoritative. Released decoys continue to fly and expire even after their carrier is removed.

## C and C++ compatibility and access

The corresponding methods are available on `fsim::World`; C uses the `fsim_world_*` engagement functions and sized result structures in `fsim_c.h`. New weapon fields and terminal enums are appended. Initializers set `struct_size`; getters copy no more than the caller's advertised size, preserving older binaries' result buffers. Existing mission reason values are preserved, including the appended `store_not_flyable` reason.

World weapon, event and decoy truth snapshots are range/referee interfaces. C agent handles cannot read these world snapshots or the on-rail query's entity IDs. Dispenser inventory and actions are scoped to the owning platform. Agent launch-zone queries require an own designation. The generated [boundary table](boundary.md) enumerates the exported C++, C and Python surface and is checked by `sdk_boundary`.

## Modeling assumptions

The following are game estimates (`basis: assumed`), not calibrated real-world missile performance:

- Proportional navigation gain 3.5, bounded lateral acceleration and an ideal fin servo applied through native JSBSim force/moment hooks. Native axial drag, propulsion, mass, gravity and motion integration remain active.
- Card overrides apply to seeker range, gimbal, acceleration limit, battery life and fuze radius. Native release defaults are 17 km for active radar, 60 km for semi-active radar and 13 km for infrared; gimbal limits are 60 degrees for radar and 45 for infrared; acceleration limits are 35 g and 40 g respectively; battery limits are 110 s and 45 s respectively. Arming takes one second. Radar midcourse loft is 20 degrees; infrared loft is zero. Default fuze radius is `7 + sqrt(warhead_kg)` metres, with 12 kg used when that card parameter is absent.
- Infrared acquisition varies with target aspect and afterburner. Radar uses a simplified look-down Doppler gate of 28 m/s. Relative target strength is normalized to one; decoy strength uses deterministic bloom/fade rather than random rolls.
- The association cell is two degrees. Ordinary seekers require a decoy strength exceeding 1.6 times the held return; imaging seekers require five times. Fresh flare strength peaks at six, then decays with a two-second time constant, so imaging seekers resist moderate flares but are not immune. Chaff blooms over 0.3 s and decays over six seconds.
- Installed dispensers begin with 30 chaff and 15 flares, share a 0.35 s release interval, and use still-air drag/gravity. Chaff lasts six seconds and flares four seconds. Dispenser refusal consumes no inventory.
- Active seeker emissions feed the existing receiver simulation with assumed transmitter power and antenna pattern. Clients display measured receiver contacts, without a geometric warning fallback.
- Launch zones are advisory energy envelopes based on the card, launch altitude/speed and observed target velocity. The `no_escape_m` name expresses an estimate; it is neither an integrated intercept prediction nor a guaranteed hit.

This milestone does not calibrate fin aerodynamics, imaging classification, electronic counter-countermeasures or wind coupling for decoys. It establishes one simulation authority and reviewable native engagement behavior.

## Validation

`weapon_guidance` covers activation, stale estimates, illumination, infrared observations, bounded acceleration, arming, swept proximity and deterministic decoy discrimination. `countermeasure_core` covers inventory, interval, invalid inputs, analytic motion and expiry. `engagement_native` runs actual JSBSim releases, head-on encounters, motor burnout/coast, event replay, worker-count determinism, unsupported truth targets and inert jettison. C and Python suites cover telemetry compatibility, dispenser access and carrier-independent decoy lifetime. The [delivery validation record](engagement-validation.md) lists the verified commands, outcomes and remaining modeling limits.
