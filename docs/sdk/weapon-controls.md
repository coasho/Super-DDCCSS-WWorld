# Actual weapon fin diagnostics

`World::weaponControlsV1(id)`, `fsim_world_weapon_controls_v1` and Python `World.weapon_controls(id)` expose actual native individual-fin positions and the latest finite-controller decision. They are read-only Range/referee diagnostics. Agent handles are refused; the values do not enter actor reports, observations, action masks or command authorization.

The production path is `libfsim` → native World guidance decision → normalized channel commands → JSBSim channel response and physical-fin mixer → each finite actuator → aerodynamic forces/moments → rigid-body integration. The query reads the resulting native actuator properties. Viewer code draws these values and supplies no simulation forces.

## Version 1 values

The new C++ value types are in `fsim/WeaponControls.h`, under `fsim::ms`. Existing public structures, classes' data layouts, plugin vtables and enum values are unchanged.

| Snapshot field | Meaning |
| --- | --- |
| `entity`, `slot`, `generation` | Live native weapon identity and current slot generation. Slot alone is insufficient to identify a weapon. |
| `available` | The Version 1 physical-fin identity registry is bounded and has unique stable IDs. Individual numerical measurements/limits still have their own `valid` flag. |
| `controllerAvailable` | The latest guidance pre-step produced an admitted finite-controller solution. False includes an unavailable solution or a weapon without an active guidance decision. |
| `sampleTimeS` | Native World time at this read, after its most recent completed step. |
| `controllerTimeS` | Time of the latest pre-step controller decision; unknown before a decision. It precedes the resulting actuator sample. |
| `commands` | Latest normalized aileron, elevator and rudder demands, in that order. These are controller inputs to the native FCS, not fin positions. |
| `fins` | Each authored physical fin once, independent of aerodynamic patch subdivision. |

| Fin field | Meaning |
| --- | --- |
| `id` | Nonzero uint32 identity derived from authored surface name, control name and mirror side. Stable under enumeration changes and aerodynamic patch subdivision. Collisions fail generation explicitly. |
| `nativeIndex` | Current `fcs/fin-<index>-…` property index. It is diagnostic indexing, not persistent identity. |
| `valid` | The actual/demand/stops/rate and native actuator failure properties exist and are finite, with ordered stops and positive rate. |
| `actualRad`, `demandRad` | Measured actuator position and its mixed, clipped demand, in the native fin convention. |
| `minimumRad`, `maximumRad`, `rateRadS` | Authored stops and existing modeled rate limit. |
| `failedStuck`, `failedZero`, `failedHardover` | Current native actuator malfunction flags. No failure is inferred from an unchanged angle. |

Unknown numbers are NaN in C++/C and `None` in Python. Python returns `WeaponControlSnapshotV1` and `WeaponFinStateV1` records, with snake_case field names and generation as an unsigned decimal string. An unknown, non-weapon or ended entity returns no C++ optional / Python `None`; the C call returns `FSIM_INVALID_ARGUMENT`. A live weapon with unavailable metadata returns an identified snapshot with `available=false`; missing data is never synthesized from the three equivalent channel positions.

Optional property inventories are queried quietly once per model generation. Only present leaves become cached handles. Ordinary bombs/tanks without finite-control metadata therefore remain explicitly unavailable without repeated property lookup warnings. Reset discards these bindings before another query.

The C API uses new sized records `fsim_weapon_controls_v1` and `fsim_weapon_fin_v1`. Initialize the header with `fsim_weapon_controls_v1_init` and every fin row with `fsim_weapon_fin_v1_init`. A call with `fins=NULL, capacity=0` returns the required row count without consuming state. A smaller initialized buffer receives at most its capacity while `required` remains the full count. Rows use the first row's `struct_size` as stride. Repeated reads at the same native time are non-consuming. As with other World queries, serialize them with step, reset and destruction.

```cpp
if (auto sample = world.weaponControlsV1(missile)) {
    for (const auto& fin : sample->fins)
        if (sample->available && fin.valid)
            std::printf("fin %u: %.9f rad\n", fin.id, fin.actualRad);
}
```

## Live transport and recording

WorldLayout remains **v9**, byte-for-byte. An optional separate `fsim.weapon-controls.v1.<world>` mapping carries bounded Version 1 snapshots. Its publisher PID, incarnation and exact base snapshot sequence must match. Each accepted row must also match the base entity, live slot generation and sample time. Missing, malformed, mismatched, removed or overwritten data clears that frame's actual-fin poses. Readers never substitute a retained older frame.

Recording **v8** adds explicit size-prefixed `F` records containing the same fin wire values. The v7 base vehicle sample stride is unchanged. The reader still converts v1–v6 short samples and reads v7/v8 full samples. An `F` record applies only to the next `S` record with the same sample time; it is not retained across frames. The public `Recording`, `Frame` and C recording layouts remain unchanged. The internal recording reader supplies the extra values to Viewer replay.

New missile GLB hinge nodes are `fsim:fin:<stable-id>:<sign>|<legacy axis/mix@stops>`. The generator derives the sign from the actual hinge direction, with one identity shared by native force patches and their physical visual fin. Distinct authored surfaces cannot reuse an identity; only aerodynamic patches of the same authored surface instance share its actuator. A jammed fin stays at its actual position while other fins move independently.

Actual valid fin angles always take precedence over the legacy mixer. When actual telemetry is absent, the explicitly authored fallback preserves existing axis/mix/stops animation for v1–v7 recordings and older publishers. That pose is approximate and does not claim an actual individual-fin measurement or a recoverable jam. Bare `fsim:fin:<id>:<sign>` nodes without fallback remain neutral. Old models with axis nodes retain their existing animation. Replay seek/loop, world change/restart, model change and removal clear cached actual poses.

Actual-fin presentation retains the same bounded history as body interpolation. It uses the identical render clock: native simulation time for demo/replay and publisher wall time for an external live source. A weapon's model age is not substituted for either clock. Linear presentation blending requires matching slot, entity, generation and fin ID, with valid angles at both endpoints. A missing endpoint or changed identity removes actual availability rather than blending unrelated values. With interpolation disabled, exact newest values are used. Native query timestamps, recorded samples and simulation behavior are unchanged by this display interpolation.

## Installed SDK example and acceptance

`examples/simulation/python/missile_controls.py` runs against the installed Python package/DLL. It creates a diagnostic Range, uses an own radar measurement to designate a target, submits the strike through a launcher Agent and answers a delivered consent request through a C2 Agent. It then reads actual fin motion and applies an explicit Range diagnostic `fail_stuck` fault. `--realtime` permits live Viewer inspection; `--record` writes the same native states for replay.

Actual-fin availability and controller-decision availability are independent. A jammed trajectory can leave the admitted controller solution domain while actual actuator measurements remain valid. The example requires real movement, an available native decision and retained jam state, checks neutral commands on unavailable decisions, and reports their count. It does not assert that every later controller decision succeeds or that the engagement hits.

```powershell
python -s -B examples/simulation/python/missile_controls.py --world missile-controls --realtime --record missile-controls.fsrec
```

The installed C++, C99 and Python consumers in `tests/compatibility/equipment_package_consumer/` exercise the same legal release chain, require actual movement and a retained native jam, and retain isolated-prefix header/module/DLL/data provenance checks. C99 additionally checks the bounded, non-consuming fin buffer and actor refusal. These are executable acceptance fixtures; current build, test and GUI outcomes are recorded by the serial acceptance owner, not implied by this document.

Focused regressions cover stable authored fin identity under reordering/subdivision, refusal of distinct duplicate authored surfaces, quiet unsupported-model queries/publication, native public fin state/jam/removal, optional-stream identity/sequence/time matching, invalid wire flags and IDs, v8 replay state, actual generated GLB poses, old v7 playback fallback, actual-over-fallback precedence and matching-clock midpoint/identity guards. Native and installed gates and the complete live/replay GUI scenario remain required before acceptance.

This adds observability and faithful presentation of the admitted reduced-order control model. The existing assumed 25/s channel lag, 4 rad/s actuator rate and geometry-derived aerodynamic estimates remain uncalibrated. It supplies no new equipment constants, and does not establish a 30 km hit or full SIM-M03 acceptance.
