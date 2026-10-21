# Payload plant adapters

The immutable-by-ownership component profile selects compiled SDK providers and
generated JSBSim plant assets. It does not identify a manufacturer's classified
control implementation or improve the existing weapon model. JSBSim remains the
only flight, actuator, motor and aerodynamic integrator. The adapter returns
normalized commands and validates native asset identity; it adds no force path.

## Supported configuration version 1

All components use the shared `component_models` sidecar and resolve role names
to copies of existing `card.parameters` values and provenance. An absent block
retains the previous generator/runtime route. An explicit unsupported ID,
version, role, unit, binding or unavailable component cannot select that route
silently. Unavailable and not-applicable declarations contain only their
availability and reason. They remain distinct from an available implementation.

| Component | Compiled implementation ID | Parameters and bindings |
|---|---|---|
| Autopilot | `fsim.control.load_feedback_cascade` | `execution_mode=legacy` has no parameter roles; `configured` requires `max_incidence_deg`, `rate_time_min_s`, `rate_lag_multiplier`, `incidence_time_min_s`, `incidence_rate_multiplier`; both bind `model` |
| Allocator | `fsim.allocator.jsbsim_authored_fin_mixer` | No numerical roles; `model`, `design` |
| Actuator | `fsim.actuator.jsbsim_lag_rate_stops` | `lag_s` in s, `rate_rad_s` in rad/s; `model` |
| Propulsion | `fsim.propulsion.jsbsim_generated_solid_rocket` | Existing phase, nozzle and grain roles below; `model`, `design`, `motor`, `nozzle`, `smoke` |
| Aerodynamics | `fsim.aerodynamics.jsbsim_generated_store` | No fitted numerical roles; `model`, `design`, `source`, and `table` only for an explicitly selected existing offline table |

The model binding equals the store's `jsbsim:<name>` identity. Design bindings
are `aircraft/<name>/<name>.toml`; motor and nozzle bindings are
`Engines/<name>_motor.xml` and `Engines/<name>_nozzle.xml`. Paths are canonical
relative asset references. The selected aerodynamic design must match the
geometry actually read by Hangar. The supported source names are
`component_build_up`, `offline_table`, `slender_store` and `bluff_cargo`.
Existing offline tables retain their strict card, geometry digest, reference,
complete grid and producer validation. This change supplies no new tables.

Hangar's aero stage uses a separate validated diagnostic dispatch. Tank and pod
geometry/carriage reports retain their original `storeaero.coefficients` values
and checks, labelled `geometry_diagnostic`; actual native forces continue to
come from `storemodels.store_xml` with the declared `slender_store` source.
Cargo retains its existing face-drag/terminal-speed diagnostic calculations,
labelled `face_drag_diagnostic`, while native generation uses `cargo_xml` and
`bluff_cargo`. Cargo's early report branch validates explicit implementation,
source/class, model and authored-design bindings before returning. These
diagnostic labels do not promote build-up report coefficients to native forces.

Legacy autopilot selection retains control version zero and its original
operation order/constants. Merely adding a component profile does not add
`weapon_execution_version` or turn other cards into version-one guidance. A
configured autopilot consumes the five explicit tuning roles. The native fixed
fin mixer remains the actual allocator; the generator's pseudoinverse only
reconstructs equivalent positions from actual fins and does not implement
failure recovery. Both execution modes validate the selected native actuator
time constant, channel rate and every physical-fin rate before control runs.

## Motor authority and unchanged numerical inputs

The propulsion descriptor references `phase_count` and each contiguous
`phase_i_duration_s` (s) and `phase_i_thrust_kn` (kN), with indices from zero.
Its remaining roles are `isp_s` (s), `coast_thrust_n` (N), `buildup_s` (s),
`grain_density_kg_m3` (kg/m^3), `grain_radius_fraction` (1),
`grain_port_area_fraction` (1), `grain_aft_clearance_m` (m), `exit_x_m/y_m/z_m`
(m), `exit_diameter_m`, `throat_diameter_m` and `cone_length_m` (m).
The existing axial generator explicitly refuses transverse nozzle offsets
rather than accepting unconsumed physical inputs. Phase count is bounded to
1024 in this declared schema; legacy declarations keep their original domain.

The migration proposal transcribes the inputs actually consumed from the
existing generated TOML, including its rounding. For example, the AIM-120C
nozzle inputs are 0.0925 m exit diameter, 0.0416 m throat diameter and 0.1017 m
cone length. Re-evaluating their original unrounded drawing formulas would
silently change grain placement and CG and is not this migration's behavior.

The retained defaults are the existing estimated phase curves, Isp 245 s
(255 s for the previously aluminised model), 2 N coast floor, 0.05 s buildup,
1750 kg/m^3 grain density, 0.9 grain-radius fraction, 0.15 port-area fraction
and 0.03 m aft clearance. Their explicit parameters carry assumed provenance.
Smoke remains a visual/source binding; explicit Isp is no longer inferred from
it. These values are simulator inputs, not measured AIM-120 motor/servo data.

`missiles.design`, `storebodies.files`, propulsion reporting, build verification
and the burn fixture all consume the same resolver. Existing consumed-mass
tables, the versioned ignition-time table for existing zero-thrust pulse phases,
the sine buildup, cylinder/grain/structure formulas and physical failure checks
remain intact. Selected values are never silently reduced to display precision.

## Native dispatch and identity

`configurePlantV1` validates the five resolved descriptors atomically and
produces one private `PlantAdapterV1`. Unsupported strings cannot reach the
compiled cascade. `plantAvailableV1` verifies actual loaded native metadata;
`plantCommandsV1` then checks the actual actuator inputs and calls the existing
`finCommands` implementation. Unavailable providers refuse execution, while
not-applicable providers require no fabricated native metadata. Public SDK
layouts and plugin vtables are unchanged.

The providers are typed C++ adapters compiled into the SDK. Implementation IDs
select these linked functions; this version has no dynamic DLL provider
registration. The sidecar's string names are configuration identifiers.

`configureBallisticPlantV1` selects the same validated aerodynamic provider for
an inert or jettisoned release. Its phase dependency mask requires only the
actual aerodynamic metadata, and `hasControl` is false. Omitted armed providers
are not asserted available. The complete original `ProfileV1`, including every
unavailable reason, remains separately owned for reporting. Selecting this
phase cannot make an unsupported seeker, fuze or propulsion model available.
Unknown, missing or stale aerodynamics still refuses ballistic release.

`configureUnguidedPlantV1` handles a non-inert release without guidance. Its
dependency mask requires propulsion and aerodynamics, with control disabled.
An available propulsion descriptor selects and validates the existing solid
rocket backend; an explicitly not-applicable descriptor admits an unpowered
body. Unavailable propulsion refuses this phase. The World checks actual native
engine inventory before publication: not-applicable propulsion requires zero
engines, and the available single-solid-rocket provider requires one. Inert
jettison uses the ballistic phase; absence of guidance alone does not disable
a native motor or bypass its declared implementation/asset identity.

The actual generated model publishes three numeric properties under
`fsim/components/<kind>/`: `schema-version`, `implementation-code` and
`profile-fingerprint`. Available providers use schema/code 1. The runtime reads
these from the loaded model in autopilot, allocator, actuator, propulsion,
aerodynamics order. An old or mismatched asset cannot substitute the expected
metadata from the card itself.

These tags establish generated-configuration consistency. Runtime does not
recompute hashes of loaded XML coefficients or engine-file bytes; generation
and asset-equivalence checks provide that separate evidence.

The content tag uses FNV-1a32: length-prefixed UTF-8 kind and implementation ID,
little-endian configuration version, sorted resolved role/unit/IEEE binary64
values, sorted parameter references and sorted string bindings. It describes
the configured simulator inputs and asset references. Provenance is retained
separately and does not alter the native mathematical model. The synthetic
cross-language fixture's tag is `3348686125`; its 0.08 s/3.25 rad/s actuator is
not an equipment setting. These identities establish configuration consistency,
not physical calibration or a manufacturer performance claim.

## Verification and migration output

`python -m hangar.components --migration-out
build/architecture-reviews/payload-plant-migration-proposal.json` reads the
maintained sources and writes proposed additions under the review directory.
It never edits a card, TOML, XML or GLB. The root owner merges all nine component
roles and performs maintained generation and native gates.

The pure Python tests cover immutable resolved inputs, explicit
refusal, unavailable status, nondefault motor/actuator consumption, unchanged
motor/nozzle and physics XML, cross-language identity and legacy applicability.
The persistent catalogue comparison visits every maintained store (currently
53), strips only newly introduced component/actuator-version metadata, resolves
native lag/rate property references to their actual numeric property values,
and compares all remaining XML nodes, attributes and text. Every engine/nozzle
asset must remain byte-identical. Indentation is insignificant; coefficients,
tables, physical limits and asset inventory are retained in the comparison.
The first five tests were captured failing before implementation. Native
adapter verification, including the ballistic phase, is root-owned.
Current full native, installation and scene acceptance remain root-owned.
