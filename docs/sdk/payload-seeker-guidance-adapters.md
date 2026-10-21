# Payload seeker and guidance adapters

The native payload component sidecar selects the existing encounter seeker and
velocity-normal, closing-speed proportional-navigation implementation. These
adapters make their assumptions explicit and reusable across store cards. They
add no seeker electronics, new guidance law, performance identification or
equipment calibration. JSBSim remains the sole flight integrator; guidance
returns an acceleration demand for the native finite-fin controller.

Runtime now selects separate `SeekerProviderV1` and `GuidanceProviderV1`
interfaces through the [typed native catalogue](payload-native-providers.md).
Their current concrete models are `EncounterSeekerV1` and
`VelocityNormalPnGuidanceV1`, in separate component translation units. New
compatible implementations register their own factories; role/mode names are
not themselves implementation classes. Sensor observations still come only
from World-admitted native inputs.

`StoreCard` and installed SDK layouts are unchanged. The immutable-by-ownership
`components::ProfileV1` sidecar resolves `component_models` from the same store
JSON. Each numeric descriptor role references an existing `card.parameters`
entry, carrying its value, exact unit and provenance. The component adapter
consumes those resolved values, not a second numeric configuration table.

## Selection and typed interfaces

| Kind | `implementation_id` | `configuration_version` | Bindings |
| --- | --- | --- | --- |
| `seeker` | `fsim.seeker.encounter.v1` | 1 | `mode` |
| `guidance` | `fsim.guidance.velocity_normal_pn.v1` | 1 | None |

The seeker mode must be exactly `active_radar`, `semi_active_radar`, `infrared`
or `imaging_infrared`. An explicit descriptor cannot select a mode through a
substring match. Both adapters require availability `available`, their exact
kind/implementation/version, all required numeric roles, finite values, exact
units and supported model domains. Unknown roles or bindings are rejected.
Missing, unavailable, not-applicable or unsupported descriptors cannot silently
become legacy execution. Root composition decides whether a component is
required for that payload and preserves its native refusal semantics.

The private interfaces in `simulation/mission_systems/weapon` are:

```cpp
bool configureSeeker(const components::DescriptorV1&, SeekerConfigV1&,
                     std::string& error);
bool configureSeeker(const components::DescriptorV1&, Spec&,
                     std::string& error);
bool configureGuidance(const components::DescriptorV1&, Spec&,
                       std::string& error);
```

Failure leaves the supplied configuration unchanged and returns a diagnostic.
The seeker-to-`Spec` overload copies the resolved configuration into
`spec.seekerModel` and synchronizes `seeker`, `acquireM` and `gimbalRad`; it does
not enable versioned guidance execution. The guidance overload selects existing
algorithm 1 and sets its own fields and independent capability booleans. It
does not rewrite control or fuze configuration. Shared lifetime/body constraints
remain the responsibility of complete payload composition.

The existing `configureGuidance(StoreCard, Spec, error)` overload, constructors,
inputs and outputs retain legacy compatibility. A `SeekerConfigV1` with version
zero follows the old helper expressions. An absent external sidecar does not
implicitly enable version-one execution. Legacy `Spec` defaults remain
1.047 rad gimbal and .349 rad loft; World-created defaults use the existing
45/60-degree and 0/20-degree conversions. These distinctions are preserved.

## Atomic composition

The native armed-guided composition entry point is:

```cpp
bool configureCompositionV1(const StoreCard&, const components::ProfileV1&,
                            const Spec& baseline, Spec& output,
                            std::string& error);
```

`weapon/Composition.cpp` invokes the actual seeker, guidance, plant, fuze and
warhead adapters against a local candidate. Schema zero returns the exact
baseline without enabling a component. Explicit schema one requires the same
store identity and all nine correctly identified component roles. Armed guided
composition requires an authored AAM seeker, finite control and both effect
providers; unavailable required providers refuse composition.

The selected seeker mode must equal the card's retained `seeker` identity so
on-rail, release admission and execution cannot disagree. When the card has an
authored guidance-capabilities version-one declaration, its booleans must match
the selected numeric roles. An absent legacy declaration is not interpreted as
an authored claim that a capability is absent. Unknown capability versions
refuse composition.

Guidance and fuze must select exactly the same lifetime in seconds; the fuze
cannot silently replace the guidance lifetime. A V1 fuze's selected body length
and diameter must equal the actual card's canonical positive finite `length_m`
and `diameter_m` values in metres. Its station checks therefore cannot rely on
a different, oversized surrogate body. Output is published only after all
providers and shared constraints succeed. Failure leaves the caller's output
unchanged. Inert and ballistic release applicability remains with World's
separate native plant checks.

This is the initial compiled typed-adapter capability. The seeker and guidance
implementation IDs select implementations compiled into the SDK; there is no
general provider registry, runtime plugin registration or external-DLL provider
ABI. A new implementation requires native dispatch/code changes and a rebuild.

## Required seeker roles

Each row is a required `parameter_refs` role; the actual flat card parameter key
may differ. The retained values describe the current generic assumptions and
are not measured equipment specifications.

| Role | Exact unit | Retained value |
| --- | --- | --- |
| `acquire_m` | `m` | ARH 17000; SARH 60000; either IR mode 13000 |
| `gimbal_deg` | `deg` | Radar 60; IR 45 |
| `launch_fov_deg` | `deg` | Radar 55; IR 25 |
| `minimum_range_m` | `m` | 1 |
| `aspect_speed_floor_m_s` | `m/s` | 1 |
| `afterburner_threshold` | `1` | .5 |
| `ir_aspect_min` | `1` | .45 |
| `ir_aspect_span` | `1` | .55 |
| `ir_afterburner_multiplier` | `1` | 1.35 |
| `radar_rcs_floor_m2` | `m^2` | .001 |
| `radar_rcs_reference_m2` | `m^2` | 5 |
| `radar_rcs_exponent` | `1` | .25 |
| `radar_range_scale_min` | `1` | .4 |
| `radar_range_scale_max` | `1` | 1 |
| `radar_lookdown_cosine` | `1` | -.03 |
| `radar_notch_m_s` | `m/s` | 28 |
| `illumination_beam_multiplier` | `1` | 3 |
| `aircraft_signal` | `1` | 1 |
| `ir_decoy_peak` | `1` | 6 |
| `ir_decoy_decay_s` | `s` | 2 |
| `radar_decoy_peak` | `1` | 3 |
| `radar_decoy_rise_s` | `s` | .3 |
| `radar_decoy_decay_s` | `s` | 6 |
| `decoy_resistance` | `1` | Imaging IR 5; other modes 1.6 |
| `association_cosine` | `1` | .999390827019096 |
| `emitter_power_w` | `W` | 20 |
| `emitter_peak_gain_dbi` | `dBi` | 25 |
| `emitter_sidelobe_dbi` | `dBi` | -15 |
| `emitter_half_width_rad` | `rad` | .35 |

`association_cosine` retains the exact existing comparison literal, originally
described as a two-degree cell. Recomputing its cosine from a rounded angle
would change the last bits and its strict boundary. The adapter keeps the
original `>` comparison. The notch likewise retains strict look-down and
absolute target-radial-speed comparisons. Neither introduces a new Doppler
model. Mathematical constants and finite-arithmetic guards remain implementation
contracts rather than payload performance settings.

## Required guidance roles

| Role | Exact unit | Retained value or meaning |
| --- | --- | --- |
| `navigation_gain` | `1` | 3.5; supported domain greater than 2 |
| `cue_deg` | `deg` | 10 |
| `activation_ratio` | `1` | 1.15 |
| `loft_deg` | `deg` | Radar 20; IR 0 |
| `loft_acceleration_g` | `1` | 2 |
| `max_g` | `1` | Radar 35; IR 40 |
| `life_s` | `s` | Radar 110; IR 45 |
| `inertial_midcourse` | `1` | Exact 0 or 1 from the card's existing declaration |
| `launcher_updates` | `1` | Exact 0 or 1 from the card's existing declaration |

Capabilities are independent authored choices, including for SARH and imaging
IR. They are not inferred from mode, designation, availability of target truth
or another card's declaration. The existing numerical law, lateral projection,
midcourse loft, gravity compensation, clipping and timing calculations retain
their evaluation order. Gravity conversion remains 9.80665 m/s².

## World integration and lifecycle

World owns actual release admission, scene enumeration, captured kinematics,
launcher-designation support, current illumination, modeled RCS/RF/terrain
queries, fuze/contact/damage decisions and events. It supplies body-fixed
boresight and current admitted observations. Descriptor selection does not
expand observation authority or invent observations from a raw target ID.

The configured observation helpers are:

```cpp
infraredAcquisitionRangeM(baseM, aspectCosine, afterburning, config);
radarAcquisitionRangeM(baseM, sigmaM2, modelled, config);
radarNotched(verticalCosine, targetRadialSpeedMs, config);
decoySignal(ageS, infrared, config);
withinIlluminationBeam(bodyRay, azimuthRad, elevationRad,
                       azimuthWidthRad, elevationWidthRad, config);
```

World uses `minimumRangeM`, `aspectSpeedFloorMs`, `afterburnerThreshold` and
`aircraftSignal` at the existing candidate gates, `launchFovRad` for the
on-rail/release gate, and the emitter fields for the existing active-seeker
emission. Actual terrain/illumination queries remain in World. The original
mode-specific applicability of each equation is unchanged, including the
base-range treatment of decoys and the aircraft-only IR aspect adjustment.

`Guidance` owns a value copy of `Spec`, including `SeekerConfigV1`, and all
active/held/estimate/accepted-age state for one actual release. No adapter
retains card/descriptor pointers or global mutable execution state. Updating a
card or constructing another release cannot change an executing instance.
Configuration and seeker/guidance processing add no RNG draws.

The existing update stages seeker state before calculating the complete finite
command and commits only on an accepted update. A finite arithmetic overflow
must not advance acquisition, estimate or accepted-clock state. The extraction
keeps this transaction intact. Existing `Vehicle.reset` behavior does not
implicitly replace this per-release object or rearm its lifecycle.

## Focused regressions

`tests/simulation/test_weapon_guidance.cpp` provides exact selectors:

- `component seeker configuration`: all four modes, resolved references and
  synchronization without an implicit guidance upgrade.
- `component seeker refusal`: strict/atomic implementation, version, mode,
  availability, unit, role and domain rejection.
- `component seeker arithmetic`: independent IR/RCS/notch/decoy/beam equations
  and actual native configured association/discrimination.
- `component guidance selection`: existing PN dispatch consumes configured
  gain while preserving independent control/fuze settings and refusals.
- `component output equivalence`: retained four-mode settings preserve phase,
  timing and acceleration outputs across acquisition, decoy and loss updates.
- `component release independence`: each execution owns configuration values,
  held contact and accepted-clock history.

An unmatched selector returns a nonzero exit status. Existing guidance,
on-rail, finite-overflow and capability regressions remain in the same runner.
These focused tests do not replace complete native release, installed SDK,
recording/Viewer or unchanged scene-outcome acceptance.
