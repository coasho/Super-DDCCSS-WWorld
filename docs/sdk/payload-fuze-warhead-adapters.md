# Native fuze and warhead adapters

These internal versioned providers execute the existing native fuze and damage
functions. They select existing reference models and preserve their mathematics,
random draws, captured-motion requirements and initiation lifecycle. The provider
selection does not establish physical calibration or close open physical-model
acceptance gates. StoreCard and public SDK layouts are unchanged.

The shared `components::DescriptorV1` resolves `parameter_refs` against the same
card parameter snapshot. Each available declaration requires configuration version
1, explicit provenance, a note, validity, the complete provider roles below, and
the exact string bindings. Unknown roles, bindings or implementations refuse
construction. Unavailable and not-applicable descriptors return no executable
adapter and an explicit reason; callers must retain that availability instead of
substituting another provider. An absent profile remains separately identifiable
legacy execution; it is never silently upgraded to a V1 physical law.

## Warhead providers

`damage::makeWarheadAdapter(card, descriptor, error)` returns an owned immutable
native warhead snapshot. `model()` exposes its burst station and source metadata.
`assess(burst, geometry, target, rng)` dispatches to the unchanged native `assess`
with the caller's existing damage RNG and attached-geometry membership. Construction
also verifies resolved values and provenance against the native card fields, so a
selected descriptor cannot label a different card model.

| Implementation | Required parameter roles and units |
| --- | --- |
| `native.warhead.legacy.v1` | `warhead_kg`: kg; `explosive_kg`: kg; `gurney_kms`: km/s; `diameter_m`: m |
| `native.warhead.side_case.v1` | The four legacy roles, plus `warhead_model_version`: 1; `warhead_fragment_source`: 1; `warhead_tnt_equivalent_kg`: kg; `length_m`: m; `warhead_structure_fraction`: 1; `warhead_diameter_ratio`: 1; `warhead_case_density_kg_m3`: kg/m^3; `warhead_explosive_density_kg_m3`: kg/m^3; `warhead_structure_density_kg_m3`: kg/m^3; `warhead_fragment_cd`: 1; `warhead_station_x_m`, `warhead_station_y_m`, `warhead_station_z_m`: m |

The legacy provider preserves the existing fragmentation or rod source. The
side-case provider requires existing model/source version 1 and the native
annular side-case constructor's full domain checks. The descriptor configuration
version identifies this interface; it does not change the native source version.

| Binding | Legacy value | Side-case value |
| --- | --- | --- |
| `burst_reference` | `captured_weapon_cg` | `captured_authored_station_from_nose` |
| `blast` | `kingery_bulmash.swisdak_1994.free_air.v1` | same |
| `penetration` | `thor.aluminium_2024_t3.v1` | same |
| `assessment` | `native.fragment_blast.v1` | same |

World supplies the actual captured position, velocity, nose axis and angular
velocity of the selected burst reference. The adapter does not generate a pose
or authorize target truth. Inert body contact and its mechanical consequences
remain independent of the availability of an armed warhead.

## Fuze providers

`weapon::makeFuzeAdapter(descriptor, nativeSpec, error)` copies the native Spec and
sets only its fuze timing, radius, law, delay and sensor station from the explicit
roles. Non-fuze configuration is retained. `spec()` is the selected configuration
for World's initiation windows and captured sensor-reference transformations.
The caller's Spec is unchanged on success and refusal.

| Implementation | Required parameter roles and units |
| --- | --- |
| `native.fuze.legacy.v1` | `model_version`: 1, value 0; `algorithm`: 1, value 0; `arming_s`, `life_s`: s; `radius_m`: m |
| `native.fuze.turning_cpa.v1` | `model_version`: 1, value 1; `algorithm`: 1, value 1; `arming_s`, `life_s`, `delay_s`: s; `radius_m`, `station_x_m`, `station_y_m`, `station_z_m`, `body_length_m`, `body_diameter_m`: m |

The legacy provider executes the existing constrained-minimum `sweptPathFuze`.
The turning-CPA provider executes `FuzeTurnStateV1::evaluate`, including retained
contiguous same-target corner history. It uses the existing `configureGuidance`
V1 parser for body-envelope, timing and model-domain validation. A legacy provider
cannot acquire delay or authored-station behavior through incidental defaults.

| Binding | Legacy value | Turning-CPA value |
| --- | --- | --- |
| `sensor_reference` | `captured_weapon_cg` | `captured_authored_station_from_nose` |
| `target_reference` | `captured_target_cg` | same |
| `contact_reference` | `captured_authored_nose` | same |
| `contact` | `native.authored_point_contact.v1` | same |
| `history` | `native.captured_piecewise_linear.v1` | same |
| `delay` | `none` | `native.captured_fixed_delay.v1` |

`evaluateProximity` consumes captured paths and the native eligible interval. It
does not extrapolate or turn a retained estimate into tracking/fuze authority.
`evaluateContact` calls the existing authored-geometry contact sweep. Its
ContactWindow must use `spec().armingS` and `spec().lifeS`; disagreement refuses
evaluation. Compare all targets before calling `commitContact` for the globally
earliest crossing. An unarmed crossing still consumes only the contact channel.

Each adapter owns independent contact, turn-history and delayed-initiation state.
`scheduleDelay`, `dueDelay` and `cancelDelay` execute the existing delay state
machine with the selected Spec. The captured native pose at the returned due age
must still exist before World can apply detonation. Target loss or switch does
not replace the pending delay's captured target/time.

`resetTurnHistory()` clears transient CPA history only. `resetRelease()` resets
all initiation state and is reserved for a new actual release. Vehicle.reset,
seeker reacquisition and decision boundaries must not rearm contact or delay.
Native state accessors support migration of existing internal call sites; normal
execution goes through the selected adapter operations.

When declaring legacy fuze scalars in the flat parameter map, use keys such as
`component_fuze_arming_s` in `parameter_refs`. Adding `fuze_arming_s` to an old
card would independently opt the existing native parser into its complete V1
block. Existing V1 cards reference their existing `fuze_*`, `life_s`, `length_m`
and `diameter_m` values without changing them.

## Reference inputs and limits

The per-card empirical inputs are exactly the roles above. Legacy dimensions,
mass, explosive charge and Gurney energy retain their existing estimates; legacy
zone weights, structure fraction, diameter ratio, material densities, rod speed
cap and drag coefficient remain the unchanged native reference. Side-case V1
already makes its densities, geometry ratios, drag and independent TNT equivalent
explicit. The fixed blast and penetration bindings select existing fit tables,
obliquity treatment and material conventions; they introduce no new coefficients.

| Fixed native reference input | Existing value/convention |
| --- | --- |
| Legacy densities: case, explosive, structure/skin | 7850, 1700, 2780 kg/m^3 |
| Legacy inert structure share, warhead/body diameter ratio | 0.15, 0.9 |
| Legacy TNT Gurney-energy proxy, rod speed cap, fragment Cd | 2440 m/s, 1400 m/s, 1.3 |
| Legacy fragmentation zones `(from deg, to deg, share)` | `(20,60,.075)`, `(60,80,.10)`, `(75,100,.60)`, `(100,120,.10)`, `(120,160,.075)`; shares normalized by the native constructor |
| Legacy rod zone | `(85,95,1)` |
| Blast reference atmosphere | 101325 Pa, 288.15 K; free-air equivalent uses TNT mass/2 |
| THOR `(c, alpha, beta, gamma, lambda)` | `(7.047, 1.029, -1.072, 1.251, -.139)` in the existing inch/in^2/grain/ft/s convention |
| THOR obliquity convention | Absolute angle capped at 1.45 rad; cosine floor 0.12 |
| Target skin model | Aluminium density 2780 kg/m^3; inferred thickness clamped to 0.0005..0.02 m, unavailable-section/area fallback 0.002 m |
| Target fragment/blast reporting conventions | Fragment relative speed must exceed 1 m/s; retained blast threshold 1000 Pa; half-panel projected area for blast impulse |
| Legacy surface/shield ordering approximation | Virtual surface pair at plane +/-0.001 m; component shielding lookahead less than 1 m when outside all body panels |

The unchanged Kingery-Bulmash polynomial inputs are listed here as fixed
reference data. Each row gives the scaled-range interval and five coefficients
for `exp(A + B*ln(Z) + C*ln(Z)^2 + D*ln(Z)^3 + E*ln(Z)^4)`.

| Fit | Scaled-range interval | A, B, C, D, E |
| --- | --- | --- |
| Pressure | 0.2..2.9 | 7.2106, -2.1069, -0.3229, 0.1117, 0.0685 |
| Pressure | 2.9..23.8 | 7.5938, -3.0523, 0.40977, 0.0261, -0.01267 |
| Pressure | 23.8..198.5 | 6.0536, -1.4066, 0, 0, 0 |
| Impulse | 0.2..0.96 | 5.522, 1.117, .6, -.292, -.087 |
| Impulse | 0.96..2.38 | 5.465, -.308, -1.464, 1.362, -.432 |
| Impulse | 2.38..33.7 | 5.2749, -.4677, -.2499, .0588, -.00554 |
| Impulse | 33.7..158.7 | 5.9825, -1.062, 0, 0, 0 |

Numerical protections are distinct from those empirical choices: ray-direction
parallel threshold 1e-12 and quad-plane denominator threshold 1e-9; fragment mass
denominator floor 1e-9 kg; legacy cylinder length denominator floor 0.001 m;
blast atmosphere floors 1 Pa and 100 K; finite/integer fragment-count checks;
and contact mesh limits of 1,000,000 vertices and 2,000,000 triangles. The exact
authored contact sweep introduces no contact radius or fitted epsilon. Its
closed-mesh topology, initial-overlap refusal, unavailable-sample refusal and
exact chronology are existing acceptance conditions. Fuze time/radius/lifetime
roles are model inputs; finite arithmetic checks and `nextafter` are numerical
boundary handling. No new numerical tolerance is added by these adapters.

Mathematical and unit constants remain pi, the cylinder/annular and projection
identities, 1000 m/km, 0.0254 m/in, 0.3048 m/ft, and the existing THOR grain
conversion 6.479891e-5 kg/grain. They are not configurable empirical coefficients.

Native free-air Kingery-Bulmash/Sachs pressure/impulse fits, reflected-pressure
relations, THOR penetration coefficients, skin fallback rules, fragment stopping
rules and the 1000 Pa blast reporting threshold remain in Damage.cpp. Numerical
guards for parallel rays, finite arithmetic, source/body bounds and resource
counts are still the original implementation's guards. Fuze windows retain exact
inclusive arming, exclusive lifetime/cutoff semantics and existing nextafter
handling. Unit conversion factors, pi and geometric identities are distinguished
from empirical fits. Target component failure, fuel/fire, debris response and
collision mechanics belong to their existing native target-response models.

The regressions compare selected providers with direct native output and RNG
state, distinguish the legacy and explicit law, and exercise refusal and the
three independent lifecycle states. This is software dispatch/preservation
evidence, not validation of weapon effectiveness or fitted physical fidelity.
