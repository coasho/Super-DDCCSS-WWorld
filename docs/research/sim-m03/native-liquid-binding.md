# Native liquid material binding

This private additive path binds Hangar occupancy to current native fuel. Its
implementation and focused methods are under integration; a producer receipt or
synthetic fluid test alone is not complete native or installed acceptance.

`WorldLiquidMaterials` reads the real design's `liquid-v1.json` beside its
existing damage/flight data. A current damage declaration or concrete sidecar
opts in. Missing, malformed or inconsistent current data refuses vehicle
creation. Truly undeclared legacy inputs retain their original execution.
The parser checks design/schema/frame, structural origin, complete native tank
identities, section ownership/capacity, component mappings, explicit physical
density and liquid drag policy, disjoint positive cells and capacity volume.
It does not independently re-run geometric Boolean containment: generated
boundary/provenance and complete-population admission are separate gates.

The owned world's captured material samples contain optional native tank kg,
read from actual `propulsion/tank[index]/contents-lbs` properties. Unavailable,
negative or nonfinite quantities remain unavailable. Interpolation requires
both actual endpoint samples. Reset clears property bindings and captured
history. Capacity never replaces a missing current sample.

At the encounter epoch, all attached capacity cells of a native tank participate
in a free-surface volume solve. Density is unchanged when contents change. The
body-frame plane uses captured geodetic up; it is a quasi-static equilibrium
placement estimate. Acceleration-dependent effective gravity, dynamic slosh,
bladder deformation and hydrodynamic ram are not solved by this plane. Numerical
volume tolerances are separate from those physical uncertainties.

Detachment retains the original source component/tank identities and the actual
transferred fuel quantity. A child's cell bounds shift by its accumulated
structural-origin displacement, matching its translated native damage geometry.
The parent's departed tank has zero current fuel and no attached capacity. No
child reads the parent's remaining fuel as its own. A positive quantity without
owned capacity refuses the material assessment.

Shared-scene execution freezes these inputs before any consequence is applied.
A causal continuation uses its interval-start capture and frozen body-frame
plane. Required unknown input refuses the scene; it does not select the old
perfect-absorber fuel boxes. An external adapter using its own original
`assess` contract does not consume or require this additive material input.

Only actual relative drag work is deposited. A later energy increment with no
new puncture uses the existing exponential ignition hazard through the additive
nonvirtual `VehicleDamage::applyLiquidEnergyV1` method. It cannot manufacture
hits or hole area. Referee diagnostics retain the actual energy increment while
physical hit counters remain unchanged. Existing public layouts, provider
vtables, actor reports and old scene entry points are unchanged.

The default producer policy explicitly assumes liquid Cd 1 for unresolved
tumbling pressure drag and a factor-two screening range (Cd 0.5 to 2). This is
not a confidence interval or equipment calibration. Point/rod analytic
sensitivity, chronological shared-scene tests, actual World empty/partial/full
contents, reset/separation, complete generated products, installation and
original combat/presentation gates must each provide their own evidence.

See [fuel transit sources](fuel-transit-method.md),
[occupancy geometry](liquid-occupancy-method.md) and
[chronological scene integration](liquid-scene-method.md).
