# Fuel transit: physical method and implementation requirements

Date: 2026-11-03. Status: the additive private liquid scene and native capture
binding are implemented, with the scoped native receipts below. The initial
preintegration audit and proposal are retained as historical observations;
they do not describe the current opted-in production path. Current all-model
production, full-program acceptance and equipment-specific fluid-drag, ignition
or lethality calibration are separate requirements and are not established here.

## Primary sources and their scope

N. A. Moussa, M. D. Whale, D. E. Groszmann and X. J. Zhang,
*The Potential for Fuel Tank Fire and Hydrodynamic Ram From Uncontained Aircraft
Engine Debris*, DOT/FAA/AR-96/95, January 1997:

- [Section 4.2, printed pp.22-23, PDF pages 34-35](https://rosap.ntl.bts.gov/view/dot/13265/dot_13265_DS1.pdf#page=34):
  equation (4) models quadratic drag; equation (5) gives finite velocity decay.
  Ullage and liquid differ in density and deceleration. The liquid example
  "slows down very quickly in the liquid"; it does not stop at entry.
- [Table 6, printed p.25, PDF page 37](https://rosap.ntl.bts.gov/view/dot/13265/dot_13265_DS1.pdf#page=37):
  the report's Jet-A example uses liquid density 800 kg/m3. This is not a
  universal density for every maintained fuel or condition.
- [Section 5.1, printed p.41, PDF page 53](https://rosap.ntl.bts.gov/view/dot/13265/dot_13265_DS1.pdf#page=53):
  ERAM transfers projectile energy to fluid and models pressure separately.
  Its drag coefficient depends on tumbling and test fitting. The report concerns
  engine debris, not calibrated missile-warhead fragments.

Additional primary experiments establish relevant limits, not replacement
coefficients:

- D. Varas, R. Zaera and J. Lopez-Puente, [*Experimental analysis of fluid-filled
  aluminium tubes subjected to high-velocity impact*](https://doi.org/10.1016/j.ijimpeng.2008.04.006),
  *International Journal of Impact Engineering* 36 (2009), pp.81-91: steel spheres,
  water-filled aluminium specimens and different fill volumes. Water/test-shape
  results cannot be relabeled kerosene/irregular-fragment calibration.
- E. Deletombe, J. Fabis, J. Dupas and J. M. Mortier,
  [*Experimental analysis of 7.62 mm hydrodynamic ram in containers*](https://doi.org/10.1016/j.jfluidstructs.2012.11.003),
  *Journal of Fluids and Structures* 37 (2013), pp.1-21: projectile/fluid dynamics,
  cavity evolution and container-boundary effects require separate observations.

## Historical preintegration implementation gap (2026-11-03)

This section records the source audit before liquid-scene integration. The
legacy entry points retain their original contracts; current declared liquid
recipients use the additive path described under current production integration.

The inspected pre-transit implementation has two perfect-component-absorption
paths in [Damage.cpp](../../../src/simulation/mission_systems/damage/Damage.cpp):

1. The legacy `assess` component branch deposits all remaining kinetic energy
   and sets the fragment velocity to zero.
2. `assessSceneV1` initializes residual speed to zero; its component branch never
   replaces that value. The subsequent absorption branch removes the complete
   material element from continuation, including a fuel-volume interception.

At that audit, the [Target and scene contracts](../../../src/simulation/mission_systems/damage/Damage.h)
contained motion and atmospheric inputs, but no current liquid occupancy.
`Airframe::Component::fuelKg` is authored capacity material, not current native
contents. [WorldDamage.cpp](../../../src/simulation/runtime/world/WorldDamage.cpp)
did not pass tank contents to those fragment assessments. Empty and full fuel
boxes consequently provide the same stopping behavior. This differs from the
actual contents already available to native leakage/consequence processing in
`damage::Flight::tankKg` and detached-piece fuel state.

The audit found a second, independent occupancy gap. Hangar's vulnerability tank
construction initially uses capacity divided by density, but `_fit` shrinks
the damage probe box without conserving that volume. The serialized nominal
volume metadata is not its actual box volume. The inspected preintegration examples
include F-16 wing probes with 210 kg nominal capacity but approximately
0.03999 or 0.01944 m3 box volume; H-6K's 2750 kg wing probe occupies about
0.010328 m3, and MiG-29 tank 1's 1200 kg probe about 0.009618 m3. These are
probes, not demonstrated capacity-conserving liquid containers. Dividing their
capacity by those box volumes would invent liquid densities of thousands to
hundreds of thousands of kg/m3. Neither a hydrostatic inverse nor a finite drag
law repairs invalid occupancy geometry. Production must first supply physically
volume-conserving tank geometry and authoritative contents; no density smearing
or expanded damage probe is justified merely to pass an engagement.

The existing effects guide declares first solid-component absorption as a
reduced model. A capacity fuel volume cannot thereby become a solid absorber
independent of its contents. Correcting this distinction does not imply that
fuel ceases to shield downstream components, or that any frozen engagement must
produce engine, crew or control loss.

## Derived finite-drag reference

For constant mass, presented area, coefficient and stationary homogeneous fluid,
write relative speed as `v`, density as `rho`, and

```text
k = rho * Cd * A / (2*m)
dv/dt = -k*v*v
v(t) = v0 / (1 + k*v0*t)
x(t) = log1p(k*v0*t) / k
v_out(L) = v_in * exp(-k*L)
t_cross(L) = expm1(k*L) / (k*v_in)
```

These are derived constant-coefficient references, not fitted performance.
Their zero-density/zero-length limits retain incoming motion. They provide an
independent oracle for finite depth and time subdivision. Relative kinetic-energy
loss and momentum transfer follow from the incoming/outgoing states; finite
liquid depth does not authorize instantaneous complete absorption.

## Minimum concrete inputs

The production policy needs the following inputs at the physical
encounter epoch. These are material/motion data, not navigation estimates or
actor observations.

| Input | Required provenance and meaning |
| --- | --- |
| Victim identity and generation | Existing native scene identity; reject stale or replaced bodies. |
| Tank identity, capacity and current contents in kg | Captured native tank properties; detached bodies use their existing authoritative per-source fuel partition. Capacity never substitutes for contents. |
| Fuel kind and density policy | Explicit authored material selection and reference condition. Hangar currently distinguishes jet fuel and avgas; do not turn the Jet-A example into a universal value. |
| Liquid-containing volume | Capacity-conserving authored or inferred cells, native tank membership and attached ownership, with the inference declared. Fitted damage probes do not supply this volume. Overlaps and shared ownership must not duplicate volume. |
| Fill/occupancy policy | A mass-conserving liquid region or explicitly labeled homogenization. A scalar fill fraction alone does not determine the chord of liquid encountered by a particular ray. |
| Own material motion | Captured native origin, attitude, translation and angular motion at residence time; velocity is relative to the local medium. |
| Projectile material state | Actual source ordinal, mass, point or finite-rod geometry, velocity, projected area and separately justified fluid-drag law. |
| Time bounds | Encounter epoch, ordered motion/material samples and remaining field lifetime. No future interpolation or extrapolation beyond admitted captures. |

For a simple quiescent-tank policy, an explicit free-surface direction and level
can define the liquid region. The level must solve the occupied-volume/mass
constraint over the tank's actual modeled volumes. Assuming body-down settling
is a disclosed approximation; it is not a slosh calculation. Zero/negative-g,
rotation, baffles, bladder motion and compartments require their own validity
limits. Do not infer an instantaneous physical free surface from an unvalidated
pose alone.

An empty tank has no liquid drag. Gas/ullage can retain its separately declared
medium treatment. Existing wall/skin penetration remains; do not remove walls
or invent an extra enclosing THOR plate. Avoid charging ambient-air drag twice
over a liquid-residence interval.

Unknown quantities must remain unavailable. A newly selected fluid-transit
policy must reject missing required material inputs rather than silently assume
a full tank, an empty tank, or a measured coefficient.

## Chronological state and conservation

The proposed implementation must split motion at entry, liquid/gas boundaries,
and exit, comparing those events with every other component/skin intersection.
It must retain the surviving projectile and resume global chronological
traversal. Marking an entry visited cannot replace residence or permit a fragment
to jump across the liquid instantly.

Use local medium-relative drag, and account for its equal momentum transfer to
the victim. Retain the outgoing material mass, residual velocity and epoch. A
finite causal interval can end while material remains inside a tank; continuation
must preserve that state. Time pruning and numerical underflow must not appear
as unexplained mass/energy deposition.

For finite rods, determine the immersed material fraction along the element.
Integrate distributed drag and its moment over that fraction, including differing
endpoint velocities where the selected representation supports them. A partial
endpoint contact does not immerse the whole rod. Connected material must retain
one conserved source population; no cloned fragments or automatic extra damage
may be introduced at a fluid boundary.

Energy deposited by liquid transit is distinct from wall perforation energy,
structural work, retained projectile energy and later combustion. Summing full
incoming kinetic energy into the fuel hit and also propagating a residual
projectile would double-count energy. A reduced energy-transfer model does not
by itself implement pressure-wave loading, cavity collapse, tank rupture or
ignition.

## Historical additive private integration proposal (2026-11-03)

Keep installed public signatures/layouts and existing `Target`,
`SceneVictimV1`, `FragmentFieldV1`, and `WarheadAdapter` vtables unchanged.
One possible private design uses new sibling value types and an opt-in free
function/overload:

```text
FluidTransitPolicyV1
VictimMaterialSnapshotV1 / timed tank-material samples
FluidResidenceContinuationV1
assessSceneWithMaterialsV1(field, victims, material_snapshots, environment,
                          policy, prior_residence)
```

These names were design proposals, not API identifiers; the implemented sibling
path is listed below. Match snapshots by native
identity/generation and residence by immutable source ordinal plus victim,
generation and tank identity. Borrowed geometry/snapshots must outlive only the
synchronous assessment; retained continuation owns values and stable identities,
never Entry pointers. Validate complete input before mutating continuation or
random history. Reset/replacement closes old residence, and a detached body's
fuel uses its existing conserved partition rather than duplicating parent fuel.

An explicit opt-in preserves historical diagnostic behavior at retained entry
points. It does not authorize silently
falling back to the old perfect absorber after new-policy validation fails.

## Current additive production integration (2026-11-03)

[LiquidSceneV1.h](../../../src/simulation/mission_systems/damage/LiquidSceneV1.h)
declares the private `assessSceneLiquidsV1` sibling and its liquid inputs. Its
implementation in [Damage.cpp](../../../src/simulation/mission_systems/damage/Damage.cpp)
advances actual point and distributed rod material through occupied intervals,
retaining residual motion, elapsed work, reaction loads and downstream solid
interception. Capacity-cell cuts are not bladder walls. Ambient air acts only
on dry material; gravity acts once. The detailed event, boundary, numerical and
conservation contracts and their correction history are in
[the native liquid-scene method](liquid-scene-method.md).

[WorldLiquidMaterials.cpp](../../../src/simulation/runtime/world/WorldLiquidMaterials.cpp)
implements `readLiquidMaterialDatasetV1`, `freezeLiquidRecipientV1` and
`World::appendLiquidMaterialV1`. Declared current products must provide valid
capacity-conserving cells, native ownership and explicit material policy;
invalid declared input refuses rather than reverting to the legacy absorber.
The freeze uses captured native tank kg, recipient identity/generation, attached
component ownership and the actual body-frame up direction. A partial tank's
plane is solved from mass divided by physical density over its owned cells;
neither fitted vulnerability-box volume nor half-density substitutes for fill.
The selected free surface remains quasistatic, not a slosh solution.

[WorldDamage.cpp](../../../src/simulation/runtime/world/WorldDamage.cpp) supplies
those snapshots to `assessSceneLiquidsV1` for synchronous and continuing native
effects. Departing bodies retain their own conserved native fuel and shifted
capacity geometry, without duplicating the parent's cells or contents. Existing
`Vehicle.reset` semantics remain per-vehicle: the detached-piece model restores
its authored empty fuel state, while a parent restores its model's authored
initial contents. This is not a full World episode reset.

The current capacity geometry and its assumptions are described separately in
[the occupancy method](liquid-occupancy-method.md) and
[the explicitly inferred RQ4 root-vessel method](rq4-inferred-vessels.md).
An inferred sealed-module fracture rule and a cell centroid differing from a
native lumped fuel anchor are declared model limitations, not OEM tank data or
permission to move the accepted native mass/CG inputs. A source policy does not
establish successful production for every model.

The original public `Target`, `SceneVictimV1`, `FragmentFieldV1` and provider
vtables are unchanged. The new private types and call sites select liquid
semantics explicitly; retained legacy/custom inputs retain their original
entry-point behavior.

## Private method contract and limitations

The new private [FluidTransitV1.h](../../../src/simulation/mission_systems/damage/FluidTransitV1.h)
contains parameter-explicit primitives, not a selected production tank policy:

- `immersedVolume` clips an AABB by `normal dot position <= offset` and
  computes the closed polyhedron's volume. `freeSurface` inverts actual volume
  with explicit absolute/relative volume tolerances and iteration budget. Its
  residual uses the returned, representable plane offset; numerical offset
  quantization cannot be reported as a successful local-only solve.
- `clipImmersedLine` intersects a material segment with both the box and the
  liquid halfspace. A point-only tangency has zero interval length; a segment
  lying along a boundary follows closed-halfspace clipping, with scene ownership
  resolved separately. Invalid inputs remain
  distinct from a valid dry interval.
- `advancePoint` and `advancePointDistance` use the derived exact constant-medium
  quadratic-drag solution. Distance is medium-relative. Results include elapsed
  time, position, residual velocity, medium impulse and relative kinetic-energy
  dissipation. Inertial energy loss equals dissipation plus medium velocity
  dotted with the transferred impulse. Exponential underflow or unrepresentable
  results fail explicitly instead of becoming perfect absorption.
- `rodLoads` uses uniform area and mass per material coordinate `s` in `[0,1]`,
  endpoint-affine position/velocity, and an explicit immersed interval. It
  integrates only that interval. With `N0=1-s`, `N1=s`, its consistent forces
  are `F0=integral(N0*f ds)`, `F1=integral(N1*f ds)` where
  `f=-rho*Cd*A*|v(s)-v_medium|*(v(s)-v_medium)/2`. The endpoint mass matrix is
  `m*[[1/3,1/6],[1/6,1/3]]`; therefore accelerations are
  `(4*F0-2*F1)/m` and `(-2*F0+4*F1)/m`, rather than two independent masses of
  `m/2`. Momentum is `m*(v0+v1)/2` and kinetic energy is
  `m*(|v0|^2+v0 dot v1+|v1|^2)/6`. This retains intrinsic rotational/deformation
  energy in the affine representation. Torque is about the rod midpoint;
  power is `F0 dot v0+F1 dot v1`.

Rod integration uses bounded adaptive Simpson quadrature with positive accepted
weights, explicit depth/evaluation budgets and estimated error. The error
estimate is not a certified bound. A failed or exhausted integration publishes
no partial force/acceleration. Component sums and sums of absolute error
estimates are maintained globally. Refinement splits the largest remaining
error contribution to a component whose global criterion is unsatisfied, within
the original total evaluation/depth budget. Relative tolerances apply to the
global result, rather than larger subinterval values that may cancel. A
relative-only request with a near-zero global component can fail explicitly;
no absolute floor is invented. All fluid density, area and
drag coefficients are caller inputs; no coefficient default or hardware
calibration is supplied. Numerical controls are integration accuracy requests,
not material uncertainty.

The rod method returns instantaneous generalized loads, not an exact finite
time step. A production integrator must select stable chronological steps,
recompute immersion as endpoints and the tank move, preserve the consistent
mass metric, and enforce any separately selected rod rigidity/breakup law. A
moving or rotating fluid velocity field needs subdivision or a further explicit
field model; the current primitive assumes constant translating medium velocity.
It neither models slosh, hydrodynamic ram, cavitation, compressibility, wall
reactions nor ignition. No whole-element drag follows from one wet endpoint.

The corresponding private method tests are in
[test_fluid_transit.cpp](../../../tests/simulation/test_fluid_transit.cpp), tagged
`[fluid-transit]`. **Historical first-run failure, 2026-11-03:** the integration
owner's first native run exposed a real
global-convergence failure in the affine-velocity case: locally satisfied
relative criteria did not meet the final global error guard. The global
refinement correction retained those original test controls. The first failure
is preserved in `fluid-method.log` (6 cases, 167 assertions, one failure).
The subsequent `fluid-method-global.log/xml` rerun passed 6 cases and 194
assertions; later expanded receipts below supersede that narrower passing
snapshot. Source authorship alone was not treated as passing evidence.

## Scoped native receipts (2026-11-03)

Artifacts below are under
`build/architecture-reviews/sim-m03-production-20261103/`. They record specific
source snapshots, not a promise that future source changes inherit a pass.
The [scene method](liquid-scene-method.md) retains the associated source hashes
and the intervening RED/GREEN history.

- `liquid-right-limit-methods-final.log/xml`: bounded, nonempty serial CTests
  pass `fluid_transit_methods` (10 cases / 375 assertions),
  `liquid_scene_methods` (39 / 3,925) and `liquid_material_binding` (9 / 472):
  **58 native cases / 4,772 assertions**. These are three disjoint executables.
- `current-native-materials.raw.log/xml`: `liquid_native_binding` passes
  **3 cases / 1,554,105 assertions / 78.19 s** using the real F-16 products,
  actual native empty/partial/full contents, continuation, ownership/departure
  and reset controls. `fragment_reach_native` separately passes
  3 / 2,393 / 73.66 s. `weapon_effects_population` passes
  **2 / 224,798 / 182.67 s**, including all **26 original authored explosive
  cards** and the existing terminal controls. These are actual native execution
  gates, not proof of every aircraft's tank geometry or complete campaign
  delivery. Their assertion counts must not be summed with earlier overlapping
  snapshots and presented as additional independent coverage.

The separately reported **64 Python liquid methods** concern producer,
packing/publication and collector behavior at their own source epoch. They are
not the 58 native fluid/scene/binding cases and do not replace these native
receipts.

| Retained artifact | SHA-256 |
| --- | --- |
| `fluid-method.log` (original failure) | `5a204502345b71e7b561d93545095fc070da743cead80dc8d3c5f9cfbed1a3ad` |
| `fluid-method-global.log` | `0cadc597ad28a2b80e38eaa8b41268b0c6a5486de2418a724eba73da5f8d1d74` |
| `fluid-method-global.xml` | `9a8b5ff891d9593b7447b2487c929bda63cc3145a2f490820546b40fe9c7a55c` |
| `liquid-right-limit-methods-final.log` | `fc74aa9f7674b00c076c0100a04d815f3f40af2a1087444c85b4c34c15c67ece` |
| `liquid-right-limit-methods-final.xml` | `bdbc1d0cea43f6562a1eaf6a492e53066ec36f213a10386a07058d9b2a271adc` |
| `current-native-materials.raw.log` | `4f5a0ee00aba468647064c63f1b364ef4339839b371a2dee4e249bcf3aa12eb8` |
| `current-native-materials.xml` | `9ce31b110632e63f71061d2582ab81cd60d412f028e321b63879260054edde51` |

## Uncertainty and independent acceptance

Required uncertainty records are the material-property basis, drag-law shape and
orientation assumptions, fill/geometry approximation and their validity ranges.
They are not a fabricated navigation covariance or a calibrated Cartesian
posterior. If statistical input distributions are unavailable, record bounded
assumptions/sensitivity cases and unknown uncertainty rather than zero variance.
Do not copy the projectile's air `Cd` into liquid and call it hardware data.

Independent checks must cover analytic finite-depth/finite-time solutions;
empty/full/partial fill; volume/mass conservation; zero-density/zero-length
limits; residence across several publication intervals; moving-medium momentum;
entry/exit skins and downstream components; partially immersed rods; detached
fuel; generation/reset rejection; and invariance under time subdivision.
Outcome checks must retain misses and surviving aircraft. No changed kill
probability, widened geometry, reduced acceptance threshold or forced downstream
hit is justified by this method.

The scoped implementation and native receipts above do not establish current
all-35-model production, complete campaign/installed/exported/Viewer acceptance,
or equipment calibration. The unchanged 5 km functional predicate and other
full-program gates require their own current evidence; this method note is not
their acceptance record.
