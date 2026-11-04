# Finite liquid transit in the native material scene

`assessSceneLiquidsV1` is an additive execution path in `Damage.cpp`, declared
in the new private `LiquidSceneV1.h`. It advances the existing material
population chronologically through actual occupied liquid and then through
downstream structural skin/components. It is not an occupancy-only query or a
claim that the earlier pure fluid primitives constituted production execution.
The original `assess`, `assessSceneV1`, existing structs and provider virtual
layouts retain their contracts. The old scene entry point selects the same
null-liquid branches; unlisted recipients retain the old component semantics.

## Inputs and admission

Each opted-in recipient binds the existing victim identity and generation.
Capacity cells carry a stable nonzero runtime cell ID, native tank ID, original
component index and owner section. Serialized zero-based cell IDs are mapped by
the native reader to `stableCellId = id + 1`; no source ordering is inferred.
Cells are positive finite AABBs with mutually disjoint interiors. Their faces
partition capacity, not material walls. A deterministic median-split BVH
supports up to 65,536 cells per recipient and admits point/line/swept queries.
An overlapping cell is rejected rather than accepted with a packing tolerance.

Tank state supplies actual captured contents, physical density, an explicit
liquid drag coefficient and one frozen body-frame free-surface plane. No air Cd,
capacity mass, vulnerability-box fit or invented density substitutes for these
inputs. The caller's plane-volume ledger must agree with
`actualContentsKg / densityKgM3` within the declared numerical volume budget.
Positive contents without local capacity is invalid. Every attached fuel probe
must have its native tank state; several probes may refer to the same aggregate
tank. A zero-current orphan state with no local cells is valid after its whole
capacity owner has separated. Each actual cell still maps to an attached probe
of its real native tank and section.

Input refusal is transactional. Stale/duplicate recipient identities, unknown
cells/tanks, nonfinite state, missing coefficients, invalid numerical controls,
volume inconsistency, overlapping capacity or integration-budget exhaustion
returns an invalid assessment without mutating caller state or RNG. Only a
valid opted-in recipient suppresses its fuel probes' previous perfect-absorber
behavior. Other solid components and actual structural THOR skins remain active.

The producer's geometric meaning and uncertainty are specified in
[the occupancy method](liquid-occupancy-method.md). Native contents, shifted
departed-body origins, tank ownership and capture epochs belong to the caller's
native integration; the scene function does not create those observations.

## Point transit and event time

The reduced pressure-drag law is explicit:

```
u = fragment velocity - local material velocity
F = -0.5 * rho * Cd * area * |u| * u
k = rho * Cd * area / (2 * fragment mass)
```

Actual immersed material displaces ambient air. Point air drag is absent while
that point occupies liquid. For a finite rod, actual wet material intervals
from all admitted cells are sorted and united using exact endpoint comparisons;
the original air law and full-rod shape functions are integrated only over the
remaining dry intervals. Neither a wet fraction of whole-rod force nor added
air-plus-liquid density is used. Occupancy determines this exclusion even for
material comoving with the liquid, where liquid force and work are zero. Dry
capacity and zero-current tanks do not exclude air. Gravity remains active once
over the complete fragment mass.

For a homogeneous translating medium with no air/gravity forcing, between
physical occupied-region boundaries the point solution is analytical:

```
speed(t) = speed0 / (1 + k * speed0 * t)
distance(t) = log1p(k * speed0 * t) / k
speed(distance) = speed0 * exp(-k * distance)
time(distance) = expm1(k * distance) / (k * speed0)
```

Occupied intervals belonging to the same tank are merged before scheduling
entry/exit, so capacity subdivision cannot add walls, stopping work or spurious
time events. At a shared face the direction of motion assigns the occupied
side. Point clipping uses the original AABB planes and long-double offsets,
so shared source planes produce identical intersection endpoints. Only exactly
touching occupied intervals are united; a real dry gap is retained. Actual drag work and impulse are still
apportioned to each crossed cell's original component index; `expm1` preserves
small attenuation increments. A finite time window never receives a future
full-chord loss or future downstream impact. A downstream point intersection's
chord fraction is mapped back to its actual flight time by bounded bisection.

A point's phase integrator uses the same labelled ownership partition as its
forces and work. Touching spans with the same native tank and probe remain one
smooth phase. A change of admitted tank/probe, including a stable-owner change
along a tangential shared interface, ends that phase even if both tanks
separately have continuous capacity there. Actual body-plane signed distances
bracket the event on the integrated trajectory, including recipient advection.
All RK4 stages up to that event use the pre-event medium; directed ownership
then starts the following phase. This avoids sampling a discontinuous future
medium at the last RK4 stage and mistaking that force jump for smooth-method
truncation error. Homogeneous translating zero-gravity phases use the exact
point law above, including an air phase. There is no initial-tank whole-window
shortcut. A point's outer free-air chord query only selects possible liquid
flight and the existing numerical maximum step; it does not maintain a second,
inconsistent boundary clock. Rods retain their distributed interval horizon.

At an exact material face with zero relative normal velocity, instantaneous
physical relative acceleration selects the right-hand temporal phase. This
classification is required before integrating a positive interval: a point
accelerating immediately out of a tank cannot charge that tank's work merely
because another tank is reachable later. The relative acceleration includes
the captured rotating-medium velocity derivative. When both normal velocity
and normal acceleration are zero, the existing stable-cell tie convention is
retained. This rule moves no coordinate or event time and does not close a
finite gap; subsequent entries still use actual trajectory/plane roots.

With air/gravity forcing or spatially varying material motion, adaptive RK4
step doubling integrates position, velocity and reaction loads. A whole step
and two half steps must satisfy the explicit absolute/relative position and
velocity budgets. Failure to meet them before the minimum step or iteration
bound refuses the assessment. Air and gravity enter this integration once;
they are not also applied by the legacy flight update. Captured material
intervals and the additional liquid maximum step bound geometry approximation.

The liquid scene performs these calculations in one frozen burst-origin ECEF
translation. Burst/projectile positions and every recipient origin, including
captured motion samples, share that translation. Physical continuation positions
are converted back once. Velocity, rotation bases, air/gravity, impulses and the
moving-origin angular convention are vectors or relative quantities and remain
unchanged. This prevents repeated sub-ULP Earth-coordinate additions from
erasing real displacement. It cannot recover precision already absent from an
input ECEF double. Phase and outer event ages use the represented upper
absolute-time endpoint, capped by the existing captured/window boundary, rather
than adding an unrepresentable positive duration forever. This is at most one
clock quantum, without a spatial tolerance, geometry nudge or larger iteration
budget. Body-relative phase admission, root evaluation and torque arms form the
same `point - (origin + velocity*time)` expression at a common epoch; mixing
floating-point associations can otherwise select different sides of one face.
The first actual fluid-work phase retains its elapsed start time, so dry travel
does not acquire an encounter timestamp at the start of the interval.

## Distributed finite rods

A rod retains uniform mass and presented area over its affine material
coordinate `s in [0,1]`. Each cell and the actual free surface clip the truly
immersed interval. `fluid::rodLoads` integrates the nonlinear local relative
velocity only over that interval, using adaptive quadrature with explicit
absolute/relative force and evaluation budgets. It does not scale the fluid
density by a wet fraction or replace the rod with two independently wet points.

Closed capacity cells can both contain a rod lying exactly on their common
face. They do not authorize two copies of that material's drag. The liquid
path clips against original AABB/free-surface planes and sweeps the represented
interval endpoints, retaining exact positive gaps. Within one recipient each
material subinterval has one admitted cell. Nonzero relative boundary-normal
motion selects the entered side; affine endpoint velocity can change that side
along a rod, so its exact represented normal-velocity zeros also split the
sweep. An exactly tangential interface uses the smallest stable cell ID,
independent of serialized storage order, for both the medium and original probe
attribution. This rule also resolves exact interfaces between distinct native
tanks without adding their densities or coefficients. It is an explicit
zero-radius material-line boundary convention, not a resolved cross-section
mixture or measured interface-flow law. Point admission and its analytical
chord work apportionment use the same unique ownership; duplicated closed-cell
membership cannot add momentum, energy or torque. Endpoint events and an
ordered active-cell set bound the partition work by `O(n log n)` for `n` queried
cells rather than comparing every pair of interval pieces.

For endpoint shape functions `N0=1-s`, `N1=s`, the generalized endpoint forces
and accelerations use the consistent mass matrix:

```
Fi = integral(Ni * local_force_density ds)
M = mass/6 * [[2,1],[1,2]]
a0 = (4*F0 - 2*F1) / mass
a1 = (4*F1 - 2*F0) / mass
```

Thus partial immersion can accelerate one endpoint while the total material
momentum still decreases. Rod endpoint positions and velocities advance with
the same adaptive time method. Local material velocity includes captured
recipient translation and `omega cross material_offset`; quadrature therefore
uses different relative velocities at different material coordinates. Initial
entry and geometric exit bounds prevent a thin immersed interval from being
skipped solely because all coarse quadrature times lie beyond it. The connected
ring's existing finite-length break condition is evaluated on the propagated
liquid-aware positions, with no resampling or new material.

## Loads, punctures and conservation

The boundary of each native tank's capacity-cell union represents a negligible
bladder wall. Only an actual exposed-face intersection records its one puncture
area per tank and projectile, retained in the existing continuation history
under a separate private kind value. A point or entire rod born strictly inside
capacity has crossed no wall. Its initial occupancy cannot create a hole; a
later real exit can. A true initial face straddle remains a physical contact.

The scene lazily caches exposed rectangular face patches. The existing BVH
selects neighbors; exact coplanar opposing-face coverage is subtracted only for
the same native tank. Partial coverage retains the exposed patches. Internal
cell cuts disappear without a gap tolerance, while disconnected regions retain
their real entry and exit surfaces. Both point and finite-line queries use the
actual remaining face patches. A rod's volume-occupancy time interval cannot
replace this test: one material portion may cross an exposed face while another
stays within a neighboring cell. A finite topology budget refuses excessive
patch construction transactionally rather than substituting a bounding box.

No momentum or incoming kinetic energy is credited merely for a bladder event,
including a dry tank. Actual drag alone increments deposited relative energy.
Before any wall contact and on later continuation, this uses `hits=0, areaM2=0`;
the additive native energy-only state/observer seams consume these rows without
repeating holes. For this new liquid path `ComponentHits.hits` counts punctures,
whereas `fragmentsStriking`, `projectilesStriking` and `firstStrikeAgeS` retain
actual material-encounter meaning. Positive liquid work, angular impulse or
linear impulse can establish that encounter at birth even with no puncture.
Symmetric rod rotation may cancel net linear impulse while retaining real
dissipation and torque; that cancellation cannot hide the encounter. A rotating
rod's actual surface contact also cannot be rejected merely because its centre
of mass has zero relative speed. The old nonliquid solid/skin semantics remain
unchanged.

Reaction momentum is minus the integrated fluid force. Reaction angular impulse
is integrated about the recipient's instantaneous moving structural origin,
including the distributed rod lever. Recipient body axes are the scene's
initial reference, matching the existing native scene application convention.
Relative deposited energy is minus `integral(u dot F dt)`. For a translating
medium the inertial fragment kinetic-energy loss additionally contains
`mediumVelocity dot reactionImpulse`; ignoring that work term would fail a
common-velocity-boost control. The existing scene mass/momentum ledger includes
fluid reaction, air reaction, gravity, surviving material and material absorbed
by later solid components. Capacity transit itself never consumes fragment mass.

## Method tests and delivery boundary

`tests/simulation/test_liquid_scene.cpp`, filter `[liquid-scene]`, specifies
independent analytical residence time, downstream residual energy/contact time,
split-window no-future work, dry/partial/full columns, subdivision and the full
65,536-cell bound, multiple legacy probes sharing one native tank, translated
medium energy/moving-origin torque, distributed half-wet rod loads, combined
air/gravity momentum, downstream THOR work, transactional invalid inputs and
legacy/orphan controls. These tests define the method gates; source creation is
not a passing native execution receipt. Native registration, actual captured
contents/parser/separation integration and fresh serial runs are owned by the
delivery owner and must be recorded separately.

The bladder-boundary correction first reproduced 34 failed assertions in six
cases (139 assertions, five cases failed); the actual outside-entry/initial-face
control already passed. An additional rotating zero-COM surface control then
failed its missing puncture assertion. The former interior half-wet Cd rod
fixture's `hits==1` premise was corrected to `hits==0` only after this RED:
positive drag work, reaction, mass, energy and tolerance predicates are retained.
A separate interior-spin control tests both an ordinary short interval and a
positive `2^-60 s` resolution-limit interval. In the latter, work and torque
remain representable while the odd net-force integral cancels; the previous
force-only encounter gate failed three assertions. These controls distinguish
real material interaction, actual wall contact and numerical availability;
none changes an engagement scene, liquid coefficient or acceptance tolerance.
Raw RED files are `liquid-bladder-red.log`, `liquid-bladder-spin-red.log` and
`liquid-bladder-interior-spin-red-2.log` under
`build/architecture-reviews/sim-m03-production-20261103/`.

At the preceding bladder-boundary source snapshot, the strict UCRT64 release build of the three
focused targets passes. Fresh serial CTest execution with nonempty selection
and bounded timeouts passes `fluid_transit_methods` (8 cases, 329 assertions),
`liquid_scene_methods` (24 cases, 2,543 assertions) and
`liquid_material_binding` (9 cases, 472 assertions): 41 cases and 3,344
assertions across these three disjoint executables. The eight bladder/encounter
controls are included in the scene count and must not be added again.

| Bound artifact | SHA-256 |
| --- | --- |
| `liquid-bladder-final.log` | `a5905e5acf0802231fbf7339a5ab8a96d8ec8c00aa12dbd749554f5eef635bfd` |
| `liquid-bladder-final.xml` | `825b70180dd1d7aa0e96236d63532ab7b859e700b0e352634420a7e16f7ea6dd` |
| `liquid-bladder-interior-spin-red-2.log` | `d5869a983bffa9fa1e8d36806ea5eb6a555cc44e3fc571f9605f4136b7d40ae3` |
| `src/simulation/mission_systems/damage/Damage.cpp` | `95bc63df5110f50bc16959200b63f78d5ed6cac9f4390967462572cbbdee8d22` |
| `src/simulation/mission_systems/damage/LiquidSceneV1.h` | `3f64d978cf13f7739fc6162e9e1e124e4c6809deb68ac8edcfb50a908fb96d1c` |
| `tests/simulation/test_liquid_scene.cpp` | `6bd4567038c12104923ed019b0e2471bc22833aecf0bf32be91a193b51460023` |

Log paths in this table are relative to that review directory; source paths are
relative to the repository. These focused method/binding fixtures do not
establish actual full-roster native liquid operation, complete flight aftermath
or Viewer acceptance. Those current-production gates remain separately owned.

The subsequent ambient-medium correction first reproduced 71 failed assertions
in three of four cases (241 assertions) with unchanged production. Fully wet
point/rod flight incorrectly depended on ambient wind/density; ambient forcing
also manufactured liquid work for initially comoving material. The passing dry
and zero-current controls established that ordinary air drag remained necessary.
The half-wet control independently integrates `v(s)^2`, `s*v(s)^2` and `v(s)^3`
for `v(s)=50+100s` over distinct wet/dry halves, checking consistent-mass endpoint
response, reaction torque/work and the air ledger. No engagement, coefficient,
capacity, numerical tolerance or one-second field lifetime changes.

Only two incorrect fixture premises were corrected after that RED. The existing
fully submerged momentum/gravity test now requires zero ambient reaction while
retaining its unchanged conservation and gravity budgets. The new comoving
control permits zero-valued component bookkeeping rows, but still requires zero
work, holes, area, impulse and actual encounters. The intermediate two-row
premise failure is retained as `liquid-air-scene-green.log/xml`.

Fresh strict UCRT64 builds and serialized nonempty bounded CTest execution at
this air-complement snapshot pass `fluid_transit_methods` (8 cases, 329
assertions), `liquid_scene_methods` (28 cases, 2,784 assertions) and
`liquid_material_binding` (9 cases, 472 assertions): 45 cases and 3,585
assertions. These counts supersede the preceding method snapshot, without
claiming the separately owned actual World or full-roster gates.

| Bound artifact | SHA-256 |
| --- | --- |
| `liquid-air-red.log` | `a7916eeeb2b2b41fb66571c7f16bf1f964b3d61a3ea9f511db3206d81dacb657` |
| `liquid-air-final.log` | `b60b4ff6cfb21835876746e10b2abaf73cfcb98db39a465646da6501cd29865e` |
| `liquid-air-final.xml` | `4bae0c772270b1f29b8ba09d8dc82db7cfdb0174446715459badc8df38c5546b` |
| `src/simulation/mission_systems/damage/Damage.cpp` | `548885b28fecf7e6921eacb7bf795ac39b0cbbc9e0ee6edeb818685af9874c49` |
| `src/simulation/mission_systems/damage/LiquidSceneV1.h` | `e8e039bbf92fea1f7dc84110ab32f12c4497d5258a57c7e1be9f58887df00054` |
| `tests/simulation/test_liquid_scene.cpp` | `5472d3426507836cd79d004ab740f2979aade9adf26beb08a53ed9a04b172ff6` |

The subsequent tangential material-partition correction reproduced 170 failed
assertions in two of three cases (662 assertions); the actual finite-gap
negative control passed. Extending the independent same-tank case to a point
also reproduced duplicate analytical chord work: the expanded unchanged-source
RED contains 936 assertions, with 230 failures in two of three cases.
The regressions prescribe the homogeneous liquid solution for an interior
point/rod on an exact capacity seam, compare 1/2/4/128 cells and reversed cell
storage, preserve summed force/work/torque across different probes, and test
unequal-density/Cd native tanks with both stable-ID permutations and directed
normal motion. The gap control retains ordinary air drag between distinct
planes separated by both millimetres and `2^-30 m` half-gaps. No tolerance,
physical coefficient, volume or preexisting outcome predicate was relaxed.

At the material-partition snapshot the same three bounded serial CTests pass:
`fluid_transit_methods` (8 cases, 329 assertions), `liquid_scene_methods`
(31 cases, 3,680 assertions) and `liquid_material_binding` (9 cases, 472
assertions), totaling 48 cases and 4,481 assertions. The focused partition
filter itself passes 3 cases and 896 assertions. Its count is lower than RED
because duplicate returned component rows no longer add per-row checks; all
analytical force/work/momentum/torque and negative-control predicates remain.
Actual World integration, full-roster delivery and visual acceptance remain
separate gates; these are focused method/binding results only.

| Bound artifact | SHA-256 |
| --- | --- |
| `liquid-tangent-red-2.log` | `90c713fc076199a0b3ff15b471ee6ceb77edca9c601911e4c2caa381b7beddf8` |
| `liquid-tangent-final.log` | `448f87bcf50716db894e0c2a406ca5c8a83e4f4e1aed7677006463d736d68eda` |
| `liquid-tangent-final.xml` | `98bb52e9ac55237f807e5a99d4b80df3796908b20909192490ed3ce798fdec32` |
| `src/simulation/mission_systems/damage/Damage.cpp` | `3b9c04c9d720351e936eab031d8f85103297ef9e30aa415ac4beabc0ce189aa8` |
| `tests/simulation/test_liquid_scene.cpp` | `129e9b3b6bbb3a55f5e1f445c4b99f70bf03b32cd5173b1c1077ed9f776a343f` |

Independent review then identified an additional point-horizon defect using
four exact cells on a two-tank interface. Lower cells select TK0 by stable ID;
upper cells select TK1. Each tank's geometric union stays continuous, so the
previous per-tank union horizon missed the change of admitted material law.
The native RED reproduced an incorrect `99.00990099 m/s` residual versus the
independent chronological reference `98.13547506 m/s`, incorrect position and
absent TK1 work (1 case, 11 assertions, 3 failures). The reference applies a
fixed `0.01 m` TK0 chord followed by elapsed TK1 transit; the independent exact
transition split already passed. The new whole-window, transition-split and
reversed-storage controls retain their analytical predicates unchanged.

The final ownership-horizon snapshot passes the same strict build and three
serial CTests: fluid methods 8 cases/329 assertions, scene methods 32/3,714,
and material binding 9/472, totaling 49 cases and 4,515 assertions. This
supersedes the narrower 48-case partition snapshot; it does not replace actual
World, full-roster or visual gates.

| Bound artifact | SHA-256 |
| --- | --- |
| `liquid-owner-horizon-red.log` | `3fce07142c69240c26a3e2f0040ce5863713b7517fa994027834ad082e1598fd` |
| `liquid-owner-horizon-final.log` | `607619b8cbe131ceb976eddc5688618c52fbdf8f8db1fb23761d02b3c97a2162` |
| `liquid-owner-horizon-final.xml` | `a92f5a0cc7b62a28736b5e333580f6df23f70fa7db72f95e6300249cf09ea7a4` |
| `src/simulation/mission_systems/damage/Damage.cpp` | `dd6d47a6ebf4d9c5b6446f409eef0b0df1a065812f7b8437267a967f87e791e4` |
| `tests/simulation/test_liquid_scene.cpp` | `6d52450fa4c556b3968275616e030354a68ef8995a9b47dd9970127cf0df0581` |

### Native high-speed material integration correction

The unchanged actual authored-population gate subsequently refused its first
AIM-120C/F-16 scene, producing no Hit because the complete assessment was invalid
(1 case, 8,480 assertions, one failed original `hits>0` predicate). The original
source ordinal 136 exhausted the existing 100,000-iteration budget near age
`0.000250860133 s`: Earth-scale coordinate increments were repeatedly rounded
away. Burst-origin translation repairs this accumulated cancellation, without
restoring information absent from input doubles. The independent identical-
represented-input Earth/local pair first failed four strict invariance checks
(31 assertions). Its initial attempted absolute analytical bounds were below
the unchanged declared integration error allowance even in the near-origin arm;
that attempted-bound failure is retained separately. The final fixture derives
analytical allowances from those existing controls and retains strict paired
velocity/momentum/torque/work invariants. No production tolerance was increased.

Translation exposed a second native source 220 loop at local age
`0.13996734720110465 s`. Its predicted positive entry duration
`6.23277042749e-18 s` was smaller than that age's `2.77555756156e-17 s` ULP, so
adding the duration did not advance the clock. This was a dry-entry contraction,
not a measured wet-exit failure. Upper represented absolute-age scheduling and
the existing capture/window cap repair that finite-clock stall. Independent
positive-age translating/oblique-face and whole/resumed controls are retained
as negative controls, not mislabeled as formerly failing fixtures.

The newly reached AIM-9M scene exposed a genuine discontinuous-RHS failure:
point source 3's RK4 stage error was `0.00226196 m/s` versus its unchanged
`0.00198788 m/s` allowance; another halving would cross `minStepS=1e-10 s`.
An independent stiff-entry point reproduced invalid assessment before the
phase correction (1 case, 3 assertions, one failure). Event-separated smooth
point phases now bracket actual material-plane roots and retain pre-event
endpoint forces, while rods continue to integrate changing wet material
intervals. The first real fluid interaction carries its elapsed phase time.
Later MAGIC-2 coverage proved that a second free-air-chord boundary scheduler
could still approach the same face indefinitely despite represented clock
progress; it was removed from point event authority. Its temporary exact-card
observer case was removed byte-for-byte after diagnosis. The maintained
26-card population source remains SHA-256
`fb9cb5df6ccef36399d3ad0e73850298524b03c5f465970fee733eae059070a7`.

At this clean source snapshot the strict focused build and three serial
nonempty bounded CTests pass: fluid methods 8 cases/329 assertions, scene
methods 37/3,833 and material binding 9/472, totaling 54 cases and 4,634
assertions. The original 49 cases remain included; five added method cases
cover represented translation, positive-age motion, oblique entry, stiff entry
and tangent outward acceleration without future liquid work. Their result does
not stand in for the separate full authored/native integration gates.

Later actual R-27ER coverage exposed a positive immersed rod interval whose
endpoints were adjacent doubles (`0.84047982744282568` and
`0.84047982744282579`). The shared rod quadrature previously refused because
no interior midpoint could be represented. Independent uniform/varying rod
controls reproduced two failures (2 cases, 6 assertions). The interval is not
dropped or merged with a neighbor. A positive endpoint rule retains its force,
shape loads, work and moment. For affine relative velocity and
`K = rho*Cd*area/2`, convexity gives `S = max(|u(first)|, |u(last)|)` and
`|df/ds| <= 2*K*S*|du/ds|`. The corresponding weighted-integrand bound adds
`K*S*S`. Conservative `L*width^2/2` and arithmetic-roundoff estimates enter the
unchanged global component error ledger. If the requested budget cannot be
certified, the unsplittable interval still refuses explicitly. Direct positive
endpoint shape-weight and centre-moment reductions avoid subtracting two
nearly equal total-force doubles near the rod endpoints or midpoint. The
strict original controls pass 46 assertions; an intentionally unattainable
`1e-40 N` error request remains refused. No density, Cd, area, interval width,
quadrature limit or force tolerance was adjusted.

The new elapsed-interaction timestamp also required the same earliest-event
reduction already used by solid/bladder contacts. A two-point storage-order
control first reproduced one failure in eight assertions: a later-entering
projectile hid another point's real liquid work at birth. The fluid timestamp
now takes the minimum actual phase time across the population; this changes
neither energy nor puncture counts.

The resulting clean focused receipt is `liquid-production-methods-final.log/xml`:
fluid methods 10 cases/375 assertions, scene methods 38/3,841, and material
binding 9/472, totaling 57 cases and 4,688 assertions. Original acceptance
scenarios and numerical controls are retained. Separately, the restored actual
R-27ER diagnostic passes its unchanged fragment-Hit and tank-hole predicates;
single-card diagnostics remain distinct from the complete 26-card gate.

The subsequent clean restored original population gate passes all **26 authored
explosive cards**, including their unchanged native fragment-Hit and positive
tank-hole predicates (1 Catch case, 224,102 assertions). Its measured wall time
is `182.0563718 s`, below the unchanged `300 s` bound. Raw output is
`liquid-production-authored-final.log`, with native per-card observations in
the paired `.stderr.log` and timing in `.seconds.txt`, under the same review
directory as the earlier receipts. The original population test SHA remains
`fb9cb5df6ccef36399d3ad0e73850298524b03c5f465970fee733eae059070a7`:
temporary selected-card cases and every diagnostic print were removed. All
earlier failures remain retained as separate raw files. This is actual complete
population execution for this source snapshot, not a claim that isolated-card
runs covered the roster. The delivery owner retains the required merged native
`weapon_effects_population`, `fragment_reach_native` and `liquid_native_binding`
rerun after the next queued integration changes; this handoff does not waive
those gates or establish complete program/Viewer acceptance.

| Bound artifact at this complete population snapshot | SHA-256 |
| --- | --- |
| `liquid-production-authored-final.log` | `600bbda5ffec5cb48f34a54132d00f5089ef6af98c7ef72becea2927c958d242` |
| `liquid-production-authored-final.stderr.log` | `bb39c5137ff6351e2e1e48b1e213ddce70725f28d2a4fe34a1852f88e03fa6f2` |
| `liquid-production-authored-final.seconds.txt` | `c4bfec2f65abb39ff78b70fe842fd2dfde7102feaf637c94b135e960b970937b` |
| `liquid-production-methods-final.log` | `71ae26d701518225a7c79959999f5fa78cc2f1d48150859474d99d7278c2a780` |
| `liquid-production-methods-final.xml` | `4aff2d2d83819dae6bfdc6e27c68180f86a583f0f43b3cc5d889bdcaa2acda8f` |
| `src/simulation/mission_systems/damage/Damage.cpp` | `909038ad820a6da56faff18ea3985ce6cc8f5b91377df58473b6eb0c9c943f2c` |
| `src/simulation/mission_systems/damage/FluidTransitV1.h` | `b8b284027f29a778ac0da9832605a6ecf10292539a4f3d5e03716e9392ccbb03` |
| `tests/simulation/test_liquid_scene.cpp` | `6cdebf23613123c733008b7ba36dd9de7052362e3735650794b815327cc47b48` |
| `tests/simulation/test_fluid_transit.cpp` | `132718d24fca263c933fe70c7f2295c120cf6ea2ba6384b8d4e3afecddf242fa` |

### Exact tangent departure followed by a later tank

Further independent review found that a lone tangent-departure control did not
exercise the point solver when the outer query found no later wet material.
A second valid control adds a distinct tank across an actual `1e-8 m` gap. A
point starts on tank A's face with tangent velocity; outward gravity places it
outside A for every positive time, while tank B becomes reachable at
`sqrt(2*gap/10)`. The unchanged-source RED produced 8 failures in 42 assertions:
A received `1.9992669 J` of future liquid work, and whole-window/resumed flight
differed. The exact zero-time departure must be classified before a positive
phase is integrated; skipping a zero-length owned span cannot authorize A's
force for that phase. This was a separate observed mechanism defect, not
covered by the preceding full-population or lone-face pass.

The right-limit rule described above now uses normal velocity followed, only
at exact zero, by physical relative acceleration. Mirrored lower/upper faces,
both storage orders, no-work-before-B-entry, real dry-gap and whole/resumed
predicates pass with their original bounds. There is no coordinate or time
nudge, different density/Cd or integration-budget change. At this final
right-limit snapshot fresh strict builds and serial focused CTests pass fluid
methods 10 cases/375 assertions, scene methods 39/3,925 and material binding
9/472: **58 cases and 4,772 assertions**. The restored original 26-card native
population gate passes again with **224,102 assertions in 181.1565144 s** under
the unchanged 300-second bound. This supersedes the preceding 57-case kernel
snapshot. The delivery owner's merged native/broad/visual gates remain
required after its separately queued integration work.

| Bound artifact at the right-limit snapshot | SHA-256 |
| --- | --- |
| `liquid-right-gap-red.log` | `6f9b74585a973ae10b993e54c21378c09b0f49edc8d70541396c0fac6a625f6c` |
| `liquid-right-limit-authored-final.log` | `e8e262dcb1a0b73bde923f0a46398fdd0ade4f3e3a0393d4e2e917dc088a1200` |
| `liquid-right-limit-authored-final.stderr.log` | `bb39c5137ff6351e2e1e48b1e213ddce70725f28d2a4fe34a1852f88e03fa6f2` |
| `liquid-right-limit-authored-final.seconds.txt` | `2fbdba53851eb1248fd661a473af684ecd73691a5afd1df42171ebf5e45dbb6e` |
| `liquid-right-limit-methods-final.log` | `fc74aa9f7674b00c076c0100a04d815f3f40af2a1087444c85b4c34c15c67ece` |
| `liquid-right-limit-methods-final.xml` | `bdbc1d0cea43f6562a1eaf6a492e53066ec36f213a10386a07058d9b2a271adc` |
| `src/simulation/mission_systems/damage/Damage.cpp` | `b201e604c0109a5a48e0a330de620fc884db69564a5841fc72706312b453b88b` |
| `tests/simulation/test_liquid_scene.cpp` | `1707a14bd9d5a274feb87d3ce548200116c7f591a4242597b08557f309b2698d` |

The explicit liquid Cd remains an engineering input, not an FAA/OEM calibration.
Captured free surfaces are quasistatic, without slosh. Finite straight material
elements, captured orientation and chord-based solid interception are reduced
models requiring cadence/refinement checks. The method does not implement
cavitation, pressure-wave propagation, resolved hydrodynamic ram, bladder
failure stresses, fluid-structure interaction, fuel chemistry or projectile
deformation. These distinctions follow the physical separation in the
[FAA engine-debris fuel-tank report](https://rosap.ntl.bts.gov/view/dot/13265),
which studies ignition and hydrodynamic ram as separate consequences, and the
[Fraunhofer EMI tank experiments](https://www.emi.fraunhofer.de/en/business-units/aviation/research/hram-effect-on-water-filled-tanks.html),
which measure transient cavities and pressure alongside projectile deceleration.
Neither source supplies a measured coefficient for this roster. Those larger
phenomena cannot be certified merely because finite drag is now executable.
