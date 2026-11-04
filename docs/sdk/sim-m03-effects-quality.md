# SIM-M03 shared physical effects model

This contract describes generic engineering execution, separately from measured
equipment accuracy. The maintained explosive roster contains 24 air-to-air
missiles, MK82 and CJ-10A; 27 other store configurations explicitly have no fuze
or warhead. Available model functions must execute through installed native
providers. Missing generic execution cannot be relabeled as calibration.

The currently admitted engagement/damage domain is aircraft plus independently
owned departed aircraft bodies. The maintained cards and approved M03 scenarios
do not declare an implemented missile-interception function. Existing candidate
admission in `WorldEngagements.cpp` explicitly selects Aircraft, while
`WorldDamage.cpp` additionally admits owned Debris. This delivery therefore makes
no weapon-on-weapon or ground-structure lethality claim. That is the configured
domain observed in the current source/design, not a prohibition on future target
families.

## One detonation, one material population

`emitFragmentFieldV1` creates one finite immutable population before inspecting
any victim. Every element carries mass, presented area, drag coefficient,
inertial material position and velocity. Each mass is exactly `caseKg / count`;
integerizing the equivalent-fragment rule therefore cannot add metal. Source
positions occupy the already declared finite case envelope. Angular velocity
contributes `omega cross offset` to the actual material velocity. Existing
angular source laws and declared Gurney inputs remain engineering assumptions.

`World::detonateAt` freezes eligible aircraft and independently owned departed
bodies before applying any consequence. A release-specific fragment stream is
separate from each recipient's component-failure stream. The recipient list
cannot resample, duplicate or steer the physical source. Explicit scripted
bursts use their own World-local ordinal; reconstructing the World reconstructs
that sequence.

`assessSceneV1` orders all admitted crossings by physical flight time, with stable
recipient identity and source array order at exact ties. A crossed skin consumes
the THOR penetration work and retains the surviving projectile. Air slowdown
transfers momentum to the air. A solid component absorbs one projectile once;
another recipient does not receive a fresh copy. The fragment momentum ledger is

```
initial material momentum + gravity impulse
  = recipient impulse + air impulse
  + surviving material momentum + absorbed material comoving momentum
```

The last term matters for a moving recipient: absorbing material does not erase
its comoving momentum. Component kinetic energy is relative to that recipient.
No comparison of total inertial kinetic energy alone can replace the associated
moving-recipient work term.

The additive scene kernel accumulates fragment angular impulse about the
instantaneous moving structural origin at each crossing, including finite-rod
intrinsic spin, and expresses the sum in the initial target body axes. Native
scene application converts that moment to COM spin without applying a second
fixed-world translation from the old pose to the publication pose. Otherwise,
an unrelated common velocity boost would add
`(boost * (impact_time - publication_time)) cross impulse` to the body spin.
The independent vacuum fixtures check angular invariance and a known off-axis
material lever across captured translations and different publication windows.

Legacy `assess` providers and explicit mechanical contact keep their existing
fixed-reference moment convention, including mechanical contact combined with a
scene burst. No original adapter layout or vtable changes. The orbital/spin
distinction follows [MIT rigid-body dynamics, section21.3](https://ocw.mit.edu/courses/8-01sc-classical-mechanics-fall-2016/mit8_01scs22_chapter21.pdf).
Finite captured orientation and CG references remain bounded numerical
approximations; this correction does not introduce a resolved flexible-body or
continuous impulse-response integrator.

## Flight and interception approximations

Between crossings, quadratic drag uses `rho * Cd * area / (2 * mass)` and the
declared local air velocity. Endpoint speed and displacement use the analytical
constant-air solution. Symmetric standard-gravity kicks give exact vacuum
gravity and second-order drag/gravity coupling. Captured affine intervals, by
default at most 2 ms, resolve interception. Live execution consumes actual
captured native recipient origins and attitudes over each available interval;
geometry orientation is frozen only within that small segment. Native angular
velocity supplies the material velocity at contact. Finite-rod absorption also
transfers intrinsic angular momentum and relative rotational energy. This is
not a deforming-body or fluid/structure interaction solver.

Surface panels with authored depth expose their two finite skins. Closed body
panels with admitted contact meshes expose only their authored skin triangles;
artificial ownership caps never become penetration-resistant material, even
after the neighboring section departs. Adjacent coplanar triangles share a face
identity, so a triangulation diagonal does not charge the same plate twice.
Distinct faces remain distinct; the query does not assume a convex shell or
limit a panel to one entry and one exit. Without an admitted mesh, legacy body
boxes retain their existing closed entry/exit semantics. Finite rods are line material, not two point
rays: a small component between their endpoints can be intercepted. For an
affinely expanding line `x = a + v*t + d*q`, the material constraints are
`0 <= q <= 1 + k*t`. Convex face constraints become half-planes in `(t,q)`;
clipping their feasible polygon selects the first contact. A final time-window
endpoint alone does not prove an exit. Source array IDs remain immutable.

Live released bursts own a resumable field. Only the captured burst-to-boundary
remainder is processed first; subsequent native World intervals advance surviving
material without resampling. Crossed source faces and recipient generations
remain attached to each projectile. No future impulse is published to satisfy a
hit objective. Field ownership outlives the removed weapon Entry; immutable
weapon/launcher/store/activity/sample correlations retain existing native damage
meaning. The retained Hit wire reference is the original historical source
stream slot plus one, captured with that immutable weapon identity before its
Entry is removed. It never becomes the identity of a replacement occupant;
native combat damage reports retain the original weapon ID.
The generic field retains its bounded one-second flight window. Shared physical
scenes admit actual captured recipient geometry and let the retained material
path/geometry broad phase decide interception. They do not discard recipients
by distance from the original burst: authored AIM-120C and R-77M material can
travel beyond 200 m within that window under the implemented air-drag law.
Only retained external `assess` adapters keep the original 150 m dispatch
radius. Neither the finite window nor that legacy radius is an equipment
lethality guarantee.

Every existing synchronous diagnostic `World::detonate` input retains its
reduced-order full-flight assessment contract, including a weapon ID and finite
captured fraction. The additive internal `World::detonateCapturedV1` seam opts
explicitly into causal built-in execution; native live-burst call sites select
that execution through their scene-capable provider. Legacy external providers
retain their original synchronous `assess` authority. Synchronous diagnostics do
not grant causal live-flight acceptance. Native event publication still occurs at
World decision boundaries. Triangle/box/quad interception and first solid-component
absorption remain declared reduced-order material choices.

Blast remains the existing analytical free-air peak/impulse loading, applied
once at the first available publication. It does not resolve finite pressure-wave
arrival, reflection, occlusion or structural pressure-time histories. Fragment
continuation is not evidence of those separate blast mechanisms. Numerical
cadence convergence and hardware/material calibration are different tasks.

## Connected and discrete rods

Only the four R-27 configurations select
`native.warhead.connected_ring.v1`. Their card descriptions explicitly name
continuous rods. R-60M, R-73 and the paired-rod R-77M configuration retain discrete
finite rods; a broad `rod` family does not imply welded connectivity.

[Brown and Meyer, JHU/APL Technical Digest 3(2), 1982, pp.157-159](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V03-N02/03-02-Brown.pdf)
describes the hinged expanding-ring mechanism and distinguishes it from discrete
rods. That mechanism motivates the topology; Talos dimensions and lethality are
not R-27 calibration inputs.

The implemented ring is a homogenized finite material continuum. Neighboring
elements share exactly identical initial endpoint positions and velocities.
Actual propagated material length bounds continuity, including air slowdown;
a vacuum opening clock cannot cause premature breakup. After opening, elements fly
as independent constant-length material. A local collision also breaks the
affected element while retaining its mass and residual momentum. This model
does not solve individual weld deformation, hinge shock, fracture distribution
or calibrated rod tumble. Retained Gurney/rod-speed priors and the derived case
dimensions have not become measured R-27 properties. Declared source-card
uncertainties remain in force.

## Departed body ownership

`VehicleDamage::detachedState` copies accumulated owned-section holes, torn skin,
fire, engine/control/crew consequences and their immutable source indices. It
translates the immutable geometry, contact meshes, component positions and
structural reference from the parent's origin to the piece's native empty-CG
origin. Nonowned sections and unsectioned parent geometry are excluded. The
already consumed parent joint cannot fail a second time on the departed body.

Native piece flight models already contain aggregate fuel as a point mass.
Per-source tank quantities in damage state partition that same mass; their sum
is written back to the native point mass after leakage or spill, together with
the actual remaining weighted fuel centroid. A spill publication is capped by
available native fuel. No second fuel
body is added. A transfer retains current fuel and burning state, terminates the
parent emitter and publishes later child emission at the child's actual material
point. A later burst can damage or be shielded by that independent body.
The piece retains its owned section identity separately from accumulated damage,
so physical reset clears damage but cannot silently make the body immune.

## Compatibility and evidence

### Authored skin and ownership cuts (2026-11-03)

The old closed-body damage-box query charged axial ownership cuts as aluminium
skin although the body panel's authored area contains lateral skin only. The
scene query now uses the existing immutable contact mesh's explicit provenance:
null-neighbor triangles are actual skin; triangles naming another owner are
volume closures and never plates. Bounds only reject impossible candidates;
actual triangle intersections authorize work. Original panel indices, section
skin thickness, THOR coefficients, source geometry, component boxes, surface
panels and the legacy absent-mesh path are unchanged. Coplanar face grouping
uses a scalar-triple-product floating-point roundoff bound, not a physical
thickness or hit-radius allowance.

Four independent synthetic cases first failed 15 of 124 assertions. They cover
point fragments and finite rods, an interior source already inside a fuel
component, traversal to an interior component across an ownership cut, a cut
exposed by separation, genuine entry/exit work, a triangulation diagonal and
the physical skin inside a broader damage box. A fifth case additionally
requires all four crossings through two closed shell regions belonging to one
source panel, guarding against an implicit convexity restriction. The current
five cases plus the unchanged six rod-interior/entry/exit cases pass 273
assertions in 11 cases. The overlapping complete `[scene-effects]` selection
passes 11,447 assertions in 40 cases; these totals must not be added.

At this integrated UCRT64 release snapshot, the exact-card native gate passes
the 26 explosive-card component checks and the four CJ-10A armed/inert terrain
controls at frame skips 1 and 4 (one CTest entry, 15.9306 s). The unchanged 5 km
functional scene also passes: the weapon detonates with reported miss
1.38667 m, the recipient loses its engine, then continues five seconds and
travels 971.527 m, with speed changing from 249.752 to 152.975 m/s. The F-16
asset used here includes the separate typed structural-skin correction; its
serialized comparison records only `skin_kg` changes, with geometry and rigid
mass partition unchanged. These integrated receipts do not isolate either
correction as the sole cause of the native outcome.

Raw files are under
`build/architecture-reviews/sim-m03-production-20261103/`:

| Receipt | SHA-256 |
| --- | --- |
| `rod-interior/authored-red.log` | `13c57fb0d39169a31027806b0053f0b1b4687be925a6d73e6a147eee5dab7335` |
| `rod-interior/authored-green.log` | `31d319e5fe53ad126bbb55686bd628c3b2d12dd623f24cf9b79326969ff27e35` |
| `rod-interior/authored-scenes.log` | `77b22946affe5b940c861c1aed3ed4ceff3d4e8aeefbf85974da24974749ed37` |
| `effects-population-authored-skin.xml` | `f22f9ee37c03f8e6c9dba239d98dce6d2902e5a988b8d82211d3d40c2ddaaed7` |
| `effects-observation/functional-authored-skin.log` | `a61cee6c3a46b47cc6aa10abf6e2aa98408de8b0c5fa02675c7e9789d76d1a32` |
| `skin-structure-mass/f16-serialized-comparison.json` | `b754602a5574fff94236c365d99f392f2b705bd6de4d54ba33314dcaaa0ba46a` |

Reviewed `Damage.cpp` SHA-256 is
`efbcacd60fbb35f906e129feab13858bc7b3bc39fec2299177d1864b455b50e3`;
`test_damage.cpp` is
`4620b6b83a9ade87320df7d53c7c0df1e984e8fbe3c8e2668360e47336cf0bfd`.
Finite rods still apply a lumped element's THOR work per distinct contacted
face. Coplanar diagonal deduplication does not establish refinement convergence
for a long rod spanning differently oriented facets of a curved shell. Finite
liquid transit based on actual remaining fuel is also a separate implementation
gate; a successful local tank-hole check does not verify that mechanism.
These receipts establish this repair at the recorded snapshot, not measured
equipment vulnerability, complete model regeneration, both-preset/staged
admission, full-envelope aftermath, installed-consumer or visual acceptance.

### Typed structural skin mass (2026-11-03)

Hangar's unchanged `SKIN_SHARE = 0.35` applies to the existing spatial mass
items whose type is `structure`. Engines, systems, gear and other installed
items retain their complete rigid mass contribution; they cannot also become
homogeneous penetration-resistant skin. The previous writer multiplied the
whole section mass, including that equipment, by the structural skin fraction.
The corrected allocation follows each structural item's actual quadrature
weights and section membership. Geometry, whole/section mass, CG, full inertia,
root strength, native flight and the assumed fraction are unchanged.

The independent `test_damage_skin_mass.py` fixtures first failed 14 assertions
across four tests, then passed all four tests. They add engine/system/gear/item
mass at three prescribed magnitudes without thickening skin, move real
structural material between sections, require identical non-skin outputs for
identical physical particles with different material types, and independently
check serialized mass/first/full second moments for all 31 nonempty retained
section combinations. These are synthetic source-construction tests, not
equipment thickness measurements or an all-model native run. Raw RED/GREEN
receipts are `skin-structure-mass/red.log` (SHA-256
`1d5c05a2c04ba1947dccebebe6cd922bacd66c60a9af59c3e225bd7f52b1e36e`)
and `skin-structure-mass/green.log` (SHA-256
`cdcf5f5a5736bf81e39fd37d3f67517cb777a063e262d08b97142d97a260b89a`),
under `build/architecture-reviews/sim-m03-production-20261103/`.

Fresh complete model regeneration, serialized all-model moment closure and
native effects/engagement execution remain required after this writer change.
The panel's geometric depth is separate from its aluminium skin thickness:
surface depth places the two physical skins; THOR uses section skin mass over
density and area, with the existing declared bounds. Neither quantity is a
measured aircraft-specific vulnerability map.

### Moving-origin angular impulse verification (2026-11-03)

On main baseline `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`, the
unchanged `[scene-moving-origin-moment]` selection first reproduced eight failed
assertions in two cases (65 assertions), then passed all 65 assertions after the
moving-origin correction. These independently specified vacuum fixtures test a
common velocity boost and off-axis contact across captured translations and
publication windows; linear impulse alone is not their acceptance predicate.

The actual native `[scene-native-moving-origin]` selection subsequently passed
164 assertions in one case. Its eight independently constructed Worlds cover
frame skips 1 and 4, zero and nonzero initial body rates, and both the shared
physical scene and the external fixed-reference negative control. The synthetic
scene-capable provider emits one central-line material element strictly inside
an actual F-16 fuel component at birth. First absorption therefore has zero
flight time, with no gravity or air-drag impulse. The test bounds the immediate
native spin increment before reading its result, using known impulse, observed
pose/CG change, binary64 position roundoff and a lower bound on the actual full
native inertia. Each physical scene also requires its actual fragment Hit. The
external control instead requires the original transported fixed-reference
moment. This verifies the production scene/application convention and its
bounded captured-pose approximation; it does not establish exact rotating-CG
integration or equipment-specific lethality.

Raw receipts are under
`build/architecture-reviews/sim-m03-production-20261103/moving-origin-moment/`:

| Receipt | SHA-256 |
| --- | --- |
| `red.log` | `d8c955608edfaf9890b6f9bb31efd3853816a11b13e3cf51eb8fb1cc86f6bac4` |
| `green.log` | `559f6005190e7ad8529d8188e0b41858dca3635ac155ab068717aa05c8af7447` |
| `native-application.log` | `3ed415f122fffdfa6f9eac8925644873e3842a3452e89c70aa4a2613566b23cc` |

The fixture populations overlap broader damage selections and must not be
summed with them. These current focused receipts do not replace fresh complete
generation, regression, installed-consumer, fleet or presentation acceptance.

### Provider compatibility

The exact-card effects gate is defined by
`tests/simulation/test_weapon_effects_population.cpp` and admitted by
`tools/validation/weapon_quality/effects_population.py`. It independently
enumerates the 26 available explosive cards instead of treating a successful
flight or an inventory declaration as warhead execution. Each selected provider
emits its actual finite material population. Raw method and native source CSVs
retain immutable element ordinals, mass, area, drag, endpoints, velocities and
connected-material expansion. A uniform absorbing volume in vacuum receives
every source at birth, so first-flight time, gravity and air impulse are zero.
Independent energy is the sum of `m*|v_COM-v_target|^2/2` and, for each finite
uniform rod, `m*|v_end-v_start|^2/24`; momentum is the sum of
`m*(v_COM-v_target)`. The collector independently integrates the raw material,
reconstructs authored case mass/equivalent count/Gurney speed and rejects a
changed roundoff budget. This is a material-method check, not a lethality fit.

Each card separately executes the real shared-scene `World::detonate` path
against a maintained F-16 fuel component, with unchanged selected emission,
native atmosphere and full blast. A native fragment Hit, a positive tank hole
and finite immediate World state are all required. Blast-only structural loss
cannot satisfy that predicate. This local component-execution experiment does
not claim survival of an intentionally catastrophic aircraft or replace the
complete flight/aftermath fleet checks.

The orthogonal CJ-10A terrain fixture uses an actual H-6K Host release and a
genuinely delivered radar/link designation for the armed case. After the authored
arming interval, an explicit low-altitude downward initial condition supplies a
native terrain contact. The fixture never calls scripted detonation. At frame
skips 1 and 4, the armed release must produce one native detonation and its exact
selected material population, while the inert jettison must produce neither.
These four terminal controls supplement the original unchanged cruise-flight
scene, which deliberately remains in flight and forbids detonation.

Admission requires both presets, actual nonzero successful native test reports,
exact per-card and terminal populations, source epochs frozen before execution,
staged producer/runtime hashes and unchanged raw material bytes. The new gate
has a current release native execution receipt in the authored-skin section
above. Complete both-preset, source-epoch and staged-artifact admission remains
separate; that release receipt alone does not satisfy the full delivery gate.

The original `WarheadAdapter` virtual layout and the original single-target
`damage::assess` entrypoints retain their contracts. The sibling
`WarheadSceneExecutionV2` capability admits shared populations for built-in
providers. An external adapter without that capability continues executing its
own original `assess`; its absence never authorizes a different native provider.

The effects allocation and exact source ownership are recorded in
[the implementation window](../superpowers/plans/2026-11-02-sim-m03-effects.md).
Independent `[scene-effects]` fixtures cover finite source mass/bounds/spin,
shared shielding and residual momentum, recipient permutation, translating-frame
invariance, finite-rod middle contact, connected-ring continuity and central gap,
air momentum, malformed populations, detached state/fuel transfer and a later
native burst against a departed body. Existing fixed-scene hit, native failure,
reset, recording, public provider and full regression predicates remain active.
Current native results are recorded by the exclusive serial delivery owner;
this contract itself is not a passing test receipt.

### Intermediate verification and authored-fixture corrections

The original scene/fuze/contact checkpoint passed 55 cases and 16,469 assertions;
the overlapping damage selection passed 54 cases and 5,248 assertions. These
populations overlap and must not be added. The checkpoint-3 native detached-reset
failure demonstrated a real missing owned-state reconstruction path and is now
covered by an independent later-burst fixture.

The first air-slowed ring fixture had an incorrect physical premise: at its
chosen area and 20 ms window the analytic radius already exceeded available
material length. Its observed breakup therefore did not prove a production
defect. The corrected synthetic setup independently requires
`R0 + log(1+k*v0*t)/k < availableRadius` while the old vacuum opening time is
already inside the window, before checking absence of breakup. No production
coefficient or established physical scenario was fitted to that outcome.
The provider-admission fixture also now supplies the required derivation note
on its synthetic assumed parameters; the runtime provenance requirement was
preserved. Failed checkpoint logs remain part of the delivery record.

Checkpoint-4 executed 65 cases and 16,541 assertions: 63 cases passed and the two
authored-fixture issues above accounted for three failed assertions. Causal
source removal/slot reuse, captured timing and detached reset passed. One bounded
native scenario advanced one simulated second in 0.1204854 wall seconds. This is
a single mechanism timing receipt, not a fleet, dense-engagement or Viewer
performance claim. The explicit legacy diagnostic timing regression and the
corrected fixtures require the next fresh serial checkpoint before acceptance.
