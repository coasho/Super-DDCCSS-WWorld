# Capacity-conserving liquid geometry

This M03 product separates physical liquid occupancy from the existing fitted
vulnerability hit probes. A probe's fitted dimensions do not determine the
physical fluid density or capacity. Its nominal `volume` may exceed its actual
box volume; treating that small box as containing all the authored fuel would
invent a dense material and overestimate projectile deceleration.

The new normal-model product is `<model>.liquid-v1.json`, with schema
`fsim.liquid-occupancy.v1`. It does not change the authored airframe geometry,
JSBSim XML, native tank points, whole/section dry masses, CG, inertia, strength,
flight controls or aerodynamic tables. Generation failure is a required product
failure, not permission to substitute a probe or change density.

## Sources and physical assumptions

- [Manifold's official README](https://github.com/elalish/manifold) describes
  oriented manifold triangle inputs, explicit invalid-input status and robust
  Boolean output. The producer uses the already installed Python `Mesh64` and
  `Manifold` bindings, checks every Boolean status, and requires empty geometric
  differences/intersections. These are numerical polygonal-geometry checks,
  not proof of smooth CAD or measured OEM internal layout.
- [SUAVE's OpenVSP tank-property implementation documentation](https://suave.stanford.edu/doxygen/group__Input__Output-OpenVSP.html)
  treats full fuel volume, fuel mass and tank CG as geometric properties.
  The M03 ledger independently requires volume = authored capacity / density.
- [Budd, *Height-volume characteristics of a fuel tank*, Airbus study group](https://miis.maths.ox.ac.uk/miis/102/1/Airbus-FuelTanks.pdf)
  explains the importance of internal geometry and attitude to static fuel
  height/volume. It expressly studies equilibrium, not slosh. A runtime consumer
  must reduce actual occupied volume as contents fall, at unchanged density;
  this full-capacity producer does not implement runtime free surfaces or slosh.

Reference density follows the existing declared engineering estimate:
800 kg/m3 for jet fuel and 720 kg/m3 for piston-aircraft avgas. Temperature,
fuel specification and measured tank distribution are uncalibrated. Capacity
and native tank numbering come from the design, not from tank-name inference.
Helicopters retain their one native aggregate tank 0 and convert authored pounds
using exactly 0.45359237 kg/lb. Two real UH-60 cells do not create two native IDs.

All products, including explicit no-liquid products, carry this predeclared
engineering policy (not fitted to an engagement):

```json
"fluid_policy_v1": {
  "law": "quadratic_unresolved_tumbling_v1",
  "drag_coefficient": 1.0,
  "basis": "assumed",
  "screening_uncertainty_fraction": 1.0,
  "note": "Unresolved tumbling pressure-drag engineering policy; not FAA/OEM calibration or air Cd."
}
```

The screening fraction is not a confidence bound. Kernel half/double-coefficient
sensitivity is a separate physical-method gate; serialization tests do not
establish dynamic response. Every product also states method version 1 and
`method.origin_design_m`, including no-liquid designs.

## Geometry and ownership algorithm

1. Read the delivered GLB's embedded buffer with bounded file, accessor, vertex
   and triangle counts. Real models carry nonrendered document extras
   `fsim_liquid_boundary_v1`, schema `fsim.liquid-boundary.v1`, with original
   unsplit airframe position/index accessors, frame `gltf_model_origin_m` and
   the model root node. These preserve the original exterior before inferred
   caps and section-local float32 pivots. They add no rendered node/mesh.
   Apply every ancestor matrix/TRS and the referenced node's transform;
   use the actual producer convention `g = [-dy, +dz, -dx]` and convert to native
   body `[g.z, -g.x, -g.y]` (forward/right/down). Gear, rotors, pylons,
   control surfaces, decorative children and animation are not extra fuel space.
2. Weld identical original position coordinates and require a positive-volume
   oriented closed Manifold for this original exterior. Do not union inferred
   owner caps or use their volumes as material. Reject nonfinite/negative
   Boolean volumes. Sectioned models without the original accessors fail;
   a plain unsectioned synthetic legacy airframe remains a supported fixture.
   Exact accessor hashes bind the boundary receipt to the delivered GLB bytes.
3. Read each fixed-wing native tank's single owner from the existing damage
   document's `sections[].tanks`. Require exact tank-ID coverage, matching design
   and origin, and the authored capacity (allowing only the existing five-place
   damage serialization rounding). Do not silently move tanks to other owners.
   Unsectioned rotor material uses owner `airframe`.
4. Build conservative equipment exclusions from existing unfitted engine/crew
   dimensions. A cylinder's enclosing AABB excludes extra space intentionally.
   Also exclude **every actual native nonfuel component** in the current real
   damage document: engine, crew and control enclosing boxes. Those coordinates
   already use the structural body frame; require the identical origin and do
   not translate, shrink or crop the represented component. Unknown kinds,
   unsupported shapes and malformed/nonpositive extents fail explicitly.
   Rotor source geometry lacks the fixed-wing component declarations, so exclude
   authored cowling/nacelle/deck bounds as estimated engine/transmission space.
   Its explicitly estimated cockpit exclusion occupies fuselage span fractions
   0.08–0.38, upper 0.65 height and 0.90 width. These estimates describe uncertain
   excluded space; they do not introduce native crew/engine identities or claim
   a newly functional rotor damage recipient.
5. Subtract exclusions from the valid original exterior. Propagate outward-
   rounded intervals through the exact existing `breakup.classify` decision
   sequence used by native dry-mass ownership: nose/tail planes, mirrored
   absolute Y, projected/clipped segment coordinates, chord/plane inequalities,
   segment unions, outboard wings, and priority overrides. Each elementary
   addition, subtraction, multiplication and division expands both bounds with
   `math.nextafter`; dot accumulation does so at every operation. A box's
   possible owner set must be the native owner's singleton. Corners and centroid
   are not certificates. Impossible-owner boxes are discarded; unknown boxes
   refine before any expensive hull Boolean. Breadth/depth comes first; within
   the same depth certified boxes precede uncertain boxes, followed by nearest-
   anchor distance with stable ties. Unbounded certified-first priority would
   let a curved hull boundary starve coarser pending owner regions.
   Necessary candidate bounds derive only from these same native planes and
   outward inversion of segment t/chord/plane bands. Clip the valid original
   hull once to the union of these valid necessary AABBs to localize repeated
   Boolean work. Inferred caps never supply bounds. This localization retains
   the original pre-clip grid criterion and does not replace final predicates.
   Accept only a whole cell for which `cell - available` is empty and the owner
   and spatial-resolution checks hold. Target cell volume is required owner
   fuel volume / max(64, 16 × tank count); maximum edge is available bounding
   extent / 8. Conservative unexplored-box volume can prove insufficiency;
   it never supplies material or admits capacity.
   The search evaluates strict box-minus-interior containment before requesting
   an intersection. A positive-volume box with an empty difference is already
   contained and nonempty. If it must split for spatial resolution, its exact
   midpoint children inherit only that physical containment proof; native-owner
   certificates are still evaluated on every pushed box and final slab. The
   proof flag follows the original heap fields and unique sequence ID, so it
   cannot change exploration order, owner refinement, capacity or cell choice.
   A partially intersecting parent already supplies a materialized overlap with
   the immutable available interior. Its exact midpoint children may reuse this
   smaller immutable domain: for child `C` inside parent `P`, both
   `C intersect available = C intersect (P intersect available)` and
   `C - available = C - (P intersect available)` hold as set identities.
   An uncertain-owner child may forward an ancestor domain while still running
   every original owner certificate. Cached solids are shared between queued
   children and charged once, including the currently processed reference;
   the last reference releases the record. Aggregate retained vertices,
   triangles and record count are bounded by the existing `max_vertices`,
   `max_triangles` and `max_screened_cells` controls. Admission follows the
   unchanged processing order. A record that would exceed a bound is not
   retained; those children use the **original available interior**. No cache
   or observational metrics survive the call or enter serialized products.
   These identities do not assert numerical associativity of floating-point
   Manifold triangulations. Frozen-reference controls require identical output
   bytes and certificate order across cavities, real gaps, adjacent binary64
   planes, thin/sliver and tilted geometry, reversed input order, uncertain
   ownership, mixed cache pressure and complete cache fallback. A classification
   discrepancy requires diagnosis rather than a changed geometric tolerance.
   Unknown/partial hull membership is never inherited as containment. Every
   emitted actual slab still receives its original fresh hull, authority,
   occupied-material and equipment checks. Screening refusals preserve the
   original count-first/strict-deadline condition and prefix, adding native tank
   IDs and search state. A count refusal does not sample the clock; its diagnostic
   explicitly records that remaining time is unknown rather than inventing it.
6. Allocate the shared certified owner-cell pool by increasing cell-center
   distance to native tank anchors, with stable tank/cell ties. Each tank has
   its exact volume quota. Consume cells once; a partial allocation removes an
   actual slab along the longest dimension, on the anchor-facing side. The
   remaining slab remains available to other tanks. Density never changes.

### Bounded ordinary-owner execution

The ordinary native singleton-owner path may execute independent owner groups
in two owned Windows processes. Each still reconstructs the original neutral
GLB and current classifier from hash-bound plain design/GLB/damage recipes and
executes every original per-cell hull, owner and equipment predicate. Group
completion order never changes the canonical owner order, cell IDs, capacity
allocation or returned schema. Explicit one-CPU execution remains serial.

One absolute 300-second liquid-stage deadline starts before worker startup,
imports and reconstruction. Actual screened boxes, assignment edges and emitted
cells consume shared original aggregate ceilings; canonical per-owner pool and
remaining-resource checks still run before merging. Final global nonoverlap and
complete-result admission also check that deadline. No final geometry audit is
moved outside it. Fixed lane shares keep aggregate retained cache complexity
within the original vertex/triangle/record bounds, including active references.
The existing original-domain fallback applies when a lane reaches its share.
The measured single-worker U-2 cache peak exceeds a half share: its single-worker
timing cannot predict parallel production timing.

The parent and children stay inside the same verified two-CPU affinity lease;
each worker is pinned to one CPU before interpreter execution. This bounds
executing logical CPUs, not the number of created TBB threads or access by other
applications. Distinct physical cores and SMT siblings can perform differently.
The generation queue has FIFO disjoint CPU leases and one outer CPU union;
eight writers times two logical CPUs never exceeds sixteen. `engineering`
defaults its explicit geometry reservation to two; an explicit value of one and
the source-level `generate(..., geometry_threads=1)` default remain serial.

Automatic Windows selection obtains actual `RelationProcessorCore` masks with
[GetLogicalProcessorInformation](https://learn.microsoft.com/en-us/windows/win32/api/sysinfoapi/nf-sysinfoapi-getlogicalprocessorinformation)
and the documented
[record layout](https://learn.microsoft.com/en-us/windows/win32/api/winnt/ns-winnt-system_logical_processor_information).
Within the current allowed affinity set, it orders one available logical CPU
from each physical core before remaining SMT siblings. Core-record enumeration
order cannot change that order. The verified automatic implementation supports
one processor group; incomplete or ambiguous topology fails explicitly.
`FSIM_HANGAR_CPU_LEASE` and explicit CPU tuples retain their supplied identities
and order and bypass automatic topology selection. Default scopes preserve the
ordered inventory when intersecting current affinity; they do not revert to the
OS's numeric tuple. The generation lease pool uses the same ordering and keeps
its slots disjoint. The allowed CPU/job set is never enlarged by topology.

The reason for this resource choice is measured, not an assumed speedup: the
same frozen old U-2 producer and original inputs completed all 62,400 cells in
267.198 seconds on logical CPUs 0,2 (different physical cores), but exhausted
the unchanged global 300-second deadline at 300.240 seconds on SMT siblings
0,1 after 40,009 cell/audit emissions. Its search and edge counts were identical.
Those were DENV-only diagnostics of the old immutable package using an explicitly
authorized outer kill-only job to isolate its then-unrepaired nested-affinity
failure. They do not admit the repaired production startup path, current normal
model publication, a per-model fleet result or a contention-free CPU reservation.
Current production must still be retested with the automatic selection and all
original geometry/resource predicates.

Windows children start suspended, receive child-only affinity/lifetime jobs and
verified handle restrictions, then resume through the retained primary-thread
handle. The coordinator is never placed in its own kill-on-close job. Workers
are terminated and joined on refusal, crash or deadline, including descendants;
process affinity is restored only after owned children join. The private
CPython adapter validates the supported 3.13 initializer contract and compares
its loaded code with the actual source file; it does not mutate stdlib globals.
Positive products bind both `execution_resources` and the actual `spawn_runtime`
source bytes. Ordinary positive products therefore have 15 source roles and
the inferred RQ product 23; no-liquid applicability retains its seven roles.

The inferred RQ root-vessel policy is an explicit exception to the ordinary
whole-cell **native structural-owner singleton** predicate. Its disclosed
per-native-tank vessel eligibility can cross the original wing-root structural
cut while remaining inside the original hull and clear of equipment. The whole
unchanged native tank/module still owns its full capacity and attachment; this
does not claim OEM bay geometry, seal survival or physical fracture calibration.
Original-hull, declared per-tank eligibility, current material and global
nonoverlap predicates remain mandatory. This ordered prior-cell-subtraction
path, and the synthetic overlapping-owner path, execute serially. They are not
silently admitted to ordinary owner-process parallelism.

7. Certify every final slab against the original exterior, equipment exclusions
   and singleton native owner predicate. Distinct singleton owner certificates
   prove cross-owner disjointness without Boolean-subtracting previously filled
   cells. Recursive partition/consumption proves same-owner disjointness.
   Additionally run an independent exact endpoint AABB overlap audit over all
   final cells before publishing. Abstract synthetic overlapping owner solids
   retain their separate occupied-cell subtraction fixture path. Each actual
   cell carries its tank's native `section`; the complete tank stays in that
   native point partition for separation.
   For the declared AABB exclusions, require exact positive-interior endpoint
   disjointness: overlap in all three axes rejects, shared faces/edges are legal.
   There is no epsilon or volume allowance. Valid union subtraction still carves
   the available space. Arbitrary non-AABB synthetic exclusions retain their
   Boolean test, and original-hull difference remains a strict Boolean check.
8. Sum actual endpoint-derived volumes with `math.fsum` for every native tank.
   The fixed numerical ledger tolerance is
   `512 * binary64_epsilon * max(1, required_m3) * max(1, cell_count)`;
   this covers coordinate and summation roundoff, not packing shortage.
   Residual assignment uses `math.fsum`; an already-covered binary64 residual
   within this existing ledger budget does not become a zero-width slab.

The interval inclusion rule follows the [Boost Interval Library guidance](https://www.boost.org/doc/libs/latest/libs/numeric/interval/doc/interval.htm)
on outward rounding and true/false/indeterminate predicates. This is a Python
implementation of the existing partition, not a new Boost/toolchain dependency.
Unit controls include a narrow canard region missed by all eight equal-labelled
corners and the equally labelled centroid, and frozen classifier comparisons
one ULP either side of thresholds.

Cells use explicit low/high endpoint vertices for Boolean construction. Building
a translated cube as `low + (high - low)` can move a common face by one ULP;
the producer does not convert that numerical overlap into a permissive tolerance.

Every owner must fit independently. A tank cannot borrow another breakup
section's volume, because native separation transfers the aggregate tank's live
contents to exactly one owner. An infeasible owner is an actionable source/
ownership finding requiring separate review; it is not a density correction.

## Frozen consumer seam

Required document fields are `schema`, `design`,
`frame=body_origin_forward_right_down_m`, `sources`, `tanks`, `cells`, `ledger`,
`uncertainty` and `limits`. Each source record contains `role`, `path` and exact
`sha256`. Tank records contain native `index`, `capacity_kg`, `density_kg_m3`,
`anchor_body_m` and `section`. Cell records contain stable integer `id`, native
`tank`, `section`, `low_m`, `high_m` and actual `volume_m3`.

The sidecar binds the GLB, design/effective design spec, producer/exclusion
method, native point-partition source, Manifold binary and canonical tank-owner
mapping. `native_material_exclusion` binds the canonical object containing
`origin_design_m`, `native_damage_present` and indexed nonfuel component rows.
The whole annotated damage document is deliberately absent from this digest;
adding its required-sidecar declaration therefore cannot create a hash cycle.
An absent rotor recipient produces an explicit false-presence/empty-component
digest and retains the declared geometry estimates, never a fabricated document.
The sidecar intentionally does
not hash the annotated damage document, avoiding a hash cycle. The final
acceptance manifest must additionally bind the complete product bytes and every
consumed generator. Numerical budgets and uncertainty are explicit, without
wall-clock timings in deterministic product bytes.

After successful sidecar production, a real existing damage JSON receives only:

```json
"liquid_geometry_v1": {
  "file": "<model>.liquid-v1.json",
  "schema": "fsim.liquid-occupancy.v1",
  "required": true
}
```

Existing decoded physical numbers retain binary64 round-trip precision. A rotor
without a damage document receives a standalone material sidecar; no empty
damage document stands in for a working recipient. A no-liquid design receives
an explicit reason and empty tanks/cells. An obsolete declaration is removed
from a real existing document when the design has no positive liquid capacity.

The native consumer must validate identity, version, finite extents, actual
endpoint volume, tank/capacity ledger, ownership and nonoverlap before use.
It must consume actual native contents and material-relative transit; this
producer does not admit the separate kernel, lifecycle or installed-data gates.

## Finite budgets and reproduction

Default per-model packing limits are 65,536 cells, 1,048,576 screened boxes,
2,097,152 assignment edges, depth 60, minimum split extent 1e-7 m and 300 seconds.
GLB limits are 256 MiB, 2,097,152 input vertices and 2,097,152 triangles. Count
budgets are aggregated across owners. Time is checked between geometric calls;
an individual Manifold call is synchronous, so the outer generation runner also
needs its bounded process timeout. Budget exhaustion raises an explicit error.

## Delivered breakup topology diagnosis

The first actual F-16 preflight rejected the existing centre GLB section as
`Error.NotManifold`. Exact edge statistics on unchanged GLB SHA256
`20872257a9a5256162f1493ef0aca93a63cca1dc82e26a22922fbf3856b1a0c8`
find 29 unmatched edges in centre and 31 in left wing. Exact-coordinate welding
creates no degenerate index triangles. Manifold `Mesh64.merge()` pairs only
zero-displacement points and still rejects both sections; a numerical stitching
waiver cannot repair their missing cap topology. The other three original
sections have closed opposite-directed edge pairs.

The source cap walk used a visited-vertex set, losing distinct boundary loops
that touch at one vertex. It also closed an annulus as two overlapping disks:
the independent square-ring control has unsigned cap area 20 instead of 12.
Whole-airframe preexport closure does not test these later generated caps.

The corrected `breakup.caps` follows each boundary halfedge through its actual
incident face fan. It consumes every boundary edge once, then uses installed
[Manifold triangulation](https://manifoldcad.org/docs/jsuser/functions/triangulate.html)
with nested opposite-wound contours grouped as holes. Parallel openings at
distinct normal stations stay separate. Exactly repeated-point zero-area
triangulation bridges contribute no material surface and are removed. If an
internal cap diagonal already exists in the skin, local interior Steiner points
and an edge flip remove the four-face ambiguity while preserving the projected
domain, original skin vertices/triangles and cap boundary. Final directed-edge
audits require every skin boundary once in reverse and every internal cap edge
once in each direction; no internal skin edge may be reused.

Five independent cap tests pass, including in-memory reconstruction of all five
actual F-16 section skins with unchanged exterior coordinates, exact edge closure
and Manifold `NoError`. Raw diagnosis, RED and GREEN are `actual-topology.json`,
`caps-red.log` and `caps-expanded.log`; the five-test GREEN takes 0.492 seconds.
This does not modify or accept the existing GLB. The root must regenerate through
the normal model stage and repeat actual occupancy admission. The existing
`Design.model_key` already hashes `shape/*.py`, including `breakup.py`, so this
source correction invalidates cached models without an arbitrary version bump.

The later all-aircraft normal-model preflight exposes another cap failure on
EA-18G, E-3G and F-22A. An isolated pair of open quadrilateral fans sharing
opposite endpoints reproduces the same interior-incidence refusal for all 32
cyclic-start/storage-order cases. Every original boundary edge closes correctly,
but both independent projected caps choose the same diagonal after endpoint
welding: two halfedges in each direction, absent from the actual skin. The
repair must therefore inspect cap-only diagonal multiplicity as well as skin
edge reuse. It applies the same local within-group Steiner/flip construction
and retains the strict final directed-edge checks and unchanged source skin.
The original implementation and raw synthetic RED remain under
`build/architecture-reviews/sim-m03-production-20261103/caps-cap-collision/`
and `caps-synthetic-counterexample.{log,json}`. Actual failed-section extraction
and repaired aircraft replays are separate required evidence; the synthetic
counterexample alone does not establish the cause of every aircraft refusal.

The subsequent scratch-only authored-geometry observers confirm the same
cap-only multiplicity on EA-18G's tail, E-3G's left wing and F-22A's centre.
Each archived original input preserves every skin boundary while its offending
diagonal has two halfedges in each direction and no corresponding skin edge.
The repaired function closes those unchanged inputs. Independent reads of all
15 exported section meshes require exact coordinate-welded opposite edge pairs
and Manifold `NoError`; all pass. Geometry times are 399.299, 515.519 and
322.426 seconds respectively, each under the same 900-second diagnostic bound
with one CPU/thread. These observers write only DENV scratch products; they do
not publish normal aircraft products or constitute a fleet regeneration gate.

The maintained synthetic regression reproduces 32 subcase errors with archived
source `38ed550b0d0e8017b688c101f9664080f5b00cf8ad10aa2b620ed59b4de46c21`.
All nine breakup tests then pass in 0.571 seconds with repaired source
`d2f196f3216fa7fccf5efb26bf6e97cacb93358e4707824634d45d5a28ca0bc3`.
An older cube test's literal four-triangle fan count also fails against the
archived pre-repair function, which already uses two triangles. Its corrected
assertions require unchanged skin, the original unit-square domain, outward
winding, unit area and strict paired closure, independent of triangulation.
The historical count failure is retained separately from the new collision RED.
The scoped source, exact failed inputs, raw logs and export checks are bound in
`caps-cap-collision/closure-receipt.json` under the same evidence root.

Subsequent actual F-16 observations (`actual-owner-overlap.json`) demonstrate why
this cap topology pass is insufficient for physical containment: inferred centre
and left-wing cap solids overlap by 0.019075911429 m3, and the centre/right-wing
intersection is nonempty with signed volume -8.9091101e-6 m3. Manifold's
[official overlap discussion](https://github.com/elalish/manifold/wiki/Manifold-Library#overlaps)
distinguishes guaranteed index topology from geometric validity for self-overlap.
The method therefore uses original unsplit exterior accessors as its sole
physical envelope and interval-certified native point ownership, without any
owner-cap Boolean. Rendered caps remain explicit visual breakup estimates.

The first interval-owner F-16 attempt fails its unchanged 300-second budget
(`interval-actual/f16c.receipt.json`, 300.251 seconds), without a product. A
separate 20-second observational profile (`f16-baseline-cost.json`) finds
17.78 seconds in cell/hull intersection/difference versus 0.753 seconds in owner
intervals; centre emitted 104 cells while left wing had accepted none after
3,382 screened boxes. This motivates logical pruning and certified-cell priority;
the initial prior-cell subtraction itself cost only 0.0109 seconds. No budget,
density, capacity, physical predicate or uncertainty was relaxed. Current method
units pass 30/30 (`interval-owner-pruned-green.log`, 2.502 seconds); actual
optimized production feasibility remains a separate recorded gate.

The first pruning implementation's absolute certified-first queue was itself
incorrect: `interval-pruned/f16c.receipt.json` retains 300.210 seconds, 103,767
screened centre boxes and only 0.000317925 m3 accepted of 2.91875 m3. A bounded
spherical-cap regression (`fairness-red.log`) reproduces this starvation without
an aircraft or wall-time assertion. Breadth-first fairness with certified
priority within one depth fixes it; 32 producer methods pass in 3.001 seconds
(`fairness-green.log`). The original capacity, 300-second production bound and
containment predicates remain unchanged. An additional source-only provenance
control binds electric multirotors to the actual rotor model stage.

Classifier-derived bounds and then necessary-box-union localization preserve
the intervening failure receipts. `interval-bounded/f16c.receipt.json` closes the
left wing's full 0.525 m3 in 216.676 seconds, but the shared 300-second budget
expires during right-wing packing; this proves left capacity feasible, not full
admission. `interval-localized/f16c.receipt.json` subsequently closes all four
tanks in 141.758 seconds with 14,134 cells. That epoch is **not equipment-clearance
admitted**: `actual-component-clearance.json` independently finds liquid in four
actual native actuator boxes (flaperons and leading-edge flaps).

The native-solid repair adds those current native components, along with all
other nonfuel native boxes, to the existing conservative exclusions. A separate
diagnosis (`native-tangent-diagnostic.json`) shows why zero-volume Manifold faces
cannot define box interior overlap: a slab ending at y=0.25 exactly touches the
solid starting at y=0.25, while a nonempty, zero-volume coplanar intersection is
returned after a particular preceding mesh sequence. Exact AABB endpoints give
overlap lengths [0.125, 0, 0.125]. The strict positive-interior predicate allows
that boundary and rejects one-ULP penetration; no tolerance is substituted for
an arbitrary non-AABB geometry result.

Current producer controls pass **39/39 in 3.024 seconds**
(`native-exclusion-green.log`); independent admission controls pass **18/18 in
0.087 seconds** (`collector-native-exclusion-green.log`). The actual read-only
F-16 producer closes four tanks into **14,432 cells in 123.705 seconds**, within
the unchanged 300-second budget (`native-solid-clear/f16c.receipt.json`). Input
TOML/GLB/damage bytes remain unchanged. Product SHA256:
`f78013d4a08a769bfacbece3c0a24df6d7821cc27e2c8d47eeaccd85688a27e6`.
`native-solid-clearance.json` independently verifies native XML capacities,
owners/origin, exact original-accessor bytes, 96,009 AABB candidate comparisons,
the canonical native exclusion digest and **zero positive intersections with
all nine actual nonfuel components**. It is a single-model diagnostic receipt;
normal-entrypoint generation, all35 actual products, native/installed execution
and equipment accuracy remain distinct gates.

## Actual population admission

`tools/validation/weapon_quality/liquid_population.py` exposes the pure
`admit(coverage, coverage_sha256, root=ROOT)` function and
`--coverage --root --output` CLI. It derives identities from maintained TOML
sources independently of supplied coverage, requires exactly all35/32 liquid/
149 native tank IDs and three explicit no-liquid designs, and binds actual
source, native XML, GLB, real damage and sidecar bytes. Re-running admission must
reproduce the exact deterministic dictionary; no timestamp or cached success
stands in for current bytes.

The collector independently checks native units/capacities, density policy,
complete tank/component ownership, structural origin, actual endpoint volumes,
global AABB nonoverlap, every actual native nonfuel exclusion and all source
epochs (including native point partition and canonical material exclusion).
It binds original-boundary accessor receipts to actual GLB bytes, but **does not
rerun Manifold containment or claim independent geometric truth from JSON
booleans**. Producer geometry receipts, runtime contents/transport, installation
and empirical equipment calibration retain their separate meanings. Missing,
stale, mixed, malformed or incomplete products fail admission.

The existing normal `model` stage writes the sidecar even on a GLB cache hit:
`Design._components` for fixed wings and `Rotorcraft.model` for rotors. The
product is outside the GLB cache key. There is no extra flight build or remesh.
Use the existing launcher and its declared D:/ENV Python dependency path. The
independent geometry-method fixture command is:

```powershell
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
$env:PYTHONPATH = 'D:/ENV/fsim-m03-python-dependencies;D:/Projects/Cpp/Super-DDCCSS-WWorld/tools/models/hangar'
python -m unittest discover -s tools/models/hangar/tests -p test_liquid.py -v
```

Fixtures cover analytic capacity, real partial slabs, hole/cavity containment,
engine/crew exclusions, independent exact AABB nonoverlap, stable bytes,
all-ancestor transforms, material-split vertex welding, closed-section union,
malformed/open GLBs, invalid materials, finite budgets, owner shortage, native
ID/unit conversion, additive damage metadata and no fake rotor damage proxy.
Raw RED and subsequent executions are retained under
`build/architecture-reviews/sim-m03-production-20261103/liquid-occupancy/`.

The frame-correction run passes 20 Python/Manifold tests in 1.538 seconds
(`frame-roundtrip-green.log`); the subsequent explicit fluid-policy fixture
extends the producer suite to 21 tests (`producer-policy-final-green.log`).
The actual `model3d._to_gltf` asymmetric box and
tetrahedron roundtrip, with nonzero origin, ancestor matrix and child TRS,
first fails both subcases against the erroneous X/Y signs
(`frame-roundtrip-red.log`). The corrected importer preserves the physical
native frame and all capacity/containment predicates. Earlier 19-test evidence
did not establish this actual-producer frame roundtrip and is superseded.

Four earlier synthetic packing cost receipts in `metrics.json` take
0.296037 seconds combined: two-tank box 113 cells/623 screened boxes, equipment
cavities 138/728, rotated thin prism 120/752 and full-capacity box 512/1,023.
Their maximum absolute volume error is 2.78e-17 m3. These small synthetic timings
precede the import-frame correction and are not a current production-model
cost estimate.

Source enumeration currently identifies all 35 aircraft: 32 liquid models and
149 native tank IDs; skua, cf2 and iris are liquid-inapplicable. Independent
method fixtures are not all-model packing acceptance. Current-source complete
generation, all-owner feasibility, measured production cost, installed products,
native finite-liquid transit, original combat scenes and presentation remain
the root integration gates.
