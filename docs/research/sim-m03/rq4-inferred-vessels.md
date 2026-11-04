# RQ4B inferred inboard liquid vessels

This is an explicit theoretical geometry and attachment method for the current
RQ4B design. It supplies no OEM tank CAD, measured fracture response, or new
equipment calibration. Its source declaration is
[`rq4b.liquid-geometry-v1.json`](../../../aircraft/rq4b/rq4b.liquid-geometry-v1.json).
Accepted flight TOML/XML, capacities, native tank numbers, native tank positions,
CG, inertia, aerodynamic tables and physical reset semantics remain authoritative.

## Evidence and the earlier negative result

The retained singleton structural-owner calculation required 4.905 m3 per wing
at the existing 800 kg/m3 reference density, against a reported 4.724 m3 upper
wing volume. That is an impossibility result for that placement constraint;
this feature does not relabel it as a pass. The historical numbers and original
failure remain in the production evidence. They are not recomputed or certified
by this document.

The [NASA Global Hawk description](https://www.nasa.gov/aeronautics/global-hawk-uas/)
lists 15,436 lb maximum tank capacity for its aircraft. This differs from the
RQ4B capacity context in the accepted source design; no NASA aircraft tank
dimensions or capacity are copied into this model. NASA
[TM 102840](https://ntrs.nasa.gov/api/citations/19910007688/downloads/19910007688.pdf)
evaluates the LRN 1015 airfoil and gives its thickness/design context. It is a
source for section geometry, not internal fuel bays, spars, plumbing, or an
RQ4B fracture interface. The accepted local LRN coordinates and original wing
stations remain the numerical geometry authority. Public A-family cutaways or
qualitative root/fuselage descriptions cannot establish B-family vessel CAD.

The narrow inference made here is that each existing inboard tank may include
the corresponding side of the original wing root volume up to the centreline.
This is an engineering hypothesis constrained by original geometry and actual
capacity. It is not inferred merely from an RQ4B name or fitted to an engagement.
The source declaration contains no new capacity, density, hull dimensions or
flight parameters.

## Geometry, tank eligibility and allocation

Only native TK0 (left inboard) and TK1 (right inboard) receive the additional
eligibility. TK2 and TK3 retain their original ordinary wing region. The
existing structural root cut is derived from the original fuselage width at
the original wing root half-chord, plus the existing 0.05 m partition offset;
for the retained design the reported cut is ±0.7738875 m. It is not enlarged to
meet capacity.

[`liquid_vessels.py`](../../../tools/models/hangar/hangar/liquid_vessels.py)
uses the original wing primitive, original section airfoils, taper, twist and
coordinates. It meshes the centreline-to-root-cut strip at the existing shape
cell/error settings, without wing skin inflation, body smoothing or an
extrapolated root. It rejects nonclosed, nonoriented or invalid solids. This
polygonal solid is an additional eligibility restriction, not an exterior:
every candidate and every final slab must also be inside the delivered
original unsplit hull and outside the unchanged equipment exclusions.

The inferred packing identity is `nested_eligible_shared_pool_v1`. It searches
one disjoint pool inside the original exterior minus equipment and the union
of the unchanged necessary eligibility regions. It does not repeatedly regrid
an interior punctured by each earlier tank's cells. Ordinary aircraft continue
through their existing producer path, without this inferred method.

Every candidate has definite per-tank eligibility before entering the pool.
An unresolved restricted-tank predicate cannot be treated as false merely
because its flexible tank is already certified. The permitted masks are left
ordinary `{0,2}`, right ordinary `{1,3}`, left root-only `{0}`, and right
root-only `{1}`. Unknown masks refine; unsupported overlapping or nonnested
families refuse. The accepted cell-volume target is the minimum of the original
single-tank targets among its eligible tanks, each `capacity/density/64`.
Combining demands must not coarsen those targets. The original maximum-edge,
depth, cache, screening, assignment-edge, cell and 300-second global limits
remain; search, allocation, fresh audits and receipt construction share that
deadline.

Only all-definite parent eligibility facts propagate to exact child subsets;
any unknown mask refreshes the original predicate. A refreshed box computes
the original structural classifier once, and each native tank receives that
same immutable result in the unchanged certificate body. The public scalar
predicate retains its original values and refusals, including adjacent-double
geometries for which the existing Boolean kernel refuses.

For a materialized valid overlap `S = B intersection D`, the search may contract
`B` once before its existing midpoint split. The accepted contraction uses the
raw finite ordered axis extrema of `S` only if they lie inside `B`. Therefore
`S subset bbox(S) subset B`: no available material is discarded, disjoint
queued siblings remain disjoint, and the same bounded immutable partial domain
can be shared by their children. Unsupported, nonfinite or outlying bounds
retain the original cell and normal split. No shrink factor, source mesh,
resolution or finite budget changes. Native overlap/candidate temporaries are
cleared after use, including empty intersections and cache refusal.

This relies on Manifold 3.5.4's documented represented-vertex enclosure, not
an approximate centre/extent or smooth-CAD bound. Its
[C++ BoundingBox implementation](https://github.com/elalish/manifold/blob/v3.5.4/src/manifold.cpp)
returns the materialized vertex bounding box; its
[double-coordinate Box type](https://github.com/elalish/manifold/blob/v3.5.4/include/manifold/common.h)
uses componentwise vertex extrema, and the
[Python binding](https://github.com/elalish/manifold/blob/v3.5.4/bindings/python/manifold3d.cpp)
returns those six doubles directly. Linear triangle interiors lie within their
vertex extrema. Adding an outward ULP was tested and rejected: it moved an
equipment boundary from `-0.875` to `-0.8749999999999999`, creating a genuine
positive overlap; another cavity escape measured
`4.336808689942018e-19 m3`. The retained strict final gates refused both.
The correction preserves raw represented faces; it does not move them inward
or apply an epsilon allowance.

For each side let `O` be disjoint certified ordinary pool volume, `R` certified
root-only pool volume, `dr` its restricted demand, and `df` its flexible demand.
Search closes only when both `O >= dr` and `O + R >= dr + df`. These conditions
are sufficient for this nested fractional-volume allocation: first give `dr`
from ordinary material to the restricted tank, leaving `O-dr+R >= df` for the
flexible tank. A combined-volume check alone is insufficient. This proof uses
actual certified pool boxes, never the geometric `available.volume` upper
bound. The latter may reject insufficient physical volume, but cannot admit
either demand.

Allocation is canonical TK2, TK3, TK0, TK1, ranked by the original pool-box
centres' distances to each unchanged native anchor within its eligible pool.
Each partial allocation consumes an
exact axis-aligned slab and retains only its disjoint residual. It does not
borrow another tank's capacity or the opposite root half-strip. Every emitted
slab receives fresh original-hull, per-tank certificate and typed eligibility
checks and unchanged equipment checks. The prior-tank audit compares the
validated exact endpoints of every earlier native cell, captured once per tank
in read-only arrays, with no current-tank cells. All three intersection lengths
must be strictly positive to constitute an overlap. A single-ULP positive
intersection refuses, without tolerance or refinement to hide it. Zero positive
candidates prove volume disjointness and omit the redundant general CSG contact
query; no fake empty query or query count is recorded.

That numerical change has a retained independent counterexample: 134 actual
prior boxes and one TK0 slab had zero positive-interior intersections and nine
boundary contacts under exact `Fraction.from_float` min/max arithmetic. The
general whole-union Manifold query nevertheless returned `NoError` with signed
volume `-8.673617379884035e-19`. The original strict refusal remains in the
evidence. The endpoint proof changes the numerical algorithm, not the physical
nonoverlap criterion already used for native equipment boxes. No negative
Manifold volume is accepted. General hull/source-wing/arbitrary-exclusion
Booleans retain their strict status and finite/nonnegative-volume checks.
The independent global AABB audit rejects positive-interior overlap; common
faces occupy no fluid volume. All capacity ledgers use the existing physical
density and binary64 roundoff contract.

The product's `inferred_vessels_v1.packing_v1` records the method, canonical
assignment order, four original resolution targets, definite-mask contract and
both nested certified-volume conditions. The independent collector checks
these identities, capacities, numerical bounds and final cell sizes. As with
the original hull receipt, it binds the actual producer source and inputs; it
does not independently reconstruct Manifold containment from JSON booleans.

This is a new estimated internal distribution. It does not claim cell-payload
equivalence to the earlier failed per-tank RQ packing. The retained full attempt
exhausted its original deadline after TK2/3 completed and TK0 certified only
1.3884633577 of 2.875 m3. A separate interrupted 60-second TK0 profile measured
3.12058229947 m3 net geometric availability after 2,444 prior cells, with zero
cache fallbacks and materialized Boolean evaluation dominating its cost.
That positive upper-volume margin did not establish eligible capacity or a
successful product. Neither retained failure is relabeled as passing evidence.

Manifold's [performance guidance](https://github.com/elalish/manifold/wiki/Performance-Considerations)
describes lazy CSG evaluation and reusing evaluated results. It supports
measuring the materialization calls and avoiding redundant searches; it does
not establish bitwise equivalence under Boolean reassociation. The shared
method therefore retains literal fresh final checks and requires actual
complete production within unchanged controls before admission.

Each cell has a typed `eligibility_v1`:

- `native_owner_interval_v1`: the original singleton structural-owner enclosure;
- `inferred_original_wing_root_v1`: the selected inboard tank's original source
  wing root half-strip.

The new product sets `native_owner_interval_certified` to **false**, because
centre-classified root material is not proved to belong to the original wing
skin partition. It instead requires
`per_tank_vessel_eligibility_certified: true`. The complete embedded
`inferred_vessels_v1` declaration, actual mesh digest/settings, ordinary/root
capacity split and centroid report are required by the current collector and
private native parser. A malformed or unsupported current marker must refuse;
it cannot become legacy geometry.

The declaration's optional presence and exact hash participate in normal
generation reuse, including recursively referenced designs. Adding, removing,
or editing it invalidates reuse. Source receipts bind the declaration, helper,
original shape/section/airfoil methods, actual LRN coordinate bytes, mesher
wrapper and actual mesher binary. A stale loaded foil table must refuse before
root meshing; a coordinate edit with unchanged TOML/old GLB cannot reuse a fresh
root receipt. Existing GLB,
native owner, native exclusion, design, canonical effective-spec and Manifold
bindings remain mandatory. The collector checks source roles independently
and does not regenerate geometry or claim independent Manifold containment.

## Attachment and mass-distribution discrepancy

`sealed_whole_native_tank_wing_module_v1` explicitly assumes the inferred root
bladder/module stays sealed and attached to its whole existing native wing
tank during coarse native section separation. Existing skin cuts do not prove
this. The feature does not move a root cut, invent a new fracture plane, add
dry structural mass, change native damage strength, or validate seal survival.
Partial fracture, bladder tearing, plumbing disconnection and redistributed
fluid-rigid inertia are outside this inference.

All cells and the actual captured native kilograms follow the same existing
TK mechanical owner once on departure. Child coordinates subtract the native
section origin shift; the parent retains none of that TK's material. Root
cells can extend beyond an old fuel probe or the separated visible skin. A
real native scene must demonstrate later child-only liquid work at an actual
root cell, exercising material bounds in broad phase, rather than proving only
that a parser accepted the cell. Opposite-side tank contents must be preserved.
Physical child reset remains the existing empty native piece reset; it does
not refill detached fuel or cancel an independently emitted field.

The emitted full-capacity and half-capacity/reference-up reports integrate
actual cell volumes and first moments. In body axes reference up is `[0,0,-1]`,
so half contents occupies the exact cell portions with z above a common free
surface. It uses unchanged density, not half density throughout full boxes.
Reports give occupied bounds, centroid, and vector/norm offset from the
unchanged native lumped tank anchor. These offsets are a model discrepancy,
not covariance, confidence bounds, or permission to move native CG. Runtime
occupancy uses the actual captured attitude/up and contents rather than this
reference diagnostic. Slosh and thermal density corrections remain separate.

## Delivery and required verification

The producer embeds the complete runtime policy in `rq4b.liquid-v1.json`, which
normal runtime staging delivers; the runtime does not open the authored declaration from
an installed aircraft directory. Source export must retain the new authored
declaration, producer, tests and this method document. No new public layout,
vtable, wire format, native tank ID or runtime product identity is introduced.

The initial source window authorizes no model publication or native execution.
Source implementation alone does not establish the available net root volume,
successful packing, independent admission, actual contents/departure effects,
installed consumer provenance, all-model production or SIM-M03 acceptance.
Required gates include producer declaration/geometry/allocation negatives,
collector missing/stale/forged marker and source-role negatives, source input
epoch tests, real RQ4 normal generation and all35 collection, native parser
refusals, actual root-cell child-only work/kg/ownership/reset controls, and the
unchanged broader production, native, installed/exported and visual gates.

The shared-pool feature's initial ten-case missing-implementation RED,
subsequent prior-array RED, initial full 300-second search failure, refinement
REDs and exact boundary counterexamples are retained separately. The initial
101-case scoped Python gate passed fifteen shared-pool controls, source and
admission negatives, ordinary liquid fixtures and the sixteen frozen ordinary
packing equivalence cases. It ran on one CPU with an explicit one-CPU geometry
lease. The earlier widened invocation omitted that explicit lease width and
five existing normal-produce fixtures refused the mismatched two-CPU default;
its raw failure is retained. The refined scoped gate passes 106 cases, including
twenty shared-pool controls, the self-contained 134-box exact-Fraction witness,
one-ULP positive-overlap refusal, original predicate values/refusals, subset
inheritance, lossless bbox/cavity controls and all earlier ordinary equivalence
cases. It uses no DENV witness file or custom test injection. These unit results establish neither actual RQ
runtime completion nor final normal-product/native/full-campaign acceptance.

## Inscribed-column development state

The current producer additionally contains an engineering search proposal for
thin sloping material. All represented MeshGL64 facets constrain a central
vertical band; exact rational plane extrema propose inward-representable
candidate endpoints. A proposed core receives an actual intersection check
before any containment flag. A conservative eight-corner/twelve-triangle
certificate requires outward faces, two triangles per box face, supplied merge
topology and reverse-paired edges; unrecognized cases retain the original
strict difference proof. It never coordinate-welds distinct coincident fans or
accepts a negative Boolean result. The core and two residual slabs partition
the parent box; after maximum-edge compliance, facet sensitivity guides
residual refinement. Source geometry, density, capacities, root Policy and final
original-hull/equipment/eligibility/prior/global checks remain unchanged.

This is our engineering search method, not a Manifold packing algorithm. Six
new bounded controls pass, including seam topology, duplicate shell/cavity and
represented one-ULP notch refusal, exact partition and an actual shared-search
affine-wedge fixture. The latter conserves its declared 15/512 m3 demand under
unchanged finite cell/screen/edge/depth limits and a stricter 60-second fixture
bound. No actual RQ subset or full product has been admitted for this producer
epoch. Broader current-source verification and final independent review remain
pending. Two resource observations remain for follow-up: eager vertex-to-rational
conversion precedes its first loop deadline check, and already-contained
resolution children unnecessarily retain a native overlap cache record.
