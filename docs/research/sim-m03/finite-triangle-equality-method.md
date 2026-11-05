# Finite triangle equality reduction

The finite source/triangle CCD query represents a moving finite line by
`p(t,q)=a+v*t+d*q`, with `0<=t<=h` and `0<=q<=1+k*t`. Authored triangle planes
include an exact opposite normal/bound pair. That pair is one equality, not a
finite-thickness band. Sequential polygon clipping by both half-planes can
round a feasible line down to a point and later discard it.

The retained exact-coefficient witness has a feasible interval from
2.314661032949634e-5 to 2.4254221073377637e-5 seconds. The ordinary native
public-scene method test reported RED at the unchanged authored triangle:
one named case, 25 assertions, one failure; all strict interior/finite-coordinate
checks passed and the expected skin contact was absent. This is a generic CCD
method witness derived from an independent replay query. It is not presented
as an actual internal R27R contact trace, a hardware measurement, or proof of
which encounter outcome the numerical defect affects.

## Method and numerical contract

For original rounded double coefficients `O+R*t+S*q=0`, the usual substitution
for `S!=0` is `q=-O/S-(R/S)*t`. Every remaining half-plane becomes a scalar
linear inequality on time. If `S=0,R!=0`, time is fixed and the feasible along
coordinate is an interval. A zero offset/rate/slope equality is redundant;
a nonzero offset with zero rate/slope is inconsistent. Equality elimination
preserves the feasible set under substitution; this is the standard affine
constraint elimination described in Boyd and Vandenberghe, section 4.1.3.
[Convex Optimization, printed p. 132](https://www.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf)

The implementation uses `x=t/h` and eliminates the larger of `|R*h|` and `|S|`
in extended arithmetic. It clips the remaining coordinate by the original
four domain inequalities and remaining planes. It returns one genuine point
or both interval endpoints to the original common lexicographic `(time,along)`
selection. In particular, fixed-time entry chooses the lower along coordinate
and exit chooses the upper; it does not publish an arbitrary representative.

Only EXACT opposite original normal components AND plane bounds qualify.
The actual originally computed DOUBLE offset/rate/slope tuples must also be
exact negatives before conversion to extended arithmetic. Near-opposites,
finite slabs and unexpected coefficient asymmetry retain the old polygon path.
The two equality inequalities are not re-tested after reconstruction, which
would recreate the original cancellation. Nonfinite or unsafe arithmetic also
falls back. Nonzero products/divisions becoming zero, remaining rate/time
scaling and output conversion underflow also fall back; this is conservative
portability protection, not a claimed native test on every long-double format.
There is no geometric epsilon, angular cutoff, thickened triangle,
forced feasible clamp or newly created interval around a singleton.

Extended precision reduces arithmetic error; it is not a formal exact-IEEE
certificate for arbitrary data. The exact Fraction method reference separately
compares the one-dimensional solution with all-pair two-dimensional vertex
enumeration. All 383 reference comparisons agree, including fixed-time,
coplanar, parallel, singleton, contraction and endpoint controls.

## Exact implementation boundary

The additive flag on the private `finiteLineContact` defaults to false. It is
enabled at ONLY the two calls in the existing closed-mesh triangle visitor:

1. The mesh AABB broadphase before triangle traversal.
2. Each finite authored-skin triangle query.

The broad box belongs to the whole validated closed mesh, not an arbitrary
per-triangle open volume. Valid nonzero mesh volume normally prevents an exact
zero-width axis, but the same narrow opt-in covers a rounded degenerate box.
No material box is enlarged. Generic solid boxes, finite surface slabs,
capacity/liquid faces and other callers retain the original default path.

Original normal selection, initial closed-skin occupancy/inward-boundary
logic, endpoint-exit refusal, final finite/time checks and stable inter-face
ordering remain in one unchanged common postselection block. New helpers are
private geometry types; no installed interface, enum value, old class layout
or vtable changes. No source/scene/model/card/strength/fuze/energy criterion is
changed by this method repair.

## Native validation boundary

The method tests include the original native RED witness, private fixed-time/
lexicographic/singleton/finite-slope controls, exact rounded-coefficient
asymmetry and nonfinite fallback, finite-width/near-opposite controls, a
six-plane axial broad-box control, and actual closed-tetrahedron axial,
coplanar, initial/final and receding negative scene cases. Synthetic unit
identities/history are explicitly labelled; they are not encounter evidence.

Candidate preparation does not claim a native GREEN. The normal native build,
all original damage/liquid/contact tests and affected encounter gates must be
executed by the serial native-slot owner after review. Changed skin traversal
counts in a fresh run remain actual native results, not a reason to edit an
original acceptance predicate. The retained pre-repair RF/IR captures and
their independent-reference discrepancies remain separate numerical cohorts.
