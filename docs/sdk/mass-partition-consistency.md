# Mass distribution and detachable section consistency

The empty flight model and every detachable section use one nonnegative spatial
mass distribution. This closes a source inconsistency exposed by the fixed
R-37M launcher-loss encounter: the F-16 parent had negative pitch/yaw inertia
after its nose, tail and wings separated. Native parallel-axis subtraction was
algebraically correct. The damage writer partitioned a different distribution
from the authoritative whole-aircraft tensor.

For the previous F-16 source, geometry particles gave pitch inertia
99,977.5762 kg m², while the declared whole-aircraft tensor gave
72,065.3431 kg m². Section centroid offsets alone required 85,201.4026 kg m².
Scaling each section's own-centre tensor therefore could not repair the source.

## Reference pose and source moments

`MassModel.empty` retains the previously declared mass, CG and full tensor for
compatible aircraft with authored gyration. The source-supported F-16 diagonals
remain 11,236.47875796528, 72,065.343134412 and 78,834.83737123005 kg m².
NASA-derived gyration supports the diagonal moments; the existing geometry
cross products are an engineering policy, not measured NASA full-tensor data.
The full target is frozen before changing its inferred spatial realization.

Retractable landing gear in this clean, in-flight reference uses the authored
`Leg` hinge/swing/twist transformation. Deployed contact-wheel coordinates do
not describe the retracted empty-mass distribution. Each gear item's mass and
the transformed representative point stay fixed. Fixed gear is unchanged.
The representative point approximation is not a measured hardware density.

The inferred systems distribution occupies the actual fuselage superellipse
volume, using 40 longitudinal stations and seven-point Gauss-Legendre transverse
quadrature. It replaces the former constant-height axis support where necessary
to realize the declared tensor. Structure retains its geometric support.
Engine, gear and explicitly placed item weights are fixed, preserving their
mounted centroids and own tensors. Each other component retains its mass and
point coordinates; only its nonnegative normalized quadrature weights change.

## Deterministic constrained spatial realization

For particles of mass `m_i w_ij` at `r_ij`, normalized weights satisfy
`sum_j w_ij = 1`, `w_ij >= 0`. The constraints preserve the declared first moment
and all six second moments. The solver minimizes mass-weighted relative entropy
to the engineering prior. Its convex dual uses bounded Newton iterations and a
bounded line search in dimensionless coordinates. The residual predicate stays
`2e-12`; objective roundoff does not permit a larger physical residual.
Nonfinite, physically impossible or support-incompatible targets are refused.
There is no fallback, encounter-dependent fit, native inertia clamp or reset.

This solution is not unique equipment calibration. A 25% section-moment
screening half-width is an explicit assumed uncertainty allowance, not a
confidence interval. Whole measured/source-derived target constraints remain
fixed. Distribution uncertainty is correlated: section samples cannot vary
independently while retaining the whole tensor. The receipt records the support
extents, normalization scales, method, residual, reference pose and policy.

Aircraft without declared gyration retain their existing geometry method.
The ten explicitly revised class-prior sources below opt into
`inertia_reference = "geometry-clean-v1"`, including the clean gear transform.

## Explicit retirement of incompatible class priors

These sources explicitly described class/other-aircraft estimates, rather than
equipment-specific measurements. Five priors violate physical second-moment
positivity; the others cannot be realized with their fixed mounted components
and actual geometry support. They now declare the same geometry/component-mass
distribution for whole and sections, retaining mass, CG, geometry and engines.
The old values and reason remain in each authored source. Gripen's original
prior is feasible with physical volume support and is retained.

Values below are source-construction results in kg m², ordered
`Ixx,Iyy,Izz,Pxy,Pxz,Pyz` with structural plain products. Actual generated
products and native execution require their own receipts.

| Model | Retired estimate | Geometry-clean source result |
|---|---|---|
| C-17A | 6.24019e6,1.16762e7,1.77860e7,18.8861,1.98629e6,45.4748 | 7.86957e6,1.32445e7,1.91387e7,18.7276,2.20684e6,45.3163 |
| E-3G | 3.97796e6,5.58308e6,9.60029e6,-4.74914,580589,3.50996 | 4.01735e6,8.10653e6,1.14564e7,-4.74914,748293,3.50996 |
| F-22A | 66016.6,325976,392920,-.160158,26446,1.44223 | 48031.3,286568,317948,-.158994,24038.2,1.43454 |
| F-35A | 25714.6,131201,156657,.218069,5685.66,.419482 | 23065.9,132531,148388,.218069,6732.59,.419482 |
| J-10A | 14375.4,112366,112890,.0584098,2834.5,.0090475 | 9829.32,127622,131287,.0584098,4428.89,.0090475 |
| J-20A | 55493.6,406090,415723,.805263,-946.469,.370896 | 33787.6,385726,413204,.805263,2700.31,.370896 |
| KC-46A | 2.91268e6,7.19005e6,1.00273e7,-1.75126,337085,2.25930 | 3.63964e6,8.50423e6,1.17091e7,-1.75126,395356,2.25930 |
| MiG-29A | 23076.5,134377,152272,-.00165893,3656.86,.0716492 | 22750.5,122751,138413,-.00165893,5853.23,.0716492 |
| Su-27S | 59630.8,364306,408855,.243270,15673,.501759 | 63952.7,400498,443019,.243270,18730,.501759 |
| Su-57 | 62826.4,338502,399674,-.846264,25827.4,.604250 | 60908.9,289458,333279,-.874273,25342.2,.639661 |

## Store inertia conventions

The 24 missile sources and CJ-10A explicitly use
`mass.gyration_reference = "body-length-v1"`: `I_i = M (g_i L_body)^2`.
The existing dimensional native missile and cruise writers already used this
convention; their intended native tensors, propulsion and phase times are
unchanged. Applying aircraft Roskam denominators inside an intermediate mass
model was incorrect. Unknown bases and nonfinite radii are refused.

Other generated stores retain Roskam normalization: radii use half-span,
half-length and quarter-sum respectively. `storedesigns._gyration` already maps
the dimensional solid-cylinder, thin-shell or box approximation into this
basis. No blanket reinterpretation is applied. Stores use an analytic whole-body
mass representation and do not acquire aircraft detachable section partitions.

## Independent production checks

`engineering.mass_partition_audit` reads actual generated XML and serialized
damage JSON, without calling `MassModel` or its fitting equations. It checks:

- All 88 primary models: finite positive empty mass, finite CG, body-frame full
  tensor positive semidefiniteness and physical second-moment covariance
  `C = trace(J)/2 * identity - J` positive semidefiniteness.
- Every fixed-wing section: nonnegative mass and a physical own-CG tensor.
- Additive dry mass, first moment and full tensor via the parallel-axis theorem.
- Every nonempty retained subset: subtract removed moments, recompute its CG,
  shift to that CG, and check both tensor and covariance positivity.

Mass, CG and tensor fields serialize at binary64 round-trip precision. The
oracle maps the unchanged dimensionless solver tolerance to a dimensional error
budget, separately from arithmetic roundoff; it never modifies physical inputs.
For a nonzero solver tolerance, the normalization axes must fit a conservative
extent independently recovered from serialized panel, body, contact and
component geometry. Missing or nonfinite geometry proof is refused, so arbitrary
finite metadata cannot enlarge the admission budget. Derived budgets, aggregate
moments and computed errors must also remain finite.
Generation receipts bind the actual XML/damage hashes and policy, and include a
required `serialized-mass-partition` check. The normal engineering campaign
publishes `mass-partition-closure.json`; completeness requires exactly the full
declared 31 fixed-wing, four rotorcraft and 53 store roster.

The root-observed Python source/serialized F-16 RED and normal native sequential
removal RED precede regeneration. Source feasibility is not native acceptance.
All-88 normal generation, installed execution, sequential removal and the fixed
physical encounter remain required in the final campaign evidence.
