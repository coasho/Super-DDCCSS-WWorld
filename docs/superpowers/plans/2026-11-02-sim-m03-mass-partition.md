# Consistent authored mass moments and physical section partitions

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.

Actual fixed R-37M launcher-loss evidence exposed negative F-16 parent pitch/yaw
inertia after sequential nose, tail and wing separation. The native negative
point-mass parallel-axis removal and section own-centre tensor subtraction use
consistent rigid-body algebra. The source distributions do not: `MassModel.empty`
replaces geometry moments with authored gyration moments, while `damage._masses`
partitions the unadjusted quadrature.

## Exclusive write allowlist

- `tools/models/hangar/hangar/mass.py`: deterministic bounded nonnegative
  quadrature reconciliation to the existing authoritative mass/CG/full tensor;
  component masses and mounted engine/gear/item weights remain fixed;
  existing no-gyration behavior stays except explicitly authored clean policies.
- `tools/models/hangar/hangar/damage.py`: consume that same distribution and retain
  precision of physical mass, CG and tensor values.
- `tools/models/hangar/hangar/engineering.py`: independent generated XML/damage
  additive-moment and section/complement positive-semidefinite closure receipts.
- `tools/models/hangar/tests/test_damage.py` and new
  `tools/models/hangar/tests/test_mass_partition.py`: independent physical oracles.
- `tests/simulation/test_mass_change.cpp`: sequential actual F-16 section removal.
- New `docs/sdk/mass-partition-consistency.md`: equations, provenance, inferred
  distribution uncertainty and declared infeasibility behavior.
- This plan.
- `tools/models/hangar/hangar/missiles.py`: emit the existing full-body-length
  inertia convention as an explicit typed source contract.
- The exact authored missile/CJ-10A source files
  `aircraft/{aim120c,aim120d,aim7m,aim9m,aim9x,asraam,cj10a,irist,magic2,meteor,micaem,micair,pl10,pl12,pl15,pl8,r27er,r27et,r27r,r27t,r37m,r60m,r73,r77m,super530d}/<name>.toml`:
  declare `gyration_reference="body-length-v1"`; scalar values stay fixed.
- The exact unsupported-prior source files
  `aircraft/{su57,f35a,f22a,c17a,e3g,j20a,kc46a,j10a,su27s,mig29a}/<name>.toml`:
  explicitly retire the incompatible class/analog prior, retaining its old
  full tensor/reason, and declare geometry-clean reference and uncertainty.
  Gripen's compatible original prior is retained.

No native mass solver, thresholds, inertia clamps, source-supported diagonal
targets or historical acceptance documentation is modified. Root owns all native execution,
normal all-88 generation, source export and produced-overlay delivery.

## Method and gates

Use a minimum-relative-entropy redistribution of nonnegative normalized weights
on geometry quadrature points, preserving each source item's mass and
mounted component centroids. Source-semantic corrections use the existing
authored Leg stow transform and systems support within the actual fuselage
volume. The nine global first/second-moment constraints match the
already authored total. Solve the dimensionless convex dual with deterministic
bounded Newton/line search; detect redundant constraints and reject infeasible,
nonfinite or nonconvergent source inputs explicitly. Spatial redistribution is an
inferred engineering distribution with disclosed uncertainty, not new equipment
calibration or an encounter-dependent correction.

Before production edits, root must execute the new independent F-16 raw-moment
and generated-artifact closure fixtures and the native sequential-removal case.
They must expose the existing mismatch/negative inertia. The final independent
oracle uses actual generated XML and damage records, not the fitting equations,
and checks additive mass, first moment and full tensor plus PSD sections and all
nonempty complements. All 88 normal production models and the unchanged actual
R-37M scene remain required. Pure source analysis finds all 31 fixed-wing and
53 store constructors viable. Four rotors use their dedicated authored mass
schema. This is not generated/native acceptance. Root recorded the two Python
physical REDs in checkpoint32 and the old-XML native sequential-removal RED in
checkpoint36 before regeneration.
