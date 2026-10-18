# SIM-M02 Continuation Implementation Plan

> **For agentic workers:** Use test-driven implementation with independent review. The root agent alone owns the serial native SDK build/test slot.

**Goal:** Advance SIM-M02 from import/runtime prerequisites to executable independent flight and aerodynamic comparisons, preserving explicit physical evidence limits.

**Architecture:** Offline reference programs consume authored game designs and verify generated native models. They never execute client-side simulation in the game. Source admission, numerical agreement and equipment validation are separate report results.

**Tech Stack:** C++17, Python 3.13, UCRT64 presets, JSBSim, RocketPy 1.13.0, SU2 8.5.0, OpenVSP 3.54.0.

**Spec:** [Missile flight validation](../../sdk/missile-flight-validation.md).

## Global Constraints

- Main only; baseline `57bc4c78c58f52b952126371319db9879131ff8a`.
- Preserve public layouts, signatures, native release authority and unselected generated outputs.
- Never traverse excluded directories or other worktrees. All tools/temp data stay under D:/ENV.
- Record actual nonzero checks and raw results. Missing evidence never counts as passed physics.

## Review Focus

- Colliding numeric IDs across Worlds must not dispatch a release.
- Finite inputs whose conversion/aggregation overflows must fail before XML output.
- An incomplete or nonfinite time series must not return a passing comparison.
- A stale generated model or different loaded native DLL must fail provenance checks.
- Common assumed inputs must not be reported as independent real-equipment calibration.

### Task 1: Release ownership prerequisite

- [x] Add C++ and Python foreign-World regressions; observe failed rejection and changed native state.
- [x] Validate native identity before dispatch, preserving wrappers and numeric IDs.
- [x] Run related native/Python checks serially: 3/3 passed.

### Task 2: Derived motor input validity

- [x] Add overflow, residual mass/inertia and grain installation regressions; observe red failures.
- [x] Reject invalid derived values without changing valid generator arithmetic.
- [x] Run focused input and byte-for-byte model checks: root final selection passed 26/26, including pulse integration, serialized-grid rejection and explicit Hangar burn ignition.

### Task 3: Independent powered-flight comparison

Files: new `tools/validation/missile/{reference,crosscheck,test_reference}.py`.

- [x] Add failing impulse-conservation and complete finite time-series checks.
- [x] Implement `phase_curve(phases, buildup_s=0.05, spacing_s=0.001)` and `compare(native, reference, limits)`.
- [x] Run helper tests, then the actual native/RocketPy comparison at both declared timesteps. The matched spherical campaign passed 48/48 cases over 24 authored AAMs at 480/960 Hz.
- [x] Verify model bytes, runtime library provenance, phase boundaries, burnout and raw trajectories. Independent review confirmed all 122 unique recorded input/model/DLL hashes and listed numerical-source hashes.
- [x] Reproduce the consumed-mass pulse-skipping defect before correction; add a versioned ignition-time table without adding public layouts or engine members. Generate only the three selected motor files through Hangar.
- [x] Reproduce unlit weapons in existing end-to-end engagements and public armed/inert release/reset; preserve weapon launch/requested reset inputs, leaving aircraft authority/control unchanged. Existing engagement acceptance passed; actual Hangar burn checks passed for AIM-120C and PL-15.

### Task 4: Aerodynamic solver/reference evidence

- [x] Acquire pinned supported binaries and archive/executable/license records.
- [x] Run bounded upstream reference cases: SU2 matched its published hybrid regression; VSPAERO completed its packaged example without a new experimental residual claim.
- [x] Execute authored-geometry SU2/VSPAERO static trials and coefficient/sign/reference checks. Actual runs failed physical/convergence gates and remain rejected.
- [x] Diagnose singular nose and mesh asymmetry; integrate guarded point/conical/full-reflection meshes and source-matched VSP point noses. Independent review's folded-neighbour finding is fixed. Combined safeguards passed 44/44; actual CLI retains failed model acceptance.
- [x] Require the full final nonlinear VSP history against matching polar conditions/loads; passing GMRES alone cannot pass a comparison.
- [ ] Establish credible same-design geometry/mesh sensitivity and independent coefficient agreement. Ill-shaped CFD cells, failed symmetry/loads and sampled VSP loft differences remain open.
- [ ] Admit only computed channels and supported operating conditions into generated tables.

### Task 5: Integration and review

- [x] Register maintained reference safeguard tests and document reproducible entrypoints.
- [x] Run related generator, native ownership/propulsion, Python and compatibility gates: final headless 22/22 including installed equipment consumer, Release 24/24, reset/authority 2/2, engagement/release 3/3 and generator selection 26/26.
- [x] Independently review the complete diff and correct substantive findings with regressions, including failed-report handling, Windows reruns and shared zero-alpha pitch-moment bias.
- [x] Reconcile exact achieved and pending SIM-M02 acceptance criteria and fleet scope in the validation record. Real equipment admission and calibrated model acceptance remain open; no failed solver output is imported.
