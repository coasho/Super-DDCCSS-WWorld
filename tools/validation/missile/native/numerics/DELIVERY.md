# Completion revision: three original numerical issues (reference_body), consolidated delivery

**Package status: INCOMPLETE.** Rows D and E pass. Row C fails exactly one criterion: the physical-validity gate
"nonnegative wind drag" on the coarse comparator at alpha 0, with native wind CD = −0.01322806303. Every other row-C
criterion passes at all four grid/angle points. Completion is not claimed. Independent review is performed by Codex.

The prior completion delivery (`deliveries/completion`, `work/completion`) is unchanged and was re-verified. This
revision lives in `work/completion-revision` and `deliveries/completion-revision`.

## Acceptance matrix

| Problem | Status | Passed | Unmet |
|---|---|---|---|
| **C. Mesh sensitivity** (SU2 8.5.0; archived coarse comparator vs Task 01 cylinder-local grid; alpha 0/2) | **FAIL (one criterion)** | All four points: native and accepted-parser convergence, with rms ≤ −9 and every Cauchy < 1e-10 under the original thresholds. The 1e-8 envelope holds for CA, CN, CM and CD; **local alpha 2 CMy is now 4.22e-9**, previously 2.55e-8. Finite positive state, native vs reconstructed loads with closure, hash-bound lineage, and native/parser predicates reported separately. Conditions and zero-alpha symmetry pass on both grids, and nonnegative drag passes on the local grid. Coarse-vs-local totals, increments, groups and per-CAD contributions are given, with an evidence-based drag account | **Item 7, coarse grid, alpha 0: nonnegative wind drag.** CD = −0.01322806303 |
| **D. Panel closure** | **PASS** (carried forward, hashes verified) | Original defect reproduced (128/192, 64 failures); corrected checks 192 + 88 + 56 = 336; CTest 3/3 with nonzero counts; `--no-tests=error`; both saved-geometry fixtures pass without repair or filtering | none |
| **E. VSPAERO outer free-wake convergence** (released alpha 0/2, N=8, 80 iterations) | **PASS** | Corrected native runs converge under the original outer criterion: MAX log10 −3.051 (alpha 0, iteration 6) and −3.019 (alpha 2, iteration 16). Both printed GMRES targets are met on every wake iteration; history/polar agreement holds; ADB surface/TE/wake checks and physical gates pass. **The new wake/body containment check passes**, with no active wake point or segment inside the body or within coordinate precision of it. The package includes a causal explanation, regressions, and the containment tests and regression | none |

## Constraints honoured

- **Workspace.** Only the LAB was used. No other project or worktree was opened, and no `.codex` directory was read,
  searched, enumerated or modified (the integrity walks prune that name).
- **Locations.** Candidate code, tests, configurations and runs are under `work/completion-revision`. External
  candidate sources and builds are only below `D:/ENV/physics/aero-numerics-lab/vspaero-completion`, which the
  revision only added to; its completion files are unchanged (`integrity/after/`).
- **Tools.** Pinned tools and read-only sources were used unchanged.
- **Environment.** TMP, TEMP and TORCH_EXTENSIONS_DIR were `D:/ENV/tmp` for every native step.
- **Native slot.** One serial native slot. VSPAERO builds and runs came first, then the single SU2 launch (the only
  background entry). There were no polling jobs, and raw output went straight to files with `.exit` records.
- **Fixed inputs.** The original geometry, physical conditions and acceptance thresholds are preserved. VSPAERO used
  only the released N8/80 inputs. In SU2, the only configuration changes are the allowlisted `ITER` budget and the
  native stop `CONV_CAUCHY_EPS` 1E-10 → 1E-11, derived below; acceptance thresholds are untouched.
- **Nothing hidden.** No clamping, omitted cases or frozen-wake substitutes. The prior interior-sample candidate V5
  is withdrawn.
- **Budgets.** Every series had a budget record written before it ran: `vspaero/BUDGET-RECORD-SERIES-R1.md`, `-R2`,
  `-RG` and `su2/BUDGET-RECORD.md`.
- **Prior evidence.** Every prior file is unchanged: the protected snapshot taken before any revision work is
  identical afterwards, and every earlier SHA256SUMS verifies (`integrity/after/`).

## Row E. VSPAERO outer free-wake convergence (released alpha 0/2, N8/80)

### The missing containment check (fixed first)

`aero_vsp_adb.assess_adb` proves only that the ADB wake equals the solver's wake. `vspaero/tools/
wake_containment.py` adds what it lacked. It reads the run's own ADB, VSPGEOM and VKEY.

- **Body.** The closed thick body is the set of alternate triangles of the thick parts, at the exact coordinates the
  solver parsed.
- **Active versus disabled trails.** It uses the source contract of `convergence/strength-summary.json`:
  - strip value: the signed binary64 solve-level Gamma0 of the thin-sheet loops owning each TE edge;
  - trail strength: left strip minus right strip;
  - **active**: not concave (concave trails induce no velocity in the native edge code) and nonzero strength;
  - disabled or dummy trails are listed but never judged.
- **Coordinate precision.** A point counts as inside or outside only when its exact distance to the body exceeds its
  binary32 rounding radius plus the largest binary32 radius of a body vertex. Otherwise it is uncertain, and
  uncertain also fails.
- **Segments.** Every active segment is tested at 65 points and between its body crossings.
- **Integrity.** Every internal binary64 wake point must round to its packed binary32 copy, or the input is refused.

**Tests.** `vspaero/tests/test_wake_containment.py` ran RED against the coordinate-matching stand-in (6 of 7 fail)
and GREEN 7/7 (`logs/vspaero/tests/`). The failures it rejects are:

- an active interior node;
- an active node on the surface within precision;
- a segment through the body between two outside nodes;
- an inconsistent binary64/binary32 point.

It accepts a clear wake while disabled trails stay inside, and it treats a zero-strength trail as disabled. It
reproduces exactly the LAB owner's three definite interior points (trails 14/20/21 at alpha 0) and every trail
strength magnitude.

**Every prior run fails containment** (`vspaero/containment/prior-*.json`):

| Run | Active interior points (alpha 0 / 2) | Active segments not outside (alpha 0 / 2) |
|---|---|---|
| archived, V0 rebuild | 3 / 0 | 9 / 2 |
| C1 stock core | 8 / 12 | 16 / 18 |
| C2 freeze root (diagnostic) | 5 / 3 | 9 / 8 |
| V4 | 3 / 7 | 9 / 11 |
| **V5** | **14 / 13** | **21 / 20** |

V5 is therefore withdrawn as a correction, as the review found.

### Numerical cause

The completion witness evidence (`deliveries/completion`, V1–V3) is unchanged:

- the linear solve is consistent, with a reconstructed post-solve residual of at most 1.1e-3;
- the outer residual is a limit cycle of the explicit wake update;
- the root-adjacent free trails sample the singular near field of the zero-core surface vortex lattice, inside and
  just outside the body, and those directions turn their segments through the body.

VSPAERO 7.2.2 already treats this near field for off-body points. `VSP_SOLVER::
CalculateSurfaceInducedVelocityAtOffBodyPoint` (used for survey points) replaces the raw lattice sum within one
fine-edge length of a fine surface-edge centre by the 1/distance-weighted mean of the adjacent surface-loop
velocities. **The explicit wake update never applies it:** `CalculateEdgeVelocities` gives every wake sample the raw
sum. That omission is the numerical cause corrected here.

### Correction (insert-only in `VSP_Solver.C`, before the explicit steady wake update)

| Series | Change | alpha 0 | alpha 2 | Containment |
|---|---|---|---|---|
| R1 | the native off-body rule applied, unchanged, to every solve-level wake sample | converged, iteration 6, −3.133 | **−1.199**, period-2 cycle | pass / pass |
| **R2** | the same rule with continuous weights: phi = 1 − d/L per fine surface edge (zero at the native cutoff), surface direction weighted by phi/d, blend beta = 1 − Π(1 − phi); **no new parameter** | **converged, iteration 6, −3.051** | **converged, iteration 16, −3.019** | **pass / pass** |

- **R1 failure.** At alpha 2, trail TE 855, node 3, alternated between two positions 1.2 mm apart, holding the
  surface residual of loop 864 at 0.0632. The native rule is discontinuous: an edge enters with full weight at its
  cutoff. R2 removes exactly that discontinuity.
- **Unchanged.** Surface solve, residuals, forces (surface edges only), kernels, geometry, settings, N8/80, the
  GMRES factors and the native −3 criterion. This is not stock VSPAERO.
- **Lineage.** Source diffs are in `source-diffs/vspaero/` (with the OpenVSP licence) and build records in
  `vspaero/builds/`. The R2 executable is `bf462933…`: the generated `VSP_Solver.C` `1bf4afa5…` linked with the three
  unchanged reference units and the pinned stock objects.

### R2 acceptance (both angles, original thresholds; `vspaero/r2/`)

| Check | alpha 0 | alpha 2 |
|---|---|---|
| native outer criterion MAX log10 ≤ −3 | −3.0514 (iteration 6) | −3.0190 (iteration 16) |
| L2 log10 | −3.986 | −4.170 |
| both printed GMRES targets on every wake iteration | pass | pass |
| history/polar agreement, ADB surface/TE/wake checks | pass | pass |
| containment: active interior / uncertain points / segments | 0 / 0 / 0 | 0 / 0 / 0 |
| smallest active clearance | 5.1 mm (a tail-fin TE root) | 5.1 mm |
| wake-update samples inside the body after iteration 2 (`trail-sampling-*.json`) | 0 | 0 |

The pair also passes the physical gates: paired conditions, nonnegative wind drag, and zero-alpha CN and CM symmetry.

**Loads (R2; archived unconverged run in brackets).**

- **Alpha 0:** CA 0.012169 (0.012166), CN 0.00477 (0.00480), CM 0.00878 (0.00823).
- **Alpha 2:** CN 0.39046 (0.38742), CM 0.23461 (0.22527), wind CD 0.017702 (0.018997).

These are converged loads of a modified VSPAERO model. Native residual convergence does not by itself establish load
accuracy.

### Regression (series RG)

- **RG1.** Reruns of the R2 executable equal the R2 runs under `compare_runs.py`, with only wall-time lines
  differing.
- **RG2.** `vspaero/tools/row_e_regression.py` exits 0 when given the reruns as candidate and the uncorrected V0
  rebuild as control. It applies the accepted readers and containment to both pairs. The candidate passes everything
  at both angles, and the control fails at each angle (−0.422/+1.230, containment fails).
- **RG3.** The same regression with V5 as candidate exits 1.

## Row C. Mesh sensitivity (SU2 8.5.0, Euler, Mach 0.6, alpha 0/2)

### The moment-settling failure and its correction

- **Why ITER could not help.** The completion run at the local grid, alpha 2 stopped natively before CMy had
  settled. SU2's Cauchy value is the mean over 100 iterations of |dC|/max(|C|, 0.1) (`COutput.cpp:954-965`). For
  |CMy| = 1.55, the native stop at 1e-10 allows about 1.5e-8 of drift across the window, which exceeds the absolute
  1e-8 envelope.
- **Derived limit.** `su2/tools/derive_cauchy_limit.py` (`su2/cauchy-limit-derivation.json`) evaluates the bound
  eps ≤ (1e-8 − 2q)/((1 + 2g)·100·M) per functional and point on the completion histories. The smallest bound is
  1.63e-11: **1E-10 is not admissible, and 1E-11 is the largest admissible power of ten.** It applies identically to
  both grids and angles. The acceptance thresholds are not changed.
- **Protocol.** The frozen manifest is `316c50f5…`. The candidate is `work/completion-revision/su2`; tests ran RED
  with 7 failures against the completion candidate and GREEN 33/33 (`logs/su2/`).
- **Run.** One launch: supervisor exit 0 after 7323 s, one uninterrupted cold start per grid and angle, `-t 1`.

### Results (`su2/analysis/mesh-sensitivity.json`)

| Grid | Alpha | Native stop (rows / rms ρ) | Max Cauchy | Solver s | Accepted parser | 1e-8 envelope (largest) | State | Load closure | Wind CD |
|---|---|---|---|---|---|---|---|---|---|
| coarse | 0 | 1765 / −10.348 | 9.8e-12 | 244 | converged | pass (CM 9.9e-11) | pass | pass | **−0.01322806** |
| coarse | 2 | 1470 / −9.665 | 9.8e-12 | 205 | converged | pass (CN 2.0e-9) | pass | pass | 0.02165442 |
| local | 0 | 9116 / −10.662 | 9.7e-12 | 3741 | converged | pass (CM 1.6e-10) | pass | pass | 0.01877669 |
| local | 2 | 7991 / −10.280 | 1.0e-11 | 3130 | converged | **pass (CM 4.22e-9)** | pass | pass | 0.05045088 |

- **Predicates.** Native and accepted-parser predicates are recorded separately per point. Every point also meets
  the original Cauchy < 1e-10 threshold. Native and reconstructed loads agree within their print-quantum budgets,
  with per-CAD and group closure.
- **Determinism.** Each revision history equals the completion history of the same grid and angle row for row up to
  the completion stop: 1379, 1174, 6724 and 5597 identical rows. The local histories also repeat the Task 05 cold
  starts for 1600 rows. Only the stop moved.
- **Physical gates (accepted `aero.physical_gates`).**
  - Local grid: all pass.
  - Coarse grid: conditions and zero-alpha CN/CM symmetry pass; **nonnegative wind drag fails at alpha 0.**

### Coarse versus local (local minus coarse)

| Quantity | Coarse | Local | Change |
|---|---|---|---|
| alpha 0 CA (wind CD) | −0.013228 | +0.018777 | +0.032005 |
| alpha 2 CN | 1.055955 | 0.989966 | −0.06599 |
| alpha 2 CM | −0.723060 | −1.545814 | −0.82275 |
| increment CN (alpha 2 − alpha 0) | 1.055955 | 0.989966 | −0.06599 |
| increment CM | −0.723060 | −1.545814 | −0.82275 |
| increment CA | −0.001979 | −0.002865 | −0.000886 |

- **Group CM increment changes:** body −0.5114, fore fins +0.0013, rear fins −0.3126.
- **Group alpha-0 CA changes:** body +0.0091, fore fins +0.0071, rear fins +0.0158.
- **Per-CAD contributions** are in the JSON. The analysis inherits the completion endpoint names, so
  "completion-coarse/local" there means the revision runs.
- Two grids are not a convergence sequence, so no GCI and no spatial-convergence claim is made. The local grid
  refines more than the cylinder, so no cylinder-only causality is claimed.

### The negative coarse alpha-0 drag: exact failure and evidence

**Failed criterion.** Accepted physical gate `nonnegative_wind_drag` on the coarse comparator: alpha 0, native wind
CD = −0.01322806303. Coarse alpha 2 passes (+0.021654).

**It is not an iterative, stopping or reconstruction artefact.** The value matches the archived coarse run and the
completion rerun within 1.4e-10, at rms −9.45 and −10.35. Native and reconstructed loads agree, and the coarse wall is
closed and manifold (Σ n·A ≈ 1e-15 m², every edge used twice).

**It is a sign that the coarse discretization cannot resolve.**

- **Exact drag is zero.** The exact inviscid drag of this closed body is zero, so a valid Euler result is zero plus
  a small positive numerical loss. The coarse total is instead the difference of +0.149 of drag (blunt base +0.102,
  nose +0.047) and −0.162 of thrust: eight fins −0.152, boattail −0.010. That is a cancellation ratio of 0.08
  (`drag_account`).
- **Where the fin thrust comes from** (`su2/analysis/fin-drag-location.json`).
  - The fins' 62.6°-swept leading edges reach only Cp 0.20, the swept attachment-line value (cos²Λ = 0.21).
  - Their unswept trailing edges recover to Cp 0.82.
  - The aft 20 % of chord carries −0.102 of the −0.152 fin thrust.
- **What local refinement does.** On the local grid the trailing-edge recovery is Cp 0.69 and the aft-20 % term is
  −0.078. The total moves by +0.032, larger than the coarse total itself. The four archived grids give −0.013,
  −0.037, −0.014 and −0.011: nonmonotone, and as large as the change.

**What this revision did not do.** It did not change the coarse grid, the discretization or the threshold to force a
sign. Those are fixed by the assignment, and the archived coarse comparator is the declared case. A grid that
resolves the fins' trailing-edge pressure recovery against their leading edges would be needed for this gate. This
statement records evidence; it does not claim that no in-scope correction exists.

## Row D. Panel closure (carried forward)

The revision changes no panel-closure input, tool, OpenVSP source or build. `work/completion-revision/panel_closure/
carry_forward.py` (`panel_closure/carry-forward.json`, exit 0) re-verifies:

- the 58 completion files of row D against `deliveries/completion/SHA256SUMS`: the assessment tool, tests,
  results, native step drivers, CTest logs and summary, OpenVSP source diffs and the variant CMake;
- all 9204 files of the external candidate OpenVSP tree (source and build) against their completion listing, byte
  for byte.

The accepted evidence therefore stands as delivered in `deliveries/completion` (README row D):

- the original defect is reproduced: boundary 128/192, 64 failures;
- the corrected checks give 192 + 88 + 56 = 336, CTest 3/3 with nonzero counts, and `--no-tests=error` rejects an
  empty selection;
- both saved-geometry fixtures (1159/1040/2267 and 1163/1040/2279) pass every gate without repair or filtering, and
  their uncorrected versions reproduce the defect.

Exact saved-source identity does not certify continuous CAD Boolean exterior coverage.

## Separation of evidence

| Class | Established here |
|---|---|
| Iterative | SU2: four native stops at 1E-11, rms ≤ −9.66, every 1e-8 envelope passes. VSPAERO R2: native outer −3 met at both angles; inner GMRES targets met every wake iteration |
| Spatial | SU2 coarse vs cylinder-local only: two grids, no GCI; the coarse alpha-0 drag sign is not resolved by the coarse grid |
| Geometric | Panel closure: saved discrete geometry passes, native guards 336/336, defect reproduced. VSPAERO: the active wake is outside the body beyond coordinate precision |
| Physical | SU2 local grid passes every physical gate, while the coarse grid fails nonnegative drag at alpha 0. VSPAERO R2 passes the physical gates on converged solutions of a modified, not stock, model |

## Package layout

| Path | Contents |
|---|---|
| `vspaero/` | budget records R1, R2, RG; `containment/` (prior runs); `r1/`, `r2/`, `rg/` (assessments, containment, sampling, regression); `run-summary.{md,json}`; `builds/` |
| `su2/` | budget record, Cauchy-limit derivation, `analysis/mesh-sensitivity.json`, `analysis/fin-drag-location.json` |
| `panel_closure/` | `carry-forward.json` |
| `source-diffs/vspaero/` | R1 and R2 patches against the reference `VSP_Solver.C`, OpenVSP licence |
| `logs/` | raw output and `.exit` of every step; index in `commands.md` |
| `integrity/` | `protected-before.sha256` (before any revision work) and `after/` (protected diff, earlier deliveries, external snapshots, candidate trees, pinned tools) |
| `commands.md`, `commands-verbatim.md` | curated commands with the exit index, and the machine-extracted shell record |
| `SHA256SUMS` | every file of `work/completion-revision` and `deliveries/completion-revision` except itself and its own check log |

The SU2 source changes (diffs in `source-diffs/su2/`) are in `work/completion-revision/su2/su2_configuration.py` (the `CONV_CAUCHY_EPS` allowlist),
together with the path and schema retargeting of `experiment_supervisor.py`, the tests and the analysis. Their
completion originals are in `work/completion/su2`. Files copied from `work/completion` are listed with both hashes in
`logs/copied-from-completion.txt`. The revision derivatives are the integrity script, log index, run summary and
analysis.
