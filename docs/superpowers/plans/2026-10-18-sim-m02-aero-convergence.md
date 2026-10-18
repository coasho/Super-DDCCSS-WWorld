# SIM-M02 Aerodynamic Convergence Continuation

Baseline: main `57bc4c78c58f52b952126371319db9879131ff8a`.
The preceding SIM-M02 changes remain in the working tree. No SIM-M01 changes
are included in this continuation.

Delivery handoff, 2026-10-18: the user requested commit/push of current results,
then an isolated numerical research directory for Claude to address only mesh
sensitivity, panel closure and solver convergence. Codex changes to independent
acceptance. The implementation/build window below is closed; unexecuted
proposals remain unexecuted, and no additional mesh or flow follows from this
historical plan. Research proceeds under its own numerical-only assignment.

## Write and execution boundaries

The aerodynamic implementation window owns
`tools/validation/missile/aero*.py`, `tools/validation/missile/test_aero*.py`,
this plan, `docs/sdk/missile-flight-validation.md`, and the SIM-M02 status and
allocation entries. Numerical case files are confined to
`build/architecture-reviews/sim-m02/aero/`. Production coefficients and generated
aircraft assets require separate physical admission; failed numerical output
must not be imported. Public SDK interfaces and native simulation sources are
outside this continuation's write window.

Independent offline cases may execute concurrently with bounded CPU/process
limits. The root retains the serial SDK configure/build/test slot; no SDK build
is required to diagnose the offline solvers.

## Required sequence

1. Separate SU2 iterative load stability from spatial discretization. Preserve
   the original density-only stopping results. Validate complete ordered finite
   histories, then require stable force and moment windows before comparing
   grids. Establish a controlled three-level refinement study where possible.
2. Diagnose VSPAERO panel edge connectivity before solving. Require actual
   closed oriented solid panels, valid lifting wake connectivity, and recorded
   geometry fidelity. A rigid coordinate transformation must also transform
   freestream, forces and moments consistently.
3. Require the solver's outer nonlinear criterion as well as inner linear
   convergence. Resolve the Kutta-Joukowski versus pressure-area discrepancy
   through pinned source and canonical checks before admitting an observable.
4. Repeat actual source-geometry cases, retain raw inputs, histories and hashes,
   and independently assess static agreement and refinement uncertainty.
5. Run maintained safeguard tests and document exact achieved and pending
   acceptance. A closed mesh or converged residual alone is not physical model
   calibration.

The numerical method references are the official
[SU2 convergence documentation](https://su2code.github.io/docs_v7/Solver-Setup/)
and NASA's
[spatial convergence procedure](https://www.grc.nasa.gov/www/wind/valid/tutorial/spatconv.html).
Refinement results require iteratively converged solutions; a small density
residual at a changing load does not establish spatial convergence.

## Native panel geometry correction window

Main remains `57bc4c78c58f52b952126371319db9879131ff8a`. The locally built,
unmodified OpenVSP 3.55 source remains immutable. A separate offline candidate
may change only `VSP_GEOM::{UpdateMeshes(int),DeflectControlSurfaces(),UnDeflectControlSurfaces()}`
in `src/vsp_aero/Solver/VSP_Geom.C` under `D:/ENV/physics/`, mapping trailing-edge
and wake-node IDs from the solve grid to the finest grid before dereferencing
them. The zero-control deflection path is still called by the native solve and
must preserve the same mapping when copying internal wake coordinates. The source archive
and all other numerical files remain unchanged.

The output-only probe already reproduces a physical surface mutation when
unused-node compaction makes those mappings nonidentity. A native public-library
regression must fail before correction and retain identity-mapping controls.
After correction, require immutable surface coordinates, correct wake-root
attachment, canonical controls and fresh same-geometry missile flows. Any
remaining nonlinear, physical or spatial gate stays mandatory. Geometry safety
does not admit coefficients or the separate experimental core law.

Probe, regression, patch and raw case evidence belong under
`build/architecture-reviews/sim-m02/aero/{source-build,vsp-closure,load-audit}/`.
The root alone compiles native diagnostic/candidate objects; no SDK build inputs
or production solver defaults change in this window.

The maintained regression delivery owns only
`tools/validation/missile/aero_vsp_native.py`, its `test_aero_vsp_native.py`, and
`tools/validation/missile/native/{vsp_wake_attachment.cpp,openvsp-3.55.0-wake-root.patch,LICENSE.OpenVSP,README.md}`.
It requires explicit external compiler/source/library paths and a fresh bounded
output directory. Ordinary safeguard suites do not acquire or compile OpenVSP.
The root retains shared test registration and project documentation ownership.

## Actual solved-geometry gate and integration window

Main HEAD is `57bc4c78c58f52b952126371319db9879131ff8a`. The exclusive parser
window owns `tools/validation/missile/aero_vsp_adb.py` and
`tools/validation/missile/test_aero_vsp_adb.py`; the sequential integration
window owns only `tools/validation/missile/aero.py` and
`tools/validation/missile/test_aero.py`. The root retains all other maintained
files and documentation sections. This section is the delegated integration
record; none of these four paths has a concurrent writer.

The normal worker, including zero frame roll, uses two explicitly paired
single-case native runs so the supported single-case ADB layout can be checked
for every result. The worker and parent must freshly assess current raw
polar/history/log/config/ADB files, including actual source-matched physical
geometry, authored TE associations and internal wake attachment. A saved JSON
summary cannot replace that assessment. The actual-geometry gate is mandatory
alongside existing physical, inner, outer and static-agreement gates.

This offline integration changes no production parameters, public SDK/native
interfaces or solver/wake flags. Historical native campaigns remain labeled by
the code and binaries that actually produced them; the new parser's read-only
checks do not claim that archived runs used the integrated worker.

## Initial generated-model comparison

At the same main HEAD, this window also owns
`tools/validation/missile/{aero_hangar.py,test_aero_hangar.py}` and its additive
`--initial-model` integration in `aero.py`/`test_aero.py`, after the preceding
shared-file integration is released. The optional input is an explicit generated
XML path; no variant or asset is selected by a filesystem search.

Evaluate only the bounded supported static expression grammar. Match emitted
area/chord/AERORP within generator precision, normalize to solver references and
transport pitch moment to the exact requested origin. Preserve generated-origin
values for native function readback. Require explicit conditions, finite
arithmetic and supported UTF-8 XML; reject declaration/entity expansion.
Report each solver's diagnostic differences separately. Empirical axial drag
and crossflow remain part of this initial full-static basis, so these differences
cannot replace inviscid solver agreement or supply missing physical channels.
An explicitly requested unavailable comparison fails visibly. All admission
flags remain false.

## Frozen numerical follow-up controls

Main remains `57bc4c78c58f52b952126371319db9879131ff8a`. These offline controls
own only their recorded case folders, temporary diagnostic source copies and
root-built candidate objects under `D:/ENV/physics`. Maintained SDK simulation,
public interfaces, generated coefficients and the immutable source-built
baseline solvers remain outside this window.

The SU2 Wang spatial family freezes geometry, operator and actual mesh hashes
before flow, using global factors 1, 0.75 and 0.5625. The measured effective
mesh ratios, not the nominal factors, govern any later uncertainty assessment.
Each new medium/fine pass has 600 iterations, at most eight normal passes per
case, two threads and a 300-second process limit. Only a valid normal
NotConverged result may continue its own checkpoint. Native density -9,
all three Cauchy criteria, both load windows and physical gates remain intact.

The independent smooth-sphere geometry/control is frozen in
`aero/load-audit/su2-entropy/SPHERE-PROPOSAL.md`. Its three mesh preflights retain
exact CAD, globally scaled sizing, closure, geometry error and hash evidence.
After reviewing actual counts, the coarse Mach-0.2 flow alone is authorized:
1,000 iterations per normal pass, at most three passes, two threads and a
180-second limit per pass. Keep the same declared Roe/Wang/WLS/MG3/CFL1
operator and strict convergence checks. Timeout, failed process or malformed
state is terminal. Medium/fine flow requires review of the coarse result.
The incompressible sphere Cp expression is a low-Mach diagnostic limit,
not an invented exact compressible acceptance band.

VSPAERO output-only residual and stored-velocity reporters must reproduce the
existing canonical and missile histories/loads before interpretation. Actual
ADB geometry and literal wake/closed-body contact checks remain separate.
The native implicit-wake control has one unchanged canonical replay and one
120-second N8/two-degree missile attempt; its timeout cannot be continued or
relabeled as convergence. No unreviewed clipping, projection, relaxation or
acceptance-threshold change is authorized by these diagnostic controls.

## Primary panel-domain stage witness

At the same main HEAD, the independent native triangulation audit establishes
that the three LE-cluster-0.5 buried regions already belong to the exported
primary NGon domains. Alternate constrained Delaunay triangles exactly cover
those domains. The source-stage witness therefore precedes any clipping fix.

The root owns the exclusive native build slot and the fresh external diagnostic
tree `D:/ENV/physics/openvsp-3.55.0-geometry-stage-build/`. Upstream README supports
headless geometry core/API builds and Windows Visual Studio; use the existing
MSVC 14.44 toolchain and exact bundled dependency archives. Dependency and
geometry builds have separate frozen commands, hashes, raw logs and bounded
two-thread execution. The original source, standalone solvers and released API
remain unchanged. No dependency-version substitution is part of this window.

The optional output-only hook owns only an isolated copy of
`src/geom_core/MeshGeom.cpp`: capture SI original/split triangles and native
inside/ignore flags at the end of `IntersectTrim`, before flattening, then the
flattened input and PGMesh cleanup stages in `CreateNGonMeshGeom`. Preserve
native coordinates, UV, tags and identities; do not call mutating triangulation
or point-renumbering helpers to produce diagnostics. Require an unchanged
baseline replay and exact failed-case geometry provenance before interpreting
the earliest invalid domain. Native source repair requires that runtime witness
and a meaningful reproducing regression. These diagnostics admit no loads.

The first twelve-stage witness has mixed nested coordinates: active triangle
nodes return to SI, but intersection-edge-only nodes retain native working
scale because `TMeshUtil::ApplyScale` does not visit those auxiliary endpoints.
Preserve raw records and qualify any displayed conversion. Active-leaf area
budgets do not use those auxiliary coordinates.

An additional output-only window owns isolated `src/geom_core/TMesh.cpp` and
`VspTriangleSplitAudit.h`, reusing the earlier stage header. Capture exact input
before `TTri::SplitTri` mutation, its prepared constraint graph, rotated and
normalized coordinates, the existing constraint-call return and connectivity.
Store that return without adding a native call or changing control flow.
Capture every intersecting parent into four bounded streams. Require dormant
and enabled exact raw-hash replays before deriving a regression input or
choosing an intersection/split correction. Original upstream sources outside
this diagnostic copy remain unchanged.

The exact working witness establishes complete supplied-segment coverage in
the prepared graph and returned split triangulation for all four bad parents.
The next native regression owns the isolated `src/vsp/CMakeLists.txt` additive
test target and offline audit fixture/data. It exercises
`tri_tri_intersection_test_3d` and `TTri::InTri` on twelve exact captured triangle
pairs plus simple controls, with eight order/axis/power-of-two scale variants
each (112 cases). Expected segments come from independent exact-rational
geometry of the six input doubles. Original-library failure is required before
any kernel/filter correction; preserve all native algorithms and geometry in
this regression window. This test adds no SDK target or dependency.

The expanded original-library control adds eighty point/edge, coplanar,
parallel, near-parallel and separated cases, for 192 total. The original
library passes 128 and fails the same 64 captured-pair variants. Independent
exact-rational review confirms every expected result. Separate reviewer
counterexamples exercise true point contact and a one-ulp positive gap; they
must be tested natively before accepting a revised kernel. The first proposed
rounded-cut kernel is rejected by these counterexamples, before application.

The next isolated correction window is limited to
`src/external/tritri/tri_tri_intersect.cpp` and its `CMakeLists.txt`, plus additive
private fixture targets in `src/vsp/CMakeLists.txt`. Preserve the first proposal
and original RED reports. A versioned candidate may use the already bundled
GTE arbitrary-precision rational headers for exact noncoplanar cut construction
and interval ordering, rounding only the accepted output endpoints. Review
upstream compiler/license support and the complete candidate before applying
it. Do not widen overlap tolerances, weaken `InTri`, modify the original source
tree or repair exported meshes. Require independent review, unchanged inputs,
all original/boundary/reviewer controls, and full saved-input geometry closure
before any downstream flow. This window changes no SDK build or model input.

For reproducibility, the maintained offline window additionally owns
`tools/validation/missile/native/openvsp-3.55.0-exact-intersection.patch`,
`OpenVspIntersectionFixture.cmake`, the three
`vsp_triangle_{intersection,boundary,contact}.cpp` fixture sources and their
three `_data.h` headers, and the existing native README/license notices.
The explicit nonfinite-input rejection additionally owns
`vsp_triangle_nonfinite.cpp` and `vsp_triangle_nonfinite_data.h`; its 54 invalid
cases and two finite controls remain separate from the 280 geometry controls.
The optional fixture module is included only in an explicit external OpenVSP
headless build; it must not enter the SDK's default targets or apply a patch.
Preserve the exact captured numerical cases and independent endpoint oracles.
Any source override is explicit and hashed, with original and corrected runs
retained separately. Native fixture success remains distinct from geometry,
flow convergence and equipment-model acceptance.

After both saved mixed-surface geometry audits pass, an additional geometry-only
control may reuse the unchanged original saved CAD with all nine components
thick (`GeomSet=SET_ALL`, `ThinGeomSet=SET_NONE`). Only these analysis-set values
and output destinations change; retain all source parameters, refinement and
clustering. Freeze dormant/enabled scripts and inputs, two threads and a
30-second process limit, then assess literal union closure and authored
surface/TE fidelity without repair. This checks the full authored foil
thickness as well as the mixed thin-fin approximation; no flow follows until
the appropriate geometry gate passes and a separate case freeze is reviewed.

## Conditional finer-grid conditioning campaign

The reviewed execution manifest is
`aero/load-audit/su2-entropy/cfl5-fine-fourth-family-v2/execution.json`, SHA-256
`e75a612747bb87df37c990c58efd8b933ce8eb633ae082daac6154bb57132aac`.
Its offline write allowlist is that case folder and its temporary runner/tests;
no maintained or native solver source changes. Main HEAD is unchanged.

First run one byte-identical 600-row auxiliary verification of the existing
fine-grid CFL-1 two-degree reference, retaining its original native-converged
but empirical-tail-failed result. Then cold-start both fine-grid CFL-5 angles:
at most eight normal 600-row passes each and one explicit 600-row tail
verification when eligible. Both references and both new endpoints must pass
native/window checks, the 1e-8 endpoint-tail screen and 2e-8 pair-difference
screen before either fourth-grid flow starts.

The reviewed 63,629-node fourth grid uses at most sixteen normal 300-row passes
per angle and one explicit 300-row tail verification. This pre-run partition
replaces the earlier unexecuted 600-row partition because its timing estimate
reached the fixed process cap; the normal-row budget remains 4,800. Native
cold-W/restart-V conditioning therefore changes after 300 rather than 600
rows, with the spatial operator intact. All runs retain two threads, 300-second
process limits, SI/DIM gas/BC/reference values and unchanged physical gates.
Only valid normal NotConverged states continue their own checkpoints. Timeout,
failed/nonfinite/nonpositive output or pressure mismatch stops dependent flow.
The earlier 37-config freeze remains immutable. Numerical equivalence and a
new mesh endpoint do not independently admit aerodynamic coefficients.

The first reference verification stopped normally after 201 rows, with strict
native/window convergence but a Cm tail envelope of 1.153e-8; the v2 campaign
therefore stopped with no fine/fourth case executed. A separate one-process
reference-only freeze, SHA-256
`78dca5e50f7757844454f7d0ba42730a14f30ffe35ad6617bccc7411ff02087a`,
uses that exact outgoing checkpoint and byte-identical original 600-row config,
with the same 300-second limit and unchanged criteria. It cannot launch any
dependent case or overwrite the blocked v2 aggregate. An eligible new reference
requires a separately reviewed aggregate continuation record for the still
unexecuted staged cases; this does not repeat their reference process or extend
their frozen per-case budgets.

That separate reference passed after 201 rows, with maximum tail envelope
8.11999984e-9. The reviewed continuation manifest is
`aero/load-audit/su2-entropy/cfl5-fine-fourth-continuation/execution.json`, SHA-256
`d4afeedee736cf11b92044d58fc714307997dc70a469090142ee361614455374`.
It consumes the eligible reference without repeating it, reuses the 52 exact
unexecuted staged config/mesh paths and writes its aggregate into a new folder.
The original blocked v2 result and every frozen input remain hash-protected.
Both grid budgets, the conditional fourth-grid gate and all criteria above
remain unchanged. Five aggregate-isolation/stop/budget regressions pass in an
independent root rerun; numerical solver acceptance still comes from actual
native processes rather than those orchestration guards.

The bounded continuation completed both fine and fourth-grid angles. All four
new endpoints pass the unchanged native/window and endpoint-tail gates, and
both fine CFL-5/CFL-1 comparisons pass the pair-difference gate. The fourth grid
needs 823/832 normal rows at zero/two degrees. Its zero-angle drag remains
negative and its two-degree loads still change materially from the fine grid.
These results close the specified conditioning campaign, not physical or
spatial-convergence acceptance. Further flow requires a new source-qualified
diagnostic freeze; do not extend this completed campaign.

## Supported upwind-operator diagnostic

The next offline SU2 control is one cold coarse-grid zero-degree case, frozen
in `aero/load-audit/su2-entropy/hllc-coarse-alpha0-v2/execution.json`, SHA-256
`716806ec2da0bbe7e3e93e41d5364c8917ac6259ca026b8b345196531f4aaaaf`.
It changes the Roe CFL-5 reference's flux selection to supported HLLC while
retaining the same mesh, SI/DIM gas, boundary data, references, MUSCL/Wang/WLS,
MG3, linear settings and strict criteria. HLLC selects a different native
scalar reconstruction/residual path as well as flux/Jacobian; interpret it as
an upwind-operator/path control, not pure Riemann-flux isolation or an assumed
entropy-stable cure. Source excerpts and exact file hashes qualify support.

The write allowlist is that case folder and its named offline runner/tests.
At most eight normal 600-row processes, each 180 seconds and two threads, may
continue only their own valid normal NotConverged checkpoint. There is no
verification or extension process. Stop on native/window convergence even if
the separate 1e-8 empirical tail remains unresolved; nonzero exit, timeout,
nonfinite/nonpositive output or pressure mismatch is terminal. Retain original
symmetry/drag gates and false model admission. Root's independent eight-test
mocked rerun passes; actual solver evidence remains separate. Review this exact
freeze before execution and retain its separate root approval record.

The reviewed control stopped normally after one 452-row process in 53.79
seconds. Native/window convergence passes, but the unchanged empirical tail
does not; zero verification or extra processes run. Negative drag worsens.
Retain the source-qualified operator sensitivity and independent read-only
pressure/thermodynamic follow-up. No continuation or HLLC mesh family follows
from this result.

## Corrected mixed-geometry wake control

The reviewed paired VSPAERO control is
`aero/load-audit/vsp-native-clipping-audit/kernel-v2/flow-control/proposal-v3.json`,
SHA-256 `4efc351892b58fba78957c83d258f39788c59d6572d9ba6452b89f34a4803ecb`.
It binds the corrected original-cluster geometry, unchanged VKEY and exact
prior joint zero/two-degree configurations to the previously equivalent
output-only velocity witness. Native settings remain N8/80 free wake, original
relaxation/core/GMRES/outer targets, two threads and 120 seconds per independent
case. No continuation or extension is allowed. Its write allowlist is the
versioned control folder only; geometry audits and all source inputs stay
hash-protected. Root's independent eight-test read-only rerun passes.

Inspect all packed solve-grid wake points, including concave trails whose
separate internal-only records omit points. Classify against the literal body
with explicit IEEE binary32 coordinate uncertainty; ambiguous boundary samples
are not proof of penetration. Segment inside lengths remain nonrigorous
diagnostics rather than certified lower bounds. Keep native, physical and ADB
failures explicit; no diagnostic result admits coefficients or full noncontact
between sampled points. Earlier unexecuted proposals remain immutable.

The first native zero-degree process completed normally, but its subsequent
localization helper rejected a missing convenience field in the process record.
Keep that failed controller report and every original byte intact. The separate
recovery freeze is `flow-control/recovery-v4/proposal-v4.json`, SHA-256
`d1f5f9d648c9e19ab84fb8d2affb59a2a2ba2cc4617413682956502dc0d6ae37`.
It derives compatibility metadata from equality of frozen-before, recorded-after
and actual current input hashes, then reproduces the existing zero-degree
assessment in a new folder without flow. A real-helper regression reproduces
the schema failure before correction; root's eight bridge/immutability tests
pass. Only the still-unexecuted two-degree case may run once, with the identical
three inputs, binary, settings and 120-second/two-thread cap. Repeating the
zero-degree native process or extending either case remains forbidden.

The recovery completed the missing two-degree process normally in 24.745 s;
the original zero-degree process took 26.480 s and was not rerun. Both actual
ADB geometry checks, all inner linear targets and paired physical checks pass.
Both outer targets fail, at -0.42156/+1.23022 against -3. All 768 packed points
and 160 MAX/L2 rows are assessed, with explicit boundary uncertainty. Preserve
the completed failed diagnostic as `recovery-v4/report-v4.json`, SHA-256
`992783879c859d435c144d12766e8ee778388d471e50ebcee5af7cec734f768a`.
No continuation follows from it. The next read-only audit reconstructs steady
trail strengths from actual surface circulation and signed TE edges, respecting
the native concave-edge induced-velocity exclusion, and localizes the remaining
body normal residual. It changes no flow source, settings or coefficients.

## Final intersection guard publication

At the same main HEAD, the external-only maintained fixture module additionally
owns `tools/validation/missile/native/vsp_triangle_nonfinite{.cpp,_data.h}`.
Its 54 coordinate-wise NaN/infinity rejection checks and two finite controls
retain the declared input policy independently of the 280 finite geometric
checks. The original kernel passes 2/56; the corrected kernel passes 56/56.
The final maintained external build passes 3/3 CTest cases and 336 native checks;
`aero/source-build/maintained-guard-final/freeze.json` has SHA-256
`a54df3e8306bf97698970cf30aa7c0230ee3bad86cb93e4eed0dd3610a252a63`.
Protected source and the main geometry executable remain unchanged. The SDK
architecture check passes with 653 classified files and no layout exceptions.
The original standalone controller's postcheck type error remains preserved;
a separate verified result checks its completed native outputs without rerun.
