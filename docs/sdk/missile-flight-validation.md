# Missile flight validation (SIM-M02)

## Continuation scope, 2026-10-17

Baseline: main `57bc4c78c58f52b952126371319db9879131ff8a`. The working tree was clean.
The user assigned the complete SIM-M02 continuation to Codex; SIM-M01 remains
Claude's. Work stays on main, without another checkout. This supersedes the
earlier first-slice stop point, but does not change the physical acceptance
criteria in [the simulation plan](../architecture/modules/simulation/plan.md#5-functional-roadmap-retained-after-structural-work)
or [MS-7c](../architecture/modules/simulation/adr-30-mission-systems.md).

The goal is reproducible aerodynamic and propulsion validation of generated
missile models used by the SDK. Existing import, metadata and reset work is a
prerequisite, not physical calibration. The first representative remains the
existing AIM-120C-7 game model. Its motor phases, specific impulse, internal mass
distribution and detailed geometry remain assumptions until independently
admitted evidence establishes them. Cross-checking the same assumed input in two
programs verifies their implementation agreement, not the real equipment.

### Write ownership

| Package | Exclusive paths and symbols |
| --- | --- |
| Release ownership prerequisite | `src/simulation/api/World.cpp::World::launchWeapon`, `python/fsim/world.py::World.launch_weapon`, related cases in `tests/compatibility/test_sdk.cpp` and `tests/compatibility/python/test_engagement.py` |
| Motor input prerequisite | `tools/models/hangar/hangar/motorinputs.py`, `storebodies.py::motor_of`, related cases in `tools/models/hangar/tests/test_missiles.py` |
| Independent flight reference | New `tools/validation/missile/` scripts, fixtures and tests; this document |
| Solver reference campaign | Temporary input/output and executable records under `build/architecture-reviews/sim-m02/`; tools and archives under `D:/ENV/physics/` |
| Pulse timing correction | `cmake/JsbsimPatches.cmake` generated `FGRocket::Calculate` coordinate selection; new synthetic `tests/simulation/data/rocket_pulse/` fixtures and case in `tests/simulation/test_vehicle_pool.cpp`; `storebodies.py` pulse table emission; generator-owned `aircraft/stores/{pl15,r37m,r77m}/Engines/*_motor.xml`; source-change notice in `THIRD_PARTY_NOTICES.md` |
| Released-motor ignition correction | `src/simulation/runtime/world/World.cpp::World::{preStep,resetVehicle}` weapon control-cascade eligibility and reset inputs; related public SDK armed/inert release regression in `tests/compatibility/test_sdk.cpp`; `tools/models/hangar/hangar/storestages.py::_burn` explicit actuator request and related isolated test in `tools/models/hangar/tests/test_missiles.py`; existing `engagement_native` acceptance. Added after that unchanged end-to-end test reproduced unlit released AIM-120C motors at the same baseline HEAD |
| Final integration | Related standard-library validation registrations in `cmake/ArchitectureChecks.cmake`, `tools/validation/testing/scopes.toml`, and links/status in `docs/{hangar,status}.md`, `docs/architecture/modules/simulation/ms-work-allocation.md` after reviewing the final implementation |

No radar source/card, guidance/warhead calibration or public ABI layout is in
this window. Only the root agent runs native SDK configure/build/test commands;
build inputs are frozen while those commands run. Offline reference solvers run
with bounded threads. Every command uses `D:/ENV/tmp` for temporary files.

On 2026-10-18 the user closed this implementation window for delivery and
assigned the three numerical investigations (mesh sensitivity, panel closure
and solver convergence) to Claude in a separate research directory. Codex
performs independent acceptance. That handoff contains only those numerical
problems and their required inputs; it authorizes no SDK or equipment-parameter
changes and does not close the remaining physical acceptance criteria.

### Execution and acceptance

1. Add failing ownership and motor-input regressions, then correct the native
   and Python release boundary and unsafe derived motor values. Preserve valid
   current generated outputs and local numeric launcher IDs.
2. Pin actual RocketPy, SU2 and VSPAERO versions, executable/archive hashes,
   licenses and support requirements; run upstream or independently defined
   canonical cases before treating tools as usable.
3. Implement a repeatable comparison of the generated native missile's powered
   flight and RocketPy with matched declared inputs. Record trajectory, thrust,
   mass and residuals with explicit tolerances. Invalid/missing dependencies,
   stale models and incomplete comparisons fail explicitly.
4. Admit geometry and solver inputs by exact variant and operating envelope;
   exercise mesh/step sensitivity and independent aerodynamic comparisons.
   A static force sweep cannot supply uncomputed damping/crossflow channels.
5. Integrate only validated datasets through the generator; run related native,
   Python, generator and installed-consumer checks, then independent review.
   Record every remaining equipment/fleet criterion as pending until achieved.

Reference reports must distinguish software agreement, numerical convergence,
source admission and real-equipment validation. No passing comparison is allowed
to upgrade any other category implicitly. The span discrepancy recorded in
allocation section 10.6 remains open until the exact source pages are reconciled.

The fleet comparison found the consumed-mass table could skip the complete 2 N
coast in three declared pulse motors: its mass interval is smaller than one
preceding high-thrust step. The correction adds the private versioned table name
`fsim/ignition-time-thrust-v1`, evaluated against the existing native engine's
ignition-relative burn time. Other table names retain consumed-mass semantics.
No new member, virtual method or public signature is added. The generator selects
this form only for declarations with a zero-thrust phase, retaining their current
2 N coast and assumed grain/impulse inputs. It emits an explicit zero after the
last pulse. These generated pulse tables require the matching patched native
runtime; the comparison and installed consumers verify that pairing. Reset,
delayed ignition and ignition persistence through coast are
native fixture requirements. This is a schedule correction, not motor calibration.
The clock retains upstream semantics under starvation: it advances only during
non-starved burning, so it is active burn time rather than unconditional wall
time. The marked fixture also checks fuel-unavailable shutdown and refilled reset.

## Results

Final software verification: 2026-10-18, on the same main baseline with the
feature changes below. Aerodynamic follow-up retains its separate failed gates.

SIM-M02 remains **open against its physical-model acceptance criteria**. The
delivered software comparison passes across all 24 authored AAM models; the
same-design aerodynamic experiment fails its physical gates. No external aero
dataset or real motor curve was admitted. The changes are based on the baseline
above and are recorded independently of the historical results in allocation
sections 10.1-10.6.

| Delivered behavior | Current evidence |
| --- | --- |
| Reject a launcher object from another native World before dispatch | Native and Python regressions first reproduced release/state changes; local numeric IDs and wrappers of the same native World still work |
| Reject unsafe derived motor inputs | Overflow, nonpositive remaining structure/inertia, grain installation and collapsed raw/serialized consumed-mass knots fail before XML generation; focused generator selection passed 26/26 in 0.582 s, including explicit Hangar burn ignition and refusal cleanup |
| Preserve pulse coast timing | The synthetic native regression first failed 145 assertions. The marked/unmarked table fixture now covers 60/120 Hz, delayed ignition, coast, terminal zero, starvation, reset and fresh replay |
| Ignite actual released weapons | The unchanged end-to-end engagement test exposed an aircraft neutral command overwriting launch throttle. The public armed/inert regression first failed 32 assertions; preserving weapon inputs and their requested reset state restores native ignition without load-time burning. The three related release/eligibility/engagement cases pass, including worker determinism, powered/coast and inert jettison |
| Generate the timing correction through Hangar | Only PL-15, R-37M and R-77M motor XML changed. A census of 53 generated store models (101 files) found no other generated differences |
| Compare powered flight independently | 48/48 vacuum/air cases passed, comprising 96 native runs and 16,592 common time samples across all 24 AAM models |
| Preserve public integration | Final headless selection passed 22/22 in 64.77 s, including the installed C++/C99/Python consumer (25.90 s). Separate final reset/authority checks passed 2/2 in 1.16 s. Installed model hashes and loaded-library provenance match |
| Verify the Release configuration | Final Release build and related selection passed 24/24 in 39.97 s, including the requested engine-state reversal on reset and the existing authority/task/C2 preservation case |

The generator selection includes current authored-design and generated-output
controls. The related CTest selection covers the three native rocket cases, both
new release-ownership cases, armed/inert release and reset, Python engagement/stores/weapons modules, C ABI and
SDK boundary checks, reference guards, architecture checks and contract/scope
checkers. These are current focused checks, not a full-suite or Viewer claim.

The fleet's worst residuals over both native timesteps are **0.052202 kg** mass,
**1.285336 m/s** axial speed and **4.157134 m** altitude; maximum vacuum-impulse
discrepancy is **0.184871 percent**. Every case also passed the declared timestep
sensitivity condition. These values describe the matched central-gravity
campaign below, under assumed game inputs. They do not establish real-equipment
accuracy, Earth-trajectory agreement or control-response fidelity.

Raw logs, full trajectories and explicitly listed source/library SHA-256 values
are retained under `build/architecture-reviews/sim-m02/`: `fleet-flight-final.json`,
`fleet-flight-final.log`, `generator-final.log`, `final-headless-tests.log`,
`released-ignition-green.log`, `reset-authority-final.log`, `burn-native.log`,
`release-final-build.log` and `release-final-tests.log`. The two actual Hangar
burn checks (AIM-120C and PL-15) also passed. Earlier diagnostic failures remain in separate reports;
they are not included in the passing fleet result. The report hashes actual
loaded native DLLs against the explicitly requested build and records the Python
interpreter, RocketPy/NumPy/SciPy versions, selected numerical sources, authored
design/card, production generated files and temporary instrumented files. This
is a declared provenance scope, not a complete dependency closure.

### Powered-flight reference conditions (frozen before comparison)

The initial diagnostic comparison is axial unguided flight: an already airborne, vertically
oriented body at 8,000 m and 250 m/s, equator, no wind, terrain or guidance.
Cases use vacuum and a spatially constant atmosphere sampled from the native
initial state. The native World keeps its own gravity and rotating ellipsoid;
RocketPy uses the initial local gravity with its flat local flight frame. These
approximations bound this short axial comparison; lateral/control responses are
not validated by it. Both engines use the same declared grain, vacuum impulse,
nozzle area, ignition buildup and axial drag table. RocketPy's grain burns
radially at fixed length, matching the generated JSBSim cylindrical grain.

RocketPy receives axial drag only; its optional fin/nose stability model is not
constructed in this reference. Its static-margin warnings remain in the raw log.
The comparison establishes no aerodynamic stability or lateral-response result.

At each complete common sample through burnout plus 0.25 s, absolute residual
limits are 0.10 kg mass, 2.0 m/s axial speed and 10.0 m altitude. Total vacuum
impulse must agree within 0.5 percent. Native timesteps are 1/240 and 1/480 s;
both must pass, and the finer run's maximum speed discrepancy may exceed the
coarse run's by at most 0.1 m/s. No channel may be empty, truncated, nonfinite,
misaligned or silently interpolated across missing data. These limits test the
implementation under declared game inputs; they do not admit a real motor.

### Matched central-gravity campaign (frozen before its first execution)

The broader diagnostic run exposed the first comparison's environment limits on
longer burns, separately from the pulse-schedule bug. A second explicitly selected
case, `--spherical-reference --native-hz 480 960`, matches the environment in both
programs: a nonrotating sphere of radius 6,378,137 m, central gravitational
parameter 398,600,441,800,000 m3/s2 and zero J2. The temporary native model adds a
standard JSBSim `planet` element; RocketPy uses `mu / (radius + altitude)^2`.
Production World/Earth configuration and production assets are unchanged. This
isolates the axial comparison from Earth rotation and mismatched gravity rather
than admitting a real trajectory. The same residual/impulse limits and step
sensitivity condition apply; no limit is widened. Both constant atmospheres and
the complete declared motor interval are retained. Exact instrumentation and
input hashes are included in the report. The earlier rotating-Earth results,
including their failures, remain separate evidence.

### Aerodynamic reference outcome

The pinned tools are SU2 8.5.0 (Windows OpenMP), OpenVSP 3.54.0 with bundled
VSPAERO 7.2.2 and its matching Python 3.13 binding, Gmsh 4.15.2, and RocketPy
1.13.0. SU2/OpenVSP archives, executable hashes, licenses, upstream inputs and raw
logs are listed in `solver-readiness/readiness.json` under the evidence
directory; `aero/evidence.json` records Gmsh's DLL and `fleet-flight-final.json`
records RocketPy's selected numerical sources and installed version.
SU2's upstream hybrid/OpenMP NACA0012 result matched its iteration-20 reference
to 0.000001, within the upstream 0.0001 tolerance. The packaged VSPAERO TR1208
ten-angle example completed successfully; this is an execution check without a
new experimental residual claim. Install locations and support requirements were
checked against the [SU2 release](https://github.com/su2code/SU2/releases/tag/v8.5.0),
[OpenVSP API documentation](https://openvsp.org/api_docs/3.54.0/index.html) and
[RocketPy motor API](https://docs.rocketpy.org/en/latest/reference/classes/motors/SolidMotor.html).

The AIM-120C game geometry experiment uses Mach 0.6, alpha 0/2 degrees, beta 0,
frontal reference area 0.024828666475777392 m2, diameter 0.1778 m and moment
reference x=1.83 m. Geometry coordinates are x aft, y right, z up. Wind-to-body
forces use `CA=CD*cos(alpha)-CL*sin(alpha)` and
`CN=CL*cos(alpha)+CD*sin(alpha)`; pitch moment retains the y-axis sign. The CFD
mesh includes the body and all eight authored/mirrored fins. The VSP route uses
a FUSELAGE and eight WING components with authored airfoil coordinates and fin
placements, retaining lifting Kutta/wake topology. The initial trials used a
truncated nose; the corrected worker represents an authored point as a native
point. Circularization and sampled loft differences remain explicit geometry
approximations.

| Gate | Observed outcome |
| --- | --- |
| Initial mesh suitability | Closed/oriented shell; 8,705 nodes and 34,766 tetrahedra, but minimum SICN 0.00128956 and 14 ill-shaped cells. Positive volumes do not establish sufficient CFD resolution |
| Initial SU2 convergence and physical loads | First-order initialization reached density residual -5 at both angles, but zero-angle CN=0.306 and negative wind-axis drag fail physical gates. Both second-order restarts missed -5 after 3,000 iterations |
| Initial VSPAERO lifting solve | 48 Kutta nodes, 280 wake loops and observed GMRES targets passed; near-surface inviscid CDi approximately -75.8 and zero-angle CLi approximately -3.25 fail drag-sign/symmetry checks |
| Geometry agreement | The sampled VSP body differs from authored PCHIP radius by up to 2.586 mm; exact parity and mesh sensitivity are unaccepted |
| Independent coefficient agreement | Failed; no coefficients were imported. Far-wake loads are separate diagnostics and cannot replace failed pressure loads or supply a missing pitch moment |

The maintained runner returns failure for these results. Its static comparison
requires matching conditions, density convergence, physical drag/symmetry,
lifting wakes, every observed GMRES target, final nonlinear convergence and inter-solver coefficient
differences within 25 percent. Even a future static pass retains
`model_accepted=false`: damping, crossflow, viscous effects, mesh convergence and
equipment admission require separate evidence. The aerodynamic record is
`aero/README.md` and `aero/evidence.json` under the evidence directory, with raw
commands, geometry/executable hashes, successful executions and failed gates.
The normal-force and pitching-moment symmetry tolerances at zero alpha are
separately declared absolute coefficient limits of 0.02. Matching spurious
pitch-moment bias in both solvers cannot pass static acceptance. Independent
review corrections also cover Windows archive replacement and explicit failed
reports after unexpected helper errors; the final aerodynamic safeguard suite
passes 44/44, including full steady-history and mesh topology safeguards. Both
maintained CTest reference guards passed 2/2 in 0.62 s after this integration.

### Numerical diagnosis, 2026-10-18

The corrected SU2 worker uses native point/conical body patches, authored foil
splines and a quality-optimized quarter mesh reflected into a full volume. No
flow symmetry boundary condition constrains the solution. It rejects unsupported
asymmetric source geometry. Pure guards check finite coordinates, signed cell
volumes, connected cells, opposite-sided neighbours at each shared face, exact
exterior coverage, closed wall topology and shared reflection seams. Nonfluid
CAD nodes are excluded before export. The monitored wall excludes internal
reflection planes; all eight source fins total 0.30124445935 m2 planform area.
The printed SU2 reference force, 633.9726869 N, agrees with `q*Sref`.

The experimental coarse mesh reached density residual below -5 at alpha 0 and
both +/-2 degrees. Zero-angle CN and CM were below 0.000000002; the unconstrained
positive/negative responses were antisymmetric. A finer mesh also converged,
but CA changed **126.2 percent**, CN **9.10 percent** and CM **71.9 percent** at
two degrees. This fails spatial acceptance. The earlier maintained CLI run
uses its own recorded mesh: it passes density/symmetry checks but has zero-angle
CD=-0.00270929, so it correctly fails the drag check. Density convergence alone
does not establish force stability or spatial accuracy. Evidence and exact
mesh/configuration hashes are in `aero/su2-diagnosis/` and `aero/current-run/`.

VSPAERO's tiny elliptical nose approximation produced missing first-band edges.
Using `XS_POINT` only for an authored zero-width/zero-height first section closes
the isolated body and changes its volume by only 0.000000000672 m3. Isolated body
CD changes from -65.5085 to +0.0132631, CN from -2.94831 to +0.00182109, and CM
from -30.5854 to -0.0153686. The implementation preserves a nonzero flat base and
rejects a partially collapsed nose or unsupported displaced origin. The full
assembly still has unmatched fin edges, unphysical loads and unconverged flow;
refinement trials did not establish a repair and timed out at their 120 s bounds.

The full steady VSPAERO history now has to match the current polar's conditions
and inviscid loads. The campaign explicitly sets `NonLinearConvergenceFactor=1`
and requires the final log10 maximum residual at or below -3 in **both** cases,
as [the pinned solver implementation](https://github.com/OpenVSP/OpenVSP/blob/db6c6fe8f4eca27d7517f4ae0b2078abdd3fd927/src/vsp_aero/Solver/VSP_Solver.C#L3225)
requires. Inner GMRES success is a separate condition. Stale, incomplete,
nonfinite, duplicated or mismatched histories fail. A regression first reproduced
matching plausible loads returning false success despite unconverged flow; the
integrated checker rejects them and accepts the exact threshold as a static
check only, with `model_accepted=false`.

Pinned source inspection identifies the polar's CLi/CDi/CMiy as near-surface
Kutta-Joukowski edge-force loads. They are not labeled literal pressure-area
integration. On one closed, numerically converged isolated fin, a separately
labeled Cp-area diagnostic gives CA=0.003614 and CN=-0.017740, versus native
edge-force CA=0.194362 and CN=0.416345. That discrepancy remains unresolved and
does not authorize changing the frozen comparison basis. Indexed panel edges,
source records, literal integration, units and precision limits are retained in
`aero/vsp-diagnosis/`. No far-wake or Cp diagnostic replaces a failed native
near-surface comparison.

That earlier maintained CLI actually executed both solvers and returned
`failed_static_comparison`, `model_accepted=false`. Its complete report and a
24-file source/tool/case hash manifest are under `aero/current-run/`; this manifest
also has an explicitly limited provenance scope. All earlier failed trials remain
separate. These improvements establish safer tooling and isolate numerical
defects; they admit no production aerodynamic table.

### Controlled convergence continuation, 2026-10-18

The same archived SU2 coarse mesh and restart, with unchanged Roe/MUSCL/limiter
settings, fails sustained density convergence with adaptive CFL but reaches
log10 density residual -10.57 and all three native force Cauchy limits with fixed
CFL=1. The former adaptive case ran 1600 additional iterations without meeting
those criteria. The retained fine mesh also meets the stronger limits with fixed
CFL. These controlled results isolate pseudo-time control as an iterative
convergence defect; they do not remove the observed spatial sensitivity.

The maintained runner now checks every finite, strictly ordered history row,
density residual <=-8, native CD/CL/CMy Cauchy values <=1e-10, and two complete
100-sample force/moment windows. For each body coefficient the final-window
span limit is `1e-4 + 1e-4*abs(mean)` and the adjacent-window mean-drift limit is
`1e-5 + 1e-5*abs(mean)`. These are explicit numerical checking budgets, not
equipment accuracy claims. Separate bounded passes retain their own raw
histories/configurations and input/output checkpoint hashes. A failed process,
timeout or malformed history cannot qualify for restart continuation or success.

The new mesh family applies Gmsh's global `Mesh.MeshSizeFactor` to every size
source, including curvature and farfield spacing, with fixed CAD/domain/field
transition locations. Single-thread generation, seed 1 and reproducible mode
produce identical coarse mesh and surface hashes on repeated generation. This
family is separate from the earlier near-wall-only refinement experiment.

The actual maintained CLI at global size 0.04 completed both 0/2-degree cases in
two bounded passes each. Final density residuals are -10.62770494/-10.82279945;
all native Cauchy and engineering load-window limits pass. At zero alpha,
CD=0.0007080121205, CN=-2.78e-15 and CM=-1.91e-14. At two degrees,
CA=-0.0009737681982, CN=1.143678250869 and CM=-0.8371866832. Its mesh SHA-256
`bd797f89fd7bd373f0482b91024a60c7cdc63f3ea0d79d29aaff0e727bca27d9`
matches the independent global-coarse case exactly. Raw evidence is
`aero/converged-run/`. This report still fails panel/static acceptance; it is a
successful SU2 iterative-convergence check, not a passing model.

The VSPAERO worker now checks indexed panel closure, edge direction, one connected
outward shell, nonzero polygon edges and dominant-plane polygon simplicity
before flow execution. It performs no vertex welding or face repair. These
checks do not prove absence of intersections between distinct panels. The old
full mesh has 95 genuine open edges and fails before the flow sweep.

Pinned OpenVSP source and independent geometric comparison strongly localize
the missing tip faces to orientation-dependent native intersection trimming.
The default inside/outside ray is `(1, 1e-6, 1e-6)`. Inverse-transforming the
0.123-degree rolled case matches every original node and restores exactly 53
front-fin tip triangles, removing none; every restored triangle is tangent to
that ray. The private hit parity was not instrumented. This source-supported
diagnosis is distinct from proof of a converged panel solution. Indexed evidence
is in `aero/vsp-closure/stages-evidence.json` and
`aero/load-audit/ray-audit.json`.

The maintained complete-frame geometry worker reproduces that closed mesh
exactly: 1710 nodes, 1648 polygons, 3356 opposite-paired edges, one outward shell
and zero boundary edges. Its geometry SHA-256 is
`05cc657c7a929405bc7596dd92d74b362b2b716a8b4b008e3c63aba2d03f2c72`.
It rotates all nine component origins/orientations and the moment reference,
and records exactly two paired native wind conditions. The pure recovery helper
rotates all six loads, first dimensionalizing moment coefficients by their
possibly unequal reference lengths. Analytic covariance and mutation checks
protect that distinction. Explicit frame mode now executes two separate native
single-case flows with precisely paired alpha/beta values, never a Cartesian
angle sweep. Each case retains and rechecks its own current raw log, polar,
history, geometry hash, reference area and rotated moment origin. The real
OpenVSP 3.55 worker completes this path in 15.77 s with a single closed geometry
hash shared across both cases. Its maximum residual logs +1.23635/+1.20737 and
negative wind drag still fail acceptance. The geometry-only result cannot
produce a static/model pass.
Raw native geometry, log and a scoped 14-file manifest are retained in
`aero/frame-geometry/`.

Linear log checks now use actual case banners and every observed wake, allowing
legitimate nonlinear early termination. Both native printed GMRES targets remain
mandatory; the final nonlinear history is still independently required. The
pressure-output audit also establishes why a literal emitted Cp-area sum need
not reproduce native Kutta-Joukowski loads: boundary/tangential edge contributions
and pressure postprocessing differ. It does not replace native inviscid loads
with a more favorable observable.

Three-grid uncertainty arithmetic is checked against NASA's published pressure
recovery example and unequal-ratio manufactured data. Oscillatory/constant or
nonpositive-order trends fail instead of assuming second order. The function
requires an explicit positive normalization scale and does not declare an
asymptotic regime from a three-point fit. NASA's
[spatial convergence procedure](https://www.grc.nasa.gov/www/wind/valid/tutorial/spatconv.html)
also cautions against relative error near zero. Existing failed physical gates
are not relabeled by these diagnostics. Full spatial and panel acceptance
remain under investigation.

### Frozen-surface and source-kernel controls, 2026-10-18

Independent integration of SU2's emitted wall pressures reproduces native drag
and the nonzero pitching moment to within 0.000000000017 in coefficient units.
The large grid change is therefore not a wind/body conversion or reference-area
error. Component pressure contributions partly cancel. In particular, changing
the globally refined grid changes base drag from 0.108692 to 0.060710, while
nose, boattail and fin contributions also change.

A controlled volume remesh retains all 12,528 wall and 736 farfield triangles
exactly, including coordinates, connectivity and cyclic orientation. Only
virtual reflection boundaries may be remeshed; they become internal faces in
the full exported domain. The added wake field covers ten diameters downstream
with 0.025 m nominal spacing. The volume mesher can change off-box tetrahedra,
so this is a volume intervention with frozen physical boundaries, not proof
that every exterior-to-box volume cell is unchanged. Its final restored
CFL=1/MG3 calculation meets native density and all three Cauchy criteria;
independent pressure integration again matches native drag. Zero-angle drag
changes from +0.000708 to -0.036300, principally through base pressure. A matched
MG3 control changes the original coarse drag by less than 0.000000001.
This establishes material volume-resolution sensitivity and retains a failed
physical gate. Evidence is `aero/su2-convergence/wake-comparison.json` and
`pressure-evidence.json`.

The subsequent coherent wake-resolved family generates its wake field before
all meshing and scales every size source together. Its three meshes contain
14,691/26,325/61,077 nodes and 64,708/118,536/296,088 tetrahedra; all topology
guards pass and minimum SICN exceeds 0.075. This is not yet an accepted
refinement sequence: the first coarse flow stalls with CFL=1, and local volume
node counts are not uniformly monotone despite smaller measured spacing.
No uncertainty estimate or finer-flow acceptance is inferred from those meshes.

OpenVSP 3.55.0 removes the earlier repeated-vertex spurs and DLB errors in the
mixed-surface case. Later, stronger source-planform checks still identify
missing forward trailing-edge root wedges and embedded fin fragments; passing
the initial perimeter and closed-body checks did not establish complete mixed
geometry fidelity.
This diagnostic deliberately omits the authored 4 percent fin thickness. Both
80-iteration free-wake cases complete and all 160 inner linear targets pass,
but final maximum residual logs -0.213853/+1.228673 fail -3. Freezing only wake
positions before the first solve produces constrained convergence in three
iterations; the zero-angle pitch coefficient still fails its frozen guard.
Reducing only native free-wake relaxation to 0.25 reduces several load spans
but still fails the unchanged pre-damping nonlinear criterion. Fixed-wake
results do not replace a free-wake acceptance result. Raw geometry, configurations,
histories and checks are under `aero/vsp-closure/mixed-3.55*`.

A complete 0.123-degree rigid frame restores all 48 mixed-surface trailing-edge
nodes. Independent intersection checks then locate exactly one wholly embedded
triangle, area 0.00000910494449 m2, among 699 native alternate thin triangles.
An experimental four-line native-file adapter removes only that interior ear;
the replacement follows an existing body edge. All coordinates, exposed UV
data, retained winding, body faces and wake lists remain unchanged. All eight
source-planform-minus-body areas and perimeters agree within native coordinate
printing precision. This narrow geometry gate has independent review; it is
not a generic repair or physical admission. The corrected-geometry source-built
free-wake follow-up still fails badly: final maximum residual logs are
+15.1042/+3.65935, with negative drag and broken zero-angle symmetry. The first
nonzero force row already fails, so subsequent relaxed wake motion alone cannot
explain its onset. Evidence is `aero/vsp-closure/corrected-mixed-source-freewake/`
and `mixed-3.55-roll0.123-geometry/body-domain/`. The frame also restored native
root wedges; changes cannot be attributed solely to the removed triangle.

An output-only SU2 source probe reads the stored nonlinear maximum and its
native node coordinates without reevaluating residuals. Its upstream NACA
control preserves all printed residuals and loads; independent node lookup
confirms all 84 reported locations. This corrects the initial interpretation
of exported `RESIDUAL` fields: multigrid can overwrite those linear-system work
arrays with corrections. Continuous downstream grading removes the old
transition maximum, but the remaining true density maximum lies at the outlet
centre. The graded case remains unconverged. A separate 20-step linear-solver
diagnostic reaches every estimated 1e-6 target in at most 4 of 10 iterations;
it does not establish a recomputed true linear residual or nonlinear convergence.
Source/probe provenance is under `aero/source-build/`; current mesh, checkpoint,
component-pressure and linear diagnostics are under `aero/su2-convergence/`.

SU2 8.5.0 and standalone VSPAERO 7.2.2 from OpenVSP 3.55.0 have now also been
built locally with UCRT64 GCC 16.2.0 and OpenMP under `D:/ENV`. SU2 required one
source-relative include correction, with no numerical-code change. Standalone
VSPAERO needed C++17 for its existing `std::filesystem` use and the explicit
`WIN32` platform definition; its numerical sources remain byte-identical to
the archive. The local SU2 NACA0012 iteration-20 result agrees with the upstream
reference within 0.000001 against its 0.0001 tolerance. The local VSPAERO
canonical wing reproduces the released executable's selected inviscid loads
exactly at printed precision for all three angles. Archive comparison covers
2,542 SU2 and 8,933 OpenVSP files. The upstream SU2 downloader fetches an Eigen
3.4 branch archive; its actual extracted bytes are frozen in a separate
6,839-file dependency manifest instead of claiming the downloader's commit
marker proves the archive's revision. Build failures, corrections, commands,
executable/runtime hashes and bounded reference checks remain in
`aero/source-build/` and the named `D:/ENV/physics/*-source/` directories.

A public-API test against that unmodified native VSPAERO library establishes
a separate core-regularization similarity defect. Scaling all lengths and
circulation together must preserve induced velocity divided by reference
speed. All six zero-core groups satisfy this condition at scales
0.001/1/1000; all six nonzero-core groups fail, with every output finite.
For example, an outside-core incompressible probe gives 0/3.1673024/3.1673024.
The source restores integral quantities to squared/fourth-power lengths before
comparing them with inconsistent powers of a length-valued core width.
This is a compiled kernel failure, not evidence that it alone explains every
whole-model failure. A separate subsonic metric-distance guard experiment
restores all 36 similarity samples and matches the declared hard-tube analytic
kernel. All 3,240 endpoint/probe/circulation derivative components pass in both
libraries. The 324 Mach-derivative checks expose a separate existing adjoint
chain-rule defect, retained as failed evidence; this experiment does not fix it.
The changed whole-missile cases still fail, with maximum residual logs
+0.129795/+1.303594 and failed drag/pitch guards. No modified solver or its
coefficients are admitted. Source paths, native regressions and analytic controls are under
`aero/load-audit/wake-controls/`.

An explicit `--vspaero` path selects a native executable through OpenVSP's
supported path API, verifies the selected directory and records its hash.
Missing helpers or a refused/mismatched path fail before flow execution.
The local source bundle contains the source-built solver and load helper plus
the released viewer required by API readiness; the viewer is not launched.
Its actual paired-flow replay reproduces the released solver's failure result.
The bundle's component origins and hashes are explicit in
`D:/ENV/physics/openvsp-3.55.0-source/api-bundle/bundle.json`.

The complete maintained CLI also executes with both source-built solvers and
the paired frame mode. SU2 0/2-degree cases each meet all strict criteria in two
passes; printed CA/CN/CM reproduce the released-binary coarse results. Panel
closure, native conditions, lifting wakes and all inner linear targets pass,
while nonlinear convergence, wind-drag sign and inter-solver agreement fail.
The final report is `aero/source-built-campaign/report.json`, with an immutable
copy/hash of the executed validation sources. Independent review subsequently
added regressions against hashing an unexecuted platform-basename decoy and
combining default-path logs/histories with different case/iteration endpoints.
The safeguard suite at that checkpoint passed 92/92; these fixes do not change
the actual selected `.exe` or paired-case results in that failed campaign.

Subsequent native geometry tracing identifies three separate cross-grid ID
errors in `VSP_GEOM::UpdateMeshes(int)`, `DeflectControlSurfaces()` and
`UnDeflectControlSurfaces()`. A solve-grid trailing-edge or wake-node ID must
follow each `FineGridNode` mapping before indexing grid zero. Unused source
vertices make these mappings nonidentity. The original corrected mixed case
actually moves a rear-fin surface vertex by 1.5376 m; an unchanged input file
alone cannot establish unchanged solver geometry. The first root-only fix
preserves the surface but still miscopies all 48 internal wake roots through
the unconditional zero-control deflection path.

The complete 25-line, three-method correction is retained as an optional
[upstream-source patch](../../tools/validation/missile/native/openvsp-3.55.0-wake-root.patch),
with its license and a public-library regression. Four fixtures cover identity
and nonidentity mappings at levels zero, one and two, physical surface
preservation, every internal wake point, repeat updates, and zero-control
deflection/undeflection. The original library fails both nonidentity fixtures;
the root-only candidate still fails both copy guards; the complete correction
passes all four. The maintained bounded runner independently reproduces the
original failure and complete-candidate pass. Commands, input hashes and the
retained unsuccessful library-path attempt are described in the
[native validation instructions](../../tools/validation/missile/native/README.md).
This is an external offline solver correction, not a production SDK change.

Exact v7 ADB decoding distinguishes the input geometry from the actual solved
geometry. Binary32 source point and trailing-edge multisets, solve-grid
connectivity, internal root attachment and all exported internal wake points
are checked through exact EOF. The complete candidate passes these checks on
the actual mixed missile cases. The retained original all-thick case also
passes: its negative drag cannot be attributed to this compaction defect.
Concave trailing edges export only one internal point, so these checks do not
claim visibility of their unexported trail coordinates.

With complete mapping correction and the original core law, both mixed
80-iteration cases retain all inner linear targets and actual geometry but
fail the outer -3 limit at -0.1100/+0.2151. Combining the independently tested
subsonic metric-core experiment with the complete mapping correction passes
the canonical wing control but still fails the missile outer criterion at
-0.3332/+0.2636 and gives a positive two-degree pitch moment. At zero degrees,
changing only wake nodes from 8 to 16 to 32 gives maximum residual logs
-0.3332/-0.7035/-0.7544. Axial force changes little while pitch moment changes
materially; none is iteratively converged, so no spatial extrapolation or
coefficient admission follows. Evidence is under
`aero/vsp-closure/wake-mapping-fix-v2/` and `aero/wake-spatial-refinement/`.

The SU2 outlet investigation preserves every one of the 12,544 oriented wall
triangles and all 6,274 wall coordinates while extending the wake-resolution
corridor to the outlet. The accepted mesh contains 41,827 nodes and 223,948
positive tetrahedra; minimum SICN is 0.0944. Outlet-axis spacing falls from
2.855 m to 0.0865 m. An earlier mesh that changed base triangles is retained as
a rejected frozen-wall control. The final 4,420-iteration fixed-settings
campaign improves density residual to -7.24153 but plateaus before -9; drag
and pitch Cauchy criteria also remain above 1e-10. Drag is -0.03486782.
Independent integration of both the exact outgoing checkpoint and a fresh
surface export reproduces native drag within 0.000000000002. The true final
density maximum is again at the outlet centre despite diameter-scale cells.
These findings establish a remaining discrete-flow problem, without assigning
it to an unproved boundary-condition or pressure-integration fault. Evidence is
`aero/su2-convergence/outlet-4420-iteration-results.json`.

The pinned native multigrid behavior qualifies stored configuration labels:
`CFL_ADAPT=NO` holds the finest-grid CFL, while SU2 still adapts coarse-grid
CFL within its multigrid implementation. It also forces a V cycle on restart,
even when the stored configuration says `W_CYCLE`; cold runs use the configured
W cycle. The continuation records preserve this distinction. Neither behavior
changes the declared steady spatial residual or its acceptance criteria.

Pinned SU2 source tracing and its official limiter documentation support a
single-variable `VENKATAKRISHNAN_WANG` comparison at unchanged K=0.05. Original
dimensional Venkat uses the same epsilon for pressure, temperature and velocity;
Wang scales it by each field's range. This changes the limited operator, so
any later refinement family must use one declared limiter consistently. The
source already implements nonphysical reconstructed-state fallback and an
entropy eigenvalue floor of 0.001. The audit does not establish an
entropy-related cause; no entropy-coefficient tuning is adopted. The source audit is retained in
`aero/load-audit/su2-discretization/`. Cold original-coarse two-degree controls
then strictly converge with both limiters, reproducing the original fixed point
before comparison. Wang changes CN from 1.14367825 to 1.055955 and CM from
-0.83718668 to -0.72306022. Its zero-degree counterpart also strictly converges,
but CD=-0.01322806 fails the physical drag gate. Exact checkpoint pressure
integration reproduces these loads. No old-Venkat and Wang grids are combined
in a refinement estimate. Acceptance thresholds remain unchanged.

The completed coherent Wang family now has strictly converged zero/two-degree
solutions on all three grids. Zero-degree CD is -0.01322806, -0.03698914 and
-0.01422554; this oscillatory sequence rejects a GCI estimate. Two-degree CN
is 1.0559550, 0.9475360 and 0.8000998, with CM -0.7230602, -1.1221129 and
-1.3337563. These values also do not establish a positive-order asymptotic
regime. Exact outgoing pressure integration reproduces every native force and
moment channel within 0.000000000402. Per-CAD zero-degree load partitioning
shows why similar coarse/fine total drag is misleading: body CA changes by
-0.015508 while the fin contribution changes by +0.014510. The remaining
-0.000997 total difference hides substantial cancellation. Evidence is
`aero/su2-convergence/wang-family-run/family-audit.json` and its component CSV.

Output-only VSPAERO residual reporters reproduce every ordinary canonical and
missile history/load token, excluding elapsed time. All 160 mixed-case maximum
and L2 residuals are independently reconstructed from the exact stored surface
and wake aggregates. At zero degrees the final maximum is a wake-position
defect; at two degrees it is a rear-fin no-penetration velocity residual.
Literal closed-body intersection checks find internal wake nodes and crossing
segments inside the solid, despite correct root attachment. N8/N16/N32
zero-degree cases have 15/0/13 interior nodes respectively. N16 is contact-free
but still unconverged, so penetration does not establish a complete causal
explanation. ADB consistency and panel closure do not establish a valid
free-wake solution. Evidence is `aero/load-audit/vsp-outer-followup/`.

A matched five-iteration all-thick control with the complete mapping/core
candidate preserves actual solved geometry and both inner targets, but still
has CA approximately -3.07713/-3.06110 and outer logs +1.27307/+2.03703. The
metric-core correction does not cure that all-thick failure; no longer run is
inferred to help. See `aero/allthick-joint-control/`.

A second output-only reporter separates that same stored winning velocity into
bound-surface, pure-wake and local-freestream contributions. Both N8 replays
preserve all original reporter rows and ordinary history/load values. All
3,520 captures per case close their velocity sum within 0.000000000000057 m/s.
Every upstream-directed captured velocity (79 at zero degrees and 82 at two
degrees) is evaluated inside the actual closed body. Bound-surface plus
freestream velocity reverses in every such capture; pure-wake plus freestream
reverses in none. At the final two-degree first-segment midpoint, the axial
contributions are approximately -493.500, +0.00284 and +204.052 m/s. This
identifies the source of these particular reversals; it does not prove that
removing solid penetration alone converges the coupled wake/surface solve.

The source-supported implicit wake method passes the unchanged canonical wing
gates, including both inner GMRES targets and the outer criterion. The actual
two-degree N8 missile run times out at its frozen 120-second bound during wake
iteration 17. Its last complete outer maximum log is +2.30175, so no final
implicit missile coefficients are admitted. Source inspection also corrects
an earlier diagnostic explanation: the native GMRES loop continues while
either target is unmet and stops only when **both** are met. Executed acceptance
checks already enforced both; their classifications do not change. The dormant
whole-Jacobian test is private, disabled and itself uses an unmapped fine-grid
node index, so it cannot be enabled as trustworthy coupled evidence unchanged.

The native `FreezeWakeRootVortices=1` control prescribes four additional
front-root trails, leaving 40 of 48 trails free. Its canonical output remains
exactly unchanged. Both missile cases pass actual ADB and both inner targets,
but outer logs -0.16190/+0.16642 still fail, and closed-body penetration remains.
Native residual aggregation omits the prescribed lines; their unmeasured
streamline defects remain unknown. This is a partially prescribed diagnostic,
not a converged free-wake result. A pinned source audit finds no safe dormant
body-avoidance switch: near-surface velocity damping has no solid-membership or
finite-step guarantee, and reusing total off-body velocity inside an
induced-only accumulator would double-count freestream.

An independent SU2 thermodynamic audit reads five released conserved-state
checkpoints with exact header/EOF and input hashes. Every state is positive and
subsonic (maximum Mach approximately 0.689--0.694). Local total-enthalpy defects
reach approximately +15% at sharp rear-fin trailing edges; global tetrahedron-
volume-weighted RMS defects are much smaller (approximately 0.000073--0.000098
for the compared two-degree cases). The source audit and these field values do
not establish an entropy-fix or global energy-flux defect. A separate frozen
smooth slip-sphere control tests the common three-dimensional Euler wall path.
Its three closed, positive-volume meshes have 2,736/6,480/16,034 nodes. All
three flows now pass the same strict native and load-window criteria after
2,201/4,756/5,941 cumulative rows. CD decreases from 0.06837301 to 0.03227248
to 0.02862947. Area-weighted RMS differences from the incompressible Cp limit
decrease from 0.405106 to 0.322443 to 0.253215; the formula is not an exact
finite-Mach-0.2 acceptance band. Fine-grid volume-weighted H0 RMS is 0.00001055.
Pressure integration independently reproduces native drag within
0.000000000004. These results do not establish a universal wall/normalization
failure, nor do they establish missile spatial convergence. Original budget-
limited snapshots remain intact beside separately frozen normal continuations.
Evidence is `aero/load-audit/su2-entropy/`.

A separate conditioning control on the released 10,351-node coarse mesh changes
only finest-grid CFL from 1 to 5. Both angles now pass the unchanged native/window criteria and the
additional 0.00000001 endpoint-tail screen; corresponding CA/CN/CM/wind-CD
differences are below 0.000000001. The linear diagnostics describe the last
finest-grid solve, not every multigrid solve. Native restart still forces a
V-cycle and coarse-level CFL still adapts. Original finite but unconverged or
tail-limited captures remain intact beside explicitly frozen verification
passes. This control qualifies pseudo-time conditioning on that mesh; it does
not cure negative drag or establish grid convergence. After a separately frozen
auxiliary verification qualified the fine-grid two-degree reference, both
fine-grid CFL-5 cases also passed the same gates and agreed with their CFL-1
references within 0.00000002. The two-degree case required its preauthorized
tail verification; the original failed tail remains recorded. See
`aero/load-audit/su2-entropy/`.

Geometry-only panel controls also retain their failures. Increasing leading-edge
clustering from 0.25 to 0.5 preserves sampled CAD but exports three buried thin-fin
regions, with areas approximately 0.00001261/0.00002052/0.00001261 square meters.
Independent polygon checks establish that these regions already belong to the
primary NGon boundaries; the native alternate triangles partition those domains
without adding overlap. This is not evidence of a triangulation defect. A
source-preserving fore-fin root reparameterization produces a 30.635-micrometer
body T-junction, duplicated/repeated fin vertices and two zero-area alternate
triangles. Its solid-domain gate is unavailable because closure fails. Neither
control proceeds to flow; see `aero/load-audit/vsp-outer-followup/root-freeze/`.

The exact-source headless geometry core now reproduces both original raw mesh
hashes. An optional output-only probe also preserves both hashes when disabled
and enabled, with twelve complete snapshots per case. Independent budgets
locate the buried area in active split triangles at the earliest post-trim
snapshot, before flattening or any PGMesh cleanup. Each offending child retains
approximately 8.422% of its area inside the closed body while its centroid is
outside; native inside flags correctly describe that centroid. Exact oriented
XYZ/UW/tag multisets match trim-active, flattened and initial PGMesh domains.
This establishes incomplete geometric partition before centroid classification,
without establishing whether input intersection constraints or later split
processing are responsible. A further pre-split working-coordinate witness is
required before an algorithm correction. See
`aero/load-audit/vsp-native-clipping-audit/`,
`aero/load-audit/vsp-stage-independent-audit/` and
`aero/source-build/geometry-stage-replay/`.

An exact pre-split working-coordinate witness now confirms that every supplied
intersection segment survives constraint cleanup and native split triangulation
for the four offending parents. Independent rational intersections of captured
body/fin triangle pairs establish missing segments before that split. A native
regression using twelve captured pairs and two simple controls, each with eight
order/axis/power-of-two scale variants, reproduces 64 failures among 112 checks.
These are incorrect, collapsed or missing kernel segments; the tested failures
do not require weakening the later `InTri` filter. The native build and all input
immutability checks pass; the fixture's exit 1 records the required original
regression. Eighty additional boundary controls pass against the original
library, leaving 64 failures among 192 total. Independent exact-rational review
confirms all expectations. A first candidate is rejected before application:
rounded cut coordinates can miss true point contact or admit a one-ulp positive
gap. Native compilation confirms all 88 first-candidate failures; the main
native library was never changed for that rejected experiment. The revised
kernel uses bundled GTE exact rational cuts and interval classification, then
rounds accepted outputs. All **280 native controls pass**, with zero endpoint
error for the 104 positive segments in the original 112-case set. Independent
source, data, patch-roundtrip and native-report reviews pass. The maintained
[patch and fixtures](../../tools/validation/missile/native/README.md#exact-triangle-intersection-correction)
reproduce the same original RED and corrected GREEN reports; their external
boundary/contact CTest selection passed **2/2 in 0.10 s**. The final selection
adds 54 nonfinite-input rejection checks and two finite controls, passing
**3/3 with 336 native checks** in 0.134 s of controller wall time. The original
kernel fails all 54 rejection checks while both finite controls pass; the
corrected kernel passes all 56. This new input policy does not certify the
legacy degenerate/coplanar algorithm. Evidence is
`aero/source-build/maintained-guard-final/`. The earlier
post-trim witness has mixed nested units: active nodes are SI, while auxiliary
intersection-edge-only nodes retain working scale; area budgets use active
leaves and exact new working inputs. Evidence is
`aero/source-build/triangle-split-replay/` and
`aero/source-build/intersection-fixture-original/`.

The corrected native core replays both unchanged saved CAD inputs in
0.55-0.75 seconds each, with identical dormant/enabled diagnostic output.
Independent checks pass primary/alternate body closure and outward orientation,
strict polygon simplicity, exact alternate partitioning, all eight connected
thin-sheet boundaries, all 48 trailing-edge nodes and source associations.
Every saved source Geom XML subtree is unchanged. Maximum exposed-area and
perimeter discrepancies are 0.000000000006643 square meters and
0.0000000004831 meters, within the original native-print budgets. All 24 stages
have zero significant buried area using unchanged per-face rounding budgets;
the former lobes vanish already at the earliest active post-trim stage.
These results close the demonstrated native partition/closure defect, without
admitting aerodynamic coefficients. The exact patch, final-geometry report and
independent stage report are recorded under
`aero/load-audit/vsp-native-clipping-audit/kernel-v2/` and
`aero/load-audit/vsp-stage-independent-audit/kernel-review/`. Downstream actual
ADB, wake, spatial and physical gates remain mandatory.

The corrected mixed geometry has now completed the same frozen paired
zero/two-degree free-wake diagnostic, one native process per angle. Both actual
ADB geometries and every inner linear target pass; all 768 packed wake points
and 160 outer MAX/L2 rows are independently assessed. The paired physical
symmetry/nonnegative-drag checks pass, but the unchanged outer target -3 fails
at -0.42156/+1.23022. At two degrees the last recorded body surface-equation
residual has magnitude 16.99097 near the fore-fin root; it includes the native
Kelvin regularization as well as normal velocity. Its pre-update phase differs
from the final ADB state. Definite geometric interior packed points
remain, with float32 boundary uncertainty kept separate. A subsequent source
and circulation audit distinguishes active trails from concave trails whose
native induced velocity is disabled; packed geometry alone is not proof of
active-vortex penetration. No coefficients are admitted.

The first controller stopped after its completed zero-angle native process
because postprocessing omitted a required metadata key. A separately reviewed
recovery verifies its frozen inputs and raw outputs, reuses that result and
executes only the missing two-degree case. The original failed controller is
preserved; zero-angle flow was not repeated. Final evidence is
`aero/load-audit/vsp-native-clipping-audit/kernel-v2/flow-control/recovery-v4/`;
the active-circulation diagnosis is recorded separately under
`kernel-v2/flow-strength-audit/`.

A geometry-only all-thick replay also passes literal closed outward topology
(1,710 nodes, 1,648 primary panels and eight wakes). Export-ID-independent
comparison finds its physical points, oriented primary/alternate XYZ/UV/tag
domains and all ordered trailing edges exactly equal to the historical
all-thick case; no numerical tolerance is needed. Its different raw hash
therefore supplies no new physical-geometry control or reason to repeat the
same flow. The exact-intersection fix changes the demonstrated mixed thin-fin
partition defect; it is not claimed to repair the historical all-thick flow.
Continuous authored Boolean-exterior coverage remains separate from finite
mesh topology/partition and saved-source identity. Evidence is
`aero/source-build/intersection-kernel-v2-allthick-replay/` and
`aero/load-audit/vsp-stage-independent-audit/kernel-review/allthick-v2-review/`.

The native near-field KJ output also has an independent six-channel force and
moment reconstruction, agreeing within 0.000000000000476. The first temporary
reader omitted negative signed boundary IDs and confused the Kutta count with
its edge table; both errors are reproduced and corrected in a separate audit,
with the original failure retained. At two degrees, the native incident-loop
body contribution CM=+0.82320 is partly cancelled by rear fins CM=-0.70443.
This partition explains the positive total CM without changing the observable;
neither underlying free-wake solution is converged. See
`aero/load-audit/vsp-kj-phase-audit/`.

A fourth same-CAD global mesh, size 0.016875 m, has independently verified
63,629 nodes, 266,504 positive tetrahedra and 65,120 complete CAD-labeled wall
faces. Its node-based refinement ratio against the preceding fine grid is
1.23429, rather than the nominal spacing ratio 4/3. No flow result follows from
this preflight; see `aero/su2-convergence/finer-global-preflight-0.016875-review/`.
The subsequent separately frozen CFL-5 campaign does supply strict numerical
endpoints: 823/832 normal rows at zero/two degrees, density residuals below
-11.71, all native Cauchy/window criteria, positive states, pressure agreement
and maximum endpoint-tail envelopes below 0.000000008. At zero degrees CD is
-0.01086299638. At two degrees CA=-0.01181209992, CN=0.7579182363,
CM=-1.140464075 and wind CD=0.01464606068. Relative to the fine endpoint,
CN changes by -0.04218153 and CM by +0.19329221. Negative zero-angle drag and
unsettled grid behavior remain physical failures; no GCI or model update is
admitted. Exact checkpoint pressure/CAD and thermodynamic diagnostics are in
`aero/load-audit/su2-entropy/cfl5-fine-fourth-continuation/`. All selected states
are subsonic; local total-enthalpy errors remain measurable despite small
volume-weighted RMS values. The completed conditioning budget is not extended.

Per-CAD lift/moment comparisons now subtract each grid's own zero-angle
pressure loads before interpreting its two-degree result. Equal/opposite
radial fin loads cancel globally but can dominate raw per-fin values. The
fine-to-fourth incremental CN change -0.04218153 separates into body -0.06275267,
fore fins +0.03117249 and rear fins -0.01060134; the CM change +0.19329221
separates into +0.07553017/+0.01895377/+0.09880826. Every component's incremental
CN is nonnegative. These attributions avoid a false fin-asymmetry inference;
they still supply no asymptotic order or mesh acceptance. See the same folder's
`INCREMENTAL-CAD.md` and immutable checkpoint-based JSON.

A source-qualified same-coarse-mesh HLLC control changes only the selected
upwind method and declared process partition. It stops normally after 452 rows
in 53.79 seconds: density residual -11.32090 and native Cauchy/load windows pass,
but the unchanged 1e-8 empirical tail fails (CA/CD 1.68534e-8, CM 2.43597e-8).
No verification or extra pass runs. CD=-0.02709224893 remains negative,
compared with the qualified Roe CFL-5 value -0.01322806301. Pressure agreement,
positivity, zero-angle symmetry and stored finest linear targets pass. The
comparison concerns the complete native operator path: ideal-gas Roe uses
SIMD reconstruction, while HLLC uses scalar primitive-gradient reconstruction
and different flux/Jacobian routines. SIMD updates nodal H by reconstructed
temperature/kinetic-energy differences; this contrast alone does not prove a
defect. HLLC supplies neither a repair nor a spatial-convergence certificate.
Exact hashes, source qualification and the retained unresolved tail are in
`aero/load-audit/su2-entropy/hllc-coarse-alpha0-v2/`.

The maintained `--initial-model` option also compares both freshly assessed
solver pairs with explicit generated Hangar XML. It uses emitted coefficient
precision, converts area/chord references, and transports pitching moment from
the rounded AERORP to the requested solver origin. Generated-origin values
remain separate. Three actual SDK released-body samples at approximately
0/+2/-2 degrees reproduce the expression reader exactly at their measured
Mach/angle and emitted references; loaded DLL bytes match the requested
headless build. This is expression verification, not another aerodynamic
calibration. At Mach 0.6 and two degrees the current initial model gives
CN approximately 0.60608 and CM approximately -1.35906 on the solver references.
Its full static basis includes empirical crossflow and viscous axial estimates;
the diagnostic differences never alter inviscid agreement gates. The current
integrated standard-library safeguard suite passes 150/150 in 2.544 s.
Native readback and initial-comparison records are under `aero/load-audit/`.

The integrated panel worker has also been rerun on freshly generated all-thick
geometry with the source-built baseline solver. Both separate native cases and
the parent's fresh raw-file assessment pass actual geometry, trailing-edge and
wake attachment checks. The previous negative axial force and failed outer
criteria remain visible. This is an actual worker/parent-helper replay, not a
new complete SU2/VSPAERO CLI campaign; see `aero/adb-integrated-worker/replay.json`.

### Reproducible entrypoints

Run from the repository root with the pinned optional solver environments under
`D:/ENV`. The two maintained safeguard suites require only Python's standard
library; normal SDK builds do not acquire RocketPy, SU2, OpenVSP or Gmsh.

```powershell
$env:TMP = $env:TEMP = $env:TORCH_EXTENSIONS_DIR = 'D:/ENV/tmp'
& D:/ENV/python/3.13/python.exe -B tools/validation/missile/test_reference.py
& D:/ENV/python/3.13/python.exe -B tools/validation/missile/test_aero.py

# One model, under the matched reference conditions.
& tools/launch/fsim.ps1 --preset ucrt64-headless python tools/validation/missile/crosscheck.py `
  --model aim120c --spherical-reference --native-hz 480 960 `
  --native-bin build/ucrt64-headless/bin `
  --output build/architecture-reviews/sim-m02/aim120c-flight.json

# All authored AAMs; the emitted card is still checked against its exact design.
$models = @(Get-ChildItem -LiteralPath assets/stores -Filter '*.json' | ForEach-Object {
  $card = Get-Content -LiteralPath $_.FullName -Raw | ConvertFrom-Json
  if ($card.class -eq 'aam') { '--model'; $card.model.Substring(7) }
})
& tools/launch/fsim.ps1 --preset ucrt64-headless python tools/validation/missile/crosscheck.py @models `
  --spherical-reference --native-hz 480 960 --native-bin build/ucrt64-headless/bin `
  --output build/architecture-reviews/sim-m02/fleet-flight.json

# This experiment currently returns failure; retain its complete raw logs.
$env:PATH = 'D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin;' + $env:PATH
& D:/ENV/python/3.13/python.exe -B tools/validation/missile/aero.py `
  --design aircraft/aim120c/aim120c.toml `
  --output build/architecture-reviews/sim-m02/aero/reproduction `
  --gmsh D:/ENV/physics/gmsh-4.15.2 `
  --openvsp D:/ENV/physics/openvsp-3.55.0/OpenVSP-3.55.0-win64 `
  --su2 D:/ENV/physics/su2-8.5.0-source/build/SU2_CFD/src/SU2_CFD.exe `
  --vspaero D:/ENV/physics/openvsp-3.55.0-source/api-bundle/vspaero.exe `
  --initial-model aircraft/stores/aim120c/aim120c.xml `
  --frame-roll-deg 0.123 --mesh-size 0.04 --iterations 2200 `
  --su2-passes 4 --solver-timeout 300
```

The flight CLI invalidates an earlier passing report before starting and records
ordinary native/numerical failures explicitly. Missing runtime dependencies,
different native DLLs, stale generated assets, ignored atmosphere/gravity
overrides and incomplete/nonfinite histories cannot produce a passing report.

### Remaining SIM-M02 acceptance

- Reconcile exact AIM-120C-7 external geometry from readable primary pages; the
  NAVAIR/brochure span discrepancy remains open as recorded in section 10.6.
- Admit quantitative motor/grain/mass inputs with uncertainty and an exact
  variant. MS-7b permits explicit engineering assumptions where data are
  unpublished; those assumptions require sensitivity evidence and must retain
  that label. The reviewed manufacturer material supplies no usable real
  thrust-time curve; the existing phase/ISP/grain values are not measurements.
- Correct and demonstrate aerodynamic geometry/mesh quality, symmetry and
  credible converged force/moment loads, then establish independent agreement
  and sensitivity over the required Mach/angle/control envelope.
- Supply separately computed dynamic damping, crossflow and viscous channels;
  a static Euler/panel run cannot create them.
- Admit only supported data through the existing generator import path, followed
  by native flight/performance evidence and variant/fit acceptance across the
  fleet. Meteor's current solid boost/sustain approximation is not validation of
  its real ducted-rocket propulsion.

The software fixes and powered-flight agreement do not close these gates.

The earlier delivery review identified an orchestration issue in `parse_su2`:
when density has not reached its target, missing native Cauchy columns produce
`NotConverged` before the schema check. A mocked two-pass reproduction shows
that the outgoing checkpoint can then continue to a later valid history.
Required-column validation must precede continuability classification; malformed
output must terminate without restart. The numerical research handoff includes
this convergence-check defect. Existing retained native histories contain the
required columns, and no failed aerodynamic result is admitted. Evidence is
`aero/load-audit/su2-entropy/release-missing-cauchy-review.json`.

### AeroNumericsLab integration, 2026-10-21

The user opened the integration/regeneration window at main `d5c8203` and
authorized retaining improved numerical candidates for subsequent correction.
The source allowlist and serial build/test ownership are recorded in the
[campaign plan](../superpowers/plans/2026-10-21-numerics-integration-regeneration.md).
The standalone LAB remains read only.

The [import manifest](../../tools/validation/missile/native/numerics/PROVENANCE.json)
binds selected delivery source bytes to their project derivatives and retained
native data. The Task 02 schema-order correction is now in `parse_su2`:
mandatory columns precede any continuable classification. The delivery's 12
regressions reproduce 17 failing assertions against the old project parser and
then pass against the correction. The complete standard-library aerodynamic
guard suite passes 162/162. These checks execute no CFD process.

The modified VSPAERO R2 patch is retained with its OpenVSP license, exact base
and resulting source hashes, native executable identity and unchanged released
N8/80 cases. Independent patch application reconstructs the delivered source
exactly. Existing exact-intersection/panel-closure fixes are reused; no second
geometry repair or replacement of their original assertions is introduced.

The active-wake containment check now runs in both the worker and parent's
fresh raw-case assessment, including VKEY identity. A coordinate-only stand-in
fails six of the original seven delivery tests; the actual reader passes them.
Integration review additionally reproduced acceptance of NaN circulation and
added a failing regression before finite-data and resource-bound guards. The
resulting native-fixture suite passes 8/8. Synthetic process-orchestration tests
mock this binary-reader seam; the separate geometry suite reads real retained
ADB/VSPGEOM/VKEY bytes and changes both packed and internal coordinates together.

The reproducible [review entrypoint](../../tools/validation/missile/native/numerics/README.md)
reassesses all four retained SU2 histories and both R2 native cases. The R2
iteration, solved-geometry, containment and paired physical checks pass; the
local SU2 pair passes its physical checks. The original coarse alpha-0 wind
drag remains negative and explicitly fails its original physical gate. The
review requires that failure to remain visible; `review_passed` is reproducible
delivery disposition, while `model_accepted` stays false. Two-grid sensitivity
and iteration convergence do not establish load accuracy or superiority over
the current full empirical game model. No candidate coefficients are imported
into runtime tables, and no calibrated damping, crossflow, viscous or real
equipment channel is inferred.

#### Regeneration and complete-suite follow-up, 2026-10-21/22

All 35 aircraft designs execute the full forced campaign, with raw per-design
logs. The generator reports zero changed aircraft products: 25 designs end
cleanly and ten retain the previously documented [B-29 limitations](../hangar.md#built-again).
The ceiling-table checks fail for C-17A, EA-18G, F-15C, F-22A, F/A-18C,
MiG-29A, Mirage 2000, Su-27S and Su-57; Su-25 short-period damping is 0.2868
against the existing 0.3 lower bound. No calibration or assertion changes are
made to clear them. The first forced store batch recreates all 53 models and
flight models, with every child exit code zero. Registration also restores the
generator's existing C130-to-C130J viewer mapping, and generated AIM-9X names
now reflect its already declared Block I card identity.

The unchanged native RCS verifier checks all 36 existing tables using their
original recipes, including Phong 0.5 rather than the current CLI default.
B-52H, F-35A and J-20A fail the original noise-relative comparison. Their old
tables and headers are retained in the evidence directory; complete four-band,
61-elevation recomputation followed by the original five-elevation comparison
passes for all three. Only those three numerical payloads change. The other
33 payloads remain byte-identical; verified provenance is updated where needed.
The new persistent [binding gate](../../tools/validation/resources/README.md)
rejects stale model bytes and invalid recorded verification. Its ten unit
regressions include CLI stale-success replacement; no electromagnetic accuracy
or full angular verification is inferred from file binding.

The first full suites discover and execute 909 Release and 835 Headless cases,
without skips. They finish at 905/909 and 831/835, with the same four failures.
Both original showcase gates pass. During that first campaign, 1,057 source
inputs remain unchanged, and each of four staged runtime roots matches 813
source files by raw SHA256. All 105 frozen RCS generator/model/signature inputs
also retain their hashes.

The failures lead to narrow software corrections, rather than different model
parameters or relaxed gates:

- Physical reset cleared the damage object but retained `crewLost`, suppressing
  later controller updates. A new real-nose-separation regression first produces
  native pilot loss and fails six exact control/trajectory comparisons after
  reset. Clearing the physical incapacitation flag makes it pass and restores
  the original SDK bitwise reset check. Authority, tasks and MA bindings retain
  their existing reset semantics.
- Boundary-history material capture repeatedly resolves two long atmospheric
  property paths at both boundaries: four allocations per healthy vehicle per
  decision. Per-Entry bindings now follow the existing cached-handle pattern;
  values and validity are sampled afresh, missing bindings remain retryable,
  and reset invalidates the cache. The original 34 zero-allocation cases pass.
  The complete control digest is byte-identical before and after correction,
  SHA256 `ac6ebfe62c84dfd154dfcbcb0a6ef64f41242097c7253e24b685d6bef9d26bba`.
- Two AIM-120C source labels used undeclared vocabulary. The existing categories
  now identify the [APL original](https://www.jhuapl.edu/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf)
  as `research_publication` and the [secondary publication of Hughes artwork](https://ausairpower.net/amraam.html)
  as `secondary_reference`. Attribution, caveats and all parameter objects remain
  unchanged; the source-category validator is unchanged.
- Old synthetic propulsion fixtures did not supply the new card-file boundary
  and mocked the former motor factory. Correcting those seams preserves all 128
  original assertion-call ASTs and adds a factory-argument check. The aggregate
  suite passes 43/43 without changing production motor validation.

The parallel focused run additionally reproduces two live showcase fixtures
publishing the same fixed world simultaneously. A shared CTest resource lock
serializes those two fixtures within the existing concurrent scheduler. Their
scenes, assertions and 120-second timeouts remain unchanged. The resulting
focused selection passes 8/8. The corrected showcase recording retains SHA256
`a04d9dc1f5054cd0d736b68d3631996aead09e96f5e702fce3b4ed6a56834093` and
its complete engagement JSON retains
`f995ba37a10f55ec9ee21ed473f5214918eef732075466117ddc09571853f8e3`.

Raw generation logs, failed and corrected JUnit files, source/runtime hash
snapshots and native replay results are under
`build/architecture-reviews/numerics-integration-20261021/`. Final full-suite
results after the repairs and renewed store generation are kept in its separate
`final/` directory. Numerical-model admission remains independent of software
and resource-consistency acceptance.

The final campaign recreates all 53 stores on the corrected SDK and repeats
both complete inventories, without excluding the initial failures:

| Final check | Result |
| --- | --- |
| Forced store recreation | 53/53 child exit codes zero; registration and effects audit pass |
| Complete UCRT64 Release CTest | 910/910, 1161.75 s, zero skips |
| Complete UCRT64 Headless CTest | 836/836, 1142.08 s, zero skips |
| Inventory coverage | Each JUnit name multiset exactly matches its freshly discovered inventory |
| Resources after both full suites | All 1,057 frozen source inputs unchanged; four runtime roots each match 813 source files |
| RCS generation identity | All 105 frozen executable/operator/model/signature inputs unchanged after final builds |
| Physical input identity | All 53 store parameter objects and 130 payload cards equal the baseline; only two source-category labels differ |
| Original native showcase | Both presets pass; original recording and complete engagement JSON retain the hashes above |
| Payload evidence report | Fresh report imports both complete successful JUnit files; physical acceptance is not promoted |

Every one of the final workflow's 15 phases exits zero. Initial failed full
runs, the six-assertion crew-reset RED, the publisher-conflict RED and corrected
focused/full results are retained separately. These are current software and
resource-consistency results; no new manual fullscreen Viewer acceptance or
external SU2 cold-start campaign is claimed.

Remaining physical/input gates are explicit: coarse zero-angle negative drag;
a spatial-convergence family and independent load-accuracy reference; dynamic
damping, crossflow and viscous channels; equipment/variant calibration,
including detector/initiation, fragment field, vulnerability and measured motor
curves. The ten aircraft ceiling/damping failures above remain failures in
their own right. Iterative convergence and software regression success do not
establish overall aerodynamic superiority over the existing game model.
