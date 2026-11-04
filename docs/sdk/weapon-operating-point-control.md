# Operating-point bounded missile control

R-77M explicitly selects `fsim.control.operating_point_bounded.v1` (native
implementation code3). Retained providers, public layouts, plugin vtables and
`AutopilotInput` are unchanged. A private input and additive sibling carry actual
finite actuator state and unchanged physical fin curves. Current selection is
restricted to the authored axisymmetric R-77M diagonal-inertia plant; this method
does not read or claim a general nine-element inertia tensor.

The original `finCommands` rate/incidence cascade and existing response constants
define the desired moment. Per axis its target is the measured native CG moment
plus the retained neutral derivative times (the original cascade request converted
to its signed authored channel angle, less same-frame actual equivalent fin angle).
Original incidence and command limits remain. Actuator-aware pole compensation is
not combined with another finite-lag inversion. No aerodynamic table, physical
scalar, controller gain or frozen encounter is tuned by this allocator.

For this selection the native channel lag is exposed as an upstream native
`lag_filter` followed by the original rate/stop actuator. The predictor reads its
actual previous input, lag output and final channel output separately, then executes
the native Tustin lag, channel rate/stops, signed mixer and physical-fin rate/stops
through every generated integration substep. At68substeps it still executes all68.
The admitted substep is at most twice the lag time constant, keeping lag weights
nonnegative and its state within channel stops. Native load/reset executes `RunIC`
before control. Per-fin curves include current incidence/Mach, induced/ground drag,
wind/body rotation and the shift of full moments to current CG. Frozen-incidence
prediction is a numerical approximation; native FCS/aerodynamics produce all forces.

Production uses bounded sequential allocation. Each call seeds actual prior native
input plus the retained ten neutral/request/corner starts. Each seed permits four
iterations. The full predictor supplies both feasible one-sided derivatives for
all three normalized channels. The local angular-acceleration least-squares model
uses all27 box active sets; where directional derivatives differ it covers all eight
directional orthants. Equal derivatives reduce to the identical single convex box
problem. Pivoted Householder QR solves each small augmented least-squares face,
without normal equations or scalar effectiveness division. After common matrix
scaling, numerical damping is `sqrt(64*machine_epsilon)` toward prior input. This
regularizes numerical rank deficiency; it is not a physical gain. Every proposed
command is judged by the original undamped full-native cost.

At most eight halved steps seek an actual cost/secondary-effort improvement. If no
step improves, six physical box-end polls guard flat rate-limited, singular and
reversed-effectiveness branches. All accepted commands remain finite and bounded.
Unreachable torque retains a finite angular-acceleration residual. This is a bounded
local numerical method, with no claim of a global nonlinear optimum or closed-loop
stability proof. Per-call diagnostics bound eleven seeds,44 iterations,264
differential predictions,700 scalar predictions,9504 active faces,352 backtracks and
264 polls. Those are work bounds, not a real-time performance claim.

The bounded least-squares/active-set and previous-solution approach follows
[Harkegard2002, sections2–4](https://rt.isy.liu.se/research/reports/2002/2426.pdf).
Local convex approximations for constrained nonlinear allocation are described by
[Johansen, Fossen and Berge2004, sectionIV](https://www.fossen.biz/publications/2004%20Johansen%20et%20al%20TCST.pdf).
Our finite-actuator predictor, one-sided branch handling, small exhaustive active
faces and scalar-validated polling are engineering adaptations; those papers do
not establish stability or hardware accuracy for this missile model.

The exhaustive affine tracer and incremental piecewise-affine implementation remain
independent private numerical references. Their tests retain every intermediate
clipping/knot event, original start/sweep scope and scalar-verified candidate. Exact
per-call bit-key memoization and cached unchanged-channel trajectories do not cross
input snapshots. Bounded map storage overflows fall back to the complete tracer.
Private verification options select these references, disable caches or capture
candidate/objective audits; they do not add actor-facing API or physical settings.

Native failure inputs follow `FGActuator::Run`: zero input first, sign-selected
hardover next (strict negative selects the negative stop; zero selects positive),
then stuck overrides processing. Otherwise native rate/stops follow. The upstream
lag continues to update while its downstream actuator is stuck. Actual scalar
verification preserves exact zero, signed-mixer and underflow/nextafter endpoints;
a failed fin cannot become available authority through a linearized model.

The versioned sibling executes through nonvirtual
`PlantProvidersV1::commandsOperatingPointV1`. Every required provider binding,
actuator acceptance, sibling capability, finite/bounded output check and final
allocator decision remains authoritative. World cannot bypass custom-provider
refusal. Retained container fields and object layouts are unchanged.

The allocator is stateless; native reset owns actuator history and each call owns
its caches. Bounded scratch and caught allocation failures protect the native
`noexcept` boundary. Mathematical/reference equivalence, same-scene physical
predicates and measured whole-scene1x performance are separate required evidence.
No hit, complete stability envelope or equipment calibration is inferred from a
bounded command, helper test or visual presentation.
