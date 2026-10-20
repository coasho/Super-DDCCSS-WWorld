# SIM-M03 technical research and input readiness

The subsequent [reference decisions and current verification](sim-m03-reference-continuation.md)
supersede this initial readiness note where mechanisms have since been delivered.
They record mature control/sensor/geometry/blast references, explicit remaining
weapon-link/fused-input seams, current 30 km evidence and retained full-suite failures.

Date: 2026-10-20. Research baseline: main
`85e7d251c5de702de5d2ec66115a0886bd0702cd`, after the HC-53 correction.
The user requested technical preparation while Claude continues the independent
SIM-M02 numerical investigation. This documentation window covers this file and
links in `docs/status.md` and `docs/sdk/weapon-fidelity-validation.md` only.
That initial research stage changed no model, dependency, generated resource or
active research input. The user subsequently prioritised the
[finite-control implementation](../superpowers/plans/2026-10-20-finite-missile-controls.md).
Its mechanism uses explicit engineering estimates while calibrated model
acceptance retains the required evidence below.

Prefer the existing JSBSim facilities for future finite control response.
Prepare synthetic decoy timing and post-separation geometry cases independently
of aerodynamic calibration. Finite-fin integration requires control-deflection
derivatives and actuator response evidence in addition to a validated flight
plant. Complete model acceptance retains the
[allocation](../architecture/modules/simulation/ms-work-allocation.md) and
[validation](weapon-fidelity-validation.md) gates.

## 1. Production seams inspected at the research baseline

| Current implementation | Consequence for future work |
| --- | --- |
| `tools/models/hangar/hangar/storebodies.py` emits missile `<flight_control name="none">` | A finite-fin feature needs a generator-owned control chain and admitted force/moment response to control deflection; stability derivatives do not establish control authority |
| `World::preStep` in `src/simulation/runtime/world/World.cpp` supplies ideal forces/moments while canceling normal aerodynamic force/moment | Finite-fin acceptance must exercise native surface commands and achieved aerodynamic response; retaining this replacement path would conceal plant/control errors |
| Private Guidance returns ECEF acceleration demand; World supplies body attitude and admitted observations | Separate body/seeker/inertial coordinates, observation timing, guidance demand and actual acceleration |
| Vendored `third_party/jsbsim/CMakeLists.txt` identifies source version 1.3.1; `cmake/JsbsimPatches.cmake` carries actuator reset initialization fixes | Upstream 1.3.3 is a research reference, not an upgrade instruction; retain the project's launch/reset and ignition semantics |
| `Dispenser::step` integrates drag/gravity analytically and removes expired decoys; World advances it before the weapon observation pass | HC-54 needs matching sample times and actual boundary poses, including the earlier valid interval of a decoy expiring during a step |
| `World::detonate` passes the complete shared Airframe to `damage::assess`; VehicleDamage records separation separately | HC-55 needs an active geometry view with stable panel/component identities and no mutation of shared instance/World resources |

The first two rows describe `85e7d25`, before the user prioritised the
[finite-control implementation](missile-control-validation.md). They are not
the current control path after that correction.

## 2. Primary-source reuse evaluation

These are source/documentation findings. No candidate was installed, compiled,
benchmarked or exercised as an independent oracle in this research.

| Candidate/reference | Verified identity and capability | Proposed reuse and limits |
| --- | --- | --- |
| JSBSim actuator | [v1.3.3](https://github.com/JSBSim-Team/jsbsim/releases/tag/v1.3.3), commit `66f7fafc07fe13e7ede9db20873030fed2881713`; release lists Windows x64/MSVC 2022 and Ubuntu packages. [FGActuator](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/flight_control/FGActuator.h) declares LGPL version 2 or later and documents lag, asymmetric rate limits, deadband, hysteresis, stops and failures | Prefer the already integrated mechanism. Qualify actual response data, initialization and cadence. Upstream package availability does not establish SDK/UCRT64 compatibility |
| JSBSim filter/PID | Versioned [FGFilter](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/flight_control/FGFilter.h) and [FGPID](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/flight_control/FGPID.h) provide transfer-function/filter and integration/saturation mechanisms; headers declare LGPL version 2 or later | Candidate native FCS components. Controller structure, coefficients, sample timing and response tolerances need accepted plant/reference inputs |
| Upstream actuator tests | [TestActuator.py, v1.3.3](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/tests/TestActuator.py) exercises rate/lag/failure behavior; its header declares GPL version 3 or later | Validation precedent for independently formulated analytical SDK tests. No test code copied or executed; upstream tests do not validate this SDK installation |
| MIXR gimbal | [IGimbal implementation](https://github.com/doughodson/mixr/blob/master/src/models/system/IGimbal.cpp); [official license](https://www.mixr.dev/license.html) publishes LGPL version 3. [Downloads](https://www.mixr.dev/downloads.html) distinguish Linux-only developmental 26.10 and release 18.01 | Reference for angle wrapping, rate/position limits and gimbal modes. No immutable commit/executable pairing was verified; runtime adoption remains deferred. Kinematic logic supplies no calibrated seeker electronics |
| Homing-loop structure | Palumbo, Blauwkamp and Lloyd, [Basic Principles of Homing Guidance](https://www.jhuapl.edu/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf), JHU/APL Technical Digest 29(1), 2010, Rev2018 PDF; printed pp.29-32, 35-36 | Engineering reference for LOS reconstruction, seeker stabilization/filtering and separate autopilot/airframe response. Paper examples establish no equipment constants or software reuse rights |
| Continuous-time event reference | [SUNDIALS/CVODE v7.4.0](https://github.com/LLNL/sundials/releases/tag/v7.4.0), release-page commit abbreviation `8e17876`; [BSD-3-Clause](https://github.com/LLNL/sundials/blob/v7.4.0/LICENSE). [Rootfinding docs](https://sundials.readthedocs.io/en/v7.4.0/cvode/Mathematics_link.html#rootfinding) warn that roots without a sign change may be missed | Optional offline synthetic-trajectory oracle. Proximity tangency needs explicit closest-distance/equivalent reference handling. JSBSim remains the native flight integrator; build/executable qualification is pending |
| Geometry oracle | [BRL-CAD downloads](https://brlcad.org/download/) list 7.46.0 source and 7.42.2 Windows binaries. [NIRT](https://brlcad.org/docs/cad/articles/nirt.html) describes scripted region, entry/exit and normal reports. [COPYING](https://github.com/brlcad/BRL-CAD/blob/main/COPYING) distinguishes LGPL libraries/applications and separately licensed scripts/data/docs | Consider offline synthetic geometry comparisons. No archive hash, immutable source pin or executable was qualified. Units, transforms, ray origins and stable region IDs must be explicit |
| Evidence structure | [NASA-STD-7009B](https://standards.nasa.gov/sites/default/files/standards/NASA/B/1/NASA-STD-7009B-Final-3-5-2024.pdf), sections 4.2.3-4.2.8 | Useful guidance for separate implementation/solution verification, empirical validation domains and uncertainty records. It supplies neither equipment data nor this project's acceptance criteria |

BRL-CAD's [release history](https://github.com/brlcad/BRL-CAD/blob/main/NEWS)
records removal of bundled `burst` in 7.38.0 after deprecation in 7.32. Plan
around maintained geometry queries rather than assuming that historical utility
is present. No new library has been selected as an SDK runtime dependency.

## 3. Inputs required for calibrated finite-fin response

| Input | Required content |
| --- | --- |
| Accepted flight plant | Exact variant and admitted Mach/altitude/dynamic-pressure/angle envelope; forces, moments, reference geometry, axes, signs, units, provenance, uncertainty and extrapolation limits |
| Evolving vehicle | Mass, CG and inertia versus propulsion state; consistent moment-reference/CG conversion and accepted ignition/reset behavior |
| Stability/damping | Static and rate derivatives with their normalization and envelope; these describe the uncontrolled plant |
| Control authority | Forces/moments versus each control deflection, mixing/sign conventions, actuator-to-surface mapping, trim and attainable limits. Confirm explicitly whether the M02 handoff supplies these quantities |
| Actuator/controller | Stops, positive/negative rates, lag/bandwidth, latency, load dependence, initialization and independent response evidence; library example values remain examples |
| Seeker | Measurement/sensor assumptions, body/gimbal conventions, sampling/latency, stabilization, rate/position limits, validity/noise, dropout/reacquisition references |
| Integration | Native cadence and execution order, permitted sensor/uplink inputs, deterministic RNG streams, generator ownership and public/installed/Viewer evidence |

Static convergence, maximum-G declarations and successful encounters cannot
substitute for these response inputs. No new equipment value was admitted.

## 4. Proposed independent validation cases

The following are future fixture proposals, not implemented or passing results.
Synthetic parameters must be identified explicitly and tolerances frozen before
calibration or integration.

| Case | Independent observable and expected failure detection |
| --- | --- |
| Actuator/filter response | Analytical step/ramp and frequency response; asymmetric rates, hard stops, saturation recovery, reset initialization and timestep convergence |
| Fixed inertial LOS, rotating body | Body rotation must not create a false inertial LOS rate; distinguish body/gimbal coordinates, stabilization and measurement latency |
| Gimbal limits/dropout | Angular ramps below/above admitted slew limits, position boundaries, invalid observations and reacquisition without truth updates |
| Achieved plant response | With an accepted plant, measure actual acceleration, delay, overshoot and saturation through JSBSim controls/aerodynamics, without ideal replacement force |
| HC-54 aligned decoy samples | Compare stored poses with the current analytic motion law at matching boundaries; reject future observations and preserve an earlier valid encounter before in-step expiry |
| Temporal edge geometry | Closest approach before/after arming, before/at expiry and exact tangency. A sign-change-only event detector cannot cover every case |
| HC-55 separated geometry | Rays crossing only a separated section no longer shield or damage the parent; retain stable source identities and isolated shared resources |
| Independent ray comparison | Synthetic boxes/panels against a qualified NIRT executable, explicit units/transforms, grazing rays and inside origins; this validates geometry, not failure probabilities |
| Conservation/lifecycle | Reconcile skin/component linear and angular impulse after drag; compare fresh/reset runs, worker counts, unrelated Worlds and deterministic repeat bursts |

HC-54/HC-55 preparation can use current motion/section state before M02
calibration. A future write window still requires a reviewed scope and native
regressions. Detached-body damage needs an explicit contract within the admitted
entity scope.

## 5. Recommended next handoffs

First freeze HC-54 sample/expiry semantics and HC-55 active-geometry membership
as separately reviewable features. Retain source indices and deterministic
iteration instead of compacting away identities used by damage state. These
are engineering proposals from the project audit, not external-tool claims.

The current priority is generator-owned finite FCS and actual fin force/moment
response, using the existing authored control geometry and explicitly estimated
actuator parameters. Missing measured data does not justify preserving ideal
force/moment injection. Preserve the actuator reset initialization patch and
verify the actual release/failure/reset path. Admit the M02 plant and measured
control-response inputs separately before claiming variant accuracy. Measure
cost on the native path rather than inferring it from component counts.

Calibrated active/SARH/IR observations retain their sensor-input prerequisites.
Weapon flyout feedback and fused input additionally need the
[modeled delivery and owner handoff](weapon-feedback-validation.md), including
sample/receive times and explicit loss semantics. M02 success would not supply
those contracts; referee records remain outside actor observations/action masks.

Source tags, archives/executable hashes, platform support and upstream reference
cases must be qualified when an external campaign starts. The source evaluation
above is research, separate from the finite-control implementation and its
software checks; it produces no physical acceptance result.
