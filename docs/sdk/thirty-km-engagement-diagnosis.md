# Thirty-kilometre engagement diagnosis, 2026-10-20

Baseline: main `fabeaa4fe56d1a878a5f3d0614729d8e940d5719`.
The user requested cause isolation without changing parameters to obtain a hit.
The [write/build allocation](../superpowers/plans/2026-10-20-thirty-km-diagnosis.md)
records the implementation window. This is a software diagnostic using the
existing estimated model, not an admitted real AIM-120 envelope.
The measurements below belong to that historical diagnostic handoff. Current
expanded-source verification and fixed-scene hit acceptance are recorded in the
[SDK delivery continuation](sim-m03-reference-continuation.md#7-sdk-delivery-continuation-and-regression-repair).

## Result and causal controls

The dominant demonstrated cause is a controller implementation error: the
linear inverse omitted nonlinear forces/moments already present in the native
plant. A separate ARH search-state guard also interrupted valid estimate-driven
steering. After those corrections, the unchanged native model reached eligible
proximity detonation in this exact case. That establishes neither target damage
nor a passed hit-acceptance gate, and does not isolate a SIM-M02 motor or
aerodynamic defect.

All four runs use the existing 30 km head-on fixture, 6000 m setup, 250 m/s
aircraft, seed 17, native release/consent, target designation and original model.
There are no gain, response-time, incidence, deflection/rate, motor, drag,
activation, arming, lifetime or fuze-radius changes.

| Software configuration | Native terminal outcome | Eligible swept miss distance |
| --- | --- | --- |
| Baseline | Impact after missing | 15.0118 m |
| ARH search steering correction only | Impact after missing | 14.4009 m |
| Native-load controller correction only | Detonated | 6.62639 m |
| Both corrections, initial diagnosis before the Euler-coupling follow-up | Detonated | 8.12776 m |

The control-only run temporarily restored the original single search guard;
the delivered source restores both corrections. No experimental runtime switch
is added. The delivered case produces identical terminal records and complete
event streams with one and two workers. The combined result is retained even
though another experimental configuration has a smaller miss: the objective is
correct execution, not minimization of this fixture's miss distance.

Do not compare these swept distances to the coarser minimum endpoint ranges.
The final live sample in the delivered case is still 30.9521 m from the target;
the native substep sweep finds the subsequent 8.12776 m encounter before the
next World snapshot. No radius expansion or geometric hit fallback is used.

## Timeline and category separation

| Category | Measured baseline evidence | Disposition |
| --- | --- | --- |
| Motor/energy | Motor thrust ends around launch age 8.0667 s; maximum speed is 780.622 m/s. The first miss occurs around 35.15 s while speed is about 417 m/s. | Burnout precedes normal coast; it does not explain loss of steering. The corrected model, unchanged, peaks at 778.286 m/s and reaches eligible proximity detonation during coast. The speed difference follows a different controlled trajectory, not a motor edit. |
| Guidance state | ARH activates at 11.1667 s, searches without a seeker hold until acquisition at 16.7333 s, and remains tracking through the first closest encounter. It loses track only at 35.2000 s. | Post-pass loss is a consequence, not the cause of the initial miss. The supported search interval contains an independent zero-command defect. |
| Control execution | Fins move and remain finite. Before late guidance saturation, body incidence stays below the 15-degree bound and requested elevator is well below its stop, yet native acceleration materially undershoots demand. | The inverse omits nonlinear load terms already present in the native plant. Correcting that bias permits proximity detonation with the same limits; actual damage-hit acceptance remains a separate gate. |
| Native model fidelity | The baseline already includes nonlinear crossflow and finite-fin geometry, but its parameters are estimates. Identical model bytes permit proximity detonation after the software corrections. | No specific SIM-M02 model defect is established by this case. Real response, derivative, mass/CG/inertia and per-variant acceptance remain pending. |

## Guidance correction

`weapon/Guidance.cpp` returned immediately for every `Seeking` output. ARH
activation therefore removed all acceleration demand, including gravity
compensation, even while a permitted launcher estimate remained valid. The
search interval lasted about 5.57 seconds in the baseline.

ARH now continues PN/gravity steering from its permitted estimate, including
the established inertial extrapolation after support loss. `Seeking`,
`tracking=false`, the unchanged holding value and explicit support availability
remain truthful. Only a current eligible seeker observation can establish a
hold or authorize fuze processing. Existing IR/SARH observation requirements
are preserved. No loft/activation threshold or navigation gain is changed.

The new independent regression first failed on absent gravity compensation.
It now checks supported search, crossing-estimate steering and unsupported
extrapolation without manufactured support or a seeker observation.

## Controller correction

The old inverse used only `Fa * incidence + Fd * deflection` and
`Ma * incidence + Md * deflection + Mq * rate`. Native body crossflow,
nonlinear angle functions, finite-fin projection and deflection drag were
already integrated by JSBSim. These terms do not disappear because the inverse
omits them.

For example, at the baseline's completed native frame around 33 s, the signed
linear pitch approximation predicts approximately **+357.20 N m**, whereas
the actual native aerodynamic moment about the current CG is only **+2.73 N m**.
The simultaneous normal-force residual is about **-537.37 N**. These residuals
include existing nonlinear body/fin contributions; they are not attributed to
one isolated coefficient. Native pitch `Cm` already contains
`-37.87226 * sin(alpha) * abs(sin(alpha))`, and normal force also includes
crossflow. None of these plant terms or coefficients is edited.

The private autopilot now consumes an optional, same-frame own-aircraft sample
of native aerodynamic force, aerodynamic moment about the current CG, and
**actual fin-equivalent positions**. World always supplies it for supported
controlled weapons; missing/invalid feedback fails closed. There is no extra
CG transfer of already-transferred native moments.

For each pitch/yaw channel the additive residuals are:

```
F0 = nativeForce - Fa * incidence - Fd * actualDeflection
M0 = nativeMoment - Ma * incidence - Md * actualDeflection - Mq * rate
incidenceReference =
    (demandForce - F0 + Fd * (Mq * pathRate + M0) / Md) / (Fa - Fd * Ma / Md)
deflectionDemand =
    (inertia * (rateDemand - rate) / rateTime - Ma * incidence - Mq * rate - M0) / Md
```

Roll uses the corresponding measured moment residual. This is local load
accounting, not identification of new effectiveness derivatives. The existing
15-degree incidence bound, 0.12/0.4-second nominal response, finite channel/fin
lag/rate/stops and normalized command limits are unchanged. No integrator,
additional tuning constant, applied force or applied moment is introduced.
Actuator failures and insufficient aerodynamic authority remain physical limits.

An independently integrated synthetic nonlinear plant first settled at
**-9.22426 / +4.84893 m/s2** for demands of **-20 / +8 m/s2** with the old
inverse, despite receiving measured loads. The correction passes six signed
tail/canard and finite-lag variations within the predeclared 0.1 m/s2 bound.
Other regressions preserve the exact linear solution for zero residual, a
constant-load trim at actual fin position, bounded commands and invalid/overflow
refusal. No fixture coefficient is copied into production.

## Evidence, reproduction and limits

Raw evidence is under `build/architecture-reviews/thirty-km-*`: baseline,
guidance-only, control-only and delivered native logs/CSV, independent RED/GREEN,
complete builds, focused preset gates and frozen-source/model provenance.

The opt-in CSV belongs to the native test executable only:

```powershell
$env:FSIM_ENGAGEMENT_TRACE = 'build/architecture-reviews/thirty-km-final'
try {
    & .\build\ucrt64-headless\tests\fsim_engagement_native_test.exe 'thirty-kilometre'
} finally {
    Remove-Item Env:FSIM_ENGAGEMENT_TRACE
}
```

Each row's `own_*` position/velocity/attitude and target/estimate geometry refer
to the World step start; unprefixed state, loads and physical fin positions are
the completed native frame. Replayed guidance demand versus completed-frame
force is a **lagged tracking diagnostic**, not an exact simultaneous measurement
of the internal demand. Linear/native load residuals use completed-frame state,
derivatives, actual positions and CG consistently. The analysis script accepts
`--legacy-search` for the baseline/control-only guard. Trace values never feed
back into execution or actor observations.

The old motor/coast acceptance is unchanged. A separate exact-scene regression
requires native target damage as well as detonation and worker determinism.
The detonation measurements above do not establish that this hit requirement
passes; the unchanged requirement must fail if target damage is absent. This
is not a general launch-envelope gate. The native-load sample uses ideal own
simulation telemetry. Real IMU/load estimation, sample delay/noise, derivative
fidelity, nonlinear/high-incidence coupled flow, hinge loads and calibrated
autopilot response require their corresponding admitted sensor/SIM-M02 inputs.
The local additive residual does not establish those properties. The accepted
SIM-M02 handoff must be independently integrated and this case repeated.

The subsequent [reference continuation](sim-m03-reference-continuation.md#6-historical-frozen-state-verification-2026-10-20)
added independently regressed Euler coupling and fragment-boundary corrections.
That earlier frozen source reproduced 30 km detonation at **8.12775 m** and maximum speed
**778.286 m/s** on both presets, with exact one/two-worker records and event
streams. Its 5 km case detonated at 10.2878 m with zero sampled fragment strikes
and weak blast loading. The earlier contract-only check verified assessment and
event semantics; it did not establish fixed-scene hit acceptance. Current fixed
5 km and 30 km hit regressions retain their actual target-damage requirement.
The separate swept-damage positive control does not replace either requirement.

At that earlier frozen handoff, the complete Headless build passed and its full
inventory passed **744/746** in 1076.82 s, retaining ground-store impact and synthetic package-fixture failures
as HC-57/HC-58. Complete Release build and its focused selection passed
**56/56** in 98.98 s, including installed equipment consumers. The continuation
records exact evidence/provenance and limits. Those historical results do not
verify the expanded SDK delivery source or its restored fixed-scene hit gates.
The [SDK delivery continuation](sim-m03-reference-continuation.md#7-sdk-delivery-continuation-and-regression-repair)
records current verification separately. Full regression acceptance, new Viewer
evidence, physical calibration and complete SIM-M03 acceptance remain open.
