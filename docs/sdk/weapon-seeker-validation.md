# Weapon seeker observation consistency (SIM-M03)

This package corrects the native seeker's observation envelope, body reference,
invalid-input handling and unavailable timing reports. It preserves the existing
encounter assumptions. It does not establish calibrated seeker, infrared-signature
or missile-autopilot performance. Full SIM-M03 acceptance remains open under the
[simulation plan](../architecture/modules/simulation/plan.md) and
[work allocation](../architecture/modules/simulation/ms-work-allocation.md).

## Scope frozen before implementation

- Date: 2026-10-20; baseline: main
  `0d4f25a5df3fdd23696e696644cbd0ee32ab23af`.
- Native core: `src/simulation/mission_systems/weapon/Guidance.h`,
  `Guidance.cpp`, and header-only `Seeker.h`.
- Core regression: `tests/simulation/test_weapon_guidance.cpp`.
- Runtime integration: `World::prepareEngagements`, `World::seekerTarget` and
  their native encounter regressions. The SDK remains the simulation authority;
  JSBSim remains the sole released-weapon flight integrator.
- Public C++/C99/Python layouts, enum numbers, permissions and method signatures
  remain unchanged. The new input fields are private simulation-core contracts.

The existing native implementation is reused. This correction adds no physical
solver, runtime dependency, fin-response model, random stream or equipment
calibration. Radar processing, propulsion/aerodynamics and future physical IR
signatures retain their existing owners and separate acceptance requirements.

## Observation envelope and body reference

`infraredAcquisitionRangeM(baseM, aspectCosine, afterburning)` centralizes the
existing approximation:

```text
aspect = 0.5 * (1 + clamp(aspectCosine, -1, 1))
reach_m = base_m * (0.45 + 0.55 * aspect) * (afterburning ? 1.35 : 1)
```

`aspectCosine` uses the target's velocity direction and the observer-to-target
ray. These factors are inherited engineering/game assumptions, not measurements
of a particular seeker or aircraft signature. A nonfinite input, negative base
range or arithmetic overflow yields zero admitted reach. No new real-equipment
parameter is inferred.

On-rail and released observations use the same reach calculation. For example,
the existing 13 km baseline admits 17.55 km at maximum tail aspect with the
afterburner multiplier. Previously, an on-rail observation could be admitted
there while the released seeker rejected every observation beyond 13 km.

An observation can carry `admittedRangeM`, the current caller-filtered sensor
reach. Guidance independently checks the actual geometric distance against
that reach. NaN means the field was not supplied and retains `Spec::acquireM`;
an explicit negative or infinite value invalidates that observation. Caller
admission cannot replace a current observation, gimbal eligibility, terrain,
aspect, radar or decoy filtering. Loss of the current observation still clears
`tracking`; the last `holding` identity remains historical telemetry.

`Input::boresight`, when supplied, is the missile's body-forward direction in
ECEF. It defines the gimbal independently of translational velocity, including
at zero speed. It must be finite and nonzero. Missing boresight preserves the
core's legacy velocity-axis approximation for existing internal callers. This
adds no gimbal slew rate, antenna servo or acquisition dwell assumption.

## Timing, invalid inputs and reports

An update with negative or nonfinite launch age or step duration is refused
before any estimate, activation, cue or accepted-age state changes. A launch age
older than the last accepted update is also refused. Repeated observations at
the same launch age remain supported; zero duration is valid because this core
does not integrate motion or servo dynamics.

Every physical `Spec` field must be finite. Acquisition range, acceleration
limit, lifetime, arming delay and fuze radius must be nonnegative; the gimbal
angle must lie in `[0, pi]`. Finite loft values retain the established clamping
behavior. Invalid specifications neither generate steering nor authorize arming.
An explicitly invalid boresight refuses the update without a velocity fallback
or state mutation.

| Report | Available value | Unavailable value |
| --- | --- | --- |
| `timeToGoS` | Range divided by positive closing speed for the accepted prediction | NaN without a prediction or positive closing speed |
| `timeToActiveS` | Active radar: zero once active; otherwise estimated time to the activation boundary for a closing prediction | NaN for a nonclosing/missing prediction or a seeker without an active-radar transition |

Refused and unguided updates report unknown timing. Unknown values propagate
through the existing `WeaponRecord` fields and public sized-copy paths; they do
not become fabricated zero-second estimates. Gateway JSON must represent an
unknown numeric report as null under its existing availability contract.

## Regressions and evidence

The `weapon_guidance` executable contains 17 internal cases. Existing cases
retain activation/reacquisition, stale-estimate extrapolation, SARH illumination,
IR observation requirements, bounded acceleration, swept proximity/arming,
unguided stores, decoy discrimination and on-rail cue behavior.

| Added case | Required evidence |
| --- | --- |
| `invalid duration` | Negative, NaN and infinite duration refuse a future update; the next valid update matches an untouched reference |
| `invalid launch age` | Negative, NaN, infinite and reversed age cannot replace the accepted estimate; equal-age observations still work |
| `invalid spec` | Invalid physical fields and impossible gimbal angles cannot steer or arm |
| `unknown timing` | Missing/nonclosing predictions remain unknown; active radar reports a completed transition; IR/SARH do not invent that transition |
| `infrared reach` | Existing head/tail/crossing-aspect factors and afterburner reach are preserved; invalid/overflowing inputs fail closed |
| `observation range` | The 15 km afterburner observation survives release with a 13 km baseline; actual geometry beyond admitted reach, invalid reach and an unset/narrower envelope remain rejected |
| `body boresight` | Body-aligned observations remain eligible despite crossing velocity; velocity-aligned observations outside the body gimbal and stationary off-axis observations are rejected |
| `invalid boresight` | Zero/nonfinite explicit directions refuse atomically and leave the prior estimate and time state intact |

Before production changes, the four existing-interface regressions `invalid
duration`, `invalid launch age`, `invalid spec` and `unknown timing` each failed
against unchanged production with exit status 1 and the intended assertion.
The integrated `weapon_guidance` and native engagement fixtures passed in the
9/9 Headless core selection. The related public, Python, installed-consumer and
boundary selection passed 8/8. The [package validation record](weapon-fidelity-validation.md)
supplies the exact commands, internal counts, logs and scope limits.

## Remaining fidelity gates

- Active seeker detection still requires accepted radar inputs for frequency,
  power, aperture, processing, noise, clutter, measurement error and scheduling.
  The current range/association approximation is not a calibrated seeker radar.
- SARH needs per-candidate received illumination and bistatic geometry; one
  launcher illumination flag does not establish a received return at every
  candidate.
- IR needs sourced band-dependent signatures, atmospheric transmission,
  background/noise, acquisition and tracking limits. The shared aspect helper
  is an envelope consistency correction only.
- Gimbal rate, scan timing, measurement latency and classification remain
  uncalibrated. This package adds no assumed default dynamics to conceal those
  missing inputs.
- Fin/actuator/autopilot response needs accepted SIM-M02 force/moment derivatives,
  mass/CG/inertia evolution and admitted control-response references. The current
  ideal external-force/moment path is not evidence of those properties.
- Fuze and lethality assumptions, model-specific uncertainty, independent
  encounter references and complete fleet coverage require separate acceptance.
  Detonation, native damage, crew loss and mission outcome retain distinct
  meanings.
