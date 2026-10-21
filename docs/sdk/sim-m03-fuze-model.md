# Explicit captured-motion fuze model

The SDK supports an explicit generic ideal turning-point CPA law and a configured
initiation delay. These are simulator model choices, not calibrated AIM-120C-7
RF sensing or FZU-49/B hardware timing. Native flight remains the only motion
integrator; these helpers consume captured motion and never generate it.

## Card and geometry contract

The independent numeric card block uses `fuze_model_version=1` and
`fuze_proximity_algorithm=1` (unit `1`), `fuze_arming_s` and `fuze_delay_s`
(seconds), and `fuze_radius_m` plus `fuze_station_x_m/y_m/z_m` (metres).
`configureGuidance` validates and admits the complete block atomically; see
[control configuration](sim-m03-control-configuration.md). The station is in the
authored structural frame: x aft from the nose, y right, z up. The card owner
records provenance and uncertainty separately from this software contract.

World must transform the fixed sensor point with each native sample's model
origin and attitude, using the same captured CG epoch for the offset. The
resulting ECEF point samples, and actual captured target-reference samples, are
the input to the fuze. A moving CG is not the authored sensor station. Contact
geometry and the warhead's burst source are separate material points; selecting
a sensor point does not move either of them.

Absent versioned configuration retains the unversioned interval-minimum law.
That legacy law may initiate at a clamped active-interval endpoint within the
radius. Explicit V1 deliberately changes this timing, even when its configured
delay is zero. Unknown version/law or malformed geometry fails closed.

## Actual CPA rather than approach-only overlap

`Fuze.h::sweptFuze` handles one captured affine relative segment.
`MotionHistory.h::sweptPathFuze` merges the actual own/target sample boundaries.
V1 accepts an actual stationary point of relative squared range within a
captured active segment, or a captured corner where closing changes to opening
or a stationary plateau. A still-approaching endpoint inside the radius is not
an actual CPA. Stationary overlap without preceding approach establishes no
turn. No unobserved future root is clamped into a trigger, and a short target
history is not extrapolated.

The first qualified captured turn initiates. A later, closer encounter cannot
replace that earlier decision. An earlier turn outside the radius does not
suppress a later qualified one. Arming is inclusive; weapon life and an
explicit exclusive native cutoff remain exclusive. An excluded CPA is never
nudged backward into an initiation. Non-triggering interval-minimum geometry may
still be returned for referee miss-distance diagnostics.

Redundant samples on the same piecewise affine path preserve the selected
turn. An exact stationary point at a captured segment's final endpoint is
represented and eligible when that endpoint is live and inclusive. A corner
with nonzero closing rate at the decision endpoint requires a later captured
opening segment to confirm it.

`FuzeTurnStateV1` retains only the preceding actual relative segment. At the next
decision it admits a shared corner only if target identity, absolute launch-age
boundary, and the captured own/target boundary positions match exactly. It then
returns current-window fraction zero for that actual shared epoch. Target
switch, missing interval, position discontinuity, unknown identity or `reset()`
discards this transient history. It never bridges the gap. A previously returned
endpoint trigger is not returned a second time from the retained segment.

The caller must provide one consistent captured launch-age timeline: form a
window duration from its admitted endpoint ages and retain the exact shared
boundary rather than recomputing it through independently rounded clocks.
`fraction` always refers to the duration passed to the helper. These contracts
establish partition invariance for the same captured piecewise path with its
retained boundary history; they do not assert that changing the flight integrator
or observation cadence produces the same physical trajectory or observations.

## Delayed-initiation lifecycle

Each release owns a separate `FuzeDelayStateV1`. `schedule(triggerAgeS,targetId,
spec)` stores the first admitted captured trigger, original target identity,
and configured nonnegative delay. It rejects unknown model/law, invalid target,
pre-arming/expired trigger, nonfinite delay or timestamp overflow. Scheduling
does not infer a future position and cannot replace an existing trigger.

Call `due(ageBeforeS,durationS,spec,eligibleEndS,exclusiveEnd)` first in the
trigger's containing captured window and then once in every contiguous captured
weapon window. This continues through seeker loss or a target switch. Its
result distinguishes invalid input, pending/not due, and due. Only a due result
contains a detonation age and local fraction; it retains the original target.
Pending results have no usable detonation fraction. World must obtain the
actual native pose and warhead point velocity at the returned age.

A due time must be inside the supplied captured window and precede weapon life
and any exclusive cutoff. An inclusive live endpoint is eligible. Missing
windows, changed timing configuration, malformed input or a known earlier
native cutoff fail closed and seal the pending channel. Expiry cancels a due
time at or beyond life. A due result consumes the channel exactly once.

`cancel()` seals it after earlier authoritative contact/termination; it does not
permit another target to schedule. `reset()` is for a new release only. Resetting
transient seeker/turn history must not clear a pending initiation, and the
existing `Vehicle.reset` authority/lifecycle contract is unchanged. World owns
the final earliest-contact/proximity/due-time arbitration and terminal events.

## Methodology and verification scope

[OpenEaagles `Missile::weaponGuidance`](https://github.com/doughodson/OpenEaagles/blob/master/src/models/player/Missile.cpp)
uses closing-to-opening range-rate history before evaluating CPA, and clears
that history when a target changes. It is a public simulation methodology
reference, not an AMRAAM sensor model. This implementation independently uses
captured piecewise motion and bounded epochs; it does not copy the reference's
unbounded backward interpolation, default radius, or time-of-flight constants.

[MIL-HDBK-1211(MI), section 2-2.4 and Fig2-19](https://quicksearch.dla.mil/qsDocDetails.aspx?ident_number=116871)
explains the relationship between static/dynamic warhead patterns and fuze
timing. It does not prescribe this ideal CPA law or supply modern equipment
delay parameters. The authored zero delay is an idealization, not a measurement.

The root captured all twelve original exact V1 selectors as behavioral RED in
`build/architecture-reviews/sim-m03-fuze-model-red-{1..12}.log` and its cases JSON.
The independent tests retain all fifteen legacy cases and cover approach-only
endpoints, arming/lifetime/cutoffs, earliest turn, short histories, unknown
dispatch, native/split corners, history gaps and target changes, positive/zero
delay, once-only consumption, cancellation, overflow, and window partitioning.
Native GREEN and World/card integration acceptance are owned by the root gate;
this document alone makes no passing or fixed-scene kill claim.

The later root-owned [scoped regression](sim-m03-five-km-repair.md#current-scoped-regression-evidence)
records passing fuze and contact-native gates on both current presets, including
the canonical capture-epoch case. This software result does not close the
remaining fragmentation/physical-model gates or establish equipment fuze data.
