# Native fuze timing and fragment momentum

This SIM-M03 slice corrects numerical execution of the existing assumed model.
It does not calibrate an equipment fuze, seeker, autopilot or warhead. Its starting
main commit is `0d4f25a5df3fdd23696e696644cbd0ee32ab23af`.

## Fuze convention and temporal validity

The private, header-only `ms::weapon::sweptFuze` evaluates linear relative motion
of the weapon and its held observation over one native step. The caller supplies
both launch ages and both bodies' step endpoints. It restricts the sweep to
inclusive arming and exclusive battery expiry, `[armingS, lifeS)`.

The result distinguishes invalid inputs from a valid sweep without a burst.
Invalid inputs, inverted or zero-width step times, negative launch ages, invalid
arming/lifetime/radius values, nonfinite values and overflowing geometry fail
closed. A valid step without an active interval has unavailable closest-distance
and fraction values. Otherwise the fraction refers to the original step, even
when the eligible interval begins partway through it.

The existing native convention places a proximity burst at closest approach in
the eligible interval. It does not move the burst to first entry into the radius.
This preserves the meaning of miss distance and the geometry consumed by native
damage. Contact reached only at expiry cannot burst. An overlap that exists
before expiry can burst, using a representable fraction whose reconstructed age
remains below expiry. The helper changes no arming time or radius parameter and
does not itself authorize an inert release or create a current observation.

Native event timestamps remain at the step boundary. A swept fraction selects
the encounter position; it is not a new public substep timestamp. One terminal
outcome per weapon, inert jettison behavior and existing refusal/permission paths
remain part of the engagement contract.

## Captured initial stationary points and retained boundaries

The explicit version1 stationary/corner convention differs from the retained
version0 constrained-interval minimum described above. It admits an actual
analytic stationary point at either inclusive captured endpoint, subject to
the same inclusive arming and exclusive expiry rules. An approaching or
receding minimum merely clamped to that endpoint remains insufficient.

A standalone `sweptPathFuze` now preserves an exact initial stationary point
already admitted by `sweptFuze`. For the synthetic captured relative path
`r(u)=(u,0.5,0)`, squared separation is `u^2+0.25` and its derivative is `2u`.
The stationary point is exactly `u=0`, with separation0.5, rather than a
stationary point outside the interval clamped to its boundary. This numerical
contract introduces no equipment parameter, sensor capability or trajectory.

`FuzeTurnStateV1` owns the shared endpoint whenever its same-target, same-age
and exact-position history is contiguous. An internal initial-point inclusion
policy leaves the public path signature unchanged: the history-aware call
checks that shared point from both captured sides and suppresses it if the
preceding interval already reported it. Suppressing this one point does not
stop the current path's search for a later legitimate stationary point or
corner. A receding-to-tangent shared boundary is not promoted into a two-sided
closest approach. No timestamp is nudged to implement these decisions.

The bounded regression additions cover direct/path/first-release consistency,
an already reported stationary endpoint, a later turn after that endpoint,
a clamped receding negative control and a contiguous receding-to-tangent
negative control. Original arming, expiry, target/history discontinuity,
fixed-delay and retained version0 assertions remain unchanged. Current native
normal-preset execution is required separately; standalone defect reproduction
does not establish integrated or complete-program acceptance.

## Fragment impulse accounting

`damage::assess` already slows fragments in air and uses their resulting speed
for skin penetration and component impact energy. Target impulse now accumulates
only velocity changes at skin penetration and component stopping. Momentum lost
to aerodynamic drag before or between impacts is not applied to the aircraft.
Angular impulse uses that same transferred momentum along each fragment's line.
Blast impulse and all assumed fragment, material, damage-probability and weapon
parameters remain unchanged.

The regression uses one synthetic fragment stopped by a component, without blast
or skin. Its independently reported impact energy determines stopped momentum
through `p = sqrt(2 m E)`. These synthetic values describe no real equipment.

## Evidence and limits

Before implementation, the new fragment-momentum regression failed its impulse
assertion against the existing implementation. The new temporal-fuze test target
failed to build because the proposed private helper did not exist. Current
verification passed the integrated temporal-fuze and native engagement fixtures
within the 9/9 Headless core selection. The complete native damage tag passed
20 cases and 4,814 assertions. The [package validation record](weapon-fidelity-validation.md)
records commands, logs, public/installed-consumer checks and scope limits.

The temporal cases cover a wholly safe step, arming-crossing sweeps, inclusive
arming, expiry-only radius contact, pre-expiry overlap, expired intervals,
stationary/moving targets and malformed/nonfinite/overflowing inputs. Native
integration must additionally preserve the existing release, seeker eligibility,
detonation pose, damage, inert-jettison and terminal-event regressions.

These corrections reuse the existing analytic relative-motion and fragment
models; they add no solver or runtime dependency. A physical calibration remains
open. The maintained [engagement assumptions](engagement.md#modeling-assumptions)
identify assumed fuze radius and one-second arming. The existing warhead model
labels its case/fragment geometry and fragment spray as estimates, including a
game-reference spray distribution. Damage-state probabilities and several
structural/fire parameters are also explicitly assumed. No admitted
variant-specific safe-arm law, proximity-sensor response/delay, fragment
mass/angular distribution, vulnerable-component conditional response or
independent encounter validation dataset is supplied by this slice. General
Gurney, blast and penetration methods cannot establish those equipment-specific
values. No new parameter is invented to close these gaps.

Public C++/C/Python signatures, layouts, enum values and result meanings are
unchanged. `detonated` is a fuze/warhead event, `damage.destroyed` reports native
crew loss, and neither alone establishes mission success. Weapon/referee truth
does not become actor feedback through this numerical correction.
