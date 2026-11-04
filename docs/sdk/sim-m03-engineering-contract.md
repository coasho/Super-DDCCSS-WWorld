# SIM-M03 complete engineering contract

SIM-M03 applies to all 53 maintained store designs and their authored loadouts
across the 35 current airframe configurations. The executable homing population
is 24 AAM variants; the explosive population also includes CJ-10A and Mk82.
The other 27 configurations retain explicit applicability rather than receiving
invented seekers, fuzes, warheads or radios. SIM-M01 sensing/fusion and SIM-M02
flight/geometry remain the upstream authorities.

All applicable component selections use [equipment-specific model identities](equipment-model-names.md).
Software interface/backend revisions describe mechanisms separately; they are
not missile or component hardware variants. Missing exact OEM part numbers
remain unknown rather than receiving invented designations.

The engineering acceptance rule follows the accepted M01/M02 distinction:
complete applicable mechanisms, independently checked methods, native execution,
public/installed integration, visual presentation and reproducible delivery are
mandatory. Empirical equipment accuracy remains a separate uncertainty ledger.
An unavailable equipment calibration cannot waive a failed implementation or an
unexecuted current model. The 28 previously accepted M02 rotor deferrals retain
their exact policy hash and are never counted as passed M03 tests.

| Component or function | Executed mechanism and evidence route |
| --- | --- |
| IR/IIR/SARH/ARH sensing | [V3 RF/radiometric receiver](weapon-sensing-engineering.md), finite gimbal/search, exposure/processing/publication clocks, noisy association and modeled covariance; no identity-assisted detection |
| Midcourse support and navigation | [Finite release-bound transport](weapon-links.md), accepted M01 source/covariance/age, actual serialization/delivery/loss and [sampled inertial/GNSS estimate](weapon-navigation.md); no launcher truth refresh |
| Guidance, control and actuation | Measured-history guidance, delayed estimate prediction, finite load/rate feedback, current authored fin allocation/lag/rate/stops, [bounded operating-point control](weapon-operating-point-control.md) and selected physical thrust vectoring; JSBSim owns the forces and integration |
| Propulsion | Native authored solid phases, actual commanded reserve pulse for PL-15/R-37M, Meteor booster/ducted sustainer and CJ-10A turbofan/fuel/CG; [pulse policy and uncertainty](weapon-pulse-energy.md) |
| Safe-arm and initiation | Captured native segment contact/proximity/terrain/delay/lifetime ordering, one-shot initiation, arming-time and inert-jettison controls; preserve original hit and crew semantics |
| Warhead and damage | [One conserved source field](sim-m03-effects-quality.md), global ordered interception, finite fragment/rod propagation and component/material response, causal event identity and detached native bodies; [consistent mass partitions](mass-partition-consistency.md) and dissipative full-angle detached-body/skin-drag forces |
| Countermeasures | Native chaff/flare cartridge consumption, finite lifetime, drag/gravity/air-relative transport and actual receiver response; no outcome-dependent deception rule |
| Actor access and assessment | Additive C++/C99/Python opt-in facade, scoped controller/instance identity, bounded non-consuming reports, finite request/cancel/expiry/terminal retention and explicit optional process proxy |
| Presentation | Native pose/fin replay and actual primary fullscreen foreground 1x views with Earth imagery, terrain, clouds, sunlight/shadows and complete native effects; misses retain their native outcomes |

Retained V2 seeker IDs and timed motor tables keep their own meanings. The old
immediate-observation fixture uses four explicitly retained V2 profiles in an
isolated test-only asset tree; current V3 cards are exercised separately at
frameSkip1/2/4 and through installed public releases. No V3 factory is relabeled
while executing an ideal V2 receiver. The generic unavailable-only
`engagement.reports.v1`, default actor callable lists, old public layouts, enum
values and provider vtables remain frozen. The explicit weapon-link extension
does not assert that another owner's generic gateway handlers are installed.

The resolved navigation estimator includes the held IMU error in its covariance
state. It uses the same discrete exposure model for realized draws, fractional
propagation and GNSS Joseph updates. Independent sampled matrix exponentials,
impulse-response sums, fractional aiding and realized error populations check
that contract. A separate continuous limit is a convergence reference, not an
exact covariance claim for a held measurement.

Current execution includes the [radiometric focal plane and quadrant receiver](weapon-thermal-image.md),
[finite RF signal processor](weapon-radio-receiver.md) and
[coupled body-IMU strapdown estimator](weapon-navigation-strapdown-v2.md).
Those production mechanisms supersede the earlier scalar-image and resolved-ECEF
approximations on explicitly selected current cards. Their current integration,
all-model runtime and final presentation evidence must still pass the complete
campaign. Model-order limits remain explicit: projected opaque rectangles and
a Gaussian PSF approximate thermal shape/optics; SARH uses admitted target RCS
as a bistatic proxy; IMU thermal/scale-factor hardware is not reconstructed;
warhead free-air blast is a reduced pressure/impulse fit without
hydrocode/reflection/wave-propagation fidelity. Proprietary receiver,
motor, material and component vulnerability curves are not reconstructed from
nominal names. Their assumptions have nonzero uncertainty. No comparative DCS
benchmark or real-equipment error percentage is inferred from these scenes.

The normal reproduction entry is
[the complete campaign](../../tools/validation/weapon_quality/README.md).
The reviewed `Engagement.cmd` normal entry runs the native F-16C/AIM-120C
encounter in one complete fullscreen Viewer until it is closed. It records
nothing and imposes no 40-second play limit; separate acceptance runs retain
their bounded recordings and original outcomes. Closing the Viewer ends only
that launcher's owned World. Build the normal Release preset before using it.
Its final admission joins all actual discovered tests, per-model traces,
installed product identities, exact source replay, independent source review
and hash-bound actual image review. Only a successful completed receipt can
declare this contract accepted; a design page or successful subset cannot.
