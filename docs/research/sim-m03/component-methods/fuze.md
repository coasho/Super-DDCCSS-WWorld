# Fuze method, initiation lifecycle and primary geometry reference

Recorded method evidence update: the actual24 native source capture and independent Python comparison now exist under the exact axis-methods/effects producer epoch described below. Original documentation row/admission flags remain unchanged pending root integration; they are not a claim that this new method run was absent. Method execution does not establish encounter/public/visual or hardware calibration. Original historical evidence retains its own epoch.


The companion [fuze.json](fuze.json) binds all24 AAMs individually to their
original selector, native mechanism and complete resolved parameter/source
records. There are two current mechanism cohorts, retained swept-path legacy
and turning CPA. The model is a declared reduced-order geometric initiation
model; it is not a simulated OEM RF/optical TDD circuit. A passing method row
does not establish a coupled detonation or target damage.

## Geometry, state and discretization

For captured relative endpoints `r0=own0-target0` and `r1=own1-target1`, let
`d=r1-r0`. A nonstationary segment has an unconstrained minimum at

\[
u_*=-\frac{r_0\cdot d}{d\cdot d},\quad
r(u)=r_0+ud,\quad d_{CPA}=\|r(u_*)\|.
\]

The independent reference is [David Eberly's Geometric Tools point/segment
distance query](https://raw.githubusercontent.com/davideberly/GeometricTools/master/GTE/Mathematics/DistPointSegment.h),
file version8.0.2025.05.10, operator lines42-85. It handles non-unit segments,
clamped endpoints and stationary segments. Our checker computes the projection
with exact `Fraction` endpoints and evaluates the final square root independently.
This source establishes distance geometry, not a fuze, arming or target-detection
law. The original file is Boost Software License1.0; no source is vendored or
executed by this checker. The downloaded bytes and their scope are recorded in
the primary-source manifest.

The actual SDK merges native captured own/target path knots and evaluates each
piecewise-linear relative segment. It does not propagate a target into a future
window. Fractions refer to the original captured decision window. The armed,
live interval is `[armingS,lifeS)`; arming is inclusive and expiry exclusive.
Malformed/nonfinite geometry and illegal model combinations are refused.
Coverage is version-specific: V0 retains target hold-at-last-pose semantics,
while V1 also truncates to target-path end. World owns physical removal/contact
cutoffs; neither branch authorizes inventing future moving geometry.

Legacy V0 takes the constrained minimum on the active interval, including
eligible endpoints. Turning V1 initiates only at an actual stationary point or
a closing-to-opening path corner in the active interval. A clamped approach
endpoint inside the radius is not a V1 turn. Stationary relative overlap alone
also supplies no V1 turn. Its cross-decision state stores the previous exact
boundary pose/time, relative movement and generation/target identity. Gaps,
target changes and noncontiguous captured paths invalidate that history; a
history reset is not a new release or a rearming command.

For V1, the delayed-initiation state is separate: Fresh -> Pending -> Sealed. Scheduling
captures the authoritative trigger launch age, fixed target ID and declared
delay. Due is authorized only in the containing or exactly contiguous live
captured window. A cutoff/contact/expiry cancels pending initiation rather than
inventing future pose. Delay configuration changes and time gaps refuse. A
completed/cancelled release cannot schedule again; only a new release resets it.

Physical contact is a separate selected native path. Authored point/structure
contact uses validated closed triangles, captured contact geometry and exact
finite segment/triangle tests, including coplanar contact. A broadphase box or
artificial ownership cap alone does not authorize contact. World chooses the
earliest actual contact, proximity/delay or terrain terminal event with its
native ordering and one-shot lifecycle; the reference checker grants none of
those authorities.

## Input/output boundary and independent target detection

Inputs to the pure adapter are selected timing/radius/station/body parameters,
captured own/target paths, current target identity and live cutoff. Outputs are
validity, actual fraction, miss distance and trigger, or delayed due/pending
status. No main-seeker tracking bit is an input to either geometry method.

Current World orchestration must be distinguished from this pure contract.
V1 scans its explicitly assumed ideal physical sphere independently of main-
seeker tracking. The23 current V0 AAM cards now explicitly select the additive
binding `target_detection_v1="native.time_armed_geometric_scene.v1"`. This
assumed geometric detector also scans captured physical aircraft, debris and
finite decoys independently of the main seeker, using each card's unchanged
V0 constrained-minimum sweep, arming, lifetime and radius. No additional delay
or V1 turn law is introduced. Candidate choice is earliest fraction, then stable
native identity; none of this scene data is fed into guidance. Inert jettison
does not arm that field. V1 station/history and initiation remain unchanged.

Absent-policy V0 profiles retain their original tracking-gated, held-target
dispatch and original provider behavior. Unknown policy names/values are
refused; this particular optional binding is admitted only on V0 mechanisms.
No provider ID, public interface, layout, numeric parameter or native outcome
meaning is changed. The declaration is a new explicit simulator assumption,
not a newly discovered RF/laser OEM law.

The retained original AIM120D5km head-on counterexample had actual armed CPA
1.1723276214934657m at launch age5.421845701496136s, within the original
11.47213595499958m radius, with no main track at either captured endpoint and
no proximity detonation. All1201 original kinematic trace rows were unchanged
by the diagnostic observation. This establishes the missing dispatch seam;
it does not attribute unrelated misses such as the retained165m AIM9M case to
the same cause. The original full-chain scene remains the repair regression.

Primary technical literature supports several sensor architectures, not one
universal OEM behavior. [US8033221B2](https://patents.google.com/patent/US8033221B2/en)
describes sharing SAL optics while distinguishing guidance and additional
proximity returns. [US9709372B2](https://patents.google.com/patent/US9709372B2/en)
describes a separate semi-active RF proximity embodiment with multiple antenna
channels and angle/rate processing. Neither patent assigns that hardware to all
24 cards. COMAERO's primary [Missiles tactiques, printed pp147-148](https://www.irsem.fr/storage/file_manager_files/2025/03/missiles-tactiques-comaero.pdf)
specifically distinguishes Magic2's TRT electromagnetic proximity fuze and
AIM9L's active laser fuze. The [Polish Air Force R-60MK manual, sections3.3-3.5](https://djvu.online/file/fUvs0bhsAlgza)
separates radar proximity, contact and safe/arming functions from thermal
homing. MK facts are not silently transferred to our M card.

Consequently, the current ideal radius law remains an engineering assumption.
No numerical radius, delay, look angle, TDD sensitivity or safe-arm timing is
promoted from a general patent to an OEM parameter. Every per-model card basis
and uncertainty remains visible. A main-seeker dropout followed by no detonation
requires actual captured minimum range/live/arming evidence before attribution;
it must not be repaired by relabeling a miss as a hit.

## Numerical comparisons, limits and failure modes

The actual selected provider is instantiated for every AAM. Two identical
captured crossing geometries have transverse miss0.5 and1.5 times the **actual
resolved** radius. Their independent projected fraction is0.5; the first is
inside and the second outside. A third approach-only case ends inside the
radius without an actual turn and distinguishes the two method cohorts. The
fixture retains the actual model version, arming/lifetime/delay and original
card hash. It does not swap a V0 card to V1, alter its radius, or force detonation.

The comparison budget is64 binary64 eps relative plus1e-12 absolute for distance
and fraction; discrete decisions have zero tolerance. Exact-rational stationary,
endpoint and adjacent-representable controls exercise the reference. Malformed
JSON, foreign/duplicate/missing populations, changed reference geometry,
stale epochs and failed/skipped native reports are refused. Timing/one-shot,
terrain, contact, body-station and actual World lifecycle have separate native
tests; the common projection pass is not their complete acceptance.

Known limits include linear interpolation between captured knots, an ideal
spherical detection field instead of RF/laser physics, model-specific retained
legacy endpoint behavior, frozen short-time geometry and assumed source
parameters. Numerical verification does not calibrate real safe-arm distance,
TDD jamming resistance, target discrimination, delay electronics or fuze
probability of functioning. These limitations cannot justify missing executable
functionality; they define the evidence needed for the corresponding claim.

Historical residuals, complete24 rows and hash-bound raw reports are in
[fuze.json](fuze.json) and [effects-results.v1.json](references/effects-results.v1.json).
Before a passing actual capture, these products remain incomplete. Native,
installed/public, coupled-chain and visual evidence are joined separately.

The pre-policy Release actual native case passed818 assertions for the full
effects observation fixture. Its independent fuze comparison historically admitted all24
original selected providers, both fuze mechanisms and192 numerical constraints,
with no changed radius, arming time, captured geometry or tolerance. The original
four-entry CTest JUnit and a fresh actual case-level Catch JUnit are both retained;
the checker consumed the latter. That exact source/card epoch is retained in
the policy-window snapshots; current-policy proof requires fresh capture and
source binding. These results establish the stated geometry
and cohort distinctions, not a guarantee that World dispatches every physical
encounter to a fuze or that an entire engagement detonates.

## Exact current model differences

The23 `swept_path.v1` selections share a constrained-minimum law, while their own radius/lifetime and captured histories remain different. The current explicit time-armed scene is independent of main seeker lock; it includes aircraft, detached debris and real decoy paths without inventing RF/optical discrimination. AIM-120C instead uses its authored station and turning-CPA state.

| Model | Law | Arming / life s | Radius m | Sensor reference |
| --- | --- | --- | ---: | --- |
| aim-120c | turning_cpa.v1 | 1 / 110 | 11.472135955 | captured_authored_station_from_nose |
| aim-120d | swept_path.v1 | 1 / 110 | 11.472135955 | captured_weapon_cg |
| aim-7m | swept_path.v1 | 1 / 110 | 13.3166446789 | captured_weapon_cg |
| aim-9m | swept_path.v1 | 1 / 45 | 10.0708305066 | captured_weapon_cg |
| aim-9x | swept_path.v1 | 1 / 45 | 10.0594117082 | captured_weapon_cg |
| asraam | swept_path.v1 | 1 / 45 | 10.1622776602 | captured_weapon_cg |
| iris-t | swept_path.v1 | 1 / 45 | 10.3763886032 | captured_weapon_cg |
| magic-2 | swept_path.v1 | 1 / 45 | 10.5637059362 | captured_weapon_cg |
| meteor | swept_path.v1 | 1 / 110 | 10.4641016151 | captured_weapon_cg |
| mica-em | swept_path.v1 | 1 / 110 | 10.4641016151 | captured_weapon_cg |
| mica-ir | swept_path.v1 | 1 / 45 | 10.4641016151 | captured_weapon_cg |
| pl-10 | swept_path.v1 | 1 / 45 | 10.3166247904 | captured_weapon_cg |
| pl-12 | swept_path.v1 | 1 / 110 | 11.472135955 | captured_weapon_cg |
| pl-15 | swept_path.v1 | 1 / 110 | 11.7434164903 | captured_weapon_cg |
| pl-8 | swept_path.v1 | 1 / 45 | 10.3166247904 | captured_weapon_cg |
| r-27er | swept_path.v1 | 1 / 110 | 13.2449979984 | captured_weapon_cg |
| r-27et | swept_path.v1 | 1 / 45 | 13.2449979984 | captured_weapon_cg |
| r-27r | swept_path.v1 | 1 / 110 | 13.2449979984 | captured_weapon_cg |
| r-27t | swept_path.v1 | 1 / 45 | 13.2449979984 | captured_weapon_cg |
| r-37m | swept_path.v1 | 1 / 110 | 14.7459666924 | captured_weapon_cg |
| r-60m | swept_path.v1 | 1 / 45 | 8.87082869339 | captured_weapon_cg |
| r-73 | swept_path.v1 | 1 / 45 | 9.72029410175 | captured_weapon_cg |
| r-77m | swept_path.v1 | 1 / 110 | 11.7434164903 | captured_weapon_cg |
| super-530d | swept_path.v1 | 1 / 110 | 12.4772255751 | captured_weapon_cg |

### aim-120c

aim-120c: selected version1, arming=1s, live end=110s, radius=11.472135955m, delay=0s; authored station=[1.46, 0, 0]m from nose, actual-turn law. Only selected finite V2 AAM source and turning-CPA/station fuze. Explicit finite geometry differs from retained V0 cousins; no C7 hardware internal reconstruction is inferred.

Parameter/source matrix: 11 resolved roles; original basis counts {"assumed": 9, "published": 2}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `ff55455def74ac1b58f355fec31994d45e91a92a3cfbffecadaa62698e7f5862`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Only selected finite V2 AAM source and turning-CPA/station fuze. Explicit finite geometry differs from retained V0 cousins; no C7 hardware internal reconstruction is inferred. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-120d

aim-120d: selected version0, arming=1s, live end=110s, radius=11.472135955m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. The card explicitly reuses C7 warhead information because exact D data are unpublished. Selected V0 source and CG sweep differ from AIM120C V2 despite equal total/charge/radius; do not equate the software constructions.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `1a9d0c576e3bf81da99b392b0cdec73c8f0e10df9d42d6984a9371b357c36781`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: The card explicitly reuses C7 warhead information because exact D data are unpublished. Selected V0 source and CG sweep differ from AIM120C V2 despite equal total/charge/radius; do not equate the software constructions. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-7m

aim-7m: selected version0, arming=1s, live end=110s, radius=13.3166446789m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `767ba2fcda0befcec949761c67979920699859e3cc421eaa31b8da27c0ac6c7b`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-9m

aim-9m: selected version0, arming=1s, live end=45s, radius=10.0708305066m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Reticle receive law is newly selected; WDU17/B label does not establish the single-detector/spoke hardware. Historical passive-fin assertion abort was a fixture failure, not a warhead result.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `c880bbdc9dda7117fd6533e960a0b60043d25fcf84b59fe606ca854a317e5282`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Reticle receive law is newly selected; WDU17/B label does not establish the single-detector/spoke hardware. Historical passive-fin assertion abort was a fixture failure, not a warhead result. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-9x

aim-9x: selected version0, arming=1s, live end=45s, radius=10.0594117082m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Imaging selection and its own card mass differ from AIM9M; shared WDU17/B reference is explicitly inherited, not independent measured X case geometry.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `d1ac2cc60cbf51a03062056e88926bcbd1e0a0f2de518363528329230e298755`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Imaging selection and its own card mass differ from AIM9M; shared WDU17/B reference is explicitly inherited, not independent measured X case geometry. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### asraam

asraam: selected version0, arming=1s, live end=45s, radius=10.1622776602m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `9bda81a0ed014546e29e21da3d4669fcadd6b174108ba29254c312cd8865721f`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### iris-t

iris-t: selected version0, arming=1s, live end=45s, radius=10.3763886032m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `b78916999d2fa3fb15f95572f91d26c62fc192a85134191f6954a1a6de06fb6f`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### magic-2

magic-2: selected version0, arming=1s, live end=45s, radius=10.5637059362m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Magic2 frontal detection and a historical multi-element description are not evidence for this scalar reticle. Pre-fragmented wording is represented by equivalent point fragments, not an authored preform map.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `a5ab904aaf400f4c90da5d1f3943bb2cc0e7c7b8edaf096b0f1bb5d5ed8b87b3`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Magic2 frontal detection and a historical multi-element description are not evidence for this scalar reticle. Pre-fragmented wording is represented by equivalent point fragments, not an authored preform map. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### meteor

meteor: selected version0, arming=1s, live end=110s, radius=10.4641016151m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `eacf6c946880d0f828a600ebe47a8b22c7fa94ee3697d17109b60c9dcdc91e88`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### mica-em

mica-em: selected version0, arming=1s, live end=110s, radius=10.4641016151m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `860eeebf25b54aee3a44eb6a81371c59eaaf991b1dd7d6fba65d5b8a7e1974b5`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### mica-ir

mica-ir: selected version0, arming=1s, live end=45s, radius=10.4641016151m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. The actual receiver is dual-band imaging with separate channel and pairing policies, not the single-band or scalar-reticle receiver.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `7486de74a3b2872984b9d7074eda8a746923a799b91764a6d18d121be51feac5`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: The actual receiver is dual-band imaging with separate channel and pairing policies, not the single-band or scalar-reticle receiver. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-10

pl-10: selected version0, arming=1s, live end=45s, radius=10.3166247904m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Ambiguous fragmentation/expanding-rod description is resolved by the authored fragmentation kind and selected zoned_case.v1; this source does not implement an expanding connected rod.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `e0bb8e8dde9579c267b2737d5e249a521bded8dbb7183a7a0bdbdb8dcbfa41ac`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Ambiguous fragmentation/expanding-rod description is resolved by the authored fragmentation kind and selected zoned_case.v1; this source does not implement an expanding connected rod. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-12

pl-12: selected version0, arming=1s, live end=110s, radius=11.472135955m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `e98fc0f0da48327e5fce22b4928e145e22a499f77972da5e6214862e7c134773`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-15

pl-15: selected version0, arming=1s, live end=110s, radius=11.7434164903m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `ae938cb68e0cdffb2cd3dc0ac173dba7e3d11b808dfa63d92175cf836bfe7a11`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-8

pl-8: selected version0, arming=1s, live end=45s, radius=10.3166247904m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `fc38df9519ac0200049946a177d8e71c7da6dbe0e791dcb21420c21dc702325f`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27er

r-27er: selected version0, arming=1s, live end=110s, radius=13.2449979984m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `925f74a728e471de6192f7812eeda73bc96818e5cdb960a4b2a34601aad66fe9`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27et

r-27et: selected version0, arming=1s, live end=45s, radius=13.2449979984m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `0b057fb60a807b7aa236ec5ea59497fe3670d3b59618ddc07003e36c5dcbb383`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27r

r-27r: selected version0, arming=1s, live end=110s, radius=13.2449979984m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `5052fabe66cb9f6885575f8763a81226967af416de5fc43ac2356ebfa0a25e1f`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27t

r-27t: selected version0, arming=1s, live end=45s, radius=13.2449979984m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `6e592128535664f9ce92b500bed91c321ad6d9f56bf7a17216e60c419475b4f6`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-37m

r-37m: selected version0, arming=1s, live end=110s, radius=14.7459666924m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `344484540137f23afcefaf4baab3e9938a48ef5d32735e7e3c8e3baa25ae5bc0`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-60m

r-60m: selected version0, arming=1s, live end=45s, radius=8.87082869339m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. The card description mentions depleted-uranium rods, but the selected zoned-case source uses steel7850kg/m3 and discrete equivalent rods; no DU density, pyrophoricity or connected-ring capability is implemented by that name.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `83a22cbc1d50da2399e30936e655a148b55f3f4c54eec56d55119beecac0a21d`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: The card description mentions depleted-uranium rods, but the selected zoned-case source uses steel7850kg/m3 and discrete equivalent rods; no DU density, pyrophoricity or connected-ring capability is implemented by that name. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-73

r-73: selected version0, arming=1s, live end=45s, radius=9.72029410175m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Rod description maps to discrete finite rod elements under zoned_case.v1, not connected_ring.v1; exact weld topology and material construction are not inferred.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `4b39dd8f507af1313258876538d000c5e24c0a34166dcb5a4209947b62e17cd5`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Rod description maps to discrete finite rod elements under zoned_case.v1, not connected_ring.v1; exact weld topology and material construction are not inferred. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-77m

r-77m: selected version0, arming=1s, live end=110s, radius=11.7434164903m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `152fda5406066e22ebbe0ee3ff8442877b4c4b42076a62ed9d0d23aa8e6ccf5b`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### super-530d

super-530d: selected version0, arming=1s, live end=110s, radius=12.4772255751m, delay=0s; captured CG sensor, zero delay, constrained-minimum sweep, explicit time-armed geometric scene independent of main seeker lock. The card explicitly inherits Super530F warhead information; exact D case geometry and fragment distribution remain inferred.

Parameter/source matrix: 5 resolved roles; original basis counts {"assumed": 5}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `af3c8a5f620b7f0d6f37fc5234a362be558e28d439f3662081185cde1ae2aa01`; selected mechanism-source SHA256 `86e2fe352926b429e32f2c2e479d0d376c73c4dc831266455a3b5a6cb9c2378f`.

Fidelity matrix: The card explicitly inherits Super530F warhead information; exact D case geometry and fragment distribution remain inferred. All23 swept-path cards share a software law, not sensor hardware; individual radius/lifetime and own release/target histories remain distinct. Capture gaps, removed-target cutoffs, arming/expiry tangencies and fixed-delay missing pose can prevent valid initiation; absence of a burst is not automatically a warhead failure. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

## Parameter equality is a disclosed modeling gap

When complete numeric policy and selected mechanism/receiver coincide, the
JSON lists `identical_numeric_policy_models` explicitly. This is reused
engineering parameterization, not proof of identical hardware or separate
per-model tuning. Every model still needs its own actual capture. Different
equipment selection IDs, source notes and metadata hashes alone do not supply
different physical response, OEM internal construction, or calibration.


## Authored-input/source closure and recorded method epoch

Each previously unresolved current-row source role now points to its exact `assets/stores/<model>.json` parameter and descriptor-reference JSON pointer, together with `parseProfileV1`, adapter precedence and construction symbols. This resolves software authoring provenance; it supplies no external variant measurement. Original values, basis, notes, uncertainty (including explicitly unquantified values), source identifiers and all existing admission flags remain unchanged. Historical unresolved records are retained as historical evidence.

`axis-methods/effects/fuze.json` records the already executed actual24 native/independent-Python method results. The producer input epoch is `d4923c75e70bdbcf6347195439414db143d637219c468ff4f137402142eb8f76`; its native run is `build/ucrt64-release/validation/component_reference_runs/effects/run-0a9f9332b2b14ef996c774f17653cc44`. Each corrected row copies the exact per-model method residuals and raw file/binary/module bindings into `executed_method_evidence`. Exact producer family/raw bytes are additionally retained under this candidate evidence directory's `producer-snapshot/`, with original-to-retained hash mappings, so later axis regeneration cannot silently change the referenced result. Those bindings retain their source epoch; authoring hashes are labelled separately. No new native execution or broad source-binding refresh occurs here. The recorded epoch contains parser headers but omits the directly used `Cards.cpp` and `Profile.cpp` implementation bodies. Their authoring traces are explicitly `missing_from_recorded_epoch`; current hashes do not retrospectively repair that source-binding gap. No arithmetic failure is inferred from this provenance limitation.

The exact scalar geometric rules are:

- For each merged captured relative segment r(u)=r0+u*d, d=r1-r0. If d dot d>0, u0=-(r0 dot d)/(d dot d); otherwise V0 chooses the first eligible fraction and V1 stationary overlap supplies no turn.
- The active interval is [armingS,lifeS), further cut off by native eligibility/contact. V0 takes the globally smallest constrained miss over merged segments, keeping the earliest exact tie; this is not first entry into the radius.
- V1 admits an analytic stationary point at an eligible initial/interior/final captured endpoint, including the final endpoint if its unconstrained stationary point is exactly represented there; a merely clamped approaching endpoint is not a turn. Initial-point deduplication/two-sided eligibility belongs to contiguous per-target history.
- V1 additionally admits a captured corner when r dot leftMovement<=0 and r dot rightMovement>=0, with previous actual motion; all candidate launch ages must remain inside the armed/live interval.
- Native fractions remain normalized to the original whole decision duration. Radius classification is miss<=radius; expiry is exclusive and an expiry-only tangency is excluded before any representable-live-age adjustment.

Paths require first offset0 and strictly increasing finite offsets/positions. motionAt holds the final captured pose after path end. V0 bounds its sweep by eligibleEnd and own-path end, retaining target hold-at-last semantics for native stopped-target ownership; V1 additionally bounds by target-path end. These adapter semantics do not authorize inventing a future moving path; World supplies native target/contact/removal cutoffs.

V0 has stateless proximity and no installed delay/turn-history law; contact consumption remains release-owned. V1 has contiguous per-target boundary history and Fresh/Pending/Sealed delay state. All23 current V0 cards explicitly select the time-armed geometric scene; no RF/laser discrimination is inferred.

The following are calculated margins of the *already executed synthetic crossing bench*, not an encounter. Arming is1s for all24. The inside case has original launch age arming+0.75s; all boundary rules remain unchanged.

| Model | Radius m | Life s | Inside radial margin m | Exclusive expiry margin s |
|---|---:|---:|---:|---:|
| aim-120c | 11.472136 | 110 | 5.73606798 | 108.25 |
| aim-120d | 11.472136 | 110 | 5.73606798 | 108.25 |
| aim-7m | 13.3166447 | 110 | 6.65832234 | 108.25 |
| aim-9m | 10.0708305 | 45 | 5.03541525 | 43.25 |
| aim-9x | 10.0594117 | 45 | 5.02970585 | 43.25 |
| asraam | 10.1622777 | 45 | 5.08113883 | 43.25 |
| iris-t | 10.3763886 | 45 | 5.1881943 | 43.25 |
| magic-2 | 10.5637059 | 45 | 5.28185297 | 43.25 |
| meteor | 10.4641016 | 110 | 5.23205081 | 108.25 |
| mica-em | 10.4641016 | 110 | 5.23205081 | 108.25 |
| mica-ir | 10.4641016 | 45 | 5.23205081 | 43.25 |
| pl-10 | 10.3166248 | 45 | 5.1583124 | 43.25 |
| pl-12 | 11.472136 | 110 | 5.73606798 | 108.25 |
| pl-15 | 11.7434165 | 110 | 5.87170825 | 108.25 |
| pl-8 | 10.3166248 | 45 | 5.1583124 | 43.25 |
| r-27er | 13.244998 | 110 | 6.622499 | 108.25 |
| r-27et | 13.244998 | 45 | 6.622499 | 43.25 |
| r-27r | 13.244998 | 110 | 6.622499 | 108.25 |
| r-27t | 13.244998 | 45 | 6.622499 | 43.25 |
| r-37m | 14.7459667 | 110 | 7.37298335 | 108.25 |
| r-60m | 8.87082869 | 45 | 4.43541435 | 43.25 |
| r-73 | 9.7202941 | 45 | 4.86014705 | 43.25 |
| r-77m | 11.7434165 | 110 | 5.87170825 | 108.25 |
| super-530d | 12.4772256 | 110 | 6.23861279 | 108.25 |

`dm_R/dR=1` at fixed captured geometry; classification is locally constant away from its discontinuous boundary. The JSON includes exact native input/output and arm/live margins for both crossing cases. Station uncertainty affects only the actual V1 material-point reference, not an absent V0 station role.

Reproduction of these offline candidate calculations: run `D:/ENV/python/3.13/python.exe -B build/architecture-reviews/sim-m03-current-20261104/effects-theory-source-correction-candidate/prepare-candidate.py` from the project root while the preserved before/actual evidence bytes remain present. It writes only this evidence directory, refuses changed role/binding/native identities, and records analytic checks in `analytic-checks.json`. The maintained native/Python producer commands remain in the existing method runner/collector; this candidate does not rerun or rename their gates.
