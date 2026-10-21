# SIM-M03 five-kilometre root-cause investigation

## Current repair delivery status

The subsequent native-provider separation follows baseline `bf16f80` and is
documented in the [interface/concrete-model inventory](payload-native-providers.md).
Nine role interfaces and eleven concrete classes are separated. Production
World execution selects per-release instances from a copied catalogue. All
24 guided cards now declare explicit control configuration; controller numbers,
physical XML and engine/nozzle bytes remain unchanged. This architecture work
does not repair the original physical-model failures.

On the frozen provider source, both complete builds pass. Fresh focused gates
pass 7/7 Release CTest programs, 14/14 Hangar tests and 1/1 installed equipment
consumer (34.28 s). Current scoped physical regression is **118/120** Release
(75.35 s) and **118/119** Headless (75.09 s), zero skips. The original showcase
fragment Hit and 6 m side-burst component-hit assertions remain failed. Logs
are `payload-providers-focused.{log,xml}`, `payload-providers-hangar.log`,
`payload-providers-installed.{log,xml}` and
`payload-providers-{release,headless}-physical.{log,xml}` under
`build/architecture-reviews/`. Later totals in this document describe the
preceding delivery and remain historical.

Both original 5 km scenes were rerun fullscreen with current binaries. Native
screenshots `native-providers-functional_004.png` and `_005.png` show the actual
target's missing nose and upset at 18.9 s and 21.8 s. Its unchanged functional
fixture reports 2874.88 kg separated dry mass, crew loss, and five seconds of
continued native motion ending at 5870.09 m and 127.486 m/s; `crashed` is false.
Showcase screenshot `native-providers-showcase_003.png` shows nose separation
and upset at 30.3 s. Its native recording ends at 40 s, 4807.9217 m and
70.5778 m/s, with `crashed=false` and `diverged=false`. These are actual
mechanical-contact aftermath observations, not fragmentation acceptance or a
claimed crash. Screenshots are 3840 by 2160 under
`build/architecture-reviews/sim-m03-five-km-viewer/`, alongside
`NATIVE-PROVIDERS-functional-native-engagement-1-5000.000000.fsrec` and
`NATIVE-PROVIDERS-showcase.fsrec` / `.fsrec.engagement.json`. Viewer follows
the missile during approach and the target after the encounter. The first
visual harness attempt read a null process exit code; caching the owned process
handle fixed the harness, and both repeated native producers exited zero.
Neither producer exit status replaces the original failed damage assertions.

The preceding [uniform component delivery](payload-component-configuration.md) added
versioned configuration and compiled SDK dispatch for all nine component
roles across all 53 store cards. Existing numerical inputs and legacy/V1
execution distinctions are retained. Exact card-source binding, refused
composition, inert release and non-inert unguided native plant validation
are exercised in production paths before entity publication. Unsupported
roles remain explicitly unavailable. This does not repair the remaining
fragmentation-model failures or import external numerical data.

The later, explicit V1 card supersedes the legacy-card intermediate counts
below. It is a generic analytical profile, not calibrated equipment. The
unchanged native 5 km scene currently produces **29/1688** fragment strikes;
the unchanged showcase produces **0/1688**. The original showcase Hit assertion
remains enabled and failed. Actual nose separation, crew loss and subsequent
native upset now occur in both scenes, principally through the separately
modeled mechanical contact impulse. Those losses do not validate fragmentation
or close SIM-M03 acceptance.

An independent standard-library reconstruction reproduces both exact counts
and RNG boundaries. All 29 native intersections are confined to the authored
pitot damage box; the showcase rays miss even the enclosing damage boxes.
There is no demonstrated ray-transform, attached-membership or penetration
threshold defect accounting for these V1 counts. This identifies a limitation
of the configured ideal radial source and encounter geometry, not evidence
that the real equipment behaves this way. No angular width, end-effect value,
fuze delay, scene, gain or damage threshold is fitted to obtain a hit.
The archived reconstruction is
`build/architecture-reviews/sim-m03-radial-ray-reconstruction.md` with exact
input JSONL, per-ray CSV and reconstruction script in the same directory.
The temporary production input trace has been removed after reconstruction.

Hangar regenerated the current AIM-120C native control XML from its source
card, 31 authored contact-geometry documents and 123 section model XMLs. The
section-model change preserves every byte outside `<mass_balance>` and
retains the original physical inputs at round-trip precision. Generation
proofs are `sim-m03-aim120c-generation.json` and
`sim-m03-pieces-generation.json` under the evidence directory. No GLB, solver
output, aerodynamic coefficient or motor data is regenerated for this repair.
Explicit model-version guards now reject stale staged payload cards in the
native fixture; build dependencies stage the source cards for targeted builds.

The loaded-section regression exposed incorrect reaction at the dry section
CG, stale pre-impulse launch state, coarse generated mass/CG values and use of
an ellipsoid different from the actual native model. The SDK now uses the
actual loaded piece CG/mass, post-impulse state, native coordinate conversion
and paired impulses at one common point. Current Release regression:
**1 case, 29 assertions passed**. Linear residual is about **1.54e-7 Ns**;
angular residual in the frame translating with the initial CG is about
**6.8e-10 Nms**, under the unchanged **.001 Ns/.001 Nms** bounds.
Total mass and relative first moment are checked separately at **1e-8 kg**
and **1e-8 m**. The old Earth-frame angular calculation still reports about
**.00126 Nms**, explicitly retained as diagnostic: its residual is the
first-moment rounding term crossed with the common bulk velocity. It is not
reported as passing the Earth-frame bound. Independent mathematical review
approved the common translating-frame oracle and separate positional checks.
Log: `sim-m03-loaded-conservation-translating-frame.log`.

The native functional fixture has been rerun with fullscreen live publication
on current source, showing the actual target at 18.9 s and 22.0 s after the
16.7667 s encounter. Its unchanged native execution continues five seconds
after structural loss. The target slows from about 250 to 127.5 m/s and loses
height; the Viewer shows the separated nose and subsequent upset. Evidence:
`sim-m03-five-km-viewer/CURRENT-canonical-native-native-engagement-1-5000.000000.fsrec`
and `current-canonical-native_001.png` / `_002.png` (3840 by 2160).
Later frames after world destruction are waiting views and are not damage
evidence. The second unchanged showcase was rerun fullscreen after trace
removal, retaining 0/1688 strikes. Its recorded target has an actual missing
nose and sustained upset; at 40 s it remains finite and airborne, at about
4808 m and 70.6 m/s. This is not a claimed crash. The wider 35 m aftermath
camera uses replay time to avoid wall-clock capture errors:
`current-verified-aftermath_003.png` shows the whole target at 33 s. The
two-scene ray plot is `sim-m03-recorded-ray-coverage.png`; its projections
illustrate archived 3-D classifications and cannot independently prove hits.

## Preceding component/contact delivery regression evidence

On that preceding delivery, both UCRT64 Release and Headless complete build
targets passed. Its Release physical selection discovered **119 cases/programs:
117/119 pass, two fail, zero skips**, in 74.53 s.
The native engagement program passes all **12 cases**, including the original
5 km/30 km worker checks, actual functional aftermath, swept encounter state,
later-CG invariance and the new no-weapon boundary history case. Contact-native
passes its seven cases. Logs and JUnit are
`payload-release-physical.{log,xml}`. These are scoped tests, not a
current complete CTest-suite acceptance claim. The Headless counterpart
independently discovers **118 cases/programs: 117/118 pass, one fails, zero
skips**, in 74.62 s. Its remaining failure is the
same unchanged 6 m component-hit assertion; the rendered showcase program is
not part of that preset. Logs/JUnit: `payload-headless-physical.{log,xml}`.

The focused Release component gate passes six CTest programs covering profile
source/refusal isolation, effect adapters, composition, native plant selection,
all 53 actual native assets and existing guidance regressions. The installed
equipment consumer passes 1/1 in 37.03 s. These are feature-owned installed
consumer checks, not complete generic integration acceptance. Logs/JUnit:
`payload-components-release.xml` and `payload-installed-equipment.{log,xml}`.
The final focused Hangar selection passes 13/13 in 5.65 s, including the original
three store-contract checks and persistent all-53 numerical/engine-byte
equivalence. Tank/pod diagnostic dispatch was repaired after capturing its
25 original errors; the report remains distinct from the selected native
force model. Log: `payload-hangar-final.log`.

The two current Release failures remain unchanged assertions:

* `showcase_native`: the recording has genuine inventory release, detonation,
  separation and removal, but no fragment Hit. The original required Hit is
  not bypassed by mechanical functional loss.
* The 6 m left-burst damage fixture: panel hits occur, but the required nonempty
  component list is empty. Independent geometry finds eight fuel-component
  intersections; each is stopped by the existing protective-skin/THOR path.
  The finite wing midsurface and future-skin protection approximation have no
  corresponding authored closed wing volume here. This is an unvalidated
  shielding-model limitation, not proof of a frame/intersection discard.
  `sim-m03-left-burst-component-reconstruction.md` records the exact independent
  geometry and penetration reconstruction. No physical input is fitted to
  change this result.

The V1 card-input regression now explicitly requires V1, absent unused angular
zones and exact integer fragment mass. Its prior sum-of-zone-weights assertion
tested a legacy-only field. The versioned contract RED is
`sim-m03-versioned-zone-red.log`; original hit/component requirements remain.
The broad single-process tag selection timed out at its 180 s bound and is not
reported as passed. The per-case CTest selection excludes only the declared
hidden manual benchmark `what a JSBSim piece costs a step`, absent from the
maintained CTest inventory. Exact selection metadata accompanies the JUnit.

The earlier Hangar combined gate ran **30 tests with two failures**: C172 and
Skua damage-document numerical drift already reproduced with the original HEAD
generator against the original documents. Neither failure is hidden or repaired
by changing unrelated physical inputs. The new round-trip mass-property and
committed-piece checks pass. Log: `sim-m03-hangar-verified.log`; baseline proof:
`sim-m03-hangar-preexisting-drift.json`.

The earlier source/test/card/generated freeze remains archived as
`sim-m03-current-frozen-inputs.json` with 198 fingerprints. It predates the
uniform component delivery and is not reused as current input evidence.
Complete current full-suite and full SIM-M03 acceptance remain pending while
the original physical-model gates fail.

After both preset gates, the frozen current Release binary also reruns the
unchanged showcase live/fullscreen through its entire 40 s. It follows launcher,
released missile and physical victim in that order. The recording
`sim-m03-five-km-viewer/CURRENT-boundary-history-five-km.fsrec` contains 1200
frames; `current-boundary-history-showcase_003.png` shows the missing nose and
native upset at about 30.4 s. The result still has 0/1688 fragments striking.
All 198 fingerprints matched after that historical run on `92ae5ed`; it
preceded the component delivery. The owned publisher and Viewer processes exited.

The final component binary reruns both original scenes live and fullscreen at
3840 by 2160, following the released missile and then the native target. The
functional native case passes its unchanged checks: five seconds after loss,
the target has moved 833.725 m, speed is 127.486 m/s and altitude is 5870.09 m;
it has not crashed. `final-native_004.png` and `_005.png` show its actual absent
nose and upset. The separate showcase retains the same release and encounter
epochs (13.033333 s and 17.633333 s). Its recording ends with altitude
4807.92 m, speed 70.58 m/s and `crashed=false`; `final-showcase_003.png`
shows the parent and separated nose at 30.4 s. No visible crash is claimed.
Final recordings and logs are under `sim-m03-five-km-viewer/`:
`FINAL-components-native-native-engagement-1-5000.000000.fsrec`,
`FINAL-components-showcase.fsrec`, `final-native.log` and `final-showcase.log`.
These diagnostic runs are not actor-isolated acceptance. The final input
manifest is `payload-components-final-inputs.json`; the earlier 198-file freeze
is retained separately. Both owned publishers and Viewers exit after capture.

## Historical baseline and intermediate repair evidence

Date: 2026-10-20. Baseline: main
`92ae5ed16295b001592db30eae0928a6dcc00e10`. The
[implementation window](../superpowers/plans/2026-10-20-sim-m03-five-km-repair.md)
retains both original scenes, seeds, equipment values and real-hit assertions.
No external numerical delivery is consumed.

## Native baseline and visualization

Fresh baseline runs reproduce both failures. The native head-on case detonates
at 10.2878 m with zero of 2023 sampled fragments striking. The separate public-SDK
showcase detonates at 9.5780992851 m, also with zero of 2023 striking. Neither
result has an actual target damage event. The failed assertions remain active.

The showcase's exact native recording and evidence are preserved in
`build/architecture-reviews/sim-m03-five-km-viewer/`. The visible Viewer replays
`FAILED-five-km-showcase.fsrec` fullscreen, following weapon slot 2 at 0.5 times
speed. This changes presentation only. It is a diagnostic failed-scene replay,
not evidence of a successful hit. The native head-on fixture also has an opt-in
recording output without changing its scene or assertions.

## Damage calculation and diagnostic boundary

The SDK selects an authorized initiation time and burst pose, then samples the
existing warhead's 2023 fragment directions from its unchanged per-victim RNG.
The sampled distribution is primarily lateral to the missile's nose. Each ray's
relative velocity is missile velocity plus ejection velocity minus target
velocity, transformed into target body axes. The assessor intersects attached
finite panels and component boxes, applies air-drag and penetration losses, and
passes actual momentum, holes and blast loading to native damage state.

Zero fragment strikes here means no usable sampled ray intersects the admitted
attached geometry. It is not a hidden energy/penetration threshold failure:
an admitted intersection sets the assessor's struck flag even when the skin
stops the fragment. Detonation and blast impulse do not by themselves guarantee
a fragment Hit event, structural failure or crew loss.

An independent Python oracle reconstructs the unchanged warhead and SplitMix64
draws from the exact captured RNG state. It uses finite-triangle and slab-box
intersection predicates separately from the native assessor. Both baseline
captures yield zero raw intersections. The proximity-corrected one/two-worker
native captures each yield 106, distributed across the five zones as
`[51,6,42,4,3]`. All four replayed RNG end states exactly match the native
assessment. Evidence is `sim-m03-five-km-ray-oracle.{py,json,log}`. No penetration
or damage threshold can account for the old zero raw-intersection result.

The current reduced model uses straight relative rays and a captured target
pose. It does not simulate each fragment as an independently stepped rigid body
or continuously rotate the target during fragment flight. Software repairs do
not establish calibrated real-warhead angular distributions or vulnerability.

Temporary opt-in diagnostics capture exact burst axes/position/velocity,
decision-step fraction, target model-origin pose/velocity, native path samples,
CG/rates, attached membership and RNG states. Diagnostics never draw from the
simulation RNG or feed execution. Raw captures are
`sim-m03-five-km-{native,showcase}-inputs.jsonl` under the evidence directory's
parent. Before/after scene outcomes remain separate from these diagnostics.

## Cause 1: sample-dependent proximity initiation

`Fuze.h` defines closest approach within the captured armed/live interval.
`MotionHistory.h` previously returned the first native segment whose local
minimum was inside radius. An arbitrary inserted sample could therefore make
the selected burst an approaching segment endpoint rather than the interval's
closest approach.

The independent RED uses an unchanged linear weapon path x=-2 to +2 and a
stationary target at y=0.5, with radius 1. Endpoints alone select fraction 0.5,
miss 0.5. Adding only a redundant target sample at fraction 0.3 changes the old
result to fraction 0.3, miss sqrt(0.89). Neither physical motion nor equipment
input changed. The same invariant covers weapon-only and combined subdivision.

The repair minimizes over all captured eligible native pieces and keeps the
earliest exact tie. Arming, expiry, native cutoff and the separate contact
candidate retain their boundaries. It adds no extrapolation or future motion.
This is invariance to representing the same interval, not a claim of invariance
to changing World decision boundaries or guidance cadence.

The RED log is `sim-m03-five-km-fuze-red.log`. After repair, the unchanged native
5 km scene produces **106/2023 actual fragment strikes**, at **8.55161 m**, and
passes its real damage requirement for workers one and two with exact records
and events. The separate showcase still fails, so this correction is not claimed
as its repair.

## Cause 2: damage approximation used as a physical contact surface

The showcase's precise contact trace selects panel 44 at decision fraction
0.82025665033708051. That panel is the fuselage damage approximation: body-axis
centre `(7.5,0,0.31)`, half extents `(0.5,0.23,0.17)`. Its flat front is x=8,
with z in `[0.14,0.48]` and y in `[-0.23,0.23]`.

The captured missile nose is approximately
`(7.99999924,0.01993252,0.14092219)` in those axes. The unchanged authored
fuselage tip at x=8 instead has zero width and z in `[0.33,0.37]`. The supposed
contact is about **189 mm beyond the authored tip's top**, as well as outside
its zero-width station. Independent source and arithmetic reviews agree.

`damage.py::_panels` intentionally extrudes a midpoint cross-section over each
one-metre longitudinal interval for damage assessment. `ContactFuze.cpp` had
promoted this approximation to a collision surface. The early false contact
ends the missile before a later legitimate proximity or skin encounter can be
evaluated. The resulting burst's sampled fragment rays miss the target.

The correction window adds versioned authored-skin contact geometry from the
existing `Body::skin` source while retaining the original damage boxes and all
their numerical values. A tapered-body negative fixture must reject a box-only
entry; a positive continuation must reach the actual face. Initial-inside
suppression, immutable section ownership, detached exclusions, finite geometry
and malformed-input refusal remain required. Whether the unchanged showcase
then produces real fragment damage remains a pending native gate.

The explanatory plot is
`build/architecture-reviews/sim-m03-five-km-viewer/premature-contact.png`, derived
from the captured values and unchanged design. The baseline proof preserves
31 original damage documents and hashes 606 protected TOML/XML/GLB/card inputs.

## Historical intermediate verification status

The proximity subdivision RED has been captured and its focused gate is GREEN.
The native 5 km hit and worker checks pass after that correction. The unchanged
showcase still fails. Authored-skin contact repair, final production review,
current full regression, installed provenance and final commit are pending.
Full SIM-M03 remains unaccepted.

## Historical snapshot availability without a released weapon

A later native regression exposed a separate SDK availability defect: a
scripted `detonate(stepFraction=0.5)` had a captured encounter position but no
captured CG/atmosphere when no live weapon enabled the full native history.
Each decision now clears the previous histories and captures synchronized
opening/final pose, CG and matter boundaries in this mode. Historical queries
interpolate those captured boundaries; they do not substitute the current CG
after a later mass change. Complete native substep histories remain conditional
on a live weapon. A newly terminal endpoint in boundary-only mode supplies no
valid moving interval; reset still invalidates the history. The unchanged
encounter-position fixture and an additional no-weapon later-CG-change fixture
remain subject to the root-owned native verification gate.
