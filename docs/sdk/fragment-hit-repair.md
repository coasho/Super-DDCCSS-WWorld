# Fragment intersection repair and showcase acceptance

The [2026-10-21 sourced-model continuation](#sourced-finite-fragment-field-and-original-showcase-acceptance)
passes the original positive-Hit showcase. Earlier failures below are dated
investigation evidence, retained to explain the software fixes and the separate
model-admission decision.

Date: 2026-10-20. Baseline: main
`8147c071d824a16572023dc93b547b1a17254b53`.
The [assigned repair window](../superpowers/plans/2026-10-20-sim-m03-fragment-hit-repair.md)
freezes equipment parameters, authored geometry, scene inputs and original
acceptance assertions. This delivery fixes the 6 m internal-component defect.
The showcase fragment-Hit gate remains failing; its diagnosis is separate.
The subsequent [coordinate/event-chain follow-up](#coordinate-and-hit-event-chain-follow-up)
adds native software corrections and current verification without changing
the fragment model or the original showcase requirement.

## Fixed: a later skin was moved before an earlier component

`damage::assess` already sorts skin crossings and component interceptions by
distance along each actual fragment ray. It then overrode that ordering: when
its enclosure counter was zero, it processed the next skin within one metre
before the current component. A future skin was therefore allowed to stop a
fragment before an earlier component interception. A component near a surface
mid-plane does not establish that an earlier unmodeled enclosure exists.

In the unchanged F-16C side-burst fixture, independent ray/box intersections
identify eight rays entering the left-wing component. For example, fragment 80
enters the component at ray distance 4.471815 m, but the surface-mid-plane entry
at 4.747279 m was processed first. That later skin stopped the fragment. These
are diagnostic distances, not fitted equipment parameters.

The repair removes that look-ahead and its now-unused enclosure/done bookkeeping.
Every skin and component is processed in sorted ray order. Earlier skins still
shield components, panel-before-component ties retain their stable order,
fragments born inside boxes still meet their forward exit skin, and a stopped
fragment cannot proceed to another event. Source sampling, penetration formula,
drag, equipment coefficients and the RNG draw sequence are unchanged.

The new analytic regression places a component at x=1 with its front at x=.9
and a stopping plate at x=1.5. Before the fix, the component incorrectly receives
zero hits. The independent expected result is one interception with the full
fragment energy and momentum. A second section moves the same plate upstream
to x=.5 and requires it to stop the fragment. These synthetic fixture dimensions
are not equipment tuning.

After repair, the unchanged 6 m case passes all 12 assertions. It retains
124 distinct fragment intersections, now including seven left-wing component
hits. One of the eight geometric component candidates is still legitimately
stopped by earlier crossed skin. No assertion, warhead value or aircraft asset
was changed to obtain this result.

This corrects traversal of the supplied reduced geometry. It does not calibrate
wing fuel-tank dimensions or replace surface mid-planes with physical enclosed
wing volumes. Missing shielding geometry must be authored and validated through
its generator, rather than inferred by a one-metre runtime search.

## Historical V1 failure: the showcase spray misses the supplied target geometry

The unchanged showcase has a different cause. Temporary native instrumentation
captured all 1688 sampled fragment origins and relative directions. Every ray
has an empty intersection-event list, so changing skin traversal cannot make it
hit anything. An independent Python implementation uses slab intervals for
component/body boxes and Moller-Trumbore barycentric intersections for the two
triangles of each surface quad. It also finds zero component and panel
intersections for those same captured rays. The instrumentation was removed
from production source before final builds.

At the captured contact burst, the charge centre in the target's model-origin
body frame is approximately `(9.260162, .042316, .427678)` m. The actual missile
axis in that frame is `(-.989294, -.007315, -.145753)`, and the charge-centre
velocity relative to the target origin is approximately
`(-915.764907, -1.079929, -26.258878)` m/s. The charge is still outside the
aircraft when the nose contact initiates it. The configured finite cylindrical
side-case model emits radial fragments, not an additional axial/end-cap spray.
Translation and the measured rigid rotation are included, but their resulting
sampled rays all pass outside the supplied damage geometry.

The selected ideal proximity law waits for an actually captured turning-point
closest approach to the target CG. Physical nose contact occurs before that
turn, and its armed contact channel correctly initiates the burst. Being inside
the configured detection radius is not, by itself, a turning-point trigger.
Increasing that radius would not correct this law/geometry mismatch. Replacing
the current law with a first-radius crossing, extrapolated future trigger or
fitted early burst would change its explicit contract and existing regressions.

The existing diagnostic provenance is retained: real launch, activation,
acquisition, contact detonation, mechanical damage, crew loss and nose
separation. `WorldDamage.cpp` admits a recorded fragment `Hit` only when
`impact.fragmentsStriking > 0`. The unchanged showcase condition requires that
specific event, so it continues to fail. The additional failure message reports
burst, fragment-Hit, inventory and removal counters independently; it does not
weaken the condition or relabel contact/blast damage.

An additional analytic negative regression verifies a known consequence of the
selected source: a small forward-axis box outside a finite radial spray envelope
receives no fragment hit or momentum, for three RNG seeds. Its positive-hit,
rigid-motion, finite-origin and three-draw source regressions remain separate.
This protects against adding axial fragments or a fabricated hit to satisfy the
showcase. It is not a new assertion that real AIM-120C-7 endgame behavior is
represented accurately.

## Public-source investigation

The follow-up instruction was to research the missing evidence. This review
checks public primary sources and actual implementation code. A paper's
existence, a public fact sheet and a mature simulation framework establish
different things; none alone establishes equipment-specific fidelity.

| Source inspected | What it supports | Admission boundary |
| --- | --- | --- |
| [USAF AIM-120 fact sheet](https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104576/aim-120-amraam/) | Official family-level identification of blast fragmentation and guidance architecture. | No C-7-specific fragment field or trigger response; not a quantitative calibration dataset. |
| [Air Combat Command AIM-120 fact sheet, 27 January 2022](https://www.acc.af.mil/Portals/92/Docs/Fact%20Sheets%20-%202020%20Update/Facts%20Sheets%202022%20Final/AIM-120_final.pdf) | Indexed official text distinguishes the guidance, armament, propulsion and control sections, including a target detection device. | Direct PDF retrieval returned HTTP 403. Indexed text is contextual evidence only; no constants were imported from it. |
| [JHU/APL, Palumbo, 2010, pp.4-5](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_GuestEditor.pdf) | Treats approach direction, fuzing and fragment interception as related endgame concerns. | General engineering context, not a C-7 fixture or source of card constants. |
| [Felix, Brighton doctoral thesis, December 2020, printed p.104, section 5.3.4](https://cris.brighton.ac.uk/ws/portalfiles/portal/25754302/Felix_Final_thesis.pdf) | Describes a generic real-time model and explicitly identifies an unverified end-plate result because experimental data was absent. | Does not justify selecting an end-plate model on C-7 merely to close the showcase gate. This is a limitation of that result, not a claim that all fragment research lacks experiments. |
| [Felix, Colwill and Harris, accepted 30 December 2020, DOI 10.1016/j.dt.2020.12.004](https://cris.brighton.ac.uk/ws/files/22793218/1_s2.0_S221491472030502X_main_1_.pdf) | Follow-on work examines generic fragment shapes, dimensions and source locations and reports comparisons with experimental data. | Its generic experiments are not C-7 acceptance evidence. The paper is CC BY-NC-ND; no code, figures, tables or derived asset were imported. |
| [MIXR IMissile.cpp, pinned commit 1b4029419e382bb532f1e142f9c490dc9adc27f7](https://raw.githubusercontent.com/doughodson/mixr/1b4029419e382bb532f1e142f9c490dc9adc27f7/src/models/player/weapon/IMissile.cpp) | Actual mature-framework code separates its detonation decision, effect dispatch and recording. | The inspected default implementation uses target truth and optional Robot Aircraft dynamics. It is not a fragment-intersection implementation or an equipment validation dataset. It is not copied into the SDK. |

The reviewed sources do not supply an admitted C-7 endgame reference case for
this fixed encounter. In particular, the family-level fact sheets cannot
validate the present source assumption. A generic model must remain labeled
generic; passing a deterministic scene cannot convert it into measured C-7
behavior. No fetched hardware quantities, empirical coefficients or additional
fragment directions were inserted into the game model in this repair.

## Existing source boundary

[JHU/APL, Palumbo, Guest Editor's Introduction: Homing Missile Guidance and
Control, 2010, pp. 4-5](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_GuestEditor.pdf)
discusses approach direction together with fuzing and fragment interception.
It supports treating this as a coupled endgame problem, not deriving equipment
effectiveness from a contact or a scalar miss distance. It supplies no
AIM-120C-7-specific spray or detector constants.

The already admitted generic handbook is
[MIL-HDBK-1211, 17 July 1995, section 2-2.4](https://quicksearch.dla.mil/qsDocDetails.aspx?ident_number=116871).
Its catalogue separately lists later validation notices. It is generic
methodology, not a fitted C-7 fuze/warhead model. The card's sensor/charge stations,
zero-delay idealization and pure side-case source remain explicitly assumed.

The software diagnosis does not establish realistic C-7 endgame performance.
The existing model and the original acceptance gate remain visible rather than
being changed until they agree. No SIM-M02 numerical input, solver or result is
needed to establish the traversal defect or the captured geometric
non-intersections; Claude's AeroNumericsLab work was not inspected or modified.

## Evidence

Artifacts are under `build/architecture-reviews/fragment-hit-repair/`:

* `side-burst-baseline.log`, `showcase-baseline.log`: both original failures
  reproduced on the baseline before any repair.
* `regression-red.log`: the new analytic ordering case fails first, with the
  component incorrectly absent; upstream shielding still passes.
* `side-burst-trace.log`, `showcase-rays.log`, `analyze.py`,
  `side-analysis.json`, `showcase-analysis.json`: captured native rays and
  independent geometric checks. The side trace contains repeated deterministic
  calls from the original fixture; ray numbers are identical across them.
* `fix-focused.log`: new ordering and unchanged 6 m fixture pass 2/2 gates.

Both existing UCRT64 presets built successfully after temporary instrumentation
was removed. All native execution was serial, using the launcher and
`D:/ENV/tmp` for TMP, TEMP and TORCH_EXTENSIONS_DIR. The selector includes every
discovered non-hidden case in `test_damage.cpp`, `test_warhead_model.cpp`,
`test_impact.cpp` and `test_contact_fuze.cpp`, plus the payload, representative
weapon, native engagement, contact, equipment and architecture gates. Release
also includes the original showcase gate. Every selected name must exist in
CTest discovery; execution uses `--no-tests=error`, `-j1` and bounded timeouts.

| Current check | Result | Artifact |
| --- | --- | --- |
| New ordering regression on original production code | Fails before repair; earlier component incorrectly absent | `regression-red.log` |
| Ordering regression and unchanged 6 m fixture after repair | 2/2 pass | `fix-focused.log` |
| Headless selected gates | 99/99 pass, 0 skipped, 165.30 s | `headless.xml` |
| Release selected gates | 99/100 pass, 0 skipped, 163.10 s | `release.xml` |
| Remaining Release failure | `showcase_native`: one burst, zero fragment Hits; inventory/release/removal all present | `release.xml` |
| Protected input comparison with baseline | No changes to cards, aircraft, public headers or Hangar sources | `preservation.log` |

`select_tests.py`, the two `*-discovered.json` and `*-selection.txt` files record
the exact discovered selection. `summarize.py` derives `summary.json` from the
two JUnit files. These are current focused gates, not a claim of the full SDK
suite, installed-consumer compatibility, calibrated physical fidelity or Viewer
acceptance. The native build/test slot is released at handoff. The showcase gate
remains open; this delivery does not claim both original failures are repaired.

## Coordinate and Hit event-chain follow-up

Baseline: main `dded7aed2c043a9f2f8b611bcc4c3f8fd9a9be69`, 2026-10-20.
The user requested further software diagnosis rather than configuration reports.
The [exclusive scope](../superpowers/plans/2026-10-20-showcase-event-chain-correctness.md)
preserves the original scene, model inputs, native Hit meaning and assertions.

### Proven software defects and corrections

1. **Initial component Fire coordinates mixed references/epochs.** Component
   centres are fixed-model-origin body coordinates. The event started at a live
   CG and added the full component centre using encounter attitude, omitting
   the CG offset and combining different epochs for swept encounters.
   `World::applyDamage` now reuses the existing `weapon::materialPoint` helper
   with the captured pose, captured CG and component centre. The published
   position and rigid-point velocity use that same epoch. The existing spill
   velocity addition, damage assessment, probabilities and RNG are unchanged.
2. **Subsequent Fire-rate events jumped back to the CG.** `World::stepDamage`
   published the default vehicle event without component attachment. It now
   samples that component's current point and rigid-body velocity, including
   rotational point velocity. Missing CG/invalid points are logged and not
   replaced with a fabricated component location. Event timestamps retain the
   existing World publication tick; ground-wreck events keep their separate
   surface/contact meaning.
3. **Live showcase camera treated general damage as fragment Hit.** The tool
   switched to the target after a `damage/warhead` notification even when its
   native recording contained no Hit. It now consumes the native diagnostic
   outbox once per step and matches actual `EngagementKind::Hit` by victim slot,
   weapon slot + 1, store, positive intersection count and terminal time.
   `engage()` already publishes/records before this outbox is drained. The
   internal read-only `World::vehicleStreamSlot` supplies actual live slots;
   numeric entity IDs are never treated as slots. Removed IDs cannot resolve
   to a reused slot. No installed public signature, layout, enum or vtable changes.

`tests/integration/test_showcase_native.cpp` also rejects a target Hit from
another source/store/epoch instead of accepting any Hit on that target. Its
original positive-Hit and functional-damage requirements remain active.

### Independent software evidence

Raw logs use `build/architecture-reviews/showcase-*`.

| Case | Before correction | Current result |
| --- | --- | --- |
| Native initial component Fire pose, existing MiG fixture | 3 of 8 assertions fail; CG-offset error | Pass |
| Native Fire-rate point/velocity, deterministic synthetic tank depletion | 6 of 13 assertions fail | Pass |
| Live fixed showcase camera versus actual recorded Hit | Fails after the unchanged 40 s scene: target focus without matching Hit | Pass in Headless and Release |
| Slot identity after removal/reuse | New positive/negative identity controls | Pass |
| Actual native Hit through outbox and recording readback | Existing positive damage fixture, not a replacement showcase | Pass: count, source, time, position and velocity retained |
| Hit matching predicate | Valid positive Hit plus eight negative source/target/kind/count/time/name/validity controls | Pass in Headless and Release |

The positive recording control uses the existing scripted native damage fixture.
It demonstrates event transport, not an armed installed-consumer engagement or
equipment acceptance. The depletion fixture deliberately changes test-only fuel
contents to force a rate/expiry publication; it changes no production card or
showcase scene. The native publication regressions assert independently
reconstructed point positions and rotational velocity, not the helper's output.

Both complete UCRT64 builds pass. Headless discovers/executes **107** selected
gates: **106 pass**, only the unchanged `showcase_native` fails, **zero skipped**,
**208.45 s**. Release independently executes the same **107** selected gates:
**106 pass**, only `showcase_native` fails, **zero skipped**, **208.18 s**.
Both JUnit case sets are reconciled with their discovered selections. Selection covers all discovered
non-hidden damage, warhead, impact and contact-fuze cases, plus architecture,
component/provider, native engagement, public equipment, strike-capture and both
new showcase software gates. Full generic compatibility, installed consumers
and rendered Viewer acceptance are not claimed.

The unchanged Release showcase's complete recording is byte-identical before
and after these fixes, SHA-256
`f91edfa8680e4c9d9779f8e584ec13ad79b959fb637bf2016f286ac54d4138d7`.
Its full engagement-evidence JSON also matches. The new live run requests only
carrier and missile focus because there is no matching Hit. These are software
and native-stream observations, not rendered GUI acceptance. Source/binary
fingerprints and exact selections accompany the raw logs. The serial native
slot is released at this handoff; no GUI slot was occupied.

### Coordinate/intersection review and remaining inputs

The authored-nose conversion uses inches to metres once, with structural aft/up
to body forward/down signs; station offsets then use their already resolved
metre values. World captures pose/CG at the same eligible encounter time.
Target basis columns and the ray assessor's inverse projection agree. No new
defect in that source-to-target transform was demonstrated by this review.

All **44** F-16C surface quads are convex and coplanar within the independent
1e-8 m diagnostic check. No production tolerance is changed. The damage geometry
SHA-256 is `18ab046f7020a8ae99d948cc534314ac6ba4d57bb03b738c8c62778541a32d9a`.
The historical 1688-ray capture was rechecked independently against those
unchanged boxes/quads: still zero intersections. The fresh analysis is under
`build/architecture-reviews/showcase-event-chain/`; this is a reanalysis of the
previous captured rays, not a new capture or an independent physical experiment.
The source distribution, geometry solver and native Hit admission are unchanged.

The original showcase still requires a positive native fragment Hit. These
software corrections do not turn a non-intersecting ray set into a positive
intersection. The project still lacks admitted, independently validated inputs
for the selected equipment's detector/initiation and installation reference,
fragment field/source geometry, and target geometry/material/vulnerability
response with independent encounter evidence. Those are project admission gaps,
not a claim that no public research exists. General methods and family fact
sheets cannot establish variant-specific precision or justify fitting this scene.

No AeroNumericsLab input/result, solver directory or other worktree is used.
Claude's SIM-M02 investigation and its physical acceptance remain independent.

## Fresh runtime resource and complete source-chain check

Baseline: main `dded7aed2c043a9f2f8b611bcc4c3f8fd9a9be69`.
The [coordinate investigation window](../superpowers/plans/2026-10-20-showcase-coordinate-chain.md)
preserves the unchanged original `showcase_native`. This follow-up reproduces
its one burst and zero fragment Hits before changing production code. It adds
no further Fire, camera, equipment-input or scene changes.

### Actual loaded inputs and independent reconstruction

Temporary tracing at native loading/assessment boundaries records these actual
paths, rather than assuming which source assets the resolver selected:

* JSBSim root: `third_party/jsbsim`; F-16C flight XML and damage JSON:
  `aircraft/f16c/`; released weapon XML: `aircraft/stores/aim120c/`.
* Bound executable store/component profile:
  `build/ucrt64-release/share/flightsim/stores/aim-120c.json`, byte-identical
  to `assets/stores/aim-120c.json`, SHA-256
  `53cc6213ad3a32a2943d49f3b764a4917a7826c152257170f2accf5d405380e1`.
* The loaded F-16C damage JSON has the same `18ab046f...` hash documented above.
  Its 87 panels and 15 components are all attached at assessment. Its sealed
  contact geometry is present. There is no stale staged-aircraft substitution.

The fresh trace captures target/weapon poses, CG offsets, bases, fixed nose and
charge points, contact identity, source rigid motion and all 1688 rays. The
independent Python check rebuilds the charge station from XML structural inches
and card metres, reconstructs ECEF points, derives the finite source from the
actual loaded card, and regenerates every ray from the documented SplitMix64
damage stream. It does not use the native ray output to generate its expected
source positions or directions.

| Independent comparison | Observed result |
| --- | --- |
| Authored nose/charge station and target/burst ECEF reconstruction | Zero difference at the printed native precision |
| Native fixed structural origin versus damage-file design origin | 1.22e-8 m difference |
| Regenerated ray source positions versus native | Maximum error 1.81e-15 m |
| Regenerated ray unit directions versus native | Maximum error 4.97e-16 |
| Regenerated ray speeds versus native | Maximum error 9.10e-13 m/s |
| Captured nose reconstruction versus piecewise-linear selected contact | 5.41e-6 m difference; different interpolation constructions, not metres of origin error |
| Fresh rays versus actual damage boxes and finite panel triangles | Zero intersections, independently and natively |
| Fresh rays versus all 42,752 authored external contact triangles | Zero intersections independently |
| Fresh rays versus each authored closed mesh's derived bounding box | Zero intersections independently, before any triangle-edge predicate |

The independent triangle/box checkers have analytic positive/negative controls.
An additional positive ray hits the actual selected contact mesh at exactly
1 m; the reverse ray misses. Thus a zero result is not an always-false checker.
Contact mesh surfaces and damage/penetration boxes have deliberately separate
existing contracts. This investigation does not substitute one model for the
other merely because they differ.

### Proven geometric kernel correction

Two new native regressions first fail on the pre-fix implementation:

1. A finite nonzero direction smaller than the slab's hardcoded parallel cutoff
   loses a real thin-box crossing and also admits a disjoint thin-box ray.
2. A finite shallow ray crossing within a quad is rejected by its angular
   cutoff. A follow-up regression also exposes an overflow false Hit when an
   infinite intersection reaches the old NaN edge classifier.

`Damage.cpp::slab` and `quad` now distinguish exact parallelism from nonzero
directions. Finite bounds and quad edge tests remain authoritative; non-finite
denominators, intersection distances, points and edge predicates are rejected.
No smaller replacement tolerance is fitted. The tests retain actual hit/miss,
component energy and momentum assertions, with truly parallel and finite-face
miss controls. These synthetic geometries are not equipment calibration.

The primary implementation review includes
[Geometric Tools line/box clipping, file version 8.0.2025.05.10](https://raw.githubusercontent.com/davideberly/GeometricTools/master/GTE/Mathematics/IntrLine3AlignedBox3.h),
whose clipping distinguishes positive, negative and zero direction denominators,
and [CGAL's AABB-tree documentation](https://doc.cgal.org/latest/AABB_tree/),
which distinguishes primitive intersections from bounding-box traversal.
No upstream code, library or numerical equipment parameter is imported.

These kernel defects do not explain the original showcase: its captured rays
miss both geometric representations independently. That gate remains failed.
The source-to-target chain does not establish an equipment-valid source model:
the loaded provider explicitly selects the assumed
`native.warhead.side_case.v1` radial source. At contact its charge centre is
1.66 m behind its authored tip; the observed encounter places that source
outside the target's forward geometry. The ideal target-CG turning-CPA channel
has not initiated before contact. Changing detector timing, adding axial/end
fragments or moving the charge would require an admitted replacement model,
not a coordinate or intersection patch.

The inputs still missing for that replacement are variant-specific detector/
initiation behavior and installation reference, a supported spatial/angular
fragment field including the treatment of case ends, and an independent
encounter reference to validate their combination. The current geometry files
are available and were checked; missing penetration/fire calibration is not
offered as an explanation for zero geometric intersections.

Raw evidence is in `build/architecture-reviews/showcase-coordinate-chain/`:
`baseline.log`, `native-trace.log`, `resource-trace.log`, `check_chain.py`,
`chain-analysis.json`, `intersection-red.log`, and `overflow-red.log`.
Temporary native instrumentation is removed from the delivered sources.

### V1 verification at the 2026-10-20 handoff

Both complete UCRT64 builds pass. Release and Headless each discover and execute
the same 109 selected cases: **108 pass**, **one failure (`showcase_native`)**,
**zero skipped**. The selection contains every non-hidden damage, warhead,
impact and contact-fuze case plus the native engagement, component/provider,
equipment, architecture, capture and showcase gates. All execution is serial
with bounded timeouts and `--no-tests=error`.

`release-ctest-raw.log` and `headless-ctest-raw.log` retain CTest's complete native
case output. `release-discovered.json`, `headless-discovered.json`, the two
`*-selection.txt` files and each preset's `showcase-coordinate-chain.xml` retain
discovery/execution provenance. `summary.json` reconciles their exact case sets;
`source-input-binary-manifest.json` fingerprints the sources, selected assets,
executables and JUnit reports.

Both presets' complete original-showcase recordings remain byte-identical to
the pre-correction recording, SHA-256
`f91edfa8680e4c9d9779f8e584ec13ad79b959fb637bf2016f286ac54d4138d7`.
Their full engagement-evidence JSON is also unchanged. The geometric fix does
not alter this encounter. No equipment card, generated native aircraft,
Hangar input, public header or original acceptance assertion changes. No GUI,
installed-consumer or full generic compatibility acceptance is claimed.
The SDK serial native slot is released; AeroNumericsLab and other worktrees
were not inspected or modified.

## Sourced finite fragment field and original showcase acceptance

Date: 2026-10-21. Baseline: main `dded7ae` and the preceding uncommitted
coordinate/event corrections. The user explicitly admits a sourced generic
model replacement while preserving the original scene, equipment physical
values and positive Hit assertions. This continuation supersedes the earlier
open-showcase status; it does not relabel the earlier V1 failures as passes.

### Why a distinct model is required

The continuous V1-envelope check bounds all possible finite pure-radial
samples, not only the recorded seed. All 145 conservative target bounds
(87 panels, 15 components and 43 authored contact meshes) remain separated;
the smallest conservative radial separation is 0.488874 m. Exact piecewise
fuze replay is still closing through the contact window, with no earlier CPA
root. The observed zero-Hit scene therefore does not establish a coordinate,
resource or fuze defect. No fuze behavior is changed.

The existing legacy angular approximation cited seven game-reference sectors
but retained and normalized only five middle sectors. V0 keeps those historical
outputs and its RNG sequence for compatibility. Pure-radial finite V1 also
keeps its exact three-draw contract. The new concrete
`FiniteZonedCaseWarheadV2` combines the existing finite annulus with the complete
seven-sector reference and has a separate implementation ID, paired model/source
selectors `(2,2)` and required angular-profile binding
`wt.aam.seven_zones.ea96f3af75af.v1`. Unknown or missing bindings fail explicitly.
V2 takes exactly five draws per fragment: axial position, azimuth, annular
radius, sector choice and polar cosine. The velocity includes source rotation
at each material point and subtracts the captured target-origin velocity.

### Admitted source and assumptions

The [pinned extracted game data](https://raw.githubusercontent.com/gszabi99/War-Thunder-Datamine/ea96f3af75afe1d858f80ac13f212db218bc69ca/aces.vromfs.bin_u/gamedata/weapons/rocketguns/us_aim_120c_7.blkx)
at commit `ea96f3af75afe1d858f80ac13f212db218bc69ca`, path
`rocket.damage.shatter.segment[*].angles/countPortion`, supplies these exact
angular intervals and relative shares:

| Angle from nose, degrees | Relative share |
| --- | ---: |
| 0-20 | 0.0375 |
| 20-60 | 0.075 |
| 60-80 | 0.10 |
| 75-100 | 0.60 |
| 100-120 | 0.10 |
| 120-160 | 0.075 |
| 160-180 | 0.0125 |

The overlapping intervals are retained as a mixture. The source file SHA-256
is `b0952b5a3c11a1d21d38b00c0d1d94bd019da0abcff20dfa14b84229c147388c`.
This is community-extracted game data, not measured C-7/WDU-41 evidence.
Uniform solid angle within a sector, finite annular spatial sampling and a
common position/ejection azimuth are explicitly project assumptions. No source
radius, penetration, damage, explosive, motor or fuze multiplier is imported.
No profile boundary, share or physical card value is fitted to this encounter.

### Fresh coordinate, geometry and resource evidence

`v2-native-trace.log` captures every source ray and the actual assessment
transforms. `check_v2.py` independently reconstructs all five draws from the
documented native damage stream, the loaded card and pinned reference file;
native ray output supplies comparisons only. Its slab and barycentric triangle
tests include positive and negative controls. `v2-chain-analysis.json` records:

* All 1688 ray origins, directions and speeds agree: maximum differences
  1.82e-15 m, 4.52e-16 and 9.10e-13 m/s respectively.
* All captured source/target transforms exactly match the V1 trace. All
  87 damage panels and 15 components are attached at assessment. The native
  and independent damage-geometry checks identify the same 53 ray indices.
* The independent closed-contact-mesh check intersects 28 of those rays across
  42,752 external triangles. Contact volumes and reduced damage panels/boxes
  serve different SDK contracts; 53 is the native damage-geometry Hit count,
  not a claim of 53 intersections with the closed contact meshes.
* The actual bound card is
  `build/ucrt64-release/share/flightsim/stores/aim-120c.json`, byte-identical to
  its maintained source, SHA-256
  `4fa4f89be5f9800074e7c65af51466205f6e2d3e9368dbdcd9ef10242eee4788`.
  Every physical parameter object, including uncertainty and provenance,
  matches the saved V1 card. Only model/source selectors and descriptive
  component/source metadata change. The other 52 cards are unchanged.
* F-16C damage geometry retains SHA-256
  `18ab046f7020a8ae99d948cc534314ac6ba4d57bb03b738c8c62778541a32d9a`.
  No aircraft/generator/engine data changes. The original scene hash and
  every weapon-evidence field except the exact warhead model version match
  the saved baseline. All 227 recorded pre-burst target samples match exactly.

The original scene still launches at 13.033333333 s and detonates at
17.633333333 s, with maximum weapon speed 665.589057592 m/s. The native
recording now contains a positive Hit for the actual target slot, source weapon
slot + 1, store and terminal time, with 53 intersections. Its functional
acceptance verifies native pilot loss and nose separation, their native
consequences and 22.366667 s of continued target motion. At 40 s the target
is at altitude 4772.943973 m and speed 76.822644 m/s, with neither a diverged nor
a crashed flag. Contact-induced nose separation
must not be attributed exclusively to fragmentation; no crash or crew loss is
invented. The original test also checks repeat recording, live pacing, capture
cadence and invalid input rejection. Its only existing assertion update is
the exact selected warhead version from 1 to 2; execution/fuze remain exactly 1.
The separate native-engagement fixture makes the same exact version update.
An initial matrix also exposes a preparation dependency in the newly added
Fire-rate coordinate regression: the first spill-grid point does not ignite a
tank with V2. It now uses the unchanged maintained spill-fixture grid, retaining
every positive burning/update, coordinate and rigid-velocity assertion. That
publication fixture does not set the original showcase's geometry or outcome.

### Verification and remaining physical inputs

New V2 model/provider regressions fail before implementation (`v2-red.log`),
then pass with legacy V0/V1 preservation controls. The new finite-source
regression checks all sectors over 512 synthetic seeds, exact RNG state,
source-point motion, energy, momentum and moment. Original `showcase_native`
first passes in 43.37 s (`v2-showcase-first-pass.log`). Fresh final build,
matrix and rendering evidence is recorded below after the source freeze.

Missing real-equipment acceptance inputs remain explicit: variant-specific
detector installation/initiation and delay behavior; measured spatial/angular
fragment mass and velocity distributions, including case ends; target
construction/shielding and penetration/functional vulnerability data; and
independent encounter references with uncertainty and convergence evidence.
This generic software repair closes the showcase Hit failure, not SIM-M03
physical acceptance or a beyond-DCS fidelity claim. Claude's SIM-M02 workspace
and results are not accessed or used.

### Final frozen-source gates and render evidence

| Gate | Fresh result | Raw evidence |
| --- | --- | --- |
| Complete Release build | Pass | `v2-final-release-build.log` |
| Release selected native matrix | 111/111, zero skips, 263.60 s | `v2-final-release-ctest.log`, Release `v2-final-showcase-coordinate-chain.xml` |
| Complete Headless build | Pass | `v2-final-headless-build.log` |
| Headless selected native matrix | 111/111, zero skips, 262.90 s | `v2-final-headless-ctest.log`, Headless `v2-final-showcase-coordinate-chain.xml` |
| Feature-owned installed C++/C99/Python equipment gate | 1/1, zero skips, 38.79 s | `v2-equipment-package.log`, Release `v2-equipment-package.xml` |
| Configuration-report regressions | 12/12 pass | `v2-report-green.log` |
| Current catalogue with both fresh JUnit reports | 53 cards, 477 roles, zero unmapped available IDs | `v2-report-generate.log`, `sim-m03-model-report-v2/report.json` |
| Original-scene continuous fullscreen strike harness | Pass, native/Viewer exits 0/0 | `v2-original-viewer.log`, external run directory below |

The matrix discovers its exact nonzero case sets before execution, uses serial
CTest, bounded timeouts and `--no-tests=error`, and includes all non-hidden
damage/warhead/impact/contact-fuze cases plus the relevant provider, native
engagement, equipment, architecture, capture and showcase gates. Fresh JUnit
case sets agree exactly with each preset's selection. The final summary is
`v2-final-summary.json` under `build/architecture-reviews/showcase-coordinate-chain/`,
which also holds the raw logs and independent reconstruction artifacts.
The source/binary/artifact fingerprints are in
`v2-source-input-binary-manifest.json`. The two original-scene recordings and
complete engagement JSON files are byte-identical across presets; recording
SHA-256 is `a04d9dc1f5054cd0d736b68d3631996aead09e96f5e702fce3b4ed6a56834093`.
The installed gate retains its separate prefix, source/header/data fingerprints
and consumer logs under `build/ucrt64-release/equipment-package-consumer/`.
No generic compatibility build or full regression total is substituted here.

The unchanged `tools/validation/viewer_perf/strike_acceptance.py` consumes the
original showcase scene. Its only logical input change is the diagnostic World
name; target heading remains 270 degrees, not the separate historical 210-degree
Viewer fixture. The exact command is:

```powershell
.\fsim.cmd --preset ucrt64-release python tools/validation/viewer_perf/strike_acceptance.py --build build/ucrt64-release --scene build/ucrt64-release/showcase-native-test/first.json --out D:/ENV/tmp/showcase-v2-original-20261021
```

One fullscreen Viewer follows carrier, weapon and actually hit target in order.
Its live native readback reports `nativeFrames=1200 preHitDestructiveEffects=0
matchingHitFragments=53 nativeCameraSubjects=3 captures=2 duration_s=40`.
Native wall time is 40.070264 s; Viewer wall time is 42.211702 s. Capture times
are 14.033333333 s and 17.933333333 s, the exact requested native ticks. The
Viewer continues to 40 s after both. Preflight and live outcomes agree exactly;
pilot loss freezes SDK commands, the nose is separated, and the target remains
airborne without a native crash at the end. No engine failure, control-channel
loss or panel tearing is inferred: their reported counts are zero.

Both PNGs decode at 3840 x 2160 and were visually inspected. The first shows
the released missile in powered flight; the second follows the struck target
with its nose absent. That close target view is cropped, and offline terrain
logs include missing subtiles; this run does not claim polished framing or
complete terrain coverage. No Viewer/camera/render implementation is changed
for this acceptance. `result.json`, both native recordings/readbacks, raw logs
and `strike_launch_plus_1s.png` / `strike_hit_plus_0.3s.png` are retained in
`D:/ENV/tmp/showcase-v2-original-20261021/`. These are diagnostic SDK/Viewer
checks, not installed public-endpoint joint acceptance.

The serial native and GUI slots are released at this handoff. No external
worker was contacted, and no AeroNumericsLab or other worktree was accessed.
