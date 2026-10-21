# Fragment intersection repair and remaining showcase gate

Date: 2026-10-20. Baseline: main
`8147c071d824a16572023dc93b547b1a17254b53`.
The [assigned repair window](../superpowers/plans/2026-10-20-sim-m03-fragment-hit-repair.md)
freezes equipment parameters, authored geometry, scene inputs and original
acceptance assertions. This delivery fixes the 6 m internal-component defect.
The showcase fragment-Hit gate remains failing; its diagnosis is separate.

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

## Unresolved: the showcase spray misses the supplied target geometry

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
