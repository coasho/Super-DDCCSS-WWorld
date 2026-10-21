# Representative weapon mechanisms and validation

Date: 2026-10-20. Implementation baseline: main
`8daf1e0f3e3bd01359a2b6a8f85b538b93d29bf8`.
The user approved the [design](../superpowers/specs/2026-10-20-sim-m03-representative-weapons-design.md)
and requested inline execution of the
[implementation plan](../superpowers/plans/2026-10-20-sim-m03-representative-weapons.md).
This is a software mechanism delivery, not calibrated equipment or complete
SIM-M03 acceptance.

Follow-up from main `8147c07`: the [fragment traversal repair](fragment-hit-repair.md)
corrects the 6 m side-burst internal-component failure. The showcase fragment-Hit
gate remains open with an independently checked non-intersecting spray. The
verification record below is the original representative-model delivery; use
the linked repair record for the latest damage checks and remaining model work.

## Selected equipment and concrete execution

| Card and identity | Concrete seeker | Actual differentiated behavior |
| --- | --- | --- |
| `aim-9m`, AIM-9M | `NonImagingIrSeekerV2` | Intensity-weighted centroid of unresolved signals in the retained assumed angular cell; measured angular history supplies bearing-only guidance |
| `aim-9x`, AIM-9X Block I | `ImagingIrSeekerV2` | Separate angular/spatial association and retained projected-extent shape continuity when all competing feature inputs are available; missing features are not evidence of a decoy |
| `aim-7m`, AIM-7M | `SemiActiveRadarSeekerV2` | Associated illuminator, transmitter and receiver propagation legs, illumination interruption, track loss and reacquisition; no invented independent target-range measurement |
| `aim-120c`, AIM-120C-7 | `ActiveRadarSeekerV2` | Separate inactive/search/acquisition/tracking state, admitted launcher estimates and autonomous sensing after activation; support loss does not switch off its active seeker |

The component IDs are respectively `fsim.seeker.nonimaging_ir.v2`,
`fsim.seeker.imaging_ir.v2`, `fsim.seeker.semi_active_radar.v2` and
`fsim.seeker.active_radar.v2`. All four select
`fsim.guidance.measured_pn.v2`, implemented by `MeasuredPnGuidanceV2`. Bearing
models use own speed and measured LOS history; the active-radar model uses its
admitted radial-speed measurement. Existing PN gain and finite-fin control
limits remain explicitly assumed. A shared guidance/control algorithm does not
need an equipment-named duplicate class.

Only these four cards migrate. The other 20 AAMs retain the legacy encounter
model and Cartesian guidance. Existing V1 interfaces, public layouts, enum
values and installed API signatures are unchanged. New V2 execution interfaces
are private, additive contracts implemented alongside the existing provider
interfaces. The component descriptor envelope remains version one. Strict new
schema identities validate exact mode, variant, observable family and fidelity
bindings. A mismatched pair or a factory lacking the selected V2 interface
refuses native release before entity publication or inventory consumption.

## Measurement, attribution and lifecycle

World performs physical scene queries and emits time-stamped observables.
`SeekerSampleV2` has no target identity, semantic aircraft/decoy flag or hidden
target velocity. IR and the current SARH receiver provide bearings; ARH also
receives ideal geometric range and radial speed. Referee attribution is an
external mapping from the selected sample index. It is not an association or
guidance input. Range-dependent IR/SARH timing stays unavailable, and absent
hardware covariance is not replaced with zero measurement uncertainty.

The IIR scene projects currently attached authored airframe panels/components
into angular extents from the actual pose and CG. This is an ideal geometric
feature source. It is not radiometric imagery, detector resolution or a
temperature/signature model. Cartridge flame geometry is unavailable; a missing
extent remains missing. No target/flare truth label, automatic immunity or
legacy decoy-resistance multiplier is used in the new tracking algorithms.

Each installed store owns its sampled on-rail provider and attribution snapshot.
Repeated queries at one simulation epoch neither construct another provider nor
step its filter. World samples on simulation boundaries. A successful load
invalidates the affected station, a loadout replacement invalidates the installed
set, and a vehicle reset clears its sensor history without changing authority,
tasks or C2 ownership. Release seeds the actual new instance from a current
admitted on-rail angular track. Other releases and Worlds own independent state.
Observable ordering, including coincident feature-bearing samples, supplies
deterministic association ties without semantic identity.

JSBSim remains the only motion integrator. Guidance requests acceleration through
the existing finite-fin control path; it does not apply ideal forces, moments
or angular-state corrections. No new RNG draws are introduced.

## Primary references and limitations

The source decisions and immutable reference versions are recorded in the
[approved source table](../superpowers/specs/2026-10-20-sim-m03-representative-weapons-design.md#10-primary-references).
JHU/APL supplies the LOS/homing-loop reference. Native RF/tracking primitives
and Stone Soup inform the measurement/association boundary; no Stone Soup runtime
is imported or executed as part of this delivery. pyBSM remains an offline
image-formation candidate, not a production seeker or an executed comparison.
MIXR's gimbal is a reference; its simplified default missile is not adopted.

[NAVAIR's Block I account](https://www.navair.navy.mil/node/10456) distinguishes
the represented early AIM-9X from subsequent Block II datalink and
lock-on-after-launch additions. No production lot or flight-program revision is
invented. The card removes the unsupported detector-array-dimension claim from
its guidance prose and identifies Block I explicitly.

The following remain open:

* Fitted detector bands, noise/covariance, optical transfer/resolution, actual
  servo/gimbal response and proprietary countermeasure processing. Sampling is
  the World cadence, not an admitted hardware update rate.
* AIM-9X Block I jet-vane force/moment and actuator inputs. The current selected
  plant is fin-only; no TVC model or physical acceptance is claimed.
* AIM-120C-7-specific sensor/control data and a fitted modeled weapon link. Its
  existing launcher-designation updates remain explicitly idealized. No return
  telemetry capability is inferred.
* Accepted SIM-M02 flight/control inputs, uncertainty and independent reference
  comparisons. AeroNumericsLab, its solvers and its inputs were not inspected
  or modified for this package.
* The existing fixed 6 m side-burst component-hit failure and showcase fragment
  `Hit` failure, full required Viewer acceptance, and complete SIM-M03
  calibration. Detonation, damage, crew loss and mission success remain
  different native outcomes.

No equipment numerical parameter was added or changed. The preserved-input
comparison covers all 53 card parameter objects and SHA-256 checks of all 101
generated native model/engine/nozzle files. The existing generator stamps only
the five plant roles; these seeker/guidance selections therefore require no
hand-edited or regenerated physical XML. Hangar independently confirms the four
new selections produce identical native plant files to their legacy selections.

## Verification record

Raw logs are under `build/architecture-reviews/representative-weapons/`.

* `red-tests.log`: two pre-change behavioral regressions fail, 10 failed
  assertions. Hidden IR velocity changes acceleration and intercept timing;
  flipping only a semantic decoy flag changes association and commands.
* `native-red-tests.log`: native dispatch/cache regressions fail before World
  integration. The initial test fixture also tried an unsupported AIM-7M/F-16
  wingtip fit; it was corrected to the existing F/A-18C station 4 fit. No
  platform admission rule was relaxed.
* `review-red-tests.log`: additional regressions first reproduce unsupported IR
  update reporting and order-dependent coincident spatial association; both
  are corrected without adding a performance coefficient.
* `admission-red-tests.log`: a native observer first detects that SARH frames
  still contain an unused launcher Cartesian estimate. World now supplies that
  field only to the configured active-radar update path; bearing-only frames
  contain no such estimate.
* `headless-final-build.log` and `release-final-build.log`: complete UCRT64
  Headless and Release builds pass on the final source.
* `headless-final.log` / `.xml`: 19/20 selected CTest gates pass, zero skips,
  118.65 s. All 17 software gates pass: architecture/configured graph, ten new
  behavioral cases, native four-model/cache/illumination integration, existing
  component contracts, guidance, fuze, finite controls, contact and equipment
  C++/C tests. The two additional passing gates are the installed consumer and
  native engagement program. The additional side-burst gate fails as below.
* `release-final.log` / `.xml`: 17/18 selected CTest gates pass, zero skips,
  20.72 s. The same 17 software gates pass. The additional unchanged
  `showcase_native` gate fails as below; this is a recording test, not a
  fullscreen visual inspection.
* `hangar-tests.log`: 12/12 component tests pass, including the four-model
  generated-plant equivalence regression.
* `headless-final.log` / `.xml`: installed feature-owned C++/C99/Python equipment
  consumer passes 1/1 in 32.85 s, using its private install trees and provenance
  checks. Generic compatibility ownership remains separate.
* `preservation.json`: all 53 numeric card objects and 101 native assets are
  unchanged against the pre-migration capture.
* `headless-final-detail.log`: the unchanged native engagement program passes
  all 12 internal cases in 65.74 s. Its fixed 5 km
  and 30 km encounters retain exact one/two-worker trajectories and events.
  The 5 km functional case retains actual target damage and five seconds of
  continued flight. The separate 6 m side-burst case still fails its unchanged
  internal-component-hit assertion (`test_damage.cpp:745`). This is not a new
  sensor/control coefficient or a relaxed physical acceptance criterion.
* `release-final-detail.log`, `showcase-comparison.json`: the showcase emits
  native launch, activation, acquisition, contact detonation, damage, crew loss
  and nose separation, but 0/1688 fragments strike. `WorldDamage.cpp` emits the
  recorded `Hit` event only for fragment intersections; the unchanged showcase
  assertion therefore fails. The retained pre-migration diagnostic also has
  0/1688 fragment intersections and the same terminal/functional event kinds.
  Its recorded miss is 9.0333 m versus 9.1973 m now. That historical comparison
  establishes the existing failure mode, not trajectory equivalence, a freshly
  rebuilt baseline or calibrated physical accuracy. No damage/scene/test
  thresholds were changed to make this gate pass.
* `diagnostics.log` / `.xml`: final-source four-model native recording run
  passes 1/1 in 2.98 s. The four store
  recordings are loaded through the native replay reader and contain actual
  weapon-control snapshots. They are diagnostic, non-isolated recordings, not
  fullscreen Viewer or actor-isolation acceptance.

Complete full-program regression, independent mature-implementation comparisons
and calibrated physical acceptance are not inferred from these scoped results.
No claim of exceeding DCS follows from this software delivery.

To reproduce the feature gates through the existing launcher:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/launch/fsim.ps1 --preset ucrt64-headless build
powershell -NoProfile -ExecutionPolicy Bypass -File tools/launch/fsim.ps1 --preset ucrt64-headless test -R '^(representative_weapon.*|payload_.*|weapon_guidance|fuze|missile_autopilot|launch_eligibility|contact_fuze_native|ma_equipment|ma_equipment_c)$' --no-tests=error --timeout 120 -j 1
```

For the optional native recordings, create an output directory and set
`FSIM_REPRESENTATIVE_RECORD_DIR` to its absolute path before running
`representative_weapon_native`. The four store-named `.fsrec` files are checked
by the replay reader; `summary.jsonl` records the selected provider and actual
dispatch/tracking evidence. The additional on-rail and SARH-support recordings
retain their respective lifecycle and support scenarios.
