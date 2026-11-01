# Weapon fidelity and kill-chain validation

<!-- SIM-M02 final closure begin -->
SIM-M02 is **closed for the user-authorized 31 fixed-wing and 53 store functional scope** on 2026-11-01. All 4 rotor functional populations are explicitly deferred at lowest priority, not passed. The common engineering matrix still covers all 88 designs: **88 engineering-complete-uncalibrated, 0 blocked**. Missing equipment measurements are optional calibration limits, not missing components. Required but unavailable components remain required, absent, failed and blocked; the native armed-store implementations do not use an inert-release waiver.

The current clean-source execution covers all 840 selected headless CTest cases, followed by the exact 2 approved repair rechecks (79 nested methods). The first headless run's 2 failures are retained. Independent current Release execution passes **914/914**, with the exact 28 authorized rotor-only cases excluded from both preset inventories. All 16 fleet shards retain all 31 fixed wings. Additional current evidence is 24/24 matched-state AAM observations (3,454 native evaluations), 48/48 powered axial comparisons at both 480/960 Hz, and 84 native plus 84 installed consumers with 2,016 short-flight observations, 106 released bodies and 10/10 component/package fixtures. These distinct populations are not added into a fabricated single test total.

Exhaustive static checks cover 7,067 native XML tables, 3,611,335 coefficient cells, 522,331 axis knots and 16,776 inventory entries. Every table retains method/source/version/basis, finite-value/grid and interpolation/extrapolation checks. The 1,165 applicability/check/status consistency predicates are **not physical tests**. Native short-flight, pressure and axial checks are sampling/method comparisons, not full-envelope equipment validation. CFD tables: 0; newly measured aerodynamic tables: 0.

Current-source all-design generation, mandatory-component recovery, installation, basic use and full-table auditing take **2,845.286 seconds in total**. This exceeds the 20-minute aspiration; no under-20-minute or historical aggregate speedup is claimed. The dominant measured job is C-17A at 1,290.243 seconds, including 926.975 seconds for fresh damage/detached models. Fresh preparation avoids repeating cold generation in each regression fixture, bounded immutable caches preserve exact inputs, and both native integration resolutions reuse one identical-state numerical reference per atmosphere/design. Complete and interrupted regression/attempt times and end-to-end elapsed time are separately recorded, not hidden by batch partitions.

The root normal `run.ps1` entry and the clean-source entry each complete the unchanged 40-second native encounter at 1x, fullscreen 3840x2160. The SDK produces launch, acquisition, contact detonation and damage; rendered captures preserve those outcomes. The scene executable statically links `fsim::core` and was rebuilt from current source; actual JSBSim DLL identity is recorded separately from the root public Python SDK's actual libfsim/JSBSim DLL identities. All 439 generated products match the accepted manifest in source, staged data and the installed package. This is concrete entrypoint and package evidence, without claiming all-design visual or equipment calibration acceptance.

The [final receipt](../data/sim-m02-final-closure-20261101.json), [88-design matrix](../data/sim-m02-engineering-completeness.csv), and `build/architecture-reviews/sim-m02-final-closure-20261101/delivery.md` identify the exact commands, original failures, independent reviews, current counts, source/product hashes, complete installed SDK, one HEAD-based exact-byte patch and successful patch replay. This dated closure supersedes the historical SIM-M02 nonacceptance entries below; those original failures remain available as history. Rotor fitness remains in its separate authorized deferral ledger.
<!-- SIM-M02 final closure end -->

<!-- SIM-M02 authorized closure boundary begin -->
The superseding 2026-11-01 user instruction defers **all rotor functional
issues at lowest priority**. This SIM-M02 functional closure covers 31 fixed-wing
and 53 store designs. The 4 rotor designs retain their all-design engineering
component/table records; their flight fitness is deferred, not passed.
The [versioned deferred-scenario ledger](../data/sim-m02-deferred-rotor-issues-20261101.json) names 28 rotor-only native
flight cases and 4 rotor-only Python methods. Mixed fixtures retain fixed-wing
conditions, World clock advances, assertions and thresholds through the
explicit tests-only `sim-m02-nonrotor-v1` population scope. All 16 fleet shards
remain and must collectively cover exactly 31 maintained fixed wings. Generic
API/lifecycle/layout/mathematical checks may include rotor types without
claiming rotor flight acceptance. Default test behavior remains all-family.
All 88 source/component/table applicability rules remain unchanged; no required
role becomes not_applicable. Closure execution reconciles the complete CTest
inventory minus the exact deferred native cases, separately accounts for
nested deferred methods/platform skips, and binds current source/binaries.
Use `tools/validation/aerodynamics/sim_m02_closure.py --preset PRESET --output
OUTPUT --powershell POWERSHELL_PATH --production PRODUCTION --git-source-root MAIN_CHECKOUT`
after building the preset and forcing all-design generation. The generation
preflight binds current authored/optional profile inputs, runtime products and
intermediate cache keys, and rejects unequal bin/tests/Python DLL copies.
HEAD-tracked historical output inputs and all explicitly consumed profile files
remain immutable during regression; other new derived reports/caches are
recorded separately from source files.
<!-- SIM-M02 authorized closure boundary end -->

<!-- SIM-M02 functional acceptance correction begin -->
Historical pre-deferral functional correction, 2026-11-01: SIM-M02 is not formally accepted. All88 designs were regenerated from current geometry/cards; the common engineering matrix has88 engineering-complete-uncalibrated and0 blocked designs, separately from functional acceptance. The retained native repairs cover body quadrature, physical rotor inertia, passive tail damping, mechanical hub work, capture damping with unity cruise gain, complete-cascade response/lookahead, waypoint transit, landing-position capture and momentum-aware terminal hover. Original headless control fixtures: 6/6, 498 assertions passed and0 failed. Current Release original selection: 6/9 cases, 18 failed assertions; native AAM: 12/13; installed package consumer: 1 executed. Original scenes, assertions and thresholds are unchanged. RPM-window survival, fall preconditions and the qbar matched-state premise remain unclosed; missing exact-model measurements are not the reason. Normal run.ps1 Release entry loaded the current DLLs and exact regenerated UH-1H XML, refused a poisoned exact XML, and flew a real-time orbit-to-hover diagnostic at3840x2160. This is runtime/entrypoint evidence, not whole-fleet physical or complete visual acceptance. All-table integrity checks cover7067 XML tables and3611335 cells; 1165 consistency predicates are not physical tests. Generation/install took1139.453s; the separately timed all88 basic-use pass took812.276s. That basic pass predates final native-only control changes and is not relabeled as a fresh final-DLL run. The earlier full headless attempt was interrupted with753/867 completed records and11 failures; no final full headless/Release pass is claimed. Current source/products, matrix, full-table inventory, raw failures, source review and replay patch: `build/architecture-reviews/sim-m02-functional-resolution-20261101/`.
<!-- SIM-M02 functional acceptance correction end -->

## Current boundary

Component applicability describes the project configuration, not runtime
implementation capacity. Unavailable required fuze/warhead implementations are
`required`, `present=false`, check failed and `blocked`. Armed store configurations
have not been approved as configurations without those roles. The former phrase
"inert release only" described missing execution and could not waive it. A formally
declared unarmed configuration may use `not_applicable`, with a passing check.
Missing component declarations cannot silently create an unarmed configuration.
Both engineering-complete labels are accepted engineering completion states;
optional measurement-calibration absence does not block SIM-M02.

The SDK owns sensing, designation, consent, release, propulsion, actual fin
motion, guidance, fuzes, collision and damage. Viewer only presents native
snapshots. A legal release, detonation, fragment Hit, functional damage and
mission outcome are distinct observations. Report each separately.

The [Hangar guide](../hangar-system.md#sim-m02-aerodynamic-engineering-completeness-gate)
is the single aerodynamic acceptance policy. SIM-M02 uses one generic
engineering-completeness gate for every design: VLM is the baseline, bounded
CFD and labeled engineering estimates are admissible, and exact-model measured
data is optional calibration rather than a blocker. There are no per-model
waivers. VLM/DATCOM/CFD outputs remain simulated estimates and do not establish
measured equipment accuracy. A working kill chain does not remove the separate
measured-calibration gap.

The common [88-design production and basic-use evidence](../../build/architecture-reviews/sim-m02-delivery-20261031/delivery.md)
records native flight, sampled instantaneous forces/performance, physical
component applicability, generated product hashes and an isolated installed
consumer. Its engineering status is independent of the optional measurement
ledger and of full equipment-envelope/kill-chain acceptance. Internal integration
substeps, finite actuator estimates and shared six-component VLM tables are
numerical/engineering construction evidence; none is a measured seeker, motor,
countermeasure or lethality calibration. The short flight and 210-condition native
sampling do not certify trimmed maximum performance or long-duration stability.
The current delivery separates production generation from the bounded basic-use
pass; it does not run a new full regression or the historical envelope-sampling
campaign. Historical results retain their original dates and outcome meanings.

The historical 86/2 common execution preceded the generic armed-store repair.
Current source has native armed unguided contact effects and a typed turbofan /
inertial-waypoint composition. The latter formally installs no terminal seeker;
it freezes an authorized launch waypoint and never reacquires truth after launch.
The [component matrix](../data/sim-m02-engineering-completeness.csv) is regenerated
from actual required components and current products. Historical 48-scenario
counts are diagnostic evidence, not current regression totals. No new visual
acceptance or measured lethality calibration is claimed.

## Native mechanism and card interpretation

| Selected profile | Concrete execution | Scope |
|---|---|---|
| AIM-9M | NonImagingIrSeekerV2 | Bearing history from unresolved intensity-weighted angular cells |
| AIM-9X Block I | ImagingIrSeekerV2 | Angular/spatial association and projected-extent continuity when features exist |
| AIM-7M | SemiActiveRadarSeekerV2 | Associated illumination and bistatic propagation, track loss/reacquisition; no invented range measurement |
| AIM-120C-7 | ActiveRadarSeekerV2 | Inactive/search/acquisition/tracking, admitted launcher updates and autonomous terminal sensing |

Here "measured" names a sensor-measurement input contract; it does not claim
measured equipment calibration. Newly authored engineering estimates and
simulations retain those explicit evidence bases.

All24 cards now select these measured families and MeasuredPnGuidanceV2:
seven nonimaging IR, five imaging IR, four SARH and eight ARH. The original
four profiles above established the family contracts; the remaining 20 now
use exact variant bindings checked against their card identity. The legacy
encounter/Cartesian providers remain supported for existing consumers, but
are no longer selected by the current AAM roster. Card/provider identities
determine which mechanism a campaign exercises.

World supplies time-stamped scene observables. V2 IR/SARH provide bearings;
ARH also receives ideal geometric range and radial speed. Referee attribution
is outside tracking/guidance inputs. IIR projected geometry is not calibrated
radiometric imagery, detector resolution or a temperature/signature model.
Missing features are unavailable, not proof that a contact is a decoy.

IR/SARH inertial navigation propagates only an admitted initial estimate and
explicitly enabled dated launcher updates. These remain separate from terminal
bearing samples. Future/invalid fixes do not poison the last accepted epoch;
reset clears both sensor and navigation history. This repairs software
execution, not fitted weapon-link delivery, GPS or navigation-error fidelity.

Meteor selects a distinct native solid booster plus atmosphere-fed throttled
ducted sustainer. Its explicit engineering cycle conserves mass, total enthalpy
and net momentum, including inlet ram drag. JSBSim integrates fuel, valve motion,
forces and moments. The original launch mass/CG/inertia and booster are retained.
Cycle inputs carry assumed uncertainty; no manufacturer cycle map or range
calibration is claimed. AIM-9X, IRIS-T and MICA EM/IR select finite equivalent
native thrust vectoring with explicit assumed angle/rate limits. The native
nozzle rotates existing thrust and produces its real current-CG moment;
it does not add an ideal guidance force.

On-rail providers belong to each installed store. Reads at one epoch are
non-consuming; release seeds a new independent instance from an admitted track.
Reset/loadout replacement clears relevant sensor histories. Missing selected
interfaces refuse release before inventory consumption. Existing public layouts,
enum values and native ownership contracts remain unchanged.

The [actual-fin and controller contract](weapon-controls.md) is the single source
for finite command response, signed control inversion, actuator limits, stable
physical fin IDs, native telemetry, transport, replay and visual hinge rotation.
The Viewer must prefer valid actual per-fin angles over equivalent-axis fallback.
Fallback animation is approximate and must not be reported as actual telemetry.

## Fleet execution and analysis

Run every authored AAM through the same declared native scenarios, retaining
card/model/source/executable hashes. Record launch refusals, tracking/support,
activation, motor motion, closest approach, fuze outcome, fragment Hit and target
functional response. Do not change scenarios or tolerances to turn a miss into
a pass. A rear-aspect restriction and an implementation defect are different
findings; identify the card's actual claim before judging an observation.

Analyze actual Mach/incidence/load histories, fin stops/rates/saturation and
controller availability. A short-range encounter cannot verify a card's maximum
range, maximum Mach or full overload envelope. Missing measured hardware data
remains unknown. Check generated fin IDs against physical GLB hinge nodes and
replay actual native fin samples through VehicleVisuals; inspect the corresponding
Viewer captures for each model. A screenshot alone does not establish actuator
or guidance fidelity.

The current all-AAM campaign result, its exact commands and visual evidence are
recorded in [status](../status.md). Earlier representative encounters do not
substitute for fleet coverage.

## All24 AAM repairs and remaining gates, 2026-10-30

The [repair report](../../build/architecture-reviews/aam-closure-20261030/index.html)
retains the original report, every failed intermediate attempt and the fresh
per-model native/visual results. The same 48 scene hashes produce zero reverse-flow
cases (previously eight). Fleet peak absolute alpha decreases from 179.9218 to
13.9515 degrees, and peak absolute load from 1178.2669 to 12.8935 g. This repairs
observed departures; it does not establish exact-equipment aerodynamic accuracy.
There are 37 detonations and 11 weapons still in flight at 40 s, with native
damage in 29 cases and native destroyed flags in seven. Lower counts are retained;
neither control stability nor a repair is defined by making every shot hit.

The initiating shared fault was a wing-normalization fallback being treated as
a physical main wing when applying Mach-dependent tail downwash. Wingless
strakes/fins now retain their coupled base VLM interaction without that fictitious
wing wake. All53 store products were regenerated through normal Hangar entrypoints.
PL-10 additionally selects finite-actuator state feedback and local incidence/Mach
Jacobians, as specified in the [control contract](weapon-controls.md).

Current validation: 60 Python tests; 12 native gates using 11 passing current
cases plus one targeted fixture retest; 13 internal native-control cases and
16 independent autopilot cases pass. The legacy numerical-equivalence test still
covers all53 stores with the retained legacy providers; the intentionally new
TVC, ducted-rocket and local-derivative physics have separate source/native checks.
Six public Python Range diagnostics check the four vector models' actual nozzle
forces/current-CG moments, stops/rates and stuck behavior, plus Meteor positive
airflow/fuel consumption and a no-fuel counterfactual. These are diagnostic
mechanism checks, not actor isolation or installed public-endpoint joint acceptance.

All24 live 40-second worlds were rerun fullscreen at the actual 3840 x 2160 primary
display resolution and real time 1x with complete effects. All96 captures were
inspected, and all24 native documents exactly match preflight. Native wall time
is 40.034--40.059 s per 40 simulated seconds. Production VehicleVisuals replay
checks 83,436 actual fin angles across all48 recordings: no angle/matrix mismatch,
rate violation or stop violation. The 48 unavailable-controller samples are one
initial release sample per case, not a sustained outage. The version-2 PL-10
telemetry refusal found by this replay was repaired without weakening assertions.

The shared generator repair preserves the two frozen store calibration gains:
MK82 holdout CD NRMSE 7.2435% versus baseline 7.4365%; A-22 scaled bare-container
CD 18.3121% versus 24.5713%. Original parameters and reference partitions remain
unchanged. The [fresh receipts](../../build/architecture-reviews/aam-closure-20261030/closure.json)
do not imply full-scale A-22 or unmeasured lift/moment validation.

Full closure remains incomplete. The fleet still needs the generic
engineering-completeness audit and aggregate generation/validation timing evidence;
measured aerodynamic calibration remains a separate optional label, not an
entry blocker. GPS error/aiding, fitted two-way weapon-link transport/feedback
and commanded dual-pulse management are not implemented by this repair.
The [feedback contract](weapon-feedback-validation.md#pending-complete-feedback-feature)
requires a reviewed versioned extension and an owner handoff; unavailable actor
feedback cannot be filled with referee truth. Hardware detector/servo, RF/IR,
ECM/ECCM, motor/vector maps and full maneuver/energy envelopes remain separately
unvalidated. The report's issue ledger distinguishes missing measurements,
open implementation and pending acceptance rather than claiming all issues closed.

Reproduce the current evidence after normal Hangar publication and UCRT64 build:

```powershell
.\run.ps1 -Mode python -Arguments @('-m','unittest','discover','-s','tools/models/hangar/tests','-p','test_storecontrol.py')
.\run.ps1 -Mode python -Arguments @('-m','hangar.aero_evidence','--out','build/architecture-reviews/aam-retest/bindings.json')
.\run.ps1 -Mode python -Arguments @('tools/validation/missile/propulsion_controls.py','--out','build/architecture-reviews/aam-retest/propulsion-controls.json')
.\run.ps1 -Mode python -Arguments @('tools/validation/viewer_perf/aam_campaign.py','--out','build/architecture-reviews/aam-retest/after','--stage','native')
$env:FSIM_AAM_CHAIN_MANIFEST = (Resolve-Path 'build/architecture-reviews/aam-retest/after/manifest.json').Path
& .\tools\launch\fsim.ps1 --preset ucrt64-release test --no-tests=error --output-on-failure --timeout 180 -j1 -R 'all authored AAM physical fins synchronize' --output-junit (Join-Path (Get-Location) 'build/architecture-reviews/aam-retest/fin-render.junit.xml')
.\run.ps1 -Mode python -Arguments @('tools/validation/viewer_perf/aam_campaign.py','--out','build/architecture-reviews/aam-retest/after','--stage','visual')
.\run.ps1 -Mode python -Arguments @('tools/validation/viewer_perf/aam_campaign.py','--out','build/architecture-reviews/aam-retest/after','--stage','report')
```

## All24 AAM campaign, 2026-10-30

This section is the frozen before-repair diagnostic snapshot. Its departures
and provider mismatches describe that original state; current repairs and
remaining gates are recorded above.

All 24 models ran two unchanged native SDK cases (5 km head-on; 3 km tail with a 300 m altitude offset), 6000 m and 250 m/s, release requested at 12 s, 40 s duration, seed 17. All 24 then reran one complete live fullscreen 1x tail case with the final mandatory-presentation program; each replaced four 3840 x 2160 frames. Live simulation took 40.032-40.059 wall seconds per 40 simulated seconds and all native outcome documents matched the preflight runs. The rejected windowed/accelerated captures carry no acceptance credit.

The 48 cases contain 39 detonations and nine weapons still in flight at 40 s. Native engagement records contain damage in 35 cases and a true native destroyed flag in nine; these flags retain native semantics, not a claim that every such aircraft exploded. Detonation does not imply fragment Hit, destruction or functional loss. All24 exact-model aerodynamic records remain blocked in this dated optional measured-calibration ledger; those labels do not decide SIM-M02 engineering acceptance. These short forward shots do not establish maximum range/speed/g, high off-boresight, LOAL, countermeasure or datalink fidelity. No plant/card/calibration parameter was fitted from these outcomes.

Actual v8 recordings were replayed through production VehicleVisuals: every physical fin ID, angle and independent hinge matrix agrees; authored actuator stops and rates agree. This verifies synchronization, not control stability. One controller-unavailable sample at release is distinguished from repeated loss of inversion after departure. Full raw telemetry and card/provider comparisons are in [the generated per-model report](../../build/architecture-reviews/aam-chain-20261030/index.html), with original scenes, hashes, logs and captures beside it.

| Model | Head / tail fuze miss (m) | Peak Mach | Peak incidence / load | Card and control finding |
|---|---:|---:|---:|---|
| AIM-120C-7 AMRAAM | 7.18 / 7.52 | 2.61 | 9.7 deg / 1.3 g | Midcourse estimates use an idealized launcher designation path, not a fitted weapon datalink. |
| AIM-120D AMRAAM | 7.49 / 7.53 | 2.65 | 9.7 deg / 1.2 g | GPS-aided navigation and a two-way weapon link in the description are not independently modeled by the legacy provider. |
| AIM-7M Sparrow | 5.28 / 7.61 | 2.03 | 3.7 deg / 2.7 g | Illumination-dependent SARH is selected; this pair does not validate loss-of-illumination rejection or CW/PD receiver fidelity. |
| AIM-9M Sidewinder | in flight, 12.56 / 2.54 | 2.98 | 3.3 deg / 2.6 g | The head-on case remains in flight after the close pass; the tail case detonates. All-aspect detection is not an all-aspect kill guarantee. |
| AIM-9X Block I Sidewinder | 1.05 / 7.28 | 3.06 | 10.2 deg / 2.9 g | The card explicitly discloses absent jet-vane TVC in the fin-only plant; these forward launch cases do not validate high off-boresight capability. |
| AIM-132 ASRAAM | 3.37 / 5.57 | 3.47 | 7.6 deg / 6.2 g | A 128x128 IIR detector, strapdown INS and lock-on-after-launch are not established by the legacy encounter provider or predesignated forward shots. |
| IRIS-T | 6.22 / 10.31 | 3.18 | 64.9 deg / 96.4 g | The card describes TVC; the current physical plant is fin-only. The tail case has a large incidence/load excursion despite detonation. |
| R.550 Magic 2 | 1.03 / 3.86 | 2.72 | 2.5 deg / 5.8 g | Basic infrared family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| Meteor | 8.72 / 7.71 | 1.93 | 11.9 deg / 1.0 g | The card describes a throttleable ducted rocket; the bound plant is a generated solid rocket. This is a propulsion mechanism mismatch. |
| MICA EM | in flight, 45.41 / in flight, 2350.10 | 1.94 | 179.9 deg / 448.8 g | Both cases depart and miss; terminal active-radar labeling does not cure physical loss of control. |
| MICA IR | in flight, 206.13 / in flight, 2352.24 | 1.92 | 179.4 deg / 446.4 g | Both cases depart and miss; the legacy IR encounter model does not establish imaging detector behavior. |
| PL-10 | in flight, 196.62 / in flight, 1777.39 | 2.57 | 178.8 deg / 1178.3 g | Both cases depart and miss; LOAL and an imaging detector are not demonstrated by predesignated legacy encounter shots. |
| PL-12 | 5.79 / 7.51 | 2.58 | 6.8 deg / 3.7 g | Basic active_radar family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| PL-15 | 9.21 / 7.75 | 2.40 | 12.0 deg / 2.3 g | A generated timed thrust schedule does not establish commanded dual-pulse management or its long-range benefit. |
| PL-8 | 7.81 / 4.22 | 2.54 | 2.4 deg / 7.2 g | Basic infrared family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-27ER | 11.45 / 8.02 | 2.21 | 11.4 deg / 1.2 g | Basic semi_active_radar family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-27ET | 9.19 / 12.61 | 2.21 | 12.2 deg / 1.2 g | Basic infrared family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-27R | 8.71 / 12.49 | 2.13 | 8.8 deg / 1.2 g | Basic semi_active_radar family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-27T | 8.49 / 7.66 | 2.12 | 10.0 deg / 1.2 g | Basic infrared family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-37M | 9.14 / 7.73 | 2.25 | 12.8 deg / 1.6 g | A generated timed thrust schedule does not establish commanded dual-pulse management or very-long-range performance. |
| R-60M | 3.31 / 5.44 | 2.58 | 4.9 deg / 4.8 g | Basic infrared family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-73 | 5.11 / 7.24 | 2.56 | 7.5 deg / 2.9 g | Basic infrared family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |
| R-77M | in flight, 235.93 / in flight, 352.83 | 2.22 | 179.9 deg / 224.9 g | Both cases depart and miss; a legacy active-radar encounter label does not implement or validate an AESA receiver. |
| Super 530D | 11.18 / 7.64 | 2.03 | 9.6 deg / 1.2 g | Basic semi_active_radar family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated. |

MICA EM/IR, PL-10 and R-77M show reverse-flow incidence and severe load excursions, repeated controller unavailability and no detonation in either case. The current controller refuses inversion when cos(alpha)*cos(beta) <= 0; this is an observed departure/refusal sequence, not proof of the initiating instability. IRIS-T tail reaches 64.9 deg and 96.4 g despite detonating. These are open physical-control defects; fin visualization has not concealed them. All other models still lack measured equipment response validation.

Commands through the normal entrypoints:

```powershell
./run.ps1 -Mode python -Arguments @('-B','tools/validation/viewer_perf/aam_campaign.py','--out','build/architecture-reviews/aam-chain-20261030','--stage','native')
./run.ps1 -Mode python -Arguments @('-B','tools/validation/viewer_perf/aam_campaign.py','--out','build/architecture-reviews/aam-chain-20261030','--stage','visual')
./run.ps1 -Mode python -Arguments @('-B','tools/validation/viewer_perf/aam_campaign.py','--out','build/architecture-reviews/aam-chain-20261030','--stage','report')
./fsim.cmd --preset ucrt64-release test --no-tests=error --output-on-failure --timeout 180 -j1 -L render
```

The final Viewer executable SHA-256 is `e73f91b0f853dd11e220553ecebc674368f0adcd26d4f50b142ef633b03986d9`; all24 live records identify that same executable.

Current renderer regression: 77/77, 170.47 s, `build/architecture-reviews/viewer-mandatory-20261030/render.log` and `render.junit.xml`. The actual-recording check uses `FSIM_AAM_CHAIN_MANIFEST` with the campaign manifest (48 cases) or `live-fin-audit/manifest.json` (24 live cases); select CTest `all authored AAM physical fins synchronize`. Current live retest: 24 models, 8,358 snapshots, 40,104 actual fin angles, zero angle/matrix/stop/rate mismatches; `viewer-mandatory-20261030/live-fins.log` and `live-fins.junit.xml` also include the mandatory-panel check (2/2). The original 48-case check remains separate: 15,015 snapshots and 70,192 angles. Previous additional gates (12/12 Release and 12/12 headless) remain historical evidence in `additional-release.log` and `additional-headless.log`, not fresh runs. Current CLI regression: 2/2, `.local/logs/cd8d901b4c504647bd3a92796a235c37-000.log`. Resource rejection, actual key input, native burst and aircraft shadow evidence are recorded in the [Viewer validation record](viewer-validation.md).

## Remaining equipment-specific gaps

| Area | Missing evidence or mechanism |
|---|---|
| Aerodynamics | Generic engineering-completeness audit: six-component and derivative tables, finite full-envelope coverage, provenance, installed-product evidence and aggregate runtime with a disclosed 20-minute target; measured exact-variant calibration is separate and optional |
| Fin/autopilot response | Measured actuator/control derivatives, mass/CG/inertia evolution, sensor latency/noise and per-variant dynamic response |
| ARH/SARH | Fitted RF/noise/clutter/scheduling inputs; per-candidate illumination and independent receiver cases |
| IR/IIR | Band-dependent signatures/background/atmosphere, detector and gimbal response, acquisition/measurement timing and countermeasure processing |
| TVC/propulsion | Measured jet-vane/nozzle and motor/cycle maps; current finite equivalent TVC and Meteor atmosphere-fed engineering cycle are implemented but uncalibrated |
| Fuze/lethality | Safe-arm/delay/response, measured fragment distributions and vulnerable-component response |
| Weapon feedback | Modeled fitted return path, delivered timestamps and LOST/SOFT_LOST/END_OF_FLIGHT lifecycle; referee truth is not actor feedback |
| Fused input/integration | Accepted sensor/entity provenance and modeled delivery; generic gateway ownership remains separate |

## Consolidated records and retained lessons

This document absorbs the former representative-weapon-models record: four
concrete profiles, on-rail/release identity, observable/attribution boundaries,
public compatibility and remaining detector/TVC/link limitations are retained
above. Its original delivery logs remain under
`build/architecture-reviews/representative-weapons/`.

The actual-fin contract absorbs the former missile-control-validation record:
finite signed inversion, current-CG/Euler/nonlinear-load feedback, absent-roll
handling, response assumptions, saturation/neutral refusal and independent jam
poses are retained there. Its historical logs remain under
`build/architecture-reviews/finite-missile-controls/`.

Other unique mechanism records remain linked rather than duplicated:
[seeker](weapon-seeker-validation.md), [fuze](weapon-fuze-validation.md),
[actor feedback](weapon-feedback-validation.md),
[physical event ordering](sim-m03-physical-chain-audit.md),
[fragment repair and native Hit](fragment-hit-repair.md),
[public provider index](payload-native-providers.md), and
[Viewer lifecycle/recording contract](viewer-validation.md).
Historical failures and test counts keep their dated scope; they do not replace
fresh native, visual or measured-equipment evidence.


<!-- SIM-M02 typed-source and replay contracts -->
An authored engine fixes propulsion applicability. A versioned air-breathing
store source with a missing, unavailable, not-applicable or incompatible
propulsion selection is rejected; generation cannot silently discard its
engine. Single-turbofan v1 rejects mixed rocket hardware. Legacy numerical
compatibility and explicit new-type contracts together cover every store;
rocket propellant and fed air-breathing fuel retain the same canonical mass
checks. No physical flight scenario, assertion or tolerance is replaced.

Full verification records every discovered headless/Release CTest case and
nested unittest method separately. An ordinary source export has no Git
metadata. Its original Git-only skip remains visible; the unchanged original
fixture may execute once on main only after its project code and declared input
bytes match the tested export. External execution is recorded separately and
never counted as multiple CTest executions. Uncovered skips remain failures.
The test-reference evaluator may reuse immutable arrays and per-state
interpolation work only when exact recorded-state equivalence is demonstrated.


<!-- SIM-M02 historical generated verification begin -->
Current production delivery: 88 engineering-complete-uncalibrated, 0 engineering-complete and 0 blocked designs. All 31 fixed wings, four rotorcraft and 53 stores (including all24 AAMs) use the same source-driven method and component rules. Aggregate production generation/install: 1045.836 seconds; current basic usability: 813.329 seconds. Current basic execution covers 88 native and 88 installed consumers, 2112 short-flight observations and 10 core/package fixtures (0 failed, 0 skipped). Production integrity inventories 16776 logical/serialized table entries and 3611335 XML coefficient cells. These are separately counted from document consistency predicates. This dated basic-use result does not establish formal SIM-M02 acceptance; no full regression or full-envelope dynamic coverage is claimed. Exact-model measurement calibration remains optional. Evidence: `build/architecture-reviews/sim-m02-delivery-20261031/`.
<!-- SIM-M02 historical generated verification end -->
