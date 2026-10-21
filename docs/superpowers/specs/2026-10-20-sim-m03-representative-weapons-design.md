# SIM-M03 representative weapon model differentiation

Date: 2026-10-20. Status: design for review; no implementation acceptance.
Source baseline: main `b42928644b77fda483927d86e3e87a296f7d5acf`.

## 1. Objective and assigned boundary

The user assigns AIM-9M, an explicitly identified AIM-9X block, AIM-7M and
AIM-120C-7 differentiation across component configuration and concrete native
implementation. The intended game benefit is observable differences in target
acquisition, tracking, support dependence, countermeasure interaction and
achievable control. Sources and missing data must be explicit. More model
names, extra C++ classes or passing software tests do not establish superiority
over another simulator.

This design uses the existing native component catalogue, sensor and tracking
primitives, World lifecycle and sole JSBSim flight integrator. Claude's SIM-M02
numerical investigation in AeroNumericsLab is an external dependency. This
package neither inspects that workspace nor imports an unaccepted result.
Generic MA schema, gateway, actor interfaces and installed generic-consumer
integration remain outside this assignment.

The current write window permits only this design document. Prospective source
areas in section 9 are not an implementation write window. A reviewed plan must
record the then-current main HEAD and exact file/symbol ownership before code
changes or native configure/build/test execution.

## 2. Established source findings

The maintained 53 store cards include 24 AAMs. All 24 select the same encounter
seeker, PN guidance, autopilot, fin allocator, actuator and solid-rocket provider.
All 24 also resolve the same five autopilot settings. The four seeker modes are
eight active-radar, four semi-active-radar, seven infrared and five imaging-IR
cards. These are current catalogue declarations, not independent equipment
validation.

`SeekerProviderV1` supplies range, notch, decoy-strength and illumination helper
functions; it has no sampled measurement or tracking update. In
`World::prepareEngagements`, eligible aircraft and decoys become Cartesian
position/velocity observations. `Guidance::update` holds contacts by observation
ID and treats the explicit decoy flag differently during association. Changing
the helper factory alone cannot produce an independent imaging or RF tracking
model.

`World::onRailSeekerObservation` constructs a temporary provider per query.
Simply adding state to that object would lose track history, and making a query
advance state would make repeated reads alter the simulation. The new path must
address both release and on-rail ownership.

The current AIM-9X card has no block designation. Its capability booleans are
explicitly assumed and do not enable launcher updates. That software declaration
alone is not evidence of the fitted real weapon. Its prose mentions imaging and
jet vanes while the selected allocator executes only the existing fin channels.

Relevant source contracts are [provider inventory](../../sdk/payload-native-providers.md),
[component configuration](../../sdk/payload-component-configuration.md),
[seeker/guidance adapters](../../sdk/payload-seeker-guidance-adapters.md) and
[current physical investigation](../../sdk/sim-m03-five-km-repair.md).

## 3. Equipment identity and admitted facts

Preserve existing store IDs and loadout references. Identify the represented
equipment in each card's designation, source records and implementation bindings.

| Existing store ID | Representative identity | Admitted qualitative mechanism | Data not established by those facts |
| --- | --- | --- | --- |
| `aim-9m` | AIM-9M; no invented modification number | Passive IR homing with background/countermeasure processing | Exact detector/scan implementation, band response, noise, tracking logic and response constants |
| `aim-9x` | AIM-9X Block I | Staring focal-plane-array seeker and jet-vane thrust-vector control | Detector dimensions, optical transfer function, processing thresholds, servo rates and jet-vane force/moment map |
| `aim-7m` | AIM-7M | Semi-active radar homing on supported illumination | Fitted receiver/waveform, bistatic sensitivity, tracking response and interruption tolerances |
| `aim-120c` | AIM-120C-7 | Inertial midcourse, launcher target updates and terminal active radar homing at family level | C-7-specific sensor, update-link, navigation and control performance |

AIM-9X Block I is the first explicit representation of the existing unsuffixed
card, not a claim that the old card was already a validated Block I model. The
user directs selection according to the actual setting. Current no-update
execution plus the early-variant public evidence supports this conservative
representation. Do not inherit Block II datalink or lock-on-after-launch claims.
Do not assign an operational flight-program revision or production lot without
evidence. Existing Block II scenarios, if subsequently supplied, need a distinct
authored variant rather than changing the meaning of the frozen Block I profile.

The USAF Sidewinder fact sheet distinguishes the M's background/countermeasure
improvements and the X's imaging seeker and jet-vane steering [R1]. NAVAIR's
2002 first-production report supports the early X's focal-plane array and jet
vanes [R2]. Its 2010 report explicitly identifies Block I and describes the
subsequent Block II datalink and lock-on-after-launch additions [R3]. Neither
report provides engineering rate or image-processing constants.

The USAF Sparrow fact sheet supports semi-active CW/pulse-Doppler homing [R4].
NAVAIR's AMRAAM description supports the midcourse/update/terminal architecture
[R5]; family descriptions do not establish C-7-specific numerical performance.

## 4. Implementation approach

Three approaches were considered:

1. Add equipment-named wrappers around the encounter seeker. This preserves
   compatibility cheaply but leaves the same observations, state transitions
   and discrimination mechanism. It does not satisfy this assignment.
2. Extend native execution with measurement-owned seeker models and use mature
   software as independent references. This is selected: it retains native
   authority and can distinguish mechanisms without importing another runtime.
3. Adopt an entire alternative simulation framework. This changes authority,
   dependencies and model assumptions without supplying equipment data. It is
   not selected.

### 4.1 Additive measurement and guidance contracts

Keep existing public signatures, layouts, enum values, V1 provider virtual
declarations and the legacy encounter path. Introduce a separate private
`SeekerExecutionV2` contract implemented alongside the existing provider contract.
V2 is an execution-interface version; the existing descriptor envelope remains
version one. Each new implementation has its own explicitly validated component
configuration schema and selection ID. A new registration may not masquerade
as the old encounter schema to bypass required slots or units.

The new contract receives time-stamped sensor scene inputs and returns sampled
measurements and a local track. It separates:

* Simulation scene inputs used to compute physical observations.
* Measurement time, receive time, frame, observables and their availability.
* Estimated track state, uncertainty availability, age, loss and reacquisition.
* Referee-only attribution retained outside association and guidance.

IR observables are bearings and image-derived features when actually supplied.
They do not contain exact target range or velocity. RF observables include only
the range, bearing and Doppler channels that the selected measurement model
provides. Missing covariance is explicitly unavailable; zero is not substituted
to imply perfect measurement accuracy.

Tracking cannot consult target identity, aircraft-versus-decoy labels, the
launcher's desired truth target or the old `Observation::decoy` flag. A native
scene may use geometry and signatures to generate measurements, but it must
not pass semantic truth labels to discrimination. Local track identities and
stable tie handling are owned by each seeker. Referee correlation is separate.

Add a corresponding private measured-guidance execution contract. IR guidance
uses measured LOS evolution and own navigation; it cannot reconstruct closing
speed or time to intercept from hidden target Cartesian state. Existing PN
constants can remain explicitly assumed where reused. A missing range-dependent
timing estimate stays unavailable. RF and midcourse guidance consume only
measurements and admitted estimates. No synthetic target state is invented to
make the old Cartesian observation API appear to be a bearing measurement.

### 4.2 Concrete seeker implementations

The following are proposed class names, not claims about existing classes or
reverse-engineered equipment software. Each has a matching header/source pair
under `components/implementations/seeker/` and its own behavioral fixtures.

| Concrete model | Required executable difference | Explicit fidelity boundary |
| --- | --- | --- |
| `NonImagingIrSeekerV2` | Directional IR measurement and tracking of unresolved signal within the admitted sensor gate; bearing-based guidance input | No invented reticle pattern or proprietary countermeasure algorithm |
| `ImagingIrSeekerV2` | Spatial sample/feature association and retained angular track history; independent sources affect the result through measured image structure | No semantic truth classification; no claim of imaging discrimination when image/feature inputs or required optical settings are unavailable |
| `SemiActiveRadarSeekerV2` | Receiver measurements depend on the actual associated illuminator, both propagation legs and current illumination; interruption changes measurements and track age | Geometry-only illumination is identified as a reduced-order mode; it is not calibrated received-SNR performance |
| `ActiveRadarSeekerV2` | Explicit inactive/search/acquire/track lifecycle; its own emitted sensing and measurements continue independently of launcher support after activation | Existing activation and range settings remain assumed until admitted; launcher estimates cannot become current seeker detections |

Shared coordinate transforms, finite arithmetic and filter primitives may be
reused. Sensor models must not be four aliases around one unchanged selection
algorithm. Keep a common autopilot where it implements the same supported
mechanism; inventing different gains merely to distinguish a designation is
prohibited.

### 4.3 Persistent ownership and reset

On-rail seeker state belongs to the actual installed store instance, using the
store index, immutable card binding and an installation generation. World
updates it once per configured sensor sample; repeated seeker queries read its
cached result. They do not advance filters, consume messages or draw RNG.
Release validates composition before inventory/ID effects and transfers or
initializes only compatible, current seeker state. A launch cue is an admitted
observation, not permission to acquire a hidden target.

Each released seeker and measured-guidance instance owns its mutable state.
Reset clears its permitted sensor/controller transient state without changing
existing Vehicle.reset authority, tasks or C2 ownership. Destruction, jettison,
removed stores, pooled-slot reuse and episode generations cannot retain another
instance's measurements or track history. New stochastic mechanisms require
stable component-local streams; legacy paths retain their existing draws.

### 4.4 Control, propulsion, links and effects

JSBSim continues to integrate actual forces, moments, actuators and flight.
No ideal acceleration or direct angular-state correction may be added to make
AIM-9X achieve an authored turn. Jet-vane commands need an admitted native
actuator and force/moment mapping. The present fin-only allocator is not renamed
as a thrust-vector implementation. Until that mapping exists, its omission is
explicit in the represented Block I profile and physical acceptance remains
open. Synthetic native plant fixtures may verify a future backend interface;
their coefficients cannot become equipment parameters.

Keep the existing motor, aerodynamic, mass/inertia and controller numerical
inputs unchanged in this package. AIM-120C-7 launcher updates remain explicitly
idealized until a fitted modeled link is delivered. A selected modeled-link
mode with missing inputs fails explicitly; it never substitutes the legacy
ideal path. This package does not add a truth-fed link or claim weapon return
telemetry. Freeze any later shared DMS/fused-report integration with its owner.

Preserve the current fuze, warhead and mechanical contact meanings. Acquisition,
detonation, fragment intersection, component failure and mission outcome remain
separate. The existing fixed-scene fragmentation failures remain active; no
fuze radius, burst geometry, damage threshold or test criterion is tuned to
manufacture an acceptance result.

## 5. Configuration and missing-data policy

Only the four selected cards migrate. Other cards retain their selected
implementations and numerical behavior. Preserve their IDs and the four
selected cards' existing sourced physical parameters. Resolve configuration
from the same immutable loaded card; never reopen a same-ID card from another
library during release.

Each migrated component records its concrete implementation ID, exact supported
configuration contract, represented variant, parameter references, basis,
sources, validity and limitations. Use the existing finite numeric parameter
contract for actual numbers. Do not put null into a required finite numeric
slot or fill a missing number with zero. Missing submodel inputs are stated
in model limitations and an explicit execution-mode choice; selecting that
submodel without its required inputs refuses composition.

Existing assumed game inputs can be retained as assumptions. They are not new
measurements, must not be relabeled published, and cannot be tuned to a desired
hit rate. New synthetic values belong only in clearly marked test fixtures.
Device noise, servo rates, optical resolution and processing thresholds require
source admission before equipment use. A paper demonstrating an algorithm does
not supply the corresponding weapon's coefficients.

The runnable approximation and full equipment model must remain distinguishable.
In particular, adding an IIR algorithm tested on synthetic imagery does not
justify marking production imaging discrimination available when World has no
admitted imagery input. Likewise, an implemented TVC command interface does not
establish that a selected physical jet-vane plant exists. A source class, a
selected instance and a validated equipment capability are separate evidence.

## 6. Mature references and reuse decisions

| Reference | Selected use | Limits |
| --- | --- | --- |
| Existing native radar detection, RF propagation and tracking | Reuse suitable mathematical primitives and a weapon-owned scene/track state | Do not copy fighter-radar default coefficients into a missile seeker or alter shared SIM-M01 semantics |
| JHU/APL homing guidance and flight-control papers [R6, R7] | Measurement/LOS reconstruction, homing-loop decomposition and independent analytical fixtures | General engineering references; no equipment-specific gains or sensor constants |
| Stone Soup 1.9.1 [R8] | Offline measurement, association and tracking comparison | MIT; no production Python dependency; not an equipment model |
| pyBSM 0.15.0 [R9] | Offline image-formation/degradation comparison when admitted sensor inputs exist | Apache-2.0; not a seeker tracker or target-signature library; upstream documents Unix/Linux testing |
| MIXR gimbal and default missile [R10] | Bounded servo/frame reference | LGPL-3.0 as published by the project; default missile uses robot-aircraft dynamics and is not adopted as the weapon runtime |

No external candidate was installed, compiled or executed for this design.
Before any implementation reuse, record the applicable source/license and
immutable version; dependency installation remains under D:/ENV. Source review
does not establish UCRT64 executable compatibility.

## 7. Required evidence

Regressions precede implementation. Expected failures must demonstrate an
actual absent mechanism or violated contract, not merely an absent class name.

| Evidence | Required observation |
| --- | --- |
| Card composition | Four cards resolve their intended concrete mechanisms and exact variant; wrong mode, unknown slots, unavailable required data and conflicting model bindings refuse atomically |
| IR information boundary | Changing hidden target range/velocity or semantic decoy attribution while preserving all admitted observations cannot change bearing-only guidance or association |
| Non-imaging versus imaging | Frozen synthetic scenes distinguish unresolved signal response from spatial association; ambiguous/unsupported imagery remains explicitly ambiguous/unavailable |
| Frames and finite tracking | Body rotation with fixed inertial LOS, bounded servo modes where configured, sample timing, observation dropout and reacquisition follow independently specified inputs |
| SARH support | Actual illumination interruption, wrong illuminator association, either masked propagation leg and restoration change the received observation/track state without truth updates |
| ARH transition | Pre-activation launcher estimates, search without a detection, acquisition, support loss after activation and stale updates remain distinct |
| Lifecycle | Repeated on-rail reads do not step state; two installed stores, two releases, reset, destruction, pooled reuse and independent Worlds cannot share transient state or RNG |
| Native production path | Actual card loading, preparation, consent/release, selected factories, World observations, measured guidance and real fin response execute together; a factory-only test is insufficient |
| Compatibility and assets | Existing V1/legacy consumers and unaffected cards retain their contracts; regenerated changes are limited to selected metadata, with motor and physical-model integrity checked independently |
| Diagnostics | Labelled native runs and replay show each representative mechanism and its missing capabilities; referee diagnostics cannot become actor observations |

Use bounded preset-native gates with actual discovered counts and
`--no-tests=error`, and only while holding the exclusive native slot. Run the
feature-owned installed consumer checks through the existing public API without
taking over Claude's generic compatibility package. Preserve current failed
physical assertions and report them separately from software mechanism checks.

## 8. SIM-M02 dependency and completion criteria

Source admission, component configuration, measurement interfaces, synthetic
sensor/tracking regressions and native lifecycle integration can progress
without accepting a new numerical result. Physical control and flight evidence
must consume a reviewed SIM-M02 handoff that identifies model/version, units and
frames, operating envelope, mass/CG/inertia evolution, forces/moments/control
authority, reference cases, uncertainty and numerical residuals.

Do not edit AeroNumericsLab, vary its environments, run its solvers or occupy its
build processes. Do not modify the existing aircraft designs, force/moment
tables or generated physical inputs during this feature's independent slice.
Any required later plant integration needs a recorded handoff and exclusive
write/build window.

The package can establish four explicitly configured executable mechanism
models with current production-path evidence. Full four-weapon physical
acceptance additionally requires admitted sensor/signature/actuator/link data
and the relevant SIM-M02 plant. Missing IIR inputs, AIM-9X TVC, modeled AMRAAM
updates and existing fragmentation gates must remain visible; partial software
delivery cannot be called complete SIM-M03 acceptance. Superiority over DCS
requires a later defined comparison and is not inferred from this package.

## 9. Prospective implementation areas

The implementation plan must turn these areas into exact path/symbol windows:

* New private measurement/guidance interfaces and concrete classes under
  `src/simulation/mission_systems/components/` and `weapon/`.
* Existing component registration, composition and validators, including
  `registration/SeekerProvidersV1.cpp`, `registration/GuidanceProvidersV1.cpp`,
  `weapon/Seeker.cpp` and `weapon/Composition.cpp` as required by the new schemas.
* `src/simulation/runtime/world/{World.h,World.cpp,WorldEngagements.cpp}` for
  installed-store/release ownership and observation execution.
* `assets/stores/{aim-9m,aim-9x,aim-7m,aim-120c}.json` only.
* `tools/models/hangar/hangar/components.py`, its direct tests and the selected
  generated model metadata through the established generator. Physical XML
  functions and engine/nozzle bytes remain outside numerical changes.
* Feature source/test registration in Mission Systems CMake and
  `tests/CMakeLists.txt`; focused component and production-path fixtures.
* Feature-owned SDK documentation and the relevant status/allocation record
  after verification. No generic schema, public layout or transport changes.

## 10. Primary references

All web sources below were inspected on 2026-10-20. Publication and access dates
are distinct. Existing card access dates are not substituted for publication
dates. Algorithm references establish mechanisms, not fitted equipment values.

* **R1:** U.S. Air Force, [AIM-9 Sidewinder](https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104557/aim-9-sidewinder/).
* **R2:** NAVAIR, 2002-05-23, [First production AIM-9X rolls out to the warfighter](https://www.navair.navy.mil/node/7616).
* **R3:** NAVAIR, 2010-05-19, [WD helps Sidewinder reach production milestone](https://www.navair.navy.mil/node/10456).
* **R4:** U.S. Air Force, [AIM-7 Sparrow](https://www.af.mil/About-Us/Fact-Sheets/Display/Article/104575/aim-7-sparrow/).
* **R5:** NAVAIR, [AMRAAM](https://www.navair.navy.mil/product/AMRAAM).
* **R6:** Palumbo, Blauwkamp and Lloyd, [Basic Principles of Homing Guidance](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_Principles_Rev2018.pdf), JHU/APL Technical Digest 29(1), 2010, revised PDF; measurement and LOS reconstruction discussion.
* **R7:** Jackson, [Overview of Missile Flight Control Systems](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf), JHU/APL Technical Digest 29(1), 2010, pp.9-24.
* **R8:** DSTL, [Stone Soup 1.9.1 radar documentation](https://stonesoup.readthedocs.io/en/v1.9.1/stonesoup.sensor.radar.html); tag resolves to commit `a4336b920a799cfe0a77ecb05867c5deeb371c7a`, with [MIT license](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/LICENSE).
* **R9:** Kitware, [pyBSM 0.15.0 README](https://github.com/Kitware/pybsm/blob/6191fdc32997d9f5e9c4df741a903974fe407b1c/README.md), commit `6191fdc32997d9f5e9c4df741a903974fe407b1c`; [Apache-2.0 license](https://github.com/Kitware/pybsm/blob/6191fdc32997d9f5e9c4df741a903974fe407b1c/LICENSE).
* **R10:** MIXR, commit `1b4029419e382bb532f1e142f9c490dc9adc27f7`, [gimbal implementation](https://github.com/doughodson/mixr/blob/1b4029419e382bb532f1e142f9c490dc9adc27f7/src/models/system/IGimbal.cpp) and [default missile implementation](https://github.com/doughodson/mixr/blob/1b4029419e382bb532f1e142f9c490dc9adc27f7/src/models/player/weapon/IMissile.cpp); [official license](https://www.mixr.dev/license.html) and [release/support notes](https://www.mixr.dev/downloads.html).
