# SIM-M03 reference decisions and complete-package acceptance

Date: 2026-10-20. Source-review baseline: main
`fabeaa4fe56d1a878a5f3d0614729d8e940d5719`.

Complete SIM-M03 physical acceptance remains open. Mature reference software can
supply mechanisms and independent checks, but cannot supply an actual weapon's
missing sensor, actuator, link or vulnerability data. A successful 30 km encounter
would establish that encounter's result under its admitted assumptions; it would
not close those separate gates.

Sections 1-5 record the source and primary-reference review. No external
candidate was installed, built or executed. Section 6 records the subsequent
independent SDK corrections and current native verification; these do not
change generated models, equipment values or public contracts.

The governing scope is the [simulation plan](../architecture/modules/simulation/plan.md)
and [work allocation](../architecture/modules/simulation/ms-work-allocation.md).
Existing mechanism evidence remains in the [finite-control record](missile-control-validation.md),
[physical-chain audit](sim-m03-physical-chain-audit.md),
[seeker record](weapon-seeker-validation.md) and
[feedback record](weapon-feedback-validation.md). Their dated software results
must not be relabeled as new verification or calibrated equipment acceptance.

## 1. Mature implementation decisions

| Area | Reference and finding | Decision for this SDK |
| --- | --- | --- |
| Actuators and flight integration | JSBSim's versioned [FGActuator](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/flight_control/FGActuator.h) documents lag, rate limits, deadband, hysteresis, stops and failures; its header declares LGPL v2 or later. | Retain the integrated JSBSim mechanism and sole native flight integrator. Preserve the project's actuator initialization/reset patch. Admit actual response coefficients separately; an upstream example is not missile calibration. The v1.3.3 source is a reference, not an upgrade instruction. |
| Coupled rigid-body verification | Jackson's flight-control paper, printed p.13, gives the coupled Euler equations. Versioned [JSBSim FGAccelerations](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/FGAccelerations.cpp) implements the native rigid-body calculation. [NASA NESC's atmospheric cases](https://nescacademy.nasa.gov/flightsim/2015) publish specified tumbling/damped bodies, wind, aircraft and rocket models with independent trajectory results; [NASA SimuPy Flight](https://nasa.github.io/simupy-flight/nesc_test_cases/index.html) supplies another implementation. | The autopilot must account for `omega cross (J omega)` as well as aerodynamic torque. Use independently constructed equations and finite-actuator response cases. NASA cases can qualify flight/environment numerics; no NASA trajectory was executed here and none supplies missile control derivatives. |
| Active seeker detection and tracking | Existing native `radar::Radar` already supplies dwell scheduling, received-SNR detection, measurement errors and `track::MultiTargetTracker`. Its `Scene` separates sensor physics from the owning runtime. [Stone Soup 1.9.1 radar documentation](https://stonesoup.readthedocs.io/en/v1.9.1/stonesoup.sensor.radar.html) exposes measurement covariance, sensor frames, dwell/FOV and clutter; its [versioned license](https://github.com/dstl/Stone-Soup/blob/v1.9.1/LICENSE) is MIT. | Prefer reuse of the existing native radar/track components through a weapon-specific private scene and admitted specification. Stone Soup is a suitable offline measurement/tracking comparison reference. Do not import fighter-radar defaults as seeker data or add a Python runtime dependency. |
| Stabilized seeker and homing-loop structure | Palumbo, Blauwkamp and Lloyd, [Basic Principles of Homing Guidance](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_Principles_Rev2018.pdf), JHU/APL Technical Digest 29(1), 2010, Rev2018 PDF, printed pp.29-32, separates body/gimbal/inertial angles, seeker tracking lag, LOS reconstruction and guidance filtering. Jackson's [Missile Flight Control Systems](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf) separates guidance, autopilot, actuator, airframe and IMU. | Use these as engineering references for interfaces and independent synthetic cases. They establish neither fitted equipment constants nor this SDK's performance. Test body rotation with a fixed inertial LOS, finite tracking response and achieved acceleration separately. |
| Gimbal kinematics | MIXR's [IGimbal header](https://raw.githubusercontent.com/doughodson/mixr/master/include/mixr/models/system/IGimbal.hpp) and [servoController](https://raw.githubusercontent.com/doughodson/mixr/master/src/models/system/IGimbal.cpp) distinguish servo modes, mechanical rates, position limits and body/gimbal transforms. | Useful reference for bounded gimbal kinematics. Do not adopt example rates or a complete replacement framework. The inspected `master` source is mutable and needs an immutable pin before reuse. |
| Complete alternative missile runtime | MIXR's [IMissile implementation](https://raw.githubusercontent.com/doughodson/mixr/master/src/models/player/weapon/IMissile.cpp) identifies its default guidance/dynamics as Robot Aircraft behavior: it commands heading/pitch/velocity, directly integrates angular motion, and uses target truth in its default fuze path. | Do not replace this SDK's finite-fin, JSBSim and sensor-authority path with that default missile. A mature framework can intentionally use a lower-fidelity model for another purpose. |
| Validation evidence | [NASA-STD-7009B](https://standards.nasa.gov/sites/default/files/standards/NASA/B/1/NASA-STD-7009B-Final-3-5-2024.pdf), sections 4.2.3-4.2.8, separates verification, validation, sensitivity and uncertainty activities. | Retain source admission, frozen conditions/tolerances, numerical residuals and applicability limits alongside software checks. This is evidence guidance, not replacement equipment data or a new project acceptance standard. |
| Fragment geometry oracle | [BRL-CAD NIRT](https://brlcad.org/docs/cad/articles/nirt.html) reports ray partitions, distances and normals. Its documented inside-origin behavior backs the ray to an entry. | Suitable offline geometry comparison after executable/source qualification. Clip reference partitions to the forward half-ray; a backed-up entry is not an SDK forward crossing. No executable was installed or exercised. |
| Blast reference | blastFoam v6.2.0 resolves to [`4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c`](https://github.com/synthetik-technologies/blastfoam/tree/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c). Its [README](https://github.com/synthetik-technologies/blastfoam/blob/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/README.md) requires OpenFOAM-9; its COPYING is GPLv3 or later. The upstream [Kingery-Bulmash wedge case](https://github.com/synthetik-technologies/blastfoam/blob/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/validation/blastFoam/KingeryBulmash/KingeryBulmash_wedge/README.md) uses a 10 kg hemispherical TNT surface burst and pressure/impulse/arrival comparisons at 1-10 m. | Candidate offline blast-method reference, not a runtime dependency or a free-air missile/aircraft-vulnerability calibration. The [spherical Sedov case](https://github.com/synthetik-technologies/blastfoam/blob/4e6ee07a0c1fc4629ee7206804f4f1fe802ec64c/validation/blastFoam/Sedov/Sedov_3D/README.md) is a separate numerical strong-shock check and requires deposited-energy correction when its mesh changes. No solver run is claimed. |
| Connected rods and initiation | The [JHU/APL Talos continuous-rod paper](https://secwww.jhuapl.edu/techdigest/Content/techdigest/pdf/V03-N02/03-02-Brown.pdf) describes connected expanding rods and directional initiation with closing-rate-dependent delay. | Current independently sampled rod zones do not establish a connected-ring model. Talos-specific values cannot calibrate another fitted weapon. Admit topology, delay and structural response separately. |

MIXR's [official downloads](https://www.mixr.dev/downloads.html) label 26.10 as a
Linux-only developmental release and 18.01 as the current release; its
[official license](https://www.mixr.dev/license.html) publishes LGPL v3. This
review establishes no MIXR/UCRT64 executable compatibility. No framework runtime
adoption is selected.

## 2. Native seams and present information boundaries

| Exact path and symbols | Inspected behavior | Bounded continuation |
| --- | --- | --- |
| `src/simulation/mission_systems/radar/Radar.h`: `Spec`, `Scene`, `Radar::step`, `Radar::report`; `radar/Radar.cpp`: `Radar::report` | The radar receives scene truth for simulation physics, creates detections, tracks measured estimates and predicts reports to the requested time. `Detection::truth` is explicitly scoring metadata. | A private weapon-owned radar instance can use the same detection/measurement/tracking machinery after its input profile and cadence are admitted. Keep scoring truth out of association, guidance observations, actor reports and masks. |
| `src/simulation/mission_systems/track/Tracker.h`: `Detection`, `TrackerSettings`, `MultiTargetTracker` | Native filtering, GNN/JPDA association, confirmation, coast/deletion and covariance already exist. | Reuse applicable components. Angle-only IR observability and unknown range require a suitable measurement contract; a radar range-bearing model cannot silently create IR range/velocity knowledge. |
| `src/simulation/runtime/world/WorldEngagements.cpp`: `World::prepareEngagements` | Aircraft truth becomes exact position/velocity observations after range/aspect/RCS/notch/terrain gates. One local illumination flag admits SARH observations. A valid launcher-owned designation directly refreshes guidance. | Replace those approximations only through admitted sensor/received-illumination and modeled-link features. Current body-axis eligibility is already explicit; do not reinstate a velocity-axis seeker or an ideal force/moment execution path. |
| `src/simulation/mission_systems/weapon/Guidance.h`, `Guidance.cpp`: `Observation`, `Input`, `Guidance::update` | Guidance consumes caller-admitted observations, retains/extrapolates its estimate and emits acceleration demand. It does not integrate weapon motion. | Add private measurement/timing contracts only after freezing their validity and sampling semantics. Keep guidance demand separate from achieved acceleration and current observation separate from retained memory. |
| `src/simulation/runtime/world/World.cpp`: `estimateOf`, `refreshDesignations`, `publishNative`, `stepDms` | Local estimates come from predicted radar reports; network estimates come from delivered `LinkTrack` reports. Native entity publication uses platform radar tracks. | Preserve measured input and delivery provenance. A missing fused source contract cannot be filled from the designated true entity. |
| `include/fsim/Comm.h`: `EntityReport`, `LinkTrack`; `src/simulation/communications/dms/Dms.h`, `Dms.cpp`: `Dms::send`, `deliver`, `picture` | `EntityReport` is an un-fused source radar track. DMS already models queue/rate, range/LOS, configured loss, delivery, duplicate suppression and retained received reports. | Reuse these delivery concepts through an agreed new native weapon-link contract. Existing platform DMS membership does not establish fitted missile transmit/receive equipment, a return channel or fused entity semantics. |
| `src/simulation/api/Agent.cpp`: `Agent::strikeStatus`; `src/simulation/runtime/services/MaReports.cpp`: `equipmentStrikes` | Direct and collection strike reports redact targets outside the platform's own designations. Released weapon IDs are correlations, not telemetry permissions. | Preserve these rules. Filtering referee `WeaponRecord` values by launcher is not a modeled feedback implementation. |
| `include/fsim/MissionSystems.h`: `WeaponPhase`, `WeaponEnd`, `WeaponRecord`; `World.cpp`: `World::endWeapon` | Seeker `Lost` can reacquire; native terminal reasons and bounded referee terminal records remain distinct. | Preserve existing enum values and meanings. Assessment publication cessation needs its own accepted state mapping and retained delivered terminal report; do not equate seeker loss, detonation, damage, crew loss and mission success. |

The review checked an apparent local-track age concern: `Radar::report` predicts
its track state to `now` before the designation consumes it. The local path does
not merely relabel an unchanged last-detection position as fresh. The network
path separately extrapolates the delivered report from its sample time. These
are retained estimates, not evidence of a physical weapon uplink.

No new normal-envelope seeker defect was established by this source-only review.
Two boundary candidates remain unverified: the runtime's unconfigured
`range < 1 m` observation exclusion can suppress tracking at a close armed
start, and finite but extreme vectors can overflow Guidance's squared norms or
PN intermediates. A synthetic native regression must establish the required
behavior and actual failure before either becomes a correction or acceptance
claim. Neither candidate explains the 30 km result without trajectory evidence.

## 3. Complete-package acceptance matrix

"Mechanism present" below means inspected production structure or already dated
evidence, not a new passing test. Every physical row needs its exact variant,
units/frames, source/version/hash where applicable, admitted envelope, uncertainty
and independent residual tolerance. Synthetic profiles may verify mechanisms
when labeled explicitly; they cannot grant actual-model support.

| Gate | Present basis | Missing admitted inputs or contract | Required closure and owner seam |
| --- | --- | --- | --- |
| Flight plant and evolving mass properties | JSBSim released-weapon integration and generator-owned models | Accepted SIM-M02 forces/moments over Mach/altitude/incidence; mass, CG, inertia and propulsion evolution; sign/reference conventions and uncertainty | Accept the independent numerical owner's handoff before physical tuning. Static convergence alone does not establish dynamic or control authority. |
| Fin/autopilot response | Actual finite individual-fin execution and geometry-derived loads are recorded | Per-fin force/moment derivatives, mixing and limits; actuator rate/lag/load/failure response; controller and achieved-response references | Native fin commands must produce independently compared acceleration/attitude response through JSBSim. Preserve reset/failure behavior and absence of ideal reaction fallback. Codex consumes the accepted M02/control inputs. |
| Guidance performance | PN, loft, acceleration bounds and retained-estimate steering exist | Named variant guidance assumptions, timing, achievable demand envelope and accepted encounter references | Freeze reference encounters before tuning; measure demand, actual response, saturation, miss and cadence convergence. A hit alone is not calibration. |
| Active radar seeker | Range/RCS/notch/terrain admission and body-axis gimbal eligibility exist | Frequency/power/aperture/pattern, waveform/processing/noise/clutter, scheduling/latency, measurement covariance and track/acquisition behavior | Use received-signal/measurement logic and independent detection/tracking cases. SIM-M01 sensor ownership and Codex weapon integration meet at admitted inputs. |
| Semi-active seeker | Source-scoped local STT support and loss/reacquisition exist | Candidate-specific transmitter-target-receiver geometry, illumination pattern/waveform, both propagation legs, received return/threshold and supported interruption modes | Prove an illuminated designated region does not authorize unrelated returns. Cooperative/cross-source support needs explicit association; matching track numbers cannot establish it. |
| Infrared seeker | Shared aspect/afterburner envelope and finite body field of regard exist | Band-dependent target/decoy signatures, atmosphere/background/noise, angular measurement model, finite gimbal/track response, sampling and latency | Admit physical sensor inputs with the SIM-M06/signature owner. Do not grant exact target range/velocity from an angular sensor or calibrate with the present aspect factors. |
| Countermeasure interaction | Time-aligned analytic particles and native observation/fuze paths are recorded | Cartridge/signature/aerodynamic variation, sensor discrimination, clutter/environment effects and uncertainty | Independent synthetic motion/expiry tests verify mechanics; actual seeker/decoy comparison requires admitted sensor and cartridge inputs. Preserve deterministic streams. |
| Fuze | Armed/live interval clipping, native-substep paths and explicit terminal outcomes exist | Actual safe-arm, sensor geometry/scan/response/delay and initiation evidence | Independently compare armed/unarmed, near miss, grazing, expiry/contact and cadence cases. Preserve detonation as a native event, separate from a kill. |
| Warhead and vulnerable components | Native fragment/blast/intersection/component failure mechanisms and attached-geometry filtering exist | Explosive/case/fragment or connected-rod distributions, material stacks and structural/component vulnerability evidence | Compare distributions, penetration/loading and response under frozen uncertainty. Detached-body damage needs its own geometry/state/impulse/event contract. |
| Weapon uplink | Launcher-owned designation support is currently idealized | Exact fitted receive path, direction, payload, endpoint binding, update period/latency/range/loss/resource and stale/cancel behavior | New native link must deliver the allowed estimate before Guidance can consume an update. Codex owns the weapon feature; shared DMS changes require an exclusive agreed window. |
| Weapon flyout assessment and return | Actor feedback is explicitly unavailable without modeled transport | Fitted return capability and content; assessment identity/accept-reject/lifetime; report period/time/validity; exact LOST/SOFT_LOST/END_OF_FLIGHT mapping | Produce onboard reports, model their delivery and retain only delivered platform knowledge. Versioned public access and a reviewed generic-owner extension are required. |
| Fused weapon input | Un-fused measured radar/DMS tracks and own designations exist | Accepted SIM-M01 fused Entity ID/source/status/identity evidence/kinematics/covariance/time/loss contract | Claude delivers the fused report feature; Codex consumes it after actual platform and weapon delivery. Missing/stale/lost/unknown stays explicit. |
| Fleet and integration | Current native/public compatibility and diagnostic evidence are separately dated | Complete fitted-variant support inventory, admitted profiles and independent cases; current required compatibility, installed provenance, deterministic replay/reset and Viewer cases | Root serial native owner records current commands and nonzero counts. Claude retains generic gateway/schema/registration and installed generic-consumer ownership. |

The allocation's weapon exchanges also retain separate SIM-M04 DLZ/WEZ and
SIM-M10 store/task/MDF work. They must not be claimed complete by this SIM-M03
review. The specific M03 flyout and fused-input duties are the tracked section
7.8 entries 1.2.10.10 and 1.2.10.12. This review did not freshly inspect the
original MS PDF/schema and makes no new OMS wire-conformance claim.

## 4. Implementable native weapon-link and fused seams

The following is a feature boundary proposal, not a frozen wire schema or an
equipment fit declaration. Freeze it with the responsible owners before public
implementation.

1. **Admit a fitted link profile.** Record exact weapon variant and launcher
   compatibility, receive/transmit directions, allowed payloads, modeled
   period/latency/range/loss/resource behavior and source of every value.
   Unsupported fits remain unavailable. A sensor class name, a launcher network
   membership or an active seeker does not imply a return transmitter.
2. **Bind native endpoints at release.** The native store/release path binds the
   weapon to its permitted launcher/service and profile. Request documents and
   arbitrary actor bytes cannot replace sender identity, weapon ownership,
   target identity or equipment capabilities. Preserve release consent and
   activity/weapon correlations.
3. **Separate support production from delivery.** The launcher may construct an
   update only from its permitted retained report. Model its transmission,
   capacity, delay/loss and actual delivery to the live weapon. Only delivered,
   accepted updates refresh Guidance. Local native seeker measurements remain a
   separate onboard sensor path; lost uplink does not silently disable a valid
   onboard observation.
4. **Consume the accepted fused report contract.** Preserve fused Entity identity,
   source/provenance, identity evidence, state/loss, kinematics, sample/receive
   times and validity/uncertainty. Do not extend `comm::EntityReport` in place:
   it is the existing public un-fused type. The native consumer must reject
   wrong ownership, wrong generation and invalid/future/expired updates before
   state mutation. Claude's SIM-M01 output is a dependency, not an invitation to
   invent the report here.
5. **Generate and deliver onboard feedback.** An assessment request receives an
   explicit native accept/refuse result for its fitted profile. Onboard state
   produces only the allowed report content. A modeled return message enters
   the launcher's retained reports after delivery, with original sample and
   delivery times. Referee holdings/target IDs, true damage and mission outcome
   cannot populate that report. Freeze cessation and cancellation behavior,
   including the final report when a weapon is removed before pending delivery.
6. **Expose additive, versioned public access.** Define reviewed C++/C99/Python
   types and entry points with non-consuming count/value reads and existing
   sized-buffer conventions. Preserve old layouts, enum numbers, default
   permissions, error semantics and generation/lifetime rules. The frozen
   `engagement.reports.v1` stays `unavailable / no_modeled_feedback` with null
   weapon telemetry. Claude integrates a reviewed extension into generic
   registry/gateway/snapshot behavior; a local native implementation cannot
   mark an uninstalled extension installed.
7. **Close the native feature before the generic handoff.** Exercise admitted and
   unsupported fits, no-link/one-way/two-way profiles, loss/delay/duplicate/reorder,
   assessment retry/cancel/expiry, unrelated platform/service refusal, bounded
   retention, terminal removal, deterministic worker/replay and full-reset stale
   handle rejection. Correlate public reads with actually delivered packets.

Existing `Dms::send`/`deliver` can provide a modeled communication substrate,
but weapon-native payload production/consumption, assessment state and retention
are missing features. Reusing that substrate needs a reviewed topic/dispatch and
lifetime seam with its owner; this proposal does not reserve new topic numbers,
change DMS public types or transfer generic feature ownership.

Detached-body damage also needs an explicit cross-body contract. A piece needs
immutable source geometry/component identities, transferred damage/fire and
fuel/stores state, piece-local references, deterministic random-stream ownership
and piece-owned events. Present assessment samples fragments separately for each
victim. Cross-body shielding therefore additionally needs shared detonation
fragment identities and globally ordered intersections. Merely adding Debris to
the victim loop would establish neither state ownership nor fragment conservation.

Public equipment evidence supports narrow fit claims only. The
[NAVAIR AIM-9X description](https://www.navair.navy.mil/product/AIM-9X-Sidewinder)
states that Block II has a datalink and an imaging infrared seeker, while listing
range and speed as classified. It does not publish return-report content or
latency. The [Raytheon AMRAAM description](https://www.rtx.com/raytheon/what-we-do/air/amraam-missile)
states inertial guidance, midcourse updates and onboard active radar; that does
not establish the same return capability for every variant. Neither source
admits numerical link/seeker coefficients here.

## 5. Verification and disposition of this review

The branch and HEAD above were read immediately before creating this document.
The write window contains only this new file. Source inspection and browsing
produced no build, native test, installed-consumer, performance or Viewer result.
No AeroNumericsLab delivery, external worktree or numerical-owner input was
consumed or accepted.

The immediate development decisions are to retain JSBSim finite control
execution, reuse applicable native radar/tracker mechanisms, refuse a wholesale
MIXR default-missile substitution, and keep missing physical/link/fused inputs
explicit. Independent correctness work can continue under its own scope and
serial gates. Complete physical SIM-M03 acceptance requires the unresolved rows
above and current integration evidence.

## 6. Historical frozen-state verification, 2026-10-20

The [implementation allocation](../superpowers/plans/2026-10-20-sim-m03-reference-continuation.md)
records the inherited delta, exact disjoint write windows and root serial native
slot. The inherited [30 km diagnosis](thirty-km-engagement-diagnosis.md) was
reviewed and verified, preserving its nonlinear native-load feedback and ARH
search steering corrections. New corrections are:

| Correction | Independent evidence |
| --- | --- |
| Diagonal-inertia Euler coupling in autopilot trim and pitch/yaw/roll rate inversion | Signed asymmetric-inertia cases reconstruct achieved angular acceleration from actual commanded aerodynamic torque. Separate constant-rate equilibria retain direct fin force, and overflowing required coupling returns unavailable/neutral. Each new behavior failed before its correction. No controller gain, response time, limit or ideal reaction was added. |
| Forward fragment crossings for inside/on-box and on-plane origins | Synthetic stopped fragments transfer initial momentum to the correct forward skin; entry and exit use independent face normals. A 0.5 mm surface case previously increased 270 J to 270.00435 J through negative drag distance. Event distance and drag progression are now nonnegative and monotonic. |
| Nested/overlapping closed-body containment | Independent review found an even-number parity error. The added case failed before correction; explicit entry/exit occupancy counts preserve component-before-exit ordering, including after leaving an inner skin. Six fragment-boundary cases include preservation controls. |
| Native encounter event assertion | The unchanged 5 km scene detonates with zero sampled fragment strikes and weak blast loading. The fixture requires native assessment and checks actual hit/failure event semantics. It does not manufacture damage from detonation. The separate swept-damage positive hit control remains. |

Final native source/test state was frozen and independently reviewed with no
remaining actionable findings. All 10 changed source/test files and 250 tracked
model/card/generator files were fingerprinted; verification after both presets
found **zero hash mismatches**. Generated models, equipment cards, motors,
controller constants, fuze radius and original encounter conditions are unchanged.

| Historical gate | Result |
| --- | --- |
| Complete UCRT64 Headless build | Passed |
| Focused numerical GREEN | 11/11 CTest cases, 0.35 s; the autopilot executable contains 12 passing internal cases |
| Complete discovered Headless inventory | **744/746 passed**, 1076.82 s; two retained failures below, no skipped cases |
| Complete render-enabled UCRT64 Release build | Passed |
| Release focused native/public/installed/generator/contract selection | **56/56 passed**, 98.98 s |
| Final documentation/contract checks | 3/3 on each preset after the evidence updates; Headless 0.23 s, Release 0.25 s |
| Installed equipment consumers | Passed on Headless in 28.96 s and Release in 32.09 s, with each preset's own C++/C99/Python installation and `equipment-package-consumer/provenance.sha256` |
| Final 5 km native encounter | Detonated at 10.2878 m swept miss, maximum speed 699.947 m/s; exact one/two-worker records/events |
| Final 30 km native encounter | Detonated during coast at **8.12775 m** swept miss, maximum speed **778.286 m/s**; exact one/two-worker records/events |

Both presets reproduce those encounter values. The final 30 km endpoint range
is still 30.9521 m; native substep geometry supplies the subsequent eligible
encounter. A detonation is not a crew-loss or mission-success assertion. These
are software cases using estimated equipment inputs, not a real launch envelope.

The full Headless failures are retained in the [carryover register](../architecture/validation/handoff-carryover-register.md#sim-m03-reference-continuation-2026-10-20):

* **HC-57, ground-store impact:** the existing `test_impact.cpp:235` requires
  immediate tank burning above 300 kg; actual total is 265.77651731737785 kg.
  Five strikes, three fires and the other 17 assertions pass. The pre-rebuild
  Release executable and the newly rebuilt Release executable reproduce exactly
  the same failure. Inert jettison never enters Guidance, the autopilot or
  fragment assessment. Historical cause needs a further controlled comparison;
  neither threshold nor fuel coefficients were altered.
* **HC-58, package fixture:** unchanged `TestDistComponents.cmake` creates a
  synthetic patched-source tree without `models/FGPropulsion.cpp`, which the
  unchanged installation rules require. `dist_components` fails in its fixture
  install. Actual installed equipment consumers pass. This remains a packaging
  test-infrastructure correction, not an accepted full-regression pass.

Raw logs, full CTest details, RED controls, source/model fingerprints and
installed-manifest hashes are under `build/architecture-reviews/sim-m03-continuation-*`.
The pre-rebuild Release executable SHA-256 is
`9fcc18ef9d0da4471b0db18dc9b765c80f3a970e94cb76b1d1a3ab9b01627c7e`;
it is an observed binary control, not an independently reconstructed historical
source build. The initial malformed Catch selector is retained separately and
is not counted as an executed control.

No new Viewer run, external solver result or equipment calibration is claimed.
The complete-package rows above remain open, including modeled weapon links,
fused input, sensor/control/lethality admission and variant/Viewer acceptance.

## 7. SDK delivery continuation and regression repair

The user subsequently assigned both retained failures, the remaining independent
mechanisms and a committed production SDK delivery. The [delivery plan](../superpowers/plans/2026-10-20-sim-m03-sdk-delivery.md)
records the unchanged main HEAD and exact exclusive write/build windows. Section
6 remains the historical frozen-state result; it is not a claim that the current
expanded source has completed full verification.

HC-58's synthetic install fixture omitted four required patched files:
FGPropulsion.cpp, FGRocket.h, FGRocket.cpp and FGThruster.cpp. Its fixture now
contains and checks the installed copies. The enabled/disabled component checks
pass 1/1 on Headless (1.60 s) and Release (1.53 s).

HC-57 is a sampled-state defect, established independently of the burning bound.
The opt-in native trace shows first tank contact at 74.13/74.12/69.36 m/s, then
several ground-reaction steps before strike reporting at 52.28/52.35/48.42 m/s.
Mass and fuel are unchanged across those samples. A separate first-contact
energy case is RED at 1.618516 MJ reported versus 3.596499 MJ incoming. The native
adapter now retains the state entering the contiguous STRUCTURE-contact episode
and exposes it only after a terrain crash through a separate optional interface.
Gear-only landings cannot start that episode. Ground disengagement and load/reset clear
it. Models without this interface retain the existing fallback. World uses the
sample's own coordinate frame for incoming velocity and orientation.

The original external-fuel-tank scene and its `burnt > 300.0` requirement remain unchanged.
No ground force, crash depth, spill/mist constant, guidance gain, motor or model
parameter was adjusted. The original case, independent energy case and component
fixture pass **3/3 CTest cases in 2.10 s**; the impact/terrain selection passes
**215 assertions in 9 cases**. Temporary trace instrumentation was removed.
This establishes consistent incoming energy, not real fuel-fire calibration.
Current raw evidence is `build/architecture-reviews/sim-m03-sdk-*`.

The current write window also completes feature-owned native/public fin state
delivery, selected-store/release eligibility, explicit guidance capability and
candidate-scoped illumination, and contact-fuze initiation. These remain in
progress until their current source, installed and visualization gates are
recorded; no future check is counted as a pass here.

### Production mechanisms and current focused evidence

The implementation is in the SDK's Mission Systems and native World path, not a
test-side flight or combat simulation. Versioned actual-fin Range diagnostics
are additive in C++, C99 and Python, with an optional live sidecar and recording
v8 data; the base World stream and existing public layouts remain frozen. The
Viewer consumes actual actuator positions at its body presentation time and
retains authored legacy animation when actual fin data is unavailable. Details
and availability meanings are in [weapon controls](weapon-controls.md).

Selected-store preview and same-activity due-release validation reject every
ineligible selected member before spawning anything. Refusal preserves prior
releases and unrelated activities. Ready IR stores use a real on-rail contact;
released cue identity remains distinct from a designation. Versioned capability
metadata separates inertial steering from launcher updates, without adding a
modeled radio link. Candidate SARH gates use the actual transmitter beam and both
propagation paths. The 24 card changes add capability metadata only; they do not
alter numerical equipment parameters or establish actual fitted capability.

Contact initiation uses the exact authored point tip, per-native-boundary pose
and CG, and currently attached finite geometry. First physical contact consumes
the contact channel even before arming; armed contact competes with proximity
without requiring seeker tracking or creating a second detonation. Unsupported
metadata/history remains unavailable. [Contact semantics](weapon-contact-fuze.md)
record the linear native-segment approximation and conservative arrest cutoffs.
This does not implement kinetic penetration, detached-body damage or equipment
initiator calibration. Multiple weapons still apply damage in their existing
weapon-ID order within a decision step; this delivery does not claim global
fractional-time ordering across simultaneous weapons.

Independent review found a second mixed-epoch defect: swept damage reconstructed
encounter geometry and its impulse lever using the final model CG. The paired
regression changes only a later runtime point mass, holding the earlier captured
encounter fixed. Before correction it reports 208 versus 214 fragment strikes.
After using the matching captured CG, both early assessments agree. Uncaptured
scripted bursts retain an explicitly documented current-CG fallback. The native
impact-speed contract also now compares the authoritative incoming contact
event; it does not assert that contact speed must be lower than launch speed.

The first expanded Headless focused batch records:

| Executed gate | Result |
| --- | --- |
| Real native contact initiation, including lost seeker, batching, unarmed first crossing, inert, near-miss and detached controls | **6/6 passed** |
| Fin wire/recording/mirror contracts | **5 cases, 115 assertions passed** |
| Repeated absent optional-fin queries | **1 case, 63 assertions passed**; previous RED emitted 80 warnings |
| Swept encounter CG invariant | Behavioral RED, then passed after production correction |
| Batched terrain impact contract | Passed; Impact and zero-detonation requirements retained |
| Native engagement executable | **9/10 passed**; 5 km fixed-scene hit gate failed |
| Unchanged 30 km fixed scene | **Passed actual damage evidence**, 275/2023 sampled fragments struck; miss 8.12775 m, maximum speed 778.286 m/s, powered-to-coast transition; exact one/two-worker records/events |
| Unchanged 5 km fixed scene | **Failed actual damage evidence**; detonation at 10.2878 m, 0/2023 sampled fragments struck, maximum speed 699.947 m/s |

These hit results are distinct from physical-model acceptance. No scene, gain,
fuze radius, warhead constant or numerical card value was tuned to obtain them.
The original 5 km scene remains a failing acceptance case. Expanded complete
builds, installed execution, Release presentation and full regression are still
pending at this focused checkpoint. Raw logs use `sim-m03-sdk-*` under
`build/architecture-reviews/`.

### Expanded builds, installed execution and presentation

Both complete UCRT64 builds now pass. Release's focused architecture, generated
boundary, package-component, installed equipment, C controls, weapon employment,
contact, guidance and controller selection passes **11/11 in 44.80 s**. Its
related impact/terrain/fin selection passes **22 cases and 1,021 assertions**;
this includes the unchanged original tank-burning acceptance. Actual-fin render
and legacy-recording/clock checks pass **6 cases and 243 assertions**, including
the independently RED strict-before-first-generation boundary. The corrected
public C++ fin fixture passes **43 assertions**.

The first Headless focused selection passed 9/11; the two infrastructure failures
were the new public value header's missing architecture classification and the
generated boundary page awaiting refresh. The header is classified as a contracts
value without an exception. The full Release SDK regenerates the page with
**1,965 classified entry points and zero errors**. Both checks now also pass at
the start of the current complete Headless run. Neither failure was hidden by
omitting a check or weakening an interface rule.

Installed equipment execution passes on each preset's own isolated C++, C99 and
Python packages: **29.23 s Headless, 31.68 s Release**. Each language performs
own measured designation, delivered C2 consent, one armed release, actual fin
movement and retained native jam. C99 additionally checks bounded/non-consuming
buffer reads and Agent refusal. The existing corrupt-installed-data negative
probe and restoration remain required. Manifests are each preset's
`equipment-package-consumer/provenance.sha256`.

The visible fullscreen live example and its v8 replay each exit **0**. The example
imports the Release consumer's installed Python package/DLL, rather than build
wrappers. Eight fresh **3840 x 2160** captures decode; representative live and
replay frames were visually inspected for the released AIM-120C and distinct fin
poses. The native diagnostic jams fin ID **453677541** at
**-0.04865689280385491 rad** and verifies it remains there. The same native
recording is replayed at its own simulation times with actual fin data. Evidence
is `build/architecture-reviews/sim-m03-sdk-viewer/`, including
`flight-final.fsrec`, `release.json`, `live-result.json`, `replay-result.json`,
native logs and the eight `*-final_*.png` captures.

The fault diagnostic also exposes an actual control-domain limitation: **106
decision samples are unavailable** after the injected jam. Their returned demands
are neutral while the fin measurements remain valid. The example's former
universal claim that every later decision must be available was false; a focused
reproduction separately records that failure, then verifies availability and
neutral refusal as distinct contracts. Neither the diagnostic exit nor these
images establish successful fault recovery, an intercept, a performance gate or
per-variant physical acceptance. Reset/seek/identity boundaries and v7 fallback
have focused native render coverage; manual timeline seeking and all-variant
fullscreen coverage are not newly claimed.

The first expanded **775-case** Headless inventory completed **769/775 passed**
in **1129.11 s**, with six failures and zero skipped cases. All **152** frozen
changed source/test/model fingerprints still match. It used nonzero discovery,
`--no-tests=error` and bounded CTest timeouts.

Two failures are valid physical hit objectives and remain unchanged:

* `engagement_native`: the 5 km scene above, 10.2878 m detonation and no native
  target damage event. Its other nine internal cases pass, including actual
  30 km damage and one/two-worker identity.
* `showcase_native`: its separate fixed 5 km/seed-17 scene detonates at
  **9.578099 m**, with **0/2023** fragment strikes and no damage event. Its real
  native hit/recording requirements remain in force. It is not the same scene
  as the 10.2878 m regression and is not replaced with the passing 30 km scene.

The four test-infrastructure failures are corrected only after that frozen full
run, under the [recorded test-only handoff](../superpowers/plans/2026-10-20-sim-m03-sdk-delivery.md#full-regression-fixture-handoff):

| Failure | Demonstrated cause and retained criterion |
| --- | --- |
| Old recording fixture | Its converter encounters v8 `F` where it expects `S`, before calling the reader. Strip the bounded extension when constructing v1/v5 files; retain all original byte/default/frame/engine/CG checks and v1-v7 reading. |
| Launch-policy fixture | The two-store mixed-selection quantity leaks into a separately named one-AIM-120C test. Restore one selected store; retain all target/FOV reasons and unchanged-state assertions. |
| Demo scheduling fixture | Process-wide handles fall from 124 to 123. Exact equality is a false leak criterion, and the shared failure counter falsely labels scheduling as wrong. Require no positive net growth with zero allowance; retain every exact step/lifetime check and print actual counts. |
| Hangar child-status fixture | A global Popen mock intercepts supported CPython 3.13's platform metadata subprocess. Isolate host metadata and assert only the intended child uses the mock; retain the real record writer and every stale-report/nonzero-child assertion. |

The four corrected fixtures now pass **4/4 in 13.03 s** on Headless. Comparison
with the first full freeze identifies exactly one changed existing test file and
three newly included test files; **no production source or model input changed**.
The first full result is not relabeled as an all-green run or a later one-shot
773/775 result. The complete current Release inventory finishes **847/849 passed**
in **1127.24 s**, with zero skipped cases. Its only failures are the two valid
fixed 5 km hit objectives above. All four corrected fixtures pass in that full
run, as do HC-57, HC-58, installed equipment execution, native-state reset,
episode/interface compatibility and the Viewer screenshot. All **155** frozen
changed source/test/model fingerprints match afterward. No production/model
input changed between the complete Headless and Release runs.

Both full runs use the maintained UCRT64 launcher, discovered nonzero inventories,
`--no-tests=error`, `--timeout 300` and the existing bounded per-case timeout
overrides. Raw final Release output is
`build/architecture-reviews/sim-m03-sdk-release-full.log`; inventory output is
`sim-m03-sdk-release-inventory.json` in the same directory. The final input hashes
are `sim-m03-sdk-final-inputs.json`. Generated-output proof is
`sim-m03-fin-generation-final.log`: 24 missile XML/GLB products, unchanged motor
and non-metadata numerical loads, and identical GLB binary geometry payloads.
Card comparisons retain all prior numerical content; only explicit capability
metadata is added to the 24 current AAM cards.

Independent final read-only assertion review reports no blocking findings.
The real 5 km hit gates, original HC-57 bound, exact schedule/lifetime checks,
old-reader state checks and child-failure controls remain active. This package
delivers SDK production mechanisms and their evidence; it does not claim an
all-green full regression or complete SIM-M03 acceptance.

After the final evidence update, the documentation/architecture/MA-contract/SDK
boundary selection passes **4/4 on each preset** (Headless **2.62 s**, Release
**2.56 s**). Logs are `sim-m03-sdk-final-docs-{headless,release}.log` in the same
evidence directory. These repository checks do not change either failed hit
objective.

Read-only terminal-path review finds no demonstrated moving-CG, axis or fragment
velocity-frame defect explaining the two 5 km misses. Controller derivatives and
native aero moments both use current CG; fragment velocities include weapon and
target motion. The proximity channel selects the first qualifying native path
segment, so different closing speeds and sample phases can change initiation
range. This does not prove the cause of either miss. The CSV mixes explicitly
named before-step geometry and later state/command fields and omits the exact
terminal burst attitude. Resolving the remaining physical result requires the
exact burst/target assessment inputs and unchanged RNG replay, rather than
inferring burst geometry from that CSV or adjusting gains, fuze or fragment
zones. The 30 km one/two-worker CSVs are byte-identical, SHA-256
`f7956afae8bb5235df2207e74b9d61fa9ffe6c91926921e2b5d3b04082e99f15`.
