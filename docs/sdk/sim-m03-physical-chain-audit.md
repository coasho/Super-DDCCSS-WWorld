# SIM-M03 physical-chain audit, 2026-10-20

Baseline: main `16da711bf59665a538919f2abb2dfda2ab11278f`, following the
[finite-fin correction](missile-control-validation.md). This review follows the
production release, guidance, sensing, decoy, fuze, damage and separation paths.
It distinguishes software defects from declared engineering estimates. Passing
these regressions does not calibrate equipment or complete SIM-M03 acceptance.

## Independent mechanism corrections

| Path | Defect and resulting correction |
| --- | --- |
| `World::prepareEngagements`, `Dispenser::advance` | Seekers previously received decoy positions advanced to the next tick with signature age from the current tick. Observe all geometry at the current tick first; advance afterward, retaining owned time-tagged trajectories. |
| `World::finishEngagements`, `Dispenser::sampleAt` | Endpoint minus final velocity was not the previous position under drag/gravity; expiring particles lost their last partial living interval. Reuse the analytic trajectory at actual native sample times, retain the final interval, and exclude expiry before closest-approach minimization. Invalid propagation removes unavailable motion rather than publishing a frozen fresh pose. |
| `Dispenser::advance` environment callback | All particles used the first particle's radial down and ignored modeled wind. Each now receives its own geodetic down and world mean wind converted to ECEF; drag uses velocity relative to that air. Local effect gust/turbulence fields are still unsupported. |
| `World::preStep`, `sweptPathFuze` | An outer-step chord could invent or miss an encounter; a crash pose was treated as moving through the entire batch and could arm after contact. During live-weapon steps, capture each vehicle's own native samples on its worker, consume histories after joining, and clip the weapon interval at native contact. Damage poses, burst velocity and terminal speed use the encounter segment. Inert weapon impacts also retain their last moving native sample. External diagnostic bursts without a live-weapon history retain the existing outer-step pose interpolation. |
| `VehicleDamage::attachedGeometry`, `damage::assess` | Removed panels/components still intercepted fragments, shielded surviving geometry, and loaded the parent with blast/fragment impulses. Filter before intersections, armor lookahead and blast, preserving original immutable source indices. Reject stale detached hits in state application. Unsectioned fixtures remain supported; invalid positive section IDs are excluded. |
| `panelBlastImpulse`, `World::detonate` | Section ejection used full panel area without incidence while aggregate/root impulse used half area times facing. One canonical expression now supplies all three calculations. This corrects consistency of the existing loading model; it does not calibrate that model. |
| `estimateOf`, `completeTrackKinematics` | Measured NED track velocity became zero when consumers converted default body velocity. Complete a consistent synthetic velocity frame for all existing geometry consumers. Its zero-roll heading/climb frame is derived from measured velocity, not measured target attitude. The legacy scalar speed proxy is ground-relative, not measured true airspeed. Invalid conversion rejects/lapses the designation. |
| `checkLaunchIllumination`, in-flight illumination | Matching payload/track numbers from another source inherited unrelated local STT illumination. Local illumination now requires a local-source designation as well as matching owner/payload/track. Remote or cooperative illumination needs an explicit association contract. |
| `World::finishEngagements` motor/lifetime | Motor telemetry used pre-step thrust. Sample current native thrust after stepping. Remove the arbitrary post-five-second, sub-180-m/s deletion rule: a valid slow weapon continues native flight to contact or configured life expiry. Native divergence still terminates explicitly. |

The [implementation allocation](../superpowers/plans/2026-10-20-sim-m03-physical-chain-audit.md)
records exclusive file/symbol ownership and the serial native slot.

## Remaining approximation inventory and required inputs

| Source/symbol | Current simplification and practical consequence | Required input / disposition |
| --- | --- | --- |
| `weapon/Autopilot`, Hangar `storecontrol.py` / `storeaero.py` | Estimated actuator lag/rate, controller bandwidth, incidence bound, fin effectiveness and CP; no hinge-load, high-incidence coupled flow or calibrated failure response. Fin demands execute through finite physical actuators, with no ideal guidance reaction fallback. | Await independently accepted SIM-M02 mass/CG/inertia evolution and force/moment/control derivatives, plus per-variant actuator/fin references. |
| `weapon/Guidance` | Fixed PN gain, acceleration cap, loft and activation/range rules from cards/defaults. An acceleration request is not demonstrated achievable acceleration. | Consume accepted dynamics/control inputs and named variant guidance evidence; measure saturation, response and miss distributions. |
| `WorldEngagements` seeker observation construction | Eligible native target truth becomes exact position/velocity observation; range/aspect/RCS/notch/terrain gates replace received-signal estimation. No measurement covariance, sampling/processing delay, stabilized finite gimbal or tracking filter. | Sensor owner must supply admitted measurement, timing, gimbal and signature/receiver contracts. M02 alone cannot establish sensing fidelity. |
| SARH observation admission | Source identity is corrected, but one local illumination flag still gates all candidate returns. No candidate-specific bistatic received illumination or clutter processing. | Launcher/candidate/receiver geometry, antenna/waveform, propagation and receiver thresholds, with an explicit cross-source association for cooperative support. |
| `weapon/Guidance` association and `WorldEngagements` signatures | Two-degree association cells, fixed discrimination ratios, normalized aircraft strength and deterministic decoy bloom/fade. IR aspect/afterburner factors and radar notch remain estimates. | Measured/admitted spectral, range/range-rate/image discrimination and countermeasure signature inputs, with uncertainty and provenance. |
| `countermeasure/Dispenser` | Point particles with constant local environment per World step, fixed linear drag/gravity, lifetime/inventory/cooldown. Mean wind now couples; particle tumble/size evolution, gust/turbulence and cartridge variation do not. | Countermeasure cartridge/aerodynamic/signature data and a spatial/time-varying air-field contract. Test cadence convergence separately from calibration. |
| `World::prepareEngagements` support / feedback | A valid permitted designation directly refreshes guidance. Weapon uplink delivery, delay/loss and weapon return assessment lack dedicated modeled paths. | Accepted SIM-M01 fused identity/provenance plus configured weapon-link delivery and feedback contracts; preserve existing Agent knowledge permissions. |
| `World::launchZone` | Card/altitude/speed/closing arithmetic and fixed no-escape factors; no integrated finite-control flyout. Corrected measured velocity fixes its inputs only. | SIM-M04 envelope work consuming independently accepted SIM-M02 flyout and uncertainty. Advisory output remains an estimate. |
| `weapon/Fuze`, `MotionHistory` | Spherical proximity, fixed arming/life/radius; linear relative motion within each native substep and burst at segment closest approach. No sensor scan, directional beam, delay or fragment aimpoint selection. Event/terminal timestamps retain World publication ticks; encounter poses/velocities are sampled within that tick. | Actual fuze geometry/response and admitted initiation data. Native substep timing is corrected; continuous substep contact localization remains a numerical-resolution limit. |
| `damage::warheadOf` / fragment spray | Generic charge/case estimates, Gurney velocity and sampled fragment/rod zones. Rods do not model a connected expanding continuous-rod ring. | Admitted variant explosive/case/fragment distributions and rod topology/dispersion; retain explicit assumptions until supplied. |
| `damage::assess` interception/penetration | Derived aluminium skin thickness, one-metre skin lookahead, first solid component absorption and one target's geometry per assessment. No material stack, internal residual exit or cross-entity shielding. | Material/armor/component geometry, penetration and vulnerability references; separate residual-flight and cross-body ownership contracts. |
| `damage::blastAt` / panel loading | Free-air fit, unshadowed panel loading and half-area/projected-facing impulse. Canonical consistency is repaired; pressure-time response and occlusion remain approximate. | Validated blast/reflection/shielding and structural response data. Do not replace the declared fit with invented coefficients. |
| `VehicleDamage::apply` component failures | Explicit per-hit engine/control/crew probabilities and energy/pressure-based tank ignition estimates. No whole-aircraft random kill roll was found. `destroyed` in damage events remains native cumulative crew-loss state, not mission success. | Component vulnerability/fire/material evidence and explicit outcome semantics; independent of M02 acceptance. |
| `VehicleDamage::aeroLoss`, section separation in `WorldDamage` | Fixed normal/side shares subtract a lost section's estimated aerodynamic load; spring/capped ejection and approximate debris models stand in for deforming/fracturing geometry. | Accepted section loads, CP, structural mass/inertia/strength and fracture/ejection inputs. Await the corresponding M02/airframe contract. Do not introduce a new ideal control force path. |
| `World::detonate` detached victims | Attached-parent membership is corrected, but debris is excluded from subsequent weapon damage and cross-body shielding. It retains native flight/contact behavior. | Independent detached geometry/component identity, state, fuel/stores, impulse and event ownership. HC-55's detached-body portion remains pending. |
| Weapon terminal lifecycle | Battery/life expiry is an explicit terminal removal; telemetry retains bounded terminal records. There is no separate inert post-expiry body lifecycle. | A distinct lifetime/entity contract is required before retaining a formerly guided body; preserve existing `WeaponEnd` meanings and public compatibility. |

## Verification

The initial production regression failed on lost designation velocity; the new
private seams also had captured missing-interface RED builds. Raw evidence is
under `build/architecture-reviews/sim-m03-*`; no previous totals are reused.

* UCRT64 Headless complete target build: passed. Focused CTest: **43/43**,
  **100.60 seconds**. This includes damage/fragment/blast/separation, analytic
  decoy trajectories, guidance/kinematics, fuze paths, finite controls, native
  engagement, public C++/C, four Python modules, installed equipment consumers,
  simulation boundary, contract/architecture/SDK/scope guards and generator
  damage-resource checks. `sim-m03-headless-final.log` retains exact selection.
* The installed equipment gate ran in **31.90 seconds** with its independent
  install tree and `equipment-package-consumer/provenance.sha256`; it consumes
  installed C++, C99 and Python interfaces/data. Generic package ownership does
  not transfer to this feature.
* UCRT64 Release complete target build: passed. The same focused CTest selection
  passed **43/43**, **98.88 seconds**; its installed equipment gate took
  **32.69 seconds**. `sim-m03-release-final.log` retains the exact selection and
  its own installation provenance remains in
  `build/ucrt64-release/equipment-package-consumer/provenance.sha256`.
* The native 5 km case detonates at **9.95659 m** minimum eligible miss distance,
  with identical records/events for one and two workers. The 30 km motor/coast
  case ends in **Impact**, with **15.0118 m** minimum eligible miss distance and
  **780.622 m/s** maximum speed. It validates powered/coast execution, **not a
  long-range hit**. The earlier finite-control handoff's 30 km hit is historical;
  it must not be reused after correcting measured velocity and substep timing.
* The guided `frameSkip=240` fixture acquires its crossing target, contacts terrain
  at native step 2 before arming, and retains Impact without a later detonation.
  Its terminal speed uses the final moving native sample. The slow/starved
  post-five-second fixture remains airborne and live below 180 m/s. Current
  burning matches native thrust on every live engagement sample.
* Independent read-only review found and resolved encounter-body/current-body
  impulse conversion and historical NED/terminal-speed consistency. Re-review
  found no further material defect in clipping, attached membership, decoy expiry
  or canonical panel impulse. Terminal speed also matches event ECEF speed.

The 20 changed/new native source and regression files are fingerprinted in
`build/architecture-reviews/sim-m03-source-manifest.sha256`. Full regression,
Viewer evidence, equipment calibration and complete variant/fleet acceptance
remain separate gates. No external numerical delivery is accepted by this change.
