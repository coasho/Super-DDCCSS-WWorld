# SIM-M03 weapon execution configuration

The store card selects the SDK's guidance and finite-control implementation. This is an MS payload configuration: it does not authorize an external force controller, change Agent permissions, or certify a real missile's classified implementation. The existing `StoreCard.parameters` representation carries the versioned numeric settings; no public SDK structure, plugin vtable or observation/action layout changes.

## Version 1 contract

A genuinely absent execution block retains the existing unversioned execution. A present version must be exactly `1` and every parameter below must be present, finite and in the stated unit. An orphan setting without `weapon_execution_version` is an incomplete explicit block and fails. Unsupported versions/algorithm IDs and malformed values fail explicitly and atomically; they cannot select legacy defaults. Each card parameter retains the existing `value`, `unit`, `uncertainty`, `basis`, `source` and `note` provenance fields.

| Parameter | Unit | Version 1 meaning | Existing engineering value |
|---|---|---|---:|
| `weapon_execution_version` | 1 | Configuration schema | 1 |
| `guidance_algorithm` | 1 | `1`: velocity-normal PN with closing-speed gain | 1 |
| `guidance_navigation_gain` | 1 | PN navigation ratio | 3.5 |
| `guidance_cue_deg` | deg | Acquisition association cone | 10 |
| `guidance_activation_ratio` | 1 | Activation range / acquisition range | 1.15 |
| `guidance_loft_deg` | deg | Existing midcourse loft angle | 20 |
| `guidance_loft_acceleration_g` | 1 | Existing loft multiplier | 2 |
| `control_algorithm` | 1 | `1`: reduced-order load-feedback cascade | 1 |
| `control_allocator` | 1 | `1`: authored native fin mixer | 1 |
| `control_max_incidence_deg` | deg | Combined reduced-order incidence bound | 15 |
| `control_rate_time_min_s` | s | Inner rate-response minimum | 0.12 |
| `control_rate_lag_multiplier` | 1 | Actuator time-constant multiplier | 3 |
| `control_incidence_time_min_s` | s | Outer incidence-response minimum | 0.4 |
| `control_incidence_rate_multiplier` | 1 | Rate-response multiplier | 3.333333333333333 |
| `control_actuator_lag_s` | s | Native channel actuator time constant | 0.04 |
| `control_actuator_rate_rad_s` | rad/s | Native channel and physical-fin rate | 4 |

These values preserve the existing execution; they are not fitted against either 5 km scene. Algorithm identifiers name supported implementations, rather than equipment claims. Version 1 currently implements only ID `1` for each algorithm. The release freezes the validated settings in its private guidance specification; the native substep controller consumes that same specification.

The supported profile requires navigation gain greater than two, cue within `[0,180]` degrees, positive activation ratio, loft within the existing `[0,0.7]`-radian model envelope, nonnegative loft multiplier, positive time constants/multipliers, and combined incidence strictly between zero and ninety degrees. These are the declared model's admissibility conditions, rather than fitted equipment limits. A failed combined parse retains the entire previous specification.

## Independent fuze block

`configureGuidance` also admits a separate optional numeric block. `fuze_model_version=1` and `fuze_proximity_algorithm=1` use unit `1`; `fuze_arming_s` and `fuze_delay_s` use seconds; `fuze_radius_m` and `fuze_station_x_m/y_m/z_m` use metres. Algorithm `1` identifies the generic ideal captured turning-point CPA model implemented by the fuze owner; it is distinct from a clamped approaching interval endpoint. A declared zero simulator delay is an idealization, not an AIM-120 hardware delay claim.

All fields are required when any new member of this block is present. The pre-existing standalone `fuze_radius_m` parameter retains its caller-owned legacy meaning and does not opt a card into this block. The structural station is x aft from the authored nose, y right, z up. Its x must lie within the card's positive `length_m`; lateral radius must not exceed half its positive `diameter_m`. Radius, arming and delay are nonnegative, arming precedes the configured life, and arming plus delay must leave a live interval. The body dimensions require metre units. Unknown model/law, missing fields, wrong units, nonfinite values or out-of-body/lifetime values fail atomically. A card may declare this block while retaining unversioned guidance, or declare execution without a fuze block. The helper configures these fields; native encounter geometry, true-CPA evaluation, scheduled delay, contact and detonation remain World/fuze responsibilities.

Guidance computes `omega = (r cross relative_velocity) / |r|^2`, then `a = N * closing_speed * (omega cross velocity_unit)`, before the existing loft, gravity compensation and acceleration limit. This explicitly names the implemented velocity-normal, closing-speed convention. It is not relabelled as textbook true PN, whose command is normal to the LOS. The supported controller computes trim from signed force/moment derivatives and measured native load residuals, then requests body-rate response using `rate_time = max(rate_time_min, rate_lag_multiplier * actuator_time_constant)` and `incidence_time = max(incidence_time_min, incidence_rate_multiplier * rate_time)`. Euler inertial coupling remains part of the moment calculation. The result contains normalized actuator commands only.

The authored mixer remains in the native JSBSim FCS, with each physical fin's actual stops, rate and failure state. The pseudoinverse in the generator reconstructs equivalent axis positions from actual fins for reporting; it is not a constrained allocation solver or a claim of fault recovery. Unknown allocator IDs do not execute that mixer silently.

## Geometry and native normalization

The generated `fsim/missile-control/normalization-version = 1` declares per-axis `aileron/elevator/rudder-min-rad` and `-max-rad`. These are derived from the existing authored channel ranges, alongside the retained legacy maximum-absolute `-limit-rad` properties. A negative requested deflection is divided by the magnitude of the negative stop; a positive request uses the positive stop. This matches JSBSim's default zero-centred, piecewise aerosurface scale. For example, stops `[-0.1,+0.3]` rad require command `-1` for `-0.1` rad, rather than `-1/3`. Absent metadata selects the explicit legacy symmetric mapping. Present invalid or unsupported metadata refuses control with neutral commands.

Reference area, reference diameter, Mach-dependent signed derivatives, fin identity/mix, geometric stops, current mass/inertia/CG, actual fin positions and dynamic pressure remain plant data. They are not arbitrary controller gains. The runtime uses native `qbar` and the same generated reference metrics: force derivatives scale by `q*S`, moment derivatives by `q*S*d`, and rate derivatives additionally by `d/(2*V)`. Nonfinite or absent aerodynamic authority cannot be replaced by an inflated pressure or an ideal reaction force. The configured incidence envelope and bandwidth values remain stated engineering assumptions.

Version 1 explicitly configures `control_actuator_lag_s` and `control_actuator_rate_rad_s` in the card, requiring positive finite values and a representable reciprocal lag coefficient. The retained `0.04 s` and `4 rad/s` are unchanged assumptions. They are not inferred from the AIM-120's mass or fin span, and this configuration does not claim a published AIM-120 servo specification. Geometric stops retain their authored design/model provenance.

The generator entrypoint is `storemodels.files(card)` → `storebodies.files(card)` → `storecontrol.apply(..., card=card)`. For a versioned card, all native channel `<lag>` elements reference `fsim/missile-control/lag-coefficient-s-1`, channel `<rate_limit>` elements reference `fsim/missile-control/channel-rate-rad_s`, and physical-fin rate limits reference their individual `fsim/missile-control/fin-i-rate-rad_s` properties. The published values are the native FCS inputs, rather than independent diagnostic copies. `fsim/missile-control/actuator-version=1` declares this binding. The generated XML records a SHA-256 of the consumed card identity, version, algorithm choices and actuator inputs. Changing a selected lag/rate changes that fingerprint. Store build executes this generator with the current card on every build; the visual-only GLB mesh cache does not need an unrelated actuator dependency.

The runtime keeps the actual native lag input and checks the declared native actuator version, channel rate and every physical fin's rate against the selected card. Missing, malformed, unknown or mismatched metadata makes versioned control unavailable/neutral; it cannot substitute a card value for a different physical plant. Only sixteen double-precision epsilons of relative representation error are admitted for reciprocal conversion. This is an arithmetic comparison tolerance, not a physical variation or hit-acceptance allowance. Legacy unversioned cards/calls retain the existing numeric `25 s^-1`/`4 rad/s` native actuator literals and legacy controller constants.

## Primary-source scope

[Palumbo, Blauwkamp and Lloyd, *Basic Principles of Homing Guidance*, APL Technical Digest 29(1), 2010](https://www.jhuapl.edu/Content/techdigest/pdf/V29-N01/29-01-Palumbo_Principles_Rev2018.pdf), equations 19–22, distinguishes LOS-normal true PN from velocity-normal pure PN. Pages 37–38 discuss the separate problem of preserving a guidance command when an interceptor cannot control axial acceleration. This supports naming the mathematical convention precisely; it supplies no AIM-120 navigation gain or seeker calibration.

[Durham and Nelson, *Control Law—Control Allocation Interaction*, NASA NAG-1-2246 final report](https://ntrs.nasa.gov/api/citations/20010029666/downloads/20010029666.pdf), sections 2.1.3–2.1.6, separates dynamic inversion, native aerodynamic effectiveness and actuator allocation/limits. [NASA NESC's control-allocation theory material, pages 11–17](https://ntrs.nasa.gov/api/citations/20220018521/downloads/202212107_Orr_NESC_Academy_CA_theory.pdf) explains why an unconstrained generalized inverse alone does not enforce attainable-moment constraints. Neither source calibrates this missile's reduced-order response constants.

The official [JSBSim `FGGain` reference](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGGain.html) defines zero-centred asymmetric aerosurface normalization. The [JSBSim `FGActuator` reference](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGActuator.html) defines native lag, rate limiting, hard stops and failure behavior. These are implementation semantics, not published equipment limits.

## Verification boundary

The added regressions require a supported card value to change an independent unsaturated synthetic request, an unknown selection to refuse execution, malformed parsing to preserve the previous specification, and asymmetric normalization to reconstruct the requested physical radians. The root captured behavioral REDs for card parsing, PN dispatch, asymmetric trim and missing generator normalization metadata in `build/architecture-reviews/sim-m03-control-{config,guidance,autopilot,generator}-red.log`. Additional checks cover independent fuze blocks, malformed units, partial profiles, body/lifetime bounds, nondefault native lag/rate generation and configured-versus-native actuator mismatches. Generator checks compare metadata with the actual native scale ranges and native actuator property references. Existing unversioned regressions remain in place. The later actuator and fuze regressions have not yet been executed at this handoff.

Both unchanged 5 km scenes retain their original hit requirements and separately require actual damage and native flight-feedback evidence. No algorithm parameter, physical coefficient, random seed, scene or acceptance threshold may be adjusted to manufacture a hit or kill.

The later root-owned [current regression record](sim-m03-five-km-repair.md#current-scoped-regression-evidence) supersedes the pending software-check status above. Guidance, autopilot, launch eligibility and native control gates pass in both presets. The complete fixed-scene/physical-model package remains unaccepted; card provenance and generic algorithm checks do not calibrate real equipment. Hangar's current combined gate retains two independently confirmed pre-existing damage-document failures, separately from its passing control-generation checks.
