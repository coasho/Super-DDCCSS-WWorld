# Finite missile control mechanism

Date: 2026-10-20. Baseline: main
`85e7d251c5de702de5d2ec66115a0886bd0702cd`.
The user prioritised actual surface control before further SIM-M03 work.
The [implementation plan](../superpowers/plans/2026-10-20-finite-missile-controls.md)
records the exclusive native slot and file ownership.

## Model and execution

Guidance holds an ECEF specific-acceleration demand. At each native step, the
autopilot converts it to normalized aileron/elevator/rudder commands, retaining
the release's ignition throttle. JSBSim realises these commands through finite
actuators and fin aerodynamics, then integrates the complete rigid body.
Guidance neither cancels native aerodynamic forces/moments nor writes an
external reaction. Other native damage/environment reaction channels retain
their existing meanings.

The generator consumes the authored moving-fin metadata. Control effectiveness
comes from the existing geometric build-up and its signed control mixing.
Each fin's force and pressure-centre moment use the same geometry. Additional
drag opposes the airflow. Fixed/uncontrolled stores retain their existing path.
Generated `aircraft/stores` files must be regenerated, never edited by hand.
Motor, nozzle, grain and ignition inputs are outside this change.

The channel command response has a lag coefficient of 25 per second (40 ms time
constant) and a rate limit of 4 rad/s. Each physical fin after mixing has its
own 4 rad/s rate limit and stop, with no second lag. Stops come from authored control metadata (currently
20 degrees for the maintained AAM designs). These response constants and the
geometric control derivatives are engineering estimates, not equipment data.
The existing neutral initial-condition pass primes actuator state before the
first nonzero guidance command; the global JSBSim reset patch is preserved.

The force-driving positions are `fcs/fin-i-delta-rad`; each fin also exposes
its demand, stops, rate and native failure flag. Private
`fcs/{elevator,rudder,aileron}-response-rad` values feed the mixer. Standard
`fcs/*-pos-rad` and existing VehicleState surface fields instead report the
actual fins' equivalent axis positions using the fixed authored mixer's
pseudoinverse. Jamming every fin neutral therefore reports neutral positions
despite a live guidance request. An individual jam or mixed saturation can
leave a residual that three axis fields cannot describe. Per-fin properties
retain that detail through existing Range property access; the unchanged
Viewer/state layout does not establish individual-fin visual acceptance.

## Private controller

Body axes are x forward, y right, z down. All scheduled force/moment derivatives
are signed in those axes. The generator's reference is the launch AERORP;
the controller transfers moments to the current motor-dependent CG before
forming its dimensional plant. It reads current mass/inertia and dynamic
pressure. No public layout or interface changes.

The controller solves the signed two-variable force/moment trim equations,
then uses incidence and angular-rate feedback to request deflection. Pitch
path-rate feedforward is `-(specific_z + gravity_z)/airspeed`; yaw is
`(specific_y + gravity_y)/airspeed`. Gravity-compensating level lift therefore
does not request an extra pitch rotation. Rotating-coordinate `UVWdot` is not
used as an accelerometer measurement.

The estimated nominal rate/incidence response times are 0.12/0.4 seconds,
scaled slower when actuator lag is larger. Requested combined incidence is
bounded at 15 degrees and commands at each channel's declared stop. The native
fin mixer applies physical stops after mixing. Saturation can prevent demand
achievement; there is no integrator windup or ideal-force fallback. Missing
metadata, singular/zero authority, invalid data or reversed airflow produces
neutral surface commands. The controller is stateless across releases/resets.
An absent roll effector is independent: the valid pitch/yaw channels of
AIM-9M, Magic 2 and R-60M remain active with neutral roll command.

This is a reduced-order engineering autopilot, not a calibrated hardware
autopilot. Tail/canard direct-force signs and moving-wing moment signs are
retained; an initial acceleration opposite a tail-controlled turn is possible.
The architecture follows separate guidance, autopilot, actuator and airframe
blocks described in [JHU/APL's missile autopilot overview](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf).

## Verification record

Tests were added first. The unchanged native release path failed all three
initial finite-control cases because no actuator metadata existed. The Python
generator regression independently failed the three missing channel actuators
and missing control-force function. Logs are retained under
`build/architecture-reviews/finite-missile-controls/`.

The independently integrated finite-lag synthetic plant supplements the native
checks. The all-physical-fins-jammed position case preceded its correction. That reporting
case reproduced one intended native failure (10/11 cases passed) before the
generator correction. Native parser checks exposed numeric-only property path
segments; names now use `fin-i`, with an all-model syntax guard. Textual parser
exceptions retain their diagnostics while preserving native load refusal.

| Current check | Result |
| --- | --- |
| Generator/control and missile Python modules | 55/55 methods passed in 2.596 s, including 12 new control methods, unchanged engine/nozzle bytes for all 24 AAMs, unchanged uncontrolled bomb output and actual-position projection |
| Private autopilot | 7/7 cases passed, including signed tail/canard force variation, absent roll, level trim, invalid/singular/zero authority, saturation recovery and six independently integrated synthetic finite-lag plants |
| Production native controls | 11/11 cases passed; finite post-mix stops/rates, zero ideal reactions, channel/individual-fin failures, jammed actual position feedback, pressure scaling, exact worker determinism, all 24 native models, three control families and reset/fresh replay |
| Final Headless focused selection after complete rebuild | 30/30 CTest cases passed in 98.08 s, including existing native encounters, allocation, ABI/interface boundary, public C++/C99, four Python modules, installed C++/C99/Python consumers, generator and reference/architecture guards |
| Render-enabled Release build and same focused selection | Complete build/staging passed; 30/30 CTest cases passed in 94.38 s, including installed consumers in 31.13 s and native encounters in 59.43 s |
| Preserved ignition, motor-CG/reset and existing aircraft behavior | 8/8 additional CTest cases passed on each preset; Headless 16.61 s, Release 16.70 s. Includes armed/inert/reversed ignition reset at 60/120 Hz, rocket reset/pulse/startup, original CG bounds, exact aircraft reuse/reset, recorded C172 control and the full native-state reset fixture |
| Final documentation/architecture contract checks | 3/3 on each preset after the evidence/document changes; Headless 3.00 s, Release 2.95 s |
| Independent read-only review | Approved after correcting absent-roll handling, parser paths and actual-fin position reporting; no remaining actionable findings |

The final Headless native 5 km encounter detonated at a swept miss distance of
2.18819 m for both worker counts, with maximum speed 700.108 m/s. The 30 km
encounter burned, coasted and detonated at 7.80757 m, maximum speed 779.216 m/s.
These use the existing fixture's native event meanings and acceptance bounds;
they are software diagnostics with estimated inputs.

An earlier 29/30 focused run included an unrelinked interface-boundary binary
and failed with a segmentation fault. The complete native rebuild relinked the
fixture, after which the same unchanged model passed that gate (4.14 s).
The final 30/30 run above followed complete rebuild/staging. Historical failure
logs remain `headless-initial-{gates,details}.log`; the rebuild isolation is
`rebuilt-boundary.log`. Final logs are `headless-all-build.log`,
`headless-gates.log`, `headless-details.log` and `generator-test.log`.
The installed-consumer manifest is
`build/ucrt64-headless/equipment-package-consumer/provenance.sha256`.

The additional preservation selection first exposed an outdated CG fixture:
it created a standalone aircraft endpoint, left its default throttle neutral,
then assumed the motor had burned. The fixture now explicitly requests full
throttle through the existing actuator command and verifies fuel consumption
before retaining the original `0.31 < CGx < 0.36` bounds. Production neutral
defaults and motor data are unchanged. Its initial failure remains in
`headless-initial-preservation.log`; final preservation logs/details are named
`{headless,release}-preservation{,-details}.log`. Independent review approved
the strengthened positive fixture.

Release logs are `release-all-build.log`, `release-gates.log` and
`release-details.log`, with its own installed-consumer manifest under
`build/ucrt64-release/equipment-package-consumer/provenance.sha256`.
This is a bounded mechanism correction; full-suite/milestone, Viewer and physical
acceptance are not claimed. The source research references and remaining
dependent calibration continue while the numerical owner works independently.
Neither a software pass nor an intercepted target establishes real performance.
Accepted SIM-M02 dynamics, measured per-variant control/actuator data and the
remaining [SIM-M03 gates](weapon-fidelity-validation.md) remain separate.

Reproduction uses the existing `tools/launch/fsim.ps1` launcher, each preset's
own complete `build -j 8`, then `test --no-tests=error --timeout 180 -j 4 -R`
with this frozen focused expression:

```text
^(missile_autopilot|missile_control_native|engagement_native|weapon_guidance|fuze|launch_eligibility|ma_equipment|ma_equipment_c|equipment_package_consumer|simulation_interface_boundary|ma_engagement_contract|ma_contract|sdk_boundary|sdk_guard_checker|architecture_boundary|architecture_configured|testing_scope_checker|missile_(flight|aero)_reference_checker|adapter_compatibility|control_alloc|python_sdk test_(engagement|weapons|stores|ma_equipment)|hangar test_(storecontrol|missiles|stores).*)$
```

The complete source generator command is `python -m hangar.storemodels`, from
`tools/models/hangar`, followed by `python -m unittest tests.test_storecontrol
tests.test_missiles` against those generated bytes. No solver or dependency is
installed or upgraded by this procedure. Temporary compilation paths use
`D:/ENV/tmp` through the existing launcher.
