# Weapon fidelity validation (SIM-M03)

## Current acceptance boundary, 2026-10-20

SIM-M03 remains **open** against the physical-model acceptance criteria in
[MS-7d](../architecture/modules/simulation/adr-30-mission-systems.md) and the
[MS allocation](../architecture/modules/simulation/ms-work-allocation.md).
The user assigned the complete package, then confirmed that Claude's SIM-M02
numerical investigation continues in the standalone
`D:/Projects/Cpp/AeroNumericsLab/` directory and that dependent work must await
its results. Independently reproducible software corrections and the subsequently
prioritised [finite control mechanism](missile-control-validation.md) proceed
within the SDK. They import no numerical research result or equipment calibration.

The [technical research and input-readiness note](weapon-model-research.md)
evaluates reusable control/geometry/event mechanisms, candidate source versions
and independent future cases while the numerical handoff remains pending.
Its proposals are separate from implemented or physically accepted results.

The subsequent [reference continuation and complete-package matrix](sim-m03-reference-continuation.md)
records mature implementation choices and independently regressed Euler/fragment
corrections. Its earlier frozen source detonated in the unchanged 30 km scene
at 8.12775 m on both presets; detonation alone does not pass hit acceptance.
That handoff recorded 744/746 Headless checks, with HC-57/HC-58 retained, and
56/56 focused Release checks. Those historical results do not verify the
expanded SDK delivery source or its actual target-damage requirements. The
[SDK delivery continuation](sim-m03-reference-continuation.md#7-sdk-delivery-continuation-and-regression-repair)
records current gates separately. Sensor, modeled weapon-link, fused-input,
calibration and Viewer acceptance remain open.

Baseline before edits: main `0d4f25a5df3fdd23696e696644cbd0ee32ab23af`, clean
working tree. There was no pending source change to commit at task entry.
The [implementation plan](../superpowers/plans/2026-10-20-sim-m03-independent-correctness.md)
records exact feature and shared-file ownership. The project checkout had one
native configure/build/test owner. The research directory was read only; none
of its candidates, manifests, solvers or results was changed or executed here.

## Delivered native and public behavior

| Area | Reproduced defect | Correction and evidence contract |
| --- | --- | --- |
| IR observation reach | On-rail acquisition could admit an afterburning target beyond the released seeker's unconditional base-range cap | On-rail and released paths share the existing aspect/reach rule; Guidance rechecks the observation's admitted geometric reach. See [seeker evidence](weapon-seeker-validation.md) |
| Seeker field of regard | The released seeker used velocity as its forward axis even when body attitude differed | World supplies the native body-forward ECEF direction; the private core retains the velocity approximation only for internal callers that omit body attitude |
| Invalid guidance input | Negative/nonfinite duration, invalid or reversed launch age, and invalid physical parameters could update or activate a seeker | Refuse before state mutation; subsequent valid updates match an untouched reference history |
| Unavailable timing | Missing/nonclosing prediction and passive seeker transitions were reported as zero seconds | Unknown time estimates are NaN; zero time to active is reserved for an active radar's completed activation |
| Temporal fuze | Arming at the end of a step authorized a swept encounter that occurred before arming | Clip to `[armingS, lifeS)` and return the closest approach in original-step coordinates. Preserve the existing closest-approach burst convention and step-boundary event timestamp. See [fuze evidence](weapon-fuze-validation.md) |
| Fragment momentum | Target impulse included momentum transferred into air before or between target impacts | Sum transfer at skin penetration and component stopping only; stopped-fragment energy independently checks `p = sqrt(2 m E)` |
| Direct actor status | C++/C99/Python direct strike-status getters exposed a raw Range target ID although equipment reports redacted it | Apply the existing own-designation visibility rule at both native public boundaries. Preserve legal own/lapsed designations, correlations, Range access and C sized-copy behavior. See [feedback evidence](weapon-feedback-validation.md) |

Public signatures, layouts, enum values, plugin interfaces and default Agent
permissions are unchanged. Private observation/boresight inputs are not public
ABI. That initial correctness commit changed no generated model, equipment
card, radar algorithm, motor or physical parameter. The later finite-control
feature regenerates the 24 AAM models from authored controls and explicitly
estimated response constants, as recorded separately. The SDK remains the simulation authority and JSBSim the
only released-weapon flight integrator. Detonation, damage, crew loss and mission
outcome remain distinct.

## Verification

Tests preceded implementation. Against unchanged production, four guidance
selectors each failed their intended assertion, the fragment momentum invariant
failed, and the direct status regression exposed the raw target ID in all three
languages. The proposed temporal-fuze fixture first failed to build on its
missing helper. The new helper's boundary cases are synthetic numerical checks,
not measured equipment reference cases.

The UCRT64 Headless build of the changed native targets, shared SDK and staged
Python package passed. Commands use the existing PowerShell launcher, which sets
TMP, TEMP and TORCH_EXTENSIONS_DIR to `D:/ENV/tmp`.

| Final check | Result |
| --- | --- |
| Headless core/native engagement, equipment C++/C99 and architecture/contract selection | 9/9 CTest cases passed in 67.34 s; native encounter fixture 59.15 s |
| Headless public interface and installed integration | 8/8 CTest cases passed in 44.16 s, including installed C++/C99/Python consumers in 31.54 s, the new direct getter case, four Python modules, adapter compatibility and SDK boundary classification |
| Complete native damage tag | 20/20 Catch2 cases passed, 4,814 assertions, including the energy-to-momentum regression |
| Final documentation contract | `ma_contract` passed in 0.17 s |
| Test-selector self-check | Failed: 100/101 internal tests passed; the pre-existing routing defect for `OpenVspIntersectionFixture.cmake` is recorded below as HC-53 |
| Private guidance/temporal-fuze case coverage | The passing executables contain 17 and 11 internal cases respectively |
| Independent read-only code review | No actionable code defect; corrected the stale zero-timing explanation in the public engagement guide |

Raw evidence is retained under `build/architecture-reviews/sim-m03/`:
`headless-core.log`, `headless-public.log`, `headless-public-details.log` and
`headless-damage.log`, plus `headless-checkers.log` retaining the selector failure.
The installed-consumer record names the actually loaded
modules, DLLs and data roots, including missing-data negative controls and
restoration; hashes are in
`build/ucrt64-headless/equipment-package-consumer/provenance.sha256`.
The four Python modules execute 7 engagement, 6 equipment, 1 store and 2 weapon
tests. CTest groups modules/executables; these internal counts are not extra
CTest cases. These are focused Headless checks, with no new render-enabled
Release, full regression, Viewer, performance or physical-calibration claim.

The combined CTest selections pass 18 of 19 cases. The remaining selector check
fails `RulesTest.test_every_tracked_cmake_input_is_routed`: the existing
`tools/validation/missile/native/OpenVspIntersectionFixture.cmake` matches only
the Python-reference safeguard scope, so the checker cannot classify it as a
CMake/script input. A read-only probe loaded the rules from baseline HEAD and
reproduced exactly that headless-only routing with no escalation. `git diff
--exit-code HEAD` confirmed that the rules, selector implementation/test and
fixture script are unchanged. This baseline issue is retained for its test
infrastructure owner; these results do not claim every repository check passes.
See [the carryover register](../architecture/validation/handoff-carryover-register.md#sim-m03-audit-follow-up-2026-10-20).

Subsequent HC-53 correction, 2026-10-20: the exact external fixture now has its
own required reference-check route. The selector and aero safeguards pass 2/2
CTest cases on each configured Headless/Release inventory (105 and 150 internal
methods respectively). Missing mandatory guards refuse before execution; a
mixed shared-SDK CMake change retains both-preset escalation. HC-53 is closed
within this routing scope. The original 18/19 run above is retained as dated
evidence. See [the correction record](../architecture/validation/hc-53-test-routing-2026-10-20.md)
for current commands, logs and independent review.

Reproduction uses `powershell -NoLogo -NoProfile -ExecutionPolicy Bypass -File
tools/launch/fsim.ps1 --preset ucrt64-headless` followed by:

```text
build --target fsim_weapon_guidance_test fsim_fuze_test fsim_tests fsim_sdk_tests fsim_ma_equipment_c_test fsim_engagement_native_test deploy
build --target python_package
test --no-tests=error --timeout 180 -j 1 -R "^(weapon_guidance|fuze|engagement_native|ma_equipment|ma_equipment_c|architecture_boundary|architecture_configured|ma_engagement_contract|sdk_guard_checker)$"
test --no-tests=error --timeout 180 -j 1 -R "^(direct Agent strike status redacts range targets without changing native correlations|python_sdk test_(engagement|weapons|stores|ma_equipment)|sdk_boundary|equipment_package_consumer|adapter_compatibility)$"
```

The damage selection is `build/ucrt64-headless/tests/fsim_tests.exe "[damage]"
--reporter compact` with the same temporary-directory and UCRT64/runtime PATH
environment. It completed successfully and its full raw output is retained.

## Required follow-up inputs and acceptance

| Full SIM-M03 gate | Required input and closure |
| --- | --- |
| Active radar seeker | Admitted per-variant frequency, power, aperture, processing/noise, clutter and measurement/scheduling inputs; native own-sensor acquisition based on received signal and independent detection/track cases |
| Semi-active seeker | Per-candidate received illumination and bistatic geometry; supported continuous/interrupted modes, loss and reacquisition evidence. The existing single illumination flag is an assumption |
| IR seeker | Admitted band-dependent signatures, background/noise, atmosphere, gimbal/track-rate and acquisition/measurement timing. The shared aspect helper is only a consistency correction |
| Fin/autopilot response | The ideal force/moment path is replaced by [finite native fin controls](missile-control-validation.md). Real variant acceptance still needs accepted aerodynamic control derivatives, mass/CG/inertia evolution and actuator/control-response references; a static aerodynamic comparison alone cannot supply dynamic/control data |
| Fuze and lethality | Actual safe-arm/delay/response inputs, fragment mass/angular distributions and vulnerable-component response; frozen uncertainty bounds, independent encounter conditions and residual tolerances |
| Weapon flyout feedback | A modeled fitted return path, assessment accept/reject, delivered sample/receive times, bounded retention and explicit LOST/SOFT_LOST/END_OF_FLIGHT lifecycle; versioned C++/C99/Python access. Referee weapon records cannot populate actor feedback |
| Fused weapon input | Accepted SIM-M01 Entity identity/source/status/kinematic/loss contract and modeled delivery into weapon guidance. Current un-fused DMS tracks and idealized designation support do not establish that contract |
| Complete fleet and integration | Reconcile actual fitted weapon variants against the maintained supported fleet; admit per-variant inputs and independent validation, then consume the generic owner's gateway/installed-consumer seams and required Viewer cases |

At this initial correctness baseline, dispensers advanced before observation and
fuze processing reconstructed a prior decoy position from final velocity. The
subsequent [physical-chain correction](sim-m03-physical-chain-audit.md) fixes those
timing mechanisms with retained analytic trajectories. Cartridge/signature and
sensor calibration remain open.

That baseline also assessed separated panels as attached. The physical-chain
correction supplies immutable attached membership; the reference continuation
fixes forward fragment crossings and containment ordering. Damage to detached
bodies and cross-body shielding still require explicit ownership and
shared-fragment contracts. These mechanism corrections do not calibrate lethality.

Existing native geometry, fragment and guidance implementations are reused for
these correctness fixes, with no new solver or dependency. Finite control
integration reuses the existing actuator mechanism described in
[JSBSim FGActuator documentation](https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGActuator.html), which
describes lag, rate and position limits. It supplies a mechanism, not a particular
missile's response parameters. No upstream example value is admitted as
equipment calibration here.

The research directory's inspected `deliveries/completion/README.md` explicitly
marks its three-issue package incomplete. Later candidates remain that owner's
active work and need their own independent handoff acceptance. No numerical
research result is promoted into this SDK merely because the folder exists.
