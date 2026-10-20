# SIM-M03 Independent Correctness Implementation Plan

> **For agentic workers:** Use parallel, disjoint feature tasks. The root worker owns shared integration and the exclusive serial native build/test slot. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct independently reproducible seeker, fuze, damage and actor-feedback defects while awaiting the SIM-M02 numerical handoff.

**Architecture:** Preserve SDK simulation authority and existing public layouts. Keep seeker/temporal geometry in private weapon helpers, target damage in the native damage model, and actor visibility at the existing public endpoint boundary. World alone integrates those behaviors with JSBSim and terminal events.

**Tech Stack:** C++17, existing UCRT64 presets, Catch2/native fixtures, C99 and the existing CPython SDK.

**Spec:** [MS-7d](../../architecture/modules/simulation/adr-30-mission-systems.md), [SIM-M03 allocation and physical acceptance](../../architecture/modules/simulation/ms-work-allocation.md), [engagement interface](../../sdk/engagement.md).

## Baseline and constraints

- Main HEAD before edits: `0d4f25a5df3fdd23696e696644cbd0ee32ab23af`; the working tree was clean. There was no pending change to commit at task entry.
- The user assigned the whole SIM-M03 package, then confirmed SIM-M02 research continues in the standalone `D:/Projects/Cpp/AeroNumericsLab/` directory and dependency-bound work must await its results.
- Work remains on main. No other worktree, generated model, sensor card or generic gateway/schema is modified. Public signatures, layouts, enums and native outcome meanings remain unchanged.
- Only the root worker executes native configure/build/tests. Freeze their inputs while the serial slot is occupied. Set TMP, TEMP and TORCH_EXTENSIONS_DIR to `D:/ENV/tmp` using the existing launcher.
- Synthetic tests establish software invariants, not equipment calibration. Existing acquisition/aspect/discrimination constants and the ideal force/moment servo remain explicit assumptions.
- Full SIM-M03 remains open until accepted aerodynamic/control and sensor inputs, actual-equipment provenance and independent quantitative validation are available.

## Review focus

1. Invalid/nonfinite/reversing guidance input must not mutate an accepted seeker history.
2. On-rail and released IR reach must agree, including the existing afterburner multiplier; defensive geometry limits must remain effective.
3. Body-fixed seeker field of regard must not silently follow the velocity vector at nonzero angle of attack or sideslip.
4. A step crossing arming or expiry must not use excluded closest approaches; fractional damage geometry retains the original step coordinates.
5. A readable own-platform activity must not reveal an unpermitted target identity through C++, C99 or Python; Range/referee access and legal own designations retain their meanings.

## Task 1: Seeker observations and telemetry

**Exclusive files:** `src/simulation/mission_systems/weapon/{Guidance.h,Guidance.cpp,Seeker.h}`, `tests/simulation/test_weapon_guidance.cpp`, `docs/sdk/weapon-seeker-validation.md`.

**Interfaces:** `infraredAcquisitionRangeM(baseM, aspectCosine, afterburning)` preserves the existing assumed IR rule. An observation may carry a finite admitted reach; absent reach retains the base cap. An input may carry a body-fixed boresight. Unknown timing uses NaN; active-radar time to active is zero after activation.

- [x] Add failing duration, age, specification and unknown-timing regressions; observe each fail against unchanged production.
- [x] Add shared-envelope and body-boresight checks before implementing their private interfaces.
- [x] Implement fail-closed input validation without changing admitted equipment parameters.
- [x] Run the complete core fixture and native engagement controls; document actual results and limitations.

## Task 2: Temporal fuze and target momentum

**Exclusive files:** `src/simulation/mission_systems/weapon/Fuze.h`, `src/simulation/mission_systems/damage/Damage.cpp`, `tests/simulation/{test_fuze.cpp,test_damage.cpp}`, `docs/sdk/weapon-fuze-validation.md`.

**Interfaces:** `sweptFuze(ownBefore, ownAfter, targetBefore, targetAfter, ageBeforeS, ageAfterS, spec)` returns validity, trigger, miss and original-step fraction. Its active interval is `[armingS, lifeS)`. Keep burst-at-closest-approach semantics. Sum fragment momentum only where it transfers into the target.

- [x] Add synthetic arming/expiry/relative-motion and invalid-input checks before the helper; observe its missing-header RED.
- [x] Add a fragment energy-versus-momentum conservation regression; observe the uncorrected native model fail it.
- [x] Implement the temporal helper and remove air-drag impulse from target momentum.
- [x] Integrate the helper in World for aircraft and decoy fuzes; verify native damage/engagement outcomes.

## Task 3: Direct actor strike-status visibility

**Exclusive files:** `src/simulation/api/Agent.cpp::Agent::strikeStatus`, `src/adapters/sdk/c/fsim_mission_c.cpp::fsim_strike_status_get`, `tests/compatibility/{test_sdk.cpp,python/test_engagement.py}`, `tests/training/test_ma_equipment_c.c`, `docs/sdk/weapon-feedback-validation.md`.

**Interfaces:** Keep existing getters and sized C output. Apply the established equipment-report target mapping: an actor sees its own designation handle, otherwise zero. Preserve store/release/activity correlation and Range access. This does not introduce a modeled weapon return link.

- [x] Reproduce raw target-ID disclosure through all three public interfaces.
- [x] Apply the existing own-designation visibility rule to direct getters; retain foreign-activity refusal and short-buffer behavior.
- [x] Verify positive own/lapsed designation controls and C++/C99/Python negative cases on the final binary.

## Task 4: Shared integration, review and delivery

**Root-only files:** `src/simulation/runtime/world/WorldEngagements.cpp`, `tests/simulation/test_engagement_native.cpp`, `tests/CMakeLists.txt`, this plan, `docs/sdk/{engagement.md,weapon-fidelity-validation.md}`, `docs/status.md`, the dated SIM-M03 evidence in `docs/architecture/modules/simulation/ms-work-allocation.md`, and the dated findings in `docs/architecture/validation/handoff-carryover-register.md`.

- [x] Supply current body attitude and admitted IR reach from World; reuse the same helper for on-rail acquisition.
- [x] Replace the full-step armed-at-end sweep with the active-interval helper.
- [x] Update the native unknown-time assertion and register the temporal-fuze fixture.
- [x] Run bounded related native/C/Python and architecture checks with nonzero discovery; add cross-feature review and resolve material findings.
- [x] Record exact achieved/pending gates, inspect the reviewed staged diff, commit only feature-owned files, and release the slot.

Final verification: 9/9 core CTest, 8/8 public/installed CTest, 20 damage cases
(4,814 assertions), and `ma_contract` pass. The additional selector self-check
has 100/101 internal passes and one baseline CMake-routing failure, reproduced
with the unchanged HEAD rules. Preserve that explicit limitation as HC-53;
correcting unrelated test infrastructure is outside this independent feature.

## Deferred full-package gates

Accepted SIM-M02 aerodynamic/control derivatives and motor/geometry data are prerequisites for calibrated finite fin/autopilot response. Active/SARH reception needs admitted radar parameters and per-candidate signal/illumination, while IR fidelity needs admitted signatures, transmission and gimbal/track-rate data. A modeled weapon return transport with assessment lifecycle and accepted fused Entity updates needs the frozen SIM-M01 and generic-owner seams. Actual per-variant fuze/lethality data, independent validation envelopes and fleet evidence remain required. None is replaced by the software checks above.
