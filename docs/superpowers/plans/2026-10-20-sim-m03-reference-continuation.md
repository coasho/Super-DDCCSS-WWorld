# SIM-M03 reference continuation implementation plan

This is the historical reference-window plan. The subsequent
[SDK delivery plan](2026-10-20-sim-m03-sdk-delivery.md) supersedes its acceptance
checkpoint: both fixed 5 km real-damage gates remain active and fail when actual
damage is absent. Historical software-only checks are not current hit acceptance.

> **For agentic workers:** Use the existing accepted MS-7d design and independent regression workflow. The user explicitly selected parallel work; shared native writes and all native configure/build/test execution remain serial.

**Goal:** Advance the complete SIM-M03 package through primary-source comparison, independently reproducible corrections and current production-path evidence while preserving unaccepted model inputs as explicit dependencies.

**Architecture:** Keep the SDK and JSBSim as simulation authority. Guidance commands the finite autopilot, channel and individual-fin actuators, then native aerodynamic loads drive rigid-body motion. Sensor observations, fuze eligibility, native damage and actor-permitted communication retain separate authority and lifecycle boundaries.

**Tech stack:** Existing C++/C99/Python SDK, Catch2, Hangar generator, UCRT64 Headless and render-enabled Release presets. No dependency installation or toolchain change.

**Spec:** `docs/architecture/modules/simulation/adr-30-mission-systems.md` (MS-7d), `docs/architecture/modules/simulation/ms-work-allocation.md`, and `docs/sdk/weapon-fidelity-validation.md`.

## Baseline and exclusive windows

Baseline: main `fabeaa4fe56d1a878a5f3d0614729d8e940d5719`.
At entry, eight tracked source/test files and the two thirty-kilometre diagnosis documents already had changes. Preserve and review that delta; do not imply it was authored in this continuation. No branch, worktree, external research directory or generic MA integration is included.

Root holds the exclusive shared write/native configure/build/test slot for this continuation. No competing native builder, CTest runner or Viewer was present at the initial process check. The three parallel agents have read-only domain scopes: control, seeker/feedback, and fuze/lethality. They may browse primary sources, but may not edit build inputs or execute native gates. Transfer any implementation window explicitly before its first write; pause all affected inputs during a native build/test.

## File and symbol allowlist

* Inherited delta: `src/simulation/mission_systems/weapon/{Autopilot,Guidance}.{h,cpp}`, `src/simulation/runtime/world/World.{h,cpp}`, `tests/simulation/test_{engagement_native,missile_autopilot,weapon_guidance}.cpp`, and the existing `docs/{sdk/thirty-km-engagement-diagnosis.md,superpowers/plans/2026-10-20-thirty-km-diagnosis.md}`. Review/verify before any further change.
* Root documentation: this plan, `docs/sdk/weapon-model-research.md`, new `docs/sdk/sim-m03-reference-continuation.md`, `docs/sdk/{weapon-fidelity-validation,missile-control-validation,sim-m03-physical-chain-audit}.md`, `docs/status.md`, and only the SIM-M03 entries in the allocation/carryover register.
* Any newly demonstrated source defect requires a recorded exact file/symbol extension and a failing regression before implementation. No generated model, motor, gain, fuze bound or acceptance threshold changes to obtain an interception.

### Recorded independent correction windows

The initial complete Headless build passed before these windows. Native gates are paused while agents write their disjoint test/source inputs.

* Control agent: `src/simulation/mission_systems/weapon/Autopilot.cpp` (`finCommands`, `surface`) and `tests/simulation/test_missile_autopilot.cpp`. Regress the omitted diagonal-inertia Euler coupling `omega x (I omega)` against an independently formed rigid-body equation; preserve the inherited measured-load correction and all constants. Add tests first, then wait for root's captured RED run before the production edit.
* Fuze/lethality agent: `src/simulation/mission_systems/damage/Damage.cpp` (ray/slab and fragment assessment only) and `tests/simulation/test_damage.cpp` (exact tracked path confirmed before its first edit). Regress inside-body outward skin crossings and sub-millimetre near-origin surface crossings before correction. Preserve stable component identities, physical coefficients and native event meanings. Add tests first, then wait for root's captured RED run before the production edit.
* Root alone runs native builds/tests, updates shared documents and reviews each delta. These disjoint writes do not authorize changes to World, public interfaces, model generators or other-owner seams.
* Root also corrects the inherited native engagement fixture's unconditional implication from detonation to a fragment-hit event. The unchanged 5 km scene reproduced a burst at 10.2878 m, zero of 2023 fragments striking and applied native blast impulse; the old assertion failed. Keep the exact detonation/target and event/terminal-pose bounds, require native damage assessment, and validate every emitted damage event against native crew state. Retain the separate swept-damage fixture's positive hit requirement. Do not add a synthetic damage event to production.

## Review focus

* Native load samples must use matching frames, CG, rates and actual fin positions; applied reaction channels remain zero.
* Active seeker search may steer from retained permitted estimates without manufacturing a current observation or fuze authority.
* A software interception and numerical agreement do not establish admitted equipment response or a real launch envelope.
* Missing/invalid/stale information fails explicitly; modeled delivery and actor visibility cannot be replaced with referee truth.
* Detached-body damage needs component/state/geometry/event ownership before extending the parent damage path.

## Task 1: Review inherited diagnosis and reproduce current control chain

- [x] Review all inherited source/test changes, prior RED evidence and exact model provenance.
- [x] Build the current complete Headless target set with the existing launcher and bounded execution.
- [x] Discover the actual tests, execute focused autopilot/guidance/finite-control/native engagement gates and reproduce one/two-worker results.
- [x] Resolve demonstrated correctness findings with independent regressions, without changing physical values or original acceptance bounds.

## Task 2: Compare mature implementations across the full package

- [x] Parallel read-only primary-source evaluation of flight control, seeker/feedback, fuze/lethality and numerical reference cases.
- [x] Record candidate versions/licenses, exact reuse purpose, native seam, independent reference cases and limits. Research examples are not equipment calibration.
- [x] Implement independently justified corrections within an explicitly extended exclusive window; leave unavailable physical and other-owner inputs pending.

## Task 3: Verify and reconcile package acceptance

- [x] Run appropriate nonzero bounded Headless and Release gates on frozen source, including affected public/installed compatibility paths.
- [x] Broaden to current regression only for package-level evidence and record the actual discovered count/results: 744/746 Headless, 56/56 focused Release; HC-57/HC-58 retained without changing assertions.
- [x] Review the final diff and update current diagnosis, status and remaining acceptance dependencies with exact evidence paths.
- [x] Release shared write/build slots at handoff. Full SIM-M03 remains open unless every physical, modeled-link, variant and required Viewer gate is supported by current evidence.
