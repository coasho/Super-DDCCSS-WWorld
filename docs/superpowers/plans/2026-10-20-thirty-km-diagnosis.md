# Thirty-kilometre engagement diagnosis

This is the initial diagnosis window. The subsequent
[SDK delivery plan](2026-10-20-sim-m03-sdk-delivery.md) records the expanded
production/public/model scope and current gates; its unchanged fixed-scene hit
requirements are not satisfied by detonation alone.

Baseline: main `fabeaa4fe56d1a878a5f3d0614729d8e940d5719`.
The user authorizes cause isolation and independent repairs, followed by verified
commit. No parameter or acceptance-bound adjustment is authorized to obtain a hit.

Root owns the exclusive shared write and serial native configure/build/test slot.
Claude's separate SIM-M02 validation directory remains untouched. No branch or
worktree, generic MA package, public ABI or generated model change is included.

## File and symbol allowlist

* `tests/simulation/test_engagement_native.cpp`: existing 30 km scenario,
  opt-in diagnostic trace, focused regression and causal controls. Keep the
  original geometry, seed, gains, actuator/model values and acceptance bounds.
* `src/simulation/mission_systems/weapon/{Guidance,Autopilot}.{h,cpp}` and
  `tests/simulation/test_{weapon_guidance,missile_autopilot}.cpp`: only identified
  algorithm/implementation corrections supported by independent regressions.
* `src/simulation/runtime/world/World{,Engagements}.{h,cpp}`: only identified
  execution/measurement wiring defects; no ideal force/moment fallback.
* `tests/CMakeLists.txt`, root CMakeLists and
  `tools/validation/testing/scopes.toml`: test registration/routing if required.
* This plan, new `docs/sdk/thirty-km-engagement-diagnosis.md`,
  `docs/sdk/{sim-m03-physical-chain-audit,missile-control-validation}.md`,
  `docs/status.md`, and the SIM-M03 allocation/carryover sections.

## Evidence and decision sequence

1. Reproduce the unchanged fixture. Trace aligned estimated/observed target
   geometry, guidance phases, commands, finite fins, native forces/moments,
   attitude/rates, motor/energy and closest encounter. Diagnostics cannot feed
   target truth or derived demands back into execution.
2. Isolate guidance geometry, control tracking/execution and model authority
   using analytic limits and controlled tests. Distinguish the initial miss
   from subsequent track loss, pursuit, ground impact or configured expiry.
3. Regress definite independent defects before repairs. Preserve all empirical
   constants and generated model bytes. Counterfactual experiments are labelled
   diagnostics, never admitted equipment inputs or calibration.
4. Verify focused nonzero bounded gates on both existing UCRT64 presets,
   including production path and relevant public/installed compatibility.
   Record remaining SIM-M02/sensor dependencies and current quantitative result.
   Inspect the staged owned diff, commit, and release the write/build slot.
