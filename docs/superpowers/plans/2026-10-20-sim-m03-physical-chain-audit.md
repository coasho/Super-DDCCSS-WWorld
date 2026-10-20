# SIM-M03 physical-chain audit and independent corrections

Baseline: main `16da711bf59665a538919f2abb2dfda2ab11278f`.
The user authorizes continued parallel SIM-M03 review and independent repairs;
Claude's SIM-M02 work remains in its separate validation directory and pending.

## Exclusive ownership

Root owns the serial native configure/build/test slot. Workers must not build,
edit shared integration, or inspect other checkouts. No public ABI, generic MA,
generated store model, motor, or calibrated equipment parameter change is included.

* Damage worker: `src/simulation/mission_systems/damage/{Damage,State}.{h,cpp}`
  and `tests/simulation/test_damage.cpp`. Symbols: attached geometry membership,
  assess overload, canonical panel blast impulse, stale detached-hit rejection.
* Decoy worker: `src/simulation/mission_systems/countermeasure/Dispenser.{h,cpp}`
  and `tests/simulation/test_countermeasure_core.cpp`. Symbols: owned timed
  trajectories, analytic sampling, individual environment and propagation validity.
* Guidance worker: `src/simulation/mission_systems/weapon/LaunchEligibility.{h,cpp}`,
  new private `TrackKinematics.h`, and
  `tests/simulation/test_{weapon_guidance,launch_eligibility}.cpp`.
  Symbols: measured NED conversion and source-scoped local illumination checks.
* Root: `src/simulation/runtime/world/World{,Engagements,Damage}.{h,cpp}`,
  `src/simulation/mission_systems/weapon/Fuze.h`, new private `MotionHistory.h`,
  `tests/simulation/test_{fuze,engagement_native}.cpp`, test and root CMakeLists,
  and `tools/validation/testing/scopes.toml` where routing needs correction.
  Symbols: decoy observation/advance order, attached assessment integration,
  measured designation velocity, local illumination identity, native substep
  histories/contact clipping, current motor reporting and physical weapon lifetime.

Documentation: this plan, `docs/sdk/{engagement,weapon-fidelity-validation,
weapon-seeker-validation,missile-control-validation,sim-m03-physical-chain-audit}.md`,
`docs/{status,backlog}.md`, the SIM-M03 allocation and handoff carryover register.

## Sequence and acceptance

1. Regress defects before changing their implementations. Preserve actual
   before/after decoy samples including partial final lifetimes; use each decoy's
   own local environment. Invalid propagation cannot publish frozen fresh motion.
2. Exclude separated geometry before interception, armor, blast and impulse;
   retain immutable shared resources and original source indices. Use one panel
   blast impulse expression for aggregate and section decomposition.
3. Consume measured track velocity without inventing target attitude. Scope
   local SARH support by source, payload and track. Report current motor thrust
   after native integration. Remove arbitrary low-speed deletion of live weapons.
4. Align fuze sweeps with actual native substeps and contact times; clip live
   target/decoy intervals before minimizing closest approach. Preserve terminal
   outcome meanings and deterministic worker-independent processing.
5. Run focused nonzero bounded tests through both UCRT64 presets, including
   production integration, public/installed interfaces and appropriate guards.
   Review staged feature-owned diff and commit after verification.

Maintain a source-linked approximation inventory with required inputs and owner.
Mechanism corrections do not establish physical calibration, complete SIM-M02/M03
acceptance, full regression or new Viewer evidence. Missing equipment, material,
sensor or detached-body contracts remain explicitly pending.
