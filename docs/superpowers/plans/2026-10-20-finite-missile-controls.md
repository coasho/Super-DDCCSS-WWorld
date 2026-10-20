# Finite missile controls

Baseline: main `85e7d251c5de702de5d2ec66115a0886bd0702cd`.
The user explicitly prioritised replacing ideal guidance force/moment injection
with controllable surfaces before further SIM-M03 work. Existing uncommitted
research links and `docs/sdk/weapon-model-research.md` belong to this task.

## Ownership and boundaries

The root owns the exclusive native configure/build/test slot and shared
integration in `World::{preStep,flyReleases}`, private `World::Entry`,
`WorldEngagements.cpp` guidance demand naming, mission-systems/test/deployment
CMake, and generated missile XML. A generator worker owns only
`tools/models/hangar/hangar/{storeaero,storebodies,storecontrol}.py` and matching
Hangar tests. A regression worker owns only
`tests/simulation/test_missile_control_native.cpp`. An independent read-only
worker assesses controller dynamics. No worker executes native builds.
Root-owned helper/test paths are
`src/simulation/mission_systems/weapon/Autopilot.{h,cpp}` and
`tests/simulation/test_missile_autopilot.cpp`. CMake paths are the root,
`src/simulation/mission_systems/` and `tests/` CMakeLists files. Generated
ownership is the 24 controlled AAM definitions under `aircraft/stores/`;
motor/nozzle outputs must remain identical.
The existing launch-CG case in `tests/simulation/test_mass_change.cpp` may
explicitly request/verify ignition before asserting burned-out CG. Its default
neutral aircraft controller must not be changed to make a powered fixture.
No generic MA/gateway/public ABI change, other worktree or AeroNumericsLab
mutation is authorised by this feature.
Native model-load diagnostics in `src/simulation/flight/JsbsimModel.cpp` may
preserve textual JSBSim parser exceptions while retaining load refusal.

Documentation allowlist: this plan, `docs/status.md`, `docs/backlog.md`,
`docs/sdk/{weapon-fidelity-validation,weapon-seeker-validation,weapon-model-research,missile-control-validation}.md`,
`docs/architecture/modules/simulation/ms-work-allocation.md`, and the SIM-M03
section in `docs/architecture/validation/handoff-carryover-register.md`.
Generator/native test routing additions are confined to
`tools/validation/testing/scopes.toml` if the existing graph lacks coverage.

## Implementation

1. Regress the actual release path: finite deflection/rate, neutral launch,
   native aerodynamic response, stuck-actuator loss of control authority,
   zero guidance external reactions and worker determinism.
2. Generate a finite actuator chain and force/moment increments from authored
   moving fin geometry and control metadata. Preserve uncontrolled stores,
   motor/grain/ignition files and generator ownership. Fin forces and their
   moments share a pressure centre; deflection drag dissipates energy.
3. Add a private bounded autopilot that converts held guidance acceleration
   demand into standard surface commands at native FDM cadence. Only JSBSim
   actuators and aerodynamics realise the demand. Remove ideal replacement
   force/moment code completely. Unsupported/invalid control inputs produce
   neutral commands, never ideal-force fallback.
4. Validate generator, controller and production regressions, then the existing
   engagement, reset, public/installed and architecture guards appropriate to
   the changed paths in the existing presets. Review staged changes and commit.

All new response constants and control-effectiveness estimates must be named
engineering assumptions. Software-mechanism verification does not establish
real variant calibration, complete SIM-M02/M03 acceptance or new Viewer evidence.
