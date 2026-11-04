# SIM-M03 commanded dual-pulse energy management

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`.
Root assigns the complete engineering feature. Sensing worker owns the paths
below; root serializes World declarations/calls and every native build/test.
No vendored source or public SDK layout changes are permitted.

## Read before implementation

The vendored `third_party/jsbsim/README.md` identifies the native C++ integration
and current supported Windows tooling. Dependency version changes are unnecessary.
Vendored `FGRocket.cpp` starts solids with throttle=1 and then latches BurnTime>0;
later throttle commands cannot pause or relight one authored pulse. Its fuel
flow derives from native vacuum thrust/Isp. Existing generated
`fsim/ignition-time-thrust-v1` tables use native BurnTime, including the complete
original 2 N coast and buildup correction. These remain the exact legacy mode.

## Exact write allowlist and handoff boundaries

- This plan and `docs/sdk/weapon-pulse-energy.md` (new).
- `src/simulation/mission_systems/weapon/PulseManagerV1.h`, `PulseManagerV1.cpp` (new).
- `src/simulation/mission_systems/weapon/PlantAdapter.cpp`: admit the new versioned propulsion selection and validate complete explicit pulse roles.
- `src/simulation/mission_systems/components/registration/PropulsionProvidersV1.cpp`: register the new selection using existing native solid-engine binding.
- `tools/models/hangar/hangar/components.py`: recognize/stamp new propulsion code and explicit policy roles.
- `tools/models/hangar/hangar/storebodies.py`: emit the opt-in commanded table name and native metadata; retain old timed output unchanged.
- `assets/stores/pl-15.json`, `assets/stores/r-37m.json`: propulsion selection/policy parameters and uncertainty only; no existing phase/thrust/Isp/mass/guidance scalar change.
- `tests/simulation/test_weapon_pulse_energy.cpp` (new), plus new fixture files under `tests/simulation/data/rocket_commanded_pulse/`.
- `tests/simulation/test_payload_plant_adapters.cpp`: focused code-4 actual-native identity admission and invalid-slot regressions, added after the current PL-15 release exposed the missing `plantAvailableV1` dispatch case at unchanged main HEAD `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`. The root granted an exclusive correction window for this test and `PlantAdapter.cpp`; all other native inputs remain frozen.
- `src/simulation/runtime/world/WorldWeaponControls.cpp`: only after link owner explicitly releases it, implement root-approved private energy helper.

Root owns applying reviewed snippets to `cmake/JsbsimPatches.cmake`, relevant
CMake source/test lists, `World.h`, `World.cpp` preStep and release setup. Those
files are not worker write paths. Vendored README/rocket source is read-only.

## Physical method

New selection `fsim.propulsion.jsbsim_commanded_dual_pulse.v1` chooses an explicit
new native named table. The original first pulse and complete original coast
execute unchanged. After the original coast ends, thrust is zero while a second
ignition decision is pending. A one-shot property-backed native ignition latch
then maps elapsed second-pulse time onto the unchanged authored second phase.
This avoids adding arbitrary 2 N coast impulse while waiting. Total authored
vacuum impulse and propellant remain unchanged when the second pulse completes.
An uncommanded or expired second pulse retains its unspent propellant.

The pure energy manager receives only own measured speed, native motor age/fuel
and the guidance's admitted time-to-go/measurement validity. Within the finite
configured window it requests ignition when terminal time-to-go requires the
reserved impulse or own speed falls below the declared energy reserve threshold.
No referee target position/velocity or mission outcome enters this decision.
The policy is a disclosed engineering rule, not proprietary optimal guidance.
Every policy coefficient has explicit card units, provenance and nonzero
screening uncertainty. Invalid/rewound input does not mutate the latch.

Native properties are initialized/reset per engine without new FGRocket members
or vtable changes. Ignition is one-shot and a native empty/starved motor remains
authoritative. Repeated commands cannot restart a consumed second pulse.

## Required evidence

1. Missing interface/native property RED before production.
2. Independent policy early/late/unknown/invalid/no-fuel/retry/reset cases.
3. Real native first/coast/held/commanded-second phases at two integration rates.
4. Original timed mode exact replay and no post-ignition throttle semantics change.
5. Independent vacuum impulse and propellant accounting, no-command retained fuel,
   one-shot ignition, starvation and fresh/reset equality.
6. Both current variants through normal generation, actual public release and
   installed/native fleet acceptance. No frozen scene or gain tuning is allowed.
