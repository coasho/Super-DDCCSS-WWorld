# SIM-M03 variant actuation and atmosphere-fed propulsion completion

Baseline: main `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f` with the reviewed retained dirty implementation. The root stopped Source G and owns all native build/test/GUI execution. No branch, worktree, public layout, provider vtable or retained implementation ID changes are authorized here.

## Exclusive ownership

Source-only effects/actuation owner: `tools/models/hangar/hangar/{missiles.py,storecontrol.py,storestages.py,thrustvector.py,ductedrocket.py}`, new `rolleron.py`; corresponding `tools/models/hangar/tests/{test_missiles.py,test_storecontrol.py,test_thrustvector.py,test_ductedrocket.py,test_rolleron.py}`; this plan and `docs/research/sim-m03/actuation-propulsion.md`. Preserve old v1 output contracts. Root explicitly approved the additional private `src/simulation/mission_systems/weapon/PassiveRolleronV1.{h,cpp}` and `tests/simulation/test_passive_rolleron.cpp` paths. Root retains JsbsimModel native-substep integration. No public layout changes.

Root retains cards, component descriptor/provider admission, shared World, CMake, public interfaces and all normal generation/native/installed/visual gates. This owner supplies exact additive parameter/property contracts. No native execution or mutation of those shared paths.

## Observed production state

R-73 and PL-10 currently do not select thrust vectoring in their actual autopilot bindings. `thrustvector.py` already produces finite native nozzle pitch/yaw actuators and uses JSBSim's native thrust/moment transform for selected AIM-9X/IRIS-T fits. Reuse this real plant and admit the missing fits, with explicit uncertain inputs and independent native sign/lag/stops/thrust-on/coast tests; do not substitute a requested guidance acceleration.

AIM-9M rolleron geometry is currently visual only. An actual implementation needs airflow-driven wheel angular momentum, a physical hinge state and finite stops, with its native aerodynamic patch changing forces/moments. A wheel picture or direct body damping coefficient cannot satisfy this feature.

Meteor v1 already has an atmosphere-fed cycle, separate booster/fuel reservoirs, finite fuel-valve lag/rate/stops and actual native mass flow/enthalpy/net momentum thrust. It must not be described as only a timed sustainer. A concrete remaining gap is capture from scalar true airspeed even under reverse/oblique flow. Add an explicit v2 inlet/response contract using positive axial air-relative flux and coherent mass/enthalpy/ram-drag accounting; preserve retained v1 outputs and selection semantics. A finite response must describe its actual inventory/through-flow approximation.

## Models and acceptance

- R-73/PL-10: actual current profile admission and finite native nozzle execution, no fabricated measured coefficients. Primary mechanism evidence and hardware-input uncertainty are separate.
- AIM-9M: passive gyro-actuated tail-tip control patches, no electrical roll-command authority, finite wheel/hinge state, stops/reset and actual shared VLM/native loads. NACA RM SL55C22 supplies the mechanism and warns that hinge damping matters; its old test configuration is not AIM-9M calibration.
- Meteor: explicit opt-in v2 positive axial intake and finite response, current native fuel depletion/CG, mass/energy/ram momentum balance, throttle/lightoff/unstart/starvation/reset and malformed input behavior. NASA Glenn ramjet conservation equations support the method; no manufacturer inlet/combustor/nozzle map is invented.

Write meaningful negative tests before production correction. Root captures actual RED then runs GREEN, complete normal model generation, native response and unchanged full encounter predicates. Reference consultation or a provider label is not a passing mechanism gate. Existing fixed physical scenes, seeds, acceptance predicates and calibration limits remain unchanged.

## Native flight implementation handoff

Root's explicit extension assigns `src/simulation/flight/JsbsimModel.{h,cpp}`, relocated private `src/simulation/flight/PassiveRolleronV1.{h,cpp}`, `tests/simulation/test_passive_rolleron.cpp` and new `tests/simulation/test_variant_actuation_native.cpp`. The former mission_systems helper paths are removed by relocation, not installed or exposed. Native helper ownership stays in `fsim::sim` with a local three-double vector; flight must not depend on mission_systems. Root owns CMake/architecture registration and actual model generation/build/test. Native tests load actual normally generated AIM-9M/Meteor products and step JSBSim; no replacement dynamics fixture is admitted.

Root additionally assigned V2 production admission in `src/simulation/mission_systems/components/registration/PropulsionProvidersV1.cpp`, `src/simulation/mission_systems/components/implementations/propulsion/JsbsimDuctedRocketPropulsionV1.{h,cpp}`, `src/simulation/mission_systems/weapon/PlantAdapter.cpp`, `tools/models/hangar/hangar/components.py`, plus new `tests/simulation/test_variant_plant_admission.cpp`. V2 code is 5; v1 code 2 and old provider class layout/virtual contract remain unchanged. Sensor owner alone writes the current Meteor card.

The native numerical-boundary fixture is exactly `tests/simulation/data/inlet_step_admission/inlet_step_admission.xml`: an unpowered physical rigid body declaring only the inlet numerical contract. Its positive/negative load tests claim no airbreathing capability; all dynamic inlet/propulsion tests use actual normally generated Meteor products.

Root assigned `cmake/JsbsimPatches.cmake` for the explicit V2-only finite fuel reservation fix. The upstream README and exact native scheduling/fuel source were checked. Vendored files remain unchanged; the normal existing exact-text patch mechanism generates modified FGRocket/FGPropulsion sources and an additive nonvirtual helper declaration. No data member, vtable or legacy V1 branch changes. `JsbsimModel` also implements the separately authored `NativeThermalSourceV1` sibling using real native temperatures; the sensor owner owns that header and World consumer.

Checkpoint96 first native behavior revealed the actual authored `aircraft/aim9m/aim9m.toml` still lacked physical patches, so root explicitly assigned that exact source path as well as preset consistency. Root also assigned fresh native FDM load/reload lifecycle repair in the existing JsbsimModel window: copied complete RNG state, cleared owned property/cache handles before old FDM destructor Unbind/DeAllocate, unchanged Vehicle.reset behavior and public contracts. Existing native reload regression remains mandatory.
