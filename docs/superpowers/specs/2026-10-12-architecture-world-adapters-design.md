# Architecture A4: World, Scenario and Environment Adapters

## Intent and baseline

The user requests a further language-adapter split after committing A3, preserving public API/ABI, principal
permissions and simulation behavior. A3 is committed as `e79f268`; this slice continues from that baseline
and remains uncommitted until requested. Files use English.

Move existing implementations literally into focused source owners. Do not redesign native World state, scenario
application, environment physics, Python public classes, ownership, defaults or threading policy.

## Selected ownership

### C SDK

- `fsim_world_c.cpp` retains World/agent lifecycle, vehicle/state/property access, effects/communication and recording.
- New `fsim_scenario_c.cpp` owns the opaque Scenario implementation and its six existing exported definitions.
- New `fsim_environment_c.cpp` owns eight existing exports: environment initialization/get/set, terrain sampling,
  magnetic-field initialization/calculation, decimal year and World magnetic year.
- Five Environment definitions currently in `fsim_control_c.cpp` move to the Environment owner. All remaining
  control source stays literal. World exports decrease from 65 to 56; control from 233 to 228.
- Existing Handles, CAbiSupport, CParameterSupport, Mission and DMS owners remain unchanged. All files link into
  the existing SDK binary; no public declaration, principal classification or exported name changes.

Scenario preserves inherited C linkage, shared guard/error support, borrowed option strings and versioned writes.
Its apply function retains validation before count clearing, full created count despite truncated ID output, and
partial application if a later vehicle fails. No rollback is introduced. Environment retains four first principal
guards; Scenario retains its apply guard. Direct Environment success retains LastError, while guarded Scenario
success clears it. Preserve initializer behavior, finite checks, short output handling, NaN terrain gaps and
environment revision/publication effects.

### Python native module

- New `_native_world.c` owns 23 existing generic World handlers, including stepping, audits/identity, vehicle and
  property access, communication, gathers, capsule and borrowed status; its private vehicle-spec field table moves
  alongside the handlers.
- New `_native_environment.c` owns six World handlers for environment, effects, terrain and magnetic year, plus
  magnetic-field and decimal-year module functions; its Environment field table moves alongside them.
- New `_native_scenario.c` owns the four Scenario constructor/destructor/query callbacks.
- `_native.c` retains VecEnv/Recording, Error, BufferType, shared conversions, all method/slot/type/module registration
  and PyInit. World construction/borrowing/destruction, agent creation/sealing, state buffers and scenario application
  stay here because they directly use centrally owned types and lifetime support.
- `_native_world.h` shares the unchanged ScenarioObject layout, existing COUNT macro and private callable declarations.
  New `_native_world_options.h` shares the literal World-option metadata only between core and Scenario.
- Every new owner defines FSIM_PY_NO_BUFFER_TYPE. Only moved callable definitions lose static linkage; no new type,
  buffer or option-conversion accessor is introduced. Existing DMS/control/Mission sources and fsim_py stay unchanged.

This approach keeps moved bodies literal and registration central. Moving runtime types too would require additional
private accessors and widen lifetime risk; replacing conversion calls with wrappers would make body comparisons less
direct. The selected split permits shared immutable option metadata in the two translation units that consume it.

## Acceptance

1. Add meaningful public C/Python characterization for Scenario option lifetimes/versioning, truncated application,
   Environment errors/NaN/defaults/principals and native World/Scenario ownership. Run it against A3 before extraction.
2. Capture protected hashes, SDK exports and Python descriptors/functions/type layouts. Existing public headers,
   object-model files, resources, native domains, prior owners and compatibility fixtures remain unchanged.
3. Observe missing-owner source-verifier failure before extraction. Compare all moved definitions, original-file
   remainders, field tables, layouts, shared helpers and registration against A3 afterward.
4. Build headless with warnings as errors; run C ABI, Python SDK, principal/architecture checks and the full applicable
   functional suite, including determinism, lifecycle, buffers, agents and both package gates.
5. Refresh dist, compare installed-consumer DLL bytes and all runtime manifest hashes, and run binding compatibility
   against distributed Python. Independently review the complete adapter/test diff.
6. Update architecture, development, navigation and status; retain raw failure/correction evidence and explicit limits.

## Environment and stopping point

Use existing UCRT64 GCC/Ninja, C++17, C11, CMake 3.25+ and CPython >=3.11 stable ABI. Keep tools/caches under D:/ENV;
set TMP, TEMP and TORCH_EXTENSIONS_DIR to D:/ENV/tmp before compilation. No dependencies are installed.
Stop after local verification and independent review. Leave A4 uncommitted. Unchanged Hangar generation, renderer/GPU,
render SDK installation, CPack ZIP and remote CI are outside this slice. World state-owner decomposition, aggregate
history retention, finer control/store ownership, geometry, weapon observations and export policy remain separate work.
