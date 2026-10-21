# Native payload provider registration and execution

Baseline: main `bf16f80b514e166ee8a618ae032bf99303202dad`. The scoped working
tree is clean. The user assigns nine-role native adapter replacement, card
selection, explicit configuration, regressions and fullscreen evidence.

## Gaps and order

1. Provide a typed native factory catalogue with explicit registration,
   replacement, immutable World snapshots and atomic creation refusal. Reject
   unknown identities, wrong roles, wrong configuration versions, null results
   and exceptions. No mutable process-wide provider state.
2. Route seeker and guidance execution, all five native plant bindings and
   command stages, and existing fuze/warhead instances through selected factories.
   Construct required instances before native entity publication. World retains
   permissions, observations, RNG, events and damage authority; JSBSim retains
   integration. No physical algorithm or numerical input is fitted or upgraded.
3. All 24 guided cards use explicit autopilot parameters under the same schema.
   Preserve existing values and arithmetic. Existing swept-path/zoned-source
   models remain explicit algorithm choices; descriptive names are not claims
   of physical-model upgrades. Old IDs remain compatibility aliases only.
4. Add meaningful rejection, replacement, snapshot/isolation and production
   dispatch regressions before implementation. Regenerate maintained native
   assets through Hangar and verify numerical equivalence.
5. Root alone holds the serial native build/test and complete GUI scenario
   slots. Run bounded current gates and both fixed 5 km scenes fullscreen.
   Retain original physical failures and commit the reviewed software delivery.

## Exact write allowlist and symbols

* `src/simulation/mission_systems/components/Providers.h` and `.cpp`: typed
  `ProviderTableV1`, `ProviderRegistryV1`, runtime interfaces and built-ins.
* `src/simulation/mission_systems/components/providers/`: `Builtins.h`,
  `NativePlantBinding.h`, and `Seeker.cpp`, `Guidance.cpp`, `Autopilot.cpp`,
  `Allocator.cpp`, `Actuator.cpp`, `Propulsion.cpp`, `Aerodynamics.cpp`,
  `Fuze.cpp`, `Warhead.cpp`: separate concrete providers and registration.
* `src/simulation/mission_systems/weapon/adapters/NativeFuze.h` and `.cpp`,
  `src/simulation/mission_systems/damage/adapters/NativeWarhead.h` and `.cpp`:
  move concrete effect classes unchanged out of descriptor/factory code.
  Follow-up self-audit separates abstract `CapturedFuzeLifecycle.h` and
  `WarheadSnapshot.h` from concrete `SweptPathFuzeV1.cpp`,
  `TurningCpaFuzeV1.cpp`, `ZonedCaseWarheadV1.cpp`, `SideCaseWarheadV1.cpp`.
  Each of the five native plant roles now has its own typed abstract interface
  and factory table; shared binding helpers contain no component algorithm.
* `src/simulation/mission_systems/weapon/Composition.h` and `.cpp`:
  registry-aware `configureCompositionV1` overload; retain existing signature.
* `src/simulation/mission_systems/weapon/PlantAdapter.h` and `.cpp`:
  additive native command-input predicate; unchanged existing calculations.
* `src/simulation/mission_systems/weapon/FuzeAdapter.cpp` and
  `src/simulation/mission_systems/damage/WarheadAdapter.cpp`: descriptive
  aliases for existing algorithms, with unchanged parameter contracts.
* `src/simulation/runtime/world/World.h`, `World.cpp`, `WorldEngagements.cpp`,
  `WorldDamage.cpp`: per-World catalogue, per-release provider instances,
  prepublication admission and selected native execution.
* `src/simulation/mission_systems/CMakeLists.txt`, `tests/CMakeLists.txt`:
  feature source and regression registration.
* `tests/simulation/test_payload_providers.cpp` and
  `tests/simulation/test_payload_composition_native.cpp`: typed replacement,
  isolation, refusal and actual native dispatch fixtures.
* `tests/simulation/test_payload_composition.cpp` and
  `tests/simulation/test_payload_composition_contract.cpp`: the maintained
  catalogue now requires explicit control version 1 independently of the old
  top-level flag, with exact preservation checks for every controller value.
* `assets/stores/*.json`: only existing guided-card control declarations and
  descriptive effect implementation IDs; no existing equipment values change.
* `aircraft/stores/*/*.xml`: Hangar regeneration of changed component metadata;
  engine/nozzle and physical functions remain unchanged.
* `tools/models/hangar/tests/test_components.py`: explicit-control migration
  and preserved generation regressions.
* `tools/models/hangar/hangar/components.py`: migration proposals now expose
  the existing generic control constants explicitly without adding a false
  top-level execution-version block or changing consumed native values.
* `docs/sdk/payload-native-providers.md`,
  `docs/sdk/payload-component-configuration.md`,
  `docs/sdk/payload-plant-adapters.md`,
  `docs/sdk/payload-seeker-guidance-adapters.md`,
  `docs/sdk/payload-fuze-warhead-adapters.md`,
  `docs/sdk/payload-model-composition-audit.md`,
  `docs/sdk/sim-m03-five-km-repair.md`, `docs/status.md`, this plan:
  contracts, actual evidence and pending gates.

This native source-extension package does not add a binary DLL ABI or claim
calibrated equipment performance. Existing public layouts/signatures and
plugin vtables are unchanged. Missing MK 82/CJ-10A physical providers and the
original fragment/component failures are not closed by software refactoring.

## Delivered verification on the frozen provider source

The implementation now uses nine distinct typed execution interfaces and
eleven named concrete classes, with abstract asset-binding/lifecycle helpers.
The native World catalogue is copied before vehicle/store loading. Required
instances are constructed before publication and inventory/identity consumption.
The production replacement fixture checks selected factories, retained World
membership, native seeker/guidance/plant dispatch and separate referee warhead
dispatch. That referee fixture makes no intercept or damage acceptance claim.
The fuze factory is counted; its lifecycle uses the existing virtual execution
path rather than a separate execution-counter wrapper.

Meaningful RED evidence was captured before implementation: the missing
built-in catalogue failed nine registration assertions, and the explicit-control
card tests exposed 24 failures before migration. The later control-contract
fixtures caught two obsolete version expectations. Updated fixtures require
explicit version 1 and exact equality of every pre-existing controller value;
the original physical Hit/component-hit assertions are unchanged.

All evidence paths below are under `build/architecture-reviews/`:

| Gate | Current result | Evidence |
| --- | --- | --- |
| Complete Release and Headless build targets | Both pass | `payload-providers-{release,headless}-build.log` |
| Focused Release CTest | 7/7 pass, 2.95 s | `payload-providers-focused.{log,xml}` |
| Focused Hangar | 14/14 pass, 5.603 s | `payload-providers-hangar.log` |
| Installed equipment consumer | 1/1 pass, 34.28 s | `payload-providers-installed.{log,xml}` |
| Scoped Release physical selection | 118/120, two failures, zero skips, 75.35 s | `payload-providers-release-physical.{log,xml}` |
| Scoped Headless physical selection | 118/119, one failure, zero skips, 75.09 s | `payload-providers-headless-physical.{log,xml}` |
| All-store generation comparison | 53 physical XML models equivalent; 48 engine/nozzle files byte-identical | `payload-providers-generation-proof.json`, `payload-providers-card-migration.json` |
| Source freeze after all gates | All 87 fingerprints match | `payload-providers-code-freeze.json` |

The selections were resolved against fresh CTest inventories (891 Release,
817 Headless). These inventories are not complete-suite execution claims.
The original rendered showcase fragment Hit and 6 m side-burst component-hit
requirements remain failed; the latter is also the Headless failure. The
pre-existing C172/Skua damage-generation drift is not reclassified or repaired
by this package. No test is disabled, weakened or replaced by an event check.

Both unchanged 5 km scenes ran sequentially with fullscreen Viewers on the final
binaries. The native fixture passes its existing mechanical/continued-flight
requirements and shows actual nose separation/upset at 18.9 s and 21.8 s:
`sim-m03-five-km-viewer/native-providers-functional_004.png` / `_005.png` and
`NATIVE-PROVIDERS-functional-native-engagement-1-5000.000000.fsrec`.
The showcase shows the separated nose and sustained upset at 30.3 s:
`native-providers-showcase_003.png`, `NATIVE-PROVIDERS-showcase.fsrec` and its
`.fsrec.engagement.json`. Its final 40 s native sample is 4807.9217 m,
70.5778 m/s, `crashed=false`, `diverged=false`. These mechanical-contact
observations do not pass the remaining fragmentation gates. All screenshots
are 3840 by 2160. The visual harness's initial null process exit-code read was
corrected by retaining the owned process handle; both rerun producers exit zero.

The reviewed software package remains separate from complete SIM-M03 physical
acceptance, missing MK 82/CJ-10A models and any external numerical handoff.
