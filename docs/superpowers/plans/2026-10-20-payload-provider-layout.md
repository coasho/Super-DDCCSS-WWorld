# Payload provider source organization

Baseline: main `ab7f89d055f38f5ecdb9612426dabf431469eda0`, clean working tree.
The user assigns consistent component directories, discoverable named concrete
classes, separate abstract interfaces and registration, verification and commit.
Root owns the exclusive native build/test and sequential Viewer slots for this
package. No other write/build owner is invoked.

## Structure and implementation order

1. Give each abstract role its own class-named header under
   `src/simulation/mission_systems/components/interfaces/`. Retain the existing
   namespaces and exact virtual declarations, including FuzeAdapter and
   WarheadAdapter. Existing effect factory headers remain forwarding entry
   points with their unchanged factory declarations.
2. Put each of the eleven concrete classes in a matching `.h`/`.cpp` pair under
   `components/implementations/<role>/`. Declarations are named types in their
   existing namespaces; no concrete model class remains anonymous. Preserve
   numerical expressions, ownership and native backend delegation.
3. Move abstract shared helpers into `components/support/`. Separate context,
   typed factory table, registry and native command composition infrastructure.
   Retain `Providers.h` as a compatibility include. Put built-in registration,
   legacy construction helpers and validated effect construction dispatch into
   `components/registration/`; model files contain no registration code.
4. Update production includes and CMake sources, add compile-time checks for
   discoverable concrete classes and abstract shared helpers, and document the
   actual tree and extension locations. Keep the old physical assertions intact.
5. Run both native builds, focused component gates, installed equipment consumer
   and the unchanged scoped physical selections. Repeat both fixed 5 km scenes
   fullscreen with native recording; compare the showcase execution and target
   response with the preceding provider delivery. Record failures accurately.
   Review and commit only this structural package.

## Exact write boundaries

* `src/simulation/mission_systems/components/`: remove `Providers.cpp` and the
  old `providers/` files; retain `Providers.h` as an umbrella. Add
  `ProviderContextV1.h`, `ProviderTableV1.h`, `ProviderRegistryV1.h/.cpp`,
  `PlantProvidersV1.h/.cpp` and a short source-navigation `README.md`.
* `components/interfaces/`: `SeekerProviderV1.h`, `GuidanceProviderV1.h`,
  `NativeBindingProviderV1.h`, `AutopilotProviderV1.h`, `AllocatorProviderV1.h`,
  `ActuatorProviderV1.h`, `PropulsionProviderV1.h`, `AerodynamicsProviderV1.h`,
  `FuzeAdapter.h`, `WarheadAdapter.h`.
* `components/implementations/`: class-named header/source pairs for
  `seeker/EncounterSeekerV1`, `guidance/VelocityNormalPnGuidanceV1`,
  `autopilot/LoadFeedbackCascadeAutopilotV1`,
  `allocator/JsbsimAuthoredFinMixerV1`, `actuator/JsbsimLagRateStopsActuatorV1`,
  `propulsion/JsbsimSolidRocketPropulsionV1`,
  `aerodynamics/JsbsimGeneratedStoreAerodynamicsV1`,
  `fuze/SweptPathFuzeV1`, `fuze/TurningCpaFuzeV1`,
  `warhead/ZonedCaseWarheadV1`, `warhead/SideCaseWarheadV1`.
* `components/support/`: `NativePlantBindingV1.h`,
  `CapturedFuzeLifecycleV1.h`, `WarheadSnapshotV1.h`.
* `components/registration/`: `BuiltInProvidersV1.h/.cpp`,
  `EffectFactoriesV1.h/.cpp` and one `<Role>ProvidersV1.cpp` registration unit
  for each of Seeker, Guidance, Autopilot, Allocator, Actuator, Propulsion,
  Aerodynamics, Fuze and Warhead.
* `src/simulation/mission_systems/weapon/FuzeAdapter.h/.cpp`,
  `damage/WarheadAdapter.h/.cpp`: retained interface/factory includes; remove
  the old effect implementation/helper files under `weapon/adapters/` and
  `damage/adapters/` only after creating their reviewed canonical replacements.
* `src/simulation/mission_systems/CMakeLists.txt`,
  `src/simulation/runtime/world/World.cpp`, `WorldEngagements.cpp`:
  source/include paths only.
* `tests/simulation/test_payload_providers.cpp`: compile-time concrete/interface
  contracts; original runtime and physical assertions unchanged.
* `tests/CMakeLists.txt`, `fsim_payload_profile_test`: configured-graph
  verification exposed the pre-existing direct compilation of Profile.cpp and
  Bindings.cpp outside their production module. Link the production `fsim::ms`
  target instead; retain the architecture rule and all fixture assertions.
* `docs/sdk/payload-native-providers.md`, `docs/status.md`, this plan:
  organization, extension instructions and current evidence.

No public `include/` file, model card, generated native model, engine/nozzle,
Hangar logic or physical algorithm is changed. Missing equipment models and
the original fragment/component-hit failures remain outside this structural
delivery and retain their pending status. No external plugin ABI is introduced.

## Organization and verification evidence

All eleven concrete models now have class-named headers and out-of-line method
definitions in the matching source files. Nine role contracts and the common
native binding contract have separate headers. Abstract shared helpers no
longer depend on registration. Factories, catalogue validation and native
command composition have their own files. The old three implementation
directories are removed after their files have canonical replacements.

The move proof compares all 25 outlined method bodies with the baseline
(whitespace-insensitive) and all ten abstract class declarations exactly.
`payload-layout-move-proof.json` records the class/source index and comparisons
under `build/architecture-reviews/`. The final code freeze contains 64 current
input fingerprints and 22 removed predecessor paths. Cards, generated native
models, Hangar logic and public headers are unchanged.

Both complete native build targets pass after the final CMake repair:
`payload-layout-{release,headless}-final-build.log`. Initial Release focused
verification was 9/10 because the existing profile test directly compiled
production Profile.cpp and Bindings.cpp outside their module. The baseline
test definition and the configured-graph diagnostic demonstrate that cause.
The test now links `fsim::ms`; no architecture rule or assertion is exempted.
Final focused Release verification passes 10/10 (9.89 s), including the seven
component/guidance programs and three architecture/interface gates. Focused
Headless passes 7/7 (3.20 s). The installed equipment consumer passes 1/1
(34.17 s). Logs and JUnit are `payload-layout-release-focused-final.{log,xml}`,
`payload-layout-headless-focused-final.{log,xml}` and
`payload-layout-installed.{log,xml}` in the same evidence directory.

Current scoped physical selections discover the same 120 Release and 119
Headless entries against fresh inventories of 891 and 817 total entries.
Release passes 118/120 in 75.14 s; Headless passes 118/119 in 75.21 s, both with
zero skips. The original showcase fragment Hit and 6 m side-burst component-hit
requirements remain failed; the latter is the Headless failure. No fixture is
disabled or weakened. Logs/JUnit are
`payload-layout-{release,headless}-physical.{log,xml}`. This is scoped regression,
not complete-suite or complete SIM-M03 physical acceptance.

Both fixed scenes then ran sequentially with fullscreen Viewers on the final
binaries. Native producer exit codes are zero. Current recordings are
`sim-m03-five-km-viewer/LAYOUT-functional-native-engagement-1-5000.000000.fsrec`
and `LAYOUT-showcase.fsrec`, each byte-identical to the corresponding preceding
`NATIVE-PROVIDERS` recording. The native functional stdout is byte-identical;
the entire showcase `.fsrec.engagement.json`, including all engagement events
and target-response samples, is exactly equal. The comparison is recorded in
`payload-layout-visual-comparison.json`.

Inspected 3840 by 2160 screenshots `layout-functional_005.png` (21.8 s) and
`layout-showcase_003.png` (30.4 s) show the actual separated nose and subsequent
native upset. At the showcase's final 40 s sample, altitude is 4807.9217 m and
speed is 70.5778 m/s, with `crashed=false` and `diverged=false`. Mechanical
contact aftermath is not reported as fragmentation acceptance or a crash.
All 64 frozen input fingerprints still match after every gate and visual run.
