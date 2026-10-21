# Native payload provider interfaces and concrete models

Component role, abstract execution interface, concrete implementation and
configured instance are distinct. A card chooses a role's implementation ID
and parameters. Its factory creates a new instance for the actual release.
Different cards may share an implementation when they differ only in supported
parameters. A different algorithm or backend selects another concrete class;
neither a role name nor a renamed compatibility ID establishes such a change.

## Interface and implementation inventory

| Role | Abstract runtime interface | Current concrete model classes |
| --- | --- | --- |
| Seeker | [SeekerProviderV1](../../src/simulation/mission_systems/components/interfaces/SeekerProviderV1.h) | [EncounterSeekerV1](../../src/simulation/mission_systems/components/implementations/seeker/EncounterSeekerV1.h) |
| Guidance | [GuidanceProviderV1](../../src/simulation/mission_systems/components/interfaces/GuidanceProviderV1.h) | [VelocityNormalPnGuidanceV1](../../src/simulation/mission_systems/components/implementations/guidance/VelocityNormalPnGuidanceV1.h) |
| Autopilot | [AutopilotProviderV1](../../src/simulation/mission_systems/components/interfaces/AutopilotProviderV1.h) | [LoadFeedbackCascadeAutopilotV1](../../src/simulation/mission_systems/components/implementations/autopilot/LoadFeedbackCascadeAutopilotV1.h) |
| Allocator | [AllocatorProviderV1](../../src/simulation/mission_systems/components/interfaces/AllocatorProviderV1.h) | [JsbsimAuthoredFinMixerV1](../../src/simulation/mission_systems/components/implementations/allocator/JsbsimAuthoredFinMixerV1.h) |
| Actuator | [ActuatorProviderV1](../../src/simulation/mission_systems/components/interfaces/ActuatorProviderV1.h) | [JsbsimLagRateStopsActuatorV1](../../src/simulation/mission_systems/components/implementations/actuator/JsbsimLagRateStopsActuatorV1.h) |
| Propulsion | [PropulsionProviderV1](../../src/simulation/mission_systems/components/interfaces/PropulsionProviderV1.h) | [JsbsimSolidRocketPropulsionV1](../../src/simulation/mission_systems/components/implementations/propulsion/JsbsimSolidRocketPropulsionV1.h) |
| Aerodynamics | [AerodynamicsProviderV1](../../src/simulation/mission_systems/components/interfaces/AerodynamicsProviderV1.h) | [JsbsimGeneratedStoreAerodynamicsV1](../../src/simulation/mission_systems/components/implementations/aerodynamics/JsbsimGeneratedStoreAerodynamicsV1.h) |
| Fuze | [weapon::FuzeAdapter](../../src/simulation/mission_systems/components/interfaces/FuzeAdapter.h) | [SweptPathFuzeV1](../../src/simulation/mission_systems/components/implementations/fuze/SweptPathFuzeV1.h), [TurningCpaFuzeV1](../../src/simulation/mission_systems/components/implementations/fuze/TurningCpaFuzeV1.h) |
| Warhead | [damage::WarheadAdapter](../../src/simulation/mission_systems/components/interfaces/WarheadAdapter.h) | [ZonedCaseWarheadV1](../../src/simulation/mission_systems/components/implementations/warhead/ZonedCaseWarheadV1.h), [SideCaseWarheadV1](../../src/simulation/mission_systems/components/implementations/warhead/SideCaseWarheadV1.h) |

`NativeBindingProviderV1` is a common abstract asset-binding contract, not a
plant implementation. `NativePlantBindingV1<Interface>` shares immutable model
identity and metadata checks but remains abstract through its role interface.
`CapturedFuzeLifecycleV1` shares captured-contact/delay lifecycle and remains
abstract until a model supplies proximity evaluation. `WarheadSnapshotV1`
shares immutable model ownership and remains abstract until assessment is
implemented. None is registered as an executable component.

`ProfileV1`/`DescriptorV1` describe component selections and resolved inputs;
they are not runtime implementations. `ProviderRegistryV1` and
`ProviderTableV1<Interface>` are concrete factory infrastructure, not component
base classes or physical models. A runtime instance is the factory's concrete
object, owned through its role's abstract interface.

## Source organization

All nine roles use the same layout under
`src/simulation/mission_systems/components/`:

```text
components/
  interfaces/                 one abstract type per class-named header
  implementations/
    seeker/EncounterSeekerV1.h + .cpp
    guidance/VelocityNormalPnGuidanceV1.h + .cpp
    autopilot/LoadFeedbackCascadeAutopilotV1.h + .cpp
    allocator/JsbsimAuthoredFinMixerV1.h + .cpp
    actuator/JsbsimLagRateStopsActuatorV1.h + .cpp
    propulsion/JsbsimSolidRocketPropulsionV1.h + .cpp
    aerodynamics/JsbsimGeneratedStoreAerodynamicsV1.h + .cpp
    fuze/SweptPathFuzeV1.h + .cpp
    fuze/TurningCpaFuzeV1.h + .cpp
    warhead/ZonedCaseWarheadV1.h + .cpp
    warhead/SideCaseWarheadV1.h + .cpp
  support/                    abstract shared binding/lifecycle helpers
  registration/               role registrations and construction dispatch
  ProviderContextV1.h         borrowed factory input
  ProviderTableV1.h           typed factory table
  ProviderRegistryV1.h + .cpp  catalogue validation and instance construction
  PlantProvidersV1.h + .cpp    native command-stage composition
  Providers.h                 compatibility umbrella include
```

Each concrete header declares one named class, and its matching source defines
that class's methods. Concrete models are no longer declared in anonymous
namespaces or hidden in role-named source files. The abstract interfaces have
their own matching headers under `interfaces/`; original class namespaces and
virtual declarations are retained. Existing `weapon/FuzeAdapter.h` and
`damage/WarheadAdapter.h` include the canonical interface and retain their
factory declarations. There is only one class definition for each interface.

`registration/<Role>ProvidersV1.cpp` contains the role's built-in factory
registration. `BuiltInProvidersV1.cpp` assembles the catalogue;
`EffectFactoriesV1.cpp` dispatches validated effect construction. Model files
contain no registration code. Existing descriptor validation remains in the
weapon/damage adapter factory files. Shared helpers include their contracts
without depending on registration infrastructure.

The current encounter seeker is one simplified model with four modes, not four
independently validated equipment implementations. The two warhead wrappers
retain their existing versioned numerical models and shared native assessor.
The descriptive swept-path and zoned-case IDs identify the existing models;
they do not claim upgraded physical fidelity.

## Registration and selection

Every role has a separate `ProviderTableV1<Interface>` member. For example, an
allocator factory returns `unique_ptr<AllocatorProviderV1>`; an autopilot
factory cannot be registered there by returning its unrelated interface.
Registration needs an implementation ID, a compatible configuration schema ID,
a factory, and an explicit replacement flag. Duplicate registration without
replacement fails. Unknown identities, wrong roles/versions, missing instances
and throwing factories fail with a reason; there is no fallback provider.

V1 registrations declare an existing supported configuration schema. A new
model requiring another parameter contract must introduce a versioned schema;
registering its name cannot bypass slot, unit, provenance or domain validation.
Generated native metadata uses the card's selected implementation and resolved
values, not its validation schema alias. Alternative native assets must match
that selected configuration.

`session::World::setPayloadProvidersV1` copies the catalogue before the first
vehicle or store is loaded. Reconfiguring a World after that point fails. Later
edits to the caller's catalogue cannot change the World's factory membership.
Factories must own their captures and support independent Worlds; mutable
simulation state and RNG remain instance/World-owned. This is a native source
extension compiled with the SDK, not an installed external-DLL plugin ABI.
The existing public SDK headers, layouts and plugin vtables are unchanged.
The build context is borrowed for the duration of factory invocation. Runtime
instances must copy their configuration and must not retain context references.

To add another model, declare its class in
`implementations/<role>/<ClassName>.h`, define its methods in the matching
`.cpp`, and register its factory in `registration/<Role>ProvidersV1.cpp`.
Add its source explicitly to the Mission Systems CMake target. Select that
implementation ID and its supported parameter bindings in the model card;
regenerate matching native metadata through Hangar when required. Add fixtures
for that model's behavior and refusal paths. A new name with the same factory
is an alias, not another algorithm. A parameter-only equipment variant may
share a concrete class; a different algorithm needs its own class. Public
binary plugin loading and parameter/output contracts beyond the current V1
schemas require separate additive interfaces.

## Production execution and responsibility

Seeker helpers and guidance updates invoke the selected runtime objects. The
autopilot produces normalized virtual channels; the allocator adapter admits
those channels to the authored JSBSim mixer. The actuator adapter validates
actual native lag/rate metadata. Its dynamic response and physical fin motion
are integrated by JSBSim, not duplicated in the adapter. The solid-rocket
adapter validates actual engine inventory and interprets native burning state.
The aerodynamic adapter binds the selected generated coefficient source.

Propulsion/aerodynamic integration and authored mixing remain in JSBSim. These
concrete classes are backend bridges, not replacement force integrators. They
cannot be described as independent physical solvers. A backend requiring a
different actuation/output contract needs an additive versioned interface.

Required runtime objects and native bindings are admitted before publication,
inventory consumption or identity allocation. A null or failed selected factory
preserves the store and returns the native execution refusal. Inert phases
instantiate only their required backend roles. Scripted referee assessment also
selects the registered warhead provider. None of these extensions expands an
Agent's sensor observations or access to referee truth.

## Verification boundary

The focused fixtures cover typed catalogue coverage, explicit replacement,
retained catalogue snapshots, invalid selection and creation failures. Native
fixtures exercise actual selected instances in World execution and check that
a refusing factory preserves inventory, event history and the next entity ID.
The independent referee dispatch fixture checks provider execution only; it
does not replace either original fixed-scene damage requirement.

Physical-model acceptance remains separate. Existing unsupported equipment
components and original fragmentation/component-hit failures remain pending.
Current build, test, catalogue migration and fullscreen evidence are recorded
in the delivery plan after the final source freeze.

Provider-registration evidence after baseline `bf16f80`: both complete native build targets
pass; focused Release CTest is 7/7, focused Hangar is 14/14 and the installed
equipment consumer is 1/1. Scoped physical regression remains 118/120 Release
and 118/119 Headless, zero skips. Both original failed assertions remain active.
Fullscreen reruns show actual mechanical nose separation and subsequent native
upset in both fixed 5 km scenes; they do not demonstrate fragmentation-model
acceptance or a crash. The [physical investigation](sim-m03-five-km-repair.md)
records the exact current logs, recordings and screenshots.

## Source-organization verification after ab7f89d

The subsequent organization package moves declarations and method definitions
without changing any abstract virtual declaration or model algorithm. All 25
outlined method bodies match the baseline after whitespace normalization; all
ten abstract class definitions match exactly. Concrete headers compile as named
final implementations of their intended abstract roles, and shared helpers
remain abstract. All 64 final build/test inputs are frozen for verification.

Both complete native builds pass. Focused Release checks pass 10/10, including
the three architecture/interface gates; focused Headless passes 7/7, and the
installed equipment consumer passes 1/1. The profile test now links the actual
production module, repairing a discovered configured-graph violation without
adding a checker exception. Scoped physical selections retain the original
failures: 118/120 Release and 118/119 Headless, zero skips. No model card,
generated native model, numerical input or physical acceptance assertion is
changed. The [organization delivery record](../superpowers/plans/2026-10-20-payload-provider-layout.md)
records the exact logs and visual comparison separately from older evidence.

Both fixed 5 km scenes were rerun fullscreen after organization. The complete
native recordings are byte-identical to their preceding provider-delivery
counterparts, and the showcase engagement document and all target-response
samples are exactly equal. This confirms retained behavior for these scenes;
it does not close their original physical-model failures. Comparison evidence
is `build/architecture-reviews/payload-layout-visual-comparison.json`.
