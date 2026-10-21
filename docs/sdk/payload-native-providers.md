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
| Seeker | `SeekerProviderV1` | `EncounterSeekerV1` |
| Guidance | `GuidanceProviderV1` | `VelocityNormalPnGuidanceV1` |
| Autopilot | `AutopilotProviderV1` | `LoadFeedbackCascadeAutopilotV1` |
| Allocator | `AllocatorProviderV1` | `JsbsimAuthoredFinMixerV1` |
| Actuator | `ActuatorProviderV1` | `JsbsimLagRateStopsActuatorV1` |
| Propulsion | `PropulsionProviderV1` | `JsbsimSolidRocketPropulsionV1` |
| Aerodynamics | `AerodynamicsProviderV1` | `JsbsimGeneratedStoreAerodynamicsV1` |
| Fuze | `weapon::FuzeAdapter` | `SweptPathFuzeV1`, `TurningCpaFuzeV1` |
| Warhead | `damage::WarheadAdapter` | `ZonedCaseWarheadV1`, `SideCaseWarheadV1` |

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

The role interfaces and typed factory catalogue are in
`src/simulation/mission_systems/components/Providers.h`. Registrations and
concrete seeker, guidance and native backend classes are separated by role in
`components/providers/`. Concrete fuze and warhead models have their own
translation units under `weapon/adapters/` and `damage/adapters/`; descriptor
validation remains in the corresponding adapter factory files.

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

To add another model, implement the relevant abstract role in a new model
translation unit and register its factory in that role's table. Select that
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

Current evidence after baseline `bf16f80`: both complete native build targets
pass; focused Release CTest is 7/7, focused Hangar is 14/14 and the installed
equipment consumer is 1/1. Scoped physical regression remains 118/120 Release
and 118/119 Headless, zero skips. Both original failed assertions remain active.
Fullscreen reruns show actual mechanical nose separation and subsequent native
upset in both fixed 5 km scenes; they do not demonstrate fragmentation-model
acceptance or a crash. The [physical investigation](sim-m03-five-km-repair.md)
records the exact current logs, recordings and screenshots.
