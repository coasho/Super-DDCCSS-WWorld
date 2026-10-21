# Payload component configuration and implementation audit

The initial inventory below records the pre-composition snapshot. The subsequent
[SDK implementation](payload-component-configuration.md) now supplies nine-role
profiles across all 53 maintained cards, compiled native adapter selection,
per-release effect state, phase-specific native asset checks and Hangar input
ownership. This does not close equipment-fidelity or the remaining SIM-M03
physical gates. The later [native provider delivery](payload-native-providers.md)
adds role-specific abstract interfaces, concrete model classes, typed factories
and per-World registration snapshots. An installed external-DLL ABI remains
absent. The inventory below is the earlier snapshot, not a current absence claim.

Date: 2026-10-20. Main HEAD: `92ae5ed`. This audit includes the uncommitted
SIM-M03 repair; it is not an accepted implementation or equipment-fidelity claim.

## Initial inventory

The repository does not yet provide complete per-component configuration with
replaceable native adapters/plugins for all payload components. Parameters,
descriptive equipment facts, generated assets, native implementation selection
and validated physical behavior are different capabilities.

Read-only inspection of the 24 current AAM cards finds four seeker family
strings: active radar, semi-active radar, infrared and imaging infrared. Only
`aim-120c` declares the current numeric `weapon_execution_version`,
`fuze_model_version` and `warhead_model_version` profiles. It is also the only
current AAM card with explicit `seeker_acquire_m` and `seeker_gimbal_deg`.
No AAM card declares `seeker_model_version` or a structured seeker, warhead,
propulsion or motor implementation descriptor. These are inventory facts, not
proof that other cards contain no useful published parameters.

| Component | Existing source-to-runtime route | Remaining gap |
| --- | --- | --- |
| Seeker | `StoreCard.seeker`, optional acquisition/gimbal parameters, World observation filtering, shared `Guidance` observation association and header-only seeker helpers | Family-string branches, shared embedded assumptions and no independently registered seeker implementation/configuration contract. Variant-specific sensor dynamics and validity remain unaccepted. |
| Guidance | Optional versioned numeric execution profile parsed by `configureGuidance`, frozen into the released weapon's private `Spec` | Supported numeric algorithm ID selects the built-in implementation; there is no external component factory. Explicit current profile exists on one card. |
| Autopilot/allocator/actuators | Versioned control parameters; Hangar generates the native fin plant; runtime checks signed stops and actuator metadata | One implemented algorithm/allocator selection, not a replaceable component provider. Other AAM cards retain legacy execution. |
| Fuze | Optional numeric V1 law/station/arming/delay profile plus built-in captured-history/contact logic | No registered provider interface for independently supplied detector/law implementations. Remaining equipment assumptions and fixed-scene gates are not accepted. |
| Warhead/effects | Card facts and numeric parameters feed `warheadOf` and built-in `assess`; V1 has explicit source/configuration validation | Model-version/source branches do not constitute a replaceable warhead provider. Fragment geometry, penetration, blast and structural vulnerability remain separate fidelity concerns. |
| Propulsion | Card facts plus Hangar missile tables generate design TOML; validated motor declarations generate JSBSim engine/nozzle assets; JSBSim executes native propulsion | Important inputs remain in generator tables/designs rather than a complete card component descriptor. The existing backend bridge is a flight-model adapter, not a per-propulsion plugin selected by the card. |

## Evidence in maintained code

* [`Cards.h`](../../src/simulation/mission_systems/Cards.h) and
  [`Cards.cpp`](../../src/simulation/mission_systems/Cards.cpp) provide a flat
  `parameters` map and selected typed store fields. The top-level card `motor`,
  `guidance` and `fuze` prose is not parsed into independent runtime component
  configurations by `readStoreCard`. Adding prose to JSON does not configure a
  runtime model.
* [`World.cpp`](../../src/simulation/runtime/world/World.cpp) constructs the
  built-in guidance directly and supplies family-dependent fallback values.
  Current vehicle creation accepts only the JSBSim family in this build.
* [`WorldEngagements.cpp`](../../src/simulation/runtime/world/WorldEngagements.cpp),
  [`Guidance.cpp`](../../src/simulation/mission_systems/weapon/Guidance.cpp) and
  [`Seeker.h`](../../src/simulation/mission_systems/weapon/Seeker.h) still contain
  shared assumptions for sensor eligibility, aspect response, observation
  association and decoy response. A different seeker string is not a complete
  different modeled seeker.
* [`Damage.h`](../../src/simulation/mission_systems/damage/Damage.h) exposes the
  internal built-in warhead values and `warheadOf`/`assess` functions; it has no
  registered component-provider seam.
* [`missiles.py`](../../tools/models/hangar/hangar/missiles.py),
  [`motorinputs.py`](../../tools/models/hangar/hangar/motorinputs.py) and
  [`storebodies.py`](../../tools/models/hangar/hangar/storebodies.py) own the
  current motor-table/design validation and native engine generation route.
  Generated TOML/XML must not become a second hand-maintained authority.
* [`Registry.h`](../../src/foundation/core/Registry.h) is a reusable internal
  factory utility. Its existence does not register seeker/warhead/propulsion
  implementations. Public [`VecEnvPlugins.h`](../../include/fsim/VecEnvPlugins.h)
  registers training tasks, observation builders and action mappers; those
  extension points do not provide payload-component plugins.

## Required composition contract

The user's requirement covers seeker together with the other payload components.
The proposed route is:

```text
payload card and referenced component data
  -> validated, versioned component descriptions
  -> explicitly registered native implementation/adapters
  -> independent instances owned by the released payload
  -> native state, reports and recorded outcomes
```

This is a required design direction, not a currently loadable schema. A component
description needs an implementation identity, configuration version, typed/unit
checked inputs or referenced datasets, source/basis/uncertainty, validity domain
and explicit availability. Description text, presence of a version number or a
generic family name cannot stand in for that binding.

Multiple variants can share a validated implementation with different sourced
configuration/data. Different algorithms or backends can be supplied through
the same versioned provider contract. A separate DLL for each equipment name
is unnecessary. An initial built-in registration and adapter route can satisfy
replaceability; dynamic loading is an additional lifecycle/ABI feature and
must not be described as delivered until implemented and verified.

Each created instance must own its mutable state and stable component RNG
stream. Reset, worker count, model lifetime and report provenance must preserve
the SDK's existing contracts. Unknown providers, unsupported versions and
incomplete explicit configurations must fail explicitly. Any retained legacy
fallback must be separately identified, not silently selected for an invalid
explicit description. A diagnostic can distinguish authored configuration from
the actual resolved implementation/version and generated-asset fingerprint.

Native sensor implementations form modeled observations within SDK authority;
guidance consumes admitted observations. Adapters/plugins must not move
simulation into a client, grant actor truth access or introduce a second flight
integrator. A propulsion backend adapter must preserve native integration,
mass/energy accounting and engine ownership. Existing public structures and
plugin vtables remain frozen; any new public provider surface must be additive
and versioned with its own acceptance.

## Completion criteria and pending scope

Completion requires a demonstrated chain from each declared card component to
the actual implementation and its independent evidence, including real
configuration consumption, refusal of invalid/unknown input, state/reset/RNG
isolation, native consequences, installed-consumer binding and recorded/live
presentation consistency where relevant. Module configuration coverage and
physical-model acceptance must be reported separately for each supported
variant. Existing failed hit/damage requirements remain active.

Public data must be attributed and admitted for the model's stated purpose.
Missing data remain unavailable or explicitly assumed; a reference to general
theory does not calibrate equipment-specific performance. This audit neither
fills those gaps with invented numbers nor marks the new provider contract
implemented. It does not alter the current frozen simulation inputs or test
results recorded in [the five-kilometre investigation](sim-m03-five-km-repair.md).
