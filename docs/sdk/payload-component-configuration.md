# Payload component configuration

The SDK resolves one versioned component profile for each loaded store card.
The maintained catalogue has 53 cards and nine declared roles per card: seeker,
guidance, autopilot, allocator, actuator, fuze, warhead, propulsion and
aerodynamics. Component availability and physical-model acceptance are separate.
The current implementation supplies [typed native provider registration and
runtime instances](payload-native-providers.md). Each role has its own abstract
execution interface and independently registered concrete models. Registration
is a native source extension compiled with the SDK; an installed external-DLL
loading ABI is not provided.

The availability declarations describe what actually executes today. All 53
cards select aerodynamics; the 24 existing guided stores select all nine
providers. The remaining 29 declare seeker/guidance/control/propulsion
not-applicable. Twenty-seven also declare fuze/warhead not-applicable; the
MK 82 and CJ-10A declare those two roles unavailable. Their ballistic native
models do not establish armed effect support. A complete declaration is not
a claim that every role on every store has a physical implementation.

All 24 maintained guided cards now select explicit autopilot configuration with
five parameter references. The other 23 cards transcribe their existing generic
controller defaults; no existing equipment parameter changes. Swept-path fuze
and zoned-case effect models use descriptive IDs; their former legacy IDs remain
compatibility aliases. The underlying physical models and their assumptions are
unchanged. A current interface version does not establish a newer physical law.

## Card contract

`component_models` is a sidecar to the unchanged StoreCard layout:

```json
{
  "component_models": {
    "schema_version": 1,
    "components": {
      "seeker": {
        "availability": "available",
        "implementation_id": "fsim.seeker.encounter.v1",
        "configuration_version": 1,
        "parameter_refs": {"acquire_m": "component_seeker_acquire_m"},
        "bindings": {"mode": "active_radar"},
        "basis": "assumed",
        "note": "Existing simulator approximation, without equipment calibration.",
        "validity": "Reduced-order game simulation."
      }
    }
  }
}
```

This shortened example illustrates the syntax; a complete profile must declare
all nine roles and every required provider slot. The seeker provider requires
29 roles, not just the single reference shown above. The role-specific contracts
are [seeker/guidance](payload-seeker-guidance-adapters.md),
[fuze/warhead](payload-fuze-warhead-adapters.md) and
[native plant](payload-plant-adapters.md).

Available components require a nonempty implementation identity, configuration
version 1, explicit parameter-reference and string-binding objects, a recognized
basis, a limitation note and a validity statement. Published/derived provenance
requires a source the card lists. Parameter references resolve to copies of the
same card's value, exact unit, uncertainty, basis, source and note. Provider
validation additionally checks its complete slots, units, domain and supported
reference models. Unknown IDs/versions, missing or extra slots and invalid
explicit configurations refuse execution. They never select legacy defaults.

`unavailable` and `not_applicable` declarations contain only `availability` and a
nonempty `reason`. Unavailable means a required component/model has not been
supplied; not-applicable means the represented store has no executing function
of that kind. Inert body flight cannot establish armed equipment capability.
An absent entire profile is separately identified as legacy compatibility.

## Composition and authority

The selected seeker mode must agree with the card's authored mode. Authored
guidance capability booleans must agree with their resolved roles. Guidance and
fuze lifetime values must agree; a fuze cannot silently shorten the guidance
lifetime. A V1 fuze's envelope must use the card's canonical body dimensions.
The SDK validates the complete composition before publishing a selected Spec.

Each actual guided release owns its guidance, immutable profile and selected
fuze/warhead adapters. The fuze owns its turn history, consumed contact channel
and one-shot delayed-initiation state. Vehicle.reset clears transient captured
history without creating a new release or rearming initiation. Existing native
RNG ownership and draw sequences are retained; configuration parsing adds no
draws. The current seeker/guidance processing draws no RNG.

World retains observation permissions, scene enumeration, RF/terrain queries,
captured motion/material-point construction, candidate arbitration, mechanical
contact response, native damage and events. JSBSim integrates the selected
native actuator, propulsion and aerodynamic assets. Control adapters supply
normalized commands; no adapter adds an ideal force/moment integration path.

Execution dependencies follow the actual phase:

| Release phase | Required native components |
| --- | --- |
| Armed guided body | Autopilot, allocator, actuator, applicable propulsion, aerodynamics; selected seeker, guidance, fuze and warhead |
| Non-inert unguided body | Applicable propulsion and aerodynamics |
| Inert jettison | Aerodynamics; armed components remain disabled |

Selected propulsion also checks the actual native engine inventory: the current
solid-rocket provider has one engine; a declared absent propulsion has zero.
An inert jettison may retain a disabled motor in its native model. Full card
availability remains intact when a phase does not select a component.

Native metadata is checked immediately after model load/reset and before pool
registration, ID allocation, network/host binding or publication. Failure keeps
the store aboard, returns an authoritative failed activity and creates no
weapon/removal history. A reused inactive pool slot remains free on refusal.
These checks compare generated configuration metadata, not arbitrary edited
XML bytes; generation and asset-equivalence checks are separate evidence.

## Source and generated assets

A fit's store profile comes from the exact store library used to read that fit.
World retains a binding for the actual loaded immutable card object, including
failed bindings. A second same-ID card cannot recover another source's cached
profile. Origin, card or component-only conflicts remain refused, and the
accepted earlier binding stays intact. No lookup reopens an independently
selected library by ID during release.

Numeric propulsion inputs now resolve from card parameters through the same
Hangar resolver used by design generation, native model generation and stage
verification. The migration transcribes the already consumed TOML values,
including its rounding, without changing existing numerical inputs. Existing
unquantified assumptions remain marked assumed; zero stored uncertainty does
not establish measurement accuracy.

Regenerate native store assets from the repository root with:

```powershell
$env:PYTHONPATH = 'tools/models/hangar'
python -m hangar.storemodels
```

Generated XML is not a second maintenance authority. The persistent catalogue
comparison checks all 53 stores: engine/nozzle bytes remain identical and
initial physical XML values/functions remain equivalent after resolving actual
actuator property references. Only component identity and actuator-version
metadata are excluded from that comparison. This does not validate equipment
performance, trajectories or destructive effects.

## Acceptance

Configuration tests, adapter tests, native binding/refusal tests and generator
equivalence are software evidence. The original 5 km showcase still produces
0/1688 fragment intersections in the recorded full-screen run; its visible nose
loss is mechanical contact. The other fixed fragment/component requirements
remain active. No model calibration, test criterion or Hit meaning is changed
to manufacture acceptance. See [the physical investigation](sim-m03-five-km-repair.md)
for the distinct SIM-M03 gates and their current evidence.

Current software evidence is seven focused Release CTest programs passing,
14 focused Hangar tests passing, and the installed equipment consumer passing
(1/1, 34.28 s). The typed-provider inventory and extension boundary are recorded
in [native payload providers](payload-native-providers.md).
Native tests load and check all 53 generated store models (24 controlled and
29 ballistic). The separate scoped physical selections retain their failures:
Release 118/120 and Headless 118/119, with zero skips. The two complete native
build targets pass. Full-program compatibility and calibrated per-variant
physical acceptance are not inferred from these results.
