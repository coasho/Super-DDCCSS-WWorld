# Uniform payload component execution

Main HEAD: `92ae5ed16295b001592db30eae0928a6dcc00e10`. The user authorizes
development of all payload component seams. Existing uncommitted SIM-M03 work
and its failing physical gates are retained. This package changes composition,
configuration ownership and adapter dispatch; it does not calibrate equipment
or change acceptance thresholds.

## Contract and write windows

Root owns `components/Profile.h/.cpp`, `Cards.cpp`, runtime World files,
mission-system and test CMake, `test_payload_profile.cpp`, all store card JSON,
generated aircraft assets and this package's integration/status documentation.
Root also owns the additive private `weapon/LaunchEligibility.h/.cpp` overload
so the selected seeker configuration supplies the release field of view.
The integration window adds private checked World creation/release overloads,
`Host.h`/`HostStores.cpp` execution refusal for inert releases, and
`test_payload_composition_native.cpp`. Native binding is checked before entity
registration/publication, including reused inactive slots. Store sidecars retain
the exact library chosen by each fit. Cross-component consistency belongs in
`weapon/Composition.h/.cpp` and `test_payload_composition_contract.cpp`.
`components/Bindings.h/.cpp` owns durable per-loaded-card origin/snapshot
bindings and refusals, covered by `test_payload_profile.cpp`. Unguided powered
release has a separate native propulsion/aerodynamics dependency mask; inert
jettison does not enable armed providers. These seams retain full card
availability and exact existing physics values.
Existing `StoreCard` and public SDK layouts, signatures and vtables are frozen.
An immutable profile is an explicit sidecar, never a global mutable registry.

Disjoint component windows, with no native execution by collaborators:

- Seeker/guidance: `weapon/Seeker.h/.cpp`, `weapon/Guidance.h/.cpp`,
  `test_weapon_guidance.cpp`, `payload-seeker-guidance-adapters.md`.
- Fuze/warhead: new `weapon/FuzeAdapter.h/.cpp`, `damage/WarheadAdapter.h/.cpp`,
  `test_payload_effect_adapters.cpp`, `payload-fuze-warhead-adapters.md`.
- Native plant/Hangar: new `weapon/PlantAdapter.h/.cpp`, Hangar `components.py`,
  `missiles.py`, `storebodies.py`, `storecontrol.py`, `storestages.py`,
  `storemodels.py`, `test_components.py`, `test_payload_plant_adapters.cpp`,
  `payload-plant-adapters.md`. Preserve earlier generator fixes.

## Version 1 composition

`component_models.schema_version=1` requires all nine roles: seeker, guidance,
autopilot, allocator, actuator, fuze, warhead, propulsion, aerodynamics.
Every role declares availability. Available roles identify an implementation,
configuration version, scalar parameter references and string bindings, basis,
validity and a limitation/provenance note. Unavailable/not-applicable roles
declare a reason and contain no executable configuration. Parameter values are
resolved and copied from the same card with units and provenance intact.
Explicit unknown/malformed configurations refuse; absence is separately
identified as legacy compatibility. Implementations validate their own slots,
units, domain and supported references before release mutation.

SDK adapters execute the current algorithms. JSBSim remains the sole flight
integrator. Existing legacy/V1 distinctions, arithmetic, RNG consumption,
one-shot channels and Vehicle.reset semantics remain intact. Model constants
and equipment assumptions retain their current numbers and explicit limits.
No external numerical delivery is consumed.

## Sequence and evidence

1. Add configuration/refusal/isolation regressions, then the strict sidecar
   parser and adapter dispatch. Root records RED/GREEN when holding the serial
   slot; source order alone is not runtime RED evidence.
2. Migrate applicable cards uniformly, preserving existing values and selecting
   existing legacy or V1 implementations explicitly. Generate native assets
   through Hangar. Check identities and unchanged numerical outputs.
3. Freeze all native build inputs. Root alone runs bounded Release/Headless
   component and existing regression gates with actual counts and no skips.
4. Run original encounters with fullscreen Viewer and inspect actual damage
   aftermath. A Hit, component descriptor or passing software gate does not
   close the existing physical-model failures.
5. Inspect owned staged diff and commit only verified delivery. Report pending
   physical, installed-consumer and full-program gates separately.

Recorded baseline before this phase: scoped Release 112/114, Headless 112/113;
showcase fragment Hit and side-burst component Hit remain failing. Historical
input fingerprints remain historical; new inputs require new evidence.

## Delivery evidence

All 53 maintained cards declare all nine roles (477 declarations). Actual
availability is preserved: all have native aerodynamic assets; 24 guided
stores select the nine providers; two other stores explicitly lack armed
fuze/warhead providers. Typed providers are linked into the SDK; a dynamic
provider registry or external plugin ABI is outside this delivery.

Both complete UCRT64 builds pass. Focused Release component gates pass six
CTest programs, and the installed equipment consumer passes 1/1 in 37.03 s.
Scoped physical selections pass 117/119 Release and 117/118 Headless with zero
skips, retaining the original showcase fragment Hit and 6 m component-hit
failures. Final focused Hangar checks pass 13/13, including all-53 native
generation equivalence. Existing C172/Skua generation drift remains recorded;
no complete current full-program or physical-model acceptance is claimed.

Final diagnostic source window: only Hangar `components.py`, `storestages.py`,
`test_components.py` and `payload-plant-adapters.md`. The original tank/pod stage
regression reproduced 25 errors; focused tank/pod/cargo RED precedes the fix.
Strict report dispatch retains original diagnostic numbers and adds no native
force model or generated-resource change. Native build inputs remain frozen.
Full-screen current-binary visual runs and reviewed feature-owned staging
complete the software handoff; the physical failures remain open separately.
Both unchanged scenes have now run live/fullscreen at 3840 by 2160. Actual
native nose loss, crew loss and subsequent upset agree with the saved native
records. Neither run claims a crash or closes the fragmentation gates.
`payload-components-final-inputs.json` records this delivery's exact owned
inputs, native executables and evidence hashes independently of historical
fingerprint manifests.
