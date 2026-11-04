# Equipment model identities

Equipment identity names the actual configured equipment and component function. An interface or solver revision is a separate implementation detail. The single maintained catalogue is `tools/models/hangar/hangar/data/equipment_models.json`; CMake generates the neutral native lookup from those same bytes, and the SDK/Python package installs the JSON unchanged.

For example, the retained store-card key `aim-120c` selects **AIM-120C-7**, with `equipment.aim-120c-7.seeker`, `equipment.aim-120c-7.guidance` and its other applicable roles. The retained key `aim-9x` selects **AIM-9X Block I**, with `equipment.aim-9x-block-i.seeker`. Existing source cards already bound these variants; this correction exposes those bindings rather than treating a receiver revision as a missile variant. Official references distinguish [AIM-120C-7](https://media.defense.gov/2024/Dec/11/2003606517/-1/-1/0/JORDAN_09-33.PDF) and [AIM-9X Block I](https://www.navy.mil/Resources/Fact-Files/Display-FactFiles/Article/2168989/aim-9x-sidewinder-missile/).

A label such as **AIM-9M seeker model** identifies the simulator instance for that equipment and function. It does not invent an OEM part number. `hardware_model_name` remains null where the admitted model sources do not establish the exact component designation. Each catalogue record preserves its source IDs and card path; the card retains all parameter-specific references, assumed/derived bases and uncertainty. Shared numerical mechanisms do not establish equal real equipment capability or calibration.

## Selection and compatibility

`src/contracts/equipment/EquipmentModels.h` is a generated, reviewed source artifact. The root CMake configuration verifies its exact content against the catalogue/template; it never rewrites a source file during a normal build. After an intentional catalogue edit, regenerate from the source root with `cmake -DFSIM_WRITE_EQUIPMENT_MODELS=ON -P cmake/EquipmentModels.cmake`, then review the catalogue and header together. The native compiler uses the existing narrow contract include view.

All 53 current store cards use equipment selectors for their 254 applicable component selections across nine roles. A nonapplicable role retains its explicit reason. The native provider table checks the selected equipment, component role, card identity, designation and native plant before construction. A selector for another weapon, an invented variant or another component fails explicitly.

The descriptor schema, virtual tables, public layouts, status semantics and retained store keys are unchanged. Old mechanism IDs remain supported. A named selector resolves to the existing mechanism schema before configuration; exact physical fingerprints and numerical coefficients remain unchanged. Existing replacement factories and independent provider-table copies retain their behavior. Legacy immediate-observation tests continue using their explicit retained detector profiles; their regenerated fixture bytes are identical after the naming migration.

Schema/interface identifiers such as `SeekerV1` and backend implementation revision suffixes describe software contracts or numerical mechanisms. They are never used as equipment model names. A future solver revision must keep the equipment identity and publish its own reviewed behavior/provenance change.

## Consumers

The `equipment.` provider namespace is reserved for catalogue entries. Registering an invented equipment variant, the wrong component role or a different mechanism schema fails before modifying a provider table. Custom mechanisms retain their own non-equipment provider identities.

```python
import fsim.catalog
model = fsim.catalog.equipment_model("aim-120c-7")
assert model["designation"] == "AIM-120C-7"
assert model["components"]["seeker"]["selector"] == "equipment.aim-120c-7.seeker"
# Retained store-card aliases resolve to the same equipment definition.
assert fsim.catalog.equipment_model("aim-120c") == model
```

These calls return fresh static installed metadata. They do not report live sensor knowledge. The Viewer labels its component tree as declared catalogue models; custom/replayed runtime state must not be inferred from that tree. Actual native observations and outcomes keep their original authority.

## Current model mapping

This table is generated from the catalogue. The JSON remains authoritative for all component selectors and applicability reasons.

| Retained store key | Equipment identity | Equipment designation | Applicable components |
| --- | --- | --- | --- |
| `463l-pallet` | `463l-pallet` | 463L pallet, loaded | 1 |
| `a10-tank-600gal` | `a10-tank-600gal` | 600 US gal external tank (A-10) | 1 |
| `aim-120c` | `aim-120c-7` | AIM-120C-7 | 9 |
| `aim-120d` | `aim-120d` | AIM-120D | 9 |
| `aim-7m` | `aim-7m` | AIM-7M | 9 |
| `aim-9m` | `aim-9m` | AIM-9M | 9 |
| `aim-9x` | `aim-9x-block-i` | AIM-9X Block I | 9 |
| `an-aaq-28` | `an-aaq-28` | AN/AAQ-28 LITENING | 1 |
| `an-alq-131` | `an-alq-131` | AN/ALQ-131(V) | 1 |
| `an-alq-184` | `an-alq-184` | AN/ALQ-184(V) | 1 |
| `an-alq-99` | `an-alq-99` | AN/ALQ-99 | 1 |
| `an-asq-228` | `an-asq-228` | AN/ASQ-228 ATFLIR | 1 |
| `asraam` | `asraam` | AIM-132 ASRAAM | 9 |
| `cds-a22` | `cds-a22` | CDS bundle (A-22) | 1 |
| `cj-10a` | `cj-10a` | CJ-10A | 8 |
| `damocles` | `damocles` | Damocles | 1 |
| `f22-tank-600gal` | `f22-tank-600gal` | 600 US gal external tank (F-22) | 1 |
| `fpu-11` | `fpu-11` | FPU-11/A 480 US gal tank | 1 |
| `fpu-8` | `fpu-8` | FPU-8/A 330 US gal tank | 1 |
| `gripen-tank-1100l` | `gripen-tank-1100l` | 1,100 L external tank (Gripen) | 1 |
| `iris-t` | `iris-t` | IRIS-T | 9 |
| `j10-tank-1600l` | `j10-tank-1600l` | 1,600 L external tank (J-10) | 1 |
| `j10-tank-800l` | `j10-tank-800l` | 800 L external tank (J-10) | 1 |
| `magic-2` | `magic-2` | R.550 Magic 2 | 9 |
| `meteor` | `meteor` | Meteor | 9 |
| `mica-em` | `mica-em` | MICA EM | 9 |
| `mica-ir` | `mica-ir` | MICA IR | 9 |
| `mk82` | `mk82` | Mk 82 | 3 |
| `pl-10` | `pl-10` | PL-10 | 9 |
| `pl-12` | `pl-12` | PL-12 | 9 |
| `pl-15` | `pl-15` | PL-15 | 9 |
| `pl-8` | `pl-8` | PL-8 | 9 |
| `ptb-1150` | `ptb-1150` | PTB-1150 | 1 |
| `ptb-1500` | `ptb-1500` | PTB-1500 | 1 |
| `ptb-2000` | `ptb-2000` | PTB-2000 | 1 |
| `ptb-800` | `ptb-800` | PTB-800 | 1 |
| `r-27er` | `r-27er` | R-27ER | 9 |
| `r-27et` | `r-27et` | R-27ET | 9 |
| `r-27r` | `r-27r` | R-27R | 9 |
| `r-27t` | `r-27t` | R-27T | 9 |
| `r-37m` | `r-37m` | R-37M | 9 |
| `r-60m` | `r-60m` | R-60M | 9 |
| `r-73` | `r-73` | R-73 | 9 |
| `r-77m` | `r-77m` | R-77M | 9 |
| `rpl-522` | `rpl-522` | RPL-522 | 1 |
| `rpl-541` | `rpl-541` | RPL-541/542 | 1 |
| `rpl-701` | `rpl-701` | RPL-701 | 1 |
| `rpl-751` | `rpl-751` | RPL-751 | 1 |
| `super-530d` | `super-530d` | Super 530D | 9 |
| `tank-300gal` | `tank-300gal` | 300 US gal centerline tank (F-16) | 1 |
| `tank-370gal` | `tank-370gal` | 370 US gal external tank (F-16) | 1 |
| `tank-610gal` | `tank-610gal` | 610 US gal external tank (F-15) | 1 |
| `typhoon-tank-1000l` | `typhoon-tank-1000l` | 1,000 L supersonic tank (Typhoon) | 1 |

## Verification scope

Native coverage checks every named selection against its unchanged mechanism and physical fingerprint; negative controls reject wrong-equipment, wrong-role, wrong-native-model and invented selectors before construction. Hangar checks all named/legacy fingerprints and caller-owned catalogue data. The installed Python catalogue must agree with its own packaged cards.

Naming verification does not certify complete SIM-M03 physical acceptance. At the naming-migration review snapshot, the AIM-9M native countermeasure coupling failure required a separate repair; that historical failure remains evidence of that snapshot. Subsequent receiver repairs still require current all-model acceptance, which successful naming or synthetic servo tests cannot replace. Full installed/fleet, both-preset regression and actual fullscreen 1x visual gates retain their existing requirements.
