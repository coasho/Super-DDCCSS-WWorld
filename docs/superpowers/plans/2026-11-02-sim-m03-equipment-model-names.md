# SIM-M03 equipment model identity correction

The user's current instruction requires actual equipment/model names instead of using V1/V2/V3 mechanism revisions as equipment identities. This extends the active complete M03 delivery; no original physical gate is waived.

Main baseline remains `e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f`. Root retains the exclusive native configure/build/test and GUI slots. No other native process is active at acquisition. The complete inherited source/card byte receipt is `build/architecture-reviews/sim-m03-delivery-20261102/equipment-names/initial-window.json`.

## Model identity and authority

Use one maintained catalogue of the 53 store configurations and all nine component roles. A model selector names the actual equipment/construction, for example `equipment.aim-120c-7.seeker` or `equipment.aim-9x-block-i.seeker`. Specific hardware part numbers are used only with admitted evidence; otherwise the model is explicitly scoped to the named equipment and component function. An unknown part number is never invented.

The existing component configuration schema, public layouts, provider virtual tables and old implementation IDs remain compatible. Register new equipment-specific selections using the existing provider-table schema/selection separation. Validate the selected equipment, native model and component role before construction. All shared physics backends, source parameters, physical fingerprints, numerical mechanisms and defaults retain their meanings. The current explicit model selector changes no device coefficient or scene criterion.

One catalogue supplies the native registry, neutral model-name contract, Hangar resolver, inventory/evidence and Viewer. The installed producer must carry the same data and compile source. Model names are not acceptance evidence: each selected instance still needs its source-bound component configuration and actual native/public/installed/visual gates.

## Exclusive path window

Root owns new `tools/models/hangar/hangar/data/equipment_models.json`, new `hangar/equipment_models.py` and direct tests; new neutral `src/contracts/equipment/EquipmentModels.h.in` and contract CMake generation; new `components/registration/EquipmentModels.{h,cpp}`, built-in registration and ProviderRegistry validation; Mission Systems CMake source registration; current 53 `assets/stores/*.json` implementation selectors only; Hangar `components.py` resolution and provenance; new direct native naming tests and their registration; `WeaponDiagnosticsGui.cpp` model names; weapon-quality audit/countermeasure/evidence naming integration; and the named-equipment guide. Source copies are retained before modification. Add exact further paths before changes. Do not alter another owner's generic gateway/schema or equipment coefficients.

## Required verification

- Complete unique catalogue/role coverage, exact actual-model aliases and honest hardware identity/source limits.
- Wrong-model/wrong-role/wrong-native-plant selections refuse before effects; old IDs and configurations remain usable.
- Native/Hangar mechanism normalization and physical fingerprints agree exactly, with original numeric parameters unchanged.
- Current all-model tests and evidence use actual equipment selectors; version suffixes identify only compatibility interfaces/backends.
- The original AIM-9M countermeasure gate remains failed and must still be repaired and rerun; the successful moving-LOS unit case is not its replacement.

The extended window additionally owns `ProviderTableV1.h` only for an additive named-model alias operation using its existing Registration storage and unchanged copy/override behavior; `ReceiverConfigurationV1.cpp` named-schema normalization; `cmake/{EquipmentModels,Assets,Install}.cmake`; `python/CMakeLists.txt` catalogue staging and `python/fsim/catalog.py` static catalogue access. No existing layout, vtable, schema field or numeric model input changes. Exact inherited bytes are in equipment-names/extended-window.json.

The normalization window additionally owns PlantAdapter.cpp only for allocation-free mechanism-name fingerprint normalization, PulseManagerV1.cpp named-profile decoding, and Hangar engineering.py/storecontrol.py only for canonical mechanism selection before their unchanged numerical branches. Other helper/schema changes require a further exact window.

The compatibility window additionally owns tests/simulation/fixtures/weapon_v2_assets.py, verify_weapon_v2.cmake and weapon-v2/{manifest,aim-9m,aim-9x,aim-7m,aim-120c}.json for source-bound regeneration after selector migration. Preserve exact legacy mechanism/numeric fixture behavior; record generator and catalogue normalization dependencies. The inherited bytes are in equipment-names/compatibility-window.json.

The public-catalogue verification window additionally owns tests/compatibility/python/test_catalog.py, preserving its existing assertions while checking all installed equipment/card identities, independent returned values and missing catalogue refusal. No generic endpoint/boundary implementation changes.

The decoder integration window additionally owns components/support/EquipmentSelectionV1.h, weapon/{Seeker,Guidance,FuzeAdapter,PlantAdapter}.cpp and damage/WarheadAdapter.cpp for canonical selection at direct configuration entry points, with unchanged numerical decoders. It owns test_weapon_seeker_models.cpp only for the explicit retained ideal-observation fixture to use the current single/dual physical-policy parser. The M03 extension boundary repair owns fsim_mission_c.cpp only to put its existing two Agent guards first, boundary.toml only to remove the unused value-record rule (records are not callable entry points), and boundary.md regeneration. Snapshot: equipment-names/decoder-window.json.

The reporting window owns viewer_perf/aam_campaign.py and test_aam_campaign.py for source-validated named selections with separately decoded mechanisms. It owns test_payload_composition.cpp to include the already authored PL-10 vector model in the exact roster, and test_payload_plant_adapters.cpp to retain the unknown-code refusal using an actually unknown code after the supported resident-air mechanism added code 5. No acceptance threshold changes. It owns weapon_quality/installed.py catalogue provenance/public checks and countermeasures.py decoder-source freezing. Snapshot: equipment-names/reporting-window.json.

The architecture integration window owns root CMakeLists.txt, cmake/EquipmentModels.cmake, src/contracts/CMakeLists.txt and generated contracts/equipment/EquipmentModels.h. Generate the tracked neutral header from the catalogue explicitly and verify exact bytes at root configuration; use the existing contract include view with no extra compiler search root. policy.json registers only the actual ms-to-neutral-contract dependency (already allowed by domain direction). No checker exception or broad include root is added. test_payload_composition_native.cpp normalizes mechanism comparisons and supplies the actual mechanism schema when replacing named factories. Snapshot: equipment-names/architecture-window.json. Unrelated untracked tools/maintenance scripts remain untouched and outside the delivery allowlist.
