# Payload component configuration and evidence report

The 2026-10-21 [sourced finite-field implementation](fragment-hit-repair.md#sourced-finite-fragment-field-and-original-showcase-acceptance)
closes the original showcase Hit gate. The source index now includes
`FiniteZonedCaseWarheadV2`; the catalogue still separates model selection from
runtime evidence and physical acceptance. The dated failure/counts below are
the earlier reporting snapshot, not current showcase status.

Date: 2026-10-20. Baseline: main
`dded7aed2c043a9f2f8b611bcc4c3f8fd9a9be69`.

The read-only [report command](../../tools/validation/payload_models/README.md)
produces an offline searchable HTML catalogue and a complete JSON snapshot.
It reports all immediate maintained store cards, their nine component roles,
configured provider IDs, indexed concrete classes, resolved parameter
provenance, sources, limitations and validity. It does not change SDK behavior,
model parameters, cards, generated assets, scenes or acceptance assertions.

## What the current catalogue actually distinguishes

Fresh generation reads 53 cards and 477 role declarations. Available means
declared/configured, not demonstrated execution or physical acceptance.

| Role | Available cards | Indexed concrete classes | Unavailable | Not applicable |
| --- | ---: | ---: | ---: | ---: |
| Seeker | 24 | 5 | 0 | 29 |
| Guidance | 24 | 2 | 0 | 29 |
| Autopilot | 24 | 1 | 0 | 29 |
| Allocator | 24 | 1 | 0 | 29 |
| Actuator | 24 | 1 | 0 | 29 |
| Fuze | 24 | 2 | 2 | 27 |
| Warhead | 24 | 2 | 2 | 27 |
| Propulsion | 24 | 1 | 0 | 29 |
| Aerodynamics | 53 | 1 | 0 | 0 |

The four representative cards select their separate V2 seeker classes; twenty
other guided cards share `EncounterSeekerV1`. The 24 guided cards share the
same concrete autopilot, allocator and actuator classes and each role's single
resolved numerical parameter configuration. This does not imply identical
complete aircraft/control response: geometry, asset bindings and generated
aerodynamics may differ. Report parameter fingerprints deliberately exclude
paths and identities. MK 82 and CJ-10A retain unavailable fuze/warhead entries;
unavailable is not converted to not-applicable.

A class count is not a maturity score. Per-component `runtime_evidence` and
`physical_acceptance` remain `not_assessed` by this reporting tool. The HTML
keeps component assumptions distinct from published parameter sources.
Provider interface/configuration version does not establish improved physical
fidelity. For actual package evidence, consult the
[representative-model delivery](representative-weapon-models.md),
[provider contract](payload-native-providers.md) and
[physical-chain limitations](sim-m03-physical-chain-audit.md).

## Reproduction and evidence ownership

```powershell
D:/ENV/python/3.13/python.exe -B tools/validation/payload_models/report.py --output-dir build/architecture-reviews/sim-m03-model-report --native-junit build/ucrt64-release/sim-m03-report-native.xml
```

That command produces `report.html` and `report.json`. With the current imported
failed showcase it exits **1**, retaining the diagnostic artifacts and failure.
The input manifest hashes all cards, the command/resolver, selected source
headers/implementations/registrations and adapter selection validators. The
JUnit artifact has its own hash. HEAD alone is not treated as proof of a clean
checkout or matching binary. The maintained ID-to-class index checks referenced
class declarations and registration IDs; it is not C++ dispatch analysis or
instance instrumentation. Unknown IDs stay unmapped and produce exit 1.

Current checks:

| Check | Result | Artifact under `build/architecture-reviews/` |
| --- | --- | --- |
| Report class-sharing and alias regressions before mapping | Two expected assertion failures | `sim-m03-report-behavior-red.log` |
| All report regressions | 11/11 pass | `sim-m03-report-green.log` |
| Existing Hangar component metadata/generation suite | 12/12 pass | `sim-m03-report-resolver.log` |
| Fresh catalogue/source generation | 53 cards, 477 role declarations, 16 indexed classes, zero unmapped available IDs | `sim-m03-report-generate.log`, `sim-m03-model-report/report.json` |
| Serial existing Release native selection | 4/5 pass, zero skipped, 4.81 s | `sim-m03-report-native.log`; JUnit in `build/ucrt64-release/sim-m03-report-native.xml` |

The native selection was discovered first: `representative_weapon_models`,
`representative_weapon_native`, `payload_providers`, `payload_profile` and
`showcase_native`. Only `showcase_native` fails. The existing binaries were
used without rebuilding. No full-suite, installed-consumer, physical-model or
Viewer acceptance is claimed by this follow-up. The native slot is released.
Browser policy rejected local `file:` preview; HTML visual and interaction
validation remains pending. No alternative browser route was used.

## Mature implementation/reference decision

* [Godot 4.4-stable `ray_cast_3d.cpp`](https://github.com/godotengine/godot/blob/4.4-stable/scene/3d/physics/ray_cast_3d.cpp),
  `_update_raycast_state`, lines 205-240, keeps explicit collision validity
  separate from the point/normal fields. Its
  [4.4 documentation](https://docs.godotengine.org/en/4.4/classes/class_raycast3d.html)
  describes per-physics-frame result caching and immediate refresh. The
  [MIT license](https://github.com/godotengine/godot/blob/4.4-stable/LICENSE.txt)
  was inspected. This is a mature game collision/reporting precedent, not a
  warhead model, independent execution oracle or equipment dataset. No Godot
  code/dependency was adopted: the preceding independent native/analytic checks
  establish genuine non-intersections, so replacing the engine is not justified
  by this failure. Compatibility, numerical equivalence, runtime cost and
  determinism of a Godot integration were not measured.
* [NASA-STD-7009B, 5 March 2024](https://standards.nasa.gov/standard/NASA/NASA-STD-7009)
  supplies an evidence/credibility framework. This report's separation of
  configuration, execution and physical acceptance is a project reporting
  decision consistent with that purpose, not a NASA certification or equipment
  validation result. It supplies no numerical model inputs.

## Historical reporting snapshot: the fragment-Hit repair was still open

The unchanged native showcase again records one burst, zero fragment Hits,
and actual inventory/release/removal. The preceding
[fragment investigation](fragment-hit-repair.md) captured 1688 sampled rays;
native and independent geometric checks both found zero intersections.
This follow-up reruns the original acceptance and preserves its failure; it
does not repeat that independent ray capture or invent a new positive Hit.

Contact/blast functional effects cannot be relabeled as fragment Hits, and a
source catalogue does not make a physical acceptance case pass. Further
real-equipment model acceptance retains its own admitted-input and independent
validation requirements. SIM-M02 calibration is not imported or inferred;
Claude's AeroNumericsLab workspace, solvers and build directories are untouched.
