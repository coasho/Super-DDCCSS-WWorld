# SIM-M03 representative weapon implementation

Baseline: main `8daf1e0f3e3bd01359a2b6a8f85b538b93d29bf8`, initially clean.
The user approved the [design](../specs/2026-10-20-sim-m03-representative-weapons-design.md)
and requested implementation in this session. Execution is inline. The feature
owns the serial SDK native build/test slot; no native configure/build/test process
was active at acquisition. AeroNumericsLab and its SIM-M02 work are excluded.

## Acceptance and sequence

1. Add behavioral regressions for bearing-only information, semantic-label and
   identity independence, unresolved IR centroid versus spatial association,
   illuminator interruption, active-radar activation and observation lifetime.
   Demonstrate the current mechanism gap before implementing the replacement.
2. Add private measurement execution contracts without changing any existing
   public layout or V1 provider virtual interface. Separate sensor observables
   and local tracks from referee attribution. Reuse the existing authored
   angular gate and assumed range/strength inputs; add no equipment coefficients.
3. Implement separate non-imaging IR, imaging IR, semi-active radar and active
   radar provider classes with matching headers/sources. Implement measured LOS
   PN and RF measured-state guidance. Keep unknown range/timing/uncertainty
   unavailable. Legacy Cartesian guidance remains available for unmigrated cards.
4. Register strict schemas and migrate only AIM-9M, AIM-9X Block I, AIM-7M and
   AIM-120C-7. Preserve all existing numeric equipment inputs. Record the exact
   variant, inherited assumptions, unavailable image/servo/TVC data and idealized
   launcher updates. A model cannot claim a richer missing submodel is running.
5. Integrate World scene projection, sampled execution, per-installed-store
   on-rail state, release ownership, reset and per-release tracking. Repeated
   queries read snapshots. Guidance receives no semantic target/decoy labels or
   hidden Cartesian IR target state. Referee attribution remains external.
6. Add production-path regressions covering configured factories, native release,
   tracking, cache/lifetime behavior and authoritative refusal. Run fresh scoped
   UCRT64 checks and Hangar equivalence checks. Preserve all pre-existing physical
   failures and their criteria. Complete a self-review and record exact evidence.

## Exact file and symbol window

New files:

* `src/simulation/mission_systems/components/interfaces/SeekerExecutionV2.h`:
  sensor samples, frames, measured tracks and optional execution interface.
* `src/simulation/mission_systems/components/interfaces/MeasuredGuidanceV2.h`:
  measured-guidance contract, separate from V1 Cartesian observations.
* `src/simulation/mission_systems/components/support/MeasuredSeekerV2.h` and
  `.cpp`: finite angular math, shared provider helper bridge and per-instance
  track history; no target identity or decoy classification input.
* `src/simulation/mission_systems/components/implementations/seeker/`:
  `NonImagingIrSeekerV2`, `ImagingIrSeekerV2`, `SemiActiveRadarSeekerV2` and
  `ActiveRadarSeekerV2`, each in matching `.h` and `.cpp` files.
* `src/simulation/mission_systems/components/implementations/guidance/MeasuredPnGuidanceV2.h`
  and `.cpp`: measured-bearing and measured-RF guidance, explicit unavailable
  legacy-call refusal.
* `tests/simulation/test_weapon_seeker_models.cpp` and
  `tests/simulation/test_weapon_seeker_native.cpp`: behavioral and native
  production/lifecycle regressions.
* `docs/sdk/representative-weapon-models.md`: source admission, executed mechanisms,
  commands/results and remaining physical gates.

Existing files:

* `src/simulation/mission_systems/weapon/{Seeker.cpp,Guidance.cpp,Composition.cpp}`:
  additive strict selection/configuration and compatible composition checks.
* `src/simulation/mission_systems/components/registration/{SeekerProvidersV1.cpp,GuidanceProvidersV1.cpp}`:
  new model factories with explicit schema identities; preserve legacy factories.
* `src/simulation/mission_systems/CMakeLists.txt`, `tests/CMakeLists.txt`:
  explicit feature translation units and bounded regression registration.
* `src/simulation/runtime/world/{World.h,World.cpp,WorldEngagements.cpp}`:
  private installed-store caches, release state transfer, sensor scene sampling,
  reset and measured guidance dispatch. No changes to generic MA behavior.
* `assets/stores/{aim-9m,aim-9x,aim-7m,aim-120c}.json`: concrete selection,
  bindings, variant identity, source and limitations; no numeric parameter edits.
* `tools/models/hangar/hangar/components.py` and
  `tools/models/hangar/tests/test_components.py`: strict selected-schema support
  and unchanged-physical-output checks.
* `aircraft/stores/{aim9m/aim9m,aim9x/aim9x,aim7m/aim7m,aim120c/aim120c}.xml`:
  generator-owned component metadata only; no physical functions, engine/nozzle
  bytes or model inputs change.
* `tests/simulation/{test_payload_composition.cpp,test_payload_composition_contract.cpp,test_payload_composition_native.cpp}`:
  retain generic assertions while accommodating the four explicit model choices.
* `docs/sdk/{payload-native-providers.md,payload-seeker-guidance-adapters.md}`,
  `docs/status.md`, `docs/architecture/modules/simulation/ms-work-allocation.md`:
  current component inventory and dated, scoped acceptance/handoff evidence.
* This plan and its approved design: status/evidence clarification only.

No writes to `include/`, generic schema/transport/gateway files, aircraft design
TOML, aerodynamic tables, third-party sources or AeroNumericsLab are authorized
by this implementation window. Scope expansion requires a recorded reason and
an updated exact window before the added file is edited.

## Verification and source boundaries

Use the existing UCRT64 preset and `tools/launch/fsim.ps1` conventions. Set TMP,
TEMP and TORCH_EXTENSIONS_DIR to D:/ENV/tmp before native execution. Run one
serial native writer at a time, with bounded test timeouts and --no-tests=error.
Save raw logs under `build/architecture-reviews/representative-weapons/`; do not
wrap a reporting background job in a second polling job.

Primary sources and immutable reference pins are in the approved design.
JHU/APL supplies the LOS/homing-loop reference; native RF/tracker primitives and
Stone Soup supply measurement/association comparisons. pyBSM is an offline
image-formation reference, not an installed dependency. Native projection, if
available, is explicitly ideal geometric imagery; no detector resolution,
thermal signature, noise distribution or proprietary countermeasure logic is
inferred from geometry. The existing AIM-9X profile remains fin-controlled until
accepted TVC plant data is supplied; that limitation cannot be renamed away.

Software mechanism acceptance, installed feature provenance, diagnostic scenes,
calibrated equipment behavior and full SIM-M03 acceptance are separate results.

## Delivery notes

The [SDK delivery record](../../sdk/representative-weapon-models.md) contains
current evidence. The implemented generator regression establishes identical
plant output for the new selections. No Hangar generator or generated XML edit
was required: the existing metadata stamp covers plant roles and is unaffected
by seeker/guidance identity. The broader prospective generator/XML paths above
therefore remain unchanged.

The four-model native diagnostics optionally write checked recordings through
`FSIM_REPRESENTATIVE_RECORD_DIR`; they do not alter actor observations or qualify
as full Viewer acceptance. Final source review also adds a native regression
requiring bearing-only frames to omit launcher Cartesian estimates entirely,
even when the implementation would otherwise ignore the unused field.

The source window is frozen after the recorded Headless/Release builds and
checks. The SDK serial native build/test slot is released at this handoff.
No solver slot, build inputs or numerical results in AeroNumericsLab were used.
The handoff preserves the side-burst component and showcase fragment-Hit failures;
independent reference comparisons, hardware calibration and Viewer acceptance
remain pending. The resulting main commit contains the reviewed feature paths;
no remote publication is part of this window.
