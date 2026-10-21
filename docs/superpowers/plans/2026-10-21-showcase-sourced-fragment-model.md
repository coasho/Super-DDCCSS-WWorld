# Sourced generic finite fragment field for the original showcase

Baseline: main `dded7aed2c043a9f2f8b611bcc4c3f8fd9a9be69`, with existing
feature changes and the user-edited prior repair plan preserved.
The user explicitly authorizes replacing the incompatible generic assumptions
with a sourced generic model while preserving the original scene, positive Hit
assertion and equipment physical values. No calibrated C-7 claim is authorized.

The fixed contact encounter has a strictly separated pure radial envelope.
The existing legacy source claims a seven-zone game reference but retains only
five middle zones and renormalizes them. Review of pinned datamine commit
`ea96f3af75afe1d858f80ac13f212db218bc69ca`, file
`aces.vromfs.bin_u/gamedata/weapons/rocketguns/us_aim_120c_7.blkx`, confirms seven
angular intervals and shares, including forward and aft sectors. Only those
angular intervals and relative shares are admitted as a game-reference profile.
No upstream explosive, propulsion, fuze, radius, penetration or damage multiplier
is imported. Community extraction is not hardware validation.

Add `FiniteZonedCaseWarheadV2` with an explicit paired V2 model/source selector,
finite annular origins, the complete pinned angular field and correlated
azimuth. Keep V0 and pure-radial V1 behavior and their RNG contracts intact.
V2 consumes exactly five draws per fragment: axial position, azimuth, annular
radius, zone choice and polar cosine. The finite source, mass/count/area,
Gurney speed, atmosphere/drag, blast and penetration inputs retain their
existing construction and values. Preserve captured authored charge location
and rigid-point velocity for V2 in World. Keep the existing fuze unchanged
unless independent evidence demonstrates a separate defect.

## Exclusive implementation paths and symbols

* `damage/Damage.{h,cpp}`: additive source enum/version, pinned V2 angular
  profile, finite sampler and strict validation. Existing V0/V1 semantics stay.
* `damage/WarheadAdapter.cpp`, `components/registration/EffectFactoriesV1.*`,
  `WarheadProvidersV1.cpp`, new `implementations/warhead/FiniteZonedCaseWarheadV2.*`
  and `src/simulation/mission_systems/CMakeLists.txt`: exact provider selection.
* `World.cpp`: fixed authored charge capture for V2. `WorldDamage.cpp`: finite
  target-origin velocity handling. Remove temporary `WorldEngagements.cpp`
  tracing; no new fuze behavior is planned.
* `assets/stores/aim-120c.json`: model/source selector and component/provenance
  metadata only. All physical values, uncertainties and other store cards stay.
* `test_warhead_model.cpp`, `test_payload_effect_adapters.cpp`,
  `test_payload_providers.cpp`, `test_damage.cpp`: model-specific exact tests,
  analytic finite-source/RNG/conservation and refusal controls. Preserve all
  original showcase acceptance assertions and the 6 m equipment fixture.
* `tests/integration/test_showcase_native.cpp`: update only the exact selected
  warhead model-version expectation from V1 to V2. Keep execution/fuze version
  checks exact and preserve all scene, positive Hit, provenance, functional,
  continuation, determinism, cadence and refusal assertions unchanged.
* `tests/simulation/test_engagement_native.cpp`: the same exact selected
  warhead-version expectation 1 to 2; retain every trajectory, native control,
  functional, determinism and event assertion.
* The newly added Fire-rate publication regression uses the already maintained
  spill-fixture grid to establish a burning tank. Its former single-point
  preparation no longer ignites a tank with the V2 field. Keep all positive
  burning/update, coordinate and rigid-velocity assertions and all grid inputs;
  do not modify the original showcase scene or any physical equipment value.
* Feature acceptance/status documents: source mapping and current evidence.
* `tools/validation/payload_models/report.py` and `test_report.py`: update the
  existing source index for the new concrete provider without promoting
  configuration reporting to physical acceptance.

First run new V2 construction/provider tests on the unimplemented version and
retain RED. Then implement and run those tests, original showcase and 6 m gates.
Validate repeated native recording, source attribution, functional consequences
and continuation. Run serial Headless/Release builds and relevant discovered
nonzero tests, then appropriate render evidence. Do not change the source
profile or any physical value in response to an encounter failure.

The serial SDK native slot remains exclusively held for this continuation;
TMP/TEMP/TORCH_EXTENSIONS_DIR stay under `D:/ENV/tmp`. No external worker,
AeroNumericsLab input, other worktree, branch or generic MA contract is used.

## Original-scene render window

After both 111-case native matrices and the feature-owned installed-equipment
gate, acquire the single GUI slot for `strike_acceptance.py`. No Viewer is
already running. Pass the original showcase's `first.json`, retaining its
270-degree target, seed, geometry, device values and 40 s duration. The existing
harness changes only the diagnostic World name, validates preflight, then
requires the live native Hit/CrewLost and two event-relative fullscreen frames
through one continuous run. Output:
`D:/ENV/tmp/showcase-v2-original-20261021`. Do not change the Viewer or its
acceptance assertions. This is diagnostic SDK/Viewer evidence, not installed
public-endpoint joint acceptance.

## Handoff

The original `showcase_native` now passes with its original scene and positive
Hit/functional gates. Native and independently regenerated rays agree on the
same 53/1688 damage-geometry intersections. Every physical parameter object
and the other 52 cards remain unchanged. The newly selected V2 source is an
explicitly sourced generic assumption, not a calibrated equipment model.

Both complete builds pass. Release executes 111/111 selected cases in 263.60 s;
Headless executes the same 111/111 in 262.90 s; zero skips. Original recordings
and complete event JSON match byte-for-byte across presets. The separate
installed-equipment C++/C99/Python gate passes 1/1 in 38.79 s. The current source
catalogue reports zero unmapped available IDs and its regressions pass 12/12.

The unchanged fullscreen harness also passes on the original 270-degree scene:
1200 native frames through 40 s, positive native Hit/CrewLost, two exact
event-relative 3840 x 2160 captures, both visually inspected. The target view is
close/cropped and offline subtiles are incomplete; no cinematic/terrain
completeness claim is made. Sources, limitations, exact commands and raw
artifact paths are recorded in `docs/sdk/fragment-hit-repair.md`.

Temporary tracing is removed. The pre-existing user edit to the earlier repair
plan remains unchanged and outside this delivery. No public SDK layout/vtable,
generic MA contract, aircraft/generated geometry or fuze behavior changes.
Full SIM-M03 physical acceptance and generic joint acceptance remain pending.
Release the serial native and GUI slots; all feature execution is finished.
