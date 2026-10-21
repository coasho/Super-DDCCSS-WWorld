# SIM-M03 fragment intersection and hit repair

Baseline: main `8147c071d824a16572023dc93b547b1a17254b53`, initially clean.
The user assigned diagnosis and repair of the fixed 6 m side-burst internal
component failure and showcase fragment-Hit failure. Existing equipment
parameters, scene inputs and acceptance assertions are frozen. AeroNumericsLab
and Claude's SIM-M02 inputs, solvers and build directories are excluded.

## Sequence and evidence

1. Reproduce both unchanged failures. Trace fragment source, reference frames,
   surface/volume intersections, skin traversal, component interception and
   recording-event admission. Separate an implementation defect from an
   unsupported physical expectation before changing production behavior.
2. Reduce each established defect to deterministic analytic regressions with
   independent expected intersections or momentum/energy. Run those regressions
   before the fix. Synthetic fixture values are not equipment calibration.
3. Repair only the proven mechanism. Keep native event meanings, fragment RNG
   draws, authored geometry/physics inputs and original assertions intact.
4. Run serial UCRT64 Headless/Release builds, damage/warhead/contact/engagement
   regressions, the unchanged showcase test and appropriate component/recording
   gates. Record current discovered counts and residual failures. Review and
   commit only feature-owned files on main, with no remote publication.

## Exclusive path and symbol window

* `src/simulation/mission_systems/damage/Damage.cpp`: private fragment ray
  intersections, event ordering and traversal in `assess`; temporary diagnostic
  instrumentation must be removed before delivery.
* `tests/simulation/test_damage.cpp` and `test_warhead_model.cpp`: independent
  synthetic fragment/geometry and conservation regressions. Preserve existing
  equipment fixture and all its acceptance assertions.
* `src/simulation/runtime/world/WorldDamage.cpp` and `WorldEngagements.cpp`:
  captured burst/target reference transforms or damage dispatch only if tracing
  proves a defect. Do not relabel contact/blast events as fragment hits.
* `tests/integration/test_showcase_native.cpp`: additional diagnostic/provenance
  assertions only; existing scene and hit/functional criteria remain frozen.
* This plan, `docs/sdk/representative-weapon-models.md`,
  `docs/sdk/fragment-hit-repair.md`, `docs/status.md` and
  `docs/architecture/modules/simulation/ms-work-allocation.md`: repair scope,
  root-cause evidence and dated validation/handoff.

No public API/layout, generic MA/gateway/schema, card, native aircraft asset,
Hangar generator, coefficient, fuze threshold or scene change is authorized by
this window. Record a reason and an updated exact scope before editing any
additional file. Diagnostic scripts/data/logs belong under
`build/architecture-reviews/fragment-hit-repair/`.

No native configure/build/test process was active at slot acquisition. This
feature owns the serial SDK native slot during its checks. Use the existing
UCRT64 launcher, TMP/TEMP/TORCH_EXTENSIONS_DIR at D:/ENV/tmp, bounded timeouts,
`--no-tests=error` and one reporting background entry per operation. The
independent AeroNumericsLab workspace is neither inspected nor used.

## Research review and implementation boundary

The user explicitly requested continued research. Review public primary sources
and mature implementation code, record exactly what they establish and identify
their validation limits. The source review belongs in
`docs/sdk/fragment-hit-repair.md`; downloaded evidence stays in the diagnostic
artifact directory. Research is not equipment calibration or an instruction to
make the unchanged showcase pass by selecting an unverified source model.

A generic closed-case candidate was considered during research. The cited
Brighton thesis explicitly leaves its end-plate velocity result unverified
(printed p.104, section 5.3.4). Its subsequent fragmentation paper is also generic;
neither establishes AIM-120C-7-specific acceptance. No new source implementation,
provider, card selection, geometry or quantitative lethality model is admitted
in this repair. The original exact write scope and frozen inputs remain in
force. Deliver the verified traversal fix and source audit with the showcase
failure visible; do not claim both failures have been repaired.

## Handoff

Headless and Release builds completed serially. The selected gates passed
99/99 in Headless and 99/100 in Release, with zero skipped tests. The only
remaining failure is `showcase_native`; the original 6 m fixture and both new
regressions pass. No shared integration, installed-consumer or Viewer acceptance
is claimed. Release the SDK native build/test slot at this handoff. No branch,
worktree, remote publication or AeroNumericsLab access is part of this delivery.
