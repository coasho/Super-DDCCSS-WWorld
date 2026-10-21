# Showcase transform and event-chain correctness

Baseline: main `dded7aed2c043a9f2f8b611bcc4c3f8fd9a9be69`.
The user authorized continued diagnosis of coordinate transforms, intersections
and Hit propagation, with unchanged model inputs and original acceptance gates.
Existing dirty documentation and the configuration-report work are preserved.

Exclusive write/symbol window:

* `src/simulation/runtime/world/WorldDamage.cpp`: component event position and
  velocity publication only; preserve assessment, damage, RNG and Hit admission.
* `tests/simulation/test_damage.cpp`: independent event-coordinate regressions
  through existing native fixtures. Do not change existing fixture/assertions.
* `tools/validation/showcase/main.cpp` and `FragmentHit.h`:
  native Hit attribution and live camera focus, without changing scene/outcome.
* `tests/integration/test_showcase_native.cpp`: additive event-chain regressions;
  original acceptance assertions remain enforced.
* `src/simulation/runtime/world/World.h`: additive internal read-only
  `World::vehicleStreamSlot` to match stream events without ID assumptions;
  no installed public signature, layout, vtable or enum change.
* `tests/CMakeLists.txt`: register only new feature-owned software regressions
  if they cannot use an existing discovered executable.
* This plan, `docs/sdk/fragment-hit-repair.md`, `docs/status.md`: current software
  corrections, evidence and unresolved input requirements.

Read-only review includes captured pose/CG reconstruction in WorldEngagements,
material-point transforms, damage ray/quad/box code, immutable F-16C geometry,
recording events and the existing independent ray analysis. No numerical model,
card, geometry, source distribution or event semantics will be changed to make
an unsupported physical expectation pass. No AeroNumericsLab or other worktree
access. Report the original showcase outcome even if it remains failing.

No native configure/build/test process was active at slot acquisition. This
feature holds the serial SDK native slot for its bounded checks, with existing
UCRT64 launcher and D:/ENV/tmp environment. No GUI slot is needed. Release slots
at handoff. First demonstrate each software defect with a failing regression,
then implement only the proven correction.

The read-only coordinate review also includes `World.cpp`'s authored-nose
inch-to-metre conversion and structural-to-body station offsets. That file is
not changed. The helper's exact path is
`tools/validation/showcase/FragmentHit.h`; its sole predicate is
`fsim::showcase::matchingFragmentHit`.

## Handoff

The behavioral REDs establish initial Fire reference/CG mismatch, later Fire
point/velocity loss, and live target focus without a matching recorded Hit.
The corrections reuse existing material-point transforms and frozen native Hit
semantics. Additional positive/negative controls cover live slot identity/reuse,
non-consuming recording after outbox reads, and strict Hit attribution.

Both complete UCRT64 builds pass. The discovered scoped selections each contain
107 gates: Headless 106/107 in 208.45 s; Release 106/107 in 208.18 s; zero skipped.
Only the original positive-Hit `showcase_native` gate fails. Both new live
event-chain gates pass. The original Release recording is byte-identical to
the saved baseline; the full engagement JSON also matches. Physical inputs,
geometry, native assessment and original positive-Hit assertions are preserved.

The full explanation and remaining admitted-data requirements are recorded in
`docs/sdk/fragment-hit-repair.md`. These are software corrections, not complete
SIM-M03/model acceptance. No installed-consumer, generic full-suite or rendered
Viewer acceptance is claimed. Changes remain in the main working tree; no
branch, worktree, commit, push or external-worker message is created. The
pre-existing dirty fragment-repair plan remains untouched. Release the native
write/build/test slot at this handoff; no GUI slot was occupied.
