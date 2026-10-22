# Numerical handoff integration and full regeneration

Baseline: main `d5c820380d4bed50fb45ef3d5399175586a8f094`, clean.
The user authorizes integrating Claude's AeroNumericsLab delivery, regenerating
all aircraft and stores, and running the complete supported test suites.

## Ownership and execution

The LAB is a standalone research folder, read-only during this integration.
No other worktree or `.codex` directory is inspected. No external worker is
contacted. There is no active SDK native build/test or Viewer process at entry;
this campaign holds the serial native build/test slot until its final handoff.

Write allowlist: `tools/validation/missile/` (strict readers, delivery-derived
regressions, numerical candidate patches and provenance),
`cmake/ArchitectureChecks.cmake` (test registration), this plan,
`docs/sdk/missile-flight-validation.md`, `docs/status.md`, generated aircraft
products under `aircraft/`, generated store drag fields under `assets/stores/`,
and the tool-owned `docs/hangar-stores-build.{md,json}` and
`docs/hangar-effects.md`, plus `THIRD_PARTY_NOTICES.md` for imported patch
attribution. Proven generator defects
may be repaired in `tools/models/hangar/` only with regressions first.

The resource audit reproduced a stale B-52H RCS table with the unchanged native
verifier. The integration additionally owns `tools/validation/resources/` and
its two registrations in `cmake/ArchitectureChecks.cmake`: a persistent
model/table byte-identity check and independent synthetic regressions. It
follows the existing native bake/verification provenance and noise criteria;
it does not redefine electromagnetic accuracy or change runtime lookup rules.

## Sequence

1. Verify selected delivery files against their SHA256SUMS. Integrate the
   required-schema correction and reproduce the continuation defect before
   applying it. Import the active-wake containment check and R2 source patch,
   retaining their source identities and independent negative controls.
2. Reassess retained native numerical results using the integrated readers.
   Preserve the coarse zero-angle negative-drag failure and the absence of a
   spatial-convergence family. Improved iteration convergence does not admit
   aerodynamic tables or establish superiority over the current flight model.
3. Reconfigure/build the maintained UCRT64 presets serially. Recreate missile
   and other store designs from their owners. Force a complete aircraft campaign
   with raw per-design logs and a store campaign with the owning tool's batch
   record, child exit codes and per-stage JSON; regenerate registrations/effects.
   Do not invoke calibration or change physical parameters to clear failures.
4. Restage the regenerated products into each build. Discover and run the full
   Release and Headless CTest inventories with bounded timeouts,
   `--no-tests=error`, raw logs and JUnit results. Investigate real failures,
   preserving original assertions and failed evidence.
5. Record exact generated changes, source/runtime provenance, current nonzero
   counts and remaining physical or software failures. Inspect the final diff.

## Numerical admission boundary

The latest delivery is `completion-revision`. It reports panel closure and
modified VSPAERO R2 convergence/containment as passing. Its SU2 coarse alpha-0
wind drag remains negative. Candidate implementations and results belong in the
project with explicit status, independently of acceptance as runtime model data.
No failed result, threshold relaxation, hand-edited generated XML or fabricated
equipment parameter is authorized by this integration.

## Full-suite software follow-up

The first complete inventories finish at 905/909 Release and 831/835 Headless,
with no skips and the same four failing cases. Both original showcase gates
pass; their recordings retain SHA256
`a04d9dc1f5054cd0d736b68d3631996aead09e96f5e702fce3b4ed6a56834093`.
All 1,057 frozen source inputs and four sets of 813 staged runtime files remain
unchanged through that first run. Main remains `d5c8203` before this follow-up.

This window additionally owns these narrow, evidence-backed SIM-M03 repairs:

- `src/simulation/runtime/world/World.h`: private Entry bindings and
  `World::captureMatter`'s private argument list.
- `src/simulation/runtime/world/World.cpp`: capture call sites and physical
  damage/cache restoration in `World::resetVehicle`.
- `src/simulation/runtime/world/WorldDamage.cpp`: property-binding reuse in
  `World::captureMatter` and its internal callers. Sampled values, availability
  checks, units and finiteness requirements remain unchanged.
- `tests/simulation/test_damage.cpp`: a regression using real nose separation,
  native pilot loss, physical reset and exact fresh-flight comparisons.
- `assets/stores/aim-120c.json`: only two source-category labels, mapped to the
  existing provenance vocabulary; every parameter and source description stays
  unchanged.
- `tools/models/hangar/tests/test_missiles.py`: repair synthetic card resolution
  and the selected motor-factory mock boundary. All 128 original assertion-call
  ASTs remain unchanged; the candidate aggregate passes 43/43.

The existing SDK bitwise-reset and zero-allocation assertions are retained.
The new crew-loss case must fail against the uncorrected implementation before
the physical flag is restored. Property handles follow the existing per-Entry
binding pattern; values are read afresh, invalid bindings remain retryable and
reset invalidates the cache. No grant, task, controller, MA binding, episode
contract, public signature/layout or generic owner implementation changes.
The full suites are rerun after repair and runtime restaging. Initial failed
JUnit files remain separate from final results. The LAB stays read only.

The parallel focused rerun additionally reproduces a fixture resource conflict:
`showcase_native` and `showcase_event_chain` both publish `native-showcase-test`.
The second publisher is correctly refused. `tests/CMakeLists.txt` is added to
the allowlist solely for a shared CTest `RESOURCE_LOCK`; both fixtures keep
their original scenes, assertions and timeouts. Re-run the pair with the
original concurrent scheduler after adding the lock.
