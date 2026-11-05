# SIM-M03 development checkpoint — 2026-11-04

Status: **not accepted**. Development was paused for a checkpoint commit and
push. This checkpoint does not satisfy the complete SIM-M03 acceptance
contract. Acceptance flags remain false. Continue on `main`; the preceding
committed baseline is `f8bd88224c1d36a1efb6017580c852e177d9c095`.

## Included work

The checkpoint retains native mass/inertia reconciliation, queued-store
ownership and release-state corrections, normal 53-store mass serialization,
liquid integration/support corrections, measured guidance/control references,
explicit authored source policies for 19 field and four connected-ring models,
finite-surface/triangle collision corrections, and private transactional
finite-rod contact history. It also retains the versioned moving thermal
inverse, certified conic moments, guarded exact-order source sums and the V5
field-edge numerical domain. Existing V2 and materialized V4 calibration
operators are preserved separately.

The explicit four-profile V2 compatibility generator was actually rerun.
Its 339 original parameter records and all 36 component definitions are
unchanged; only descriptive production-model metadata changes in three cards.
The normal CMake compatibility verifier remains enabled.

## Recorded verification and its limits

Every result below belongs to its own executable and input epoch. It is not a
new complete population result. Raw evidence remains in the ignored local
`build/architecture-reviews/sim-m03-current-20261104/` directory; it is not
distributed as source. The companion checkpoint JSON binds the selected local
receipts. Rebuild and rerun after checkout or further changes.

| Gate | Recorded result | Scope |
| --- | --- | --- |
| Original ownership-partition regression | Same 147 assertions: six failures before repair, zero afterward | Native method, not a model chain |
| Original shell entry/exit regression | Same 52 assertions: six failures in the first history integration, zero after the earliest-time slice correction | Original predicate retained |
| Contact-history controls after slice correction | 23 JUnit sections, 192 assertions, zero failures | Includes call boundaries, reentry, material differences and unknown-ordinal compatibility |
| Complete damage source-owned run before slice correction | 91/91 executed; 90 passed, one failed | The later 52-assertion spot repair does not replace a new complete run |
| Retained V2/V4 and explicit V5 basis checks | Four cases, 14662 assertions, zero failures | Latest executable `4c5ef0d6…` |
| Registered exact-order source-wave method | Seven groups, 17638 assertions, zero failures | Normal CTest entrypoint |
| Actual field-edge/native and independent integral comparison | Four JUnit sections, 49188 assertions, zero failures | Three captured fields and 18 independent high-precision values |
| Latest original captured-moving thermal gate | Timeout at 180.008611 seconds | No timeout or search reduction; gate remains red |
| Latest R-27R chain | Aborted after 12040 assertions with one JUnit error | Ownership handoff refusal; not a completed energy-predicate result |
| R-27ET and R-27T individual chains | One energy-predicate failure each; actual detonation and zero internal work | 570.273 and 497.218 seconds respectively, at their recorded source epoch |

The latest thermal edge-domain run finishes the first two captured inferences
in 20.080 and 11.163 seconds; the complete three-case gate still times out.
The selected materialized fast-field benchmark measures about 345 ms to
203 ms over 512 changing-weight slots, including cold setup and fallback.
The separate scalar contraction candidate is not active production code.

Earlier AIM-120D/MICA-IR individual passes, the historical 16/24 complete
population and the independent 216 method projections remain historical
results with their own epochs. They do not establish current 24/24 or admit
the 216 model/component cells.

## Concrete restart points

1. **World material ownership transfer:** the current handoff contains all
   detached nose panel indices, but the history rebind assumes every index
   owns a `ContactMesh`. Real finite-surface panels cause
   `material contact ownership handoff refused`. Validate actual panel kinds
   in the trusted native builder and project only applicable mesh-backed
   transfers; preserve rejection of malformed or one-sided mappings. The
   latest unchanged R-27R fixture is the actual failing World witness.
2. **Moving-origin contact continuity:** stored history advances a target
   origin incrementally; the next query recomputes it from a captured anchor
   and age. One-ULP regrouping differences can discard a valid fresh episode.
   Use the canonical represented motion expression without introducing a
   tolerance or pretending to provide rotational CCD.
3. **Arithmetic-domain guard:** the separate fail-closed Cramer-certificate
   FPU guard is frozen but unapplied. Its candidate is not part of this
   checkpoint's active implementation.
4. **Thermal performance:** keep the original 180-second gate and all
   likelihood, mode, rate and error bounds. V5 removes an obsolete quadrature
   veto only after its own conic certificate succeeds. Remaining clipped
   topology cases still fall back; further work must use actual requests and
   measured costs.
5. **Carrier/material traversal:** the R-27ET ring's native miss is supported
   by independent geometry. Rigid-contact dissipation is not component work.
   A conditional finite-carrier path reaches fuel/engine geometry before the
   end-of-C boundary, but surviving penetration and work are unproved. Source
   TOI mass/inertia/tank observations and a conservative coupled material
   contract are needed; do not substitute C values, add energy credit, or
   double-count the emitted case/contact impulse.

No World handoff/canonical-pose repair or carrier implementation was applied
after this pause. Frozen research candidates and their reviews remain local
under the dated evidence root. In particular, the seven-path source-geometry
reproducer, six-path thermal-table reproducer, scalar public-delivery overlay
and passive source-material observer are not integrated merely by this commit.

The complete current 24-model chain, all 216 cells and their three matrices,
both preset regressions, installed/public consumers, 53 stores/35 airframes,
fullscreen primary-resolution 1x presentation and replay remain unaccepted.
Normal RQ-4B model generation also needs a fresh run after the support repair.
Packaging must include every changed JSBSim source/header, including the
remaining `FGEngine.h` notice/source-delivery check, before package acceptance.
