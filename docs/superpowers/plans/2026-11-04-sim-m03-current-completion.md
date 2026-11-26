# SIM-M03 current production completion

## RQ4B inferred liquid-vessel closure window — 2026-11-25

Continuing on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9` (preserving
`assets/stores/cj-10a.json` and holding the exclusive serial native slot; all
24/24 Release AAM models, JSBSim LGPL notices, and preflight Hangar/component
unit suites verified):

Allowlist:
- `tools/models/hangar/hangar/liquid_vessels.py`:
  - `_strict_partial_box`: O(1) exact zero-epsilon `bounding_box()` partiality
    check prior to the 64-vertex cap (guarded for test doubles without
    `bounding_box`), plus bounded exact `Fraction` separating-plane fallback
    over the 6 nearest vertices to any missing box corner (zero tolerances;
    retains original hard negative-volume refusal when unproven).
  - `_inscribed_vertical_band`: exact outward box-face skip (scale-safe across
    all finite float64 scales) and referenced-vertex subsetting preserving
    bit-identical dyadic output.
  - `Policy._certificate`: check `box ^ self.root`, `_represented_box`, and
    `_strict_partial_box` prior to `box - self.root`, preserving exact return
    values and refusal semantics.
  - `_pack_shared`, `_pack_shared_side`, `_shared_vessel_worker`: exact
    per-wing-pair (`left wing` `{0, 2}`, `right wing` `{1, 3}`) search with
    largest-potential-volume priority, deferred root-strip `Policy._certificates`
    evaluated freshly on every contained cell, `_represented_box`-only optional
    column-core admission, per-section `screened_cells`/`assignment_edges`
    reconciliation in `ledger['sections']`, and optional 2-core spawned wing
    workers via `liquid._process_owner_jobs` under shared `_OwnerBudget`
    producing bit-identical canonical `[2, 3, 0, 1]` output within all unchanged
    default limits (`max_seconds=300.0`, `max_cells=65536`,
    `max_screened_cells=1048576`, `max_depth=60`).
- `tools/models/hangar/hangar/liquid.py`:
  - `produce` and `_pack_owned`: bind inferred-vessel source inputs into the
    worker recipe and pass `_recipe` into `liquid_vessels._pack_shared`.
  - `_process_owner_jobs`: raise plain owner job batch JSON ceiling from
    `32768` to `262144` bytes so multi-section aircraft (`kc135r`) whose
    repeated per-owner bindings/exclusions exceed 32 KiB dispatch cleanly.
  - `pack`: compute `overlap = box ^ candidate_domain` once and classify
    containment via `liquid_vessels._cell_contained` (`_represented_box` /
    `_strict_partial_box` before `box - candidate_domain`), eliminating the
    redundant second Boolean on empty/partial/contained cells while preserving
    bit-identical outputs (gives `u2s` large wall-clock margin under 300 s).
- `aircraft/uh1h/tail-damping.json`: refresh via normal `hangar uh1h fly`
  stage so `source_hashes` match the current Hangar method epoch.
- `tools/validation/weapon_quality/campaign.py`: pass `method_environment` to
  the `liquid-population` admission stage so `rq4b` inferred-vessel verification
  can resolve `hangar.shape.meshkit` and `HANGAR_MESHKIT_DIR`.
- `tools/models/hangar/tests/test_damage.py`: validate and strip the
  post-`damage.write` `liquid_geometry_v1` annotation written by `liquid.write`
  when comparing `damage.write` output against committed `c172.damage.json`.
- `assets/stores/{aim-9m,magic-2,pl-10,pl-8,r-27et,r-27t,r-60m,r-73}.json`:
  canonicalize the 9 non-standard `evidence_kind` metadata strings
  (`engineering_derivation`/`assumed` -> `unclassified`, `manufacturer` ->
  `manufacturer_publication`) to `provenance.SOURCE_KINDS` and resync
  `tests/simulation/fixtures/weapon-v2/`.
- `tools/models/hangar/tests/test_liquid_partial_support.py` and
  `tools/models/hangar/tests/test_liquid_vessel_packing.py`: add regression
  coverage for the captured 9-vertex/14-triangle missing-corner `rq4b` witness
  and serial/parallel parity.
- `tests/simulation/test_liquid_vessels_native.cpp`: set `rq4b` initial
  airspeed at 6,000 m from `220 m/s` (copied from `f16c` in
  `test_liquid_native.cpp`, exceeding `rq4b` `max_tas_ms ≈ 192 m/s` and
  causing immediate structural nose/tail/wing breakup on step 1) to `122 m/s`
  (inside `rq4b`'s valid flight envelope `[82, 192] m/s` at 6,000 m); retain
  all tolerances, assertions, and native physics paths unchanged.
- `tests/simulation/test_liquid_population_native.cpp`: mark unused helper
  overload `coordinate(V3&, unsigned, double)` `[[maybe_unused]]` for `-Werror`,
  and perform the matching post-reset `release()`/`select()`/`burst()` capture
  at lines 409–412 (mirroring the `fresh` comparison at lines 413–415) since
  `World::appendLiquidMaterialV1` only populates `liquidMaterialDiagnosticV1`
  during a detonation/fragment encounter.
- `tests/simulation/fixtures/weapon-v2/`: regenerate the 4 frozen V2
  compatibility fixtures and `manifest.json` via
  `tests/simulation/fixtures/weapon_v2_assets.py` after the approved
  `equipment_models.json` and `assets/stores/aim-9m.json` provenance updates.

## Equipment selection, JSBSim LGPL notices, and 24/24 closure window — 2026-11-25

Following user approval on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9`
(holding the exclusive serial native slot; 22/24 Release AAM models already
passing after `aim-9m`, `magic-2`, `r-60m` recovery and 1-ULP pointing
normalization fix on `pl-8`):

Allowlist:
- `tools/models/hangar/hangar/data/equipment_models.json` and generated
  `src/contracts/equipment/EquipmentModels.h`:
  - Select compiled `fsim.control.actuator_aware_cascade.v1` for
    `equipment.pl-8.autopilot` (retaining unchanged `0.04 s` actuator lag,
    `4.0 rad/s` rate limit, `0.12 s` rate time, `0.40 s` incidence time, and
    `15 deg` max incidence; resolves Mach-2.5 `Ma/J = -1001 s^-2` closed-loop
    dominant time-constant degradation from `0.40 s` to `1.55 s`).
  - Select compiled `native.fuze.turning_cpa.v1` for `equipment.aim-120d.fuze`
    (matching the same-family AMRAAM `aim-120c` source-backed TDD / turning-CPA
    mechanism, retaining unchanged `11.472 m` radius and `1 s` arming) and for
    `equipment.pl-8.fuze` (approved by user after `pl-8` achieved a `< 0.8 m`
    bullseye under `actuator_aware_cascade.v1` and encountered the identical V0
    `swept_path.v1` pre-nose clamped endpoint `0 J` boundary; retaining
    unchanged `10.317 m` radius, `1 s` arming, `0 s` delay, and disclosed
    MIL-HDBK-1211 turning-CPA / assumed station provenance).
- `assets/stores/pl-8.json` and regenerated `aircraft/stores/pl8/pl8.xml`:
  update autopilot provenance/summary and regenerate JSBSim control fingerprint
  through the normal Hangar entrypoint.
- `assets/stores/aim-120d.json`: update fuze parameters (`model_version = 1`,
  `algorithm = 1`, `delay_s = 0`, AMRAAM family station `x = 1.46 m`), sources
  (`mil-hdbk-1211`, `hughes-amraam-cutaway`), component descriptor bindings,
  and uncalibrated summary text.
- `cmake/Install.cmake` (`fsim_install_notices`) and `THIRD_PARTY_NOTICES.md`:
  install all patched JSBSim files (`models/propulsion/FGEngine.h`,
  `models/propulsion/FGEngine.cpp`, `models/propulsion/FGTurbine.cpp`) under
  `share/doc/flightsim/jsbsim-changes/` and accurately document `FGEngine.h`
  and `FGEngine.cpp` edits.
- `tests/simulation/test_thermal_reticle_receiver.cpp` and
  `tests/simulation/test_payload_effect_adapters.cpp`: add unit coverage for
  1-ULP pointing normalization, coarse-dwell LOS-rate bracketing, rotation-
  invariant closing scale, Pass 8–11 conic/parallel exactness, and `pl-8` /
  `aim-120d` method/sensitivity checks.

## Reticle prior continuity and V5 mode/bank throughput window — 2026-11-24

The fresh 24-model AAM Release full-chain roster (`full24-release/full24.log`,
`5293.95 s`) closed **20 / 24** models (including all 4 SARH models plus
`r-60m`, `r-73`, `r-27r`, `r-27et`, `r-27t`). Profiling the 3 captured-moving
scenes (`profile-v5/prof.log`) and inspecting the 4 remaining failures isolated
two concrete defects and one exact throughput bottleneck:

1. **Coarse-capture erasure of valid causal prior (`pl-8`, `aim-9m`)**:
   `EngineeringSeekerV3` required `!filteredQuadrantUnresolved_` and
   `filtered_->sampleTimeS == priorRecord->captureTimeS` to build a
   `ThermalReticleMeasuredPriorV2`. A single intervening coarse capture
   (`res=True, pre=False`, e.g. `pl-8` at `16.1000 s` and `aim-9m` at
   `16.4000 s`) overwrote `filtered_` and wiped out the still-valid causal
   precise prior from 33 ms earlier. Retaining the admitted inertial
   `(direction, directionRate, localId)` from the exact `priorRecord` epoch
   preserves strict causality (`available <= (*window)[0]`, age `<= memoryS`)
   across an intervening coarse dwell without mixing epochs.
2. **Mode/bank/trajectory thrashing and unbounded prior cutoff fanout
   (`aim-9m`, `magic-2` 1200 s timeouts)**:
   On the 16-thread host, `maximumPhaseWorkersV5` was capped at 8, the mode-bank
   scan iterated geometries outer / cutoffs inner (rebuilding `phaseBankV5` per
   geometry and evicting it on every `information()` `cutoff +- h` probe),
   `prepare()` recomputed identical perturbed trajectories across 600+ cutoffs
   during `modeCache`, `dampedBoxStep` lacked the exact unconstrained-face
   interior shortcut already used in `boxStep`, and `priorStart` fanned out
   across 6 cutoffs (18 starts) on unresolvable captures.

Continuing on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9` with the
exclusive serial native slot. Allowlist:
- `src/simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h`
  and `.cpp`: store exact-epoch admitted `(localId, rateAvailable, direction,
  directionRate)` and strictly closing rotation-invariant inverse-diameter rate
  (`invDiameterRatePerS` from $D=\sqrt{w^2+h^2}$ across two consecutive
  same-track precise captures, applied isotropically only when closing) on
  `LastPreciseReticleV2`; retain `opticalPoseHistory_` across `memoryS`;
  compare `priorRecord->pointing` against `unit(previousPose->pointing)` (the
  exact receiver-normalized pointing) to avoid 1-ULP double-normalization
  mismatch; and bracket LOS rate across an intervening coarse dwell strictly
  from two resolved same-track precise exposures within `memoryS` (never
  differencing the coarse capture).
- `src/simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.cpp`:
  bit-identical parallel forward phase rendering across `c.samplesPerScan` in
  `exposeWithMeasuredPriorV2` (preserving exact per-phase `k`/`pixel` sum order,
  caller FPU/SSE rounding environment, and sequential `rng_` draws); exact
  unconstrained interior face-0 shortcut in `dampedBoxStep`; exact-zero
  pixel skip in `spotJacobianV1`; exact `levelBounds` sky-backed /
  ground-backed linear basis shortcut in `fitHorizonWithMeasuredPriorV2`
  (identical to `MovingRadiatorInverseV2` `sourceRelativeGround`, calling
  `unitGroundFieldMemoV2` with `groundField` callback and linear waveform
  subtraction only when the horizon intersects the support box); O(1)
  `calibrationIndex`, 4-entry `groundFieldCache`, exact horizon-box kink
  splitting in `unitGroundFieldMemoV2`, bit-identical parallel pure-optics
  Jacobian evaluation in `fitHorizonWithMeasuredPriorV2::trust()` (inheriting
  caller FPU/SSE rounding environment), and cap `priorCutoffs` to
  `(priorCutoff, nullCutoff)`.
- `src/simulation/mission_systems/components/implementations/seeker/MovingRadiatorInverseV2.cpp`:
  `maximumPhaseWorkersV5 = 16`, 16-entry exact-cutoff `phaseBankCacheV5`,
  parallel column pre-warming via `Bank::ensureColumnV5` before phase loops,
  O(1) `groundIndex`, 4-entry `groundFieldCache`, zero-skip `wave()`,
  16-entry `modeCache` `Trajectory` LRU, parallel V5 `prepare()` across
  `phasePoolV5` (disjoint per-phase slices, bit-identical output), cutoff-outer
  / geometry-inner mode scan with early exit on unresolved/non-finite envelope,
  profile-seed check before mode bank scan, and cap `priorCutoffs` to
  `(priorCutoff, nullCutoff)`.
- `src/simulation/mission_systems/components/support/MovingConicMomentsV4.h`
  and `.cpp`: `constexpr` binomial lookup table for `n, k <= 8`; exact
  uncut-column V5 precomputation (`uncutMass0/3/7`, `uncutHoriz3/7`); hoisted
  jet powers, shared adjacent row edges, and single-entry `bound[jetDegree]`
  in `buildColumn()`; additive `Bank::ensureColumnV5(unsigned col) const`.
- `tests/simulation/test_thermal_reticle_receiver.cpp`: additive prior-continuity
  control across an intervening coarse capture.
- `docs/superpowers/plans/2026-11-04-sim-m03-current-completion.md`,
  `docs/sdk/sim-m03-full-chain-current.md`,
  `docs/research/sim-m03/thermal-reticle-method.md`, and raw receipts under
  `build/architecture-reviews/sim-m03-current-20261124/`.

## Hangar cruciform wing-body dihedral symmetry repair window — 2026-11-24

The R-60M causal measured-prior + roll-aware aspect start + Moving V2 prior
window passed all 4 unit suites (84/84 assertions, 0 warnings) and recovered
continuous R-60M scalar tracking through 16.1000 s (203.70 m range; residuals
183.3, 114.6, 113.2, 108.2, 111.5, 97.0). In the fresh unchanged R-60M Release
encounter (`r60m-current/run.json`, 72,321 assertions, 4 failures, 987.114 s,
min sampled range 12.881555 m), inspection of `kinematics.csv`, `control_pre_json`
and `r60m.xml` isolated an exact aerodynamic symmetry defect:

- `tools/models/hangar/hangar/aero/model.py::_wing_body_dihedral()` applied
  DATCOM 5.1.2.1 high-wing interference (`clb_wing_body`) without checking
  `wing.kind == "wing"`. For all 24 symmetric cruciform AAMs and Mk 82 (which
  have only `kind = "fin"` surfaces and no `kind = "wing"`), `aircraft.wing`
  falls back to the first upper fin pair (`z > 0`), injecting a phantom
  `Cl_beta ≈ -0.884 / rad` that violates exact `z -> -z` symmetry. On R-60M
  (free-rolling two-axis canard, `cl_aileron = 0`, `Ixx = 0.0462 slugs*ft^2`),
  1–2 deg sideslip spun the missile to 15.66 rad/s, reversing achieved lateral
  acceleration relative to guidance demand (`cos = -0.59`).

Continuing on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9` with the
exclusive serial slot. Allowlist:
- `tools/models/hangar/hangar/aero/model.py`: guard `_wing_body_dihedral()` on
  `wing is None or wing.kind != "wing"`. Preserves all 33 fixed-wing aircraft
  and `cj10a` unchanged.
- `tools/models/hangar/tests/test_methods.py`: additive RED/GREEN symmetry test
  for symmetric cruciform missiles (`r60m`) and high-wing retention control
  (`c172`).
- Regenerated store flight models under `aircraft/stores/` via the normal
  Hangar `aero build --force` entrypoint (starting with `r60m` verification,
  then affected cruciform stores before the complete 24-model roster).
- `docs/sdk/sim-m03-full-chain-current.md`, this plan, and raw receipts under
  `build/architecture-reviews/sim-m03-current-20261124/`.

## R-60M causal measured-prior moving-inference window — 2026-11-24

Pass 2 of the V5 throughput window passed all 15 focused JUnit sections
(97,116 assertions in 1.360 s), passed the unchanged 180-second three-scene
`[thermal-reticle-captured-moving-v2]` gate in 103.627 s (48 assertions, 0
failures), and passed the unchanged **PL-8** Release full-chain encounter in
951.308 s (59,820 assertions, 0 failures, 77,504.81 J internal component work).

Resuming on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9` with the
exclusive serial native slot for R-60M near-approach scalar tracking recovery.
Allowlist:
- `src/simulation/mission_systems/components/implementations/seeker/MovingRadiatorInverseV2.h`
  and `.cpp`: additive optional `const ThermalReticleMeasuredPriorV2* prior = nullptr`
  on `moving_radiator_v2::infer`; exact causal endpoint/support/cutoff
  continuation in `Inverse::optimize()` only when cold-start residual exceeds
  the unchanged `residualBound`, with exact derived `maximumGrounds` budget.
  No threshold, noise, null, physical-budget or mode-envelope change.
- `src/simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h`
  and `.cpp`: forward `prior` into `inferMovingMeasuredV2`, plus additive
  optional `ThermalReticleMovingDiagnosticV2*` output on
  `exposeWithMeasuredPriorV2` / `inferMeasuredWithPriorV2` preserving all
  existing signatures and V1 layouts.
- `src/simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h`
  and `.cpp`: retain last accepted precise reticle prior metadata synchronized
  to `filtered_->sampleTimeS` and use Moving V2's accepted cutoff when Moving
  V2 resolved the capture; clear cleanly in `resetTracking()`.
- `tests/simulation/test_thermal_reticle_receiver.cpp`: additive captured
  R-60M `15.890176 s` / `15.995733 s` moving measured-prior RED/GREEN
  regressions and source-free / invalid-prior negative controls.
- `docs/superpowers/plans/2026-11-04-sim-m03-current-completion.md`,
  `docs/research/sim-m03/thermal-reticle-method.md`,
  `docs/sdk/sim-m03-full-chain-current.md`, and raw receipts under
  `build/architecture-reviews/sim-m03-current-20261124/`.

## V5 conic region-hoisting, SSE2 ball arithmetic and worker pool window — 2026-11-24

Resumed on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9` with the
November 5–11 working changes intact (`assets/stores/cj-10a.json` line-ending
change preserved). Process inspection confirmed no competing native build, test
or Viewer process. Root holds the exclusive serial native slot.

Implementation allowlist:
- `src/simulation/mission_systems/components/support/MovingConicMomentsV4.h`:
  additive private `FieldWorkspaceV5` and `Bank::evaluateFastIntoV5`.
- `src/simulation/mission_systems/components/support/MovingConicMomentsV4.cpp`:
  V5-only certified early-stop trig/horizon helpers, per-region hoisted edge
  roots and midpoint topology certification, stack-buffered zero-allocation V5
  evaluation, pure binary64 outward-certified `Ball`/error-bound arithmetic and
  `evaluateFastIntoV5`. Preserve V4 (`Bank::evaluate`, `moments`, calibration
  entrypoints), V5 `1e-9` field budget, `5e-19` omitted-tail allowance,
  `GaussianMemoV5` exact semantics and non-nearest/unsupported-arithmetic
  refusals.
- `src/simulation/mission_systems/components/implementations/seeker/MovingRadiatorInverseV2.cpp`:
  private persistent phase worker pool (at most 8 workers) and per-worker
  `FieldWorkspaceV5` reuse in `parallelRegionalPhasesV5` and `regionalPhase`,
  removing shared-mutex `GaussianMemoV5` contention from the parallel V5 path.
  Preserve exact chronological subexposure/pixel accumulation, thresholds,
  search domain, hypothesis enumeration and World RNG/worker independence.
- `tests/simulation/test_thermal_reticle_receiver.cpp`: additive V5 workspace
  exactness control; retain all existing assertions and predicates unchanged.
- `docs/superpowers/plans/2026-11-04-sim-m03-current-completion.md`,
  `docs/research/sim-m03/thermal-reticle-method.md`,
  `docs/sdk/sim-m03-full-chain-current.md`, and raw receipts under
  `build/architecture-reviews/sim-m03-current-20261124/`.

## V5 arithmetic throughput window

The user resumed SIM-M03 on `main` at
`57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9`. No competing native process was
present. The initial implementation scope is
`MovingConicMomentsV4.cpp` at `upperPositive`, `midpoint`, `radiusUpper`,
the `Ball` division and normal polynomial, `Bank::Impl::finish` and `sum`,
and V5 call sites. Preserve the existing V4 arithmetic. Elide a padding
operation only when round-to-nearest makes it an identity. Compare against
the pre-edit Release library over the captured conic input sweep, then run
the original focused and 180-second captured-moving tests. Evidence goes to
`build/architecture-reviews/sim-m03-current-20261111/conic-arithmetic/`.

After that execution ends, extend the same window to `Bank::prepareSharedV5`,
the column publication in `Bank::Impl`, `Inverse::parallelRegionalPhasesV5`
and its cutoff bank members, and a concurrent conic control in the existing
thermal test. Share one cutoff bank across phase workers. Publish the degree
three data once and retain it when the degree seven extension is built. Compare
the same requests with independent banks, then run the unchanged captured gate
under `build/architecture-reviews/sim-m03-current-20261111/shared-conic-bank/`.

After that execution ends, extend the window to the V5 `Inverse` constructor,
`groundField`, `regionalPhase` and `parallelRegionalPhasesV5` for a bounded
list of nonzero mask entries and a shared ordered phase accumulator. Keep the
dense loop for uncertified optics or fields. Run existing exact phase controls
and the unchanged captured gate under
`build/architecture-reviews/sim-m03-current-20261111/sparse-regional-phase/`.

Restore the shared-bank source after the sparse experiment. The next native
run is the original model-identified `r-60m` Release fixture at 1200 seconds.
Write its measured status and the original twenty-model failure inventory to
`docs/sdk/sim-m03-full-chain-current.md`. Keep raw R-60M evidence under
`build/architecture-reviews/sim-m03-current-20261111/r60m-current/`.

## Resumed Gaussian memo verification — 2026-11-11

Resume on `main` at `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9` with the
November 5 working changes intact. Process inspection found no competing native
build or test. The exclusive write scope is the existing Gaussian memo test in
`tests/simulation/test_thermal_reticle_receiver.cpp`, profile-only memo counters
at `Inverse::basis` in `MovingRadiatorInverseV2.cpp`, and this plan, the thermal
method document and `docs/sdk/sim-m03-full-chain-current.md` for the measured
result and dated status. The shared memo test must keep
one instance alive across different cutoffs. Run the existing Release target
and the original three captured-moving scenes at the existing 180-second limit.
Record the actual result under `build/architecture-reviews/sim-m03-current-20261111/`.
Do not change the stimuli, arithmetic policy, acceptance criteria or flags.

## Resumed material-owner repair window — 2026-11-05

The user resumed development from `main` commit
`57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9`. Root exclusively owns the serial
native build/test slot; process inspection found no competing project runner.
The unrelated `assets/stores/cj-10a.json` line-ending change is preserved.
The initial implementation allowlist is:

- `src/simulation/runtime/world/WorldDamage.cpp`: `World::separatedAt` native
  material-handoff preparation only.
- `src/simulation/mission_systems/damage/MaterialContactHistoryV3.h` and
  `src/simulation/mission_systems/damage/Damage.cpp`: private handoff panel
  preparation and canonical captured-motion contact-boundary pose only.
- `tests/simulation/test_damage.cpp`: additive mixed native geometry and
  captured-motion continuation regressions; retain existing assertions.
- This plan, `docs/research/sim-m03/finite-rod-material-contact-v3.md` and
  `docs/sdk/sim-m03-full-chain-current.md`: mechanism and actual new results.
- Raw current receipts under
  `build/architecture-reviews/sim-m03-current-20261105/`.

Extract the existing handoff preparation without repairing its behavior, run
the mixed finite-surface/contact-mesh regression RED, then correct the trusted
producer. Reproduce the original R-27R World chain without changing its scene,
seed, thresholds, timeout or outcome predicates. Existing checkpoint results
remain dated history; acceptance flags stay false until the complete contract
passes. No other implementation owner has an active write or build window.

## Serial conic row-work window — 2026-11-05

After material-owner native execution ends, root retains the sole native slot
for a fresh unchanged 180-second captured-moving RED. At the same main HEAD,
the additional write allowlist is
`src/simulation/mission_systems/components/support/MovingConicMomentsV4.cpp`
(`Bank::Impl` moment contraction and `Bank::evaluateFastMoments` only),
`tests/simulation/test_thermal_reticle_receiver.cpp` (additive independent
Gaussian/retained-operator controls), and
`docs/research/sim-m03/thermal-reticle-method.md`. Preserve V2/V4 calibration,
V5's existing field budget, all inference/search bounds and original captured
fixtures. The candidate reuses identical per-column stored moments and admits
only certified Gaussian row tails within the existing omitted-tail budget.
No native build inputs are edited while the RED process runs.

The same private C++ file's V5-only helpers `fastGaussianPrimitive`,
`fastGaussianCumulative`, `normalBallPolynomial`, `fastNormalAtMagnitude` and
`fastNormalCdf` are included in this exclusive window. They reuse the retained
coefficient table and existing arithmetic certificates. Independent direct
integrals, field bounds and V2/V4 byte controls precede each runtime gate.

The same file's V5-only `Bank::Impl::clippedRoots` and `clippedFastColumn`
extend the integral across actual rectangle/horizon crossings using enclosed
quadratic roots and explicit uncertain-strip bounds. Additional reference
delivery paths are `tests/simulation/fixtures/thermal_conic_v5_clipped.json`,
`tests/simulation/fixtures/thermal_conic_v5_clipped_reference.h` and
`tools/validation/weapon_quality/conic_clipped_reference.py`. They preserve six
actual optical request stimuli and independently reproduce 120-decimal
conic/Gaussian reference values. They do not contain a native outcome override.

Root additionally owns
`src/simulation/mission_systems/components/implementations/seeker/MovingRadiatorInverseV2.cpp`
for bounded profile-only mode-bank dimensions and existing envelope status in
`Inverse::run`. No inference criterion or mode enumeration is changed by that
observation. A third-scene-only bounded diagnostic is not the original complete
three-scene gate.

The inverse implementation window additionally covers the private V5 regional
phase batch, its per-worker fixed-optics banks, and associated profile counters.
At most eight independent phase workers may evaluate pure optical fields; each
phase keeps its original pixel/subexposure operation order and the root retains
the original temporal filter and every hypothesis. World worker settings and
RNG streams stay unchanged. Exact serial/materialized phase checks precede the
original complete 180-second gate. Worker arithmetic inherits the admitted
caller environment, mutable numerical banks are not shared, and all jobs join
before inverse state publication.

The conic support header is additionally allowlisted for an additive private
`GaussianMemoV5` and `Bank::evaluateFastMemoV5` interface. Its implementation
reuses exact Gaussian row/moment factors across cutoff values, never terrain
coefficients or final fields. An explicit instance owns the bounded concurrent
cache; every input key is exact, misses execute the complete computation, and
unsupported arithmetic bypasses it. Serial direct-versus-memo IEEE-byte checks,
zero capacity, eviction, configuration mismatch and original phase equality
must pass before another complete runtime gate. Existing public SDK signatures
and existing conic entrypoint signatures remain unchanged.

**Goal:** Complete the existing SIM-M03 production contract for every maintained
model and applicable component without changing accepted M01/M02 scope, original
scenarios, tolerances or native outcome meanings.

**Architecture:** Continue the production closure and complete-quality plans.
The SDK owns simulation; clients display actual native observations. Preserve
public ABI, existing providers and explicitly selected versioned mechanisms.

**Baseline:** `main` at `f8bd88224c1d36a1efb6017580c852e177d9c095`;
the starting working tree was clean.

**Contracts:** `docs/sdk/sim-m03-engineering-contract.md`,
`docs/research/sim-m03/component-matrix.md`, and
`docs/superpowers/plans/2026-11-03-sim-m03-production-closure.md`.

## Execution boundaries

Initial process inspection found no competing project configure/build/test or
Viewer process. Root holds the exclusive serial native and GUI slots. Two
independent reviewers have read-only source/evidence windows and may not build,
generate products or launch a Viewer. No external worker is contacted.

The initial write allowlist is this plan and raw evidence under
`build/architecture-reviews/sim-m03-current-20261104/`. Record exact additional
paths and symbols before each implementation edit. Do not create a branch or
worktree. Preserve unrelated changes. Tools and temporary files stay in D:/ENV.

Ruling: continue the existing user-assigned design and acceptance contract;
do not restart requirements approval or replace the complete roster with a
representative subset. Existing historical test totals are not current proof.

## Required work

- [ ] Rebuild the normal current Release preset and reproduce current failures;
  bind actual commands, logs, source and product bytes.
- [ ] Diagnose the unchanged AIM-9M native full-chain case, preserving all
  original physical inputs and terminal/damage predicates.
- [ ] Repair independently demonstrated production defects with a failing
  regression followed by the corrected native execution.
- [ ] Reconcile all-model native, method, installed and visual component proofs;
  complete liquid/model production and all other required populations.
- [ ] Execute the complete campaign with both UCRT64 presets, approved unchanged
  rotor deferrals, actual installed public consumers and current fleet cases.
- [ ] Inspect complete fullscreen primary-display 1x presentation, resolve
  independent review findings and bind reproducible source/package delivery.

## Priority and acceptance correction from the current assignment

The user explicitly orders model/component closure before evidence integration:
recover the original twenty failed models, classify their concrete mechanisms,
then obtain fresh identifiable 24/24 native full-chain execution. Do not change
scenes, tolerances, seeds, thresholds or native outcomes. Current regression
results must be distinguished from the original twenty-model failure record.

All 216 model/component cells require mechanism, parameter provenance, theory
delta, independent method, actual native, installed/public and visual execution,
status and exact artifact hashes. Shared algorithms require an explicit physical
equivalence argument plus individual model executions. Complete the guidance,
zoned-case, swept-path and load-feedback model families with their equations,
states, assumptions, domain, sensitivity and failure modes; selectors are not
substitute implementations. Every family requires an executed independent
open-source, textbook, standard or analytic comparison. Publish theory,
parameter-source and fidelity matrices together, all keyed to the 216 cells.

Source-freeze, provenance and report-join work is subordinate to a concrete
model blocker and otherwise follows model/component closure. The liquid-native
integration draft was frozen without product edits. Normal RQ generation failed
its original positive-volume geometry predicate; its retained raw log is a
separate unresolved model-production result, not current AAM progress.

Final acceptance requires both current full presets, installed consumers,
53 stores/35 airframes, complete fullscreen 1x presentation and replay. Keep
acceptance flags false and do not commit a completion claim before those pass.

## Current evidence

The fresh unchanged 24-model Release diagnostic baseline executes 14 passes
and ten failures. Every baseline model's seven raw artifacts are preserved
under `baseline24/` and remain distinct from later output. The subsequent clean
RK4 numerical repair passes all 44 liquid cases and the original R-27ER chain;
the same clean-source execution still fails AIM-120D and R-27R's component-energy
predicate. Scalar-reticle repairs remain in active RED/GREEN verification. See
`docs/sdk/sim-m03-full-chain-current.md` for exact boundaries and current logs.
Neither a partial rerun nor a unit pass is a current 24/24 or 216-cell admission.
Both full presets, installed/fleet, visual and replay remain required.

## Exclusive scalar-reticle background repair window

At main `f8bd88224c1d36a1efb6017580c852e177d9c095`, the physical reviewer owns
`tests/simulation/test_thermal_reticle_receiver.cpp` for an independent no-source
physical-horizon RED first. After root captures RED, its bounded production
window is only `ThermalReticleReceiverV1.cpp/.h` under the existing seeker
implementation directory and `docs/research/sim-m03/thermal-reticle-method.md`.
Exact before bytes and hashes are retained in
`build/architecture-reviews/sim-m03-current-20261104/reticle-repair/window.json`.
The reviewer runs no native builds/tests, model generation or GUI. Preserve
original edge/clutter cases, thresholds, card inputs and terminal scenes. Root
retains the serial native slot. A compact-template correlation is not sufficient
when its residual rejects that model; the background hypothesis must remain
part of receiver inference. Real target-edge functionality remains required.

## Root full-chain observation extension

At unchanged main `f8bd88224c1d36a1efb6017580c852e177d9c095`, root exclusively
owns the scalar-packet serialization block in
`tests/simulation/test_aam_full_chain_native.cpp`. Add actual precise/coarse
quality and fitted-support observations to the existing model/entity/generation
identified trace. Indeterminate support coordinates remain null with explicit
fit availability. Do not edit scenario setup, seed, timing, tolerances,
assertions, execution predicates or native outcomes. This directly supports the
seven remaining scalar-model diagnoses; it supplies no acceptance by itself.

The current normal Release build additionally reaches the previously unbuilt
`tests/simulation/test_component_reference_native.cpp` target and fails
`-Werror=misleading-indentation` at its existing feed-capture JSON serialization.
At the same main HEAD, root owns only splitting that conditional output and its
following unconditional output onto separate lines. No reference values,
scenarios, limits, assertions or production behavior change.

## Captured joint-background optical regressions

The subsequent same-version Release encounter population executes all 24 models:
16 pass and eight fail. All 168 per-model raw artifacts are archived separately
under `full24-current-models/`; the source observer is under
`full24-current-effects/`. This population is diagnostic, not acceptance.

At the same main HEAD, root integrates the reviewed observation/calibration-only
patch in `ThermalReticleReceiverV1.cpp/.h` and
`tests/simulation/test_thermal_reticle_receiver.cpp`. The shared const inference
seam consumes measured ADC charges, their variance, configured optics and own
optical pose; it does not resample, advance cache/RNG or accept target geometry.
Three exact captured native regressions isolate a finite mirror refusal, a
missed background-null basin and a centered source initialization failure.
All original 24 receiver predicates and all encounter predicates remain intact.
Root retains the exclusive serial native slot; no other owner writes its inputs.

## Existing JSBSim getter visibility

At unchanged main HEAD, root owns `cmake/JsbsimPatches.cmake` only for the reviewed
additive export of existing nonvirtual `FGEngine::GetSourceTank(unsigned) const`.
The object defines the symbol, while the normal DLL/import library omit it and
the current 24-model native reference target cannot link. Generate the narrowly
annotated header through the existing maintained-patch helpers and declare its
incremental `OBJECT_DEPENDS` edge. Do not change the method body, signature,
class layout, vtable, feed mapping or physical behavior. Exact before/symbol and
candidate evidence is under `engine-getter-export/`. Root waits for all active
native-input owners to freeze before configuring/building/testing this change.

## Exclusive passive-bistatic SARH and direct-contact carrier traversal window

At `main` HEAD `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9`, root owns the exclusive
serial native write/build/test slot for two physical-chain repairs without
changing any scenario, seed, tolerance, threshold, or native outcome predicate:

1. **Passive bistatic SARH receiver (no monostatic transmitter blanking):**
   - `src/simulation/mission_systems/radar/Signal.h` (`Settings::passiveBistatic`, default `false`)
   - `src/simulation/mission_systems/radar/Signal.cpp` (`dwellImpl` passive-bistatic receive branch)
   - `src/simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.cpp` (`RadioReceiverV1::expose` for `semi && observed`)
   - `tests/simulation/test_weapon_radio_receiver.cpp` (RED->GREEN passive bistatic short-delay & monostatic-blanking negative control)
   - `docs/sdk/weapon-radio-receiver.md`
2. **Conservative finite non-warhead carrier material traversal on armed direct `body_contact`:**
   - `src/simulation/mission_systems/damage/Impact.h` (`CarrierTraversalInputV1`, `buildContactCarrierFieldV1`)
   - `src/simulation/mission_systems/damage/Impact.cpp` (`buildContactCarrierFieldV1`)
   - `src/simulation/runtime/world/World.h` (`ContactTransfer::normalEcef`, internal private member only)
   - `src/simulation/runtime/world/WorldDamage.cpp` (`WorldDamageResponseV1::carrierTraversal`, `WorldDamageContinuationV1::Pending::carrierTraversal`, `World::contactTransfer`, `World::detonateAt`, `World::advanceDetonationEffects`)
   - `tests/simulation/test_impact.cpp` (RED->GREEN conservation, non-double-counting of warhead mass & contact impulse, separating/invalid negative controls, and THOR/component penetration)

Preserves all public ABI, existing monostatic ARH/radar behavior, inert
body-contact conservation, warhead case-only fragment ledgers, and existing
scenarios/seeds/thresholds/tolerances.

## Exclusive 2026-11-25 SIM-M03 closure preflight window

At `main` HEAD `57a42c67f7dc1e1376f5a006b6f94bd8f23b2fc9`, root owns the exclusive
serial write/build/test slot for:

1. **Reticle pointing ULP fix, LOS-rate bracketing, unit controls, and user-approved PL-8 / AIM-120D equipment selection:**
   - `src/simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.cpp` (1-ULP `unit(previousPose->pointing)` comparison in prior join; two-resolved-exposures LOS-rate bracketing across intervening coarse dwell)
   - `tests/simulation/test_thermal_reticle_receiver.cpp` (concurrent conic V5 pre-warm, parallel forward exposure bit-exactness/rounding preservation, 1-ULP pointing join controls)
   - `tools/models/hangar/hangar/data/equipment_models.json` and generated `src/contracts/equipment/EquipmentModels.h` (`equipment.pl-8.autopilot` -> `fsim.control.actuator_aware_cascade.v1`; `equipment.aim-120d.fuze` and `equipment.pl-8.fuze` -> `native.fuze.turning_cpa.v1`)
   - `assets/stores/pl-8.json`, `assets/stores/aim-120d.json`, regenerated `aircraft/stores/pl8/pl8.xml`
2. **JSBSim LGPL-2.1 notice packaging alignment:**
   - `cmake/Install.cmake` (`fsim_install_notices`: include `FGEngine.h`, `FGEngine.cpp`, `FGTurbine.cpp`)
   - `THIRD_PARTY_NOTICES.md`
3. **Hangar store-component / store-control / liquid / missile / ductedrocket / producer-publication test and source-kind provenance alignment:**
   - `tools/models/hangar/tests/test_components.py` (`test_retained_legacy_store_profiles_preserve_existing_generated_numerics`)
   - `tools/models/hangar/tests/test_storecontrol.py` (`Expressions.__init__`, `test_operating_point_exports_unchanged_complete_fin_loads_and_native_lag_state`)
   - `tools/models/hangar/tests/test_liquid_owner_processes.py` (relative sibling test imports + `PYTHONPATH` propagation to temp helper subprocesses)
   - `tools/models/hangar/tests/test_liquid_vessel_inputs.py` (relative sibling test import)
   - `tools/models/hangar/tests/test_liquid_vessel_packing.py` (relative sibling test import)
   - `tools/models/hangar/tests/test_producer_publication.py` (relative sibling test import)
   - `tools/models/hangar/tests/test_ductedrocket.py` (`fixture`: drop V2-only `inlet_residence_s` ref when constructing V1 fixture)
   - `tools/models/hangar/tests/test_missiles.py` (`test_pulse_motor_uses_ignition_time_without_changing_other_generated_files` for commanded dual-pulse cards; `NativeShowcaseConsumers` `plume.render` signature)
   - `assets/stores/{aim-9m,magic-2,pl-10,pl-8,r-27et,r-27t,r-60m,r-73}.json` (canonical `provenance.SOURCE_KINDS` `evidence_kind` metadata only; zero numeric/model changes)
4. **SIM-M03 test-harness & packaging alignment (post-headless-full diagnostic):**
   - `assets/stores/{aim-120d,pl-8}.json` (rename `component_fuze_*` keys/refs to canonical `fuze_*` matching `aim-120c.json`)
   - `tests/simulation/test_weapon_sensor_native.cpp` (`Sensor` wrapper inherits and forwards `ThermalReticleSeekerExecutionV1` + `probeReticle` counterfactual check)
   - `tests/simulation/test_weapon_seeker_models.cpp` (`Runtime`: `receiverConfigurationV3` for `thermal_conscan_reticle.v1`)
   - `cmake/TestDistComponents.cmake` (create synthetic `tools/models/hangar/hangar/data/equipment_models.json` and `FGEngine.{h,cpp}` / `FGTurbine.cpp` in temp dist fixture)
   - `tests/CMakeLists.txt` (`thermal_reticle_receiver` and `sensor_native.*` `TIMEOUT 600`)
   - `src/simulation/runtime/world/WorldEngagements.cpp` (bounded `lower_bound` attribution lookup for continuous-midpoint reticle `measurementTimeS` in `prepareEngagements` and `sampleOnRailSeeker`, plus World-side post-return referee attribution for scalar reticle on-rail tracks)
   - `src/simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.cpp` (`exposeWithMeasuredPriorV2`: preserve `result.pointing = unit(pointing)` and record `result.horizonDiagnostic.ownUp = scene.up` for non-horizon replay provenance without adding referee attribution to raw scalar features)

Preserves all scenarios, seeds, tolerances, thresholds, native outcome predicates, and unrelated working-tree files (`assets/stores/cj-10a.json`).
