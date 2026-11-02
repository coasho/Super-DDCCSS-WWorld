# SIM-M01 radar and fusion completion

## Current full quality closure (2026-11-02)

SIM-M01 is complete under the user's confirmed SIM-M02 engineering rule. All
required M01 components, applicable functions, method fixtures, integration and
reproducible delivery gates are closed. Actual equipment accuracy calibration is
the separately accepted uncertainty ledger: 0/17 admitted, 17 uncalibrated.
It is not counted as measured accuracy or a waived engineering function.

The current source starts at main `ad63ed484016f94e8ddb9cb0b0621abe860a2f9f`.
Three further native defects are fixed: recycled sensor IDs cannot erase admitted
provenance on a capacity refusal or route a newer track back into older history;
link publication preserves the last measured posterior's epoch rather than
renewing its freshness whenever a predicted estimate is broadcast; bounded
prediction history cannot reset the cached last measurement time to zero. Own AMTI
getters still provide current-time predictions. Original public layouts, enum
numbers, defaults, permission checks and strict actor callable lists remain
unchanged. No generic gateway schema is added.

Native fusion now reaches the normal Viewer through a versioned observational
sidecar. Exact frame sequence, publisher PID/incarnation and vehicle generation
are checked. IDs, state, source count/kinds, sample age, unknown identity, complete
state/covariance and refusal counts come from the SDK. The display reports its
128-entity truncation explicitly; public SDK reports remain untruncated. The
Viewer performs no association or state estimation. Missing sidecars, including
old recordings, display unavailable. Recording-only worlds retain their original
working path and have an explicit regression guard.

| Current acceptance population | Result |
| --- | --- |
| Entire Headless CTest inventory | 865/865 executed passed; 893 discovered |
| Entire Release CTest inventory | 939/939 executed passed; 967 discovered |
| Existing approved M02 rotor scope | 28 excluded per preset, not passed; unchanged policy SHA; no M01 exclusions |
| Installed C++ / C99 / Python | Fresh consumers, actual loaded-module identity and source/product hashes passed |
| Authored airframe applicability | 35/35 selected configurations reconciled; 17 air-radar fits fully exercised |
| Bundled JSBSim models | 60 exact model/adjacent-fit contracts; no inferred aliases |
| Native mode/lifecycle stages | 181 passed; 181 exact one/four-worker replay stages |
| Actual fullscreen presentation | Two complete 65-second scenes; ten reviewed 3840x2160 primary-display captures |
| Actual combined knowledge | Receiver radar and delivered link coexist in native fused entities in both scenes |
| Production inputs | All 88 current generations bound; E7 forcibly regenerated after the MESA change, products unchanged |

Dependency counts (77 Headless / 78 Release) overlap the complete inventories;
they are not additional independent physical tests. Nested Python methods,
approved rotor methods, source/binary/configuration stability and all 16 fixed-wing
fleet shards are reconciled from actual logs. Original physical bounds and
independent numerical tolerances are unchanged. The terrain-free eight-aircraft
native radar benchmark sustains 47.377x real time with
28 fused entities and zero refused native inputs. This is a
bounded SDK throughput result, not full-program performance or equipment accuracy.

The observational scenes run at 1x, primary-display fullscreen with Earth imagery,
terrain, clouds, sunlight, shadows and native effects enabled. They exercise
fighter search/STT/TWS/velocity search, all-round AEW surveillance, delivered-link
knowledge, actual multi-source fusion, power-off, loss and retention expiry.
The visual-only capture driver was corrected after full native regression so the
Viewer stays open through expiry; both complete scenes were then re-executed
and their actual primary-display captures reviewed. Native inputs and binaries
are unchanged. The earlier screenshot-driver run remains superseded evidence.
No hit, explosion or identity is invented. These are non-isolated diagnostics,
not strict actor-process joint acceptance or a DCS performance comparison.

The complete selected-fit and equipment-family distinction is recorded in
[applicability](../data/sim-m01-20261102-applicability.json). Missing authored
equipment is never relabeled as proven physical absence. MESA equivalent sector
gains, the two CV-diffusion IMM models, local-plane multipath centroid, bounded
CI grid, greedy association and identity-default optional residual coefficients
retain their explicit mathematical/equipment limits. These disclosed model
approximations are not hidden calibrated hardware behavior.

[Current quality receipt](../data/sim-m01-20261102-full-quality.json) joins raw full
inventories/JUnit, installed consumers and hash-bound actual capture review.
[Reproduction](../../tools/validation/radar/README.md) uses the normal UCRT64
presets, full.py, current 88 generation/cache inputs and the existing map set.
The source, complete installed SDK/Viewer/maps, raw evidence, exact source patch
and archive manifests are delivered under
`build/architecture-reviews/sim-m01-full-20261102/`. Prior failed preflight and
diagnostic attempts remain evidence; they are not counted as completed gates.

## Superseded bounded engineering closure (2026-11-02)

The user confirmed the SIM-M02 completion rule: required engineering components,
functions, theory and reproducible delivery must close; unavailable equipment
accuracy calibration remains a separate uncertainty ledger. This supersedes the
older mandatory-equipment-calibration condition preserved below. SIM-M01 is
**engineering-complete-uncalibrated for all 17 declared air-radar fits/cards**.
The actual-equipment accuracy ledger remains **0/17 admitted, 17 uncalibrated**.
No required missing function is waived by that separation.

The first reviewed 62-file M01 baseline was committed and pushed as `b1e8a20`,
after rerunning 17 selected checks. The continuation starts from full main HEAD
`b1e8a208da7fa344b4b9bee4324944678c293b94`. Unrelated working-tree changes are
preserved; no branch, Git worktree or excluded directory is used.

### Delivered mechanisms and repairs

Signal-level volume, cued acquisition, STT, TWS and velocity search use the native
receiver, scheduling, waveform and processing chain. Ambiguous high-PRF range
does not update Cartesian tracks. The common optional residual-error mechanisms,
IMM and source-preserving native radar/link fusion remain available through
additive C++/C99/Python interfaces with unchanged legacy layouts, enum values,
default actor method lists and permissions. Cards keep legacy CV and identity
residual defaults unless authored; no invented per-equipment accuracy is enabled.

Five reproduced production defects are repaired:

1. Expired terminal retention releases entity capacity before this tick's input
   admission, rather than refusing a valid new source for another tick.
2. Expired contributor history can release bounded source slots on a live entity.
   Current/recent provenance is preserved; the bounded source list is not an
   indefinite historical audit. A stale replay cannot reclaim its slot.
3. Sample age, including delayed transport, governs association validity.
4. Dwell scheduling roundoff cannot timestamp a measurement after world time;
   strict future-input validation remains intact.
5. An explicitly configured fixed-face electronic aperture supports the declared
   all-round MESA scan. Detection, signal echoes and clutter use the same
   beam-selected face transform. Single-face defaults are unchanged.

MESA's manufacturer identifies side-emitting and top-hat end-fire hardware with
360-degree coverage. The four equivalent look directions, uniform sectors and
common face gain are explicitly **assumed approximations**, not four asserted
physical panels or admitted directional gain measurements. Its legacy
`rotation_rpm` denotes the assumed revisit clock, not physical antenna rotation.
The [component/method matrix](../../tools/validation/radar/README.md#theory-and-independent-references)
records the other model limits, including the local-plane angular multipath
centroid, two CV-diffusion IMM modes, bounded-grid CI and greedy association.
Original physical assertions and reference tolerances are unchanged.

### Current verification

| Population | Actual result | Limit |
| --- | --- | --- |
| Clean-source Headless dependency CTest | 71/71 passed | Complete frozen M01/RF/interface selector |
| Clean-source Release dependency CTest | 72/72 passed | Same selector, including its render-side geometry case |
| Fresh installation and C++/C99/Python consumers | 9/9 commands passed | Actual loaded SDK/JSBSim identities and source/product hashes |
| Declared radar fits | 17/17; 181 stages | All 35 airframes reconciled; 18 have no declared air radar |
| Exact one/four-worker native replay | 17/17; 181 repeated stages | Actual status, local tracks, fused state/covariance/provenance |
| Independent CI method | 12 Stone Soup 1.9.1 cases | One CTest; not hardware calibration |
| Existing IMM and filter references | Passing original FilterPy / Stone Soup fixtures | Original numerical tolerances |
| Normal repository product | 15/15 related checks; SDK usability and exact E7 installed-report replay | Native paths/hashes retained; no GUI presentation claim |
| Receipt admission/refusal checker | 7/7 checks | Artifact consistency, not physical-model tests |

The eight-aircraft/four-worker native benchmark, radar on, measures
47.672x real time: 600 timed steps / 20 simulated seconds in
0.419537s, p99 1.048ms, 28 fused entities and zero rejected inputs.
Radar-off is measured separately. This bounded terrain-free SDK scene establishes
neither full-simulator nor Viewer performance. Test names containing class
"calibration" refer to the preexisting SNR fit to published detection-range
claims; they do not change the equipment measurement-accuracy ledger.

The final complete verification command takes 435.673s including both
configure/build/test presets, installation, full fit replay, performance and
normal entry. The first completed 589.194s campaign is retained as superseded
after the all-round defect was found. Additional diagnostic and root-production
build/test costs are separately logged; these numbers are not total engineering
time. Failed-before logs remain intact.

### Reproduction and delivery

Use the single command in the [verification README](../../tools/validation/radar/README.md).
`campaign.py` uses the existing normal UCRT64 presets and SDK launcher, requires
a fresh output/prefix, executes serially and joins actual raw/JUnit/module/hash
receipts before producing acceptance. Inventory or a policy change alone cannot
pass. The source export used for final verification matches 2,892 independently
replayed source/input files; source stayed frozen throughout the final campaign.
The final documentation updates do not alter those compiled inputs.

Tracked [delivery](../data/sim-m01-20261102-delivery.json),
[radar matrix](../data/sim-m01-20261102-radar-matrix.csv) and
[fit matrix](../data/sim-m01-20261102-fit-matrix.csv) retain calibration separately.
Detailed evidence, one main-HEAD-based feature patch, complete installed SDK ZIP,
archive manifests and handoff are under
`build/architecture-reviews/sim-m01-closure-20261102/`. The actual final campaign
and installed prefix are under
`build/sim-m01-closure-replay-20261102/build/architecture-reviews/sim-m01-final-v2-20261102/`.
The existing public-source ledger remains dated evidence rather than an
exhaustive claim that no further public measurement exists. Whole-project full
regression, new Viewer presentation and measured equipment accuracy are separate
from this engineering closure.

## Historical 2026-11-01 execution record

The requirements and "current" counts below describe the superseded assignment.
They remain historical evidence; the 2026-11-02 engineering closure above governs
the delivered state.

The 2026-11-01 user assignment transfers the remaining SIM-M01 package to the
current implementation owner. Main baseline: `596bacacbbbe06d9985227add12393db8e93a4cc`.
The user explicitly retains actual-equipment accuracy calibration as a formal
closure requirement. Engineering estimates and numerical-method agreement must
not be substituted for that requirement.

### Scope and implementation order

1. Reconcile all maintained airframes and fitted air-radar variants. Admit only
   source-backed equipment measurements with their conditions, units and
   uncertainty; keep assumptions and calibration/validation data separate.
2. Add common measurement mechanisms for extended-target glint, correlated
   boresight/radome errors and unresolved multipath. Missing source parameters
   remain explicit and identity defaults preserve existing callers.
3. Support signal-level cued acquisition, track/STT and velocity search through
   the existing receiver/processing chain, with proper PRFs, range ambiguity,
   scheduling, track lifetimes and resource limits.
4. Add an interacting multiple-model filter using existing Gaussian/UKF
   components; verify mixing, probabilities, missing observations and maneuver
   response against an independent established implementation.
5. Fuse only sensor-derived and actually delivered platform knowledge. Preserve
   source identity, covariance, timestamps, unknown identity, duplicates and
   lost/stale states. Do not consume the public DMS inbox or use target truth
   for association. Expose additive versioned public reports without changing
   existing layouts, callable permissions or VecEnv shapes.
6. Exercise the complete supported radar/mode/fit population through the SDK,
   installed consumer and normal entrypoint. Keep method, functional,
   performance and equipment-calibration evidence separate.

### Method references and reuse

- Existing Stone Soup-derived nonlinear filters and association remain the
  foundation. The historical absence of an IMM in that pinned reference does
  not prevent comparison with FilterPy's MIT-licensed `IMMEstimator`:
  <https://filterpy.readthedocs.io/en/latest/kalman/IMMEstimator.html>.
- Source-backed accuracy requirements remain equipment-specific. The Chalmers
  2011 thesis *GPS for use in radar verifications and tests* includes actual
  PS-05/A STT range comparisons, but section 3.5 (PDF page 45, printed 35) says
  the aircraft schematics are not to scale and their metrics were removed
  for product confidentiality. Figures 3-21/3-24 therefore cannot be digitized
  into metre accuracy or scaled from an assumed aircraft length. The work
  tests the comparison method. Its GPS/INS accuracy is not radar accuracy:
  <https://publications.lib.chalmers.se/records/fulltext/140898.pdf>.

### Acceptance accounting

Current execution receipts belong under
`build/architecture-reviews/sim-m01-completion-20261101/`. Source/path ownership,
preexisting changes and exclusive native-slot acquisition are recorded before
implementation. No SIM-M01 completion is claimed by this design record.

### Implemented mechanisms and explicit limits

| Mechanism | Current implementation | Equipment-calibration limit |
| --- | --- | --- |
| Nonthermal residuals | Common fixed/radome bias, stationary OU drift, geometry-scaled Student-t4 glint residual, local-plane coherent image/direct angular centroid; independent RNG stream and identity defaults | Phenomenological estimated mechanisms; no guessed per-equipment coefficients installed. Reflection power remains in the existing propagation chain. This is not a complete coherent two-path receiver model. |
| Signal modes | Existing receiver/CFAR/M-of-N chain used for cued acquisition and STT; high PRF uses bounded receive domains, a known-noise exponential threshold and Doppler-only association; range remains folded | Generic waveform/processing design, not a reconstructed classified processor or equipment calibration. |
| Ambiguity handling | Preserve equally supported aliases; full raw-receiver ECEF position covariance; enumerate feasible prior/alias associations; competing aliases and shared prior returns remain unresolved; unresolved reports cannot update Cartesian tracks | Conservative temporal resolution uses the declared kinematic/noise policy. Legacy reports expose bounds through sigma rather than an alias list. Raw thermal-domain resolution precedes separately modeled nonthermal residuals. |
| IMM | Two CV diffusion modes, continuous-time switching/mixing, nonlinear UKF observations, Cholesky log likelihood and missing-observation propagation | Card parameters are explicit; zero maneuver diffusion preserves legacy CV behavior. No unmeasured equipment tracking constants are invented. |
| Platform fusion V1 | CI with unknown cross-correlation, measured local radar tracks and actually delivered DMS reports, timestamps/provenance, bounded entity/source storage, duplicates, coasting and loss | Greedy gated association is not a full multi-hypothesis fusion architecture; identity remains unknown without identity evidence. |
| Legacy link covariance | Preserve sample epoch; six-axis PSD upper bound from transmitted three-axis RMS summaries; propagate latency before fusion | RMS summaries discard anisotropy/cross-covariance. Bounds are deliberately conservative, not measured full covariance. |
| SDK | Additive C++/C99/Python V1 records and getters; own-Agent checks, sized non-consuming C reads, separate source query | Existing layouts, enum values and default strict-actor callable lists remain unchanged. V1 actor-process opt-in is separate from the existing default list. |

The generic measurement mechanism reference is MIT Lincoln Laboratory,
[Tracking and Parameter Estimation](https://www.ll.mit.edu/media/6981),
slides 30–31. The Student-t4 distribution is an explicit finite-variance
engineering residual model, not a measured glint distribution for any card.
No AI-generated value is admitted as an equipment measurement.

The independent IMM fixture uses FilterPy 1.4.5 with disclosed adapters:
`predict(u)` to variable `dt`, regeneration of measurement sigma points from
the full additive-Q prior, circular angle mean/residual callbacks, and SciPy's
[SPD Cholesky covariance representation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.Covariance.from_cholesky.html)
for the Gaussian log likelihood. Eigen and Cholesky likelihoods from the same
innovation are both recorded. Earlier mismatched fixtures and failure logs
remain retained; tolerances stay mean 1e-8, covariance 1e-7 and probability 1e-10.
This is software-method comparison, not hardware validation.

### Equipment evidence admission

The public-source ledger covers all 17 air-radar cards. A qualitative flight-test
announcement, detection range, SAR resolution, another variant, GPS reference
accuracy or simulated agreement does not establish equipment measurement
accuracy. The bounded search has admitted zero complete equipment-calibration
records; it does not prove that no further public source exists.

`tools/validation/radar/calibration.py` is an optional raw-sample proposal tool,
not the sole admissible evidence format. Its four-channel split-data input
and plug-in Gaussian check do not create additional mandatory project gates
or automatically admit equipment. A suitably identified primary published
accuracy statistic may also be reviewed, with its stated conditions, definition
and uncertainty. The tool currently emits `equipment_acceptance=false` because
source/variant applicability and predictive parameter uncertainty require
independent admission. The roster audit separately reports this unresolved
mandatory calibration condition. No native functional pass changes that status.

### Current execution and delivery (2026-11-01 assignment)

The clean HEAD-derived source export passes **65/65 M01-related Release CTest
cases** in 230.869s. The repository Headless build passes **17/17 selected
integration/compatibility cases** in 36.490s. These populations overlap; they
are not added together or described as a whole-project regression. The Release
selector also includes the architecture, SDK boundary and C ABI gates. The dirty
root architecture check still reports two preexisting unrelated maintenance
scripts; they are preserved and excluded from the feature patch. The clean
HEAD-plus-feature export passes that gate.

The installed C++, C99 and Python consumers pass, including actual local-radar
and delivered-link fusion, ownership, non-consuming reads and native loaded-DLL
identity. Installation and all consumer jobs take 83.632s, including 42.606s for
the common **17 declared radar fits /79 native sampling stages**. The matrix
also inventories 18 airframes without a radar in the authored fit; this is not a
verified equipment-absence claim. Fighter STT/TWS stages require a track update
after the command, with a separate fresh cued-acquisition stage. AEW surveillance
uses the common hardware-class applicability rule. High-PRF checks preserve
range ambiguity and prohibit new Cartesian range-track updates.

The repository normal SDK launcher completes one native fusion usability case
and records actually loaded SDK/JSBSim paths and hashes. Both root presets have
been rebuilt. No new M01 Viewer presentation acceptance or whole-project full
Headless/Release regression is claimed. Historical M02 results remain historical.

An eight-aircraft, four-worker, 20s native scene exposed 726 valid covariance
inputs refused by fusion. Posterior roundoff had left off-diagonal covariance
pairs asymmetric. The producer now stores their symmetric part; diagonals,
measurement/process noise and strict fusion validation remain unchanged. The
same valid scene now has **0 refused inputs**, 28 fused entities and 24.672x
real-time throughput with detection-level radar on (0.811s for 600 timed steps).
The additional exact-symmetry case and existing Stone Soup/FilterPy references
pass. This is a bounded functional/performance scene, not hardware validation.

Processing-loss integration retains the same 24x24x120 quadrature and original
CDF/threshold/bisection rules. Hoisting repeated scallop calculations and
grouping exactly equal loss values reduces the measured high-PRF steady case
from 9.778s to 0.149s (65.707x), and the medium-PRF Rayleigh case from 0.293s to
0.051s (5.746x). Both measured loss differences are 0 dB. No global bitwise
equivalence or whole-simulator speedup is claimed.

Current authored cards keep legacy CV tracking (`track_maneuver_noise=0` by
default); the optional IMM is implemented and method-validated, not silently
enabled or calibrated for all equipment. The new residual coefficients likewise
remain identity defaults unless explicitly authored. Calibration remains
**0/17 admitted, 17 blocked** under the user's mandatory equipment standard.
The source review found no remaining production blocker in its reviewed scope;
it is a source/artifact review, not an independently executed hardware test.

The tracked [radar matrix](../data/sim-m01-20261101-radar-matrix.csv),
[fit matrix](../data/sim-m01-20261101-fit-matrix.csv) and
[delivery record](../data/sim-m01-20261101-delivery.json) separate functional
sampling from calibration. Exact commands, current and unsuccessful logs,
independent review, execution accounting, installed product and the single
HEAD-based patch are in
`build/architecture-reviews/sim-m01-completion-20261101/delivery.md`.
Matrix rows and source/product hashes are consistency checks, not physical tests.

The final provenance-hardened fleet sampling is separately retained in
`fleet-provenance-results.json` (42.594s). It records the
SDK and JSBSim modules loaded inside that very sampling process. The final
tracked delivery/matrices join this record, not the earlier sampling without
in-process DLL identity. The common report rejects stale inventory, duplicate
designs, failed/missing stages and mismatched loaded native products. Six
artifact-consistency acceptance/refusal checks pass; these are not physical tests.
