# Scalar conical-scan reticle engineering method

This is a distinct photon-domain receiver selected by `thermal_conscan_reticle.v1`.
The retained `thermal_quadrant.v1`, image, dual-band and RF receivers keep their
existing meanings. Equipment selection IDs remain the actual card IDs; the new
receiver performs different optics, scalar electronics and temporal processing.
It is an engineering approximation with disclosed parameter assumptions, not an
OEM reconstruction or equipment calibration. Native verification is pending.

The original AIM-9M head-on miss and the quadrant observability RED/GREEN remain
evidence. Neither a different scene nor a reduced detection threshold establishes
this method. The five quadrant-specific regressions now explicitly exercise the
retained compatibility binding. New reticle and actual seven-card cases are
separate; their future results must not be replaced by old compatibility counts.

The [Navy handbook](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf), 2013,
printed7-1.33–37, figures27–34, explains reticle spatial filtering and AM/FM
angle encoding. It supports the physical architecture, not our AAM constants.
[Howser](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V16-N01/16-01-Howser.pdf),
1995, pp27–33, describes scene/waveform/electronics/DSP simulation and measured
scan artifacts; it also warns that artifacts can resemble targets. Its generic
spin-scanning detector-array example is not identified as our AIM-9M hardware.
[Birchenall et al.](https://doi.org/10.1016/j.infrared.2010.07.001), 2010,
tracking-loop/pre-amplifier sections, supplies an independently executable
100Hz,12-period,1200Hz reference for an SA-14 model. Those hardware numbers are
not assigned to any maintained AAM by this implementation.

For local optical angles `u=(x,y)`, scan phase `phi`, radius `r` and reticle radius
`R`, the focal image is `q=u+r(cos(phi),sin(phi))`. The physical transmission is
`T(q)=1[|q|<=R] 1[cos(n atan2(q.y,q.x))>=0]`. Transmission is applied before any
photon draw. The noiseless scene renderer retains the existing finite source
supports, geometric near-to-far occlusion, path radiance, normalized Gaussian PSF
and solid-angle quadrature. Referee attribution never enters modulation or DSP.
Numerical cells are optical integration quadrature, not independently observed
imaging pixels. The reticle integrates each cell's transmitted radiance into ONE
scalar electron count. No image centroid, segmentation or source bearing is
delivered to its processor.

The new declared design rules are `R=min(width,height)*pitch/2`, `r=R/2`,
`n=max(1,floor(pi*r/(8*psfSigma)))` and `f=1/(2*pi*n*responseTime)`.
The first rule gives a finite physical stop; the second leaves nutation clearance;
the period rule reserves eight PSF sigmas per half-sector near the scan circle;
the frequency rule places the carrier at the admitted first-order detector's
minus3dB point. These are our design assumptions, not an upstream packing or
seeker algorithm. The selected current baseline is12 periods and132.629119Hz.
The implementation rejects unsupported carrier counts below3 because its
background nuisance space includes the first two scan harmonics.128 phase samples
and4 by4 within-cell spatial quadrature are explicit numerical design inputs.
No original card parameter record, scene, seed or acceptance tolerance is changed.

Each phase sample retains the original10us integration and4 within-exposure time
samples. For incident band power `P`, the photoelectron mean is
`mu=P*integration*QE*wavelength/(h*c)`, with SI exact `h` and `c`. Independent
Poisson charge is sampled before the scalar detector's first-order response.
Incident charge is never clipped to ADC full well before that pole. In the
independent short-pulse control, incident mean2C and pole gain0.05 imply mean0.1C;
substituting C would incorrectly produce0.05C despite unsaturated readout. The
strict positive-mean standard-library Poisson domain is used, with deterministic
zero for zero photons. Means above2^52 explicitly refuse the numerical reference
domain; this leaves a factor-two margin below binary64's consecutive-integer
limit. No clipped source or unvalidated huge-mean noise approximation supplies a
measurement. Refused numerical samples restore detector and RNG history.
The pole update uses `alpha=1-exp(-actualElapsed/responseTime)`; the finite model
holds the sampled input over that elapsed interval, including a gap between
requested readout windows. It does not claim an unsampled continuous waveform.
The unknown initial pole transient is removed as a fitted nuisance. The expected
charge trace evolves independently of random charge draws.

Equivalent-area electronics are an explicit inference. For `Pcells=width*height`,
scalar full well and dark charge are `Pcells` times their retained per-cell
reference; independent equivalent read noise is `sqrt(Pcells)` times its
reference. This is a capacitance/charge-area model, not a claim of4096 physical
read amplifiers. Fully correlated read charge would scale as `Pcells`; sensitivity
must cover that alternative. ADC bits remain unchanged, with charge step equal
to derived full well divided by `2^bits-1`. Saturation, quantization and raw scalar
signals remain observable; saturation cannot be rescued by a selected source.

Scan phase belongs to the receiver from its first accepted call. Startup cannot
invent a past scan. A read uses the most recent complete phase-zero cycle ending
at or before its native read time. The measurement epoch is the cycle midpoint;
processing becomes ready at cycle end plus the unchanged processing delay.
Source rays use the existing constant direction-rate lookback at every optical
integration sample. Bounded own physical-axis, pointing and estimated-navigation
history brackets the midpoint. Within one scan the own optical pose is held at
that interpolated midpoint: a disclosed finite dynamics approximation, not
current-pose relabeling. Source motion and pulse shape can therefore violate the
stationary compact-radiator fit and must produce a residual/refusal rather than
a truth-derived correction. History gaps or unavailable midpoint pose refuse.

DSP reads only scalar ADC samples and the declared instrument. It projects out
DC, calibrated uniform-radiance etendue, the unknown initial pole transient, and
the first two scan harmonics. A compact-radiator template is generated by the
instrument's rectangle-convolved Gaussian PSF and reticle response, including the
same first-order pole; its angle, support widths and signed amplitude are fitted
from the waveform. No actual scene extent is supplied to this fit. Coarse angle
search uses the original optical grid, then a fixed finite support bank and18
levels of bounded coordinate refinement. If that point-seeded inverse cannot
explain the measured waveform, a joint coarse centre/rectangle-support search
supplies a second start before the same exact refinement and residual screen.
Its fixed dyadic proposal bank and dyadic midpoints through128cells, plus the actual admitted
detector dimension, uses source cells of
`ceil(2*PSF_sigma/pixel_pitch)*pixel_pitch`, with a minimum of one optical cell.
The current seven-card half-cell PSF retains the original proposal spacing.
Broader blur uses a coarser numerical start grid followed by the same exact
continuous support fitting; no receiver pixel, PSF, detection threshold or
measured resolution changes. Its summed-area waveforms are linear unions of separately integrated unit source cells,
including the original nine-sigma off-field optical support. They propose a
start only: the selected support is re-evaluated by the original continuous
rectangle-convolved PSF before it can improve the accepted objective. This
avoids treating a point-template alias as the only possible centre of an
extended source. A rectangle may have an unobserved edge beyond the
nine-sigma bank: only its proposal integral is clipped, with full extent,
normalization and centre retained before original exact re-evaluation.
It introduces no scene extent, identity or source label.
All candidates remain in the declared
finite optical field. A fixed global lattice bound, rather than the count of
data-adaptive queries, supplies the nominal Gaussian trial correction.

The original `detectionSigma` remains the base screen. Noise scale is estimated
from measured scalar charge, known read noise, ADC step and the response gain;
expected/source charge cannot supply it. The first-order filter has bounded
unit l2 gain. A Laurent–Massart Gaussian residual screen and finite Fisher
conditioning reject inconsistent precise fits; amplitude and fitted
support widths are nuisance parameters in the bearing information matrix.
These are high-count engineering screens with a measured-count noise estimate,
not a certified equipment ROC or an exact low-count Poisson false-alarm bound.
Two resolved measured bearings are required before LOS-rate authority. A single
centered carrier does not manufacture a target angular rate.

A large compact-template correlation is never sufficient when its measured
residual rejects that template. Such a capture proceeds to the existing joint
unknown-horizon/compact-source inverse. Every residual-consistent compact fit
also tests the unknown-horizon null: an adequately explained background vetoes
the direction even when a local Fisher matrix appears well conditioned.
A residual-consistent compact fit with a singular local Fisher matrix may
retain a provisional coarse direction only after that null is rejected.
A retained singular-fit direction uses the squared
diameter of the entire admitted optical field as its conservative mean-squared
angular-error bound; fitted source width and numerical lattice spacing cannot
supply a local covariance when that local information is singular. This is a
coarse engineering bound, not a reconstructed equipment resolution.
The private `preciseBearing` state remains separate from `bearingResolved`:
a finite coarse direction still supports association and optical centering,
but the V3 wrapper withholds angular-rate authority until two consecutive
precise captures. The existing uncertainty and measured epoch follow the
actual selected observation. Retained V2 behavior and all public layouts are
unchanged. A new slow-servo optical fixture retains genuinely one-edge support
while checking unavailable rate and a finite field-wide bound; its compact
control independently requires actual rate-bearing tracking.

## Independent inverse comparison and bounded native refinement

The unchanged source-edge and wide-source fixtures exposed failure of local
centre/extent coordinate descent, independently of optical correctness. The
retained coarse waveform has128measured phases; its source-free angular
inverse receives no source geometry. The known fixture geometry is used
separately to validate the forward model, never as an inverse start.

The independent NumPy/SciPy reference reconstructs64by64optical cells,
128scan phases, four within-exposure samples,4by4sub-cell masks, exact solid
angles and Gaussian rectangle integrals, the same finite pole, and the
seven-dimensional nuisance space. It analytically eliminates unknown signed
amplitude and verifies its analytic centre/extent Jacobian by central
differences. Official [SciPy least-squares documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
and its open `optimize/_lsq/trf.py` describe bounded trust regions, scaling and
rank-deficient Jacobian handling. They support the numerical method, not any
maintained seeker's OEM electronics or equipment accuracy.

Executed with NumPy2.4.6 and SciPy1.18.1 under `D:/ENV/python/3.13/`, the
retained coarse expected waveform has normalized forward residual1.2477e-8;
the wide negative-source expected waveform gives4.0348e-20. Analytic Jacobian
relative errors are7.0716e-9and4.2269e-9. The actual noisy coarse waveform
fits at36.5713from the current production start in eight bounded SciPyTRF
evaluations; an independent broad start reaches37.0710. The actual wide
negative waveform fits at34.27385from a generic centre start in27evaluations.
Both original admission bounds remain unchanged:217.1577after nuisance/shape
degrees of freedom and233for the independently conservative all-samples
necessary bound. These are numerical residuals, not real-equipment errors.

The native refinement uses a separate bounded, column-scaled damped
variable-projection solver. Pivoted HouseholderQR solves the augmented
Jacobian on all81box faces; it avoids normal equations and preserves all
original shape bounds. Squared extents have the Gaussian second-derivative
limit at zero, so a point-source start can expand when measured information
supports finite extent. Numerical damping follows actual versus predicted
undamped cost reduction. It is an in-house bounded LM/trust-region method,
not a claim that native execution implements SciPyTRF or an OEM processor.

Every proposed centre and extent is quantized back to the pre-existing
`pixelPitch/2^18`lattice before the original `spot()` objective can accept it.
The existing global finite-trial correction therefore does not silently grow
to cover a continuous search. A separately executed NumPy implementation of
this numerical scheme reaches36.57142for the coarse waveform and34.27385482
for the wide waveform on that exact lattice; alternate starts retain the
same minima within numerical quantization. The current native receiver gate is
recorded below; actual model encounters require separate evidence.

The coarse reference also exposes global support ambiguity: remote centre/
extent fits can have nearly equal measured residual even when one local
Fisher matrix is finite. Reflected modes and boundary-edge-preserving
profiles are consequently checked with the existing likelihood/trial bound
before granting `preciseBearing`. A finite local reflected cluster is retained
with the envelope `max_i(sqrt(V_i)+d_i)^2`, including the selected mode, each
admitted mode's own nuisance-marginalized local information and geodesic
displacement from the selected ray. Local coordinate covariance is converted
with the ray Jacobian's largest metric eigenvalue. This is an engineering
envelope built on the disclosed Fisher/Gaussian approximation, not a universal
calibrated covariance claim.

Resolved authority additionally requires common nonempty intersection of all
finite compact source-support rectangles, finite local information for every
admitted reflection, and interior uncertainty within the configured angular
domain. The existing detectionSigma multiplier certifies interior validity;
it does not change detection admission. Great-circle distances to detector
planes and a conservative spherical cone inside the actual stop/scan disk
use the same geodesic angular metric as the envelope. Disjoint, singular,
domain-truncated and remote source-edge profiles remain coarse/no-rate.
A narrow15.85microradian mirror over3milliradian support consequently retains
its measured small uncertainty instead of receiving full-field covariance.
An elongated source can still provide coarse centering in its compact axis.

Every visited background cutoff now projects its scalar amplitude onto the
physical raw-radiance domain before its measured residual can compete.
With measured raw uniform/transient coefficients `s0` and `k`, the necessary
constraints are `sky=s0-k*b>=0` and `ground=sky+b>=0`. Their affine feasible
interval admits valid signed contrast, refuses an empty domain, and prevents
an impossible negative-radiance null from vetoing a measured source. Source
and competing mirror amplitudes receive the corresponding finite-area raw
source/background feasibility check. Original residual, noise, cardinality,
card and terminal-outcome predicates remain unchanged.

Exact scripts, frozen waveforms, hashes, starts, dependency paths, singular
values and raw receipts are retained under
`build/architecture-reviews/sim-m03-current-20261104/reticle-repair/` as
`fit_reticle_reference.py`, `fit_reticle_candidate_reference.py`,
`independent-{coarse,wide}-trf.json` and
`independent-{coarse,wide}-native-candidate.json`. These executed independent
method comparisons do not assert native/full-program acceptance.

The retained positive075waveform independently demonstrates why a single
global running score is insufficient. A generic48by2cell support proposal,
whose centre is selected only from the measured source-grid correlations,
converges to residual38.26397in six SciPy evaluations. Numerical recovery now
retains distinct support-scale minima and allows each seed to descend in its
own local state before comparing physical residuals. It tries at most16ranked
scale seeds and stops when an unchanged physical residual gate is met.
No nominal source width or scene label is supplied to these proposals.
Valid interior amplitudes retain their original bits: a divide/multiply unit
conversion is not evidence that the physical amplitude constraint was active.

The prefix holds at most5.76MB with the current64 by64/128-phase optics.
For every admitted detector shape and the existing4,194,304 exposure-work
limit, its conservative analytic maximum is42,700,800bytes. The nine-sigma
margin needs at most five proposal cells on each side. This is an allocation
bound, not a measured runtime claim. Joint proposal scoring runs when the
original exact point-seeded fit fails its unchanged residual screen, or when
an initially detectable unconstrained fit fails that same screen after raw
radiance feasibility is imposed. This prevents an impossible raw optimum from
suppressing recovery through physically valid support-scale proposals.

The independent empty-Earth regression uses the actual AIM-9M receiver and
scene temperatures, an independently derived6km spherical limb, three viewing
offsets and no source/flare/attribution inputs. The original execution emits
false bearings at both noncentral offsets. The current native gate below
verifies this regression together with the unchanged wide/stop-edge predicates;
that component result does not establish complete-program acceptance.

## Current native receiver evidence

On 2026-11-04 the normal Release receiver executable passed all **24 named
Catch cases and 226,815 assertions**, with zero failures. The retained
`reticle-repair/final-full.log` contains the Catch summary, and
`reticle-repair/final-full.xml` records a measured suite time of **104.242 s**.
These paths are relative to
`build/architecture-reviews/sim-m03-current-20261104/`.
The tested `ThermalReticleReceiverV1.cpp` SHA-256 is
`d305669693f80e7af968153ccd2c03ce16a72f62b6b2402f71efc1fecaf548b6`.

The XML contains 28 `testcase` elements because the wide signed radiator case
also emits four dynamic sections: contrasts -2.5 and +2.5, each at widths
0.075 rad and 0.09 rad. These are four sections within the 24-case population,
not four additional independent named cases. All four passed with the original
input values and predicates; their recorded section times range from 0.972 s
to 0.995 s. The gate also includes the original near-stop and navigation
fixtures, the independent empty-Earth false-bearing regression, coarse-rate
negative control, physical null/source controls, small-stop mask control,
finite mirror uncertainty controls and frozen legacy optical equivalence.

These times measure the receiver test population, including fixture setup and
multiple exposures. They are neither an AAM simulation-to-wall-time ratio nor
evidence of model-chain or full-program acceptance. The actual seven scalar
reticle model chains, their guidance and terminal outcomes, runtime cost and
required real-time visual presentation remain separate integration gates.
The receiver's authored optics, bounded source/background hypothesis families
and local Fisher/Gaussian uncertainty retain the method and hardware limits
stated here; passing these fixtures does not establish OEM calibration,
universal identifiability or immunity to unmodeled clutter.

## Captured joint-background inference follow-up

The subsequent unchanged model encounters exposed three additional inverse
failures. The exact captured native RED run retained in
`reticle-repair/horizon-capture-red/exact-pose-red.xml` executes three cases,
32 assertions and three failures:

- AIM-9M's centered source reaches residual 114.3252975447 with finite local
  variance 3.22926e-9 rad2, then refusal 18 discards an admitted reflected
  cluster. Searching every measured coarse-grid null minimum independently
  gives best physical background residual 2761.531008565, above the unchanged
  null limit 192.741618292. That captured source cannot be replaced by the
  tested source-free background hypotheses.
- Magic-2's source-free late capture emits a false strip while its original
  one-basin null search stalls at residual 12066.624317094 (refusal 6).
  Independent bounded refinement of the other measured null minima finds a
  positive-radiance background with residual 112.387801924. Independent
  32-node and 64-node optical quadratures agree within 2e-12 in that residual.
- Magic-2's centered source is initialized from a wrong background basin and
  fails the unchanged joint residual bound 214.498344353 at residual
  4691.930328421221 (refusal 8). A generic source start at a separately refined
  measured null minimum fits residual 112.512684 in ten SciPy evaluations;
  no actual target pose or source extent initializes that inverse.

The captured regression reuses a const `inferMeasuredV1` calibration seam
shared with `expose()`. It consumes measured scalar ADC charges, the existing
variance bound, configured optics and exact own optical pose. It neither
reads expected photons/source identities nor mutates the RNG or cached
exposure. The first Magic replay used geodetic own-up reconstructed from
bracketed kinematics and did not reproduce the native minimum. An unchanged
observational full encounter proved all original packet fields bit-identical
and provided the exact native own-up: a roughly 9.1e-12 vector difference
changed the unstable local-basin result. Only those two Magic fixture input
triples were corrected; charges, epochs, pointing and tolerances were kept.
The failed reproduction log and before/after provenance remain retained.

The follow-up implementation independently refines every strict measured
coarse-grid null minimum through the existing 18 dyadic levels; unmodulated
flat plateaus do not receive redundant refinement. Each visited cutoff still
competes as a physically feasible null. The first follow-up retained the old
512-calibration cap, but the complete 27-case native run then passed 25 cases
and failed two original positive controls (226,449 assertions, two failures).
The focused diagnostic run proved refusal 21 in both: null residuals 790068
and 12063.8 did not explain those sources, and the legitimate all-minimum
search had exhausted its cache/work allowance before any joint fit. The
retained receipts are `horizon-capture-red/joint-full.log`/`.xml` and
`horizon-capture-red/positive-diagnostic.xml`. The subsequent internal resource
bound below is derived from the complete existing search; no basin is skipped
and no detection or residual threshold is changed. Exceptional exhaustion
still reports numerical unavailability and withholds source authority.
This finite search is an engineering method,
not a proof that an arbitrary continuous background family has a unique
global minimum.

Joint source fitting uses its initial measured null basin plus up to four
additional independently descended measured null basins, with the original finite support starts, shape domain and
18-level/three-sweep coordinate budgets. A physically invalid optimum cannot
outscore a valid candidate. No scene geometry supplies these starts. Each
admitted background/reflection mode receives its own local information after
marginalizing background and source amplitudes, source extents and unknown
cutoff. The same finite-support/geodesic Minkowski envelope qualifies a local
mirror cluster. Singular information, disjoint modes, domain truncation and
admitted remote support-edge profiles retain only a field-wide coarse
bearing, with no precise-rate authority. A source that cleared the original
physical/evidence/residual screens is not discarded solely because precise
bearing is unavailable.

Let `n = detector_height + 1`. The existing minimum predicate can retain both
ends of a low plateau, but three consecutive entries cannot all qualify: their
middle would have no strictly higher neighbour. Therefore a sufficient bound
is `m = n - floor(n/3)` retained minima. Coarse sampling and the existing 18
dyadic levels require at most `n + 36*m` distinct null calibrations. The initial
joint fit plus four additional starts add at most `5*18*3*2 = 540` cutoff
calibrations. Selected-mode curvature adds two. Each snapshot cutoff can then
add only its two neighbouring cutoff derivatives, shared by all four reflected
source geometries. Consequently the complete cache/work bound is
`3*(n + 36*m + 540 + 2)`: 6573 calibrations at height 64 and at most 11301 at
the admitted height 128. Constants used by that count also bound the actual
refinement/start loops.

Each calibration now retains only two raw/projected phase vectors and scalar
cost/amplitude metadata. Their payload is bounded by 13,461,504 bytes for the
current 64-row/128-phase optics and conservatively 92,577,792 bytes across the
admitted phase/height domain, excluding allocator metadata and the separate
bounded 128-source-vector cache. Spatial fields remain transient renderer
workspace; they are not multiplied by the calibration count. A zero occulted
field reuses the original ground waveform bit for bit. For partial occultation,
the same pure renderer reconstructs the ground field and performs the original
spatial subtraction and original summation order. No linear-waveform
subtraction approximation or new numerical error allowance is introduced.
These caches introduce no new receiver resolution, hardware threshold, RNG
draw or source information. Runtime remains subject to the actual native and
model-chain gates; these allocation/work counts are not a speed result.

The resource-corrected receiver subsequently passes all 27 native cases and
226,655 assertions, but its roughly 210 s wall time exceeds the registered
180 s receiver gate. Functional results therefore do not admit that normal
gate. The next exact memo retains only the most recently rendered full ground
field within a single inverse call. Its key is the exact finite cutoff value,
including signed zero; detector configuration, pointing and own-up remain
constant in that call. Calibration and partial-support evaluation reuse those
same field bytes. Partial-support evaluation copies the memo before the
original spatial subtraction and summation. Replacement occurs only for a
different cutoff; no source trial, search order, arithmetic expression,
threshold or receiver history changes. Added payload is bounded by one
128-by-128 double field (131,072 bytes). A new native result/time comparison
and the registered gate are required before claiming this optimization works.

The one-field memo's duration-enabled native diagnostic remains functionally
green at 27 cases and 226,655 assertions, but takes 195.6254662 s; the unchanged
registered 180 s gate times out. The next one-entry memo retains the raw outside
phase vector for one exact five-coordinate source/cutoff hypothesis. Every
request still calls `calibration(cutoff)` first. All five key values, including
signed zeros, must match. Projected requests copy the raw memo and invoke the
existing nuisance projection; they never modify or move from the memo itself.
This preserves `waveform(field, true)`'s existing raw-filter-then-project order.
Raw-budget evaluation immediately reuses that same unprojected vector without
re-rendering occulted support. `sourceWave`, candidate/search order and all
scores/gates remain unchanged. Additional payload is at most one 512-phase
double vector (4096 bytes). Native equivalence and the normal registered
timeout still require fresh verification.

The unchanged actual AIM-9M encounter subsequently sampled a long-frame stack
inside generic `spot()` called by `fit()`, rather than the joint-horizon path
(`reticle-after/aim9m-live-stack.log`). In that kernel, X interval probability
was recomputed inside every active detector row despite all its inputs being
row-independent. The exact axis reuse prepares X probabilities once, in the
original column order, at the first active Y row. It retains identical
`lowX`, `highX=lowX+pitch`, nine-sigma inclusion and probability expressions;
then each row uses the same `wy*wx*solid_angle/pixel_area` operations. Pixels
and phase accumulations remain in their original ascending order. A wholly
off-field Y support performs no extra X evaluation.

For `R` active rows and `C` included columns, this reduces X probability
evaluations from `R*C` to `C` (zero for `R=0`), removing `(R-1)*C` repeated
calls. It uses only 128 cached doubles and 128 inclusion flags, within the
already admitted detector dimensions. It changes no mask, source support,
kernel integral, numerical threshold or search. These are exact operation
counts, not a measured speed factor: the new native gate and actual encounter
timings still determine whether this optimization is sufficient.

The axis-only registered receiver gate still exceeds 180 s. The next storage
change reuses the existing `weights_` buffer in pixel-major order, updating all
five coefficient accesses: construction, flat accumulation, `spot`, its
Jacobian and horizon waveform integration. It adds no array, payload, header
member or allocation. Actual photon `timeWeights_` and `expose()` are unchanged.
Each individual phase still accumulates detector pixels in ascending order,
with identical coefficient values and multiplication/division grouping. The
horizon waveform loop interleaves independent phase accumulators, preserving
each phase's original ascending-pixel sequence. Phase loads are now contiguous
instead of the current 32,768-byte stride. The normal toolchain uses `-O3` and
C++17 without an explicit fast-math/native-architecture/FMA override; native
compiler/result equivalence and full actual physics comparisons are still
required. Intel's primary [Optimization Reference Manual, section 5.4.1.2](https://cdrdv2-public.intel.com/814198/248966-Optimization-Reference-Manual-V1-049.pdf)
supports contiguous data organization; it does not establish this kernel's
speed or an inferred equipment accuracy improvement.

The three new captured fixtures retain their original admission requirements.
The AIM replay's old local-minimum equality is changed only to an upper bound
of `old_residual + 1e-5`: native reproduction was already verified, and a
legitimately lower objective must not fail a regression. Its compact-proposal
checks and the original 24 cases are unchanged. This follow-up requires a new
native receiver gate and actual model encounters; the earlier 24-case receipt
above does not verify these later source changes.

The shader weights occupy at most4194304 doubles across within-exposure samples.
Coarse templates and mean mask weights are additionally bounded by the admitted
128 by128 field and512 phase samples. Native fixtures must measure actual cost;
no synthetic speed estimate is acceptance. The original receiver replay is
checked against a separate frozen single-band implementation, with original math
preserved and only reversible type/include spelling changes; its old dual-band
suffix is deliberately omitted to avoid duplicate production definitions. A
single `maybe_unused` declaration annotation retains an otherwise unused
dual-only helper without altering its mathematical body or compiler warnings.

`thermal-reticle-models.json` binds each of the seven actual cards and its full
resolved seeker parameter records. Shared software policy does not establish
hardware equivalence: notably the Magic2 historical multi-element description
is not a single-detector reticle specification. Exact reticle/detector details
remain inferred for the other variants too. Field, spectral response, signatures,
IRCCM, dome/baffle scan artifacts, detector correlations and gyro dynamics retain
explicit uncertainty. Small bright clutter, flares, multiple sources and edges
can produce false tracks or loss. No class/identity channel repairs them.

The independent Python reference executes exact point-scan angular velocity,
primary1200Hz carrier, SI photon conversion, pole response and invalid-domain
controls. Native tests independently count actual mask transitions, inspect the
real rendered photon spectrum, compare the real pole/ADC outputs, test all seven
current factories and exercise no-source, extended/horizon clutter, off-field,
ordering, causality and legacy optical equivalence. Optional native CSV is only
numeric input; a separate current source/runtime/Catch receipt is required for
execution admission. No numeric or hardware PASS is inferred from file presence.

### Current PL-8 and R-60M captured joint-search repair

The pixel-major-only source epoch passed the registered180s receiver gate in
143.39s:27 named cases and226655 assertions. Current actual AIM-9M, Magic2 and
R-73 encounters passed their original gates; PL-8 and R-60M still failed. Those
subset receipts do not establish an all-model pass or real-time performance.
The complete Magic2 kinematics, fins, events, summary and component files were
byte-identical to the preceding accepted-physics encounter. All live trace
fields also matched exactly; postterminal stamp differences reflect the separate
World entity-removal repair. This checks storage equivalence, not hardware fidelity.

Four exact current ADC replays independently isolate an additional joint
optimizer defect. The table uses native capture epochs, positive recovered raw
sky/ground/source budgets and measured-only bounded SciPy fits. Expected photons
and opponent geometry never enter the inverse.

| Model | Capture time (s) | Independent legal residual | Original bound |
|---|---:|---:|---:|
| PL-8 |15.69414026027317|196.267753254|214.498344353|
| PL-8 |15.72429954974763|184.263303165|214.498344353|
| R-60M |15.69414026027317|119.982800940|214.498344353|
| R-60M |15.72429954974763|88.919160023|214.498344353|

The preceding PL-8 capture reaches219.046011234 and remains above the unchanged
bound. It is not a positive source regression. Native execution reproduced all
four later refusal8 failures before repair. Every fixture binds its own model
card, exact measured charges, variance, exposure epochs, native pointing and
actual native up vector. Original27 predicates and full-chain scenarios remain
unchanged. Numeric receipts and input hashes are retained in the dated
`reticle-repair/current-captured-fit` evidence directory.

The fallback now refines the complete five-parameter reduced residual: two
angles, two squared extents and unknown horizon cutoff. Each evaluation
re-solves both linear amplitudes, including the occulted background column.
Central differences therefore include amplitude changes and source/background
coupling. The private column-scaled damped Householder solver visits all243
box faces; its4D specialization retains the original operation/face order.
Each start has independent local numerical progress, while only physically
feasible candidates can replace the published physical best. Fixed boresight
two-pixel compact proposals are evaluated at up to five measured null minima;
existing field-wide proposals and coordinate/basin searches are retained.
Every scored proposal is quantized back to the existing angular and cutoff
lattices. The original raw budget, null, residual, rank, mode-envelope and
coarse/rate rules remain authoritative.

This is a bounded numerical fallback, not a global convergence guarantee or a
copy of SciPy's algorithm. An independent reduced-residual/damped243-face Python
implementation reaches198.3149/184.9050/120.7043/89.4014 after original lattice
quantization, each with positive raw budgets. The reference uses the official
[SciPy bounded least-squares method](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
for the separate comparison. Native execution at receiver source75445c35 and
test62c460c5 passes all four captured cases with64 assertions. The unchanged
registered180s gate then passes all31 named cases and226719 assertions in
159.17s. Current reference/navigation gates also pass. These verify this
receiver implementation and its normal test budget; fresh all24 model-chain
acceptance at the new source epoch remains separate from the preceding
pixel-major encounter receipts. No1x simulation or hardware fidelity claim
follows from the unit timings.

Resource coverage includes all rejected trial cutoffs. Let
`U=(height+1)+36*M+540`, with the previously derived plateau-minimum bound M.
Five starts,64 iterations and12 damping trials add at most
`5*(1+64*(2+12))=4485` distinct cutoffs, including two cutoff derivative probes
per iteration. The later finite snapshot profiles every visited cutoff; each
may add its two derivative neighbours shared by all reflected modes. The safe
capacity is therefore `3*(U+4485+2)`:20028 entries at64 rows or24756 at128 rows.
Only raw/projected phase vectors and scalar calibration data are retained;
spatial fields remain transient. Maximum phase payload at512 phases/128 rows
is202801152 bytes, before bounded vector metadata. Actual exhaustion still
refuses numerically. No source basin or mode is skipped to fit the resource cap.

### MovingRadiatorInverseV2

The current PL-8 chain still loses three later centred captures after the5D
repair. Independent stationary fits reproduce272.250700/336.471570/283.681030,
above the unchanged stationary bound. A time-resolved source model fitted ONLY
to measured ADC reaches87.401990/115.919985/102.363182 with positive raw budgets.
Separate expected-photon diagnostics reduce deterministic residuals150/219/195
to approximately1e-12. Those expected values never initialize the measured
inverse. The third comparison includes actual time-varying source/ground
occultation; the coarse8-node pilot is superseded by32-node zero-motion basis
agreement6.17e-13. Three captured native RED cases reproduce the missing
measurements before this extension. New native GREEN/runtime gates are pending.
R-60M has separate remaining numerical/physical evidence and is not admitted
by the PL-8 comparison.

The versioned moving family has seven nonlinear coordinates: two endpoint
optical angles at each of the first/last outer subexposure state epochs,
constant source width/height and unknown horizon cutoff. Both signed linear
amplitudes are eliminated at every candidate. A canonical read-time unit ray
and perpendicular derivative are reconstructed from the endpoints. With their
times relative to read `a,b` and ray dot product `c`, the endpoint scale ratio
solves `a*r^2-(a+b)*c*r+b=0`. Stable quadratic roots, explicit zero-read-norm
refusal, both physical branches and the EXISTING renderer sampling domain
`norm(rate)*integrationTimeS <= .5` bound the model. The zero-rate branch uses
the original static kernel directly. Negative discriminants are never clamped
into a finite-precision solution; branch/rate-bound uncertainty stays coarse.

All actual within-phase exposure times and masks are retained. The current
forward code first normalizes the ray at `sample+subexposureDuration/2`, then
the inner image integration propagates back half a subexposure using the
unchanged derivative and normalizes again. V2 mirrors BOTH operations. This is
the current software forward law, not an assertion that the composition is a
textbook continuous canonical ray. Its difference from direct canonical
propagation is proportional to `(sqrt(1+|rate|^2*deltaTime^2)-1)*rate*halfDuration`;
high rates in the numerical domain need explicit error accounting. No forward
scene, clock, rate, noise, seed or outcome is changed here.

The scalar electronic pole updates ONCE after summing the subexposure inputs.
After its first update, elapsed time is1/(frequency*phaseCount). Any prior charge
and different first-gap response contribute one unknown transient
`C*(1-alpha)^j`, in the same span as the retained nuisance `(1-alpha)^(j+1)`.
Thus zero-state filtering plus existing nuisance projection needs no hidden
detector charge, expected charge or simulated forward state. The additive V2
diagnostic explicitly distinguishes read/capture, photon-sample and outer-state
epochs. Existing V1 compact/horizon diagnostics retain their STATIC meanings.
No V1 member/layout, signature or plugin vtable changes; new pure calibration
methods return optional caller-owned V2 output synchronously, without TLS,
global caches or borrowed-pointer joins.

Static thresholds are unchanged. V2 residual degrees are
`phaseCount - actualInstrumentNuisanceRank - 9`,112 for the captured cards.
Its endpoint/support/cutoff lattice contains at most `Nx^3*Ny^4`, where
`Nx=width*2^18+1` and `Ny=height*2^18+1`. Two ray branches and four linear-amplitude
sign combinations yield the conservative8-times trial count. Under the SAME
projected zero-mean Gaussian nominal-null assumption, compact-source Mills
tails consume at most `alpha/(sqrt(2*pi)*z4)`, the existing two-amplitude joint
family at most `alpha/2`, and V2 at most `alpha/4`, with
`alpha=exp(-detectionSigma^2/2)`. Supported width/height>=8 give `z4>=10.855184`
even as the strictly-positive sigma approaches0. Their union is therefore
below.786752*alpha. The source score is capped by both physical-null improvement
and fixed two-column projection energy. This does not turn an impossible raw
null into a background explanation. Structured-background completeness and
non-Gaussian equipment ROC remain separate; nonlinear residual DOF is the
disclosed local engineering screen, not an exact universal distribution.

Full seven-dimensional information marginalizes both motion coordinates and
both amplitudes before propagating to the capture ray. Each admitted physical
branch, reflection and visited cutoff has its own local information. Singular
observable directions, endpoint/rate-domain confidence truncation and remote
support continuations stay coarse. The finite-mode envelope covers admitted
capture-centre displacements and local uncertainties; fitted within-scan rates
never directly grant guidance authority. Existing V3 two-precise-capture rate
qualification remains authoritative.

Work and storage are bounded from actual queries: at most one retained measured
static continuation plus two branches at five measured null starts; each has64
iterations,12 trials and two cutoff derivative probes per iteration. Up to five
initial cutoff representations, all coarse/local-minimum null queries and
selected curvature are included before the nonrecursive mode snapshot. Every
snapshot cutoff may add its two covariance neighbours. Raw/projected phase
vectors are retained per cutoff; spatial fields are streamed. The source phase
cache covers at most104fixed branch/reflection/derivative geometries. All-sky
support reuses the exact background basis; a wholly ground-backed radiator
uses an exact linear change to contrast relative to ground. Partial support
uses time-varying spatial subtraction. This is bounded numerical work, not
a global convergence guarantee or1x performance result. Native capture tests,
the unchanged180s regression and fresh actual model/visual runs must measure it.


Moving V2 numerical and resource correction checkpoint (native verification pending):
all declared independent trust starts now execute even after an earlier residual-consistent
fit. Each physical local minimum participates in the ambiguity check; each physical
endpoint branch uses its own optimized geometry in the cutoff/reflection bank. An
admitted distinct canonical local geometry receives its full cutoff/reflection bank
and endpoint/support-boundary continuation probes, with its own marginalized covariance.
Only exact branch/geometry identities are deduplicated; differing cutoff minima share
a bank that explicitly visits every recorded cutoff. This closes the declared finite
multistart continuation bank and does not prove global convergence of the continuous
nonlinear family. The second-root existence test follows
the canonical angular separation on the existing rate domain: for same-sign endpoint
times a,b, the separation peaks at |v|=1/sqrt(|ab|); its value at the maximum
admitted rate bounds the fast component from below. Configuration-derived diagonal
interior proposals cover a nonempty fast component without a scene-derived start.

The endpoint reconstruction uses dimensionless time and a gamma(256) arithmetic
certificate for the radial component before orthogonal projection. Its endpoint
roundoff radius augments the reported capture envelope; a certified rate interval
touching the existing boundary remains unresolved. The final selected hypothesis is
quantized again and re-solved with the exact lattice geometry, eliminating the
sqrt(square(width)) round-trip as a source of off-lattice candidates. The 216-case
independent endpoint reference is supporting numerical evidence, not native acceptance.

Regional fallback memoization retains exactly one original native background pixel
vector per inverse call/cutoff. The V1 entry supplies no memo. The V2-only synchronous
provider constructs the same unit-radiance scene, normalization, and single-exposure
policy used by the retained fallback; the subsequent opaque-patch render and subtraction
are unchanged. The memo is separate from the accelerated full-ground field because
their numerical integrals cannot be silently substituted. The cutoff and sign of zero
invalidate the one-entry memo; no background truth or electronic history enters it.
The added native optical regression compares all pixel values exactly for interior,
edge, and full-field supports and three cutoff values. No native timing or pass is
claimed before root execution.

The phase-only cutoff capacity is derived from C=height+1 coarse cutoffs,
M=C-floor(C/3) possible strict-or-one-sided plateau minima, 36 null-refinement
queries per minimum, at most five canonical initial-cutoff representations, eleven
independent trust starts with 1+64*(2+12) cutoff queries each, and selected plus
eleven local-basin curvature checks with two neighbours each. A final factor of
three includes both covariance neighbours of every immutable snapshot cutoff:
3*(C+36*M+5+11*(1+64*(2+12))+2*(1+11)). Actual exceptional exhaustion still refuses.
No spatial-field bank, hypothesis removal, detector threshold, trial count, or scene
parameter is introduced by this resource accounting.

Root native fast-control checkpoint: four named cases / 1,435 assertions passed
in 9.99 s, including the unchanged 216-manufacture endpoint controls, trial-budget
and zero-motion basis controls, and the exact-pixel memo equivalence case. The
receipt is correction-fast-controls.xml (SHA256 665eb96d8f9a6f6d21229732d1876031dcd5933fa7a1cca8fbb43c2f50288e49).
These results precede the complete local-basin bank update; captured moving-family
and registered timeout acceptance remain pending root execution.

The V2 fixed-mode likelihood bank uses a certified partial-phase refusal; its
optimizer, physical null search, covariance derivatives, detector thresholds,
degrees of freedom and finite trial family are unchanged. An inconclusive
certificate executes the complete original likelihood. No approximate `Fit` or
invented residual is returned for a pruned hypothesis.

For the actual binary64 pole coefficients alpha and beta=fl(1-alpha), rows
L_j=e_j-beta*e_(j-1) use nonadjacent j>=1. Thus L L^T=(1+beta^2)I exactly in
real arithmetic. Nested stride8/4/2 banks select 16/32/64 phases for the current
128-phase policies. Selected regional phase scalars use the same native
four-subexposure, ascending-pixel loop as the full basis. The one-shape scalar
memo reuses them on full fallback; it neither changes an integral nor omits a
subexposure. Every required regional domain is checked before pruning so an
unrendered native refusal cannot be masked.

The native sequential projection is not silently replaced by an exact
orthoprojector. Its actual expansion is x_projected=x-sum(c_i*n_i)+delta.
Each native coefficient has an outward bound H_i obtained from the current
intermediate norm, the complete dot-product error, and absolute underflow.
Each individual exact I-n_i*n_i^T is nonexpansive when the checked squared norm
is at most two. A chosen twice-orthogonalized basis Q defines W=I-Q*Q^T;
the bound ||Q^T Q-I||<1 certifies the ideal contraction, and formation and
application rounding are included. Every original nuisance, including a
collapsed or nearly dependent transient, contributes H_i*||W L n_i|| to the
background-column error. No nuisance direction is truncated or assumed absent.
This avoids requiring a well-conditioned augmented matrix after L annihilates
the exact pole transient.

Only the two radiator columns require a singular-value certificate. Let their
computed normalized matrix be A, target y, data error epsilon0, and combined
column error eta. A posteriori QR reconstruction/orthogonality and triangular
inverse-residual bounds give a lower bound s on sigma_min(A). If s<=eta,
the complete likelihood is used. Otherwise any full native score no greater
than T implies ||z|| <= (||y||+epsilon0+R)/(s-eta), where R bounds
||W L r_native||. For an arbitrary computed witness w, refusal requires the
outward-certified strict inequality

|w^T y| > ||w||*(R+epsilon0)
            +(||w^T A||+||w||*eta)*(||y||+epsilon0+R)/(s-eta).

The score-to-norm bound includes rounded residual formation, squared products,
positive summation, and absolute underflow. The pole shortcut alpha*raw_j has
an explicit error for the actual update previous+=alpha*(raw-previous),
including real(1-alpha)-fl(beta). Unit-field clipping bounds the background
input; all native weighted products/divisions/sums are covered. Source and
data use their complete stored native projected vectors. Certificate
accumulations require an extended exponent range covering every binary64
product, nearest rounding, and gradual underflow; actual SSE FTZ/DAZ/rounding
control and fast-math status are checked. Unsupported or uncertain arithmetic
uses the full path. No floating-environment state is changed in production.

The retained exact-Fraction independent reference contains 300 perturbation
cases, including 180 full-score-admissible controls, with zero false certified
refusals. It also demonstrates false refusals from an unguarded residual and
from dropping a nuisance direction. This verifies the derived inequality;
native C++ controls and captured likelihood/runtime gates remain separate.
The added C++ controls exercise the production certificate, explicit FTZ/DAZ
fallback, and exact A/B full-basis bytes from the retained moving phase loop.
No native speed or real-time admission is inferred before those gates execute.

The conditioning and separable least-squares context is supported by
[O'Leary and Rust, sections 2.2-2.3](https://www.cs.umd.edu/users/oleary/software/varpro.pdf)
and the [LAPACK least-squares error analysis](https://netlib.org/lapack/lug/node83.html).
Those sources do not supply this partial-phase certificate, and their optional
rank regularization is not used to justify deleting a nuisance column here.


The V2 optimizer's source-only ranking bank now retains its exact projected
source phase vectors and norms across the at most five background-cutoff
starts. Geometry/branch and native source ordering are unchanged; only the
cutoff-dependent residual correlations are recomputed. All independent trust
continuations still execute. The explicit bound is 100 source entries per
branch plus four fast-branch diagonal entries: at most 204*N binary64 phase
values (835,584 bytes at N=512), released before mode scanning. This is a local
pure-optics reuse and does not retain a spatial-field bank, RNG, or prior truth.

The retained image integral also has an exact sparse-node fast path. Only
when sky/ground radiances are each in[0,1] and actual rounding is nearest does
it evaluate the original Gaussian x probability before the y-root work. A
zero probability returns the same signed zero. The native y factor has at
most three nonnegative clipped pieces, each at most one detector pitch; it is
finite and its zero is +0 under this rounding mode. Therefore this return is
bit-identical to the old zero-probability product. Nonzero probabilities keep
the original y operations, and radiances above one or non-nearest rounding
keep the entire original path. No quadrature tolerance, interval, node,
renderer outcome, or public layout changes. The additive A/B regression uses
four actual 64x64 field-edge/horizon opaque scenes, including reversed/fractional
unit and nonunit control radiances, against the immutable frozen image receiver;
it requires exact electron, noise, contrast and quadrant arrays. Native timing
and captured/full-population acceptance remain pending for this checkpoint.


Regional quadrature reuse is scoped to one visible opaque part in a single native
incident-field call. Exact binary64 x keys retain the original sorted cone edges
and midpoint light classifications; y integrals are cached only for the same
detector row and x probabilities only for the same column. The immutable scene,
plane, region, detector and sigma define this local context. The initial 8-segment
rule nodes are reserved before adaptive descent. Capacity=max(24,width+height)
is at most 256 nodes; misses compute the complete old expression and every
adaptive decision/tolerance remains unchanged. No global cache, retained
receiver state or new spatial-field bank is introduced. Nearest rounding and
FLT_EVAL_METHOD==0 guard exact stores. Added immutable-reference controls
compare IEEE bytes, explicitly distinguish signed zero, and exercise directed
rounding fallback with complete fenv/CSR restoration. Native admission remains
pending for this checkpoint.
