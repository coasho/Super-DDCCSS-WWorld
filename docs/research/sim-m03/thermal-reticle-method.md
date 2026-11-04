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
levels of bounded coordinate refinement. All candidates remain in the declared
finite optical field. A fixed global lattice bound, rather than the count of
data-adaptive queries, supplies the nominal Gaussian trial correction.

The original `detectionSigma` remains the base screen. Noise scale is estimated
from measured scalar charge, known read noise, ADC step and the response gain;
expected/source charge cannot supply it. The first-order filter has bounded
unit l2 gain. A Laurent–Massart Gaussian residual screen and finite Fisher
conditioning reject inconsistent or unobservable fits; amplitude and fitted
support widths are nuisance parameters in the bearing information matrix.
These are high-count engineering screens with a measured-count noise estimate,
not a certified equipment ROC or an exact low-count Poisson false-alarm bound.
Two resolved measured bearings are required before LOS-rate authority. A single
centered carrier does not manufacture a target angular rate.

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
