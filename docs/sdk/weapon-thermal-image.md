# Native thermal focal plane and quadrant receiver

`ThermalSeekerExecutionV1` is an additive private execution interface selected
only by a new explicit optical binding. Retained V2 and scalar V3 keep their
existing meanings. Registration resolves every `ThermalImageConfigV1` field;
zero-initialized policy is invalid. Public structs and plugin vtables do not
change. The root owns card parsing, World integration and native acceptance.

## Scene and radiometry

World supplies an epoch-aligned `ThermalSceneFrameV1`: band-integrated sky and
ground radiance, local up, and opaque emitting/occluding angular patches. Each
patch carries projected width/height, direction/rate, range, surface radiance,
path transmission and path radiance. Multiple patches can represent the same
physical object. A separate optional attribution-index vector is read only
after measured features exist; image formation and tracking never read it.

Radiance is W/(m² sr), angular dimensions are radians, direction rate is the
derivative of a unit ECEF ray per second. A grey surface helper evaluates
`epsilon * sigma * T^4 * PlanckBandFraction / pi`. World may instead supply
independently calculated band radiance. The effective wavelength and quantum
efficiency are explicit band-equivalent detector parameters, rather than an
implicit conversion between radiant energy and photons.

`ThermalImageReceiverV1::expose` resolves opaque visibility in continuous angular
coordinates before optical response or detector sampling. A canonical near-to-far
edge sweep selects the visible surface in every rectangle-arrangement cell;
adjacent cells with the same owner coalesce. Disjoint subpixel patches cannot
attenuate one another. Foreground radiance at the receiver is
`transmission * source + (1-transmission) * path`. Thus a source equal to its
background produces no expected contrast; a colder source produces negative
contrast. Both are processed without source classifications.

Finite exposure uses the declared number of midpoint samples over the past
integration interval, with the supplied angular rate extrapolated backward.
This is an executing constant-rate exposure model; it does not invent future
poses or claim captured native history. The separable Gaussian optical PSF is
convolved with each continuous visible source rectangle before pixel integration.
In one dimension the response is
`integral_source [Phi((pixel_high-s)/sigma)-Phi((pixel_low-s)/sigma)] ds`.
Its closed form uses the primitive `z*Phi(z)+phi(z)`. Subpixel displacement
therefore changes the detector response continuously. Nine-sigma source support
omits less than `3e-19` Gaussian probability per axis; detector/FOV losses are
never renormalized into the image. The angular chart ends at +/-1.55 rad;
near-hemisphere chart clipping is separate from the Gaussian tail bound.
Uniform background uses an exact integral. Across the native conical horizon,
analytic roots in `tan(y)` partition sky and ground; bounded adaptive Simpson
integration in x integrates their continuous optical response. Background and
the background displaced by opaque patches use the same physical integral.
Projected rectangles and a Gaussian PSF are declared engineering shape/optical
models, not equipment-specific optical prescriptions.

The exact rectangular-pyramid pixel solid angle is evaluated from its four
corner antiderivatives. The expected photoelectron count is

`N = L * aperture_area * solid_angle * optical_transmission * integration_time
     * quantum_efficiency * effective_wavelength / (h*c)`.

Pixel response is first-order with an explicit time constant. Its retained
state responds over the actual exposure spacing, starting with the integration
interval at the first exposure. Photon and dark counts are Poisson, read noise
is Gaussian, full well clips charge, and ADC clipping/quantization follows
readout. Counts far above saturation avoid enormous Poisson draws only when
the well/read-noise tail is more than twenty standard deviations away.
Repeated epochs return cached counts; rejected epochs draw no random values.
Reset clears response, image and RNG state. Identical builds/seeds replay the
same counts; the standard-library Poisson sampler does not promise bit identity
between different C++ standard-library implementations.

The captured `pointing` retains the exact exposure argument. The optical basis
normalizes it internally; a diagnostic must not normalize that argument again
before replay, because a second normalization can change subpixel charge by a
few floating-point bits. Exact input/history replay preserves both expected
charge and actual detector draws without weakening the zero-error predicate.

## Measurements and tracking

Imaging processing estimates local background from measured annular pixels.
Signed significance selects connected components; each component supplies its
measured flux centroid, bounding shape, signal-to-noise ratio and angular error
bound. No projected source extent is copied as a measured shape. Two optical
sources may resolve into separate components, merge under blur, saturate or
disappear at background crossover. The existing imaging family associates
these image-derived angular/shape measurements; a configured angular gate and
the retained finite alpha-beta response track their history.

Nonimaging processing integrates four signed quadrant photocurrents relative
to a measured outer guard detector. It includes both active-detector noise and
the common guard-reference uncertainty. The normalized differences are
`Dx=(right-left)/total`, `Dy=(top-bottom)/total`. For an isolated Gaussian spot,
`Dx=erf(x/(sqrt(2)*sigma_psf))`; inversion supplies the measured error. This
response is evaluated from detector counts, not a source bearing plus noise.
An absent or statistically unsupported total contrast produces no error.
Opposing sources can cancel, and an extended or saturated spot can bias or
ill-condition the response. Resolved uncertainty follows the inverse-response
slope; a confidence interval reaching a normalized-ratio endpoint instead
uses a conservative full-detector diagonal squared error bound. This bound
is not a calibrated Gaussian posterior or proprietary reticle reconstruction.

`EngineeringSeekerV3::sampleThermalV1` feeds only these measurements into the
existing finite pointing, exposure, tracker and processing queue. It disables
the old synthetic one-cell IR false contact on this path: detector noise now
creates image/quadrant false measurements itself. Passive range remains
unavailable. Original capture/processing/publication epochs remain distinct.

The selected path requires a valid `SeekerNavigationPoseV1` at the exact frame
epoch. It does not silently use true pose when navigation is missing or stale.
Target cues are transformed by `C_actual * transpose(C_estimated)` before the
physical gimbal command. Detector rays are reconstructed by
`C_estimated * transpose(C_actual)` after image processing. Association uses
estimated own position/velocity and reconstructed body forward. Thus attitude
error affects homing while raw photon/image formation remains physical. World
must use the same estimated pose in its guidance-facing frame. On-rail carrier
alignment is an explicitly supplied pose, not an implicit truth fallback.

`thermalImageV1()` exposes the last actual counts, expected counts and measured
features through a private diagnostic sibling. Its epoch is the captured image
epoch, not a renewed telemetry timestamp. Actor interfaces do not gain access
to source patches or referee attribution through this diagnostic.

The centroid and extent shown beside the raw image describe the strongest
current detector feature. They do not identify the homing family's selected
or delivered track. `selectedBlob` remains zero because that association is
not exported by this interface; the delivered guidance LOS has its own
measurement and delivery stamps. Segmentation labels remain current image
component numbers, never simulation identities.

Referee attribution is a separate pass after all measurements exist. Each
source's *visible* signed detector contribution follows the same finite pixel
response as the image. Scoring integrates that charge over the measured component
mask, or over the quadrant detector, with the expected full-well transfer factor.
An entirely hidden bright source contributes no charge and receives no credit.
No unoccluded source-flux proxy supplies native holding or fuze attribution.
Scene attribution indices are current-frame slots, not stable object identities.
At each new exposure, historical named charge transfers into an unattributed
response bank; only newly visible current-frame charge can name a current slot.
The detector retains that historical energy, which can suppress an incorrect
weaker current attribution. At most 128 current source slots and one unknown
charge image are retained. None of this bookkeeping enters detection or
association. Stable cross-frame source provenance would require an additional
explicit identity contract rather than assuming that a reused index means the
same object.

## Additive dual-waveband imaging execution

[MBDA's MICA manufacturer page](https://www.mbda-systems.com/products/air-dominance/mica-family/mica)
identifies IR MICA as dual-waveband imaging infrared. The `mica-ir` card selects
`thermal_dual_image.v1`, admitted through the additive `ReceiverConfigurationV2`
envelope and `ThermalDualBandSeekerExecutionV1` sibling interface. Existing V1
scene/configuration layouts and virtual methods are unchanged. The old V1 parser
continues to reject an unknown dual binding explicitly; current production
admission uses V2. Missing channel data cannot select a single-band fallback.

`ThermalDualBandSceneFrameV1::channels` carries two actual scenes with a shared
exposure epoch, horizon and navigation pose. World independently integrates
Planck radiance for each passband and applies that band's atmospheric
transmission/path radiance to aircraft and flares. Each receiver has its own
aperture throughput, PSF, photon energy, quantum efficiency, response, dark/read
noise, full well and ADC. Separate deterministic noise streams form two actual
images; the second image is not a rescaled copy. Validation of both scenes
precedes either detector update, so a rejected channel consumes no noise state.

Dual processing detects local extrema in measured contrast, with optical-scale
nonmaximum suppression and a maximum of 32 retained peaks per channel. This
separates resolved local maxima even when their optical tails make one
threshold-connected island. Whole-island centroids can represent different
source mixtures in different bands and cannot be paired as individual objects.
Same-polarity peaks pair by angular distance within an explicit gate, with each
peak used at most once. A shared angular aperture then measures both images at
the same location. Its radius is the larger of twice either channel's PSF sigma
and 1.5 times either pixel pitch. A 4x4 detector-area quadrature supplies partial
pixel weights; measured pixel variances propagate with squared weights. Aperture
centroid and significant-pixel extent are measured from those detector counts.
Each channel independently enforces its declared `minimumFeaturePixels`: support
is the number of detector pixels intersecting the aperture with the feature's
polarity at or above half that channel's detection threshold. A partial pixel
counts once as detector support while its photometric/noise weight remains its
fractional aperture area. Both independent support minima must pass before a
paired feature exists; a compact peak cannot bypass a `width*height` minimum.
Finite aperture contamination and unresolved sources remain physical sources
of spectral error, not evidence of a truth-labelled countermeasure.

Signed aperture photoelectron contrasts are converted to radiant contrast using
each channel's declared photon/throughput gain. The measured colour is
`log(abs(signal_0/gain_0))-log(abs(signal_1/gain_1))`; its first-order variance is
`1/SNR_0^2+1/SNR_1^2`. An acquired measured colour survives for an explicit finite
memory interval. Subsequent components must satisfy the configured log-colour
gate plus three propagated standard deviations. Candidate ordering is diagnostic
and does not choose spectral history. The existing native imaging family selects
its actual measured exposure sample using its reference/shape association.
`EngineeringSeekerV3` maps that raw selected sample index back to the dual
detector feature and calls `commitSelectedMeasurementV1` before any referee
index mapping or finite track filtering. Thus spectral memory follows the
measurement actually selected by native association, including when a brighter
off-axis differently coloured return is present at initial acquisition.
The receiver never infers that selection from sorted SNR, a filtered LOS or
referee provenance. Standalone detector callers must explicitly commit their
selected measured feature; exposure alone does not update spectral memory.
Only the latest exact exposure epoch and a valid feature index can commit.
Identical epoch/index retries succeed without mutation; a changed same-epoch
selection or stale/future/invalid request fails without changing history.
Neither target type, simulation identity nor an assumed aircraft/flare colour
class is read. A countermeasure with the same measured colour can remain
indistinguishable. Saturation, merged components and background contamination
can change the measured colour and therefore the native association.
Referee attribution is evaluated only afterward from the physically visible
retained charge near the measured aperture; both channels must agree on its
provenance. A connected island's brightest object cannot own another local
peak merely because they share a segmentation label.

Only paired, spectrally admitted measurements enter the existing finite family
tracker, gimbal and capture/processing/delivery queue. Passive range remains
unavailable. The diagnostic `thermalDualBandV1()` reports both images and measured
spectral features; legacy `thermalImageV1()` does not publish a fabricated
single-channel composite. Native integration and diagnostic consumers must
explicitly select the additive sibling.

MBDA does not publish exact bands, focal-plane coefficients or countermeasure
algorithms on that page. The card's 3-5 um and 8-12 um bands, 4/10 um effective
photon wavelengths, second-channel detector coefficients, pairing gate and
colour/memory policy are explicit engineering assumptions with nonzero
screening uncertainty. They are not attributed as manufacturer measurements.
The software grid and gates are selected numerical policies; their uncertainty
fields do not establish equipment confidence intervals. The implemented second
channel and measured discrimination mechanism execute even while these
equipment coefficients await calibration.

The second-channel dynamic range requires its own radiometric design. For the
card's 0.0015 rad pitch, 0.001256637 m² aperture, 10 us integration, QE 0.6 and
10 um effective photon wavelength, an exact central-pixel solid angle gives
555111 photoelectrons per W/(m² sr) at throughput 0.65. Independent wavelength-domain
Planck integration gives 8-12 um blackbody radiance 14.5593 W/(m² sr) at 250 K
and 38.5004 W/(m² sr) at 300 K: settled means of 8.082 million and 21.372 million
electrons exceed a one-million-electron well. Initial response lag can hide
this saturation; first-image existence alone cannot validate a fitted channel.
An assumed neutral attenuation to throughput 0.01, with 0.005 screening
uncertainty, gives a 300 K background mean about 328800 electrons without changing
either exposure duration. It is a dynamic-range design derived from a declared
300 K background, not a fitted MICA hardware measurement or engagement outcome.
Hot extended sources may still saturate physically. The root's actual-card
settled-background gate must establish the applied policy's native behavior.

The native domain is bounded: 8–128 pixels per axis, at most 128 scene patches,
1–16 exposure samples, PSF sigma between one-eighth and eight pixel pitches,
and at most 32 detected components. Backward angular displacement is at most
0.5 rad per exposure. Invalid policies/scenes fail explicitly. These bounds
admit deterministic finite work; actual scene cost must still be measured.
The complete finite image is scanned before the strongest 32 components are
retained; image raster order cannot exclude a stronger later feature. Patch
ordering includes angular motion and excludes referee identity.
For N admitted rectangles, the visibility sweep visits at most `(2N-1)^2`
arrangement cells per exposure sample before owner-run coalescing. It does not
recursively propagate an expanding fragment list. Horizon quadrature has eight
initial x intervals and at most ten adaptive bisection levels per interval.
Finite work bounds do not constitute real-time performance evidence; maximum
population, maximum exposure sampling and horizon cases require measured cost.

## Sources and independent physical checks

The implemented photon/dark/read/full-well/ADC ordering is grounded in the
inspected [pyradi rystare implementation at commit c7cfe9d](https://github.com/NelisW/pyradi/blob/c7cfe9df96fdf0e6c258ee6fd624f62f19ebab06/pyradi/rystare.py).
The underlying high-level sensor treatment is [Konnik and Welsh, arXiv:1412.4031v1](https://arxiv.org/abs/1412.4031v1).
The normalized quadrant response and Gaussian inversion follow [Narag and
Hermosa, arXiv:1703.00719v1, equations 1–2 and Gaussian response](https://arxiv.org/abs/1703.00719v1).
These are physical methods, not missile hardware constants. No upstream source
code is copied into production; the native implementation is independently
written. The reviewed pyradi file retains a BSD-3-Clause notice while its
repository license is MIT; original research copies/notices are retained by
the root under the persistent SIM-M03 research entry.

Continuous optical response before detector sampling also follows the
[Spitzer IRAC PSF/PRF calibration documentation](https://irsa.ipac.caltech.edu/data/SPITZER/docs/irac/calibrationfiles/psfprf/)
and the primary software method documentation for
[Photutils' pixel-integrated Gaussian PRF](https://docs.astropy.org/projects/photutils/en/latest/api/photutils.psf.GaussianPRF.html)
(the native rectangle antiderivative is independently derived; no library
implementation is copied). An independent Simpson source-plane Gaussian
oracle checks displaced subpixel rectangles and true finite-FOV energy loss.
Separate regressions cover disjoint opaque subpixel additivity, partial overlap,
fully occluded bright attribution for imaging and quadrants, measured two-band
colour continuity without identity information, and dual finite delivery/replay.

Focused native cases independently check absolute radiance-to-electron units,
Planck band radiance, pixel solid angle, optical flux conservation, exposure
motion broadening, resolved/merged components, opaque occlusion, cold contrast,
quadrant error direction, measured shape/identity separation, shot/read noise
statistics, saturation, invalid-scene replay and actual finite wrapper delay.
No native result is asserted here before the root executes those cases and the
ordinary per-card World/installed/scene gates. Equipment coefficients remain
explicit uncertain inputs; that fact does not replace implementing a required
function or passing its physical predicates.


## Finite nonimaging optical centering

A quadrant Gaussian inverse loses angular-magnitude information near a signed
ratio of one. A position-only servo can therefore lag an attainable optical LOS
rate even though the detector still observes a significant imbalance. Native
V3 now carries a coarse centering demand through the same measurement-processing
packet, rather than applying an immediately available raw image to the gimbal.

For each measured axis, the existing detector SNR supplies the conservative
ratio-error scale `2/SNR`. With the admitted detector significance multiplier,
`delta = detectionSigma * 2/SNR`, a signed ratio `d` has a significant sign but
unresolved magnitude when `abs(d) > delta` and `abs(d) + delta >= 1`. That axis
requests one physical detector half-field in the measured direction. A resolved
axis retains its measured Gaussian inverse. This is a bounded corrective
command, not an invented angular measurement or a new equipment parameter.
The existing gimbal lag, rate and mechanical cone still determine actual motion.
No source identity, target position, referee attribution or desired hit enters it.
Only an accepted tracked optical packet can install that demand; ordinary packet
delivery, dropout, retention and reset own its lifetime.

The original fixed physical detector fixture uses a 0.5 rad/s source, unchanged
64-pixel optics and the actual admitted processing/servo policy. The position-only
baseline tracks 35/51 observations and reaches 0.140082 rad error, beyond its
0.048 rad half-field. The current centering mechanism tracks 51/51 with maximum
error 0.0381966 rad. This is an engineering mechanism result, not equipment
calibration or all-model/native/installed/Viewer acceptance.

The [NASA/GSFC NGSLR Hardware Manual v0.3, 2011, section 5.14, p.30](https://space-geodesy.gsfc.nasa.gov/docs/2011/NGSLR_Hardware_Manual_v0.3.pdf)
describes closing a gimbal loop by balancing four measured quadrant signals.
The pinned [MIXR IGimbal reference](https://github.com/doughodson/mixr/blob/1b4029419e382bb532f1e142f9c490dc9adc27f7/src/models/system/IGimbal.cpp)
was reviewed for bounded position/rate servo organization. Neither supplies
missile-specific gains. The native implementation reuses its existing finite
SDK servo; no external controller or upstream code is copied. This coarse
conditioning rule is an explicitly derived engineering controller, alongside
the previously documented Gaussian response/sensitivity reference. It does not
claim a reconstructed proprietary Sidewinder reticle or counter-countermeasure
law.


## Measured background and endpoint validity

The guard-ring quadrant approximation requires a spatially uniform reference
field. Its readout first removes the known dark offset and normalizes guard
counts by the exact pixel etendue relative to the central angular cell. A
uniform radiance must not produce a fictitious error just because corner pixels
subtend less solid angle. The median of these measured equivalent counts supplies
the reference; the actual scene radiance or source attribution never supplies it.
The guard samples retain their noise pairing when a separate copy is ordered
for the median.

Read/shot/ADC variance is estimated from measured counts and the configured
readout policy. The common median-reference variance uses the conservative
largest normalized guard variance times `pi/(2*N)`. A guard residual larger
than `sqrt(k^2 + 2*log(N))` times its combined noise scale makes Gaussian
bearing inversion unavailable. Here `k` is the existing detection-sigma setting;
no equipment coefficient is fitted to an encounter. This is a Gaussian/asymptotic
median and union-tail engineering screening rule, not an exact low-count
probability or a manufacturer CFAR curve. A structured horizon can fail it while
raw charge, ADC values and quadrant photocurrents remain valid and observable.

Let `T` be measured total signed contrast, `D` either measured quadrant
difference, and `V` the modeled summed/reference variance. A physical difference
has magnitude at most the physical total. Noise affects both measured terms:
the existing confidence setting therefore admits
`abs(D) <= abs(T) + 2*k*sqrt(V)`. A fixed `abs(D/T) <= 1.05` cutoff incorrectly
rejected valid noisy endpoint observations. The corresponding ratio uncertainty
is `u = 2*k/SNR`. If `abs(D/T)>u` and `abs(D/T)+u>=1`, only a centering direction
is resolved; a local Gaussian derivative must not report a precise angular
magnitude there.

The independent 64-seed endpoint fixture exercises actual detector readout;
its original version wrongly rejected five admissible observations. A separate
32-seed offset fixture exposes the original angular overconfidence. The
nonuniform-guard fixture preserves raw detector values while refusing an
unsupported bearing. These are engineering method checks, not equipment accuracy.

## Optical balance and release continuity

The device-local quadrant loop continues even when precise bearing association
rejects a measurement. Both coarse and resolved optical errors pass through the
existing processing queue before the finite physical servo consumes them. The
navigation-facing associated track does not gate this local acquisition loop.

A credible quadrant difference describes a hemisphere of possible optical
balance directions. At capture basis `(o,d)`, a minimum offset `m` gives the
normal `n = sign * (d - tan(m)*o)` and constraint `n.dot(ray) >= 0`.
For an unresolved Gaussian ratio, `erf(z)<=2*z/sqrt(pi)` supplies the conservative
minimum `abs(m) = min(half_field, sqrt(pi/2)*sigma_psf*(abs(ratio)-u))`.
Resolved observations use their raw finite error for fine control. Historical
constraints retain their actual capture basis and time, rather than becoming
point rays that acquire incorrect cross-axis meaning after the gimbal moves.

In the current tangent chart, each hemisphere is a linear half-plane. The
native loop clips the actual finite detector rectangle by these constraints
and commands an interior vertex mean for unresolved balance. Oldest inconsistent
constraints are removed when measured source motion or changed aggregate
contrast invalidates them. Retention uses the existing track-memory policy;
the internal history is additionally bounded to 2048 entries. Empty/invalid
optical observations, reset and queue-overflow handling do not retain an
unlimited constraint history. No source position, identity or semantic decoy
label enters this solver. No fitted servo gain, enlarged field, faster exposure
or immediate processing path is introduced.

This is a local engineering set-membership centering construction, informed by
the uncertainty treatment of [Bertsekas and Rhodes, 1971](https://web.mit.edu/dimitrib/www/RecursiveStateEstimation.pdf).
Their general linear-system/ellipsoidal results are not a proof of this nonlinear
optical loop. [Narag and Hermosa](https://arxiv.org/abs/1703.00719v1) supplies the
Gaussian quadrant-response basis. The [2020 optical-communication study](https://www.mdpi.com/1424-8220/20/24/7164)
provides separate evidence that background/noise and nonuniform response affect
quadrant position estimates; its fitted compensation coefficients are not used.
These references do not identify an exact Sidewinder reticle or its gains.

The unchanged moving-source fixture (0.5 rad/s) now tracks 51/51 observations
with maximum optical error 0.0151864 rad against the 0.048 rad half-field. The
new diagonal stationary fixture originally oscillated by 0.035649 rad and tracked
30/61 observations. The retained half-plane implementation settles to maximum
error 3.54967e-6 rad and tracks 61/61 after the declared two-second settling period,
within the unchanged two-cell 0.003 rad numerical-resolution check. No source
identity is present in either fixture. Neither result substitutes for native
flight, all-model countermeasure, installed or actual Viewer acceptance.

The physical receiver is the same component before and after rail separation.
Preparation keeps its native scene, exposure, servo and processing clocks alive
while the existing public warming/powering-down query gate remains unavailable.
Successful physical release transfers ownership of that receiver, including its
actual pointing, optical response, pending packets and RNG state. A filtered
bearing alone cannot recreate those physical states. Retained scalar/legacy
providers keep their own initialization path.

For fitted midcourse/update capability, the optional private
`SeekerLaunchNavigationV1` seam seeds only the exact authorized initial fusion
estimate previously supplied to a fresh factory. It does not reset the physical
receiver. Disabled passive navigation and malformed estimates are rejected
before mutation; no new target truth, Agent method, public layout or old vtable
is introduced.
