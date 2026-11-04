# Finite engineering weapon sensing

The current 24 AAM cards select versioned V3 sensor execution. Retained V2 IDs
still mean ideal geometric measurements with unavailable servo dynamics; their
configuration and numerical behavior are unchanged. V3 is a simulated engineering
receiver, not an exact hardware calibration or a proprietary countermeasure law.

World supplies candidate scene geometry, signatures, associated illumination and
propagation. The receiver computes received signal before measurement formation.
Association never receives entity identity or an aircraft/decoy label. ARH admits
range and radial-speed measurements; passive IR/IIR/SARH tracking remains angular.
Referee attribution retains the original scene epoch and index through delay.

RF uses the monostatic/bistatic radar equation with actual transmitter power,
gain, wavelength, target RCS, both spreading ranges and excess propagation loss.
Thermal noise is k*T*F*B; matched integration gain B*exposure gives the energy SNR.
Seeded detection reuses the native steady-target Marcum-Q detector with the
explicit threshold's exp(-threshold) false-alarm relation. IR uses a Gaussian
irradiance-noise threshold. A declared lookdown-clutter screening penalty depends
on scene elevation; this reduced-order penalty is not a spatial terrain map.
One independent effective angular-cell noise trial per exposure can produce an
unattributed contact even with no target. The declared threshold sets its false
alarm probability. It never inherits a previous true target for fuze purposes.
This is one modeled cell, not an executed complete focal-plane or FFT search
population. Angular covariance availability reports the explicit modeled detector
error bound; it never implies measured hardware calibration. Quantized geometric
extent features remain distinct from radiometric band response and pixel imagery.
The explicit bandwidth cancels algebraically; it is not an additional sensitivity
gain. Candidate illumination and both path gates still apply to SARH.

IR uses point-source band irradiance: equivalent broadband radiant intensity,
greybody Planck band fraction, inverse-square spreading and Beer-Lambert
extinction, divided by detector/background quadrature noise. Band integration is
dimensionless Planck integration with Simpson panels no wider than 1/16 and a
negligible x>80 tail cutoff. A supplied scene source temperature scales total
greybody power by (T/T_reference)^4 as well as changing the band fraction.
The current cards declare a 3-5 um band and a 900 K equivalent source. The
equivalent intensity is normalized to the inherited nominal reach using an
explicit reference band fraction of 0.325; it is an assumed design input, not
measured target thermography. Flare strengths and scene temperature remain
separate physical inputs; labels cannot authorize discrimination. Projected
extents below angular resolution are unavailable; resolved extents are quantized.
V3 source aspect uses actual target body-forward orientation, including at zero
speed or sideslip. The retained V2 envelope keeps its legacy velocity proxy.

Each receiver owns finite pointing lag/rate, body-relative mechanical stops,
exposure cadence, processing delay, a bounded delivery queue, angular estimation
and its seeded noise stream. Actual body axes transport the gimbal during body
rotation before its finite stabilization response. A retained angular estimate
drives tracking; an admitted navigation estimate may cue search. Measurements
below threshold or outside the instantaneous field produce no detection.
LOS estimation uses a tangent-plane alpha-beta response with finite time constant.
Guidance predicts delayed bearings to decision time without renewing sample age.
One missed exposure preserves the angular association, filter and pointing cue
only inside the existing authored track-memory interval. It still reports no
current detection or referee attribution; its retained track keeps the original
sample epoch. A later actual detection inside that interval continues the same
local track and filter. Expiry discards angular association without resetting
authorized navigation, active-radar phase or either random stream. Retained V2
providers keep their previous behavior.

Physical V3 ARH receivers with admitted inertial guidance also reconstruct
target navigation from their own measured channels:
`p_target = p_own + range * LOS` and
`v_target = v_own + range_rate * LOS + range * LOS_rate`.
All terms use the same capture's estimated own pose and filtered measurements;
range, range rate and LOS rate must actually be available. No passive IR/SARH
range is inferred. The resulting inertial estimate retains its original capture
epoch through the ordinary processing/delivery queue. After terminal acquisition
it takes precedence over launcher coordinates during receiver eclipse/coast,
preserving the correlation with own-navigation error. This is a constant-velocity
engineering coast, not a covariance-qualified target filter or a new detection.
Launcher estimates still support acquisition and preterminal midcourse. Reset,
receiver reconfiguration and an incompatible new local track clear the old
onboard target estimate; a new contact needs its own complete measured state.

When the same capture has an actually delivered, supported launcher estimate,
the ARH coast uses its transverse velocity prior and the onboard measured Doppler:
`v_target = v_prior + LOS * (range_rate + v_own dot LOS - v_prior dot LOS)`.
The target position still uses onboard range/LOS and the correlated estimated
own origin. This projection is applied on every eligible radar capture, rather
than switched on by range, eclipse or an expected outcome. It preserves the
measured/filtered LOS, range, Doppler and detection/attribution.
Eligibility requires the authored launcher-update capability, actual support,
an original finite nonfuture state epoch, no adopted-prior epoch regression,
and predicted-bearing consistency within the existing acquisition cue gate.
Absent, unsupported, reordered or inconsistent priors use the measured-only
velocity above. Passive channels acquire no new measured range or velocity.

The retained source prior keeps its original fused state epoch; this interface
does not expose the underlying measurement or receive epochs. The native
weapon communication channel separately enforces underlying update age,
expiry and link state before providing the input. A later support arrival
without a radar capture cannot rewrite a frozen coast or renew its capture
time. Reset and contact replacement clear both the coast and adopted-prior
history. The projection does not claim a Cartesian posterior covariance or
zero prior uncertainty. An incorrect transverse prior can bias the coast,
including a cue-consistent countermeasure association; it is not a proprietary
target discriminator. Tests deliberately preserve this possible prior error.

On those same eligible supported captures, terminal guidance uses the kinematic
angular-rate estimate
`LOS_rate = (I - LOS * LOS') * (v_prior - v_own) / range`.
This is the transverse derivative of the relative-position vector, not a
Doppler measurement of transverse velocity: the projected Doppler term cancels.
The prior state epoch remains separately retained; the current track epoch
describes the current bearing/range capture and does not renew that velocity
source. The ordinary finite bearing tracker remains the unsupported/passive
fallback. Support loss therefore retains real angular measurement noise rather
than replacing it with perfect kinematics. No optimal weighting or proprietary
filter claim follows without an admitted prior covariance. The relative-motion
identity follows the engagement geometry in [Palumbo et al., Basic Principles
of Homing Guidance](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_Principles_Rev2018.pdf);
the cited paper does not establish these assumed equipment coefficients or the
accuracy of this support-selection rule.

Expired angular association still reports no tracking or attribution and does
not manufacture a track. A release-owned V3 guidance phase latch preserves Lost
after terminal acquisition and prevents launch loft from restarting on detector
loss. It commits only after accepted guidance inputs and a finite command;
rejected packets cannot advance it. Retained V2 behavior is unchanged. The
optional private guidance histories reset alongside the sensor at the existing
native pose-reset boundary; authority, tasks and C2 ownership retain their
established semantics. The original guidance provider vtables are unchanged.
These separate sensor/estimator/guidance functions follow the homing-loop structure in
the Palumbo and Jackson references below; no proprietary hardware law is claimed.

Exposures occur at the next decision boundary after their period is due; original
scene/sample epochs are retained. Timing quantization is at most one World decision
step. A coarse decision step does not invent intermediate scene samples. Repeated
reads do not consume samples or redraw noise. Reset clears queues, tracking,
pointing and RNG state; a stable per-release seed isolates independent receivers.
Search uses a central-row-interleaved raster. Its row spacing derives from the
detector field and mechanical limit; the central row is revisited within two
sweeps. Search phase starts at the sensor's first accepted sample, including on
rail where no launch age exists. Finite lag/rate still limit achieved pointing.

`processingReadyTimeS` is the nominal processing deadline. `deliveredTimeS` is
the actual first decision boundary making that packet available; retained reads
never renew it. For a constant decision interval h, the implemented exposure
period is ceil(samplePeriod/h)*h and the minimum publication delay is
ceil(processingDelay/h)*h. In particular, a nominal20ms exposure under33.3ms
decisions runs at30Hz, not50Hz. Quantized motion/exposure and publication errors
must be measured across frameSkip1/2/4 in native/public integration.

All 24 cards explicitly resolve every new coefficient, unit, method and positive
uncertainty. The shared 50 percent screening intervals are not confidence bounds
from equipment data. Sampling, pointing, thermal/RF, radiometric and estimation
constants require independent sensitivity and dynamic checks; uncertainty cannot
excuse failed engineering tests. Native/installed/fleet results are recorded only
after their root-owned execution. No such gate is claimed by this method page.

The finite homing-loop separation follows [Palumbo et al., JHU/APL 29(1), pp.29-34](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_Principles_Rev2018.pdf).
The flight-control/sensor distinction follows [Jackson, pp.9-14](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf).
These references support method structure, not the fitted numerical constants.
Planck radiance follows [NIST SP 250-43, section 2](https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication250-43.pdf).
