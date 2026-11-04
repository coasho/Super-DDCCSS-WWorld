# Quadrant angular-rate observability

The nonimaging thermal receiver forms four measured charge integrals. For
an isolated Gaussian spot against an admissible measured background,
`dx=(QR-QL)/Q` and `dy=(QT-QB)/Q` have response
`d=erf(theta/(sqrt(2)*sigma_psf))`. Inverting that response near `|d|=1`
is ill-conditioned. The existing confidence test marks a channel unresolved
when `abs(d)>u` and `abs(d)+u>=1`, with
`u=2*detection_sigma/SNR`. Neither that test nor its inputs changed.

An unresolved measurement retains a centering constraint and its conservative
angular-error bound. It cannot supply a resolved angular position for a finite
difference. In particular, differentiating the clamped inverse response as the
gimbal moves produces an apparent target velocity even for a stationary source.
The track therefore sets `rateAvailable=false` and canonicalizes its derivative
payload to zero. Two resolved measurements must bracket a new angular rate. An
unavailable-rate track-only seed carries no separate resolved-position flag;
the receiving physical quadrant implementation conservatively waits for two
fresh resolved samples. Reset, expiry and invalid-track rejection clear this
private observability state. The public track layout is unchanged.

For two admitted resolved points, the existing filter remains
`alpha=1-exp(-dt/tau)`, `beta=alpha^2`,
`p_pred=unit(p_prev+dt*v_prev)`,
`p=unit(p_pred+alpha*(p_meas-p_pred))`, and
`v=tangent(v_prev+beta*(p_meas-p_pred)/dt,p)`.
No noise draw, source radiance, detector threshold, servo bound, processing
delay, sample cadence, guidance gain or equipment-card number changes.

This observability rule applies to the current `thermal_quadrant.v1` bindings
for AIM-9M, Magic2, PL-8, R-27ET, R-27T, R-60M and R-73. They share the same
measurement/inversion mechanism; their individual card parameters and native
execution remain separate matrix inputs. The rule does not establish that
these seven weapons use identical physical detector or reticle hardware.

The source/seed regressions first retained eleven and one failing assertions,
respectively. The subsequent Release run executed all 32 engineering-sensor
cases and 7,255 assertions successfully. The same unchanged AIM-9M encounter
still misses: its captured CPA changed from 165.11396243092608 m to
187.8849499460225 m after the rate-availability repair. Those misses and the
original terminal/damage assertions are retained in
`build/architecture-reviews/sim-m03-production-20261103/quadrant-rate-observability/`.
This sensor regression does not close the full-chain component gate.

The separate structured-background confidence rejection remains active.
Resolving a rate-availability bug does not justify treating rejected horizon
clutter as an isolated Gaussian target. Scan/reticle background discrimination
requires its own physical mechanism and numerical reference.

Primary references:

- [Narag and Hermosa, Gaussian quadrant-detector response](https://doi.org/10.1364/JOSAA.34.001212):
  the Gaussian charge-integral response; not missile-specific calibration.
- [Howser, Measuring and Modeling Scan Modulation of an Infrared Seeker](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V16-N01/16-01-Howser.pdf):
  distinct optical waveform, dynamics and signal-processing stages in a
  scan-based seeker. It does not identify AIM-9M equipment constants.
