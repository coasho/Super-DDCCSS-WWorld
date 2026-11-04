# Current AAM sensing: production mechanisms and reference map

Current24 implement finite scalar RF/IR receiver and feature-level association, not a complete radar signal processor or focal-plane imager. Core equations are executable; the model-order gaps below remain distinct from equipment calibration.

This read-only audit binds current source and all 24 authored cards to exact SHA256 in the companion JSON. It changes no production input and claims no active G campaign outcome.

## What actually executes

### rf-energy

- `EngineeringSeekerV3::rfSnr`
- `World in-flight sceneFor`

- Pr=Pt*Gt*Gr*sigma*lambda^2/((4*pi)^3*Rt^2*Rr^2*L)
- N=k*T*F*B; processing_gain=B*exposure; rho=Pr*exposure/(k*T*F)

- ARH Rt=Rr and two receiver-path excess losses; SARH separate launcher-target and target-seeker ranges/losses, actual illuminating power/frequency/gain.
- Receiver and ARH transmitter gain equal authored peak within accepted cone; no aperture rolloff applied.
- Stationary coherent energy integration; waveform/duty/ambiguity/scalloping not resolved.
- SARH sigma is current monostatic RCS query used as an explicit bistatic engineering proxy.

### rf-detection

- `EngineeringSeekerV3::detectionProbability`
- `radar::probabilityOfDetection`
- `radarNotched`

- Pfa=exp(-threshold); Pd=Q1(sqrt(2*rho),sqrt(2*threshold))
- lookdown rho/=10^(clutter_db/10); reject when vertical<lookdown_cosine and abs(target_ground_radial_speed)<notch_m_s

- Steady RCS detector selected even though generic native radar library has fluctuating families.
- One effective independent background-cell Bernoulli trial/exposure; not all physical range-Doppler-angular cells.
- Lookdown/notch are geometry-driven reduced-order screening, not terrain clutter spectra or a Doppler filterbank.

### ir-radiometry

- `EngineeringSeekerV3::bandFraction`
- `EngineeringSeekerV3::irSnr`
- `EngineeringSeekerV3::sourceAspectCosineV3`

- fband=15/pi^4 * integral[xlo,xhi] x^3/(exp(x)-1) dx; x=hc/(lambda*k*T)
- E=Iref*(T/Tref)^4*strength*fband*exp(-extinction*R)/(R^2*10^(excess_loss_db/10))
- s=E/hypot(detector_noise,background_noise); Pd=0.5*erfc((threshold-s)/sqrt(2))

- Equivalent greybody point source and fixed scalar extinction; normal World aircraft/flare scene leaves temperature0, selecting card900K rather than modeled component temperatures.
- Body-forward aspect and afterburner envelope supplies scalar strength; no target class enters association.
- Fixed background term is positive noise, not spatial radiance or signed target/background contrast.

### association

- `NonImagingIrSeekerV2::sample`
- `ImagingIrSeekerV2::sample`
- `ActiveRadarSeekerV2::sample`
- `SemiActiveRadarSeekerV2::sample`
- `World::projectedSeekerExtent`

- Nonimaging unresolved direction=unit(sum(signal_i*direction_i)) within association gate.
- IIR association minimizes normalized width/hypot(width,height) residual; angular tie break.
- ARH cue/activation uses admitted launcher posterior; active detector provides noisy range/range-rate. SARH requires matching illuminator and both paths; angular only.

- IIR extents are geometric projections of currently attached damage geometry, resolution-censored/rounded; no focal-plane pixels or classifier.
- ARH selection is nearest angular candidate; measured range/range-rate do not form a joint probabilistic association gate.
- Single current track; no explicit multi-hypothesis management or target-acceleration state.

### finite-loop

- `EngineeringSeekerV3::sample`
- `EngineeringSeekerV3::resetTracking`
- `MeasuredPnGuidanceV2::updateMeasured`

- pointing rotation increment=min(gimbal_rate*dt,error*(1-exp(-dt/gimbal_lag))) subject to body cone
- alpha=1-exp(-elapsed/track_lag); beta=alpha^2; tangent direction-rate correction
- sigma_angle=hypot(angle_noise,angle_resolution/sqrt(12*SNR))
- sample period at constant decision step h:ceil(Tsample/h)*h; publication delay:ceil(Tprocessing/h)*h

- Body rotation uses actual frame axes; separate seeker gyro/encoder errors and radome/irdome distortion absent.
- Raster/measurement capture run only at actual decision boundaries; no intermediate exposures fabricated.
- Published angular variance is a conservative detector-error bound, not an estimator posterior covariance fit.

### authority

- `SeekerExecutionV2 private contract`
- `World delayed attribution scene map`


- Scene truth stays in signal formation; association receives direction/strength and only admitted channels.
- Referee attribution uses original measurement epoch and candidate index. Synthetic noise has no source identity and cannot inherit a previous true target.
- Independent seeded measurement/background streams; same-time reads cached, rewind/invalid inputs refused; reset clears pending/history.

## All 24 selections

| Card | Executed selection | Nominal exposure/delay | Angular cell | Family association |
|---|---|---|---|---|
| aim-120c | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| aim-120d | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| aim-7m | `fsim.seeker.semi_active_radar.v3` | 0.02 / 0.01s | 0.002rad | Angular nearest + matching illuminator/two paths |
| aim-9m | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| aim-9x | `fsim.seeker.imaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Angular + normalized extent consistency |
| asraam | `fsim.seeker.imaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Angular + normalized extent consistency |
| iris-t | `fsim.seeker.imaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Angular + normalized extent consistency |
| magic-2 | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| meteor | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| mica-em | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| mica-ir | `fsim.seeker.imaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Angular + normalized extent consistency |
| pl-10 | `fsim.seeker.imaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Angular + normalized extent consistency |
| pl-12 | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| pl-15 | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| pl-8 | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| r-27er | `fsim.seeker.semi_active_radar.v3` | 0.02 / 0.01s | 0.002rad | Angular nearest + matching illuminator/two paths |
| r-27et | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| r-27r | `fsim.seeker.semi_active_radar.v3` | 0.02 / 0.01s | 0.002rad | Angular nearest + matching illuminator/two paths |
| r-27t | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| r-37m | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| r-60m | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| r-73 | `fsim.seeker.nonimaging_ir.v3` | 0.02 / 0.01s | 0.002rad | Unresolved intensity centroid |
| r-77m | `fsim.seeker.active_radar.v3` | 0.02 / 0.01s | 0.002rad | Cue/activation + angular nearest; range/range-rate to guidance |
| super-530d | `fsim.seeker.semi_active_radar.v3` | 0.02 / 0.01s | 0.002rad | Angular nearest + matching illuminator/two paths |

All 26 new role resolves to a positive authored parameter with positive uncertainty. The JSON preserves each value, unit, basis, source and uncertainty; shared finite-sensor roles are not evidence of independent equipment identification. RF cards also carry required IR roles that their RF branch does not execute, and conversely IR cards carry dormant RF roles.

At constant 1/120 s native dt, frameSkip 1/2/4 yield exposure periods 25/33.333/33.333 ms for the current 20 ms role, and minimum publication delays 16.667/16.667/33.333 ms for the 10 ms role. These are caller quantization examples, not new native measurements.

## Primary implementations and literature genuinely read

### pyradi

[c7cfe9df96fdf0e6c258ee6fd624f62f19ebab06](https://github.com/NelisW/pyradi/blob/c7cfe9df96fdf0e6c258ee6fd624f62f19ebab06/pyradi/ryplanck.py)

Reviewed; planckel body executed only in independent artifact numerical oracle. Not linked/imported/copied into production.

Repository LICENSE:MIT; ryplanck.py retained file notice:BSD-3-Clause. Both preserved; do not infer away the file-level notice.

Read: PlanckConstants.__init__, planckel, planckInt.

Spectral exitance units and constants; independent wavelength integration versus production dimensionless band integration.

Equipment-specific spectra, emissivity, optics, actual focal plane or hardware calibration.

### stone-soup

[v1.9.1 resolved a4336b920a799cfe0a77ecb05867c5deeb371c7a](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/sensor/radar/radar.py)

Reviewed design/reference only; not executed, linked or ported into production.

MIT; retained LICENSE with copyright notices.

Read: AESARadar._snr_constant, AESARadar.gen_probability, AESARadar._swerling_1, ClutterModel.function.

Executable comparison for radar SNR, off-boresight beam gain, RCS fluctuation and measurement-space clutter concepts.

Upstream AESA uses North approximation; production reuses native steady-target Marcum-Q. Upstream ClutterModel returns empty without ground_truths; copying it directly would regress production empty-scene noise contacts.

### mixr

[Official 26.10 archive content SHA256192cc967c7850153fbfbb826f6caa9c72b13765f616374add9820b7d20c4bd82](https://s3.amazonaws.com/mixr-platform/releases/dev_v26.10/mixr_v26.10.tgz)

Reviewed production-simulation reference only; not executed, linked or ported into this production.

Archive license header LGPL-2.1-or-later; bundled text and current official license webpage print LGPL v3. Both preserved; integration would need exact selected-file/version license review. No production copy made.

Read: IrSensor::calculateIrQueryReturn, IRadar::receive, MergingIrSensor::mergeIrReturns, IIrSignature::getIrSignature.

IR target/background contrast, projected area/IFOV, NEI, atmospheric contribution and unresolved merging; RF signal/interference distinction.

Proprietary missile trackers or per-equipment numerical calibration.

### jsbsim

[Tracked vendored 1.3.1, main e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f plus exact current file hashes in production_sources](https://github.com/JSBSim-Team/jsbsim/tree/v1.3.1)

Adopted native flight/actuator physics runtime. Its generic filter code is not the implemented seeker servo; EngineeringSeekerV3 has its own exponential angular response.

COPYING LGPL 2.1; individual inspected source notices retain version 2-or-later.

Read: README introduction/features, FGFilter::CalculateDynamicFilters, CMake project version.

Actual vehicle dynamics/frame/actuator execution boundary, not RF/IR detector or image fidelity.



### palumbo

[2010 article, revised 2018 PDF; retained content hash in source manifest](https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Palumbo_Principles_Rev2018.pdf)

Method structure already cited by production documentation; source reviewed again, not an executed model or numerical calibration.

Literature reference; no implementation copied; article rights retained.

Read: Mechanization of PN and LOS reconstruction pp. 29–32, Radome/irdome requirements and distortion pp. 32–36.

Separating sensor measurement, gimbal/gyro reconstruction, guidance filter and vehicle response; highlights missing radome/irdome bias and gyro-error dynamics.



Retained source bytes, immutable Git commit IDs or archive hashes, and license notices are under `sensing-reference-sources/`. No Stone Soup or MIXR code is integrated. Native `radar::probabilityOfDetection` is the detector implementation actually reused by V3; JSBSim is the adopted vehicle plant, not an RF/IR engine.

## Independent radiometry check

The artifact-only script executes the inspected official pyradi `planckel` scalar function body and integrates wavelength with SciPy quad, then compares against a literal Python transcription of the C++ dimensionless Simpson method. It does not load native SDK or install packages. Fifteen explicit temperature/band combinations and nine irradiance cases retain exact numeric results in `sensing-radiometry-comparison.json`.

Maximum absolute fraction difference: 3.76299318372e-08. At 900 K, 3–5 µm fraction is 0.35894628824601; card reference 0.325 yields irradiance ratio 1.10445011768 (about 10.45 percent higher band irradiance than that normalization). This is a disclosed normalization approximation, not a hardware-fit conclusion or an instruction to retune reach.

## Highest-impact missing model order

1. **image-formation-and-ir-contrast**: Band-resolved target/background scene formation, signed contrast and a finite image formation/centroid/shape measurement pipeline.

   Current IIR distinguishes quantized geometric extents but cannot produce PSF blur, partial pixel filling, saturation, image clutter, target/background crossover or image-based flare rejection. Fixed 900 K scalar source gives flares/aircraft no independent spectral histories.

   Concrete next mechanism: Versioned native spectral radiance/signature interface with temperature/emissivity/projected area; sky/terrain radiance and band transmission; bounded focal-plane/PSF sensor producing only measured features, photon/read/dark noise and finite integration. Preserve nonimaging centroid family separately.

   Independent physical checks: Wavelength radiometry energy/units and zero-contrast disappearance; subpixel centroid/PSF/partial-area limits; two sources crossing resolution; fixed input never class-label discrimination.

   Missing model-order functions, not hardware-calibration-only; implementation requires source window and new native integration/performance evidence.

2. **rf-resolution-clutter-and-angular-gain**: Angle-dependent seeker aperture gain and explicit measurement-space resolution/false-alarm population; target/clutter Doppler and finite waveform processing.

   Current hard-cone peak-gain RF gives equal receiver gain at center and edge; range/range-rate noise constants do not scale with waveform/SNR, and false-alarm count does not scale with search volume. Geometric notch cannot show continuous clutter-edge or bistatic Doppler behavior.

   Concrete next mechanism: Versioned aperture pattern and bounded statistical range-Doppler-angle cells with declared bandwidth/coherent interval and CFAR/Pd model; terrain/clutter backscatter and jammer input as received interference, plus explicit bistatic Doppler and RCS method. No need to require full RF carrier time sampling to execute these functions.

   Independent physical checks: R^-4/bistatic legs, off-axis gain/beam conservation, clutter-only populations with zero targets, Pd/Pfa under noise/jamming and reference ambiguity/Doppler cases.

   Missing model-order functions, not a request merely to adjust hardware constants.

3. **los-instrument-and-estimation-model**: Own gyro/encoder/boresight/radome errors and covariance-consistent LOS state estimation with range/range-rate association where available.

   True body-axis transport plus first-order finite servo and alpha-beta filter omit sensor-to-body distortion/cross-coupled stabilization errors; published covariance cannot support an equipment posterior claim.

   Concrete next mechanism: Versioned measured inertial/angular instrument inputs and radome/irdome distortion transfer; bounded estimator with explicit process/measurement covariance and innovation gates.

   Independent physical checks: Stationary LOS with rotating body and gyro bias; radome error-slope coupling; delayed/missed exposures; covariance/innovation consistency, not only finite output.

   Missing instrument/estimator order; actual equipment coefficients remain separately uncalibrated.

4. **sampling-and-parameter-identification**: Exposure substeps/captured motion integration and model-specific independently identified receiver/signature constants.

   All 24 share the26new finite-sensor coefficients except family-level intensity differences; positive 50 percent intervals are screening assumptions, not 24 equipment characterizations. Native cadence follows decision quantization rather than declared nominal50Hz.

   Concrete next mechanism: Keep original epochs; if finer exposures are required, capture actual native pose/signature history or run sensor at native substeps. Identify coefficients using independently sourced/held-out per-variant data and report sensitivity when unavailable.

   Independent physical checks: Exposure integral under motion versus constant-scene limit; unchanged-criteria cadence convergence; per-variant held-out signature/detection/track response.

   Motion-integration model order plus separate hardware parameter-identification/calibration limits.

These are concrete functionality differences. A successful scalar detection, finite loop or complete encounter does not demonstrate image formation, spectral countermeasure discrimination, RF clutter/ECCM processing, radome calibration or equipment accuracy. Implementing additional mechanisms needs an authorized source window and actual native cost/behavior evidence; those changes were not made during this audit.

Upstream integration caution: MIXR IrSensor returns true target position/velocity/acceleration, and Stone Soup TrueDetection retains ground-truth provenance. Those reports cannot be copied into the SDK actor sensor path. Reuse their physical methods at the scene boundary while retaining this project’s angular/passive channel limits and World-only attribution.
