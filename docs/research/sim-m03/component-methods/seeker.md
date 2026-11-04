# Seeker method, applicability and independent numerical references

Current-status update, 2026-11-03: the JSON now documents each current model
individually in theory, parameter and fidelity matrices. Its current `passed`
flag is false until a fresh actual native comparison under these source/card
hashes succeeds. Earlier72-row/836-constraint and older818-assertion results
are historical only and remain nested as `historical_evidence`. They do not
verify the new reticle receiver or the current scene-policy epoch. The separate
[72-cell parameterized table](per-model-effects-components.json) retains exact
source records and does not grant native/public/visual or calibration status.


This document describes the executing SDK methods for the **24 authored AAM
cards**, not 24 verified OEM seeker designs. The companion [seeker.json](seeker.json)
contains each original equipment selector, actual mechanism, full resolved
parameter records, provenance, uncertainties, theory delta and numerical
residuals. `equipment.<card>.seeker` establishes selection identity; it is not a
new hardware designation or a calibration result. A family artifact is admitted
only after the actual native observations pass the independent checker.

## Equations and actual mechanisms

The current V3 composition uses four native mechanisms: nonimaging IR, imaging
IR, semi-active radar and active radar. These share the finite gimbal, measurement
queue and track/navigation machinery, but receive through explicit physical
receiver bindings rather than substituting the retained scalar V2/V3 model.

For grey thermal sources, the vacuum spectral radiance is

\[
L_\lambda(T,\epsilon)=\epsilon\frac{2hc^2}{\lambda^5}
 [\exp(hc/(\lambda kT))-1]^{-1},\qquad
L_{band}=\int_{\lambda_l}^{\lambda_h}L_\lambda\,d\lambda .
\]

The primary basis is [NIST SP250-43, section2, equation4, printed page6](https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication250-43.pdf).
The reference uses the exact h, c and k in the [BIPM SI Brochure, section2.2](https://www.bipm.org/documents/20126/41483022/SI-Brochure-9.pdf).
NIST's historical rounded constants are not misrepresented as current exact SI.
The SDK integrates `x^3/expm1(x)` with composite Simpson quadrature, at least64
even intervals and approximately16 intervals per unit x, with the tail cut at80.
It uses the rounded second radiation constant0.01438776877 m K and the rounded
Stefan constant5.670374419e-8 W/(m² K⁴). Their small numerical differences are
included in the declared comparison budget.

Projected source rectangles are depth ordered and clipped into visible regions.
Gaussian optical blur is integrated over finite detector rectangles; off-detector
flux is lost, not renormalized. Exposure uses the explicitly selected number of
backward scene samples. The effective photon conversion is

\[
Q_{ph}=L_{band}\Omega A\tau t_{int} QE\lambda_{eff}/(hc),\quad
Q_{state}\leftarrow Q_{state}+(1-e^{-\Delta t/\tau_p})(Q_{ph}-Q_{state}).
\]

Dark current adds `Idark*tint`. Actual measured charge then has Poisson shot
noise, Gaussian read noise, finite full well and ADC quantization. The retained compatibility quadrant receiver derives signed contrast errors; the seven currently selected nonimaging AAMs instead use the distinct scalar conical-scan reticle described below. Imaging
receivers estimate background/contrast, segment finite image features and
associate measured feature direction/extent; they do not receive source IDs as
measurement inputs. MICA IR has two independent channel policies and explicit
spectral pairing gates. Its second authored band is included in the radiance
comparison; a common Planck pass does not independently calibrate dual-band
discrimination.

The RF equation is independently derived as transmitter flux, target scattering,
effective receive aperture and thermal noise:

\[
P_r=\frac{P_tG_t}{4\pi R_t^2}\frac{\sigma}{4\pi R_r^2}
       \frac{G_r\lambda^2}{4\pi L},\qquad
SNR=P_rt/(kTF).
\]

This follows [MIT Lincoln Laboratory's Radar Equation course, slides5-10 and13](https://www.ll.mit.edu/media/6946).
The coherent energy representation cancels matched-filter bandwidth; it does
not add a range bonus. Actual aperture gain comes from the authored aperture's
native Fourier pattern. Monostatic propagation uses the same range and excess
loss twice. Bistatic propagation uses separate transmitter/receiver ranges and
losses. SARH additionally requires actual captured illumination, pulse-program
phase/crop and temporal M-of-N PRF coincidence. ARH uses its own transmitter.
The production receiver synthesizes finite range/Doppler cells, noise and
clutter, applies CFAR/gates and temporal association, and produces measurements;
referee attribution is attached after detection.

## State, inputs, outputs and time

Inputs are the authored resolved receiver policies; finite time, boresight and
body axes; navigation estimate at the observation epoch; physical radiance
patches or RF echoes/ground patches; and a release/component RNG stream. Native
World scene formation owns position, altitude, skin/exhaust thermal estimates,
occlusion, atmosphere and actual illumination. Actor knowledge does not become
referee scene truth. The model's permitted launch estimate and later modeled
launcher updates are separate from a live seeker observation.

State includes finite gimbal/search direction and rate/lag; sample cadence and
processing-delay queue; detector temporal charge and RNG; RF illumination
clock/coincidence memory; measured local tracks/association; and navigation
reconstruction. Outputs include observation/sample/receive times, validity,
direction and optional rates/range, measured image features or spectrum cells,
and the selected track. Unknown range remains unavailable. A detector sample is
not an automatic homing lock. Equal-time rereads are cached; time reversal,
malformed policy/scene or absent required navigation is refused. Reset/retarget
semantics are those of the actual release provider and World; this document
does not grant an additional reset.

Finite sample timing, processing lag and captured navigation can cause a
coupled-chain miss even with a correct static radiometric equation. Source
extent/temperature, atmosphere, receiver FOV and gimbal/track dynamics must be
examined at their actual epochs before assigning a cause to a miss.

## Applicability, assumptions and equipment uncertainty

All 24 current cards select available mechanisms. That says nothing about an
OEM acquisition envelope. All full resolved values, units, original basis,
source IDs, notes and nonzero engineering uncertainties are retained in the
companion artifact; game-reference values remain game references. Common
mechanism code permits an equation-level equivalence cohort only with separate
native outputs for each model's parameters. It does not permit representative
equipment calibration or a substituted provider.

Aspect and range must be read for the exact variant. In particular, Magic2 must
not inherit Magic1's rear-sector restriction. The primary COMAERO account
[Missiles tactiques, printed page147](https://www.irsem.fr/storage/file_manager_files/2025/03/missiles-tactiques-comaero.pdf)
describes Magic2 frontal detection at least4.5km and a separate TRT electromagnetic
proximity fuze. This historical statement does not guarantee a5km kill against
the present target, altitude, thermal scene or missile state. R-60M must not
inherit original R-60 limits, or silently inherit R-60MK equipment parameters.
The [Polish Air Force R-60MK technical manual, Lot2821/89,1990](https://djvu.online/file/fUvs0bhsAlgza)
is a primary MK reference, not proof of an exact M head-on envelope. Its separate
thermal head and radar-fuze sections establish that MK has distinct functions;
they do not authorize a new M seeker model. Current M aspect/range coefficients
remain their disclosed engineering assumptions. An all-aspect label neither
guarantees head-on acquisition at every range nor excuses an observed miss.

The program's unchanged5km head-on and3km tail-turn population scenes have
different physical conditions. Both must retain their actual outcomes. A method
pass, availability label or aspect label cannot turn a native miss into a hit.

Known model limits include grey-body and effective-wavelength approximations,
rectangular projected supports, finite exposure sampling and frozen short-time
scene motion; reduced clutter/scintillation and waveform-equivalent processing;
assumed PSF, detector noise, target heat and atmosphere; and no admitted OEM
Pd/Pfa/ECCM calibration. Numerical tolerances below are not confidence intervals
for these physical uncertainties.

## Executed method comparison and reproduction

The historical authoring-time native fixture
`test_component_effect_reference_native.cpp` creates every original selected
provider, resolves its original physical policy and captures production methods.
It does not replace a descriptor with a scalar alias. For thermal cards it
captures band radiance at300/600/1000/2000 K, including MICA's second band, and a
uniform-scene detector charge through `ThermalImageReceiverV1::expose`. The charge
comparison verifies the actual imaging/quadrant policy's solid-angle, photon,
dark-current and temporal response path, before random noise/ADC. For RF cards
it captures both monostatic and bistatic production energy evaluations with
authored receiver parameters and actual native antenna gain. The separately
declared transmitter1000 W/linear gain100 and5/10km geometry are numerical
reference inputs, not an invented operational launcher fit.

`component_references_effects.py` uses55-digit Decimal analytic exponential-series
integration instead of native Simpson quadrature, factored link flux/aperture
instead of the collapsed radar expression, and independent photon-budget
evaluation. Predeclared relative budgets are1e-6 for Planck,1e-9 for charge and
2e-12 for radar energy, plus1e-12 absolute. The reference series has an explicit
positive geometric tail bound below1e-42. Independent wavelength-space numerical
quadrature, band additivity, grey scaling, link range/dwell/loss scalings and
failed/stale/foreign evidence controls exercise the checker itself.

The exact executed native and independent residuals, source hashes and passing
native report are in [seeker.json](seeker.json) and
[effects-results.v1.json](references/effects-results.v1.json). Until those actual
captures are admitted, their status remains incomplete. Source reference hashes
and scope are in [effects-primary-sources.json](references/effects-primary-sources.json).
The checker requires a pre-execution current input epoch and a nonzero passing,
non-skipped native report. Installed/public behavior and visual/full-chain gates
are separate evidence; this document is no substitute for them.

The historical pre-reticle Release capture passed the actual native Catch case with818
assertions. The independent comparison historically admitted all24 seeker rows across all
four mechanisms and88 numerical constraints, with the original budgets. The
shared input epoch and actual case-level JUnit are bound in `seeker.json`.
The whole four-entry CTest JUnit was retained separately; it was not rewritten
into a case report to satisfy admission. All72 effects-family rows historically passed
the historical matrix's144 separate theory/method gate validations. This records
the method scope above; the other matrix gates retain their own requirements.

## Exact current model differences

The seven nonimaging selections are `thermal_conscan_reticle.v1`, not the compatibility quadrant receiver. Their current native13-case first run had6 passes and7 fixture failures involving five nonunit test rays; normalized-ray rerun is pending. The actual new AIM-9M encounter still missed (CPA123.6438687m, age5.1667s; only4/1201 tracking rows, after age19.8s). None of that is promoted to a method or model pass here.

The reticle mask is applied before photon sampling: q=u+r(cos(phi),sin(phi)), T=1[|q|<=R]1[cos(n atan2(q))>=0]. Numerical optical cells feed ONE scalar detector; they are not imaging readout pixels. Complete phase-zero cycles, midpoint own-pose capture, pole state, measured-only waveform nuisance fitting and Fisher conditioning determine observability. Two resolved bearings are needed for a measured rate. Current inferred design uses R=min(W,H)*pitch/2, r=R/2, n=floor(pi*r/(8*sigma))=12 and f=1/(2*pi*n*tau)=132.629119Hz, with128 phases and4×4 spatial quadrature. The SA14 paper100Hz/1200Hz constants are not AAM data. See [the complete reticle method](../thermal-reticle-method.md).

| Model | Exact receiver | Acquisition cue m / gimbal deg | Cadence / delay s |
| --- | --- | --- | --- |
| aim-120c | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| aim-120d | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| aim-7m | rf_signal.v1 | 60000 / 60 | 0.02 / 0.01 |
| aim-9m | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| aim-9x | thermal_image.v1 | 13000 / 45 | 0.02 / 0.01 |
| asraam | thermal_image.v1 | 13000 / 45 | 0.02 / 0.01 |
| iris-t | thermal_image.v1 | 13000 / 45 | 0.02 / 0.01 |
| magic-2 | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| meteor | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| mica-em | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| mica-ir | thermal_dual_image.v1 | 13000 / 45 | 0.02 / 0.01 |
| pl-10 | thermal_image.v1 | 13000 / 45 | 0.02 / 0.01 |
| pl-12 | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| pl-15 | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| pl-8 | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| r-27er | rf_signal.v1 | 60000 / 60 | 0.02 / 0.01 |
| r-27et | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| r-27r | rf_signal.v1 | 60000 / 60 | 0.02 / 0.01 |
| r-27t | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| r-37m | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| r-60m | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| r-73 | thermal_conscan_reticle.v1 | 13000 / 45 | 0.02 / 0.01 |
| r-77m | rf_signal.v1 | 17000 / 60 | 0.02 / 0.01 |
| super-530d | rf_signal.v1 | 60000 / 60 | 0.02 / 0.01 |

### aim-120c

aim-120c: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.10668000000000001, receiver_rf_aperture_height_m=0.10668000000000001, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Only selected finite V2 AAM source and turning-CPA/station fuze. Explicit finite geometry differs from retained V0 cousins; no C7 hardware internal reconstruction is inferred.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `e7ad26a202c12753855b5a91df439b04680dc9397add0aad89f1b9a41ae719bb`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Only selected finite V2 AAM source and turning-CPA/station fuze. Explicit finite geometry differs from retained V0 cousins; no C7 hardware internal reconstruction is inferred. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-120d

aim-120d: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.10668000000000001, receiver_rf_aperture_height_m=0.10668000000000001, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. The card explicitly reuses C7 warhead information because exact D data are unpublished. Selected V0 source and CG sweep differ from AIM120C V2 despite equal total/charge/radius; do not equate the software constructions.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `54836e2c858069e4ef89367cd9d76946641e87fcea08ab70eb0c0d7e4c44a7a8`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: The card explicitly reuses C7 warhead information because exact D data are unpublished. Selected V0 source and CG sweep differ from AIM120C V2 despite equal total/charge/radius; do not equate the software constructions. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-7m

aim-7m: rf_signal.v1; acquire_m=60000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.1218, receiver_rf_aperture_height_m=0.1218, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `f7dc340def1ad096fdb31d00bd4effa7d6efcdcba80e2043ee9b630fbccf08ae`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-9m

aim-9m: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0007917304360898403, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. Reticle receive law is newly selected; WDU17/B label does not establish the single-detector/spoke hardware. Historical passive-fin assertion abort was a fixture failure, not a warhead result.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `d10455f1bfd2e195a73e467aeed201fcd15f2a6ee3009418d6b45f236b89f288`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Reticle receive law is newly selected; WDU17/B label does not establish the single-detector/spoke hardware. Historical passive-fin assertion abort was a fixture failure, not a warhead result. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### aim-9x

aim-9x: thermal_image.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0007917304360898403, receiver_quantum_efficiency=0.6. Imaging selection and its own card mass differ from AIM9M; shared WDU17/B reference is explicitly inherited, not independent measured X case geometry.

Parameter/source matrix: 86 resolved roles; original basis counts {"assumed": 86}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `876bc671addb5ec1fc9e7f1a4c2e9d919a5d799653fe665f3b0a41d33f44deef`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Imaging selection and its own card mass differ from AIM9M; shared WDU17/B reference is explicitly inherited, not independent measured X case geometry. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### asraam

asraam: thermal_image.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0013526519869112555, receiver_quantum_efficiency=0.6. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 86 resolved roles; original basis counts {"assumed": 86}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `328641597eaaa1551f90a1e63c9707c4410d5b54d1b86749dbd52015998a0e6c`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### iris-t

iris-t: thermal_image.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0007917304360898403, receiver_quantum_efficiency=0.6. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 86 resolved roles; original basis counts {"assumed": 86}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `f1d498294fd7306ec8c543fe5dac15e16dbc499b201f77a312e98a2407cf189a`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### magic-2

magic-2: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0012099549580989815, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. Magic2 frontal detection and a historical multi-element description are not evidence for this scalar reticle. Pre-fragmented wording is represented by equivalent point fragments, not an authored preform map.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `a611327d8851e464ac42c8f2f50345f50fe37d4e5a7f2e5289469ee704029336`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Magic2 frontal detection and a historical multi-element description are not evidence for this scalar reticle. Pre-fragmented wording is represented by equivalent point fragments, not an authored preform map. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### meteor

meteor: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.10679999999999999, receiver_rf_aperture_height_m=0.10679999999999999, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `b71c05fbe4fcc98582906922a08842e1e012e6253ee5e7ed52e57db52a377112`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### mica-em

mica-em: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.096, receiver_rf_aperture_height_m=0.096, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `53815f9175376a704166fdfa391d5f7844cc7db76f81a71bd9e9e3f1f75a021b`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### mica-ir

mica-ir: thermal_dual_image.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0012566370614359172, receiver_quantum_efficiency=0.6. The actual receiver is dual-band imaging with separate channel and pairing policies, not the single-band or scalar-reticle receiver.

Parameter/source matrix: 111 resolved roles; original basis counts {"assumed": 111}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `4a04a671660e9c01ec0bcd7d8212163543636dbeaf3d8548143f0dcb748f8f08`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: The actual receiver is dual-band imaging with separate channel and pairing policies, not the single-band or scalar-reticle receiver. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-10

pl-10: thermal_image.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0012566370614359172, receiver_quantum_efficiency=0.6. Ambiguous fragmentation/expanding-rod description is resolved by the authored fragmentation kind and selected zoned_case.v1; this source does not implement an expanding connected rod.

Parameter/source matrix: 86 resolved roles; original basis counts {"assumed": 86}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `c4952df4b7e95a0d90f9d735f058512b9e9ada7483a2fa9506e4cd4af314c513`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Ambiguous fragmentation/expanding-rod description is resolved by the authored fragmentation kind and selected zoned_case.v1; this source does not implement an expanding connected rod. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-12

pl-12: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.1218, receiver_rf_aperture_height_m=0.1218, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `f648212b0dba9b419da1d546cf106c2038a804229003280c5cebc16ae0efec7b`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-15

pl-15: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.1218, receiver_rf_aperture_height_m=0.1218, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `d3fcf56320613bd9a987c079d86b726322eededf722620ae41331a5d4f7fb828`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### pl-8

pl-8: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0012566370614359172, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `99a57a0a41a4e9c994f60fe6c843004e03c24115302c237c4951feea510f9fce`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27er

r-27er: rf_signal.v1; acquire_m=60000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.156, receiver_rf_aperture_height_m=0.156, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `2bb75662658c764d026dbfb323d8f32fefc2e3b599b3bdd3a4900e63a25d1090`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27et

r-27et: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0033183072403542195, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `7cdde5ea7dee28a792b26cc52c32eaf9d5f80f9b12f41c6bf2f412b0fa355265`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27r

r-27r: rf_signal.v1; acquire_m=60000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.138, receiver_rf_aperture_height_m=0.138, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `a6c652c709fc743e91964d63ed1fc3bbcd674958e72d9bf14129af030b939ceb`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-27t

r-27t: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0025967226777328135, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `dd36e416a24f61471aaa5945846e4aa2f59100240dd23965fcd50c5a39ebb5e5`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-37m

r-37m: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.22799999999999998, receiver_rf_aperture_height_m=0.22799999999999998, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `09f554a3fb58e9892ab71c6b747f6c818abf4e43eced0828bad905a274ca4977`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-60m

r-60m: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0007068583470577034, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. The card description mentions depleted-uranium rods, but the selected zoned-case source uses steel7850kg/m3 and discrete equivalent rods; no DU density, pyrophoricity or connected-ring capability is implemented by that name.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `b869f7c3072b2be2816221da7742b02ef37eadf93b3766990d987bbcedb7dd57`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: The card description mentions depleted-uranium rods, but the selected zoned-case source uses steel7850kg/m3 and discrete equivalent rods; no DU density, pyrophoricity or connected-ring capability is implemented by that name. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-73

r-73: thermal_conscan_reticle.v1; acquire_m=13000, gimbal_deg=45, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, ir_band_low_um=3, ir_band_high_um=5, receiver_pixel_pitch_rad=0.0015, receiver_psf_sigma_rad=0.00075, receiver_integration_s=1e-05, receiver_aperture_area_m2=0.0013364040624059708, receiver_quantum_efficiency=0.6, receiver_reticle_periods=12, receiver_reticle_frequency_hz=132.62911924324612, receiver_reticle_scan_radius_rad=0.024, receiver_reticle_stop_radius_rad=0.048. Rod description maps to discrete finite rod elements under zoned_case.v1, not connected_ring.v1; exact weld topology and material construction are not inferred.

Parameter/source matrix: 92 resolved roles; original basis counts {"assumed": 92}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `3cdc049d662ab8946df83a42e38eeeebdee04157295a97a080374636216da04e`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Rod description maps to discrete finite rod elements under zoned_case.v1, not connected_ring.v1; exact weld topology and material construction are not inferred. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### r-77m

r-77m: rf_signal.v1; acquire_m=17000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.12, receiver_rf_aperture_height_m=0.12, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `79fb56394f6bfb4ca13cbd91775cba966eab767476abb1c7d56e672b81a5d5df`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: Exact card variant and its own mass, charge, sensor and timing records are used. Shared code neither supplies undisclosed OEM internals nor establishes equipment equivalence. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

### super-530d

super-530d: rf_signal.v1; acquire_m=60000, gimbal_deg=60, sample_period_s=0.02, processing_delay_s=0.01, field_half_angle_rad=0.12, angle_noise_rad=0.0004, track_memory_s=0.12, frequency_hz=10000000000.0, receiver_rf_aperture_width_m=0.1578, receiver_rf_aperture_height_m=0.1578, receiver_rf_dwell_s=0.012, receiver_rf_prfs=3, receiver_rf_gate_m=15, receiver_rf_pfa_interval=1e-06. The card explicitly inherits Super530F warhead information; exact D case geometry and fragment distribution remain inferred.

Parameter/source matrix: 87 resolved roles; original basis counts {"assumed": 87}. Exact units, source IDs, source titles/URLs, notes and uncertainties are retained in this model's JSON row. Canonical parameter SHA256 `f01a6f21bff7a2f15e08a54de3226bd2e5e092822a054dc899e624c9198ba860`; selected mechanism-source SHA256 `cc9203c44477f59e6dde85c620f1a1083147218f2bdd75bfdcad80b240a77202`.

Fidelity matrix: The card explicitly inherits Super530F warhead information; exact D case geometry and fragment distribution remain inferred. Exact selected receiver binding matters: old thermal_quadrant.v1 compatibility results do not verify thermal_conscan_reticle.v1. Navigation/measurement epoch error, eclipse, false image/waveform fits, clutter/flare contamination and unresolved angular rates can break the coupled chain despite correct static photon/RF arithmetic. Current source/parameter mapping is documented; fresh parameterized native execution and equipment accuracy remain separate requirements.

## Primary reticle architecture and scope

[Navy TP8347](https://ed-thelen.org/pics/Radar-NAWCWD-TP-8347.pdf), printed
7-1.33–37, supports spatial masking and temporal angle encoding.
[Howser1995](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V16-N01/16-01-Howser.pdf),
pp27–33, includes scan modulation, detector/electronics, scene artifacts and
processing in a primary seeker simulation. These sources motivate a distinct
receiver architecture; they identify neither our seven AAM reticles nor our
assumed scan/noise constants. Uniform radiance, off-field flux, short pulses,
clutter and waveform observability remain required separate tests. The dated
first13-case status above is a historical snapshot pending root's fresh run,
not a claim about a run completing later.

## Parameter equality is a disclosed modeling gap

When complete numeric policy and selected mechanism/receiver coincide, the
JSON lists `identical_numeric_policy_models` explicitly. This is reused
engineering parameterization, not proof of identical hardware or separate
per-model tuning. Every model still needs its own actual capture. Different
equipment selection IDs, source notes and metadata hashes alone do not supply
different physical response, OEM internal construction, or calibration.

MICA-IR has two actual optical policies:3–5um and8–12um. The second
channel uses its own10um effective photon wavelength, QE0.6,10us integration,
noise/ADC state and spectral-pairing gates. These exact second-band records are
in its physical parameterization as well as the complete parameter matrix;
common band-integrated radiance does not by itself verify spectral association.
