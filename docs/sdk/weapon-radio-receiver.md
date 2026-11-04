# Native waveform and aperture weapon receiver

The additive private `RadioSeekerExecutionV1` selects an actual pulse-domain-
equivalent processor. The separately selected scalar V3 and retained V2 paths
retain their meaning. A new card binding must explicitly resolve
`RadioReceiverConfigV1`; the root owns parsing, cards, World and native gates.

## Physical inputs and received energy

World supplies epoch-aligned target echoes and ground patches containing only
direction, receiver/transmitter ranges, path-delay-equivalent range, actual
Doppler, RCS, both excess losses and illuminator power/gain. A separate optional
source-index vector supplies referee scoring only. No target ID or source class
enters signal processing or association. Source order is canonicalized from
physical observables before noise and fluctuation draws.

The configured `rf::Pattern` is the native aperture Fourier/Hankel pattern,
including declared shape, taper, efficiency, sidelobe floor and back response.
Mechanical pointing uses the actual gimbal face; electronic steering uses the
actual body face and direction cosines. Pattern gain is evaluated separately
for every echo and terrain patch. This replaces the selected path's constant
peak gain and hard detection cone. Transmitter and receiver use the same
pattern for ARH; SARH uses the actual illuminator gain and the seeker's
receiving pattern.

On the physical ARH route the existing family's nonmutating
`ActiveRadarSeekerV2::previewPhaseV1` supplies the same authorized-estimate
activation calculation used by its retained `sample`. Standby executes cue,
gimbal and navigation/phase decisions but does not transmit, form receiver
cells or consume RF noise. Actual activation starts a finite coherent window;
the first raw capture waits for the complete configured/pulse-quantized CPI,
then incurs normal processing and delivery delay. Standby/warm-up phase reports
do not carry invented hardware measurement stamps. First configuration of a
fresh receiver preserves the launch-authorized estimate already supplied to
the family; reconfiguration resets tracking and clocks.

`RadioReceiverV1::energySnr` implements

`rho = Pt * Gt * Gr * sigma * wavelength^2 * coherent_dwell /
       ((4*pi)^3 * Rtx^2 * Rrx^2 * L * k * temperature * noise_factor)`.

Both actual propagation legs and excess losses execute. Monostatic range is
recovered when the legs coincide. The root supplies the RCS method and actual
illumination geometry; this receiver does not substitute a semantic target
type for a scattering model. As in the native radar model, a dwell assumes
approximately constant direction, power and Doppler over its coherent interval.

## Native processing reused

The implementation calls existing `radar::signal::dwell` from
`radar/Signal.{h,cpp}` directly. That accepted native chain builds the exact
Doppler-domain equivalent of finite pulse samples: compressed-pulse range
straddle and transmit eclipse, coherent tone phase, source fluctuation,
complex Gaussian receiver noise, four-term Blackman–Harris window, range
cell-averaging CFAR, local peaks, multi-PRF M-of-N coincidence and ambiguous
range/Doppler hypotheses. Difference-channel noise and echo ratios supply
measured monopulse angles. No true bearing is added to an arbitrary error draw.

Ground patches form a finite physical clutter-power map in those same receive
gates and Doppler cells. RCS and aperture-weighted radar energy are apportioned
across the patch's folded Gaussian Doppler distribution and the actual gate
response. The Gaussian width is an explicit uncertain receiver/scene policy.
Broadband interference enters the map as received power divided by thermal
noise `k*T*F*B`, with the actual finite receive-window share. It therefore
changes noise power and the CFAR threshold rather than an invented rejection
probability. Native complex noise realization creates clutter/noise peaks even
with zero targets; the old one-cell synthetic false contact is not used here.

The native processor's measurement-space ground-clutter treatment is reused,
including its known local clutter-power screening. This does not claim a
proprietary ECCM classifier, a full sampled RF carrier, or calibrated terrain
backscatter. Statistical cell processing and its actual outputs execute in
production rather than being replaced by those unavailable hardware claims.

## Measurements, ambiguity and navigation

ARH receives measured range/range-rate only after waveform processing. Multiple
equally supported aliases remain explicit diagnostic hypotheses; unresolved
aliases do not become a true target range. SARH requires associated illumination
and its actual waveform supplied through the existing native radar `waveform()`
accessor. High-PRF unambiguous velocity can coincide directly in Doppler. Medium
PRFs use measured path-delay cells to resolve Doppler aliases internally. That
bistatic delay-equivalent quantity is never published as passive Cartesian
range or range-rate to the homing family.

The illuminator's native `illuminationClockV1()` view exposes only its existing
current-dwell start and native dwell period. Sequential interval pulse times
remain `start + interval_offset + n/PRF`; a 12 ms listener does not cause a
90 ms transmitter to visit all PRFs in 12 ms. The listener accepts only complete
pulses inside `[capture - receiver_rf_dwell_s, capture]`, starting no earlier
than its first observed continuous illumination. Native pulse rounding cannot
extend an interval through the next native dwell boundary. If a window crosses
two separated occurrences of one PRF, only the newest complete coherent block
is processed; separated blocks are not joined into an invented continuous FFT.
The configured M remains unchanged even when one window contains fewer PRFs.

`dwellObservedV1` executes the same native window/noise/CFAR/monopulse chain and
exports its folded measured cell peaks. Energy shares are proportional to the
actual selected `N/PRF`. Consequently each physical pulse has energy
`P_average/PRF`, independent of how much of its transmitter dwell was observed.
Actual transmitter pulse widths, compression gates and bandwidth determine
eclipse, range response and receiver noise. The ordinary radar `dwell` and
diagnostic-null paths retain their prior arithmetic and random draws.

`RadioTemporalCoincidenceV1` keeps the latest observed coherent block per
distinct native PRF within the existing explicit `track_memory_s`. Each measured
range/rate alias migrates to the newest capture using that candidate's measured
Doppler, `r_now = r_capture + rate * elapsed`; distinct-PRF agreement uses the
native configured cell tolerances and M. Noise/clutter peaks participate in
the same procedure. A current capture member is required, so history alone
cannot renew a detection. Original-capture navigation reconstructs every
measured bearing before retention; inverse-variance linear fitting of those
measured unit rays estimates the newest LOS and its finite measurement variance.
Scoring attribution is read only after association and comes from a current
capture member. No cached source ID or Cartesian target state resolves aliases.

Illumination loss, a changed program, a changed schedule phase, or an unobserved
gap longer than memory clears temporal history. A new association therefore
incurs the actual time needed to observe M PRFs, followed by the existing finite
processing delay and delivery quantization. Raw diagnostics expose actual
per-interval pulse start/end times, energy fractions, current observation bounds
and the coincidence observation span. They do not relabel old peaks as current
raw cells. As in the reused native dwell method, geometry and Doppler are held
at the captured scene epoch during each short coherent block; range migration
between separately captured blocks executes explicitly.

Measured sum/difference ratios become angular directions and SNR-dependent
measurement-error bounds. Family association receives only those processed
measurements, with an explicit angular gate. The finite pointing, capture,
tracking and processing-delay pipeline is shared with the thermal receiver.

The physical beam uses orthonormal forward, right and down axes. Its difference
channels encode azimuth `atan2(right, forward)` and elevation
`atan2(-down, hypot(forward, right))`. Both the ARH direct and SARH temporal
paths invert those spherical coordinates as
`forward*cos(el)*cos(az) + right*cos(el)*sin(az) - down*sin(el)` before
navigation reconstruction. Independent diagonal-source cases exercise both
paths in every angular sign quadrant and a rotated physical frame. The
unchanged native pulse, CFAR and receiver-noise processing executes in those
cases; their strong test-only illumination isolates the coordinate convention
without changing authored equipment parameters.

Both physical receiver selections require current admitted
`SeekerNavigationPoseV1`. Cue commands use `C_actual * C_estimated^T`; measured
rays use `C_estimated * C_actual^T` and the association/midcourse frame uses
estimated own position/velocity. Actual apertures, gimbals and RF geometry use
native physical axes. Missing, stale or malformed navigation refuses the new
path; it cannot silently restore true orientation. Root guidance uses the same
estimated own frame. Ordinary V2/V3 paths remain compatible.

`radioSpectrumV1()` returns the actual last waveform, capture epoch, received
range–Doppler snapshot, peaks, processed hits and cell/clutter/process statistics
through a private diagnostic interface. Referee source scoring remains outside
actor observations. No synthetic preview stands in for the executed spectrum.

## Bounded domain and verification

The receiver's own waveform design has at most five PRF intervals; an actual
native illuminator program and its observed subset admit up to 32, matching
the native radar processor's distinct-interval domain. There are 4–256 pulses
per processed interval,
131072 receive cells in total, 128 target echoes and 512 ground patches. The
unfolded coincidence domain is at most 64 * 1024 * 1024 theoretical cells.
The native search enumerates detected peaks and their aliases sparsely; it
does not allocate or scan that entire domain for every dwell. Actual received
arrays remain bounded by 131072 cells. Exact finite
PRF/gate/blanking/range-rate consistency is checked before processing; invalid
policies do not modify the accepted one. The configured coherent waveform fits
the selected exposure period. Actual native cost still requires measurement;
bounded work alone does not certify real time.

Temporal storage admits at most 32 PRFs and 4096 actual detected peaks per
retained block. Sparse alias/candidate work is capped at 1048576 entries and
512 retained hypotheses per detection. Exceeding capacity refuses that capture;
it never coarsens the physical gates, PRFs, search domain or M requirement.

The independent focused cases check monostatic/bistatic energy and gain scaling,
native cell/spectrum/peak execution, noisy monopulse, false peaks without
targets, actual ground-clutter/interference effects, SARH channel limits,
source-order/seed/reset/rejected-input invariants and navigation-coupled LOS
with unchanged physical spectrum. The root executes native and current-card
World/installed gates; this document does not claim their result in advance.
Additional cases enumerate a long transmitter's actual pulses independently,
exercise measured range migration across three captures, preserve original
navigation bearing epochs, distinguish repeated PRFs from distinct-PRF support,
reject expired/inconsistent/invalid histories and verify finite wrapper delivery.

The production implementations reused are
[`signal::dwell`](../../src/simulation/mission_systems/radar/Signal.h) and
[`rf::Pattern`](../../src/simulation/mission_systems/rf/Antenna.h), already used
by the SDK radar. The independent review also read
[Stone Soup AESARadar at v1.9.1/a4336b9](https://github.com/dstl/Stone-Soup/blob/a4336b920a799cfe0a77ecb05867c5deeb371c7a/stonesoup/sensor/radar/radar.py)
and the official MIXR 26.10 `IRadar::receive` source; those external detectors
are references, not replacement runtime dependencies. Their exact originals,
versions and notices are indexed in the persistent
[SIM-M03 research entry](../research/sim-m03/README.md).

The temporal ambiguity design also reviewed the actual public-domain U.S.
Naval Research Laboratory
[`disambiguateClust1D` source at 27317126c57c19f57f6d14eb6aa35700f57b869a](https://github.com/USNavalResearchLaboratory/TrackerComponentLibrary/blob/27317126c57c19f57f6d14eb6aa35700f57b869a/Mathematical_Functions/Signal_Processing/disambiguateClust1D.m):
bounded alias expansion, distinct-PRI gating and weighted measurements. This
source was reviewed, not copied or added as a runtime dependency. Its own
comments warn about multi-target ambiguity ghosts; the implementation retains
competing measured hypotheses rather than choosing a true state. The official
[MathWorks PRF waveform explanation](https://www.mathworks.com/help/phased/ug/waveforms-with-staggered-prfs.html)
and [range/range-rate ambiguity equations](https://www.mathworks.com/help/fusion/ug/track-objects-with-wrapping-azimuth-angles-and-ambiguous-range-and-range-rate-measurements.html)
were checked independently for pulse timing and the ambiguity periods.
