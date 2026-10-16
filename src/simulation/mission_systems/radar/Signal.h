#pragma once
// The radar at the signal level (ADR-30, MS-6): what a pulse-Doppler radar's
// receiver and processor make of one dwell, sample by sample.
//
// A dwell is split into coherent intervals, one per PRF of the radar's
// schedule. In each, every range gate between transmissions holds one
// complex sample a pulse; across the interval's pulses each gate is
// windowed and Doppler filtered. The chain builds those filtered samples
// directly - the Doppler-domain values an FFT of the windowed pulses would
// give, with their exact distribution:
//   - receiver noise: white complex Gaussian in each filter, then the
//     window's few DFT taps (a cosine-sum window convolves the spectrum
//     with them): the same samples an FFT of windowed white noise gives;
//   - a target: its echo in the gates its folded range straddles (the
//     compressed pulse's triangle), eclipsed where its echo meets a
//     transmission, at its folded Doppler: the window's transform there, in
//     closed form; its amplitude drawn for the interval from its
//     fluctuation (the intervals' frequencies differ: independent).
// Then the processor: cell-averaging CFAR along range in each Doppler
// filter, a cell detected that crosses its threshold and is the largest of
// its neighbours; the PRFs' M of N, each interval's detection unfolded to
// every range and range rate it may be and counted where M intervals agree;
// monopulse angles from the difference channels at the detected cell. The
// threshold is the one the detection level designs for the same false-alarm
// time (Radar::dwellPfa), so both levels promise the same false-alarm rate.

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace fsim::ms::radar::signal {

/// A deterministic stream: xoshiro256** (Blackman and Vigna), normals by Marsaglia and Tsang's ziggurat.
class Stream {
public:
    explicit Stream(std::uint64_t seed = 1) noexcept { reseed(seed); }
    void reseed(std::uint64_t seed) noexcept;
    std::uint64_t next() noexcept;
    double uniform() noexcept { return static_cast<double>(next() >> 11) * 0x1.0p-53; } ///< [0, 1)
    double normal() noexcept;

private:
    std::uint64_t s_[4] = {1, 2, 3, 4};
};

/// One coherent interval: its PRF and what it gives.
struct Interval {
    double prfHz = 0.0;
    int pulses = 0;               ///< in the interval (its Doppler filters)
    int gates = 0;                ///< range gates in a pulse repetition interval
    int blankedGates = 0;         ///< the first gates, while it transmits: no samples
    double ambiguousRangeM = 0.0; ///< c / (2 PRF)
    double ambiguousRateMs = 0.0; ///< lambda PRF / 2
    double pulseS = 0.0;          ///< the transmitted (uncompressed) pulse
};

/// A dwell's waveform: its intervals, one per PRF.
struct Waveform {
    std::vector<Interval> intervals;
    int m = 3;                    ///< detected in at least m of them
    double gateM = 150.0;         ///< range gate (the compressed pulse's resolution)
    double rateResolutionMs = 0.0; ///< lambda / (2 interval): the unambiguous grid's range-rate cell
    double wavelengthM = 0.03;
};

/// The medium-PRF schedule of a radar at `frequencyHz`: `prfs` PRFs geometric between the ones whose blind speeds are
/// 150 and 300 m/s (the same design for every radar, in speed: a lower frequency's PRFs lower in proportion), each
/// interval dwellS / prfs, transmitting `duty` of the time, its gates `gateM` apart.
Waveform mediumPrf(double frequencyHz, double dwellS, int prfs, int m, double gateM, double duty);

/// A target as one dwell sees it.
struct Echo {
    double rangeM = 0.0;
    double rangeRateMs = 0.0;     ///< positive opening
    double snr = 0.0;             ///< the mean SNR (linear) the whole dwell's energy would give it, in the sum channel
    int fluctuation = 1;          ///< 0 steady, 1 Rayleigh (Swerling 1/2), 2 a dominant scatterer (Swerling 3/4)
    double monopulseAz = 0.0;     ///< the difference channels' ratio to the sum its direction gives: slope x offset /
    double monopulseEl = 0.0;     ///< beamwidth (azimuth right, elevation up)
    std::uint32_t truth = 0;
};

struct Settings {
    double pfaInterval = 1e-6;    ///< a cell's false-alarm probability in one interval (before the M of N)
    int cfarCells = 16;           ///< reference cells (half each side)
    int cfarGuard = 2;            ///< guard cells each side
    int coincidence = 1;          ///< the M of N's tolerance: cells either way in range and in rate
    double rangeScaleM = 150e3;   ///< unfolded no farther
    double minRateMs = -1e9, maxRateMs = 1e9; ///< the range rates searched
    /// The processor's notches: the filters within `notchMs` of the main beam's clutter (its range rate, NaN: the
    /// beam meets no ground) are not tested, nor those within it of zero rate in the gates of the altitude line (the
    /// ground straight below, at `altitudeM`; NaN: none).
    double mainBeamRateMs = std::numeric_limits<double>::quiet_NaN();
    double notchMs = 0.0;
    double altitudeM = std::numeric_limits<double>::quiet_NaN();
};

/// The ground's clutter in a dwell: for each interval, each receive gate's Doppler filters' power (before the window,
/// in units of the noise a pulse) - [interval][(gate - blankedGates) * pulses + filter]. Clutter is a sum of many
/// scatterers: Gaussian, its power added to the noise's in each filter.
using ClutterMap = std::vector<std::vector<double>>;

/// A declared detection: M intervals agreeing.
struct Hit {
    double rangeM = 0.0, rangeRateMs = 0.0;
    double monopulseAz = 0.0, monopulseEl = 0.0; ///< as measured (difference / sum, real part)
    double snr = 0.0;                            ///< estimated, the agreeing intervals' summed (linear)
    int intervals = 0;                           ///< that agreed
    std::uint32_t truth = 0;                     ///< the echo most of its cells were made by (0: noise)
};

/// What a dwell's processing did (for the tests and the displays).
struct Stats {
    std::uint64_t cells = 0, peaks = 0, hits = 0;
    double clutterS = 0.0, processS = 0.0; ///< wall time building the clutter, and in dwell()
};

/// One interval's processed cells as a range-Doppler display shows them (MS-6c): each receive gate's filters' power
/// over the noise's in a filter, and the cells the processor took as peaks (before the M of N).
struct Snapshot {
    int interval = 0;               ///< the interval kept (the waveform's first: its lowest PRF)
    bool kept = false;              ///< the dwell had it
    int rows = 0, filters = 0;      ///< receive gates (the first `firstGate` blanked), Doppler filters
    int firstGate = 0;
    double prfHz = 0.0, gateM = 0.0, ambiguousRangeM = 0.0, ambiguousRateMs = 0.0;
    double mainBeamRateMs = std::numeric_limits<double>::quiet_NaN(); ///< notched about it by notchMs (NaN: no notch)
    double notchMs = 0.0;
    double altitudeM = std::numeric_limits<double>::quiet_NaN();      ///< the altitude line blanked (NaN: none)
    std::vector<float> power;       ///< [row * filters + filter], over the noise's mean in a filter
    std::vector<std::pair<int, int>> peaks; ///< (row, filter)
};

/// The ways, along one axis, m detections agree within a tolerance of `tol` cells either way: the (m - 1)-tuples of
/// offsets that span, with 0, no more than 2 tol (m 3, tol 1: 19).
int coincidences(int m, int tol) noexcept;
/// The share of noise's threshold crossings that are peaks (the largest of their neighbours) under CA-CFAR with
/// `cfarCells` reference cells at `pfa`: a crossing's power is the threshold and an exponential beyond it (given it
/// crossed); its two Doppler neighbours are correlated with it through the window's taps, the rest independent - a
/// conditional simulation, its stream fixed (the same answer every time).
double peakSurvival(double pfa, int cfarCells) noexcept;
/// An interval's false-alarm probability a cell for a dwell's `pfaDwell` per unambiguous cell. False agreements a cell
/// come at C(n, m) K^2 (c q p)^m: K = coincidences(m, tol) in range and in rate, c the share of each interval's
/// gates it receives in (not transmitting), q = peakSurvival(p) - solved for p.
double intervalPfa(double pfaDwell, const Waveform& waveform, int coincidence, int cfarCells) noexcept;
/// The same for a peak survival `q` already known (it changes slowly with p: a radar keeps it while p stays near).
double intervalPfa(double pfaDwell, const Waveform& waveform, int coincidence, double q) noexcept;

/// Cell-averaging CFAR's threshold multiplier on the reference cells' mean power for a false-alarm probability with
/// n reference cells in noise: (1 + alpha / n)^-n = pfa.
double cfarAlpha(int n, double pfa) noexcept;

/// The window's taps on the Doppler filters (a 4-term Blackman-Harris: its spectrum is the filters' convolved with
/// them), and its losses.
struct Window {
    static constexpr int kTaps = 4;
    static constexpr double a[kTaps] = {0.35875, 0.48829, 0.14128, 0.01168};
    /// The noise power in a filter, per unit sample power and pulse: sum of the taps' squares (each side's half).
    static double noiseGain() noexcept;
};

/// A compressed pulse's response in a range gate, its amplitude against an echo whole and on the gate: the echo starting
/// `x` gates after its transmission's start, the gate at `gate`, the pulse `chips` gates long (its compression ratio),
/// the PRI `period` gates. Each of the code's chips overlaps its own in the gate's matched window by 1 - |gate - x| (the
/// straddle); what of that overlap the receiver hears - after its transmission, before the next - is the response (the
/// eclipse, gate by gate: an echo meeting the next transmission loses only what of it falls in that gate's window).
/// The code's range sidelobes are left out (an ideal compression, tolerant of Doppler).
double gateResponse(double x, int gate, double chips, double period) noexcept;
/// The share of the noise a gate's matched window hears: its part after the transmission and before the next.
double noiseShare(int gate, double chips, double period) noexcept;

/// The chain's mean processing loss (dB) for a target fluctuating as `fluctuation` (Echo's: 0 steady, 1 Rayleigh, 2 a
/// dominant scatterer), against the detection level's ideal (each interval 1/N of the energy, M of N at the same
/// interval threshold, the same fluctuation): the SNR its Pd is 0.5 at over the ideal's. Each interval's Pd is CA-CFAR's
/// - for a Rayleigh target its closed form, (1 + alpha / (N (1 + S L)))^-N; for the others the fixed threshold's exact
/// Pd averaged over the reference cells' mean (a table over S L) - averaged over where the target falls: its gate
/// straddle and eclipse (gateResponse: the larger of the two gates, over the noise that gate hears), its filter's offset
/// (the window's scalloping), L the window's loss and those; then the M of N. The ideal's interval threshold is
/// `idealPfaInterval` where given (> 0: a detection level's own, which counts its false agreements otherwise), else the
/// chain's. Deterministic: about a millisecond for a Rayleigh target, some tens for the others (their table).
double processingLossDb(const Waveform& waveform, double pfaInterval, int cfarCells, int fluctuation = 1,
                        double idealPfaInterval = 0.0) noexcept;

/// One dwell: its echoes through the waveform's intervals and the processing. `stream` draws the noise and the
/// fluctuations.
std::vector<Hit> dwell(const Waveform& waveform, const std::vector<Echo>& echoes, const Settings& settings, Stream& stream,
                       Stats* stats = nullptr, const ClutterMap* clutter = nullptr, Snapshot* snapshot = nullptr);

} // namespace fsim::ms::radar::signal
