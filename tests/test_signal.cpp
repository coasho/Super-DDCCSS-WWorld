// The radar at the signal level (ADR-30, MS-6): its stream's normals, one
// interval's detection against the closed form for a fluctuating target under
// cell-averaging CFAR, noise alone against the threshold's design, a dwell's
// M of N unfolding the truth, measurement errors, eclipsing; one interval's
// cells against FERS's samples put through a receiver's processing
// (tools/fers/crosscheck.py -> tests/data/signal/fers_crosscheck.json).
#include "core/Json.h"
#include "ms/radar/Detection.h"
#include "ms/radar/Signal.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

using namespace fsim::ms::radar::signal;

namespace {

constexpr double kLight = 299792458.0;

/// One interval, its range gates an exact number to the PRI and its filters an exact number to the PRF: a target can
/// sit on a gate and a filter (no straddle) in every interval.
Waveform grid(int intervals, int m, int pulses, int gates, double gateM = 150.0) {
    Waveform w;
    w.wavelengthM = 0.03;
    w.gateM = gateM;
    w.m = m;
    for (int i = 0; i < intervals; ++i) {
        Interval iv;
        iv.ambiguousRangeM = gates * gateM;
        iv.prfHz = kLight / (2.0 * iv.ambiguousRangeM);
        iv.pulses = pulses;
        iv.gates = gates;
        iv.pulseS = 0.1 / iv.prfHz;
        iv.blankedGates = static_cast<int>(std::ceil(kLight * iv.pulseS / 2.0 / gateM));
        iv.ambiguousRateMs = w.wavelengthM * iv.prfHz / 2.0;
        w.intervals.push_back(iv);
    }
    w.rateResolutionMs = w.wavelengthM / (2.0 * pulses / w.intervals[0].prfHz);
    return w;
}

} // namespace

TEST_CASE("signal level: the stream's normals", "[ms][signal]") {
    Stream s(42), t(42);
    double sum = 0, sum2 = 0, sum4 = 0;
    int beyond3 = 0, beyondR = 0;
    const int n = 2000000;
    for (int i = 0; i < n; ++i) {
        const double x = s.normal();
        CHECK((i > 10 || x == t.normal())); // (the same seed, the same draws)
        sum += x, sum2 += x * x, sum4 += x * x * x * x;
        beyond3 += std::fabs(x) > 3.0 ? 1 : 0;
        beyondR += std::fabs(x) > 3.442619855899 ? 1 : 0; // (the ziggurat's tail)
    }
    CHECK(std::fabs(sum / n) < 0.003);
    CHECK_THAT(sum2 / n, Catch::Matchers::WithinAbs(1.0, 0.004));
    CHECK_THAT(sum4 / n, Catch::Matchers::WithinAbs(3.0, 0.03));
    CHECK_THAT(beyond3 / static_cast<double>(n), Catch::Matchers::WithinRel(0.0026998, 0.05));
    CHECK_THAT(beyondR / static_cast<double>(n), Catch::Matchers::WithinRel(5.7615e-4, 0.08));
}

TEST_CASE("signal level: noise alone crosses each cell's threshold at its probability", "[ms][signal]") {
    // one interval, many cells: the peaks a cell over CA-CFAR's threshold (Settings.pfaInterval), the largest of its
    // neighbours - in noise, nearly every crossing
    const Waveform w = grid(1, 1, 64, 400);
    Settings set;
    set.pfaInterval = 1e-3;
    set.rangeScaleM = w.intervals[0].ambiguousRangeM;
    set.minRateMs = -w.intervals[0].ambiguousRateMs / 2.0, set.maxRateMs = w.intervals[0].ambiguousRateMs / 2.0;
    Stream s(7);
    Stats st;
    for (int i = 0; i < 200; ++i) dwell(w, {}, set, s, &st);
    const double rate = static_cast<double>(st.peaks) / static_cast<double>(st.cells);
    // a crossing beside a larger one is not a peak: the share that are, from the window's correlation (peakSurvival)
    const double q = peakSurvival(1e-3, set.cfarCells);
    INFO(st.peaks << " peaks in " << st.cells << " cells: " << rate << ", expected " << 1e-3 * q << " (q " << q << ")");
    CHECK_THAT(rate, Catch::Matchers::WithinRel(1e-3 * q, 0.05));
}

TEST_CASE("signal level: a Rayleigh target on a gate and a filter, against CA-CFAR's closed form", "[ms][signal]") {
    // one interval: P_d = (1 + alpha / (N (1 + S)))^-N for an exponentially distributed signal power over N exponential
    // reference cells, S the cell's mean SNR - the window's loss in it: a0^2 / sum(taps^2)
    const int pulses = 64, gates = 200;
    const Waveform w = grid(1, 1, pulses, gates);
    Settings set;
    set.pfaInterval = 1e-4;
    set.rangeScaleM = w.intervals[0].ambiguousRangeM;
    set.minRateMs = -w.intervals[0].ambiguousRateMs / 2.0, set.maxRateMs = w.intervals[0].ambiguousRateMs / 2.0;
    const int cells = set.cfarCells;
    const double alpha = cfarAlpha(cells, set.pfaInterval);
    const double windowLoss = Window::a[0] * Window::a[0] / Window::noiseGain();
    for (const double snrDb : {8.0, 12.0, 16.0, 20.0}) {
        Echo e;
        e.rangeM = 100 * w.gateM;                                // (on a gate)
        e.rangeRateMs = -8.0 * w.wavelengthM * w.intervals[0].prfHz / (2.0 * pulses); // (on the 8th filter)
        e.snr = std::pow(10.0, snrDb / 10.0);
        e.fluctuation = 1;
        e.truth = 9;
        Stream s(11);
        int found = 0;
        const int trials = 3000;
        for (int t = 0; t < trials; ++t)
            for (const Hit& h : dwell(w, {e}, set, s))
                if (h.truth == 9) {
                    ++found;
                    break;
                }
        const double cellSnr = e.snr * windowLoss;
        const double expect = std::pow(1.0 + alpha / (cells * (1.0 + cellSnr)), -cells);
        const double got = static_cast<double>(found) / trials;
        INFO(snrDb << " dB: Pd " << got << ", closed form " << expect);
        CHECK(std::fabs(got - expect) < 4.0 * std::sqrt(expect * (1.0 - expect) / trials) + 0.01);
    }
}

TEST_CASE("signal level: a dwell's M of N finds the target where it is, through its PRFs' ambiguities", "[ms][signal]") {
    // the medium-PRF schedule of an X-band fighter radar: 8 PRFs, 3 of 8; a steady target far beyond every PRF's
    // unambiguous range and rate, at a high SNR: found at its range and rate, measured within the errors the SNR gives
    const Waveform w = mediumPrf(9.86e9, 0.025, 8, 3, 150.0, 0.1);
    REQUIRE(w.intervals.size() == 8);
    for (const auto& iv : w.intervals) {
        CHECK(iv.ambiguousRateMs >= 150.0 - 1e-9);
        CHECK(iv.ambiguousRateMs <= 300.0 + 1e-9);
        CHECK(iv.ambiguousRangeM < 20e3);
        CHECK(iv.blankedGates >= 1);
    }
    Settings set;
    set.pfaInterval = intervalPfa(1e-9, w, set.coincidence, set.cfarCells);
    set.rangeScaleM = 150e3;
    set.minRateMs = -1250.0, set.maxRateMs = 1250.0;
    Echo e;
    e.rangeM = 83'417.0;
    e.rangeRateMs = -437.0;
    e.snr = std::pow(10.0, 35.0 / 10.0);
    e.fluctuation = 0;
    e.monopulseAz = 0.3, e.monopulseEl = -0.2;
    e.truth = 3;
    Stream s(5);
    int found = 0, ghosts = 0;
    double r2 = 0, v2 = 0, a2 = 0;
    const int trials = 300;
    for (int t = 0; t < trials; ++t)
        for (const Hit& h : dwell(w, {e}, set, s)) {
            if (h.truth != 3) continue;
            if (std::fabs(h.rangeM - e.rangeM) > 600.0 || std::fabs(h.rangeRateMs - e.rangeRateMs) > 20.0) {
                ++ghosts;
                continue;
            }
            ++found;
            r2 += (h.rangeM - e.rangeM) * (h.rangeM - e.rangeM);
            v2 += (h.rangeRateMs - e.rangeRateMs) * (h.rangeRateMs - e.rangeRateMs);
            a2 += (h.monopulseAz - e.monopulseAz) * (h.monopulseAz - e.monopulseAz);
        }
    INFO(found << " found, " << ghosts << " ghosts; rms range " << std::sqrt(r2 / found) << " m, rate " << std::sqrt(v2 / found)
               << " m/s, monopulse " << std::sqrt(a2 / found));
    CHECK(found >= trials * 95 / 100);
    CHECK(ghosts <= trials / 50);
    CHECK(std::sqrt(r2 / found) < w.gateM / 3.0);
    CHECK(std::sqrt(v2 / found) < w.rateResolutionMs);
    CHECK(std::sqrt(a2 / found) < 0.05);
}

TEST_CASE("signal level: a display's snapshot of the first interval - the noise as each gate hears it, the target's peak where it folds",
          "[ms][signal]") {
    const Waveform w = mediumPrf(9.86e9, 0.025, 8, 3, 150.0, 0.1);
    const Interval& first = w.intervals[0];
    Settings set;
    set.pfaInterval = 1e-6;
    set.rangeScaleM = 150e3;
    set.minRateMs = -1250.0, set.maxRateMs = 1250.0;
    Echo e;
    e.rangeM = 4.0 * first.ambiguousRangeM + 50.3 * w.gateM; // (gate 50 of the first interval: received whole)
    e.rangeRateMs = -412.0;
    e.snr = std::pow(10.0, 35.0 / 10.0);
    e.fluctuation = 0;
    e.truth = 1;
    Stream s(11);
    Snapshot snap;
    // where it folds: its gate, and its filter (k / m of the PRF is the Doppler -2 rate / lambda, folded)
    const int gate = static_cast<int>(std::fmod(e.rangeM, first.ambiguousRangeM) / w.gateM + 0.5);
    double nu = -2.0 * e.rangeRateMs / w.wavelengthM / first.prfHz;
    nu -= std::floor(nu);
    const int filter = static_cast<int>(std::lround(nu * first.pulses)) % first.pulses;
    const double chips = first.pulseS * kLight / 2.0 / w.gateM, period = first.ambiguousRangeM / w.gateM;
    double noise = 0.0, expected = 0.0;
    std::size_t noiseCells = 0;
    int atTarget = 0;
    for (int t = 0; t < 20; ++t) {
        dwell(w, {e}, set, s, nullptr, nullptr, &snap);
        REQUIRE(snap.kept);
        REQUIRE(snap.interval == 0);
        REQUIRE(snap.filters == first.pulses);
        REQUIRE(snap.rows == first.gates - first.blankedGates);
        REQUIRE(snap.power.size() == static_cast<std::size_t>(snap.rows) * static_cast<std::size_t>(snap.filters));
        for (int r = 0; r < snap.rows; ++r)
            for (int k = 0; k < snap.filters; ++k)
                if (std::abs(r + snap.firstGate - gate) > 3) {
                    noise += static_cast<double>(snap.power[static_cast<std::size_t>(r) * static_cast<std::size_t>(snap.filters) + static_cast<std::size_t>(k)]);
                    expected += noiseShare(r + snap.firstGate, chips, period), ++noiseCells;
                }
        for (const auto& [r, k] : snap.peaks) {
            const int dk = std::abs(k - filter);
            if (std::abs(r + snap.firstGate - gate) <= 1 && std::min(dk, snap.filters - dk) <= 1) ++atTarget;
        }
    }
    INFO("the noise's mean " << noise / static_cast<double>(noiseCells) << " (its gates' shares " << expected / static_cast<double>(noiseCells)
                             << "), the target's peak in " << atTarget << " of 20 snapshots");
    CHECK(std::fabs(noise / expected - 1.0) < 0.03);
    CHECK(atTarget >= 19);
}

TEST_CASE("signal level against FERS: one interval's cells - straddle, scalloping, folding, eclipse, noise", "[ms][signal]") {
    // FERS (run by tools/fers/crosscheck.py, not linked) formed the samples of a 10 GHz interval - 10 kHz PRF, 32
    // pulses, a 1 m^2 target - and a receiver's processing made its cells (blanked while transmitting, matched filtered
    // at the target's Doppler, a gate a chip, Blackman-Harris and the DFT across the pulses), each over kTB through it.
    // The chain, which never forms samples, is given the same target, its SNR from the same radar equation; its cells
    // (their mean over many dwells, less the noise's 1) are FERS's. The plain 1 us pulse is what the chain assumes
    // exactly; the Barker-13 code differs by its range sidelobes (1/13 in amplitude a lag, more for a code cut short by
    // the next transmission), which the chain leaves out: its cells are compared in the gates its echo straddles.
    // FERS's pulse is padded with zero samples (it renders a pulse in whole samples from its fractional start and drops
    // the last of some: a 10 % step in the echo pulse to pulse, the Doppler spectrum spread, until padded).
    namespace fs = std::filesystem;
    const fs::path source = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();
    std::ifstream in(source / "tests" / "data" / "signal" / "fers_crosscheck.json");
    REQUIRE(in.good());
    std::stringstream text;
    text << in.rdbuf();
    const fsim::core::Json j = fsim::core::Json::parse(text.str());
    const double fc = j.child("carrier_hz").asNumber(), prf = j.child("prf_hz").asNumber(), chip = j.child("chip_s").asNumber();
    const int pulses = static_cast<int>(j.child("pulses").asNumber()), gates = static_cast<int>(j.child("gates").asNumber());
    const double energy = j.child("pulse_energy_j").asNumber(), rcs = j.child("rcs_m2").asNumber();
    const double kT = j.child("boltzmann").asNumber() * j.child("noise_k").asNumber();
    // the noise: FERS's cells' mean is kTB's through the processing, and its neighbouring filters correlated as the
    // window's taps make them (the chain's noise is drawn through the same taps)
    double same = 0.0, next = 0.0;
    for (int t = -(Window::kTaps - 1); t < Window::kTaps; ++t) {
        const double h = t == 0 ? Window::a[0] : (std::abs(t) % 2 ? -0.5 : 0.5) * Window::a[std::abs(t)];
        const int u = t + 1;
        const double hn = std::abs(u) >= Window::kTaps ? 0.0 : u == 0 ? Window::a[0] : (std::abs(u) % 2 ? -0.5 : 0.5) * Window::a[std::abs(u)];
        same += h * h, next += h * hn;
    }
    for (const auto& code : j.child("codes").asArray()) {
        const std::string name = code.child("code").asString();
        const auto& noise = code.child("noise");
        INFO(name << ": FERS's noise " << noise.child("cell_mean").asNumber() << " of kTB's a cell, neighbouring filters correlated "
                  << noise.child("adjacent_filter_correlation").asNumber() << " (the window's " << next / same << ")");
        CHECK(std::fabs(noise.child("cell_mean").asNumber() - 1.0) < 0.03);
        CHECK(std::fabs(noise.child("adjacent_filter_correlation").asNumber() - next / same) < 0.02);
        const int chips = static_cast<int>(code.child("chips").asNumber());
        // the last gate's matched window reaches into the next transmission: it hears that much less of the noise
        const double lastShare = noiseShare(gates - 1, chips, gates);
        INFO("the last gate's noise: FERS's " << noise.child("last_gate_mean").asNumber() << ", the chain's " << lastShare);
        CHECK(std::fabs(noise.child("last_gate_mean").asNumber() - lastShare) < 0.02 + 0.1 * lastShare);
        Waveform w;
        w.wavelengthM = kLight / fc;
        w.gateM = kLight * chip / 2.0;
        w.m = 1;
        Interval iv;
        iv.prfHz = prf;
        iv.pulses = pulses;
        iv.gates = gates;
        iv.blankedGates = chips;
        iv.pulseS = chips * chip;
        iv.ambiguousRangeM = kLight / (2.0 * prf);
        iv.ambiguousRateMs = w.wavelengthM * prf / 2.0;
        w.intervals.push_back(iv);
        w.rateResolutionMs = w.wavelengthM / (2.0 * pulses / prf);
        Settings set;
        set.pfaInterval = 1e-6;
        set.rangeScaleM = 150e3;
        set.minRateMs = -1250.0, set.maxRateMs = 1250.0;
        for (const auto& c : code.child("cases").asArray()) {
            Echo e;
            e.rangeM = c.child("range_m").asNumber();
            e.rangeRateMs = c.child("range_rate_ms").asNumber();
            const double lambda = w.wavelengthM;
            e.snr = pulses * energy * lambda * lambda * rcs / (std::pow(4.0 * 3.14159265358979323846, 3) * std::pow(e.rangeM, 4)) / kT;
            e.fluctuation = 0;
            e.truth = 1;
            Stream stream(29);
            Snapshot snap;
            std::vector<double> mean;
            const int dwells = 600;
            for (int d = 0; d < dwells; ++d) {
                dwell(w, {e}, set, stream, nullptr, nullptr, &snap);
                REQUIRE(snap.kept);
                if (mean.empty()) mean.assign(snap.power.size(), 0.0);
                for (std::size_t i = 0; i < mean.size(); ++i) mean[i] += static_cast<double>(snap.power[i]) / dwells;
            }
            const auto& rows = c.child("gates").asArray();
            const auto& cells = c.child("cells").asArray();
            const int firstFilter = static_cast<int>(c.child("first_filter").asNumber());
            double peak = 0.0;
            for (const auto& row : cells) for (const auto& v : row.asArray()) peak = std::max(peak, v.asNumber());
            const double x = std::fmod(e.rangeM, iv.ambiguousRangeM) / w.gateM;
            double worstDb = 0.0, worstAbs = 0.0;
            for (std::size_t r = 0; r < rows.size(); ++r) {
                const int g = static_cast<int>(rows[r].asNumber());
                const auto& row = cells[r].asArray();
                for (std::size_t dk = 0; dk < row.size(); ++dk) {
                    const int k = (firstFilter + static_cast<int>(dk)) % pulses;
                    const double fers = row[dk].asNumber();
                    const double ours = mean[static_cast<std::size_t>(g - chips) * static_cast<std::size_t>(pulses) + static_cast<std::size_t>(k)] -
                                        noiseShare(g, chips, gates);
                    // the plain pulse: every cell; the code: the gates its compressed pulse straddles (its sidelobes
                    // elsewhere the chain leaves out), within them: 2/13 in amplitude either way
                    if (chips > 1 && !(std::fabs(g - x) < 1.0 - 1e-6)) continue;
                    INFO(name << " " << c.child("name").asString() << ": gate " << g << " filter " << k << " FERS " << 10.0 * std::log10(std::max(fers, 1e-9))
                              << " dB, the chain " << 10.0 * std::log10(std::max(ours, 1e-9)) << " dB");
                    // within 0.2 dB where the cell is within 6 dB of the case's peak, 0.3 dB on the window's skirts to 30 dB below
                    // it (FERS's sit up to 0.22 dB above the transform's); below, within the ripple a
                    // thousandth of the peak's power gives (FERS's range migration over the interval, a percent or two
                    // pulse to pulse, which the chain - a dwell at one range - leaves out) and the mean's noise
                    if (fers > peak * 1e-3 && fers > 10.0) {
                        const double db = 10.0 * std::log10(std::max(ours, 1e-9) / fers);
                        worstDb = std::max(worstDb, std::fabs(db));
                        CHECK(std::fabs(db) < (chips > 1 ? 20.0 * std::log10(1.0 + 2.0 / 13.0) : fers > peak / 4.0 ? 0.2 : 0.3));
                    } else {
                        worstAbs = std::max(worstAbs, std::fabs(ours - fers) / std::max(1.0, peak * 1e-3));
                        CHECK(std::fabs(ours - fers) < 0.25 + peak * 1e-3 + 0.3 * fers);
                    }
                }
            }
            WARN(name << " " << c.child("name").asString() << ": worst " << worstDb << " dB (cells within 30 dB of the peak), " << worstAbs
                      << " (below: of the larger of the noise and a thousandth of the peak)");
        }
    }
}

TEST_CASE("signal level: an interval blind where the echo meets its transmission", "[ms][signal]") {
    // a target whose folded range lies in the first PRF's transmitted pulse: that interval gives it nothing
    const Waveform w = mediumPrf(9.86e9, 0.025, 8, 3, 150.0, 0.1);
    const Interval& first = w.intervals[0];
    Settings set;
    set.pfaInterval = 1e-6;
    set.rangeScaleM = 150e3;
    set.minRateMs = -1250.0, set.maxRateMs = 1250.0;
    Echo e;
    e.rangeM = 5.0 * first.ambiguousRangeM + 0.3 * kLight * first.pulseS / 2.0; // (eclipsed in the first interval)
    e.rangeRateMs = -300.0;
    e.snr = 1e4;
    e.fluctuation = 0;
    e.truth = 1;
    Stream s(3);
    int withFirst = 0, hits = 0;
    for (int t = 0; t < 50; ++t)
        for (const Hit& h : dwell(w, {e}, set, s))
            if (h.truth == 1) ++hits, withFirst += h.intervals == 8 ? 1 : 0;
    INFO(hits << " hits, " << withFirst << " with all eight intervals");
    CHECK(hits >= 45);
    CHECK(withFirst == 0);
}

TEST_CASE("signal level: noise alone, a dwell's false alarms as its threshold was designed", "[ms][signal]") {
    // the M of N's interval threshold set for a dwell's false-alarm probability a cell of the unambiguous grid: what
    // noise alone declares, against cells x that probability
    const Waveform w = mediumPrf(9.86e9, 0.025, 8, 3, 150.0, 0.1);
    Settings set;
    set.rangeScaleM = 150e3;
    set.minRateMs = -1250.0, set.maxRateMs = 1250.0;
    const double cells = (set.rangeScaleM / w.gateM) * ((set.maxRateMs - set.minRateMs) / w.rateResolutionMs);
    const double pfaDwell = 0.05 / cells; // (0.05 false alarms a dwell: enough to count)
    set.pfaInterval = intervalPfa(pfaDwell, w, set.coincidence, set.cfarCells);
    Stream s(13);
    std::size_t alarms = 0;
    const int dwells = 6000;
    Stats plain, shown;
    Snapshot snap;
    for (int t = 0; t < dwells; ++t) alarms += dwell(w, {}, set, s, t % 2 ? &shown : &plain, nullptr, t % 2 ? &snap : nullptr).size();
    // (its cost: a dwell's processing, and with a display's snapshot kept)
    WARN("a dwell in noise " << plain.processS / (dwells / 2) * 1e3 << " ms, with the snapshot " << shown.processS / (dwells / 2) * 1e3 << " ms");
    const double expect = 0.05 * dwells;
    INFO(alarms << " false alarms in " << dwells << " dwells, designed " << expect);
    CHECK(static_cast<double>(alarms) > 0.5 * expect);
    CHECK(static_cast<double>(alarms) < 1.5 * expect);
}

TEST_CASE("signal level against detection level: the same dwells, the signal chain's processing loss", "[ms][signal]") {
    // a target at random ranges and rates (straddling gates and filters, eclipsed where it falls) - Rayleigh, a dominant
    // scatterer, steady: the SNR at which the signal level detects half the time, against the detection level's ideal
    // closed form - each interval 1/N of the energy, M of N at the same interval threshold. The gap is the chain's
    // processing loss: the window's (3.0 dB), the gates' and filters' straddle, CFAR's, the eclipses'; processingLossDb
    // the same for each fluctuation (the radar gives each target back its own).
    const Waveform w = mediumPrf(9.86e9, 0.025, 8, 3, 150.0, 0.1);
    Settings set;
    set.pfaInterval = intervalPfa(5e-10, w, set.coincidence, set.cfarCells);
    set.rangeScaleM = 150e3;
    set.minRateMs = -1250.0, set.maxRateMs = 1250.0;
    const int n = static_cast<int>(w.intervals.size());
    for (const int fluctuation : {1, 2, 0}) {
    const auto model = fluctuation == 0 ? fsim::ms::radar::Fluctuation::Steady : fluctuation == 2 ? fsim::ms::radar::Fluctuation::DominantScatterer
                                                                                                  : fsim::ms::radar::Fluctuation::Rayleigh;
    auto ideal = [&](double snrDb) {
        const double p = fsim::ms::radar::probabilityOfDetection(std::pow(10.0, snrDb / 10.0) / n, set.pfaInterval, model);
        double total = 0.0, c = 1.0;
        for (int k = 0; k <= n; ++k) {
            if (k > 0) c *= static_cast<double>(n - k + 1) / k;
            if (k >= w.m) total += c * std::pow(p, k) * std::pow(1.0 - p, n - k);
        }
        return total;
    };
    Stream s(21), place(22);
    auto measured = [&](double snrDb) {
        int found = 0;
        const int trials = fluctuation == 1 ? 4000 : 2000; // (a Pd's noise about 0.1 dB of SNR here; steadier: steeper)
        for (int t = 0; t < trials; ++t) {
            Echo e;
            e.rangeM = 20e3 + 120e3 * place.uniform();
            e.rangeRateMs = -1200.0 + 2400.0 * place.uniform();
            e.snr = std::pow(10.0, snrDb / 10.0);
            e.fluctuation = fluctuation;
            e.truth = 1;
            for (const Hit& h : dwell(w, {e}, set, s))
                if (h.truth == 1 && std::fabs(h.rangeM - e.rangeM) < 300.0) {
                    ++found;
                    break;
                }
        }
        return static_cast<double>(found) / trials;
    };
    auto half = [](auto&& pd) { // the SNR at Pd 0.5, by bisection
        double lo = 0.0, hi = 40.0;
        for (int i = 0; i < 9; ++i) {
            const double mid = 0.5 * (lo + hi);
            (pd(mid) < 0.5 ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    };
    const double idealDb = half(ideal), signalDb = half(measured);
    const double analytic = processingLossDb(w, set.pfaInterval, set.cfarCells, fluctuation);
    WARN("fluctuation " << fluctuation << ", Pd 0.5: detection level's ideal " << idealDb << " dB, signal level " << signalDb
                        << " dB: processing loss " << signalDb - idealDb << " dB measured, " << analytic << " dB from processingLossDb");
    CHECK(signalDb - idealDb > 3.0); // (at least the window's)
    CHECK(signalDb - idealDb < 12.0);
    CHECK(std::fabs(analytic - (signalDb - idealDb)) < 0.3); // (the loss the radar adds back: what its chain takes)
    }
}
