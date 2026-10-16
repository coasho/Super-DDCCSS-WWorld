#include "simulation/mission_systems/radar/Signal.h"

#include "simulation/mission_systems/radar/Detection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <chrono>
#include <unordered_map>

namespace fsim::ms::radar::signal {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kLight = 299792458.0;
using C = std::complex<double>;

std::uint64_t rotl(std::uint64_t x, int k) noexcept { return (x << k) | (x >> (64 - k)); }

std::uint64_t splitmix(std::uint64_t& x) noexcept {
    std::uint64_t z = (x += 0x9e3779b97f4a7c15ull);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
}

// Marsaglia and Tsang's ziggurat (128 layers), its tables built once
struct Ziggurat {
    std::uint32_t kn[128];
    double wn[128], fn[128];
    Ziggurat() {
        const double m1 = 2147483648.0;
        double dn = 3.442619855899, tn = dn;
        const double vn = 9.91256303526217e-3;
        const double q = vn / std::exp(-0.5 * dn * dn);
        kn[0] = static_cast<std::uint32_t>((dn / q) * m1);
        kn[1] = 0;
        wn[0] = q / m1;
        wn[127] = dn / m1;
        fn[0] = 1.0;
        fn[127] = std::exp(-0.5 * dn * dn);
        for (int i = 126; i >= 1; --i) {
            dn = std::sqrt(-2.0 * std::log(vn / dn + std::exp(-0.5 * dn * dn)));
            kn[i + 1] = static_cast<std::uint32_t>((dn / tn) * m1);
            tn = dn;
            fn[i] = std::exp(-0.5 * dn * dn);
            wn[i] = dn / m1;
        }
    }
};
const Ziggurat& zig() {
    static const Ziggurat z;
    return z;
}

// D(x) = sum_{n<M} e^{j 2 pi x n}: a tone's unwindowed Doppler filter output, x its offset from the filter in cycles a pulse
C dirichlet(double x, int m) noexcept {
    const double s = std::sin(kPi * x);
    if (std::fabs(s) < 1e-12) return C(static_cast<double>(m), 0.0) * std::polar(1.0, kPi * x * (m - 1) - kPi * std::round(x) * (m - 1));
    return std::polar(std::sin(kPi * x * m) / s, kPi * x * (m - 1));
}

int binomial(int n, int k) noexcept {
    double c = 1.0;
    for (int i = 0; i < k; ++i) c *= static_cast<double>(n - i) / static_cast<double>(i + 1);
    return static_cast<int>(std::lround(c));
}

// the window's convolution of a spectrum (circular in the filters; m >= kTaps - 1): the spectrum padded with its own
// ends, so the taps index it straight
void windowed(const C* in, C* out, int m) noexcept {
    constexpr int pad = Window::kTaps - 1;
    thread_local std::vector<C> ext;
    ext.resize(static_cast<std::size_t>(m + 2 * pad));
    for (int k = 0; k < m; ++k) ext[static_cast<std::size_t>(k + pad)] = in[k];
    for (int k = 0; k < pad; ++k) {
        ext[static_cast<std::size_t>(k)] = in[m - pad + k];
        ext[static_cast<std::size_t>(m + pad + k)] = in[k];
    }
    const double a0 = Window::a[0], h1 = -0.5 * Window::a[1], h2 = 0.5 * Window::a[2], h3 = -0.5 * Window::a[3];
    const C* e = ext.data() + pad;
    for (int k = 0; k < m; ++k) out[k] = a0 * e[k] + h1 * (e[k - 1] + e[k + 1]) + h2 * (e[k - 2] + e[k + 2]) + h3 * (e[k - 3] + e[k + 3]);
}

} // namespace

void Stream::reseed(std::uint64_t seed) noexcept {
    for (auto& s : s_) s = splitmix(seed);
}

std::uint64_t Stream::next() noexcept {
    const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
    const std::uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 45);
    return result;
}

double Stream::normal() noexcept {
    const Ziggurat& z = zig();
    constexpr double r = 3.442619855899;
    for (;;) {
        const auto hz = static_cast<std::int32_t>(static_cast<std::uint32_t>(next() >> 32));
        const std::uint32_t iz = static_cast<std::uint32_t>(hz) & 127u;
        const std::uint32_t mag = static_cast<std::uint32_t>(hz < 0 ? -static_cast<std::int64_t>(hz) : hz);
        if (mag < z.kn[iz]) return hz * z.wn[iz];
        // the slow way: the base strip's tail, or a wedge
        const double x = hz * z.wn[iz];
        if (iz == 0) {
            double xt, y;
            do {
                xt = -std::log(1.0 - uniform()) / r;
                y = -std::log(1.0 - uniform());
            } while (y + y < xt * xt);
            return hz > 0 ? r + xt : -r - xt;
        }
        if (z.fn[iz] + uniform() * (z.fn[iz - 1] - z.fn[iz]) < std::exp(-0.5 * x * x)) return x;
    }
}

double Window::noiseGain() noexcept {
    double g = a[0] * a[0];
    for (int t = 1; t < kTaps; ++t) g += 0.5 * a[t] * a[t];
    return g;
}

double cfarAlpha(int n, double pfa) noexcept { return n * (std::pow(pfa, -1.0 / n) - 1.0); }

Waveform mediumPrf(double frequencyHz, double dwellS, int prfs, int m, double gateM, double duty) {
    Waveform w;
    w.wavelengthM = kLight / frequencyHz;
    w.gateM = gateM;
    w.m = std::clamp(m, 1, std::max(1, prfs));
    const int n = std::max(1, prfs);
    const double interval = dwellS / n;
    w.rateResolutionMs = w.wavelengthM / (2.0 * interval);
    for (int i = 0; i < n; ++i) {
        // blind speeds 150 .. 300 m/s, geometric (the same design in speed for every radar)
        const double blind = 150.0 * std::pow(2.0, n > 1 ? static_cast<double>(i) / (n - 1) : 0.0);
        Interval iv;
        iv.prfHz = 2.0 * blind / w.wavelengthM;
        iv.pulses = std::max(4, static_cast<int>(std::lround(interval * iv.prfHz)));
        iv.ambiguousRangeM = kLight / (2.0 * iv.prfHz);
        iv.ambiguousRateMs = blind;
        iv.gates = std::max(1, static_cast<int>(iv.ambiguousRangeM / gateM));
        iv.pulseS = duty / iv.prfHz;
        iv.blankedGates = std::min(iv.gates, static_cast<int>(std::ceil(kLight * iv.pulseS / 2.0 / gateM)));
        w.intervals.push_back(iv);
    }
    return w;
}

std::vector<Hit> dwell(const Waveform& waveform, const std::vector<Echo>& echoes, const Settings& settings, Stream& stream, Stats* stats,
                       const ClutterMap* clutter, Snapshot* snapshot) {
    const auto started = std::chrono::steady_clock::now();
    if (snapshot) snapshot->kept = false, snapshot->peaks.clear();
    const int n = static_cast<int>(waveform.intervals.size());
    const double noiseGain = Window::noiseGain();
    const double lambda = waveform.wavelengthM;
    // an interval's detection, folded
    struct Peak {
        int interval;
        double rangeM, rateMs; // folded: range in [0, Ra), rate in [-Va/2, Va/2)
        double snr;            // the interval's coherent SNR, estimated (linear)
        double monoAz, monoEl;
        std::uint32_t truth;
    };
    std::vector<Peak> peaks;
    thread_local std::vector<C> grid, noise, spectrum;
    thread_local std::vector<double> power;
    thread_local std::vector<int> echoGate;
    thread_local std::vector<double> echoAmp;
    thread_local std::vector<C> echoSpectra;
    std::uint64_t cells = 0;

    for (int i = 0; i < n; ++i) {
        const Interval& iv = waveform.intervals[static_cast<std::size_t>(i)];
        const int m = iv.pulses, g0 = iv.blankedGates, gn = iv.gates, rows = gn - g0;
        if (rows <= 0) continue;
        cells += static_cast<std::uint64_t>(rows) * static_cast<std::uint64_t>(m);
        grid.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(m), C());
        noise.resize(static_cast<std::size_t>(m));
        // receiver noise: each filter's white sum of m unit samples (variance m), the ground's clutter added to it
        // (Gaussian too), then the window
        const double* cl = clutter && static_cast<std::size_t>(i) < clutter->size() &&
                                   (*clutter)[static_cast<std::size_t>(i)].size() == static_cast<std::size_t>(rows) * static_cast<std::size_t>(m)
                               ? (*clutter)[static_cast<std::size_t>(i)].data()
                               : nullptr;
        // (the gates whose matched window reaches into the next transmission hear less of the noise)
        const double chips = iv.pulseS * kLight / 2.0 / waveform.gateM, periodGates = iv.ambiguousRangeM / waveform.gateM;
        // (all but the last few gates hear it whole: their shares 1, the rest worked out)
        thread_local std::vector<double> share;
        share.assign(static_cast<std::size_t>(rows), 1.0);
        for (int r = std::max(0, static_cast<int>(std::floor(periodGates - chips)) - g0); r < rows; ++r)
            share[static_cast<std::size_t>(r)] = noiseShare(g0 + r, chips, periodGates);
        const double sdWhole = std::sqrt(0.5 * m);
        for (int r = 0; r < rows; ++r) {
            const double heard = m * share[static_cast<std::size_t>(r)];
            if (cl) {
                const double* c = cl + static_cast<std::size_t>(r) * static_cast<std::size_t>(m);
                for (int k = 0; k < m; ++k) {
                    const double s = std::sqrt(0.5 * (heard + c[k]));
                    noise[static_cast<std::size_t>(k)] = C(s * stream.normal(), s * stream.normal());
                }
            } else {
                const double sd = share[static_cast<std::size_t>(r)] < 1.0 ? std::sqrt(0.5 * heard) : sdWhole;
                for (int k = 0; k < m; ++k) noise[static_cast<std::size_t>(k)] = C(sd * stream.normal(), sd * stream.normal());
            }
            windowed(noise.data(), grid.data() + static_cast<std::size_t>(r) * static_cast<std::size_t>(m), m);
        }
        // the echoes: each its windowed spectrum, in the gates its folded range straddles
        const std::size_t ne = echoes.size();
        echoSpectra.assign(ne * static_cast<std::size_t>(m), C());
        echoGate.assign(ne * 2, -1);
        echoAmp.assign(ne * 2, 0.0);
        spectrum.resize(static_cast<std::size_t>(m));
        for (std::size_t e = 0; e < ne; ++e) {
            const Echo& echo = echoes[e];
            if (!(echo.rangeM > 0.0) || !(echo.snr > 0.0)) continue;
            // this interval's amplitude: its fluctuation's draw, its phase uniform
            double p = echo.snr / (n * m);
            if (echo.fluctuation == 1) p *= -std::log(1.0 - stream.uniform());
            else if (echo.fluctuation == 2) p *= 0.5 * (-std::log(1.0 - stream.uniform()) - std::log(1.0 - stream.uniform()));
            const double phase = 2.0 * kPi * stream.uniform();
            // folded range: the gates either side of it, each its response (straddle and eclipse)
            const double folded = std::fmod(echo.rangeM, iv.ambiguousRangeM);
            const double x = folded / waveform.gateM;
            const int g = static_cast<int>(std::floor(x));
            const int gate[2] = {g, g + 1};
            const double w[2] = {gateResponse(x, g, chips, periodGates), gateResponse(x, g + 1, chips, periodGates)};
            if (!(w[0] > 0.0) && !(w[1] > 0.0)) continue;
            // folded Doppler (cycles a pulse) and the window's transform there
            const double nu = -2.0 * echo.rangeRateMs / lambda / iv.prfHz;
            for (int k = 0; k < m; ++k) spectrum[static_cast<std::size_t>(k)] = dirichlet(nu - static_cast<double>(k) / m, m);
            C* out = echoSpectra.data() + e * static_cast<std::size_t>(m);
            windowed(spectrum.data(), out, m);
            const C a = std::polar(std::sqrt(p), phase);
            for (int k = 0; k < m; ++k) out[k] *= a;
            for (int s = 0; s < 2; ++s) {
                if (gate[s] < g0 || gate[s] >= gn || w[s] <= 0.0) continue;
                echoGate[e * 2 + static_cast<std::size_t>(s)] = gate[s];
                echoAmp[e * 2 + static_cast<std::size_t>(s)] = w[s];
                C* row = grid.data() + static_cast<std::size_t>(gate[s] - g0) * static_cast<std::size_t>(m);
                for (int k = 0; k < m; ++k) row[k] += w[s] * out[k];
            }
        }
        // the processor: power, CA-CFAR along range in each filter, a peak the largest of its neighbours
        power.resize(grid.size());
        for (std::size_t c = 0; c < grid.size(); ++c) power[c] = std::norm(grid[c]);
        const bool shown = snapshot && snapshot->interval == i;
        if (shown) { // (a display's copy: the cells over a filter's noise)
            Snapshot& s = *snapshot;
            s.kept = true, s.rows = rows, s.filters = m, s.firstGate = g0;
            s.prfHz = iv.prfHz, s.gateM = waveform.gateM, s.ambiguousRangeM = iv.ambiguousRangeM, s.ambiguousRateMs = iv.ambiguousRateMs;
            s.mainBeamRateMs = settings.notchMs > 0.0 ? settings.mainBeamRateMs : std::numeric_limits<double>::quiet_NaN();
            s.notchMs = settings.notchMs;
            s.altitudeM = settings.notchMs > 0.0 ? settings.altitudeM : std::numeric_limits<double>::quiet_NaN();
            s.power.resize(power.size());
            const double unit = 1.0 / (m * noiseGain);
            for (std::size_t c = 0; c < power.size(); ++c) s.power[c] = static_cast<float>(power[c] * unit);
        }
        const int half = settings.cfarCells / 2, guard = settings.cfarGuard;
        double alpha[64] = {};
        for (int c = 1; c <= std::min(63, settings.cfarCells); ++c) alpha[c] = cfarAlpha(c, settings.pfaInterval);
        auto at = [&](int r, int k) { return power[static_cast<std::size_t>(r) * static_cast<std::size_t>(m) + static_cast<std::size_t>((k + m) % m)]; };
        thread_local std::vector<double> prefix;
        prefix.resize(static_cast<std::size_t>(rows) + 1);
        // the processor's notches: filters (as fractions of the PRF's band) it does not test
        const double filterRate = lambda * iv.prfHz / (2.0 * m); // (a filter's width in range rate)
        auto nearRate = [&](int k, double rate) {                // filter k within the notch of a range rate
            double x = -2.0 * rate / lambda / iv.prfHz * m - k;  // (in filters)
            x -= m * std::floor(x / m + 0.5);
            return std::fabs(x) * filterRate <= settings.notchMs;
        };
        const bool mainBeam = std::isfinite(settings.mainBeamRateMs) && settings.notchMs > 0.0;
        const bool altitude = std::isfinite(settings.altitudeM) && settings.notchMs > 0.0;
        const double altitudeGate = altitude ? std::fmod(settings.altitudeM, iv.ambiguousRangeM) / waveform.gateM - g0 : -1e9;
        for (int k = 0; k < m; ++k) {
            if (mainBeam && nearRate(k, settings.mainBeamRateMs)) continue;
            const bool zeroRate = altitude && nearRate(k, 0.0);
            // the filter's powers summed along range: a reference window's sum two lookups
            prefix[0] = 0.0;
            for (int r = 0; r < rows; ++r)
                prefix[static_cast<std::size_t>(r) + 1] = prefix[static_cast<std::size_t>(r)] + power[static_cast<std::size_t>(r) * static_cast<std::size_t>(m) + static_cast<std::size_t>(k)];
            for (int r = 0; r < rows; ++r) {
                if (zeroRate && std::fabs(r - altitudeGate) <= 2.0) continue;
                const int lo0 = std::max(0, r - guard - half), lo1 = std::max(0, r - guard);
                const int hi0 = std::min(rows, r + guard + 1), hi1 = std::min(rows, r + guard + half + 1);
                const double sum = (prefix[static_cast<std::size_t>(lo1)] - prefix[static_cast<std::size_t>(lo0)]) +
                                   (hi1 > hi0 ? prefix[static_cast<std::size_t>(hi1)] - prefix[static_cast<std::size_t>(hi0)] : 0.0);
                const int count = (lo1 - lo0) + std::max(0, hi1 - hi0);
                if (count < 4) continue;
                const double mean = sum / count, here = at(r, k);
                if (!(here > alpha[count] * mean)) continue;
                // where the ground's clutter is (above a tenth of the noise), the clutter map's threshold too: the
                // cell's expected power - the noise's and the clutter's through the window's taps - times -ln(p) (an
                // exponential of known mean crosses it with probability p, however the clutter varies around it)
                if (cl) {
                    const double* c = cl + static_cast<std::size_t>(r) * static_cast<std::size_t>(m);
                    double clutterPart = 0.0;
                    for (int t = -(Window::kTaps - 1); t < Window::kTaps; ++t) {
                        const double tap = t == 0 ? Window::a[0] : 0.5 * Window::a[std::abs(t)];
                        clutterPart += tap * tap * c[((k - t) % m + m) % m];
                    }
                    const double noisePart = m * share[static_cast<std::size_t>(r)] * noiseGain;
                    if (clutterPart > 0.1 * noisePart && !(here > -std::log(settings.pfaInterval) * (noisePart + clutterPart))) continue;
                }
                bool largest = true;
                for (int dr = -1; dr <= 1 && largest; ++dr)
                    for (int dk = -1; dk <= 1; ++dk) {
                        if ((dr == 0 && dk == 0) || r + dr < 0 || r + dr >= rows) continue;
                        if (at(r + dr, k + dk) > here) {
                            largest = false;
                            break;
                        }
                    }
                if (!largest) continue;
                // measured: range and Doppler interpolated, monopulse at the cell, the SNR over the reference
                const double a0 = std::sqrt(here);
                const double up = r + 1 < rows ? std::sqrt(at(r + 1, k)) : 0.0, down = r > 0 ? std::sqrt(at(r - 1, k)) : 0.0;
                const double dg = up >= down ? up / (a0 + up) : -down / (a0 + down);
                const double km = std::sqrt(at(r, k - 1)), kp = std::sqrt(at(r, k + 1));
                const double den = km - 2.0 * a0 + kp;
                const double dk = std::fabs(den) > 1e-30 ? std::clamp(0.5 * (km - kp) / den, -0.5, 0.5) : 0.0;
                double nu = (k + dk) / m;
                nu -= std::floor(nu + 0.5); // [-1/2, 1/2)
                Peak pk;
                pk.interval = i;
                pk.rangeM = (g0 + r + dg) * waveform.gateM;
                pk.rateMs = -nu * iv.prfHz * lambda / 2.0;
                pk.snr = std::max(0.0, here / mean - 1.0) * noiseGain / (Window::a[0] * Window::a[0]);
                // the difference channels: the echoes' sum-channel parts times their ratio, and their own noise
                const std::size_t cell = static_cast<std::size_t>(r) * static_cast<std::size_t>(m) + static_cast<std::size_t>(k);
                const C sum0 = grid[cell];
                C dAz(0.0, 0.0), dEl(0.0, 0.0);
                double strongest = mean; // (an echo must outweigh the noise to own the cell)
                pk.truth = 0;
                for (std::size_t e = 0; e < ne; ++e)
                    for (int s = 0; s < 2; ++s) {
                        if (echoGate[e * 2 + static_cast<std::size_t>(s)] != g0 + r) continue;
                        const C part = echoAmp[e * 2 + static_cast<std::size_t>(s)] * echoSpectra[e * static_cast<std::size_t>(m) + static_cast<std::size_t>(k)];
                        dAz += part * echoes[e].monopulseAz;
                        dEl += part * echoes[e].monopulseEl;
                        if (std::norm(part) > strongest) strongest = std::norm(part), pk.truth = echoes[e].truth;
                    }
                const double nd = std::sqrt(0.5 * m * share[static_cast<std::size_t>(r)] * noiseGain);
                dAz += C(nd * stream.normal(), nd * stream.normal());
                dEl += C(nd * stream.normal(), nd * stream.normal());
                pk.monoAz = std::norm(sum0) > 0.0 ? (dAz / sum0).real() : 0.0;
                pk.monoEl = std::norm(sum0) > 0.0 ? (dEl / sum0).real() : 0.0;
                peaks.push_back(pk);
                if (shown) snapshot->peaks.emplace_back(r, k);
            }
        }
    }

    // the M of N: each detection unfolded to every range and range rate it may be, counted where intervals agree
    struct Cell {
        std::uint32_t mask = 0;
        int best[32];
    };
    std::unordered_map<std::int64_t, Cell> cellsAt;
    const double dv = waveform.rateResolutionMs, dr = waveform.gateM;
    auto key = [](std::int64_t rc, std::int64_t vc) { return (rc << 32) ^ (vc & 0xffffffffll); };
    for (std::size_t p = 0; p < peaks.size(); ++p) {
        const Peak& pk = peaks[p];
        const Interval& iv = waveform.intervals[static_cast<std::size_t>(pk.interval)];
        for (double r = pk.rangeM; r < settings.rangeScaleM; r += iv.ambiguousRangeM) {
            const double lo = std::ceil((settings.minRateMs - pk.rateMs) / iv.ambiguousRateMs);
            const double hi = std::floor((settings.maxRateMs - pk.rateMs) / iv.ambiguousRateMs);
            for (double l = lo; l <= hi; l += 1.0) {
                const double v = pk.rateMs + l * iv.ambiguousRateMs;
                Cell& c = cellsAt[key(std::llround(r / dr), std::llround(v / dv))];
                const std::uint32_t bit = 1u << pk.interval;
                if (!(c.mask & bit) || peaks[static_cast<std::size_t>(c.best[pk.interval])].snr < pk.snr) c.best[pk.interval] = static_cast<int>(p);
                c.mask |= bit;
            }
        }
    }
    struct Candidate {
        std::int64_t rc, vc;
        int count;
        double snr;
        int best[32];
        std::uint32_t mask;
    };
    std::vector<Candidate> candidates;
    const int tol = settings.coincidence;
    // every cell within the tolerance of a detection may be where m of them agree (two, 2 tol apart, agree only at the
    // cell between them)
    // - each cell's intervals spread over the centres around it, and only a centre m of them reach looked at again
    std::unordered_map<std::int64_t, std::uint32_t> reach;
    reach.reserve(cellsAt.size() * static_cast<std::size_t>((2 * tol + 1) * (2 * tol + 1)));
    for (const auto& kc : cellsAt) {
        const std::int64_t rc = kc.first >> 32;
        const std::int64_t vc = static_cast<std::int32_t>(kc.first & 0xffffffff);
        for (std::int64_t a = -tol; a <= tol; ++a)
            for (std::int64_t b = -tol; b <= tol; ++b) reach[key(rc + a, vc + b)] |= kc.second.mask;
    }
    std::vector<std::int64_t> centres;
    for (const auto& [k, mask] : reach)
        if (__builtin_popcount(mask) >= waveform.m) centres.push_back(k);
    std::sort(centres.begin(), centres.end()); // (the same order every run)
    for (const std::int64_t k : centres) {
        const std::int64_t rc = k >> 32;
        const std::int64_t vc = static_cast<std::int32_t>(k & 0xffffffff);
        Candidate cand{rc, vc, 0, 0.0, {}, 0u};
        for (int i = 0; i < n; ++i) cand.best[i] = -1;
        for (std::int64_t a = -tol; a <= tol; ++a)
            for (std::int64_t b = -tol; b <= tol; ++b) {
                const auto it = cellsAt.find(key(rc + a, vc + b));
                if (it == cellsAt.end()) continue;
                for (int i = 0; i < n; ++i)
                    if (it->second.mask & (1u << i)) {
                        const int p = it->second.best[i];
                        if (cand.best[i] < 0 || peaks[static_cast<std::size_t>(cand.best[i])].snr < peaks[static_cast<std::size_t>(p)].snr) cand.best[i] = p;
                        cand.mask |= 1u << i;
                    }
            }
        cand.count = __builtin_popcount(cand.mask);
        if (cand.count < waveform.m) continue;
        for (int i = 0; i < n; ++i)
            if (cand.best[i] >= 0) cand.snr += peaks[static_cast<std::size_t>(cand.best[i])].snr;
        candidates.push_back(cand);
    }
    // one detection a neighbourhood: the most intervals, then the most energy, first
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.count != b.count) return a.count > b.count;
        if (a.snr != b.snr) return a.snr > b.snr;
        return a.rc != b.rc ? a.rc < b.rc : a.vc < b.vc; // (the same order on every run)
    });
    // each interval's detection resolves one target at most: a candidate counts the detections no better candidate
    // used (the rest of a resolved target's aliases agree elsewhere only by its own detections - ghosts)
    std::vector<Hit> hits;
    std::vector<char> used(peaks.size(), 0);
    for (Candidate c : candidates) {
        int count = 0;
        for (int i = 0; i < n; ++i) {
            if (c.best[i] >= 0 && used[static_cast<std::size_t>(c.best[i])]) c.best[i] = -1;
            count += c.best[i] >= 0 ? 1 : 0;
        }
        if (count < waveform.m) continue;
        for (int i = 0; i < n; ++i)
            if (c.best[i] >= 0) used[static_cast<std::size_t>(c.best[i])] = 1;
        Hit h;
        double weight = 0.0;
        std::unordered_map<std::uint32_t, int> votes;
        const double rc = static_cast<double>(c.rc) * dr, vc = static_cast<double>(c.vc) * dv;
        for (int i = 0; i < n; ++i) {
            if (c.best[i] < 0) continue;
            const Peak& pk = peaks[static_cast<std::size_t>(c.best[i])];
            const Interval& iv = waveform.intervals[static_cast<std::size_t>(i)];
            // the peak's own unfolding nearest the candidate
            const double r = pk.rangeM + std::round((rc - pk.rangeM) / iv.ambiguousRangeM) * iv.ambiguousRangeM;
            const double v = pk.rateMs + std::round((vc - pk.rateMs) / iv.ambiguousRateMs) * iv.ambiguousRateMs;
            const double w = std::max(pk.snr, 1e-6);
            h.rangeM += w * r, h.rangeRateMs += w * v, h.monopulseAz += w * pk.monoAz, h.monopulseEl += w * pk.monoEl;
            weight += w;
            h.snr += pk.snr;
            ++h.intervals;
            ++votes[pk.truth];
        }
        h.rangeM /= weight, h.rangeRateMs /= weight, h.monopulseAz /= weight, h.monopulseEl /= weight;
        int most = 0;
        for (const auto& [t, v] : votes)
            if (v > most || (v == most && t < h.truth)) most = v, h.truth = t;
        hits.push_back(h);
    }
    if (stats) {
        stats->cells += cells, stats->peaks += peaks.size(), stats->hits += hits.size();
        stats->processS += std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    }
    return hits;
}

int coincidences(int m, int tol) noexcept {
    // the (m - 1)-tuples of offsets that, with 0, span no more than 2 tol: one axis's ways for m detections to agree
    const int span = 2 * tol, side = 2 * span + 1;
    int count = 0, total = 1;
    for (int i = 1; i < m; ++i) total *= side;
    for (int code = 0; code < total; ++code) {
        int lo = 0, hi = 0, c = code;
        for (int i = 1; i < m; ++i, c /= side) {
            const int o = c % side - span;
            lo = std::min(lo, o), hi = std::max(hi, o);
        }
        if (hi - lo <= span) ++count;
    }
    return count;
}

double peakSurvival(double pfa, int cfarCells) noexcept {
    // the window's correlation between filters 1 and 2 apart (its taps are real and symmetric)
    double c[2 * Window::kTaps - 1];
    for (int t = 0; t < Window::kTaps; ++t) c[Window::kTaps - 1 + t] = c[Window::kTaps - 1 - t] = (t == 0 ? 1.0 : (t % 2 ? -0.5 : 0.5)) * Window::a[t];
    const int nc = 2 * Window::kTaps - 1;
    auto lag = [&](int d) {
        double s = 0.0;
        for (int i = 0; i + d < nc; ++i) s += c[i] * c[i + d];
        return s;
    };
    const double r1 = lag(1) / lag(0), r2 = lag(2) / lag(0);
    // the neighbours (k - 1, k + 1) given the cell X: rho1 X plus a pair of correlation r2 - r1^2, variance 1 - r1^2
    const double var = 1.0 - r1 * r1, cov = r2 - r1 * r1;
    const double l11 = std::sqrt(var), l21 = cov / l11, l22 = std::sqrt(std::max(0.0, var - l21 * l21));
    const double alpha = cfarAlpha(cfarCells, pfa);
    Stream s(0x9eab);
    double num = 0.0, den = 0.0;
    for (int i = 0; i < 40000; ++i) {
        // the reference cells' mean (its CFAR threshold) and the probability noise crosses it
        double mean = 0.0;
        for (int j = 0; j < cfarCells; ++j) mean -= std::log(1.0 - s.uniform());
        mean /= cfarCells;
        const double t = alpha * mean, w = std::exp(-t);
        // given it crossed: its power the threshold and an exponential beyond, its phase uniform
        const double x2 = t - std::log(1.0 - s.uniform());
        const C x = std::polar(std::sqrt(x2), 2.0 * kPi * s.uniform());
        const double h = std::sqrt(0.5);
        const C z1(h * s.normal(), h * s.normal()), z2(h * s.normal(), h * s.normal());
        const C y1 = r1 * x + l11 * z1, y2 = r1 * x + l21 * z1 + l22 * z2;
        // the six neighbours in the gates either side: independent of it
        const double others = std::pow(1.0 - std::exp(-x2), 6);
        den += w;
        if (std::norm(y1) < x2 && std::norm(y2) < x2) num += w * others;
    }
    return den > 0.0 ? num / den : 1.0;
}

double gateResponse(double x, int gate, double chips, double period) noexcept {
    const double delta = gate - x, len = 1.0 - std::fabs(delta);
    if (!(len > 0.0) || !(chips > 0.0)) return 0.0;
    // the chips' overlaps: a comb from `lo`, a tooth of `len` a gate - the whole chips', then the last's (a pulse not a
    // whole number of gates long: its last chip shorter, its overlap the shorter by the straddle); its measure below t
    const double lo = std::max(static_cast<double>(gate), x);
    const double whole = std::floor(chips), last = std::max(0.0, chips - whole - std::fabs(delta));
    auto below = [&](double t) {
        const double u = t - lo;
        if (!(u > 0.0)) return 0.0;
        const double k = std::floor(u);
        if (k < whole) return k * len + std::min(u - k, len);
        return whole * len + std::min(u - whole, last);
    };
    const double all = whole * len + last; // (heard whole: the straddle's 1 - |delta| exactly)
    return all > 0.0 ? len * std::max(0.0, below(period) - below(chips)) / all : 0.0;
}

double noiseShare(int gate, double chips, double period) noexcept {
    if (!(chips > 0.0)) return 1.0;
    return std::clamp((std::min(gate + chips, period) - std::max(static_cast<double>(gate), chips)) / chips, 0.0, 1.0);
}

double processingLossDb(const Waveform& waveform, double pfaInterval, int cfarCells, int fluctuation, double idealPfaInterval) noexcept {
    const int n = static_cast<int>(waveform.intervals.size()), m = waveform.m;
    if (n <= 0) return 0.0;
    const double alpha = cfarAlpha(cfarCells, pfaInterval);
    const double window = Window::a[0] * Window::a[0] / Window::noiseGain();
    // each interval's losses over where a target falls, as weights over a grid of their values
    constexpr int kGrid = 24, kPositions = 120; // (the positions in the PRI: a few in each eclipse at a 10 % duty)
    std::vector<std::vector<double>> losses(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const Interval& iv = waveform.intervals[static_cast<std::size_t>(i)];
        const int pulses = iv.pulses;
        auto& l = losses[static_cast<std::size_t>(i)];
        // the window's scalloping at a filter offset x (in filters), its transform against its peak
        std::vector<C> spec(static_cast<std::size_t>(pulses)), out(static_cast<std::size_t>(pulses));
        auto scallop = [&](double x) {
            for (int k = 0; k < pulses; ++k) spec[static_cast<std::size_t>(k)] = dirichlet((x - k) / pulses, pulses);
            windowed(spec.data(), out.data(), pulses);
            return std::norm(out[0]) / std::norm(Window::a[0] * pulses);
        };
        const double chips = iv.pulseS * kLight / 2.0 / waveform.gateM, period = iv.ambiguousRangeM / waveform.gateM;
        for (int a = 0; a < kGrid; ++a) {
            const double delta = (a + 0.5) / kGrid; // the gate straddle
            for (int b = 0; b < kGrid; ++b) {
                const double x = (b + 0.5) / kGrid - 0.5; // the filter offset
                const double sc = scallop(x);
                for (int c = 0; c < kPositions; ++c) {
                    // where in the PRI the echo falls: blanked, partly eclipsed at either end, or whole - the larger
                    // of its two gates' responses (a gate the receiver samples)
                    const double start = std::floor((c + 0.5) / kPositions * period) + delta;
                    const int g = static_cast<int>(std::floor(start));
                    double gate = 0.0;
                    for (int s = 0; s < 2; ++s)
                        if (g + s >= iv.blankedGates && g + s < iv.gates) {
                            const double heard = noiseShare(g + s, chips, period); // (the SNR there: its noise less too)
                            if (heard > 0.0) gate = std::max(gate, std::pow(gateResponse(start, g + s, chips, period), 2) / heard);
                        }
                    l.push_back(window * gate * sc);
                }
            }
        }
    }
    // one interval's Pd under CA-CFAR at an SNR x: a Rayleigh target's closed form; the others', the fixed threshold's
    // Pd at alpha z averaged over the reference cells' mean z (a gamma of N, mean 1), tabulated in x (dB)
    const Fluctuation f = fluctuation == 0 ? Fluctuation::Steady : fluctuation == 2 ? Fluctuation::DominantScatterer : Fluctuation::Rayleigh;
    constexpr double kLoDb = -20.0, kStepDb = 0.25;
    constexpr int kTable = 281; // (to +50 dB)
    std::vector<double> table;
    if (f != Fluctuation::Rayleigh) {
        const double cells = cfarCells, spread = 1.0 / std::sqrt(cells);
        const double z0 = std::max(1e-6, 1.0 - 7.0 * spread), z1 = 1.0 + 10.0 * spread;
        constexpr int kZ = 160;
        std::vector<double> z(kZ), w(kZ);
        double norm = 0.0;
        for (int j = 0; j < kZ; ++j) {
            z[static_cast<std::size_t>(j)] = z0 + (j + 0.5) * (z1 - z0) / kZ;
            w[static_cast<std::size_t>(j)] = std::exp((cells - 1.0) * std::log(z[static_cast<std::size_t>(j)]) - cells * z[static_cast<std::size_t>(j)]);
            norm += w[static_cast<std::size_t>(j)];
        }
        table.resize(kTable);
        for (int t = 0; t < kTable; ++t) {
            const double x = std::pow(10.0, (kLoDb + t * kStepDb) / 10.0);
            double pd = 0.0;
            for (int j = 0; j < kZ; ++j)
                pd += w[static_cast<std::size_t>(j)] * probabilityOfDetection(x, std::exp(-alpha * z[static_cast<std::size_t>(j)]), f);
            table[static_cast<std::size_t>(t)] = pd / norm;
        }
    }
    auto pdCell = [&](double x) {
        if (f == Fluctuation::Rayleigh) return std::pow(1.0 + alpha / (cfarCells * (1.0 + x)), -cfarCells);
        const double u = (10.0 * std::log10(std::max(x, 1e-30)) - kLoDb) / kStepDb;
        if (u <= 0.0) return table.front();
        if (u >= kTable - 1) return table.back();
        const auto i = static_cast<std::size_t>(u);
        return table[i] + (u - static_cast<double>(i)) * (table[i + 1] - table[i]);
    };
    auto pdSignal = [&](double snr) { // the dwell's Pd at a total SNR
        std::vector<double> p(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            double sum = 0.0;
            for (const double l : losses[static_cast<std::size_t>(i)])
                sum += pdCell(snr / n * l);
            p[static_cast<std::size_t>(i)] = sum / static_cast<double>(losses[static_cast<std::size_t>(i)].size());
        }
        // at least m of the n, each its own probability (a recursion over the count)
        std::vector<double> dist(static_cast<std::size_t>(n) + 1, 0.0);
        dist[0] = 1.0;
        for (int i = 0; i < n; ++i)
            for (int k = i + 1; k >= 0; --k)
                dist[static_cast<std::size_t>(k)] = dist[static_cast<std::size_t>(k)] * (1.0 - p[static_cast<std::size_t>(i)]) +
                                                    (k > 0 ? dist[static_cast<std::size_t>(k - 1)] * p[static_cast<std::size_t>(i)] : 0.0);
        double total = 0.0;
        for (int k = m; k <= n; ++k) total += dist[static_cast<std::size_t>(k)];
        return total;
    };
    auto pdIdeal = [&](double snr) { // each interval 1/N of the energy, its threshold fixed (no CFAR), no losses
        const double p = probabilityOfDetection(snr / n, idealPfaInterval > 0.0 ? idealPfaInterval : pfaInterval, f);
        double total = 0.0, coef = 1.0;
        for (int k = 0; k <= n; ++k) {
            if (k > 0) coef *= static_cast<double>(n - k + 1) / k;
            if (k >= m) total += coef * std::pow(p, k) * std::pow(1.0 - p, n - k);
        }
        return total;
    };
    auto half = [](auto&& pd) {
        double lo = -10.0, hi = 60.0;
        for (int i = 0; i < 40; ++i) {
            const double mid = 0.5 * (lo + hi);
            (pd(std::pow(10.0, mid / 10.0)) < 0.5 ? lo : hi) = mid;
        }
        return 0.5 * (lo + hi);
    };
    return half(pdSignal) - half(pdIdeal);
}

double intervalPfa(double pfaDwell, const Waveform& waveform, int coincidence, double q) noexcept {
    const int n = static_cast<int>(waveform.intervals.size()), m = waveform.m;
    if (n <= 0) return pfaDwell;
    double cover = 0.0;
    for (const Interval& iv : waveform.intervals) cover += static_cast<double>(iv.gates - iv.blankedGates) / (iv.ambiguousRangeM / waveform.gateM);
    cover = std::clamp(cover / n, 1e-3, 1.0);
    const double k = coincidences(m, coincidence);
    return std::min(0.5, std::pow(pfaDwell / (binomial(n, m) * k * k), 1.0 / m) / (cover * std::max(1e-3, q)));
}

double intervalPfa(double pfaDwell, const Waveform& waveform, int coincidence, int cfarCells) noexcept {
    const int n = static_cast<int>(waveform.intervals.size()), m = waveform.m;
    if (n <= 0) return pfaDwell;
    // the share of each interval's gates it receives in
    double cover = 0.0;
    for (const Interval& iv : waveform.intervals) cover += static_cast<double>(iv.gates - iv.blankedGates) / (iv.ambiguousRangeM / waveform.gateM);
    cover = std::clamp(cover / n, 1e-3, 1.0);
    const double k = coincidences(m, coincidence);
    const double target = std::pow(pfaDwell / (binomial(n, m) * k * k), 1.0 / m) / cover; // = q(p) p
    double p = target;
    for (int i = 0; i < 4; ++i) p = std::min(0.5, target / std::max(1e-3, peakSurvival(p, cfarCells)));
    return p;
}

} // namespace fsim::ms::radar::signal
