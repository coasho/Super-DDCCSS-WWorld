#include "ms/rcs/Table.h"

#include "core/Json.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

namespace fsim::ms::rcs {
using core::Json;

namespace {

constexpr char kMagic[8] = {'F', 'S', 'R', 'C', 'S', 0, 1, 0};
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kFloats = sizeof(Moments) / sizeof(float);
static_assert(kFloats == 10, "Moments is ten floats");

/// The band below (`lo`) and above (`hi`) a frequency and the log-frequency weight of `hi`.
void bracket(const Table& t, double f, std::size_t& lo, std::size_t& hi, double& w) noexcept {
    lo = hi = 0, w = 0.0;
    if (t.bands.empty()) return;
    if (f <= t.bands.front().frequencyHz) return;
    for (std::size_t i = 1; i < t.bands.size(); ++i)
        if (f <= t.bands[i].frequencyHz) {
            lo = i - 1, hi = i;
            w = std::log(f / t.bands[lo].frequencyHz) / std::log(t.bands[hi].frequencyHz / t.bands[lo].frequencyHz);
            return;
        }
    lo = hi = t.bands.size() - 1;
}

} // namespace

double sigmaOf(const Moments& m, double tx, double rx) noexcept {
    const double tt = std::cos(tx), tp = std::sin(tx), rt = std::cos(rx), rp = std::sin(rx);
    const double a = rt * tt, b = rp * tp, c = rt * tp + rp * tt; // (reciprocal: S_pt = S_tp)
    const double s = a * a * double(m.tt) + b * b * double(m.pp) + c * c * double(m.tp) + 2.0 * a * b * double(m.ttppRe) +
                     2.0 * a * c * double(m.tttpRe) + 2.0 * b * c * double(m.pptpRe);
    return std::max(0.0, s);
}

void Table::bin(const double d[3], int& azimuth, int& elevation) const noexcept {
    const double h = std::hypot(d[0], d[1]);
    double az = std::atan2(d[1], d[0]) * 180.0 / kPi;
    if (az < 0.0) az += 360.0;
    const double el = std::atan2(-d[2], h) * 180.0 / kPi;
    azimuth = static_cast<int>(std::floor(az / binDeg + 0.5)) % azimuthBins;
    elevation = std::clamp(static_cast<int>(std::floor((el + 90.0) / binDeg + 0.5)), 0, elevationBins - 1);
}

double Table::sigma(const double d[3], double f, double tx, double rx) const noexcept {
    if (bands.empty()) return 0.0;
    int az, el;
    bin(d, az, el);
    std::size_t lo, hi;
    double w;
    bracket(*this, f, lo, hi, w);
    const double a = std::max(sigmaOf(at(lo, az, el), tx, rx), 1e-12);
    if (lo == hi) return a;
    const double b = std::max(sigmaOf(at(hi, az, el), tx, rx), 1e-12);
    return std::exp(std::log(a) * (1.0 - w) + std::log(b) * w);
}

double Table::fluctuation(const double d[3], double f) const noexcept {
    if (bands.empty()) return 1.0;
    int az, el;
    bin(d, az, el);
    std::size_t lo, hi;
    double w;
    bracket(*this, f, lo, hi, w);
    return double(at(lo, az, el).fluctuation) * (1.0 - w) + double(at(hi, az, el).fluctuation) * w;
}

std::shared_ptr<const Table> readTable(const std::filesystem::path& file, std::string* error) {
    auto failed = [&](const std::string& why) -> std::shared_ptr<const Table> {
        if (error) *error = file.string() + ": " + why;
        return nullptr;
    };
    std::ifstream in(file, std::ios::binary);
    if (!in) return failed("cannot open");
    char magic[8];
    std::uint32_t length = 0;
    if (!in.read(magic, 8) || std::memcmp(magic, kMagic, 8) != 0) return failed("not an RCS table (FSRCS 1)");
    if (!in.read(reinterpret_cast<char*>(&length), 4) || length > (1u << 24)) return failed("bad header");
    std::string text(length, '\0');
    if (!in.read(text.data(), length)) return failed("truncated header");
    auto t = std::make_shared<Table>();
    try {
        const Json h = Json::parse(text, file.string());
        t->aircraft = h.string("aircraft", "");
        t->binDeg = h.number("bin_deg", 3.0);
        t->azimuthBins = static_cast<int>(h.number("azimuth_bins", 0));
        t->elevationBins = static_cast<int>(h.number("elevation_bins", 0));
        t->provenance = h.child("provenance").dump();
        for (const Json& f : h.child("bands_hz").asArray()) t->bands.push_back(Band{f.asNumber(), {}});
    } catch (const std::exception& e) {
        return failed(e.what());
    }
    if (t->azimuthBins <= 0 || t->elevationBins <= 0 || t->binDeg <= 0.0 || t->bands.empty()) return failed("bad dimensions");
    const auto n = static_cast<std::size_t>(t->azimuthBins) * static_cast<std::size_t>(t->elevationBins);
    for (Band& b : t->bands) {
        b.bins.resize(n);
        if (!in.read(reinterpret_cast<char*>(b.bins.data()), static_cast<std::streamsize>(n * sizeof(Moments)))) return failed("truncated data");
    }
    return t;
}

bool writeTable(const Table& t, const std::filesystem::path& file, std::string* error) {
    Json h;
    h.set("aircraft", t.aircraft);
    h.set("bin_deg", t.binDeg);
    h.set("azimuth_bins", t.azimuthBins);
    h.set("elevation_bins", t.elevationBins);
    Json bands;
    for (const Band& b : t.bands) bands.push(b.frequencyHz);
    h.set("bands_hz", bands);
    h.set("moments", "tt pp tp ttpp_re ttpp_im tttp_re tttp_im pptp_re pptp_im fluctuation (float32, [band][elevation][azimuth])");
    try {
        h.set("provenance", t.provenance.empty() ? Json() : Json::parse(t.provenance));
    } catch (const std::exception&) {
        h.set("provenance", t.provenance);
    }
    const std::string text = h.dump();
    std::ofstream out(file, std::ios::binary);
    if (!out) {
        if (error) *error = file.string() + ": cannot write";
        return false;
    }
    const auto length = static_cast<std::uint32_t>(text.size());
    out.write(kMagic, 8);
    out.write(reinterpret_cast<const char*>(&length), 4);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    for (const Band& b : t.bands) out.write(reinterpret_cast<const char*>(b.bins.data()), static_cast<std::streamsize>(b.bins.size() * sizeof(Moments)));
    return static_cast<bool>(out);
}

} // namespace fsim::ms::rcs
