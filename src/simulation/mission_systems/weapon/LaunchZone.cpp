#include "simulation/mission_systems/weapon/LaunchZone.h"
#include "foundation/core/Json.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>

namespace fsim::ms::weapon {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

double dot3(const double a[3], const double b[3]) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
double norm3(const double a[3]) noexcept {
    return std::sqrt(dot3(a, a));
}

bool validStrictAxis(const std::vector<double>& axis, double minVal, double maxVal) noexcept {
    if (axis.size() < 2) return false;
    for (std::size_t i = 0; i < axis.size(); ++i) {
        if (!std::isfinite(axis[i]) || axis[i] < minVal || axis[i] > maxVal) return false;
        if (i > 0 && !(axis[i] > axis[i - 1])) return false;
    }
    return true;
}

bool readDoubleArray(const core::Json& node, const char* key, std::vector<double>& out, std::string& error) {
    if (!node.has(key) || !node.child(key).isArray()) {
        error = std::string("missing or non-array field: ") + key;
        return false;
    }
    const auto& arr = node.child(key).asArray();
    out.clear();
    out.reserve(arr.size());
    for (const auto& item : arr) {
        if (!item.isNumber()) {
            error = std::string("non-numeric entry in array: ") + key;
            return false;
        }
        const double v = item.asNumber();
        if (!std::isfinite(v)) {
            error = std::string("non-finite entry in array: ") + key;
            return false;
        }
        out.push_back(v);
    }
    return true;
}

struct Bracket {
    std::size_t i0 = 0;
    std::size_t i1 = 0;
    double t = 0.0;
    bool inDomain = true;
};

Bracket findBracket(const std::vector<double>& axis, double value) noexcept {
    Bracket b;
    if (axis.empty()) return b;
    if (value <= axis.front()) {
        b.i0 = 0;
        b.i1 = 0;
        b.t = 0.0;
        b.inDomain = (value >= axis.front() - 1e-9);
        return b;
    }
    if (value >= axis.back()) {
        b.i0 = axis.size() - 1;
        b.i1 = axis.size() - 1;
        b.t = 0.0;
        b.inDomain = (value <= axis.back() + 1e-9);
        return b;
    }
    const auto it = std::upper_bound(axis.begin(), axis.end(), value);
    b.i1 = static_cast<std::size_t>(it - axis.begin());
    b.i0 = b.i1 - 1;
    const double span = axis[b.i1] - axis[b.i0];
    b.t = span > 0.0 ? std::clamp((value - axis[b.i0]) / span, 0.0, 1.0) : 0.0;
    b.inDomain = true;
    return b;
}

double interp1d(const std::vector<double>& xs, const std::vector<double>& ys, double x) noexcept {
    if (xs.empty() || xs.size() != ys.size()) return 1.0;
    const Bracket b = findBracket(xs, x);
    return ys[b.i0] * (1.0 - b.t) + ys[b.i1] * b.t;
}

struct GridSample {
    double rMaxM = 0.0;
    double rNeM = 0.0;
    double rMinM = 0.0;
    double tofMaxS = 0.0;
    double tofNeS = 0.0;
    double tofMinS = 0.0;
    bool inEnvelope = false;
};

GridSample sampleGrid5D(const LaunchZoneTableV1& table,
                        double launchAltM, double launchMach,
                        double targetDhM, double targetMach, double aspectDeg) noexcept {
    const Bracket b0 = findBracket(table.launchAltitudeM, launchAltM);
    const Bracket b1 = findBracket(table.launchMach, launchMach);
    const Bracket b2 = findBracket(table.targetAltitudeDeltaM, targetDhM);
    const Bracket b3 = findBracket(table.targetMach, targetMach);
    const Bracket b4 = findBracket(table.aspectDeg, aspectDeg);

    const std::size_t n1 = table.launchMach.size();
    const std::size_t n2 = table.targetAltitudeDeltaM.size();
    const std::size_t n3 = table.targetMach.size();
    const std::size_t n4 = table.aspectDeg.size();

    const std::size_t s3 = n4;
    const std::size_t s2 = n3 * s3;
    const std::size_t s1 = n2 * s2;
    const std::size_t s0 = n1 * s1;

    const std::size_t idx0[2] = {b0.i0, b0.i1};
    const std::size_t idx1[2] = {b1.i0, b1.i1};
    const std::size_t idx2[2] = {b2.i0, b2.i1};
    const std::size_t idx3[2] = {b3.i0, b3.i1};
    const std::size_t idx4[2] = {b4.i0, b4.i1};

    const double w0[2] = {1.0 - b0.t, b0.t};
    const double w1[2] = {1.0 - b1.t, b1.t};
    const double w2[2] = {1.0 - b2.t, b2.t};
    const double w3[2] = {1.0 - b3.t, b3.t};
    const double w4[2] = {1.0 - b4.t, b4.t};

    GridSample out;
    out.inEnvelope = b0.inDomain && b1.inDomain && b2.inDomain && b3.inDomain && b4.inDomain;

    for (int a = 0; a < 2; ++a) {
        if (w0[a] == 0.0) continue;
        const std::size_t o0 = idx0[a] * s0;
        for (int b = 0; b < 2; ++b) {
            if (w1[b] == 0.0) continue;
            const double w01 = w0[a] * w1[b];
            const std::size_t o1 = o0 + idx1[b] * s1;
            for (int c = 0; c < 2; ++c) {
                if (w2[c] == 0.0) continue;
                const double w012 = w01 * w2[c];
                const std::size_t o2 = o1 + idx2[c] * s2;
                for (int d = 0; d < 2; ++d) {
                    if (w3[d] == 0.0) continue;
                    const double w0123 = w012 * w3[d];
                    const std::size_t o3 = o2 + idx3[d] * s3;
                    for (int e = 0; e < 2; ++e) {
                        if (w4[e] == 0.0) continue;
                        const double w = w0123 * w4[e];
                        const std::size_t flat = o3 + idx4[e];
                        out.rMaxM += w * table.rMaxM[flat];
                        out.rNeM += w * table.rNeM[flat];
                        out.rMinM += w * table.rMinM[flat];
                        out.tofMaxS += w * table.tofMaxS[flat];
                        out.tofNeS += w * table.tofNeS[flat];
                        out.tofMinS += w * table.tofMinS[flat];
                    }
                }
            }
        }
    }

    // Enforce physical monotonicity after floating-point summation.
    out.rMaxM = std::max(0.0, out.rMaxM);
    out.rNeM = std::clamp(out.rNeM, 0.0, out.rMaxM);
    out.rMinM = std::clamp(out.rMinM, 0.0, out.rMaxM);
    out.tofMaxS = std::max(0.0, out.tofMaxS);
    out.tofNeS = std::clamp(out.tofNeS, 0.0, out.tofMaxS);
    out.tofMinS = std::clamp(out.tofMinS, 0.0, out.tofMaxS);
    return out;
}

struct GeometryMetrics {
    double r[3] = {0.0, 0.0, 0.0};
    double slantRangeM = 0.0;
    double closingSpeedMs = 0.0;
    double aspectRad = 0.5 * kPi;
    double offBoresightRad = 0.0;
    double launchMach = 0.0;
    double targetMach = 0.0;
    double targetDhM = 0.0;
};

GeometryMetrics computeGeometry(const LaunchZoneQueryV1& q) noexcept {
    GeometryMetrics g;
    for (int k = 0; k < 3; ++k) g.r[k] = q.targetPosEcef[k] - q.launchPosEcef[k];
    g.slantRangeM = norm3(g.r);

    double u[3] = {1.0, 0.0, 0.0};
    if (g.slantRangeM > 1e-6) {
        for (int k = 0; k < 3; ++k) u[k] = g.r[k] / g.slantRangeM;
    }

    double dv[3] = {q.launchVelEcef[0] - q.targetVelEcef[0],
                    q.launchVelEcef[1] - q.targetVelEcef[1],
                    q.launchVelEcef[2] - q.targetVelEcef[2]};
    g.closingSpeedMs = dot3(dv, u);

    const double fwdNorm = norm3(q.launchForwardEcef);
    if (fwdNorm > 1e-6 && g.slantRangeM > 1e-6) {
        const double cosAta = std::clamp(dot3(q.launchForwardEcef, u) / fwdNorm, -1.0, 1.0);
        g.offBoresightRad = std::acos(cosAta);
    }

    const double tvNorm = norm3(q.targetVelEcef);
    if (tvNorm > 1.0 && g.slantRangeM > 1e-6) {
        // Aspect 0 = head-on (target velocity points toward launcher, i.e. along -u).
        const double cosAspect = std::clamp(-dot3(q.targetVelEcef, u) / tvNorm, -1.0, 1.0);
        g.aspectRad = std::acos(cosAspect);
    } else {
        g.aspectRad = 0.5 * kPi;
    }

    const double lvNorm = q.launchAirspeedMs > 0.0 ? q.launchAirspeedMs : norm3(q.launchVelEcef);
    g.launchMach = lvNorm / isaSoundSpeedMs(q.launchAltitudeM);
    g.targetMach = tvNorm / isaSoundSpeedMs(q.targetAltitudeM);
    g.targetDhM = q.targetAltitudeM - q.launchAltitudeM;
    return g;
}

double predictSeparationAt(const LaunchZoneQueryV1& q, double dtS) noexcept {
    if (!std::isfinite(dtS) || dtS < 0.0) return std::numeric_limits<double>::quiet_NaN();
    double d[3];
    for (int k = 0; k < 3; ++k) {
        const double pT = q.targetPosEcef[k] + q.targetVelEcef[k] * dtS;
        const double pL = q.launchPosEcef[k] + q.launchVelEcef[k] * dtS;
        d[k] = pT - pL;
    }
    return norm3(d);
}

} // namespace

double isaSoundSpeedMs(double altitudeM) noexcept {
    const double h = std::clamp(std::isfinite(altitudeM) ? altitudeM : 0.0, 0.0, 20000.0);
    const double tempK = std::max(216.65, 288.15 - 0.0065 * h);
    return std::sqrt(1.4 * 287.05287 * tempK);
}

std::string computeSha256Hex(const std::string& bytes) {
    static constexpr std::uint32_t k[64] = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
    };
    std::uint32_t h[8] = {
        0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
        0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u
    };
    const auto rotr = [](std::uint32_t x, unsigned n) noexcept { return (x >> n) | (x << (32u - n)); };

    const std::uint64_t bitLen = static_cast<std::uint64_t>(bytes.size()) * 8ull;
    const std::size_t rem = bytes.size() % 64u;
    const std::size_t pad = (rem < 56u) ? (56u - rem) : (120u - rem);
    std::vector<std::uint8_t> msg;
    msg.reserve(bytes.size() + pad + 8u);
    for (char ch : bytes) msg.push_back(static_cast<std::uint8_t>(static_cast<unsigned char>(ch)));
    msg.push_back(0x80u);
    msg.insert(msg.end(), pad - 1u, 0x00u);
    for (int i = 7; i >= 0; --i) msg.push_back(static_cast<std::uint8_t>((bitLen >> (i * 8)) & 0xffu));

    for (std::size_t off = 0; off < msg.size(); off += 64u) {
        std::uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            const std::size_t j = off + static_cast<std::size_t>(i) * 4u;
            w[i] = (static_cast<std::uint32_t>(msg[j]) << 24) |
                   (static_cast<std::uint32_t>(msg[j + 1]) << 16) |
                   (static_cast<std::uint32_t>(msg[j + 2]) << 8) |
                   static_cast<std::uint32_t>(msg[j + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const std::uint32_t s0 = rotr(w[i - 15], 7u) ^ rotr(w[i - 15], 18u) ^ (w[i - 15] >> 3u);
            const std::uint32_t s1 = rotr(w[i - 2], 17u) ^ rotr(w[i - 2], 19u) ^ (w[i - 2] >> 10u);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const std::uint32_t S1 = rotr(e, 6u) ^ rotr(e, 11u) ^ rotr(e, 25u);
            const std::uint32_t ch = (e & f) ^ ((~e) & g);
            const std::uint32_t t1 = hh + S1 + ch + k[i] + w[i];
            const std::uint32_t S0 = rotr(a, 2u) ^ rotr(a, 13u) ^ rotr(a, 22u);
            const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t t2 = S0 + maj;
            hh = g; g = f; f = e; e = d + t1;
            d = c; c = b; b = a; a = t1 + t2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d;
        h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }

    static constexpr char kHex[] = "0123456789abcdef";
    std::string hex;
    hex.resize(64);
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            hex[static_cast<std::size_t>(i * 8 + j)] = kHex[(h[i] >> ((7 - j) * 4)) & 0xfu];
        }
    }
    return hex;
}

bool loadLaunchZoneTableV1(const std::filesystem::path& path, LaunchZoneTableV1& out, std::string& error) {
    error.clear();
    std::ifstream in(path, std::ios::binary);
    if (!in.good()) {
        error = "unable to open DLZ table: " + path.string();
        return false;
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    if (!in.good() && !in.eof()) {
        error = "failed to read DLZ table: " + path.string();
        return false;
    }
    const std::string raw = buffer.str();
    try {
        const auto doc = core::Json::parse(raw, path.string());
        if (!doc.isObject()) {
            error = "DLZ table root must be a JSON object";
            return false;
        }
        LaunchZoneTableV1 next;
        next.tableSha256 = computeSha256Hex(raw);
        if (!doc.has("schema") || !doc.child("schema").isString() || doc.child("schema").asString() != "fsim.ms.dlz.v1") {
            error = "unsupported DLZ table schema";
            return false;
        }
        next.schema = doc.child("schema").asString();
        const auto reqStr = [&](const char* k, std::string& dst) -> bool {
            if (!doc.has(k) || !doc.child(k).isString() || doc.child(k).asString().empty()) {
                error = std::string("missing string field: ") + k;
                return false;
            }
            dst = doc.child(k).asString();
            return true;
        };
        if (!reqStr("card", next.card) || !reqStr("model", next.model) || !reqStr("seeker", next.seeker) ||
            !reqStr("basis", next.basis) || !reqStr("model_xml_sha256", next.modelXmlSha256) ||
            !reqStr("card_sha256", next.cardSha256)) {
            return false;
        }
        if (doc.has("equipment_calibrated") && doc.child("equipment_calibrated").isBool()) {
            next.equipmentCalibrated = doc.child("equipment_calibrated").asBool();
        }
        const auto reqNum = [&](const core::Json& obj, const char* k, double& dst) -> bool {
            if (!obj.has(k) || !obj.child(k).isNumber() || !std::isfinite(obj.child(k).asNumber())) {
                error = std::string("missing or non-finite numeric field: ") + k;
                return false;
            }
            dst = obj.child(k).asNumber();
            return true;
        };
        if (!reqNum(doc, "arming_s", next.armingS) || !reqNum(doc, "life_s", next.lifeS) ||
            !reqNum(doc, "active_range_m", next.activeRangeM) ||
            !reqNum(doc, "max_off_boresight_rad", next.maxOffBoresightRad) ||
            !reqNum(doc, "escape_turn_g", next.escapeTurnG) ||
            !reqNum(doc, "min_terminal_speed_ms", next.minTerminalSpeedMs)) {
            return false;
        }
        if (next.armingS < 0.0 || next.lifeS <= 0.0 || next.activeRangeM < 0.0 ||
            next.maxOffBoresightRad <= 0.0 || next.maxOffBoresightRad > kPi ||
            next.escapeTurnG <= 0.0 || next.minTerminalSpeedMs <= 0.0) {
            error = "DLZ scalar metadata outside physical bounds";
            return false;
        }

        if (!doc.has("axes") || !doc.child("axes").isObject()) {
            error = "missing axes object";
            return false;
        }
        const auto& axes = doc.child("axes");
        if (!readDoubleArray(axes, "launch_altitude_m", next.launchAltitudeM, error) ||
            !readDoubleArray(axes, "launch_mach", next.launchMach, error) ||
            !readDoubleArray(axes, "target_altitude_delta_m", next.targetAltitudeDeltaM, error) ||
            !readDoubleArray(axes, "target_mach", next.targetMach, error) ||
            !readDoubleArray(axes, "aspect_deg", next.aspectDeg, error)) {
            return false;
        }
        if (!validStrictAxis(next.launchAltitudeM, 0.0, 40000.0) ||
            !validStrictAxis(next.launchMach, 0.05, 6.0) ||
            !validStrictAxis(next.targetAltitudeDeltaM, -30000.0, 30000.0) ||
            !validStrictAxis(next.targetMach, 0.05, 6.0) ||
            !validStrictAxis(next.aspectDeg, 0.0, 180.0)) {
            error = "DLZ axes must be strictly increasing and within physical bounds";
            return false;
        }

        if (!readDoubleArray(doc, "off_boresight_deg", next.offBoresightDeg, error) ||
            !readDoubleArray(doc, "off_boresight_factor", next.offBoresightFactor, error)) {
            return false;
        }
        if (!validStrictAxis(next.offBoresightDeg, 0.0, 180.0) ||
            next.offBoresightDeg.size() != next.offBoresightFactor.size()) {
            error = "invalid off-boresight curve";
            return false;
        }
        for (double f : next.offBoresightFactor) {
            if (!(f > 0.0 && f <= 1.000001)) {
                error = "off-boresight factor out of (0, 1]";
                return false;
            }
        }

        if (!doc.has("uncertainty_fraction") || !doc.child("uncertainty_fraction").isObject()) {
            error = "missing uncertainty_fraction object";
            return false;
        }
        const auto& unc = doc.child("uncertainty_fraction");
        if (!reqNum(unc, "r_max", next.uncRMax) || !reqNum(unc, "r_ne", next.uncRNe) ||
            !reqNum(unc, "r_min", next.uncRMin) || !reqNum(unc, "tof", next.uncTof)) {
            return false;
        }
        if (!(next.uncRMax > 0.0 && next.uncRMax <= 1.0 &&
              next.uncRNe > 0.0 && next.uncRNe <= 1.0 &&
              next.uncRMin > 0.0 && next.uncRMin <= 1.0 &&
              next.uncTof > 0.0 && next.uncTof <= 1.0)) {
            error = "uncertainty fractions must lie in (0, 1]";
            return false;
        }

        if (!doc.has("grid") || !doc.child("grid").isObject()) {
            error = "missing grid object";
            return false;
        }
        const auto& grid = doc.child("grid");
        if (!readDoubleArray(grid, "r_max_m", next.rMaxM, error) ||
            !readDoubleArray(grid, "r_ne_m", next.rNeM, error) ||
            !readDoubleArray(grid, "r_min_m", next.rMinM, error) ||
            !readDoubleArray(grid, "tof_max_s", next.tofMaxS, error) ||
            !readDoubleArray(grid, "tof_ne_s", next.tofNeS, error) ||
            !readDoubleArray(grid, "tof_min_s", next.tofMinS, error)) {
            return false;
        }
        const std::size_t expected = next.gridSize();
        if (expected == 0 || next.rMaxM.size() != expected || next.rNeM.size() != expected ||
            next.rMinM.size() != expected || next.tofMaxS.size() != expected ||
            next.tofNeS.size() != expected || next.tofMinS.size() != expected) {
            error = "DLZ grid array size mismatch";
            return false;
        }
        for (std::size_t i = 0; i < expected; ++i) {
            if (next.rMaxM[i] < 0.0 || next.rNeM[i] < 0.0 || next.rMinM[i] < 0.0 ||
                next.tofMaxS[i] < 0.0 || next.tofNeS[i] < 0.0 || next.tofMinS[i] < 0.0) {
                error = "negative range or time-of-flight in DLZ grid";
                return false;
            }
            if (next.rNeM[i] > next.rMaxM[i] + 1e-6 || next.rMinM[i] > next.rMaxM[i] + 1e-6) {
                error = "inconsistent R_min/R_ne/R_max ordering in DLZ grid";
                return false;
            }
        }

        out = std::move(next);
        return true;
    } catch (const std::exception& ex) {
        error = ex.what();
        return false;
    }
}

LaunchZoneV2 evaluateLaunchZoneV2(const LaunchZoneTableV1& table, const LaunchZoneQueryV1& query) noexcept {
    LaunchZoneV2 z;
    z.version = 2;
    z.valid = true;
    z.flyoutDerived = true;
    z.basis = table.basis;
    z.tableSha256 = table.tableSha256;

    const GeometryMetrics g = computeGeometry(query);
    z.slantRangeM = g.slantRangeM;
    z.closingSpeedMs = g.closingSpeedMs;
    z.aspectRad = g.aspectRad;
    z.offBoresightRad = g.offBoresightRad;

    const GridSample s = sampleGrid5D(table, query.launchAltitudeM, g.launchMach,
                                      g.targetDhM, g.targetMach, g.aspectRad * kRadToDeg);
    z.inEnvelope = s.inEnvelope;

    const double ataDeg = g.offBoresightRad * kRadToDeg;
    const double ataFactor = std::clamp(interp1d(table.offBoresightDeg, table.offBoresightFactor, ataDeg), 0.1, 1.0);

    z.maximumM = s.rMaxM;
    z.probabilityInterceptM = s.rMaxM * ataFactor;
    z.noEscapeM = std::min(s.rNeM * ataFactor, z.probabilityInterceptM);
    z.minimumM = std::min(s.rMinM, z.maximumM);

    z.maximumUncertaintyM = z.maximumM * table.uncRMax;
    z.noEscapeUncertaintyM = z.noEscapeM * table.uncRNe;
    z.minimumUncertaintyM = z.minimumM * table.uncRMin;

    const bool withinFov = g.offBoresightRad <= table.maxOffBoresightRad + 1e-9;
    z.inRange = withinFov && z.probabilityInterceptM > 0.0 &&
                z.slantRangeM >= z.minimumM && z.slantRangeM <= z.probabilityInterceptM;
    z.inNoEscape = withinFov && z.noEscapeM > 0.0 &&
                   z.slantRangeM >= z.minimumM && z.slantRangeM <= z.noEscapeM;

    // Predict time of flight across [R_min, R_ne, R_pi] when slant range is achievable.
    if (z.slantRangeM >= z.minimumM && z.slantRangeM <= z.probabilityInterceptM && z.probabilityInterceptM > 0.0) {
        const double rMin = z.minimumM;
        const double rNe = std::clamp(z.noEscapeM, rMin, z.probabilityInterceptM);
        const double rMax = z.probabilityInterceptM;
        const double tMin = s.tofMinS;
        const double tNe = std::max(tMin, s.tofNeS);
        const double tMax = std::max(tNe, s.tofMaxS);

        double tof = tMin;
        if (z.slantRangeM <= rNe && rNe > rMin + 1e-6) {
            const double alpha = (z.slantRangeM - rMin) / (rNe - rMin);
            tof = tMin + alpha * (tNe - tMin);
        } else if (rMax > rNe + 1e-6) {
            const double alpha = (z.slantRangeM - rNe) / (rMax - rNe);
            tof = tNe + alpha * (tMax - tNe);
        } else if (rMax > rMin + 1e-6) {
            const double alpha = (z.slantRangeM - rMin) / (rMax - rMin);
            tof = tMin + alpha * (tMax - tMin);
        }
        z.timeOfFlightS = std::max(0.0, tof);
        z.timeOfFlightUncertaintyS = z.timeOfFlightS * table.uncTof;
        z.fPoleM = predictSeparationAt(query, z.timeOfFlightS);

        if (table.seeker == "active_radar" && table.activeRangeM > 0.0) {
            if (z.slantRangeM <= table.activeRangeM) {
                z.aPoleM = z.slantRangeM;
            } else {
                const double tActive = z.timeOfFlightS * std::clamp(1.0 - table.activeRangeM / z.slantRangeM, 0.0, 1.0);
                z.aPoleM = predictSeparationAt(query, tActive);
            }
        }
    }

    return z;
}

LaunchZoneWezV1 evaluateLaunchZoneWezV1(const LaunchZoneTableV1& table, const LaunchZoneQueryV1& query) {
    LaunchZoneWezV1 wez;
    wez.version = 1;
    wez.valid = true;
    wez.flyoutDerived = true;
    wez.basis = table.basis;
    wez.tableSha256 = table.tableSha256;

    const GeometryMetrics g = computeGeometry(query);
    wez.launchAltitudeM = query.launchAltitudeM;
    wez.launchMach = g.launchMach;
    wez.targetAltitudeM = query.targetAltitudeM;
    wez.targetMach = g.targetMach;
    wez.maxOffBoresightRad = table.maxOffBoresightRad;

    static constexpr double kAspectsDeg[7] = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
    wez.aspectContour.reserve(7);
    bool allInEnvelope = true;
    for (double deg : kAspectsDeg) {
        const GridSample s = sampleGrid5D(table, query.launchAltitudeM, g.launchMach,
                                          g.targetDhM, g.targetMach, deg);
        allInEnvelope = allInEnvelope && s.inEnvelope;
        LaunchZoneWezSampleV1 sample;
        sample.aspectRad = deg * kDegToRad;
        sample.maximumM = s.rMaxM;
        sample.noEscapeM = s.rNeM;
        sample.minimumM = s.rMinM;
        sample.maximumUncertaintyM = s.rMaxM * table.uncRMax;
        sample.noEscapeUncertaintyM = s.rNeM * table.uncRNe;
        wez.aspectContour.push_back(sample);
    }
    wez.inEnvelope = allInEnvelope;
    return wez;
}

LaunchZoneV2 evaluateLegacyLaunchZoneV2(const StoreCard& card, const LaunchZoneQueryV1& query) {
    LaunchZoneV2 z;
    z.version = 2;
    z.valid = true;
    z.flyoutDerived = false;
    z.inEnvelope = false;
    z.basis = "assumed";

    const GeometryMetrics g = computeGeometry(query);
    z.slantRangeM = g.slantRangeM;
    z.closingSpeedMs = g.closingSpeedMs;
    z.aspectRad = g.aspectRad;
    z.offBoresightRad = g.offBoresightRad;

    const bool heat = card.seeker.find("infrared") != std::string::npos;
    const double reference = card.value("launch_zone_reference_m", heat ? 14000.0 : 60000.0);
    const double altitude = std::clamp(0.65 + query.launchAltitudeM / 20000.0, 0.5, 1.4);
    const double speed = std::clamp(query.launchAirspeedMs / 300.0, 0.5, 1.5);
    const double tv = norm3(query.targetVelEcef);
    const double closingTarget = -dot3(query.targetVelEcef, g.r) / std::max(g.slantRangeM, 1.0);

    z.maximumM = std::max(0.0, reference * altitude * speed + closingTarget * 25.0);
    z.noEscapeM = std::clamp(z.maximumM * 0.55 - tv * 12.0, 0.0, z.maximumM);
    z.minimumM = 900.0;
    z.probabilityInterceptM = z.maximumM;

    // Wide conservative uncertainty for assumed advisory fallback.
    z.maximumUncertaintyM = z.maximumM * 0.35;
    z.noEscapeUncertaintyM = z.noEscapeM * 0.35;
    z.minimumUncertaintyM = z.minimumM * 0.35;

    const double maxFovRad = card.value("seeker_gimbal_deg", heat ? 45.0 : 60.0) * kDegToRad;
    z.inRange = (g.offBoresightRad <= maxFovRad) && (z.slantRangeM >= z.minimumM) && (z.slantRangeM <= z.maximumM);
    z.inNoEscape = (g.offBoresightRad <= maxFovRad) && (z.slantRangeM >= z.minimumM) && (z.slantRangeM <= z.noEscapeM);
    return z;
}

LaunchZoneWezV1 evaluateLegacyLaunchZoneWezV1(const StoreCard& card, const LaunchZoneQueryV1& query) {
    LaunchZoneWezV1 wez;
    wez.version = 1;
    wez.valid = true;
    wez.flyoutDerived = false;
    wez.inEnvelope = false;
    wez.basis = "assumed";

    const GeometryMetrics g = computeGeometry(query);
    wez.launchAltitudeM = query.launchAltitudeM;
    wez.launchMach = g.launchMach;
    wez.targetAltitudeM = query.targetAltitudeM;
    wez.targetMach = g.targetMach;

    const bool heat = card.seeker.find("infrared") != std::string::npos;
    const double reference = card.value("launch_zone_reference_m", heat ? 14000.0 : 60000.0);
    const double altitude = std::clamp(0.65 + query.launchAltitudeM / 20000.0, 0.5, 1.4);
    const double speed = std::clamp(query.launchAirspeedMs / 300.0, 0.5, 1.5);
    const double tv = norm3(query.targetVelEcef);
    wez.maxOffBoresightRad = card.value("seeker_gimbal_deg", heat ? 45.0 : 60.0) * kDegToRad;

    static constexpr double kAspectsDeg[7] = {0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0};
    wez.aspectContour.reserve(7);
    for (double deg : kAspectsDeg) {
        const double rad = deg * kDegToRad;
        const double closingTarget = tv * std::cos(rad);
        LaunchZoneWezSampleV1 sample;
        sample.aspectRad = rad;
        sample.maximumM = std::max(0.0, reference * altitude * speed + closingTarget * 25.0);
        sample.noEscapeM = std::clamp(sample.maximumM * 0.55 - tv * 12.0, 0.0, sample.maximumM);
        sample.minimumM = 900.0;
        sample.maximumUncertaintyM = sample.maximumM * 0.35;
        sample.noEscapeUncertaintyM = sample.noEscapeM * 0.35;
        wez.aspectContour.push_back(sample);
    }
    return wez;
}

} // namespace fsim::ms::weapon
